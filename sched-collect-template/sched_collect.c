/**
 * \defgroup sched_collect implementation
 *
 * @{
 */

/**
 * \file
 *         A brief description of what this file is.
 * \author
 *         Sebin Shaji Philip <sebin.shajiphilip@studenti.unitn.it>
 *         
 *         This is the implementation of time-synchronised multi-hop
 *         data-collection protocol for wireless sensor networks.   
 *         
 */


#include <stdbool.h>
#include "contiki.h"
#include "lib/random.h"
#include "net/rime/rime.h"
#include "leds.h"
#include "net/netstack.h"
#include <stdio.h>
#include <stdlib.h>
#include "core/net/linkaddr.h"
#include "node-id.h"
#include "sched_collect.h"
/*---------------------------------------------------------------------------*/
#define RSSI_THRESHOLD -91 // filter bad links
/*---------------------------------------------------------------------------*/

/*
 * DELAY_CEIL is the maximum delay allowed before propogating a beacon. and 
 * BEACON_FORWARD_DELAY is the random delay within the DELAY_CEIL
 */
#define DELAY_CEIL 350
#define BEACON_FORWARD_DELAY (random_rand() % DELAY_CEIL)

/*
 * PREPROCESSING_DELAY is the delay caused by pre-processing steps in bc_recv()
 * before arming the beacon_timer and POSTPROCESSING_DELAY is the time taken
 * after the beacon_forward_timer_cb() is called (and before broadcast send)
 */
#define POSTPROCESSING_DELAY 10
#define PREPROCESSING_DELAY 16

/*
 * MAX_UNICST_PROCESSING_DELAY is the maximum time required for a unicast 
 * packet from the farthest end (max. hop count) to reach the sink node,
 * after all the processing and transmisions. '6' is the time spend in
 * the uc_recv() callback.
 */
#define MAX_UNICST_PROCESSING_DELAY ((MAX_HOPS)*6)

/*
 * COLLECTION_SEQUENCE_DELAY is the time each non-sink node have to wait
 * (after the time-sync phase) before sendin the scheduled unicast packet .
 */
#define COLLECTION_SEQUENCE_DELAY (node_id-2)* MAX_UNICST_PROCESSING_DELAY


/*
 * RADIO_TURN_OFF_DELAY is the time each non-sink node have to wait
 * (after the time-sync phase) before turning the radio-off .
 */
#define GREEN_LED_GUARD 200
#define RADIO_TURN_OFF_DELAY (MAX_NODES * MAX_UNICST_PROCESSING_DELAY + GREEN_LED_GUARD)

/*
 * DATACOLLECTION_COMMON_GREEN_START_DELAY is the time each non-sink node have to wait
 * (after forwarding a beacon) before entering into the data-collection phase .
 */
#define BLUE_LED_GUARD 200
#define DATACOLLECTION_COMMON_GREEN_START_DELAY  (((MAX_HOPS-1)*DELAY_CEIL + BLUE_LED_GUARD) - bc_recv_delay) - bc_recv_metric


/*
 * RADIO_TURN_ON_DELAY is the time each non-sink node have to wait
 * (after the RADIO_OFF phase) before turning the radio-on again .
 */
#ifndef CONTIKI_TARGET_SKY
#define GUARD_TIME -50 //cooja
#else
#define GUARD_TIME 0 // This value needs to be optimised for testbed
#endif
#define RADIO_TURN_ON_DELAY (EPOCH_DURATION - (RADIO_TURN_OFF_DELAY + DATACOLLECTION_COMMON_GREEN_START_DELAY + (bc_recv_delay+bc_recv_metric))) + GUARD_TIME

/*---------------------------------------------------------------------------*/
/* Callback function declarations */
void bc_recv(struct broadcast_conn *conn, const linkaddr_t *sender);
void uc_recv(struct unicast_conn *c, const linkaddr_t *from);
void beacon_timer_cb(void* ptr);
/*---------------------------------------------------------------------------*/
/* The static variabes  used for time-stamp, calculating delay, rssi etc.*/
static linkaddr_t sink_node;
static clock_time_t bc_recv_ts_t1, bc_recv_ts_t2,
 bc_recv_ts_tforward, bc_recv_delay, bc_recv_ts_t1_temp, temp;
static uint8_t *buffer;
static int buffer_length;
static bool flag_buffer_full;
static int16_t rssi;
static uint16_t bc_recv_metric;
/*---------------------------------------------------------------------------*/
/* This struture from App is used for debug pupose */
typedef struct {
  uint16_t seqn;
}
__attribute__((packed))
test_msg_t;

/*---------------------------------------------------------------------------*/
/* Header structure for data packets */
struct collect_header {
  linkaddr_t source;
  uint8_t hops;
} __attribute__((packed));
/*---------------------------------------------------------------------------*/
/* Rime Callback structures */
struct broadcast_callbacks bc_cb = {
  .recv = bc_recv,
  .sent = NULL
};
struct unicast_callbacks uc_cb = {
  .recv = uc_recv,
  .sent = NULL
};
/*---------------------------------------------------------------------------*/
/* Routing and synchronization beacons */
struct beacon_msg { // Beacon message structure
  uint16_t seqn;
  uint16_t metric;
  clock_time_t delay; // embed the transmission delay to help nodes synchronize
} __attribute__((packed));
/*---------------------------------------------------------------------------*/




/*---------------------------------------------------------------------------*/
/**
 * \brief         Function to open connections
 * \param conn    The pointer to connection instance of type sched_collect_conn
 * \param channels    The channel number to be used for opening broadcast and unicast connections
 * \param is_sink    The bool to indicate if the caller is sink node or not
 * \param callbacks    The pointer to  app callback instance of type sched_collect_callbacks
 * \return     No return value
 * 
 *             This function can be called by any node to open its broadcast and unicast connections.
 *             The sink node will be able to assign external callbacks, which could be invoked on 
 *             arrival of a unicast packet. Also if its a sink node, periodic beaconing could be 
 *             inovoked for the EPOCH period (for routing initiation).
 */
void
sched_collect_open(struct sched_collect_conn* conn, uint16_t channels,
  bool is_sink, const struct sched_collect_callbacks *callbacks)
{
  /* Initialize the connector structure */
  linkaddr_copy(&conn->parent, &linkaddr_null);
  conn->metric = 65535; /* The MAX metric (the node is not connected yet) */
  conn->beacon_seqn = 1; /*initial value from 1, when overflow occurs (0) we force flush everything*/
  conn->callbacks = callbacks; /*assign broadcast and unicast callbacks*/
  flag_buffer_full = false;
  buffer = (uint8_t*) malloc (20); /*Allocate data buffer for each node*/

  /* Open the underlying Rime primitives for broadcast and unicast*/
  broadcast_open(&conn->bc, channels,     &bc_cb);
  unicast_open  (&conn->uc, channels + 1, &uc_cb);

  /* If SINK node,  send beacons periodically (EPOCH_DURATION)
   */
  if(is_sink) {
    ctimer_set(&conn->beacon_timer, 0, beacon_timer_cb, (void*)conn);
    sink_node = linkaddr_node_addr;
  }
}

/*---------------------------------------------------------------------------*/
/**
 * \brief         Function to schedule a unicast sending of packets
 * \param conn    The pointer to connection instance of type sched_collect_conn
 * \param data    This pointer to an unsigned 8-bit integer, holds address to data
 * \param len    The length of the data to be send in bytes
 * 
 * \return     Returns 0 if not able to schedule, otherwise sucess
 * 
 *             This function can be called by any node (non-sink) to send a unicast data
 *             to the sink node. If the scheduling buffer of the corresponding node is free
 *             data will be buffered and send immediately the next feasible EPOCH duration.
 */
int
sched_collect_send(struct sched_collect_conn *conn, uint8_t *data, uint8_t len)
{
  /* Store packet in a local buffer to be send during the data collection 
   * time window. If the packet cannot be stored, e.g., because there is
   * a pending packet to be sent, return zero. Otherwise, return non-zero
   * to report operation success. */

  if (flag_buffer_full) {
    printf ("sched_collect: BUFFER FULL!!!\n");
    return 0;
  }
  if (NULL == data || 0 >= len || 20 <= len) {
    printf ("sched_collect: Error in data!!\n");
    return 0;
  }
  
  flag_buffer_full = true;
  /* Store data into buffer, to be send later*/
  memcpy((void*)buffer, (void*)data, len);
  buffer_length = len;

  printf ("sched_collect: Buffer queued: %u %u length:%d\n",
   ((test_msg_t*)buffer)->seqn, ((test_msg_t*)data)->seqn,buffer_length);
  
  return 1; 
}


/*---------------------------------------------------------------------------*/
/**
 * \brief         Function to Send beacon using the current seqn and metric
 * \param conn    The pointer to connection instance of type sched_collect_conn
 * 
 * \return     No retun value
 * 
 *             This function will be called internally by both sink and non-sink
 *             nodes to actually send a beacon packet ( broadcast) with current 
 *             hop-count and sequence number present in the \param conn. The delay
 *             which was taken in this non-sink node (from recieving the broadcast
 *             callback till the sending) will be also updated to the accumulated
 *             delay section of the packet.
 */
void
send_beacon(struct sched_collect_conn* conn)
{
  /*Pack the beacon message with valid data stored in conn*/
  struct beacon_msg beacon = {
    .seqn = conn->beacon_seqn, .metric = conn->metric};

  if(linkaddr_cmp(&sink_node, &linkaddr_node_addr)) {
    beacon.delay = 0;
  }
  else {
    printf ("sched_collect: EPOCH START: %u\n", (uint16_t)(bc_recv_ts_t1 - conn->received_packet_from_parent_delay));
    bc_recv_ts_t2 = clock_time();
    printf("sched_collect:bc_recv_ts_t1:%u\n", (uint16_t)bc_recv_ts_t1);
    printf ("sched_collect:bc_recv_ts_t2:%u\n", (uint16_t)bc_recv_ts_t2);

    /* The total delay to be embedded into the sending packet*/
    beacon.delay=(bc_recv_ts_t2 - bc_recv_ts_t1) + conn->received_packet_from_parent_delay ;
  }
  

  packetbuf_clear();
  packetbuf_copyfrom(&beacon, sizeof(beacon));
  printf("sched_collect: sending beacon: seqn %d metric %d delay:%u\n",
    conn->beacon_seqn, conn->metric, (uint16_t)beacon.delay);
  /* Debug prints
   * bc_recv_ts_t2 = clock_time();
   * printf ("sched_collect:bc_recv_ts_t3:%u\n",bc_recv_ts_t2);
   * printf ("my_parent: %d --> %d\n", node_id, conn->parent);
   * printf ("my_parent: RSSI: %d\n", rssi);
   */
  broadcast_send(&conn->bc);
  
  
}

/*---------------------------------------------------------------------------*/
/**
 * \brief         Callback timer function to re-broadcast a beacon from a non-sink node
 * \param ptr    The pointer to void, the callback argument
 * 
 * \return     No retun value
 * 
 *             This function will be called internally by the non-sink node, upon
 *             successfully recieving a beacon packet inside the broadcast recieve
 *             callback and decides to re-broadcast the same beacon with updated metrics.
 *             This timer callback is responsible to call the actual send_beacon() to
 *             repropogate the beacon.
 */

void 
beacon_forward_timer_cb(void* ptr)
{
  struct sched_collect_conn* conn = (struct sched_collect_conn* ) ptr;
  printf ("sched_collect: Inside  beacon_forward_timer_cb()\n");
  send_beacon (conn);
  leds_on(LEDS_BLUE);
  ctimer_stop(&conn->beacon_timer);
}

/*---------------------------------------------------------------------------*/
/**
 * \brief        Callback timer function to turn-on the node radio
 * \param ptr    The pointer to void, the callback argument
 * 
 * \return     No retun value
 * 
 *             This function will be called internally by the non-sink to turn-on,
 *             the radio node when the radio_timer (in struct sched_collect_conn)
 *             expires (immediately before each time synchronisation phase). 
 * 
 */

void 
turn_radio_on_cb(void *ptr)
{
  struct sched_collect_conn* conn = (struct sched_collect_conn* ) ptr;
  NETSTACK_MAC.on ();
  printf ("sched_collect: Radio turned back on!!\n");
  ctimer_stop (&conn->radio_timer);
}
 
/*---------------------------------------------------------------------------*/
/**
 * \brief        Callback timer function to turn-off the node radio
 * \param ptr    The pointer to void, the callback argument
 * 
 * \return     No retun value
 * 
 *             This function will be called internally by the non-sink node to
 *             turn-off the radio,  when the radio_timer (in struct sched_collect_conn)
 *             expires (immediately after data collection phase). 
 * 
 */

void
turn_radio_off_cb(void *ptr)
{
  struct sched_collect_conn* conn = (struct sched_collect_conn* ) ptr;
  NETSTACK_MAC.off (false);
  printf ("sched_collect: Radio turned OFF!\n");
  leds_off(LEDS_GREEN);
  ctimer_set(&conn->radio_timer, RADIO_TURN_ON_DELAY,
    turn_radio_on_cb, (void*)conn);
}

/*---------------------------------------------------------------------------*/
/**
 * \brief        Callback timer function to actually send the unicast packet
 *               scheduled from the non-sink node.
 * \param ptr    The pointer to void, the callback argument
 * 
 * \return     No retun value
 * 
 *             This function will be called internally by the non-sink node to
 *             send the unicast packet scheduled,  when the sync_timer (in struct
 *             sched_collect_conn) expires. This timer is armed inside function 
 *             datacollection_green_start_cb (). 
 * 
 */

void 
datacollection_send_unicast_cb(void* ptr)
{
  int ret;

  if (!flag_buffer_full) {
    printf ("sched_collect: Buffer empty, nothing to send!!\n");
    return;
  }
  /* The header info to be send with the unicast data*/
  struct collect_header hdr = {.source=linkaddr_node_addr, .hops=0};
  struct sched_collect_conn* conn = (struct sched_collect_conn* ) ptr;
  /* Turn -ON green LEDS to indicate actual sending of unicast data*/
  leds_on(LEDS_GREEN);
  packetbuf_clear();
  memcpy(packetbuf_dataptr(), buffer, buffer_length);
  printf ("sched_collect: Buffer:%d length:%d to_parent:%d \n",
  ((test_msg_t*)buffer)->seqn, buffer_length, conn->parent);

  packetbuf_set_datalen(buffer_length);
  ret = packetbuf_hdralloc (sizeof(struct collect_header));
  if (!ret) {
    printf ("sched_collect: Error in allocating packet collect header! returning..\n");
    return ret;
  }
  memcpy(packetbuf_hdrptr(), &hdr, sizeof(struct collect_header));
  /* Send unicast packet.*/
  ret = unicast_send (&conn->uc, &conn->parent);

  /* Clear buffer, now ready to accept more messages*/
  flag_buffer_full = false;
  buffer_length = 0;
  ctimer_stop (&conn->sync_timer);
}

/*---------------------------------------------------------------------------*/
/**
 * \brief        Callback timer function to schedule unicast send and radio turn-off
 *               of a non-sink node.
 * \param ptr    The pointer to void, the callback argument
 * 
 * \return     No retun value
 * 
 *            This function will be called internally by the non-sink node to
 *            schedule unicast send and radio turn-off,  when the sync_timer 
 *            (in struct sched_collect_conn) expires. This timer is armed inside
 *            function bc_recv (). Thus when a non-sink node receives a beacon packet
 *            which qualifies for re-broadcasting, this timer callback is set.
 * 
 */

void 
datacollection_green_start_cb(void *ptr)
{
  printf ("sched_collect: Inside  datacollection_green_start_cb, node_id:%d\n", node_id);
  struct sched_collect_conn* conn = (struct sched_collect_conn* ) ptr;
  leds_off(LEDS_BLUE);
  /* Arm timer for actual sending of unicast packet according to node_id*/
  ctimer_set(&conn->sync_timer, COLLECTION_SEQUENCE_DELAY,
    datacollection_send_unicast_cb, (void*)conn);
  /* Arm timer for turning off radio, after complete data collection*/
  ctimer_set(&conn->radio_timer, RADIO_TURN_OFF_DELAY,
    turn_radio_off_cb, (void*)conn);

}

/*---------------------------------------------------------------------------*/
/**
 * \brief        Callback timer function to send beacon from sink node
 * \param ptr    The pointer to void, the callback argument
 * 
 * \return     No retun value
 * 
 *            This function will be called repeatdly by the timer beacon_timer
 *            (inside struct sched_collect_conn) to send beacons at the beginning
 *            of each EPOCH (by the sink).
 */

void
beacon_timer_cb(void* ptr)
{
  struct sched_collect_conn* conn = (struct sched_collect_conn* ) ptr;
  conn->metric = 0; /* metric always 0 for sink */
  send_beacon (conn);
  conn->beacon_seqn++;
  /* Arm timer to send beacon for each EPOCH */
  ctimer_set(&conn->beacon_timer, EPOCH_DURATION, beacon_timer_cb, (void*)conn);
}

/*---------------------------------------------------------------------------*/
/**
 * \brief        Broadcast recieve callback (to receive beacons by non-sink node)
 * \param ptr    The pointer to broadcast_conn instance, which was used to open the broadcast
 * \param sender Pointer to the sender link address.
 * 
 * \return     No retun value
 * 
 *            This function will be called upon successful receival of a broadcast packet
 *            in non-sink node (beacons for time-sync). The main functions of this procedure is:
 *              1.Analyze the received beacon based on RSSI, seqn, and metric.
 *              2.Update (if needed) the local/node current routing info (parent, metric).
 *              3.If the metric or the seqn has been updated, retransmit the beacon to update
 *                the node neighbors about the changes (by arming beacon_timer)
 *              4.Arm sync_timer to schedule data collection phase based on accumulated delay
 *                received in the beacon packet.
 * 
 */

void
bc_recv(struct broadcast_conn *bc_conn, const linkaddr_t *sender)
{
  bc_recv_ts_t1_temp = clock_time ();
  printf("sched_collect:bc_recv_ts_t1_temp:%u\n", bc_recv_ts_t1_temp);
  struct beacon_msg beacon;
  int16_t rssi_temp;
  bool flag_propogate = 0;
  /* Get the pointer to the overall structure sched_collect from its field bc */
  struct sched_collect_conn* conn = (struct sched_collect_conn*)(((uint8_t*)bc_conn) - 
    offsetof(struct sched_collect_conn, bc));
  

  if(linkaddr_cmp(&sink_node, &linkaddr_node_addr)) {
    /* No need to service broadcast receive for sink node!*/
    return;
  }

  if (packetbuf_datalen() != sizeof(struct beacon_msg)) {
    printf("sched_collect: broadcast of wrong size\n");
    return;
  }
  memcpy(&beacon, packetbuf_dataptr(), sizeof(struct beacon_msg));
  rssi_temp = packetbuf_attr(PACKETBUF_ATTR_RSSI);
  printf("sched_collect: recv beacon from %02x:%02x seqn %u metric %u delay :%u rssi_temp %d \n", 
      sender->u8[0], sender->u8[1], 
      beacon.seqn, beacon.metric, beacon.delay, rssi_temp);
  
      
  /* 
   * 1. Analyze the received beacon based on RSSI, seqn, and metric.
   * 2. Update (if needed) the local/node current routing info (parent, metric).
   */

  if (rssi_temp >= RSSI_THRESHOLD) {
    if (((0 == beacon.seqn) && (0 != conn->beacon_seqn)) ||
             (beacon.seqn > conn->beacon_seqn) ) {
      /*flush everything right away! data needs to be refreshed!*/
      flag_propogate = 1;
      printf ("sched_collect:Sequence number flush happened! \n");
      
    }
    else if ((beacon.seqn == conn->beacon_seqn) && (beacon.metric < conn->metric)) {
      if ((beacon.metric == conn->metric - 1) && !(linkaddr_cmp (&conn->parent, &linkaddr_null))) {
        flag_propogate = 0;
        printf ("sched_collect: SAME METRIC DIFFERENT PARENT ALERT!! nothing happened! (%02x:%02x metric %u )\n",
        sender->u8[0], sender->u8[1], 
        beacon.metric);
      }
      else {
        flag_propogate = 1;
        printf ("sched_collect: Metric number flush happened!, new parent selection (%02x:%02x metric %u )\n",
        sender->u8[0], sender->u8[1], 
        beacon.metric);

      }
      
    }
    else {
      flag_propogate = 0;
      printf ("sched_collect: same metric (sibblings) or lower metric (parent) or lower seqn .. nothing happened! \n");
    }

  }

  /* 
   * If the metric or the seqn has been updated, retransmit the beacon to update
   * the node neighbors about the changes
   */

  if (flag_propogate) {

    bc_recv_ts_tforward = BEACON_FORWARD_DELAY;
    if (bc_recv_ts_tforward < (PREPROCESSING_DELAY + POSTPROCESSING_DELAY)) {
      /* To avoid negative delay bc_recv_ts_tforward is assigned 0*/
      bc_recv_ts_tforward = 0;
    }
    else {
      /* Calculate resulting forward delay by taking into account the preprocessing
       * and postprocessing delays 
       */
      bc_recv_ts_tforward -= (PREPROCESSING_DELAY + POSTPROCESSING_DELAY);
    }
    printf ("sched_collect: Here inside flag propogate !! delay:%d\n", bc_recv_ts_tforward);
    /* Debug print
     * temp = clock_time ();
     * printf("sched_collect:difference:%u\n", temp-bc_recv_ts_t1_temp);
     */
    conn->received_packet_from_parent_delay =  beacon.delay;
    bc_recv_delay = beacon.delay;
    /* bc_recv_metric is calculated to adjust the time-sync based on hop count */
    bc_recv_metric = beacon.metric * 20;
    /* Beacon propogate timer*/
    ctimer_set(&conn->beacon_timer, bc_recv_ts_tforward, beacon_forward_timer_cb, (void*) conn);
   
    /*common data collection  timer*/
    ctimer_set(&conn->sync_timer, DATACOLLECTION_COMMON_GREEN_START_DELAY,
          datacollection_green_start_cb, (void*) conn);
      
    conn->metric = beacon.metric + 1;
    conn->beacon_seqn = beacon.seqn;
    conn->parent.u8[0] = sender->u8[0];
    conn->parent.u8[1] = sender->u8[1];
    /* The time stamp when entering bc_recv()*/ 
    bc_recv_ts_t1 = bc_recv_ts_t1_temp;
    rssi = rssi_temp;
  }

  
}

/*---------------------------------------------------------------------------*/
/**
 * \brief          Unicast recieve callback (to receive data send by non-sink nodes)
 * \param uc_conn  The pointer to unicast_conn instance, which was used to open the unicast
 * \param from     Pointer to the sender link address.
 * 
 * \return     No retun value
 * 
 *            This function will be called upon successful receival of a unicast packet
 *            in nodes (data packets). The main functions of this procedure is:
 *             1. Extract the header
 *             2. On the sink, remove the header and call the application callback
 *             3. On a router, update the header and forward the packet to the parent using unicast
 * 
 */

void
uc_recv(struct unicast_conn *uc_conn, const linkaddr_t *from)
{
  /* Get the pointer to the overall structure my_collect_conn from its field uc */
  struct sched_collect_conn* conn = (struct sched_collect_conn*)(((uint8_t*)uc_conn) - 
    offsetof(struct sched_collect_conn, uc));

  struct collect_header hdr;

  if (packetbuf_datalen() < sizeof(struct collect_header)) {
    printf("sched_collect: too short unicast packet %d\n", packetbuf_datalen());
    return;
  }

  /* 
   * 1. Extract the header
   * 2. On the sink, remove the header and call the application callback
   * 3. On a router, update the header and forward the packet to the parent using unicast
   */
  memcpy(&hdr, packetbuf_dataptr(), sizeof(struct collect_header));
  printf ("sched_collect: source|%02x:%02x hop|%d\n", hdr.source.u8[0],
                   hdr.source.u8[1], hdr.hops);
 
  if (linkaddr_cmp (&sink_node, &linkaddr_node_addr)) {
    packetbuf_hdrreduce (sizeof(struct collect_header));
    conn->callbacks->recv (&hdr.source, hdr.hops);
  }
  else {
    hdr.hops += 1;
    memcpy(packetbuf_dataptr(), &hdr, sizeof(struct collect_header));
    unicast_send (&conn->uc, &conn->parent);
  }
}


/** @} */