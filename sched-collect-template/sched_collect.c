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
#define BEACON_FORWARD_DELAY (random_rand() % CLOCK_SECOND)
//#define BEACON_FORWARD_DELAY ( CLOCK_SECOND)
/*---------------------------------------------------------------------------*/
#define POSTPROCESSING_DELAY 10
#define PREPROCESSING_DELAY 16
//#define MAX_UNICST_PROCESSING_DELAY ((MAX_HOPS-1)*6)
#define MAX_UNICST_PROCESSING_DELAY ((MAX_HOPS)*6)
#define COLLECTION_SEQUENCE_DELAY (node_id-2)* MAX_UNICST_PROCESSING_DELAY
//#define RADIO_TURN_OFF_DELAY (MAX_NODES * MAX_UNICST_PROCESSING_DELAY + 200)

//#define GREEN_LED_GUARD 1000
#define GREEN_LED_GUARD 200
#define RADIO_TURN_OFF_DELAY (MAX_NODES * MAX_UNICST_PROCESSING_DELAY +GREEN_LED_GUARD)


#define GUARD_TIME -1300 //testbed
//#define GUARD_TIME 350 //cooja
#define RADIO_TURN_ON_DELAY (EPOCH_DURATION - ((MAX_HOPS)*CLOCK_SECOND)) + GUARD_TIME
#define BLUE_LED_GUARD 200 // This can be zero as-well => even-though some final beacon packets might disappear (less DC)
//#define DATACOLLECTION_COMMON_GREEN_START_DELAY  (((MAX_HOPS-1)*CLOCK_SECOND + BLUE_LED_GUARD) - bc_recv_delay)
#define DATACOLLECTION_COMMON_GREEN_START_DELAY  (((MAX_HOPS-1)*CLOCK_SECOND + BLUE_LED_GUARD) - bc_recv_delay) - bc_recv_metric


//#define DATACOLLECTION_COMMON_GREEN_START_DELAY  ((MAX_HOPS*CLOCK_SECOND) - bc_recv_delay)
//#define DATACOLLECTION_COMMON_GREEN_START_DELAY  ((MAX_HOPS*CLOCK_SECOND) - conn->received_packet_from_parent_delay)
//#define GUARD_TIME 1000
//#define GUARD_TIME 600
//#define RADIO_TURN_ON_DELAY (EPOCH_DURATION - (RADIO_TURN_OFF_DELAY + DATACOLLECTION_COMMON_GREEN_START_DELAY + GUARD_TIME))

/*---------------------------------------------------------------------------*/
/* Callback function declarations */
void bc_recv(struct broadcast_conn *conn, const linkaddr_t *sender);
void uc_recv(struct unicast_conn *c, const linkaddr_t *from);
void beacon_timer_cb(void* ptr);
static linkaddr_t sink_node;
static clock_time_t bc_recv_ts_t1, bc_recv_ts_t2,
 bc_recv_ts_tforward, bc_recv_delay, bc_recv_ts_t1_temp, temp;
static uint8_t *buffer;
static int buffer_length;
static bool flag_buffer_full, flag_datacollection_cb;
static int16_t rssi;
static uint16_t bc_recv_metric;
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
/*---------------------------------------------------------------------------*/
/* Routing and synchronization beacons */
struct beacon_msg { // Beacon message structure
  uint16_t seqn;
  uint16_t metric;
  clock_time_t delay; // embed the transmission delay to help nodes synchronize
} __attribute__((packed));
/*---------------------------------------------------------------------------*/
void
sched_collect_open(struct sched_collect_conn* conn, uint16_t channels,
  bool is_sink, const struct sched_collect_callbacks *callbacks)
{
  /* Create 2 Rime connections: broadcast (for beacons) and unicast (for collection)
   * Start the appropriate process to perform the necessary epoch operations or 
   * use ctimers and callbacks as necessary to schedule these operations.
   */

  /* Initialize the connector structure */
  linkaddr_copy(&conn->parent, &linkaddr_null);
  conn->metric = 65535; /* The MAX metric (the node is not connected yet) */
  conn->beacon_seqn = 1; //initial value from 1, when overflow occurs (0) we force flush everything
  conn->callbacks = callbacks;
  flag_buffer_full = false;
  buffer = (uint8_t*) malloc (20);

  /* Open the underlying Rime primitives */
  broadcast_open(&conn->bc, channels,     &bc_cb);
  unicast_open  (&conn->uc, channels + 1, &uc_cb);

  /* TO DO 1: SINK
   * 1. Make the sink send beacons periodically (BEACON_INTERVAL)
   */
  if(is_sink)
  {
    //ctimer_set(&conn->beacon_timer, BEACON_INTERVAL, beacon_timer_cb, (void*)conn);
    ctimer_set(&conn->beacon_timer, 0, beacon_timer_cb, (void*)conn);
    sink_node = linkaddr_node_addr;
  }
}
/*---------------------------------------------------------------------------*/
int
sched_collect_send(struct sched_collect_conn *conn, uint8_t *data, uint8_t len)
{
  /* Store packet in a local buffer to be send during the data collection 
   * time window. If the packet cannot be stored, e.g., because there is
   * a pending packet to be sent, return zero. Otherwise, return non-zero
   * to report operation success. */

/*
  if (linkaddr_cmp(&conn->parent, &linkaddr_null))
    return 0; // no parent
*/
  if (flag_buffer_full)
  {
    printf ("BUFFER FULL!!!\n");
    return 0;
  }
  if (NULL == data || 0 >= len || 20 <= len)
  {
    printf ("sched_collect: Error in data!!\n");
    return 0;
  }
  
  flag_buffer_full = true;
  /* Send data collection packet to sink */
  
  memcpy((void*)buffer, (void*)data, len);
  //*buffer = 
  buffer_length = len;
  printf ("sched_collect: Buffer queued: %u %u length:%d\n",
   ((test_msg_t*)buffer)->seqn, ((test_msg_t*)data)->seqn,buffer_length);
  
  return 1; 
}

/*---------------------------------------------------------------------------*/
/* Send beacon using the current seqn and metric */
void
send_beacon(struct sched_collect_conn* conn)
{
  struct beacon_msg beacon = {
    .seqn = conn->beacon_seqn, .metric = conn->metric};

  if(linkaddr_cmp(&sink_node, &linkaddr_node_addr)) {
    beacon.delay = 0;
  }
  else
  {
    printf ("EPOCH START: %u\n", (uint16_t)(bc_recv_ts_t1 - conn->received_packet_from_parent_delay));
    bc_recv_ts_t2 = clock_time();
    printf("sched_collect:bc_recv_ts_t1:%u\n", (uint16_t)bc_recv_ts_t1);
    printf ("sched_collect:bc_recv_ts_t2:%u\n", (uint16_t)bc_recv_ts_t2);
    beacon.delay=(bc_recv_ts_t2 - bc_recv_ts_t1) + conn->received_packet_from_parent_delay ;
  }
  

  packetbuf_clear();
  packetbuf_copyfrom(&beacon, sizeof(beacon));
  printf("sched_collect: sending beacon: seqn %d metric %d delay:%u\n",
    conn->beacon_seqn, conn->metric, (uint16_t)beacon.delay);
  //bc_recv_ts_t2 = clock_time();
  //printf ("sched_collect:bc_recv_ts_t3:%u\n",bc_recv_ts_t2);
  broadcast_send(&conn->bc);
  printf ("my_parent: %d --> %d\n", node_id, conn->parent);
  printf ("my_parent: RSSI: %d\n", rssi);
  
}

void beacon_forward_timer_cb (void* ptr)
{
  struct sched_collect_conn* conn = (struct sched_collect_conn* ) ptr;
  printf ("Here LEDS BLUE!!\n");
  send_beacon (conn);
  leds_on(LEDS_BLUE);
  ctimer_stop(&conn->beacon_timer);
}

void turn_radio_on_cb (void *ptr)
{
  struct sched_collect_conn* conn = (struct sched_collect_conn* ) ptr;
  NETSTACK_MAC.on ();
  printf ("Turned back on!!\n");
  ctimer_stop (&conn->radio_timer);
}
 

void turn_radio_off_cb (void *ptr)
{
  struct sched_collect_conn* conn = (struct sched_collect_conn* ) ptr;
  NETSTACK_MAC.off (false);
  printf ("sched_collect: LEDS GREEN OFF SIGNALLLL !!!!!! \n");
  leds_off(LEDS_GREEN);
  ctimer_set(&conn->radio_timer, RADIO_TURN_ON_DELAY,
    turn_radio_on_cb, (void*)conn);
}


void datacollection_send_unicast_cb (void* ptr)
{
  int ret;

  if (!flag_buffer_full)
  {
    printf ("sched_collect: Buffer empty, nothing to send!!\n");
    return;
  }
  struct collect_header hdr = {.source=linkaddr_node_addr, .hops=0};
  struct sched_collect_conn* conn = (struct sched_collect_conn* ) ptr;
  leds_on(LEDS_GREEN);
  packetbuf_clear();
  memcpy(packetbuf_dataptr(), buffer, buffer_length);
  printf ("sched_collect: Buffer:%d length:%d to_parent:%d \n",
  ((test_msg_t*)buffer)->seqn, buffer_length, conn->parent);
  packetbuf_set_datalen(buffer_length);


  ret = packetbuf_hdralloc (sizeof(struct collect_header));
  if (!ret)
  {
    printf ("Error in allocating packet collect header! returning..\n");
    return ret;
  }
  memcpy(packetbuf_hdrptr(), &hdr, sizeof(struct collect_header));
  ret = unicast_send (&conn->uc, &conn->parent);
  flag_buffer_full = false;
  buffer_length = 0;
  ctimer_stop (&conn->sync_timer);
  //ctimer_set(&conn->radio_timer, RADIO_TURN_OFF_DELAY - COLLECTION_SEQUENCE_DELAY,
 //   turn_radio_off_cb, (void*)conn);
}


void datacollection_green_start_cb (void *ptr)
{
  struct sched_collect_conn* conn = (struct sched_collect_conn* ) ptr;
  leds_off(LEDS_BLUE);
  ctimer_set(&conn->sync_timer, COLLECTION_SEQUENCE_DELAY,
    datacollection_send_unicast_cb, (void*)conn);
  flag_datacollection_cb = false;
  
  ctimer_set(&conn->radio_timer, RADIO_TURN_OFF_DELAY,
    turn_radio_off_cb, (void*)conn);



}
/*---------------------------------------------------------------------------*/
/* Beacon timer callback */
void
beacon_timer_cb(void* ptr)
{
  /* TO DO 2: Implement the beacon callback
   * 1. Send beacon
   * 2. Should the sink do anything else?
   */
  
  //NETSTACK_MAC.on();
  struct sched_collect_conn* conn = (struct sched_collect_conn* ) ptr;
  conn->metric = 0; // metric always 0 for sink
  send_beacon (conn);
  conn->beacon_seqn++;
  //ctimer_reset(&conn->beacon_timer);
  ctimer_set(&conn->beacon_timer, EPOCH_DURATION, beacon_timer_cb, (void*)conn);
}
/*---------------------------------------------------------------------------*/
/* Beacon receive callback */
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
  
      
  /* TO DO 3:
   * 1. Analyze the received beacon based on RSSI, seqn, and metric.
   * 2. Update (if needed) the local/node current routing info (parent, metric).
   */
  if (rssi_temp >= RSSI_THRESHOLD)
  {
    if (((0 == beacon.seqn) && (0 != conn->beacon_seqn)) ||
             (beacon.seqn > conn->beacon_seqn) )
    {
      //flush everything right away! data needs to be refreshed!
      flag_propogate = 1;
      printf ("sched_collect:Sequence number flush happened! \n");
      
    }
    else if ((beacon.seqn == conn->beacon_seqn) && (beacon.metric < conn->metric))
    {
      if ((beacon.metric == conn->metric - 1) && !(linkaddr_cmp (&conn->parent, &linkaddr_null)))
      {
        flag_propogate = 0;
        printf ("sched_collect: SAME METRIC DIFFERENT PARENT ALERT!! nothing happened! (%02x:%02x metric %u )\n",
        sender->u8[0], sender->u8[1], 
        beacon.metric);
      }
      else
      {
        flag_propogate = 1;
        printf ("sched_collect: Metric number flush happened!, new parent selection (%02x:%02x metric %u )\n",
        sender->u8[0], sender->u8[1], 
        beacon.metric);

      }
      
    }
    else
    {
      flag_propogate = 0;
      printf ("sched_collect: same metric (sibblings) or lower metric (parent) or lower seqn .. nothing happened! \n");
    }

  }

  /* TO DO 4:
   * If the metric or the seqn has been updated, retransmit the beacon to update
   * the node neighbors about the changes
   */

  if (flag_propogate)
  {
    bc_recv_ts_tforward = BEACON_FORWARD_DELAY;
    if (bc_recv_ts_tforward < (PREPROCESSING_DELAY + POSTPROCESSING_DELAY))
    {
      printf ("After negation !! %d\n",bc_recv_ts_tforward);
      bc_recv_ts_tforward = 0;
    }
    else
    {
      bc_recv_ts_tforward -= (PREPROCESSING_DELAY + POSTPROCESSING_DELAY);
    }
    printf ("Here inside flag propogate !! delay:%d\n", bc_recv_ts_tforward);
    //temp = clock_time ();
    //printf("sched_collect:difference:%u\n", temp-bc_recv_ts_t1_temp);
    conn->received_packet_from_parent_delay =  beacon.delay;
    bc_recv_delay = beacon.delay;
    bc_recv_metric = beacon.metric * 20;
    ctimer_set(&conn->beacon_timer, bc_recv_ts_tforward, beacon_forward_timer_cb, (void*) conn);
    //if (!flag_datacollection_cb)
    //{
      
      //data collection (green led) timer
      printf ("Here inside flag_datacollection_cb !!");
      ctimer_set(&conn->sync_timer, DATACOLLECTION_COMMON_GREEN_START_DELAY,
       datacollection_green_start_cb, (void*) conn);
      //flag_datacollection_cb = true;
    //}
    conn->metric = beacon.metric + 1;
    conn->beacon_seqn = beacon.seqn;
    conn->parent.u8[0] = sender->u8[0];
    conn->parent.u8[1] = sender->u8[1];
    
    //bc_recv_delay = beacon.delay;
    bc_recv_ts_t1 = bc_recv_ts_t1_temp;
    rssi = rssi_temp;
  }

  
}
/*---------------------------------------------------------------------------*/
/* Data receive callback */
void
uc_recv(struct unicast_conn *uc_conn, const linkaddr_t *from)
{
  temp = clock_time ();
  //printf("sched_collect:temp1:%u\n", temp);
  /* Get the pointer to the overall structure my_collect_conn from its field uc */
  struct sched_collect_conn* conn = (struct sched_collect_conn*)(((uint8_t*)uc_conn) - 
    offsetof(struct sched_collect_conn, uc));

  struct collect_header hdr;
  printf ("HHEY\n");

  if (packetbuf_datalen() < sizeof(struct collect_header)) {
    printf("my_collect: too short unicast packet %d\n", packetbuf_datalen());
    return;
  }

  /* TO DO 6:
   * 1. Extract the header
   * 2. On the sink, remove the header and call the application callback
   * 3. On a router, update the header and forward the packet to the parent using unicast
   */
  memcpy(&hdr, packetbuf_dataptr(), sizeof(struct collect_header));
  printf ("my_collect source|%02x:%02x hop|%d\n", hdr.source.u8[0],
                   hdr.source.u8[1], hdr.hops);
 
  if (linkaddr_cmp (&sink_node, &linkaddr_node_addr))
  {
    //printf ("Here at the sink node!!\n");
    packetbuf_hdrreduce (sizeof(struct collect_header));
    conn->callbacks->recv (&hdr.source, hdr.hops);
  }
  else
  {
    hdr.hops += 1;
    memcpy(packetbuf_dataptr(), &hdr, sizeof(struct collect_header));
    temp = clock_time ();
    //printf("sched_collect:temp2:%u\n", temp);
    unicast_send (&conn->uc, &conn->parent);
  }
}