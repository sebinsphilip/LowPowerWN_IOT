#include <stdbool.h>
#include "contiki.h"
#include "lib/random.h"
#include "net/rime/rime.h"
#include "leds.h"
#include "net/netstack.h"
#include <stdio.h>
#include "core/net/linkaddr.h"
#include "my_collect.h"
/*---------------------------------------------------------------------------*/
#define BEACON_INTERVAL (CLOCK_SECOND * 60)
//#define BEACON_INTERVAL (CLOCK_SECOND * 5)
#define BEACON_FORWARD_DELAY (random_rand() % CLOCK_SECOND)
/*---------------------------------------------------------------------------*/
#define RSSI_THRESHOLD -95
/*---------------------------------------------------------------------------*/
/* Callback function declarations */
void bc_recv(struct broadcast_conn *conn, const linkaddr_t *sender);
void uc_recv(struct unicast_conn *c, const linkaddr_t *from);
void beacon_timer_cb(void* ptr);
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
#ifndef CONTIKI_TARGET_SKY
linkaddr_t sink_node = {{0xF7, 0x9C}}; /* Firefly node 1 will be our sink */
#else
linkaddr_t sink_node = {{0x01, 0x00}}; /* Sky node 1 will be our sink */
#endif
/*---------------------------------------------------------------------------*/
void
my_collect_open(struct my_collect_conn* conn, uint16_t channels, 
                bool is_sink, const struct my_collect_callbacks *callbacks)
{
  /* Initialize the connector structure */
  linkaddr_copy(&conn->parent, &linkaddr_null);
  conn->metric = 65535; /* The MAX metric (the node is not connected yet) */
  conn->beacon_seqn = 1; //initial value from 1, when overflow occurs (0) we force flush everything
  conn->callbacks = callbacks;

  /* Open the underlying Rime primitives */
  broadcast_open(&conn->bc, channels,     &bc_cb);
  unicast_open  (&conn->uc, channels + 1, &uc_cb);

  /* TO DO 1: SINK
   * 1. Make the sink send beacons periodically (BEACON_INTERVAL)
   */
  if(is_sink)
    //ctimer_set(&conn->beacon_timer, BEACON_INTERVAL, beacon_timer_cb, (void*)conn);
    ctimer_set(&conn->beacon_timer, CLOCK_SECOND* 5, beacon_timer_cb, (void*)conn);
}
/*---------------------------------------------------------------------------*/
/*                              Beacon Handling                              */
/*---------------------------------------------------------------------------*/
/* Beacon message structure */
struct beacon_msg {
  uint16_t seqn;
  uint16_t metric;
} __attribute__((packed));
/*---------------------------------------------------------------------------*/
/* Send beacon using the current seqn and metric */
void
send_beacon(struct my_collect_conn* conn)
{
  struct beacon_msg beacon = {
    .seqn = conn->beacon_seqn, .metric = conn->metric};

  packetbuf_clear();
  packetbuf_copyfrom(&beacon, sizeof(beacon));
  printf("my_collect: sending beacon: seqn %d metric %d\n",
    conn->beacon_seqn, conn->metric);
  broadcast_send(&conn->bc);
}

void beacon_forward_timer_cb (void* ptr)
{
  struct my_collect_conn* conn = (struct my_collect_conn* ) ptr;
  send_beacon (conn);
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
  struct my_collect_conn* conn = (struct my_collect_conn* ) ptr;
  conn->metric = 0; // metric always 0 for sink
  send_beacon (conn);
  conn->beacon_seqn++;
  //ctimer_reset(&conn->beacon_timer);
  ctimer_set(&conn->beacon_timer, BEACON_INTERVAL, beacon_timer_cb, (void*)conn);
}
/*---------------------------------------------------------------------------*/
/* Beacon receive callback */
void
bc_recv(struct broadcast_conn *bc_conn, const linkaddr_t *sender)
{
  struct beacon_msg beacon;
  int16_t rssi;
  bool flag_propogate = 0;
  /* Get the pointer to the overall structure my_collect_conn from its field bc */
  struct my_collect_conn* conn = (struct my_collect_conn*)(((uint8_t*)bc_conn) - 
    offsetof(struct my_collect_conn, bc));

  if(linkaddr_cmp(&sink_node, &linkaddr_node_addr)) {
    /* No need to service broadcast receive for sink node!*/
    return;
  }

  if (packetbuf_datalen() != sizeof(struct beacon_msg)) {
    printf("my_collect: broadcast of wrong size\n");
    return;
  }
  memcpy(&beacon, packetbuf_dataptr(), sizeof(struct beacon_msg));
  rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI);
  printf("my_collect: recv beacon from %02x:%02x seqn %u metric %u rssi %d\n", 
      sender->u8[0], sender->u8[1], 
      beacon.seqn, beacon.metric, rssi);

  /* TO DO 3:
   * 1. Analyze the received beacon based on RSSI, seqn, and metric.
   * 2. Update (if needed) the local/node current routing info (parent, metric).
   */
  if (rssi >= RSSI_THRESHOLD)
  {
    if (((0 == beacon.seqn) && (0 != conn->beacon_seqn)) ||
             (beacon.seqn > conn->beacon_seqn) )
    {
      //flush everything right away! data needs to be refreshed!
      flag_propogate = 1;
      printf ("Sequence number flush happened! \n");
      
    }
    else if ((beacon.seqn == conn->beacon_seqn) && (beacon.metric < conn->metric))
    {
      if ((beacon.metric == conn->metric - 1) && !(linkaddr_cmp (&conn->parent, &linkaddr_null)))
      {
        flag_propogate = 0;
        printf ("SAME METRIC DIFFERENT PARENT ALERT!! nothing happened! (%02x:%02x metric %u )\n",
        sender->u8[0], sender->u8[1], 
        beacon.metric);
      }
      else
      {
        flag_propogate = 1;
        printf ("Metric number flush happened!, new parent selection (%02x:%02x metric %u )\n",
        sender->u8[0], sender->u8[1], 
        beacon.metric);

      }
      
    }
    else
    {
      flag_propogate = 0;
      printf ("same metric (sibblings) or lower metric (parent) or lower seqn .. nothing happened! \n");
    }

  }

  /* TO DO 4:
   * If the metric or the seqn has been updated, retransmit the beacon to update
   * the node neighbors about the changes
   */

  if (flag_propogate)
  {
    ctimer_set(&conn->beacon_timer, BEACON_FORWARD_DELAY, beacon_forward_timer_cb, (void*) conn);
    conn->metric = beacon.metric + 1;
    conn->beacon_seqn = beacon.seqn;
    conn->parent.u8[0] = sender->u8[0];
    conn->parent.u8[1] = sender->u8[1]; 
  }

  
}
/*---------------------------------------------------------------------------*/
/*                               Data Handling                               */
/*---------------------------------------------------------------------------*/
/* Header structure for data packets */
struct collect_header {
  linkaddr_t source;
  uint8_t hops;
} __attribute__((packed));
/*---------------------------------------------------------------------------*/
/* Data Collection: send function */
int
my_collect_send(struct my_collect_conn *conn)
{
  struct collect_header hdr = {.source=linkaddr_node_addr, .hops=0};
  int ret;

  if (linkaddr_cmp(&conn->parent, &linkaddr_null))
    return 0; // no parent

  /* TO DO 5:
   * 1. Allocate space for the data collection header 
   * 2. Insert the header in the packet buffer
   * 3. Send the packet to the parent using unicast
   */
 
  ret = packetbuf_hdralloc (sizeof(struct collect_header));
  if (!ret)
  {
    printf ("Error in allocating packet collect header! returning..\n");
    return ret;
  }
  memcpy(packetbuf_hdrptr(), &hdr, sizeof(struct collect_header));
  ret = unicast_send (&conn->uc, &conn->parent);
  return ret;
}

/*---------------------------------------------------------------------------*/
/* Data receive callback */
void
uc_recv(struct unicast_conn *uc_conn, const linkaddr_t *from)
{
  /* Get the pointer to the overall structure my_collect_conn from its field uc */
  struct my_collect_conn* conn = (struct my_collect_conn*)(((uint8_t*)uc_conn) - 
    offsetof(struct my_collect_conn, uc));

  struct collect_header hdr;

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
    unicast_send (&conn->uc, &conn->parent);
  }
}
/*---------------------------------------------------------------------------*/

