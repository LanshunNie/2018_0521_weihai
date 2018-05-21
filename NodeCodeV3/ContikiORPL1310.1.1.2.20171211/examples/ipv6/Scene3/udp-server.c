/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"
#include "net/ip/uip.h"
#include "net/rpl/rpl.h"
#include "net/netstack.h"
#include "dev/button-sensor.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#define UIP_MCAST6_CONF_ENGINE UIP_MCAST6_ENGINE_SCF
#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"
#include "net/ipv6/sicslowpan.h"
#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

#define MAX_REPLY_LEN 30
#define SERVER_REPLY  0

#define UDP_CLIENT_PORT 8765
#define UDP_SERVER_PORT 5678

#define UDP_EXAMPLE_ID  190
#define MCAST_TEST_UDP_PORT 6105 /* Host byte order */
#define INACTIVE_INTERVAL (600 * CLOCK_SECOND)
#define ROOT_SEND_INTERVAL (10 * CLOCK_SECOND)
static struct uip_udp_conn *server_conn;
static struct uip_udp_conn *multicast_server_conn;
static uip_ipaddr_t multicast_ipaddr;
static uint8_t multi_send_count = 0;
#define NODE_MAX  30
struct  upcount_array {
  uint16_t  addr;
  uint16_t  upcount;
  uint16_t  last_count;
  uint16_t  duplicate_count;
};

static struct upcount_array node_array[NODE_MAX];
static uint16_t  app_duplicate =0;

PROCESS(udp_server_process, "UDP server process");
AUTOSTART_PROCESSES(&udp_server_process);
/*---------------------------------------------------------------------------*/

static void
tcpip_handler(void)
{

  linkaddr_t sender;
  uint16_t sender_addr =0;
  uint16_t count ;
  uint16_t node_rank =0;
  char buf[MAX_REPLY_LEN];
  int pos;
  pos = 0;
  memset(buf,0,10);
  uint8_t send_type_flag;

  if(uip_newdata()) {
   uip_udp_received_data_preprocess();

    sender.u8[0] = UIP_IP_BUF->srcipaddr.u8[15];
    sender.u8[1] = UIP_IP_BUF->srcipaddr.u8[14];  
  //  appdata = (char *)uip_appdata;
  //  appdata[uip_datalen()] = 0;
    sender_addr = sender.u8[0] + (sender.u8[1] << 8);
    memcpy(&count, ((uint16_t *)(uip_appdata)), sizeof(count));   
    memcpy(&node_rank, ((uint16_t *)(uip_appdata+2)), sizeof(node_rank));  
    memcpy(&send_type_flag, ((uint8_t *)(uip_appdata+2+2)), sizeof(send_type_flag));  
    uint8_t i;
      for ( i=0; i< NODE_MAX ;i++)
       {
         if(node_array[i].addr == 0xffff)
         {
           node_array[i].addr = sender_addr ;
           node_array[i].upcount ++;
           node_array[i].last_count = uip_htons(count);
           break; 
          }
         else if (sender_addr == node_array[i].addr)
         {
            if(node_array[i].last_count!=uip_htons(count)){
                node_array[i].upcount ++;
                node_array[i].last_count = uip_htons(count);
            }else{
                 app_duplicate++;
                 node_array[i].duplicate_count ++;
               // printf("app_duplicate:%u\n", app_duplicate); 
            }    
            break; 
         }       
      } 
      // printf("totle app_duplicate:%u\n", app_duplicate);  
      printf("addr :%2x, send count: %u ,receive count: %u,duplicate count: %u\n",
      node_array[i].addr, uip_htons(count), node_array[i].upcount,node_array[i].duplicate_count);
     
      // printf("addr :%2x, node rank: %u\n",node_array[i].addr,node_rank);

#if SERVER_REPLY
      // printf("app_send\n");
    // PRINTF("DATA sending reply\n");
    // uip_ipaddr_copy(&server_conn->ripaddr, &UIP_IP_BUF->srcipaddr);
    // uip_udp_packet_send(server_conn, "Reply", sizeof("Reply"));
    // uip_create_unspecified(&server_conn->ripaddr);

    if(send_type_flag == 0){
        // uip_udp_packet_sendto(server_conn, "reply", sizeof("Reply"),
        //         &UIP_IP_BUF->srcipaddr,UIP_HTONS(UDP_CLIENT_PORT));
      printf("receive unicast, don't reply\n");
    }else if(send_type_flag == 1 && multi_send_count == 0){
      multi_send_count++;
      printf("receive multicast, reply \n");
      uip_udp_packet_sendto(multicast_server_conn, "reply", sizeof("Reply"),
                    &multicast_ipaddr, UIP_HTONS(MCAST_TEST_UDP_PORT));
    }
#endif
  }
}

// static void
// send_multi_packet(void *ptr)
// {
//   char buf[MAX_PAYLOAD_LEN];
//   uint16_t id;
//   int pos;
  
//   send_type_flag = 1;
//   pos = 0;
//   memset(buf,0,10);

//   rpl_dag_t *dag = rpl_get_any_dag();
//   if(dag == NULL){
//      return;
//   }
//   int16_t seq_id = -1;
//   id=uip_htons(seq_id);
//   rpl_rank_t curr_rank = dag->rank;
//   // printf("DATA send to '%d'\n", seq_id);
//   // PRINTF("DATA send to %d '%d'\n",
//   //        server_ipaddr.u8[sizeof(server_ipaddr.u8) - 1], seq_id);
//   memcpy(buf,&id,sizeof(seq_id));
//   pos+= sizeof(seq_id);
//   memcpy(buf+pos,&curr_rank, sizeof(curr_rank));
//   pos+= sizeof(curr_rank);
//   memcpy(buf+pos,&send_type_flag, sizeof(send_type_flag));
//   pos+= sizeof(send_type_flag);
//   // printf("send \n");
//   uip_udp_packet_sendto(mcast_conn, buf, pos,
//                   &multicast_ipaddr, UIP_HTONS(MCAST_TEST_UDP_PORT));

// }
/*---------------------------------------------------------------------------*/
/*
static void
print_local_addresses(void)
{
  int i;
  uint8_t state;

  PRINTF("Server IPv6 addresses: ");
  for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
    state = uip_ds6_if.addr_list[i].state;
    if(state == ADDR_TENTATIVE || state == ADDR_PREFERRED) {
      PRINT6ADDR(&uip_ds6_if.addr_list[i].ipaddr);
      PRINTF("\n");
      // hack to make address "final" 
      if (state == ADDR_TENTATIVE) {
  uip_ds6_if.addr_list[i].state = ADDR_PREFERRED;
      }
    }
  }
} */
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_server_process, ev, data)
{
  uip_ipaddr_t ipaddr;
  struct uip_ds6_addr *root_if;
  static struct etimer root_multisend_timer;
  PROCESS_BEGIN();

  PROCESS_PAUSE();

  //SENSORS_ACTIVATE(button_sensor);

  PRINTF("UDP server started\n");

#if UIP_CONF_ROUTER
/* The choice of server address determines its 6LoPAN header compression.
 * Obviously the choice made here must also be selected in udp-client.c.
 *
 * For correct Wireshark decoding using a sniffer, add the /64 prefix to the 6LowPAN protocol preferences,
 * e.g. set Context 0 to aaaa::.  At present Wireshark copies Context/128 and then overwrites it.
 * (Setting Context 0 to aaaa::1111:2222:3333:4444 will report a 16 bit compressed address of aaaa::1111:22ff:fe33:xxxx)
 * Note Wireshark's IPCMV6 checksum verification depends on the correct uncompressed addresses.
 */
 
#if 0
/* Mode 1 - 64 bits inline */
   uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 1);
#else
/* Mode 3 - derived from link local (MAC) address */
  uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
#endif
  uip_create_linklocal_allnodes_mcast(&multicast_ipaddr);

 uint8_t i=0;
 for ( i=0; i< NODE_MAX ; i++)
  {
    node_array[i].addr = 0xffff ;
    node_array[i].upcount = 0 ;
    node_array[i].last_count = 0;
    node_array[i].duplicate_count = 0;
  }
  uip_ds6_addr_add(&ipaddr, 0, ADDR_MANUAL);
  root_if = uip_ds6_addr_lookup(&ipaddr);
  if(root_if != NULL) {
    rpl_dag_t *dag;
    dag = rpl_set_root(RPL_DEFAULT_INSTANCE,(uip_ip6addr_t *)&ipaddr);
    uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
    rpl_set_prefix(dag, &ipaddr, 64);
    PRINTF("created a new RPL dag\n");
  } else {
    PRINTF("failed to create a new RPL DAG\n");
  }
#endif /* UIP_CONF_ROUTER */
  
//  print_local_addresses();

  /* The data sink runs with a 100% duty cycle in order to ensure high 
     packet reception rates. */
 // NETSTACK_MAC.off(1);
  server_conn = udp_new(NULL, UIP_HTONS(UDP_CLIENT_PORT), NULL);

  if(server_conn == NULL) {
    PRINTF("No UDP connection available, exiting the process!\n");
    PROCESS_EXIT();
  }
  udp_bind(server_conn, UIP_HTONS(UDP_SERVER_PORT));

  multicast_server_conn = udp_new(NULL, UIP_HTONS(MCAST_TEST_UDP_PORT), NULL);
  if(multicast_server_conn == NULL) {
    PRINTF("No UDP connection available for multicast, exiting the process!\n");
    PROCESS_EXIT();
  }
  udp_bind(multicast_server_conn, UIP_HTONS(MCAST_TEST_UDP_PORT));
  PRINTF("Created a server connection with remote address ");
  PRINT6ADDR(&server_conn->ripaddr);
  PRINTF(" local/remote port %u/%u\n", UIP_HTONS(server_conn->lport),
         UIP_HTONS(server_conn->rport));
  PRINTF(" multicast local/remote port %u/%u\n", UIP_HTONS(multicast_server_conn->lport),
         UIP_HTONS(multicast_server_conn->rport));
  while(1) {
    PROCESS_YIELD();
    extern uint8_t app_send_flag;
    if(ev == tcpip_event) {
      tcpip_handler();
      etimer_set(&root_multisend_timer , ROOT_SEND_INTERVAL);
      app_send_flag = 0;
    }else if(ev == PROCESS_EVENT_TIMER){
      if(data == &root_multisend_timer){
        app_send_flag = 1;
        uip_udp_packet_sendto(multicast_server_conn, "Root_Multicast", sizeof("Root_Multicast"),
              &multicast_ipaddr, UIP_HTONS(MCAST_TEST_UDP_PORT));
        app_send_flag = 0;
      }
    }

    /*else if (ev == sensors_event && data == &button_sensor) {
      PRINTF("Initiaing global repair\n");
      rpl_repair_root(RPL_DEFAULT_INSTANCE);
    } */
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
