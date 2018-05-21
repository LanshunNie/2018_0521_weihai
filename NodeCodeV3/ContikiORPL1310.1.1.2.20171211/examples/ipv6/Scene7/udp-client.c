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
#include "lib/random.h"
#include "net/ip/uip.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ip/uip-udp-packet.h"
#include "net/rpl/rpl.h"
#ifdef WITH_COMPOWER
#include "powertrace.h"
#endif
#include <stdio.h>
#include <string.h>

#include "task-schedule.h"

#include "cpu/cc26xx-cc13xx/dev/netconfig.h"
#include "net/ipv6/sicslowpan.h"
#define UIP_MCAST6_CONF_ENGINE UIP_MCAST6_ENGINE_SCF
#define UDP_CLIENT_PORT 8765
#define UDP_SERVER_PORT 5678

#define UDP_EXAMPLE_ID  190

// #define DEBUG DEBUG_PRINT
#define DEBUG 0
#include "net/ip/uip-debug.h"

#ifndef PERIOD
#define PERIOD  20
#endif


#define MCAST_TEST_UDP_PORT 6105 /* Host byte order */
#define INACTIVE_INTERVAL (600 * CLOCK_SECOND)
#define REACTIVE_INTERVAL (600 * 35*CLOCK_SECOND)
// #define START_INTERVAL    (10 * CLOCK_SECOND)
#define SEND_INTERVAL   (PERIOD * CLOCK_SECOND)
#define PRINT_INTERVAL (5 * CLOCK_SECOND)
// #define SEND_TIME   (random_rand() % (SEND_INTERVAL))
#define SEND_TIME   (SEND_INTERVAL/2)
#define MAX_PAYLOAD_LEN   30

static struct uip_udp_conn *client_conn;
static struct uip_udp_conn * mcast_conn;
static uip_ipaddr_t server_ipaddr;
static uip_ipaddr_t multicast_ipaddr;
static uint16_t seq_id ;
static uint8_t init_severaddr_flag;
static uint8_t ten_minute_period_count = 0;
static uint8_t send_type_flag = 1;
static uint8_t send_packet_count = 1;
/*---------------------------------------------------------------------------*/
PROCESS(udp_client_process, "UDP client process");
AUTOSTART_PROCESSES(&udp_client_process);
/*---------------------------------------------------------------------------*/
static void
tcpip_handler(void)
{
  char *str;

  if(uip_newdata()) {
    str = uip_appdata;
    str[uip_datalen()] = '\0';
   // printf("DATA recv '%s'\n", str);
  }
}
/*---------------------------------------------------------------------------*/
static void
set_server_address(void)
{
  uip_ip6addr_t *globaladdr = NULL;
  uip_ds6_addr_t * addr_desc = uip_ds6_get_global(ADDR_PREFERRED);
  if(addr_desc != NULL) {
     globaladdr = &addr_desc->ipaddr;
#if UIP_CONF_IPV6_RPL
     rpl_dag_t *dag = rpl_get_any_dag();
     if(dag) {
        init_severaddr_flag =1;
        uip_ipaddr_copy(&server_ipaddr, globaladdr);
        memcpy(&server_ipaddr.u8[8], &dag->dag_id.u8[8], sizeof(uip_ipaddr_t) / 2);  

        uip_create_linklocal_allnodes_mcast(&multicast_ipaddr);
    }
#endif
   }
}
/*---------------------------------------------------------------------------*/
static void
print_energest(void){
  unsigned long cpu,lpm,transmit,listen,receive;
  cpu = energest_type_time(ENERGEST_TYPE_CPU);
  lpm = energest_type_time(ENERGEST_TYPE_LPM);
  transmit = energest_type_time(ENERGEST_TYPE_TRANSMIT);
  listen = energest_type_time(ENERGEST_TYPE_LISTEN);
  receive = energest_type_time(ENERGEST_TYPE_RECEIVE);
  // printf("Period %d:\tcpu: %lu\tlpm: %lu\ttransmit: %lu\tlisten: %lu\n",++ten_minute_period_count,cpu,lpm,transmit,listen );
  if(init_severaddr_flag == 0){
    printf("Period START:\tcpu: %lu\tlpm: %lu\ttransmit: %lu\tlisten: %lu\treceive: %lu\n",cpu,lpm,transmit,listen,receive);
  }else{
    printf("Period END:\tcpu: %lu\tlpm: %lu\ttransmit: %lu\tlisten: %lu\treceive: %lu\n",cpu,lpm,transmit,listen,receive);
  }
  return;
}
/*---------------------------------------------------------------------------*/
static void
send_packet(void *ptr)
{
  char buf[MAX_PAYLOAD_LEN];
  uint16_t id;
  int pos;
  send_type_flag ++;
  send_type_flag = send_type_flag%2;
  pos = 0;
  memset(buf,0,10);

  rpl_dag_t *dag = rpl_get_any_dag();
  if(dag == NULL){
     return;
  }
  extern uint8_t app_send_flag;
  if(init_severaddr_flag==0){
     set_server_address();
  }
  seq_id++;
  id=uip_htons(seq_id);
  rpl_rank_t curr_rank = dag->rank;
  // printf("DATA send to '%d'\n", seq_id);
  // PRINTF("DATA send to %d '%d'\n",
  //        server_ipaddr.u8[sizeof(server_ipaddr.u8) - 1], seq_id);
  memcpy(buf,&id,sizeof(seq_id));
  pos+= sizeof(seq_id);
  memcpy(buf+pos,&curr_rank, sizeof(curr_rank));
  pos+= sizeof(curr_rank);
  memcpy(buf+pos,&send_type_flag, sizeof(send_type_flag));
  pos+= sizeof(send_type_flag);
  // printf("send \n");
  if(send_type_flag == 0){
    app_send_flag = 1;
    uip_udp_packet_sendto(client_conn, buf, pos,
                      &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));
    // app_send_flag = 0;
  }
  else if(send_type_flag == 1){
    app_send_flag = 1;
    uip_udp_packet_sendto(mcast_conn, buf, pos,
                    &multicast_ipaddr, UIP_HTONS(MCAST_TEST_UDP_PORT));
    // app_send_flag = 0;
  }

  // PRINT6ADDR(&server_ipaddr);
  // PRINTF("\n");
}
static void
send_uni_packet(void *ptr)
{
  // printf("send_uni\n");
  char buf[MAX_PAYLOAD_LEN];
  uint16_t id;
  int pos;
  send_type_flag = 0;
  pos = 0;
  memset(buf,0,10);

  rpl_dag_t *dag = rpl_get_any_dag();
  if(dag == NULL){
     return;
  }
  extern uint8_t app_send_flag;
  // printf("send_uni_packet\n");
  if(init_severaddr_flag==0){
     // print_energest();
     set_server_address();
  }
  seq_id++;
  id=uip_htons(seq_id);
  rpl_rank_t curr_rank = dag->rank;
  // printf("DATA send to '%d'\n", seq_id);
  // PRINTF("DATA send to %d '%d'\n",
  //        server_ipaddr.u8[sizeof(server_ipaddr.u8) - 1], seq_id);
  memcpy(buf,&id,sizeof(seq_id));
  pos+= sizeof(seq_id);
  memcpy(buf+pos,&curr_rank, sizeof(curr_rank));
  pos+= sizeof(curr_rank);
  memcpy(buf+pos,&send_type_flag, sizeof(send_type_flag));
  pos+= sizeof(send_type_flag);
  // printf("send \n");
  app_send_flag = 1;
  uip_udp_packet_sendto(client_conn, buf, pos,
                    &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));
  // app_send_flag = 0;
  send_packet_count++;
  // init_severaddr_flag = 0;
  // print_energest();       
  // init_severaddr_flag = 1; 
}
static void
send_multi_packet(void *ptr)
{
  // printf("send_multi\n");
  char buf[MAX_PAYLOAD_LEN];
  uint16_t id;
  int pos;
  
  send_type_flag = 1;
  pos = 0;
  memset(buf,0,10);

  rpl_dag_t *dag = rpl_get_any_dag();
  if(dag == NULL){
     return;
  }
  extern uint8_t app_send_flag;
  // printf("send_multi_packet\n");
  if(init_severaddr_flag==0){
     // print_energest();
     set_server_address();
  }
  seq_id++;
  id=uip_htons(seq_id);
  rpl_rank_t curr_rank = dag->rank;
  // printf("DATA send to '%d'\n", seq_id);
  // PRINTF("DATA send to %d '%d'\n",
  //        server_ipaddr.u8[sizeof(server_ipaddr.u8) - 1], seq_id);
  memcpy(buf,&id,sizeof(seq_id));
  pos+= sizeof(seq_id);
  memcpy(buf+pos,&curr_rank, sizeof(curr_rank));
  pos+= sizeof(curr_rank);
  memcpy(buf+pos,&send_type_flag, sizeof(send_type_flag));
  pos+= sizeof(send_type_flag);
  // printf("send \n");
  app_send_flag = 1;
  uip_udp_packet_sendto(mcast_conn, buf, pos,
                  &multicast_ipaddr, UIP_HTONS(MCAST_TEST_UDP_PORT));
  // app_send_flag = 0;
  send_packet_count++;

  // PRINT6ADDR(&server_ipaddr);
  // PRINTF("\n");
}

/*---------------------------------------------------------------------------*/
/*
static void
print_local_addresses(void)
{
  int i;
  uint8_t state;

  PRINTF("Client IPv6 addresses: ");
  for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
    state = uip_ds6_if.addr_list[i].state;
    if(uip_ds6_if.addr_list[i].isused &&
       (state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
      PRINT6ADDR(&uip_ds6_if.addr_list[i].ipaddr);
      PRINTF("\n");
      // hack to make address "final" 
      if (state == ADDR_TENTATIVE) {
  uip_ds6_if.addr_list[i].state = ADDR_PREFERRED;
      }
    }
  }
}*/
/*---------------------------------------------------------------------------*/
static void
set_global_address(void)
{
  uip_ipaddr_t ipaddr;

  uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
  uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);

/* The choice of server address determines its 6LoPAN header compression.
 * (Our address will be compressed Mode 3 since it is derived from our link-local address)
 * Obviously the choice made here must also be selected in udp-server.c.
 *
 * For correct Wireshark decoding using a sniffer, add the /64 prefix to the 6LowPAN protocol preferences,
 * e.g. set Context 0 to aaaa::.  At present Wireshark copies Context/128 and then overwrites it.
 * (Setting Context 0 to aaaa::1111:2222:3333:4444 will report a 16 bit compressed address of aaaa::1111:22ff:fe33:xxxx)
 *
 * Note the IPCMV6 checksum verification depends on the correct uncompressed addresses.
 */
 
#if 1
/* Mode 1 - 64 bits inline */
  uip_ip6addr(&server_ipaddr, 0xaaaa,0,0,0,0x0012,0x7400,0x0001,0x0028);
  //   uip_ip6addr(&server_ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 1);
#else
/* Mode 3 - derived from server link-local (MAC) address */
  uip_ip6addr(&server_ipaddr, 0xaaaa, 0, 0, 0, 0x0250, 0xc2ff, 0xfea8, 0xcd1a); //redbee-econotag
#endif
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_client_process, ev, data)
{
  static struct etimer inactive_trigger;
  static struct etimer periodic;
  static struct ctimer backoff_timer;
  static struct etimer print_timer;
  static struct etimer reactive_trigger;
#if WITH_COMPOWER
  static int print = 0;
#endif

  PROCESS_BEGIN();

  PROCESS_PAUSE();

  set_global_address();
 // PRINTF("UDP client process started\n");

  init_severaddr_flag =0;
  seq_id =0;
 // print_local_addresses();

  /* new connection with remote host */
  client_conn = udp_new(NULL, UIP_HTONS(UDP_SERVER_PORT), NULL); 
  if(client_conn == NULL) {
    PRINTF("No UDP connection available, exiting the process!\n");
    PROCESS_EXIT();
  }
  udp_bind(client_conn, UIP_HTONS(UDP_CLIENT_PORT)); 

  mcast_conn = udp_new( NULL, UIP_HTONS(MCAST_TEST_UDP_PORT), NULL);
  if(mcast_conn == NULL) {
    PRINTF("No UDP connection available for multicast, exiting the process!\n");
    PROCESS_EXIT();
  }
  udp_bind(mcast_conn, UIP_HTONS(MCAST_TEST_UDP_PORT));

 // PRINTF("Created a connection with the server ");
//PRINT6ADDR(&client_conn->ripaddr);
 // PRINTF(" local/remote port %u/%u\n",
  //UIP_HTONS(client_conn->lport), UIP_HTONS(client_conn->rport));

#if WITH_COMPOWER
  powertrace_sniff(POWERTRACE_ON);
#endif

   // etimer_set(&periodic, SEND_INTERVAL);
   etimer_set(&inactive_trigger, INACTIVE_INTERVAL);
   etimer_set(&reactive_trigger, REACTIVE_INTERVAL);
   // etimer_set(&print_timer, PRINT_INTERVAL);

  

  while(1) {
    PROCESS_YIELD();
    if(ev == tcpip_event) {
      tcpip_handler();
    }
  if(ev == PROCESS_EVENT_TIMER) {
       if(data==&periodic) {
        extern uint8_t test_use_flag;
        // if(test_use_flag != 0 && send_packet_count < 5){
        if(get_active_flag()!=0){
          etimer_reset(&periodic);
        }
        rpl_dag_t *dag = rpl_get_any_dag();
        if(dag == NULL){
           continue;
        }
        if(init_severaddr_flag == 0){
          print_energest();
        }
        // }
        // ctimer_set(&backoff_timer, SEND_TIME, send_packet, NULL);          
        // if(send_packet_count <= 3){
        if(send_packet_count != 3 ){
          ctimer_set(&backoff_timer, SEND_TIME, send_uni_packet, NULL);  
          // send_uni_packet(NULL);

        // }else if(send_packet_count > 3 &&send_packet_count <= 5){
        }else if(send_packet_count == 3){        
          ctimer_set(&backoff_timer, SEND_TIME, send_multi_packet, NULL);   
          // send_multi_packet(NULL);
        }
   #if WITH_COMPOWER
        if (print == 0) {
     			powertrace_print("#P");
         }
         if (++print == 3) {
           print = 0;
        }
   #endif
      }else if(data == &inactive_trigger){
        extern uint8_t test_use_flag;
        test_use_flag = 0;
        // print_energest();
        // set_active_flag();
      }else if(data == &reactive_trigger){
        extern uint8_t test_use_flag ;
        test_use_flag = 1;
        print_energest();
      }else if(data == &print_timer){
        etimer_reset(&print_timer);
        print_energest();
      }

    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
