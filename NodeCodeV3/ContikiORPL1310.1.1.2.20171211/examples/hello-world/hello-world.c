/*
 * Copyright (c) 2006, Swedish Institute of Computer Science.
 * All rights reserved.
 *
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

/**
 * \file
 *         A very simple Contiki application showing how Contiki programs look
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include "contiki.h"
#include "dev/watchdog.h"
#include "node-id.h"
// #include "bat-voltage.h"

#include <stdio.h> /* For printf() */
#include <string.h>
// #include "cpu/cc26xx-cc13xx/dev/soc-rtc.h"
#include "dev/serial-line.h"
#include "dev/uart1.h"
#include "dev/leds.h"
#include "node-id.h"

unsigned char my_cmd[14]={0x05,0x10,0x5B,0xFE,0x59,0x16};
/*---------------------------------------------------------------------------*/
PROCESS(hello_world_process, "Hello world process");
AUTOSTART_PROCESSES(&hello_world_process);

PROCESS_THREAD(hello_world_process, ev, data)
{

  PROCESS_BEGIN();
  // // // // //printf(" %x\n",get_voltage());
  static struct etimer et;
  static char mybuf[5];
  static int rssi=-110;
  etimer_set(&et,1*CLOCK_SECOND);
  // // // unsigned long cpu,lpm,transmit,listen,irq;
  // uint8_t my_cmd[14]={8,0x08,9,0x09,10,0x0A,11,0x0B,12,0x0C,16,0x10};
  // uint8_t my_cmd[14]={1,2,3,4,5,6,7,8,9,10,11,12};
  uart1_set_input(serial_line_input_byte2);
  while(1)
  {
     PROCESS_YIELD();
     if(etimer_expired(&et)&& ev==PROCESS_EVENT_TIMER)
     {
         // if(cmd_read_meter[0] != 0xFF)
         // {
         //    // uart1_set_input(serial_line_input_byte2);
         //    for(int i=0;i< cmd_read_meter[0];i++)
         //    {
         //      printf("%c",(unsigned char )cmd_read_meter[i+1]);
         //    }

         // }
         rssi=-110;
         printf("before: %d ",rssi);
         memcpy(mybuf,&rssi,sizeof(rssi));
         rssi = 0;
         memcpy(&rssi,(int *)mybuf,sizeof(rssi));
         printf("after: %d\n",rssi);
  //      for(int i=1;i<6;i++){
  //          unsigned char ch= my_cmd[i];
  //          printf("%c",ch);
  //          // cc26xx_uart_write_byte(ch);
  //      }
  //         //  logic =logic^1;
  //         // led_toggle(logic);
  // // // //     // cpu      = energest_type_time(ENERGEST_TYPE_CPU)      ;
  // // // //     // lpm      = energest_type_time(ENERGEST_TYPE_LPM)      ;
  // // // //     // listen   = energest_type_time(ENERGEST_TYPE_LISTEN)   ;
  // // // //     // //printf("cpu  %lu  , lpm  %lu\n", cpu,lpm);
  // // // //     // printf("listen  %lu\n", listen);
  // // // //     // energest_type_set(ENERGEST_TYPE_CPU,0);
  // // // //     // energest_type_set(ENERGEST_TYPE_LPM,0);
  // // // //     // energest_type_set(ENERGEST_TYPE_LISTEN,0);
  //     // printf("hello-world!\n");
  // //     printf("%lu   ",first );
  // //     printf("%lu\n",second );
  // // //     printf("tick:%lu\n",timecount);
  // // // //     //printf("voltage: %d\n",get_voltage());
  // // // //    //  // printf("restart count %d\n",restart_count );
  // // // //    //  restart_count_byte_restore();
  // // // //    //  printf("restart count %d\n",restart_count );
  // // // //    //  restart_count++;
  // // // //    //  restart_count_byte_burn(restart_count);
      etimer_set(&et,5*CLOCK_SECOND);
     }

    if (ev==serial_line_event_message2)
    {
      int  data_length =(int)((char *)data)[0];
      // for(int i=0;i<data_length;i++){
          // leds_toggle(LEDS_RED);
      // }
     // printf("data len:%d\n",data_length );
      // uart1_set_input(NULL);

    }

  }
 
 PROCESS_END();

}

