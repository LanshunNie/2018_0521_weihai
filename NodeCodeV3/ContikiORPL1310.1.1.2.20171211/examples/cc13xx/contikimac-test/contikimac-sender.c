#include "contiki.h"
#include "net/netstack.h"

#include <stdio.h>
#include <string.h>
#define PERIOD 1
#define SEND_INTERVAL (PERIOD*CLOCK_SECOND)
// static int i=0;
PROCESS(radio_sender_process,"radio sender process");
AUTOSTART_PROCESSES(&radio_sender_process);
PROCESS_THREAD(radio_sender_process,ev,data)
{
	
	static struct etimer periodic;
	PROCESS_BEGIN();
    printf("%s\n","Startting sender");
	etimer_set(&periodic,SEND_INTERVAL);
    char buf[20];
    char *sender=buf;
   
	while(1){
		PROCESS_YIELD_UNTIL(ev==PROCESS_EVENT_TIMER);
		if(etimer_expired(&periodic)){
			etimer_reset(&periodic);
			sprintf(buf, "ACK1");
			NETSTACK_RADIO.on();
		    NETSTACK_RADIO.send(buf,sizeof(buf));
		    NETSTACK_RADIO.off();
			printf("send %s\n",sender);
		}
	}
	
	PROCESS_END();
}