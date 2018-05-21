/*
* @Author: Guoxuenan
* @Date:   2016-07-07 09:53:31
* @Last Modified by:   Guoxuenan
* @Last Modified time: 2016-08-12 17:00:05
*/
#include "contiki.h"
#include "node_function.h"
#include "net/ip/uip.h"
#include "net/ipv6/uip-ds6.h"
#include "net/rpl/rpl.h"
#include "bat-voltage.h"
#include "contikimac.h"

#include "netsync-auto-calibrate.h"
//provide reboot service
#include "dev/watchdog.h"

#include <stdio.h>
#define DEBUG 0
#if DEBUG 
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

//zhangwei set changed for load balance
#if WITH_ENERGY_EFFICIENCY
#include "net/mac/energy-efficiency/energy-efficiency.h"
#include "rdc-efficiency.h"
#else
#include "sys/energest.h"
#define LPM_CURRENT        2100
#define CPU_CURRENT        2505000
#define CPU_SLEEP_CURRENT  1350000
#define LISTEN_CURRENT     33420130
#define TRANSMIT_CURRENT   21448000
#define RECEIVE_CURRENT    6535300
#define UNI_CARRIAGE_TIME  368
static uint32_t energy_efficiency_cal_present_current(void);
#endif
static uint8_t netDataPeriod =1;

// static uint64_t last_cpu      = 0;
// static uint64_t last_lpm      = 0;
// static uint64_t last_transmit = 0;
// static uint64_t last_listen   = 0;
static struct ctimer ct;
static void set_energy(int index,uint8_t array[],uint64_t val)
{
  int i ;

    for (i=5; i >=0; i--)
  {
    /* code */
     array[index+i]=0;
     // val=val>>8;
  }

  for (i=5; i >=0; i--)
  {
    /* code */
     array[index+i]=val&0xff;
     val=val>>8;
  }
}
/*--------------------------ADC voltage------------------------------------------------------------*/

/*--------------------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------------------*/
/*system monitoring instruction*/
void get_system_monitor_msg(uint8_t array[],int length)
{

   uint64_t cpu,lpm,transmit,listen;
   uint16_t rtmetric;
   uint16_t beacon_interval;
   uint16_t num_neighbors;
   uint16_t temp_votlage=0;
   rpl_parent_t *preferred_parent;
   rpl_dag_t *dag;


   soft_time timenow;

  if (length<SYSTEM_MONITOR_MSG_LENGTH)
  {
    PRINTF("node_function.c array error msg \n");
    return;
  }
   get_timenow(&timenow);

   array[INDEX_TIME]  =   timenow.hour;
   array[INDEX_TIME+1]=   timenow.minute;
   array[INDEX_TIME+2]=   timenow.sec;

//zhangwei set changed for load balance
   array[INDEX_CYCLETIME]  = (get_cycle_time()>>8) & 0xff;         
   array[INDEX_CYCLETIME+1]=  get_cycle_time()&0xff;

#if WITH_ENERGY_EFFICIENCY
   array[INDEX_CYCLETIME]  = (ENERGY_EFFICIENCY_GET(rdc_cycle_time)>>8) & 0xff;         
   array[INDEX_CYCLETIME+1]=  ENERGY_EFFICIENCY_GET(rdc_cycle_time)&0xff;
   array[INDEX_CYCLETIME_DIRECTION]= ENERGY_EFFICIENCY_GET(cycle_time_direction);

   array[INDEX_CURRENT_BUDGET] = (ENERGY_EFFICIENCY_GET(current_budget)>>24) & 0xff;
   array[INDEX_CURRENT_BUDGET+1] = (ENERGY_EFFICIENCY_GET(current_budget)>>16) & 0xff;
   array[INDEX_CURRENT_BUDGET+2] = (ENERGY_EFFICIENCY_GET(current_budget)>>8) & 0xff;
   array[INDEX_CURRENT_BUDGET+3] = (ENERGY_EFFICIENCY_GET(current_budget)) & 0xff;

   array[INDEX_LASTCURRENT] = (ENERGY_EFFICIENCY_GET(prev_current)>>24) & 0xff;
   array[INDEX_LASTCURRENT+1] = (ENERGY_EFFICIENCY_GET(prev_current)>>16) & 0xff;
   array[INDEX_LASTCURRENT+2] = (ENERGY_EFFICIENCY_GET(prev_current)>>8) & 0xff;
   array[INDEX_LASTCURRENT+3] = (ENERGY_EFFICIENCY_GET(prev_current)) & 0xff;
#else
   uint32_t prev_current = 0;
   prev_current = energy_efficiency_cal_present_current();
   array[INDEX_LASTCURRENT] = (prev_current>>24) & 0xff;
   array[INDEX_LASTCURRENT+1] = (prev_current>>16) & 0xff;
   array[INDEX_LASTCURRENT+2] = (prev_current>>8) & 0xff;
   array[INDEX_LASTCURRENT+3] = (prev_current) & 0xff;
#endif

  energest_flush();
  cpu      = energest_type_time(ENERGEST_TYPE_CPU)      ;
  lpm      = energest_type_time(ENERGEST_TYPE_LPM)      ;
  transmit = energest_type_time(ENERGEST_TYPE_TRANSMIT) ;
  listen   = energest_type_time(ENERGEST_TYPE_LISTEN)   ;

  PRINTF("cpu  %lu  , lpm  %lu , transmit %lu , listen %lu  \n", cpu,lpm,transmit,listen );

  // energest_type_set(ENERGEST_TYPE_CPU,0);
  // energest_type_set(ENERGEST_TYPE_LPM,0);
  // energest_type_set(ENERGEST_TYPE_TRANSMIT,0);
  // energest_type_set(ENERGEST_TYPE_LISTEN,0);
  // energest_type_set(ENERGEST_TYPE_IRQ,0);

  set_energy(INDEX_ENERGYCOST    , array , cpu         )      ;
  set_energy(INDEX_ENERGYCOST+6  , array , lpm         )      ;
  set_energy(INDEX_ENERGYCOST+12 , array , transmit)      ;
  set_energy(INDEX_ENERGYCOST+18 , array , listen   )      ;
  

// last_cpu         = cpu     ;
// last_lpm         = lpm     ;
// last_transmit    =transmit;
// last_listen      = listen  ;
  
  dag = rpl_get_any_dag();
  if(dag != NULL) 
  {
    preferred_parent = dag->preferred_parent;
    if(preferred_parent != NULL) 
    {
      uip_ds6_nbr_t *nbr;
      nbr = uip_ds6_nbr_lookup(rpl_get_parent_ipaddr(preferred_parent));
      if(nbr != NULL) 
      {
        /* Use parts of the IPv6 address as the parent address, in reversed byte order. */
        array[INDEX_TOPO] = nbr->ipaddr.u8[sizeof(uip_ipaddr_t) - 2];
        array[INDEX_TOPO+1] = nbr->ipaddr.u8[sizeof(uip_ipaddr_t) - 1];
      }
    }

    rtmetric = dag->rank;
    beacon_interval = (uint16_t) ((1L << dag->instance->dio_intcurrent) / 1000);
    num_neighbors = uip_ds6_nbr_num();

  }
  else 
  {
    rtmetric        = 0;
    beacon_interval = 0;
    num_neighbors   = 0;
  }      

  array[INDEX_PARENTRSSI]       = 0; 
  array[INDEX_PARENTRSSI+1]     = 0;;

  temp_votlage = (uint32_t)get_voltage();
  array[INDEX_ADCVOLTAGE]       = (temp_votlage>>8)&0xff; //directly show the value of voltage
  array[INDEX_ADCVOLTAGE+1]     = temp_votlage&0xff;

  array[INDEX_BEACON_INTERVAL]  = (beacon_interval>>8) & 0xff;         
  array[INDEX_BEACON_INTERVAL+1]=  beacon_interval&0xff;
  
  array[INDEX_NUM_NEIGHBORS]    = (num_neighbors>>8) & 0xff;         
  array[INDEX_NUM_NEIGHBORS+1]  = num_neighbors&0xff;

  array[INDEX_RTMETRIC]         = (rtmetric>>8) & 0xff;        
  array[INDEX_RTMETRIC+1]       = rtmetric&0xff  ;  

  array[INDEX_TIME_DIFF]        = netsynch_get_offset();

  array[INDEX_RESTART_COUNT] = restart_count;
  // printf("restart count %d\n",restart_count );//by huangxiaobing
  
  struct netsync_cal_s *cal_info = get_autocal_info();

  memcpy(array+INDEX_AUTOCAL_INTERVAL,&(cal_info->autocal_interval) ,4);
  memcpy(array+INDEX_CAL_OFFSET, &(cal_info->cal_offest) ,2);

        #if DEBUG   

            static int i ; 
            i=0;

            PRINTF("\ntime ");
            while (i<INDEX_TOPO)
            {
              PRINTF(":%d",array[i]);
              i++;
            }

            PRINTF("\ntopo ");
            while (i<INDEX_ENERGYCOST)
            {
              PRINTF(":%x",array[i]);
              i++;
            }

            PRINTF("\nenergy ");
            while (i<INDEX_ADCVOLTAGE)
            {
              PRINTF("%02x ",array[i]);
              i++;
            }
              PRINTF("\nvoltage ");
            while (i<INDEX_BEACON_INTERVAL)
            {
              PRINTF("%02x ",array[i]);
              i++;
            }
            PRINTF("\ninterval ");
               while (i<INDEX_NUM_NEIGHBORS)
            {
              PRINTF("%02x ",array[i]);
              i++;
            }
              PRINTF("\nnum_neighbors ");
               while (i<INDEX_RTMETRIC)
            {
              PRINTF("%02x ",array[i]);
              i++;
            }
              PRINTF("\n rtmetric");
               while (i<INDEX_TIME_DIFF)
            {
              PRINTF("%02x ",array[i]);
              i++;
            }
              PRINTF("\n time-diff");
               while (i<INDEX_TIME_DIFF+1)
            {
              PRINTF("%02x ",array[i]);
              i++;
            }
            PRINTF("\n");
              PRINTF("\n restart count:");
            while (i<INDEX_RESTART_COUNT+1)
            {
              PRINTF("%02x ",array[i]);
              i++;
            }
            PRINTF("\n");
        #endif 
}
/*--------------------------------------------------------------------------------------*/


// void get_system_monitor_msg1(uint8_t array[],int length)
// {

//   if(length<SYSTEM_MONITOR_MSG_LENGTH1)
//   {
//     PRINTF("node_function.c array error msg1 \n");
//     return;
//   }
//   array[INDEX_PANID]           = 0; 
//   array[INDEX_CHANNEL ]        = ((channel_byte>>8)&0xff) | (channel_byte&0xff);
//   array[INDEX_CCATHR]          = get_cca_thr();
//   array[INDEX_TRANSMIT_POWER]  = get_transmit_power();
//   array[INDEX_ACTIVE_CCA_CHECKT_RATE] = get_rdc_active_channel_check_rate();
//   array[INDEX_INACTIVE_CCA_CHECKT_INTERVAL] = get_rdc_inactive_channel_check_interval();

// #if DEBUG   
//       PRINTF("panid %d\n",  array[INDEX_PANID]   );
//       PRINTF("channel %x\n", array[INDEX_CHANNEL ] );
//       PRINTF("cca thr %x\n", array[INDEX_CCATHR]  );
//       PRINTF("transmit power %x\n", array[INDEX_TRANSMIT_POWER]);
//       PRINTF("cca active rate %d\n", array[INDEX_ACTIVE_CCA_CHECKT_RATE]);
//       PRINTF("cca inactive rate %d\n", array[RFCHANNEL_ACTIVE_CHECK_INTERVAL]);

// #endif

// }

/*--------------------------------------------------------------------------------------*/
/*Network Configuration instruction*/


// setting panid cca-check-rate channel cca thr 
// void setting_network_configuration(uint8_t array[],int length)
// {

//   #define MASK_CODE 0xFF

//   if(length!=NETWORK_CONF_LENGTH)
//   {
//     PRINTF("node_function.c array error conf \n");
//     return ;
//   }
  
//   PRINTF("set:%x,%x,%x,%x,%x,%x\n",array[0],array[1],array[2],array[3],array[4],array[5]);
//   // array[ PANID ]
//  //  if(array[RFCHANNEL_INACTIVE_CHECK_RATE]!=MASK_CODE)
//  //   set_rdc_active_channel_check_rate(array[RFCHANNEL_INACTIVE_CHECK_RATE]);

//  // if(array[RFCHANNEL_ACTIVE_CHECK_INTERVAL]!=MASK_CODE)
//  //   set_rdc_inactive_channel_check_interval(array[RFCHANNEL_ACTIVE_CHECK_INTERVAL]);
  
//   // array[ PANID ]
//   // array[ RFCHANNEL_CHECK_RATE ] 
//   // array[ RFCHANNEL_CHECK_RATE ] 
//   if(array[RFCHANNEL] !=MASK_CODE)
//   {
//     // cc1120_channel_set(array[RFCHANNEL]);        
//     channel_byte_burn(array[RFCHANNEL]);
//     channel_byte_restore();   
//   }

//   // set_cca_thr_and_transmit_power(array[CCATHR],array[RFTRANSMITPOWER]);

//   if(array[RPL_CONF_DIO_INTERVAL_MIN_CONF] !=MASK_CODE || array[RPL_CONF_DIO_INTERVAL_DOUBLINGS_CONF] !=MASK_CODE)
//     set_rpl_dio_interval_min_doublings(array[RPL_CONF_DIO_INTERVAL_MIN_CONF],array[RPL_CONF_DIO_INTERVAL_DOUBLINGS_CONF]);
// }
// set topo heart period ;
#if !WITH_ENERGY_EFFICIENCY
static uint32_t 
energy_efficiency_cal_present_current(void){
  static uint64_t l_transmit = 0;
  static uint64_t l_listen = 0; 
  static uint64_t l_lpm = 0;
  static uint64_t l_cpu = 0;
  static uint64_t l_receive = 0;
  uint64_t transmit =0,listen=0,lpm=0,cpu=0,receive=0;

  uint32_t present_cur=0;

  energest_flush();
  
  cpu      = energest_type_time(ENERGEST_TYPE_CPU)-l_cpu;//mlc change: current computed by last round rather than all rounds
  lpm      = energest_type_time(ENERGEST_TYPE_LPM)-l_lpm;
  transmit = energest_type_time(ENERGEST_TYPE_TRANSMIT)-l_transmit;
  listen   = energest_type_time(ENERGEST_TYPE_LISTEN)-l_listen;
  receive   = energest_type_time(ENERGEST_TYPE_RECEIVE)-l_receive;

  l_listen = energest_type_time(ENERGEST_TYPE_LISTEN);
  l_transmit = energest_type_time(ENERGEST_TYPE_TRANSMIT);
  l_lpm = energest_type_time(ENERGEST_TYPE_LPM);
  l_cpu = energest_type_time(ENERGEST_TYPE_CPU);
  l_receive = energest_type_time(ENERGEST_TYPE_RECEIVE);
  uint64_t uni_carriages = getUniCarriages();
  uint64_t temp1 = listen - receive - UNI_CARRIAGE_TIME * uni_carriages;

  present_cur = ((cpu-temp1*156/25)*CPU_CURRENT + lpm*LPM_CURRENT + 
                      temp1*LISTEN_CURRENT+(listen-receive)*209/20*CPU_SLEEP_CURRENT + 
                      transmit*TRANSMIT_CURRENT + receive*RECEIVE_CURRENT)/(cpu+lpm);
#if MLCDEBUG
  printf("mlc cur_cmp. cpu:%llu lpm:%llu transmit:%llu listen:%llu receive:%llu unicarriages:%llu temp1:%llu prev_current:%lu\n",cpu,lpm,transmit,listen,receive,uni_carriages,temp1,present_cur);
#endif
  return present_cur;
}
#endif

void setting_network_configuration1(uint8_t array[],int length,struct task_schedule *ts)
{
   if(length!=NETWORK_CONF_LENGTH1)
   {
      PRINTF("node_function.c array error conf1 \n");
      return;
   }
//zhangwei set changed for load balance
   #if MLCDEBUG
   printf("mlc set netDataPeriod\n");
#endif 
   netDataPeriod = array[0];

 task_schedule_set_period(ts,array[0]);

}

uint8_t getNetDataTaskPeriod(void){
  return netDataPeriod;
}

// setting  schedule;
void setting_network_configuration2(uint8_t array[],int length)
{ 
    if(length!=NETWORK_CONF_LENGTH2)
    {
       PRINTF("node_function.c array error conf2 \n");
       return ;
    }

    ctimer_set(&ct,CLOCK_SECOND*4,update_schedule,array);
   //update_schedule(array);
}

// /*Main Function instruction*/

// void heatMeterCommandEXE();
// void get_NodeID_MeterID();
int MeterCommandBurn()
{
   return 0;
}


/*reset or initialization instruction*/
void NodeReboot(void *p)
{
   watchdog_reboot();
}
void NodeReset(void *p)
{
  normalbyte_rfchannel_burn(ABNORMAL,0);  //   reset to channel 0
  NodeReboot(NULL);
}


