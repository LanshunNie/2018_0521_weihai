#include "contiki.h"
#include "contiki-conf.h"
#include "dev/netconfig.h"
#include "stdio.h"
#include "node-id.h"
#include "ti-lib.h"
#include "auto-sleep.h"
#if CONTIKI_CONF_NETSYNCH
#include "netsynch.h"
#include "task-schedule.h"
#endif

#define DEBUG 0
#if DEBUG 
void logic_test(uint32_t i);
static uint32_t logic=0;
 uint32_t timecount =0;
#endif

#if CC1310_CONF_LOWPOWER

/***********/
#if EXTER_WATCHDOG
static uint32_t add_time = 0;//by xiaobing,external watch dog DIO14
#endif 
/***********/
uint8_t test_use_flag =1;
static volatile uint8_t  active_flag_one_second_before = 0;
static volatile uint16_t  init_net_flag = 1;

#if WAKEUP_NODE_DEV
static volatile uint8_t  active_flag = 1;
#else
static volatile uint8_t  active_flag = 1; //by mlc
#endif

static uint8_t schedule_bit[18]; 

static soft_time  timenow;
static int cal_offest = 0;            
static uint32_t cal_interval = 0;
static uint32_t cal_count = 0;

static uint16_t days=0;
static uint8_t  ledon_flag =0;

#if !WAKEUP_NODE_DEV
//static void update_soft_time();
#endif

#if MAC_TIMESYN
#define TIMESYNCOTSIZE 100
#define TIMESHIFTCOTSIZE   10
#define LEARNINGRATE  (1/10000) //LR parameters
#define MAXITERATOR   1000      //LR parameters
#define COREPOINTNUM 5          //DBSCAN parameters
#define NEIGHBORAREA 2          //DBSCAN parameters
static int16_t cot[TIMESYNCOTSIZE]; //circle buffer
static int16_t shift_cot[TIMESHIFTCOTSIZE]; //circle buffer
static int32_t timeInterval[TIMESHIFTCOTSIZE];
static int16_t syn_cot_pointer = 0;
static int16_t syn_cot_size = 0;
static int16_t shift_cot_pointer = 0;
static int16_t shift_cot_size = 0;
static double theta = 0;        //`
static lr_length = TIMESHIFTCOTSIZE;
#endif

void external_watch_dog(int tick){//by xiaobing,external watch dog DIO14

  ti_lib_gpio_write_dio(BOARD_IOID_DIO14,1);
   //delay 10us
    for(int i =0; i< tick; i++){
      for(int j = 0; j < i; ++j) {
        __asm("nop");
      }
    }
    
  ti_lib_gpio_write_dio(BOARD_IOID_DIO14,0);
}

void update_soft_time()

{
  
  timenow.sec+=1;
  if( timenow.sec/60){ 
      ++timenow.minute;
      timenow.sec=0;
  }

  if( timenow.minute/60 ){
      ++timenow.hour; 
      timenow.minute=0;
  }
  if( timenow.hour/24 ){
      timenow.hour=0;
      ++days;
  }

  #if EXTER_WATCHDOG
   if(add_time >=300){ //by xiaobing,external watch dog DIO14
      external_watch_dog(15);
      add_time = 0;
   }
   add_time++;
 
  #endif 

}

void set_init_flag(uint16_t flag){
    init_net_flag =flag;
}

void set_ledon_flag(uint8_t flag){
  ledon_flag = flag;
}

uint16_t get_init_flag(){
  return init_net_flag ;
}

#if !WAKEUP_NODE_DEV
// void set_active_flag(int hour,int minute,int second_s)
void set_active_flag()
{
//   calendar_time  cal_time_now;
  int index=0;
  // read_calendar(&cal_time_now);
  // int hour   = BCD_to_dec(cal_time_now.hour);
  // int minute = BCD_to_dec(cal_time_now.min);
  // index=hour*6+minute/10;         //6 ,10
  index=timenow.hour*6+timenow.minute/10;         //6   ,10
  schedule_bitmap_get(schedule_bit);

#if 0  
  int index2=0;
  for(;index2<18;index2++){
    printf("schedule_bit [%d] = %d\n",index2,schedule_bit[index2] );
  }
#endif
#if UIP_CONF_AUTO_SLEEP
  if(get_round_auto_sleep() != index){
    clear_force_auto_sleep();
    set_round_auto_sleep(index);
  }
#endif

#if UIP_CONF_AUTO_SLEEP
  if(get_force_auto_sleep()){
     active_flag = 0;
  }else{
#endif  
    active_flag= init_net_flag&((schedule_bit[index/8]) >> (7-(index%8)));

#if UIP_CONF_AUTO_SLEEP 
  }
#endif
  #if !ROOTNODE
     // printf("active flag:%u\n",active_flag);
  #endif



  // static int tempcount = 0;
  // if(timenow.sec%10 == 0){
  //   tempcount++;    
  //   // logic =logic^1;
  //   // logic_test(logic);
  //   if(tempcount >=60){
  //     active_flag =active_flag^1;
  //     tempcount = 0;
  //   }

  // }


     // active_flag = test_use_flag;
     // active_flag = 0;
  // active_flag = 1; //for test

  // #if MYSERVER
    // active_flag = 1;
  // #endif
  
}
#endif


void syn_update_timenow(soft_time syntime)
{
   timenow.minute=syntime.minute;
   timenow.hour  =syntime.hour;
   timenow.sec   =syntime.sec;
}

void get_timenow(soft_time *times)
{
   times->hour   =timenow.hour;
   times->minute =timenow.minute;
   times->sec    =timenow.sec;
}

clock_time_t get_idle_time(void)  //return 0 means not in active mode .
{
  uint8_t counter=0;
  int index=0;
  int i=0;

  // get_timenow(&timenow);
  if(active_flag){
    index=timenow.hour*6 +timenow.minute/10 +1;
      // printf("index :%d\n",index);
    for(;i<144;i++){
      if((((schedule_bit[index/8]) >> (7-(index%8)) ) & 0x01)){
             index=(index+1)%144;
             ++counter;
    }else 
      break;
    }
    // printf("%d %d\n",counter,index);
    return ((10*counter+9-timenow.minute%10)*60+(60-timenow.sec)); 
  }
  return 0x0;
}
uint8_t get_active_bit_length(void)  //return 0 means not in active mode .
{
  uint8_t counter=0;
  int index=0;
  int i=0;

  // get_timenow(&timenow);
  if(active_flag){
    index=timenow.hour*6 +timenow.minute/10 +1;
      // printf("index :%d\n",index);
    for(;i<144;i++){
      if((((schedule_bit[index/8]) >> (7-(index%8)) ) & 0x01)){
             index=(index+1)%144;
             ++counter;
    }else 
      break;
    }
    // printf("%d %d\n",counter,index);
    #if MLCDEBUG
      printf("mlc get_active_bit_length:%d\n",counter+1);
    #endif
      if(counter+1 > ADAPTIVE_MAX_CONTINUOUS_ACTIVE_NUMBER){
        counter = ADAPTIVE_MAX_CONTINUOUS_ACTIVE_NUMBER -1;
      }
    return counter+1; 
  }
  return 0;       //equals 1 even in inactive 
}


uint8_t get_active_flag(void)
{
  return active_flag;
}

uint16_t 
get_nowdays()
{
  return days;
}
/*----------------------------------------------------------------*/

void 
set_autocal_info(int autocal_offest,uint32_t autocal_interval){
  cal_count = 0;
  cal_offest = autocal_offest;
  cal_interval = autocal_interval;
}


/********************************************************/
uint32_t get_cal_count(){
  return cal_count;
}

void set_cal_countaddone(){
  cal_count++;
  // printf
}

int get_cal_offest(){
  // cal_offest=0;
  return cal_offest;
}
int get_cal_interval(){

  return cal_interval;
}
void set_cal_countzero(){
  cal_count=0;
}

#endif

#if MAC_TIMESYN
/*--------------------DBSCAN PART---------------------------------*/
// void randomValueProducer(char* cot, char center,char centerWidth, int length){  // 0.8 for center
//   srand((unsigned)time( NULL )); 
//   int inLength = centerWidth * 2 +1;
//   int outLength = 256 - inLength;
//   float inpos;
//   int i = 0;
//   for(i = 0; i<length;i++){
//     inpos = rand()*1.0/32767;
//     if(inpos <0.8){
//       cot[i] = (char)(rand()%inLength)+center-centerWidth; 
//     }else{
//       cot[i] = (char)(rand()&outLength)+center+centerWidth+1;
//     }
//   }
//   return ;
// }

static void buildHeap(char* cot, int lastIndex){
  int i;
  for(i= (lastIndex-1)/2 ; i>=0;i--){
    adjustHeap(cot, i,lastIndex);
  }
  return;
}

static void swapIndex(char* cot, int index1, int index2){
  cot[index1] = cot[index1]^cot[index2];
  cot[index2] = cot[index1]^cot[index2];
  cot[index1] = cot[index1]^cot[index2];
  return;
}

static void adjustHeap(char* cot, int index,int lastIndex){  //Max Heap
//  int left = 2*index+1 <= lastIndex ? 2*index+1 : -1;
//  int right = 2*index +2 <= lastIndex ? 2*index+2 : -1;
//  if(left != -1){
//    if(right != -1){
//      if(cot[left] <= cot[right]){
//        if(cot[index] <= cot[left]){
//          return;
//        }else{
//          swap(cot, index, left);
//          adjustHeap(cot,left,lastIndex);
//          return;
//        }
//      }
//    }
//  } 
  int child = index*2+1;
  while(child <= lastIndex){
    if(child+1<=lastIndex && cot[child+1] > cot[child]){
      child = child +1;
    }
    if(cot[child] > cot[index]){
      swapIndex(cot, child, index);
      index = child ;
      child = index*2 + 1;
    }else{
      break;
    }
  }
  return;
   
}

static void heapSort(char* cot, int lastIndex){
  buildHeap(cot,lastIndex);
  int i;
  for(i = lastIndex; i>0;i--){
    swapIndex(cot,0,i);
    adjustHeap(cot,0,i-1);
  }
  return;
}

static int DBScan(char* cot, char* div,int lastIndex){ //div should be 255 Bytes, lastIndex = length(cot) - 1;
  int length = lastIndex+1;
  if(length <= 0){
    return 0;
  }
  int count[256];
  int corePointNum = COREPOINTNUM;
  int neighborArea = NEIGHBORAREA;
  memset(count,0,sizeof(count));
  memset(div,-1,256);
  int i = 0, j = 0;
  for(i=0;i<=lastIndex;i++){
    count[128+cot[i]]+=1;
  } 
  for(i=0;i<256;i++){
    if(count[i] >0)
    printf("A %d:%d\n",i-128,count[i]);
  }
  char startFoundTag = 0;
  int divIndex = 0;
  for(i=0;i<256;i++){
    if(startFoundTag==0){
      if(DBScanFindStart(count,i,255,corePointNum,neighborArea)){
        div[divIndex] = (char)i;
        div[divIndex++] += 128;
        startFoundTag = 1;
        i--;  //judge if its also the End
      }
    }else{
      if(DBScanFindEnd(count,i,255,corePointNum,neighborArea)){
        div[divIndex] = (char)i;
        div[divIndex++] += 128;
        startFoundTag = 0;
      }
    }
  }
  if(startFoundTag == 1){
    div[divIndex++] = 127;
  }
  //we should also get_center_point of this DB_SCAN's answer
  int32_t max_zone_count = 0;
  int time_shift = 0;
  for(i = 0; i< divIndex; i++){
    zone_start = div[i++];
    zone_end = div[i++];
    for(j = zone_start; j <= zone_end;j++){
      int32_t zone_count += count[j];
    }
    if(zone_count > max_zone_count){
      max_zone_count = zone_count;
      int64_t temp3 = 0;
      for(j = zone_start; j <= zone_end;j++){
        temp3 += 
      }
    }
  }
  return divIndex;
}

static char DBScanFindStart(int* cot, int index, int lastIndex,int corePointNum, int neighborArea){
  if(cot[index] <=0){
    return 0;
  } 
  int tmp = index + neighborArea<= lastIndex? index + neighborArea : lastIndex;
  while(tmp>= index && cot[tmp] < corePointNum ){
    tmp--;
  }
  if(tmp >= index){
    return 1;
  }else{
    return 0;
  }
}

static char DBScanFindEnd(int* cot,int index, int lastIndex,int corePointNum, int neighborArea){
  if(cot[index]<= 0){
    return 0;
  }
  int neighborCore = index - neighborArea>= 0? index - neighborArea:0;
  int tmp = index;
  while(tmp>= neighborCore && cot[tmp] < corePointNum){
    tmp--;
  }
  if(tmp < neighborCore)
    return 0;
  int tmp2 = index +1;
  while((tmp2<= lastIndex) && (tmp2 - tmp <= neighborArea)){
    if(cot[tmp2] >0)
      return 0; 
    tmp2++; 
  }
  return 1;
}


/*----------------------MAC SYN PART----------------------*/
void add_mac_stamp(soft_time* mac_time_stamp){    //can't work when diff is more than 9 hours(int 16_t)
  int16_t diff = differ_from_now(mac_time_stamp);
  cot[syn_cot_pointer] = diff;
  syn_cot_pointer = syn_cot_pointer + 1 >= TIMESYNCOTSIZE ? 0 : syn_cot_pointer + 1;
  syn_cot_size + 1 > TIMESYNCOTSIZE ? TIMESYNCOTSIZE : syn_cot_size + 1;
  return;
}

int16_t differ_from_now(soft_time* mac_time_stamp){
  soft_time receive_time = mac_time_stamp*;
  soft_time now;
  get_timenow(&now);
  int16_t diff = 0;
  diff += (receive_time.hour - now.hour) * 3600;
  diff += (receive_time.minute - now.minute) * 60;
  diff += (receive_time.sec - now.sec);
  return diff;
}

void mac_syn_init(void){
  memset(&cot, 0 , 2* TIMESYNCOTSIZE);  // 2: 16_t
  memset(&shift_cot, 0, 2*TIMESHIFTCOTSIZE);
  memset(&timeInterval, 0, 4*TIMESHIFTCOTSIZE);
  syn_cot_pointer = 0 ;
  syn_cot_size = 0;
  shift_cot_pointer = 0;
  shift_cot_size = 0;
}

/*----------------------LR PART---------------------------*/
static int LinearRegression(void){
  int avgShift = 0;
  int avgInterval = 0;
//  double alpha = LEARNINGRATE;
  int tempShift = 0;
  int tempInterval = 0;
  int i = 0;
  int lxx = 0;
  int lxy = 0;
  for(i = 0; i< lr_length;i++){
    int index = (i+shift_cot_pointer)%TIMESHIFTCOTSIZE;
    tempShift += shift_cot[index];
    tempInterval += timeInterval[index];
    avgShift+= tempShift;
    avgInterval+= tempInterval;
  }
  avgShift = avgShift/lr_length;
  avgInterval = avgInterval/lr_length;
  for(i = 0,tempShift=0,tempInterval=0;i<lr_length;i++){
    int index = (i+shift_cot_pointer)%TIMESHIFTCOTSIZE;
    tempShift += shift_cot[index];
    tempInterval += timeInterval[index];
    lxx += (tempInterval - avgInterval) * (tempInterval - avgInterval);
    lxy += (tempInterval - avgInterval) * (tempShift - avgShift);
  } 
  printf("%d %d\n",avgShift,avgInterval);
  printf("%d %d\n",lxy,lxx);
  double b = 1.0*lxy/lxx;
  double a = avgInterval - b * avgShift;
  printf("Answer: %fX +%f \n", b, a); 
  return 1;
}

static void gradientDecent(void){
  theta = 0;
  int i = 0;
  double alpha = LEARNINGRATE;
  for(i = 0; i< MAXITERATOR;i++){
    double oneGradientDecent = batchOneGradient();
    printf("%d:%f\n",i,oneGradientDecent);
    if(oneGradientDecent < 0.001)
      continue;
    theta+= oneGradientDecent;
  }
  return ;
}

static double batchOneGradient(void){
  int i = 0;
  double temp = 0;
  int tempShift = 0;
  int tempInterval = 0;
  for(i = 0; i < TIMESHIFTCOTSIZE;i++){
    int temp2 = (i+shift_cot_pointer)%TIMESHIFTCOTSIZE;
    tempShift+=shift_cot[temp2];
    tempInterval += timeInterval[temp2];
    temp += (tempShift*1.0 - ((int)theta*1000)*tempInterval*1.0/1000) * tempInterval*0.00001;
  }
  return temp;
}
// void testSetProducer(int avgTimeShift, int avgTimeLength){
//   srand((unsigned)time( NULL )); 
//   int amplitude = ceil(avgTimeShift * 0.4);
//   int parts = amplitude * 2 + 1;
//   float inpos;
//   int i = 0;
//   int shift = 0;
//   for(i = 0; i<TIMESHIFTCOTSIZE;i++){
//     inpos = rand()*1.0/32768; //inpos [0,1) 
//     shift = floor(inpos * parts) - amplitude;
//     shift_cot[i] = avgTimeShift + shift; 
//   }
//   amplitude = ceil(avgTimeLength * 0.02);
//   parts = amplitude * 2 + 1;
//   inpos;
//   i = 0;
//   shift = 0;
//   for(i = 0; i<TIMESHIFTCOTSIZE;i++){
//     inpos = rand()*1.0/32768; //inpos [0,1) 
//     shift = floor(inpos * parts) - amplitude;
//     timeInterval[i] = avgTimeLength + shift; 
//   }
//   return ;
// }

// int main(void){
//   int i=0;
//   for(i=0;i<TIMESHIFTCOTSIZE;i++){
//     printf("%d: %d-%d\n",i,shift_cot[i],timeInterval[i]);
//   }
//  gradientDecent();
// LinearRegression();
//  printf("%f",theta);
// } 
/*------------------------------------------------------*/
#endif /*MAC_TIMESYN*/
