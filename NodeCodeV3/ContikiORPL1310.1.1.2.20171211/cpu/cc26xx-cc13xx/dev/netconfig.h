#ifndef NETCONFIG_H_
#define NETCONFIG_H_

#include "contiki-conf.h"
#include "clock.h"
/************************define hit softtime*************************************/

#if CC1310_CONF_LOWPOWER

void update_soft_time();

void set_active_flag();//get active flag from 
// void set_active_flag(int hour,int minute,int second_s);//get active flag from 

void syn_update_timenow(soft_time syntime);

void get_timenow(soft_time *times);

uint8_t get_active_flag(void);

clock_time_t get_idle_time(void);
uint8_t get_active_bit_length(void);
void set_init_flag(uint16_t  flag);
     
uint16_t get_init_flag();    

void set_autocal_info(int autocal_offest,uint32_t autocal_interval);
uint16_t get_nowdays();

void set_ledon_flag(uint8_t flag);

uint32_t get_cal_count();
void set_cal_countaddone();

int get_cal_offest();

void set_cal_countzero();

int get_cal_interval();

#endif

#if MAC_TIMESYN
// void DBScan(char* cot, int lastIndex);
static void buildHeap(char* cot, int lastIndex);
static void swapIndex(char* cot, int Index1, int index2);
static void adjustHeap(char* cot, int index,int lastIndex);
static void heapSort(char* cot, int lastIndex);
static int  DBScan(char* cot, char* div,int lastIndex);
static char DBScanFindEnd(int* cot,int index, int lastIndex,int corePointNum, int neighborArea);
static char DBScanFindStart(int* cot, int index, int lastIndex,int corePointNum, int neighborArea);

static void gradientDecent(void);
static double batchOneGradient(void);
static int LinearRegression(void);

void add_mac_stamp(soft_time* mac_time_stamp);
int16_t differ_from_now(soft_time* mac_time_stamp);
void mac_syn_init(void);

#endif 

#endif/*NETCONFIG*/