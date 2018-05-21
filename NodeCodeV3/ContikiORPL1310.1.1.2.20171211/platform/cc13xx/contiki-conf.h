 /*
 * Copyright (c) 2014, Texas Instruments Incorporated - http://www.ti.com/
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
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * \addtogroup cc26xx-srf-tag
 * @{
 *
 * \file
 *  Configuration for the srf06-cc26xx platform
 */
#ifndef CONTIKI_CONF_H
#define CONTIKI_CONF_H

#include <stdint.h>
/*---------------------------------------------------------------------------*/
/* Include Project Specific conf */
#ifdef PROJECT_CONF_H
#include PROJECT_CONF_H
#endif /* PROJECT_CONF_H */
/*---------------------------------------------------------------------------*/
/**
 * \name Network Stack Configuration
 *
 * @{
 */
#ifndef NETSTACK_CONF_NETWORK
#if NETSTACK_CONF_WITH_IPV6
#define NETSTACK_CONF_NETWORK sicslowpan_driver
#else
#define NETSTACK_CONF_NETWORK rime_driver
#endif /* NETSTACK_CONF_WITH_IPV6 */
#endif /* NETSTACK_CONF_NETWORK */

#ifndef NETSTACK_CONF_MAC
#define NETSTACK_CONF_MAC     csma_driver
// #define NETSTACK_CONF_MAC     nullmac_driver
#endif

#ifndef NETSTACK_CONF_RDC
#define NETSTACK_CONF_RDC     contikimac_driver
// #define NETSTACK_CONF_RDC     nullrdc_driver
#endif
#define CLIENT_TAG 0
/*
 * If set, the systems keeps the HF crystal oscillator on even when the radio is off.
 * You need to set this to 1 to use TSCH with its default 2.2ms or larger guard time.
 */

#define CC2650_FAST_RADIO_STARTUP               0

/* Configure NullRDC for when it's selected */
#define NULLRDC_CONF_802154_AUTOACK             1

/* Configure ContikiMAC for when it's selected */
#define CONTIKIMAC_CONF_WITH_CONTIKIMAC_HEADER  0
#define CONTIKIMAC_CONF_WITH_PHASE_OPTIMIZATION 0
#define WITH_FAST_SLEEP                         1



#ifndef NETSTACK_CONF_FRAMER
#define NETSTACK_CONF_FRAMER  framer_802154
#endif

#ifdef RF_CHANNEL
#define RF_CORE_CONF_CHANNEL             RF_CHANNEL
#endif

/* Number of Prop Mode RX buffers */
#ifndef PROP_MODE_CONF_RX_BUF_CNT
#define PROP_MODE_CONF_RX_BUF_CNT        4
#endif

/*
 * Auto-configure Prop-mode radio if we are running on CC13xx, unless the
 * project has specified otherwise. Depending on the final mode, determine a
 * default channel (again, if unspecified) and configure RDC params
 */
#if CPU_FAMILY_CC13XX
#ifndef CC13XX_CONF_PROP_MODE
#define CC13XX_CONF_PROP_MODE 1
#endif /* CC13XX_CONF_PROP_MODE */
#endif /* CPU_FAMILY_CC13XX */

#if CC13XX_CONF_PROP_MODE
#define NETSTACK_CONF_RADIO        prop_mode_driver

#ifndef RF_CORE_CONF_CHANNEL
#define RF_CORE_CONF_CHANNEL                      2
#endif

#define NULLRDC_CONF_ACK_WAIT_TIME                (RTIMER_SECOND / 400)
#define NULLRDC_CONF_AFTER_ACK_DETECTED_WAIT_TIME (RTIMER_SECOND / 1000)
#define NULLRDC_CONF_802154_AUTOACK_HW            0
#define NULLRDC_CONF_SEND_802154_ACK              1

#define CONTIKIMAC_CONF_CCA_CHECK_TIME            (RTIMER_ARCH_SECOND / 1600)
#define CONTIKIMAC_CONF_CCA_SLEEP_TIME            (RTIMER_ARCH_SECOND / 192) //238:4.20
#define CONTIKIMAC_CONF_LISTEN_TIME_AFTER_PACKET_DETECTED  (RTIMER_ARCH_SECOND / 20)
#define CONTIKIMAC_CONF_SEND_SW_ACK               1
#define CONTIKIMAC_CONF_AFTER_ACK_DETECTECT_WAIT_TIME (RTIMER_SECOND / 920) //920
#define CONTIKIMAC_CONF_INTER_PACKET_INTERVAL     (RTIMER_SECOND / 200)//200<=>5.0ms
#else
#define NETSTACK_CONF_RADIO        ieee_mode_driver

#ifndef RF_CORE_CONF_CHANNEL
#define RF_CORE_CONF_CHANNEL                     25
#endif

#define NULLRDC_CONF_802154_AUTOACK_HW            1
#define NULLRDC_CONF_SEND_802154_ACK              0
#endif

#define NETSTACK_RADIO_MAX_PAYLOAD_LEN        125

/* 6LoWPAN */
#define SICSLOWPAN_CONF_COMPRESSION          SICSLOWPAN_COMPRESSION_HC06
#define SICSLOWPAN_CONF_COMPRESSION_THRESHOLD  63
#define SICSLOWPAN_CONF_FRAG                    1
#define SICSLOWPAN_CONF_MAXAGE                  8
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name IEEE address configuration
 *
 * Used to generate our RIME & IPv6 address
 * @{
 */
/**
 * \brief Location of the IEEE address
 * 0 => Read from InfoPage,
 * 1 => Use a hardcoded address, configured by IEEE_ADDR_CONF_ADDRESS
 */
#ifndef IEEE_ADDR_CONF_HARDCODED
#define IEEE_ADDR_CONF_HARDCODED             0
#endif

/**
 * \brief The hardcoded IEEE address to be used when IEEE_ADDR_CONF_HARDCODED
 * is defined as 1
 */
#ifndef IEEE_ADDR_CONF_ADDRESS
#define IEEE_ADDR_CONF_ADDRESS { 0x00, 0x12, 0x4B, 0x00, 0x89, 0xAB, 0xCD, 0xEF }
#endif
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name RF configuration
 *
 * @{
 */
/* RF Config */
#ifndef IEEE802154_CONF_PANID
#define IEEE802154_CONF_PANID           0xABCD /**< Default PAN ID */
#endif

#ifndef IEEE_MODE_CONF_AUTOACK
#define IEEE_MODE_CONF_AUTOACK               1 /**< RF H/W generates ACKs */
#endif

#ifndef IEEE_MODE_CONF_PROMISCOUS
#define IEEE_MODE_CONF_PROMISCOUS            0 /**< 1 to enable promiscous mode */
#endif

#ifndef RF_BLE_CONF_ENABLED
#define RF_BLE_CONF_ENABLED                  0 /**< 0 to disable BLE support */
#endif
/** @} */
/*---------------------------------------------------------------------------*/
/** @} */
/**
 * \name IPv6, RIME and network buffer configuration
 *
 * @{
 */
/* Don't let contiki-default-conf.h decide if we are an IPv6 build */
#ifndef NETSTACK_CONF_WITH_IPV6
#define NETSTACK_CONF_WITH_IPV6              1
#endif

#if NETSTACK_CONF_WITH_IPV6
/*---------------------------------------------------------------------------*/
/* Addresses, Sizes and Interfaces */
#define LINKADDR_CONF_SIZE                   8
#define UIP_CONF_LL_802154                   1
#define UIP_CONF_LLH_LEN                     0

/* The size of the uIP main buffer */
//#ifndef UIP_CONF_BUFFER_SIZE
//#define UIP_CONF_BUFFER_SIZE              1000
//#endif

/* ND and Routing */
#ifndef UIP_CONF_ROUTER
#define UIP_CONF_ROUTER                      1
#endif

#define UIP_CONF_ND6_SEND_RA                 0
#define UIP_CONF_IP_FORWARD                  0
#define RPL_CONF_STATS                       0

#define UIP_CONF_ND6_REACHABLE_TIME     6000000//600000
#define UIP_CONF_ND6_RETRANS_TIMER       10000

#ifndef NBR_TABLE_CONF_MAX_NEIGHBORS
#define NBR_TABLE_CONF_MAX_NEIGHBORS        60
#endif
#ifndef UIP_CONF_MAX_ROUTES
#define UIP_CONF_MAX_ROUTES                 20
#endif

#ifndef UIP_CONF_TCP
#define UIP_CONF_TCP                         1
#endif
#ifndef UIP_CONF_TCP_MSS
#define UIP_CONF_TCP_MSS                    64
#endif

#define UIP_CONF_UDP                         1
#define UIP_CONF_UDP_CHECKSUMS               1
#define UIP_CONF_ICMP6                       1
/*---------------------------------------------------------------------------*/
#else /* NETSTACK_CONF_WITH_IPV6 */
/* Network setup for non-IPv6 (rime). */
#define UIP_CONF_IP_FORWARD                  1

#define RIME_CONF_NO_POLITE_ANNOUCEMENTS     0

#endif /* NETSTACK_CONF_WITH_IPV6 */
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name Generic Configuration directives
 *
 * @{
 */
#ifndef ENERGEST_CONF_ON
#define ENERGEST_CONF_ON            1 /**< Energest Module */
#endif
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name Character I/O Configuration
 *
 * @{
 */
#ifndef CC26XX_UART_CONF_ENABLE
#define CC26XX_UART_CONF_ENABLE            1 /**< Enable/Disable UART I/O */
#endif

#ifndef CC26XX_UART_CONF_BAUD_RATE
//#define CC26XX_UART_CONF_BAUD_RATE    115200 /**< Default UART0 baud rate */
#endif

/* Enable I/O over the Debugger Devpack - Only relevant for the SensorTag */
#ifndef BOARD_CONF_DEBUGGER_DEVPACK
#define BOARD_CONF_DEBUGGER_DEVPACK        1
#endif

/* Turn off example-provided putchars */
#define SLIP_BRIDGE_CONF_NO_PUTCHAR        1
#define SLIP_RADIO_CONF_NO_PUTCHAR         1

#ifndef SLIP_ARCH_CONF_ENABLED
/*
 * Determine whether we need SLIP
 * This will keep working while UIP_FALLBACK_INTERFACE and CMD_CONF_OUTPUT
 * keep using SLIP
 */
#if defined(UIP_FALLBACK_INTERFACE) || defined(CMD_CONF_OUTPUT)
#define SLIP_ARCH_CONF_ENABLED             1
#endif
#endif
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name ROM Bootloader configuration
 *
 * Enable/Disable the ROM bootloader in your image, if the board supports it.
 * Look in board.h to choose the DIO and corresponding level that will cause
 * the chip to enter bootloader mode.
 * @{
 */
#ifndef ROM_BOOTLOADER_ENABLE
#define ROM_BOOTLOADER_ENABLE              0
#endif
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name Button configurations
 *
 * Configure a button as power on/off: We use the right button for both boards.
 * @{
 */
#ifndef BUTTON_SENSOR_CONF_ENABLE_SHUTDOWN
#define BUTTON_SENSOR_CONF_ENABLE_SHUTDOWN 1
#endif

/* Notify various examples that we have Buttons */
#define PLATFORM_HAS_BUTTON      1

/*
 * Override button symbols from dev/button-sensor.h, for the examples that
 * include it
 */
#define button_sensor button_left_sensor
//#define button_sensor2 button_right_sensor
/** @} */
/*---------------------------------------------------------------------------*/
/* Platform-specific define to signify sensor reading failure */
#define CC26XX_SENSOR_READING_ERROR        0x80000000
/*---------------------------------------------------------------------------*/
/**
 * \name Compiler configuration and platform-specific type definitions
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define CLOCK_CONF_SECOND 128

/* Compiler configurations */
#define CCIF
#define CLIF

/* Platform typedefs */
typedef uint32_t clock_time_t;
typedef uint32_t uip_stats_t;

/* Clock (time) comparison macro */
#define CLOCK_LT(a, b)  ((signed long)((a) - (b)) < 0)

/*
 * rtimer.h typedefs rtimer_clock_t as unsigned short. We need to define
 * RTIMER_CLOCK_DIFF to override this
 */
typedef uint32_t rtimer_clock_t;
#define RTIMER_CLOCK_LT(a, b)     ((int32_t)((a) - (b)) < 0)
//#define RTIMER_CLOCK_DIFF(a, b)     ((int32_t)((a) - (b)))

/* --------------------------------------------------------------------- */
/* TSCH related defines */

/* Delay between GO signal and SFD */
#define RADIO_DELAY_BEFORE_TX ((unsigned)US_TO_RTIMERTICKS(81))
/* Delay between GO signal and start listening.
 * This value is so small because the radio is constantly on within each timeslot. */
#define RADIO_DELAY_BEFORE_RX ((unsigned)US_TO_RTIMERTICKS(15))
/* Delay between the SFD finishes arriving and it is detected in software. */
#define RADIO_DELAY_BEFORE_DETECT ((unsigned)US_TO_RTIMERTICKS(352))

/* Timer conversion; radio is running at 4 MHz */
#define RADIO_TIMER_SECOND   4000000u
#if (RTIMER_SECOND % 256) || (RADIO_TIMER_SECOND % 256)
#error RADIO_TO_RTIMER macro must be fixed!
#endif
#define RADIO_TO_RTIMER(X)   ((uint32_t)(((uint64_t)(X) * (RTIMER_SECOND / 256)) / (RADIO_TIMER_SECOND / 256)))
#define USEC_TO_RADIO(X)     ((X) * 4)

/* The PHY header (preamble + SFD, 4+1 bytes) duration is equivalent to 10 symbols */
#define RADIO_IEEE_802154_PHY_HEADER_DURATION_USEC 160

/* Do not turn off TSCH within a timeslot: not enough time */
#define TSCH_CONF_RADIO_ON_DURING_TIMESLOT 1

/* Disable TSCH frame filtering */
#define TSCH_CONF_HW_FRAME_FILTERING	0

/* Use hardware timestamps */
#ifndef TSCH_CONF_RESYNC_WITH_SFD_TIMESTAMPS
#define TSCH_CONF_RESYNC_WITH_SFD_TIMESTAMPS 1
#define TSCH_CONF_TIMESYNC_REMOVE_JITTER 0
#endif

#ifndef TSCH_CONF_BASE_DRIFT_PPM
/* The drift compared to "true" 10ms slots.
 * Enable adaptive sync to enable compensation for this. */
#define TSCH_CONF_BASE_DRIFT_PPM -977
#endif

/* 10 times per second */
#ifndef TSCH_CONF_CHANNEL_SCAN_DURATION
#define TSCH_CONF_CHANNEL_SCAN_DURATION (CLOCK_SECOND / 10)
#endif

/* Slightly reduce the TSCH guard time (from 2200 usec to 1800 usec) to make sure
 * the CC26xx radio has sufficient time to start up. */
#ifndef TSCH_CONF_RX_WAIT
#define TSCH_CONF_RX_WAIT 1800
#endif

/** @} */
/*---------------------------------------------------------------------------*/
/**********************************orpl by yangqiangqiang***************************************/
/*-------------------------------------------------------*/
/* orpl bitmap config */

#ifndef UIP_CONF_IPV6_ORPL_BITMAP
#define UIP_CONF_IPV6_ORPL_BITMAP   1
#endif  /* UIP_CONF_IPV6_ORPL_BITMAP */

#if UIP_CONF_IPV6_ORPL_BITMAP	//mlc
#define orpl_bitmap_contains_ipv6(ipv6)  orpl_routing_set_contains_ipv6((ipv6))
#define orpl_bitmap_fwd(lladdr)   orpl_bitmap_satisfy_fwd(lladdr)
#define orpl_bitmap_trickle_max  4  //2

#undef  CONTIKIMAC_CONF_CCA_COUNT_MAX
#define CONTIKIMAC_CONF_CCA_COUNT_MAX    2

#undef CONTIKIMAC_CONF_CCA_COUNT_MAX_TX
#define CONTIKIMAC_CONF_CCA_COUNT_MAX_TX  CONTIKIMAC_CONF_CCA_COUNT_MAX

#if SUBNET_PANID_CONF_LIMIT //mlc
#define ORPL_BITMAP_PANID_LIMIT  0
#else
#define ORPL_BITMAP_PANID_LIMIT  1  // 0
#endif

#define ORPL_BITMAP_LIMIT_NETWORKING  0 //1
#else
#define orpl_bitmap_contains_ipv6(ipv6)  0
#define orpl_bitmap_fwd(lladdr)  0 
#endif 

#if ORPL_BITMAP_PANID_LIMIT  
#define orpl_panid_limit(dest_pid)  orpl_bitmap_panid_limit((dest_pid))
#else
#define orpl_panid_limit(dest_pid)  1
#endif

#if ORPL_BITMAP_LIMIT_NETWORKING
#define orpl_bitmap_limit(lladdr)  orpl_bitmap_limit_networking((lladdr))
#else
#define orpl_bitmap_limit(lladdr)  1
#endif
/*-------------------------------------------------------*/
/*-------------------------------------------------------*/
/* netstack encrypt config */

#define NETSTACK_CONF_WITH_ENCRYPT  0

#if NETSTACK_CONF_WITH_ENCRYPT
#define NETSTACK_CONF_LLSEC noncoresec_driver
#define NETSTACK_CONF_FRAMER  noncoresec_framer
#ifndef LLSEC802154_CONF_SECURITY_LEVEL
#define LLSEC802154_CONF_SECURITY_LEVEL   FRAME802154_SECURITY_LEVEL_MIC_32
#endif /* LLSEC802154_CONF_SECURITY_LEVEL */
#endif

#ifndef AES_128_CONF   
#define AES_128_CONF aes_128_driver
#endif   
/*-------------------------------------------------------*/
#define NETSTACK_CONF_RDC_CHANNEL_CHECK_INTERVAL  20

#ifdef NETSTACK_CONF_RDC_CHANNEL_CHECK_INTERVAL
#define NETSTACK_RDC_CHANNEL_CHECK_INTERVAL  NETSTACK_CONF_RDC_CHANNEL_CHECK_INTERVAL
#else
#define NETSTACK_RDC_CHANNEL_CHECK_INTERVAL   30
#endif

#define NETSTACK_RDC_CHANNEL_CHECK_RATE  4
#define NETSTACK_CONF_RDC_CHANNEL_CHECK_RATE      4// 8

/*---------------------------------------------------------------------------*/
/*------------time synch configure----------------*/

#define CONTIKI_CONF_NETSYNCH  1
#if CONTIKI_CONF_NETSYNCH
#define SCHEDULE_SIZE 18
#endif
/*------------------------------------------------*/
/**-------------low power configure ---------------*/
#define CC1310_CONF_LOWPOWER  1

#if CC1310_CONF_LOWPOWER
/*no need probe */ 
#undef RPL_CONF_WITH_PROBING
#define RPL_CONF_WITH_PROBING      0   

/* RPL does not use RPL's downwards routing */
#undef RPL_CONF_MOP
#define RPL_CONF_MOP RPL_MOP_NO_DOWNWARD_ROUTES

/* RPL does not use routing entries */
#undef UIP_CONF_MAX_ROUTES
#define UIP_CONF_MAX_ROUTES  0


// conf if support change dio interval
#ifndef RPL_CONF_CHANGE_DIO_INTERVAL
#define RPL_CONF_CHANGE_DIO_INTERVAL  0
#endif

#undef  RPL_CONF_DIS_INTERVAL
#define RPL_CONF_DIS_INTERVAL   (1*60)    // 300s

#undef  RPL_CONF_MAX_DAG_PER_INSTANCE
#define RPL_CONF_MAX_DAG_PER_INSTANCE   1

#undef  UIP_CONF_SUPPORT_ECHO
#define UIP_CONF_SUPPORT_ECHO  0

#undef  UIP_CONF_SUPPORT_ICMP6_ERROR_OUTPUT
#define UIP_CONF_SUPPORT_ICMP6_ERROR_OUTPUT	  0

/** Period for uip-ds6 periodic task*/
#ifndef UIP_DS6_CONF_PERIOD
#define UIP_DS6_CONF_PERIOD   (CLOCK_SECOND)
#endif


#endif /* CC1310_CONF_LOWPOWER */

/*------------------------adaptive CCA_THR------------------------*/
#define ADAPTIVE_CCA_THR 0	//by mlc
#define ADAPTIVE_MAX_CONTINUOUS_ACTIVE_NUMBER	3  //by mlc ADAPTIVE_MAX_CONTINUOUS_ACTIVE_NUMBER>=1
/*-----------------external watchdog driver by xiaobing-------------------------------*/
#define EXTER_WATCHDOG 1
/*-----------------MAC layer synchronization-------------------------------*/
#define MAC_TIMESYN 0
/*-------------uart  configure -------------------*/
#if UIP_MCAST6_CONF_ENGINE == UIP_MCAST6_ENGINE_FCF
#undef LOW_LATENCY
#define LOW_LATENCY  1     
#endif
#undef  ROOTNODE
#define ROOTNODE  0

#if ROOTNODE
	#define MLCDEBUG	1
#else
	#define MLCDEBUG	0
#endif

#define MEDIANODE 1
#if ROOTNODE || MEDIANODE
	#define PROP_MODE_CONF_RSSI_THRESHOLD 0xA2  //--94
#else
	#define PROP_MODE_CONF_RSSI_THRESHOLD 0xAB  //--85
#endif 
// #define PROP_MODE_CONF_RSSI_THRESHOLD 0xAB 
#define AUTOFEEDBACKDELAY 3 //sec
#ifndef ROOTNODE_CORRECTTIME_TIMEOUT_REBOOT
#define ROOTNODE_CORRECTTIME_TIMEOUT_REBOOT  0
#endif

#if 1
#define WAKEUP_NODE    1
#define WAKEUP_NODE_RFCHANNEL  2    //4  mlc
#endif

#ifndef HW_NEW_BIG
#define HW_NEW_BIG 0
#endif 

// #define HEAT_METER 1 // baud rate 2400

// default baud rate 9600
#if ROOTNODE
	#define ROOT_WITH_ENERGY_EFFICIENCY 0  //mlc
#else
#define WITH_ENERGY_EFFICIENCY 0
#endif
/*-------------------------------------------------------*/
/* root node fail detect config (RNFD) */

#ifndef UIP_CONF_IPV6_RNFD
#define UIP_CONF_IPV6_RNFD    1  //mlc
#endif  /* UIP_CONF_IPV6_RNFD */

/* RNFD root offline state, node auto enter sleep config */
#if UIP_CONF_IPV6_RNFD
#ifndef UIP_CONF_AUTO_SLEEP
#define UIP_CONF_AUTO_SLEEP   1
#endif  /* UIP_CONF_AUTO_SLEEP */
#endif

#ifndef RNFD_TEST
#define RNFD_TEST    0
#endif  /* RNFD_TEST */

#if RNFD_TEST
#define UIP_CONF_STATISTICS   1
#define RPL_CONF_STATS        1
#endif
/*-------------------------------------------------------*/
/* one large network to multi subnet. */
#ifndef UIP_CONF_MULTI_SUBNET
#define UIP_CONF_MULTI_SUBNET  0  //mlc
#endif  /* UIP_CONF_MULTI_SUBNET */

#if UIP_CONF_MULTI_SUBNET
#include "multi-subnet-conf.h"
#endif
/*-----------rtimer change frequence and etimer shutdown inactive------------*/
#if!ROOTNODE
#define MYSERVER  0  //for debug to set root's active flag always 1
#define CHANGERREU 1 //change contikimac frequence
#define CHANGEETIMER 1 //inactive shutdown etimer,active open etimer
#define HEAT_METER 0 //client's baud rate set as 2400
#endif
/*------------------------------------------------*/

#if HEAT_METER
#define CC26XX_UART_CONF_BAUD_RATE    2400 /**< Default UART0 baud rate */
#else
#define CC26XX_UART_CONF_BAUD_RATE    115200 /**< Default UART0 baud rate */
#endif
/*---------------------------------------------------------------------------*/

/* board.h assumes that basic configuration is done */
#include "board.h"
#include "hitlib.h"
/*---------------------------------------------------------------------------*/
#endif /* CONTIKI_CONF_H */
