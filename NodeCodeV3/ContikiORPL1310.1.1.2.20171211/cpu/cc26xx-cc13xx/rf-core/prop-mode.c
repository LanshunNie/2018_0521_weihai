/*
 * Copyright (c) 2015, Texas Instruments Incorporated - http://www.ti.com/
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
/*---------------------------------------------------------------------------*/
/**
 * \addtogroup rf-core-prop
 * @{
 *
 * \file
 * Implementation of the CC13xx prop mode NETSTACK_RADIO driver
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "dev/radio.h"
#include "dev/cc26xx-uart.h"
#include "dev/oscillators.h"
#include "dev/watchdog.h"
#include "net/packetbuf.h"
#include "net/rime/rimestats.h"
#include "net/linkaddr.h"
#include "net/netstack.h"
#include "sys/energest.h"
#include "sys/clock.h"
#include "sys/rtimer.h"
#include "sys/cc.h"
#include "lpm.h"
#include "ti-lib.h"
#include "rf-core/rf-core.h"
#include "rf-core/rf-switch.h"
#include "rf-core/rf-ble.h"
#include "rf-core/prop-mode.h"
#include "rf-core/dot-15-4g.h"
/*---------------------------------------------------------------------------*/
/* RF core and RF HAL API */
#include "hw_rfc_dbell.h"
#include "hw_rfc_pwr.h"
/*---------------------------------------------------------------------------*/
/* RF Core Mailbox API */
#include "driverlib/rf_mailbox.h"
#include "driverlib/rf_common_cmd.h"
#include "driverlib/rf_data_entry.h"
#include "driverlib/rf_prop_mailbox.h"
#include "driverlib/rf_prop_cmd.h"
/*---------------------------------------------------------------------------*/
/* CC13xxware patches */
#include "rf_patches/rf_patch_cpe_genfsk.h"
#include "rf_patches/rf_patch_rfe_genfsk.h"
/*---------------------------------------------------------------------------*/
#include "rf-core/smartrf-settings.h"
/*---------------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
/*---------------------------------------------------------------------------*/
#define LOGIC_TEST 0
#if LOGIC_TEST 
void logic_test(uint32_t i);
#endif

#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
/*---------------------------------------------------------------------------*/
/* Data entry status field constants */
#define DATA_ENTRY_STATUS_PENDING    0x00 /* Not in use by the Radio CPU */
#define DATA_ENTRY_STATUS_ACTIVE     0x01 /* Open for r/w by the radio CPU */
#define DATA_ENTRY_STATUS_BUSY       0x02 /* Ongoing r/w */
#define DATA_ENTRY_STATUS_FINISHED   0x03 /* Free to use and to free */
#define DATA_ENTRY_STATUS_UNFINISHED 0x04 /* Partial RX entry */
/*---------------------------------------------------------------------------*/
/* Data whitener. 1: Whitener, 0: No whitener */
#ifdef PROP_MODE_CONF_DW
#define PROP_MODE_DW PROP_MODE_CONF_DW
#else
#define PROP_MODE_DW 0
#endif

#ifdef PROP_MODE_CONF_USE_CRC16
#define PROP_MODE_USE_CRC16 PROP_MODE_CONF_USE_CRC16
#else
#define PROP_MODE_USE_CRC16 0
#endif
/*---------------------------------------------------------------------------*/
/**
 * \brief Returns the current status of a running Radio Op command
 * \param a A pointer with the buffer used to initiate the command
 * \return The value of the Radio Op buffer's status field
 *
 * This macro can be used to e.g. return the status of a previously
 * initiated background operation, or of an immediate command
 */
#define RF_RADIO_OP_GET_STATUS(a) GET_FIELD_V(a, radioOp, status)
/*---------------------------------------------------------------------------*/
/* Special value returned by CMD_IEEE_CCA_REQ when an RSSI is not available */
#define RF_CMD_CCA_REQ_RSSI_UNKNOWN     -128  //Can't change to 127,otherwise lead to wrong receiving_packet
// #define RF_CMD_CCA_REQ_RSSI_UNKNOWN         127   
/* Used for the return value of channel_clear */
#define RF_CCA_CLEAR                       1
#define RF_CCA_BUSY                        0

/* Used as an error return value for get_cca_info */
#define RF_GET_CCA_INFO_ERROR           0xFF

/*
 * Values of the individual bits of the ccaInfo field in CMD_IEEE_CCA_REQ's
 * status struct
 */
#define RF_CMD_CCA_REQ_CCA_STATE_IDLE      0 /* 00 */
#define RF_CMD_CCA_REQ_CCA_STATE_BUSY      1 /* 01 */
#define RF_CMD_CCA_REQ_CCA_STATE_INVALID   2 /* 10 */

#ifdef PROP_MODE_CONF_RSSI_THRESHOLD
#define PROP_MODE_RSSI_THRESHOLD PROP_MODE_CONF_RSSI_THRESHOLD
#else
#define PROP_MODE_RSSI_THRESHOLD 0xB0//0xBA<==>-70  0xB0<==>-80// 0xA6<==>-90
#endif

static int8_t rssi_threshold = PROP_MODE_RSSI_THRESHOLD;
/*---------------------------------------------------------------------------*/
static int on(void);
static int off(void);

static rfc_propRxOutput_t rx_stats;
/*---------------------------------------------------------------------------*/
/* Defines and variables related to the .15.4g PHY HDR */
#define DOT_4G_MAX_FRAME_LEN    2047
#define DOT_4G_PHR_LEN             2

/* PHY HDR bits */
#define DOT_4G_PHR_CRC16  0x10
#define DOT_4G_PHR_DW     0x08

#if PROP_MODE_USE_CRC16
/* CRC16 */
#define DOT_4G_PHR_CRC_BIT DOT_4G_PHR_CRC16
#define CRC_LEN            2
#else
/* CRC32 */
#define DOT_4G_PHR_CRC_BIT 0
#define CRC_LEN            4
#endif

#if PROP_MODE_DW
#define DOT_4G_PHR_DW_BIT DOT_4G_PHR_DW
#else
#define DOT_4G_PHR_DW_BIT 0
#endif
/*---------------------------------------------------------------------------*/
/* How long to wait for an ongoing ACK TX to finish before starting frame TX */
#define TX_WAIT_TIMEOUT       (RTIMER_SECOND >> 11)

/* How long to wait for the RF to enter RX in rf_cmd_ieee_rx */
#define ENTER_RX_WAIT_TIMEOUT (RTIMER_SECOND >> 10)
/*---------------------------------------------------------------------------*/
/* TX power table for the 431-527MHz band */
#ifdef PROP_MODE_CONF_TX_POWER_431_527
#define PROP_MODE_TX_POWER_431_527 PROP_MODE_CONF_TX_POWER_431_527
#else
#define PROP_MODE_TX_POWER_431_527 prop_mode_tx_power_431_527
#endif
/*---------------------------------------------------------------------------*/
/* TX power table for the 779-930MHz band */
#ifdef PROP_MODE_CONF_TX_POWER_779_930
#define PROP_MODE_TX_POWER_779_930 PROP_MODE_CONF_TX_POWER_779_930
#else
#define PROP_MODE_TX_POWER_779_930 prop_mode_tx_power_779_930
#endif
/*---------------------------------------------------------------------------*/
/* Select power table based on the frequency band */
#if DOT_15_4G_FREQUENCY_BAND_ID==DOT_15_4G_FREQUENCY_BAND_433
#define TX_POWER_DRIVER PROP_MODE_TX_POWER_431_527
#else
#define TX_POWER_DRIVER PROP_MODE_TX_POWER_779_930
#endif
/*---------------------------------------------------------------------------*/
extern const prop_mode_tx_power_config_t TX_POWER_DRIVER[];

/* Max and Min Output Power in dBm */
#define OUTPUT_POWER_MAX     (TX_POWER_DRIVER[0].dbm)
#define OUTPUT_POWER_UNKNOWN 0xFFFF

/* Default TX Power - position in output_power[] */
const prop_mode_tx_power_config_t *tx_power_current = &TX_POWER_DRIVER[0];
/*---------------------------------------------------------------------------*/
#ifdef PROP_MODE_CONF_LO_DIVIDER
#define PROP_MODE_LO_DIVIDER   PROP_MODE_CONF_LO_DIVIDER
#else
#define PROP_MODE_LO_DIVIDER   0x05
#endif
/*---------------------------------------------------------------------------*/
#ifdef PROP_MODE_CONF_RX_BUF_CNT
#define PROP_MODE_RX_BUF_CNT PROP_MODE_CONF_RX_BUF_CNT
#else
#define PROP_MODE_RX_BUF_CNT 4
#endif
/*---------------------------------------------------------------------------*/
#define DATA_ENTRY_LENSZ_NONE 0
#define DATA_ENTRY_LENSZ_BYTE 1
#define DATA_ENTRY_LENSZ_WORD 2 /* 2 bytes */

/*
 * RX buffers.
 * PROP_MODE_RX_BUF_CNT buffers of RX_BUF_SIZE bytes each. The start of each
 * buffer must be 4-byte aligned, therefore RX_BUF_SIZE must divide by 4
 */
#define RX_BUF_SIZE 140
static uint8_t rx_buf[PROP_MODE_RX_BUF_CNT][RX_BUF_SIZE] CC_ALIGN(4);

/* The RX Data Queue */
static dataQueue_t rx_data_queue = { 0 };

/* Receive entry pointer to keep track of read items */
volatile static uint8_t *rx_read_entry;
/*---------------------------------------------------------------------------*/
/* The outgoing frame buffer */
#define TX_BUF_PAYLOAD_LEN 180
#define TX_BUF_HDR_LEN       2

static uint8_t tx_buf[TX_BUF_HDR_LEN + TX_BUF_PAYLOAD_LEN] CC_ALIGN(4);

// /*  For Adaptive CCA_THR By MLC*/
// #ifndef NULL
// #define NULL (void *)0
// #endif /* NULL */
// #define CCA_ADJUSTMENT 0    //0 don't use adjustment; 1 use adjustment
// #define CCA_ADD_PACE 3
// #define ADD_CCA_THR 2
// #define STOP_CCA_THR 1000   //3
#define CCA_RECORD_SIZE 100 // <128
#define ANSWER_FREQUENCY 1/10//ratio rate to be thought as noise
#define ANSWER_MAX_SIZE 10  //ANSWER_MAX_SIZE = 1 / ANSWER_FREQUENCY
#define MINIMAL_CCA_CHANGE_INTERVAL 1
#define ENV_CCA_FLOAT 6
#define GUARD_CCA_THR 2
static signed char cca_recorder[CCA_RECORD_SIZE];
static signed char cca_answer[ANSWER_MAX_SIZE];
static int8_t record_pointer = -1;
static int8_t arr_size = 0;
static int8_t answer_size = 0;   //valid noise_cca number
static int8_t last_env_noise = 127;
static int8_t env_noise=127;
static int8_t last_rssi = RF_CMD_CCA_REQ_RSSI_UNKNOWN;

/*---------------------------------------------------------------------------*/
static uint8_t
rf_is_on(void)
{
  if(!rf_core_is_accessible()) {
    return 0;
  }

  return smartrf_settings_cmd_prop_rx_adv.status == RF_CORE_RADIO_OP_STATUS_ACTIVE;
}
/*---------------------------------------------------------------------------*/
static uint8_t
transmitting(void)
{
  return smartrf_settings_cmd_prop_tx_adv.status == RF_CORE_RADIO_OP_STATUS_ACTIVE;
}
/*---------------------------------------------------------------------------*/
static radio_value_t
get_rssi(void)
{
  uint32_t cmd_status;
  int8_t rssi;
  uint8_t attempts = 0;
  uint8_t was_off = 0;
  rfc_CMD_GET_RSSI_t cmd;

  /* If we are off, turn on first */
  if(!rf_is_on()) {
    was_off = 1;
    if(on() != RF_CORE_CMD_OK) {
      PRINTF("get_rssi: on() failed\n");
      return RF_CMD_CCA_REQ_RSSI_UNKNOWN;
    }
  }

  rssi = RF_CMD_CCA_REQ_RSSI_UNKNOWN;

  while((rssi == RF_CMD_CCA_REQ_RSSI_UNKNOWN || rssi == 0) && ++attempts < 10) {
    memset(&cmd, 0x00, sizeof(cmd));
    cmd.commandNo = CMD_GET_RSSI;

    if(rf_core_send_cmd((uint32_t)&cmd, &cmd_status) == RF_CORE_CMD_ERROR) {
      PRINTF("get_rssi: CMDSTA=0x%08lx\n", cmd_status);
      break;
    } else {
      /* Current RSSI in bits 23:16 of cmd_status */
      rssi = (cmd_status >> 16) & 0xFF;
    }
  }

  /* If we were off, turn back off */
  if(was_off) {
    off();
  }

  return rssi;
}
/*---------------------------------------------------------------------------*/
static uint8_t
get_channel(void)
{
  uint32_t freq_khz;

  freq_khz = smartrf_settings_cmd_fs.frequency * 1000;

  /*
   * For some channels, fractFreq * 1000 / 65536 will return 324.99xx.
   * Casting the result to uint32_t will truncate decimals resulting in the
   * function returning channel - 1 instead of channel. Thus, we do a quick
   * positive integer round up.
   */
  freq_khz += (((smartrf_settings_cmd_fs.fractFreq * 1000) + 65535) / 65536);

  return (freq_khz - DOT_15_4G_CHAN0_FREQUENCY) / DOT_15_4G_CHANNEL_SPACING;
}
/*---------------------------------------------------------------------------*/
static void
set_channel(uint8_t channel)
{
  uint32_t new_freq;
  uint16_t freq, frac;

  new_freq = DOT_15_4G_CHAN0_FREQUENCY + (channel * DOT_15_4G_CHANNEL_SPACING);

  freq = (uint16_t)(new_freq / 1000);
  frac = (new_freq - (freq * 1000)) * 65536 / 1000;

  PRINTF("set_channel: %u = 0x%04x.0x%04x (%lu)\n", channel, freq, frac,
         new_freq);

  smartrf_settings_cmd_prop_radio_div_setup.centerFreq = freq;
  smartrf_settings_cmd_fs.frequency = freq;
  smartrf_settings_cmd_fs.fractFreq = frac;
}
/*---------------------------------------------------------------------------*/
static uint8_t
get_tx_power_array_last_element(void)
{
  const prop_mode_tx_power_config_t *array = TX_POWER_DRIVER;
  uint8_t count = 0;

  while(array->tx_power != OUTPUT_POWER_UNKNOWN) {
    count++;
    array++;
  }
  return count - 1;
}
/*---------------------------------------------------------------------------*/
/* Returns the current TX power in dBm */
static radio_value_t
get_tx_power(void)
{
  return tx_power_current->dbm;
}
/*---------------------------------------------------------------------------*/
/*
 * The caller must make sure to send a new CMD_PROP_RADIO_DIV_SETUP to the
 * radio after calling this function.
 */
static void
set_tx_power(radio_value_t power)
{
  int i;

  for(i = get_tx_power_array_last_element(); i >= 0; --i) {
    if(power <= TX_POWER_DRIVER[i].dbm) {
      /*
       * Merely save the value. It will be used in all subsequent usages of
       * CMD_PROP_RADIO_DIV_SETP, including one immediately after this function
       * has returned
       */
      tx_power_current = &TX_POWER_DRIVER[i];

      return;
    }
  }
}
/*---------------------------------------------------------------------------*/
static int
prop_div_radio_setup(void)
{
  uint32_t cmd_status;
  rfc_radioOp_t *cmd = (rfc_radioOp_t *)&smartrf_settings_cmd_prop_radio_div_setup;

  rf_switch_select_path(RF_SWITCH_PATH_SUBGHZ);

  /* Adjust loDivider depending on the selected band */
  smartrf_settings_cmd_prop_radio_div_setup.loDivider = PROP_MODE_LO_DIVIDER;

  /* Update to the correct TX power setting */
  smartrf_settings_cmd_prop_radio_div_setup.txPower = tx_power_current->tx_power;

  /* Adjust RF Front End and Bias based on the board */
  smartrf_settings_cmd_prop_radio_div_setup.config.frontEndMode =
    RF_CORE_PROP_FRONT_END_MODE;
  smartrf_settings_cmd_prop_radio_div_setup.config.biasMode =
    RF_CORE_PROP_BIAS_MODE;

  // #if RF_CORE_PROP_BIAS_MODE==1

  // #else
  //   #error Bias error
  // #endif
  
  /* Send Radio setup to RF Core */
  if(rf_core_send_cmd((uint32_t)cmd, &cmd_status) != RF_CORE_CMD_OK) {
    PRINTF("prop_div_radio_setup: DIV_SETUP, CMDSTA=0x%08lx, status=0x%04x\n",
           cmd_status, cmd->status);
    return RF_CORE_CMD_ERROR;
  }

  /* Wait until radio setup is done */
  if(rf_core_wait_cmd_done(cmd) != RF_CORE_CMD_OK) {
    PRINTF("prop_div_radio_setup: DIV_SETUP wait, CMDSTA=0x%08lx,"
           "status=0x%04x\n", cmd_status, cmd->status);
    return RF_CORE_CMD_ERROR;
  }

  return RF_CORE_CMD_OK;
}
/*---------------------------------------------------------------------------*/
static uint8_t
rf_cmd_prop_rx()
{
  uint32_t cmd_status;
  rtimer_clock_t t0;
  volatile rfc_CMD_PROP_RX_ADV_t *cmd_rx_adv;
  int ret;

  cmd_rx_adv = (rfc_CMD_PROP_RX_ADV_t *)&smartrf_settings_cmd_prop_rx_adv;
  cmd_rx_adv->status = RF_CORE_RADIO_OP_STATUS_IDLE;

  /*
   * Set the max Packet length. This is for the payload only, therefore
   * 2047 - length offset
   */
  cmd_rx_adv->maxPktLen = DOT_4G_MAX_FRAME_LEN - cmd_rx_adv->lenOffset;

  ret = rf_core_send_cmd((uint32_t)cmd_rx_adv, &cmd_status);

  if(ret != RF_CORE_CMD_OK) {
    PRINTF("rf_cmd_prop_rx: send_cmd ret=%d, CMDSTA=0x%08lx, status=0x%04x\n",
           ret, cmd_status, cmd_rx_adv->status);
    return RF_CORE_CMD_ERROR;
  }

  t0 = RTIMER_NOW();

  while(cmd_rx_adv->status != RF_CORE_RADIO_OP_STATUS_ACTIVE &&
        (RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + ENTER_RX_WAIT_TIMEOUT)));

  /* Wait to enter RX */
  if(cmd_rx_adv->status != RF_CORE_RADIO_OP_STATUS_ACTIVE) {
    PRINTF("rf_cmd_prop_rx: CMDSTA=0x%08lx, status=0x%04x\n",
           cmd_status, cmd_rx_adv->status);
    return RF_CORE_CMD_ERROR;
  }

  return ret;
}
/*---------------------------------------------------------------------------*/
static void
init_rx_buffers(void)
{
  rfc_dataEntry_t *entry;
  int i;

  for(i = 0; i < PROP_MODE_RX_BUF_CNT; i++) {
    entry = (rfc_dataEntry_t *)rx_buf[i];
    entry->status = DATA_ENTRY_STATUS_PENDING;
    entry->config.type = DATA_ENTRY_TYPE_GEN;
    entry->config.lenSz = DATA_ENTRY_LENSZ_WORD;
    entry->length = RX_BUF_SIZE - 8;
    entry->pNextEntry = rx_buf[i + 1];
  }

  ((rfc_dataEntry_t *)rx_buf[PROP_MODE_RX_BUF_CNT - 1])->pNextEntry = rx_buf[0];
}
/*---------------------------------------------------------------------------*/
static int
rx_on_prop(void)
{
  int ret;

  if(rf_is_on()) {
    PRINTF("rx_on_prop: We were on. PD=%u, RX=0x%04x\n",
           rf_core_is_accessible(), smartrf_settings_cmd_prop_rx_adv.status);
    return RF_CORE_CMD_OK;
  }

  /* Put CPE in RX using the currently configured parameters */
  ret = rf_cmd_prop_rx();

  if(ret) {
    ENERGEST_ON(ENERGEST_TYPE_LISTEN);
  }
  #if LOGIC_TEST
  logic_test(1);
  #endif
  return ret;
}
/*---------------------------------------------------------------------------*/
static int
rx_off_prop(void)
{
  uint32_t cmd_status;
  int ret;

  /* If we are off, do nothing */
  if(!rf_is_on()) {
    return RF_CORE_CMD_OK;
  }

  /* Send a CMD_ABORT command to RF Core */
  if(rf_core_send_cmd(CMDR_DIR_CMD(CMD_ABORT), &cmd_status) != RF_CORE_CMD_OK) {
    PRINTF("rx_off_prop: CMD_ABORT status=0x%08lx\n", cmd_status);
    /* Continue nonetheless */
  }

  while(rf_is_on());

  if(smartrf_settings_cmd_prop_rx_adv.status == PROP_DONE_STOPPED ||
     smartrf_settings_cmd_prop_rx_adv.status == PROP_DONE_ABORT) {
    /* Stopped gracefully */
    ENERGEST_OFF(ENERGEST_TYPE_LISTEN);
    ret = RF_CORE_CMD_OK;
  } else {
    PRINTF("rx_off_prop: status=0x%04x\n",
           smartrf_settings_cmd_prop_rx_adv.status);
    ret = RF_CORE_CMD_ERROR;
  }

  return ret;
}
/*---------------------------------------------------------------------------*/
static uint8_t
request(void)
{
  /*
   * We rely on the RDC layer to turn us on and off. Thus, if we are on we
   * will only allow sleep, standby otherwise
   */
  if(rf_is_on()) {
    return LPM_MODE_SLEEP;
  }

  return LPM_MODE_MAX_SUPPORTED;
}
/*---------------------------------------------------------------------------*/
LPM_MODULE(prop_lpm_module, request, NULL, NULL, LPM_DOMAIN_NONE);
/*---------------------------------------------------------------------------*/
static int
prop_fs(void)
{
  uint32_t cmd_status;
  rfc_radioOp_t *cmd = (rfc_radioOp_t *)&smartrf_settings_cmd_fs;

  /* Send the command to the RF Core */
  if(rf_core_send_cmd((uint32_t)cmd, &cmd_status) != RF_CORE_CMD_OK) {
    PRINTF("prop_fs: CMD_FS, CMDSTA=0x%08lx, status=0x%04x\n",
           cmd_status, cmd->status);
    return RF_CORE_CMD_ERROR;
  }

  /* Wait until the command is done */
  if(rf_core_wait_cmd_done(cmd) != RF_CORE_CMD_OK) {
    PRINTF("prop_fs: CMD_FS wait, CMDSTA=0x%08lx, status=0x%04x\n",
           cmd_status, cmd->status);
    return RF_CORE_CMD_ERROR;
  }

  return RF_CORE_CMD_OK;
}
/*---------------------------------------------------------------------------*/
static void
soft_off_prop(void)
{
  uint32_t cmd_status;
  volatile rfc_radioOp_t *cmd = rf_core_get_last_radio_op();

  if(!rf_core_is_accessible()) {
    return;
  }

  /* Send a CMD_ABORT command to RF Core */
  if(rf_core_send_cmd(CMDR_DIR_CMD(CMD_ABORT), &cmd_status) != RF_CORE_CMD_OK) {
    PRINTF("soft_off_prop: CMD_ABORT status=0x%08lx\n", cmd_status);
    return;
  }

  while((cmd->status & RF_CORE_RADIO_OP_MASKED_STATUS) ==
        RF_CORE_RADIO_OP_MASKED_STATUS_RUNNING);
}
/*---------------------------------------------------------------------------*/
static uint8_t
soft_on_prop(void)
{
  if(prop_div_radio_setup() != RF_CORE_CMD_OK) {
    PRINTF("soft_on_prop: prop_div_radio_setup() failed\n");
    return RF_CORE_CMD_ERROR;
  }

  if(prop_fs() != RF_CORE_CMD_OK) {
    PRINTF("soft_on_prop: prop_fs() failed\n");
    return RF_CORE_CMD_ERROR;
  }

  return rx_on_prop();
}
/*---------------------------------------------------------------------------*/
static const rf_core_primary_mode_t mode_prop = {
  soft_off_prop,
  soft_on_prop,
};
/*---------------------------------------------------------------------------*/
static int
init(void)
{
  lpm_register_module(&prop_lpm_module);

  if(ti_lib_chipinfo_chip_family_is_cc13xx() == false) {
    return RF_CORE_CMD_ERROR;
  }

  /* Initialise RX buffers */
  memset(rx_buf, 0, sizeof(rx_buf));

  /* Set of RF Core data queue. Circular buffer, no last entry */
  rx_data_queue.pCurrEntry = rx_buf[0];
  rx_data_queue.pLastEntry = NULL;

  /* Initialize current read pointer to first element (used in ISR) */
  rx_read_entry = rx_buf[0];

  smartrf_settings_cmd_prop_rx_adv.pQueue = &rx_data_queue;
  smartrf_settings_cmd_prop_rx_adv.pOutput = (uint8_t *)&rx_stats;

  set_channel(RF_CORE_CHANNEL);

  if(on() != RF_CORE_CMD_OK) {
    PRINTF("init: on() failed\n");
    return RF_CORE_CMD_ERROR;
  }

  ENERGEST_ON(ENERGEST_TYPE_LISTEN);

  rf_core_primary_mode_register(&mode_prop);

  process_start(&rf_core_process, NULL);

  return 1;
}
/*---------------------------------------------------------------------------*/
static int
prepare(const void *payload, unsigned short payload_len)
{
  int len = MIN(payload_len, TX_BUF_PAYLOAD_LEN);

  memcpy(&tx_buf[TX_BUF_HDR_LEN], payload, len);
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
transmit(unsigned short transmit_len)
{
  int ret;
  uint8_t was_off = 0;
  uint32_t cmd_status;
  volatile rfc_CMD_PROP_TX_ADV_t *cmd_tx_adv;

  /* Length in .15.4g PHY HDR. Includes the CRC but not the HDR itself */
  uint16_t total_length;

  if(!rf_is_on()) {
    was_off = 1;
    if(on() != RF_CORE_CMD_OK) {
      PRINTF("transmit: on() failed\n");
      return RADIO_TX_ERR;
    }
  }

  /*
   * Prepare the .15.4g PHY header
   * MS=0, Length MSBits=0, DW and CRC configurable
   * Total length = transmit_len (payload) + CRC length
   *
   * The Radio will flip the bits around, so tx_buf[0] must have the length
   * LSBs (PHR[15:8] and tx_buf[1] will have PHR[7:0]
   */
  total_length = transmit_len + CRC_LEN;

  tx_buf[0] = total_length & 0xFF;
  tx_buf[1] = (total_length >> 8) + DOT_4G_PHR_DW_BIT + DOT_4G_PHR_CRC_BIT;

  /* Prepare the CMD_PROP_TX_ADV command */
  cmd_tx_adv = (rfc_CMD_PROP_TX_ADV_t *)&smartrf_settings_cmd_prop_tx_adv;

  /*
   * pktLen: Total number of bytes in the TX buffer, including the header if
   * one exists, but not including the CRC (which is not present in the buffer)
   */
  cmd_tx_adv->pktLen = transmit_len + DOT_4G_PHR_LEN;
  cmd_tx_adv->pPkt = tx_buf;

  /* Abort RX */
  rx_off_prop();

  /* Enable the LAST_COMMAND_DONE interrupt to wake us up */
  rf_core_cmd_done_en(false, false);

  ret = rf_core_send_cmd((uint32_t)cmd_tx_adv, &cmd_status);

  if(ret) {
    /* If we enter here, TX actually started */
    ENERGEST_OFF(ENERGEST_TYPE_LISTEN);
    ENERGEST_ON(ENERGEST_TYPE_TRANSMIT);

    watchdog_periodic();

    /* Idle away while the command is running */
    while((cmd_tx_adv->status & RF_CORE_RADIO_OP_MASKED_STATUS)
          == RF_CORE_RADIO_OP_MASKED_STATUS_RUNNING) {
      lpm_sleep();
    }

    if(cmd_tx_adv->status == RF_CORE_RADIO_OP_STATUS_PROP_DONE_OK) {
      /* Sent OK */
      RIMESTATS_ADD(lltx);
      ret = RADIO_TX_OK;
    } else {
      /* Operation completed, but frame was not sent */
      PRINTF("transmit: Not Sent OK status=0x%04x\n",
             cmd_tx_adv->status);
      ret = RADIO_TX_ERR;
    }
  } else {
    /* Failure sending the CMD_PROP_TX command */
    PRINTF("transmit: PROP_TX_ERR ret=%d, CMDSTA=0x%08lx, status=0x%04x\n",
           ret, cmd_status, cmd_tx_adv->status);
    ret = RADIO_TX_ERR;
  }

  /*
   * Update ENERGEST state here, before a potential call to off(), which
   * will correctly update it if required.
   */
  ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);
  ENERGEST_ON(ENERGEST_TYPE_LISTEN);

  /*
   * Disable LAST_FG_COMMAND_DONE interrupt. We don't really care about it
   * except when we are transmitting
   */
  rf_core_cmd_done_dis(false);

  /* Workaround. Set status to IDLE */
  cmd_tx_adv->status = RF_CORE_RADIO_OP_STATUS_IDLE;

  rx_on_prop();

  if(was_off) {
    off();
  }

  return ret;
}
/*---------------------------------------------------------------------------*/
static int
send(const void *payload, unsigned short payload_len)
{
  prepare(payload, payload_len);
  return transmit(payload_len);
}
/*---------------------------------------------------------------------------*/
static int
read_frame(void *buf, unsigned short buf_len)
{
  rfc_dataEntryGeneral_t *entry = (rfc_dataEntryGeneral_t *)rx_read_entry;
  uint8_t *data_ptr = &entry->data;
  int len = 0;

  if(entry->status == DATA_ENTRY_STATUS_FINISHED) {

    /*
     * First 2 bytes in the data entry are the length.
     * Our data entry consists of: Payload + RSSI (1 byte) + Status (1 byte)
     * This length includes all of those.
     */
    len = (*(uint16_t *)data_ptr);
    data_ptr += 2;
    len -= 2;

    if(len > 0) {
      if(len <= buf_len) {
        memcpy(buf, data_ptr, len);
      }

      packetbuf_set_attr(PACKETBUF_ATTR_RSSI, (int8_t)data_ptr[len]);
      packetbuf_set_attr(PACKETBUF_ATTR_LINK_QUALITY, 0x7F);
    }

    /* Move read entry pointer to next entry */
    rx_read_entry = entry->pNextEntry;
    entry->status = DATA_ENTRY_STATUS_PENDING;
  }

  return len;
}
/*---------------------------------------------------------------------------*/
static int
channel_clear(void)
{
  uint8_t was_off = 0;
  uint32_t cmd_status;
  int8_t rssi = RF_CMD_CCA_REQ_RSSI_UNKNOWN;

  /*
   * If we are in the middle of a BLE operation, we got called by ContikiMAC
   * from within an interrupt context. Indicate a clear channel
   */
  if(rf_ble_is_active() == RF_BLE_ACTIVE) {
    return RF_CCA_CLEAR;
  }

  if(!rf_core_is_accessible()) {
    was_off = 1;
    if(on() != RF_CORE_CMD_OK) {
      PRINTF("channel_clear: on() failed\n");
      if(was_off) {
        off();
      }
      return RF_CCA_CLEAR;
    }
  } else {
    if(transmitting()) {
      PRINTF("channel_clear: called while in TX\n");
      return RF_CCA_CLEAR;
    }
  }

  while(rssi == RF_CMD_CCA_REQ_RSSI_UNKNOWN || rssi == 0) {
    if(rf_core_send_cmd(CMDR_DIR_CMD(CMD_GET_RSSI), &cmd_status)
       != RF_CORE_CMD_OK) {
      break;
    }
    /* Current RSSI in bits 23:16 of cmd_status */
    rssi = (cmd_status >> 16) & 0xFF;
  }

  if(was_off) {
    off();
  }
  last_rssi = rssi; //By mlc
  // printf("mlc rssi:%d thr:%d",rssi,rssi_threshold);
  if(rssi >= rssi_threshold) {
    return RF_CCA_BUSY;
  }

  return RF_CCA_CLEAR;
}
/*---------------------------------------------------------------------------*/
static int
receiving_packet(void)
{
  if(!rf_is_on()) {
    return 0;
  }

  if(channel_clear() == RF_CCA_CLEAR) {
    return 0;
  }

  return 1;
}
/*---------------------------------------------------------------------------*/
static int
pending_packet(void)
{
  int rv = 0;
  volatile rfc_dataEntry_t *entry = (rfc_dataEntry_t *)rx_data_queue.pCurrEntry;

  /* Go through all RX buffers and check their status */
  do {
    if(entry->status == DATA_ENTRY_STATUS_FINISHED) {
      rv += 1;
      process_poll(&rf_core_process);
    }

    entry = (rfc_dataEntry_t *)entry->pNextEntry;
  } while(entry != (rfc_dataEntry_t *)rx_data_queue.pCurrEntry);

  /* If we didn't find an entry at status finished, no frames are pending */
  return rv;
}
/*---------------------------------------------------------------------------*/
static int
on(void)
{
  /*
   * If we are in the middle of a BLE operation, we got called by ContikiMAC
   * from within an interrupt context. Abort, but pretend everything is OK.
   */
  if(rf_ble_is_active() == RF_BLE_ACTIVE) {
    return RF_CORE_CMD_OK;
  }

  /*
   * Request the HF XOSC as the source for the HF clock. Needed before we can
   * use the FS. This will only request, it will _not_ perform the switch.
   */
  oscillators_request_hf_xosc();

  if(rf_is_on()) {
    PRINTF("on: We were on. PD=%u, RX=0x%04x \n", rf_core_is_accessible(),
           smartrf_settings_cmd_prop_rx_adv.status);
    return RF_CORE_CMD_OK;
  }

  if(!rf_core_is_accessible()) {
    if(rf_core_power_up() != RF_CORE_CMD_OK) {
      PRINTF("on: rf_core_power_up() failed\n");

      rf_core_power_down();

      return RF_CORE_CMD_ERROR;
    }

    /* Keep track of RF Core mode */
    rf_core_set_modesel();

    /* Apply patches to radio core */
    rf_patch_cpe_genfsk();
    while(!HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFACKIFG));
    HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFACKIFG) = 0;
    rf_patch_rfe_genfsk();

    /* Initialize bus request */
    HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFACKIFG) = 0;
    HWREG(RFC_DBELL_BASE + RFC_DBELL_O_CMDR) =
      CMDR_DIR_CMD_1BYTE(CMD_BUS_REQUEST, 1);

    /* set VCOLDO reference */
    ti_lib_rfc_adi3vco_ldo_voltage_mode(true);

    /* Let CC13xxware automatically set a correct value for RTRIM for us */
    ti_lib_rfc_rtrim((rfc_radioOp_t *)&smartrf_settings_cmd_prop_radio_div_setup);

    /* Make sure BUS_REQUEST is done */
    while(!HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFACKIFG));
    HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFACKIFG) = 0;

    if(rf_core_start_rat() != RF_CORE_CMD_OK) {
      PRINTF("on: rf_core_start_rat() failed\n");

      rf_core_power_down();

      return RF_CORE_CMD_ERROR;
    }
  }

  rf_core_setup_interrupts(false);

  init_rx_buffers();

  /*
   * Trigger a switch to the XOSC, so that we can subsequently use the RF FS
   * This will block until the XOSC is actually ready, but give how we
   * requested it early on, this won't be too long a wait/
   */
  oscillators_switch_to_hf_xosc();

  if(prop_div_radio_setup() != RF_CORE_CMD_OK) {
    PRINTF("on: prop_div_radio_setup() failed\n");
    return RF_CORE_CMD_ERROR;
  }

  if(prop_fs() != RF_CORE_CMD_OK) {
    PRINTF("on: prop_fs() failed\n");
    return RF_CORE_CMD_ERROR;
  }

  return rx_on_prop();
}
/*---------------------------------------------------------------------------*/
static int
off(void)
{
  #if LOGIC_TEST
   logic_test(0);
  #endif
  /*
   * If we are in the middle of a BLE operation, we got called by ContikiMAC
   * from within an interrupt context. Abort, but pretend everything is OK.
   */
  if(rf_ble_is_active() == RF_BLE_ACTIVE) {
    return RF_CORE_CMD_OK;
  }

  rx_off_prop();
  rf_core_power_down();

  ENERGEST_OFF(ENERGEST_TYPE_LISTEN);

  /* Switch HF clock source to the RCOSC to preserve power */
  oscillators_switch_to_hf_rc();

  /* We pulled the plug, so we need to restore the status manually */
  smartrf_settings_cmd_prop_rx_adv.status = RF_CORE_RADIO_OP_STATUS_IDLE;

  return RF_CORE_CMD_OK;
}
/*---------------------------------------------------------------------------*/
static radio_result_t
get_value(radio_param_t param, radio_value_t *value)
{
  if(!value) {
    return RADIO_RESULT_INVALID_VALUE;
  }

  switch(param) {
  case RADIO_PARAM_POWER_MODE:
    /* On / off */
    *value = rf_is_on() ? RADIO_POWER_MODE_ON : RADIO_POWER_MODE_OFF;
    return RADIO_RESULT_OK;
  case RADIO_PARAM_CHANNEL:
    *value = (radio_value_t)get_channel();
    return RADIO_RESULT_OK;
  case RADIO_PARAM_TXPOWER:
    *value = get_tx_power();
    return RADIO_RESULT_OK;
  case RADIO_PARAM_CCA_THRESHOLD:
    *value = rssi_threshold;
    return RADIO_RESULT_OK;
  case RADIO_PARAM_RSSI:
    *value = get_rssi();

    if(*value == RF_CMD_CCA_REQ_RSSI_UNKNOWN) {
      return RADIO_RESULT_ERROR;
    } else {
      return RADIO_RESULT_OK;
    }
  case RADIO_CONST_CHANNEL_MIN:
    *value = 0;
    return RADIO_RESULT_OK;
  case RADIO_CONST_CHANNEL_MAX:
    *value = DOT_15_4G_CHANNEL_MAX;
    return RADIO_RESULT_OK;
  case RADIO_CONST_TXPOWER_MIN:
    *value = TX_POWER_DRIVER[get_tx_power_array_last_element()].dbm;
    return RADIO_RESULT_OK;
  case RADIO_CONST_TXPOWER_MAX:
    *value = OUTPUT_POWER_MAX;
    return RADIO_RESULT_OK;
  default:
    return RADIO_RESULT_NOT_SUPPORTED;
  }
}
/*---------------------------------------------------------------------------*/
static radio_result_t
set_value(radio_param_t param, radio_value_t value)
{
  uint8_t was_off = 0;
  radio_result_t rv = RADIO_RESULT_OK;

  switch(param) {
  case RADIO_PARAM_POWER_MODE:
    if(value == RADIO_POWER_MODE_ON) {
      if(on() != RF_CORE_CMD_OK) {
        PRINTF("set_value: on() failed (1)\n");
        return RADIO_RESULT_ERROR;
      }
      return RADIO_RESULT_OK;
    }
    if(value == RADIO_POWER_MODE_OFF) {
      off();
      return RADIO_RESULT_OK;
    }
    return RADIO_RESULT_INVALID_VALUE;
  case RADIO_PARAM_CHANNEL:
    if(value < 0 ||
       value > DOT_15_4G_CHANNEL_MAX) {
      return RADIO_RESULT_INVALID_VALUE;
    }

    if(get_channel() == (uint8_t)value) {
      /* We already have that very same channel configured.
       * Nothing to do here. */
      return RADIO_RESULT_OK;
    }

    set_channel((uint8_t)value);
    break;
  case RADIO_PARAM_TXPOWER:
    if(value < TX_POWER_DRIVER[get_tx_power_array_last_element()].dbm ||
       value > OUTPUT_POWER_MAX) {
      return RADIO_RESULT_INVALID_VALUE;
    }

    soft_off_prop();

    set_tx_power(value);

    if(soft_on_prop() != RF_CORE_CMD_OK) {
      PRINTF("set_value: soft_on_prop() failed\n");
      rv = RADIO_RESULT_ERROR;
    }

    return RADIO_RESULT_OK;
  case RADIO_PARAM_RX_MODE:
    return RADIO_RESULT_OK;
  case RADIO_PARAM_CCA_THRESHOLD:
    rssi_threshold = (int8_t)value;
    break;
  default:
    return RADIO_RESULT_NOT_SUPPORTED;
  }

  /* If we reach here we had no errors. Apply new settings */
  if(!rf_is_on()) {
    was_off = 1;
    if(on() != RF_CORE_CMD_OK) {
      PRINTF("set_value: on() failed (2)\n");
      return RADIO_RESULT_ERROR;
    }
  }

  if(rx_off_prop() != RF_CORE_CMD_OK) {
    PRINTF("set_value: rx_off_prop() failed\n");
    rv = RADIO_RESULT_ERROR;
  }

  if(soft_on_prop() != RF_CORE_CMD_OK) {
    PRINTF("set_value: rx_on_prop() failed\n");
    rv = RADIO_RESULT_ERROR;
  }

  /* If we were off, turn back off */
  if(was_off) {
    off();
  }

  return rv;
}
/*---------------------------------------------------------------------------*/
static radio_result_t
get_object(radio_param_t param, void *dest, size_t size)
{
  return RADIO_RESULT_NOT_SUPPORTED;
}
/*---------------------------------------------------------------------------*/
static radio_result_t
set_object(radio_param_t param, const void *src, size_t size)
{
  return RADIO_RESULT_NOT_SUPPORTED;
}
/*---------------------------------------------------------------------------*/
void add_cca(void){
  static int8_t compute_noise_count = 0;
  static rtimer_clock_t last_change_time = 0;
  static int8_t change_time_count = 0;
  if(RTIMER_CLOCK_LT(RTIMER_NOW(),last_change_time + RTIMER_ARCH_SECOND/2) == 0){
    if(change_time_count < 2 *  MINIMAL_CCA_CHANGE_INTERVAL ){    //time unit is 0.5s
      change_time_count++;
     } 
    last_change_time = RTIMER_NOW();
  }
  compute_noise_count++;
  if(last_rssi == RF_CMD_CCA_REQ_RSSI_UNKNOWN){
    return;
  }
  if(record_pointer == -1){
    cca_recorder[0] = last_rssi;
    record_pointer = 0;
  }else{
    cca_recorder[record_pointer] = last_rssi;
  }
  record_pointer = (record_pointer+1)%CCA_RECORD_SIZE;
  if(arr_size < CCA_RECORD_SIZE){ 
    arr_size++;
  }
  if(compute_noise_count == (int)CCA_RECORD_SIZE * ANSWER_FREQUENCY){
  if(change_time_count == 2 *  MINIMAL_CCA_CHANGE_INTERVAL ){
      env_noise = compute_noise();
      if((env_noise - last_env_noise > GUARD_CCA_THR || last_env_noise - env_noise > GUARD_CCA_THR) && env_noise != RF_CMD_CCA_REQ_RSSI_UNKNOWN){
        last_env_noise = env_noise;
        // reset_cca_thr(env_noise + ENV_CCA_FLOAT + GUARD_CCA_THR);
#if MLCDEBUG
        printf("mlc resetTHR:%d",env_noise + ENV_CCA_FLOAT + GUARD_CCA_THR);
#endif
        if(set_value(RADIO_PARAM_CCA_THRESHOLD,(int)(env_noise + ENV_CCA_FLOAT + GUARD_CCA_THR)) != RADIO_RESULT_OK){
          PRINTF("set CCA_THR failed");
        }
        int temp_value0;
        if(get_value(RADIO_PARAM_CCA_THRESHOLD,&temp_value0) != RADIO_RESULT_OK){
          PRINTF("get CCA_THR failed");
        }
        last_env_noise = (int8_t)temp_value0 - ENV_CCA_FLOAT - GUARD_CCA_THR;
        change_time_count = 0;
      }
    }
    compute_noise_count = 0;
  }
  last_rssi = RF_CMD_CCA_REQ_RSSI_UNKNOWN;
  return;
}
/*---------------------------------------------------------------------------*/
int8_t compute_noise(void){
  // printf("1\n");   
  int8_t i;
  answer_size = 0;
  int8_t container_size = 0;
  struct cca_node container[CCA_RECORD_SIZE];
  if(arr_size == 0){
    return RF_CMD_CCA_REQ_RSSI_UNKNOWN;     //RF_CMD_CCA_REQ_RSSI_UNKNOWN means invalid env_rssi
  }
  for(i = 0; i<arr_size;i++){
  // printf("2\n");
      signed char temp2 = cca_recorder[i];
      int8_t j;
      for( j = 0; j< container_size;j++){
        if(container[j].val == temp2){
          container[j].count +=1;
          break;
        }
      }
      if(j == container_size){
        container[j].val = temp2;
        container[j].count = 1;
        container_size+=1;
      }
  }
  if(container_size == 0){
    return RF_CMD_CCA_REQ_RSSI_UNKNOWN;
  }
  int8_t k,kk;
  for( k = 0; k< container_size; k++){    
    if(container[k].count >= CCA_RECORD_SIZE*ANSWER_FREQUENCY){ //calculate frequency of each RSSI value
      cca_answer[answer_size]= container[k].val;
      answer_size++;
    }
  }
  if(answer_size == 0){
    return RF_CMD_CCA_REQ_RSSI_UNKNOWN;
  }
  for(k = 0; k < answer_size;k++){    //sort cca_answer
    int8_t temp4;
    for(kk = k+1;kk<answer_size;kk++){
      if(cca_answer[kk] < cca_answer[k]){
        temp4 = cca_answer[kk];
        cca_answer[kk] = cca_answer[k];
        cca_answer[k] = temp4;
      }
    }
  }
  for(k = 0; k < answer_size-1;k++){
    if(cca_answer[k+1] - cca_answer[k] > 2){
      break;
    }
  }
  return cca_answer[k];
}

int8_t get_cca_thr(void){
  int8_t temp_value0;
  temp_value0 = rssi_threshold;
  return temp_value0;
}
/*---------------------------------------------------------------------------*/
const struct radio_driver prop_mode_driver = {
  init,
  prepare,
  transmit,
  send,
  read_frame,
  channel_clear,
  receiving_packet,
  pending_packet,
  on,
  off,
  get_value,
  set_value,
  get_object,
  set_object,
};
/*---------------------------------------------------------------------------*/
/**
 * @}
 */
