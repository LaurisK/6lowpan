
/**
  ******************************************************************************
  * @file    radio-driver.h
  * @author  SRA Application Team
  * @brief   Header file for S2LP radio configuration/driver
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/*---------------------------------------------------------------------------*/
#ifndef RADIO_DRIVER_H__
#define RADIO_DRIVER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "s2lp.h"
#include "api_radio.h"
#include "RTE_Components.h"
#include "project-conf.h"

/**
 * @addtogroup ST_Radio
 * @ingroup Contiki-NG_STM32_Library
 * @{
 * @file subGHz radio configuration file for Contiki
 */
#if (defined S2868A1) || (defined S2868A2)
  #define USE_RADIO_868MHz
#elif defined(S2915A1)
  #define USE_RADIO_915MHz
#else /*!X_NUCLEO_S2868A1 && !X_NUCLEO_S2915A1*/
#error RADIO Nucleo Shield undefined or unsupported
#endif /*X_NUCLEO_S2868A1 || X_NUCLEO_S2915A1*/

/*  Radio configuration parameters  */
#define XTAL_OFFSET_PPM             0
#define INFINITE_TIMEOUT            0.0

/* Guard time allowed for the external reference to stabilise before the PLL is asked to
 * lock. ST bring-up guide p.14: with a TCXO the enable is outside the S2-LP and a guard
 * timer is required before any Tx or Rx, otherwise the PLL will not lock. On this board
 * the TCXO shares the radio supply (PB1, asserted once in MX_GPIO_Init) rather than
 * having its own enable line, so the wait is taken once during radio init.
 * 5ms covers the usual 1-3ms TCXO startup with margin; raise it if a slower part is
 * fitted. */
#define RADIO_TCXO_STARTUP_MS       5

//@TODO: Validate CHANNEL_NUMBER_MIN / MAX values
#ifdef USE_RADIO_433MHz
#define BASE_FREQUENCY              433.0e6
#define CHANNEL_NUMBER_MIN          0
#define CHANNEL_NUMBER_MAX          13
#endif /*USE_RADIO_433MHz*/

#ifdef USE_RADIO_868MHz
#define BASE_FREQUENCY              868.0e6
#define CHANNEL_NUMBER_MIN          0
#define CHANNEL_NUMBER_MAX          13
#endif /*USE_RADIO_868MHz*/

#ifdef USE_RADIO_915MHz
#define BASE_FREQUENCY              915.0e6
#define CHANNEL_NUMBER_MIN          0
#define CHANNEL_NUMBER_MAX          13
#endif /*USE_RADIO_915MHz*/

#define RADIO_IRQ_ENABLE()    S2868A2_EXIT_CRITICAL()
#define RADIO_IRQ_DISABLE()   S2868A2_ENTER_CRITICAL()

#define    RADIO_GPIO_MODE_DIGITAL_OUTPUT_LP                S2LP_GPIO_MODE_DIGITAL_OUTPUT_LP
#define    RADIO_GPIO_DIG_OUT_IRQ                           S2LP_GPIO_DIG_OUT_IRQ

#define radio_spi_busy() (HAL_SPI_GetState(&hspi) != HAL_SPI_STATE_READY)

#define CHANNEL_SPACE               200e3
#define CHANNEL_NUMBER              0
#define IEEE802154_CONF_DEFAULT_CHANNEL CHANNEL_NUMBER
#define MODULATION_SELECT           MOD_2GFSK_BT05

/*------------------------------------------------------------------------
 *  Environment-specific radio parameters (select in project-conf.h)
 *
 *  RADIO_ENV_INDOOR  : 100 kbps on-air (50 kbps effective w/ FEC)
 *                      h = 1.0, BW = 200 kHz, shorter preamble
 *  RADIO_ENV_OUTDOOR :  50 kbps on-air (25 kbps effective w/ FEC)
 *                      h = 1.0, BW = 100 kHz, longer preamble
 *
 *  Careful with the CLOCKREC names: the S2LP_Library numbering is offset by
 *  one from the datasheet and from the ST bring-up guide. CLOCKREC1_VALUE is
 *  written to CLOCKREC1_ADDR (0x20), which the datasheet calls CLOCKREC2, and
 *  CLOCKREC0_VALUE goes to 0x21, the datasheet's CLOCKREC1. So when the guide
 *  says "CLOCKREC2 = 0x28 / CLOCKREC1 = 0x58" that maps to
 *  CLOCKREC1_VALUE = 0x28 / CLOCKREC0_VALUE = 0x58 here.
 *----------------------------------------------------------------------*/
#if defined(RADIO_ENV_INDOOR)

#define DATARATE                    100000  /* bps (on-air symbol rate)           */
#define FREQ_DEVIATION              50e3    /* Hz  – h = 2·Fd / DR = 1.0         */
#define BANDWIDTH                   200.0e3 /* Hz  – Carson: 2·(Fd + DR/2)       */
#define RSSI_RX_THRESHOLD          -112.0   /* dBm – 3 dB below ~-109 sens.      */
#define AFC_FAST_PERIOD             0x20    /* symbols in fast-gain window        */
#define AFC_FAST_GAIN               2       /* log2 gain during acquisition       */
#define AFC_SLOW_GAIN               3       /* log2 gain during tracking          */
#define CLOCKREC1_VALUE             0x28    /* P_SLOW=1, DLL, I_SLOW=8           */
#define CLOCKREC0_VALUE             0x58    /* P_FAST=2, 16-sym postfilt, I_FAST=8*/

#elif defined(RADIO_ENV_OUTDOOR)

#define DATARATE                    50000   /* bps                                */
#define FREQ_DEVIATION              25e3    /* Hz  – h = 1.0                      */
#define BANDWIDTH                   100.0e3 /* Hz  – Carson: 2·(Fd + DR/2)       */
#define RSSI_RX_THRESHOLD          -118.0   /* dBm – S2LP sens. ~-115 @50k+FEC   */
#define AFC_FAST_PERIOD             0x30    /* symbols in fast-gain window        */
#define AFC_FAST_GAIN               2       /* log2 gain during acquisition       */
#define AFC_SLOW_GAIN               3       /* log2 gain during tracking          */
#define CLOCKREC1_VALUE             0x28    /* P_SLOW=1, DLL, I_SLOW=8           */
#define CLOCKREC0_VALUE             0x28    /* P_FAST=1, 8-sym postfilt, I_FAST=8 */

#else
#error "Define RADIO_ENV_INDOOR or RADIO_ENV_OUTDOOR in project-conf.h"
#endif

#define RADIO_POWER_DBM_MAX         14
#define RADIO_POWER_DBM_MIN        -31

#define POWER_DBM                   12.0
#define POWER_INDEX                 7

#define RSSI_TX_THRESHOLD          -90.0   /* dBm – CCA threshold for CSMA */

/*  Packet configuration parameters  */
#if RADIO_LONG_PREAMBLE
#define PREAMBLE_LENGTH             PREAMBLE_BYTE(64)
#elif defined(RADIO_ENV_INDOOR)
#define PREAMBLE_LENGTH             PREAMBLE_BYTE(8)
#else /* RADIO_ENV_OUTDOOR */
#define PREAMBLE_LENGTH             PREAMBLE_BYTE(12)
#endif

#define SYNC_LENGTH                 SYNC_BYTE(4)
#define SYNC_WORD                   0x7A0E3564
#define VARIABLE_LENGTH             S_ENABLE
#define EXTENDED_LENGTH_FIELD       S_DISABLE
#define CRC_MODE                    PKT_CRC_MODE_32BITS
#define EN_FEC                      S_ENABLE
#define EN_WHITENING                S_ENABLE

#if RADIO_ADDRESS_FILTERING
#define EN_ADDRESS                  S_ENABLE
#else /*!RADIO_ADDRESS_FILTERING*/
#define EN_ADDRESS                  S_DISABLE
#endif /*RADIO_ADDRESS_FILTERING*/

/*  Addresses configuration parameters  */
#define EN_FILT_MY_ADDRESS          S_ENABLE
#define EN_FILT_MULTICAST_ADDRESS   S_DISABLE //We use "MY" and BROADCAST
#define MULTICAST_ADDRESS           0xEE
#define EN_FILT_BROADCAST_ADDRESS   S_ENABLE
#define BROADCAST_ADDRESS           0xFF

#define PREAMBLE_BYTE(v)            (4*v)
#define SYNC_BYTE(v)                (8*v)

#if RADIO_SNIFF_MODE
#define MIN_PERIOD_WAKEUP_MS ((8000*((PREAMBLE_LENGTH/4)-2))/DATARATE)
#define RX_TIMEOUT_MS        30
#endif /*RADIO_SNIFF_MODE*/

/**
 * The MAX_PACKET_LEN is the max allowed len for the packet.
 * If it is more than S2LP_RX_FIFO_SIZE it will be handled with FIFO thresholds.
 * The RADIO supports with its packet handler a length of 65,535 bytes,
 * and in direct mode (without packet handler) there is no limit of data.
 * At system level, the max len is set in PACKETBUF_SIZE, so we se it
 * with PACKETBUF_CONF_SIZE = MAX_PACKET_LEN in contiki-conf.h
 */
#define MAX_PACKET_LEN              S2LP_RX_FIFO_SIZE

/*---------------------------------------------------------------------------*/
extern const struct radio_driver subGHz_radio_driver;
/*---------------------------------------------------------------------------*/

void Radio_process_irq_cb(void);
void RadioOverrideRxCb(void (*overRxCb)(void));

/*---------------------------------------------------------------------------*/
#ifdef __cplusplus
}
#endif

#endif /* RADIO_DRIVER_H__ */
/*---------------------------------------------------------------------------*/
/** @} */
