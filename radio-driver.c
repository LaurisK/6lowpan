/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "radio-driver.h"
#include "packetbuf.h"
#include "main.h"
#include "s2lp_interface.h"
#include "s2lp_management.h"
#include "S2LP_Types.h"
#include "S2LP_PktBasic.h"
#include "S2LP_General.h"
#include "App/common.h"
#include "cmsis_os.h"

/* Private defines ----------------------------------------------------------*/
#if RADIO_ADDRESS_FILTERING
#define ACK_LEN 3
#endif /*RADIO_ADDRESS_FILTERING*/

#define RADIO_WAIT_TIMEOUT (100)
/* The receive threshold is not derived from the measured noise floor: RSSI_THR also gates
 * when AFC starts tracking, so it stays at the sensitivity-derived RSSI_RX_THRESHOLD.
 * Only the CSMA busy level tracks the noise floor. */
#define TX_CSMA_RSSI_OFFSET 8	// signal above which channel is considered to be busy. According to AI some guidance:

#if RADIO_HW_CSMA
#define PERSISTENT_MODE_EN              S_DISABLE
#define CS_PERIOD                       CSMA_PERIOD_64TBIT
#define CS_TIMEOUT                      3
#define MAX_NB                          5
#define BU_COUNTER_SEED                 0xFA21
#define CU_PRESCALER                    32
/* How many times a single send may re-arm the CSMA engine after it reported the channel
 * still busy. Bounds what used to be an open-ended strobe loop; once spent, the transmit
 * is left to time out and report tx_collision to the MAC, which owns the real backoff. */
#define CSMA_MAX_TX_RESTARTS            3
#endif /*RADIO_HW_CSMA*/

/* S2-LP SMPS frequency switching for TX/RX (Change 8, per ST bring-up guide p.22) */
static void smps_set_tx(void) {
	uint8_t regs[2] = {0x9B, 0xF4}; /* PM_CONF3, PM_CONF2 for TX (50MHz osc) */
	S2LPSpiWriteRegisters(PM_CONF3_ADDR, 2, regs);
}

static void smps_set_rx(void) {
	uint8_t regs[2] = {0x8F, 0xF9}; /* PM_CONF3, PM_CONF2 for RX (50MHz osc) */
	S2LPSpiWriteRegisters(PM_CONF3_ADDR, 2, regs);
}

/* Private types ------------------------------------------------------------*/
typedef struct {
	uint8_t operatingChannel;
} sRadioInfo;

typedef enum {
  radio_off,
  radio_on
} eRadioStatus;

/* Pseudo global variables --------------------------------------------------*/
static sRadioInfo radioInfo = {.operatingChannel = CHANNEL_NUMBER};
/* The buffer which holds incoming data. */
static uint16_t rx_num_bytes = 0;
static uint8_t txBuf[MAX_PACKET_LEN]; //this is needed since current SPI write destroys buffer - this will need to be fixed.

static volatile eRadioStatus radio_status = radio_off;
static volatile uint8_t receiving_packet = 0;
static volatile uint8_t transmitting_packet = 0;
static volatile uint8_t pending_packet = 0;
static uint16_t last_packet_rssi = 0;
static uint16_t last_packet_lqi = 0;

static volatile uint32_t last_packet_timestamp = 0;

static int      csma_tx_threshold = RSSI_TX_THRESHOLD;
#if RADIO_HW_CSMA
/* Re-arms spent on the in-flight send. Reset by Radio_transmit(), counted up in the
 * MAX_BO_CCA_REACH handler. */
static volatile uint8_t csma_tx_restarts = 0;
#endif /*RADIO_HW_CSMA*/
static uint8_t  operation_mode = 0;

/* Poll mode disabled by default */
/*static*///uint8_t polling_mode = 0;

/* (Software) frame filtering enabled by default */
#if RADIO_ADDRESS_FILTERING
static uint8_t auto_pkt_filter = 1;
#else /*!RADIO_ADDRESS_FILTERING*/
static uint8_t auto_pkt_filter = 0;
#endif /*RADIO_ADDRESS_FILTERING*/

/* (Software) autoack is enabled by default (CSMA MAC will send by default) */
static uint8_t radio_send_auto_ack = 1;
#if RADIO_HW_CSMA
static uint8_t csma_enabled = 1;
#else /*!RADIO_HW_CSMA*/
static uint8_t csma_enabled = 0;
#endif /*RADIO_HW_CSMA*/
static int conf_tx_power = (int) POWER_DBM; //@TODO: validate

volatile FlagStatus xTxDoneFlag = RESET;
SGpioInit xGpioIRQ = { S2LP_GPIO_3, RADIO_GPIO_MODE_DIGITAL_OUTPUT_LP, RADIO_GPIO_DIG_OUT_IRQ };
SRadioInit xRadioInit = { BASE_FREQUENCY, MODULATION_SELECT, DATARATE, FREQ_DEVIATION, BANDWIDTH };
PktBasicInit xBasicInit = { PREAMBLE_LENGTH, SYNC_LENGTH, SYNC_WORD, VARIABLE_LENGTH, EXTENDED_LENGTH_FIELD, CRC_MODE, EN_ADDRESS, EN_FEC, EN_WHITENING };

#if RADIO_ADDRESS_FILTERING
PktBasicAddressesInit xAddressInit = { EN_FILT_MY_ADDRESS, 0x00, EN_FILT_MULTICAST_ADDRESS, MULTICAST_ADDRESS, EN_FILT_BROADCAST_ADDRESS, BROADCAST_ADDRESS };
#endif /*RADIO_ADDRESS_FILTERING*/

#if RADIO_HW_CSMA
/* Radio CSMA config */
SCsmaInit xCsmaInit = { PERSISTENT_MODE_EN, CS_PERIOD, CS_TIMEOUT, MAX_NB, BU_COUNTER_SEED, CU_PRESCALER };
// refer to radio-driver.h for RSSI Thresholds
SRssiInit xSRssiInit = { .cRssiFlt = 14, .xRssiMode = RSSI_STATIC_MODE, .cRssiThreshdBm = RSSI_TX_THRESHOLD };
#endif /*RADIO_HW_CSMA*/

static int8_t backgroundNoise = (-127);
static uint16_t radioEvtIdOffset;
static fRadioEvtHndl radioEvtHndl;
void (*overridenRxCb)(void);
/* Private functions --------------------------------------------------------*/
/**
 * @brief  radio_refresh_status	refresh and returns S2-LP status
 * @retval S2LPState S2-LP status
 */
static S2LPState radio_refresh_status(void) {
	S2LP_RefreshStatus();
	return g_xStatus.MC_STATE;
}

/**
 * @brief Radio_read_from_fifo can be called from Radio_process_irq_cb or from Radio_read depending on the operating mode.
 * @param buf     - pointer to buffer where data needs to be stored
 * @param bufsize - size of a buffer for data storage
 * @retval bytes count filled to buffer
 */
static int16_t Radio_read_from_fifo(sPacket *packet) {
	uint8_t rx_bytes, retval = 0;

	rx_bytes = S2LP_FIFO_ReadNumberBytesRxFifo();

	if (rx_bytes <= packetbuf_remaininglen(packet)) {
		int32_t rssiRunArr;
		uint32_t packetCrc;
		uint16_t fsc = 0;
		int8_t *rssiRun = &rssiRunArr;
		uint8_t pqiSqi[3];
		uint8_t *rxBuff = (uint8_t*)packetbuf_hdrptr(packet);
		S2LP_ReadFIFO(rx_bytes, rxBuff);
		packetbuf_set_datalen(packet, rx_bytes);
		retval = rx_bytes;
		last_packet_timestamp = HAL_GetTick(); //@TODO: validate
		last_packet_rssi = (uint16_t) S2LP_RADIO_QI_GetRssidBm();
		//last_packet_lqi  = (uint16_t) S2LP_RADIO_QI_GetLqi();
		S2LPSpiReadRegisters(LINK_QUALIF2_ADDR, 2, pqiSqi);
		S2LPSpiReadRegisters(AFC_CORR_ADDR, 1, &pqiSqi[2]);
		S2LPSpiReadRegisters(CRC_FIELD3_ADDR, 4, &packetCrc);
		rssiRunArr = S2LP_RADIO_QI_GetRssidBmRun();
		rssiRun[0] -= 146;
		rssiRun[1] -= 146;
		rssiRun[2] -= 146;
		rssiRun[3] -= 146;
		{
			int16_t localAvg = rssiRun[0] + rssiRun[1] + rssiRun[2] + rssiRun[3];
			if ((-127) != backgroundNoise) {
				localAvg += (backgroundNoise * 36);
				backgroundNoise = localAvg / 40;
				csma_tx_threshold = (backgroundNoise + TX_CSMA_RSSI_OFFSET);
			} else {
				backgroundNoise = localAvg / 4;
			}
		}
		TRice("msg:[RADIO] RX(%d) stats: RSSI(%d), noise(%d %d %d %d - avg(%d)), PQI(%d), CS(%d), SQI(%d), AFC(%d),\n",
			  rx_bytes, (int16_t)last_packet_rssi, rssiRun[0], rssiRun[1], rssiRun[2], rssiRun[3], backgroundNoise, pqiSqi[0], (0x80 & pqiSqi[1]) ? 1 : 0, (0x7F & pqiSqi[1]), (int8_t)pqiSqi[2]);
		pqiSqi[2] = rxBuff[2];
		while (rx_bytes) {
			rx_bytes--;
			fsc += *rxBuff;
			rxBuff++;
		}
		TRice("msg: \t packet info: seqNr(%d), payloadCrc(0x%08X), fsc - %04X.\n \t my addr - %d\n", pqiSqi[2], packetCrc, fsc, xAddressInit.cMyAddress);
		packetbuf_set_attr(packet, PACKETBUF_ATTR_RSSI, last_packet_rssi);
		packetbuf_set_attr(packet, PACKETBUF_ATTR_LINK_QUALITY, last_packet_lqi);
	} else {
		TRice("msg:Buf too small (%d bytes to hold %d bytes)\n", packetbuf_remaininglen(packet), rx_bytes);
	}
//	if (polling_mode) {
	S2LP_CMD_StrobeFlushRxFifo();
//	}

	return retval;
}
/**
 * @brief radio_set_polling_mode is for control of pooling mode.
 * @param buf     - pointer to buffer where data needs to be stored
 */
//static void radio_set_polling_mode(uint8_t enable) {
//	/* Polling Mode  must be fully validated. */
//	TRiceS("msg:POLLING MODE is %s.\r\n", enable?"ENABLED":"DISABLED");
//	polling_mode = enable;
//	if (polling_mode) {
//		/* Disable interrupts */
//		S2LP_GPIO_IrqConfig(RX_DATA_READY, S_DISABLE);
//		S2LP_GPIO_IrqConfig(TX_DATA_SENT, S_DISABLE);
//		S2LP_GPIO_IrqConfig(VALID_SYNC, S_DISABLE);
//	} else {
//		/* Initialize and enable interrupts */
//		S2LP_GPIO_IrqConfig(RX_DATA_READY, S_ENABLE);
//		S2LP_GPIO_IrqConfig(TX_DATA_SENT, S_ENABLE);
//		S2LP_GPIO_IrqConfig(VALID_SYNC, S_ENABLE);
//	}
//}

/**
 * @brief  radio_print_status prints to the UART the status of the radio
 */
static void radio_print_status(S2LPState s) {
	if (s == MC_STATE_STANDBY) {
		TRice("radio-driver: MC_STATE_STANDBY\n");
	} else if (s == MC_STATE_SLEEP) {
		TRice("radio-driver: MC_STATE_SLEEP\n");
	} else if (s == MC_STATE_READY) {
		TRice("radio-driver: MC_STATE_READY\n");
	} else if (s == MC_STATE_TX) {
		TRice("radio-driver: MC_STATE_TX\n");
	} else if (s == MC_STATE_RX) {
		TRice("radio-driver: MC_STATE_RX\n");
	} else if (s == MC_STATE_SLEEP_NOFIFO) {
		TRice("radio-driver: MC_STATE_SLEEP_NOFIFO\n");
	} else if (s == MC_STATE_SYNTH_SETUP) {
		TRice("radio-driver: MC_STATE_SYNTH_SETUP\n");
	} else {
		TRice("radio-driver: status: %X\n", (uint8_t) s);
	}
}

/**
 * @brief  radio_set_ready_state sets the state of the radio to READY
 */
void radio_set_ready_state(void) {
	RADIO_IRQ_DISABLE();
	S2LP_GPIO_IrqClearStatus();

#if RADIO_SNIFF_MODE
  S2LP_GPIO_IrqConfig(RX_DATA_READY,S_DISABLE);
  S2LP_TIM_LdcrMode(S_DISABLE);
  S2LP_TIM_FastRxTermTimer(S_DISABLE);
#endif /*RADIO_SNIFF_MODE*/

	if (radio_refresh_status() == MC_STATE_RX) {
		S2LP_CMD_StrobeSabort();
	} else {
		S2LP_CMD_StrobeReady();
	}
	BUSYWAIT_UNTIL(radio_refresh_status() == MC_STATE_READY, RADIO_WAIT_TIMEOUT);

	S2LP_CMD_StrobeFlushRxFifo();
	receiving_packet = 0;
	pending_packet = 0;
	rx_num_bytes = 0;

	S2LP_GPIO_IrqClearStatus();
	RADIO_IRQ_ENABLE();
}

/**
 * @brief  function to receive channel radio is currently operating on.
 * @retval number of a channel radio is working on.
 */
static uint8_t radio_get_channel(void) {
	uint8_t register_channel;
	/*Next statement is mainly for debugging purpose, it can be commented out. */
	register_channel = S2LP_RADIO_GetChannel();
	if (register_channel != radioInfo.operatingChannel) {
		TRice("wrn:Warning retrieved channel %d != saved channel %d\n", register_channel, radioInfo.operatingChannel );
		radioInfo.operatingChannel = register_channel;
	}

	return register_channel;
}

/**
 * @brief  function to set channel radio to operate on.
 * @param  channel - number of a channel radio to operate on.
 */
static void radio_set_channel(uint8_t channel) {
	/*Channel value has been validated in the calling function. */
	TRice("msg:SET CHANNEL %d.\r\n", channel);

	radioInfo.operatingChannel = channel;
	S2LP_RADIO_SetChannel(radioInfo.operatingChannel);
	S2LP_RADIO_SetChannelSpace(CHANNEL_SPACE);
}

/**
 * @brief  function to receive radio tx power.
 * @retval tx power of radio.
 */
static int32_t radio_get_txpower(void) {
	int32_t register_tx_power;
	register_tx_power = S2LP_RADIO_GetPALeveldBm(POWER_INDEX);
	if (register_tx_power != conf_tx_power) {
		TRice("wrn:Warning retrieved tx power %d != saved tx power %d\n", register_tx_power, conf_tx_power );
		conf_tx_power = register_tx_power;
	}
	return register_tx_power;
}

/**
 * @brief  function to set radio tx power.
 * @param  power - desired tx power of radio.
 */
static void radio_set_txpower(int8_t power) {
	/*Power value is validated in the calling function */
	conf_tx_power = power;

	S2LP_RADIO_SetPALeveldBm(POWER_INDEX, conf_tx_power);
}

/**
 * @brief  function to control auto packet filter function.
 * @param  enable - desired state of auto packet filter function.
 */
static void radio_set_auto_pkt_filter(uint8_t enable) {
	TRice("msg:Set Auto Packet Filtering %d\n", enable);
	auto_pkt_filter = enable;
	S2LP_PCKT_HNDL_SetAutoPcktFilter(enable ? S_ENABLE : S_DISABLE);
}

/**
 * @brief  function to control auto ack function.
 * @param  enable - desired state of auto ack function.
 */
static void radio_set_auto_ack(uint8_t enable) {
	/* Actually CSMA MAC will send anyway, TSCH (that needs them disabled) will not send in any case since the implementation for Packet Basic is done
	 in software */
	radio_send_auto_ack = enable;
}

/**
 * @brief  function to control CSMA Feature (check also RADIO_HW_CSMA macro).
 * @param  enable - desired state of CSMA function.
 */
static void radio_set_csma(uint8_t enable) {
	//@TODO: validate
	csma_enabled = enable;
}

/**
 * @brief  function to receive last received radio packet timestamp in HAL ticks.
 * @retval last received radio packet timestamp in HAL ticks.
 */
static uint32_t radio_get_packet_timestamp(void) {
//@TODO: This is to be validated.
	TRice("msg:radio_get_packet_timestamp: %u\r\n", last_packet_timestamp);
	return last_packet_timestamp;
}

static uint8_t linkaddr2devaddr(linkaddr_t *linkaddr) {
	uint16_t sum = linkaddr->u16[0] + linkaddr->u16[1] + linkaddr->u16[2] + linkaddr->u16[3];
	sum = ((sum & 0xFF) ^ (sum >> 8));
	if ((MULTICAST_ADDRESS == sum) || (BROADCAST_ADDRESS == sum)) {
		sum ^= 0xa5;
	}
	return sum;
}

/**
 * @brief Re-arms the receiver.
 *        The RX FIFO is flushed first, as ST's bring-up guide p.47 shows for every Rx
 *        command: whatever the previous reception left behind - a partial frame from an
 *        aborted RX, bytes belonging to a discarded packet - would otherwise sit at the
 *        head of the FIFO and be read back as the start of the next packet. Every caller
 *        reaches here with the part out of RX (after a Sabort, a Ready strobe or a
 *        completed TX), so nothing in flight is discarded by the flush.
 */
static void RadioSwitchToRx(void) {
	smps_set_rx();
	S2LP_CMD_StrobeFlushRxFifo();
	S2LP_CMD_StrobeRx();
}

static void HandleTxFifoError(void) {
	if (0 == transmitting_packet) {
		S2LP_CMD_StrobeFlushTxFifo();
		RadioSwitchToRx();
	}
}

static void HandleRxFifoError(void) {
	if (0 == transmitting_packet) {
		S2LP_CMD_StrobeSabort();
		RadioSwitchToRx();
	}
}

static void HandleRxError(void) {
	if ((MC_STATE_RX != radio_refresh_status()) && (0 == transmitting_packet)) {
		RadioSwitchToRx();
	}
}

/* API Realization ----------------------------------------------------------*/
static int8_t Radio_on(void) {
	TRice("msg:Radio: on\n");

	if (radio_off == radio_status) {
#if RADIO_SNIFF_MODE
    S2LP_TIM_LdcrMode(S_ENABLE);
    S2LP_TIM_FastRxTermTimer(S_ENABLE);
#endif /*RADIO_SNIFF_MODE*/
		radio_set_ready_state();
		S2LP_FIFO_MuxRxFifoIrqEnable(S_ENABLE);
		RadioSwitchToRx();
		radio_status = radio_on;
		RADIO_IRQ_ENABLE(); //--> Coming from OFF, IRQ ARE DISABLED.
	}
	return 0;
}

static int8_t Radio_off(void) {
	if (radio_on == radio_status) {
		/* Disables the mcu to get IRQ from the RADIO */
		RADIO_IRQ_DISABLE();  //Mind that it will be enabled only in the ON

#if RADIO_SNIFF_MODE
    S2LP_TIM_LdcrMode(S_DISABLE);
    S2LP_TIM_FastRxTermTimer(S_DISABLE);
    S2LP_CMD_StrobeReady();
    RadioSwitchToRx();
#endif /*RADIO_SNIFF_MODE*/

		/* first stop rx/tx */
		S2LP_CMD_StrobeSabort();

		/* Clear any pending irqs */
		S2LP_GPIO_IrqClearStatus();

#if RADIO_SNIFF_MODE
    S2LP_CMD_StrobeReady();
#endif /*RADIO_SNIFF_MODE*/
		BUSYWAIT_UNTIL(radio_refresh_status() == MC_STATE_READY,
				RADIO_WAIT_TIMEOUT);

		if (radio_refresh_status() != MC_STATE_READY) {
			TRice("Radio: failed off->ready\n");
			return 1;
		}
		/* Puts the Radio in STANDBY */
		S2LP_CMD_StrobeStandby();
		BUSYWAIT_UNTIL(radio_refresh_status() == MC_STATE_STANDBY,
				RADIO_WAIT_TIMEOUT);

		if (radio_refresh_status() != MC_STATE_STANDBY) {
			TRice("err:Radio: failed off->stdby\n");
			return 1;
		}

		radio_status = radio_off;
		rx_num_bytes = 0;
	}
	return 0;
}

static int8_t Radio_init(uint16_t evtOffset, fRadioEvtHndl packedEvtHndl) {
	TRice("msg:RADIO INIT IN\n");
	radioEvtIdOffset = evtOffset;
	radioEvtHndl = packedEvtHndl;
	S2LPInterfaceInit();

	/* The reference frequency is already established by S2LPInterfaceInit(): read from the
	 * RF module EEPROM, or measured by S2LP_ManagementComputeXtalFrequency(), with the
	 * library's own 50MHz default standing in when neither is available. Forcing
	 * XTAL_FREQUENCY over the top of that discarded the detected value, and every setting
	 * derived from the reference - datarate, deviation, channel filter, SMPS divider, timer
	 * scaling - would then be computed against the wrong number on any module not fitted
	 * with a 50MHz part. */
	TRice("msg:Radio reference %u Hz\n", S2LP_RADIO_GetXtalFrequency());

	S2LP_CMD_StrobeSres();

	/* SRES restarts the digital core: every register access below is only valid once the
	 * part has reached READY again. ST bring-up guide p.7 makes polling MC_STATE mandatory
	 * here - the 2ms Treset delay alone is explicitly called out as not sufficient. Without
	 * this the EXT_REF write and the whole S2LP_RADIO_Init() sequence can land while the
	 * part is still in reset and be silently lost. */
	BUSYWAIT_UNTIL(MC_STATE_READY == radio_refresh_status(), RADIO_WAIT_TIMEOUT);
	if (MC_STATE_READY != radio_refresh_status()) {
		TRice("err:[RADIO DRV] - not READY after SRES.\n");
		radio_print_status(radio_refresh_status());
	}

	/* Change 9: Configure oscillator type after SRES (which resets all registers) */
#if RADIO_USE_TCXO
	S2LPGeneralSetExtRef(MODE_EXT_XIN); /* Set EXT_REF=1 for TCXO */
	/* The part is now clocked from the external reference. READY only reports that the
	 * digital core is up, it says nothing about the TCXO having settled, so hold here
	 * before anything asks the synthesiser to lock (ST bring-up guide p.14). */
	HAL_Delay(RADIO_TCXO_STARTUP_MS);
#else
	S2LPGeneralSetExtRef(MODE_EXT_XO);  /* Ensure EXT_REF=0 for crystal */
#endif

	/* S2LP Radio config */
	S2LP_RADIO_Init(&xRadioInit);

	S2LP_RADIO_SetChannel(CHANNEL_NUMBER);
	S2LP_RADIO_SetChannelSpace(CHANNEL_SPACE);

#if RADIO_USE_EXTERNAL_PA
	S2LP_RADIO_SetAutoRampingMode(S_ENABLE);
	S2LP_RADIO_SetPALeveldBm(POWER_INDEX, RADIO_PA_DRIVE_DBM);
#else /*!RADIO_USE_EXTERNAL_PA*/
	S2LP_RADIO_SetPALeveldBm(POWER_INDEX, POWER_DBM);
#endif /*RADIO_USE_EXTERNAL_PA*/
	S2LP_RADIO_SetPALevelMaxIndex(POWER_INDEX);

	/* Configures the Radio packet handler part*/
	S2LP_PCKT_BASIC_Init(&xBasicInit);
	{
		SAfcInit afc = {S_ENABLE, S_ENABLE, AFC_MODE_LOOP_CLOSED_ON_SLICER,
		                AFC_FAST_PERIOD, AFC_FAST_GAIN, AFC_SLOW_GAIN};
		S2LP_RADIO_AfcInit(&afc);
	}

	/* --- S2-LP Good Practices (per ST bring-up guide v1.0) --- */

	/* Change 1: Clock recovery - "update strongly required" per PDF p.26 */
	{
		uint8_t clockrec[2] = {CLOCKREC1_VALUE, CLOCKREC0_VALUE};
		S2LPSpiWriteRegisters(CLOCKREC1_ADDR, 2, clockrec);
	}

	/* Change 4: Disable CS_Blanking per PDF p.24 - avoids 0x64 state issue */
	{
		uint8_t tmp;
		S2LPSpiReadRegisters(ANT_SELECT_CONF_ADDR, 1, &tmp);
		tmp &= (uint8_t)(~CS_BLANKING_REGMASK);
		S2LPSpiWriteRegisters(ANT_SELECT_CONF_ADDR, 1, &tmp);
	}

	/* Change 5: Enable Sleep mode B for CSMA (retain Tx FIFO) per PDF p.24 */
	{
		uint8_t tmp;
		S2LPSpiReadRegisters(PM_CONF0_ADDR, 1, &tmp);
		tmp |= 0x01; /* Set SLEEP_MODE_SEL = 1 */
		S2LPSpiWriteRegisters(PM_CONF0_ADDR, 1, &tmp);
	}
	if (S2LP_OK != S2LPManagementRcoCalibration()) {
		TRice("err:[RADIO DRV] - RCO calibration failed.\n");
	}

#if RADIO_ADDRESS_FILTERING
	S2LP_PCKT_HNDL_SetAutoPcktFilter(S_ENABLE);
	S2LP_PCKT_HNDL_SelectSecondarySync(S_DISABLE);
	xAddressInit.cMyAddress = linkaddr2devaddr(&linkaddr_node_addr);
	S2LP_PCKT_BASIC_AddressesInit(&xAddressInit);
	TRice("msg:Node Source address %2X\n", xAddressInit.cMyAddress);
#endif /*RADIO_ADDRESS_FILTERING*/

#if RADIO_HW_CSMA
	S2LP_CSMA_Init(&xCsmaInit);
	S2LP_RADIO_QI_RssiInit(&xSRssiInit);
#endif /*RADIO_HW_CSMA*/

	/* Enable the following interrupt sources, routed to GPIO */
	S2LP_GPIO_IrqDeInit(NULL);
	S2LP_GPIO_IrqClearStatus();
	S2LP_GPIO_IrqConfig(TX_DATA_SENT, S_ENABLE);
	S2LP_GPIO_IrqConfig(RX_DATA_READY, S_ENABLE);

#if RADIO_SNIFF_MODE
  S2LP_GPIO_IrqConfig(VALID_SYNC,S_DISABLE);
  S2LP_GPIO_IrqConfig(RX_DATA_DISC, S_DISABLE);
  S2LP_GPIO_IrqConfig(RX_TIMEOUT, S_DISABLE);
#else /*!RADIO_SNIFF_MODE*/
	S2LP_GPIO_IrqConfig(VALID_SYNC, S_ENABLE);
	S2LP_GPIO_IrqConfig(RX_DATA_DISC, S_ENABLE);
#endif /*RADIO_SNIFF_MODE*/

#if RADIO_HW_CSMA
	S2LP_GPIO_IrqConfig(MAX_BO_CCA_REACH, S_ENABLE);
	S2LP_CSMA_Enable(S_DISABLE); //It will be enabled in TX
#else /*!RADIO_HW_CSMA*/
  S2LP_GPIO_IrqConfig(MAX_BO_CCA_REACH , S_DISABLE);
#endif /*RADIO_HW_CSMA*/

	/* Change 6: Enable FIFO error IRQs for better error recovery per PDF p.48 */
	S2LP_GPIO_IrqConfig(TX_FIFO_ERROR, S_ENABLE);
	S2LP_GPIO_IrqConfig(RX_FIFO_ERROR, S_ENABLE);

#if RADIO_SNIFF_MODE
  SRssiInit xSRssiInit = {
    .cRssiFlt = 14,
    .xRssiMode = RSSI_STATIC_MODE,
    .cRssiThreshdBm = RSSI_TX_THRESHOLD
  };
  S2LP_RADIO_QI_RssiInit(&xSRssiInit);

  S2LP_TIM_SetWakeUpTimerUs(1000*MIN_PERIOD_WAKEUP_MS); //12 ms
  /* set the rx timeout */
  S2LP_TIM_SetRxTimerUs(1000*RX_TIMEOUT_MS); //30 ms

  S2LP_TIM_SleepB(S_ENABLE);

  /* enable LDC mode, FAST RX TERM and start Rx */
  S2LP_TIM_LdcrMode(S_ENABLE);
  /* enable the fast rx timer */
  S2LP_TIM_FastRxTermTimer(S_ENABLE);
#else /*!RADIO_SNIFF_MODE*/

	S2LP_RADIO_QI_SetRssiThreshdBm(RSSI_RX_THRESHOLD);
	SET_INFINITE_RX_TIMEOUT();
	/* Configure Radio */
	S2LP_PCKT_HNDL_SetRxPersistentMode(S_ENABLE);
#endif /*RADIO_SNIFF_MODE*/

	rx_num_bytes = 0;

	/* Configure the radio to route the IRQ signal to its GPIO 3 */
	S2LP_GPIO_Init(&xGpioIRQ);

#if RADIO_USE_EXTERNAL_PA
	S2LP_GPIO_Init(&(SGpioInit){S2LP_GPIO_0, RADIO_PA_GPIO_MODE, RADIO_PA_CSD_SELECT});
	S2LP_GPIO_Init(&(SGpioInit){S2LP_GPIO_1, RADIO_PA_GPIO_MODE, RADIO_PA_CTX_SELECT});
	S2LP_GPIO_Init(&(SGpioInit){S2LP_GPIO_2, RADIO_PA_GPIO_MODE, RADIO_PA_VCONT_SELECT});
#endif /*RADIO_USE_EXTERNAL_PA*/
}

	RadioSwitchToRx();

	radio_status = radio_on;

	TRice("msg:Radio init done\n");
	return 0;
}

static eTransmitRes Radio_prepare(sPacket *packet) {
	if (0 != operation_mode) {
		return tx_err;
	}
	/* Checks if the payload length is supported: actually this can't happen, by system design, but it is safer to have this for sanity check. */
	if (PACKETBUF_SIZE < packetbuf_totlen(packet)) {
		TRice("msg:Payload len too big (> %d), error.\n", PACKETBUF_SIZE);
		return tx_err;
	}

	/* Sets the length of the packet to send */
	RADIO_IRQ_DISABLE();

	radio_set_ready_state();
	if (radio_refresh_status() != MC_STATE_READY) {
		TRice("Set Ready State failed.\n");
		radio_print_status(radio_refresh_status());
		S2LP_CMD_StrobeSabort();
#if RADIO_SNIFF_MODE
    S2LP_TIM_LdcrMode(S_ENABLE);
    S2LP_TIM_FastRxTermTimer(S_ENABLE);
    S2LP_GPIO_IrqConfig(RX_DATA_READY,S_ENABLE);
#endif /*RADIO_SNIFF_MODE*/
    	RadioSwitchToRx();

		RADIO_IRQ_ENABLE();
		return tx_err;
	}

#if RADIO_ADDRESS_FILTERING
	if (auto_pkt_filter) {
		if (packetbuf_holds_broadcast(packet)) {
			S2LP_PCKT_HNDL_SetRxSourceReferenceAddress(BROADCAST_ADDRESS);
		} else {
			S2LP_PCKT_HNDL_SetRxSourceReferenceAddress(linkaddr2devaddr((linkaddr_t*)packetbuf_addr(packet, PACKETBUF_ADDR_RECEIVER)));
			TRice("msg:unicast to - %d.\n", linkaddr2devaddr((linkaddr_t*)packetbuf_addr(packet, PACKETBUF_ADDR_RECEIVER)));
		}
	}
#endif /*RADIO_ADDRESS_FILTERING*/

	S2LP_CMD_StrobeFlushTxFifo();

	S2LP_PCKT_BASIC_SetPayloadLength(packetbuf_totlen(packet));

	/* Currently does no happen since S2LP_RX_FIFO_SIZE == MAX_PACKET_LEN also note that S2LP_RX_FIFO_SIZE == S2LP_TX_FIFO_SIZE */
	if (packetbuf_totlen(packet) > S2LP_TX_FIFO_SIZE) {
		TRice("msg:Payload bigger than FIFO size.\n");
		RADIO_IRQ_ENABLE();
		return tx_err;
	} else {
	    memcpy(txBuf, packetbuf_hdrptr(packet), packetbuf_totlen(packet));
		S2LP_WriteFIFO(packetbuf_totlen(packet), txBuf);
	}

	RADIO_IRQ_ENABLE();
	return tx_ok;
}

static void Exit_TX(void) {
	rx_num_bytes = 0;

	RADIO_IRQ_DISABLE();

#if RADIO_SNIFF_MODE
  S2LP_TIM_LdcrMode(S_ENABLE);
  S2LP_TIM_FastRxTermTimer(S_ENABLE);
  S2LP_GPIO_IrqConfig(RX_DATA_READY,S_ENABLE);
#endif /*RADIO_SNIFF_MODE*/

    RadioSwitchToRx();
	BUSYWAIT_UNTIL(radio_refresh_status() == MC_STATE_RX
#if RADIO_SNIFF_MODE
                 || radio_refresh_status() == MC_STATE_SLEEP_NOFIFO
#endif /*RADIO_SNIFF_MODE*/
			,RADIO_WAIT_TIMEOUT);

	S2LP_CMD_StrobeFlushTxFifo();

	S2LP_GPIO_IrqClearStatus();
	RADIO_IRQ_ENABLE();

}

static eTransmitRes Radio_transmit(uint16_t payloadLen) {
	int retval = tx_err;
	eRadioStatus radio_state = radio_status;
	if (0 != operation_mode) {
		return tx_err;
	}
	/* This function blocks until the packet has been transmitted */
	if (0 == transmitting_packet) {
		TRice("msg:Radio TRANSMIT: ERROR, packet is NOT prepared.\n");
		return tx_err;
	}

	if (radio_off == radio_status) {
		Radio_on();
	}

	RADIO_IRQ_DISABLE();

	S2LP_GPIO_IrqClearStatus();
	RADIO_IRQ_ENABLE();

#if RADIO_HW_CSMA
	csma_tx_restarts = 0;
	if (csma_enabled) { //@TODO: add an API to enable/disable CSMA
		S2LP_CSMA_Enable(S_ENABLE);
		S2LP_RADIO_QI_SetRssiThreshdBm(csma_tx_threshold);
		retval = tx_collision;
	}
#endif  /*RADIO_HW_CSMA*/

	xTxDoneFlag = RESET;
	smps_set_tx(); /* Change 8: Switch SMPS to TX frequency before transmit */
	S2LP_CMD_StrobeTx();
	/* wait for TX done */
		/*To be on the safe side we put a timeout. */
	osDelay(1);
	BUSYWAIT_UNTIL(xTxDoneFlag, 10 * RADIO_WAIT_TIMEOUT);
	if (transmitting_packet) {
		S2LP_CMD_StrobeSabort();
		if (xTxDoneFlag == RESET) {
			TRice("Packet not transmitted: TIMEOUT\n");
		} else {
			TRice("Packet not transmitted: ERROR\n");
		}
	} else {
		retval = tx_ok;
	}
	xTxDoneFlag = RESET;

#if RADIO_HW_CSMA
	if (csma_enabled) {
		S2LP_CSMA_Enable(S_DISABLE);
#if !RADIO_SNIFF_MODE
		/* Put the receive threshold back where Radio_init() set it. RSSI_THR is one register
		 * serving several unrelated jobs (ST bring-up guide p.50): the CSMA busy level while
		 * transmitting, and - once back in RX - the level at which AFC starts tracking the
		 * frequency offset. Leaving it at the tracked noise floor + a few dB, as this used
		 * to, silently raised the AFC trigger well above the sensitivity-derived value and
		 * stopped AFC from ever engaging on packets near the noise floor. The noise estimate
		 * still drives csma_tx_threshold, which is its legitimate consumer. */
		S2LP_RADIO_QI_SetRssiThreshdBm(RSSI_RX_THRESHOLD);
#endif /*!RADIO_SNIFF_MODE*/
	}
#endif /*RADIO_HW_CSMA*/

	Exit_TX();

	if (radio_off == radio_state) {
		/*If the radio was OFF before transmitting the packet, we must turn it OFF (legacy for ContikiMAC like upper layer) */
		Radio_off();
	}

	return retval;
}

static eTransmitRes Radio_send(sPacket *packet) {
	eTransmitRes res = tx_err;
	if (0 != operation_mode) {
		return tx_err;
	}
	transmitting_packet = 1;
	if (tx_ok != Radio_prepare(packet)) {
#if RADIO_SNIFF_MODE
    S2LP_TIM_LdcrMode(S_ENABLE);
    S2LP_TIM_FastRxTermTimer(S_ENABLE);
    S2LP_GPIO_IrqConfig(RX_DATA_READY,S_ENABLE);
#endif /*RADIO_SNIFF_MODE*/
		transmitting_packet = 0;
    	RadioSwitchToRx();
		TRice("msg:PREPARE FAILED\n");
		return tx_err;
	}
	res = Radio_transmit(packetbuf_totlen(packet));
	transmitting_packet = 0;
	return res;
}

static int16_t Radio_read(sPacket *packet) {
	int16_t retval = 0;
	retval = Radio_read_from_fifo(packet);
	pending_packet = 0;
	/* RX command - to ensure the device will be ready for the next reception */
#if RADIO_SNIFF_MODE
      S2LP_CMD_StrobeSleep();
#else /*!RADIO_SNIFF_MODE*/
    /* AFC freezes on sync (AFC_FREEZE_ON_SYNC, set by S2LP_RADIO_Init) and only re-acquires
     * when the receiver re-enters RX. This node listens continuously - RX persistent mode
     * with an infinite RX timeout - so the part never leaves RX on its own and a bare
     * StrobeRx here is a no-op: the correction stays pinned to whichever neighbour was
     * heard last. ST bring-up guide p.31 asks for AFC to be reset every time Rx restarts.
     * Abort first so the strobe below is a genuine re-entry. The FIFO is already drained
     * and flushed by Radio_read_from_fifo(), and the deaf window is two SPI transactions,
     * so nothing is lost by cycling here rather than staying nominally in RX. */
    S2LP_CMD_StrobeSabort();
    RadioSwitchToRx();
#endif /*RADIO_SNIFF_MODE*/
	rx_num_bytes = 0;
	return retval;
}

static int8_t Radio_channel_clear(void) {
	int32_t rssiRaw;
	int32_t rssiSum;
	int16_t rssiAvgdBm;
	int8_t  ret;
	const uint8_t *sample;
	/* Local variable used to memorize the S2LP state */
	eRadioStatus radio_state = radio_status;

	TRice("msg:CHANNEL CLEAR IN\n");

	if (radio_off == radio_status) {
		/* Wakes up the Radio */
		Radio_on();
	}
	/* Despite its name S2LPRadioGetRssidBmRun() returns the raw 4-byte burst read of
	 * RSSI_LEVEL_RUN with no dBm conversion applied. That register deliberately does not
	 * auto-increment on a burst (DS11896 5.5.8.1: "the same as SPI burst mode, but no
	 * automatic address increment"), so the word carries four consecutive RSSI samples, each
	 * a raw 0..255 step where 0 means -146 dBm. Comparing the packed word straight against a
	 * negative dBm threshold made this test permanently false, i.e. the channel always
	 * reported busy. Average the four samples and convert, as Radio_read_from_fifo() does. */
	rssiRaw = S2LP_RADIO_QI_GetRssidBmRun();
	sample = (const uint8_t*)&rssiRaw;
	rssiSum = (int32_t)sample[0] + sample[1] + sample[2] + sample[3];
	rssiAvgdBm = (int16_t)((rssiSum / 4) - 146);
	ret = (rssiAvgdBm < csma_tx_threshold) ? 1 : 0;

	/* Puts the S2LP in its previous state */
	if (radio_off == radio_state) {
		Radio_off();
	}

	return ret;
}

static int8_t Radio_transmitting_packet(void) {
	return transmitting_packet;
}

static int8_t Radio_receiving_packet(void) {
	return receiving_packet;
}

static int8_t Radio_pending_packet(void) {
	return pending_packet;
}

static eRadioRes Radio_get_value(radio_param_t parameter, radio_value_t *ret_value) {
	eRadioRes get_value_result;
	get_value_result = radio_notSupported;

	if (ret_value == NULL) {
		return radio_invalidArgument;
	}
	switch (parameter) {
	case RADIO_PARAM_POWER_MODE:
		if (radio_on == radio_status) {
			*ret_value = RADIO_POWER_MODE_ON;
		} else {
			*ret_value = RADIO_POWER_MODE_OFF;
		}
		get_value_result = radio_ok;
		break;
	case RADIO_PARAM_CHANNEL:
		*ret_value = radio_get_channel();
		get_value_result = radio_ok;
		break;
	case RADIO_PARAM_RX_MODE:
		*ret_value = 0x00;
		if (radio_send_auto_ack) {
			*ret_value |= RADIO_RX_MODE_AUTOACK;
		}
		if (auto_pkt_filter) {
			*ret_value |= RADIO_RX_MODE_ADDRESS_FILTER;
		}
		get_value_result = radio_ok;
		break;
	case RADIO_PARAM_TX_MODE:
		*ret_value = 0x00;
		if (csma_enabled) {
			*ret_value |= RADIO_TX_MODE_SEND_ON_CCA;
		}
		get_value_result = radio_ok;
		break;
	case RADIO_PARAM_TXPOWER:
		*ret_value = radio_get_txpower();
		get_value_result = radio_ok;
		break;
	case RADIO_PARAM_RSSI:
		*ret_value = S2LP_RADIO_QI_GetRssidBm();
		get_value_result = radio_ok;
		break;
	case RADIO_PARAM_LAST_RSSI:
		*ret_value = last_packet_rssi;
		get_value_result = radio_ok;
		break;
	case RADIO_PARAM_CCA_THRESHOLD:
		*ret_value = csma_tx_threshold;
		get_value_result = radio_ok;
		break;
	case RADIO_CONST_CHANNEL_MIN:
		*ret_value = CHANNEL_NUMBER_MIN;
		get_value_result = radio_ok;
		break;
	case RADIO_CONST_CHANNEL_MAX:
		*ret_value = CHANNEL_NUMBER_MAX;
		get_value_result = radio_ok;
		break;
	case RADIO_CONST_TXPOWER_MIN:
		*ret_value = RADIO_POWER_DBM_MIN;
		get_value_result = radio_ok;
		break;
	case RADIO_CONST_TXPOWER_MAX:
		*ret_value = RADIO_POWER_DBM_MAX;
		get_value_result = radio_ok;
		break;
	case RADIO_OPERATION_MODE:
		*ret_value = operation_mode;
		get_value_result = radio_ok;
		break;
	case RADIO_PARAM_MAX_BACKOFF_NR:
	{
		*ret_value = xCsmaInit.cMaxNb;
		get_value_result = radio_ok;
		break;
	}
	case RADIO_CONST_MAX_PAYLOAD_LEN:
		/*TODO: check if this value is correct.*/
		*ret_value = MAX_PACKET_LEN;
		get_value_result = radio_ok;
		break;
	default:
		TRice("dbg:Radio_get_value(%d) - radio_notSupported.\n", parameter);
	}

	return get_value_result;
}

static void EnterOperationMode(uint8_t mode) {
	static uint8_t backup_mod2;
	static uint8_t backup_pcktctrl1;
	if ((0 != operation_mode) && (0 == mode)) { // exit test mode
		TRice("msg:[RADIO] - exit test mode\n");
		S2LP_CMD_StrobeSabort();
		radio_set_ready_state();
		S2LPSpiWriteRegisters(MOD2_ADDR, 1, &backup_mod2);
		S2LPSpiWriteRegisters(PCKTCTRL1_ADDR, 1, &backup_pcktctrl1);
		operation_mode = mode;
		Exit_TX();
	} else if ((0 == operation_mode) && (0 != mode)) { // enter test mode - only from normal mode
		uint8_t dummy;
		S2LPSpiReadRegisters(MOD2_ADDR, 1, &backup_mod2);
		S2LPSpiReadRegisters(PCKTCTRL1_ADDR, 1, &backup_pcktctrl1);
		switch (mode) {
		case 1:
			/* Continuous carrier. ST bring-up guide p.42 asks for both halves: MOD_TYPE = 7
			 * (unmodulated) in MOD2 *and* TX_SOURCE = 3 (PN9) in PCKTCTRL1. The PCKTCTRL1
			 * write used to be commented out, which left TX_SOURCE in normal mode - the
			 * StrobeTx below then transmitted from an empty FIFO instead of emitting a tone,
			 * so the mode was unusable for certification. Only MOD_TYPE is touched in MOD2 so
			 * the configured datarate exponent survives. */
			TRice("msg:[RADIO] - enter CW test mode\n");
			radio_set_ready_state();
			dummy = (uint8_t)((backup_mod2 & (uint8_t)(~MOD_TYPE_REGMASK)) | (0x7 << 4));
			S2LPSpiWriteRegisters(MOD2_ADDR, 1, &dummy);
			dummy = (uint8_t)((backup_pcktctrl1 & (uint8_t)(~TXSOURCE_REGMASK)) | (0x3 << 2));
			S2LPSpiWriteRegisters(PCKTCTRL1_ADDR, 1, &dummy);
			break;
		case 2:
			/* PN9: keep the application modulation, switch only the Tx source. Clear the
			 * field before setting it - PCKTCTRL1 resets to 0x2C, which already has TX_SOURCE
			 * set, so a bare OR cannot be relied on to produce the intended value. */
			TRice("msg:[RADIO] - enter PN9 test mode\n");
			radio_set_ready_state();
			dummy = (uint8_t)((backup_pcktctrl1 & (uint8_t)(~TXSOURCE_REGMASK)) | (0x3 << 2));
			S2LPSpiWriteRegisters(PCKTCTRL1_ADDR, 1, &dummy);
			break;
		default:
			return;
		}
		//S2LP_PCKT_BASIC_SetPayloadLength(0xFFFF);
		S2LP_CMD_StrobeTx();
		operation_mode = mode;
	}
}

static eRadioRes Radio_set_value(radio_param_t parameter, radio_value_t input_value) {
	eRadioRes set_value_result;
	set_value_result = radio_notSupported;

	if (parameter == RADIO_PARAM_POWER_MODE) {
		switch (input_value) {
		case RADIO_POWER_MODE_ON:
			Radio_on();
			set_value_result = radio_ok;
			break;
		case RADIO_POWER_MODE_OFF:
			Radio_off();
			set_value_result = radio_ok;
			break;
		default:
			set_value_result = radio_invalidArgument;
			break;
		}
	} else if (parameter == RADIO_PARAM_CHANNEL) {
		if ((input_value >= CHANNEL_NUMBER_MIN) && (input_value <= CHANNEL_NUMBER_MAX)) {
			radio_set_channel(input_value);
			set_value_result = radio_ok;
		} else {
			set_value_result = radio_invalidArgument;
		}
	} else if (parameter == RADIO_PARAM_RX_MODE) {
		radio_value_t valid = (RADIO_RX_MODE_ADDRESS_FILTER | RADIO_RX_MODE_AUTOACK | RADIO_RX_MODE_POLL_MODE);
		if (input_value & (~valid)) {
			set_value_result = radio_invalidArgument;
		} else {
			radio_set_auto_pkt_filter((input_value & RADIO_RX_MODE_ADDRESS_FILTER) != 0);
			radio_set_auto_ack((input_value & RADIO_RX_MODE_AUTOACK) != 0);
			set_value_result = radio_ok;
		}
	} else if (parameter == RADIO_PARAM_TX_MODE) {
		radio_value_t valid = RADIO_TX_MODE_SEND_ON_CCA;
		if (input_value & (~valid)) {
			set_value_result = radio_invalidArgument;
		} else {
			radio_set_csma((input_value & RADIO_TX_MODE_SEND_ON_CCA) != 0);
			set_value_result = radio_ok;
		}
	} else if (parameter == RADIO_PARAM_TXPOWER) {
		if ((input_value >= RADIO_POWER_DBM_MIN) && (input_value <= RADIO_POWER_DBM_MAX)) {
			radio_set_txpower(input_value);
			set_value_result = radio_ok;
		} else {
			set_value_result = radio_invalidArgument;
		}
	} else if (parameter == RADIO_PARAM_CCA_THRESHOLD) {
		csma_tx_threshold = input_value;
		set_value_result = radio_ok;
	} else if (parameter == RADIO_OPERATION_MODE) {
		EnterOperationMode(input_value);
		set_value_result = radio_ok;
	} else if (RADIO_PARAM_MAX_BACKOFF_NR == parameter) {
		if (8 > input_value) {
			xCsmaInit.cMaxNb = input_value;
			S2LPCsmaSetMaxNumberBackoff(xCsmaInit.cMaxNb);
			set_value_result = radio_ok;
		} else {
			set_value_result = radio_invalidArgument;
		}
	}

	return set_value_result;
}

static eRadioRes Radio_get_object(radio_param_t parameter, void *destination, size_t size) {
	eRadioRes get_object_retval;
	get_object_retval = radio_notSupported;

	/*@TODO: add other parameters. */
	if (parameter == RADIO_PARAM_LAST_PACKET_TIMESTAMP) {
		if ((size == sizeof(uint32_t)) && (destination != NULL)) {
			/*@TODO: this has to be validated*/
			*(uint32_t*) destination = radio_get_packet_timestamp();
			get_object_retval = radio_ok;
		} else {
			get_object_retval = radio_invalidArgument;
		}
	}
	return get_object_retval;
}

static eRadioRes Radio_set_object(radio_param_t parameter, const void *source, size_t size) {
	UNUSED(parameter);
	UNUSED(source);
	UNUSED(size);
	/*@TODO: this API is currently not supported. */

	return radio_notSupported;
}

const struct radio_driver subGHz_radio_driver = {
		Radio_init,
		Radio_prepare,
		Radio_transmit,
		Radio_send,
		Radio_read,
		Radio_channel_clear,
		Radio_receiving_packet,
		Radio_transmitting_packet,
		Radio_pending_packet,
		Radio_on,
		Radio_off,
		Radio_get_value,
		Radio_set_value,
		Radio_get_object,
		Radio_set_object };

/* Functions ----------------------------------------------------------------*/
/**
 * @brief  Radio_process_irq_cb callback when an interrupt is received
 */
void Radio_process_irq_cb(void) {
	S2LPIrqs x_irq_status;

	/* get interrupt source from radio */
	S2LP_GPIO_IrqGetStatus(&x_irq_status);

	/* Change 6: FIFO error handling - abort and flush per PDF p.48 */
	if (x_irq_status.IRQ_TX_FIFO_ERROR) {
		TRice("err:[RADIO DRV] - tx FIFO error.\n");
		S2LP_CMD_StrobeSabort();
		xTxDoneFlag = SET;
		receiving_packet = 0;
		radioEvtHndl(radioEvtIdOffset + radio_txFifoErr, HandleTxFifoError);
		return;
	}

	/* The IRQ_TX_DATA_SENT notifies the packet transmission.
	 * Then puts the Radio in RX/Sleep according to the selected mode */
	if (x_irq_status.IRQ_TX_DATA_SENT && transmitting_packet) {
		transmitting_packet = 0;
		xTxDoneFlag = SET;
		return;
	}

#if !RADIO_SNIFF_MODE
	/* The IRQ_VALID_SYNC is used to notify a new packet is coming */
	if (x_irq_status.IRQ_VALID_SYNC && !transmitting_packet) {
		receiving_packet = 1;
		//S2LP_CMD_StrobeRx();
	}
#endif /*RADIO_SNIFF_MODE*/

#if RADIO_HW_CSMA
	if (x_irq_status.IRQ_MAX_BO_CCA_REACH) {
		/* ST bring-up guide p.24: this IRQ carries two opposite meanings. Either the channel
		 * was still busy on the last CSMA slot and the transmission was cancelled, or the
		 * channel was clear, the part transmitted anyway, and TX_DATA_SENT will follow at
		 * the end of the frame. Re-strobing TX unconditionally wrecks the second case by
		 * restarting a transmission that is already on air. The guide's disambiguation is to
		 * read RSSI_LEVEL here and anticipate what the part decided. */
		if (S2LP_RADIO_QI_GetRssidBm() < csma_tx_threshold) {
			/* Channel was clear - the frame is going out, wait for TX_DATA_SENT. */
			TRice("dbg:IRQ_MAX_BO_CCA_REACH - channel clear, tx already started\n");
		} else if (csma_tx_restarts < CSMA_MAX_TX_RESTARTS) {
			csma_tx_restarts++;
			TRice("dbg:IRQ_MAX_BO_CCA_REACH - channel busy, restart %d\n", csma_tx_restarts);
			S2LP_CMD_StrobeTx();
		} else {
			/* Out of restarts. Stop the engine but leave transmitting_packet set so
			 * Radio_transmit() falls through its timeout and reports tx_collision rather
			 * than mistaking an abandoned send for a delivered one. */
			TRice("wrn:IRQ_MAX_BO_CCA_REACH - channel busy, giving up\n");
			S2LP_CMD_StrobeSabort();
		}
		return;
	}
#endif /*RADIO_HW_CSMA*/

	/* The IRQ_RX_DATA_READY notifies a new packet arrived */
	if (x_irq_status.IRQ_RX_DATA_READY && !(transmitting_packet)) {
		receiving_packet = 0;

		pending_packet = 1;
		if (NULL == overridenRxCb) {
			radioEvtHndl(radioEvtIdOffset + radio_incomingData, NULL);
		} else {
			overridenRxCb();
			overridenRxCb = NULL;
		}
		return;
	}

#if !RADIO_SNIFF_MODE
	if (x_irq_status.IRQ_RX_DATA_DISC && !transmitting_packet) {
		x_irq_status.IRQ_RX_DATA_DISC = 0;
		transmitting_packet = 0;
		receiving_packet = 0;
		if (x_irq_status.IRQ_RX_TIMEOUT) {
			TRice("\t IRQ_RX_TIMEOUT\n");
			x_irq_status.IRQ_RX_TIMEOUT = 0;
		}
		if (x_irq_status.IRQ_CRC_ERROR) {
			TRice("\t IRQ_CRC_ERROR\n");
			x_irq_status.IRQ_CRC_ERROR = 0;
		}
		if (x_irq_status.IRQ_TX_FIFO_ERROR) {
			TRice("\t IRQ_TX_FIFO_ERROR\n");
			x_irq_status.IRQ_TX_FIFO_ERROR = 0;
		}
		if (x_irq_status.IRQ_RX_FIFO_ALMOST_FULL) {
			TRice("\t IRQ_RX_FIFO_ALMOST_FULL(%d)\n", S2LP_FIFO_ReadNumberBytesRxFifo());
			x_irq_status.IRQ_RX_FIFO_ALMOST_FULL = 0;
		}
		if (x_irq_status.IRQ_RSSI_ABOVE_TH) {
			TRice("\t IRQ_RSSI_ABOVE_TH\n");
			x_irq_status.IRQ_RSSI_ABOVE_TH = 0;
		}
		if (x_irq_status.IRQ_RX_START_TIME) {
			TRice("\t IRQ_RX_START_TIME\n");
			x_irq_status.IRQ_RX_START_TIME = 0;
		}
		if (x_irq_status.IRQ_RX_FIFO_ERROR) {
			TRice("\t IRQ_RX_FIFO_ERROR\n");
			x_irq_status.IRQ_RX_FIFO_ERROR = 0;
			radioEvtHndl(radioEvtIdOffset + radio_rxFifoErr, HandleRxFifoError);
		} else {
			radioEvtHndl(radioEvtIdOffset + radio_rxDiscarded, HandleRxError);
		}
		if (*(uint32_t*)&x_irq_status) {
			uint32_t irqReg = *(uint32_t*)&x_irq_status;
			TRice("\t IRQ_RX_DATA_DISC[0x%08X]\n", irqReg);
		}
	}
#endif /*!RADIO_SNIFF_MODE*/
}

void RadioOverrideRxCb(void (*overRxCb)(void)) {
	overridenRxCb = overRxCb;
}
