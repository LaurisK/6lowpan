/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "radio-driver.h"
#include "packetbuf.h"
#include "main.h"
#include "s2lp_interface.h"
#include "s2lp_management.h"
#include "S2LP_Types.h"
#include "S2LP_PktBasic.h"
#if defined(STM32H753xx)
#include "trice.h"
#else
#include "App/common.h"
#endif

/* Private defines ----------------------------------------------------------*/
#if RADIO_ADDRESS_FILTERING
#define ACK_LEN 3
#endif /*RADIO_ADDRESS_FILTERING*/

#define RADIO_WAIT_TIMEOUT (100)
#define BUSYWAIT_UNTIL(cond, max_time)          \
do {                                            \
  uint32_t t0;                                  \
  t0 = HAL_GetTick();                           \
  while(!(cond) && TICK_TMO(t0, (max_time)));   \
} while(0)

#if RADIO_HW_CSMA
#define PERSISTENT_MODE_EN              S_DISABLE
#define CS_PERIOD                       CSMA_PERIOD_64TBIT
#define CS_TIMEOUT                      3
#define MAX_NB                          5
#define BU_COUNTER_SEED                 0xFA21
#define CU_PRESCALER                    32
#endif /*RADIO_ADDRESS_FILTERING*/

/* Private types ------------------------------------------------------------*/
typedef struct {
	uint8_t operatingChannel;
} sRadioInfo;

typedef enum {
  radio_off,
  radio_on
} eRadioStatus;

/* Private functions prototypes for Radio API -------------------------------*/
static int8_t Radio_init(void);
static eTransmitRes Radio_prepare(const void *payload, uint16_t payload_len);
static eTransmitRes Radio_transmit(uint16_t payload_len);
static eTransmitRes Radio_send(const void *data, uint16_t len);
static int16_t Radio_read(void *buf, uint16_t bufsize);
static int8_t Radio_channel_clear(void);
static int8_t Radio_receiving_packet(void);
static int8_t Radio_pending_packet(void);
static int8_t Radio_on(void);
static int8_t Radio_off(void);
static eRadioRes Radio_get_value(radio_param_t parameter, radio_value_t *ret_value);
static eRadioRes Radio_set_value(radio_param_t parameter, radio_value_t input_value);
static eRadioRes Radio_get_object(radio_param_t parameter, void *destination, size_t size);
static eRadioRes Radio_set_object(radio_param_t parameter, const void *source, size_t size);

/* Pseudo global variables --------------------------------------------------*/
static sRadioInfo radioInfo = {.operatingChannel = CHANNEL_NUMBER};
/* The buffer which holds incoming data. */
static uint16_t rx_num_bytes = 0;
static uint8_t radio_rxbuf[MAX_PACKET_LEN];

static volatile eRadioStatus radio_status = radio_off;
static volatile uint8_t receiving_packet = 0;
static volatile uint8_t transmitting_packet = 0;
static volatile uint8_t pending_packet = 0;
static volatile uint8_t packet_is_prepared = 0;
static radio_value_t last_packet_rssi = 0;
static packetbuf_attr_t last_packet_lqi = 0;
static uint8_t pending_process = 0;

static volatile uint32_t last_packet_timestamp = 0;

static int csma_tx_threshold = RSSI_TX_THRESHOLD;

/* Poll mode disabled by default */
/*static*/uint8_t polling_mode = 0;

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

const struct radio_driver subGHz_radio_driver = {
		Radio_init,
		Radio_prepare,
		Radio_transmit,
		Radio_send,
		Radio_read,
		Radio_channel_clear,
		Radio_receiving_packet,
		Radio_pending_packet,
		Radio_on,
		Radio_off,
		Radio_get_value,
		Radio_set_value,
		Radio_get_object,
		Radio_set_object };

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
static int16_t Radio_read_from_fifo(uint8_t *buf, uint16_t bufSize) {
	uint8_t rx_bytes, retval = 0;

	rx_bytes = S2LP_FIFO_ReadNumberBytesRxFifo();

	if (rx_bytes <= bufSize) {
		S2LP_ReadFIFO(rx_bytes, (uint8_t*) buf);
		retval = rx_bytes;
		last_packet_timestamp = HAL_GetTick(); //@TODO: validate
		last_packet_rssi = (radio_value_t) S2LP_RADIO_QI_GetRssidBm();
		//last_packet_lqi  = (packetbuf_attr_t) S2LP_RADIO_QI_GetLqi();
		packetbuf_set_attr(PACKETBUF_ATTR_RSSI,
				(packetbuf_attr_t) last_packet_rssi);
		packetbuf_set_attr(PACKETBUF_ATTR_LINK_QUALITY, last_packet_lqi);
	} else {
		TRice("msg:Buf too small (%d bytes to hold %d bytes)\n", bufSize, rx_bytes);
	}
	if (polling_mode) {
		S2LP_CMD_StrobeFlushRxFifo();
	}

	return retval;
}
/**
 * @brief radio_set_polling_mode is for control of pooling mode.
 * @param buf     - pointer to buffer where data needs to be stored
 */
static void radio_set_polling_mode(uint8_t enable) {
	/* Polling Mode  must be fully validated. */
	TRiceS("msg:POLLING MODE is %s.\r\n", enable?"ENABLED":"DISABLED");
	polling_mode = enable;
	if (polling_mode) {
		/* Disable interrupts */
		S2LP_GPIO_IrqConfig(RX_DATA_READY, S_DISABLE);
		S2LP_GPIO_IrqConfig(TX_DATA_SENT, S_DISABLE);
		S2LP_GPIO_IrqConfig(VALID_SYNC, S_DISABLE);
	} else {
		/* Initialize and enable interrupts */
		S2LP_GPIO_IrqConfig(RX_DATA_READY, S_ENABLE);
		S2LP_GPIO_IrqConfig(TX_DATA_SENT, S_ENABLE);
		S2LP_GPIO_IrqConfig(VALID_SYNC, S_ENABLE);
	}
}

/**
 * @brief  radio_print_status prints to the UART the status of the radio
 */
static void radio_print_status(void) {
	S2LPState s = radio_refresh_status();
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
	TRice("msg:READY IN\n");

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

	S2LP_CMD_StrobeCommand(CMD_FLUSHRXFIFO);
	receiving_packet = 0;
	pending_packet = 0;
	rx_num_bytes = 0;

	S2LP_GPIO_IrqClearStatus();
	RADIO_IRQ_ENABLE();

	//radio_print_status() ;
	TRice("msg:READY OUT\n");
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

/* Radio Driver API Functions -----------------------------------------------*/
static int8_t Radio_init(void) {
	TRice("msg:RADIO INIT IN\n");

	S2LPInterfaceInit();

	/* Configures the Radio library */
	S2LP_RADIO_SetXtalFrequency(XTAL_FREQUENCY);

	S2LP_CMD_StrobeSres();

	/* S2LP Radio config */
	S2LP_RADIO_Init(&xRadioInit);

	S2LP_RADIO_SetChannel(CHANNEL_NUMBER);
	S2LP_RADIO_SetChannelSpace(CHANNEL_SPACE);

	if (!S2LP_ManagementGetRangeExtender()) //Also check similar code in s2lp_interface.c
	{
		/* if we haven't an external PA, use the library function */
		S2LP_RADIO_SetPALeveldBm(POWER_INDEX, POWER_DBM);
	} else {
		/* in case we are using the PA board, the S2LP_RADIO_SetPALeveldBm will be not functioning because the output power is affected by the amplification
		 of this external component. Set the raw register. */
		uint8_t paLevelValue = 0x25; /* for example, this value will give 23dBm about */
		S2LP_WriteRegister(PA_POWER8_ADDR, 1, &paLevelValue);
	}
	S2LP_RADIO_SetPALevelMaxIndex(POWER_INDEX);

	/* Configures the Radio packet handler part*/
	S2LP_PCKT_BASIC_Init(&xBasicInit);

#if RADIO_ADDRESS_FILTERING
	S2LP_PCKT_HNDL_SetAutoPcktFilter(S_ENABLE);
	S2LP_PCKT_HNDL_SelectSecondarySync(S_DISABLE);
	xAddressInit.cMyAddress = linkaddr_node_addr.u8[LINKADDR_SIZE - 1];
	S2LP_PCKT_BASIC_AddressesInit(&xAddressInit); TRice("msg:Node Source address %2X\n", xAddressInit.cMyAddress);
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

	radio_set_polling_mode(polling_mode);

	/* This is ok for normal or SNIFF (RX command triggers the LDC in fast RX termination mode) */
	S2LP_CMD_StrobeRx();
	radio_status = radio_on;

	TRice("msg:Radio init done\n");
	return 0;
}

static eTransmitRes Radio_prepare(const void *payload, uint16_t payloadLen) {
	TRice("msg:Radio: prepare %u\n", payloadLen);
	uint8_t tmpbuff[PACKETBUF_SIZE]; //@TODO: see below
	packet_is_prepared = 0;

	/* Checks if the payload length is supported: actually this can't happen, by system design, but it is safer to have this for sanity check. */
	if (payloadLen > PACKETBUF_SIZE) {
		TRice("msg:Payload len too big (> %d), error.\n", PACKETBUF_SIZE);
		return tx_err;
	}

	/* Sets the length of the packet to send */
	RADIO_IRQ_DISABLE();

	radio_set_ready_state();
	if (radio_refresh_status() != MC_STATE_READY) {
		TRice("Set Ready State failed.\n");
		radio_print_status();
		S2LP_CMD_StrobeSabort();
#if RADIO_SNIFF_MODE
    S2LP_TIM_LdcrMode(S_ENABLE);
    S2LP_TIM_FastRxTermTimer(S_ENABLE);
    S2LP_GPIO_IrqConfig(RX_DATA_READY,S_ENABLE);
#endif /*RADIO_SNIFF_MODE*/
		S2LP_CMD_StrobeRx();

		RADIO_IRQ_ENABLE();
		return tx_err;
	}

#if RADIO_ADDRESS_FILTERING
	const linkaddr_t *addr;
	if (auto_pkt_filter) {
		if (payloadLen == ACK_LEN || packetbuf_holds_broadcast()) {
			TRice("msg:Preparing to send to broadcast (%02X) address\n", BROADCAST_ADDRESS);
			S2LP_PCKT_HNDL_SetRxSourceReferenceAddress(BROADCAST_ADDRESS);
		} else {
			addr = packetbuf_addr(PACKETBUF_ADDR_RECEIVER);
			TRice("msg:Preparing to send to %2X address\n", addr->u8[LINKADDR_SIZE-1]);
			S2LP_PCKT_HNDL_SetRxSourceReferenceAddress(
					addr->u8[LINKADDR_SIZE-1]);
		}
	}
#endif /*RADIO_ADDRESS_FILTERING*/

	S2LP_CMD_StrobeCommand(CMD_FLUSHTXFIFO);

	S2LP_PCKT_BASIC_SetPayloadLength(payloadLen);
	//@TODO change IO implementation to avoid the copy here
	memcpy(tmpbuff, payload, payloadLen);

	/* Currently does no happen since S2LP_RX_FIFO_SIZE == MAX_PACKET_LEN also note that S2LP_RX_FIFO_SIZE == S2LP_TX_FIFO_SIZE */
	if (payloadLen > S2LP_TX_FIFO_SIZE) {
		TRice("msg:Payload bigger than FIFO size.'n");
	} else {
		S2LP_WriteFIFO(payloadLen, (uint8_t*) tmpbuff);
//    S2LP_WriteFIFO(payload_len, (uint8_t *)payload);
		packet_is_prepared = 1;
	}

	RADIO_IRQ_ENABLE();

	TRice("msg:PREPARE OUT\n");

	return tx_ok;
}

static eTransmitRes Radio_transmit(uint16_t payloadLen) {
	int retval = tx_err;
	eRadioStatus radio_state = radio_status;

	/* This function blocks until the packet has been transmitted */
	TRice("msg:TRANSMIT IN\n");
	if (!packet_is_prepared) {
		TRice("msg:Radio TRANSMIT: ERROR, packet is NOT prepared.\n");
		return tx_err;
	}

	if (radio_off == radio_status) {
		Radio_on();
	}

	RADIO_IRQ_DISABLE();

	transmitting_packet = 1;
	S2LP_GPIO_IrqClearStatus();
	RADIO_IRQ_ENABLE();

#if RADIO_HW_CSMA
	if (csma_enabled) { //@TODO: add an API to enable/disable CSMA
		S2LP_CSMA_Enable(S_ENABLE);
		S2LP_RADIO_QI_SetRssiThreshdBm(RSSI_TX_THRESHOLD);
		retval = tx_collision;
	}
#endif  /*RADIO_HW_CSMA*/

	xTxDoneFlag = RESET;
	S2LP_CMD_StrobeTx();
	/* wait for TX done */
	if (polling_mode) { //@TODO: To be validated
		while (!xTxDoneFlag) {
			uint8_t tmp;
			BUSYWAIT_UNTIL(0, 2);
			S2LP_ReadRegister(TX_FIFO_STATUS_ADDR, 1, &tmp);
			if ((tmp & NELEM_TXFIFO_REGMASK) == 0) {
				xTxDoneFlag = SET;
				transmitting_packet = 0;
			}
		}
		BUSYWAIT_UNTIL(0, 2); //@TODO: we need a delay here, validate.
	} else {
		/*To be on the safe side we put a timeout. */
		BUSYWAIT_UNTIL(xTxDoneFlag, 10* RADIO_WAIT_TIMEOUT);
	}
	if (transmitting_packet) {
		S2LP_CMD_StrobeSabort();
		if (xTxDoneFlag == RESET) {
			TRice("Packet not transmitted: TIMEOUT\n\r");
		} else {
			TRice("Packet not transmitted: ERROR\n\r");
		}
		transmitting_packet = 0;
	} else {
		retval = tx_ok;
	}
	xTxDoneFlag = RESET;

#if RADIO_HW_CSMA
	if (csma_enabled) {
		S2LP_CSMA_Enable(S_DISABLE);
#if !RADIO_SNIFF_MODE
		S2LP_RADIO_QI_SetRssiThreshdBm(RSSI_RX_THRESHOLD);
#endif /*!RADIO_SNIFF_MODE*/
	}
#endif /*RADIO_HW_CSMA*/

	rx_num_bytes = 0;

	RADIO_IRQ_DISABLE();

#if RADIO_SNIFF_MODE
  S2LP_TIM_LdcrMode(S_ENABLE);
  S2LP_TIM_FastRxTermTimer(S_ENABLE);
  S2LP_GPIO_IrqConfig(RX_DATA_READY,S_ENABLE);
#endif /*RADIO_SNIFF_MODE*/

	S2LP_CMD_StrobeRx();
	BUSYWAIT_UNTIL(radio_refresh_status() == MC_STATE_RX
#if RADIO_SNIFF_MODE
                 || radio_refresh_status() == MC_STATE_SLEEP_NOFIFO
#endif /*RADIO_SNIFF_MODE*/
			,RADIO_WAIT_TIMEOUT);

	packet_is_prepared = 0;

	S2LP_CMD_StrobeCommand(CMD_FLUSHTXFIFO);

	S2LP_GPIO_IrqClearStatus();
	RADIO_IRQ_ENABLE();

	TRice("msg:TRANSMIT OUT\n");

	if (radio_off == radio_state) {
		/*If the radio was OFF before transmitting the packet, we must turn it OFF (legacy for ContikiMAC like upper layer) */
		Radio_off();
	}

	return retval;
}

static eTransmitRes Radio_send(const void *payload, uint16_t payloadLen) {
	TRice("msg:Radio Send\r\n");

	if (tx_ok != Radio_prepare(payload, payloadLen)) {
#if RADIO_SNIFF_MODE
    S2LP_TIM_LdcrMode(S_ENABLE);
    S2LP_TIM_FastRxTermTimer(S_ENABLE);
    S2LP_GPIO_IrqConfig(RX_DATA_READY,S_ENABLE);
#endif /*RADIO_SNIFF_MODE*/
		S2LP_CMD_StrobeRx(); TRice("msg:PREPARE FAILED\n");
		return tx_err;
	}
	return Radio_transmit(payloadLen);
}

static int16_t Radio_read(void *buf, uint16_t bufSize) {
	int16_t retval = 0;
	S2LPIrqs x_irq_status;

	if (polling_mode) {
		S2LP_GPIO_IrqGetStatus(&x_irq_status);
		if (x_irq_status.IRQ_RX_DATA_READY)
			retval = Radio_read_from_fifo(buf, bufSize);
	} else if (pending_packet && (rx_num_bytes != 0)) {
		if (rx_num_bytes <= bufSize) {
			memcpy(buf, radio_rxbuf, rx_num_bytes);
			retval = rx_num_bytes;
		} else {
			TRice("msg:Buf too small (%d bytes to hold %u bytes)\n", bufSize, rx_num_bytes);
		}
		pending_packet = 0;
	}
	/* RX command - to ensure the device will be ready for the next reception */
#if RADIO_SNIFF_MODE
      S2LP_CMD_StrobeSleep();
#else /*!RADIO_SNIFF_MODE*/
	S2LP_CMD_StrobeRx();
#endif /*RADIO_SNIFF_MODE*/
	rx_num_bytes = 0;
	TRice("msg:READ OUT: %d\n", retval);
	return retval;
}

static int8_t Radio_channel_clear(void) {
	float rssi_value;
	/* Local variable used to memorize the S2LP state */
	eRadioStatus radio_state = radio_status;

	TRice("msg:CHANNEL CLEAR IN\n");

	if (radio_off == radio_status) {
		/* Wakes up the Radio */
		Radio_on();
	}
	rssi_value = S2LP_RADIO_QI_GetRssidBmRun();
	int ret = (rssi_value < RSSI_TX_THRESHOLD) ? 1 : 0;

	/* Puts the S2LP in its previous state */
	if (radio_off == radio_state) {
		Radio_off();
	}

	return ret;
}

static int8_t Radio_receiving_packet(void) {
	S2LPIrqs x_irq_status;

	if (polling_mode) {
		S2LP_GPIO_IrqGetStatus(&x_irq_status);
		receiving_packet = (x_irq_status.IRQ_VALID_SYNC);
	}

	return receiving_packet;
}

static int8_t Radio_pending_packet(void) {
	S2LPIrqs x_irq_status;
	TRice("msg:RADIO PENDING PACKET\n");
	if (polling_mode) {
		S2LP_GPIO_IrqGetStatus(&x_irq_status);
		pending_packet = (x_irq_status.IRQ_RX_DATA_READY);
	}

	return pending_packet;
}

static int8_t Radio_on(void) {
	TRice("msg:Radio: on\n");

	if (radio_off == radio_status) {
#if RADIO_SNIFF_MODE
    S2LP_TIM_LdcrMode(S_ENABLE);
    S2LP_TIM_FastRxTermTimer(S_ENABLE);
#endif /*RADIO_SNIFF_MODE*/
		radio_set_ready_state();
		S2LP_FIFO_MuxRxFifoIrqEnable(S_ENABLE);
		S2LP_CMD_StrobeRx();
		radio_status = radio_on;
		RADIO_IRQ_ENABLE(); //--> Coming from OFF, IRQ ARE DISABLED.
	}
	return 0;
}

static int8_t Radio_off(void) {
	TRice("msg:Radio: ->off\n");

	if (radio_on == radio_status) {
		/* Disables the mcu to get IRQ from the RADIO */
		RADIO_IRQ_DISABLE();  //Mind that it will be enabled only in the ON

#if RADIO_SNIFF_MODE
    S2LP_TIM_LdcrMode(S_DISABLE);
    S2LP_TIM_FastRxTermTimer(S_DISABLE);
    S2LP_CMD_StrobeReady();
    S2LP_CMD_StrobeRx();
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

	TRice("msg:Radio: off.\n");
	//radio_print_status();
	return 0;
}

static eRadioRes Radio_get_value(radio_param_t parameter,
		radio_value_t *ret_value) {
	eRadioRes get_value_result;
	get_value_result = radio_notSupported;

	if (ret_value == NULL) {
		return radio_invalidArgument;
	}

	if (parameter == RADIO_PARAM_POWER_MODE) {
		if (radio_on == radio_status) {
			*ret_value = RADIO_POWER_MODE_ON;
		} else {
			*ret_value = RADIO_POWER_MODE_OFF;
		}
		get_value_result = radio_ok;
	} else if (parameter == RADIO_PARAM_CHANNEL) {
		*ret_value = radio_get_channel();
		get_value_result = radio_ok;
	} else if (parameter == RADIO_PARAM_RX_MODE) {
		*ret_value = 0x00;
		if (polling_mode) {
			*ret_value |= RADIO_RX_MODE_POLL_MODE;
		}
		if (radio_send_auto_ack) {
			*ret_value |= RADIO_RX_MODE_AUTOACK;
		}
		if (auto_pkt_filter) {
			*ret_value |= RADIO_RX_MODE_ADDRESS_FILTER;
		}
		get_value_result = radio_ok;
	} else if (parameter == RADIO_PARAM_TX_MODE) {
		*ret_value = 0x00;
		if (csma_enabled) {
			*ret_value |= RADIO_TX_MODE_SEND_ON_CCA;
		}
		get_value_result = radio_ok;
	} else if (parameter == RADIO_PARAM_TXPOWER) {
		*ret_value = radio_get_txpower();
		get_value_result = radio_ok;
	} else if (parameter == RADIO_PARAM_RSSI) {
		*ret_value = S2LP_RADIO_QI_GetRssidBm();
		get_value_result = radio_ok;
	} else if (parameter == RADIO_PARAM_LAST_RSSI) {
		*ret_value = last_packet_rssi;
		get_value_result = radio_ok;
	} else if (parameter == RADIO_PARAM_CCA_THRESHOLD) {
		*ret_value = csma_tx_threshold;
		get_value_result = radio_ok;
	} else if (parameter == RADIO_CONST_CHANNEL_MIN) {
		*ret_value = CHANNEL_NUMBER_MIN;
		get_value_result = radio_ok;
	} else if (parameter == RADIO_CONST_CHANNEL_MAX) {
		*ret_value = CHANNEL_NUMBER_MAX;
		get_value_result = radio_ok;
	} else if (parameter == RADIO_CONST_TXPOWER_MIN) {
		*ret_value = RADIO_POWER_DBM_MIN;
		get_value_result = radio_ok;
	} else if (parameter == RADIO_CONST_TXPOWER_MAX) {
		*ret_value = RADIO_POWER_DBM_MAX;
		get_value_result = radio_ok;
	} else if (parameter == RADIO_CONST_MAX_PAYLOAD_LEN) {
		/*TODO: check if this value is correct.*/
		*ret_value = MAX_PACKET_LEN;
		get_value_result = radio_ok;
	}

	return get_value_result;
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
			radio_set_polling_mode((input_value & RADIO_RX_MODE_POLL_MODE) != 0);
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
		//TODO: this value is not currently taken into account, only RSSI_TX_THRESHOLD macro is used
		csma_tx_threshold = input_value;
		set_value_result = radio_ok;
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

/* Functions ----------------------------------------------------------------*/
/**
 * @brief  Radio_process_irq_cb callback when an interrupt is received
 */
void Radio_process_irq_cb(void) {
	S2LPIrqs x_irq_status;

	/* get interrupt source from radio */
	S2LP_GPIO_IrqGetStatus(&x_irq_status);

	/* The IRQ_TX_DATA_SENT notifies the packet transmission.
	 * Then puts the Radio in RX/Sleep according to the selected mode */
	if (x_irq_status.IRQ_TX_DATA_SENT && transmitting_packet) {
		TRice("dbg:IRQ_TX_DATA_SENT\n");
		transmitting_packet = 0;
		xTxDoneFlag = SET;
		return;
	}

#if !RADIO_SNIFF_MODE
	/* The IRQ_VALID_SYNC is used to notify a new packet is coming */
	if (x_irq_status.IRQ_VALID_SYNC && !transmitting_packet) {
		TRice("dbg:IRQ_VALID_SYNC\n");
		receiving_packet = 1;
		S2LP_CMD_StrobeRx();
	}
#endif /*RADIO_SNIFF_MODE*/

#if RADIO_HW_CSMA
	if (x_irq_status.IRQ_MAX_BO_CCA_REACH) {
		/* Send a Tx command: i.e. keep on trying */
		S2LP_CMD_StrobeTx();
		return;
	}
#endif /*RADIO_HW_CSMA*/

	/* The IRQ_RX_DATA_READY notifies a new packet arrived */
	if (x_irq_status.IRQ_RX_DATA_READY && !(transmitting_packet)) {
		receiving_packet = 0;

		(void) Radio_read_from_fifo(radio_rxbuf, sizeof(radio_rxbuf));
		rx_num_bytes = S2LP_PCKT_BASIC_GetReceivedPktLength();

		S2LP_CMD_StrobeFlushRxFifo();
		pending_packet = 1;
		pending_process = 1;
		return;
	}

#if !RADIO_SNIFF_MODE
	if (x_irq_status.IRQ_RX_DATA_DISC && !transmitting_packet) {
		TRice("dbg:IRQ_RX_DATA_DISC\r\n");
		/* RX command - to ensure the device will be ready for the next reception */
		if (x_irq_status.IRQ_RX_TIMEOUT) {
			S2LP_CMD_StrobeFlushRxFifo();
			S2LP_CMD_StrobeRx();
		}
	}
#endif /*!RADIO_SNIFF_MODE*/
}

/**
 * @brief function for checking if any processes for radio are pending to be handled
 * @retval returns 1 if Radio process is waiting to be handled
 */
uint8_t Radio_process_is_pending(void) {
	return pending_process;
}

/**
 * @brief function for handling radio processes
 */
void Radio_process(void) {
	int len;
	pending_process = 0;
	packetbuf_clear();
	len = Radio_read(packetbuf_dataptr(), PACKETBUF_SIZE);

	if (len > 0) {
		packetbuf_set_datalen(len);

		TRice("msg:Calling MAC.Input(%d)\n", len);
		TRice8B("%02X\n", packetbuf_dataptr(), len);
#warning "NETSTACK_MAC attaches here"
		//NETSTACK_MAC.input();
	}

	if (!(rx_num_bytes == 0)) {
		TRice("msg:After MAC.input, RX BUF is not empty.\r\n");
		pending_process = 1;
	}
}

