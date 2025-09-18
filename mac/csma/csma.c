/*
 * Copyright (c) 2010, Swedish Institute of Computer Science.
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
*         The 802.15.4 standard CSMA protocol (nonbeacon-enabled)
 * \author
 *         Adam Dunkels <adam@sics.se>
 *         Simon Duquennoy <simon.duquennoy@inria.fr>
 */

/* Includes -----------------------------------------------------------------*/
#include "cmsis_os.h"
#include "csma.h"
#include "mac-sequence.h"
#include "Middlewares/Third_Party/6lowpan/radio-driver.h"
#include "../framer/framer-802154.h"
#include "../llsec802154.h"
#if defined(STM32H753xx)
#include "trice.h"
#include "main.h"
#else
#include "App/common.h"
#endif
#include "Middlewares/Third_Party/6lowpan/evt_radio.h"

/* Private defines ----------------------------------------------------------*/
#define MAX_QUEUED_PACKETS 8

/* macMaxCSMABackoffs: Maximum number of backoffs in case of channel busy/collision. Range 0--5 */
#ifdef CSMA_CONF_MAX_BACKOFF
#define CSMA_MAX_BACKOFF CSMA_CONF_MAX_BACKOFF
#else
#define CSMA_MAX_BACKOFF 5
#endif

/* macMaxFrameRetries: Maximum number of re-transmissions attampts. Range 0--7 */
#ifdef CSMA_CONF_MAX_FRAME_RETRIES
#define CSMA_MAX_FRAME_RETRIES CSMA_CONF_MAX_FRAME_RETRIES
#else
#define CSMA_MAX_FRAME_RETRIES 7
#endif

#if (false == IS_POWER_OF_2(MAX_QUEUED_PACKETS))
#error "MAX_QUEUED_PACKETS must be power of two"
#endif

/* Private types ------------------------------------------------------------*/
typedef struct {
	sPacket        *packet;
	mac_callback_t sentCb;
	void           *cptr;
	uint8_t        max_transmissions;
}sTransmitInfo;

typedef struct sNeighbor {
	struct sNeighbor *next;
	uint32_t         queuedTransmits;
	sTransmitInfo    transmit[MAX_QUEUED_PACKETS];
	linkaddr_t       addr;
	uint8_t          transfAttempt;
} sNeighbor;

/* Private functions prototypes ---------------------------------------------*/
static void TransmitFromQueue(void);

/* Pseudo global variables --------------------------------------------------*/
static volatile sNeighbor *neighborList = NULL;
static sPacket ackPacket;
static uint16_t csmaEvtIdOffset;
void (*csmaIrq2Task)(uint16_t, void(*cbFunc)(void));
static volatile uint8_t radioDataReceived = 0;

/* Private functions --------------------------------------------------------*/
/**
 *
 */
static sNeighbor* CreateNeighbor(void) {
	sNeighbor *neighbor = pvPortMalloc(sizeof(sNeighbor));
	memset(neighbor, 0x00, sizeof(sNeighbor));
	return neighbor;
}

/**
 *
 */
static sNeighbor* GetNeighborForAddr(const linkaddr_t *addr) {
  sNeighbor *walker = (sNeighbor *)neighborList;
  while (NULL != walker) {
	  if (linkaddr_cmp(addr, &walker->addr)) {
		  break;
	  }
	  walker = walker->next;
  }
  if (NULL == walker) {
	  walker = CreateNeighbor();
	  linkaddr_copy(&walker->addr, addr);
	  walker->next = (sNeighbor *)neighborList;
	  neighborList = walker;
  }
  return walker;
}

/**
 *
 */
static uint8_t GetSeqNr(void) {
#warning "for now just hardcoded random random number."
	static uint8_t seqNr = 0xA5;
	seqNr++;
	/* PACKETBUF_ATTR_MAC_SEQNO cannot be zero, due to a pecuilarity in framer-802154.c. */
	if (0 == seqNr) {
		seqNr++;
	}
	return seqNr;
}

/**
 *
 */
static uint8_t GetQueueLenOfNeighbor(const sNeighbor *neighbor) {
  uint8_t i = MAX_QUEUED_PACKETS;
  uint8_t queueLen = 0;
  while (i) {
	  i--;
	  if ((1 << i) & neighbor->queuedTransmits) {
		  queueLen++;
	  }
  }
  return queueLen;
}

/**
 *
 */
static void KickTranferQueue(void) {
	TransmitFromQueue();
}

/**
 *
 */
static void HandleTransferEnd(uint8_t transfRes, uint8_t packetPos) {
	sPacket *packet = neighborList->transmit[packetPos].packet;
	if(NULL == packet) {
		TRice("err:packet sent: missing packet.\n");
	    return;
	}
	neighborList->transfAttempt++;
	if ((MAC_TX_OK == transfRes) || (neighborList->transmit[packetPos].max_transmissions <= neighborList->transfAttempt)) {
		//notify caller of completed transfer.
		mac_call_sent_callback(neighborList->transmit[packetPos].sentCb, neighborList->transmit[packetPos].cptr, transfRes, neighborList->transfAttempt, packet);
		//remove packet from queue
		neighborList->queuedTransmits &= ~(1 << packetPos);
		neighborList->transfAttempt = 0;
		if (0 == neighborList->queuedTransmits) {
			sNeighbor *completedNeighbor = (sNeighbor *)neighborList;
			neighborList = neighborList->next;
			vPortFree(completedNeighbor);
		} else if (NULL != neighborList->next) {
			//this neighbor is served and others are in list pending - so this one need to be moved to end of a list
			sNeighbor *walker = (sNeighbor *)neighborList;
			while (NULL != walker->next) {
				walker = walker->next;
			}
			walker->next = (sNeighbor *)neighborList;
			neighborList = neighborList->next;
			walker->next->next = NULL;
		}
	}
	if (NULL != neighborList) {
		csmaIrq2Task(csmaEvtIdOffset + radio_taskCall, KickTranferQueue);
	}
}

static uint8_t FormPayload(sPacket *packet) {
  uint8_t seqNr = GetSeqNr();
  packetbuf_set_attr(packet, PACKETBUF_ATTR_MAC_SEQNO, seqNr);
  packetbuf_set_attr(packet, PACKETBUF_ATTR_FRAME_TYPE, FRAME802154_DATAFRAME);
  packetbuf_set_addr(packet, PACKETBUF_ADDR_SENDER, &linkaddr_node_addr);
  packetbuf_set_attr(packet, PACKETBUF_ATTR_MAC_ACK, 1);
#if LLSEC802154_ENABLED
#if LLSEC802154_USES_EXPLICIT_KEYS
	/* This should possibly be taken from upper layers in the future */
  packetbuf_set_attr(packet, PACKETBUF_ATTR_KEY_ID_MODE, CSMA_LLSEC_KEY_ID_MODE);
#endif /* LLSEC802154_USES_EXPLICIT_KEYS */
#endif /* LLSEC802154_ENABLED */
#warning "for now csma security is disabled - will need to be ported/implemented also..."
  packetbuf_set_attr(packet, PACKETBUF_ATTR_FRAME_TYPE, FRAME802154_DATAFRAME);
  return (/*csma_security_create_frame()*/framer_802154.create(packet) < 0)?MAC_TX_ERR_FATAL:MAC_TX_OK;
}

static void MarkDataReceived(void) {
	radioDataReceived = 1;
}
/**
 *
 */
static void TransmitFromQueue(void) {
	//config CSMA/CA (Maximum number of re-transmissions attampts is NBACKOFF_MAX)
	radio_value_t maxBackoff = 0;
	subGHz_radio_driver.get_value(RADIO_PARAM_MAX_BACKOFF_NR, &maxBackoff);
	if (CSMA_MAX_BACKOFF != maxBackoff) {
		TRice("msg:Setting max. backoff value to %u.\n", maxBackoff);
		subGHz_radio_driver.set_value(RADIO_PARAM_MAX_BACKOFF_NR, MIN(7, CSMA_MAX_BACKOFF));
	}
	//prepare buffer for sending
	if (NULL != neighborList) {
		sPacket *packet = NULL;
		uint8_t packetPos = MAX_QUEUED_PACKETS;
		while (packetPos) {
		  packetPos--;
		  if (0 != ((1 << packetPos) & neighborList->queuedTransmits)) {
			  packet = neighborList->transmit[packetPos].packet;
			  break;
		  }
		}
		if (NULL != packet) {
			uint8_t res = MAC_TX_ERR_FATAL;
			uint8_t isBroadcast = packetbuf_holds_broadcast(packet);
			uint8_t dsn = ((uint8_t *)packetbuf_hdrptr(packet))[2] & 0xff;
			uint8_t tempDly;
			switch (subGHz_radio_driver.send(packet)) {
			case tx_ok:
		        if(isBroadcast) {
		        	res = MAC_TX_OK;
		        } else {
		          /* Check for ack */
		        	radioDataReceived = 0;
		        	RadioOverrideRxCb(MarkDataReceived);
		          /* Wait for max CSMA_ACK_WAIT_TIME */
		        	tempDly = CSMA_ACK_WAIT_TIME;
		        	while ((tempDly) && (0 == subGHz_radio_driver.pending_packet())) {
		        		tempDly--;
		        		osDelay(1);
		        	}
		        	if (subGHz_radio_driver.pending_packet()) {
			            int16_t len = subGHz_radio_driver.read(&ackPacket);
			            uint8_t *ackbuf = packetbuf_dataptr(&ackPacket);
			            if((len == CSMA_ACK_LEN) && (ackbuf[2] == dsn)) {
			              /* Ack received */
			          	  res = MAC_TX_OK;
			            } else {
			              /* Not an ack or ack not for us: collision */
			          	  res = MAC_TX_COLLISION;
			            }
		        	} else {
		        		res = MAC_TX_NOACK;
		        	}
		          radioDataReceived = 0;
		          RadioOverrideRxCb(NULL);
		        }
				break;
			case tx_collision:
				res = MAC_TX_COLLISION;
				break;
			default:
				res = MAC_TX_ERR;
				break;
			}
	// now sending is blocking - so wait for completion.
	//else if broadcast - do transmit OK stuff
	//else - do waiting for ACK stuff

	//further in ACK handle stuff:
	//if no ACK received - do retransmit stuff
	//else - do transmit OK stuff

			HandleTransferEnd(res, packetPos);
	  } else {
			TRice("err:could not transmit from queue - packet is missing\n");
	  }
	} else {
		TRice("wrn:could not transmit from queue - queue is empty\n");
	}
}

/**
 *
 */
static void EnqueuePacket(sPacket *packet, mac_callback_t sent, void *ptr) {
  sNeighbor *targetNeighbor = GetNeighborForAddr(packetbuf_addr(packet, PACKETBUF_ADDR_RECEIVER));
  if (NULL == targetNeighbor) {
	  TRice("wrn:could not allocate neighbor, dropping packet\n");
  } else if (((1 << MAX_QUEUED_PACKETS) - 1) <= targetNeighbor->queuedTransmits) {
	  TRice("wrn:could not attach packet to neighbor (neighbor packet queue is full), dropping packet\n");
  } else if (MAC_TX_OK == FormPayload(packet)) {
	  uint8_t packetPos = MAX_QUEUED_PACKETS;
	  while (packetPos) {
		  packetPos--;
		  if (0 == ((1 << packetPos) & targetNeighbor->queuedTransmits)) {
			  targetNeighbor->queuedTransmits |= (1 << packetPos);
			  break;
		  }
	  }
	  targetNeighbor->transmit[packetPos].max_transmissions = packetbuf_attr(packet, PACKETBUF_ATTR_MAX_MAC_TRANSMISSIONS);
	  if (0 == targetNeighbor->transmit[packetPos].max_transmissions) {
		  /* If not set by the application, use the default CSMA value */
		  targetNeighbor->transmit[packetPos].max_transmissions = CSMA_MAX_FRAME_RETRIES + 1;
	  }
	  targetNeighbor->transmit[packetPos].sentCb = sent;
	  targetNeighbor->transmit[packetPos].cptr = ptr;
	  targetNeighbor->transmit[packetPos].packet = packet;
	  if ((NULL != neighborList) && (NULL != neighborList->next)) {
		  /* More neighbors are being processed - print some info about them */
		  sNeighbor *walker = (sNeighbor *)neighborList;
		  while (NULL != walker) {
			  //print neighbor info
			  TRice("msg:\t neighbor ");
			  linkaddr_print(&walker->addr);
			  TRice("msg: have %u packets in queue.\n", GetQueueLenOfNeighbor(walker));
			  walker = walker->next;
		  }
	  } else if ((1 << packetPos) != targetNeighbor->queuedTransmits) { // targetNeighbor here is always neighborList and we already checked if no other neighbors are present
		  /* More packets are in queue, but only for this neighbor - print some info about them */
		  TRice("msg:\t total of %u packets are queued for this neighbor\n", GetQueueLenOfNeighbor(targetNeighbor));
	  } else {
		  /* Only one packet is in queue and only for this neighbor - start transmission of it*/
		  TransmitFromQueue();
	  }
	  return;
  } else {
	  /* Failed to allocate space for headers */
	  TRice("wrn:could form payload for neighbor, dropping packet\n");
  }
  // this is failure catching
  mac_call_sent_callback(sent, ptr, MAC_TX_ERR, 1, packet);
}

/**
 *
 */
static void init_sec(void) {
#if LLSEC802154_USES_AUX_HEADER
  if(packetbuf_attr(PACKETBUF_ATTR_SECURITY_LEVEL) == PACKETBUF_ATTR_SECURITY_LEVEL_DEFAULT) {
    packetbuf_set_attr(PACKETBUF_ATTR_SECURITY_LEVEL, CSMA_LLSEC_SECURITY_LEVEL);
  }
#endif
}

/* API Realization ----------------------------------------------------------*/
/**
 *
 */
static void send_packet(sPacket *packet, mac_callback_t sent, void *ptr) {
  init_sec();
  EnqueuePacket(packet, sent, ptr);
}

/**
 *
 */
static void input_packet(sPacket *rxPacket)
{
  subGHz_radio_driver.read(rxPacket);
  if(packetbuf_datalen(rxPacket) == CSMA_ACK_LEN) {
    /* Ignore ack packets */
	  TRice("msg:ignored ack\n");
#warning "for now csma security is disabled - will need to be ported/implemented also..."
  } else if(/*csma_security_parse_frame()*/framer_802154.parse(rxPacket) < 0) {
	  TRice("err:failed to parse %u\n", packetbuf_datalen(rxPacket));
  } else if(!linkaddr_cmp(packetbuf_addr(rxPacket, PACKETBUF_ADDR_RECEIVER), &linkaddr_node_addr) && !packetbuf_holds_broadcast(rxPacket)) {
	  TRice("wrn:not for us\n");
  } else if(linkaddr_cmp(packetbuf_addr(rxPacket, PACKETBUF_ADDR_SENDER), &linkaddr_node_addr)) {
	  TRice("wrn:frame from ourselves\n");
  } else {
    int duplicate = 0;

    /* Check for duplicate packet. */
    duplicate = mac_sequence_is_duplicate(packetbuf_addr(rxPacket, PACKETBUF_ADDR_SENDER), packetbuf_attr(rxPacket, PACKETBUF_ATTR_MAC_SEQNO));
    if(duplicate) {
      /* Drop the packet. */
    	packetbuf_clear(rxPacket);
    	TRice("wrn:drop duplicate link layer packet from %02X, seqno %u\n", packetbuf_addr(rxPacket, PACKETBUF_ADDR_SENDER), packetbuf_attr(rxPacket, PACKETBUF_ATTR_MAC_SEQNO));
    } else {
      mac_sequence_register_seqno(packetbuf_addr(rxPacket, PACKETBUF_ADDR_SENDER), packetbuf_attr(rxPacket, PACKETBUF_ATTR_MAC_SEQNO));
    }

#if CSMA_SEND_SOFT_ACK
    if(packetbuf_attr(rxPacket, PACKETBUF_ATTR_MAC_ACK)) {
      uint8_t *buff = (uint8_t*)packetbuf_hdrptr(&ackPacket);
      packetbuf_clear(&ackPacket);
      buff[0] = FRAME802154_ACKFRAME;
      buff[1] = 0;
      buff[2] = ((uint8_t *)packetbuf_hdrptr(rxPacket))[2];
      packetbuf_set_datalen(&ackPacket, CSMA_ACK_LEN);
      subGHz_radio_driver.send(&ackPacket);
    }
#endif /* CSMA_SEND_SOFT_ACK */
  }
}

/**
 *
 */
static int on(void) {
  return subGHz_radio_driver.on();
}

/**
 *
 */
static int off(void) {
  return subGHz_radio_driver.off();
}

/**
 *
 */
static void init(uint16_t evtOffset, void (*packedEvtHndl)(uint16_t, void(*)(void)))
{
  radio_value_t radio_max_payload_len;
  csmaEvtIdOffset = evtOffset;
  csmaIrq2Task = packedEvtHndl;
  {
	uint8_t node_mac[8];
	(*(uint32_t*)node_mac) = HAL_GetUIDw1();
	(*(((uint32_t*)node_mac)+1)) = HAL_GetUIDw2();
	(*(((uint32_t*)node_mac)+1)) += HAL_GetUIDw0();
	TRice("msg:MCU uid %08X %08X %08X to %08X %08X MAC.\n", HAL_GetUIDw0(), HAL_GetUIDw1(), HAL_GetUIDw2(), (*(uint32_t*)node_mac), (*(((uint32_t*)node_mac)+1)));
	linkaddr_set_node_addr((linkaddr_t*)node_mac);
  }
  subGHz_radio_driver.init(evtOffset, packedEvtHndl);
  /* Check that the radio can correctly report its max supported payload */
  if(subGHz_radio_driver.get_value(RADIO_CONST_MAX_PAYLOAD_LEN, &radio_max_payload_len) != radio_ok) {
	  TRice("err:! radio does not support getting RADIO_CONST_MAX_PAYLOAD_LEN. Abort init.\n");
    return;
  }

#if LLSEC802154_USES_AUX_HEADER
#ifdef CSMA_LLSEC_DEFAULT_KEY0
  uint8_t key[16] = CSMA_LLSEC_DEFAULT_KEY0;
  csma_security_set_key(0, key);
#endif
#endif /* LLSEC802154_USES_AUX_HEADER */
  on();
}

/**
 *
 */
static int max_payload(sPacket *packet)
{
  int framer_hdrlen;
  radio_value_t max_radio_payload_len;
  eRadioRes res;

  init_sec();

  framer_hdrlen = framer_802154.length(packet);

  res = subGHz_radio_driver.get_value(RADIO_CONST_MAX_PAYLOAD_LEN, &max_radio_payload_len);

  if(res == radio_notSupported) {
	  TRice("err:Failed to retrieve max radio driver payload length\n");
    return 0;
  }

  if(framer_hdrlen < 0) {
    /* Framing failed, we assume the maximum header length */
    framer_hdrlen = CSMA_MAC_MAX_HEADER;
  }

  return MIN(max_radio_payload_len, PACKETBUF_SIZE) - framer_hdrlen - LLSEC802154_PACKETBUF_MIC_LEN();
}

/**
 *
 */
const struct mac_driver csma_driver = {
  "CSMA",
  init,
  send_packet,
  input_packet,
  on,
  off,
  max_payload,
};
/*---------------------------------------------------------------------------*/
