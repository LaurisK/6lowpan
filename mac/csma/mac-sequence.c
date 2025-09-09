/*
 * Copyright (c) 2010, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Copyright (c) 2013, ADVANSEE - http://www.advansee.com/
 * Benoît Thébaudeau <benoit.thebaudeau@advansee.com>
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
 *         MAC sequence numbers management
 * \author
 *         Adam Dunkels <adam@sics.se>
 *         Benoît Thébaudeau <benoit.thebaudeau@advansee.com>
 */

/* Includes -----------------------------------------------------------------*/
#include <string.h>
#if defined(STM32H753xx)
#include "main.h"
#else
#include "App/Time/time.h"
#endif
#include "mac-sequence.h"

/* Private defines ----------------------------------------------------------*/
#ifdef NETSTACK_CONF_MAC_SEQNO_MAX_AGE
#define SEQNO_MAX_AGE NETSTACK_CONF_MAC_SEQNO_MAX_AGE
#else /* NETSTACK_CONF_MAC_SEQNO_MAX_AGE */
#define SEQNO_MAX_AGE 20
#endif /* NETSTACK_CONF_MAC_SEQNO_MAX_AGE */

#ifdef NETSTACK_CONF_MAC_SEQNO_HISTORY
#define MAX_SEQNOS NETSTACK_CONF_MAC_SEQNO_HISTORY
#else /* NETSTACK_CONF_MAC_SEQNO_HISTORY */
#define MAX_SEQNOS 16
#endif /* NETSTACK_CONF_MAC_SEQNO_HISTORY */

/* Private types ------------------------------------------------------------*/
struct seqno {
  linkaddr_t sender;
  uint32_t   timestamp;
  uint8_t seqno;
};

/* Pseudo global variables --------------------------------------------------*/
static struct seqno received_seqnos[MAX_SEQNOS];

/* Functions ----------------------------------------------------------------*/
/**
 *
 */
int mac_sequence_is_duplicate(const linkaddr_t *addr, const uint8_t seqNr)
{
  int i;
#if defined(STM32H753xx)
	uint32_t  nowTs = GetUptime();
#else
	sDateTime nowDt    = {0};
	uint32_t  nowTs;
	Time_Get(time_local, &nowDt);
	nowTs = Time_GetTimeStamp(&nowDt);
#endif
  /*
   * Check for duplicate packet by comparing the sequence number of the incoming
   * packet with the last few ones we saw.
   */
  for(i = 0; i < MAX_SEQNOS; ++i) {
    if(linkaddr_cmp(addr, &received_seqnos[i].sender)) {
      if(seqNr == received_seqnos[i].seqno) {
#if SEQNO_MAX_AGE > 0
        if(nowTs - received_seqnos[i].timestamp <= SEQNO_MAX_AGE) {
          /* Duplicate packet. */
          return 1;
        }
#else /* SEQNO_MAX_AGE > 0 */
        return 1;
#endif /* SEQNO_MAX_AGE > 0 */
      }
      break;
    }
  }
  return 0;
}

/**
 *
 */
void mac_sequence_register_seqno(const linkaddr_t *addr, const uint8_t seqNr)
{
  int i, j;
#if !defined(STM32H753xx)
  	sDateTime nowDt = {0};
  	Time_Get(time_local, &nowDt);
#endif
  /* Locate possible previous sequence number for this address. */
  for(i = 0; i < MAX_SEQNOS; ++i) {
    if(linkaddr_cmp(addr, &received_seqnos[i].sender)) {
      i++;
      break;
    }
  }

  /* Keep the last sequence number for each address as per 802.15.4e. */
  for(j = i - 1; j > 0; --j) {
    memcpy(&received_seqnos[j], &received_seqnos[j - 1], sizeof(struct seqno));
  }
  received_seqnos[0].seqno = seqNr;
#if defined(STM32H753xx)
  received_seqnos[0].timestamp = GetUptime();
#else
  received_seqnos[0].timestamp = Time_GetTimeStamp(&nowDt);
#endif
  linkaddr_copy(&received_seqnos[0].sender, addr);
}
