/*
 * Copyright (c) 2006, Swedish Institute of Computer Science.
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
 *         Packet buffer (packetbuf) management
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

/**
 * \addtogroup packetbuf
 * @{
 */

/* Includes -----------------------------------------------------------------*/
#include <string.h>
#include "packetbuf.h"
#if defined(STM32H753xx)
#include "main.h"
#else
#include "App/common.h"
#endif

/* Private defines ----------------------------------------------------------*/
/* Private types ------------------------------------------------------------*/
/* Pseudo global variables --------------------------------------------------*/
/* Private functions --------------------------------------------------------*/
/* Functions ----------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
void packetbuf_clear(sPacket *packet) {
  packet->buffLen = 0;
  packet->buffPtr = 0;
  packet->hdrLen = 0;

  packetbuf_attr_clear(packet);
}

/*---------------------------------------------------------------------------*/
int packetbuf_copyfrom(sPacket *packet, const void *from, uint16_t len) {
  uint16_t l;

  packetbuf_clear(packet);
  l = MIN(PACKETBUF_SIZE, len);
  memcpy(packet->buffer, from, l);
  packet->buffLen = l;
  return l;
}

/*---------------------------------------------------------------------------*/
int packetbuf_copyto(sPacket *packet, void *to) {
  if(packet->buffLen + packet->hdrLen > PACKETBUF_SIZE) {
    return 0;
  }
  memcpy(to, packetbuf_hdrptr(packet), packet->hdrLen);
  memcpy((uint8_t *)to + packet->hdrLen, packetbuf_dataptr(packet), packet->buffLen);
  return (packet->buffLen + packet->hdrLen);
}

/*---------------------------------------------------------------------------*/
int packetbuf_hdralloc(sPacket *packet, int size) {
  int16_t i;
  uint8_t *packetbuf = (uint8_t *)packet->buffer;

  if(size + packetbuf_totlen(packet) > PACKETBUF_SIZE) {
    return 0;
  }

  /* shift data to the right */
  for(i = packetbuf_totlen(packet) - 1; i >= 0; i--) {
    packetbuf[i + size] = packetbuf[i];
  }
  packet->hdrLen += size;
  return 1;
}

/*---------------------------------------------------------------------------*/
int packetbuf_hdrreduce(sPacket *packet, int size) {
  if(packet->buffLen < size) {
    return 0;
  }

  packet->buffPtr += size;
  packet->buffLen -= size;
  return 1;
}

/*---------------------------------------------------------------------------*/
void packetbuf_set_datalen(sPacket *packet, uint16_t len) {
	packet->buffLen = len;
}

/*---------------------------------------------------------------------------*/
void * packetbuf_dataptr(sPacket *packet) {
  return (uint8_t *)packet->buffer + packetbuf_hdrlen(packet);
}

/*---------------------------------------------------------------------------*/
void * packetbuf_hdrptr(sPacket *packet) {
  return (uint8_t *)packet->buffer;
}

/*---------------------------------------------------------------------------*/
uint16_t packetbuf_datalen(sPacket *packet) {
  return packet->buffLen;
}
/*---------------------------------------------------------------------------*/
uint8_t packetbuf_hdrlen(sPacket *packet) {
  return packet->buffPtr + packet->hdrLen;
}

/*---------------------------------------------------------------------------*/
uint16_t packetbuf_totlen(sPacket *packet) {
  return packetbuf_hdrlen(packet) + packetbuf_datalen(packet);
}

/*---------------------------------------------------------------------------*/
uint16_t packetbuf_remaininglen(sPacket *packet) {
  return PACKETBUF_SIZE - packetbuf_totlen(packet);
}

/*---------------------------------------------------------------------------*/
void packetbuf_attr_clear(sPacket *packet) {
  int i = PACKETBUF_NUM_ADDRS;
  memset(packet->attributes, 0, (sizeof(uint16_t) * PACKETBUF_NUM_ATTRS));
  while (i) {
    i--;
    linkaddr_copy(&packet->addresses[i], &linkaddr_null);
  }
}

/*---------------------------------------------------------------------------*/
void packetbuf_attr_copyto(sPacket *packet, uint16_t *attrs, linkaddr_t *addrs) {
  memcpy(attrs, packet->attributes, (sizeof(uint16_t) * PACKETBUF_NUM_ATTRS));
  memcpy(addrs, packet->addresses, (sizeof(linkaddr_t) * PACKETBUF_NUM_ADDRS));
}

/*---------------------------------------------------------------------------*/
void packetbuf_attr_copyfrom(sPacket *packet, uint16_t *attrs, linkaddr_t *addrs) {
  memcpy(packet->attributes, attrs, (sizeof(uint16_t) * PACKETBUF_NUM_ATTRS));
  memcpy(packet->addresses, addrs, (sizeof(linkaddr_t) * PACKETBUF_NUM_ADDRS));
}

/*---------------------------------------------------------------------------*/
int packetbuf_set_attr(sPacket *packet, uint8_t type, const uint16_t val) {
  packet->attributes[type] = val;
  return 1;
}

/*---------------------------------------------------------------------------*/
uint16_t packetbuf_attr(sPacket *packet, uint8_t type) {
  return packet->attributes[type];
}

/*---------------------------------------------------------------------------*/
int packetbuf_set_addr(sPacket *packet, uint8_t type, const linkaddr_t *addr) {
  linkaddr_copy(&packet->addresses[type - PACKETBUF_ADDR_FIRST], addr);
  return 1;
}

/*---------------------------------------------------------------------------*/
const linkaddr_t * packetbuf_addr(sPacket *packet, uint8_t type) {
  return &packet->addresses[type - PACKETBUF_ADDR_FIRST];
}

/*---------------------------------------------------------------------------*/
int packetbuf_holds_broadcast(sPacket *packet) {
  return linkaddr_cmp(&packet->addresses[PACKETBUF_ADDR_RECEIVER - PACKETBUF_ADDR_FIRST], &linkaddr_null);
}
/*---------------------------------------------------------------------------*/

/** @} */
