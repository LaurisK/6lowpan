/*
 * Copyright (c) 2007, Swedish Institute of Computer Science.
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
 *         Functions for manipulating link-layer addresses
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

/**
 * \addtogroup linkaddr
 * @{
 */

/* Includes -----------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "linkaddr.h"
#include "trice.h"

/* Private defines ----------------------------------------------------------*/
/* Private types ------------------------------------------------------------*/
/* Global variables ---------------------------------------------------------*/
static linkaddr_t linkaddr_node_addr;
#if LINKADDR_SIZE == 2
const linkaddr_t linkaddr_null = { { 0, 0 } };
#else /*LINKADDR_SIZE == 2*/
#if LINKADDR_SIZE == 8
const linkaddr_t linkaddr_null = { { 0, 0, 0, 0, 0, 0, 0, 0 } };
#endif /*LINKADDR_SIZE == 8*/
#if LINKADDR_SIZE == 6
const linkaddr_t linkaddr_null = { { 0, 0, 0, 0, 0, 0 } };
#endif /*LINKADDR_SIZE == 6*/
#endif /*LINKADDR_SIZE == 2*/

static char address[20];
/* Private functions --------------------------------------------------------*/
/* Functions ----------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
void linkaddr_copy(linkaddr_t *dest, const linkaddr_t *src) {
	memcpy(dest, src, LINKADDR_SIZE);
}

/*---------------------------------------------------------------------------*/
int linkaddr_cmp(const linkaddr_t *addr1, const linkaddr_t *addr2) {
	return (memcmp(addr1, addr2, LINKADDR_SIZE) == 0);
}

/*---------------------------------------------------------------------------*/
void linkaddr_set_node_addr(linkaddr_t *t) {
  linkaddr_copy(&linkaddr_node_addr, t);
}

/*---------------------------------------------------------------------------*/
void linkaddr_get_node_addr(linkaddr_t *addr) {
  linkaddr_copy(addr, &linkaddr_node_addr);
}

/*---------------------------------------------------------------------------*/
int linkaddr_is_node_addr(const linkaddr_t *addr) {
  return linkaddr_cmp(addr, &linkaddr_node_addr);
}

/*---------------------------------------------------------------------------*/
const char *linkaddr_printAddr(const linkaddr_t *addr) {
	uint16_t a0 = __REVSH(addr->u16[0]),
			 a1 = __REVSH(addr->u16[1]),
			 a2 = __REVSH(addr->u16[2]),
			 a3 = __REVSH(addr->u16[3]);
	memset(address, 0x00, 20);
	snprintf(address, 20, "%01X:%01X:%01X:%01X", a0, a1, a2, a3);
	return address;
}
/*---------------------------------------------------------------------------*/
/** @} */
