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
 *         Module for sending UDP packets through uIP.
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include "uip-udp-packet.h"
#include "tcpip.h"

#include <string.h>
static sUipBuff txUipBuff;

/*---------------------------------------------------------------------------*/
void uip_udp_packet_send(sSocket *c, const void *data, int len)
{
#if UIP_UDP
  if(data != NULL && len <= (UIP_BUFSIZE - UIP_IPUDPH_LEN)) {
    txUipBuff.sLen = len;
    //memmove(&uip_buf[UIP_IPUDPH_LEN], data, len);
    memcpy(&txUipBuff.buff.u8[UIP_IPUDPH_LEN], data, len);
    uip_process(c, &txUipBuff, UIP_UDP_SEND_CONN);

#if UIP_IPV6_MULTICAST
  /* Let the multicast engine process the datagram before we send it */
  if(uip_is_addr_mcast_routable(&servicingUdpConn->ripaddr)) {
    UIP_MCAST6.out();
  }
#endif /* UIP_IPV6_MULTICAST */

#if NETSTACK_CONF_WITH_IPV6
    tcpip_ipv6_output(&txUipBuff);
#else
    if(txUipBuff.len > 0) {
      tcpip_output();
    }
#endif
  }
#endif /* UIP_UDP */
}
/*---------------------------------------------------------------------------*/
void uip_udp_packet_sendto(sSocket *c, const void *data, int len,
		      const uip_ipaddr_t *toaddr, uint16_t toport)
{
  uip_ipaddr_t curaddr;
  uint16_t curport;

  if(toaddr != NULL) {
    /* Save current IP addr/port. */
    uip_ipaddr_copy(&curaddr, &c->dstAddr);
    curport = c->dstPort;

    /* Load new IP addr/port */
    uip_ipaddr_copy(&c->dstAddr, toaddr);
    c->dstPort = toport;

    uip_udp_packet_send(c, data, len);

    /* Restore old IP addr/port */
    uip_ipaddr_copy(&c->dstAddr, &curaddr);
    c->dstPort = curport;
  }
}
/*---------------------------------------------------------------------------*/
