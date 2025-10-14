/*
 * Copyright (c) 2001-2003, Adam Dunkels.
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
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * This file is part of the uIP TCP/IP stack.
 *
 */

/**
 * \addtogroup uip
 * @{
 */

/**
 * \file
 *    ICMPv6 (RFC 4443) implementation, with message and error handling
 * \author Julien Abeille <jabeille@cisco.com>
 * \author Mathilde Durvy <mdurvy@cisco.com>
 */

#include <string.h>
#include "uip-icmp6.h"
#include "uip-ds6.h"
#include "tcpip.h"
#include "../routing/routing.h"
#include "App/common.h"

static struct uip_icmp6_echo_reply_notification *replyCbListHead = NULL, *replyCbListTail = NULL;
static uip_icmp6_input_handler_t *inputHndlListHead = NULL;
/*---------------------------------------------------------------------------*/
static uip_icmp6_input_handler_t *input_handler_lookup(uint8_t type, uint8_t icode) {
  uip_icmp6_input_handler_t *handler = NULL;

  for(handler = inputHndlListHead; handler != NULL; handler = handler->next) {
    if(handler->type == type && (handler->icode == icode || handler->icode == UIP_ICMP6_HANDLER_CODE_ANY)) {
      return handler;
    }
  }

  return NULL;
}
/*---------------------------------------------------------------------------*/
uint8_t uip_icmp6_input(sUipBuff *uipBuff, uint8_t type, uint8_t icode) {
  uip_icmp6_input_handler_t *handler = input_handler_lookup(type, icode);

  if(handler == NULL) {
    return UIP_ICMP6_INPUT_ERROR;
  }

  if(handler->handler == NULL) {
    return UIP_ICMP6_INPUT_ERROR;
  }

  handler->handler(uipBuff);
  return UIP_ICMP6_INPUT_SUCCESS;
}
/*---------------------------------------------------------------------------*/
void uip_icmp6_register_input_handler(uip_icmp6_input_handler_t *handler) {
    /* Add input handler to list */
	handler->next = inputHndlListHead;
	inputHndlListHead = handler;
}
/*---------------------------------------------------------------------------*/
static void echo_request_input(sUipBuff *uipBuff) {
	/** \brief temporary IP address */
	uip_ipaddr_t tmp_ipaddr;
  /*
   * we send an echo reply. It is trivial if there was no extension
   * headers in the request otherwise we need to remove the extension
   * headers and change a few fields
   */
	  TRiceS("msg:Received Echo Request from %s", uip6_printAddr(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->srcipaddr, NULL));
	  TRiceS("msg:to %s\n", uip6_printAddr(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->destipaddr, NULL));

  /* IP header */
	  IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->ttl = Ds6_GetHopLimit();

  if(uip_is_addr_mcast(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->destipaddr)){
    uip_ipaddr_copy(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->destipaddr, &IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->srcipaddr);
    uip_ds6_select_src(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->srcipaddr, &IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->destipaddr);
  } else {
    uip_ipaddr_copy(&tmp_ipaddr, &IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->srcipaddr);
    uip_ipaddr_copy(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->srcipaddr, &IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->destipaddr);
    uip_ipaddr_copy(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->destipaddr, &tmp_ipaddr);
  }

  uip_remove_ext_hdr(uipBuff);

  /* Below is important for the correctness of UIP_ICMP_BUF and the
   * checksum
   */

  /* Note: now UIP_ICMP_BUF points to the beginning of the echo reply */
  ICMP_HDR_CAST_TO_BUFF(uipBuff->buff.u8 + UIP_IPH_LEN + uipBuff->extLen)->type = ICMP6_ECHO_REPLY;
  ICMP_HDR_CAST_TO_BUFF(uipBuff->buff.u8 + UIP_IPH_LEN + uipBuff->extLen)->icode = 0;
  ICMP_HDR_CAST_TO_BUFF(uipBuff->buff.u8 + UIP_IPH_LEN + uipBuff->extLen)->icmpchksum = 0;
  ICMP_HDR_CAST_TO_BUFF(uipBuff->buff.u8 + UIP_IPH_LEN + uipBuff->extLen)->icmpchksum = ~uip_icmp6chksum(uipBuff);

  TRiceS("msg:Sending Echo Reply to %s", uip6_printAddr(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->destipaddr, NULL));
  TRiceS("msg:from %s\n", uip6_printAddr(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->srcipaddr, NULL));
  UIP_STAT(++uip_stat.icmp.sent);
  return;
}
/*---------------------------------------------------------------------------*/
void uip_icmp6_error_output(sUipBuff *faultyBuff, uint8_t type, uint8_t code, uint32_t param) {
	/** \brief temporary IP address */
	uip_ipaddr_t tmp_ipaddr;
  /* check if originating packet is not an ICMP error */
  uint16_t shift;

  if(faultyBuff->lastProto == UIP_PROTO_ICMP6 && ICMP_HDR_CAST_TO_BUFF(faultyBuff->buff.u8 + UIP_IPH_LEN + faultyBuff->extLen)->type < 128) {
    uipbuf_clear(faultyBuff);
    return;
  }

  /* the source should not be unspecified nor multicast */
  if(uip_is_addr_unspecified(&IP_HDR_CAST_TO_BUFF(faultyBuff->buff.u8)->srcipaddr) ||
     uip_is_addr_mcast(&IP_HDR_CAST_TO_BUFF(faultyBuff->buff.u8)->srcipaddr)) {
    uipbuf_clear(faultyBuff);
    return;
  }

  /* Remove all extension headers related to the routing protocol in place.
   * Keep all other extension headers, so as to match original packet. */
  if(rpl_lite_driver.ext_header_remove(faultyBuff) == 0) {
	  TRice("wrn:Unable to remove ext header before sending ICMPv6 ERROR message\n");
  }

  /* remember data of original packet before shifting */
  uip_ipaddr_copy(&tmp_ipaddr, &IP_HDR_CAST_TO_BUFF(faultyBuff->buff.u8)->destipaddr);

  /* The ICMPv6 error message contains as much of possible of the invoking packet
   * (see RFC 4443 section 3). Make space for the additional IPv6 and
   * ICMPv6 headers here and move payload to the "right". What we move includes
    * extension headers */
  shift = UIP_IPH_LEN + UIP_ICMPH_LEN + UIP_ICMP6_ERROR_LEN;
  faultyBuff->len += shift;
  faultyBuff->len = MIN(faultyBuff->len, UIP_LINK_MTU);
  faultyBuff->extLen = 0;
  memmove(faultyBuff->buff.u8 + shift, faultyBuff->buff.u8, faultyBuff->len - shift);

  IP_HDR_CAST_TO_BUFF(faultyBuff->buff.u8)->vtc = 0x60;
  IP_HDR_CAST_TO_BUFF(faultyBuff->buff.u8)->tcflow = 0;
  IP_HDR_CAST_TO_BUFF(faultyBuff->buff.u8)->flow = 0;
  IP_HDR_CAST_TO_BUFF(faultyBuff->buff.u8)->proto = UIP_PROTO_ICMP6;
  IP_HDR_CAST_TO_BUFF(faultyBuff->buff.u8)->ttl = Ds6_GetHopLimit();

  uip_ipaddr_copy(&IP_HDR_CAST_TO_BUFF(faultyBuff->buff.u8)->destipaddr, &IP_HDR_CAST_TO_BUFF(faultyBuff->buff.u8)->srcipaddr);

  if(uip_is_addr_mcast(&tmp_ipaddr)){
    if(type == ICMP6_PARAM_PROB && code == ICMP6_PARAMPROB_OPTION){
      uip_ds6_select_src(&IP_HDR_CAST_TO_BUFF(faultyBuff->buff.u8)->srcipaddr, &tmp_ipaddr);
    } else {
      uipbuf_clear(faultyBuff);
      return;
    }
  } else {
    /* need to pick a source that corresponds to this node */
    uip_ds6_select_src(&IP_HDR_CAST_TO_BUFF(faultyBuff->buff.u8)->srcipaddr, &tmp_ipaddr);
  }

  ICMP_HDR_CAST_TO_BUFF(faultyBuff->buff.u8 + UIP_IPH_LEN + faultyBuff->extLen)->type = type;
  ICMP_HDR_CAST_TO_BUFF(faultyBuff->buff.u8 + UIP_IPH_LEN + faultyBuff->extLen)->icode = code;
  ((struct uip_icmp6_error*)(faultyBuff->buff.u8 + UIP_IPH_LEN + faultyBuff->extLen))->param = __REV(param);
  uip6_uipHdrSetLen(IP_HDR_CAST_TO_BUFF(faultyBuff->buff.u8), faultyBuff->len - UIP_IPH_LEN);
  ICMP_HDR_CAST_TO_BUFF(faultyBuff->buff.u8 + UIP_IPH_LEN + faultyBuff->extLen)->icmpchksum = 0;
  ICMP_HDR_CAST_TO_BUFF(faultyBuff->buff.u8 + UIP_IPH_LEN + faultyBuff->extLen)->icmpchksum = ~uip_icmp6chksum(faultyBuff);

  UIP_STAT(++uip_stat.icmp.sent);

  TRiceS("wrn:to %s\n", uip6_printAddr(&IP_HDR_CAST_TO_BUFF(faultyBuff->buff.u8)->destipaddr, NULL));
  TRice("wrn:Sending ICMPv6 ERROR message type %d code %d to ", type, code);
  TRiceS("wrn:%s", uip6_printAddr(&IP_HDR_CAST_TO_BUFF(faultyBuff->buff.u8)->destipaddr, NULL));
  TRiceS("wrn: from %s\n", uip6_printAddr(&IP_HDR_CAST_TO_BUFF(faultyBuff->buff.u8)->srcipaddr, NULL));
  return;
}

/*---------------------------------------------------------------------------*/
void uip_icmp6_send(sUipBuff *icmpBuff, const uip_ipaddr_t *dest, int type, int code, int payload_len) {
  uipbuf_clear(icmpBuff);
  ((struct uip_ip_hdr *)(icmpBuff->buff.u8))->vtc = 0x60;
  ((struct uip_ip_hdr *)(icmpBuff->buff.u8))->tcflow = 0;
  ((struct uip_ip_hdr *)(icmpBuff->buff.u8))->flow = 0;
  ((struct uip_ip_hdr *)(icmpBuff->buff.u8))->proto = UIP_PROTO_ICMP6;
  ((struct uip_ip_hdr *)(icmpBuff->buff.u8))->ttl = Ds6_GetHopLimit();
  uip6_uipHdrSetLen(((struct uip_ip_hdr *)(icmpBuff->buff.u8)), UIP_ICMPH_LEN + payload_len);

  if(dest == NULL) {
	  TRice("err:invalid argument; dest is NULL\n");
    return;
  }

  memcpy(&((struct uip_ip_hdr *)(icmpBuff->buff.u8))->destipaddr, dest, sizeof(*dest));
  uip_ds6_select_src(&IP_HDR_CAST_TO_BUFF(icmpBuff->buff.u8)->srcipaddr, &IP_HDR_CAST_TO_BUFF(icmpBuff->buff.u8)->destipaddr);

  ICMP_HDR_CAST_TO_BUFF(icmpBuff->buff.u8 + UIP_IPH_LEN + icmpBuff->extLen)->type = type;
  ICMP_HDR_CAST_TO_BUFF(icmpBuff->buff.u8 + UIP_IPH_LEN + icmpBuff->extLen)->icode = code;

  ICMP_HDR_CAST_TO_BUFF(icmpBuff->buff.u8 + UIP_IPH_LEN + icmpBuff->extLen)->icmpchksum = 0;
  ICMP_HDR_CAST_TO_BUFF(icmpBuff->buff.u8 + UIP_IPH_LEN + icmpBuff->extLen)->icmpchksum = ~uip_icmp6chksum(icmpBuff);

  icmpBuff->len = UIP_IPH_LEN + UIP_ICMPH_LEN + payload_len;

  UIP_STAT(++uip_stat.icmp.sent);
  UIP_STAT(++uip_stat.ip.sent);

  TRiceS("msg:Sending ICMPv6 packet to %s", uip6_printAddr(&IP_HDR_CAST_TO_BUFF(icmpBuff->buff.u8)->destipaddr, NULL));
  TRice("msg:, type %u, code %u, len %u\n", type, code, payload_len);

  tcpip_ipv6_output(icmpBuff);
}
/*---------------------------------------------------------------------------*/
static void echo_reply_input(sUipBuff *uipBuff) {
  int ttl;
  uip_ipaddr_t sender;

  TRiceS("msg:Received Echo Reply from %s", uip6_printAddr(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->srcipaddr, NULL));
  TRiceS("msg:to %s\n", uip6_printAddr(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->destipaddr, NULL));

  uip_ipaddr_copy(&sender, &IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->srcipaddr);
  ttl = IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->ttl;

  uip_remove_ext_hdr(uipBuff);

  /* Call all registered applications to let them know an echo reply
     has been received. */
  {
    struct uip_icmp6_echo_reply_notification *n;
    for(n = replyCbListHead; n != NULL; n = n->next) {
      if(n->callback != NULL) {
        n->callback(&sender, ttl, (uipBuff->buff.u8 + UIP_IPH_LEN + UIP_ICMPH_LEN + uipBuff->extLen), uipBuff->len - sizeof(struct uip_icmp_hdr) - UIP_IPH_LEN);
      }
    }
  }

  uipbuf_clear(uipBuff);
  return;
}
/*---------------------------------------------------------------------------*/
void uip_icmp6_echo_reply_callback_add(struct uip_icmp6_echo_reply_notification *n, uip_icmp6_echo_reply_callback_t c) {
  if(n != NULL && c != NULL) {
    n->callback = c;
    /* Add callback to list */
    n->next = NULL;
    if (NULL != replyCbListTail) {
    	replyCbListTail->next = n;
    } else {
    	replyCbListHead = n;
    }
    replyCbListTail = n;
  }
}
/*---------------------------------------------------------------------------*/
void uip_icmp6_echo_reply_callback_rm(struct uip_icmp6_echo_reply_notification *n) {
	struct uip_icmp6_echo_reply_notification *walker = replyCbListHead, *follower = NULL;
	while (NULL != walker) {
		if (n == walker) {
			if (NULL != follower) {
				follower->next = walker->next;
			} else {
				replyCbListHead = walker->next;
			}
			walker->next = NULL;
			break;
		}
		follower = walker;
		walker = walker->next;
	}
}
/*---------------------------------------------------------------------------*/
static uip_icmp6_input_handler_t echo_request_handler = {NULL, ICMP6_ECHO_REQUEST, UIP_ICMP6_HANDLER_CODE_ANY, echo_request_input};
static uip_icmp6_input_handler_t echo_reply_handler = {NULL, ICMP6_ECHO_REPLY, UIP_ICMP6_HANDLER_CODE_ANY, echo_reply_input};
/*---------------------------------------------------------------------------*/
void
uip_icmp6_init()
{
  /* Register Echo Request and Reply handlers */
  uip_icmp6_register_input_handler(&echo_request_handler);
  uip_icmp6_register_input_handler(&echo_reply_handler);
}
/*---------------------------------------------------------------------------*/
/** @} */
