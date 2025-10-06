/*
 * Copyright (C) 1995, 1996, 1997, and 1998 WIDE Project.
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
 * 3. Neither the name of the project nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE PROJECT AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE PROJECT OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
/*
 * Copyright (c) 2006, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *   may be used to endorse or promote products derived from this software
 *   without specific prior written permission.
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
 */

/**
 * \addtogroup uip
 * @{
 */

/**
 * \file
 *    Neighbor discovery (RFC 4861)
 * \author Mathilde Durvy <mdurvy@cisco.com>
 * \author Julien Abeille <jabeille@cisco.com>
 */

#include <string.h>
#include <inttypes.h>
#include "uip-icmp6.h"
#include "uip-nd6.h"
#include "uip-ds6.h"
#include "uip-ds6-nbr.h"
#include "uip-ds6-route.h"
//#include "net/ipv6/uip-ds6.h"
//#include "net/ipv6/uip-nameserver.h"
//#include "lib/random.h"

/*------------------------------------------------------------------*/
/** @{ */
/** \name Pointers to the header structures.
 */

/**@{  Pointers to messages just after icmp header */
#define UIP_ND6_RA_BUF(uBf)            ((uip_nd6_ra *)(uBf->buff.u8 + UIP_IPH_LEN + UIP_ICMPH_LEN + uBf->extLen))
#define UIP_ND6_NS_BUF(uBf)            ((uip_nd6_ns *)(uBf->buff.u8 + UIP_IPH_LEN + UIP_ICMPH_LEN + uBf->extLen))
#define UIP_ND6_NA_BUF(uBf)            ((uip_nd6_na *)(uBf->buff.u8 + UIP_IPH_LEN + UIP_ICMPH_LEN + uBf->extLen))
/** @} */
/** Pointer to ND option */
#define ND6_OPT(uBf, opt)               ((unsigned char *)((uBf->buff.u8 + UIP_IPH_LEN + UIP_ICMPH_LEN + uBf->extLen) + (opt)))
#define ND6_OPT_HDR_BUF(uBf, opt)       ((uip_nd6_opt_hdr *)ND6_OPT(uBf, opt))
#define ND6_OPT_PREFIX_BUF(uBf, opt)    ((uip_nd6_opt_prefix_info *)ND6_OPT(uBf, opt))
#define ND6_OPT_MTU_BUF(uBf, opt)       ((uip_nd6_opt_mtu *)ND6_OPT(uBf, opt))
#define ND6_OPT_RDNSS_BUF(uBf, opt)     ((uip_nd6_opt_dns *)ND6_OPT(uBf, opt))
/** @} */

#if UIP_ND6_SEND_NS || UIP_ND6_SEND_NA || UIP_ND6_SEND_RA || !UIP_CONF_ROUTER
static uint16_t nd6_opt_offset; /** Offset from the end of the icmpv6 header to the option in uip_buf*/
static uint8_t *nd6_opt_llao;   /**  Pointer to llao option in uip_buf */
static uip_ds6_nbr_t *nbr; /**  Pointer to a nbr cache entry*/
static uip_ds6_addr_t *addr; /**  Pointer to an interface address */
#endif /* UIP_ND6_SEND_NS || UIP_ND6_SEND_NA || UIP_ND6_SEND_RA || !UIP_CONF_ROUTER */

#if UIP_ND6_SEND_NS || UIP_ND6_SEND_RA || !UIP_CONF_ROUTER
static uip_ds6_defrt_t *defrt; /**  Pointer to a router list entry */
#endif /* UIP_ND6_SEND_NS || UIP_ND6_SEND_RA || !UIP_CONF_ROUTER */

#if !UIP_CONF_ROUTER            // TBD see if we move it to ra_input
static uip_nd6_opt_prefix_info *nd6_opt_prefix_info; /**  Pointer to prefix information option in uip_buf */
static uip_ipaddr_t ipaddr;
#endif
#if (!UIP_CONF_ROUTER || UIP_ND6_SEND_RA)
static uip_ds6_prefix_t *prefix; /**  Pointer to a prefix list entry */
#endif

#if UIP_ND6_SEND_NA || UIP_ND6_SEND_RA || !UIP_CONF_ROUTER
/*------------------------------------------------------------------*/
/* Copy link-layer address from LLAO option to a word-aligned uip_lladdr_t */
static int
extract_lladdr_from_llao_aligned(uip_lladdr_t *dest) {
  if(dest != NULL && nd6_opt_llao != NULL) {
    memcpy(dest, &nd6_opt_llao[UIP_ND6_OPT_DATA_OFFSET], UIP_LLADDR_LEN);
    return 1;
  }
  return 0;
}
#endif /* UIP_ND6_SEND_NA || UIP_ND6_SEND_RA || !UIP_CONF_ROUTER */
/*------------------------------------------------------------------*/
#if UIP_ND6_SEND_NA /* UIP_ND6_SEND_NA */
/* create a llao */
static void
create_llao(uint8_t *llao, uint8_t type) {
  llao[UIP_ND6_OPT_TYPE_OFFSET] = type;
  llao[UIP_ND6_OPT_LEN_OFFSET] = UIP_ND6_OPT_LLAO_LEN >> 3;
  memcpy(&llao[UIP_ND6_OPT_DATA_OFFSET], &uip_lladdr, UIP_LLADDR_LEN);
  /* padding on some */
  memset(&llao[UIP_ND6_OPT_DATA_OFFSET + UIP_LLADDR_LEN], 0,
         UIP_ND6_OPT_LLAO_LEN - 2 - UIP_LLADDR_LEN);
}
#endif /* UIP_ND6_SEND_NA */

#if UIP_ND6_SEND_NS
static sUipBuff nsBuff = {0};
#endif /* UIP_ND6_SEND_NS */

/*------------------------------------------------------------------*/
 /**
 * Neighbor Solicitation Processing
 *
 * The NS can be received in 3 cases (procedures):
 * - sender is performing DAD (ip src = unspecified, no SLLAO option)
 * - sender is performing NUD (ip dst = unicast)
 * - sender is performing address resolution (ip dest = solicited node mcast
 * address)
 *
 * We do:
 * - if the tgt belongs to me, reply, otherwise ignore
 * - if i was performing DAD for the same address, two cases:
 * -- I already sent a NS, hence I win
 * -- I did not send a NS yet, hence I lose
 *
 * If we need to send a NA in response (i.e. the NS was done for NUD, or
 * address resolution, or DAD and there is a conflict), we do it in this
 * function: set src, dst, tgt address in the three cases, then for all cases
 * set the rest, including  SLLAO
 *
 */
#if UIP_ND6_SEND_NA
static void ns_input(sUipBuff *uipBuff) {
  uint8_t flags;
  TRiceS(iD(5544), "msg:Received NS from %s", uip6_printAddr(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->srcipaddr, NULL));
  TRiceS(iD(1395), "msg: to %s", uip6_printAddr(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->destipaddr, NULL));
  TRiceS(iD(5057), "msg: with target address %s\n", uip6_printAddr((uip_ipaddr_t *) (&UIP_ND6_NS_BUF(uipBuff)->tgtipaddr), NULL));
  UIP_STAT(++uip_stat.nd6.recv);

#if UIP_CONF_IPV6_CHECKS
  if((IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->ttl != UIP_ND6_HOP_LIMIT) ||
     (uip_is_addr_mcast(&UIP_ND6_NS_BUF(uipBuff)->tgtipaddr)) ||
     (ICMP_HDR_CAST_TO_BUFF(uipBuff->buff.u8 + UIP_IPH_LEN + uipBuff->extLen)->icode != 0)) {
	  TRice(iD(4977), "err:NS received is bad\n");
    goto discard;
  }
#endif /* UIP_CONF_IPV6_CHECKS */

  /* Options processing */
  nd6_opt_llao = NULL;
  nd6_opt_offset = UIP_ND6_NS_LEN;
  while((UIP_IPH_LEN + uipBuff->extLen + UIP_ICMPH_LEN + nd6_opt_offset) < uipBuff->len) {
#if UIP_CONF_IPV6_CHECKS
    if(ND6_OPT_HDR_BUF(uipBuff, nd6_opt_offset)->len == 0) {
    	TRice(iD(4410), "err:NS received is bad\n");
      goto discard;
    }
#endif /* UIP_CONF_IPV6_CHECKS */
    switch (ND6_OPT_HDR_BUF(uipBuff, nd6_opt_offset)->type) {
    case UIP_ND6_OPT_SLLAO:
      nd6_opt_llao = &uipBuff->buff.u8[UIP_IPH_LEN + uipBuff->extLen + UIP_ICMPH_LEN + nd6_opt_offset];
#if UIP_CONF_IPV6_CHECKS
      /* There must be NO option in a DAD NS */
      if(uip_is_addr_unspecified(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->srcipaddr)) {
    	  TRice(iD(6814), "err:NS received is bad\n");
        goto discard;
      } else {
#endif /*UIP_CONF_IPV6_CHECKS */
        uip_lladdr_t lladdr_aligned;
        extract_lladdr_from_llao_aligned(&lladdr_aligned);
        nbr = uip_ds6_nbr_lookup(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->srcipaddr);
        if(nbr == NULL) {
          uip_ds6_nbr_add(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->srcipaddr, &lladdr_aligned, 0, NBR_STALE, NBR_TABLE_REASON_IPV6_ND, NULL);
        } else {
          const uip_lladdr_t *lladdr = uip_ds6_nbr_get_ll(nbr);
          if(lladdr == NULL) {
            goto discard;
          }
          if(memcmp(&nd6_opt_llao[UIP_ND6_OPT_DATA_OFFSET], lladdr, UIP_LLADDR_LEN) != 0) {
            if(uip_ds6_nbr_update_ll(&nbr, (const uip_lladdr_t *)&lladdr_aligned) < 0) {
              /* failed to update the lladdr */
              goto discard;
            }
            nbr->state = NBR_STALE;
          } else {
            if(nbr->state == NBR_INCOMPLETE) {
              nbr->state = NBR_STALE;
            }
          }
        }
#if UIP_CONF_IPV6_CHECKS
      }
#endif /*UIP_CONF_IPV6_CHECKS */
      break;
    default:
    	TRice(iD(5021), "wrn:ND option not supported in NS");
      break;
    }
    nd6_opt_offset += (ND6_OPT_HDR_BUF(uipBuff, nd6_opt_offset)->len << 3);
  }

  addr = uip_ds6_addr_lookup(&UIP_ND6_NS_BUF(uipBuff)->tgtipaddr);
  if(addr != NULL) {
    if(uip_is_addr_unspecified(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->srcipaddr)) {
      /* DAD CASE */
#if UIP_ND6_DEF_MAXDADNS > 0
#if UIP_CONF_IPV6_CHECKS
      if(!uip_is_addr_solicited_node(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->destipaddr)) {
    	  TRice(iD(6620), "err:NS received is bad\n");
        goto discard;
      }
#endif /* UIP_CONF_IPV6_CHECKS */
      if(addr->state != ADDR_TENTATIVE) {
        uip_create_linklocal_allnodes_mcast(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->destipaddr);
        uip_ds6_select_src(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->srcipaddr, &IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->destipaddr);
        flags = UIP_ND6_NA_FLAG_OVERRIDE;
        goto create_na;
      } else {
          /** \todo if I sent a NS before him, I win */
        uip_ds6_dad_failed(addr);
        goto discard;
      }
#else /* UIP_ND6_DEF_MAXDADNS > 0 */
      goto discard;  /* DAD CASE */
#endif /* UIP_ND6_DEF_MAXDADNS > 0 */
    }
#if UIP_CONF_IPV6_CHECKS
    if(uip_ds6_is_my_addr(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->srcipaddr)) {
        /**
         * \NOTE do we do something here? we both are using the same address.
         * If we are doing dad, we could cancel it, though we should receive a
         * NA in response of DAD NS we sent, hence DAD will fail anyway. If we
         * were not doing DAD, it means there is a duplicate in the network!
         */
    	TRice(iD(3277), "err:NS received is bad\n");
      goto discard;
    }
#endif /*UIP_CONF_IPV6_CHECKS */

    /* Address resolution case */
    if(uip_is_addr_solicited_node(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->destipaddr)) {
      uip_ipaddr_copy(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->destipaddr, &IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->srcipaddr);
      uip_ipaddr_copy(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->srcipaddr, &UIP_ND6_NS_BUF(uipBuff)->tgtipaddr);
      flags = UIP_ND6_NA_FLAG_SOLICITED | UIP_ND6_NA_FLAG_OVERRIDE;
      goto create_na;
    }

    /* NUD CASE */
    if(uip_ds6_addr_lookup(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->destipaddr) == addr) {
      uip_ipaddr_copy(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->destipaddr, &IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->srcipaddr);
      uip_ipaddr_copy(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->srcipaddr, &UIP_ND6_NS_BUF(uipBuff)->tgtipaddr);
      flags = UIP_ND6_NA_FLAG_SOLICITED | UIP_ND6_NA_FLAG_OVERRIDE;
      goto create_na;
    } else {
#if UIP_CONF_IPV6_CHECKS
    	TRice(iD(1648), "err:NS received is bad\n");
      goto discard;
#endif /* UIP_CONF_IPV6_CHECKS */
    }
  } else {
    goto discard;
  }


create_na:
    /* If the node is a router it should set R flag in NAs */
#if UIP_CONF_ROUTER
    flags = flags | UIP_ND6_NA_FLAG_ROUTER;
#endif
  uipbuf_clear(uipBuff);
  IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->vtc = 0x60;
  IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->tcflow = 0;
  IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->flow = 0;
  uip6_uipHdrSetLen(IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8), UIP_ICMPH_LEN + UIP_ND6_NA_LEN + UIP_ND6_OPT_LLAO_LEN);
  IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->proto = UIP_PROTO_ICMP6;
  IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->ttl = UIP_ND6_HOP_LIMIT;

  ICMP_HDR_CAST_TO_BUFF(uipBuff->buff.u8 + UIP_IPH_LEN + uipBuff->extLen)->type = ICMP6_NA;
  ICMP_HDR_CAST_TO_BUFF(uipBuff->buff.u8 + UIP_IPH_LEN + uipBuff->extLen)->icode = 0;

  UIP_ND6_NA_BUF(uipBuff)->flagsreserved = flags;
  memcpy(&UIP_ND6_NA_BUF(uipBuff)->tgtipaddr, &addr->ipaddr, sizeof(uip_ipaddr_t));

  create_llao(&uipBuff->buff.u8[UIP_IPH_LEN + uipBuff->extLen + UIP_ICMPH_LEN + UIP_ND6_NA_LEN], UIP_ND6_OPT_TLLAO);

  ICMP_HDR_CAST_TO_BUFF(uipBuff->buff.u8 + UIP_IPH_LEN + uipBuff->extLen)->icmpchksum = 0;
  ICMP_HDR_CAST_TO_BUFF(uipBuff->buff.u8 + UIP_IPH_LEN + uipBuff->extLen)->icmpchksum = ~uip_icmp6chksum(uipBuff);

  uipbuf_set_len(uipBuff, UIP_IPH_LEN + UIP_ICMPH_LEN + UIP_ND6_NA_LEN + UIP_ND6_OPT_LLAO_LEN);

  UIP_STAT(++uip_stat.nd6.sent);
  TRiceS(iD(7977), "msg:Sending NS to %s", uip6_printAddr(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->destipaddr, NULL));
  TRiceS(iD(7902), "msg: from %s", uip6_printAddr(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->srcipaddr, NULL));
  TRiceS(iD(6231), "msg: with target address %s\n", uip6_printAddr(&UIP_ND6_NA_BUF(uipBuff)->tgtipaddr, NULL));
  return;

discard:
  uipbuf_clear(uipBuff);
  return;
}
#endif /* UIP_ND6_SEND_NA */


/*------------------------------------------------------------------*/
#if UIP_ND6_SEND_NS
void uip_nd6_ns_output(uip_ipaddr_t * src, uip_ipaddr_t * dest, uip_ipaddr_t * tgt) {
  uipbuf_clear(&nsBuff);
  IP_HDR_CAST_TO_BUFF(nsBuff.buff.u8)->vtc = 0x60;
  IP_HDR_CAST_TO_BUFF(nsBuff.buff.u8)->tcflow = 0;
  IP_HDR_CAST_TO_BUFF(nsBuff.buff.u8)->flow = 0;
  IP_HDR_CAST_TO_BUFF(nsBuff.buff.u8)->proto = UIP_PROTO_ICMP6;
  IP_HDR_CAST_TO_BUFF(nsBuff.buff.u8)->ttl = UIP_ND6_HOP_LIMIT;

  if(dest == NULL) {
    uip_create_solicited_node(tgt, &IP_HDR_CAST_TO_BUFF(nsBuff.buff.u8)->destipaddr);
  } else {
    uip_ipaddr_copy(&IP_HDR_CAST_TO_BUFF(nsBuff.buff.u8)->destipaddr, dest);
  }
  ICMP_HDR_CAST_TO_BUFF(nsBuff.buff.u8 + UIP_IPH_LEN + nsBuff.extLen)->type = ICMP6_NS;
  ICMP_HDR_CAST_TO_BUFF(nsBuff.buff.u8 + UIP_IPH_LEN + nsBuff.extLen)->icode = 0;
  UIP_ND6_NS_BUF((&nsBuff))->reserved = 0;
  uip_ipaddr_copy((uip_ipaddr_t *) &UIP_ND6_NS_BUF((&nsBuff))->tgtipaddr, tgt);
  /*
   * check if we add a SLLAO option: for DAD, MUST NOT, for NUD, MAY
   * (here yes), for Address resolution , MUST
   */
  if(!(uip_ds6_is_my_addr(tgt))) {
    if(src != NULL) {
      uip_ipaddr_copy(&IP_HDR_CAST_TO_BUFF(nsBuff.buff.u8)->srcipaddr, src);
    } else {
      uip_ds6_select_src(&IP_HDR_CAST_TO_BUFF(nsBuff.buff.u8)->srcipaddr, &IP_HDR_CAST_TO_BUFF(nsBuff.buff.u8)->destipaddr);
    }
    if (uip_is_addr_unspecified(&IP_HDR_CAST_TO_BUFF(nsBuff.buff.u8)->srcipaddr)) {
    	TRice(iD(7681), "err:Dropping NS due to no suitable source address\n");
      uipbuf_clear(&nsBuff);
      return;
    }
    uip6_uipHdrSetLen(IP_HDR_CAST_TO_BUFF(nsBuff.buff.u8), UIP_ICMPH_LEN + UIP_ND6_NS_LEN + UIP_ND6_OPT_LLAO_LEN);

    create_llao(&nsBuff.buff.u8[UIP_IPH_LEN + nsBuff.extLen + UIP_ICMPH_LEN + UIP_ND6_NS_LEN], UIP_ND6_OPT_SLLAO);

    nsBuff.len = UIP_IPH_LEN + UIP_ICMPH_LEN + UIP_ND6_NS_LEN + UIP_ND6_OPT_LLAO_LEN;
  } else {
    uip_create_unspecified(&IP_HDR_CAST_TO_BUFF(nsBuff.buff.u8)->srcipaddr);
    IP_HDR_CAST_TO_BUFF(nsBuff.buff.u8)->len[1] = UIP_ICMPH_LEN + UIP_ND6_NS_LEN;
    nsBuff.len = UIP_IPH_LEN + UIP_ICMPH_LEN + UIP_ND6_NS_LEN;
  }

  ICMP_HDR_CAST_TO_BUFF(nsBuff.buff.u8 + UIP_IPH_LEN + nsBuff.extLen)->icmpchksum = 0;
  ICMP_HDR_CAST_TO_BUFF(nsBuff.buff.u8 + UIP_IPH_LEN + nsBuff.extLen)->icmpchksum = ~uip_icmp6chksum(&nsBuff);

  UIP_STAT(++uip_stat.nd6.sent);
  TRiceS(iD(3556), "msg:Sending NS to %s", uip6_printAddr(&IP_HDR_CAST_TO_BUFF(nsBuff.buff.u8)->destipaddr, NULL));
  TRiceS(iD(4192), "msg: from %s", uip6_printAddr(&IP_HDR_CAST_TO_BUFF(nsBuff.buff.u8)->srcipaddr, NULL));
  TRiceS(iD(6331), "msg: with target address %s\n", uip6_printAddr(tgt, NULL));
  return;
}
#endif /* UIP_ND6_SEND_NS */

#if UIP_ND6_SEND_NS
/*------------------------------------------------------------------*/
/**
 * Neighbor Advertisement Processing
 *
 * we might have to send a pkt that had been buffered while address
 * resolution was performed (if we support buffering, see UIP_CONF_QUEUE_PKT)
 *
 * As per RFC 4861, on link layer that have addresses, TLLAO options MUST be
 * included when responding to multicast solicitations, SHOULD be included in
 * response to unicast (here we assume it is for now)
 *
 * NA can be received after sending NS for DAD, Address resolution or NUD. Can
 * be unsolicited as well.
 * It can trigger update of the state of the neighbor in the neighbor cache,
 * router in the router list.
 * If the NS was for DAD, it means DAD failed
 *
 */
static void na_input(sUipBuff *uipBuff) {
  uint8_t is_llchange;
  uint8_t is_router;
  uint8_t is_solicited;
  uint8_t is_override;
  uip_lladdr_t lladdr_aligned;

  TRiceS(iD(7119), "msg:Received NA from %s", uip6_printAddr(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->srcipaddr, NULL));
  TRiceS(iD(5928), "msg: to %s", uip6_printAddr(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->destipaddr, NULL));
  TRiceS(iD(3504), "msg: with target address %s\n", uip6_printAddr(&UIP_ND6_NA_BUF(uipBuff)->tgtipaddr, NULL));
  UIP_STAT(++uip_stat.nd6.recv);

  /*
   * booleans. the three last one are not 0 or 1 but 0 or 0x80, 0x40, 0x20
   * but it works. Be careful though, do not use tests such as is_router == 1
   */
  is_llchange = 0;
  is_router = ((UIP_ND6_NA_BUF(uipBuff)->flagsreserved & UIP_ND6_NA_FLAG_ROUTER));
  is_solicited =
    ((UIP_ND6_NA_BUF(uipBuff)->flagsreserved & UIP_ND6_NA_FLAG_SOLICITED));
  is_override =
    ((UIP_ND6_NA_BUF(uipBuff)->flagsreserved & UIP_ND6_NA_FLAG_OVERRIDE));

#if UIP_CONF_IPV6_CHECKS
  if((IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->ttl != UIP_ND6_HOP_LIMIT) ||
     (ICMP_HDR_CAST_TO_BUFF(uipBuff->buff.u8 + UIP_IPH_LEN + uipBuff->extLen)->icode != 0) ||
     (uip_is_addr_mcast(&UIP_ND6_NA_BUF(uipBuff)->tgtipaddr)) ||
     (is_solicited && uip_is_addr_mcast(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->destipaddr))) {
	  TRice(iD(2845), "err:NA received is bad\n");
    goto discard;
  }
#endif /*UIP_CONF_IPV6_CHECKS */

  /* Options processing: we handle TLLAO, and must ignore others */
  nd6_opt_offset = UIP_ND6_NA_LEN;
  nd6_opt_llao = NULL;
  while((UIP_IPH_LEN + uipBuff->extLen + UIP_ICMPH_LEN + nd6_opt_offset) < uipBuff->len) {
#if UIP_CONF_IPV6_CHECKS
    if(ND6_OPT_HDR_BUF(uipBuff, nd6_opt_offset)->len == 0) {
    	TRice(iD(2922), "err:NA received is bad\n");
      goto discard;
    }
#endif /*UIP_CONF_IPV6_CHECKS */
    switch (ND6_OPT_HDR_BUF(uipBuff, nd6_opt_offset)->type) {
    case UIP_ND6_OPT_TLLAO:
      nd6_opt_llao = (uint8_t *)ND6_OPT_HDR_BUF(uipBuff, nd6_opt_offset);
      break;
    default:
    	TRice(iD(3528), "wrn:ND option not supported in NA\n");
      break;
    }
    nd6_opt_offset += (ND6_OPT_HDR_BUF(uipBuff, nd6_opt_offset)->len << 3);
  }
  addr = uip_ds6_addr_lookup(&UIP_ND6_NA_BUF(uipBuff)->tgtipaddr);
  /* Message processing, including TLLAO if any */
  if(addr != NULL) {
#if UIP_ND6_DEF_MAXDADNS > 0
    if(addr->state == ADDR_TENTATIVE) {
      uip_ds6_dad_failed(addr);
    }
#endif /*UIP_ND6_DEF_MAXDADNS > 0 */
    TRice(iD(7049), "err:NA received is bad\n");
    goto discard;
  } else {
    const uip_lladdr_t *lladdr;
    nbr = uip_ds6_nbr_lookup(&UIP_ND6_NA_BUF(uipBuff)->tgtipaddr);
    if(nbr == NULL) {
      goto discard;
    }
    lladdr = uip_ds6_nbr_get_ll(nbr);
    if(lladdr == NULL) {
      goto discard;
    }
    if(nd6_opt_llao != NULL) {
      is_llchange =
        memcmp(&nd6_opt_llao[UIP_ND6_OPT_DATA_OFFSET], lladdr, UIP_LLADDR_LEN) == 0 ? 0 : 1;
    }
    if(nbr->state == NBR_INCOMPLETE) {
      if(nd6_opt_llao == NULL || !extract_lladdr_from_llao_aligned(&lladdr_aligned)) {
        goto discard;
      }
      if(uip_ds6_nbr_update_ll(&nbr, (const uip_lladdr_t *)&lladdr_aligned) < 0) {
        /* failed to update the lladdr */
        goto discard;
      }

      /* Note: No need to refresh the state of the nbr here.
       * It has already been refreshed upon receiving the unicast IPv6 ND packet.
       * See: uip_ds6_nbr_refresh_reachable_state()
       */
      if(!is_solicited) {
        nbr->state = NBR_STALE;
      }
      nbr->isrouter = is_router;
    } else { /* NBR is not INCOMPLETE */
      if(!is_override && is_llchange) {
        if(nbr->state == NBR_REACHABLE) {
          nbr->state = NBR_STALE;
        }
        goto discard;
      } else {
        /**
         *  If this is an cache override, or same lladdr, or no llao -
         *  do updates of nbr states.
         */
        if(is_override || !is_llchange || nd6_opt_llao == NULL) {
          if(nd6_opt_llao != NULL && is_llchange) {
            if(!extract_lladdr_from_llao_aligned(&lladdr_aligned) ||
               uip_ds6_nbr_update_ll(&nbr, (const uip_lladdr_t *)&lladdr_aligned) < 0) {
              /* failed to update the lladdr */
              goto discard;
            }
          }
          /* Note: No need to refresh the state of the nbr here.
           * It has already been refreshed upon receiving the unicast IPv6 ND packet.
           * See: uip_ds6_nbr_refresh_reachable_state()
           */
        }
      }
      if(nbr->isrouter && !is_router) {
        defrt = uip_ds6_defrt_lookup(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->srcipaddr);
        if(defrt != NULL) {
          uip_ds6_defrt_rm(defrt);
        }
      }
      nbr->isrouter = is_router;
    }
  }
#if UIP_CONF_IPV6_QUEUE_PKT
  /* The nbr is now reachable, check if we had buffered a pkt for it */
  /*if(nbr->queue_buf_len != 0) {
    uip_len = nbr->queue_buf_len;
    memcpy(UIP_IP_BUF, nbr->queue_buf, uip_len);
    nbr->queue_buf_len = 0;
    return;
    }*/
  if(uip_packetqueue_buflen(&nbr->packethandle) != 0) {
	uipBuff->len = uip_packetqueue_buflen(&nbr->packethandle);
    memcpy(IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8), uip_packetqueue_buf(&nbr->packethandle), uipBuff->len);
    uip_packetqueue_free(&nbr->packethandle);
    return;
  }

#endif /*UIP_CONF_IPV6_QUEUE_PKT */

discard:
  uipbuf_clear(uipBuff);
  return;
}
#endif /* UIP_ND6_SEND_NS */

#if UIP_CONF_ROUTER
#if UIP_ND6_SEND_RA
/*---------------------------------------------------------------------------*/
static void rs_input(sUipBuff *uipBuff) {

  TRiceS(iD(3499), "msg:Received RS from %s", uip6_printAddr(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->srcipaddr, NULL));
  TRiceS(iD(2470), "msg: to %s\n", uip6_printAddr(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->destipaddr, NULL));
  UIP_STAT(++uip_stat.nd6.recv);


#if UIP_CONF_IPV6_CHECKS
  /*
   * Check hop limit / icmp code
   * target address must not be multicast
   * if the NA is solicited, dest must not be multicast
   */
  if((IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->ttl != UIP_ND6_HOP_LIMIT) || (ICMP_HDR_CAST_TO_BUFF(uipBuff->buff.u8 + UIP_IPH_LEN + uipBuff->extLen)->icode != 0)) {
	  TRice(iD(7555), "err:RS received is bad\n");
    goto discard;
  }
#endif /*UIP_CONF_IPV6_CHECKS */

  /* Only valid option is Source Link-Layer Address option any thing
     else is discarded */
  nd6_opt_offset = UIP_ND6_RS_LEN;
  nd6_opt_llao = NULL;

  while((UIP_IPH_LEN + uipBuff->extLen + UIP_ICMPH_LEN + nd6_opt_offset) < uipBuff->len) {
#if UIP_CONF_IPV6_CHECKS
    if(ND6_OPT_HDR_BUF(uipBuff, nd6_opt_offset)->len == 0) {
    	TRice(iD(7912), "err:RS received is bad\n");
      goto discard;
    }
#endif /*UIP_CONF_IPV6_CHECKS */
    switch (ND6_OPT_HDR_BUF(uipBuff, nd6_opt_offset)->type) {
    case UIP_ND6_OPT_SLLAO:
      nd6_opt_llao = (uint8_t *)ND6_OPT_HDR_BUF(uipBuff, nd6_opt_offset);
      break;
    default:
    	TRice(iD(2143), "wrn:ND option not supported in RS\n");
      break;
    }
    nd6_opt_offset += (ND6_OPT_HDR_BUF(uipBuff, nd6_opt_offset)->len << 3);
  }
  /* Options processing: only SLLAO */
  if(nd6_opt_llao != NULL) {
#if UIP_CONF_IPV6_CHECKS
    if(uip_is_addr_unspecified(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->srcipaddr)) {
    	TRice(iD(4090), "err:RS received is bad\n");
      goto discard;
    } else {
#endif /*UIP_CONF_IPV6_CHECKS */
      uip_lladdr_t lladdr_aligned;
      extract_lladdr_from_llao_aligned(&lladdr_aligned);
      if((nbr = uip_ds6_nbr_lookup(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->srcipaddr)) == NULL) {
        /* we need to add the neighbor */
        uip_ds6_nbr_add(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->srcipaddr, &lladdr_aligned, 0, NBR_STALE, NBR_TABLE_REASON_IPV6_ND, NULL);
      } else {
        /* If LL address changed, set neighbor state to stale */
        const uip_lladdr_t *lladdr = uip_ds6_nbr_get_ll(nbr);
        if(lladdr == NULL) {
          goto discard;
        }
        if(memcmp(&nd6_opt_llao[UIP_ND6_OPT_DATA_OFFSET],
            lladdr, UIP_LLADDR_LEN) != 0) {
          uip_ds6_nbr_t nbr_data;
          nbr_data = *nbr;
          uip_ds6_nbr_rm(nbr);
          nbr = uip_ds6_nbr_add(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->srcipaddr, &lladdr_aligned, 0, NBR_STALE, NBR_TABLE_REASON_IPV6_ND, NULL);
          nbr->reachable = nbr_data.reachable;
          nbr->sendns = nbr_data.sendns;
          nbr->nscount = nbr_data.nscount;
        }
        nbr->isrouter = 0;
      }
#if UIP_CONF_IPV6_CHECKS
    }
#endif /*UIP_CONF_IPV6_CHECKS */
  }

  /* Schedule a sollicited RA */
  uip_ds6_send_ra_sollicited();

discard:
  uipbuf_clear(uipBuff);
  return;
}

/*---------------------------------------------------------------------------*/
void uip_nd6_ra_output(sUipBuff *dsPeriodicBuff, uip_ipaddr_t * dest) {

  IP_HDR_CAST_TO_BUFF(dsPeriodicBuff->buff.u8)->vtc = 0x60;
  IP_HDR_CAST_TO_BUFF(dsPeriodicBuff->buff.u8)->tcflow = 0;
  IP_HDR_CAST_TO_BUFF(dsPeriodicBuff->buff.u8)->flow = 0;
  IP_HDR_CAST_TO_BUFF(dsPeriodicBuff->buff.u8)->proto = UIP_PROTO_ICMP6;
  IP_HDR_CAST_TO_BUFF(dsPeriodicBuff->buff.u8)->ttl = UIP_ND6_HOP_LIMIT;

  if(dest == NULL) {
    uip_create_linklocal_allnodes_mcast(&IP_HDR_CAST_TO_BUFF(dsPeriodicBuff->buff.u8)->destipaddr);
  } else {
    /* For sollicited RA */
    uip_ipaddr_copy(&IP_HDR_CAST_TO_BUFF(dsPeriodicBuff->buff.u8)->destipaddr, dest);
  }
  uip_ds6_select_src(&IP_HDR_CAST_TO_BUFF(dsPeriodicBuff->buff.u8)->srcipaddr, &IP_HDR_CAST_TO_BUFF(dsPeriodicBuff->buff.u8)->destipaddr);

  ICMP_HDR_CAST_TO_BUFF(dsPeriodicBuff->buff.u8 + UIP_IPH_LEN + dsPeriodicBuff->extLen)->type = ICMP6_RA;
  ICMP_HDR_CAST_TO_BUFF(dsPeriodicBuff->buff.u8 + UIP_IPH_LEN + dsPeriodicBuff->extLen)->icode = 0;

  UIP_ND6_RA_BUF(dsPeriodicBuff)->cur_ttl = Ds6_GetHopLimit();

  UIP_ND6_RA_BUF(dsPeriodicBuff)->flags_reserved = (UIP_ND6_M_FLAG << 7) | (UIP_ND6_O_FLAG << 6);

  UIP_ND6_RA_BUF(dsPeriodicBuff)->router_lifetime = __REVSH(UIP_ND6_ROUTER_LIFETIME);
  //UIP_ND6_RA_BUF(dsPeriodicBuff)->reachable_time = __REV(uip_ds6_if.reachable_time);
  //UIP_ND6_RA_BUF(dsPeriodicBuff)->retrans_timer = __REV(uip_ds6_if.retrans_timer);
  UIP_ND6_RA_BUF(dsPeriodicBuff)->reachable_time = 0;
  UIP_ND6_RA_BUF(dsPeriodicBuff)->retrans_timer = 0;

  dsPeriodicBuff->len = UIP_IPH_LEN + UIP_ICMPH_LEN + UIP_ND6_RA_LEN;
  nd6_opt_offset = UIP_ND6_RA_LEN;


  /* Prefix list */
  for(prefix = uip_ds6_prefix_list;
      prefix < uip_ds6_prefix_list + UIP_DS6_PREFIX_NB; prefix++) {
    if((prefix->isused) && (prefix->advertise)) {
      ND6_OPT_PREFIX_BUF(dsPeriodicBuff, nd6_opt_offset)->type = UIP_ND6_OPT_PREFIX_INFO;
      ND6_OPT_PREFIX_BUF(dsPeriodicBuff, nd6_opt_offset)->len = UIP_ND6_OPT_PREFIX_INFO_LEN / 8;
      ND6_OPT_PREFIX_BUF(dsPeriodicBuff, nd6_opt_offset)->preflen = prefix->length;
      ND6_OPT_PREFIX_BUF(dsPeriodicBuff, nd6_opt_offset)->flagsreserved1 = prefix->l_a_reserved;
      ND6_OPT_PREFIX_BUF(dsPeriodicBuff, nd6_opt_offset)->validlt = __REV(prefix->vlifetime);
      ND6_OPT_PREFIX_BUF(dsPeriodicBuff, nd6_opt_offset)->preferredlt = __REV(prefix->plifetime);
      ND6_OPT_PREFIX_BUF(dsPeriodicBuff, nd6_opt_offset)->reserved2 = 0;
      uip_ipaddr_copy(&(ND6_OPT_PREFIX_BUF(dsPeriodicBuff, nd6_opt_offset)->prefix), &(prefix->ipaddr));
      nd6_opt_offset += UIP_ND6_OPT_PREFIX_INFO_LEN;
      dsPeriodicBuff->len += UIP_ND6_OPT_PREFIX_INFO_LEN;
    }
  }

  /* Source link-layer option */
  create_llao(ND6_OPT(dsPeriodicBuff, nd6_opt_offset), UIP_ND6_OPT_SLLAO);

  dsPeriodicBuff->len += UIP_ND6_OPT_LLAO_LEN;
  nd6_opt_offset += UIP_ND6_OPT_LLAO_LEN;

  /* MTU */
  ND6_OPT_MTU_BUF(dsPeriodicBuff, nd6_opt_offset)->type = UIP_ND6_OPT_MTU;
  ND6_OPT_MTU_BUF(dsPeriodicBuff, nd6_opt_offset)->len = UIP_ND6_OPT_MTU_LEN >> 3;
  ND6_OPT_MTU_BUF(dsPeriodicBuff, nd6_opt_offset)->reserved = 0;
  //ND6_OPT_MTU_BUF(dsPeriodicBuff, nd6_opt_offset)->mtu = __REV(uip_ds6_if.link_mtu);
  ND6_OPT_MTU_BUF(dsPeriodicBuff, nd6_opt_offset)->mtu = __REV(1500);

  dsPeriodicBuff->len += UIP_ND6_OPT_MTU_LEN;
  nd6_opt_offset += UIP_ND6_OPT_MTU_LEN;

#if UIP_ND6_RA_RDNSS
  if(uip_nameserver_count() > 0) {
    uint8_t i = 0;
    uip_ipaddr_t *ip = &ND6_OPT_RDNSS_BUF(nd6_opt_offset)->ip;
    uip_ipaddr_t *dns = NULL;
    ND6_OPT_RDNSS_BUF(nd6_opt_offset)->type = UIP_ND6_OPT_RDNSS;
    ND6_OPT_RDNSS_BUF(nd6_opt_offset)->reserved = 0;
    ND6_OPT_RDNSS_BUF(nd6_opt_offset)->lifetime = uip_nameserver_next_expiration();
    if(ND6_OPT_RDNSS_BUF(nd6_opt_offset)->lifetime != UIP_NAMESERVER_INFINITE_LIFETIME) {
      ND6_OPT_RDNSS_BUF(nd6_opt_offset)->lifetime -= clock_seconds();
    }
    while((dns = uip_nameserver_get(i)) != NULL) {
      uip_ipaddr_copy(ip++, dns);
      i++;
    }
    ND6_OPT_RDNSS_BUF(nd6_opt_offset)->len = UIP_ND6_OPT_RDNSS_LEN + (i << 1);
    TRice(iD(7722), "msg:%d nameservers reported\n", i);
    dsPeriodicBuff->len += ND6_OPT_RDNSS_BUF(nd6_opt_offset)->len << 3;
    nd6_opt_offset += ND6_OPT_RDNSS_BUF(nd6_opt_offset)->len << 3;
  }
#endif /* UIP_ND6_RA_RDNSS */

  uip6_uipHdrSetLen(IP_HDR_CAST_TO_BUFF(dsPeriodicBuff->buff.u8), dsPeriodicBuff->len - UIP_IPH_LEN);

  /*ICMP checksum */
  ICMP_HDR_CAST_TO_BUFF(dsPeriodicBuff->buff.u8 + UIP_IPH_LEN + dsPeriodicBuff->extLen)->icmpchksum = 0;
  ICMP_HDR_CAST_TO_BUFF(dsPeriodicBuff->buff.u8 + UIP_IPH_LEN + dsPeriodicBuff->extLen)->icmpchksum = ~uip_icmp6chksum(dsPeriodicBuff);

  UIP_STAT(++uip_stat.nd6.sent);
  TRiceS(iD(6586), "msg:Sending RA to %s", uip6_printAddr(&IP_HDR_CAST_TO_BUFF(dsPeriodicBuff->buff.u8)->destipaddr, NULL));
  TRiceS(iD(1909), "msg: from %s\n", uip6_printAddr(&IP_HDR_CAST_TO_BUFF(dsPeriodicBuff->buff.u8)->srcipaddr, NULL));
  return;
}
#endif /* UIP_ND6_SEND_RA */
#endif /* UIP_CONF_ROUTER */

#if !UIP_CONF_ROUTER
/*---------------------------------------------------------------------------*/
/**
 * Process a Router Advertisement
 *
 * - Possible actions when receiving a RA: add router to router list,
 *   recalculate reachable time, update link hop limit, update retrans timer.
 * - If MTU option: update MTU.
 * - If SLLAO option: update entry in neighbor cache
 * - If prefix option: start autoconf, add prefix to prefix list
 */
void ra_input(sUipBuff *uipBuff) {
  uip_lladdr_t lladdr_aligned;
  TRiceS(iD(1175), "msg:Received RA from %s", uip6_printAddr(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->srcipaddr, NULL));
  TRiceS(iD(6237), "msg: to %s\n", uip6_printAddr(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->destipaddr, NULL));
  UIP_STAT(++uip_stat.nd6.recv);

#if UIP_CONF_IPV6_CHECKS
  if((IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->ttl != UIP_ND6_HOP_LIMIT)
		  || (!uip_is_addr_linklocal(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->srcipaddr))
		  || (ICMP_HDR_CAST_TO_BUFF(uipBuff->buff.u8 + UIP_IPH_LEN + uipBuff->extLen)->icode != 0)) {
	  TRice(iD(3877), "err:RA received is bad");
    goto discard;
  }
#endif /*UIP_CONF_IPV6_CHECKS */

  Ds6_SetHopLimit(UIP_ND6_RA_BUF(uipBuff)->cur_ttl);
  Ds6_SetReachableTimes(__REV(UIP_ND6_RA_BUF(uipBuff)->reachable_time));
  Ds6_SetRetransmitTim(__REV(UIP_ND6_RA_BUF(uipBuff)->retrans_timer));

  /* Options processing */
  nd6_opt_offset = UIP_ND6_RA_LEN;
  while((UIP_IPH_LEN + uipBuff->extLen + UIP_ICMPH_LEN + nd6_opt_offset) < uipBuff->len) {
    if(ND6_OPT_HDR_BUF(uipBuff, nd6_opt_offset)->len == 0) {
    	TRice(iD(2215), "err:RA received is bad");
      goto discard;
    }
    switch (ND6_OPT_HDR_BUF(uipBuff, nd6_opt_offset)->type) {
    case UIP_ND6_OPT_SLLAO:
    	TRice(iD(1181), "dbg:Processing SLLAO option in RA\n");
      nd6_opt_llao = (uint8_t *) ND6_OPT_HDR_BUF(uipBuff, nd6_opt_offset);
      nbr = uip_ds6_nbr_lookup(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->srcipaddr);
      if(!extract_lladdr_from_llao_aligned(&lladdr_aligned)) {
        /* failed to extract llao - discard packet */
        goto discard;
      }
      if(nbr == NULL) {
        nbr = uip_ds6_nbr_add(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->srcipaddr, &lladdr_aligned, 1, NBR_STALE, NBR_TABLE_REASON_IPV6_ND, NULL);
      } else {
        const uip_lladdr_t *lladdr = uip_ds6_nbr_get_ll(nbr);
        if(lladdr == NULL) {
          goto discard;
        }
        if(nbr->state == NBR_INCOMPLETE) {
          nbr->state = NBR_STALE;
        }
        if(memcmp(&nd6_opt_llao[UIP_ND6_OPT_DATA_OFFSET],
                  lladdr, UIP_LLADDR_LEN) != 0) {
          /* change of link layer address */
          if(uip_ds6_nbr_update_ll(&nbr,
                                   (const uip_lladdr_t *)&lladdr_aligned) < 0) {
            /* failed to update the lladdr */
            goto discard;
          }
          nbr->state = NBR_STALE;
        }
        nbr->isrouter = 1;
      }
      break;
    case UIP_ND6_OPT_MTU:
    	TRice(iD(7413), "dbg:Processing MTU option in RA\n");
      uip_ds6_if.link_mtu = __REV(((uip_nd6_opt_mtu *) ND6_OPT_HDR_BUF(uipBuff, nd6_opt_offset))->mtu);
      break;
    case UIP_ND6_OPT_PREFIX_INFO:
    	TRice(iD(7174), "dbg:Processing PREFIX option in RA\n");
      nd6_opt_prefix_info = (uip_nd6_opt_prefix_info *) ND6_OPT_HDR_BUF(uipBuff, nd6_opt_offset);
      if((__REV(nd6_opt_prefix_info->validlt) >= __REV(nd6_opt_prefix_info->preferredlt))
         && (!uip_is_addr_linklocal(&nd6_opt_prefix_info->prefix))) {
        /* on-link flag related processing */
        if(nd6_opt_prefix_info->flagsreserved1 & UIP_ND6_RA_FLAG_ONLINK) {
          prefix =
            uip_ds6_prefix_lookup(&nd6_opt_prefix_info->prefix, nd6_opt_prefix_info->preflen);
          if(prefix == NULL) {
            if(nd6_opt_prefix_info->validlt != 0) {
              if(nd6_opt_prefix_info->validlt != UIP_ND6_INFINITE_LIFETIME) {
                prefix = uip_ds6_prefix_add(&nd6_opt_prefix_info->prefix,
                                            nd6_opt_prefix_info->preflen,
											__REV(nd6_opt_prefix_info->
                                                  validlt));
              } else {
                prefix = uip_ds6_prefix_add(&nd6_opt_prefix_info->prefix, nd6_opt_prefix_info->preflen, 0);
              }
            }
          } else {
            switch (nd6_opt_prefix_info->validlt) {
            case 0:
              uip_ds6_prefix_rm(prefix);
              break;
            case UIP_ND6_INFINITE_LIFETIME:
              prefix->isinfinite = 1;
              break;
            default:
              TRiceS(iD(5396), "dbg:Updating timer of prefix %s", uip6_printAddr(&addr->ipaddr, NULL));
              TRice(iD(1533), "dbg: new value %d\n", __REV(nd6_opt_prefix_info->validlt));
              Time_TimerSet(&prefix->vlifetime, __REV(nd6_opt_prefix_info->validlt));
              prefix->isinfinite = 0;
              break;
            }
          }
        }
        /* End of on-link flag related processing */
        /* autonomous flag related processing */
        if((nd6_opt_prefix_info->flagsreserved1 & UIP_ND6_RA_FLAG_AUTONOMOUS)
           && (nd6_opt_prefix_info->validlt != 0)
           && (nd6_opt_prefix_info->preflen == UIP_DEFAULT_PREFIX_LEN)) {

          uip_ipaddr_copy(&ipaddr, &nd6_opt_prefix_info->prefix);
          Addr_SetInterfId(&ipaddr, &uip_lladdr);
          addr = uip_ds6_addr_lookup(&ipaddr);
          if((addr != NULL) && (addr->type == ADDR_AUTOCONF)) {
            if(nd6_opt_prefix_info->validlt != UIP_ND6_INFINITE_LIFETIME) {
              /* The processing below is defined in RFC4862 section 5.5.3 e */
              TRiceS(iD(2134), "dbg:Updating timer of address %s", uip6_printAddr(&addr->ipaddr, NULL));
              if((__REV(nd6_opt_prefix_info->validlt) > 2 * 60 * 60) || (__REV(nd6_opt_prefix_info->validlt) > Time_TimerRemaining(&addr->vlifetime))) {
            	TRice(iD(7581), "dbg: new value %lu\n", (unsigned long)__REV(nd6_opt_prefix_info->validlt));
            	Time_TimerSet(&addr->vlifetime, __REV(nd6_opt_prefix_info->validlt));
              } else {
            	  Time_TimerSet(&addr->vlifetime, 2 * 60 * 60);
                TRice(iD(2936), "dbg: new value %lu\n", (unsigned long)(2 * 60 * 60));
              }
              addr->isinfinite = 0;
            } else {
              addr->isinfinite = 1;
            }
          } else {
            if(__REV(nd6_opt_prefix_info->validlt) == UIP_ND6_INFINITE_LIFETIME) {
              uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);
            } else {
              uip_ds6_addr_add(&ipaddr, __REV(nd6_opt_prefix_info->validlt), ADDR_AUTOCONF);
            }
          }
        }
        /* End of autonomous flag related processing */
      }
      break;
#if UIP_ND6_RA_RDNSS
    case UIP_ND6_OPT_RDNSS:
      uint8_t naddr = (ND6_OPT_RDNSS_BUF(uipBuff, nd6_opt_offset)->len - 1) / 2;
      uip_ipaddr_t *ip = (uip_ipaddr_t *)(&ND6_OPT_RDNSS_BUF(uipBuff, nd6_opt_offset)->ip);
      TRice(iD(5872), "dbg:Processing RDNSS option\n\t got %d nameservers\n", naddr);
      while(naddr-- > 0) {
    	TRiceS(iD(6219), "dbg:nameserver: %s", uip6_printAddr(ip, NULL));
        TRice(iD(7345), "dbg: lifetime: %d\n", __REV(ND6_OPT_RDNSS_BUF(uipBuff, nd6_opt_offset)->lifetime));
        uip_nameserver_update(ip, __REV(ND6_OPT_RDNSS_BUF(uipBuff, nd6_opt_offset)->lifetime));
        ip++;
      }
      break;
#endif /* UIP_ND6_RA_RDNSS */
    default:
    	TRice(iD(6834), "err:ND option not supported in RA\n");
      break;
    }
    nd6_opt_offset += (ND6_OPT_HDR_BUF(uipBuff, nd6_opt_offset)->len << 3);
  }

  defrt = uip_ds6_defrt_lookup(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->srcipaddr);
  if(UIP_ND6_RA_BUF(uipBuff)->router_lifetime != 0) {
    if(nbr != NULL) {
      nbr->isrouter = 1;
    }
    if(defrt == NULL) {
      uip_ds6_defrt_add(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->srcipaddr, (unsigned long)(uip_ntohs(UIP_ND6_RA_BUF->router_lifetime)));
    } else {
    	Time_TimerSet(&(defrt->lifetime), (unsigned long)(uip_ntohs(UIP_ND6_RA_BUF->router_lifetime)));
    }
  } else {
    if(defrt != NULL) {
      uip_ds6_defrt_rm(defrt);
    }
  }

#if UIP_CONF_IPV6_QUEUE_PKT
  /* If the nbr just became reachable (e.g. it was in NBR_INCOMPLETE state
   * and we got a SLLAO), check if we had buffered a pkt for it */
  /*  if((nbr != NULL) && (nbr->queue_buf_len != 0)) {
    uip_len = nbr->queue_buf_len;
    memcpy(UIP_IP_BUF, nbr->queue_buf, uip_len);
    nbr->queue_buf_len = 0;
    return;
    }*/
  if(nbr != NULL && uip_packetqueue_buflen(&nbr->packethandle) != 0) {
	uipBuff->len = uip_packetqueue_buflen(&nbr->packethandle);
    memcpy(IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8), uip_packetqueue_buf(&nbr->packethandle), uipBuff->len);
    uip_packetqueue_free(&nbr->packethandle);
    return;
  }

#endif /*UIP_CONF_IPV6_QUEUE_PKT */

discard:
  uipbuf_clear();
  return;
}
#endif /* !UIP_CONF_ROUTER */
/*------------------------------------------------------------------*/
/* ICMPv6 input handlers */
#if UIP_ND6_SEND_NA
static uip_icmp6_input_handler_t ns_input_handler = {NULL, ICMP6_NS, UIP_ICMP6_HANDLER_CODE_ANY, ns_input};
#endif
#if UIP_ND6_SEND_NS
static uip_icmp6_input_handler_t na_input_handler = {NULL, ICMP6_NA, UIP_ICMP6_HANDLER_CODE_ANY, na_input};
#endif

#if UIP_CONF_ROUTER && UIP_ND6_SEND_RA
static uip_icmp6_input_handler_t rs_input_handler = {NULL, ICMP6_RS, UIP_ICMP6_HANDLER_CODE_ANY, rs_input};
#endif

#if !UIP_CONF_ROUTER
static uip_icmp6_input_handler_t ra_input_handler = {NULL, ICMP6_RA, UIP_ICMP6_HANDLER_CODE_ANY, ra_input};
#endif
/*---------------------------------------------------------------------------*/
void
uip_nd6_init()
{
#if UIP_ND6_SEND_NA
  /* Only handle NSs if we are prepared to send out NAs */
  uip_icmp6_register_input_handler(&ns_input_handler);
#endif

#if UIP_ND6_SEND_NS
  /*
   * Only handle NAs if we are prepared to send out NSs. */
  uip_icmp6_register_input_handler(&na_input_handler);
#endif

#if UIP_CONF_ROUTER && UIP_ND6_SEND_RA
  /* Only accept RS if we are a router and happy to send out RAs */
  uip_icmp6_register_input_handler(&rs_input_handler);
#endif

#if !UIP_CONF_ROUTER
  /* Only process RAs if we are not a router */
  uip_icmp6_register_input_handler(&ra_input_handler);
#endif
}
/*---------------------------------------------------------------------------*/
 /** @} */
