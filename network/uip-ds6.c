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
 *    IPv6 data structure manipulation.
 *    Comprises part of the Neighbor discovery (RFC 4861)
 *    and auto configuration (RFC 4862) state machines.
 * \author Mathilde Durvy <mdurvy@cisco.com>
 * \author Julien Abeille <jabeille@cisco.com>
 */

#include <string.h>
#include <stdlib.h>
#include <stddef.h>
//#include "lib/random.h"
//#include "uip-nd6.h"
#include "uip-ds6.h"
#include "uip-ds6-route.h"
#include "uip-ds6-nbr.h"
#include "App/Time/time.h"
//#include "net/ipv6/multicast/uip-mcast6.h"
//#include "net/ipv6/uip-packetqueue.h"
#include "cmsis_os.h"
#if defined(STM32H753xx)
#include "trice.h"
#else
#include "App/common.h"
#endif

/** \brief  Interface structure (contains all the interface variables) */
typedef struct uip_ds6_netif {
  uint32_t link_mtu;
  uint8_t cur_hop_limit;
  uint32_t base_reachable_time; /* in msec */
  uint32_t reachable_time;      /* in msec */
  uint32_t retrans_timer;       /* in msec */
  uint8_t maxdadns;
#if UIP_DS6_ADDR_NB
  uip_ds6_addr_t addr_list[UIP_DS6_ADDR_NB];
#endif /* UIP_DS6_ADDR_NB */
#if UIP_DS6_AADDR_NB
  uip_ds6_aaddr_t aaddr_list[UIP_DS6_AADDR_NB];
#endif /* UIP_DS6_AADDR_NB */
#if UIP_DS6_MADDR_NB
  uip_ds6_maddr_t maddr_list[UIP_DS6_MADDR_NB];
#endif /* UIP_DS6_MADDR_NB */
} uip_ds6_netif_t;

#ifdef UIP_CONF_ND6_RETRANS_TIMER
#define UIP_ND6_RETRANS_TIMER          UIP_CONF_ND6_RETRANS_TIMER
#else
#define UIP_ND6_RETRANS_TIMER          1000
#endif

#if UIP_CONF_ROUTER
sTimeTimer uip_ds6_timer_ra;                                 /**< RA timer, to schedule RA sending */
#if UIP_ND6_SEND_RA
static uint8_t racount;                                         /**< number of RA already sent */
static uint16_t rand_time;                                      /**< random time value for timers */
#endif
#endif /* UIP_CONF_ROUTER */

/** \name "DS6" Data structures */
/** @{ */
uip_ds6_netif_t uip_ds6_if;                                     /**< The single interface */
uip_ds6_prefix_t uip_ds6_prefix_list[UIP_DS6_PREFIX_NB];        /**< Prefix list */

/* Used by Cooja to enable extraction of addresses from memory.*/
uint8_t uip_ds6_addr_size;
uint8_t uip_ds6_netif_addr_list_offset;

/** @} */

/* "full" (as opposed to pointer) ip address used in this file,  */
static uip_ipaddr_t loc_fipaddr;

/* Pointers used in this file */
static uip_ds6_addr_t *locaddr;
static uip_ds6_maddr_t *locmaddr;
#if UIP_DS6_AADDR_NB
static uip_ds6_aaddr_t *locaaddr;
#endif /* UIP_DS6_AADDR_NB */
static uip_ds6_prefix_t *locprefix;
#if (UIP_LLADDR_LEN == 2)
static const uint8_t iid_prefix[] = { 0x00, 0x00 , 0x00 , 0xff , 0xfe , 0x00 };
#endif /* (UIP_LLADDR_LEN == 2) */

/* The default prefix */
static uip_ip6addr_t default_prefix = {
    .u16 = { 0, 0, 0, 0, 0, 0, 0, 0 }
};

static TimerHandle_t periodicTim;
static TimerHandle_t rsSendTmo;

/*---------------------------------------------------------------------------*/
const uip_ip6addr_t *
uip_ds6_default_prefix()
{
  return &default_prefix;
}
/*---------------------------------------------------------------------------*/
void
uip_ds6_set_default_prefix(const uip_ip6addr_t *prefix)
{
  uip_ip6addr_copy(&default_prefix, prefix);
}

/*---------------------------------------------------------------------------*/
static void uip_ds6_periodic(void)
{

  /* Periodic processing on unicast addresses */
  for(locaddr = uip_ds6_if.addr_list;
      locaddr < uip_ds6_if.addr_list + UIP_DS6_ADDR_NB; locaddr++) {
    if(locaddr->isused) {
      if((!locaddr->isinfinite) && (Time_TimerExpired(&locaddr->vlifetime))) {
        uip_ds6_addr_rm(locaddr);
#if UIP_ND6_DEF_MAXDADNS > 0
      } else if((locaddr->state == ADDR_TENTATIVE)
                && (locaddr->dadnscount <= uip_ds6_if.maxdadns)
                && (timer_expired(&locaddr->dadtimer))
                && (uip_len == 0)) {
        uip_ds6_dad(locaddr);
#endif /* UIP_ND6_DEF_MAXDADNS > 0 */
      }
    }
  }

  /* Periodic processing on default routers */
  uip_ds6_defrt_periodic();

#if !UIP_CONF_ROUTER
  /* Periodic processing on prefixes */
  for(locprefix = uip_ds6_prefix_list;
      locprefix < uip_ds6_prefix_list + UIP_DS6_PREFIX_NB;
      locprefix++) {
    if(locprefix->isused && !locprefix->isinfinite
       && Time_TimerExpired(&(locprefix->vlifetime))) {
      uip_ds6_prefix_rm(locprefix);
    }
  }
#endif /* !UIP_CONF_ROUTER */

#if UIP_ND6_SEND_NS
  uip_ds6_neighbor_periodic();
#endif /* UIP_ND6_SEND_NS */

#if UIP_CONF_ROUTER && UIP_ND6_SEND_RA
  /* Periodic RA sending */
  if(Time_TimerExpired(&uip_ds6_timer_ra) && (uip_len == 0)) {
    uip_ds6_send_ra_periodic();
  }
#endif /* UIP_CONF_ROUTER && UIP_ND6_SEND_RA */
  return;
}

static void HandleDs6PeriodicTimer(TimerHandle_t periodicTim) {
    uip_ds6_periodic();
    tcpip_ipv6_output();
}

#if !UIP_CONF_ROUTER
/*---------------------------------------------------------------------------*/
static void uip_nd6_rs_output(sUipBuff *uipBuff)
{
  IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->vtc = 0x60;
  IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->tcflow = 0;
  IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->flow = 0;
  IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->proto = UIP_PROTO_ICMP6;
  IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->ttl = UIP_ND6_HOP_LIMIT;
  uip_create_linklocal_allrouters_mcast(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->destipaddr);
  uip_ds6_select_src(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->srcipaddr, &IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->destipaddr);
  UIP_ICMP_BUF->type = ICMP6_RS;
  UIP_ICMP_BUF->icode = 0;

  if(uip_is_addr_unspecified(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->srcipaddr)) {
	  IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->len[1] = UIP_ICMPH_LEN + UIP_ND6_RS_LEN;
    uip_len = UIP_IPH_LEN + uipBuff->extLen + UIP_ICMPH_LEN + UIP_ND6_RS_LEN;
  } else {
    uip_len = UIP_IPH_LEN + uipBuff->extLen + UIP_ICMPH_LEN + UIP_ND6_RS_LEN + UIP_ND6_OPT_LLAO_LEN;
    uip6_uipHdrSetLen(IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8), UIP_ICMPH_LEN + UIP_ND6_RS_LEN + UIP_ND6_OPT_LLAO_LEN);

    create_llao(&uip_buf[UIP_IPH_LEN + uipBuff->extLen + UIP_ICMPH_LEN + UIP_ND6_RS_LEN], UIP_ND6_OPT_SLLAO);
  }

  UIP_ICMP_BUF->icmpchksum = 0;
  UIP_ICMP_BUF->icmpchksum = ~uip_icmp6chksum();

  UIP_STAT(++uip_stat.nd6.sent);
  TRiceS(iD(5761), "msg:Sending RS to %s", uip6_printAddr(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->destipaddr, NULL));
  TRiceS(iD(5480), "msg: from %s\n", uip6_printAddr(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->srcipaddr, NULL));
  return;
}
/*---------------------------------------------------------------------------*/
static void uip_ds6_send_rs(sUipBuff *uipBuff)
{
  static uint8_t rscount = 0;  /**< number of rs already sent */
  uip_ipaddr_t dfltRoute = uip_ds6_defrt_choose();
  if((NULL == dfltRoute) && (rscount < UIP_ND6_MAX_RTR_SOLICITATIONS)) {
	TRice(iD(3460), "msg:Sending RS %u\n", rscount);
    uip_nd6_rs_output();
    rscount++;
    xTimerChangePeriod(rsSendTmo, pdMS_TO_TICKS(1000 * UIP_ND6_RTR_SOLICITATION_INTERVAL));
    xTimerStart(rsSendTmo, 0);
  } else {
	TRiceS(iD(6546), "msg:Router found ? (boolean): %s\n", (NULL != dfltRoute)?"true":"false");
  }
  return;
}

static void HandleRsSendingTmo(TimerHandle_t rsSendTim) {
#warning "rsUipBuff should be resolved with dynamic memory."
	sUipBuff rsUipBuff;
    uip_ds6_send_rs(&rsUipBuff);
    tcpip_ipv6_output(&rsUipBuff);
}

#endif /* !UIP_CONF_ROUTER */

/** \brief Compute the reachable time based on base reachable time, see RFC 4861*/
static uint32_t uip_ds6_compute_reachable_time(uint32_t baseReachTime)
{
  return (System_Random(baseReachTime) + (baseReachTime / 2));
}

/*---------------------------------------------------------------------------*/
void uip_ds6_init(void)
{
  if(uip_is_addr_unspecified(&default_prefix)) {
    uip_ip6addr(&default_prefix, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);
  }

  uip_ds6_neighbors_init();
  uip_ds6_route_init();

  TRice(iD(1950), "msg:Init: %u neighbors\n\t %u default routers\n\t %u prefixes\n\t %u routes\n\t %u unicast addresses\n\t %u multicast addresses\n\t %u anycast addresses\n",
		NBR_TABLE_MAX_NEIGHBORS, UIP_DS6_DEFRT_NB, UIP_DS6_PREFIX_NB, UIP_DS6_ROUTE_NB, UIP_DS6_ADDR_NB, UIP_DS6_MADDR_NB, UIP_DS6_AADDR_NB);

  memset(uip_ds6_prefix_list, 0, sizeof(uip_ds6_prefix_list));
  memset(&uip_ds6_if, 0, sizeof(uip_ds6_if));
  uip_ds6_addr_size = sizeof(struct uip_ds6_addr);
  uip_ds6_netif_addr_list_offset = offsetof(struct uip_ds6_netif, addr_list);

  /* Set interface parameters */
  uip_ds6_if.link_mtu = UIP_LINK_MTU;
  uip_ds6_if.cur_hop_limit = UIP_TTL;
  Ds6_SetReachableTimes(UIP_ND6_REACHABLE_TIME);
  uip_ds6_if.retrans_timer = UIP_ND6_RETRANS_TIMER;
  uip_ds6_if.maxdadns = UIP_ND6_DEF_MAXDADNS;

  /* Create link local address, prefix, multicast addresses, anycast addresses */
  uip_create_linklocal_prefix(&loc_fipaddr);
#if UIP_CONF_ROUTER
  uip_ds6_prefix_add(&loc_fipaddr, UIP_DEFAULT_PREFIX_LEN, 0, 0, 0, 0);
#else /* UIP_CONF_ROUTER */
  uip_ds6_prefix_add(&loc_fipaddr, UIP_DEFAULT_PREFIX_LEN, 0);
#endif /* UIP_CONF_ROUTER */
  uip_ds6_set_addr_iid(&loc_fipaddr, &uip_lladdr);
  uip_ds6_addr_add(&loc_fipaddr, 0, ADDR_AUTOCONF);

  uip_create_linklocal_allnodes_mcast(&loc_fipaddr);
  uip_ds6_maddr_add(&loc_fipaddr);
#if UIP_CONF_ROUTER
  uip_create_linklocal_allrouters_mcast(&loc_fipaddr);
  uip_ds6_maddr_add(&loc_fipaddr);
#if UIP_ND6_SEND_RA
  Time_TimerSet(&uip_ds6_timer_ra, 2);     /* wait to have a link local IP address */
#endif /* UIP_ND6_SEND_RA */
#else /* UIP_CONF_ROUTER */
  rsSendTmo = xTimerCreate("rsSendingTmo", pdMS_TO_TICKS(System_Random(1000 * UIP_ND6_MAX_RTR_SOLICITATION_DELAY)), pdFALSE, 0, HandleRsSendingTmo);
  xTimerStart(rsSendTmo, 0);
#endif /* UIP_CONF_ROUTER */
  periodicTim = xTimerCreate("ds6PeriodicTimer", pdMS_TO_TICKS(1000 * UIP_DS6_PERIOD), pdTRUE, 0, HandleDs6PeriodicTimer);
  xTimerStart(periodicTim, 0);

  return;
}


/*---------------------------------------------------------------------------*/
uint8_t
uip_ds6_list_loop(uip_ds6_element_t *list, uint8_t size,
                  uint16_t elementsize, uip_ipaddr_t *ipaddr,
                  uint8_t ipaddrlen, uip_ds6_element_t **out_element)
{
  uip_ds6_element_t *element;

  if(list == NULL || ipaddr == NULL || out_element == NULL) {
    return NOSPACE;
  }

  *out_element = NULL;

  for(element = list;
      element <
      (uip_ds6_element_t *)((uint8_t *)list + (size * elementsize));
      element = (uip_ds6_element_t *)((uint8_t *)element + elementsize)) {
    if(element->isused) {
      if(uip_ipaddr_prefixcmp(&element->ipaddr, ipaddr, ipaddrlen)) {
        *out_element = element;
        return FOUND;
      }
    } else {
      *out_element = element;
    }
  }

  return *out_element != NULL ? FREESPACE : NOSPACE;
}

/*---------------------------------------------------------------------------*/
#if UIP_CONF_ROUTER
/*---------------------------------------------------------------------------*/
uip_ds6_prefix_t * uip_ds6_prefix_add(uip_ipaddr_t *ipaddr, uint8_t ipaddrlen, uint8_t advertise, uint8_t flags, uint32_t vtime, uint32_t ptime)
{
  if(uip_ds6_list_loop ((uip_ds6_element_t *)uip_ds6_prefix_list, UIP_DS6_PREFIX_NB, sizeof(uip_ds6_prefix_t), ipaddr, ipaddrlen, (uip_ds6_element_t **)&locprefix) == FREESPACE) {
    locprefix->isused = 1;
    uip_ipaddr_copy(&locprefix->ipaddr, ipaddr);
    locprefix->length = ipaddrlen;
    locprefix->advertise = advertise;
    locprefix->l_a_reserved = flags;
    locprefix->vlifetime = vtime;
    locprefix->plifetime = ptime;
    TRiceS(iD(4081), "msg:Adding prefix %s ", uip6_printAddr(&locprefix->ipaddr, NULL));
    TRice(iD(4528), "msg:length %u, flags %x, Valid lifetime %lx, Preffered lifetime %lx\n", ipaddrlen, flags, vtime, ptime);
    return locprefix;
  } else {
	  TRice(iD(4000), "msg:No more space in Prefix list\n");
  }
  return NULL;
}


#else /* UIP_CONF_ROUTER */
uip_ds6_prefix_t *
uip_ds6_prefix_add(uip_ipaddr_t *ipaddr, uint8_t ipaddrlen, uint32_t interval)
{
  if(uip_ds6_list_loop
     ((uip_ds6_element_t *)uip_ds6_prefix_list, UIP_DS6_PREFIX_NB,
      sizeof(uip_ds6_prefix_t), ipaddr, ipaddrlen,
      (uip_ds6_element_t **)&locprefix) == FREESPACE) {
    locprefix->isused = 1;
    uip_ipaddr_copy(&locprefix->ipaddr, ipaddr);
    locprefix->length = ipaddrlen;
    if(interval != 0) {
      Time_TimerSet(&(locprefix->vlifetime), interval);
      locprefix->isinfinite = 0;
    } else {
      locprefix->isinfinite = 1;
    }
    TRiceS(iD(3384), "msg:Adding prefix %s ", uip6_printAddr(&locprefix->ipaddr, NULL));
    TRice(iD(6671), "msg:length %u, vlifetime %lu\n", ipaddrlen, interval);
    return locprefix;
  }
  return NULL;
}
#endif /* UIP_CONF_ROUTER */

/*---------------------------------------------------------------------------*/
void
uip_ds6_prefix_rm(uip_ds6_prefix_t *prefix)
{
  if(prefix != NULL) {
    prefix->isused = 0;
  }
  return;
}
/*---------------------------------------------------------------------------*/
uip_ds6_prefix_t *
uip_ds6_prefix_lookup(uip_ipaddr_t *ipaddr, uint8_t ipaddrlen)
{
  if(uip_ds6_list_loop((uip_ds6_element_t *)uip_ds6_prefix_list,
                       UIP_DS6_PREFIX_NB, sizeof(uip_ds6_prefix_t),
                       ipaddr, ipaddrlen,
                       (uip_ds6_element_t **)&locprefix) == FOUND) {
    return locprefix;
  }
  return NULL;
}

/*---------------------------------------------------------------------------*/
uint8_t
uip_ds6_is_addr_onlink(uip_ipaddr_t *ipaddr)
{
  for(locprefix = uip_ds6_prefix_list;
      locprefix < uip_ds6_prefix_list + UIP_DS6_PREFIX_NB; locprefix++) {
    if(locprefix->isused &&
       uip_ipaddr_prefixcmp(&locprefix->ipaddr, ipaddr, locprefix->length)) {
      return 1;
    }
  }
  return 0;
}

/*---------------------------------------------------------------------------*/
uip_ds6_addr_t *
uip_ds6_addr_add(uip_ipaddr_t *ipaddr, uint32_t vlifetime, uint8_t type)
{
  if(uip_ds6_list_loop
     ((uip_ds6_element_t *)uip_ds6_if.addr_list, UIP_DS6_ADDR_NB,
      sizeof(uip_ds6_addr_t), ipaddr, 128,
      (uip_ds6_element_t **)&locaddr) == FREESPACE) {
    locaddr->isused = 1;
    uip_ipaddr_copy(&locaddr->ipaddr, ipaddr);
    locaddr->type = type;
    if(vlifetime == 0) {
      locaddr->isinfinite = 1;
    } else {
      locaddr->isinfinite = 0;
      Time_TimerSet(&(locaddr->vlifetime), vlifetime);
    }
#if UIP_ND6_DEF_MAXDADNS > 0
    locaddr->state = ADDR_TENTATIVE;
    timer_set(&locaddr->dadtimer,
              random_rand() % (UIP_ND6_MAX_RTR_SOLICITATION_DELAY *
                               CLOCK_SECOND));
    locaddr->dadnscount = 0;
#else /* UIP_ND6_DEF_MAXDADNS > 0 */
    locaddr->state = ADDR_PREFERRED;
#endif /* UIP_ND6_DEF_MAXDADNS > 0 */
    uip_create_solicited_node(ipaddr, &loc_fipaddr);
    uip_ds6_maddr_add(&loc_fipaddr);
    return locaddr;
  }
  return NULL;
}

/*---------------------------------------------------------------------------*/
void
uip_ds6_addr_rm(uip_ds6_addr_t *addr)
{
  if(addr != NULL) {
    uip_create_solicited_node(&addr->ipaddr, &loc_fipaddr);
    if((locmaddr = uip_ds6_maddr_lookup(&loc_fipaddr)) != NULL) {
      uip_ds6_maddr_rm(locmaddr);
    }
    addr->isused = 0;
  }
  return;
}

/*---------------------------------------------------------------------------*/
uip_ds6_addr_t *
uip_ds6_addr_lookup(uip_ipaddr_t *ipaddr)
{
  if(uip_ds6_list_loop
     ((uip_ds6_element_t *)uip_ds6_if.addr_list, UIP_DS6_ADDR_NB,
      sizeof(uip_ds6_addr_t), ipaddr, 128,
      (uip_ds6_element_t **)&locaddr) == FOUND) {
    return locaddr;
  }
  return NULL;
}

/*---------------------------------------------------------------------------*/
/*
 * get a link local address -
 * state = -1 => any address is ok. Otherwise state = desired state of addr.
 * (TENTATIVE, PREFERRED, DEPRECATED)
 */
uip_ds6_addr_t *
uip_ds6_get_link_local(int8_t state)
{
  for(locaddr = uip_ds6_if.addr_list;
      locaddr < uip_ds6_if.addr_list + UIP_DS6_ADDR_NB; locaddr++) {
    if(locaddr->isused && (state == -1 || locaddr->state == state)
       && (uip_is_addr_linklocal(&locaddr->ipaddr))) {
      return locaddr;
    }
  }
  return NULL;
}

/*---------------------------------------------------------------------------*/
/*
 * get a global address -
 * state = -1 => any address is ok. Otherwise state = desired state of addr.
 * (TENTATIVE, PREFERRED, DEPRECATED)
 */
uip_ds6_addr_t *
uip_ds6_get_global(int8_t state)
{
  for(locaddr = uip_ds6_if.addr_list;
      locaddr < uip_ds6_if.addr_list + UIP_DS6_ADDR_NB; locaddr++) {
    if(locaddr->isused && (state == -1 || locaddr->state == state)
       && !(uip_is_addr_linklocal(&locaddr->ipaddr))) {
      return locaddr;
    }
  }
  return NULL;
}

/*---------------------------------------------------------------------------*/
uip_ds6_maddr_t *
uip_ds6_maddr_add(const uip_ipaddr_t *ipaddr)
{
  if(uip_ds6_list_loop
     ((uip_ds6_element_t *)uip_ds6_if.maddr_list, UIP_DS6_MADDR_NB,
      sizeof(uip_ds6_maddr_t), (void*)ipaddr, 128,
      (uip_ds6_element_t **)&locmaddr) == FREESPACE) {
    locmaddr->isused = 1;
    uip_ipaddr_copy(&locmaddr->ipaddr, ipaddr);
    return locmaddr;
  }
  return NULL;
}

/*---------------------------------------------------------------------------*/
void
uip_ds6_maddr_rm(uip_ds6_maddr_t *maddr)
{
  if(maddr != NULL) {
    maddr->isused = 0;
  }
  return;
}

/*---------------------------------------------------------------------------*/
uip_ds6_maddr_t *
uip_ds6_maddr_lookup(const uip_ipaddr_t *ipaddr)
{
  if(uip_ds6_list_loop
     ((uip_ds6_element_t *)uip_ds6_if.maddr_list, UIP_DS6_MADDR_NB,
      sizeof(uip_ds6_maddr_t), (void*)ipaddr, 128,
      (uip_ds6_element_t **)&locmaddr) == FOUND) {
    return locmaddr;
  }
  return NULL;
}


/*---------------------------------------------------------------------------*/
uip_ds6_aaddr_t *
uip_ds6_aaddr_add(uip_ipaddr_t *ipaddr)
{
#if UIP_DS6_AADDR_NB
  if(uip_ds6_list_loop
     ((uip_ds6_element_t *)uip_ds6_if.aaddr_list, UIP_DS6_AADDR_NB,
      sizeof(uip_ds6_aaddr_t), ipaddr, 128,
      (uip_ds6_element_t **)&locaaddr) == FREESPACE) {
    locaaddr->isused = 1;
    uip_ipaddr_copy(&locaaddr->ipaddr, ipaddr);
    return locaaddr;
  }
#endif /* UIP_DS6_AADDR_NB */
  return NULL;
}

/*---------------------------------------------------------------------------*/
void
uip_ds6_aaddr_rm(uip_ds6_aaddr_t *aaddr)
{
  if(aaddr != NULL) {
    aaddr->isused = 0;
  }
  return;
}

/*---------------------------------------------------------------------------*/
uip_ds6_aaddr_t *
uip_ds6_aaddr_lookup(uip_ipaddr_t *ipaddr)
{
#if UIP_DS6_AADDR_NB
  if(uip_ds6_list_loop((uip_ds6_element_t *)uip_ds6_if.aaddr_list,
                       UIP_DS6_AADDR_NB, sizeof(uip_ds6_aaddr_t), ipaddr, 128,
                       (uip_ds6_element_t **)&locaaddr) == FOUND) {
    return locaaddr;
  }
#endif /* UIP_DS6_AADDR_NB */
  return NULL;
}

/*---------------------------------------------------------------------------*/
void
uip_ds6_select_src(uip_ipaddr_t *src, uip_ipaddr_t *dst)
{
  uint8_t best = 0;             /* number of bit in common with best match */
  uint8_t n = 0;
  uip_ds6_addr_t *matchaddr = NULL;

  if(!uip_is_addr_linklocal(dst) && !uip_is_addr_mcast(dst)) {
    /* find longest match */
    for(locaddr = uip_ds6_if.addr_list;
        locaddr < uip_ds6_if.addr_list + UIP_DS6_ADDR_NB; locaddr++) {
      /* Only preferred global (not link-local) addresses */
      if(locaddr->isused && locaddr->state == ADDR_PREFERRED &&
         !uip_is_addr_linklocal(&locaddr->ipaddr)) {
        n = get_match_length(dst, &locaddr->ipaddr);
        if(n >= best) {
          best = n;
          matchaddr = locaddr;
        }
      }
    }
#if UIP_IPV6_MULTICAST
  } else if(uip_is_addr_mcast_routable(dst)) {
    matchaddr = uip_ds6_get_global(ADDR_PREFERRED);
#endif
  } else {
    matchaddr = uip_ds6_get_link_local(ADDR_PREFERRED);
  }

  /* use the :: (unspecified address) as source if no match found */
  if(matchaddr == NULL) {
    uip_create_unspecified(src);
  } else {
    uip_ipaddr_copy(src, &matchaddr->ipaddr);
  }
}

/*---------------------------------------------------------------------------*/
void
uip_ds6_set_addr_iid(uip_ipaddr_t *ipaddr, uip_lladdr_t *lladdr)
{
#if (UIP_LLADDR_LEN == 8)
  memcpy(ipaddr->u8 + 8, lladdr, UIP_LLADDR_LEN);
  ipaddr->u8[8] ^= 0x02;
#elif (UIP_LLADDR_LEN == 6)
  memcpy(ipaddr->u8 + 8, lladdr, 3);
  ipaddr->u8[11] = 0xff;
  ipaddr->u8[12] = 0xfe;
  memcpy(ipaddr->u8 + 13, (uint8_t *)lladdr + 3, 3);
  ipaddr->u8[8] ^= 0x02;
#elif (UIP_LLADDR_LEN == 2)
  /* derive IID as per RFC 6282 */
  memcpy(ipaddr->u8 + 8, iid_prefix, 6);
  memcpy(ipaddr->u8 + 8 + 6, lladdr, UIP_LLADDR_LEN);
#else
#error uip-ds6.c cannot build interface address when UIP_LLADDR_LEN is not 6, 8, or 2
#endif
}
/*---------------------------------------------------------------------------*/
void
uip_ds6_set_lladdr_from_iid(uip_lladdr_t *lladdr, const uip_ipaddr_t *ipaddr)
{
#if (UIP_LLADDR_LEN == 8)
  memcpy(lladdr, ipaddr->u8 + 8, UIP_LLADDR_LEN);
  lladdr->addr[0] ^= 0x02;
#elif (UIP_LLADDR_LEN == 2)
  memcpy(lladdr, ipaddr->u8 + 6, UIP_LLADDR_LEN);
#else
#error uip-ds6.c cannot build lladdr address when UIP_LLADDR_LEN is not 8 or 2
#endif
}

/*---------------------------------------------------------------------------*/
uint8_t
get_match_length(uip_ipaddr_t *src, uip_ipaddr_t *dst)
{
  uint8_t j, k, x_or;
  uint8_t len = 0;

  for(j = 0; j < 16; j++) {
    if(src->u8[j] == dst->u8[j]) {
      len += 8;
    } else {
      x_or = src->u8[j] ^ dst->u8[j];
      for(k = 0; k < 8; k++) {
        if((x_or & 0x80) == 0) {
          len++;
          x_or <<= 1;
        } else {
          break;
        }
      }
      break;
    }
  }
  return len;
}

/*---------------------------------------------------------------------------*/
#if UIP_ND6_DEF_MAXDADNS > 0
void
uip_ds6_dad(uip_ds6_addr_t *addr)
{
  /* send maxdadns NS for DAD  */
  if(addr->dadnscount < uip_ds6_if.maxdadns) {
    uip_nd6_ns_output(NULL, NULL, &addr->ipaddr);
    addr->dadnscount++;
    timer_set(&addr->dadtimer, uip_ds6_if.retrans_timer / 1000 * CLOCK_SECOND);
    return;
  }
  /*
   * If we arrive here it means DAD succeeded, otherwise the dad process
   * would have been interrupted in ds6_dad_ns/na_input
   */
  TRiceS(iD(6713), "msg:DAD succeeded, ipaddr: %s\n", uip6_printAddr(&addr->ipaddr, NULL));

  addr->state = ADDR_PREFERRED;
  return;
}

/*---------------------------------------------------------------------------*/
/*
 * Calling code must handle when this returns 0 (e.g. link local
 * address can not be used).
 */
int
uip_ds6_dad_failed(uip_ds6_addr_t *addr)
{
  if(uip_is_addr_linklocal(&addr->ipaddr)) {
	  TRice(iD(1905), "err:Contiki shutdown, DAD for link local address failed\n");
    return 0;
  }
  uip_ds6_addr_rm(addr);
  return 1;
}
#endif /*UIP_ND6_DEF_MAXDADNS > 0 */

/*---------------------------------------------------------------------------*/
#if UIP_CONF_ROUTER
#if UIP_ND6_SEND_RA
void
uip_ds6_send_ra_sollicited(void)
{
  /* We have a pb here: RA timer max possible value is 1800s,
   * hence we have to use stimers. However, when receiving a RS, we
   * should delay the reply by a random value between 0 and 500ms timers.
   * stimers are in seconds, hence we cannot do this. Therefore we just send
   * the RA (setting the timer to 0 below). We keep the code logic for
   * the days contiki will support appropriate timers */
  rand_time = 0;
  TRice(iD(6977), "msg:Solicited RA, random time %u\n", rand_time);

  if(Time_TimerRemaining(&uip_ds6_timer_ra) > rand_time) {
    if(Time_TimerElapsed(&uip_ds6_timer_ra) < UIP_ND6_MIN_DELAY_BETWEEN_RAS) {
      /* Ensure that the RAs are rate limited */
/*      Time_TimerSet(&uip_ds6_timer_ra, rand_time +
                 UIP_ND6_MIN_DELAY_BETWEEN_RAS -
                 Time_TimerElapsed(&uip_ds6_timer_ra));
  */ } else {
	  Time_TimerSet(&uip_ds6_timer_ra, rand_time);
    }
  }
}

/*---------------------------------------------------------------------------*/
void
uip_ds6_send_ra_periodic(void)
{
  if(racount > 0) {
    /* send previously scheduled RA */
    uip_nd6_ra_output(NULL);
    TRice(iD(7091), "msg:Sending periodic RA\n");
  }

  rand_time = UIP_ND6_MIN_RA_INTERVAL + random_rand() %
    (uint16_t) (UIP_ND6_MAX_RA_INTERVAL - UIP_ND6_MIN_RA_INTERVAL);
  TRice(iD(4040), "dbg:Random time 1 = %u\n", rand_time);

  if(racount < UIP_ND6_MAX_INITIAL_RAS) {
    if(rand_time > UIP_ND6_MAX_INITIAL_RA_INTERVAL) {
      rand_time = UIP_ND6_MAX_INITIAL_RA_INTERVAL;
      TRice(iD(4005), "dbg:Random time 2 = %u\n", rand_time);
    }
    racount++;
  }
  TRice(iD(1439), "dbg:Random time 3 = %u\n", rand_time);
  Time_TimerSet(&uip_ds6_timer_ra, rand_time);
}

#endif /* UIP_ND6_SEND_RA */
#endif /* UIP_CONF_ROUTER */
/*---------------------------------------------------------------------------*/
uint8_t Ds6_GetHopLimit(void) {
	return uip_ds6_if.cur_hop_limit;
}

void Ds6_SetHopLimit(const uint8_t ttl) {
	if (0 != ttl) {
	    uip_ds6_if.cur_hop_limit = ttl;
	    TRice(iD(6112), "msg:[uIP DS6] Hop limit set to - %u\n", uip_ds6_if.cur_hop_limit);
	}
}

uint32_t Ds6_GetRetransmitTmoInMs(void) {
	return uip_ds6_if.retrans_timer;
}

void Ds6_SetReachableTimes(const uint32_t baseReachTime) {
    if(baseReachTime != uip_ds6_if.base_reachable_time) {
      uip_ds6_if.base_reachable_time = baseReachTime;
      uip_ds6_if.reachable_time = uip_ds6_compute_reachable_time(baseReachTime);
	    TRice(iD(5591), "msg:[uIP DS6] Reachable time updated - base(%u), my(%d)\n", uip_ds6_if.base_reachable_time, uip_ds6_if.reachable_time);
    }
}

void Ds6_SetRetransmitTim(sUipBuff *uipBuff, const uint32_t retransTmo) {
  if(0 != (uip_nd6_ra *)(uipBuff->buff.u8 + UIP_IPH_LEN + UIP_ICMPH_LEN + uipBuff->extLen)->retrans_timer) {
    uip_ds6_if.retrans_timer = retransTmo;
    TRice(iD(6951), "msg:[uIP DS6] Retransmit timeout set to - %u\n", uip_ds6_if.retrans_timer);
  }
}

/*---------------------------------------------------------------------------*/

/** @}*/
