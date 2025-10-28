/*
 * Copyright (c) 2004, Swedish Institute of Computer Science.
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
 */

/**
 * \file
 *         Core of the TCP/IP stack, handles input/output/routing
 *
 * \author  Adam Dunkels <adam@sics.se>\author
 * \author  Mathilde Durvy <mdurvy@cisco.com> (IPv6 related code)
 * \author  Julien Abeille <jabeille@cisco.com> (IPv6 related code)
 */

#include <string.h>
#include "tcpip.h"
#include "uip-ds6.h"
#include "uip-ds6-nbr.h"
#include "uip-ds6-route.h"
#include "uip-nd6.h"
#include "../addressing.h"
#include "../routing/routing.h"
#include "sicslowpan.h"
#include "uipopt.h"
#include "cmsis_os.h"
#include "../evt_radio.h"
#include "App/common.h"
#include "../mac/framer/frame802154.h"



#ifdef UIP_FALLBACK_INTERFACE
extern struct uip_fallback_interface UIP_FALLBACK_INTERFACE;
#endif

#if UIP_CONF_ICMP6
process_event_t tcpip_icmp6_event;
#endif /* UIP_CONF_ICMP6 */

#if UIP_TCP
#warning "for now lets disable TPC/IP - work with udp only"
/**
 * \internal Structure for holding a TCP port and a process ID.
 */
struct listenport {
  uint16_t port;
  struct process *p;
};

static struct internal_state {
  struct listenport listenports[UIP_LISTENPORTS];
  struct process *p;
} s;

/* Periodic check of active connections. */
static TimerHandle_t periodicTim;
#endif

static uint16_t tcpipEvtIdOffset;
void (*tcpipIrq2Task)(uint16_t, void(*cbFunc)(void));
static sUipBuff rxBuff;
/*---------------------------------------------------------------------------*/

uint8_t tcpip_output(sUipBuff *tcpUipBuff, const uip_lladdr_t *addr) {
  int ret;

  /* Tag Traffic Class if we are using TC for variable retrans */
#if UIP_TAG_TC_WITH_VARIABLE_RETRANSMISSIONS
  if(uipbuf_get_attr(tcpUipBuff, UIPBUF_ATTR_MAX_MAC_TRANSMISSIONS) != UIP_MAX_MAC_TRANSMISSIONS_UNDEFINED) {
	  TRice("msg:Tagging TC with retrans: %d\n", uipbuf_get_attr(tcpUipBuff, UIPBUF_ATTR_MAX_MAC_TRANSMISSIONS));
    /* Encapsulate the MAC transmission limit in the Traffic Class field */
	IP_HDR_CAST_TO_BUFF(tcpUipBuff->buff.u8)->vtc = 0x60 | (UIP_TC_MAC_TRANSMISSION_COUNTER_BIT >> 4);
    IP_HDR_CAST_TO_BUFF(tcpUipBuff->buff.u8)->tcflow = uipbuf_get_attr(tcpUipBuff, UIPBUF_ATTR_MAX_MAC_TRANSMISSIONS) << 4;
  }
#endif
#warning "netstack.c/h is for packet filtering, firewall or other functionality which is not needed for now"
  if(1/*netstack_process_ip_callback(NETSTACK_IP_OUTPUT, addr) == NETSTACK_IP_PROCESS*/) {
#warning "uip_lladdr_t >>> linkaddr_t - why??? maybe it is same stuff?"
    ret = sicslowpan_driver.output(tcpUipBuff, (const linkaddr_t *)addr);
    return ret;
  } else {
    /* Ok, ignore and drop... */
    uipbuf_clear(tcpUipBuff);
    return 0;
  }
}

/*---------------------------------------------------------------------------*/
#if UIP_TCP
static void start_periodic_tcp_timer(void) {
  if(pdFALSE == xTimerIsTimerActive(periodicTim)) {
	xTimerStart(periodicTim, 0);
  }
}
#endif /* UIP_TCP */

/*---------------------------------------------------------------------------*/
static void check_for_tcp_syn(void) {
#if UIP_TCP
  /* This is a hack that is needed to start the periodic TCP timer if
     an incoming packet contains a SYN: since uIP does not inform the
     application if a SYN arrives, we have no other way of starting
     this timer.  This function is called for every incoming IP packet
     to check for such SYNs. */
#define TCP_SYN 0x02
  if(IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->proto == UIP_PROTO_TCP && (UIP_TCP_BUF->flags & TCP_SYN) == TCP_SYN) {
    start_periodic_tcp_timer();
  }
#endif /* UIP_TCP */
}

/*---------------------------------------------------------------------------*/
static uint16_t packet_input(sUipBuff *rxPacket) {
  uint16_t rxLen = 0;
  if(rxPacket->len > 0) {
	TRice("msg:input: received %u bytes\n", rxPacket->len);

    check_for_tcp_syn();

#if UIP_TAG_TC_WITH_VARIABLE_RETRANSMISSIONS
    {
      uint8_t traffic_class = (IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->vtc << 4) | (IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->tcflow >> 4);
      if(traffic_class & UIP_TC_MAC_TRANSMISSION_COUNTER_BIT) {
        uint8_t max_mac_transmissions = traffic_class & UIP_TC_MAC_TRANSMISSION_COUNTER_MASK;
        uipbuf_set_attr(UIPBUF_ATTR_MAX_MAC_TRANSMISSIONS, max_mac_transmissions);
        TRice("msg:Received packet tagged with TC retrans: %d (%x)",
                 max_mac_transmissions, traffic_class);
      }
    }
#endif /* UIP_TAG_TC_WITH_VARIABLE_RETRANSMISSIONS */

    uip_process(rxPacket, UIP_DATA);
    if(rxPacket->len > 0) {
      tcpip_ipv6_output(rxPacket);
    }
  }
  return rxLen;
}
/*---------------------------------------------------------------------------*/
#if UIP_TCP
#if UIP_ACTIVE_OPEN
struct uip_conn *
tcp_connect(const uip_ipaddr_t *ripaddr, uint16_t port, void *appstate)
{
  struct uip_conn *c;

  c = uip_connect(ripaddr, port);
  if(c == NULL) {
    return NULL;
  }

  //init_appstate(&c->appstate, appstate);

  tcpip_poll_tcp(c);

  return c;
}
#endif /* UIP_ACTIVE_OPEN */
/*---------------------------------------------------------------------------*/
void
tcp_unlisten(uint16_t port)
{
  unsigned char i;
  struct listenport *l;

  l = s.listenports;
  for(i = 0; i < UIP_LISTENPORTS; ++i) {
    if(l->port == port &&
       l->p == PROCESS_CURRENT()) {
      l->port = 0;
      uip_unlisten(port);
      break;
    }
    ++l;
  }
}
/*---------------------------------------------------------------------------*/
void
tcp_listen(uint16_t port)
{
  unsigned char i;
  struct listenport *l;

  l = s.listenports;
  for(i = 0; i < UIP_LISTENPORTS; ++i) {
    if(l->port == 0) {
      l->port = port;
      l->p = PROCESS_CURRENT();
      uip_listen(port);
      break;
    }
    ++l;
  }
}
///*---------------------------------------------------------------------------*/
//void
//tcp_attach(struct uip_conn *conn, void *appstate)
//{
//  //init_appstate(&conn->appstate, appstate);
//}
#endif /* UIP_TCP */
/*---------------------------------------------------------------------------*/
#if UIP_CONF_ICMP6
uint8_t icmp6_new(void *appstate) {
  if(uip_icmp6_conns.appstate.p == PROCESS_NONE) {
    //init_appstate(&uip_icmp6_conns.appstate, appstate);
    return 0;
  }
  return 1;
}

void
tcpip_icmp6_call(uint8_t type)
{
  if(uip_icmp6_conns.appstate.p != PROCESS_NONE) {
    /* XXX: This is a hack that needs to be updated. Passing a pointer (&type)
       like this only works with process_post_synch. */
    process_post_synch(uip_icmp6_conns.appstate.p, tcpip_icmp6_event, &type);
  }
  return;
}
#endif /* UIP_CONF_ICMP6 */
/*---------------------------------------------------------------------------*/
void tcpip_input(void)
{
  if (sicslowpan_driver.input(&rxBuff)) {
#warning "netstack.c/h is for packet filtering, firewall or other functionality which is not needed for now"
    if(1/*netstack_process_ip_callback(NETSTACK_IP_INPUT, NULL) == NETSTACK_IP_PROCESS*/) {
	  packet_input(&rxBuff);
    } /* else - do nothing and drop */
  //uipbuf_clear(); do not care - we clear it at start of reception. and now use different buffers for RX/TX and stuff.
  }
}
/*---------------------------------------------------------------------------*/
static void
output_fallback(sUipBuff *uipBuff)
{
#ifdef UIP_FALLBACK_INTERFACE
  uip_last_proto = *(uipBuff->buff.u8 + UIP_IPH_LEN);
  TRice("msg:fallback: removing ext hdrs & setting proto %d %d\n", uip_ext_len, uip_last_proto);
  uip_remove_ext_hdr(uipBuff);
  /* Inform the other end that the destination is not reachable. If it's
   * not informed routes might get lost unexpectedly until there's a need
   * to send a new packet to the peer */
  if(UIP_FALLBACK_INTERFACE.output() < 0) {
	  TRice("err:fallback: output error. Reporting DST UNREACH\n");
    uip_icmp6_error_output(ICMP6_DST_UNREACH, ICMP6_DST_UNREACH_ADDR, 0);
    uip_flags = 0;
    tcpip_ipv6_output();
    return;
  }
#else
  TRice("err:output: destination off-link and no default route\n");
#endif /* !UIP_FALLBACK_INTERFACE */
}
/*---------------------------------------------------------------------------*/
static void
annotate_transmission(const uip_ipaddr_t *nexthop)
{
#if TCPIP_CONF_ANNOTATE_TRANSMISSIONS
  static uint8_t annotate_last;
  static uint8_t annotate_has_last = 0;

  if(annotate_has_last) {
    printf("#L %u 0; red\n", annotate_last);
  }
  printf("#L %u 1; red\n", nexthop->u8[sizeof(uip_ipaddr_t) - 1]);
  annotate_last = nexthop->u8[sizeof(uip_ipaddr_t) - 1];
  annotate_has_last = 1;
#endif /* TCPIP_CONF_ANNOTATE_TRANSMISSIONS */
}
/*---------------------------------------------------------------------------*/
static const uip_ipaddr_t* get_nexthop(sUipBuff *uipBuff, uip_ipaddr_t *addr) {
  const uip_ipaddr_t *nexthop;
  uip_ds6_route_t *route;

  TRice("msg:output: processing %u bytes packet from ", uipBuff->len);
  TRiceS("msg:%s\n", uip6_printAddr(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->srcipaddr, NULL));
  TRiceS("msg: to %s\n", uip6_printAddr(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->destipaddr, NULL));

  if(rpl_lite_driver.ext_header_srh_get_next_hop(uipBuff, addr)) {
    TRiceS("msg:output: selected next hop from SRH: %s\n", uip6_printAddr(addr, NULL));
    return addr;
  }

  /* We first check if the destination address is on our immediate
     link. If so, we simply use the destination address as our
     nexthop address. */
  if(uip_ds6_is_addr_onlink(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->destipaddr)) {
	  TRice("msg:output: destination is on link\n");
    return &IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->destipaddr;
  }

  /* Check if we have a route to the destination address. */
  route = uip_ds6_route_lookup(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->destipaddr);

  /* No route was found - we send to the default route instead. */
  if(route == NULL) {
    nexthop = uip_ds6_defrt_choose();
    if(nexthop == NULL) {
      output_fallback(uipBuff);
    } else {
      TRiceS("msg:output: no route found, using default route: %s\n", uip6_printAddr(nexthop, NULL));
    }

  } else {
    /* A route was found, so we look up the nexthop neighbor for
       the route. */
    nexthop = uip_ds6_route_nexthop(route);

    /* If the nexthop is dead, for example because the neighbor
       never responded to link-layer acks, we drop its route. */
    if(nexthop == NULL) {
    	TRice("err:output: found dead route\n");
      /* Notifiy the routing protocol that we are about to remove the route */
      rpl_lite_driver.drop_route(route);
      /* Remove the route */
      uip_ds6_route_rm(route);
      /* We don't have a nexthop to send the packet to, so we drop it. */
    } else {
      TRiceS("msg:output: found next hop from routing table: %s\n", uip6_printAddr(nexthop, NULL));
    }
  }

  return nexthop;
}
/*---------------------------------------------------------------------------*/
#if UIP_ND6_SEND_NS
static int
queue_packet(uip_ds6_nbr_t *nbr)
{
  /* Copy outgoing pkt in the queuing buffer for later transmit. */
#if UIP_CONF_IPV6_QUEUE_PKT
  if(uip_packetqueue_alloc(&nbr->packethandle, UIP_DS6_NBR_PACKET_LIFETIME) != NULL) {
    memcpy(uip_packetqueue_buf(&nbr->packethandle), UIP_IP_BUF, uip_len);
    uip_packetqueue_set_buflen(&nbr->packethandle, uip_len);
    return 0;
  }
#endif

  return 1;
}
#endif
/*---------------------------------------------------------------------------*/
static void
send_queued(uip_ds6_nbr_t *nbr)
{
#if UIP_CONF_IPV6_QUEUE_PKT
  /*
   * Send the queued packets from here, may not be 100% perfect though.
   * This happens in a few cases, for example when instead of receiving a
   * NA after sendiong a NS, you receive a NS with SLLAO: the entry moves
   * to STALE, and you must both send a NA and the queued packet.
   */
  if(uip_packetqueue_buflen(&nbr->packethandle) != 0) {
    uip_len = uip_packetqueue_buflen(&nbr->packethandle);
    memcpy(UIP_IP_BUF, uip_packetqueue_buf(&nbr->packethandle), uip_len);
    uip_packetqueue_free(&nbr->packethandle);
    tcpip_output(uip_ds6_nbr_get_ll(nbr));
  }
#endif /*UIP_CONF_IPV6_QUEUE_PKT*/
}
/*---------------------------------------------------------------------------*/
static int
send_nd6_ns(sUipBuff *uipBuff, const uip_ipaddr_t *nexthop)
{
  int err = 1;

#if UIP_ND6_SEND_NS
   uip_ds6_nbr_t *nbr = NULL;
  if((nbr = uip_ds6_nbr_add(nexthop, NULL, 0, NBR_INCOMPLETE, NBR_TABLE_REASON_IPV6_ND, NULL)) != NULL) {
    err = 0;

    queue_packet(nbr);
  /* RFC4861, 7.2.2:
   * "If the source address of the packet prompting the solicitation is the
   * same as one of the addresses assigned to the outgoing interface, that
   * address SHOULD be placed in the IP Source Address of the outgoing
   * solicitation.  Otherwise, any one of the addresses assigned to the
   * interface should be used."*/
   if(uip_ds6_is_my_addr(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->srcipaddr)){
      uip_nd6_ns_output(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->srcipaddr, NULL, &nbr->ipaddr);
    } else {
      uip_nd6_ns_output(NULL, NULL, &nbr->ipaddr);
    }

   Time_TimerSet(&nbr->sendns, Ds6_GetRetransmitTmoInMs() / 1000);
    nbr->nscount = 1;
    /* Send the first NS try from here (multicast destination IP address). */
  }
#else
  TRiceS("err:output: neighbor not in cache: %s\n", uip6_printAddr(nexthop, NULL));
#endif

  return err;
}
/*---------------------------------------------------------------------------*/
void tcpip_ipv6_output(sUipBuff *uipBuff)
{
  uip_ipaddr_t ipaddr;
  uip_ds6_nbr_t *nbr = NULL;
  const uip_lladdr_t *linkaddr;
  const uip_ipaddr_t *nexthop;

  if(uipBuff->len == 0) {
    return;
  }

  if(uipBuff->len > UIP_LINK_MTU) {
	  TRice("err:output: Packet too big");
    goto exit;
  }

  if(uip_is_addr_unspecified(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->destipaddr)){
	  TRice("err:output: Destination address unspecified");
    goto exit;
  }


  if(!rpl_lite_driver.ext_header_update(uipBuff)) {
    /* Packet can not be forwarded */
	  TRice("err:output: routing protocol extension header update error\n");
    uipbuf_clear(uipBuff);
    return;
  }

  if(uip_is_addr_mcast(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->destipaddr)) {
    linkaddr = NULL;
    goto send_packet;
  }

  /* We first check if the destination address is one of ours. There is no
   * loopback interface -- instead, process this directly as incoming. */
  if(uip_ds6_is_my_addr(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->destipaddr)) {
	  TRice("msg:output: sending to ourself\n");
    packet_input(uipBuff);
    return;
  }

  /* Look for a next hop */
  if((nexthop = get_nexthop(uipBuff, &ipaddr)) == NULL) {
    goto exit;
  }
  annotate_transmission(nexthop);

  nbr = uip_ds6_nbr_lookup(nexthop);
#if UIP_ND6_AUTOFILL_NBR_CACHE
  if(nbr == NULL) {
    /* Neighbor not found in cache? Derive its link-layer address from it's
    link-local IPv6, assuming it used autoconfiguration. This is not
    standard-compliant but this is a convenient way to keep the
    neighbor cache out of the way in cases ND is not used */
    uip_lladdr_t lladdr;
    Addr_GetInterfId(&lladdr, nexthop);
    if((nbr = uip_ds6_nbr_add(nexthop, &lladdr, 0, NBR_REACHABLE, NBR_TABLE_REASON_IPV6_ND_AUTOFILL, NULL)) == NULL) {
      TRiceS("err:output: failed to autofill neighbor cache for host %s", uip6_printAddr(nexthop, NULL));
      TRiceS("err:, link-layer addr  %s\n", (char*)linkaddr_printAddr((linkaddr_t*)&lladdr));
      goto exit;
    }
   }
#endif /* UIP_ND6_AUTOFILL_NBR_CACHE */

  if(nbr == NULL) {
    if(send_nd6_ns(uipBuff, nexthop)) {
    	TRice("err:output: failed to add neighbor to cache\n");
      goto exit;
    } else {
      /* We're sending NS here instead of original packet */
      goto send_packet;
    }
  }

#if UIP_ND6_SEND_NS
  if(nbr->state == NBR_INCOMPLETE) {
	  TRice("err:output: nbr cache entry incomplete\n");
    queue_packet(nbr);
    goto exit;
  }
  /* Send in parallel if we are running NUD (nbc state is either STALE,
     DELAY, or PROBE). See RFC 4861, section 7.3.3 on node behavior. */
  if(nbr->state == NBR_STALE) {
    nbr->state = NBR_DELAY;
    Time_TimerSet(&nbr->reachable, UIP_ND6_DELAY_FIRST_PROBE_TIME);
    nbr->nscount = 0;
    TRice("msg:output: nbr cache entry stale moving to delay\n");
  }
#endif /* UIP_ND6_SEND_NS */

send_packet:
  if(nbr) {
    linkaddr = uip_ds6_nbr_get_ll(nbr);
  } else {
    linkaddr = NULL;
  }

  TRiceS("msg:output: sending to %s\n", (char*)linkaddr_printAddr((linkaddr_t*)linkaddr));
  tcpip_output(uipBuff, linkaddr);

  if(nbr) {
    send_queued(nbr);
  }

exit:
  uipbuf_clear(uipBuff);
  return;
}

sUipBuff uipPollBuff;
/*---------------------------------------------------------------------------*/
#if UIP_UDP
static struct uip_udp_conn *pollUdpConn = NULL;
static void PollUdp(void) {
    if(NULL != pollUdpConn) {
      servicingUdpConn = pollUdpConn;
      uip_process(&uipPollBuff, UIP_UDP_TIMER);
      tcpip_ipv6_output(&uipPollBuff);
    }
    pollUdpConn = NULL;
}

void tcpip_poll_udp(struct uip_udp_conn *conn) {
	pollUdpConn = conn;
	tcpipIrq2Task(tcpipEvtIdOffset + radio_taskCall, PollUdp);
}
#endif /* UIP_UDP */
/*---------------------------------------------------------------------------*/
#if UIP_TCP
static struct uip_conn *pollTcpConn = NULL;
static void PollTcp(void) {
    if(NULL != pollTcpConn) {
      uip_conn = pollTcpConn;
      uip_process(&uipPollBuff, UIP_POLL_REQUEST);
      tcpip_ipv6_output(&uipPollBuff);
      /* Start the periodic polling, if it isn't already active. */
      start_periodic_tcp_timer();
    }
    pollTcpConn = NULL;
}

void tcpip_poll_tcp(struct uip_conn *conn) {
	pollTcpConn = conn;
	tcpipIrq2Task(tcpipEvtIdOffset + radio_taskCall, PollTcp);
}
#endif /* UIP_TCP */
/*---------------------------------------------------------------------------*/
void tcpip_uipcall(void) {
//  uip_udp_appstate_t *ts;
//
//#if UIP_UDP
//  if(uip_conn != NULL) {
//    ts = &uip_conn->appstate;
//  } else {
//    ts = &uip_udp_conn->appstate;
//  }
//#else /* UIP_UDP */
//  ts = &uip_conn->appstate;
//#endif /* UIP_UDP */

#if UIP_TCP
  {
    unsigned char i;
    struct listenport *l;

    /* If this is a connection request for a listening port, we must
      mark the connection with the right process ID. */
    if(uip_connected()) {
      l = &s.listenports[0];
      for(i = 0; i < UIP_LISTENPORTS; ++i) {
        if(l->port == uip_conn->lport &&
            l->p != PROCESS_NONE) {
          ts->p = l->p;
          ts->state = NULL;
          break;
        }
        ++l;
      }

      /* Start the periodic polling, if it isn't already active. */
      start_periodic_tcp_timer();
    }
  }
#endif /* UIP_TCP */

//  if(ts->p != NULL) {
//    process_post_synch(ts->p, tcpip_event, ts->state);
//  }
}
/*---------------------------------------------------------------------------*/
#if UIP_TCP
static void HandleTcpipPeriodicTimer(TimerHandle_t periodicTim) {
	uint8_t i = UIP_TCP_CONNS;
	uint8_t noActiveConn = true;
    while(i) {
      i--;
      if(uip_conn_active(i)) {
        /* Only restart the timer if there are active
               connections. */
        uip_periodic(i);
        tcpip_ipv6_output();
        noActiveConn = false;
      }
    }
    if (noActiveConn) {
    	xTimerStop(periodicTim, 0);
    }
}
#endif

void tcpip_init(uint16_t evtOffset, void (*packedEvtHndl)(uint16_t, void(*)(void))) {
	linkaddr_t linkAddr;
	uip_ds6_addr_t *localLinkInfo;
	tcpipEvtIdOffset = evtOffset;
	tcpipIrq2Task = packedEvtHndl;
#if UIP_TCP
	  periodicTim = xTimerCreate("tcpipPeriodicTimer", pdMS_TO_TICKS(500), pdTRUE, 0, HandleTcpipPeriodicTimer);
	  xTimerStart(periodicTim, 0);

	  memset(s.listenports, 0, UIP_LISTENPORTS*sizeof(*(s.listenports)));
	  s.p = PROCESS_CURRENT();
#endif

#ifdef UIP_FALLBACK_INTERFACE
  UIP_FALLBACK_INTERFACE.init();
#endif
  sicslowpan_driver.init(evtOffset, packedEvtHndl);
  /* Initialize routing protocol */
  uip_init();
  rpl_lite_driver.init(evtOffset, packedEvtHndl);

#warning "mesh root is started here manualy - this should be under some logic done automaticaly"
  //rpl_lite_driver.root_start();

  linkaddr_get_node_addr(&linkAddr);
  localLinkInfo = uip_ds6_get_link_local(-1);

  TRice("info:Starting 6lowpan with:\n - 802.15.4 PANID: 0x%04x\n", IEEE802154_PANID);
  TRiceS("info: - Link-layer address: %s\n", (char*)linkaddr_printAddr(&linkAddr));
  TRiceS("info: - Tentative link-local IPv6 address: %s\n", uip6_printAddr((NULL != localLinkInfo) ? (&localLinkInfo->ipaddr) : NULL, NULL));
}

void tcpip_deinit() {
    /* This is the event we get if a process has exited. We go through
         the TCP/IP tables to see if this process had any open
         connections or listening TCP ports. If so, we'll close those
         connections. */

//    p = (struct process *)data;
	uip_deinit();
#if UIP_TCP
    l = s.listenports;
    for(i = 0; i < UIP_LISTENPORTS; ++i) {
      if(l->p == p) {
        uip_unlisten(l->port);
        l->port = 0;
        l->p = PROCESS_NONE;
      }
      ++l;
    }

    {
      struct uip_conn *cptr;

      for(cptr = &uip_conns[0]; cptr < &uip_conns[UIP_TCP_CONNS]; ++cptr) {
        if(cptr->appstate.p == p) {
          cptr->appstate.p = PROCESS_NONE;
          cptr->tcpstateflags = UIP_CLOSED;
        }
      }
    }
#endif /* UIP_TCP */
}
/*---------------------------------------------------------------------------*/
