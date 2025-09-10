/*
 * Copyright (c) 2009, Swedish Institute of Computer Science.
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
 */

/**
 * \file
 *         MAC framer for IEEE 802.15.4
 * \author
 *         Niclas Finne <nfi@sics.se>
 *         Joakim Eriksson <joakime@sics.se>
 */

/* Includes -----------------------------------------------------------------*/
#include "framer-802154.h"
#include "../llsec802154.h"
#include <string.h>
#if defined(STM32H753xx)
#include "trice.h"
#else
#include "App/common.h"
#endif

/* Private defines ----------------------------------------------------------*/
/* Private types ------------------------------------------------------------*/
/* Pseudo global variables --------------------------------------------------*/
/* Private functions --------------------------------------------------------*/
/**
 *
 */
static void framer_802154_setup_params(sPacket* packet, frame802154_t *params) {
  if (params == NULL) {
	TRice("wrn:framer-802154: cannot setup params because of invalid argument\n");
    return;
  }

  /*
   * Don't initialize params with 0 because a caller may have already set
   * something to it
   */

  /* Build the FCF. */
  params->fcf.frame_type = packetbuf_attr(packet, PACKETBUF_ATTR_FRAME_TYPE);
  params->fcf.frame_pending = 0;
  if(packetbuf_holds_broadcast(packet)) {
    params->fcf.ack_required = 0;
    /* Suppress seqno on broadcast if supported (frame v2 or more) */
    params->fcf.sequence_number_suppression = FRAME802154_VERSION >= FRAME802154_IEEE802154_2015;
  } else {
    params->fcf.ack_required = packetbuf_attr(packet, PACKETBUF_ATTR_MAC_ACK);
    params->fcf.sequence_number_suppression = FRAME802154_SUPPR_SEQNO;
  }

  /* Set IE Present bit */
  params->fcf.ie_list_present = packetbuf_attr(packet, PACKETBUF_ATTR_MAC_METADATA);

  /* Insert IEEE 802.15.4 version bits. */
  params->fcf.frame_version = FRAME802154_VERSION;

#if LLSEC802154_USES_AUX_HEADER
  if(get_attr(packet, PACKETBUF_ATTR_SECURITY_LEVEL)) {
    params->fcf.security_enabled = 1;
  }
  /* Setting security-related attributes */
  params->aux_hdr.security_control.security_level = get_attr(packet, PACKETBUF_ATTR_SECURITY_LEVEL);
#if LLSEC802154_USES_FRAME_COUNTER
  params->aux_hdr.frame_counter.u16[0] = get_attr(packet, PACKETBUF_ATTR_FRAME_COUNTER_BYTES_0_1);
  params->aux_hdr.frame_counter.u16[1] = get_attr(packet, PACKETBUF_ATTR_FRAME_COUNTER_BYTES_2_3);
#else /* LLSEC802154_USES_FRAME_COUNTER */
  params->aux_hdr.security_control.frame_counter_suppression = 1;
  params->aux_hdr.security_control.frame_counter_size = 1;
#endif /* LLSEC802154_USES_FRAME_COUNTER */
#if LLSEC802154_USES_EXPLICIT_KEYS
  params->aux_hdr.security_control.key_id_mode = get_attr(packet, PACKETBUF_ATTR_KEY_ID_MODE);
  params->aux_hdr.key_index = get_attr(packet, PACKETBUF_ATTR_KEY_INDEX);
#endif /* LLSEC802154_USES_EXPLICIT_KEYS */
#else
  params->fcf.security_enabled = 0;
#endif /* LLSEC802154_USES_AUX_HEADER */

  params->seq = packetbuf_attr(packet, PACKETBUF_ATTR_MAC_SEQNO);

  /* Set the source PAN ID to the global variable. */
  params->src_pid = frame802154_get_pan_id();

  /* Source address itself should be set outside this function. */
  if(packetbuf_attr(packet, PACKETBUF_ATTR_MAC_NO_SRC_ADDR) == 1) {
    params->fcf.src_addr_mode = FRAME802154_NOADDR;
  } else {
    if(LINKADDR_SIZE == 2) {
      /* Use short address mode if linkaddr size is short. */
      params->fcf.src_addr_mode = FRAME802154_SHORTADDRMODE;
    } else {
      params->fcf.src_addr_mode = FRAME802154_LONGADDRMODE;
    }
  }

  params->dest_pid = frame802154_get_pan_id();

  /* Destination address itself should be set outside this function. */
  if(packetbuf_attr(packet, PACKETBUF_ATTR_MAC_NO_DEST_ADDR) == 1) {
    params->fcf.dest_addr_mode = FRAME802154_NOADDR;
  } else if(packetbuf_holds_broadcast(packet)) {
    /* Broadcast requires short address mode. */
    params->fcf.dest_addr_mode = FRAME802154_SHORTADDRMODE;
  } else {
    if(LINKADDR_SIZE == 2) {
      params->fcf.dest_addr_mode = FRAME802154_SHORTADDRMODE;
    } else {
      params->fcf.dest_addr_mode = FRAME802154_LONGADDRMODE;
    }
  }

  /* Suppress Source PAN ID and put Destination PAN ID by default */
  if(params->fcf.src_addr_mode == FRAME802154_SHORTADDRMODE ||
     params->fcf.dest_addr_mode == FRAME802154_SHORTADDRMODE) {
    params->fcf.panid_compression = 1;
  } else {
    params->fcf.panid_compression = 0;
  }
}

/**
 *
 */
static int create_frame(sPacket* packet, int do_create) {
  frame802154_t params;
  int hdr_len;

  if(frame802154_get_pan_id() == 0xffff) {
	  TRice("wrn:framer-802154: invalid PAD id!\n");
    return -1;
  }

  /* init to zeros */
  memset(&params, 0, sizeof(params));

  /*
   * Before setting up "params", make sure we won't use 0 as the sequence number
   * of a newly created frame.
   *
   * The case when do_create is 0 is ignored since it's for only length
   * calculation. No sequence number is needed and should not be consumed.
   */
  if(do_create != 0 && packetbuf_attr(packet, PACKETBUF_ATTR_MAC_SEQNO) == 0) {
    packetbuf_set_attr(packet, PACKETBUF_ATTR_MAC_SEQNO, 0xa5);
  }

  framer_802154_setup_params(packet, &params);

  if(packetbuf_holds_broadcast(packet)) {
    params.dest_addr[0] = 0xFF;
    params.dest_addr[1] = 0xFF;
  } else {
    linkaddr_copy((linkaddr_t *)&params.dest_addr, packetbuf_addr(packet, PACKETBUF_ADDR_RECEIVER));
  }

  linkaddr_copy((linkaddr_t *)&params.src_addr, packetbuf_addr(packet, PACKETBUF_ADDR_SENDER));

  params.payload = packetbuf_dataptr(packet);
  params.payload_len = packetbuf_datalen(packet);
  hdr_len = frame802154_hdrlen(&params);
  if(!do_create) {
    /* Only calculate header length */
    return hdr_len;
  } else if(packetbuf_hdralloc(packet, hdr_len)) {
    frame802154_create(&params, packetbuf_hdrptr(packet));
    return hdr_len;
  } else {
	TRice("wrn:Out: too large header: %u\n", hdr_len);
    return FRAMER_FAILED;
  }
}

/* API Realization ----------------------------------------------------------*/
/**
 *
 */
static int hdr_length(sPacket* packet) {
  return create_frame(packet, 0);
}

/**
 *
 */
static int create(sPacket* packet) {
  return create_frame(packet, 1);
}

/**
 *
 */
static int parse(sPacket* packet) {
  frame802154_t frame;
  int hdr_len;

  hdr_len = frame802154_parse(packetbuf_dataptr(packet), packetbuf_datalen(packet), &frame);

  if(hdr_len && packetbuf_hdrreduce(packet, hdr_len)) {
    packetbuf_set_attr(packet, PACKETBUF_ATTR_FRAME_TYPE, frame.fcf.frame_type);
    packetbuf_set_attr(packet, PACKETBUF_ATTR_MAC_ACK, frame.fcf.ack_required);

    if(frame.fcf.dest_addr_mode) {
      if(frame.dest_pid != frame802154_get_pan_id() && frame.dest_pid != FRAME802154_BROADCASTPANDID) {
        /* Packet to another PAN */
    	TRice("wrn:15.4: for another pan %u\n", frame.dest_pid);
        return FRAMER_FAILED;
      }
      if(!frame802154_is_broadcast_addr(frame.fcf.dest_addr_mode, frame.dest_addr)) {
        packetbuf_set_addr(packet, PACKETBUF_ADDR_RECEIVER, (linkaddr_t *)&frame.dest_addr);
      }
    }
    packetbuf_set_addr(packet, PACKETBUF_ADDR_SENDER, (linkaddr_t *)&frame.src_addr);
    if(frame.fcf.sequence_number_suppression == 0) {
      packetbuf_set_attr(packet, PACKETBUF_ATTR_MAC_SEQNO, frame.seq);
    } else {
      packetbuf_set_attr(packet, PACKETBUF_ATTR_MAC_SEQNO, 0xffff);
    }
#if NETSTACK_CONF_WITH_RIME
    packetbuf_set_attr(packet, PACKETBUF_ATTR_PACKET_ID, frame.seq);
#endif

#if LLSEC802154_USES_AUX_HEADER
    if(frame.fcf.security_enabled) {
      packetbuf_set_attr(packet, PACKETBUF_ATTR_SECURITY_LEVEL, frame.aux_hdr.security_control.security_level);
#if LLSEC802154_USES_FRAME_COUNTER
      packetbuf_set_attr(packet, PACKETBUF_ATTR_FRAME_COUNTER_BYTES_0_1, frame.aux_hdr.frame_counter.u16[0]);
      packetbuf_set_attr(packet, PACKETBUF_ATTR_FRAME_COUNTER_BYTES_2_3, frame.aux_hdr.frame_counter.u16[1]);
#endif /* LLSEC802154_USES_FRAME_COUNTER */
#if LLSEC802154_USES_EXPLICIT_KEYS
      packetbuf_set_attr(packet, PACKETBUF_ATTR_KEY_ID_MODE, frame.aux_hdr.security_control.key_id_mode);
      packetbuf_set_attr(packet, PACKETBUF_ATTR_KEY_INDEX, frame.aux_hdr.key_index);
#endif /* LLSEC802154_USES_EXPLICIT_KEYS */
    }
#endif /* LLSEC802154_USES_AUX_HEADER */

    return hdr_len;
  } else if (0 == hdr_len) {
	  TRice("wrn:frame802154_parse() failed!\n");
  } else {
	  TRice("wrn:packetbuf_hdrreduce() failed!\n");
  }
  return FRAMER_FAILED;
}

const struct framer framer_802154 = {
  hdr_length,
  create,
  parse
};
/*---------------------------------------------------------------------------*/
