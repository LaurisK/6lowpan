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
 * \addtogroup rpl-lite
 * @{
 *
 * \file
 *         ICMP6 I/O for RPL control messages.
 *
 * \author Joakim Eriksson <joakime@sics.se>, Nicolas Tsiftes <nvt@sics.se>,
 * Simon Duquennoy <simon.duquennoy@inria.fr>
 * Contributors: Niclas Finne <nfi@sics.se>, Joel Hoglund <joel@sics.se>,
 *               Mathieu Pouillot <m.pouillot@watteco.com>,
 *               George Oikonomou <oikonomou@users.sourceforge.net> (multicast)
 */

//#include "net/routing/rpl-lite/rpl.h"
#include "../../network/uip-icmp6.h"
//#include "net/packetbuf.h"
//#include "lib/random.h"

#include <limits.h>
#include "rpl.h"
#include "rpl-icmp6.h"
#include "rpl-dag.h"

/*---------------------------------------------------------------------------*/
#define RPL_DIO_GROUNDED                 0x80
#define RPL_DIO_MOP_SHIFT                3
#define RPL_DIO_MOP_MASK                 0x38
#define RPL_DIO_PREFERENCE_MASK          0x07

/*---------------------------------------------------------------------------*/
static void dis_input(sUipBuff *uipBuff);
static void dio_input(sUipBuff *uipBuff);
static void dao_input(sUipBuff *uipBuff);

/*---------------------------------------------------------------------------*/
/* Initialize RPL ICMPv6 message handlers */
static uip_icmp6_input_handler_t dis_handler = {NULL, ICMP6_RPL, RPL_CODE_DIS, dis_input};
static uip_icmp6_input_handler_t dio_handler = {NULL, ICMP6_RPL, RPL_CODE_DIO, dio_input};
static uip_icmp6_input_handler_t dao_handler = {NULL, ICMP6_RPL, RPL_CODE_DAO, dao_input};

#if RPL_WITH_DAO_ACK
static void dao_ack_input(sUipBuff *uipBuff);
static uip_icmp6_input_handler_t dao_ack_handler = {NULL, ICMP6_RPL, RPL_CODE_DAO_ACK, dao_ack_input};
#endif /* RPL_WITH_DAO_ACK */

#warning "for ra, ns and other sUipBuff will need to have some dynamic memory handler or other way to obtaing and release memeory only when needed - not to waste it like now it is done."
static sUipBuff disBuff = {0};
static sUipBuff dioBuff = {0};
static sUipBuff daoBuff = {0};
#if RPL_WITH_DAO_ACK
static sUipBuff daoAckBuff = {0};
#endif /* RPL_WITH_DAO_ACK */
/*---------------------------------------------------------------------------*/
static uint32_t
get32(uint8_t *buffer, int pos)
{
  return ((uint32_t)buffer[pos] << 24 | (uint32_t)buffer[pos + 1] << 16 |
          (uint32_t)buffer[pos + 2] << 8 | buffer[pos + 3]);
}
/*---------------------------------------------------------------------------*/
static void
set32(uint8_t *buffer, int pos, uint32_t value)
{
  buffer[pos++] = value >> 24;
  buffer[pos++] = (value >> 16) & 0xff;
  buffer[pos++] = (value >> 8) & 0xff;
  buffer[pos++] = value & 0xff;
}
/*---------------------------------------------------------------------------*/
static uint16_t
get16(uint8_t *buffer, int pos)
{
  return (uint16_t)buffer[pos] << 8 | buffer[pos + 1];
}
/*---------------------------------------------------------------------------*/
static void
set16(uint8_t *buffer, int pos, uint16_t value)
{
  buffer[pos++] = value >> 8;
  buffer[pos++] = value & 0xff;
}
/*---------------------------------------------------------------------------*/
uip_ds6_nbr_t *
rpl_icmp6_update_nbr_table(uip_ipaddr_t *from, nbr_table_reason_t reason, void *data)
{
  uip_ds6_nbr_t *nbr;

  if((nbr = uip_ds6_nbr_lookup(from)) == NULL) {
    if((nbr = uip_ds6_nbr_add(from, (uip_lladdr_t *) packetbuf_addr(PACKETBUF_ADDR_SENDER), 0, NBR_REACHABLE, reason, data)) == NULL) {
      TRiceS(iD(3966), "err:could not add neighbor to cache %s, ", uip6_printAddr(from, NULL));
      TRiceS(iD(5070), "err:%s\n", (char*)linkaddr_printAddr(packetbuf_addr(PACKETBUF_ADDR_SENDER)));
    }
  }

  return nbr;
}
/*---------------------------------------------------------------------------*/
static void dis_input(sUipBuff *uipBuff) {
  if(!curr_instance.used) {
	  TRice(iD(6494), "wrn:dis_input: not in an instance yet, discard\n");
    goto discard;
  }

  TRiceS(iD(4017), "msg:received a DIS from %s\n", uip6_printAddr(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->srcipaddr, NULL));

  rpl_process_dis(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->srcipaddr, uip_is_addr_mcast(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->destipaddr));

  discard:
    uipbuf_clear(uipBuff);
}
/*---------------------------------------------------------------------------*/
void
rpl_icmp6_dis_output(uip_ipaddr_t *addr)
{
  unsigned char *buffer;

  /* Make sure we're up-to-date before sending data out */
  rpl_dag_update_state();

  buffer = disBuff.buff.u8 + UIP_IPH_LEN + UIP_ICMPH_LEN + disBuff.extLen;
  buffer[0] = buffer[1] = 0;

  if(addr == NULL) {
    addr = &rpl_multicast_addr;
  }

  TRiceS(iD(3375), "msg:sending a DIS to %s\n", uip6_printAddr(addr, NULL));

  uip_icmp6_send(&disBuff, addr, ICMP6_RPL, RPL_CODE_DIS, 2);
}
/*---------------------------------------------------------------------------*/
static void dio_input(sUipBuff *uipBuff) {
  unsigned char *buffer;
  uint8_t buffer_length;
  rpl_dio_t dio;
  uint8_t subopt_type;
  int i;
  int len;
  uip_ipaddr_t from;

  memset(&dio, 0, sizeof(dio));

  /* Set default values in case the DIO configuration option is missing. */
  dio.dag_intdoubl = RPL_DIO_INTERVAL_DOUBLINGS;
  dio.dag_intmin = RPL_DIO_INTERVAL_MIN;
  dio.dag_redund = RPL_DIO_REDUNDANCY;
  dio.dag_min_hoprankinc = RPL_MIN_HOPRANKINC;
  dio.dag_max_rankinc = RPL_MAX_RANKINC;
  dio.ocp = RPL_OF_OCP;
  dio.default_lifetime = RPL_DEFAULT_LIFETIME;
  dio.lifetime_unit = RPL_DEFAULT_LIFETIME_UNIT;

  uip_ipaddr_copy(&from, &IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->srcipaddr);

  buffer_length = uipBuff->len - (UIP_IPH_LEN + UIP_ICMPH_LEN + uipBuff->extLen);

  /* Process the DIO base option. */
  i = 0;
  buffer = uipBuff->buff.u8 + UIP_IPH_LEN + UIP_ICMPH_LEN + uipBuff->extLen;

  dio.instance_id = buffer[i++];
  dio.version = buffer[i++];
  dio.rank = get16(buffer, i);
  i += 2;

  dio.grounded = buffer[i] & RPL_DIO_GROUNDED;
  dio.mop = (buffer[i]& RPL_DIO_MOP_MASK) >> RPL_DIO_MOP_SHIFT;
  dio.preference = buffer[i++] & RPL_DIO_PREFERENCE_MASK;

  dio.dtsn = buffer[i++];
  /* two reserved bytes */
  i += 2;

  memcpy(&dio.dag_id, buffer + i, sizeof(dio.dag_id));
  i += sizeof(dio.dag_id);

  /* Check if there are any DIO suboptions. */
  for(; i < buffer_length; i += len) {
    subopt_type = buffer[i];
    if(subopt_type == RPL_OPTION_PAD1) {
      len = 1;
    } else {
      /* Suboption with a two-byte header + payload */
      len = 2 + buffer[i + 1];
    }

    if(len + i > buffer_length) {
    	TRice(iD(7515), "err:dio_input: malformed packet, discard\n");
      goto discard;
    }

    switch(subopt_type) {
      case RPL_OPTION_DAG_METRIC_CONTAINER:
        if(len < 6) {
        	TRice(iD(4104), "wrn:dio_input: invalid DAG MC, len %u, discard\n", len);
          goto discard;
        }
        dio.mc.type = buffer[i + 2];
        dio.mc.flags = buffer[i + 3] << 1;
        dio.mc.flags |= buffer[i + 4] >> 7;
        dio.mc.aggr = (buffer[i + 4] >> 4) & 0x3;
        dio.mc.prec = buffer[i + 4] & 0xf;
        dio.mc.length = buffer[i + 5];

        if(dio.mc.type == RPL_DAG_MC_NONE) {
          /* No metric container: do nothing */
        } else if(dio.mc.type == RPL_DAG_MC_ETX) {
          dio.mc.obj.etx = get16(buffer, i + 6);
        } else if(dio.mc.type == RPL_DAG_MC_ENERGY) {
          dio.mc.obj.energy.flags = buffer[i + 6];
          dio.mc.obj.energy.energy_est = buffer[i + 7];
        } else {
        	TRice(iD(4407), "wrn:dio_input: unsupported DAG MC type %u, discard\n", (unsigned)dio.mc.type);
          goto discard;
        }
        break;
      case RPL_OPTION_ROUTE_INFO:
        if(len < 9) {
        	TRice(iD(5055), "wrn:dio_input: invalid destination prefix option, len %u, discard\n", len);
          goto discard;
        }

        /* The flags field includes the preference value. */
        dio.destination_prefix.length = buffer[i + 2];
        dio.destination_prefix.flags = buffer[i + 3];
        dio.destination_prefix.lifetime = get32(buffer, i + 4);

        if(((dio.destination_prefix.length + 7) / 8) + 8 <= len &&
           dio.destination_prefix.length <= 128) {
          memcpy(&dio.destination_prefix.prefix, &buffer[i + 8],
                 (dio.destination_prefix.length + 7) / 8);
        } else {
        	TRice(iD(1639), "wrn:dio_input: invalid route info option, len %u, discard\n", len);
          goto discard;
        }

        break;
      case RPL_OPTION_DAG_CONF:
        if(len != 16) {
        	TRice(iD(3151), "wrn:dio_input: invalid DAG configuration option, len %u, discard\n", len);
          goto discard;
        }

        /* Path control field not yet implemented - at i + 2 */
        dio.dag_intdoubl = buffer[i + 3];
        dio.dag_intmin = buffer[i + 4];
        dio.dag_redund = buffer[i + 5];
        dio.dag_max_rankinc = get16(buffer, i + 6);
        dio.dag_min_hoprankinc = get16(buffer, i + 8);
        dio.ocp = get16(buffer, i + 10);
        /* buffer + 12 is reserved */
        dio.default_lifetime = buffer[i + 13];
        dio.lifetime_unit = get16(buffer, i + 14);
        break;
      case RPL_OPTION_PREFIX_INFO:
        if(len != 32) {
        	TRice(iD(2503), "wrn:dio_input: invalid DAG prefix info, len %u, discard\n", len);
          goto discard;
        }
        dio.prefix_info.length = buffer[i + 2];
        dio.prefix_info.flags = buffer[i + 3];
        /* valid lifetime is ingnored for now - at i + 4 */
        /* preferred lifetime stored in lifetime */
        dio.prefix_info.lifetime = get32(buffer, i + 8);
        /* 32-bit reserved at i + 12 */
        memcpy(&dio.prefix_info.prefix, &buffer[i + 16], 16);
        break;
      default:
    	  TRice(iD(2457), "wrn:dio_input: unsupported suboption type in DIO: %u, discard\n", (unsigned)subopt_type);
        goto discard;
    }
  }

  if (uip_is_addr_mcast(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->destipaddr)) {
	  TRiceS(iD(3541), "msg:received a multicast-DIO from %s", uip6_printAddr(&from, NULL));
  } else {
	  TRiceS(iD(1281), "msg:received a unicast-DIO from %s", uip6_printAddr(&from, NULL));
  }
  TRice(iD(3793), "msg:, instance_id %u, version %u, dtsn %u, rank %u", dio.instance_id, dio.version, dio.dtsn, (uint16_t)dio.rank);
  TRiceS(iD(2713), "msg:, DAG ID %s\n", uip6_printAddr(&dio.dag_id, NULL));

  rpl_process_dio(&from, &dio);

discard:
  uipbuf_clear(uipBuff);
}
/*---------------------------------------------------------------------------*/
void
rpl_icmp6_dio_output(uip_ipaddr_t *uc_addr)
{
  unsigned char *buffer;
  int pos;
  uip_ipaddr_t *addr = uc_addr;

  /* Make sure we're up-to-date before sending data out */
  rpl_dag_update_state();

  if(rpl_get_leaf_only()) {
    /* In leaf mode, we only send DIO messages as unicasts in response to
       unicast DIS messages. */
    if(uc_addr == NULL) {
      /* Do not send multicast DIO in leaf mode */
      return;
    }
  }

  /* DAG Information Object */
  pos = 0;

  buffer = dioBuff.buff.u8 + UIP_IPH_LEN + UIP_ICMPH_LEN + dioBuff.extLen;
  buffer[pos++] = curr_instance.instance_id;
  buffer[pos++] = curr_instance.dag.version;

  if(rpl_get_leaf_only()) {
    set16(buffer, pos, RPL_INFINITE_RANK);
  } else {
    set16(buffer, pos, curr_instance.dag.rank);
  }
  pos += 2;

  buffer[pos] = 0;
  if(curr_instance.dag.grounded) {
    buffer[pos] |= RPL_DIO_GROUNDED;
  }

  buffer[pos] |= curr_instance.mop << RPL_DIO_MOP_SHIFT;
  buffer[pos] |= curr_instance.dag.preference & RPL_DIO_PREFERENCE_MASK;
  pos++;

  buffer[pos++] = curr_instance.dtsn_out;

  /* reserved 2 bytes */
  buffer[pos++] = 0; /* flags */
  buffer[pos++] = 0; /* reserved */

  memcpy(buffer + pos, &curr_instance.dag.dag_id, sizeof(curr_instance.dag.dag_id));
  pos += 16;

  if(!rpl_get_leaf_only()) {
    if(curr_instance.mc.type != RPL_DAG_MC_NONE) {
      buffer[pos++] = RPL_OPTION_DAG_METRIC_CONTAINER;
      buffer[pos++] = 6;
      buffer[pos++] = curr_instance.mc.type;
      buffer[pos++] = curr_instance.mc.flags >> 1;
      buffer[pos] = (curr_instance.mc.flags & 1) << 7;
      buffer[pos++] |= (curr_instance.mc.aggr << 4) | curr_instance.mc.prec;
      if(curr_instance.mc.type == RPL_DAG_MC_ETX) {
        buffer[pos++] = 2;
        set16(buffer, pos, curr_instance.mc.obj.etx);
        pos += 2;
      } else if(curr_instance.mc.type == RPL_DAG_MC_ENERGY) {
        buffer[pos++] = 2;
        buffer[pos++] = curr_instance.mc.obj.energy.flags;
        buffer[pos++] = curr_instance.mc.obj.energy.energy_est;
      } else {
    	  TRice(iD(2986), "err:unable to send DIO because of unsupported DAG MC type %u\n",
               (unsigned)curr_instance.mc.type);
        return;
      }
    }
  }

  /* Always add a DAG configuration option. */
  buffer[pos++] = RPL_OPTION_DAG_CONF;
  buffer[pos++] = 14;
  buffer[pos++] = 0; /* No Auth, PCS = 0 */
  buffer[pos++] = curr_instance.dio_intdoubl;
  buffer[pos++] = curr_instance.dio_intmin;
  buffer[pos++] = curr_instance.dio_redundancy;
  set16(buffer, pos, curr_instance.max_rankinc);
  pos += 2;
  set16(buffer, pos, curr_instance.min_hoprankinc);
  pos += 2;
  /* OCP is in the DAG_CONF option */
  set16(buffer, pos, curr_instance.of->ocp);
  pos += 2;
  buffer[pos++] = 0; /* reserved */
  buffer[pos++] = curr_instance.default_lifetime;
  set16(buffer, pos, curr_instance.lifetime_unit);
  pos += 2;

  /* Check if we have a prefix to send also. */
  if(curr_instance.dag.prefix_info.length > 0) {
    buffer[pos++] = RPL_OPTION_PREFIX_INFO;
    buffer[pos++] = 30; /* always 30 bytes + 2 long */
    buffer[pos++] = curr_instance.dag.prefix_info.length;
    buffer[pos++] = curr_instance.dag.prefix_info.flags;
    set32(buffer, pos, curr_instance.dag.prefix_info.lifetime);
    pos += 4;
    set32(buffer, pos, curr_instance.dag.prefix_info.lifetime);
    pos += 4;
    memset(&buffer[pos], 0, 4);
    pos += 4;
    memcpy(&buffer[pos], &curr_instance.dag.prefix_info.prefix, 16);
    pos += 16;
  }

  if(!rpl_get_leaf_only()) {
    addr = addr != NULL ? addr : &rpl_multicast_addr;
  }

  if (uc_addr != NULL) {
	  TRice(iD(4648), "msg:sending a unicast-DIO with rank %u", (uint16_t)curr_instance.dag.rank);
  } else {
	  TRice(iD(3683), "msg:sending a multicast-DIO with rank %u", (uint16_t)curr_instance.dag.rank);
  }
  TRice(iD(5252), "msg:to %s\n", uip6_printAddr(addr, NULL));

  uip_icmp6_send(&dioBuff, addr, ICMP6_RPL, RPL_CODE_DIO, pos);
}
/*---------------------------------------------------------------------------*/
static void dao_input(sUipBuff *uipBuff) {
  struct rpl_dao dao;
  uint8_t subopt_type;
  unsigned char *buffer;
  uint8_t buffer_length;
  int pos;
  int len;
  int i;
  uip_ipaddr_t from;

  memset(&dao, 0, sizeof(dao));

  dao.instance_id = *(uipBuff->buff.u8 + UIP_IPH_LEN + UIP_ICMPH_LEN + uipBuff->extLen);
  if(!curr_instance.used || curr_instance.instance_id != dao.instance_id) {
	  TRice(iD(5842), "err:dao_input: unknown RPL instance %u, discard\n", dao.instance_id);
    goto discard;
  }

  uip_ipaddr_copy(&from, &IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->srcipaddr);
  memset(&dao.parent_addr, 0, 16);

  buffer = uipBuff->buff.u8 + UIP_IPH_LEN + UIP_ICMPH_LEN + uipBuff->extLen;
  buffer_length = uipBuff->len - (UIP_IPH_LEN + UIP_ICMPH_LEN + uipBuff->extLen);

  pos = 0;
  pos++; /* instance ID */
  dao.lifetime = curr_instance.default_lifetime;
  dao.flags = buffer[pos++];
  pos++; /* reserved */
  dao.sequence = buffer[pos++];

  /* Is the DAG ID present? */
  if(dao.flags & RPL_DAO_D_FLAG) {
    if(memcmp(&curr_instance.dag.dag_id, &buffer[pos], sizeof(curr_instance.dag.dag_id))) {
      TRiceS(iD(2799), "err:dao_input: different DAG ID %s, discard\n", uip6_printAddr((uip_ipaddr_t *)&buffer[pos], NULL));
      goto discard;
    }
    pos += 16;
  }

  /* Check if there are any RPL options present. */
  for(i = pos; i < buffer_length; i += len) {
    subopt_type = buffer[i];
    if(subopt_type == RPL_OPTION_PAD1) {
      len = 1;
    } else {
      /* The option consists of a two-byte header and a payload. */
      len = 2 + buffer[i + 1];
    }

    switch(subopt_type) {
      case RPL_OPTION_TARGET:
        /* Handle the target option. */
        dao.prefixlen = buffer[i + 3];
        memset(&dao.prefix, 0, sizeof(dao.prefix));
        memcpy(&dao.prefix, buffer + i + 4, (dao.prefixlen + 7) / CHAR_BIT);
        break;
      case RPL_OPTION_TRANSIT:
        /* The path sequence and control are ignored. */
        /*      pathcontrol = buffer[i + 3];
                pathsequence = buffer[i + 4];*/
        dao.lifetime = buffer[i + 5];
        if(len >= 20) {
          memcpy(&dao.parent_addr, buffer + i + 6, 16);
        }
        break;
    }
  }

  /* Destination Advertisement Object */
  if (dao.lifetime == 0) {
	  TRiceS(iD(1362), "msg:received a No-path DAO from %s", uip6_printAddr(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->srcipaddr, NULL));
  } else {
	  TRiceS(iD(4060), "msg:received a DAO from %s", uip6_printAddr(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->srcipaddr, NULL));
  }
  TRice(iD(1454), "msg:, seqno %u, lifetime %u, prefix length %u", dao.sequence, dao.lifetime, dao.prefixlen);
  TRiceS(iD(1406), "msg:, prefix %s", uip6_printAddr(&dao.prefix, NULL));
  TRiceS(iD(7216), "msg:, parent %s\n", uip6_printAddr(&dao.parent_addr, NULL));

  rpl_process_dao(&from, &dao);

  discard:
    uipbuf_clear(uipBuff);
}
/*---------------------------------------------------------------------------*/
void
rpl_icmp6_dao_output(uint8_t lifetime)
{
  unsigned char *buffer;
  uint8_t prefixlen;
  int pos;
  const uip_ipaddr_t *prefix = rpl_get_global_address();
  uip_ipaddr_t *parent_ipaddr = rpl_neighbor_get_ipaddr(curr_instance.dag.preferred_parent);

  /* Make sure we're up-to-date before sending data out */
  rpl_dag_update_state();

  if(!curr_instance.used) {
	  TRice(iD(3669), "wrn:rpl_icmp6_dao_output: not in an instance, skip sending DAO\n");
    return;
  }

  if(curr_instance.dag.preferred_parent == NULL) {
	  TRice(iD(6435), "wrn:rpl_icmp6_dao_output: no preferred parent, skip sending DAO\n");
    return;
  }

  if(prefix == NULL || parent_ipaddr == NULL || curr_instance.mop == RPL_MOP_NO_DOWNWARD_ROUTES) {
	  TRice(iD(4186), "wrn:rpl_icmp6_dao_output: node not ready to send a DAO (prefix %p, parent addr %p, mop %u)\n",
                    prefix, parent_ipaddr, curr_instance.mop);
    return;
  }

  buffer = daoBuff.buff.u8 + UIP_IPH_LEN + UIP_ICMPH_LEN + daoBuff.extLen;
  pos = 0;

  buffer[pos++] = curr_instance.instance_id;
  buffer[pos] = 0;
#if RPL_WITH_DAO_ACK
  if(lifetime != 0) {
    buffer[pos] |= RPL_DAO_K_FLAG;
  }
#endif /* RPL_WITH_DAO_ACK */
  ++pos;
  buffer[pos++] = 0; /* reserved */
  buffer[pos++] = curr_instance.dag.dao_last_seqno;

  /* create target subopt */
  prefixlen = sizeof(*prefix) * CHAR_BIT;
  buffer[pos++] = RPL_OPTION_TARGET;
  buffer[pos++] = 2 + ((prefixlen + 7) / CHAR_BIT);
  buffer[pos++] = 0; /* reserved */
  buffer[pos++] = prefixlen;
  memcpy(buffer + pos, prefix, (prefixlen + 7) / CHAR_BIT);
  pos += ((prefixlen + 7) / CHAR_BIT);

  /* Create a transit information sub-option. */
  buffer[pos++] = RPL_OPTION_TRANSIT;
  buffer[pos++] = 20;
  buffer[pos++] = 0; /* flags - ignored */
  buffer[pos++] = 0; /* path control - ignored */
  buffer[pos++] = 0; /* path seq - ignored */
  buffer[pos++] = lifetime;

  /* Include parent global IP address */
  memcpy(buffer + pos, &curr_instance.dag.dag_id, 8); /* Prefix */
  pos += 8;
  memcpy(buffer + pos, ((const unsigned char *)parent_ipaddr) + 8, 8); /* Interface identifier */
  pos += 8;

  if (lifetime == 0) {
	  TRice(iD(5071), "msg:sending a No-path DAO seqno %u, tx count %u, lifetime %u", curr_instance.dag.dao_last_seqno, curr_instance.dag.dao_transmissions, lifetime);
  } else {
	  TRice(iD(5915), "msg:sending a DAO seqno %u, tx count %u, lifetime %u", curr_instance.dag.dao_last_seqno, curr_instance.dag.dao_transmissions, lifetime);
  }
  TRiceS(iD(5785), "msg:, prefix %s", uip6_printAddr(prefix, NULL));
  TRiceS(iD(6485), "msg: to %s", uip6_printAddr(&curr_instance.dag.dag_id, NULL));
  TRiceS(iD(3830), "msg:, parent %s\n", uip6_printAddr(parent_ipaddr, NULL));

  /* Send DAO to root (IPv6 address is DAG ID) */
  uip_icmp6_send(&daoBuff, &curr_instance.dag.dag_id, ICMP6_RPL, RPL_CODE_DAO, pos);
}
#if RPL_WITH_DAO_ACK
/*---------------------------------------------------------------------------*/
static void dao_ack_input(sUipBuff *uipBuff) {
  uint8_t *buffer;
  uint8_t instance_id;
  uint8_t sequence;
  uint8_t status;

  buffer = uipBuff->buff.u8 + UIP_IPH_LEN + UIP_ICMPH_LEN + uipBuff->extLen;

  instance_id = buffer[0];
  sequence = buffer[2];
  status = buffer[3];

  if(!curr_instance.used || curr_instance.instance_id != instance_id) {
	  TRice(iD(6642), "err:dao_ack_input: unknown instance, discard\n");
    goto discard;
  }

  if (status < RPL_DAO_ACK_UNABLE_TO_ACCEPT) {
	  TRiceS(iD(4095), "msg:received a DAO-ACK from %s, ", uip6_printAddr(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->srcipaddr, NULL));
  } else {
	  TRiceS(iD(2641), "msg:received a DAO-NACK from %s, ", uip6_printAddr(&IP_HDR_CAST_TO_BUFF(uipBuff->buff.u8)->srcipaddr, NULL));
  }
  TRice(iD(5549), "msg:seqno %d(%d %d) and status %d\n", sequence, curr_instance.dag.dao_last_seqno, curr_instance.dag.dao_last_seqno, status);

  rpl_process_dao_ack(sequence, status);

  discard:
    uipbuf_clear(uipBuff);
}
/*---------------------------------------------------------------------------*/
void
rpl_icmp6_dao_ack_output(uip_ipaddr_t *dest, uint8_t sequence, uint8_t status)
{
  unsigned char *buffer;

  /* Make sure we're up-to-date before sending data out */
  rpl_dag_update_state();

  buffer = daoAckBuff.buff.u8 + UIP_IPH_LEN + UIP_ICMPH_LEN + daoAckBuff.extLen;
  buffer[0] = curr_instance.instance_id;
  buffer[1] = 0;
  buffer[2] = sequence;
  buffer[3] = status;

  if (status < RPL_DAO_ACK_UNABLE_TO_ACCEPT) {
	  TRiceS(iD(3181), "msg:sending a DAO-ACK to %s, ", uip6_printAddr(dest, NULL));
  } else {
	  TRiceS(iD(3124), "msg:sending a DAO-NACK to %s, ", uip6_printAddr(dest, NULL));
  }
  TRice(iD(5192), "msg:seqno %d with status %d\n", sequence, status);

  uip_icmp6_send(&daoAckBuff, dest, ICMP6_RPL, RPL_CODE_DAO_ACK, 4);
}
#endif /* RPL_WITH_DAO_ACK */
/*---------------------------------------------------------------------------*/
void
rpl_icmp6_init()
{
  uip_icmp6_register_input_handler(&dis_handler);
  uip_icmp6_register_input_handler(&dio_handler);
  uip_icmp6_register_input_handler(&dao_handler);
#if RPL_WITH_DAO_ACK
  uip_icmp6_register_input_handler(&dao_ack_handler);
#endif /* RPL_WITH_DAO_ACK */
}
/*---------------------------------------------------------------------------*/

/** @}*/
