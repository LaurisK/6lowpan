/*
 * Copyright (c) 2016, Inria.
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
 * \addtogroup uip
 * @{
 *
 * \file
 *         Source routing support
 *
 * \author Simon Duquennoy <simon.duquennoy@inria.fr>
 */

#include <stdio.h>
#include "../routing/routing.h"
#include "../routing/rpl-lite/rpl-neighbor.h"

/* Total number of nodes */
static int num_nodes;

/* Every known node in the network */
uip_sr_node_t *nodes = NULL;

static uint16_t srEvtIdOffset;
static fRadioEvtHndl srEvtHndl;
/*---------------------------------------------------------------------------*/
int uip_sr_num_nodes(void) {
  return num_nodes;
}
/*---------------------------------------------------------------------------*/
static int node_matches_address(void *graph, const uip_sr_node_t *node, const uip_ipaddr_t *addr) {
  if(node == NULL || addr == NULL || graph != node->graph) {
    return 0;
  } else {
    uip_ipaddr_t node_ipaddr;
    rpl_lite_driver.get_sr_node_ipaddr(&node_ipaddr, node);
    if (uip_is_addr_linklocal(addr)) {
    	uip_create_linklocal_prefix(&node_ipaddr);
    }
    return uip_ipaddr_cmp(&node_ipaddr, addr);
  }
}
/*---------------------------------------------------------------------------*/
uip_sr_node_t* uip_sr_get_node(void *graph, const uip_ipaddr_t *addr) {
  uip_sr_node_t *walker = nodes;
  while (NULL != walker) {
	uip_ipaddr_t nodeAddr;
	rpl_lite_driver.get_sr_node_ipaddr(&nodeAddr, walker);
    /* Compare prefix and node identifier */
    if(node_matches_address(graph, walker, addr)) {
      return walker;
    }
    walker = walker->next;
  }
  return NULL;
}
/*---------------------------------------------------------------------------*/
int uip_sr_is_addr_reachable(void *graph, const uip_ipaddr_t *addr) {
  int max_depth = UIP_SR_LINK_NUM;
  uip_ipaddr_t root_ipaddr;
  uip_sr_node_t *node;
  uip_sr_node_t *root_node;

  rpl_lite_driver.get_root_ipaddr(&root_ipaddr);
  node = uip_sr_get_node(graph, addr);
  root_node = uip_sr_get_node(graph, &root_ipaddr);

  while(node != NULL && node != root_node && max_depth > 0) {
    node = node->parent;
    max_depth--;
  }
  return node != NULL && node == root_node;
}
/*---------------------------------------------------------------------------*/
void uip_sr_expire_parent(void *graph, const uip_ipaddr_t *child, const uip_ipaddr_t *parent) {
  uip_sr_node_t *l = uip_sr_get_node(graph, child);
  /* Check if parent matches */
  if(l != NULL && node_matches_address(graph, l->parent, parent)) {
    l->lifetime = UIP_SR_REMOVAL_DELAY;
  }
}
/*---------------------------------------------------------------------------*/
uip_sr_node_t* uip_sr_update_node(void *graph, const uip_ipaddr_t *child, const uip_ipaddr_t *parent, uint32_t lifetime) {
  uip_sr_node_t *child_node = uip_sr_get_node(graph, child);
  uip_sr_node_t *parent_node = uip_sr_get_node(graph, parent);
  uip_sr_node_t *old_parent_node;

  if(parent != NULL) {
    /* No node for the parent, add one with infinite lifetime */
    if(parent_node == NULL) {
      parent_node = uip_sr_update_node(graph, parent, NULL, UIP_SR_INFINITE_LIFETIME);
      if(parent_node == NULL) {
    	  TRice("err:NS: no space left for root node!\n");
        return NULL;
      }
      TRice("msg:creating new parent link.\n");
    }
  }

  /* No node for this child, add one */
  if(child_node == NULL) {
	TRice("msg:creating new child link.\n");
    child_node = pvPortMalloc(sizeof(uip_sr_node_t));
    /* No space left, abort */
    if(child_node == NULL) {
      TRiceS("err:NS: no space left for child %s\n", uip6_printAddr(child, NULL));
      return NULL;
    }
    child_node->parent = NULL;
    child_node->next = nodes;
    nodes = child_node;
    num_nodes++;
    if (NULL != parent) {
    	srEvtHndl(srEvtIdOffset + radio_dagLinkCreated, (void*)child);
    }
  }

  /* Initialize node */
  child_node->graph = graph;
  child_node->lifetime = lifetime;
  memcpy(child_node->link_identifier, ((const unsigned char *)child) + 8, 8);

  old_parent_node = child_node->parent;
  /* Update node */
  child_node->parent = parent_node;
  /* Has the node become unreachable? May happen if we create a loop. */
  if(!uip_sr_is_addr_reachable(graph, child)) {
    /* The new parent makes the node unreachable, restore old parent.
     * We will take the update next time, with chances we know more of
     * the topology and the loop is gone. */
    child_node->parent = old_parent_node;
  }

  TRiceS("msg:NS: updating link, child %s, ", uip6_printAddr(child, NULL));
  TRiceS("msg:parent %s, ", uip6_printAddr(parent, NULL));
  TRice("msg:lifetime %u, num_nodes %u\n", (uint16_t)lifetime, num_nodes);

  return child_node;
}
/*---------------------------------------------------------------------------*/
void uip_sr_init(uint16_t evtOffset, fRadioEvtHndl packedEvtHndl) {
  num_nodes = 0;
  srEvtIdOffset = evtOffset;
  srEvtHndl = packedEvtHndl;
}
/*---------------------------------------------------------------------------*/
uip_sr_node_t * uip_sr_node_head(void) {
  return nodes;
}
/*---------------------------------------------------------------------------*/
uip_sr_node_t * uip_sr_node_next(uip_sr_node_t *item) {
  return item->next;
}
static void uip_sr_node_remove(uip_sr_node_t *itemToRemove) {
	uip_sr_node_t *walker = nodes, *follower = NULL;
	/* Remove neighbor from list */
	while (NULL != walker) {
		if (itemToRemove == walker) {
			if (NULL != follower) {
				follower->next = walker->next;
			} else {
				nodes = walker->next;
			}
			walker->next = NULL;
			break;
		}
		follower = walker;
		walker = walker->next;
	}
	{
		uip_ipaddr_t nodeAddr;
		rpl_lite_driver.get_sr_node_ipaddr(&nodeAddr, itemToRemove);
		srEvtHndl(srEvtIdOffset + radio_dagLinkDestroyed, &nodeAddr);
	}
	vPortFree(itemToRemove);
    num_nodes--;
}
/*---------------------------------------------------------------------------*/
void uip_sr_periodic(unsigned seconds) {
  uip_sr_node_t *l;
  uip_sr_node_t *next;

  /* First pass, for all expired nodes, deallocate them iff no child points to them */
  for(l = nodes; l != NULL; l = next) {
    next = l->next;
    if(l->lifetime == 0) {
      uip_sr_node_t *l2;
      for(l2 = nodes; l2 != NULL; l2 = l2->next) {
        if(l2->parent == l) {
          break;
        }
      }
      if(1/*LOG_INFO_ENABLED*/) {
        uip_ipaddr_t node_addr;
        rpl_lite_driver.get_sr_node_ipaddr(&node_addr, l);
        TRiceS("msg:NS: removing expired node %s, ", uip6_printAddr(&node_addr, NULL));
      }
      /* No child found, deallocate node */
      uip_sr_node_remove(l);
    } else if(l->lifetime != UIP_SR_INFINITE_LIFETIME) {
      l->lifetime = l->lifetime > seconds ? l->lifetime - seconds : 0;
    }
  }
}
/*---------------------------------------------------------------------------*/
void uip_sr_free_all(void) {
  uip_sr_node_t *l;
  uip_sr_node_t *next;
  for(l = nodes; l != NULL; l = next) {
    next = l->next;
    uip_sr_node_remove(l);
  }
}
/*---------------------------------------------------------------------------*/
int uip_sr_link_snprint(char *buf, int buflen, uip_sr_node_t *link) {
  int index = 0;
  uip_ipaddr_t child_ipaddr;
  uip_ipaddr_t parent_ipaddr;

  rpl_lite_driver.get_sr_node_ipaddr(&child_ipaddr, link);
  rpl_lite_driver.get_sr_node_ipaddr(&parent_ipaddr, link->parent);

  index = snprintf(buf, buflen, "%s", uip6_printAddr(&child_ipaddr, NULL));
  if(index >= buflen) {
    return index;
  }
  if(link->parent == NULL) {
	  index += snprintf(&buf[index], (buflen-index), "(DODAG root)");
  } else {
	  index += snprintf(&buf[index], (buflen-index), "(parent:%s)", uip6_printAddr(&parent_ipaddr, NULL));
  }
  if(index >= buflen) {
    return index;
  }
  if(link->lifetime != UIP_SR_INFINITE_LIFETIME) {
	  index += snprintf(&buf[index], (buflen-index), "[lifetime: %lu seconds]", (unsigned long)link->lifetime);
  } else {
	  index += snprintf(&buf[index], (buflen-index), "[lifetime: infinite]");
  }
  return index;
}
/** @} */
