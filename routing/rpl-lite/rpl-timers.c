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
 */

/**
 * \addtogroup rpl-lite
 * @{
 *
 * \file
 *         RPL timer management.
 *
 * \author Joakim Eriksson <joakime@sics.se>, Nicolas Tsiftes <nvt@sics.se>,
 * Simon Duquennoy <simon.duquennoy@inria.fr>
 */

#include "../../lp-timer.h"
#include "../../network/link-stats.h"
#include "../../network/uip-sr.h"
#include "rpl.h"
#include "rpl-dag.h"
#include "rpl-dag-root.h"
#include "rpl-icmp6.h"
#include "rpl-timers.h"

/* A configurable function called after update of the RPL DIO interval */
#ifdef RPL_CALLBACK_NEW_DIO_INTERVAL
void RPL_CALLBACK_NEW_DIO_INTERVAL(uint32_t/*clock_time_t*/ dio_interval);
#endif /* RPL_CALLBACK_NEW_DIO_INTERVAL */

#define PERIODIC_DELAY_SECONDS     60
#define PERIODIC_DELAY             ((PERIODIC_DELAY_SECONDS) * 1000)

/*---------------------------------------------------------------------------*/
/* The timers and the work that belongs to them. They are private to this module, and the waiting
 * itself belongs to lp-timer: every handler below runs on the radio task, which owns the RPL state
 * (DAG, neighbours, SR graph) and the transmit path. */
static sLpTimer *periodicTimer; /* Not part of a DAG because used for general state maintenance */
static sLpTimer *disTimer; /* Not part of a DAG because when not joined */
static sLpTimer *dioTimer;
static sLpTimer *daoResendTimer;
static sLpTimer *daoRefreshTimer;
static sLpTimer *leaveTimer;
#if RPL_WITH_PROBING
static sLpTimer *probingTimer;
static sLpTimer *urgProbingTimer;
#endif /* RPL_WITH_PROBING */

/* Work the rest of RPL asks for rather than waits for; asking twice before it runs is one run */
static sLpWork *stateUpdateWork;
static sLpWork *unicastDioWork;
#if RPL_WITH_DAO_ACK
static sLpWork *daoAckWork;
#endif /* RPL_WITH_DAO_ACK */

static uint32_t dioNextDelay; /* delay for completion of the DIO (trickle) interval */
static uint8_t dioSend; /* internal trickle timer state: do we need to send a DIO at the next wakeup? */
/*---------------------------------------------------------------------------*/
static void DisTmoHandler(void) {
  if(!rpl_dag_root_is_root() && (!curr_instance.used || curr_instance.dag.preferred_parent == NULL || curr_instance.dag.rank == RPL_INFINITE_RANK)) {
    /* Send DIS and schedule next */
    rpl_icmp6_dis_output(NULL);
    rpl_timers_schedule_periodic_dis();
  }
}
/*---------------------------------------------------------------------------*/
static void PeriodicTimerHandler(void)
{
  if(curr_instance.used) {
    rpl_dag_periodic(PERIODIC_DELAY_SECONDS);
    uip_sr_periodic(PERIODIC_DELAY_SECONDS);
  }

  if(!curr_instance.used || curr_instance.dag.preferred_parent == NULL || curr_instance.dag.rank == RPL_INFINITE_RANK) {
    rpl_timers_schedule_periodic_dis(); /* Schedule DIS if needed */
  }

  /* Useful because part of the state update is time-dependent, e.g.,
  the meaning of last_advertised_rank changes with time */
  rpl_dag_update_state(NULL);

  if(1/*LOG_DBG_ENABLED*/) {
    rpl_neighbor_print_list("Periodic");
    rpl_dag_root_print_links("Periodic");
  }
}
/*---------------------------------------------------------------------------*/
/*------------------------------- DIS -------------------------------------- */
/*---------------------------------------------------------------------------*/
void rpl_timers_schedule_periodic_dis(void) {
  if(!LpTimer_IsScheduled(disTimer)) {
    LpTimer_Arm(disTimer, RPL_DIS_INTERVAL / 2 + System_Random(RPL_DIS_INTERVAL));
  }
}
/*---------------------------------------------------------------------------*/
/*------------------------------- DIO -------------------------------------- */
/*---------------------------------------------------------------------------*/
static void new_dio_interval(void) {
  uint32_t ticks;

  dioNextDelay = 1UL << curr_instance.dag.dio_intcurrent;

  /* random number between I/2 and I */
  ticks = dioNextDelay / 2 + System_Random(dioNextDelay / 2);

  /*
   * The intervals must be equally long among the nodes for Trickle to
   * operate efficiently. Therefore we need to calculate the delay between
   * the randomized time and the start time of the next interval.
   */
  dioNextDelay -= ticks;
  dioSend = 1;
  /* reset the redundancy counter */
  curr_instance.dag.dio_counter = 0;

  /* schedule the timer */
  LpTimer_Arm(dioTimer, ticks);
  TRice("msg:DIO timer scheduled for(%d)\n", ticks);

#ifdef RPL_CALLBACK_NEW_DIO_INTERVAL
  RPL_CALLBACK_NEW_DIO_INTERVAL((CLOCK_SECOND * 1UL << curr_instance.dag.dio_intcurrent) / 1000);
#endif /* RPL_CALLBACK_NEW_DIO_INTERVAL */
}
/*---------------------------------------------------------------------------*/
static void DioTmoHandler(void) {
  if(!rpl_dag_ready_to_advertise()) {
	TRice("msg:DioTmoHandler(exit - We will be scheduled again later)\n");
    return; /* We will be scheduled again later */
  }

  if(dioSend) {
    /* send DIO if counter is less than desired redundancy, or if dio_redundancy
    is set to 0, or if we are the root */
    if(rpl_dag_root_is_root() || curr_instance.dio_redundancy == 0 ||
        curr_instance.dag.dio_counter < curr_instance.dio_redundancy) {
#if RPL_TRICKLE_REFRESH_DAO_ROUTES
      if(rpl_dag_root_is_root()) {
        static int count = 0;
        if((count++ % RPL_TRICKLE_REFRESH_DAO_ROUTES) == 0) {
          /* Request new DAO to refresh route. */
          RPL_LOLLIPOP_INCREMENT(curr_instance.dtsn_out);
          TRice("msg:trigger DAO updates with a DTSN increment (%u)\n", curr_instance.dtsn_out);
        }
      }
#endif /* RPL_TRICKLE_REFRESH_DAO_ROUTES */
      curr_instance.dag.last_advertised_rank = curr_instance.dag.rank;
#if 1/*UIP_IPV6_MULTICAST*/
      if ((NBR_TABLE_MAX_NEIGHBORS - 1) > rpl_neighbor_count()) {
        TRice("msg:Issue periodical multicast-DIO\n");
        rpl_icmp6_dio_output(NULL);
      } else {
      	rpl_nbr_t *nbr = nbr_table_head(rpl_neighbors);
      	while (NULL != nbr) {
      	  uip_ipaddr_t *nbrAddr = rpl_neighbor_get_ipaddr(nbr);
      	  if (NULL != nbrAddr) {
        	TRiceS("msg:Issue periodical unicast-DIO to neighbor %s\n", uip6_printAddr(nbrAddr, NULL));
      		rpl_icmp6_dio_output(nbrAddr);
      	  }
      	  nbr = nbr_table_next(rpl_neighbors, nbr);
      	}
      }
#else /* UIP_IPV6_MULTICAST */
     {
    	rpl_nbr_t *nbr = nbr_table_head(rpl_neighbors);
    	while (NULL != nbr) {
    		uip_ipaddr_t *nbrAddr = rpl_neighbor_get_ipaddr(nbr);
    	  if (NULL != nbrAddr) {
      	    TRiceS("msg:Issue periodical unicast-DIO to neighbor %s\n", uip6_printAddr(nbrAddr, NULL));
    		rpl_icmp6_dio_output(nbrAddr);
    	  } else {
    	    TRice("msg:Issue periodical multicast-DIO - neighbor address was not obtained\n");
    	    rpl_icmp6_dio_output(NULL);
    	  }
    	  nbr = nbr_table_next(rpl_neighbors, nbr);
    	}
     }
#endif /* UIP_IPV6_MULTICAST */
    }
    dioSend = 0;
    LpTimer_Arm(dioTimer, dioNextDelay);
    TRice("msg:DIO timer continue for(%d)\n", dioNextDelay);
  } else {
    /* check if we need to double interval */
    if(curr_instance.dag.dio_intcurrent < curr_instance.dio_intmin + curr_instance.dio_intdoubl) {
      curr_instance.dag.dio_intcurrent++;
    }
    new_dio_interval();
  }
}
/*---------------------------------------------------------------------------*/
void rpl_timers_dio_reset(const char *str) {
  if(rpl_dag_ready_to_advertise() &&
     (curr_instance.dag.dio_intcurrent == 0 || curr_instance.dag.dio_intcurrent > curr_instance.dio_intmin)) {
    /*
     * don't reset the DIO timer if the current interval is Imin; see
     * Section 4.2, RFC 6206.
     */
	  TRiceS("msg:reset DIO timer (%s)\n", (char*)str);
    if(!rpl_get_leaf_only()) {
        curr_instance.dag.dio_counter = 0;
        curr_instance.dag.dio_intcurrent = curr_instance.dio_intmin;
        new_dio_interval();
    }
  } else {
	  TRiceS("msg:DIO timer (%s) not issued ()\n", (char*)str);
  }
}
/*---------------------------------------------------------------------------*/
/*------------------------------- Unicast DIO ------------------------------ */
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
static void UnicastDioWork(void) {
  uip_ipaddr_t *target_ipaddr = rpl_neighbor_get_ipaddr(curr_instance.dag.unicast_dio_target);
  if(target_ipaddr != NULL) {
    rpl_icmp6_dio_output(target_ipaddr);
  }
}
/*---------------------------------------------------------------------------*/
void rpl_timers_schedule_unicast_dio(rpl_nbr_t *target) {
  if(curr_instance.used) {
    curr_instance.dag.unicast_dio_target = target;
    LpWork_Request(unicastDioWork);
  }
}
/*---------------------------------------------------------------------------*/
/*------------------------------- DAO -------------------------------------- */
/*---------------------------------------------------------------------------*/
#if RPL_WITH_DAO_ACK
/*---------------------------------------------------------------------------*/
static void schedule_dao_retransmission(void) {
  uint32_t expiration_time = RPL_DAO_RETRANSMISSION_TIMEOUT / 2 + (System_Random(RPL_DAO_RETRANSMISSION_TIMEOUT));
  LpTimer_Arm(daoResendTimer, expiration_time);
}
/*---------------------------------------------------------------------------*/
static void DaoRefreshTmoHandler(void)
{
#if RPL_WITH_DAO_ACK
  /* We are sending a new DAO here. Prepare retransmissions */
  curr_instance.dag.dao_transmissions = 1;
  /* Schedule next retransmission */
  schedule_dao_retransmission();
#else /* RPL_WITH_DAO_ACK */
  /* No DAO-ACK: assume we are reachable as soon as we send a DAO */
  if(curr_instance.dag.state == DAG_JOINED) {
    curr_instance.dag.state = DAG_REACHABLE;
    System_FanStatusUpdate(fan_reachable);
  }
  rpl_timers_dio_reset("Reachable");
  /* There is no DAO-ACK, schedule a refresh. */
  schedule_dao_refresh();
#endif /* !RPL_WITH_DAO_ACK */

  /* Increment seqno */
  RPL_LOLLIPOP_INCREMENT(curr_instance.dag.dao_last_seqno);
  /* Send a DAO with own prefix as target and default lifetime */
  rpl_icmp6_dao_output(curr_instance.rplLifetime);
}
#endif /* RPL_WITH_DAO_ACK */
/*---------------------------------------------------------------------------*/
static void schedule_dao_refresh(void) {
  if(curr_instance.used && curr_instance.rplLifetime != RPL_INFINITE_LIFETIME) {
#if RPL_WITH_DAO_ACK
    /* DAO-ACK enabled: the last DAO was ACKed, wait until expiration before refresh */
	  uint32_t target_refresh = 1000 * RPL_LIFETIME(curr_instance.rplLifetime);
#else /* RPL_WITH_DAO_ACK */
    /* DAO-ACK disabled: use half the expiration time to get two chances to refresh per lifetime */
    uint32_t target_refresh = (1000 * RPL_LIFETIME(curr_instance.rplLifetime) / 2);
#endif /* RPL_WITH_DAO_ACK */

    /* Send between 60 and 120 seconds before target refresh */
    uint32_t safety_margin = (SECONDS_IN_MINUTE * 1000) + (System_Random(SECONDS_IN_MINUTE * 1000));

    if(target_refresh > safety_margin) {
      target_refresh -= safety_margin;
    }

    /* Schedule transmission */
    LpTimer_Arm(daoRefreshTimer, target_refresh);
  }
}
/*---------------------------------------------------------------------------*/
void rpl_timers_schedule_dao(void) {
  if(curr_instance.used && curr_instance.mop != RPL_MOP_NO_DOWNWARD_ROUTES) {
    /* No need for DAO aggregation delay as per RFC 6550 section 9.5, as this
    * only serves storing mode. Use simple delay instead, with the only purpose
    * to reduce congestion. */
	uint32_t expiration_time = RPL_DAO_DELAY / 2 + (System_Random(RPL_DAO_DELAY));
	LpTimer_Arm(daoRefreshTimer, expiration_time);
  }
}
#if RPL_WITH_DAO_ACK
/*---------------------------------------------------------------------------*/
/*------------------------------- DAO-ACK ---------------------------------- */
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
/* DAOs arrive in bursts - after a global repair every lamp re-registers at once - and each ACK is
 * sent once the DAO that asked for it has been processed. A single slot would keep only the last of
 * a burst, and a lamp left without an ACK retransmits until it gives up and repairs locally.
 * Length: the DAOs of a burst reach us as radio events queued ahead of the post that sends the ACKs,
 * so what can pile up before one dispatcher pass is bounded by that queue (RADIO_QUEUE_LENGTH, 10);
 * 12 keeps a margin. An ACK dropped here is not lost work - the lamp retransmits - but it costs that
 * lamp one of its 5 tries. */
#define DAO_ACK_QUEUE_LEN     12

static struct {
  uip_ipaddr_t target;
  uint16_t sequence;
} daoAckQueue[DAO_ACK_QUEUE_LEN];
static uint8_t daoAckFirst; /* oldest entry */
static uint8_t daoAckCount;
/*---------------------------------------------------------------------------*/
/* The queue needs no lock: entries are added from the DAO input path and taken by the work below,
 * and both run on the radio task - tcpip_input() has one caller, that task's event loop (radio.c). */
static void DropPendingDaoAcks(void) {
  daoAckFirst = 0;
  daoAckCount = 0;
}
/*---------------------------------------------------------------------------*/
static void DaoAckWork(void) {
  uip_ipaddr_t target;
  uint16_t sequence;

  if(daoAckCount == 0) {
    return;
  }
  uip_ipaddr_copy(&target, &daoAckQueue[daoAckFirst].target);
  sequence = daoAckQueue[daoAckFirst].sequence;
  daoAckFirst = (daoAckFirst + 1) % DAO_ACK_QUEUE_LEN;
  daoAckCount--;

  TRice("msg:Calling DAO ACK call from task.\n");
  rpl_icmp6_dao_ack_output(&target, sequence, RPL_DAO_ACK_UNCONDITIONAL_ACCEPT);

  if(daoAckCount > 0) {
    LpWork_Request(daoAckWork); /* One ACK per pass, so other radio work runs between them */
  }
}
/*---------------------------------------------------------------------------*/
void rpl_timers_schedule_dao_ack(uip_ipaddr_t *target, uint16_t sequence) {
  if(curr_instance.used) {
    if(daoAckCount < DAO_ACK_QUEUE_LEN) {
      uint8_t slot = (daoAckFirst + daoAckCount) % DAO_ACK_QUEUE_LEN;
      uip_ipaddr_copy(&daoAckQueue[slot].target, target);
      daoAckQueue[slot].sequence = sequence;
      daoAckCount++;
      TRice("msg:Requesting DAO ACK call from task.\n");
      LpWork_Request(daoAckWork);
    } else {
      TRiceS("wrn:DAO-ACK queue full - %s has to retransmit\n", uip6_printAddr(target, NULL));
    }
  } else {
	TRice("msg:Current instance not used - Do not ACK DAO.\n");
  }
}
/*---------------------------------------------------------------------------*/
void rpl_timers_notify_dao_ack(void) {
  /* The last DAO was ACKed. Schedule refresh to avoid route expiration.*/
  LpTimer_Disarm(daoResendTimer);
  schedule_dao_refresh();
}
/*---------------------------------------------------------------------------*/
static void DaoResendTmoHandler(void) {
  /* Increment transmission counter before sending */
  curr_instance.dag.dao_transmissions++;
  /* Send a DAO with own prefix as target and default lifetime */
  rpl_icmp6_dao_output(curr_instance.rplLifetime);

  /* Schedule next retransmission, or abort */
  if(curr_instance.dag.dao_transmissions < RPL_DAO_MAX_RETRANSMISSIONS) {
    schedule_dao_retransmission();
  } else {
    /* No more retransmissions. Perform local repair. */
    rpl_local_repair("DAO max rtx");
    return;
  }
}
#endif /* RPL_WITH_DAO_ACK */
/*---------------------------------------------------------------------------*/
#if RPL_WITH_PROBING
static rpl_nbr_t * get_probing_target(void)
{
  /* Returns the next probing target. The current implementation probes the urgent
   * probing target if any, or the preferred parent if its link statistics need refresh.
   * Otherwise, it picks at random between:
   * (1) selecting the best neighbor with non-fresh link statistics
   * (2) selecting the least recently updated neighbor
   */

  rpl_nbr_t *nbr;
  rpl_nbr_t *probing_target = NULL;
  rpl_rank_t probing_target_rank = RPL_INFINITE_RANK;

  if(curr_instance.used == 0) {
	TRice("msg:Not in an instance - do not probe.\n");
    return NULL;
  }

  /* There is an urgent probing target */
  if(curr_instance.dag.urgent_probing_target != NULL) {
	TRice("msg:Urgent target selected for probing.\n");
    return curr_instance.dag.urgent_probing_target;
  }

  /* The preferred parent needs probing */
  if(curr_instance.dag.preferred_parent != NULL && !rpl_neighbor_is_fresh(curr_instance.dag.preferred_parent)) {
	TRice("msg:Preferred parent selected for probing.\n");
    return curr_instance.dag.preferred_parent;
  }

  /* Now consider probing other non-fresh neighbors. With 2/3 proabability,
  pick the best non-fresh. Otherwise, pick the lest recently updated non-fresh. */

  if(System_Random(3) != 0) {
    /* Look for best non-fresh */
    nbr = nbr_table_head(rpl_neighbors);
    while(nbr != NULL) {
      if(!rpl_neighbor_is_fresh(nbr)) {
        /* nbr needs probing */
        rpl_rank_t nbr_rank = rpl_neighbor_rank_via_nbr(nbr);
        if((probing_target == NULL) || nbr_rank < probing_target_rank) {
          probing_target = nbr;
          probing_target_rank = nbr_rank;
        }
      }
      nbr = nbr_table_next(rpl_neighbors, nbr);
    }
  } else {
    /* Look for least recently updated non-fresh */
	uint32_t probing_target_age = 0;
    nbr = nbr_table_head(rpl_neighbors);
    while(nbr != NULL) {
      if(!rpl_neighbor_is_fresh(nbr)) {
        /* nbr needs probing */
        const struct link_stats *stats = rpl_neighbor_get_link_stats(nbr);
        if(stats != NULL) {
          if((probing_target == NULL) || (Time_GetUptime() - stats->last_tx_time > probing_target_age)) {
            probing_target = nbr;
            probing_target_age = Time_GetUptime() - stats->last_tx_time;
          }
        }
      }
      nbr = nbr_table_next(rpl_neighbors, nbr);
    }
  }

  return probing_target;
}
/*---------------------------------------------------------------------------*/
static void Probe(void) {
  rpl_nbr_t *probing_target = get_probing_target();
  uip_ipaddr_t *target_ipaddr = rpl_neighbor_get_ipaddr(probing_target);

  /* Perform probing */
  if(target_ipaddr != NULL) {
    const struct link_stats *stats = rpl_neighbor_get_link_stats(probing_target);
    (void)stats;
    TRiceS("sig:probing %s", uip6_printAddr(target_ipaddr, NULL));
    if (curr_instance.dag.urgent_probing_target != NULL) {
    	TRice("sig: (urgent) last tx %u min ago\n", (stats != NULL) ? ((uint16_t)((Time_GetUptime() - stats->last_tx_time) / SECONDS_IN_MINUTE)) : 0);
    } else {
    	TRice("sig: last tx %u min ago\n", (stats != NULL) ? ((uint16_t)((Time_GetUptime() - stats->last_tx_time) / SECONDS_IN_MINUTE)) : 0);
    }
    /* Send probe, e.g. unicast DIO or DIS */
    rpl_icmp6_dio_output(target_ipaddr);
    /* urgent_probing_target will be NULLed in the packet_sent callback */
  } else {
	  TRice("sig:probing rejected - no target found.\n");
  }
}
/*---------------------------------------------------------------------------*/
static void ProbingTmoHandler(void) {
  Probe();
  /* Schedule next probing */
  rpl_schedule_probing();
}
/*---------------------------------------------------------------------------*/
static void UrgProbingTmoHandler(void) {
  Probe(); /* The periodic probing keeps its own schedule */
}
/*---------------------------------------------------------------------------*/
void rpl_schedule_probing(void) {
  if (curr_instance.used) {
	if (!LpTimer_IsScheduled(probingTimer)) {
	  uint16_t rescheduleTmo = ((RPL_PROBING_INTERVAL) / 2) + System_Random(RPL_PROBING_INTERVAL);
	  TRice("sig:Schedule probing in %dmS\n", rescheduleTmo);
	  LpTimer_Arm(probingTimer, rescheduleTmo);
	} else {
	  TRice("sig:Probing not started - it is already running.\n");
	}
  } else {
	TRice("sig:Probing not started - instance not used.\n");
  }
}
/*---------------------------------------------------------------------------*/
void rpl_schedule_probing_now(void) {
  if(curr_instance.used) {
	if (!LpTimer_IsScheduled(urgProbingTimer)) {
	  TRice("sig:Schedule urgent probing in 4 sec.\n");
	  LpTimer_Arm(urgProbingTimer, System_Random(1000 * 4));
	} else {
	  TRice("sig:Urgent probing not started - it is already running.\n");
	}
  } else {
	TRice("sig:Urgent probing not started - instance not used.\n");
  }
}
#endif /* RPL_WITH_PROBING */
/*---------------------------------------------------------------------------*/
/*------------------------------- Leaving-- -------------------------------- */
/*---------------------------------------------------------------------------*/
static void LeavingTmoHandler(void) {
  if(curr_instance.used) {
    rpl_dag_leave();
  }
}
/*---------------------------------------------------------------------------*/
void rpl_timers_unschedule_leaving(void) {
  if(curr_instance.used) {
    LpTimer_Disarm(leaveTimer);
  }
}
/*---------------------------------------------------------------------------*/
void rpl_timers_schedule_leaving(void) {
  if(curr_instance.used) {
    if(!LpTimer_IsScheduled(leaveTimer)) {
      LpTimer_Arm(leaveTimer, RPL_DELAY_BEFORE_LEAVING);
    }
  }
}
/*---------------------------------------------------------------------------*/
/*------------------------------- Periodic---------------------------------- */
/*---------------------------------------------------------------------------*/
static void StateUpdateWork(void) {
  rpl_dag_update_state(NULL);
}
/*---------------------------------------------------------------------------*/
void rpl_timers_init(void) {
  periodicTimer = LpTimer_Create("6lowpan-rpl-periodicTimer", true, PeriodicTimerHandler);
  LpTimer_Arm(periodicTimer, PERIODIC_DELAY);
   /*DIS (DODAG Information Solicitation) message*/
  disTimer = LpTimer_Create("6lowpan-rpl-disPeriodicTimer", false, DisTmoHandler);
  LpTimer_Arm(disTimer, RPL_DIS_INTERVAL / 2 + System_Random(RPL_DIS_INTERVAL));

  dioTimer = LpTimer_Create("6lowpan-rpl-dioTimer", false, DioTmoHandler);
  leaveTimer = LpTimer_Create("6lowpan-rpl-leaveTimer", false, LeavingTmoHandler);
#if RPL_WITH_DAO_ACK
  daoResendTimer = LpTimer_Create("6lowpan-rpl-daoResendTimer", false, DaoResendTmoHandler);
  daoRefreshTimer = LpTimer_Create("6lowpan-rpl-daoRefreshTimer", false, DaoRefreshTmoHandler);
  daoAckWork = LpWork_Create(DaoAckWork);
#endif /* RPL_WITH_DAO_ACK */
#if RPL_WITH_PROBING
  probingTimer = LpTimer_Create("6lowpan-rpl-probingTimer", false, ProbingTmoHandler);
  urgProbingTimer = LpTimer_Create("6lowpan-rpl-urgProbingTimer", false, UrgProbingTmoHandler);
#endif /* RPL_WITH_PROBING */
  stateUpdateWork = LpWork_Create(StateUpdateWork);
  unicastDioWork = LpWork_Create(UnicastDioWork);
}
/*---------------------------------------------------------------------------*/
void
rpl_timers_stop_dag_timers(void)
{
  /* Stop all timers related to the DAG */
  LpTimer_Disarm(leaveTimer);
  LpTimer_Disarm(dioTimer);
  LpTimer_Disarm(daoResendTimer);
  LpTimer_Disarm(daoRefreshTimer);
#if RPL_WITH_PROBING
  LpTimer_Disarm(probingTimer);
#endif /* RPL_WITH_PROBING */
#if RPL_WITH_DAO_ACK
  /* Any ACK still waiting belongs to the DAG we are leaving */
  LpWork_Cancel(daoAckWork);
  DropPendingDaoAcks();
#endif /* RPL_WITH_DAO_ACK */
}
/*---------------------------------------------------------------------------*/
void rpl_timers_unschedule_state_update(void) {
  LpWork_Cancel(stateUpdateWork);
}
/*---------------------------------------------------------------------------*/
void rpl_timers_schedule_state_update(void) {
  if(curr_instance.used) {
	LpWork_Request(stateUpdateWork);
  }
}

/** @}*/
