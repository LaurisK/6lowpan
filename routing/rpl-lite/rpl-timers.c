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
static TimerHandle_t disTimer; /* Not part of a DAG because when not joined */
static TimerHandle_t periodicTimer; /* Not part of a DAG because used for general state maintenance */

static uint16_t rplTimEvtIdOffset;
static fRadioEvtHndl rplTimEvtHndl;

/* Timer IDs, each one's bit in expiredTimers */
typedef enum {
  rplTim_periodic,
  rplTim_dis,
  rplTim_dio,
  rplTim_daoResend,
  rplTim_daoRefresh,
  rplTim_leave,
  rplTim_probing,
  rplTim_urgProbing,
  rplTim_last
} eRplTimer;

/* The RPL timers expire on the timer task, but their work runs on the radio task, which owns the RPL
 * state (DAG, neighbours, SR graph) and the transmit path. An expiry stays marked here until its work
 * has run, so one whose post was dropped (radio queue full) is run by the next post - at the latest
 * the periodic one. (Re)arming or stopping a timer from the radio task drops its unserviced expiry. */
static uint32_t expiredTimers;

static void RunExpiredTimers(void);
/*---------------------------------------------------------------------------*/
static uint32_t TimerBit(TimerHandle_t tim) {
  return 1UL << (uint32_t)(uintptr_t)pvTimerGetTimerID(tim);
}
/*---------------------------------------------------------------------------*/
/* Timer task: marks the expiry and hands the work to the radio task */
static void TimerExpired(TimerHandle_t tim) {
  taskENTER_CRITICAL();
  expiredTimers |= TimerBit(tim);
  taskEXIT_CRITICAL();
  rplTimEvtHndl(rplTimEvtIdOffset + radio_taskCall, RunExpiredTimers);
}
/*---------------------------------------------------------------------------*/
/* Clears an unserviced expiry, returns whether there was one */
static bool TakeExpired(uint32_t timBit) {
  bool expired;
  taskENTER_CRITICAL();
  expired = (0 != (expiredTimers & timBit));
  expiredTimers &= ~timBit;
  taskEXIT_CRITICAL();
  return expired;
}
/*---------------------------------------------------------------------------*/
/* Running, or expired with its work still to run */
static bool TimerIsScheduled(TimerHandle_t tim) {
  return (pdFALSE != xTimerIsTimerActive(tim)) || (0 != (expiredTimers & TimerBit(tim)));
}
/*---------------------------------------------------------------------------*/
static void TimerArm(TimerHandle_t tim, uint32_t ms) {
  TakeExpired(TimerBit(tim));
  xTimerChangePeriod(tim, pdMS_TO_TICKS(ms), 0);
  xTimerStart(tim, 0);
}
/*---------------------------------------------------------------------------*/
static void TimerDisarm(TimerHandle_t tim) {
  TakeExpired(TimerBit(tim));
  xTimerStop(tim, 0);
}
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
  if(!TimerIsScheduled(disTimer)) {
    TimerArm(disTimer, RPL_DIS_INTERVAL / 2 + System_Random(RPL_DIS_INTERVAL));
  }
}
/*---------------------------------------------------------------------------*/
/*------------------------------- DIO -------------------------------------- */
/*---------------------------------------------------------------------------*/
static void new_dio_interval(void) {
  uint32_t ticks;

  curr_instance.dag.dio_next_delay = 1UL << curr_instance.dag.dio_intcurrent;

  /* random number between I/2 and I */
  ticks = curr_instance.dag.dio_next_delay / 2 + System_Random(curr_instance.dag.dio_next_delay / 2);

  /*
   * The intervals must be equally long among the nodes for Trickle to
   * operate efficiently. Therefore we need to calculate the delay between
   * the randomized time and the start time of the next interval.
   */
  curr_instance.dag.dio_next_delay -= ticks;
  curr_instance.dag.dio_send = 1;
  /* reset the redundancy counter */
  curr_instance.dag.dio_counter = 0;

  /* schedule the timer */
  TimerArm(curr_instance.dag.dio_timer, ticks);
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

  if(curr_instance.dag.dio_send) {
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
    curr_instance.dag.dio_send = 0;
    TimerArm(curr_instance.dag.dio_timer, curr_instance.dag.dio_next_delay);
    TRice("msg:DIO timer continue for(%d)\n", curr_instance.dag.dio_next_delay);
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
static void handle_unicast_dio_timer(void* unused) {
  uip_ipaddr_t *target_ipaddr = rpl_neighbor_get_ipaddr(curr_instance.dag.unicast_dio_target);
  if(target_ipaddr != NULL) {
    rpl_icmp6_dio_output(target_ipaddr);
  }
}
/*---------------------------------------------------------------------------*/
void rpl_timers_schedule_unicast_dio(rpl_nbr_t *target) {
  if(curr_instance.used) {
    curr_instance.dag.unicast_dio_target = target;
    rplTimEvtHndl(rplTimEvtIdOffset + radio_taskCall, handle_unicast_dio_timer);
  }
}
/*---------------------------------------------------------------------------*/
/*------------------------------- DAO -------------------------------------- */
/*---------------------------------------------------------------------------*/
#if RPL_WITH_DAO_ACK
/*---------------------------------------------------------------------------*/
static void schedule_dao_retransmission(void) {
  uint32_t expiration_time = RPL_DAO_RETRANSMISSION_TIMEOUT / 2 + (System_Random(RPL_DAO_RETRANSMISSION_TIMEOUT));
  TimerArm(curr_instance.dag.timDaoResend, expiration_time);
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
    TimerArm(curr_instance.dag.timDaoRefresh, target_refresh);
  }
}
/*---------------------------------------------------------------------------*/
void rpl_timers_schedule_dao(void) {
  if(curr_instance.used && curr_instance.mop != RPL_MOP_NO_DOWNWARD_ROUTES) {
    /* No need for DAO aggregation delay as per RFC 6550 section 9.5, as this
    * only serves storing mode. Use simple delay instead, with the only purpose
    * to reduce congestion. */
	uint32_t expiration_time = RPL_DAO_DELAY / 2 + (System_Random(RPL_DAO_DELAY));
	TimerArm(curr_instance.dag.timDaoRefresh, expiration_time);
  }
}
#if RPL_WITH_DAO_ACK
/*---------------------------------------------------------------------------*/
/*------------------------------- DAO-ACK ---------------------------------- */
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
static void handle_dao_ack_timer(void* unused) {
  TRice("msg:Calling DAO ACK call from task.\n");
  rpl_icmp6_dao_ack_output(&curr_instance.dag.dao_ack_target, curr_instance.dag.dao_ack_sequence, RPL_DAO_ACK_UNCONDITIONAL_ACCEPT);
}
/*---------------------------------------------------------------------------*/
void rpl_timers_schedule_dao_ack(uip_ipaddr_t *target, uint16_t sequence) {
  if(curr_instance.used) {
    uip_ipaddr_copy(&curr_instance.dag.dao_ack_target, target);
    curr_instance.dag.dao_ack_sequence = sequence;
    TRice("msg:Requesting DAO ACK call from task.\n");
    rplTimEvtHndl(rplTimEvtIdOffset + radio_taskCall, handle_dao_ack_timer);
  } else {
	TRice("msg:Current instance not used - Do not ACK DAO.\n");
  }
}
/*---------------------------------------------------------------------------*/
void rpl_timers_notify_dao_ack(void) {
  /* The last DAO was ACKed. Schedule refresh to avoid route expiration.*/
  TimerDisarm(curr_instance.dag.timDaoResend);
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
	if (!TimerIsScheduled(curr_instance.dag.probing_timer)) {
	  uint16_t rescheduleTmo = ((RPL_PROBING_INTERVAL) / 2) + System_Random(RPL_PROBING_INTERVAL);
	  TRice("sig:Schedule probing in %dmS\n", rescheduleTmo);
	  TimerArm(curr_instance.dag.probing_timer, rescheduleTmo);
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
	if (!TimerIsScheduled(curr_instance.dag.urgProbeTmo)) {
	  TRice("sig:Schedule urgent probing in 4 sec.\n");
	  TimerArm(curr_instance.dag.urgProbeTmo, System_Random(1000 * 4));
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
    TimerDisarm(curr_instance.dag.leave);
  }
}
/*---------------------------------------------------------------------------*/
void rpl_timers_schedule_leaving(void) {
  if(curr_instance.used) {
    if(!TimerIsScheduled(curr_instance.dag.leave)) {
      xTimerStart(curr_instance.dag.leave, 0);
    }
  }
}
/*---------------------------------------------------------------------------*/
/*------------------------------- Periodic---------------------------------- */
/*---------------------------------------------------------------------------*/
/* Radio task: runs the work of every expired timer. Each expiry is taken just before its work runs,
 * so a timer that an earlier one's work stops or re-arms (e.g. leaving stops the DAG timers) is skipped. */
static void RunExpiredTimers(void) {
  static void (*const work[rplTim_last])(void) = {
    [rplTim_periodic]   = PeriodicTimerHandler,
    [rplTim_dis]        = DisTmoHandler,
    [rplTim_dio]        = DioTmoHandler,
#if RPL_WITH_DAO_ACK
    [rplTim_daoResend]  = DaoResendTmoHandler,
    [rplTim_daoRefresh] = DaoRefreshTmoHandler,
#endif /* RPL_WITH_DAO_ACK */
    [rplTim_leave]      = LeavingTmoHandler,
#if RPL_WITH_PROBING
    [rplTim_probing]    = ProbingTmoHandler,
    [rplTim_urgProbing] = UrgProbingTmoHandler,
#endif /* RPL_WITH_PROBING */
  };
  uint32_t id;

  for(id = 0; id < rplTim_last; id++) {
    if(TakeExpired(1UL << id) && (NULL != work[id])) {
      work[id]();
    }
  }
}
/*---------------------------------------------------------------------------*/
void rpl_timers_init(uint16_t evtOffset, fRadioEvtHndl packedEvtHndl) {
  rplTimEvtIdOffset = evtOffset;
  rplTimEvtHndl = packedEvtHndl;
  periodicTimer = xTimerCreate("6lowpan-rpl-periodicTimer", pdMS_TO_TICKS(PERIODIC_DELAY), pdTRUE, (void *)rplTim_periodic, TimerExpired);
  xTimerStart(periodicTimer, 0);
   /*DIS (DODAG Information Solicitation) message*/
  disTimer = xTimerCreate("6lowpan-rpl-disPeriodicTimer", pdMS_TO_TICKS(RPL_DIS_INTERVAL / 2 + System_Random(RPL_DIS_INTERVAL)), pdFALSE, (void *)rplTim_dis, TimerExpired);
  xTimerStart(disTimer, 0);

  curr_instance.dag.dio_timer = xTimerCreate("6lowpan-rpl-dioTimer", pdMS_TO_TICKS(1/*will set before starting*/), pdFALSE, (void *)rplTim_dio, TimerExpired);
  curr_instance.dag.timDaoResend = xTimerCreate("6lowpan-rpl-daoResendTimer", pdMS_TO_TICKS(1/*will set before starting*/), pdFALSE, (void *)rplTim_daoResend, TimerExpired);
  curr_instance.dag.timDaoRefresh = xTimerCreate("6lowpan-rpl-daoRefreshTimer", pdMS_TO_TICKS(1/*will set before starting*/), pdFALSE, (void *)rplTim_daoRefresh, TimerExpired);
  curr_instance.dag.leave = xTimerCreate("6lowpan-rpl-leaveTimer", pdMS_TO_TICKS(RPL_DELAY_BEFORE_LEAVING), pdFALSE, (void *)rplTim_leave, TimerExpired);
#if RPL_WITH_PROBING
  curr_instance.dag.probing_timer = xTimerCreate("6lowpan-rpl-probingTimer", pdMS_TO_TICKS(1/*will set before starting*/), pdFALSE, (void *)rplTim_probing, TimerExpired);
  curr_instance.dag.urgProbeTmo = xTimerCreate("6lowpan-rpl-probingTimer", pdMS_TO_TICKS(1/*will set before starting*/), pdFALSE, (void *)rplTim_urgProbing, TimerExpired);
#endif /* RPL_WITH_PROBING */
}
/*---------------------------------------------------------------------------*/
void
rpl_timers_stop_dag_timers(void)
{
  /* Stop all timers related to the DAG */
  TimerDisarm(curr_instance.dag.leave);
  TimerDisarm(curr_instance.dag.dio_timer);
  TimerDisarm(curr_instance.dag.timDaoResend);
  TimerDisarm(curr_instance.dag.timDaoRefresh);
#if RPL_WITH_PROBING
  TimerDisarm(curr_instance.dag.probing_timer);
#endif /* RPL_WITH_PROBING */
}
/*---------------------------------------------------------------------------*/
void rpl_timers_unschedule_state_update(void) {
//not as timer now - so no way to cancel
}
/*---------------------------------------------------------------------------*/
void rpl_timers_schedule_state_update(void) {
  if(curr_instance.used) {
	rplTimEvtHndl(rplTimEvtIdOffset + radio_taskCall, rpl_dag_update_state);
  }
}

/** @}*/
