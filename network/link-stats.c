/*
 * Copyright (c) 2015, SICS Swedish ICT.
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
 *
 * Authors: Simon Duquennoy <simonduq@sics.se>
 */

#include "link-stats.h"
#include "nbr-table.h"
#include "../mac/mac.h"
#include "App/Time/time.h"
#include "cmsis_os.h"

/* Maximum value for the Tx count counter */
#define TX_COUNT_MAX                    32

/* Statistics with no update in FRESHNESS_EXPIRATION_TIMEOUT is not fresh */
#define FRESHNESS_EXPIRATION_TIME       (10 * SECONDS_IN_MINUTE)
/* Half time for the freshness counter */
#define FRESHNESS_HALF_LIFE             (15 * SECONDS_IN_MINUTE)
/* Statistics are fresh if the freshness counter is FRESHNESS_TARGET or more */
#define FRESHNESS_TARGET                 4
/* Maximum value for the freshness counter */
#define FRESHNESS_MAX                   16

/* EWMA (exponential moving average) used to maintain statistics over time */
#define EWMA_SCALE                     100
#define EWMA_ALPHA                      10
#define EWMA_BOOTSTRAP_ALPHA            25

/* ETX fixed point divisor. 128 is the value used by RPL (RFC 6551 and RFC 6719) */
#define ETX_DIVISOR                     LINK_STATS_ETX_DIVISOR
/* In case of no-ACK, add ETX_NOACK_PENALTY to the real Tx count, as a penalty */
#define ETX_NOACK_PENALTY               12
/* Initial ETX value */
#define ETX_DEFAULT                      2

#define RSSI_DIFF (LINK_STATS_RSSI_HIGH - LINK_STATS_RSSI_LOW)

/* Generate error on incorrect link stats configuration values */
#if RSSI_DIFF <= 0
#error "RSSI_HIGH must be greater then RSSI_LOW"
#endif

/* Generate error if the initial ETX calculation would overflow uint16_t */
#if ETX_DIVISOR * RSSI_DIFF >= 0x10000
#error "RSSI math overflow"
#endif

/* Per-neighbor link statistics table */
NBR_TABLE(struct link_stats, link_stats);

/* Called at a period of FRESHNESS_HALF_LIFE */
static TimerHandle_t periodicTimer;

/*---------------------------------------------------------------------------*/
/* Returns the neighbor's link stats */
const struct link_stats *
link_stats_from_lladdr(const linkaddr_t *lladdr)
{
  return nbr_table_get_from_lladdr(link_stats, lladdr);
}
/*---------------------------------------------------------------------------*/
/* Returns the neighbor's address given a link stats item */
const linkaddr_t *
link_stats_get_lladdr(const struct link_stats *stat)
{
  return nbr_table_get_lladdr(link_stats, stat);
}
/*---------------------------------------------------------------------------*/
/* Are the statistics fresh? */
int
link_stats_is_fresh(const struct link_stats *stats)
{
  return (stats != NULL)
      && Time_GetUptime() - stats->last_tx_time < FRESHNESS_EXPIRATION_TIME
      && stats->freshness >= FRESHNESS_TARGET;
}
/*---------------------------------------------------------------------------*/
#if LINK_STATS_INIT_ETX_FROM_RSSI
/*
 * Returns initial ETX value from an RSSI value.
 *    RSSI >= RSSI_HIGH           -> use default ETX
 *    RSSI_LOW < RSSI < RSSI_HIGH -> ETX is a linear function of RSSI
 *    RSSI <= RSSI_LOW            -> use maximal initial ETX
 *
 * The old form computed ETX_DIVISOR * RSSI_DIFF / (rssi - RSSI_LOW), a reciprocal rather
 * than the linear relation the comment claimed, and one that bottomed out at ETX 1.0 for
 * a strong link - better than the ETX_DEFAULT of 2.0 given to a neighbour with no RSSI
 * at all. A first sample could therefore make a link look better than measured, biasing
 * initial parent selection toward whichever neighbour happened to be loudest.
 */
static uint16_t
guess_etx_from_rssi(const struct link_stats *stats)
{
  if(stats != NULL) {
    if(stats->rssi == LINK_STATS_RSSI_UNKNOWN) {
      return ETX_DEFAULT * ETX_DIVISOR;
    } else {
      const int16_t rssi_delta = LINK_STATS_RSSI_HIGH - stats->rssi;
      const int16_t bounded_rssi_delta = MIN(MAX(rssi_delta, 0), RSSI_DIFF);
      /* Penalty is in the range from 0 to ETX_DIVISOR */
      const uint16_t penalty = ETX_DIVISOR * bounded_rssi_delta / RSSI_DIFF;
      /* ETX is the default ETX value + penalty */
      const uint16_t etx = ETX_DIVISOR * ETX_DEFAULT + penalty;
      return MIN(etx, LINK_STATS_ETX_INIT_MAX * ETX_DIVISOR);
    }
  }
  return 0xffff;
}
#endif /* LINK_STATS_INIT_ETX_FROM_RSSI */
/*---------------------------------------------------------------------------*/
/* Packet sent callback. Updates stats for transmissions to lladdr */
void link_stats_packet_sent(const linkaddr_t *lladdr, int status, int numtx) {
  struct link_stats *stats;
#if !LINK_STATS_ETX_FROM_PACKET_COUNT
  uint16_t packet_etx;
  uint8_t ewma_alpha;
#endif /* !LINK_STATS_ETX_FROM_PACKET_COUNT */

  if(status != MAC_TX_OK && status != MAC_TX_NOACK) {
    /* Do not penalize the ETX when collisions or transmission errors occur. */
    return;
  }

  stats = nbr_table_get_from_lladdr(link_stats, lladdr);
  if(stats == NULL) {
    /* If transmission failed, do not add the neighbor, as the neighbor might not exist anymore */
    if(status != MAC_TX_OK) {
      return;
    }

    /* Add the neighbor */
    stats = nbr_table_add_lladdr(link_stats, lladdr, NBR_TABLE_REASON_LINK_STATS, NULL);
    if(stats == NULL) {
      return; /* No space left, return */
    }
    /* nbr_table_add_lladdr() zeroes the entry, and 0 dBm is a legal RSSI, so mark the
       field explicitly as "never sampled". Without this an entry created here - i.e. by
       a successful transmission to a neighbour we have not yet heard from - would later
       have its first real RSSI reading averaged against 0 by the EWMA below, dragging
       the estimate toward 0 dBm for tens of packets. */
    stats->rssi = LINK_STATS_RSSI_UNKNOWN;
    /* etx is left at zero deliberately. There is no RSSI to guess from on this path, and
       the transmission that created the entry is itself a measurement - it seeds etx
       directly below, rather than an EWMA having to decay away from a placeholder. */
  }

  /* Update last timestamp and freshness */
  stats->last_tx_time = Time_GetUptime();
  stats->freshness = MIN(stats->freshness + numtx, FRESHNESS_MAX);

#if LINK_STATS_PACKET_COUNTERS
  /* Update paket counters */
  stats->cnt_current.num_packets_tx += numtx;
  if(status == MAC_TX_OK) {
    stats->cnt_current.num_packets_acked++;
  }
#endif

  /* Add penalty in case of no-ACK */
  if(status == MAC_TX_NOACK) {
    numtx += ETX_NOACK_PENALTY;
  }

#if LINK_STATS_ETX_FROM_PACKET_COUNT
  /* Compute ETX from packet and ACK count */
  /* Halve both counter after TX_COUNT_MAX */
  if(stats->tx_count + numtx > TX_COUNT_MAX) {
    stats->tx_count /= 2;
    stats->ack_count /= 2;
  }
  /* Update tx_count and ack_count */
  stats->tx_count += numtx;
  if(status == MAC_TX_OK) {
    stats->ack_count++;
  }
  /* Compute ETX */
  if(stats->ack_count > 0) {
    stats->etx = ((uint16_t)stats->tx_count * ETX_DIVISOR) / stats->ack_count;
  } else {
    stats->etx = (uint16_t)MAX(ETX_NOACK_PENALTY, stats->tx_count) * ETX_DIVISOR;
  }
#else /* LINK_STATS_ETX_FROM_PACKET_COUNT */
  /* Compute ETX using an EWMA */

  /* ETX used for this update */
  packet_etx = numtx * ETX_DIVISOR;
  /* ETX alpha used for this update */
  ewma_alpha = link_stats_is_fresh(stats) ? EWMA_ALPHA : EWMA_BOOTSTRAP_ALPHA;

  if(stats->etx == 0) {
    /* First measurement for this neighbour - seed ETX with it instead of averaging
       against the zero left by nbr_table_add_lladdr(). */
    stats->etx = packet_etx;
  } else {
    /* Compute EWMA and update ETX */
    stats->etx = ((uint32_t)stats->etx * (EWMA_SCALE - ewma_alpha) +
        (uint32_t)packet_etx * ewma_alpha) / EWMA_SCALE;
  }
#endif /* LINK_STATS_ETX_FROM_PACKET_COUNT */
}
/*---------------------------------------------------------------------------*/
/* Packet input callback. Updates statistics for receptions on a given link */
void link_stats_input_callback(const linkaddr_t *lladdr, int16_t rssi) {
  struct link_stats *stats;

  stats = nbr_table_get_from_lladdr(link_stats, lladdr);
  if(stats == NULL) {
    /* Add the neighbor */
    stats = nbr_table_add_lladdr(link_stats, lladdr, NBR_TABLE_REASON_LINK_STATS, NULL);
    if(stats == NULL) {
      return; /* No space left, return */
    }
    stats->rssi = LINK_STATS_RSSI_UNKNOWN;
  }

  if(stats->rssi == LINK_STATS_RSSI_UNKNOWN) {
    /* First sample for this neighbour - seed the average rather than blending against
       the sentinel. */
    stats->rssi = rssi;
  } else {
    /* Update RSSI EWMA */
    stats->rssi = ((int32_t)stats->rssi * (EWMA_SCALE - EWMA_ALPHA) + (int32_t)rssi * EWMA_ALPHA) / EWMA_SCALE;
  }

  if(stats->etx == 0) {
    /* Initialize ETX, now that there is an RSSI to derive it from. */
#if LINK_STATS_INIT_ETX_FROM_RSSI
    stats->etx = guess_etx_from_rssi(stats);
#else /* LINK_STATS_INIT_ETX_FROM_RSSI */
    stats->etx = ETX_DEFAULT * ETX_DIVISOR;
#endif /* LINK_STATS_INIT_ETX_FROM_RSSI */
  }

#if LINK_STATS_PACKET_COUNTERS
  stats->cnt_current.num_packets_rx++;
#endif
}
/*---------------------------------------------------------------------------*/
#if LINK_STATS_PACKET_COUNTERS
/*---------------------------------------------------------------------------*/
static void
print_and_update_counters(void)
{
  struct link_stats *stats;

  for(stats = nbr_table_head(link_stats); stats != NULL; stats = nbr_table_next(link_stats, stats)) {
    struct link_packet_counter *c = &stats->cnt_current;

	TRiceS("msg:[LINK_STATS] %s ", (char*)linkaddr_printAddr(link_stats_get_lladdr(stats)));
	TRice("msg:/t TX(%d), ACK's(%d), RX(%d)\n", c->num_packets_tx, c->num_packets_acked, c->num_packets_rx);

    stats->cnt_total.num_packets_tx += stats->cnt_current.num_packets_tx;
    stats->cnt_total.num_packets_acked += stats->cnt_current.num_packets_acked;
    stats->cnt_total.num_packets_rx += stats->cnt_current.num_packets_rx;
    memset(&stats->cnt_current, 0, sizeof(stats->cnt_current));
  }
}
/*---------------------------------------------------------------------------*/
#endif /* LINK_STATS_PACKET_COUNTERS */
/*---------------------------------------------------------------------------*/
/* Periodic timer called at a period of FRESHNESS_HALF_LIFE */
static void periodic(TimerHandle_t periodicTim)
{
  /* Age (by halving) freshness counter of all neighbors */
  struct link_stats *stats;
  for(stats = nbr_table_head(link_stats); stats != NULL; stats = nbr_table_next(link_stats, stats)) {
    stats->freshness >>= 1;
  }

#if LINK_STATS_PACKET_COUNTERS
  print_and_update_counters();
#endif
}
/*---------------------------------------------------------------------------*/
/* Resets link-stats module */
void
link_stats_reset(void)
{
  struct link_stats *stats;
  stats = nbr_table_head(link_stats);
  while(stats != NULL) {
    nbr_table_remove(link_stats, stats);
    stats = nbr_table_next(link_stats, stats);
  }
}
/*---------------------------------------------------------------------------*/
/* Initializes link-stats module */
void
link_stats_init(void)
{
  nbr_table_register("link statistics", link_stats, NULL, LAYER_MAC);
  periodicTimer = xTimerCreate("6lowpan-linkStats-periodicTimer", pdMS_TO_TICKS(FRESHNESS_HALF_LIFE * 1000), pdTRUE, 0, periodic);
  xTimerStart(periodicTimer, 0);
}
