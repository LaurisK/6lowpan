/*
 * lp-timer.h
 *
 *  Created on: Sep 16, 2026
 *      Author: laurynas
 */

#ifndef THIRD_PARTY_6LOWPAN_LP_TIMER_H_
#define THIRD_PARTY_6LOWPAN_LP_TIMER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "evt_radio.h"

/*
 * Timed work for the 6LoWPAN stack.
 *
 * Every callback registered here runs on the radio task, which owns the stack state (neighbours,
 * routes, SR graph, DAG) and the transmit path - the single context Contiki assumes for its own
 * timers, and which this port lost by mapping them straight onto FreeRTOS software timers. Only
 * this module knows that the waiting is done with those timers, so no other file needs the RTOS
 * headers to schedule something.
 *
 * A callback takes no arguments. It runs in the radio task's event loop, ahead of the packets
 * queued behind it, so it should do no more work than a packet handler would.
 */
typedef void (*fLpWork)(void);

typedef struct sLpTimer sLpTimer; /* Opaque: created here, held by the caller as a pointer */
typedef struct sLpWork sLpWork;

/** Prepares the module. Call once, before anything creates a timer or work. */
void LpTimer_Init(uint16_t evtOffset, fRadioEvtHndl packedEvtHndl);

/** Creates a timer. autoReload repeats until disarmed, otherwise it runs once per arming.
 *  Returns NULL when the pool is exhausted; every call below tolerates that. */
sLpTimer *LpTimer_Create(const char *name, bool autoReload, fLpWork cb);

/** (Re)starts the timer with this timeout, dropping an expiry of it that has not run yet.
 *  Any timeout is valid; one shorter than a tick (0 included) expires on the next tick. */
void LpTimer_Arm(sLpTimer *tim, uint32_t ms);

/** Stops the timer, dropping an expiry of it that has not run yet. */
void LpTimer_Disarm(sLpTimer *tim);

/** Running, or expired with its callback still to run. */
bool LpTimer_IsScheduled(const sLpTimer *tim);

/** Creates work that runs when asked for rather than on a timeout. */
sLpWork *LpWork_Create(fLpWork cb);

/** Asks for the work to run on the radio task. Asking again before it runs is still one run. */
void LpWork_Request(sLpWork *work);

/** Drops a request that has not run yet, returns whether there was one. */
bool LpWork_Cancel(sLpWork *work);

#ifdef __cplusplus
}
#endif

#endif /* THIRD_PARTY_6LOWPAN_LP_TIMER_H_ */
