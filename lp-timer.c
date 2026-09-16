/*
 * lp-timer.c
 *
 *  Created on: Sep 16, 2026
 *      Author: laurynas
 */

#include "lp-timer.h"
#include "cmsis_os.h"
#include "App/common.h"

/* Enough for the stack as it stands: RPL takes 8 timers and 3 work items, uip-ds6 one timer and
 * link-stats one. Everything is created during initialisation, so a pool that is too small shows
 * up as an error on the first boot rather than later. */
#define LP_TIMER_NUM     12
#define LP_WORK_NUM       6

struct sLpWork {
  fLpWork cb;
  bool pending;
};

struct sLpTimer {
  struct sLpWork work;
  TimerHandle_t tim;
};

static struct sLpTimer timerPool[LP_TIMER_NUM];
static uint8_t timerCount;
static struct sLpWork workPool[LP_WORK_NUM];
static uint8_t workCount;

static uint16_t lpEvtIdOffset;
static fRadioEvtHndl lpEvtHndl;

static void RunPendingWork(void);
/*---------------------------------------------------------------------------*/
/* Marks work pending, returns whether it already was. Callers run on several tasks, so the flag is
 * read and set in one go. */
static bool MarkPending(struct sLpWork *work) {
  bool alreadyPending;
  taskENTER_CRITICAL();
  alreadyPending = work->pending;
  work->pending = true;
  taskEXIT_CRITICAL();
  return alreadyPending;
}
/*---------------------------------------------------------------------------*/
/* Clears pending work, returns whether there was any */
static bool TakePending(struct sLpWork *work) {
  bool pending;
  taskENTER_CRITICAL();
  pending = work->pending;
  work->pending = false;
  taskEXIT_CRITICAL();
  return pending;
}
/*---------------------------------------------------------------------------*/
static void PostRun(void) {
  if(NULL != lpEvtHndl) {
    lpEvtHndl(lpEvtIdOffset + radio_taskCall, RunPendingWork);
  }
}
/*---------------------------------------------------------------------------*/
/* Timer task: every timer created here expires in this callback. The post is unconditional, which is
 * what makes the whole mechanism survive a dropped post - the radio queue drops silently when full,
 * the mark stays, and the next post runs it, at the latest that of a periodic timer. A timer expires
 * at most once per arming, so posting from here cannot flood that queue. */
static void TimerExpired(TimerHandle_t tim) {
  struct sLpTimer *lpTim = (struct sLpTimer *)pvTimerGetTimerID(tim);

  if(NULL != lpTim) {
    MarkPending(&lpTim->work);
    PostRun();
  }
}
/*---------------------------------------------------------------------------*/
/* Radio task: runs everything pending. Each item is taken just before it runs, so work that an
 * earlier callback in the same pass cancelled (leaving stops the DAG timers) is skipped. */
static void RunPendingWork(void) {
  uint8_t i;

  for(i = 0; i < timerCount; i++) {
    if(TakePending(&timerPool[i].work)) {
      timerPool[i].work.cb();
    }
  }
  for(i = 0; i < workCount; i++) {
    if(TakePending(&workPool[i])) {
      workPool[i].cb();
    }
  }
}
/*---------------------------------------------------------------------------*/
void LpTimer_Init(uint16_t evtOffset, fRadioEvtHndl packedEvtHndl) {
  lpEvtIdOffset = evtOffset;
  lpEvtHndl = packedEvtHndl;
}
/*---------------------------------------------------------------------------*/
sLpTimer *LpTimer_Create(const char *name, bool autoReload, fLpWork cb) {
  struct sLpTimer *tim = NULL;

  if((NULL == cb) || (timerCount >= LP_TIMER_NUM)) {
    TRiceS("err:[6LP] no timer left in the pool for %s\n", (char*)name);
  } else {
    tim = &timerPool[timerCount];
    tim->work.cb = cb;
    tim->work.pending = false;
    /* The period is set on every arming, and a timer cannot be created with a period of 0 */
    tim->tim = xTimerCreate(name, pdMS_TO_TICKS(1), autoReload ? pdTRUE : pdFALSE, tim, TimerExpired);
    if(NULL == tim->tim) {
      TRiceS("err:[6LP] timer %s was not created\n", (char*)name);
      tim = NULL;
    } else {
      timerCount++;
    }
  }
  return tim;
}
/*---------------------------------------------------------------------------*/
void LpTimer_Arm(sLpTimer *tim, uint32_t ms) {
  if(NULL != tim) {
    /* A timer period is at least one tick. Callers compute timeouts, some of them random from 0 up
     * (urgent probing), so a timeout shorter than a tick is taken as the next tick. */
    TickType_t ticks = pdMS_TO_TICKS(ms);
    if(0 == ticks) {
      ticks = 1;
    }
    TakePending(&tim->work);
    xTimerChangePeriod(tim->tim, ticks, 0);
    xTimerStart(tim->tim, 0);
  }
}
/*---------------------------------------------------------------------------*/
void LpTimer_Disarm(sLpTimer *tim) {
  if(NULL != tim) {
    TakePending(&tim->work);
    xTimerStop(tim->tim, 0);
  }
}
/*---------------------------------------------------------------------------*/
bool LpTimer_IsScheduled(const sLpTimer *tim) {
  return (NULL != tim) && ((pdFALSE != xTimerIsTimerActive(tim->tim)) || tim->work.pending);
}
/*---------------------------------------------------------------------------*/
sLpWork *LpWork_Create(fLpWork cb) {
  struct sLpWork *work = NULL;

  if((NULL == cb) || (workCount >= LP_WORK_NUM)) {
    TRice("err:[6LP] no work slot left in the pool\n");
  } else {
    work = &workPool[workCount++];
    work->cb = cb;
    work->pending = false;
  }
  return work;
}
/*---------------------------------------------------------------------------*/
/* Posted only when the same work is not already waiting its turn. Callers ask at whatever rate their
 * own work happens - RPL asks for a state update once per neighbour removed, and one repair removes
 * up to NBR_TABLE_MAX_NEIGHBORS of them - and a post each would fill the radio queue and push out
 * whatever else was in flight. */
void LpWork_Request(sLpWork *work) {
  if((NULL != work) && !MarkPending(work)) {
    PostRun();
  }
}
/*---------------------------------------------------------------------------*/
bool LpWork_Cancel(sLpWork *work) {
  return (NULL != work) && TakePending(work);
}
