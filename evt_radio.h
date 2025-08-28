/*
 * evt_radio.h
 *
 *  Created on: Aug 27, 2025
 *      Author: laurynas
 */

#ifndef THIRD_PARTY_6LOWPAN_EVT_RADIO_H_
#define THIRD_PARTY_6LOWPAN_EVT_RADIO_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	radio_incomingData,
	radio_irqToTaskCall,
	radio_taskCall, //typically to break nesting
	radio_lastEvt
} eRadioEvent;

#ifdef __cplusplus
}
#endif

#endif /* THIRD_PARTY_6LOWPAN_EVT_RADIO_H_ */
