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
	radio_txFifoErr,
	radio_rxFifoErr,
	radio_rxDiscarded,
	radio_incomingData,
	radio_receivedData, //same as radio_incomingData, just comes from task run scope
	radio_irqToTaskCall,
	radio_taskCall, //typically to break nesting
	radio_dagLinkCreated,
	radio_dagLinkDestroyed,
	radio_pollUdp,
	radio_lastEvt
} eRadioEvent;

typedef void (*fRadioEvtHndl)(uint16_t, void*);

#ifdef __cplusplus
}
#endif

#endif /* THIRD_PARTY_6LOWPAN_EVT_RADIO_H_ */
