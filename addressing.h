/*
 * addressing.h
 *
 *  Created on: Oct 6, 2025
 *      Author: laurynas
 */

#ifndef THIRD_PARTY_6LOWPAN_ADDRESSING_H_
#define THIRD_PARTY_6LOWPAN_ADDRESSING_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "linkaddr.h"
#include "network/uip.h"


void Addr_SetInterfId(uip_ipaddr_t*, uip_lladdr_t*);
void Addr_GetInterfId(uip_lladdr_t*, const uip_ipaddr_t*);

#ifdef __cplusplus
}
#endif

#endif /* THIRD_PARTY_6LOWPAN_ADDRESSING_H_ */
