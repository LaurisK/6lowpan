/*
 * rpl-nbr-policy.h
 *
 *  Created on: Oct 21, 2025
 *      Author: laurynas
 */

#ifndef THIRD_PARTY_6LOWPAN_ROUTING_RPL_LITE_RPL_NBR_POLICY_H_
#define THIRD_PARTY_6LOWPAN_ROUTING_RPL_LITE_RPL_NBR_POLICY_H_

#include "rpl-icmp6.h"
#include "../../network/uip.h"

#define NBR_TABLE_FIND_REMOVABLE

const linkaddr_t * rpl_nbr_policy_find_removable(nbr_table_reason_t reason, void *data);

#endif /* THIRD_PARTY_6LOWPAN_ROUTING_RPL_LITE_RPL_NBR_POLICY_H_ */
