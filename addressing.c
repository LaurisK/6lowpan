/*
 * addressing.c
 *
 *  Created on: Oct 6, 2025
 *      Author: laurynas
 */

#include "addressing.h"

/*---------------------------------------------------------------------------*/
/** \brief set the last 64 bits of an IP address based on the MAC address */
void Addr_SetInterfId(uip_ipaddr_t *ipaddr, uip_lladdr_t *lladdr) {
#if (UIP_LLADDR_LEN == 8)
  memcpy(ipaddr->u8 + 8, lladdr, UIP_LLADDR_LEN);
  ipaddr->u8[8] ^= 0x02;
#elif (UIP_LLADDR_LEN == 6)
  memcpy(ipaddr->u8 + 8, lladdr, 3);
  ipaddr->u8[11] = 0xff;
  ipaddr->u8[12] = 0xfe;
  memcpy(ipaddr->u8 + 13, (uint8_t *)lladdr + 3, 3);
  ipaddr->u8[8] ^= 0x02;
#elif (UIP_LLADDR_LEN == 2)
  /* derive IID as per RFC 6282 */
  memcpy(ipaddr->u8 + 8, iid_prefix, 6);
  memcpy(ipaddr->u8 + 8 + 6, lladdr, UIP_LLADDR_LEN);
#else
#error uip-ds6.c cannot build interface address when UIP_LLADDR_LEN is not 6, 8, or 2
#endif
}
/*---------------------------------------------------------------------------*/

/** \brief Build a link-layer address from an IPv6 address based on its UUID64 */
void Addr_GetInterfId(uip_lladdr_t *lladdr, const uip_ipaddr_t *ipaddr) {
#if (UIP_LLADDR_LEN == 8)
  memcpy(lladdr, ipaddr->u8 + 8, UIP_LLADDR_LEN);
  lladdr->addr[0] ^= 0x02;
#elif (UIP_LLADDR_LEN == 2)
  memcpy(lladdr, ipaddr->u8 + 6, UIP_LLADDR_LEN);
#else
#error uip-ds6.c cannot build lladdr address when UIP_LLADDR_LEN is not 8 or 2
#endif
}

