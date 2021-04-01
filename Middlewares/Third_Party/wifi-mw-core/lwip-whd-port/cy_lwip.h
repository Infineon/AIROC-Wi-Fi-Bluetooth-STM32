/*
 * Copyright 2020 Cypress Semiconductor Corporation
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
* \addtogroup group_lwip_whd_port lwIP and WHD port
* \{
* Functions for dealing with linking the lwIP TCP/IP stack with WiFi Host Driver
*
* \defgroup group_lwip_whd_enums Enumerated Types
* \defgroup group_lwip_whd_port_functions Functions
* \defgroup group_lwip_whd_port_structures Structures
*/
#pragma once

#include <whd_wifi_api.h>
#include <cy_result.h>
#include <lwip/ip_addr.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \addtogroup group_lwip_whd_enums
 * \{
 */

/**
 * Enumeration of network interface role
 */
typedef enum
{
    CY_LWIP_STA_NW_INTERFACE = 0,  /**< STA or Client Interface  */
    CY_LWIP_AP_NW_INTERFACE  = 1   /**< SoftAP Interface         */
} cy_lwip_nw_interface_role_t;

/** \} group_lwip_whd_enums */

/**
* \addtogroup group_lwip_whd_port_structures
* \{
*/
/**
 * Structure used to pass LwIP network interface to \ref cy_lwip_add_interface
 */
typedef struct
{
    cy_lwip_nw_interface_role_t role;      /**< Network interface role */
    whd_interface_t             whd_iface; /**< WHD interface */
} cy_lwip_nw_interface_t;

/**
 * This structure represents a static IP address
 */
typedef struct ip_static
{
    ip_addr_t addr;    /**< The IP address for the network interface */
    ip_addr_t netmask; /**< The netmask for the network interface */
    ip_addr_t gateway; /**< The default gateway for network traffic */
} ip_static_addr_t;

/** \} group_lwip_whd_port_structures */

/**
* \addtogroup group_lwip_whd_port_functions
* \{
*/

/**
 * EAPOL handler to receive EAPOL data. buffer should be freed by EAPOL handler
 *
 * @param[in] buffer     buffer received from WHD.
 * @param[in] interface  WHD interface.
 *
 */
typedef void (*cy_eapol_packet_handler_t) (whd_buffer_t buffer, whd_interface_t whd_iface);

/**
 * Adds a network interface to LwIP and brings it up, a network interface contains a WHD Wi-Fi Interface and role such as STA, AP.
 * This is the entry point in this file. This function takes network interface adds to LwIP,
 * configures the optional static IP address and registers to IP change callback.
 *
 * \note static IP address is mandatory for AP interface.
 *
 * @param[in] iface      Network interface to be added.
 * @param[in] ipaddr     IPv4/IPv6 address information associated with the interface.
 *
 * @return CY_RESULT_SUCCESS for successful addition to LwIP or error otherwise.
 */
cy_rslt_t cy_lwip_add_interface(cy_lwip_nw_interface_t *iface, ip_static_addr_t *ipaddr) ;

/**
 * Removes a network interface from LwIP.
 *
 * @param[in] iface     Network interface to be removed.
 *
 * @return CY_RSLT_SUCCESS if successful, failure code otherwise.
 */
cy_rslt_t cy_lwip_remove_interface(cy_lwip_nw_interface_t *iface);

/**
 * Return the LwIP network interface for the given network interface role.
 *
 * @param[in] role      Interface role.
 *
 * @return netif structure of the WHD interface, null if the interface type passed is invalid.
 */
struct netif* cy_lwip_get_interface(cy_lwip_nw_interface_role_t role);

/**
 * This function brings up the network link layer and
 *  and starts DHCP if required
 *
 * @param[in] iface      Network interface to be brought up.
 *
 *  @return CY_RSLT_SUCCESS on successful network bring up, failure code otherwise.
 */
cy_rslt_t cy_lwip_network_up(cy_lwip_nw_interface_t *iface);

/**
 * This function brings down the network interface, brings down the network link layer
 * and stops DHCP
 *
 * @param[in] iface      LwIP Interface.
 *
 * @return CY_RSLT_SUCCESS on successful tear down of the network, failure code otherwise.
 */
cy_rslt_t cy_lwip_network_down(cy_lwip_nw_interface_t *iface);

#if LWIP_IPV4
/**
 * This function Invalidates all the ARP entries and renews the DHCP.
 * This function is typically used when handshake failure occurs on a STA interface.
 *
 * @param[in] iface      LwIP Interface to be renewed.
 *
 * @return CY_RSLT_SUCCESS if successful, failure code otherwise.
 */
cy_rslt_t cy_lwip_dhcp_renew(cy_lwip_nw_interface_t *iface);
#endif
/**
 *
 * This function takes packets from the radio driver and passes them into the
 * lwIP stack.  If the stack is not initialized, or if the LwIP stack does not
 * accept the packet, the packet is freed (dropped).
 * This function will be registered as part of the whd_netif_funcs defined in whd.h
 * of the WiFi Host Driver.
 *
 * @param[in] iface WiFi interface.
 * @param[in] buf Packet received from the radio driver.
 * 
 */
extern void cy_network_process_ethernet_data(whd_interface_t iface, whd_buffer_t buf);

/**
 * Network activity callback function prototype
 * Callback function which can be registered/unregistered for any network activity
 * needs to be of this prototype
 */
typedef void (*cy_network_activity_event_callback_t)(bool callback_arg);

/**
 * IP change callback function prototype
 * Callback function which can be registered to receive IP changes
 * needs to be of this prototype
 */
typedef void (*cy_lwip_ip_change_callback_t)(void *data);

/**
 * This function helps to register/unregister callback fn for any TX/RX packets.
 * Passing "NULL" as cb will deregister the activity callback
 *
 * @param[in] cb Network activity callback function
 * 
 */
void cy_network_activity_register_cb(cy_network_activity_event_callback_t cb);

/**
 * This function helps to register/unregister for any IP changes from LwIP.
 * Passing "NULL" as callback will deregister the IP changes callback
 *
 * @param[in] cb IP change callback function
 *
 */
void cy_lwip_register_ip_change_cb(cy_lwip_ip_change_callback_t cb);

/**
 *
 * This API allows registering callback functions to receive EAPOL packets
 * from WHD. If callback is registered and received packet is EAPOL packet
 * then it will be directly redirected to registered callback. passing "NULL"
 * as handler will de-register the previously registered callback
 *
 * @param[in] eapol_packet_handler
 *
 * @return whd_result_t
 *
 */
whd_result_t cy_eapol_register_receive_handler(cy_eapol_packet_handler_t eapol_packet_handler);

/** \} group_lwip_whd_port_functions */
#ifdef __cplusplus
}
#endif
/** \} group_lwip_whd_port */

