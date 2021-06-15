/*
 * Copyright 2021, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
*/

/**
* \addtogroup group_lwip_whd_port lwIP and WHD port
* \{
* Functions for dealing with linking the lwIP TCP/IP stack with Wi-Fi Host Driver
*
* \defgroup group_lwip_whd_enums Enumerated Types
* \defgroup group_lwip_whd_port_functions Port Functions
* \defgroup group_lwip_whd_port_structures Port Structures
*/
#pragma once

#include "whd_wifi_api.h"
#include "cy_result.h"
#include "lwip/ip_addr.h"

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

/**
 * Enumeration of network activity type
 */
typedef enum
{
    CY_NETWORK_ACTIVITY_TX = 0,  /**< TX network activity  */
    CY_NETWORK_ACTIVITY_RX = 1   /**< RX network activity  */
} cy_network_activity_type_t;

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
 * of the Wi-Fi Host Driver.
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
 * This function notifies network activity to Low Power Assistant(LPA) module.
 *
 * @param[in] activity_type Network activity type \ref cy_network_activity_type_t to be notified to Low Power Assistant(LPA) module.
 *
 * @return CY_RSLT_SUCCESS if successful, failure code otherwise.
 */
cy_rslt_t cy_network_activity_notify(cy_network_activity_type_t activity_type);

/**
 * This function helps to register/unregister for any IP changes from LwIP.
 * Passing "NULL" as callback will deregister the IP changes callback
 *
 * @param[in] cb IP change callback function
 *
 */
void cy_lwip_register_ip_change_cb(cy_lwip_ip_change_callback_t cb);


/** \} group_lwip_whd_port_functions */
#ifdef __cplusplus
}
#endif
/** \} group_lwip_whd_port */
