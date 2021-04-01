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


/** @file
 *  Interface header for a simple DHCP server
 */

#pragma once

#if LWIP_IPV4

#include "cy_lwip.h"
#include "cy_worker_thread.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

#define ALWAYS_INLINE_PRE
//#define ALWAYS_INLINE    __attribute__((always_inline))
/* Fixme: ALWAYS_INLINE should be set based on compiler GCC or IAR. but for
 * now it is forced inline is removed. Need to identify how to detect compiler flag here
 */
#define ALWAYS_INLINE

#ifndef htobe32   /* This is defined in POSIX platforms */
ALWAYS_INLINE_PRE static inline ALWAYS_INLINE uint32_t htobe32(uint32_t v)
{
    return (uint32_t)(((v&0x000000FF) << 24) | ((v&0x0000FF00) << 8) | ((v&0x00FF0000) >> 8) | ((v&0xFF000000) >> 24));
}
#endif /* ifndef htobe32 */

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/**
 * IP Version
 */
typedef enum
{
    CY_LWIP_IP_VER_V4 = 4,      /**< Denotes IPv4 version. */
    CY_LWIP_IP_VER_V6 = 6       /**< Denotes IPv6 version. */
} cy_lwip_ip_version_t;


/******************************************************
 *                    Structures
 ******************************************************/

/**
 * Structure used to store the IP address information
 */
typedef struct
{
    cy_lwip_ip_version_t version;  /**< IP version. */
    union
    {
        uint32_t v4;     /**< IPv4 address in network byte order. */
        uint32_t v6[4];  /**< IPv6 address in network byte order. */
    } ip;                /**< IP address bytes. */
} cy_lwip_ip_address_t;

/**
 * Structure for storing a MAC address (Wi-Fi Media Access Control address).
 */
typedef struct
{
    uint8_t octet[6]; /**< Unique 6-byte MAC address */
} cy_lwip_mac_addr_t;

typedef struct cy_lwip_udp_socket_struct cy_lwip_udp_socket_t;

struct cy_lwip_udp_socket_struct
{
    int                           socket;
    struct netconn                *conn_handler;
    ip_addr_t                     local_ip_addr;
    bool                          is_bound;
    cy_lwip_nw_interface_role_t   role;
};

typedef struct
{
    cy_thread_t                  thread;
    cy_lwip_udp_socket_t         socket;
    volatile bool                quit;
    cy_lwip_nw_interface_role_t  role;
} cy_lwip_dhcp_server_t;

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/
/*****************************************************************************/
/**
 *
 *                   DHCP Server
 *
 * Communication functions for DHCP server
 *
 *
 */
/*****************************************************************************/

/**
 *  Start a DHCP server instance.
 *
 * @param[in] server      Structure that will be used for this DHCP server instance allocated by caller, @ref cy_lwip_dhcp_server_t.
 * @param[in] iface_type  Which network interface the DHCP server should listen on.
 *
 * @return CY_RSLT_SUCCESS if successful, failure code otherwise.
 */
cy_rslt_t cy_lwip_dhcp_server_start(cy_lwip_dhcp_server_t *server, cy_lwip_nw_interface_role_t iface_type);


/**
 *  Stop a DHCP server instance.
 *
 * @param[in] server     Structure workspace for the DHCP server instance - as used with @ref cy_lwip_dhcp_server_t.
 *
 * @return CY_RSLT_SUCCESS if successful, failure code otherwise.
 */
cy_rslt_t cy_lwip_dhcp_server_stop(cy_lwip_dhcp_server_t *server);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif //LWIP_IPV4
