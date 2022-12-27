/*
 * Copyright 2022, Cypress Semiconductor Corporation (an Infineon company) or
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
 * @file compat.h
 * @brief An attempt to make ThreadX/NetX more compatible with POSIX functions.
 */

#ifndef THREADXNETX_COMPAT_H_
#define THREADXNETX_COMPAT_H_

#ifdef __cplusplus
extern "C" {
#endif


#include "inet.h"
#include "netdb.h"

/* Include "nx_api.h" before "sockets.h" to avoid double definition of types
 * such as ULONG/USHORT.
 */

#define  AF_INET   2                    /* IPv4 socket (UDP, TCP, etc) */

#if defined(NETWORK_NetX)
#include "nx_api.h"
#include "NetX/sockets.h"
#include "netx_applications/dns/nx_dns.h"
#elif defined(NETWORK_NetX_Duo)
#include "nx_api.h"
#include "NetX_Duo/sockets.h"
#include "netx_applications/dns/nxd_dns.h"
#elif defined(NETWORK_LwIP)
#include "LwIP/iperf_sockets.h"
#endif



#ifdef __cplusplus
}
#endif

#ifndef socklen_t
#define socklen_t INT
#endif /* socklen_t */

#endif /* THREADXNETX_COMPAT_H_ */
