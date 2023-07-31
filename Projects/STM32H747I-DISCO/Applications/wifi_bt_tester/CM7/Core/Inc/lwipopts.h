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

/*
 * Copyright (c) 2001-2003 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Simon Goldschmidt
 *
 */
#pragma once

#include <whd_types.h>
#include "cyabs_rtos.h"

#define MEM_ALIGNMENT                   (4)

#define LWIP_RAW                        (1)

//
// Enable IPV4 networking
//
#define LWIP_IPV4                       (1)

/**
 * LWIP_AUTOIP==1: Enable AUTOIP module.
 */
// #define LWIP_AUTOIP                  (1)

/**
 * LWIP_DHCP_AUTOIP_COOP==1: Allow DHCP and AUTOIP to be both enabled on
 * the same interface at the same time.
 */
// #define LWIP_DHCP_AUTOIP_COOP        (1)

//
// Enable IPV6 networking
//
#define LWIP_IPV6                       (0)

#define ETHARP_SUPPORT_STATIC_ENTRIES   (1)

//
// Enable IPV4 networking
//
#define LWIP_ICMP                       (1)
#define LWIP_TCP                        (1)
#define LWIP_UDP                        (1)
#define LWIP_IGMP                       (1)

//
// Use malloc to allocate any memory blocks instead of the
// malloc that is part of LWIP
//
#define MEM_LIBC_MALLOC                 (1)

//
// The standard library does not provide errno, use the one
// from LWIP.
//
#define LWIP_PROVIDE_ERRNO              (1)

#if defined(__GNUC__) && !defined(__ARMCC_VERSION)
//
// Use the timeval from the GCC library, not the one
// from LWIP
//
#define LWIP_TIMEVAL_PRIVATE            (0)
#endif

//
// Make sure DHCP is part of the stack
//
#define LWIP_DHCP                       (1)

//
// Enable LwIP send timeout
//
#define LWIP_SO_SNDTIMEO                (1)

//
// Enable LwIP receive timeout
//
#define LWIP_SO_RCVTIMEO                (1)

//
// Enable SO_REUSEADDR option
//
#define SO_REUSE                        (1)

//
// Enable TCP Keep-alive
//
#define LWIP_TCP_KEEPALIVE              (1)

//
// The amount of space to leave before the packet when allocating a pbuf. Needs to
// be enough for the link layer data and the WHD header
//
#define PBUF_LINK_HLEN                  (WHD_PHYSICAL_HEADER)

//
// TCP Maximum segment size
//
#define TCP_MSS                         (WHD_PAYLOAD_MTU)

#define     LWIP_CHECKSUM_CTRL_PER_NETIF   1
#define     CHECKSUM_GEN_IP   1
#define     CHECKSUM_GEN_UDP   1
#define     CHECKSUM_GEN_TCP   1
#define     CHECKSUM_GEN_ICMP   1
#define     CHECKSUM_GEN_ICMP6   1
#define     CHECKSUM_CHECK_IP   1
#define     CHECKSUM_CHECK_UDP   1
#define     CHECKSUM_CHECK_TCP   1
#define     CHECKSUM_CHECK_ICMP   1
#define     CHECKSUM_CHECK_ICMP6   1
#define     LWIP_CHECKSUM_ON_COPY   1

//
// Enable the thread safe NETCONN interface layer
//
#define LWIP_NETCONN                    (1)

/**
 * TCP_WND: The size of a TCP window.  This must be at least
 * (2 * TCP_MSS) for things to work well
 */
#define TCP_WND                         (20 * TCP_MSS)


/**
 * TCP_SND_BUF: TCP sender buffer space (bytes).
 * To achieve good performance, this should be at least 2 * TCP_MSS.
 */
#define TCP_SND_BUF                     (20 * TCP_MSS)

/**
 * TCP_SND_QUEUELEN: TCP sender buffer space (pbufs). This must be at least
 * as much as (2 * TCP_SND_BUF/TCP_MSS) for things to work.
 */
#define TCP_SND_QUEUELEN                ((6 * (TCP_SND_BUF) + (TCP_MSS - 1))/(TCP_MSS))


//
// Taken from WICED to speed things up
//
#define DHCP_DOES_ARP_CHECK             (0)

//
// Light weight protection for things that may be clobbered by interrupts
//
#define SYS_LIGHTWEIGHT_PROT            (1)
#define LWIP_ALLOW_MEM_FREE_FROM_OTHER_CONTEXT      (1)

#define LWIP_SO_RCVBUF                  (128)

#define ARP_MAXAGE                      30000

/* The MEM_SIZE should be multiple of 1024 to avoid SDMA errors */
#define MEM_SIZE                        (400 * 1024)

#define LWIP_SOCKET                     (1)
#define LWIP_NETCONN                    (1)
#define DEFAULT_TCP_RECVMBOX_SIZE       (32)
#define TCPIP_MBOX_SIZE                 (16)
#define TCPIP_THREAD_STACKSIZE          (4*1024)
#define TCPIP_THREAD_PRIO               (CY_RTOS_PRIORITY_HIGH)
#define DEFAULT_RAW_RECVMBOX_SIZE       (16)
#define DEFAULT_UDP_RECVMBOX_SIZE       (64)
#define DEFAULT_ACCEPTMBOX_SIZE         (16)

/**
 * MEMP_NUM_UDP_PCB: the number of UDP protocol control blocks. One
 * per active UDP "connection".
 * (requires the LWIP_UDP option)
 */
#define MEMP_NUM_UDP_PCB                24

/**
 * MEMP_NUM_TCP_PCB: the number of simultaneously active TCP connections.
 * (requires the LWIP_TCP option)
 */
#define MEMP_NUM_TCP_PCB                24

/**
 * MEMP_NUM_TCP_PCB_LISTEN: the number of listening TCP connections.
 * (requires the LWIP_TCP option)
 */
#define MEMP_NUM_TCP_PCB_LISTEN         1

/**
 * MEMP_NUM_TCP_SEG: the number of simultaneously queued TCP segments.
 * (requires the LWIP_TCP option)
 */
#define MEMP_NUM_TCP_SEG                127

/**
 * MEMP_NUM_SYS_TIMEOUT: the number of simultaneously active timeouts.
 */
#define MEMP_NUM_SYS_TIMEOUT            12

/**
 * PBUF_POOL_SIZE: the number of buffers in the pbuf pool.
 */
#define PBUF_POOL_SIZE                  60

/**
 * PBUF_POOL_BUFSIZE: the size of each pbuf in the pbuf pool.
 */
#define PBUF_POOL_BUFSIZE               1800

/**
 * MEMP_NUM_NETBUF: the number of struct netbufs.
 * (only needed if you use the sequential API, like api_lib.c)
 */
#define MEMP_NUM_NETBUF                 64

/**
 * MEMP_NUM_NETCONN: the number of struct netconns.
 * (only needed if you use the sequential API, like api_lib.c)
 */
#define MEMP_NUM_NETCONN                16


/* Turn off LWIP_STATS in Release build */
#ifdef DEBUG
#define LWIP_STATS 1
#else
#define LWIP_STATS 0
#endif

/**
 * LWIP_TCPIP_CORE_LOCKING
 * Creates a global mutex that is held during TCPIP thread operations.
 * Can be locked by client code to perform lwIP operations without changing
 * into TCPIP thread using callbacks. See LOCK_TCPIP_CORE() and
 * UNLOCK_TCPIP_CORE().
 * Your system should provide mutexes supporting priority inversion to use this.
 */
#define LWIP_TCPIP_CORE_LOCKING         1

/**
 * MEMP_NUM_TCPIP_MSG_INPKT: the number of struct tcpip_msg, which are used
 * for incoming packets.
 * (only needed if you use tcpip.c)
 */
#define MEMP_NUM_TCPIP_MSG_INPKT        48

/**
 * LWIP_TCPIP_CORE_LOCKING_INPUT: when LWIP_TCPIP_CORE_LOCKING is enabled,
 * this lets tcpip_input() grab the mutex for input packets as well,
 * instead of allocating a message and passing it to tcpip_thread.
 *
 * ATTENTION: this does not work when tcpip_input() is called from
 * interrupt context!
 */
#define LWIP_TCPIP_CORE_LOCKING_INPUT   1

/**
 * LWIP_NETIF_API==1: Support netif api (in netifapi.c)
 */
#define LWIP_NETIF_API                  1

#define LWIP_DNS                       (1)

#define LWIP_NETIF_TX_SINGLE_PBUF      (1)

#define LWIP_RAND               rand

#define LWIP_FREERTOS_CHECK_CORE_LOCKING             (1)

#define LWIP_ASSERT_CORE_LOCKED()       sys_check_core_locking()

#define LWIP_NETIF_STATUS_CALLBACK    (1)
#define LWIP_NETIF_LINK_CALLBACK      (1)
#define LWIP_NETIF_REMOVE_CALLBACK    (1)

#define LWIP_CHKSUM_ALGORITHM         (3)

extern void sys_check_core_locking();
