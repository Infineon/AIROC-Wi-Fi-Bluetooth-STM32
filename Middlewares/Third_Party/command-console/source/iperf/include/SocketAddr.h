/*
* Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
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

/*---------------------------------------------------------------
 * Copyright (c) 1999,2000,2001,2002,2003
 * The Board of Trustees of the University of Illinois
 * All Rights Reserved.
 *---------------------------------------------------------------
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software (Iperf) and associated
 * documentation files (the "Software"), to deal in the Software
 * without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit
 * persons to whom the Software is furnished to do
 * so, subject to the following conditions:
 *
 *
 * Redistributions of source code must retain the above
 * copyright notice, this list of conditions and
 * the following disclaimers.
 *
 *
 * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimers in the documentation and/or other materials
 * provided with the distribution.
 *
 *
 * Neither the names of the University of Illinois, NCSA,
 * nor the names of its contributors may be used to endorse
 * or promote products derived from this Software without
 * specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE CONTIBUTORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 * ________________________________________________________________
 * National Laboratory for Applied Network Research
 * National Center for Supercomputing Applications
 * University of Illinois at Urbana-Champaign
 * http://www.ncsa.uiuc.edu
 * ________________________________________________________________
 *
 * Socket.cpp
 * by Ajay Tirumala <tirumala@ncsa.uiuc.edu>
 *    and Mark Gates <mgates@nlanr.net>
 * ------------------------------------------------------------------- */


#ifndef SOCKET_ADDR_H
#define SOCKET_ADDR_H

#include "headers.h"
#include "Settings.hpp"

#ifdef __cplusplus
extern "C" {
#endif

/* IPERF_MODFIIED Start */
#define HOST_NOT_FOUND  210
#define NO_DATA         211
#define NO_RECOVERY     212
#define TRY_AGAIN       213
#define NO_ADDRESS      214
/* IPERF_MODIFIED End */

/* ------------------------------------------------------------------- */
    void SockAddr_localAddr( thread_Settings *inSettings );
    void SockAddr_remoteAddr( thread_Settings *inSettings );

    void SockAddr_setHostname( const char* inHostname,
                               iperf_sockaddr *inSockAddr,
                               int isIPv6 );          // DNS lookup
    void SockAddr_getHostname( iperf_sockaddr *inSockAddr,
                               char* outHostname,
                               size_t len );   // reverse DNS lookup
    void SockAddr_getHostAddress( iperf_sockaddr *inSockAddr,
                                  char* outAddress,
                                  size_t len ); // dotted decimal

    void SockAddr_setPort( iperf_sockaddr *inSockAddr, unsigned short inPort );
    void SockAddr_setPortAny( iperf_sockaddr *inSockAddr );
    unsigned short SockAddr_getPort( iperf_sockaddr *inSockAddr );

    void SockAddr_setAddressAny( iperf_sockaddr *inSockAddr );

    // return pointer to the struct in_addr
    struct in_addr* SockAddr_get_in_addr( iperf_sockaddr *inSockAddr );
#ifdef HAVE_IPV6
    // return pointer to the struct in_addr
    struct in6_addr* SockAddr_get_in6_addr( iperf_sockaddr *inSockAddr );
#endif
    // return the sizeof the addess structure (struct sockaddr_in)
    Socklen_t SockAddr_get_sizeof_sockaddr( iperf_sockaddr *inSockAddr );

    int SockAddr_isMulticast( iperf_sockaddr *inSockAddr );

    int SockAddr_isIPv6( iperf_sockaddr *inSockAddr );

    int SockAddr_are_Equal( struct sockaddr *first, struct sockaddr *second );
    int SockAddr_Hostare_Equal( struct sockaddr *first, struct sockaddr *second );

    void SockAddr_zeroAddress( iperf_sockaddr *inSockAddr );
    void SockAddr_incrAddress( iperf_sockaddr *inSockAddr, int value);
    int SockAddr_Ifrname(thread_Settings *inSettings);
#ifdef HAVE_LINUX_FILTER_H
    int SockAddr_Accept_BPF(int socket, uint16_t port);
    int SockAddr_Drop_All_BPF(int socket);
    int SockAddr_v4_Connect_BPF(int socket, uint32_t srcip, uint32_t dstip, uint16_t srcport, uint16_t dstport);
    int SockAddr_v4_Connect_BPF_Drop(int socket, uint32_t srcip, uint32_t dstip, uint16_t srcport, uint16_t dstport);
#  ifdef HAVE_IPV6
    int SockAddr_v6_Connect_BPF (int sock, struct in6_addr *src, struct in6_addr *dst, uint16_t dstport, uint16_t srcport);
#  endif // v6
#endif // linux_filter

#ifdef __cplusplus
} /* end extern "C" */
#endif

#endif /* SOCKET_ADDR_H */
