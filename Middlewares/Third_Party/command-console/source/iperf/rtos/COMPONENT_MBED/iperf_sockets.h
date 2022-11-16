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
#pragma once

#ifndef IPERF_SOCKETS_H
#define IPERF_SOCKETS_H

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/
#if defined (__GNUC__) && (__GNUC__ >= 6)

#undef fd_set
#undef FD_ZERO
#undef FD_ISSET
#undef FD_CLR
#undef FD_SET
#undef FD_SETSIZE

#endif

#define FD_SET(socket, fds)    (*fds = socket)
#define FD_ZERO(fds)           (*fds = -1)

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/* These types has been defined in "tx_port.h" in Threadx
 * Avoid double definition using VOID as a checker which is
 * defined as a macro in tx_port.h
 */

//#define in_addr_t   unsigned long

#define SOCK_STREAM                         1                       /* TCP Socket */
#define SOCK_DGRAM                          2                       /* UDP Socket */
#define IPPROTO_TCP                         6                       /* TCP. */
#define IPPROTO_UDP                         17                      /* UDP. */
#define IPPROTO_IP                          0

#define     AF_INET                         2                       /* IPv4 socket (UDP, TCP, etc) */
#define     AF_INET6                        3                       /* IPv6 socket (UDP, TCP, etc) */
/* Protocol families, same as address families.  */
#define     PF_INET                         AF_INET
#define     PF_INET6                        AF_INET6

#define SOL_SOCKET      1   /* Define the option category (the only one). */

#define SO_MIN          1   /* Minimum Socket option ID */
#define SO_DEBUG        1   /* Debugging information is being recorded.*/
#define SO_REUSEADDR    2   /* Enable reuse of local addresses in the time wait state */
#define SO_TYPE         3   /* Socket type */
#define SO_ERROR        4   /* Socket error status */
#define SO_DONTROUTE    5   /* Bypass normal routing */
#define SO_BROADCAST    6   /* Transmission of broadcast messages is supported.*/
#define SO_SNDBUF       7   /* Enable setting trasnmit buffer size */
#define SO_RCVBUF       8   /* Enable setting receive buffer size */
#define SO_KEEPALIVE    9   /* Connections are kept alive with periodic messages */
#define SO_OOBINLINE    10  /* Out-of-band data is transmitted in line */
#define SO_NO_CHECK     11  /* Disable UDP checksum */
#define SO_PRIORITY     12  /* Set the protocol-defined priority for all packets to be sent on this socket */
#define SO_LINGER       13  /* Socket lingers on close pending remaining send/receive packets. */
#define SO_BSDCOMPAT    14  /* Enable BSD bug-to-bug compatibility */
#define SO_REUSEPORT    15  /* Rebind a port already in use */
#define SO_SNDTIMEO     0x1005 /* send timeout */
#define SO_RCVTIMEO     0x1006 /* receive timeout */

/* Flags we can use with send and recv. */
#define MSG_PEEK       0x01    /* Peeks at an incoming message */
#define MSG_WAITALL    0x02    /* Unimplemented: Requests that the function block until the full amount of data requested can be returned */
#define MSG_OOB        0x04    /* Unimplemented: Requests out-of-band data. The significance and semantics of out-of-band data are protocol-specific */
#define MSG_DONTWAIT   0x08    /* Nonblocking i/o for this operation only */
#define MSG_MORE       0x10    /* Sender will send more */
#define MSG_NOSIGNAL   0x20    /* Uninmplemented: Requests not to send the SIGPIPE signal if an attempt to send is made on a stream-oriented socket that is no longer connected. */

/*
 * Options for level IPPROTO_IP
 */
#define IP_TOS             1
#define IP_TTL             2
#define IP_PKTINFO         8


#define ntohs(x) ( (((x)&0xff00)>>8) | \
                   (((x)&0x00ff)<<8) )
#define ntohl(x) ( (((x)&0xff000000)>>24) | \
               (((x)&0x00ff0000)>> 8) | \
               (((x)&0x0000ff00)<< 8) | \
               (((x)&0x000000ff)<<24) )

#define htons(x)  ntohs(x)
#define htonl(x)  ntohl(x)

typedef  int        fd_set;

#define IP_ADD_MEMBERSHIP  3
#define IP_DROP_MEMBERSHIP 4

/*
 * Options for level IPPROTO_TCP
 */
#define TCP_NODELAY    0x01    /* don't delay send to coalesce packets */
#define TCP_KEEPALIVE  0x02    /* send KEEPALIVE probes when idle for pcb->keep_idle milliseconds */
#define TCP_KEEPIDLE   0x03    /* set pcb->keep_idle  - Same as TCP_KEEPALIVE, but use seconds for get/setsockopt */
#define TCP_KEEPINTVL  0x04    /* set pcb->keep_intvl - Use seconds for get/setsockopt */
#define TCP_KEEPCNT    0x05    /* set pcb->keep_cnt   - Use number of probes sent for get/setsockopt */

#define IPADDR_ANY          ((uint32_t)0x00000000UL)
#define INADDR_ANY          IPADDR_ANY

/******************************************************
 *                    Structures
 ******************************************************/

struct sockaddr
{
    unsigned short          sa_family;              /* Address family (e.g. , AF_INET).                                                             */
    unsigned char           sa_data[14];            /* Protocol- specific address information.                                                      */
};

struct in6_addr
{
    union
    {
        unsigned char _S6_u8[16];
        unsigned long _S6_u32[4];
    } _S6_un;
};

struct sockaddr_in6
{
    unsigned short          sin6_family;                 /* AF_INET6 */
    unsigned short          sin6_port;                   /* Transport layer port. # */
    unsigned long           sin6_flowinfo;               /* IPv6 flow information. */
    struct in6_addr sin6_addr;                   /* IPv6 address. */
    unsigned long           sin6_scope_id;               /* set of interafces for a scope. */

};

/* internet address (a structure for historical reasons).  */

struct in_addr
{
    unsigned long           s_addr;             /* internet address (32 bits).                                                                  */
};

/* Socket address, internet style. */

struct sockaddr_in
{
    unsigned short              sin_family;         /* internet Protocol (AF_INET).                                                                 */
    unsigned short              sin_port;           /* Address port (16 bits).                                                                      */
    struct in_addr      sin_addr;           /* internet address (32 bits).                                                                  */
    char                sin_zero[8];        /* Not used.                                                                                    */
};


typedef struct ip_mreq {
    struct in_addr imr_multiaddr; /* IP multicast address of group */
    struct in_addr imr_interface; /* local IP address of interface */
} ip_mreq;

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/
void iperf_network_init( void *networkInterface );

void sockets_layer_init(void);
int iperf_getpeername(int sockID, struct sockaddr *remoteAddress, uint32_t *addressLength);
int iperf_getsockname(int sockID, struct sockaddr *localAddress, uint32_t *addressLength);
int iperf_recvfrom(int sockID, char *buffer, size_t buffersize, int flags,struct sockaddr *fromAddr, uint32_t *fromAddrLen);
int iperf_recv(int sockID, void *rcvBuffer, size_t bufferLength, int flags);
int iperf_sendto(int sockID, char *msg, size_t msgLength, int flags, struct sockaddr *destAddr, uint32_t destAddrLen);
int iperf_send(int sockID, const char *msg, size_t msgLength, int flags);
int iperf_accept(int sockID, struct sockaddr *ClientAddress, uint32_t *addressLength);
int iperf_listen(int sockID, int backlog);
int iperf_bind(int sockID, struct sockaddr *localAddress, uint32_t addressLength);
int iperf_connect(int sockID, struct sockaddr *remoteAddress, uint32_t addressLength);
int iperf_select( int nfds, fd_set *readfds, fd_set *writefds, fd_set *exceptfds, struct timeval *timeout );
int iperf_close( int sockID);
int iperf_socket(int protocolFamily, int type, int protocol);
int iperf_setsockopt(int sockID, int option_level, int option_name, const void *option_value, uint32_t option_length);
int iperf_getsockopt(int sockID, int option_level, int option_name, void *option_value,uint32_t *option_length);
int iperf_write( int sockID, const char *msg, size_t msgLength );
int iperf_read(int s, void *mem, size_t len);
bool iperf_setsock_blocking (int fd, bool blocking);
#ifdef __cplusplus
} /*extern "C" */
#endif

#endif //IPERF_SOCKETS_H
