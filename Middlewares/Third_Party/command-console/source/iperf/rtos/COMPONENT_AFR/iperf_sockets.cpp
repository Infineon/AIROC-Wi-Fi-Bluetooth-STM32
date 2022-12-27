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

/** @file
 *
 */
#include "iperf_sockets.h"
#include "iperf_utility.h"

/******************************************************
 *                 Type Definitions
 ******************************************************/
#ifdef __cplusplus
extern "C" {
#endif

#define IPERF_SOCKET_DEBUG(x) //printf x
#define IPERF_SOCKET_ERROR(x) //printf x

/******************************************************
 *                   Enumerations
 ******************************************************/
/******************************************************
 *                    Structures
 ******************************************************/

static int sock_proto[MEMP_NUM_NETCONN];

void iperf_network_init(void * networkInterface)
{
    // DUMMY
}

void sockets_layer_init(void)
{
    // DUMMY
}

int iperf_socket(int protocolFamily, int type, int protocol)
{
    int sockID;

    IPERF_SOCKET_DEBUG(("iperf_socket : Creating sockets \n"));
    sockID = socket(protocolFamily, type, protocol);
    if(sockID == -1)
    {
        IPERF_SOCKET_ERROR(("Error in creating socket \n"));
        return -1;
    }
    sock_proto[sockID] = type;
    return sockID;
}

int iperf_write(int sockID, const char *msg, size_t msgLength)
{
    IPERF_SOCKET_DEBUG(("iperf_write : Write on socket with length : %d \r\n", msgLength));
    return write(sockID, msg, msgLength);
}

int iperf_read(int s, void *mem, size_t len)
{
    IPERF_SOCKET_DEBUG(("IPERF_read with length : %d\n", len));
    return read(s, mem, len);
}

int iperf_getpeername(int sockID, struct sockaddr *remoteAddress, uint32_t *addressLength)
{
    /* TODO: Implement code for getpeername */
    IPERF_SOCKET_DEBUG(("getpeername\n"));
    return 0;
}

int iperf_getsockname(int sockID, struct sockaddr *localAddress, uint32_t *addressLength)
{
    /* TODO: Implement code for getsockname */
    IPERF_SOCKET_DEBUG(("getsockname\n"));
    return 0;
}

int iperf_recvfrom(int sockID, char *buffer, size_t buffersize, int flags, struct sockaddr *fromAddr, uint32_t *fromAddrLen)
{
    IPERF_SOCKET_DEBUG(("iperf_recvfrom : Read data with buffer length : %d \r\n", buffersize));
    return recvfrom(sockID, buffer, buffersize, flags, fromAddr, fromAddrLen);
}

int iperf_select(int nfds, fd_set *readfds, fd_set *writefds, fd_set *exceptfds, struct timeval *timeout)
{
    /*
     * iperf only uses select() in two places. Both are used to check if the socket is readable.
     * So instead of waiting here, we'll record the wait timeout and use it in the recv() call that follows
     */
    IPERF_SOCKET_DEBUG(("iperf_select \n"));
    return select(nfds, readfds, writefds, exceptfds, timeout);
}

int iperf_setsockopt(int sockID, int option_level, int option_name, const void *option_value, uint32_t option_length)
{
    IPERF_SOCKET_DEBUG(("iperf_setsockopt : Set socket option : %d\n", option_name));
    return setsockopt(sockID, option_level, option_name, option_value, option_length);
}

int iperf_getsockopt(int sockID, int option_level, int option_name, void *option_value, uint32_t *option_length)
{
    /* Socket options requested by iperf:
     * sockets.c:
     * TCP_MAXSEG
     *
     * tcp_window_size.c:
     * SO_SNDBUF
     * SO_RCVBUF
     */
    IPERF_SOCKET_DEBUG(("iperf_getsockopt %d\n", option_name));

    switch (sock_proto[sockID]) {
    case SOCK_STREAM:
        switch (option_name) {
        case SO_SNDBUF:
            *(uint32_t*) option_value = TCP_SND_BUF;
            *option_length = sizeof(uint32_t);
            break;
        case SO_RCVBUF:
            *(uint32_t*) option_value = TCP_WND;
            *option_length = sizeof(uint32_t);
            break;
        default:
            IPERF_SOCKET_DEBUG(("Getsockopt %d not supported\n", option_name));
            return -1;
        }
        break;
    case SOCK_DGRAM:
        switch (option_name) {
        case SO_SNDBUF:
        case SO_RCVBUF:
            *(uint32_t*) option_value = TCP_WND;
            *option_length = sizeof(uint32_t);
            break;
        default:
            IPERF_SOCKET_DEBUG(("Getsockopt %d not supported\n", option_name));
            return -1;
        }
        break;
    default:
        IPERF_SOCKET_DEBUG(("Invalid protocol type \n"));
        return -1;
    }

    return 0;
}

int iperf_close(int sockID)
{
    IPERF_SOCKET_DEBUG(("iperf_close : Close connection, socket id : %d \n", sockID));
    sock_proto[sockID] = 0;
    return close(sockID);
}

int iperf_bind(int sockID, struct sockaddr *localAddress, uint32_t addressLength)
{
    IPERF_SOCKET_DEBUG(("iperf_bind : Bind to a socket \r\n"));
    return bind(sockID, localAddress, addressLength);
}

int iperf_listen(int sockID, int backlog)
{
    IPERF_SOCKET_DEBUG(("iperf_listen : Listen on socket \r\n"));
    return listen(sockID, backlog);
}

bool iperf_setsock_blocking(int fd, bool blocking)
{
    IPERF_SOCKET_DEBUG(("iperf_setsock_blocking : Set the mode to blocking/non-blocking - %d \n", blocking));
    return true;
}

int iperf_connect(int sockID, struct sockaddr *remoteAddress, uint32_t addressLength)
{
    IPERF_SOCKET_DEBUG(("iperf_connect : Connect to a server \r\n"));
    return connect(sockID, remoteAddress, addressLength);
}

int iperf_accept(int sockID, struct sockaddr *ClientAddress, uint32_t *addressLength)
{
    IPERF_SOCKET_DEBUG(("iperf_accept : Accept new client connection \n"));
    return accept(sockID, ClientAddress, addressLength);
}

int iperf_recv(int sockID, void *rcvBuffer, size_t bufferLength, int flags)
{
    IPERF_SOCKET_DEBUG(("iperf_recv : Read data with buffer length : %d\n", bufferLength));
    return recv(sockID, rcvBuffer, bufferLength, flags);
}

int iperf_sendto(int sockID, char *msg, size_t msgLength, int flags, struct sockaddr *destAddr, uint32_t destAddrLen)
{
    /* TODO: Implement code for sendto */
    IPERF_SOCKET_DEBUG(("iperf_sendto : Write data to socket with length : %d \n", msgLength));
    return 1;
}

int iperf_send(int sockID, const char *msg, size_t msgLength, int flags)
{
    IPERF_SOCKET_DEBUG(("iperf_send : Write data to socket with length : %d \n", msgLength));
    return send(sockID, msg, msgLength, flags);
}

struct hostent* iperf_gethostbyname(const char *name)
{
    /* TODO: Implement code for resolving the hostname */
    return 0;
}

#ifdef __cplusplus
}
#endif
