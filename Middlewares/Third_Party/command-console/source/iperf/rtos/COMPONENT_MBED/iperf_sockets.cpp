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
#include "mbed.h"
#include "iperf_sockets.h"
#include "lwipopts.h"

#define IPERF_SOCKET_DEBUG(x) //printf x

#ifndef MAX_BSD_SOCKETS
#define MAX_BSD_SOCKETS                  (5)
#endif

#define BUFFER_LENGTH                    (100)
#define MBED_RECV_SOCKET_TIMEOUT_ERROR   (-3)

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    BSD_SOCKET_UDP,
    BSD_SOCKET_TCP,
} bsd_socket_type_t;
/******************************************************
 *                    Structures
 ******************************************************/

typedef struct
{
    bsd_socket_type_t type;
    bool              available;
    uint32_t          select_timeout;
    struct sockaddr   remoteAddress;
    TCPSocket         *client_socket;
    union
    {
        UDPSocket *udp;
        TCPSocket *tcp;
    } socket;
    char              rcv_buffer[BUFFER_LENGTH];
    int               rcv_buffer_len;
} bsd_socket_t;

static bsd_socket_t   sockets[MAX_BSD_SOCKETS];
static bool   socket_layer_inited = false;

static NetworkInterface *network;

void iperf_network_init( void * networkInterface )
{
    network = (NetworkInterface *)networkInterface;
}

void sockets_layer_init( void )
{
    int a;
    for ( a = 0; a < MAX_BSD_SOCKETS; ++a )
    {
        sockets[a].available = true;
        sockets[a].select_timeout = 0;
    }

    socket_layer_inited = true;
}

int iperf_socket( int protocolFamily, int type, int protocol )
{
    int a=0;
    bsd_socket_t* free_socket;
    int err;
    IPERF_SOCKET_DEBUG(("socket \n"));

    if( network == NULL )
    {
        IPERF_SOCKET_DEBUG(("[ERROR] :  APP has not initialized the network \r\n"));
        return -1;
    }
    if ( socket_layer_inited == false )
    {
        sockets_layer_init( );
    }

    // Find a free socket
    for ( ; a < MAX_BSD_SOCKETS; ++a )
    {
        if ( sockets[a].available == true )
        {
            free_socket = &sockets[a];
            memset(free_socket, 0, sizeof(bsd_socket_t));
            free_socket->available        = false;
            free_socket->select_timeout   = 0;

            switch ( type )
            {
                case SOCK_DGRAM:
                    free_socket->type = BSD_SOCKET_UDP;
                    sockets[a].socket.udp = new UDPSocket;
                    err = (sockets[a].socket.udp)->open(network);
                    if(err != 0)
                    {
                        IPERF_SOCKET_DEBUG(("[ERROR] :  UDP SOCKET OPEN FAILED %d\r\n", err));
                        a = -1;
                    }
                    break;

                case SOCK_STREAM:
                    free_socket->type = BSD_SOCKET_TCP;

                    sockets[a].socket.tcp = new TCPSocket;
                    err = (sockets[a].socket.tcp)->open(network);
                    if(err != 0)
                    {
                        IPERF_SOCKET_DEBUG(("[ERROR] :  TCP SOCKET OPEN FAILED\r\n"));
                        a = -1;
                    }
                    break;
            }
            IPERF_SOCKET_DEBUG(("socket %d\n", a));
            return a;
        }
    }

    IPERF_SOCKET_DEBUG(("socket %d\n", a));

    return a;
}

int iperf_write( int sockID, const char *msg, size_t msgLength )
{
    int ret=0;
    IPERF_SOCKET_DEBUG(("IPERF_write with length : %d\n", msgLength));

    switch ( sockets[sockID].type )
    {
        case BSD_SOCKET_TCP:
        {
            ret = (sockets[sockID].socket.tcp)->send(msg, msgLength );
            break;
        }
        case BSD_SOCKET_UDP:
        {
            struct sockaddr_in *sock_addr = (struct sockaddr_in *)&sockets[sockID].remoteAddress;
            unsigned long ip_addr;
            uint16_t port_no = 0;

            memcpy(&ip_addr, sockets[sockID].remoteAddress.sa_data + 2, 4);
            port_no = htons(sock_addr->sin_port);

            SocketAddress socket_address( &ip_addr, NSAPI_IPv4, port_no);
            ret = (sockets[sockID].socket.udp)->sendto(socket_address, msg, msgLength);

            break;
        }
    }

    if(ret < 0)
    {
        ret = 0;
    }

    IPERF_SOCKET_DEBUG(("IPERF_write status %d\n", ret));

    return ret;
}

int iperf_read(int sockID, void *mem, size_t len)
{
    int ret=0;
    IPERF_SOCKET_DEBUG(("IPERF_read with length : %d\n", len));

    switch ( sockets[sockID].type )
    {
        case BSD_SOCKET_TCP:
        {
            if(sockets[sockID].client_socket)
            {
                ret = (sockets[sockID].client_socket)->recv(mem, len);
            }
            break;
        }
        case BSD_SOCKET_UDP:
        {
            if(sockets[sockID].socket.udp)
            {
                ret = (sockets[sockID].socket.udp)->recv(mem, len);
            }
            break;
        }
    }

    IPERF_SOCKET_DEBUG(("iperf_read status %d \n", ret));

    return ret;
}

int iperf_getpeername(int sockID, struct sockaddr *remoteAddress, uint32_t *addressLength)
{
    IPERF_SOCKET_DEBUG(("getpeername\n"));
    return 0;
}

int iperf_getsockname(int sockID, struct sockaddr *localAddress, uint32_t *addressLength)
{
    IPERF_SOCKET_DEBUG(("getsockname\n"));
    return 0;
}

int iperf_recvfrom(int sockID, char *buffer, size_t buffersize, int flags,struct sockaddr *fromAddr, uint32_t *fromAddrLen)
{
    int ret=0;
    IPERF_SOCKET_DEBUG(("recvfrom\n"));

    SocketAddress sockAddr;
    struct sockaddr_in *sock_addr = (struct sockaddr_in *)fromAddr;
    unsigned long ip_addr = sock_addr->sin_addr.s_addr;
    uint16_t port = ntohs(sock_addr->sin_port);

    SocketAddress socket_address( &ip_addr, NSAPI_IPv4, port);

    IPERF_SOCKET_DEBUG(("recvfrom ip ip_addr %ld port %d\n", ip_addr, port));

    ret = (sockets[sockID].socket.udp)->recvfrom(&sockAddr, buffer, buffersize);
    IPERF_SOCKET_DEBUG(("recvfrom returned %d %s\n", ret, sockAddr.get_ip_address()));

    char* bytes = (char*) sockAddr.get_ip_bytes();
    port = sockAddr.get_port();

   fromAddr->sa_data[1] = (port & 0xFF);
   fromAddr->sa_data[0] = (port >> 8);

   memcpy((fromAddr->sa_data + 2), bytes, 4);

   return ret;
}

int iperf_recv(int sockID, void *rcvBuffer, size_t bufferLength, int flags)
{
    int ret=0;
    size_t received_data_length = 0;
    size_t remaining_data_length  = bufferLength;
    IPERF_SOCKET_DEBUG(("recv %d, flag : %d \n", bufferLength, flags));

    /* There is no support for MSG_PEEK flag in MBEDOS receive. IPERF calls iperf_recv with MSG_PEEK flag and bufferLength as 4 bytes
     * Hence buffer the read data to socket buffer and copy it in next recv call.
     */
    if((flags & MSG_PEEK) != MSG_PEEK)
    {
        if(sockets[sockID].rcv_buffer_len != 0)
        {
            memcpy(rcvBuffer, sockets[sockID].rcv_buffer, sockets[sockID].rcv_buffer_len);
            memset(sockets[sockID].rcv_buffer, 0, sizeof(sockets[sockID].rcv_buffer));
            received_data_length += sockets[sockID].rcv_buffer_len;
            remaining_data_length -= sockets[sockID].rcv_buffer_len;
            sockets[sockID].rcv_buffer_len = 0;
        }
    }

    switch ( sockets[sockID].type )
    {
        case BSD_SOCKET_TCP:
        {
            if(sockets[sockID].client_socket)
            {
                while(received_data_length < bufferLength)
                {
                    ret = (sockets[sockID].client_socket)->recv((((char*)rcvBuffer) + received_data_length) , remaining_data_length);
                    if(ret > 0)
                    {
                        remaining_data_length -= ret;
                        received_data_length += ret;
                    }
                    else
                    {
                        return ret;
                    }
                }
            }
            break;
        }
        case BSD_SOCKET_UDP:
        {
            if(sockets[sockID].socket.udp)
            {
                while(received_data_length < bufferLength)
                {
                    ret = (sockets[sockID].socket.udp)->recv((((char*)rcvBuffer) + received_data_length) , remaining_data_length);
                    if(ret > 0)
                    {
                        remaining_data_length -= ret;
                        received_data_length += ret;
                    }
                    else
                    {
                        /* return specific error code on socket receive timeout */
                        if(ret == NSAPI_ERROR_TIMEOUT)
                        {
                            return MBED_RECV_SOCKET_TIMEOUT_ERROR;
                        }
                        return ret;
                    }
                }
            }
            break;
        }
    }

    /* Copy read data to buffer if MSG_PEEK flag is set */
    if((flags & MSG_PEEK) == MSG_PEEK)
    {
        memcpy(sockets[sockID].rcv_buffer + sockets[sockID].rcv_buffer_len, rcvBuffer, ret);
        sockets[sockID].rcv_buffer_len += ret;
    }

    IPERF_SOCKET_DEBUG(("recv status %d \n", received_data_length));

    ret = received_data_length;
    return ret;
}

int iperf_accept(int sockID, struct sockaddr *ClientAddress, uint32_t *addressLength)
{
    int ret = 0;
    SocketAddress client_address;

    IPERF_SOCKET_DEBUG(("Accept \n"));

    switch ( sockets[sockID].type )
    {
        case BSD_SOCKET_TCP:
        {
            sockets[sockID].client_socket = (sockets[sockID].socket.tcp)->accept(&ret);
            if( ret !=0 )
            {
                ret = -1;
            }
            break;
        }
        case BSD_SOCKET_UDP:
        {
            IPERF_SOCKET_DEBUG(("Error: accept called on udp socket \n"));
            break;
        }
    }

    IPERF_SOCKET_DEBUG(("accept status %d\n", ret));
    return ret;
}

int iperf_listen(int sockID, int backlog)
{
    int ret=0;

    switch ( sockets[sockID].type )
    {
        case BSD_SOCKET_TCP:
        {
            ret = (sockets[sockID].socket.tcp)->listen(backlog);
            if( ret !=0 )
            {
                ret = -1;
            }
            break;
        }
        case BSD_SOCKET_UDP:
        {
            IPERF_SOCKET_DEBUG(("Error: listen called on udp socket \n"));
            break;
        }
    }

    IPERF_SOCKET_DEBUG(("listen status %d\n", ret));
    return ret;
}

int iperf_bind(int sockID, struct sockaddr *localAddress, uint32_t addressLength)
{
    int ret=0;
    struct sockaddr_in *sock_addr = (struct sockaddr_in *)localAddress;
    unsigned long ip_addr = sock_addr->sin_addr.s_addr;
    uint16_t port = ntohs(sock_addr->sin_port);
    SocketAddress socket_address( &ip_addr, NSAPI_IPv4, port);

    IPERF_SOCKET_DEBUG(("bind ip address %ld port %d \n", ip_addr, port));

    switch ( sockets[sockID].type )
    {
        case BSD_SOCKET_TCP:
        {
            int optval = 1;
            unsigned int optlen = sizeof(optval);
            ret = (sockets[sockID].socket.tcp)->setsockopt(NSAPI_SOCKET, NSAPI_REUSEADDR, &optval, optlen);
            if( ret !=0 )
            {
                ret = -1;
                break;
            }
            ret = (sockets[sockID].socket.tcp)->bind(socket_address);
            if( ret !=0 )
            {
                ret = -1;
            }
            break;
        }
        case BSD_SOCKET_UDP:
        {
            int optval = 1;
            unsigned int optlen = sizeof(optval);
            ret = (sockets[sockID].socket.tcp)->setsockopt(NSAPI_SOCKET, NSAPI_REUSEADDR, &optval, optlen);
            if( ret !=0 )
            {
                ret = -1;
                break;
            }

            ret = (sockets[sockID].socket.udp)->bind(socket_address);
            break;
        }
    }

    IPERF_SOCKET_DEBUG(("bind status %d\n", ret));
    return ret;
}

int iperf_connect(int sockID, struct sockaddr *remoteAddress, uint32_t addressLength)
{
    struct sockaddr_in *sock_addr = (struct sockaddr_in *)remoteAddress;
    int ret=0;
    unsigned long ip_addr = sock_addr->sin_addr.s_addr;
    uint16_t port = htons(sock_addr->sin_port);
    IPERF_SOCKET_DEBUG(("connecting to server on port : %d \n", port));

    SocketAddress socket_address( &ip_addr, NSAPI_IPv4, port);

    switch ( sockets[sockID].type )
    {
        case BSD_SOCKET_TCP:
        {
            ret = (sockets[sockID].socket.tcp)->connect( socket_address );
            break;
        }
       case BSD_SOCKET_UDP:
           memcpy(&sockets[sockID].remoteAddress, remoteAddress, sizeof(struct sockaddr) );
           ret = 0;
           break;
   }

    IPERF_SOCKET_DEBUG(("connect status %d\n", ret));
    return ret;
}

int  iperf_close( int sockID)
{
    int ret = 0;
    IPERF_SOCKET_DEBUG(("iperf_close\n"));

    if((sockets[sockID].socket.tcp) != NULL)
    {
        ret = (sockets[sockID].socket.tcp)->close();
        delete (sockets[sockID].socket.tcp);
        (sockets[sockID].socket.tcp) = NULL;
    }

    if((sockets[sockID].socket.udp) != NULL)
    {
        ret = (sockets[sockID].socket.udp)->close();
        delete (sockets[sockID].socket.udp);
        (sockets[sockID].socket.udp) = NULL;
    }

    sockets[sockID].available = true;

    IPERF_SOCKET_DEBUG(("iperf_close %d\n", ret));
    return ret;
}

int iperf_setsockopt(int sockID, int option_level, int option_name, const void *option_value, uint32_t option_length)
{

    IPERF_SOCKET_DEBUG(("setsockopt %d\n", option_name));
    /*
     * Socket options used by iperf:
     * Listener.cpp:
     * SO_REUSEADDR
     * IP_ADD_MEMBERSHIP
     * IPV6_ADD_MEMBERSHIP
     * IP_MULTICAST_TTL
     * IPV6_MULTICAST_HOPS
     *
     * PerfSocket.cpp:
     * TCP_CONGESTION
     * IP_MULTICAST_TTL
     * IPV6_MULTICAST_HOPS
     * IP_TOS
     * TCP_NODELAY
     *
     * tcp_window_size:
     * TCP_WINSHIFT
     * TCP_RFC1323
     * SO_RCVBUF
     * SO_SNDBUF
     */

    switch ( option_name )
    {
        case SO_REUSEADDR:
            IPERF_SOCKET_DEBUG(("SO_REUSEADDR\n"));
            break;
        case SO_SNDBUF:
            IPERF_SOCKET_DEBUG(("SO_SNDBUF %ld\n", (*(uint32_t*) option_value)));

            break;
        case SO_RCVBUF:
            IPERF_SOCKET_DEBUG(("SO_RCVBUF %ld\n", (*(uint32_t*) option_value)));

            break;
        case SO_RCVTIMEO:
            IPERF_SOCKET_DEBUG(("SO_RCVTIMEO %ld\n", (*(uint32_t*) option_value)));
            (sockets[sockID].socket.tcp)->set_timeout(*(uint32_t*) option_value);
            break;

        case SO_SNDTIMEO:
            IPERF_SOCKET_DEBUG(("SO_SNDTIMEO %ld\n", (*(uint32_t*) option_value)));
            (sockets[sockID].socket.tcp)->set_timeout(*(uint32_t*) option_value);
            break;


        default:
            printf("Setsockopt %d\n", option_name);
            break;
    }
    return 0;
}

int iperf_getsockopt(int sockID, int option_level, int option_name, void *option_value,uint32_t *option_length)
{
    /* Socket options requested by iperf:
     * sockets.c:
     * TCP_MAXSEG
     *
     * tcp_window_size.c:
     * SO_SNDBUF
     * SO_RCVBUF
     */
    IPERF_SOCKET_DEBUG(("getsockopt %d\n", option_name));

    switch ( sockets[sockID].type )
    {
        case BSD_SOCKET_TCP:
            switch ( option_name )
            {
                case SO_SNDBUF:
                    *(uint32_t*) option_value = MBED_CONF_LWIP_TCP_SND_BUF;
                    break;
                case SO_RCVBUF:
                    *(uint32_t*) option_value = MBED_CONF_LWIP_TCP_WND;
                    break;
                default:
                    return -1;
            }
         break;
         case BSD_SOCKET_UDP:
            switch ( option_name )
            {
                case SO_SNDBUF:
                case SO_RCVBUF:
                    *(uint32_t*) option_value = MBED_CONF_LWIP_TCP_WND;
                    break;
                default:
                    return -1;
            }
         break;
         default:
             return -1;
    }

    IPERF_SOCKET_DEBUG(("getsockopt status Done \n"));

    return 0;
}

int iperf_sendto(int sockID, char *msg, size_t msgLength, int flags, struct sockaddr *destAddr, uint32_t destAddrLen)
{
    IPERF_SOCKET_DEBUG(("sendto \n"));
    return 1;
}

int iperf_send(int sockID, const char *msg, size_t msgLength, int flags)
{
    IPERF_SOCKET_DEBUG(("send \n"));

    int ret=0;
    IPERF_SOCKET_DEBUG(("iperf_write on socket Id : %d with length : %d, sockets[sockID].remoteAddress : %0x \n", sockID, msgLength, sockets[sockID].remoteAddress));

    switch ( sockets[sockID].type )
    {
        case BSD_SOCKET_TCP:
        {
            ret = (sockets[sockID].socket.tcp)->send(msg, msgLength );
            break;
        }
        case BSD_SOCKET_UDP:
        {
            struct sockaddr_in *sock_addr = (struct sockaddr_in *)&sockets[sockID].remoteAddress;
            unsigned long ip_addr;
            uint16_t port_no = htons(sock_addr->sin_port);;

            memcpy(&ip_addr, sockets[sockID].remoteAddress.sa_data, 4);

            SocketAddress socket_address( &ip_addr, NSAPI_IPv4, port_no);
            ret = (sockets[sockID].socket.udp)->sendto(socket_address, msg, msgLength);

            if(ret < 0)
                ret = 0;

            break;
        }
    }

    IPERF_SOCKET_DEBUG(("send status %d\n", ret));

    return ret;

}

#ifdef __cplusplus
extern "C" {
#endif

struct hostent* gethostbyname(const char *name)
{
    IPERF_SOCKET_DEBUG(("gethostbyname \n"));
    return 0;
}

int  iperf_select( int nfds, fd_set *readfds, fd_set *writefds, fd_set *exceptfds, struct timeval *timeout )
{
    /*
     * iperf only uses select() in two places. Both are used to check if the socket is readable.
     * So instead of waiting here, we'll record the wait timeout and use it in the recv() call that follows
     */
    IPERF_SOCKET_DEBUG(("iperf_select \n"));
    return 1;
}

#define FALSE 0
#define TRUE 1
bool iperf_setsock_blocking (int fd, bool blocking) {

    IPERF_SOCKET_DEBUG(("iperf_setsock_blocking : blocking mode - %d \n", blocking));

    switch ( sockets[fd].type )
    {
        case BSD_SOCKET_TCP:
          (sockets[fd].client_socket)->set_blocking(blocking);
          break;

        case BSD_SOCKET_UDP:
          (sockets[fd].socket.udp)->set_blocking(blocking);
          break;
    }

    return true;

}


#ifdef __cplusplus
}
#endif
