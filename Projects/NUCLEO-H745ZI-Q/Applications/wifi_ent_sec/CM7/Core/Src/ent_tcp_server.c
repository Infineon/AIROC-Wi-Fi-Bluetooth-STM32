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
/** @file ent_tcp_server.c
 *  enterprise tcp echo server implementation
 */
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "cy_secure_sockets.h"
#include "cy_nw_helper.h"
#include "ent_sec_utility.h"

/********************************************************************/
/* Macros                                                           */
/********************************************************************/
#define IP_ADDR_BUFFER_SIZE                     (20u)
#define TCP_SERVER_RECV_TIMEOUT_MS              (500u)
#define MAX_TCP_RECV_BUFFER_SIZE                (1500u)
#define CY_RSLT_SERVER_ERR                      (1)
#define TCP_SERVER_MAX_PENDING_CONNECTIONS      (1)

/********************************************************************/
/* Static Variables                                                 */
/********************************************************************/
static cy_socket_t server_handle;
static cy_socket_t client_handle;
static bool client_connected;
static bool server_started;
static cy_socket_sockaddr_t tcp_server_addr;

/*******************************************************************************
* Function Name: tcp_receive_msg_handler
*******************************************************************************
* Summary:
*  Callback function to handle incoming TCP client messages and echo it back
*
* Parameters:
* cy_socket_t socket_handle: Connection handle for the TCP client socket
* void *args               : Parameter passed on to the function (unused)
*
* Return:
*  cy_result result: CY_RSLT_SUCCESS for success, error otherwise
*
*******************************************************************************/
static cy_rslt_t tcp_receive_msg_handler(cy_socket_t socket_handle, void* arg)
{
    char message_buffer[MAX_TCP_RECV_BUFFER_SIZE+1];
    cy_rslt_t result;
    uint32_t bytes_received = 0;
    uint32_t bytes_sent = 0;

    memset(message_buffer, 0, sizeof(message_buffer));

    result = cy_socket_recv(socket_handle, message_buffer, MAX_TCP_RECV_BUFFER_SIZE,
                            CY_SOCKET_FLAGS_NONE, &bytes_received);

    if (result == CY_RSLT_SUCCESS)
    {
        /* print received data */
        ENT_SEC_INFO(("Received Data: %s\n ", message_buffer));

        /* echo the data to client. */
        result = cy_socket_send(socket_handle, message_buffer, bytes_received,
                                CY_SOCKET_FLAGS_NONE, &bytes_sent);
        if (result == CY_RSLT_SUCCESS)
        {
            ENT_SEC_INFO(("Data echoed successfully\n"));
        }
        else
        {
            ENT_SEC_INFO(("Data echo failed! Error code: 0x%08" PRIx32 "\n", (uint32_t)result));
        }
    }
    else
    {
        if (result == CY_RSLT_MODULE_SECURE_SOCKETS_CLOSED)
        {
            /* Disconnect the socket. */
            cy_socket_disconnect(socket_handle, 0);
            /* Delete the socket. */
            cy_socket_delete(socket_handle);

            client_connected =  false;
        }
    }
    return result;
}


/*******************************************************************************
* Function Name: tcp_connection_handler
*******************************************************************************
* Summary:
*  Callback function to handle incoming TCP client connection.
*
* Parameters:
* cy_socket_t socket_handle: Handle for the TCP server socket
* void *args               : Parameter passed on to the function (unused)
*
* Return:
*  cy_result result: CY_RSLT_SUCCESS for success, error otherwise
*
*******************************************************************************/
static cy_rslt_t tcp_connection_handler(cy_socket_t socket_handle, void* arg)
{
    cy_rslt_t result;
    uint32_t peer_addr_len;
    char ip_addr_str[IP_ADDR_BUFFER_SIZE];
    cy_socket_sockaddr_t peer_addr;

    /* IP variable for network utility functions */
    cy_nw_ip_address_t nw_ip_addr =
    {
        .version = NW_IP_IPV4
    };

    /* Accept new incoming connection from a TCP client.*/
    result = cy_socket_accept(socket_handle, &peer_addr, &peer_addr_len,
                              &client_handle);
    if (result == CY_RSLT_SUCCESS)
    {
        ENT_SEC_INFO(("Incoming TCP connection accepted\n"));
        nw_ip_addr.ip.v4 = peer_addr.ip_address.ip.v4;
        cy_nw_ntoa(&nw_ip_addr, ip_addr_str);
        ENT_SEC_INFO(("IP Address : %s\n", ip_addr_str));

        client_connected = true;
    }
    else
    {
        ENT_SEC_INFO(("Failed to accept incoming client connection. Error code: 0x%08" PRIx32 "\n",
                      (uint32_t)result));
    }
    return result;
}


/*******************************************************************************
* Function Name: tcp_disconnection_handler
*******************************************************************************
* Summary:
*  Callback function to handle TCP client disconnection event..
*
* Parameters:
* cy_socket_t socket_handle: Handle for the TCP client socket
* void *args               : Parameter passed on to the function (unused)
*
* Return:
*  cy_result result: CY_RSLT_SUCCESS for success, error otherwise
*
*******************************************************************************/
static cy_rslt_t tcp_disconnection_handler(cy_socket_t socket_handle, void* arg)
{
    cy_rslt_t result;

    /* Disconnect the TCP client. */
    result = cy_socket_disconnect(socket_handle, 0);
    /* Delete the socket. */
    cy_socket_delete(socket_handle);

    ENT_SEC_INFO(("TCP Client disconnected! Please reconnect the TCP Client\n"));
    ENT_SEC_INFO(("===============================================================\n"));
    ENT_SEC_INFO(("Listening for incoming TCP client connection on Port:%d\n",
                  tcp_server_addr.port));

    client_connected = false;

    return result;
}


/*******************************************************************************
* Function Name: create_tcp_server_socket
*******************************************************************************
* Summary:
*  Create server socket with the given port.
*
* Parameters:
* uint16_t port : TCP port Number
*
* Return:
*  cy_result result: CY_RSLT_SUCCESS for success, error otherwise
*
*******************************************************************************/
static cy_rslt_t create_tcp_server_socket(uint16_t port)
{
    cy_rslt_t result;

    /* TCP socket receive timeout period. */
    uint32_t tcp_recv_timeout = TCP_SERVER_RECV_TIMEOUT_MS;

    /* Variables used to set socket options. */
    cy_socket_opt_callback_t tcp_receive_option;
    cy_socket_opt_callback_t tcp_connection_option;
    cy_socket_opt_callback_t tcp_disconnection_option;

    /* Create a TCP socket */
    result = cy_socket_create(CY_SOCKET_DOMAIN_AF_INET, CY_SOCKET_TYPE_STREAM,
                              CY_SOCKET_IPPROTO_TCP, &server_handle);
    if (result != CY_RSLT_SUCCESS)
    {
        ENT_SEC_INFO(("Failed to create socket! Error code: 0x%08" PRIx32 "\n", (uint32_t)result));
        return result;
    }

    /* Set the TCP socket receive timeout period. */
    result = cy_socket_setsockopt(server_handle, CY_SOCKET_SOL_SOCKET,
                                  CY_SOCKET_SO_RCVTIMEO, &tcp_recv_timeout,
                                  sizeof(tcp_recv_timeout));
    if (result != CY_RSLT_SUCCESS)
    {
        ENT_SEC_INFO(("Set socket option: CY_SOCKET_SO_RCVTIMEO failed\n"));
        return result;
    }

    /* Register the callback function to handle connection request from a TCP client. */
    tcp_connection_option.callback = tcp_connection_handler;
    tcp_connection_option.arg      = NULL;

    result = cy_socket_setsockopt(server_handle, CY_SOCKET_SOL_SOCKET,
                                  CY_SOCKET_SO_CONNECT_REQUEST_CALLBACK,
                                  &tcp_connection_option, sizeof(cy_socket_opt_callback_t));
    if (result != CY_RSLT_SUCCESS)
    {
        ENT_SEC_INFO(("Set socket option: CY_SOCKET_SO_CONNECT_REQUEST_CALLBACK failed\n"));
        return result;
    }

    /* Register the callback function to handle messages received from a TCP client. */
    tcp_receive_option.callback = tcp_receive_msg_handler;
    tcp_receive_option.arg      = NULL;

    result = cy_socket_setsockopt(server_handle, CY_SOCKET_SOL_SOCKET,
                                  CY_SOCKET_SO_RECEIVE_CALLBACK,
                                  &tcp_receive_option, sizeof(cy_socket_opt_callback_t));
    if (result != CY_RSLT_SUCCESS)
    {
        ENT_SEC_INFO(("Set socket option: CY_SOCKET_SO_RECEIVE_CALLBACK failed\n"));
        return result;
    }

    /* Register the callback function to handle disconnection. */
    tcp_disconnection_option.callback = tcp_disconnection_handler;
    tcp_disconnection_option.arg      = NULL;

    result = cy_socket_setsockopt(server_handle, CY_SOCKET_SOL_SOCKET,
                                  CY_SOCKET_SO_DISCONNECT_CALLBACK,
                                  &tcp_disconnection_option, sizeof(cy_socket_opt_callback_t));
    if (result != CY_RSLT_SUCCESS)
    {
        ENT_SEC_INFO(("Set socket option: CY_SOCKET_SO_DISCONNECT_CALLBACK failed\n"));
        return result;
    }

    memset(&tcp_server_addr, 0, sizeof(tcp_server_addr));
    tcp_server_addr.port = port;
    tcp_server_addr.ip_address.version = CY_SOCKET_IP_VER_V4;

    /* Bind the TCP socket created to Server IP address and to TCP port. */
    result = cy_socket_bind(server_handle, &tcp_server_addr, sizeof(tcp_server_addr));
    if (result != CY_RSLT_SUCCESS)
    {
        ENT_SEC_INFO(("Failed to bind to socket! Error code: 0x%08" PRIx32 "\n", (uint32_t)result));
    }
    return result;
}


/*******************************************************************************
* Function Name: ent_tcp_server_start
*******************************************************************************
* Summary:
*  Starts enterprise tcp echo server on given port.
*
* Parameters:
* uint16_t port : TCP port Number
*
* Return:
*  cy_result result: CY_RSLT_SUCCESS for success, error otherwise
*
*******************************************************************************/
cy_rslt_t ent_tcp_server_start(uint16_t port)
{
    cy_rslt_t result;

    if (server_started == true)
    {
        ENT_SEC_INFO(("Server is already started\n"));
        return CY_RSLT_SUCCESS;
    }

    /* Initialize secure socket library. */
    result = cy_socket_init();
    if (result != CY_RSLT_SUCCESS)
    {
        ENT_SEC_INFO(("Secure Socket initialization failed! Error code: 0x%08" PRIx32 "\n",
                      (uint32_t)result));
        return CY_RSLT_SERVER_ERR;
    }

    /* Create TCP server socket. */
    result = create_tcp_server_socket(port);
    if (result != CY_RSLT_SUCCESS)
    {
        ENT_SEC_INFO(("Failed to create socket! Error code: 0x%08" PRIx32 "\n", (uint32_t)result));
        return CY_RSLT_SERVER_ERR;
    }

    /* Start listening on the TCP server socket. */
    result = cy_socket_listen(server_handle, TCP_SERVER_MAX_PENDING_CONNECTIONS);
    if (result != CY_RSLT_SUCCESS)
    {
        cy_socket_delete(server_handle);
        ENT_SEC_INFO(("cy_socket_listen returned error. Error code: 0x%08" PRIx32 "\n",
                      (uint32_t)result));
        return CY_RSLT_SERVER_ERR;
    }
    else
    {
        ENT_SEC_INFO(("===============================================================\n"));
        ENT_SEC_INFO(("Listening for incoming TCP client connection on Port: %d\n",
                      tcp_server_addr.port));
    }
    server_started = true;
    return CY_RSLT_SUCCESS;
}


/*******************************************************************************
* Function Name: ent_tcp_server_stop
*******************************************************************************
* Summary:
*  Stops enterprise tcp echo server .
*
* Parameters:
*
* Return:
*  cy_result result: CY_RSLT_SUCCESS for success, error otherwise
*
*******************************************************************************/
cy_rslt_t ent_tcp_server_stop(void)
{
    if (server_started == false)
    {
        ENT_SEC_INFO(("Server is not started / already stopped\n"));
        return CY_RSLT_SUCCESS;
    }
    if (client_connected)
    {
        /* Disconnect the socket. */
        cy_socket_disconnect(client_handle, 0);
        /* Delete the socket. */
        cy_socket_delete(client_handle);

        client_connected =  false;
    }

    cy_socket_delete(server_handle);
    server_started = false;
    return CY_RSLT_SUCCESS;
}
