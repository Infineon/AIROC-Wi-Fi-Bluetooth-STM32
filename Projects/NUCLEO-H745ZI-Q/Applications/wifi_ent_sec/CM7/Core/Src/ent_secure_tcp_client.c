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
/** @file ent_secure_tcp_client.c
 *  secure tcp client implementation
 */

/* Header file includes. */

/* Cypress secure socket header file. */
#include "cy_secure_sockets.h"
#include "cy_tls.h"

/* Wi-Fi connection manager header files. */
#include "cy_wcm.h"
#include "cy_wcm_error.h"

/* TCP client task header file. */
#include "ent_secure_tcp_client.h"

/* IP address related header files. */
#include "cy_nw_helper.h"

#include "ent_sec_utility.h"

/* Standard C header files */
#include <inttypes.h>
#include "cy_utils.h"
/*******************************************************************************
 * Macros
 ********************************************************************************/

/* Maximum number of connection retries to the TCP server. */
#define MAX_TCP_SERVER_CONN_RETRIES (1u)

/* Length of the TCP data packet. */
#define MAX_TCP_DATA_PACKET_LENGTH (30u)

/* TCP receive timeout */
#define TCP_CLIENT_RECV_TIMEOUT_MS (1000u)

/*******************************************************************************
 * Function Prototypes
 ********************************************************************************/
static cy_rslt_t create_tcp_client_socket();
static cy_rslt_t tcp_client_recv_handler(cy_socket_t socket_handle, void* arg);
static cy_rslt_t tcp_disconnection_handler(cy_socket_t socket_handle, void* arg);
static cy_rslt_t connect_to_tcp_server(cy_socket_sockaddr_t address);

/*******************************************************************************
 * Global Variables
 ********************************************************************************/

/* TLS credentials of the TCP client. */
static const char tcp_client_cert[] = keyCLIENT_CERTIFICATE_PEM;
static const char client_private_key[] = keyCLIENT_PRIVATE_KEY_PEM;

/* Root CA certificate for TCP server identity verification. */
static const char tcp_server_ca_cert[] = keySERVER_ROOTCA_PEM;

/* Variable to store the TLS identity (certificate and private key). */
static void* tls_identity;
/* TCP client socket handle */
static cy_socket_t client_handle;

/*******************************************************************************
 * Function Name: create_tcp_client_socket
 ******************************************************************************/
static cy_rslt_t create_tcp_client_socket()
{
    cy_rslt_t result;

    cy_socket_tls_auth_mode_t tls_auth_mode = CY_SOCKET_TLS_VERIFY_REQUIRED;

    /* TCP socket receive timeout period. */
    uint32_t tcp_recv_timeout = TCP_CLIENT_RECV_TIMEOUT_MS;

    /* Variables used to set socket options. */
    cy_socket_opt_callback_t tcp_recv_option;
    cy_socket_opt_callback_t tcp_disconnect_option;

    /* Create a new secure TCP socket. */
    result = cy_socket_create(CY_SOCKET_DOMAIN_AF_INET, CY_SOCKET_TYPE_STREAM,
                              CY_SOCKET_IPPROTO_TLS, &client_handle);
    if (result != CY_RSLT_SUCCESS)
    {
        ENT_SEC_ERR(("Failed to create socket! Error Code: %" PRIu32 "\n", result));
        return result;
    }

    /* Set the TCP socket receive timeout period. */
    result = cy_socket_setsockopt(client_handle, CY_SOCKET_SOL_SOCKET,
                                  CY_SOCKET_SO_RCVTIMEO, &tcp_recv_timeout,
                                  sizeof(tcp_recv_timeout));
    if (result != CY_RSLT_SUCCESS)
    {
        ENT_SEC_ERR(("Set socket option: CY_SOCKET_SO_RCVTIMEO failed\n"));
        return result;
    }

    /* Register the callback function to handle messages received from TCP server. */
    tcp_recv_option.callback = tcp_client_recv_handler;
    tcp_recv_option.arg = NULL;
    result = cy_socket_setsockopt(client_handle, CY_SOCKET_SOL_SOCKET,
                                  CY_SOCKET_SO_RECEIVE_CALLBACK,
                                  &tcp_recv_option, sizeof(cy_socket_opt_callback_t));
    if (result != CY_RSLT_SUCCESS)
    {
        ENT_SEC_ERR(("Set socket option: CY_SOCKET_SO_RECEIVE_CALLBACK failed\n"));
        return result;
    }

    /* Register the callback function to handle disconnection. */
    tcp_disconnect_option.callback = tcp_disconnection_handler;
    tcp_disconnect_option.arg = NULL;

    result = cy_socket_setsockopt(client_handle, CY_SOCKET_SOL_SOCKET,
                                  CY_SOCKET_SO_DISCONNECT_CALLBACK,
                                  &tcp_disconnect_option, sizeof(cy_socket_opt_callback_t));
    if (result != CY_RSLT_SUCCESS)
    {
        ENT_SEC_ERR(("Set socket option: CY_SOCKET_SO_DISCONNECT_CALLBACK failed\n"));
    }

    /* Set the TCP socket to use the TLS identity. */
    result = cy_socket_setsockopt(client_handle, CY_SOCKET_SOL_TLS, CY_SOCKET_SO_TLS_IDENTITY,
                                  tls_identity, sizeof((uint32_t)tls_identity));
    if (result != CY_RSLT_SUCCESS)
    {
        ENT_SEC_ERR(("Set socket option: CY_SOCKET_SO_TLS_IDENTITY failed! "
                     "Error Code: %" PRIu32 "\n", result));
    }

    /* Set the TLS authentication mode. */
    result = cy_socket_setsockopt(client_handle, CY_SOCKET_SOL_TLS, CY_SOCKET_SO_TLS_AUTH_MODE,
                                  &tls_auth_mode, sizeof(cy_socket_tls_auth_mode_t));
    if (result != CY_RSLT_SUCCESS)
    {
        ENT_SEC_ERR(("Set socket option: CY_SOCKET_SO_TLS_AUTH_MODE failed! "
                     "Error Code: %" PRIu32 "\n", result));
    }
    return result;
}


/*******************************************************************************
* Function Name: connect_to_tcp_server
*******************************************************************************/
static cy_rslt_t connect_to_tcp_server(cy_socket_sockaddr_t address)
{
    cy_rslt_t result = CY_RSLT_MODULE_SECURE_SOCKETS_TIMEOUT;
    cy_rslt_t conn_result;

    for (uint32_t conn_retries = 0; conn_retries < MAX_TCP_SERVER_CONN_RETRIES; conn_retries++)
    {
        /* Create a TCP socket */
        conn_result = create_tcp_client_socket();

        if (conn_result != CY_RSLT_SUCCESS)
        {
            ENT_SEC_ERR(("Socket creation failed!\n"));
            CY_ASSERT(0);
        }

        conn_result = cy_socket_connect(client_handle, &address, sizeof(cy_socket_sockaddr_t));

        if (conn_result == CY_RSLT_SUCCESS)
        {
            ENT_SEC_INFO(("============================================================\n"));
            ENT_SEC_INFO(("Connected to TCP server\n"));

            return conn_result;
        }

        ENT_SEC_INFO(("Could not connect to TCP server. Error code: 0x%08" PRIx32 "\n",
                      (uint32_t)result));

        /* The resources allocated during the socket creation (cy_socket_create)
         * should be deleted.
         */
        cy_socket_delete(client_handle);

        client_handle = NULL;
    }

    /* Stop retrying after maximum retry attempts. */
    ENT_SEC_INFO(("Exceeded maximum connection attempts to the TCP server\n"));

    return result;
}


/*******************************************************************************
* Function Name: tcp_client_recv_handler
*******************************************************************************/
static cy_rslt_t tcp_client_recv_handler(cy_socket_t socket_handle, void* arg)
{
    /* Variable to store number of bytes send to the TCP server. */
    uint32_t bytes_sent = 0;

    /* Variable to store number of bytes received. */
    uint32_t bytes_received = 0;

    char message_buffer[MAX_TCP_DATA_PACKET_LENGTH];
    cy_rslt_t result;

    result = cy_socket_recv(socket_handle, message_buffer, MAX_TCP_DATA_PACKET_LENGTH -1,
                            CY_SOCKET_FLAGS_NONE, &bytes_received);

    if (bytes_received > 0)
    {
        message_buffer[MAX_TCP_DATA_PACKET_LENGTH - 1] = 0;


        ENT_SEC_INFO(("============================================================\n"));

        ENT_SEC_INFO(("MSG received : %s\n", message_buffer));

        sprintf(message_buffer, "ACK");

        /* Send acknowledgment to the TCP server in receipt of the message received. */
        result = cy_socket_send(socket_handle, message_buffer, strlen(message_buffer),
                                CY_SOCKET_FLAGS_NONE, &bytes_sent);
        if (result == CY_RSLT_SUCCESS)
        {
            ENT_SEC_INFO(("Acknowledgment sent to TCP server\n"));
        }
        ENT_SEC_INFO(("============================================================\n"));
    }
    return result;
}


/*******************************************************************************
* Function Name: tcp_disconnection_handler
*******************************************************************************
* Summary:
*  Callback function to handle TCP socket disconnection event.
*
* Parameters:
*  cy_socket_t socket_handle: Connection handle for the TCP client socket
*  void *args : Parameter passed on to the function (unused)
*
* Return:
*  cy_result result: Result of the operation
*
*******************************************************************************/
static cy_rslt_t tcp_disconnection_handler(cy_socket_t socket_handle, void* arg)
{
    cy_rslt_t result;

    /* Disconnect the TCP client. */
    result = cy_socket_disconnect(socket_handle, 0);

    /* Delete identity */
    if (tls_identity)
    {
        cy_tls_delete_identity(tls_identity);
        tls_identity = NULL;
    }

    /* release root certificate */
    cy_tls_release_global_root_ca_certificates();

    /* Free the resources allocated to the socket. */
    cy_socket_delete(socket_handle);

    /* update global var */
    client_handle = NULL;

    ENT_SEC_INFO(("Disconnected from the TCP server! \n"));

    return result;
}


/*******************************************************************************
 * Function Name: ent_secure_tcp_client_connect
 ******************************************************************************/
cy_rslt_t ent_secure_tcp_client_connect(char* ip, uint16_t port)
{
    cy_rslt_t result;

    /* IP address and TCP port number of the TCP server to which the TCP client
     * connects to.
     */
    cy_socket_sockaddr_t tcp_server_address =
    {
        .ip_address.version = CY_SOCKET_IP_VER_V4,
        .port               = port
    };

    /* IP variable for network utility functions */
    cy_nw_ip_address_t nw_ip_addr =
    {
        .version = NW_IP_IPV4
    };

    /* TCP client certificate length and private key length. */
    const size_t tcp_client_cert_len = strlen(tcp_client_cert);
    const size_t pkey_len = strlen(client_private_key);

    /* Initialize secure socket library. */
    result = cy_socket_init();

    if (result != CY_RSLT_SUCCESS)
    {
        ENT_SEC_ERR(("Secure Socket initialization failed!\n"));
        CY_ASSERT(0);
    }
    ENT_SEC_INFO(("Secure Socket initialized\n"));

    result =
        cy_tls_load_global_root_ca_certificates(tcp_server_ca_cert, strlen(tcp_server_ca_cert));
    if (result != CY_RSLT_SUCCESS)
    {
        ENT_SEC_ERR(("cy_tls_load_global_root_ca_certificates failed! Error code: %" PRIu32 "\n",
                     result));
    }
    else
    {
        ENT_SEC_INFO(("Global trusted RootCA certificate loaded\n"));
    }

    /* Create TCP client identity using the SSL certificate and private key. */
    result = cy_tls_create_identity(tcp_client_cert, tcp_client_cert_len,
                                    client_private_key, pkey_len, &tls_identity);
    if (result != CY_RSLT_SUCCESS)
    {
        ENT_SEC_ERR(("Failed cy_tls_create_identity! Error code: %" PRIu32 "\n", result));
        CY_ASSERT(0);
    }

    ENT_SEC_INFO(("Connecting to TCP Server (IP Address: %s, Port: %d)\n\n",
                  ip, port));

    cy_nw_str_to_ipv4((char*)ip, (cy_nw_ip_address_t*)&nw_ip_addr);
    tcp_server_address.ip_address.ip.v4 = nw_ip_addr.ip.v4;

    result = connect_to_tcp_server(tcp_server_address);
    if (result != CY_RSLT_SUCCESS)
    {
        ENT_SEC_ERR(("Failed to connect to TCP server.\n"));

        cy_tls_delete_identity(tls_identity);
        tls_identity = NULL;

        return CY_RSLT_MODULE_SECURE_SOCKETS_NOT_CONNECTED;
    }

    return CY_RSLT_SUCCESS;
}


/*******************************************************************************
 * Function Name: ent_secure_tcp_client_disconnect
 ******************************************************************************/
cy_rslt_t ent_secure_tcp_client_disconnect(void)
{
    if (client_handle)
    {
        tcp_disconnection_handler(client_handle, NULL);

        client_handle = NULL;
    }
    return CY_RSLT_SUCCESS;
}


/* [] END OF FILE */
