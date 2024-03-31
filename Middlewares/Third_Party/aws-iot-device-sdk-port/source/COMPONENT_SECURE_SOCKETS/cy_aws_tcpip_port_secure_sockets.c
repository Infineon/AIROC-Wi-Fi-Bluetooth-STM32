/*
 * Copyright 2023, Cypress Semiconductor Corporation (an Infineon company) or
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

/** @file cy_aws_tcpip_port_secure_sockets.c
 *  Implements TCP server APIs using secure sockets library.
 *
 */
#include <string.h>
#include "cy_secure_sockets.h"
#include "cy_tcpip_port_secure_sockets.h"
#include "cy_result_mw.h"
#include "cy_aws_iot_sdk_port_log.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Static Function Declarations
 ******************************************************/

/******************************************************
 *                 Static Variables
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

static cy_rslt_t awsport_callback( cy_socket_t socket_handle, void *arg )
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    cy_awsport_callback_t *cb = (cy_awsport_callback_t *)arg;

    if ( cb->cbf )
    {
        cb->cbf( cb->user_data );
    }

    return result;
}

cy_rslt_t cy_awsport_network_init( void )
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    /* Initialize the secure sockets library. */
    result = cy_socket_init();
    if ( result != CY_RSLT_SUCCESS )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Secure Sockets initialization failed with Error : [0x%X] ", (unsigned int)result );
        return result;
    }

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Secure Sockets initialization completed\n" );
    return result;
}

cy_rslt_t cy_awsport_network_deinit( void )
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    /* DeInitialize the secure sockets library. */
    result = cy_socket_deinit();
    if ( result != CY_RSLT_SUCCESS )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Secure Sockets deinitialization failed with Error : [0x%X] ", (unsigned int)result );
    }

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Secure Sockets deinitialization completed!\n" );
    return result;
}

cy_rslt_t cy_awsport_network_create( NetworkContext_t *network_context,
                                     const cy_awsport_server_info_t *server_info,
                                     const cy_awsport_ssl_credentials_t *ssl_credentials,
                                     cy_awsport_callback_t *discon_cb,
                                     cy_awsport_callback_t *receive_cb )
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    /* Flags to track initialization. */
    cy_socket_tls_auth_mode_t authmode = CY_SOCKET_TLS_VERIFY_NONE;
#ifdef CY_SECURE_SOCKETS_PKCS_SUPPORT
    uint8_t cert_location = 0;
#endif
    cy_socket_ip_address_t ip_addr;
    char *ptr;

    if( (network_context == NULL) || (server_info == NULL) )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_INFO, "\nInvalid parameter to cy_awsport_network_create.!\n" );
        return CY_RSLT_MODULE_SECURE_SOCKETS_BADARG;
    }

    memset( &ip_addr, 0x0, sizeof(cy_socket_ip_address_t) );

    /* Initialize members of the network context. */
    network_context->handle = NULL;
    network_context->is_rootca_loaded = false;
    network_context->tls_identity = NULL;
    memset( &(network_context->address), 0x0, sizeof(cy_socket_sockaddr_t) );

    result = cy_socket_gethostbyname( server_info->host_name, CY_SOCKET_IP_VER_V4, &ip_addr );
    if( result != CY_RSLT_SUCCESS )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "\ncy_socket_gethostbyname failed with Error : [0x%X] ", (unsigned int)result );
        goto exit;
    }

    network_context->address.port = server_info->port;
    memcpy( &(network_context->address.ip_address), &ip_addr, sizeof(cy_socket_ip_address_t) );

    ptr = (char *) &ip_addr.ip.v4;
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_INFO, "\nIP bytes %d %d %d %d \n", *(ptr+0), *(ptr+1), *(ptr+2), *(ptr+3) );
    (void)ptr;

    /* Check for a secured connection. */
    if( ssl_credentials != NULL )
    {
        if( (ssl_credentials->root_ca != NULL) && (ssl_credentials->root_ca_size > 0) )
        {
            result = cy_tls_load_global_root_ca_certificates( ssl_credentials->root_ca, ssl_credentials->root_ca_size - 1 );
            if( result != CY_RSLT_SUCCESS )
            {
                cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_INFO, "\ncy_tls_load_global_root_ca_certificates failed with Error : [0x%X] ", (unsigned int)result );
                goto exit;
            }
            network_context->is_rootca_loaded = true;
            if( ssl_credentials->root_ca_verify_mode == CY_AWS_ROOTCA_VERIFY_REQUIRED )
            {
                authmode = CY_SOCKET_TLS_VERIFY_REQUIRED;
            }
            else if( ssl_credentials->root_ca_verify_mode == CY_AWS_ROOTCA_VERIFY_NONE )
            {
                authmode = CY_SOCKET_TLS_VERIFY_NONE;
            }
            else
            {
                authmode = CY_SOCKET_TLS_VERIFY_OPTIONAL;
            }
        }
#ifdef CY_SECURE_SOCKETS_PKCS_SUPPORT
        else
        {
            if( ssl_credentials->root_ca_verify_mode == CY_AWS_ROOTCA_VERIFY_REQUIRED )
            {
                authmode = CY_SOCKET_TLS_VERIFY_REQUIRED;
            }
            else if( ssl_credentials->root_ca_verify_mode == CY_AWS_ROOTCA_VERIFY_NONE )
            {
                authmode = CY_SOCKET_TLS_VERIFY_NONE;
            }
            else
            {
                authmode = CY_SOCKET_TLS_VERIFY_OPTIONAL;
            }
        }
#endif
        if( (ssl_credentials->client_cert != NULL) && (ssl_credentials->client_cert_size > 0) &&
            (ssl_credentials->private_key != NULL) && (ssl_credentials->private_key_size > 0) )
        {
            result = cy_tls_create_identity( ssl_credentials->client_cert, ssl_credentials->client_cert_size - 1,
                                             ssl_credentials->private_key, ssl_credentials->private_key_size - 1,
                                             &(network_context->tls_identity) );
            if( result != CY_RSLT_SUCCESS )
            {
                cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "\ncy_tls_create_identity failed with Error : [0x%X] ", (unsigned int)result );
                goto exit;
            }
        }

        result = cy_socket_create( CY_SOCKET_DOMAIN_AF_INET, CY_SOCKET_TYPE_STREAM,
                                   CY_SOCKET_IPPROTO_TLS, &(network_context->handle) );
        if( result != CY_RSLT_SUCCESS )
        {
            cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "\nSocket create failed with Error : [0x%X] ", (unsigned int)result );
            goto exit;
        }

        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_INFO, "\ncy_socket_create Success\n" );

#ifdef CY_SECURE_SOCKETS_PKCS_SUPPORT
        cert_location = CY_SOCKET_DEVICE_CERT_KEY_SECURE_STORAGE;
        if( network_context->is_rootca_loaded == true )
        {
            cert_location = CY_SOCKET_DEVICE_CERT_KEY_RAM;
        }
        else
        {
            if( ssl_credentials->root_ca_location == CY_AWS_CERT_KEY_LOCATION_RAM )
            {
                cert_location = CY_SOCKET_DEVICE_CERT_KEY_RAM;
            }
        }

        result = cy_socket_setsockopt( network_context->handle, CY_SOCKET_SOL_TLS,
                                       CY_SOCKET_SO_ROOTCA_CERTIFICATE_LOCATION, (const void *)&cert_location,
                                       (uint32_t) sizeof( cert_location ) );
        if( result != CY_RSLT_SUCCESS )
        {
            cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "\ncy_socket_setsockopt failed with Error : [0x%X] ", (unsigned int)result );
            goto exit;
        }
#endif

        if( network_context->tls_identity != NULL )
        {
            /* FALSE-POSITIVE:
             * CID: 217093 Wrong sizeof argument
             *     The last argument is expected to be the size of the pointer itself which is 4 bytes; therefore, this is a false positive.
             */
            result = cy_socket_setsockopt( network_context->handle, CY_SOCKET_SOL_TLS,
                                           CY_SOCKET_SO_TLS_IDENTITY, network_context->tls_identity,
                                           (uint32_t) sizeof( network_context->tls_identity ) );
            if( result != CY_RSLT_SUCCESS )
            {
                cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "\ncy_socket_setsockopt failed with Error : [0x%X] ", (unsigned int)result );
                goto exit;
            }
        }

#ifdef CY_SECURE_SOCKETS_PKCS_SUPPORT
        cert_location = CY_SOCKET_DEVICE_CERT_KEY_SECURE_STORAGE;
        if( network_context->tls_identity != NULL )
        {
            cert_location = CY_SOCKET_DEVICE_CERT_KEY_RAM;
        }
        else
        {
            if( ssl_credentials->cert_key_location == CY_AWS_CERT_KEY_LOCATION_RAM )
            {
                cert_location = CY_SOCKET_DEVICE_CERT_KEY_RAM;
            }
        }

        result = cy_socket_setsockopt( network_context->handle, CY_SOCKET_SOL_TLS,
                                       CY_SOCKET_SO_DEVICE_CERT_KEY_LOCATION, (const void *)&cert_location,
                                       (uint32_t) sizeof( cert_location ) );
        if( result != CY_RSLT_SUCCESS )
        {
            cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "\ncy_socket_setsockopt failed with Error : [0x%X] ", (unsigned int)result );
            goto exit;
        }
#endif

        result = cy_socket_setsockopt( network_context->handle, CY_SOCKET_SOL_TLS,
                                       CY_SOCKET_SO_TLS_AUTH_MODE, (const void *) &authmode,
                                       (uint32_t) sizeof( authmode ) );
        if( result != CY_RSLT_SUCCESS )
        {
            cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "\nSetting TLS auth mode failed with Error : [0x%X] ", (unsigned int)result );
            goto exit;
        }

        if( ssl_credentials->sni_host_name != NULL )
        {
            result = cy_socket_setsockopt( network_context->handle, CY_SOCKET_SOL_TLS,
                                           CY_SOCKET_SO_SERVER_NAME_INDICATION, (const void *) ssl_credentials->sni_host_name,
                                           (uint32_t) (ssl_credentials->sni_host_name_size - 1) );
            if( result != CY_RSLT_SUCCESS )
            {
                cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "\nSetting SNI name failed with Error : [0x%X] ", (unsigned int)result );
                goto exit;
            }
        }
    }
    else
    {
        result = cy_socket_create( CY_SOCKET_DOMAIN_AF_INET, CY_SOCKET_TYPE_STREAM,
                                   CY_SOCKET_IPPROTO_TCP, &(network_context->handle) );
        if( result != CY_RSLT_SUCCESS )
        {
            cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "\nSocket create failed with Error : [0x%X] ", (unsigned int)result );
            goto exit;
        }

        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_INFO, "\ncy_socket_create Success\n" );
    }

    network_context->address.ip_address.ip.v4 = ip_addr.ip.v4;
    network_context->address.ip_address.version = CY_SOCKET_IP_VER_V4;
    network_context->address.port = server_info->port;

    /* Store the disconnect information for socket connection. */
    if ( discon_cb != NULL )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "\nStore the disconnect information for socket handle = %p\n", network_context->handle );
        network_context->disconnect_info.cbf = discon_cb->cbf;
        network_context->disconnect_info.user_data = discon_cb->user_data;
    }

    /* Store the data receive callback information for socket connection. */
    if ( receive_cb != NULL )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "\nStore the data receive callback information for socket handle = %p\n", network_context->handle );
        network_context->receive_info.cbf = receive_cb->cbf;
        network_context->receive_info.user_data = receive_cb->user_data;
    }

exit:
    /* Clean up on error. */
    if( result != CY_RSLT_SUCCESS )
    {
        if( network_context->is_rootca_loaded == true )
        {
            cy_tls_release_global_root_ca_certificates();
            network_context->is_rootca_loaded = false;
        }

        if( network_context->tls_identity != NULL )
        {
            cy_tls_delete_identity( network_context->tls_identity );
            network_context->tls_identity = NULL;
        }

        if( network_context->handle != NULL )
        {
            cy_socket_delete( network_context->handle );
            network_context->handle = NULL;
        }
    }
    else
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_INFO, "\n(Network connection %p) New network connection established.", network_context );
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_INFO, "\nNew network connection handle : %p.", network_context->handle );
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_INFO, "\nNew network connection tls_identity : %p.", network_context->tls_identity );
    }
    return result;
}

cy_rslt_t cy_awsport_network_connect( NetworkContext_t *network_context,
                                      uint32_t send_timeout_ms,
                                      uint32_t recv_timeout_ms )
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    cy_socket_opt_callback_t socket_callback;

    if( network_context == NULL )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_INFO, "\nInvalid parameter to cy_awsport_network_connect.!\n" );
        return CY_RSLT_MODULE_SECURE_SOCKETS_BADARG;
    }

    result = cy_socket_connect( network_context->handle, &(network_context->address),
                                sizeof(cy_socket_sockaddr_t) );
    if( result != CY_RSLT_SUCCESS )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "\ncy_socket_connect failed with Error : [0x%X] ", (unsigned int)result );
        return result;
    }
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_INFO, "\ncy_socket_connect Success\n" );

    /* Set the receive timeout for socket connection. */
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "\nSet socket receive timeout for socket handle = %p timeout = %ld\n",
                   network_context->handle, recv_timeout_ms );
    result = cy_socket_setsockopt( network_context->handle, CY_SOCKET_SOL_SOCKET,
                                   CY_SOCKET_SO_RCVTIMEO,
                                   &recv_timeout_ms, sizeof( recv_timeout_ms ) );
    if( result != CY_RSLT_SUCCESS )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "\nSet receive timeout for socket handle = %p failed with Error : [0x%X]",
                       network_context->handle, ( unsigned int )result );
        return result;
    }

    /* Set the send timeout for socket connection. */
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "\nSet socket send timeout for socket handle = %p timeout = %ld\n",
                   network_context->handle, send_timeout_ms );
    result = cy_socket_setsockopt( network_context->handle, CY_SOCKET_SOL_SOCKET,
                                   CY_SOCKET_SO_SNDTIMEO,
                                   &send_timeout_ms, sizeof( send_timeout_ms ) );
    if( result != CY_RSLT_SUCCESS )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "\nSet send timeout for socket handle = %p failed with Error : [0x%X]",
                       network_context->handle, ( unsigned int )result );
        return result;
    }

    if( network_context->disconnect_info.cbf != NULL )
    {
        /* Set the disconnect notification for socket connection. */
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "\nSet socket disconnect notification for socket handle = %p\n",
                       network_context->handle );

        socket_callback.callback = awsport_callback;
        socket_callback.arg = (void *)&network_context->disconnect_info;

        result = cy_socket_setsockopt( network_context->handle, CY_SOCKET_SOL_SOCKET,
                                       CY_SOCKET_SO_DISCONNECT_CALLBACK,
                                       &socket_callback, sizeof(socket_callback) );
        if( result != CY_RSLT_SUCCESS )
        {
            cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "\nSet socket disconnect notification for socket handle = %p failed with Error : [0x%X]",
                           network_context->handle, ( unsigned int )result );
            return result;
        }
    }

    if( network_context->receive_info.cbf != NULL )
    {
        /* Set Socket data receive callback */
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "\nSet socket data receive notification for socket handle = %p\n", network_context->handle );

        socket_callback.callback = awsport_callback;
        socket_callback.arg = (void *)&network_context->receive_info;

        result = cy_socket_setsockopt( network_context->handle, CY_SOCKET_SOL_SOCKET,
                                       CY_SOCKET_SO_RECEIVE_CALLBACK,
                                       &socket_callback, sizeof(socket_callback) );
        if( result != CY_RSLT_SUCCESS )
        {
            cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "\nSet socket data receive notification for socket handle = %p failed with Error : [0x%X]", network_context->handle, ( unsigned int )result );
            return result;
        }
    }

    return result;
}

cy_rslt_t cy_awsport_network_disconnect( NetworkContext_t *network_context )
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    if( network_context == NULL )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "\nInvalid parameter to cy_socket_disconnect.!\n" );
        return CY_RSLT_MODULE_SECURE_SOCKETS_BADARG;
    }
    if(network_context->handle != NULL)
    {
        /* Disconnect the network connection. */
        result = cy_socket_disconnect( network_context->handle, 0 );
        if( ( result !=  CY_RSLT_SUCCESS ) && ( result != CY_RSLT_MODULE_SECURE_SOCKETS_NOT_CONNECTED ) )
        {
            cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "\ncy_socket_disconnect failed with Error : [0x%X] ", (unsigned int)result );
        }
    }
    return result;
}

cy_rslt_t cy_awsport_network_delete( NetworkContext_t *network_context )
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    if( network_context == NULL )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "\nInvalid parameter to cy_socket_disconnect.!\n" );
        return CY_RSLT_MODULE_SECURE_SOCKETS_BADARG;
    }

    /* Free the tls_identity for a secured connection. */
    if( network_context->tls_identity != NULL )
    {
        result = cy_tls_delete_identity( network_context->tls_identity );
        if( result != CY_RSLT_SUCCESS )
        {
            cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "\ncy_tls_delete_identity failed with Error : [0x%X] ", (unsigned int)result );
            return result;
        }
        network_context->tls_identity = NULL;
    }

    /* Free the root certificates for a secured connection. */
    if( network_context->is_rootca_loaded == true )
    {
        result = cy_tls_release_global_root_ca_certificates();
        if( result != CY_RSLT_SUCCESS )
        {
            cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "\ncy_tls_release_global_root_ca_certificates failed with Error : [0x%X] ", (unsigned int)result );
            return result;
        }
        network_context->is_rootca_loaded = false;
    }

    if(network_context->handle != NULL)
    {
        /* Clean up the context of secure sockets connections. */
        result = cy_socket_delete( network_context->handle );
        if( result != CY_RSLT_SUCCESS )
        {
            cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "\ncy_socket_delete failed with Error : [0x%X] ", (unsigned int)result );
        }
        network_context->handle = NULL;
    }

    return result;
}

int32_t cy_awsport_network_receive( NetworkContext_t *network_context, void *buffer, size_t bytes_recv )
{
    cy_rslt_t result = CY_RSLT_SUCCESS ;
    int32_t bytes_received = -1;

    if( (network_context == NULL) || (buffer == NULL) )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "\nInvalid parameter to cy_awsport_network_receive.!\n" );
        return bytes_received;
    }

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "\n(Socket handle %p) requesting %lu bytes.", network_context->handle, ( unsigned long )bytes_recv );

    result = cy_socket_recv( network_context->handle, buffer, bytes_recv, 0, (uint32_t *)&bytes_received );
    if( result != CY_RSLT_SUCCESS )
    {
        /* No data is available; wait for the next event. */
        if( result == CY_RSLT_MODULE_SECURE_SOCKETS_TIMEOUT )
        {
            cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "\ncy_socket_recv timeout..\n" );
            bytes_received = 0;
        }
        else
        {
            cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "\ncy_socket_recv failed with Error = [0x%X]\n", ( unsigned int )result );
            bytes_received = -1;
        }
    }

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "\n(Socket handle %p) received %lu bytes.", network_context->handle, ( unsigned long )bytes_received );
    return bytes_received;
}

int32_t cy_awsport_network_send( NetworkContext_t *network_context, const void *buffer, size_t bytes_send )
{
    cy_rslt_t result = CY_RSLT_SUCCESS ;
    int32_t bytes_sent = -1;

    if( (network_context == NULL) || (buffer == NULL) )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "\nInvalid parameter to cy_awsport_network_send.!\n" );
        return bytes_sent;
    }
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "\n(Socket handle %p) Sending %lu bytes.", network_context->handle, ( unsigned long ) bytes_send );

    result = cy_socket_send( network_context->handle, buffer, bytes_send, 0, (uint32_t *)&bytes_sent );
    if( result != CY_RSLT_SUCCESS )
    {
        /* Check for the send timeout error. */
        if( result == CY_RSLT_MODULE_SECURE_SOCKETS_TIMEOUT )
        {
            cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "\ncy_socket_send timeout..\n" );
            bytes_sent = 0;
        }
        else
        {
            cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "\ncy_socket_send failed with Error = [0x%X]\n", ( unsigned int )result );
            bytes_sent = -1;
        }
    }

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "\n(Socket handle %p) sent %lu bytes.", network_context->handle, ( unsigned long )bytes_sent );
    return( bytes_sent );
}
