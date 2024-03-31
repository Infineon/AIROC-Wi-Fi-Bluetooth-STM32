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

/** @file
 *  The APIs defined in this file are used by Cypress MQTT and HTTP Client libraries to integrate Amazon's AWS IoT Device SDK Embedded C libray on Cypress PSoC 6 MCU platforms to provide MQTT and HTTP Client functionalities.
 *
 */

/**
 * \defgroup group_aws_iot_device_sdk_port AWS IoT Device SDK Port Library API
 * \addtogroup group_aws_iot_device_sdk_port
 * \{
 * \defgroup group_aws_iot_device_sdk_port_typedefs Typedefs
 * \defgroup group_aws_iot_device_sdk_port_structures Structures
 * \defgroup group_aws_iot_device_sdk_port_functions Functions
 */

#ifndef CY_TCPIP_PORT_SECURE_SOCKET_H_
#define CY_TCPIP_PORT_SECURE_SOCKET_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "cy_aws_iot_port_error.h"
#include "cy_secure_sockets.h"
#include "transport_interface.h"
#include "cy_tls.h"

/**
 * \addtogroup group_aws_iot_device_sdk_port_typedefs
 * \{
 */

/**
 * awsport event callback function.
 *
 * @param user_data [in]    : User data passed by the caller.
 *
 * @return                  : void
 */
typedef void ( *cy_awsport_event_callback_t )( void *user_data );
/** \} group_aws_iot_device_sdk_port_typedefs */


/**
 * \addtogroup group_aws_iot_device_sdk_port_structures
 * \{
 */

/**
 * Contains the awsport event callback information structure.
 */
typedef struct cy_awsport_callback
{
    cy_awsport_event_callback_t     cbf;         /**< Pointer to the callback function. */
    void                            *user_data;  /**< User data to be returned when the callback function is invoked. */
} cy_awsport_callback_t;

/**
 * Represents the server information.
 */
typedef struct cy_awsport_server_info
{
    const char *host_name;                       /**< Server host name. Must be NULL-terminated. */
    uint16_t    port;                            /**< Server port in host-order. */
} cy_awsport_server_info_t;

/**
 * Network context structure containing socket information.
 */
struct NetworkContext
{
    cy_socket_t           handle;                /**< Socket Handle. */
    cy_socket_sockaddr_t  address;               /**< Socket Address. */
    void                 *tls_identity;          /**< TLS Socket Identity. */
    cy_awsport_callback_t disconnect_info;       /**< Disconnect callback function and user data. */
    cy_awsport_callback_t receive_info;          /**< Callback information for data receive */
    bool                  is_rootca_loaded;      /**< Status to track whether Root CA is loaded. */
};

/**
 * Represents the RootCA verification mode for MQTT/HTTP client socket connection.
 */
typedef enum cy_awsport_rootca_verify_mode
{
    CY_AWS_ROOTCA_VERIFY_REQUIRED = 0,           /**< Peer RootCA verification to be done during TLS handshake. Handshake is aborted if verification fails. This mode is a default verify mode for client sockets. */
    CY_AWS_ROOTCA_VERIFY_NONE = 1,               /**< Skip Peer RootCA verification during TLS handshake. */
    CY_AWS_ROOTCA_VERIFY_OPTIONAL = 2            /**< Peer RootCA verification to be done during TLS handshake. Even if the RootCA verification fails, continue with the TLS handshake. */
} cy_awsport_rootca_verify_mode_t;

/**
 * Represents the memory location for reading certificates and key during TLS connection.
 */
typedef enum cy_awsport_cert_key_location
{
#ifdef CY_SECURE_SOCKETS_PKCS_SUPPORT
    CY_AWS_CERT_KEY_LOCATION_SECURE_STORAGE = 0, /**< Read certificates and key from secured element (default location for PKCS flow). */
    CY_AWS_CERT_KEY_LOCATION_RAM            = 1  /**< Read certificates and key from the given buffer. */
#else
    CY_AWS_CERT_KEY_LOCATION_RAM            = 0  /**< Read certificates and key from the given buffer (default location for Non PKCS flow). */
#endif
} cy_awsport_cert_key_location_t;

/**
 * Contains the credentials to establish a TLS connection.
 */
typedef struct cy_awsport_ssl_credentials
{
    const char *alpnprotos;                      /**< An array of ALPN protocols. Set to NULL to disable ALPN. */
    size_t      alpnprotoslen;                   /**< Length of the ALPN protocols array. */

    /**
     * @note These strings must be NULL-terminated because the secure sockets API requires them to be.
     *       Memory allocated for sni_host_name, root_ca, client_cert, private_key, username, and
     *       password should be maintained by the caller until MQTT/HTTP object is deleted.
     */
    const char                        *sni_host_name;           /**< Set a host name to enable SNI. Set to NULL to disable SNI. */
    size_t                             sni_host_name_size;      /**< Size of the SNI host name. */
    const char                        *root_ca;                 /**< String representing a trusted server root certificate. */
    size_t                             root_ca_size;            /**< Size of the Root CA certificate. */
    cy_awsport_rootca_verify_mode_t    root_ca_verify_mode;     /**< RootCA verification mode for client sockets. */
    cy_awsport_cert_key_location_t     root_ca_location;        /**< RootCA location for TLS connection. */
    const char                        *client_cert;             /**< String representing the client certificate. */
    size_t                             client_cert_size;        /**< Size of the client certificate. */
    const char                        *private_key;             /**< String representing the client certificate's private key. */
    size_t                             private_key_size;        /**< Size of the private Key. */
    cy_awsport_cert_key_location_t     cert_key_location;       /**< Client key and Client certificate location for TLS connection. */
    const char                        *username;                /**< String representing the username for the HTTP/MQTT client. */
    size_t                             username_size;           /**< Size of the user name. */
    const char                        *password;                /**< String representing the password for the HTTP/MQTT client. */
    size_t                             password_size;           /**< Size of the password. */
} cy_awsport_ssl_credentials_t;
/** \} group_aws_iot_device_sdk_port_structures */

/**
 * \addtogroup group_aws_iot_device_sdk_port_functions
 * \{
 */
/**
 * Initializes network sockets by calling the secure sockets initialize function.
 *
 * @return cy_rslt_t              : CY_RSLT_SUCCESS on success; secure sockets library error code otherwise.
 */

cy_rslt_t cy_awsport_network_init( void );

/**
 * De-initializes network sockets by calling the secure sockets deinitialize function.
 *
 * @return cy_rslt_t              : CY_RSLT_SUCCESS on success; secure sockets library error code otherwise.
 */
cy_rslt_t cy_awsport_network_deinit( void );

/**
 * Sets up a TLS session and socket creation using the secure sockets API.
 *
 * @note This API and cy_awsport_network_connect API should always be called together. The secure sockets lwIP port layer has a limitation that it will delete the connection instance created in cy_awsport_network_create() as part of cy_awsport_network_disconnect().
 *
 * @param network_context [in]        : Client network context.
 * @param server_info [in]            : Server connection info.
 * @param ssl_credentials [in]        : Credentials for the TLS connection.
 * @param discon_cb [in]              : Socket disconnect callback function.
 * @param receive_cb [in]             : Socket data receive callback function.
 *
 * @return cy_rslt_t                  : CY_RSLT_SUCCESS on success; secure sockets library error code otherwise.
 */
cy_rslt_t cy_awsport_network_create( NetworkContext_t *network_context,
                                     const cy_awsport_server_info_t *server_info,
                                     const cy_awsport_ssl_credentials_t *ssl_credentials,
                                     cy_awsport_callback_t *discon_cb,
                                     cy_awsport_callback_t *receive_cb );

/**
 * Sets up a TCP connection using the secure sockets API.
 *
 * @note
 *   1. This API should be called in pair with cy_awsport_network_create() API. The secure sockets lwIP port layer has a limitation that it will delete the connection instance created in cy_awsport_network_create() as part of the cy_awsport_network_disconnect() API. Therefore, cy_awsport_network_create() should be called before this API in pair.
 *   2. A timeout of 0 means infinite timeout.
 *
 * @param network_context [in]      : Client network context.
 * @param send_timeout [in]         : Timeout in milliseconds for transport send.
 * @param recv_timeout [in]         : Timeout in milliseconds for transport recv.
 *
 * @return cy_rslt_t                : CY_RSLT_SUCCESS on success; secure sockets library error code otherwise.
 */
cy_rslt_t cy_awsport_network_connect( NetworkContext_t *network_context,
                                      uint32_t send_timeout,
                                      uint32_t recv_timeout );

/**
 * Disconnects a TCP socket's connection using the secure sockets API.
 *
 * @note lwIP has a limitation: when disconnect is called, it will clear the netconn instance created and it will not allow any operation on that instance. Therefore, after disconnect, the connect API cannot be called directly. In addition, the secure socket must free its resources. Due to this lwIP limitation, this API and cy_awsport_network_delete() must be called for every disconnect.
 *
 * @param network_context [in]       : Client network context.
 *
 * @return cy_rslt_t                 : CY_RSLT_SUCCESS on success; secure sockets library error code otherwise.
 */
cy_rslt_t cy_awsport_network_disconnect( NetworkContext_t *network_context );

/**
 * Cleans up the context of secure socket connections using the secure sockets API.
 *
 * @note This API must be called after cy_awsport_network_disconnect() because of the secure sockets lwIP port layer limitation.
 *
 * @param network_context [in]       : Client network context.
 *
 * @return cy_rslt_t                 : CY_RSLT_SUCCESS on success; secure sockets library error code otherwise.
 */
cy_rslt_t cy_awsport_network_delete( NetworkContext_t *network_context );

/**
 * Receives the data over an established TLS session using the secure sockets API.
 *
 * This can be used as the transport-interface receive function for receiving data from the network.
 *
 * @param network_context [in]    : Client network context.
 * @param buffer [in, out]        : Buffer to receive the network data.
 * @param buffer_size [in]        : Size of the buffer to receive the data from the network.
 *
 * @return int32_t                : Number of bytes received if successful;
 *                                  Value (0) on receive timeout;
 *                                  Negative value (-1) on error.
 */
int32_t cy_awsport_network_receive( NetworkContext_t *network_context, void *buffer, size_t buffer_size );

/**
 * Sends data over an established TLS session using the secure sockets API.
 *
 * This can be used as the transport-interface send function to send data over the network.
 *
 * @param network_context [in]    : Client network context.
 * @param buffer [in]             : Buffer containing the bytes to send over the network stack.
 * @param buffer_size [in]        : Size of the send buffer.
 *
 * @return int32_t                : Number of bytes sent if successful;
 *                                  Value (0) on send timeout;
 *                                  Negative value (-1) on error.
 */

int32_t cy_awsport_network_send( NetworkContext_t *network_context, const void *buffer, size_t buffer_size );
/** \} group_aws_iot_device_sdk_port_functions */
/******************************************************************/

#endif // CY_TCPIP_PORT_SECURE_SOCKET_H_
/** \} group_aws_iot_device_sdk_port */
