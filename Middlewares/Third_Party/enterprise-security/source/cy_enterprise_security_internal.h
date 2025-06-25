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

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include  "cy_enterprise_security_error.h"
#include  "cy_type_defs.h"
#include  "cy_supplicant_core_constants.h"
#include  "cy_supplicant_structures.h"
#include  "cy_enterprise_security.h"
#include  "whd.h"

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
 *                   Structures
 ******************************************************/
/** TLS security parameters
 *
 * @brief supplicant TLS security parameters
 * Root certificate and its length
 * Client certificate and its length
 * Client private key and its length
 */
typedef struct cy_supplicant_security_s
{
    char*      ca_cert;     /**<  CA certificate */
    uint32_t   ca_cert_len; /**<  CA certificate length */
    char*      cert;        /**<  Client certificate in PEM format */
    uint32_t   cert_len;    /**<  Client certificate length */
    char*      key;         /**<  Client private key */
    uint32_t   key_len;     /**<  Client private key length */
} cy_supplicant_security_t;

/** Inner identity credentials
 *
 * @brief supplicant inner identity credentials to used
 *
 * username and its length
 * password and its length
 */
typedef struct cy_supplicant_inner_identity_s
{
    char                    identity[ CY_ENTERPRISE_SECURITY_MAX_IDENTITY_LENGTH ];  /**<  Username */
    uint8_t                 identity_length;                                         /**<  Username length */
    /* in Windows password is UNICODE */
    char                    password[ CY_ENTERPRISE_SECURITY_MAX_PASSWORD_LENGTH ];  /**<  Password */
    uint8_t                 password_length;                                         /**<  Password length */
} cy_supplicant_inner_identity_t;

/** Phase2 PEAP configuration structure
 *
 * @brief supplicant phase2 PEAP configuration structure
 *
 */
typedef struct cy_supplicant_peap_s
{
    cy_supplicant_inner_identity_t      inner_identity;  /**<  Inner identity credentials like username and password */

} cy_supplicant_peap_t;

/** Phase2 EAP-TTLS configuration structure
 *
 * @brief supplicant phase2 EAP TTLS configuration structure
 *
 */
typedef struct cy_supplicant_eap_ttls_s
{
    cy_enterprise_security_eap_type_t   inner_eap_type;           /**<  Inner EAP type */
    cy_supplicant_inner_identity_t      inner_identity;           /**<  Inner identity credentials like username and password */
} cy_supplicant_eap_ttls_t;


/** Phase2 configuration structure
 *
 * @brief supplicant phase2 configuration structure
 *
 */
typedef struct cy_supplicant_phase2_config_s
{
    cy_enterprise_security_tunnel_t tunnel_auth_type; /**<  Inner authentication protocol used (phase2) */
    union
    {
        cy_supplicant_peap_t peap;                    /**<  Inner identity information for PEAP */
        cy_supplicant_eap_ttls_t eap_ttls;            /**<  Inner identity information for EAP_TTLS */
    } tunnel_protocol;                                /**<  Tunnel protocol */
} cy_supplicant_phase2_config_t;

/** Supplicant workspace information.
 *
 * @brief supplicant internal core context
 * supplicant workspace information.
 * @note for internal use
 */
typedef struct cy_supplicant_core_s
{
    supplicant_workspace_t*           supplicant_workspace;  /**<  Supplicant workspace context */
} cy_supplicant_core_t;

/** Supplicant instance
 *
 * @brief supplicant instance
 * @note For internal use
 */
typedef struct cy_supplicant_instance_s
{
    char                                ssid[ 64 ];                                                      /**<  WIFI SSID */
    cy_enterprise_security_eap_type_t   eap_type;                                                        /**<  Authentication mechanism want to use */
    cy_supplicant_security_t            tls_security;                                                    /**<  TLS security parameters like CA certificate, client certificate and client private key to be used */
    cy_enterprise_security_auth_t       auth_type;                                                       /**<  Security auth type used */
    char                                outer_eap_identity[CY_ENTERPRISE_SECURITY_MAX_IDENTITY_LENGTH];  /**<  Outer EAP identity */
    uint8_t                             outer_eap_identity_length;                                       /**<  Outer EAP identity length  */
    cy_supplicant_phase2_config_t       phase2_config;                                                   /**<  Phase2 configuration parmaters to be filled. Used only for EAP_TTLS & PEAP */

    /* For internal management */
    cy_tls_context_t*                   tls_context;                                                     /**<  TLS context */
    cy_supplicant_core_t                supplicant_core;                                                 /**<  Supplicant core (for book keeping) */
    cy_tls_identity_t*                  tls_identity;                                                    /**<  Identity for the secure connection */
    whd_interface_t                     interface;                                                       /**<  WHD interface */
    cy_tls_session_t                    saved_session;                                                   /**<  Used during session resumption case */
} cy_supplicant_instance_t;

/******************************************************
 *               Function Declarations
 ******************************************************/

/** Allocates memory for members of Supplicant instance.
 *
 * @param[in] supplicant_instance  : pointer to Supplicant instance should be passed by user.
 *
 * @return cy_rslt_t  : CY_RSLT_SUCCESS - on success, [Enterprise Security-specific error codes](./cy_enterprise_security_error.h) otherwise.
 */
cy_rslt_t cy_supplicant_alloc( cy_supplicant_instance_t *supplicant_instance );

/** Frees the allocated memory for members of Supplicant instance.
 *
 * @param[in] supplicant_instance  : pointer to Supplicant instance should be passed by user.
 *
 * @return cy_rslt_t  : CY_RSLT_SUCCESS - on success, [Enterprise Security-specific error codes](./cy_enterprise_security_error.h) otherwise.
 */
cy_rslt_t cy_supplicant_free( cy_supplicant_instance_t *supplicant_instance );

/** Perform WiFi join to an enterprise network
 *
 * @param[in] supplicant_instance  : pointer to Supplicant instance should be passed by user.
 *
 * @return cy_rslt_t  : CY_RSLT_SUCCESS - on success, [Enterprise Security-specific error codes](./cy_enterprise_security_error.h) otherwise.
 */
cy_rslt_t cy_join_ent( cy_supplicant_instance_t *supplicant_instance );

/** Perform WiFi leave from an enterprise network
 *
 * @param[in] supplicant_instance  : pointer to an Supplicant instance should be passed by user.
 *
 * @return cy_rslt_t  : CY_RSLT_SUCCESS - on success, [Enterprise Security-specific error codes](./cy_enterprise_security_error.h) otherwise.
 */
cy_rslt_t cy_leave_ent( cy_supplicant_instance_t *supplicant_instance );

#ifdef __cplusplus
} /*extern "C" */
#endif
