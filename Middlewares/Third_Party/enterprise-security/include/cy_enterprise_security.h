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

/** @file
 *  Prototypes of functions for controlling enterprise security network
 */

#pragma once

#include "cy_enterprise_security_error.h"
#include "cy_type_defs.h"

/**
 * \defgroup enterprise_security Enterprise Security library
 * \brief The Enterprise Security library provides functions to join/leave an enterprise network.
 * \addtogroup enterprise_security
 * \{
 * \defgroup enterprise_security_defines Macros
 * \defgroup enterprise_security_enums Enumerated types
 * \defgroup enterprise_security_typedefs Typedefs
 * \defgroup enterprise_security_struct Structures
 * \defgroup enterprise_security_functions Functions
 * \defgroup enterprise_security_class Class Interface
 */


#ifdef __cplusplus
extern "C" {
#endif

#ifdef COMPONENT_CAT5
#define ENABLE_ENTPS_FEATURE
#endif /* COMPONENT_CAT5 */

/******************************************************
 *                    Constants
 ******************************************************/
/**
 * \addtogroup enterprise_security_defines
 * \{
 */

#define CY_ENTERPRISE_SECURITY_MAX_IDENTITY_LENGTH ( 32 )        /**< Maximum identity length supported */
#define CY_ENTERPRISE_SECURITY_MAX_PASSWORD_LENGTH ( 64 )        /**< Maximum password length supported */
#define CY_ENTERPRISE_SECURITY_MAX_SSID_LENGTH     ( 32 )        /**< Maximum SSID length supported */

/** \} enterprise_security_defines */

/******************************************************
 *                   Enumerations
 ******************************************************/
/**
 * \addtogroup enterprise_security_enums
 * \{
 */

/**
 * Enumeration of Enterprise Security auth modes
 */
typedef enum
{
    CY_ENTERPRISE_SECURITY_AUTH_TYPE_WPA_AES       = 0, /**<  WPA Enterprise Security with AES. Currently supported only on AnyCloud. */
    CY_ENTERPRISE_SECURITY_AUTH_TYPE_WPA_MIXED     = 1, /**<  WPA Enterprise Security with AES and TKIP. Currently supported only on AnyCloud. */
    CY_ENTERPRISE_SECURITY_AUTH_TYPE_WPA2_AES      = 2, /**<  WPA2 Enterprise Security with AES. */
    CY_ENTERPRISE_SECURITY_AUTH_TYPE_WPA2_MIXED    = 3, /**<  WPA2 Enterprise Security with AES and TKIP. */
    CY_ENTERPRISE_SECURITY_AUTH_TYPE_WPA2_FBT      = 4, /**<  WPA2 Enterprise Security with AES & FBT. Currently not supported, reserved for future. */
#ifdef ENABLE_ENTPS_FEATURE
    CY_ENTERPRISE_SECURITY_AUTH_TYPE_WPA3_AES      = 5, /**<  WPA3 Enterprise Security with AES GCM-256 */
    CY_ENTERPRISE_SECURITY_AUTH_TYPE_WPA3_192BIT   = 6, /**<  WPA3 192-BIT Enterprise Security with AES. */
    CY_ENTERPRISE_SECURITY_AUTH_TYPE_WPA3_AES_CCMP = 7, /**<  WPA3 Enterprise Security with AES CCM-128 */
#endif
    /* Add new entries above this line */
    CY_ENTERPRISE_SECURITY_AUTH_TYPE_DEFAULT = CY_ENTERPRISE_SECURITY_AUTH_TYPE_WPA2_MIXED, /** Default auth type. */
    CY_ENTERPRISE_SECURITY_AUTH_TYPE_UNKNOWN = -1   /**< Unknown auth type, used in error handling. Do not set this in input. */
} cy_enterprise_security_auth_t;

/** Enterprise Security Tunnel EAP Types
 *
 * @brief
 * Various Enterprise Security Tunnel EAP types
 */
typedef enum
{
    CY_ENTERPRISE_SECURITY_TUNNEL_TYPE_NONE        = 0, /**<  Invalid EAP type. */
    CY_ENTERPRISE_SECURITY_TUNNEL_TYPE_EAP         = 1, /**<  EAP as tunnel EAP type. */
    CY_ENTERPRISE_SECURITY_TUNNEL_TYPE_CHAP        = 2, /**<  CHAP as tunnel EAP type. Currently not supported, reserved for future. */
    CY_ENTERPRISE_SECURITY_TUNNEL_TYPE_MSCHAP      = 3, /**<  MSCHAP as tunnel EAP type. Currently not supported, reserved for future. */
    CY_ENTERPRISE_SECURITY_TUNNEL_TYPE_MSCHAPV2    = 4, /**<  MSCHAPv2 as tunnel EAP type. Currently not supported, reserved for future. */
    CY_ENTERPRISE_SECURITY_TUNNEL_TYPE_PAP         = 5  /**<  PAP as tunnel EAP type. Currently not supported, reserved for future. */
} cy_enterprise_security_tunnel_t;

/** Enterprise Security EAP Types
 *
 * @brief
 * Various Enterprise Security EAP authentication types
 */
typedef enum
{
    CY_ENTERPRISE_SECURITY_EAP_TYPE_NONE         = 0,   /**<  Invalid EAP type. */
    CY_ENTERPRISE_SECURITY_EAP_TYPE_IDENTITY     = 1,   /**<  IDENTITY type refer to RFC 3748. */
    CY_ENTERPRISE_SECURITY_EAP_TYPE_NOTIFICATION = 2,   /**<  NOTIFICATION type refer to RFC 3748. */
    CY_ENTERPRISE_SECURITY_EAP_TYPE_NAK          = 3,   /**<  Response only type refer to RFC 3748. */
    CY_ENTERPRISE_SECURITY_EAP_TYPE_MD5          = 4,   /**<  EAP-MD5 type refer to RFC 3748. */
    CY_ENTERPRISE_SECURITY_EAP_TYPE_OTP          = 5,   /**<  EAP-OTP type refer to RFC 3748. */
    CY_ENTERPRISE_SECURITY_EAP_TYPE_GTC          = 6,   /**<  EAP-GTC type refer to RFC 3748. */
    CY_ENTERPRISE_SECURITY_EAP_TYPE_TLS          = 13,  /**<  EAP-TLS type refer to RFC 2716. */
    CY_ENTERPRISE_SECURITY_EAP_TYPE_LEAP         = 17,  /**<  EAP-LEAP type and it is Cisco proprietary. */
    CY_ENTERPRISE_SECURITY_EAP_TYPE_SIM          = 18,  /**<  EAP-SIM type refer to draft-haverinen-pppext-eap-sim-12.txt. */
    CY_ENTERPRISE_SECURITY_EAP_TYPE_TTLS         = 21,  /**<  EAP-TTLS type refer to draft-ietf-pppext-eap-ttls-02.txt. */
    CY_ENTERPRISE_SECURITY_EAP_TYPE_AKA          = 23,  /**<  EAP-AKA type refer to draft-arkko-pppext-eap-aka-12.txt. */
    CY_ENTERPRISE_SECURITY_EAP_TYPE_PEAP         = 25,  /**<  PEAP type refer to draft-josefsson-pppext-eap-tls-eap-06.txt. */
    CY_ENTERPRISE_SECURITY_EAP_TYPE_MSCHAPV2     = 26,  /**<  MSCHAPv2 type refer to draft-kamath-pppext-eap-mschapv2-00.txt. */
    CY_ENTERPRISE_SECURITY_EAP_TYPE_TLV          = 33,  /**<  TLV type refer to draft-josefsson-pppext-eap-tls-eap-07.txt. */
    CY_ENTERPRISE_SECURITY_EAP_TYPE_FAST         = 43,  /**<  draft-cam-winget-eap-fast-00.txt. */
    CY_ENTERPRISE_SECURITY_EAP_TYPE_PAX          = 46,  /**<  draft-clancy-eap-pax-04.txt. */
    CY_ENTERPRISE_SECURITY_EAP_TYPE_EXPANDED_NAK = 253, /**<  RFC 3748. */
    CY_ENTERPRISE_SECURITY_EAP_TYPE_WPS          = 254, /**<  Wireless Simple Config. */
    CY_ENTERPRISE_SECURITY_EAP_TYPE_PSK          = 255  /**<  EXPERIMENTAL - type not yet allocated draft-bersani-eap-psk-09. */
} cy_enterprise_security_eap_type_t;

/** \} enterprise_security_enums */

/******************************************************
 *                 Type Definitions
 ******************************************************/
/**
 * \addtogroup enterprise_security_typedefs
 * \{
 */

/** Enterprise Security instance */
typedef void* cy_enterprise_security_t;

/** \} enterprise_security_typedefs */

/******************************************************
 *                    Structures
 ******************************************************/

/**
 * \addtogroup enterprise_security_struct
 * \{
 */

/**
 * Phase2 Parameters
 *
 * @brief
 * Enterprise Security EAP parameters for Phase2 authentication.
 */
typedef struct
{
    cy_enterprise_security_tunnel_t tunnel_auth_type;                    /**<  Tunnel authentication type. */
    cy_enterprise_security_eap_type_t inner_eap_type;                    /**<  Inner EAP type. */
    char inner_identity[ CY_ENTERPRISE_SECURITY_MAX_IDENTITY_LENGTH ];   /**<  Inner user identity. */
    char inner_password[ CY_ENTERPRISE_SECURITY_MAX_PASSWORD_LENGTH ];   /**<  Inner user password. */
} cy_enterprise_security_phase2_params_t;

/**
 * Enterprise Security parameters
 *
 * @brief
 * Enterprise Security configuration for a given network passed to \ref cy_enterprise_security_create
 */
typedef struct
{
    char                                    ssid[ CY_ENTERPRISE_SECURITY_MAX_SSID_LENGTH ];                     /**<  Wi-Fi SSID. */
    cy_enterprise_security_eap_type_t       eap_type;                                                           /**<  Authentication mechanism to be used. */
    char*                                   ca_cert;                                                            /**<  CA certificate in PEM format. */
    char*                                   client_cert;                                                        /**<  Client certificate in PEM format. */
    char*                                   client_key;                                                         /**<  Client private key in PEM format. */
    cy_enterprise_security_auth_t           auth_type;                                                          /**<  Security auth type used */
    char                                    outer_eap_identity[ CY_ENTERPRISE_SECURITY_MAX_IDENTITY_LENGTH ];   /**<  Outer EAP identity. */
    uint8_t                                 is_client_cert_required;                                            /**<  Deprecated */
    cy_enterprise_security_phase2_params_t  phase2;                                                             /**<  Phase2 authentication parameters. */
} cy_enterprise_security_parameters_t;

/** \} enterprise_security_struct */

/******************************************************
 *               Function Declarations
 ******************************************************/

/**
 * \addtogroup enterprise_security_functions
 * \{
 */

/**
 ******************************************************************************
 * Creates an Enterprise Security instance.
 *
 * This function initializes the Enterprise Security library with the given Enterprise Security parameters and returns an Enterprise Security instance to application.
 *
 * @param[out] handle         : Pointer to store the Enterprise Security instance handle allocated by this function on successful return.
 * @param[in]  ent_parameters : Pointer to \ref cy_enterprise_security_parameters_t structure,
 *                              initialized by the caller with the details required for establishing connection with the enterprise network.
 *
 * @return cy_rslt_t : CY_RSLT_SUCCESS - on success, an error code otherwise.
 *                     Error codes returned by this function are: \n
 *                     \ref CY_RSLT_ENTERPRISE_SECURITY_BADARG \n
 *                     \ref CY_RSLT_ENTERPRISE_SECURITY_NOMEM
 */
cy_rslt_t cy_enterprise_security_create(cy_enterprise_security_t *handle, cy_enterprise_security_parameters_t *ent_parameters);

/**
 ******************************************************************************
 * Deletes the given Enterprise Security instance and resources allocated by the \ref cy_enterprise_security_create function.
 *
 * @param[in] handle : Handle to Enterprise Security instance returned by \ref cy_enterprise_security_create function.
 *
 * @return cy_rslt_t : CY_RSLT_SUCCESS - on success, an error code otherwise.
 *                     Error codes returned by this function are: \n
 *                     \ref CY_RSLT_ENTERPRISE_SECURITY_BADARG
 */

cy_rslt_t cy_enterprise_security_delete(cy_enterprise_security_t *handle);

/**
 ******************************************************************************
 * Joins an enterprise security network (802.1x Access point)
 *
 * @param[in] handle : Handle to Enterprise Security instance returned by \ref cy_enterprise_security_create function.
 *
 * @return cy_rslt_t : CY_RSLT_SUCCESS - on success, an error code otherwise.
 *                     Error codes returned by this function are: \n
 *                     \ref CY_RSLT_ENTERPRISE_SECURITY_BADARG \n
 *                     \ref CY_RSLT_ENTERPRISE_SECURITY_ALREADY_CONNECTED \n
 *                     \ref CY_RSLT_ENTERPRISE_SECURITY_NOMEM \n
 *                     \ref CY_RSLT_ENTERPRISE_SECURITY_JOIN_ERROR \n
 *                     \ref CY_RSLT_ENTERPRISE_SECURITY_SUPPLICANT_ERROR
 */
cy_rslt_t cy_enterprise_security_join(cy_enterprise_security_t handle);

/**
 ******************************************************************************
 * Leaves an enterprise security network (802.1x Access point)
 *
 * @param[in] handle : Handle to Enterprise Security instance returned by \ref cy_enterprise_security_create function.
 *
 * @return cy_rslt_t : CY_RSLT_SUCCESS - on success, an error code otherwise.
 *                     Error codes returned by this function are: \n
 *                     \ref CY_RSLT_ENTERPRISE_SECURITY_BADARG \n
 *                     \ref CY_RSLT_ENTERPRISE_SECURITY_NOT_CONNECTED \n
 *                     \ref CY_RSLT_ENTERPRISE_SECURITY_LEAVE_ERROR \n
 *                     \ref CY_RSLT_ENTERPRISE_SECURITY_SUPPLICANT_ERROR
 */
cy_rslt_t cy_enterprise_security_leave(cy_enterprise_security_t handle);

/** \} enterprise_security_functions */

#ifdef __cplusplus
} /* extern "C" */
#endif

/** \} enterprise_security */

