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

#include <nx_secure_tls_api.h>
#include <stdbool.h>
/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Typedefs
 ******************************************************/

/* IMPLEMENTATION NOTE: Core supplicant implementation should not access any of the structure members defined in this file */

typedef NX_SECURE_TLS_SESSION cy_tls_session_t;
typedef NX_SECURE_X509_CERT   cy_x509_crt_t;
typedef NX_SECURE_TLS_SESSION cy_tls_workspace_t;

typedef struct
{
    NX_SECURE_X509_CERT     certificate;
    uint8_t                 *certificate_der;
    uint8_t                 *private_key_der;
    uint8_t                 is_client_auth;
} cy_tls_identity_t;

typedef struct
{
    void                    *usr_data;
    char                    *peer_cn;
    cy_tls_session_t        *session;
    cy_tls_workspace_t      context;
    cy_tls_identity_t       *identity;
    cy_x509_crt_t           *root_ca_certificates;
    uint8_t                 *root_ca_cert_der;
    int8_t                  *tls_metadata;
    uint8_t                 *tls_packet_buffer;
    uint8_t                 *certificate_buffer;
    int32_t                 resume;
    bool                    tls_handshake_successful;
    bool                    tls_v13;
    uint8_t                 expected_pkt_count;
} cy_tls_context_t;

#ifdef __cplusplus
} /*extern "C" */
#endif
