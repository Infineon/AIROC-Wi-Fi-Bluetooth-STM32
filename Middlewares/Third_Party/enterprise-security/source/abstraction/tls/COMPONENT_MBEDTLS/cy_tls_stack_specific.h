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

#include "ssl.h"
#include "ssl_internal.h"
#include "ctr_drbg.h"
#include "entropy.h"
#include "cipher.h"
#include "mbedtls/md4.h"
#include "mbedtls/sha1.h"
#include "mbedtls/des.h"
#include "mbedtls/version.h"

/******************************************************
 *                      Macros
 ******************************************************/

#define TLS_RANDOM_BYTES                         (64)
#define TLS_SERVER_RANDOM_BYTES                  (32)
#define TLS_CLIENT_RANDOM_BYTES                  (32)
#define TLS_MASTER_SECRET_BYTES                  (48)
#define TLS_SERVER_VERSION_LEN                   (2)
#define BUFFER_SIZE                              (1600)

#define MBEDTLS_VERSION_WITH_PRF_SUPPORT         (0x02130000) /* mbedTLS version 2.19.0 where PRF support is added. */

/******************************************************
 *                    Typedefs
 ******************************************************/

/* IMPLEMENTATION NOTE: Core supplicant implementation should not access any of the structure members defined in this file */
typedef struct mbedtls_ssl_context cy_tls_workspace_t;
typedef struct mbedtls_ssl_session cy_tls_session_t;
typedef struct mbedtls_x509_crt cy_x509_crt_t;
typedef struct mbedtls_pk_context cy_pk_context_t;
typedef struct mbedtls_entropy_context cy_entropy_context_t;
typedef struct mbedtls_ctr_drbg_context cy_ctr_drbg_context_t;
typedef struct mbedtls_ssl_config cy_ssl_config_t;

typedef struct
{
    cy_pk_context_t private_key;
    cy_x509_crt_t certificate;
    uint8_t is_client_auth;
} cy_tls_identity_t;

typedef struct eap_tls_keys
{
    uint8_t master_secret[TLS_MASTER_SECRET_BYTES];
    uint8_t randbytes[TLS_RANDOM_BYTES];
#if (MBEDTLS_VERSION_NUMBER > MBEDTLS_VERSION_WITH_PRF_SUPPORT)
    mbedtls_tls_prf_types tls_prf_type;
#else
    int32_t  (*supplicant_tls_prf)(const uint8_t *, size_t, const int8_t *,
                    const uint8_t *, size_t,
                    uint8_t *, size_t);
#endif
    int32_t resume;
} eap_tls_keys;


typedef struct
{
    void                    *usr_data;
    char                    *peer_cn;
    cy_tls_session_t        *session; /* This session pointer is only used to resume connection for client, If application/library wants to resume connection it needs to pass pointer of previous stored session */
    cy_tls_workspace_t      context;
    cy_tls_identity_t       *identity;
    cy_x509_crt_t           *root_ca_certificates; /* Context specific root-ca-chain */
    cy_entropy_context_t    entropy;
    cy_ctr_drbg_context_t   ctr_drbg;
    cy_ssl_config_t         conf;
    int                     resume;
    eap_tls_keys            eap_tls_keying;
    uint8_t                 tls_v13;

    /* Book-keeping information used for in-house use of TLS-security library */
    uint8_t                 buffered_data[BUFFER_SIZE];
    uint8_t                 *buffer_to_use;
    uint32_t                remaining_bytes;
    uint32_t                bytes_consumed;
    uint32_t                total_bytes;
} cy_tls_context_t;


#ifdef __cplusplus
} /*extern "C" */
#endif
