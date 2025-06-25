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

#include "cy_enterprise_security_log.h"
#include "cy_tls_abstraction.h"
#include "cy_supplicant_structures.h"
#include "cy_supplicant_process_et.h"

/******************************************************
 *                      Macros
 ******************************************************/

#if defined ( MBEDTLS_SSL_PROTO_TLS1 )
#define MIN_TLS_VERSION MBEDTLS_SSL_MINOR_VERSION_1
#elif defined ( MBEDTLS_SSL_PROTO_TLS1_1 )
#define MIN_TLS_VERSION MBEDTLS_SSL_MINOR_VERSION_2
#else
#define MIN_TLS_VERSION MBEDTLS_SSL_MINOR_VERSION_3
#endif

#if defined( MBEDTLS_SSL_PROTO_TLS1_2 )
#define MAX_TLS_VERSION MBEDTLS_SSL_MINOR_VERSION_3
#elif defined (MBEDTLS_SSL_PROTO_TLS1_1)
#define MAX_TLS_VERSION MBEDTLS_SSL_MINOR_VERSION_2
#else
#define MAX_TLS_VERSION MBEDTLS_SSL_MINOR_VERSION_1
#endif
/******************************************************
 *                    Function prototypes
 ******************************************************/
#ifdef ENABLE_TLS_WRAPPER_DUMP
static void tls_dump_bytes ( const uint8_t* bptr, uint32_t len );
#else
#define tls_dump_bytes( bptr, len )
#endif

extern tls_agent_packet_t* supplicant_receive_eap_tls_packet( void* workspace_in, uint32_t* length, uint32_t timeout );

struct options
{
    int         min_version;   /* minimum protocol version accepted */
    int         max_version;   /* maximum protocol version accepted */
    int         auth_mode;     /* verify mode for connection        */
    const char  *curves;       /* list of supported elliptic curves */
    const char  *alpn_string;  /* ALPN supported protocols          */
    int         transport;     /* TLS or DTLS?                      */
} opt;

#if defined(MBEDTLS_SSL_EXPORT_KEYS)
#if (MBEDTLS_VERSION_NUMBER >= MBEDTLS_VERSION_WITH_PRF_SUPPORT)
static int eap_tls_key_derivation( void *p_expkey, const unsigned char *ms, const unsigned char *kb, size_t maclen, size_t keylen, size_t ivlen, const unsigned char client_random[32], const unsigned char server_random[32], mbedtls_tls_prf_types tls_prf_type )
{
    cy_tls_context_t* tls_context = p_expkey;
    eap_tls_keys *keys = (eap_tls_keys *)&tls_context->eap_tls_keying;
    ( ( void ) kb );
    ( ( void ) maclen );
    ( ( void ) keylen );
    ( ( void ) ivlen );
    memcpy( keys->master_secret, ms, sizeof( keys->master_secret ) );
    memcpy( keys->randbytes, client_random, 32 );
    memcpy( keys->randbytes + 32, server_random, 32 );
    keys->tls_prf_type = tls_prf_type;
    tls_context->resume = tls_context->context.handshake->resume;
    return( 0 );
}
#else
static int eap_tls_key_derivation( void *p_expkey, const unsigned char *ms, const unsigned char *kb, size_t maclen, size_t keylen,  size_t ivlen )
{
    cy_tls_context_t* tls_context = p_expkey;
    eap_tls_keys *keys = (eap_tls_keys *)&tls_context->eap_tls_keying;
    ( ( void ) kb );
    ( ( void ) maclen );
    ( ( void ) keylen );
    ( ( void ) ivlen );
/* random bytes are collected in eap_ssl_receive_packet function */

    memcpy( keys->master_secret, ms, sizeof( keys->master_secret ) );
    tls_context->resume = tls_context->context.handshake->resume;
    keys->supplicant_tls_prf = tls_context->context.handshake->tls_prf;
    return( 0 );
}
#endif /* MBEDTLS_VERSION_NUMBER >= MBEDTLS_VERSION_WITH_PRF_SUPPORT */
#endif /* MBEDTLS_SSL_EXPORT_KEYS */

#ifdef ENABLE_TLS_WRAPPER_DUMP
static void tls_dump_bytes( const uint8_t* bptr, uint32_t len )
{
    uint32_t i = 0;

    for( i = 0; i < len; )
    {
        if( ( i & 0x0f ) == 0 )
        {
            CY_SUPPLICANT_TLS_DUMP_BYTES( ( "\n" ) );
        }
        else if( ( i & 0x07 ) == 0 )
        {
            CY_SUPPLICANT_TLS_DUMP_BYTES( (" ") );
        }
        CY_SUPPLICANT_TLS_DUMP_BYTES( ( "%02x ", bptr[i++] ) );
    }
    CY_SUPPLICANT_TLS_DUMP_BYTES( ( "\n" ) );
}
#endif /*ENABLE_TLS_WRAPPER_DUMP*/

#if defined(MBEDTLS_DEBUG_C)
static void mbedtls_debug( void *ctx, int level, const char *file, int line, const char *str )
{
    ( (void) level );
    TLS_WRAPPER_DEBUG(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s:%04d: %s", file, line, str);
}
#endif /* MBEDTLS_DEBUG_C */

int eap_ssl_receive_packet( void *ctx, unsigned char *buf, size_t len )
{
    supplicant_workspace_t* workspace = (supplicant_workspace_t*) ctx;
    uint16_t temp_length = 0;
    uint16_t temp_available_length = 0;
    uint8_t* data = NULL;
    unsigned char *temp_buf = buf;
    tls_agent_packet_t* packet = NULL;
    int requested_bytes = len;
    uint32_t length = 0;

    while( requested_bytes > 0 )
    {
        if( workspace->tls_context->remaining_bytes > 0)
        {
            int x = workspace->tls_context->remaining_bytes;
            memcpy( temp_buf, workspace->tls_context->buffer_to_use, SUPPLICANT_DEFS_MIN( x, requested_bytes ) );
            temp_buf = temp_buf + SUPPLICANT_DEFS_MIN( x, requested_bytes );
            workspace->tls_context->buffer_to_use = workspace->tls_context->buffer_to_use + SUPPLICANT_DEFS_MIN( x, requested_bytes );
            workspace->tls_context->remaining_bytes = workspace->tls_context->remaining_bytes - SUPPLICANT_DEFS_MIN( x, requested_bytes );
            workspace->tls_context->bytes_consumed =  workspace->tls_context->bytes_consumed +  SUPPLICANT_DEFS_MIN( x, requested_bytes );
            requested_bytes = requested_bytes -  SUPPLICANT_DEFS_MIN( x, requested_bytes );

        }
        else
        {
            packet = supplicant_receive_eap_tls_packet( workspace, &length, SUPPLICANT_TIMEOUT );
            if(packet == NULL)
            {
                TLS_WRAPPER_DEBUG(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "error eap_ssl_receive_packet : MBEDTLS_ERR_SSL_TIMEOUT \n");
                return CY_RSLT_ENTERPRISE_SECURITY_TIMEOUT;
            }

            supplicant_host_get_tls_data( workspace, (supplicant_packet_t) packet, 0 , &data, &temp_length, &temp_available_length );
            workspace->tls_context->buffer_to_use = workspace->tls_context->buffered_data;
            memcpy( workspace->tls_context->buffer_to_use, data, length );
            workspace->tls_context->bytes_consumed = 0;
            workspace->tls_context->remaining_bytes = length;
            workspace->tls_context->total_bytes = length;

            supplicant_host_free_packet( workspace->interface->whd_driver,packet );
        }
    }

#if !(MBEDTLS_VERSION_NUMBER >= MBEDTLS_VERSION_WITH_PRF_SUPPORT)
    /* Fetch random bytes for mppe key generation */
    if( workspace->tls_context->context.state == MBEDTLS_SSL_SERVER_HELLO )
    {
        size_t ssl_hs_hdr_len = mbedtls_ssl_hs_hdr_len( &workspace->tls_context->context );
        /* The packet length should be greater than (SSL_HS_HEADER_LEN + TLS_SERVER_VERSION_LEN + SERVER_RANDBYTE_LEN ) */
        if( len >= ssl_hs_hdr_len + TLS_SERVER_VERSION_LEN + TLS_SERVER_RANDOM_BYTES )
        {
            temp_buf = buf;
            temp_buf += mbedtls_ssl_hs_hdr_len( &workspace->tls_context->context );
            /* copy client random bytes */
            memcpy( workspace->tls_context->eap_tls_keying.randbytes, workspace->tls_context->context.handshake->randbytes, TLS_CLIENT_RANDOM_BYTES );
            /* copy server random bytes */
            memcpy( workspace->tls_context->eap_tls_keying.randbytes + TLS_CLIENT_RANDOM_BYTES, temp_buf + TLS_SERVER_VERSION_LEN, TLS_SERVER_RANDOM_BYTES );
        }
        else
        {
            TLS_WRAPPER_DEBUG(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Packet length is too short, cannot fetch server random bytes\n");
        }
    }
#endif /* !(MBEDTLS_VERSION_NUMBER >= MBEDTLS_VERSION_WITH_PRF_SUPPORT) */

    return ((int) len);
}

static cy_rslt_t supplicant_host_send_eap_tls_fragments( supplicant_workspace_t* workspace, uint8_t* buffer, uint16_t length )
{
    //CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_INFO, "TLS handshake state: %u\r\n", (unsigned int) workspace->tls_context->context.state);

    /* Send the alert message to peer */
    if(workspace->tls_context->context.out_msgtype == MBEDTLS_SSL_MSG_ALERT)
    {
        memset( workspace->buffer, 0, workspace->buffer_size );

        /* Point the buffer pointer at the start of the buffer */
        workspace->data_start   = workspace->buffer;

        memcpy(workspace->data_start, buffer, length);

        /* Point data at the start of the buffer */
        workspace->data_end = workspace->data_start + length;
    }
    else
    {
        if ( ( workspace->tls_context->context.state == MBEDTLS_SSL_SERVER_HELLO) ||
                ( ( workspace->have_packet == 0 ) && ( workspace->tls_context->context.state == MBEDTLS_SSL_CLIENT_CERTIFICATE ) ) ||
                ( ( workspace->have_packet == 0 ) && ( workspace->tls_context->context.state == MBEDTLS_SSL_CERTIFICATE_VERIFY ) ) ||
                ( ( workspace->have_packet == 0 ) && ( workspace->tls_context->context.state == MBEDTLS_SSL_CLIENT_KEY_EXCHANGE ) ) ||
                ( ( workspace->tls_context->context.state == MBEDTLS_SSL_CLIENT_FINISHED ) &&  ( workspace->tls_context->resume == 1 ) ) ) /* Session resumption case */
        {

            /* Mark that we already started a record, so that a
             * Certificate verify won't create it's own if the certificate is already sent.
             */
            workspace->have_packet = 1;
            memset( workspace->buffer, 0, workspace->buffer_size );

            /* Point the buffer pointer at the start of the buffer */

            workspace->data_start   = workspace->buffer;

            memcpy(workspace->data_start, buffer, length);

            /* Point data at the start of the buffer */
            workspace->data_end = workspace->data_start + length;

            /* If this is the client certificate exchange or change cipher spec with session resumption then return so the TLS engine provides the rest of the handshake packets */
            if ( ( ( workspace->tls_context->context.state == MBEDTLS_SSL_CLIENT_CERTIFICATE ) ) ||
                    ( ( workspace->tls_context->context.state == MBEDTLS_SSL_CERTIFICATE_VERIFY ) ) ||
                    ( ( workspace->tls_context->context.state == MBEDTLS_SSL_CLIENT_KEY_EXCHANGE ) ) ||
                    ( ( workspace->tls_context->context.state == MBEDTLS_SSL_CLIENT_FINISHED ) && ( workspace->tls_context->resume == 1 ) ) )
            {
                return CY_RSLT_SUCCESS;
            }

        }
        else if ( ( workspace->tls_context->context.state > MBEDTLS_SSL_CLIENT_CERTIFICATE )  &&  ( workspace->tls_context->context.state < MBEDTLS_SSL_SERVER_CHANGE_CIPHER_SPEC ) )
        {
            /* Append the handshake message and return for more unless it's the last message */
            memcpy(workspace->data_end, buffer, length);
            workspace->data_end += length;
            return CY_RSLT_SUCCESS;
        }
        else if ( ( workspace->tls_context->context.state == MBEDTLS_SSL_SERVER_CHANGE_CIPHER_SPEC ) ||
                ( ( ( workspace->tls_context->context.state == MBEDTLS_SSL_FLUSH_BUFFERS) || ( workspace->tls_context->context.state == MBEDTLS_SSL_HANDSHAKE_WRAPUP) )&&  ( workspace->tls_context->resume == 1 ) ) ) /* Session resumption case */
        {
            memcpy(workspace->data_end, buffer, length);
            workspace->data_end += length;

            if ( workspace->tls_context->resume == 1 )
            {
#if (MBEDTLS_VERSION_NUMBER >= MBEDTLS_VERSION_WITH_PRF_SUPPORT)
                workspace->cipher_flags = workspace->tls_context->context.handshake->ciphersuite_info->flags;
#else
                workspace->cipher_flags = workspace->tls_context->context.transform_out->ciphersuite_info->flags;
#endif
            }
        }
        else
        {
            TLS_WRAPPER_DEBUG(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Unexpected data to be fragmented\r\n");
            tls_dump_bytes( buffer, length );
            return CY_RSLT_SUCCESS;
        }
    }
    return (supplicant_fragment_and_queue_eap_response(workspace));
}

int eap_ssl_flush_output( void* context, const uint8_t* buffer, size_t length )
{
    supplicant_workspace_t* supplicant = (supplicant_workspace_t*) context;
    cy_rslt_t result = CY_RSLT_SUCCESS;
    result = supplicant_host_send_eap_tls_fragments((supplicant_workspace_t*) supplicant, (uint8_t*) buffer, length);
    if(result != CY_RSLT_SUCCESS)
    {
        return 0;
    }
    return length;
}

/*
 * @func  : cy_tls_session_cleanup
 *
 * @brief : Cleanup existing tls session.
 */
void cy_tls_session_cleanup( cy_tls_context_t* tls_context )
{
    /* No Cleanup required */
}

cy_rslt_t cy_tls_generic_start_tls_with_ciphers( cy_tls_context_t* tls_context, void* referee, cy_tls_certificate_verification_t verification )
{
    mbedtls_ssl_context *ssl = &tls_context->context;
    mbedtls_ssl_config  *conf = (mbedtls_ssl_config  *)&tls_context->conf;
    int ret = 0;
    cy_rslt_t result = CY_RSLT_SUCCESS;

    const char *pers = "supplicant_client";

    /* Cleaning up the previous allocated items */
    mbedtls_entropy_free(&tls_context->entropy );
    mbedtls_ctr_drbg_free( &tls_context->ctr_drbg );
    mbedtls_ssl_config_free( conf );
    mbedtls_ssl_free(ssl);

    /*
     * Make sure memory references are valid.
     */
    mbedtls_ssl_init( ssl );

    mbedtls_ssl_config_init( conf );

    mbedtls_ctr_drbg_init( &tls_context->ctr_drbg );

    /*
     * Initialize the Entropy function
     */
    TLS_WRAPPER_DEBUG(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "\n  . Seeding the random number generator...");

    mbedtls_entropy_init( &tls_context->entropy );
    if( ( ret = mbedtls_ctr_drbg_seed( &tls_context->ctr_drbg, mbedtls_entropy_func,
            &tls_context->entropy, (const unsigned char *) pers,
            strlen( pers ) ) ) != 0 )
    {
        TLS_WRAPPER_DEBUG(CYLF_MIDDLEWARE, CY_LOG_ERR, " failed\n  ! mbedtls_ctr_drbg_seed returned -0x%x\n", -ret);
        return CY_RSLT_MODULE_TLS_ERROR;
    }

    TLS_WRAPPER_DEBUG(CYLF_MIDDLEWARE, CY_LOG_DEBUG, " ok\n");

    /*
     * Setup stuff
     */
    TLS_WRAPPER_DEBUG(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "  . Setting up the SSL/TLS structure...");

    if( ( ret = mbedtls_ssl_config_defaults( conf, MBEDTLS_SSL_IS_CLIENT, MBEDTLS_SSL_TRANSPORT_STREAM, MBEDTLS_SSL_PRESET_DEFAULT ) ) != 0 )
    {
        TLS_WRAPPER_DEBUG(CYLF_MIDDLEWARE, CY_LOG_ERR, " failed\n  ! mbedtls_ssl_config_defaults returned -0x%x\n\n", -ret);
        return CY_RSLT_MODULE_TLS_ERROR;
    }

    mbedtls_ssl_conf_rng( conf, mbedtls_ctr_drbg_random, &tls_context->ctr_drbg );

#if defined(MBEDTLS_DEBUG_C)
    mbedtls_ssl_conf_dbg( conf, mbedtls_debug, stdout );
#endif /* MBEDTLS_DEBUG_C */
    mbedtls_ssl_conf_read_timeout( conf, 0);

    if (tls_context->root_ca_certificates != NULL)
    {
        mbedtls_ssl_conf_ca_chain(conf, tls_context->root_ca_certificates, NULL);
        mbedtls_ssl_conf_authmode(conf, MBEDTLS_SSL_VERIFY_REQUIRED);
    } else
    {
        mbedtls_ssl_conf_authmode(conf, MBEDTLS_SSL_VERIFY_NONE);
    }

    if (tls_context->identity != NULL)
    {
        if(tls_context->identity->is_client_auth == 1)
        {
            if ((ret = mbedtls_ssl_conf_own_cert(conf,&tls_context->identity->certificate, &tls_context->identity->private_key)) != 0)
            {
                result = CY_RSLT_MODULE_TLS_ERROR;
                return result;
            }
        }
        else
        {
            if ((ret = mbedtls_ssl_conf_own_cert(conf, NULL, NULL)) != 0)
            {
                result = CY_RSLT_MODULE_TLS_ERROR;
                return result;
            }
        }
    }
    else
    {
        if ((ret = mbedtls_ssl_conf_own_cert(conf, NULL, NULL)) != 0)
        {
            result = CY_RSLT_MODULE_TLS_ERROR;
            return result;
        }
    }

    /* setting minimum TLS version supported as TLSv1.0 */
    mbedtls_ssl_conf_min_version( conf, MBEDTLS_SSL_MAJOR_VERSION_3, MIN_TLS_VERSION );

    /* setting Maximum TLS version supported as TLSv1.2 */
    mbedtls_ssl_conf_max_version( conf, MBEDTLS_SSL_MAJOR_VERSION_3, MAX_TLS_VERSION );

    if( ( ret = mbedtls_ssl_setup( ssl, conf ) ) != 0 )
    {
        TLS_WRAPPER_DEBUG(CYLF_MIDDLEWARE, CY_LOG_ERR, " failed\n  ! mbedtls_ssl_setup returned -0x%x\n\n", -ret);
        return CY_RSLT_MODULE_TLS_ERROR;
    }

    /* If session is already set then assuming that this is resume connection, It is only applicable for client */
    if ( tls_context->session != NULL)
    {
        if ( tls_context->session->id_len > 0 )
        {
            mbedtls_ssl_set_session( &tls_context->context, tls_context->session );
        }
    }

#if defined(MBEDTLS_SSL_EXPORT_KEYS)
#if (MBEDTLS_VERSION_NUMBER >= MBEDTLS_VERSION_WITH_PRF_SUPPORT)
    mbedtls_ssl_conf_export_keys_ext_cb(conf, eap_tls_key_derivation, tls_context);
#else
    mbedtls_ssl_conf_export_keys_cb(conf, eap_tls_key_derivation, tls_context);
#endif
#endif

    mbedtls_ssl_set_bio( ssl, referee, eap_ssl_flush_output, eap_ssl_receive_packet, NULL );

    tls_context->buffer_to_use = tls_context->buffered_data;

    /*
     * Handshake
     */
    TLS_WRAPPER_DEBUG(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "  . Performing the SSL/TLS handshake...");

    while( ( ret = mbedtls_ssl_handshake( ssl ) ) != 0 )
    {
        if( ret != MBEDTLS_ERR_SSL_WANT_READ && ret != MBEDTLS_ERR_SSL_WANT_WRITE && ret != MBEDTLS_ERR_SSL_CRYPTO_IN_PROGRESS )
        {
            TLS_WRAPPER_DEBUG(CYLF_MIDDLEWARE, CY_LOG_ERR, " failed\n  ! mbedtls_ssl_handshake returned -0x%x\n", -ret);

            mbedtls_entropy_free(&tls_context->entropy );
            mbedtls_ctr_drbg_free( &tls_context->ctr_drbg );
            mbedtls_ssl_config_free( conf );
            mbedtls_ssl_free(ssl);
            return CY_RSLT_MODULE_TLS_HANDSHAKE_FAILURE;
        }
    }
    if( ( ret = mbedtls_ssl_get_session( ssl,tls_context->session ) ) != 0 )
    {
        TLS_WRAPPER_DEBUG(CYLF_MIDDLEWARE, CY_LOG_ERR, "\r\n failed\n  ! mbedtls_ssl_get_session returned -0x%x\n\n", -ret);
    }

    TLS_WRAPPER_DEBUG(CYLF_MIDDLEWARE, CY_LOG_DEBUG, " ok\n");

    return CY_RSLT_SUCCESS;
}

#if (MBEDTLS_VERSION_NUMBER >= MBEDTLS_VERSION_WITH_PRF_SUPPORT)
cy_rslt_t cy_tls_get_mppe_key(cy_tls_context_t *tls_context, const char* label, uint8_t *context, uint16_t context_len, uint8_t* mppe_keys, int size)
{
    (void)context;
    (void)context_len;
#if defined(MBEDTLS_SSL_EXPORT_KEYS)
    if ( mbedtls_ssl_tls_prf(tls_context->eap_tls_keying.tls_prf_type,
            tls_context->eap_tls_keying.master_secret,
            sizeof(tls_context->eap_tls_keying.master_secret), label,
            tls_context->eap_tls_keying.randbytes, sizeof(tls_context->eap_tls_keying.randbytes),
            mppe_keys, size) != 0)
    {
        TLS_WRAPPER_DEBUG(CYLF_MIDDLEWARE, CY_LOG_ERR, " failed\n  ! mbedtls_ssl_tls_prf returned\r\n");
        return CY_RSLT_MODULE_TLS_ERROR;
    }

    TLS_WRAPPER_DEBUG(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "EAP-TLS key material is: \n");
    tls_dump_bytes(mppe_keys,size);
    TLS_WRAPPER_DEBUG(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "\n");
    return CY_RSLT_SUCCESS;
#else
    return CY_RSLT_MODULE_TLS_UNSUPPORTED;
#endif /* MBEDTLS_SSL_EXPORT_KEYS */
}
#else
cy_rslt_t cy_tls_get_mppe_key(cy_tls_context_t *tls_context, const char* label, uint8_t *context, uint16_t context_len, uint8_t* mppe_keys, int size)
{
    (void)context;
    (void)context_len;
#if defined(MBEDTLS_SSL_EXPORT_KEYS)
    if ( tls_context->eap_tls_keying.supplicant_tls_prf( tls_context->eap_tls_keying.master_secret, sizeof(tls_context->eap_tls_keying.master_secret), label, tls_context->eap_tls_keying.randbytes,
            sizeof(tls_context->eap_tls_keying.randbytes), mppe_keys, size ) != 0 )
    {
        TLS_WRAPPER_DEBUG(CYLF_MIDDLEWARE, CY_LOG_ERR, " failed\n  ! mbedtls_ssl_tls_prf returned\r\n");
        return CY_RSLT_MODULE_TLS_ERROR;
    }

    TLS_WRAPPER_DEBUG(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "EAP-TLS key material is: \n");
    tls_dump_bytes(mppe_keys,size);
    TLS_WRAPPER_DEBUG(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "\n");

    return CY_RSLT_SUCCESS;
#else
    return CY_RSLT_MODULE_TLS_UNSUPPORTED;
#endif /* MBEDTLS_SSL_EXPORT_KEYS */
}
#endif /* MBEDTLS_VERSION_NUMBER >= MBEDTLS_VERSION_WITH_PRF_SUPPORT */
static cy_rslt_t tls_load_certificate_key ( cy_tls_identity_t* identity,  const uint8_t* certificate_data, uint32_t certificate_length, const char* private_key, const uint32_t key_length )
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    int ret = 0;

    if (identity == NULL)
    {
        TLS_WRAPPER_DEBUG(CYLF_MIDDLEWARE, CY_LOG_ERR, " [%s][%s][%d] Invalid TLS identity\r\n", __FILE__, __func__, __LINE__);
        return CY_RSLT_MODULE_TLS_BADARG;
    }

    if ((certificate_data != NULL) && (certificate_length != 0))
    {
        /* load x509 certificate */
        mbedtls_x509_crt_init( &identity->certificate );

        ret = mbedtls_x509_crt_parse( &identity->certificate, (const unsigned char *) certificate_data, certificate_length + 1 );

        if (ret != 0)
        {
            TLS_WRAPPER_DEBUG(CYLF_MIDDLEWARE, CY_LOG_ERR, " [%s][%s][%d] Unable to parse TLS certificate ret %d\r\n", __FILE__, __func__, __LINE__, ret);
            result = CY_RSLT_MODULE_TLS_PARSE_CERTIFICATE;
            goto ERROR_CERTIFICATE_INIT;
        }

    }

    if ((private_key != NULL) && (key_length != 0))
    {
        /* load key */
        mbedtls_pk_init( &identity->private_key );

        ret = mbedtls_pk_parse_key( &identity->private_key, (const unsigned char *) private_key, key_length+1, NULL, 0 );
        if ( ret != 0 )
        {
            TLS_WRAPPER_DEBUG(CYLF_MIDDLEWARE, CY_LOG_ERR, " [%s][%s][%d] Unable to parse TLS private key ret %d\r\n", __FILE__, __func__, __LINE__, ret);
            result = CY_RSLT_MODULE_TLS_PARSE_KEY;
            goto ERROR_KEY_INIT;
        }

    }

    return CY_RSLT_SUCCESS;

ERROR_KEY_INIT:
    mbedtls_pk_free( &identity->private_key );

ERROR_CERTIFICATE_INIT:
    mbedtls_x509_crt_free( &identity->certificate );

    return result;
}

cy_rslt_t cy_tls_init_context(cy_tls_context_t* tls_context, cy_tls_identity_t* identity, char* peer_cn)
{
    if (tls_context == NULL)
    {
        TLS_WRAPPER_DEBUG(CYLF_MIDDLEWARE, CY_LOG_ERR, " [%s][%s][%d] Invalid TLS context\r\n", __FILE__, __func__, __LINE__);
        return CY_RSLT_MODULE_TLS_BADARG;
    }
    tls_context->identity = identity;
    tls_context->peer_cn = peer_cn;

    /* tls13 not added yet */
    tls_context->tls_v13 = false;
    return CY_RSLT_SUCCESS;
}

cy_rslt_t cy_tls_deinit_context( cy_tls_context_t* tls_context )
{
    if (tls_context == NULL)
    {
        TLS_WRAPPER_DEBUG(CYLF_MIDDLEWARE, CY_LOG_ERR, " [%s][%s][%d] Invalid TLS context\r\n", __FILE__, __func__, __LINE__);
        return CY_RSLT_MODULE_TLS_BADARG;
    }
    mbedtls_ssl_config_free( (mbedtls_ssl_config*) &tls_context->conf );
    mbedtls_ssl_free( &tls_context->context );
    mbedtls_ctr_drbg_free( &tls_context->ctr_drbg );
    mbedtls_entropy_free( &tls_context->entropy );

    if (tls_context->root_ca_certificates != NULL)
    {
        TLS_WRAPPER_DEBUG(CYLF_MIDDLEWARE, CY_LOG_DEBUG, " [%s][%s][%d] Freeing Root ca certificate \r\n", __FILE__, __func__, __LINE__);
        mbedtls_x509_crt_free( tls_context->root_ca_certificates );
        free( tls_context->root_ca_certificates );
        tls_context->root_ca_certificates = NULL;
    }
    return CY_RSLT_SUCCESS;
}

cy_rslt_t cy_tls_init_root_ca_certificates( cy_tls_context_t* context, const char* trusted_ca_certificates, const uint32_t length )
{
    int result;

    if (context->root_ca_certificates == NULL)
    {
        TLS_WRAPPER_DEBUG(CYLF_MIDDLEWARE, CY_LOG_ERR, " [%s][%s][%d] bad arg\r\n", __FILE__, __func__, __LINE__);
        context->root_ca_certificates = (cy_x509_crt_t*)malloc(sizeof(cy_x509_crt_t));
        if(context->root_ca_certificates == NULL)
        {
            return CY_RSLT_MODULE_TLS_OUT_OF_HEAP_SPACE;
        }
    }

    if (trusted_ca_certificates == NULL || length == 0)
    {
        TLS_WRAPPER_DEBUG(CYLF_MIDDLEWARE, CY_LOG_ERR, " [%s][%s][%d] No certificate is passed \r\n", __FILE__, __func__, __LINE__);
        return CY_RSLT_MODULE_TLS_BADARG;
    }

    mbedtls_x509_crt_init(context->root_ca_certificates);

    /* Parse RootCA Certificate */
    result = mbedtls_x509_crt_parse(context->root_ca_certificates,(const unsigned char *) trusted_ca_certificates, length + 1);
    if (result != 0)
    {
        TLS_WRAPPER_DEBUG(CYLF_MIDDLEWARE, CY_LOG_DEBUG, " [%s][%s][%d]  Result from mbedtls_x509_crt_parse is %lx (%ld) (~%lx)\n", __FILE__, __func__, __LINE__, (uint32_t) result, (uint32_t) result, ~((uint32_t) result));
        mbedtls_x509_crt_free(context->root_ca_certificates);
        return CY_RSLT_MODULE_TLS_PARSE_CERTIFICATE;
    }

    return CY_RSLT_SUCCESS;
}

cy_rslt_t cy_tls_deinit_root_ca_certificates( cy_tls_context_t* context )
{
    if (context->root_ca_certificates != NULL)
    {
        TLS_WRAPPER_DEBUG(CYLF_MIDDLEWARE, CY_LOG_DEBUG, " [%s][%s][%d] Freeing root ca certificates\r\n", __FILE__, __func__, __LINE__);
        mbedtls_x509_crt_free(context->root_ca_certificates);
        free(context->root_ca_certificates);
        context->root_ca_certificates = NULL;
    }
    return CY_RSLT_SUCCESS;
}

cy_rslt_t cy_tls_init_identity( cy_tls_identity_t* identity, const char* private_key, const uint32_t key_length, const uint8_t* certificate_data, uint32_t certificate_length )
{
    if (identity == NULL)
    {
        TLS_WRAPPER_DEBUG(CYLF_MIDDLEWARE, CY_LOG_ERR, " [%s][%s][%d] Invalid identity \n", __FILE__, __func__, __LINE__);
        return CY_RSLT_MODULE_TLS_BADARG;
    }

    memset( identity, 0, sizeof( *identity ) );

    if ( tls_load_certificate_key(identity, certificate_data, certificate_length, private_key, key_length) != CY_RSLT_SUCCESS )
    {
        TLS_WRAPPER_DEBUG(CYLF_MIDDLEWARE, CY_LOG_ERR, " [%s][%s][%d]  Failed to load certificate & private key \n", __FILE__, __func__, __LINE__);
        return CY_RSLT_MODULE_TLS_PARSE_KEY;
    }
    if(certificate_data == NULL || private_key == NULL)
    {
        identity->is_client_auth = 0;
    }
    else
    {
        identity->is_client_auth = 1;
    }

    return CY_RSLT_SUCCESS;
}

cy_rslt_t cy_tls_deinit_identity(cy_tls_identity_t* identity)
{
    if (identity == NULL)
    {
        TLS_WRAPPER_DEBUG(CYLF_MIDDLEWARE, CY_LOG_ERR, " [%s][%s][%d] Invalid identity \n", __FILE__, __func__, __LINE__);
        return CY_RSLT_MODULE_TLS_BADARG;
    }
    if (identity != NULL)
    {
        mbedtls_x509_crt_free(&identity->certificate);
        mbedtls_pk_free(&identity->private_key);
    }
    return CY_RSLT_SUCCESS;
}

cy_rslt_t cy_tls_receive_eap_packet( supplicant_workspace_t* supplicant, supplicant_packet_t* packet )
{
    int ret = 0;
    supplicant_buffer_t *buffer = *((supplicant_buffer_t**)packet);

    ret = mbedtls_ssl_read(&supplicant->tls_context->context, buffer->payload, 1024);

    if (ret < 0)
    {
        return CY_RSLT_MODULE_TLS_ERROR;
    }

    buffer->len = ret;

    return CY_RSLT_SUCCESS;
}

cy_rslt_t cy_crypto_get_random( cy_tls_context_t *context, void* buffer, uint16_t buffer_length )
{

    int ret = mbedtls_ctr_drbg_random(&context->ctr_drbg, buffer, buffer_length);
    if(ret !=0 )
    {
        return CY_RSLT_MODULE_TLS_ERROR;
    }
    return CY_RSLT_SUCCESS;
}

void cy_tls_init_workspace_context( cy_tls_context_t *context )
{
    if( context )
    {
        context->context.conf = NULL;
    }
}

cy_rslt_t cy_tls_get_versions(cy_tls_context_t* context, uint8_t *major_version, uint8_t *minor_version)
{
    if(context == NULL || major_version == NULL || minor_version == NULL)
    {
        return CY_RSLT_MODULE_TLS_BADARG;
    }

    *major_version = (uint8_t)context->context.major_ver;
    *minor_version = (uint8_t)context->context.minor_ver;

    return CY_RSLT_SUCCESS;
}

cy_rslt_t cy_tls_calculate_overhead( void *work, cy_tls_context_t* tls_context, uint16_t available_space, uint16_t* header, uint16_t* footer)
{
    mbedtls_cipher_mode_t mode;
    supplicant_workspace_t* workspace = (supplicant_workspace_t *)work;
    cy_tls_workspace_t* context;

    if( work == NULL || tls_context == NULL || header == NULL || footer == NULL )
    {
        return CY_RSLT_MODULE_TLS_BADARG;
    }

    context = &tls_context->context;

    *header = 0;
    *footer = 0;

    if( context->state == MBEDTLS_SSL_HANDSHAKE_OVER)
    {
        /* Add TLS record size */
        *header = sizeof(tls_record_header_t);

        mode = mbedtls_cipher_get_cipher_mode( &context->transform->cipher_ctx_enc );

        if( mode == MBEDTLS_MODE_GCM || mode == MBEDTLS_MODE_CCM )
        {
            unsigned char taglen = workspace->cipher_flags & MBEDTLS_CIPHERSUITE_SHORT_TAG ? 8 : 16;
            *footer += ( taglen + ( context->transform->ivlen - context->transform->fixed_ivlen ));
        }

        if ( mode == MBEDTLS_MODE_CBC )
        {
            *footer += ( available_space - *header - *footer ) % context->transform->ivlen + 1;

            if ( context->minor_ver >= MBEDTLS_SSL_MINOR_VERSION_2 )
            {
                *footer += context->transform->ivlen;
            }
        }

        if( mode == MBEDTLS_MODE_STREAM || ( mode == MBEDTLS_MODE_CBC ) )
        {
            *footer +=  context->transform->maclen;
        }
    }
    return CY_RSLT_SUCCESS;
}

void cy_tls_free_eap_packet (void *packet)
{
    CY_TLS_UNUSED_PARAM(packet);
}

cy_rslt_t cy_tls_encrypt_data(cy_tls_context_t* tls_context, uint8_t* out, uint8_t* in, uint32_t* in_out_length)
{
    if( mbedtls_ssl_write( &tls_context->context , in, *in_out_length ) < 0 )
    {
        return CY_RSLT_MODULE_TLS_ERROR;
    }

    *in_out_length = tls_context->context.out_msglen + sizeof(tls_record_header_t);

    memcpy(out, tls_context->context.out_hdr, *in_out_length);

    return CY_RSLT_SUCCESS;
}

