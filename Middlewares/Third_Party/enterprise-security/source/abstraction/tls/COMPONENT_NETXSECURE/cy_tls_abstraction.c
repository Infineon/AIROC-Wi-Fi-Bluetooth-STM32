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

#include "cy_enterprise_tls_ciphersuite.h"
#include "cy_enterprise_security_log.h"
#include "cy_tls_abstraction.h"
#include "cy_supplicant_structures.h"
#include "cy_network_mw_core.h"
#include "cy_rtos_abstraction.h"
#include "cy_supplicant_process_et.h"
#include "cy_eap.h"

/******************************************************
 *                      Macros
 ******************************************************/

/* Maximum TLS record size */
#define CY_TLS_PACKET_BUFFER_SIZE (7*1024)

#ifndef CY_TLS_MAX_NUM_CERTS
#define CY_TLS_MAX_NUM_CERTS 3
#endif

/* Maximum certificate size */
#ifndef CY_TLS_MAX_CERTIFICATE_SIZE
#define CY_TLS_MAX_CERTIFICATE_SIZE 2500
#endif

/* TLS handshake timeout value in milli-seconds */
#ifndef CY_EAP_TLS_TIMEOUT
#define CY_EAP_TLS_TIMEOUT 20000
#endif

#define TLS_RANDOM_BYTES                        (64)
#define TLS_DEFAULT_PACKET_ALLOCATE_TIMEOUT     (2 * 1000)

#define RSA_KEY_START                           "-----BEGIN RSA PRIVATE KEY-----"
#define RSA_KEY_END                             "-----END RSA PRIVATE KEY-----"

#define EC_KEY_START                            "-----BEGIN EC PRIVATE KEY-----"
#define EC_KEY_END                              "-----END EC PRIVATE KEY-----"


#define TLS13_EXPORTER_LABEL                    "exporter"


#define CY_TLS_DEFAULT_WAIT                     NX_TIMEOUT(1000)

/******************************************************
 *                      Debugging
 ******************************************************/

static const unsigned char base64_dec_map[128] =
{
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
    127, 127, 127,  62, 127, 127, 127,  63,  52,  53,
     54,  55,  56,  57,  58,  59,  60,  61, 127, 127,
    127,  64, 127, 127, 127,   0,   1,   2,   3,   4,
      5,   6,   7,   8,   9,  10,  11,  12,  13,  14,
     15,  16,  17,  18,  19,  20,  21,  22,  23,  24,
     25, 127, 127, 127, 127, 127, 127,  26,  27,  28,
     29,  30,  31,  32,  33,  34,  35,  36,  37,  38,
     39,  40,  41,  42,  43,  44,  45,  46,  47,  48,
     49,  50,  51, 127, 127, 127, 127, 127
};

/******************************************************
 *                    Enumerations
 ******************************************************/
typedef enum
{
    CY_TLS_PEM_TYPE_CERT = 0,
    CY_TLS_PEM_TYPE_KEY = 1
} cy_tls_pem_type_t;

/******************************************************
 *                    Variables
 ******************************************************/

/******************************************************
 *                    Function prototypes
 ******************************************************/
#ifdef ENABLE_TLS_WRAPPER_DUMP
static void tls_dump_bytes ( const uint8_t* bptr, uint32_t len );
#endif

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



/* Wrapper function for free */
static void mem_free( void **ptr )
{
    if( ptr && *ptr )
    {
        cy_rtos_free( *ptr );
        *ptr = NULL;
    }
}

/* Wrapper function for malloc */
static void* mem_malloc( size_t size )
{
    void *ptr = cy_rtos_malloc(size);
    return ptr;
}

/* Wrapper function for calloc */
static void* mem_calloc( size_t size, uint32_t num )
{
    void *ptr = cy_rtos_calloc(size, num);
    return ptr;
}

/*
 * @func  : cy_tls_parse_private_key
 *
 * @brief : parse private key and return the key type
 */
static INT cy_tls_parse_private_key( const char *private_key, UINT *private_key_type )
{
    unsigned char *header = NULL;
    unsigned char *footer = NULL;

    header = (unsigned char *) strstr( private_key, RSA_KEY_START);
    footer = (unsigned char *) strstr( private_key, RSA_KEY_END);

    if( header != NULL && footer != NULL && (const char *)header == private_key )
    {
        *private_key_type = NX_SECURE_X509_KEY_TYPE_RSA_PKCS1_DER;
        return 0;
    }

    header = (unsigned char *) strstr( private_key, EC_KEY_START);
    footer = (unsigned char *) strstr( private_key, EC_KEY_END);

    if( header != NULL && footer != NULL && (const char *)header == private_key )
    {
        *private_key_type = NX_SECURE_X509_KEY_TYPE_EC_DER;
        return 0;
    }
    return -1;
}

/*
 * @func  : cy_tls_base64_decode
 *
 * @brief : base64 decode
 */
static int cy_tls_base64_decode( const unsigned char *inbuf, size_t inbuflen, unsigned char *outbuf, size_t outbuflen, size_t *bytescopied )
{
    size_t i, n;
    uint32_t j, x;
    unsigned char *p;

    /* First pass: check for validity and get output length */
    for( i = n = j = 0; i < inbuflen; i++ )
    {
        /* Skip spaces before checking for EOL */
        x = 0;
        while( i < inbuflen && inbuf[i] == ' ' )
        {
            ++i;
            ++x;
        }

        /* Spaces at end of buffer are OK */
        if( i == inbuflen )
            break;

        if( ( inbuflen - i ) >= 2 &&
            inbuf[i] == '\r' && inbuf[i + 1] == '\n' )
            continue;

        if( inbuf[i] == '\n' )
            continue;

        /* Space inside a line is an error */
        if( x != 0 )
            return( -1 );

        if( inbuf[i] == '=' && ++j > 2 )
            return( -1 );

        if( inbuf[i] > 127 || base64_dec_map[inbuf[i]] == 127 )
            return( -1 );

        if( base64_dec_map[inbuf[i]] < 64 && j != 0 )
            return( -1 );

        n++;
    }

    if( n == 0 )
    {
        *bytescopied = 0;
        return( 0 );
    }

    /* The following expression is to calculate the following formula without
     * risk of integer overflow in n:
     *     n = ( ( n * 6 ) + 7 ) >> 3;
     */
    n = ( 6 * ( n >> 3 ) ) + ( ( 6 * ( n & 0x7 ) + 7 ) >> 3 );
    n -= j;

    if( outbuf == NULL || outbuflen < n )
    {
        *bytescopied = n;
        return( -2 );
    }

   for( j = 3, n = x = 0, p = outbuf; i > 0; i--, inbuf++ )
   {
        if( *inbuf == '\r' || *inbuf == '\n' || *inbuf == ' ' )
            continue;

        j -= ( base64_dec_map[*inbuf] == 64 );
        x  = ( x << 6 ) | ( base64_dec_map[*inbuf] & 0x3F );

        if( ++n == 4 )
        {
            n = 0;
            if( j > 0 ) *p++ = (unsigned char)( x >> 16 );
            if( j > 1 ) *p++ = (unsigned char)( x >>  8 );
            if( j > 2 ) *p++ = (unsigned char)( x       );
        }
    }

    *bytescopied = p - outbuf;
    return( 0 );
}

/*
 * @func  : cy_tls_strnstr
 *
 * @brief : wrapper function for strnstr
 */
static char* cy_tls_strnstr( const char *string, const char *sub_string, size_t length )
{
    char *ptr        = NULL;
    char *new_string = NULL;

    new_string = (char *)mem_malloc(length + 1);
    if(new_string != NULL)
    {
        memcpy(new_string, string, length);
        new_string[length] = '\0';

        ptr = strstr(new_string, sub_string);
        mem_free((void**)&new_string);
        if(ptr != NULL)
        {
            return (strstr(string, sub_string));
        }
    }
    return NULL;
}

/*
 * @func  : cy_tls_convert_pem_to_der
 *
 * @brief : convert PEM format to DER
 */
static INT cy_tls_convert_pem_to_der( const unsigned char *base64input, UINT inputlen, cy_tls_pem_type_t type, unsigned char *output, UINT *outputlen )
{
    const unsigned char *header = NULL;
    const unsigned char *footer = NULL;
    const unsigned char *end    = NULL;
    int error, header_len = 0, footer_len = 0, used_len = 0;

    if( type == CY_TLS_PEM_TYPE_KEY )
    {
        UINT private_key_type;

        error = cy_tls_parse_private_key( (const char *)base64input, &private_key_type );
        if( error == -1 )
        {
            TLS_WRAPPER_DEBUG( CYLF_MIDDLEWARE, CY_LOG_ERR, " [%s][%d] Invalid private keys \r\n", __func__, __LINE__ );
            return -1;
        }

        if( private_key_type == NX_SECURE_X509_KEY_TYPE_RSA_PKCS1_DER )
        {
            header_len = strlen(RSA_KEY_START);
            footer_len = strlen(RSA_KEY_END);
            header = (unsigned char *) cy_tls_strnstr((const char *)base64input, RSA_KEY_START, (size_t)(header_len + 1));
            footer = (unsigned char *) cy_tls_strnstr((const char *)base64input, RSA_KEY_END, (size_t)inputlen);
        }
        else
        {
            header_len = strlen(EC_KEY_START);
            footer_len = strlen(EC_KEY_END);
            header = (unsigned char *) cy_tls_strnstr((const char *)base64input, EC_KEY_START, (size_t)(header_len + 1));
            footer = (unsigned char *) cy_tls_strnstr((const char *)base64input, EC_KEY_END, (size_t)inputlen);
        }
    }
    else if( type == CY_TLS_PEM_TYPE_CERT )
    {
        header_len = strlen("-----BEGIN CERTIFICATE-----");
        footer_len = strlen("-----END CERTIFICATE-----");
        header = (unsigned char *) cy_tls_strnstr((const char *)base64input, "-----BEGIN CERTIFICATE-----", (size_t)(header_len + 1));
        footer = (unsigned char *) cy_tls_strnstr((const char *)base64input, "-----END CERTIFICATE-----", (size_t)inputlen);
    }
    else
    {
        /* Invalid pem type */
        return( -1 );
    }

    if( header == NULL || footer == NULL )
    {
        return( -1 );
    }
    if( footer < header )
    {
        return(-1);
    }

    header += header_len;
    if( *header == ' ' )  header++;
    if( *header == '\r' ) header++;
    if( *header == '\n' ) header++;

    end = footer + footer_len;
    if( *end == ' ' )  end++;
    if( *end == '\r' ) end++;
    if( *end == '\n' ) end++;

    used_len = end - base64input;
    if( (inputlen - used_len) > 1 )
    {
        /*
         * Currently only one certificate/key can be configured per request.
         * Returning error if buffer having more than one certificate/key.
         */
        return(-1);
    }

    /* Decode the PEM certificate to DER */
    error = cy_tls_base64_decode( (const unsigned char *) header, footer - header, output, *outputlen, outputlen );
    if( error != 0 )
    {
        return (-1);
    }
    return 0;
}

/*
 * @func  : tls_load_certificate_key
 *
 * @brief : initializes an NX_SECURE_X509_CERT structure with a DER-encoded X509 digital certificate and associated private key.
 */
static cy_rslt_t tls_load_certificate_key ( cy_tls_identity_t* identity,  const uint8_t* certificate_data, uint32_t certificate_length, const char* private_key, const uint32_t key_length )
{
    UINT error        = 0;
    UINT der_cert_len = 0;
    UINT der_key_len  = 0;
    UINT private_key_type;

    if( identity == NULL )
    {
        TLS_WRAPPER_DEBUG( CYLF_MIDDLEWARE, CY_LOG_ERR, "[%s][%d] Invalid TLS identity\r\n", __func__, __LINE__ );
        return CY_RSLT_MODULE_TLS_BADARG;
    }

    if( ((certificate_data == NULL) || (certificate_length == 0)))
    {
        /* This is a success case as some methods doesn't require client cert */
        return CY_RSLT_SUCCESS;
    }

    /* Allocate memory for certificate's DER data. */
    identity->certificate_der = (uint8_t*)mem_calloc( certificate_length, 1 );
    if( identity->certificate_der == NULL )
    {
        return CY_RSLT_MODULE_TLS_OUT_OF_HEAP_SPACE;
    }
    /* Convert PEM certificate to DER format */
    der_cert_len = certificate_length;
    error = cy_tls_convert_pem_to_der( (const unsigned char *)certificate_data, certificate_length, CY_TLS_PEM_TYPE_CERT, identity->certificate_der, &der_cert_len );
    if( error != 0 )
    {
        mem_free( (void**)&identity->certificate_der );
        return CY_RSLT_MODULE_TLS_PARSE_CERTIFICATE;
    }

    identity->private_key_der = NULL;

    /* Check for optional private key */
    if ((private_key != NULL) && (key_length != 0))
    {
        /* Find private key type */
        error = cy_tls_parse_private_key( private_key, &private_key_type );
        if( error == -1 )
        {
            TLS_WRAPPER_DEBUG( CYLF_MIDDLEWARE, CY_LOG_ERR, "[%s][%d] Invalid private keys \r\n", __func__, __LINE__ );
            mem_free( (void**)&identity->certificate_der );
            return CY_RSLT_MODULE_TLS_PARSE_KEY;
        }

        /* Allocate memory for privatekey's DER data. */
        identity->private_key_der = (uint8_t*)mem_calloc(key_length, 1);
        if( identity->private_key_der == NULL )
        {
            mem_free( (void**)&identity->certificate_der );
            return CY_RSLT_MODULE_TLS_OUT_OF_HEAP_SPACE;
        }

        /* Convert PEM key to DER format */
        der_key_len = key_length;
        error = cy_tls_convert_pem_to_der( (const unsigned char *)private_key, key_length, CY_TLS_PEM_TYPE_KEY, identity->private_key_der, &der_key_len );
        if( error != 0 )
        {
            mem_free( (void**)&identity->certificate_der );
            mem_free( (void**)&identity->private_key_der );
            return CY_RSLT_MODULE_TLS_PARSE_CERTIFICATE;
        }
    }

    /* Initialize the certificates */
    error = nx_secure_x509_certificate_initialize( &identity->certificate, (unsigned char *)identity->certificate_der, der_cert_len, NULL, 0,
                                                   (const unsigned char *)identity->private_key_der, der_key_len, private_key_type );
    if( error != NX_SUCCESS )
    {
        TLS_WRAPPER_DEBUG( CYLF_MIDDLEWARE, CY_LOG_ERR,
                "[%s][%d] nx_secure_x509_certificate_initialize failed with error : 0x%x \r\n", __func__, __LINE__, error );
        mem_free( (void**)&identity->certificate_der );
        mem_free( (void**)&identity->private_key_der );
        return CY_RSLT_MODULE_TLS_PARSE_CERTIFICATE;
    }
    return CY_RSLT_SUCCESS;
}

/*
 * @func  : supplicant_host_send_eap_tls_fragments
 *
 * @brief :  Send EAP packet
 */
static cy_rslt_t supplicant_host_send_eap_tls_fragments( supplicant_workspace_t* workspace, uint8_t* buffer, size_t length )
{
    cy_tls_context_t *tls_context;

    /* This could be called as part of session end.
     * It is possible that the buffer would have been freed
     */
    if( workspace == NULL || workspace->buffer == NULL )
    {
        return CY_RSLT_MODULE_TLS_BAD_INPUT_DATA;
    }

    tls_context = workspace->tls_context;

    TLS_WRAPPER_DEBUG( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "NX Client State : %u\n", tls_context->context.nx_secure_tls_client_state );

    switch( tls_context->context.nx_secure_tls_client_state )
    {
        case NX_SECURE_TLS_CLIENT_STATE_IDLE:
        case NX_SECURE_TLS_CLIENT_STATE_ERROR:
        case NX_SECURE_TLS_CLIENT_STATE_ALERT_SENT:
        case NX_SECURE_TLS_CLIENT_STATE_HELLO_REQUEST:
            memset( workspace->buffer, 0, workspace->buffer_size );

            /* Point the buffer pointer at the start of the buffer */
            workspace->data_start   = workspace->buffer;

            if ( workspace->buffer_size < length )
            {
                TLS_WRAPPER_DEBUG( CYLF_MIDDLEWARE, CY_LOG_ERR, "Unable to store the TLS packet \n" );
                return CY_RSLT_MODULE_TLS_OUT_OF_HEAP_SPACE;
            }

            memcpy( workspace->data_start, buffer, length );
            workspace->data_end = workspace->data_start + length;

            workspace->have_packet = 1;
            break;

        case NX_SECURE_TLS_CLIENT_STATE_SERVERHELLO:
        case NX_SECURE_TLS_CLIENT_STATE_SERVER_CERTIFICATE:
        case NX_SECURE_TLS_CLIENT_STATE_SERVER_KEY_EXCHANGE:
        case NX_SECURE_TLS_CLIENT_STATE_CERTIFICATE_REQUEST:
        case NX_SECURE_TLS_CLIENT_STATE_SERVERHELLO_DONE:
        case NX_SECURE_TLS_CLIENT_STATE_HANDSHAKE_FINISHED:
            if( workspace->have_packet == 0 )
            {
#if (NX_SECURE_TLS_TLS_1_3_ENABLED)
                if( tls_context->context.nx_secure_tls_1_3 )
                {
                    tls_context->expected_pkt_count = 1;
                }
                else
#endif
                {
                    tls_context->expected_pkt_count = 3;
                }
                if ( tls_context->context.nx_secure_tls_client_certificate_requested )
                {
                    tls_context->expected_pkt_count += 2;
                }
                /* start of the packet */
                memset( workspace->buffer, 0, workspace->buffer_size );

                workspace->data_start   = workspace->buffer;
                workspace->data_end     = workspace->data_start;

                workspace->have_packet = 1;
            }
            if ( ( workspace->data_end - workspace->data_start + length ) > workspace->buffer_size )
            {
                TLS_WRAPPER_DEBUG( CYLF_MIDDLEWARE, CY_LOG_ERR, "Unable to store the TLS packet \n" );
                return CY_RSLT_MODULE_TLS_OUT_OF_HEAP_SPACE;
            }
            memcpy( workspace->data_end, buffer, length );
            workspace->data_end += length;

            tls_context->expected_pkt_count--;
            if( tls_context->expected_pkt_count )
            {
                /* need more. return success now */
                return CY_RSLT_SUCCESS;
            }
            break;

        default:
            memset( workspace->buffer, 0, workspace->buffer_size );

            workspace->data_start   = workspace->buffer;
            memcpy( workspace->data_start, buffer, length );
            workspace->data_end = workspace->data_start + length;

            workspace->have_packet = 1;
            break;
    }
    return ( supplicant_fragment_and_queue_eap_response(workspace) );
}

/*
 * @func  : eap_ssl_send_packet
 *
 * @brief : Callback function to send the packet to radio.
 */
static UINT eap_ssl_send_packet( void *opaque, NX_PACKET *packet_ptr, ULONG wait_option )
{
    supplicant_workspace_t* supplicant = (supplicant_workspace_t*) opaque;
    cy_rslt_t result                   = CY_RSLT_SUCCESS;

    if( packet_ptr == NULL )
    {
        return NX_NO_PACKET;
    }

    // TODO: Handle Chained packet
    result = supplicant_host_send_eap_tls_fragments( (supplicant_workspace_t*) supplicant,
                    (uint8_t*) packet_ptr->nx_packet_prepend_ptr, packet_ptr->nx_packet_length );
    if( result != CY_RSLT_SUCCESS )
    {
        TLS_WRAPPER_DEBUG( CYLF_MIDDLEWARE, CY_LOG_ERR, "Unable to send EAP TLS packet \n" );
        return NX_NOT_CONNECTED;
    }

    /* We no longer use this pkt, release it. */
    nx_packet_release(packet_ptr);
    return NX_SUCCESS;
}

/*
 * @func  : eap_ssl_receive_packet
 *
 * @brief : Callback function to receive the packet from radio.
 */
static UINT eap_ssl_receive_packet( void *opaque, NX_PACKET **packet_ptr, ULONG wait_option )
{
    supplicant_workspace_t* workspace = (supplicant_workspace_t*) opaque;
    tls_agent_packet_t* packet        = NULL;
    uint32_t length                   = 0;

    packet = supplicant_receive_eap_tls_packet( workspace, &length, SUPPLICANT_TIMEOUT );
    if( packet == NULL )
    {
        TLS_WRAPPER_DEBUG( CYLF_MIDDLEWARE, CY_LOG_ERR, "error eap_ssl_receive_packet : TLS TIMEOUT \n" );
        return NX_NO_PACKET;
    }
    /* We have received this buffer from WHD. Pass as it is. */
    *packet_ptr = (NX_PACKET*)packet;
    return NX_SUCCESS;
}

/*
 * @func  : cy_tls_session_cleanup
 *
 * @brief : Cleanup existing TLS session.
 */
void cy_tls_session_cleanup( cy_tls_context_t* tls_context )
{
    NX_PACKET *tmp_ptr;
    UINT status;

    if( tls_context && tls_context->tls_handshake_successful )
    {
        /* Get the protection. */
        tx_mutex_get(&_nx_secure_tls_protection, TX_WAIT_FOREVER);

        /* Release packets in queue. */
        while( tls_context->context.nx_secure_record_queue_header )
        {
            tmp_ptr = tls_context->context.nx_secure_record_queue_header;
            tls_context->context.nx_secure_record_queue_header = tmp_ptr -> nx_packet_queue_next;
            tmp_ptr -> nx_packet_queue_next = NX_NULL;
            nx_secure_tls_packet_release(tmp_ptr);
        }

        if( tls_context->context.nx_secure_record_decrypted_packet )
        {
            nx_secure_tls_packet_release(tls_context->context.nx_secure_record_decrypted_packet);
            tls_context->context.nx_secure_record_decrypted_packet = NX_NULL;
        }
        tx_mutex_put(&_nx_secure_tls_protection);

        /* Reset the TLS state */
        status = nx_secure_tls_session_reset(&tls_context->context);
        if ( status != NX_SUCCESS )
        {
            TLS_WRAPPER_DEBUG( CYLF_MIDDLEWARE, CY_LOG_WARNING, "nx_secure_tls_session_reset failed 0x%x\r\n", status );
        }
        /* Delete the session */
        status = nx_secure_tls_session_delete( &tls_context->context );
        if ( status != NX_SUCCESS )
        {
            TLS_WRAPPER_DEBUG( CYLF_MIDDLEWARE, CY_LOG_WARNING, "nx_secure_tls_session_delete failed 0x%x\r\n", status );
        }

        mem_free( (void**)&tls_context->tls_metadata );
        mem_free( (void**)&tls_context->tls_packet_buffer );
        mem_free( (void**)&tls_context->certificate_buffer );

        tls_context->tls_handshake_successful = false;
    }
}

/*
 * @func  : cy_tls_generic_start_tls_with_ciphers
 *
 * @brief : start tls handshake.
 */
cy_rslt_t cy_tls_generic_start_tls_with_ciphers( cy_tls_context_t* tls_context, void* referee, cy_tls_certificate_verification_t verification )
{
    UINT error;
    cy_tls_identity_t *tls_identity;
    ULONG metadata_size;
    NX_PACKET_POOL *pool;
    bool tls_session_created            = false;
    cy_rslt_t result                    = CY_RSLT_SUCCESS;
    const NX_SECURE_TLS_CRYPTO *cipher_table;

    if( tls_context == NULL || tls_context->identity == NULL )
    {
        return CY_RSLT_MODULE_TLS_BADARG;
    }

    /* start with tls13 false */
    tls_context->tls_v13 = false;

    /* cleanup if previous session is still active */
    cy_tls_session_cleanup( tls_context );

    tls_identity = (cy_tls_identity_t *)tls_context->identity;

    cipher_table = &cy_tls_ent_tls_v12_ciphers;

#if (NX_SECURE_TLS_TLS_1_3_ENABLED)
    /* NetXSecure doesnt support TLS1.3 with RSA keys/certificate. Hence only if the key type is not RSA,
     * use the TLS1.3 cipher table
     */
    if(tls_identity->certificate.nx_secure_x509_private_key_type != NX_SECURE_X509_KEY_TYPE_RSA_PKCS1_DER)
    {
        cipher_table = &cy_tls_ent_tls_v13_ciphers;
    }
#endif

    /* Find meta-data size that is needed for TLS session creation. */
    error = nx_secure_tls_metadata_size_calculate( cipher_table, &metadata_size );
    if( error != NX_SUCCESS )
    {
        TLS_WRAPPER_DEBUG( CYLF_MIDDLEWARE, CY_LOG_ERR, "nx_secure_tls_metadata_size_calculate failed 0x%x\r\n", error );
        return CY_RSLT_MODULE_TLS_ERROR;
    }

    /* Allocate memory for TLS meta-data */
    tls_context->tls_metadata = (int8_t*)mem_calloc( metadata_size, 1 );
    if( tls_context->tls_metadata == NULL )
    {
        TLS_WRAPPER_DEBUG( CYLF_MIDDLEWARE, CY_LOG_ERR, "failed to allocate memory for tls meta data \r\n" );
        return CY_RSLT_MODULE_TLS_OUT_OF_HEAP_SPACE;
    }

    /* Create TLS session */
    error = nx_secure_tls_session_create( &tls_context->context, cipher_table, tls_context->tls_metadata, metadata_size );
    if( error == NX_SUCCESS )
    {
       tls_session_created = true;
#ifdef NX_SECURE_ENABLE_ECC_CIPHERSUITE
       error = nx_secure_tls_ecc_initialize( &tls_context->context, nx_crypto_ecc_supported_groups, nx_crypto_ecc_supported_groups_size, nx_crypto_ecc_curves );
       if( error != NX_SUCCESS )
       {
           TLS_WRAPPER_DEBUG( CYLF_MIDDLEWARE, CY_LOG_ERR, "nx_secure_tls_ecc_initialize failed 0x%x\r\n", error );
           result = CY_RSLT_MODULE_TLS_ERROR;
           goto cleanup;
       }
#endif
    }
    else
    {
        TLS_WRAPPER_DEBUG( CYLF_MIDDLEWARE, CY_LOG_ERR, "nx_secure_tls_session_create failed 0x%x\r\n", error );
        result = CY_RSLT_MODULE_TLS_ERROR;
        goto cleanup;
    }

    /* Allocate memory for TLS re-assembly buffer */
    tls_context->tls_packet_buffer = (uint8_t *)mem_malloc( CY_TLS_PACKET_BUFFER_SIZE );
    if( tls_context->tls_packet_buffer == NULL )
    {
       TLS_WRAPPER_DEBUG( CYLF_MIDDLEWARE, CY_LOG_ERR, "failed to allocate memory for tls packet buffer \r\n" );
       result = CY_RSLT_MODULE_TLS_OUT_OF_HEAP_SPACE;
       goto cleanup;
    }

    error = nx_secure_tls_session_packet_buffer_set( &tls_context->context, tls_context->tls_packet_buffer, CY_TLS_PACKET_BUFFER_SIZE );
    if( error != NX_SUCCESS )
    {
        TLS_WRAPPER_DEBUG( CYLF_MIDDLEWARE, CY_LOG_ERR, "nx_secure_tls_session_packet_buffer_set failed 0x%x\r\n", error );
        result = CY_RSLT_MODULE_TLS_ERROR;
        goto cleanup;
    }

    tls_context->certificate_buffer = (uint8_t*)mem_calloc( (CY_TLS_MAX_CERTIFICATE_SIZE * CY_TLS_MAX_NUM_CERTS), 1 );
    if( tls_context->certificate_buffer == NULL )
    {
        TLS_WRAPPER_DEBUG( CYLF_MIDDLEWARE, CY_LOG_ERR, "failed to allocate memory for remote certificate buffer \r\n" );
        result = CY_RSLT_MODULE_TLS_OUT_OF_HEAP_SPACE;
        goto cleanup;
    }

    error = nx_secure_tls_remote_certificate_buffer_allocate( &tls_context->context, CY_TLS_MAX_NUM_CERTS, tls_context->certificate_buffer, CY_TLS_MAX_CERTIFICATE_SIZE * CY_TLS_MAX_NUM_CERTS );
    if(error != NX_SUCCESS)
    {
        TLS_WRAPPER_DEBUG( CYLF_MIDDLEWARE, CY_LOG_ERR, "nx_secure_tls_remote_certificate_buffer_allocate failed 0x%x\r\n", error );
        result = CY_RSLT_MODULE_TLS_ERROR;
        goto cleanup;
    }

    error = nx_secure_tls_trusted_certificate_add( &tls_context->context, tls_context->root_ca_certificates );
    if( error != NX_SUCCESS )
    {
        TLS_WRAPPER_DEBUG( CYLF_MIDDLEWARE, CY_LOG_ERR, "nx_secure_tls_trusted_certificate_add failed 0x%x\r\n", error );
        result = CY_RSLT_MODULE_TLS_ERROR;
        goto cleanup;
    }

    error = nx_secure_tls_local_certificate_add(&tls_context->context, &tls_identity->certificate);
    if( error != NX_SUCCESS )
    {
        TLS_WRAPPER_DEBUG( CYLF_MIDDLEWARE, CY_LOG_ERR, "nx_secure_tls_local_certificate_add failed 0x%x\r\n", error );
        result = CY_RSLT_MODULE_TLS_ERROR;
        goto cleanup;
    }

    TLS_WRAPPER_DEBUG( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Performing the TLS handshake\r\n" );

    result = cy_network_get_packet_pool( CY_NETWORK_PACKET_TX, (void **)&pool );
    if(result != CY_RSLT_SUCCESS)
    {
        TLS_WRAPPER_DEBUG( CYLF_MIDDLEWARE, CY_LOG_ERR, "Get packet pool failed 0x%x\r\n", result);
        result = CY_RSLT_MODULE_TLS_ERROR;
        goto cleanup;
    }

    error = cy_nx_secure_tls_session_start( &tls_context->context, eap_ssl_send_packet, eap_ssl_receive_packet, NX_TIMEOUT(CY_EAP_TLS_TIMEOUT), NX_IP_VERSION_V4, 0, pool, referee );
    if( error != NX_SUCCESS )
    {
        TLS_WRAPPER_DEBUG( CYLF_MIDDLEWARE, CY_LOG_ERR, "nx_secure_tls_session_start failed 0x%x\r\n", error );
        result = CY_RSLT_MODULE_TLS_ERROR;
        goto cleanup;
    }

#if (NX_SECURE_TLS_TLS_1_3_ENABLED)
    tls_context->tls_v13 = tls_context->context.nx_secure_tls_1_3;
#else
    tls_context->tls_v13 = false;
#endif

    if(tls_context->tls_v13)
    {
        /* tls 13 eap-tls requiers a 0x00 data at the end of transfer */
        NX_PACKET *packet = NULL;
        supplicant_workspace_t* workspace = (supplicant_workspace_t*)referee;

        if( CY_ENTERPRISE_SECURITY_EAP_TYPE_TLS == workspace->eap_type )
        {
            error = nx_secure_tls_session_receive(&tls_context->context, &packet, NX_TIMEOUT(1000));
            if( error != NX_SUCCESS )
            {
                TLS_WRAPPER_DEBUG( CYLF_MIDDLEWARE, CY_LOG_ERR, "Timeout waiting for TLS handshake success\r\n" );
                result = CY_RSLT_MODULE_TLS_ERROR;
                goto cleanup;
            }
            else if ( packet->nx_packet_length != 1 || packet->nx_packet_prepend_ptr[0] != 0x00 )
            {
                TLS_WRAPPER_DEBUG( CYLF_MIDDLEWARE, CY_LOG_ERR, "Invalid TLS handshake success msg\r\n" );
                result = CY_RSLT_MODULE_TLS_ERROR;
                nx_packet_release(packet);
                goto cleanup;
            }
            nx_packet_release(packet);
        }
    }
    tls_context->tls_handshake_successful = true;
    TLS_WRAPPER_DEBUG( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "TLS handshake successful \r\n" );
    return CY_RSLT_SUCCESS;

cleanup:
    if(tls_session_created)
    {
       nx_secure_tls_session_end( &tls_context->context, NX_WAIT_FOREVER );
       nx_secure_tls_session_delete( &tls_context->context );
    }

    tls_context->tls_v13 = false;

    mem_free( (void**)&tls_context->tls_metadata );
    mem_free( (void**)&tls_context->tls_packet_buffer );
    mem_free( (void**)&tls_context->certificate_buffer );

    return result;
}

#if (NX_SECURE_TLS_TLS_1_3_ENABLED)

static UINT cy_tls_hkdf_expand_label(NX_SECURE_TLS_SESSION *tls_session, UCHAR *secret, UINT secret_len,
                                     UCHAR *label, UINT label_len, UCHAR *context, UINT context_len, UINT length,
                                     UCHAR *output, UINT output_length, const NX_CRYPTO_METHOD *hash_method)
{
    UCHAR                   cy_tls_tls_hkdf_temp_output[100];
    UINT                    status;
    UINT                    data_len;
    const NX_CRYPTO_METHOD  *session_hkdf_method = NX_NULL;
    const NX_CRYPTO_METHOD  *session_hmac_method = NX_NULL;

    /* From RFC 8446, section 7.1:
    HKDF-Expand-Label(Secret, Label, Context, Length) =
           HKDF-Expand(Secret, HkdfLabel, Length)

      Where HkdfLabel is specified as:

      struct {
          uint16 length = Length;
          opaque label<7..255> = "tls13 " + Label;
          opaque context<0..255> = Context;
      } HkdfLabel;
    */

    if (sizeof(cy_tls_tls_hkdf_temp_output) < (10u + label_len + context_len))
    {
        /* Buffer too small. */
        return(NX_SECURE_TLS_PACKET_BUFFER_TOO_SMALL);
    }

    /* Get our HKDF method and hash routine. */
    session_hkdf_method = tls_session->nx_secure_tls_crypto_table->nx_secure_tls_hkdf_method;
    session_hmac_method = tls_session->nx_secure_tls_crypto_table->nx_secure_tls_hmac_method;

    /* Now build the HkdfLabel from our inputs. */
    cy_tls_tls_hkdf_temp_output[0] = (UCHAR)((length & 0xFF00) >> 8);
    cy_tls_tls_hkdf_temp_output[1] = (UCHAR)(length & 0x00FF);
    data_len = 2;

    /* Add the length of the label (single octet). */
    cy_tls_tls_hkdf_temp_output[data_len] = (UCHAR)(6 + label_len);
    data_len = data_len + 1;

    /* Now copy in label with TLS 1.3 prefix. */
    NX_CRYPTO_MEMCPY(&cy_tls_tls_hkdf_temp_output[data_len], "tls13 ", 6); /* Use case of memcpy is verified. */
    data_len += 6;
    NX_CRYPTO_MEMCPY(&cy_tls_tls_hkdf_temp_output[data_len], label, label_len); /* Use case of memcpy is verified. */
    data_len += label_len;

    /* Add the length of the context (single octet). */
    cy_tls_tls_hkdf_temp_output[data_len] = (UCHAR)(context_len);
    data_len = data_len + 1;

    /* Now copy in context. */
    NX_CRYPTO_MEMCPY(&cy_tls_tls_hkdf_temp_output[data_len], context, context_len); /* Use case of memcpy is verified. */
    data_len += context_len;

    /* Initialize the HKDF context. */
    status = session_hkdf_method->nx_crypto_init((NX_CRYPTO_METHOD*)session_hkdf_method, NX_NULL, 0, NX_NULL,
                                        tls_session -> nx_secure_tls_prf_metadata_area,
                                        tls_session -> nx_secure_tls_prf_metadata_size);
    if(status != NX_CRYPTO_SUCCESS)
    {
        return(status);
    }

    /* Set the hash and HMAC routines for the HKDF. */
    status = session_hkdf_method->nx_crypto_operation(NX_CRYPTO_HKDF_SET_HMAC, NX_NULL, (NX_CRYPTO_METHOD*)session_hmac_method,
                                             NX_NULL, 0, NX_NULL, 0, NX_NULL, NX_NULL, 0,
                                             tls_session -> nx_secure_tls_prf_metadata_area,
                                             tls_session -> nx_secure_tls_prf_metadata_size,
                                             NX_NULL, NX_NULL);
    if(status != NX_CRYPTO_SUCCESS)
    {
        return(status);
    }

    status = session_hkdf_method->nx_crypto_operation(NX_CRYPTO_HKDF_SET_HASH, NX_NULL,
                                             (NX_CRYPTO_METHOD*)hash_method,
                                             NX_NULL, 0,NX_NULL, 0, NX_NULL, NX_NULL, 0,
                                             tls_session -> nx_secure_tls_prf_metadata_area,
                                             tls_session -> nx_secure_tls_prf_metadata_size,
                                             NX_NULL, NX_NULL);
    if(status != NX_CRYPTO_SUCCESS)
    {
        return(status);
    }

    /* Set the PRK for the HKDF-expand operation. */
    status = session_hkdf_method->nx_crypto_operation(NX_CRYPTO_HKDF_SET_PRK,
                                             NX_NULL,
                                             (NX_CRYPTO_METHOD*)session_hkdf_method,
                                             (UCHAR*)(secret),     /* Input HKDF label. */
                                             (secret_len << 3),
                                             NX_NULL,
                                             0,
                                             NX_NULL,
                                             NX_NULL,
                                             0,
                                             tls_session -> nx_secure_tls_prf_metadata_area,
                                             tls_session -> nx_secure_tls_prf_metadata_size,
                                             NX_NULL, NX_NULL);
    if(status != NX_CRYPTO_SUCCESS)
    {
        return(status);
    }

    /* Now perform the HKDF operation. */
    status = session_hkdf_method->nx_crypto_operation(NX_CRYPTO_HKDF_EXPAND,
                                             NX_NULL,
                                             (NX_CRYPTO_METHOD*)session_hkdf_method,
                                             (UCHAR*)(cy_tls_tls_hkdf_temp_output), /* Input HKDF label. */
                                             (data_len << 3),
                                             NX_NULL,
                                             0,
                                             NX_NULL,
                                             (UCHAR *)output,
                                             output_length,
                                             tls_session -> nx_secure_tls_prf_metadata_area,
                                             tls_session -> nx_secure_tls_prf_metadata_size,
                                             NX_NULL, NX_NULL);

    return(status);
}

static UINT cy_tls_generate_tls13_hash(NX_SECURE_TLS_SESSION *tls_session, UCHAR *input, UINT in_len,
                                       UCHAR *output, UINT output_length, const NX_CRYPTO_METHOD *hash_method)
{
    UINT metadata_size = 0;
    UINT status;

    if (tls_session->nx_secure_tls_session_ciphersuite->nx_secure_tls_hash_size == 48)
    {
        metadata_size = hash_method->nx_crypto_metadata_area_size;
    }
    else
    {
        metadata_size = tls_session -> nx_secure_tls_handshake_hash.nx_secure_tls_handshake_hash_sha256_metadata_size;
    }

    if (hash_method -> nx_crypto_init)
    {
        status = hash_method -> nx_crypto_init((NX_CRYPTO_METHOD*)hash_method,
                                                NX_NULL,
                                                0,
                                                tls_session -> nx_secure_tls_handshake_hash.nx_secure_tls_handshake_hash_sha256_handler,
                                                tls_session -> nx_secure_tls_handshake_hash.nx_secure_tls_handshake_hash_scratch,
                                                metadata_size);

        if (status != NX_CRYPTO_SUCCESS)
        {
            return(status);
        }
    }

    if (hash_method -> nx_crypto_operation != NX_NULL)
    {
        status = hash_method -> nx_crypto_operation(NX_CRYPTO_HASH_INITIALIZE,
                                                    tls_session -> nx_secure_tls_handshake_hash.nx_secure_tls_handshake_hash_sha256_handler,
                                                    (NX_CRYPTO_METHOD*)hash_method,
                                                    NX_NULL,
                                                    0,
                                                    NX_NULL,
                                                    0,
                                                    NX_NULL,
                                                    NX_NULL,
                                                    0,
                                                    tls_session -> nx_secure_tls_handshake_hash.nx_secure_tls_handshake_hash_scratch,
                                                    metadata_size,
                                                    NX_NULL,
                                                    NX_NULL);

        if (status != NX_CRYPTO_SUCCESS)
        {
            return(status);
        }
    }

    if (hash_method -> nx_crypto_operation != NX_NULL)
    {
        status = hash_method -> nx_crypto_operation(NX_CRYPTO_HASH_UPDATE,
                                                    tls_session -> nx_secure_tls_handshake_hash.nx_secure_tls_handshake_hash_sha256_handler,
                                                    (NX_CRYPTO_METHOD*)hash_method,
                                                    NX_NULL,
                                                    0,
                                                    (UCHAR *)input,
                                                    in_len,
                                                    NX_NULL,
                                                    NX_NULL,
                                                    0,
                                                    tls_session -> nx_secure_tls_handshake_hash.nx_secure_tls_handshake_hash_scratch,
                                                    metadata_size,
                                                    NX_NULL,
                                                    NX_NULL);

        if (status != NX_CRYPTO_SUCCESS)
        {
            return(status);
        }
    }



    /* Generate a hash using our temporary copy of the hash metadata, place it into the TLS Session transcript hash array. */
    if (hash_method -> nx_crypto_operation != NX_NULL)
    {
        status = hash_method -> nx_crypto_operation(NX_CRYPTO_HASH_CALCULATE,
                                                    tls_session -> nx_secure_tls_handshake_hash.nx_secure_tls_handshake_hash_sha256_handler,
                                                    (NX_CRYPTO_METHOD*)hash_method,
                                                    NX_NULL,
                                                    0,
                                                    NX_NULL,
                                                    0,
                                                    NX_NULL,
                                                    output,
                                                    output_length,
                                                    tls_session -> nx_secure_tls_handshake_hash.nx_secure_tls_handshake_hash_scratch,
                                                    metadata_size,
                                                    NX_NULL,
                                                    NX_NULL);

        if (status != NX_CRYPTO_SUCCESS)
        {
            return(status);
        }

    }
    return NX_SUCCESS;
}

static UINT cy_tls_derive_secret(NX_SECURE_TLS_SESSION *tls_session, UCHAR *secret, UINT secret_len,
                                 UCHAR *label, UINT label_len,
                                 UCHAR *message_hash, UINT message_hash_len,
                                 UCHAR *output, UINT output_length, const NX_CRYPTO_METHOD *hash_method)
{
    UCHAR cy_tls_temp_hash[100];
    UINT status;
    UINT hash_length;
    UINT metadata_size = 0;

/* From RFC 8446, section 7.1:
        Derive-Secret(Secret, Label, Messages) =
                 HKDF-Expand-Label(Secret, Label,
                                   Transcript-Hash(Messages), Hash.length)
*/


    /* Get session hash routine. */
    hash_length = (hash_method->nx_crypto_ICV_size_in_bits >> 3);

    /* Our "messages" parameter is actually the ongoing hash of handshake
       messages stored in the TLS session context. In some contexts, the message hash will be of 0 length! */
    if(message_hash_len == 0)
    {
        /* Point the message hash at our temporary buffer. */
        message_hash = &cy_tls_temp_hash[0];
        message_hash_len = hash_length;

        /* IFX: WPA3 changes to add SHA384 */
        if (tls_session->nx_secure_tls_session_ciphersuite->nx_secure_tls_hash_size == 48)
        {
            metadata_size = hash_method->nx_crypto_metadata_area_size;
        }
        else
        {
            metadata_size = tls_session -> nx_secure_tls_handshake_hash.nx_secure_tls_handshake_hash_sha256_metadata_size;
        }
        /* Context has 0 length, so generate a hash on the empty string to feed into expand label call below.
         * Utilize the temporary "hash scratch" data buffer to initialize and calculate the hash. */
        if (hash_method -> nx_crypto_init)
        {
            status = hash_method -> nx_crypto_init((NX_CRYPTO_METHOD*)hash_method,
                                                   NX_NULL,
                                                   0,
                                                   tls_session -> nx_secure_tls_handshake_hash.nx_secure_tls_handshake_hash_sha256_handler,
                                                   tls_session -> nx_secure_tls_handshake_hash.nx_secure_tls_handshake_hash_scratch,
												   metadata_size);

            if (status != NX_CRYPTO_SUCCESS)
            {
                return(status);
            }
        }

        if (hash_method -> nx_crypto_operation != NX_NULL)
        {
            status = hash_method -> nx_crypto_operation(NX_CRYPTO_HASH_INITIALIZE,
                                                        tls_session -> nx_secure_tls_handshake_hash.nx_secure_tls_handshake_hash_sha256_handler,
                                                        (NX_CRYPTO_METHOD*)hash_method,
                                                        NX_NULL,
                                                        0,
                                                        NX_NULL,
                                                        0,
                                                        NX_NULL,
                                                        NX_NULL,
                                                        0,
                                                        tls_session -> nx_secure_tls_handshake_hash.nx_secure_tls_handshake_hash_scratch,
														metadata_size,
                                                        NX_NULL,
                                                        NX_NULL);

            if (status != NX_CRYPTO_SUCCESS)
            {
                return(status);
            }
        }

        if (hash_method -> nx_crypto_operation != NX_NULL)
        {
           status = hash_method -> nx_crypto_operation(NX_CRYPTO_HASH_UPDATE,
                                                      tls_session -> nx_secure_tls_handshake_hash.nx_secure_tls_handshake_hash_sha256_handler,
                                                      (NX_CRYPTO_METHOD*)hash_method,
                                                      NX_NULL,
                                                      0,
                                                      (UCHAR *)"",
                                                      0,
                                                      NX_NULL,
                                                      NX_NULL,
                                                      0,
                                                      tls_session -> nx_secure_tls_handshake_hash.nx_secure_tls_handshake_hash_scratch,
													  metadata_size,
                                                      NX_NULL,
                                                      NX_NULL);

           if (status != NX_CRYPTO_SUCCESS)
           {
               return(status);
           }
        }

        /* Generate a hash using our temporary copy of the hash metadata, place it into the TLS Session transcript hash array. */
        if (hash_method -> nx_crypto_operation != NX_NULL)
        {
            status = hash_method -> nx_crypto_operation(NX_CRYPTO_HASH_CALCULATE,
                                                        tls_session -> nx_secure_tls_handshake_hash.nx_secure_tls_handshake_hash_sha256_handler,
                                                        (NX_CRYPTO_METHOD*)hash_method,
                                                        NX_NULL,
                                                        0,
                                                        NX_NULL,
                                                        0,
                                                        NX_NULL,
                                                        message_hash,
                                                        hash_length,
                                                        tls_session -> nx_secure_tls_handshake_hash.nx_secure_tls_handshake_hash_scratch,
														metadata_size,
                                                        NX_NULL,
                                                        NX_NULL);

            if (status != NX_CRYPTO_SUCCESS)
            {
                return(status);
            }

        }
    }

    /* Now derive the output by calling HKDF-Expand-Label. */
    status = cy_tls_hkdf_expand_label(tls_session, secret, secret_len,
            label, label_len, message_hash, message_hash_len, hash_length,
            output, output_length, hash_method);

    return(status);
}

static cy_rslt_t get_tls13_mppe_key( NX_SECURE_TLS_SESSION *tls_session, const char* label, uint8_t *context,
                                     uint16_t context_len, uint8_t* mppe_keys, int size )
{
    UINT status;
    NX_SECURE_TLS_KEY_SECRETS *secrets;
    UINT hash_length;
    const NX_CRYPTO_METHOD *hash_method;
    UINT label_length;
    UCHAR data[NX_SECURE_TLS_MAX_HASH_SIZE];
    UCHAR exportsecret[NX_SECURE_TLS_MAX_HASH_SIZE];
    UCHAR *exporterlabel= (UCHAR*)TLS13_EXPORTER_LABEL;
    UINT exporterlabel_len = strlen(TLS13_EXPORTER_LABEL);


    /* Get the hash method so we know how much data we are generating. */
    hash_method = tls_session -> nx_secure_tls_session_ciphersuite -> nx_secure_tls_hash;
    hash_length = (hash_method->nx_crypto_ICV_size_in_bits >> 3);

    /* Get a pointer to our key secrets for this session. */
    secrets = &tls_session->nx_secure_tls_key_material.nx_secure_tls_key_secrets;

    /* label length */
    label_length =  strlen(label);

    /* Generate hash of the context */
    cy_tls_generate_tls13_hash(tls_session, context, context_len, data, hash_length, hash_method);

    status = cy_tls_derive_secret(tls_session, secrets->tls_exporter_master_secret,
                                  secrets->tls_exporter_master_secret_len,
                                  (UCHAR *)label, label_length,
                                  (UCHAR *)"", 0,
                                  exportsecret, hash_length, hash_method);
    if(status != NX_SUCCESS)
    {
        return(status);
    }

    status = cy_tls_hkdf_expand_label(tls_session, exportsecret, hash_length,
                                      (UCHAR *)exporterlabel, exporterlabel_len,
                                      data, hash_length, size,
                                      mppe_keys, size, hash_method);
    if(status != NX_SUCCESS)
    {
        return(status);
    }
    return CY_RSLT_SUCCESS;
}

#endif

static cy_rslt_t get_tls12_mppe_key( NX_SECURE_TLS_SESSION *tls_session, const char* label, uint8_t* mppe_keys, int size)
{
    unsigned char                         randbytes[NX_SECURE_TLS_RANDOM_SIZE * 2];
    UCHAR                                *master_sec;
    UINT                                  status;
    const NX_SECURE_TLS_CIPHERSUITE_INFO *ciphersuite;
    VOID                                 *handler             = NX_NULL;
    const NX_CRYPTO_METHOD               *session_prf_method  = NX_NULL;


    /* Figure out which cipher-suite we are using. */
    ciphersuite = tls_session->nx_secure_tls_session_ciphersuite;
    if( ciphersuite == NX_NULL )
    {
        return CY_RSLT_MODULE_TLS_BAD_INPUT_DATA;
    }

    session_prf_method = ciphersuite -> nx_secure_tls_prf;

    master_sec = tls_session->nx_secure_tls_key_material.nx_secure_tls_master_secret;

    /* copy client random bytes */
    memcpy( randbytes, tls_session->nx_secure_tls_key_material.nx_secure_tls_client_random, NX_SECURE_TLS_RANDOM_SIZE );

    /* copy server random bytes */
    memcpy( &randbytes[NX_SECURE_TLS_RANDOM_SIZE], tls_session->nx_secure_tls_key_material.nx_secure_tls_server_random, NX_SECURE_TLS_RANDOM_SIZE );


    status = session_prf_method -> nx_crypto_init( (NX_CRYPTO_METHOD*)session_prf_method,
                                                    master_sec, 48,
                                                    &handler,
                                                    tls_session->nx_secure_tls_prf_metadata_area,
                                                    tls_session->nx_secure_tls_prf_metadata_size );

    if( status != NX_CRYPTO_SUCCESS )
    {
        TLS_WRAPPER_DEBUG( CYLF_MIDDLEWARE, CY_LOG_ERR, "nx_crypto_init failed 0x%x\r\n", status );
        return CY_RSLT_MODULE_TLS_ERROR;
    }

    /* Generate MPPE keys */
    status = session_prf_method -> nx_crypto_operation( NX_CRYPTO_PRF,
                                                        handler,
                                                        (NX_CRYPTO_METHOD*)session_prf_method,
                                                        (UCHAR *)label,
                                                        strlen(label),
                                                        randbytes,
                                                        2 * NX_SECURE_TLS_RANDOM_SIZE,
                                                        NX_NULL,
                                                        mppe_keys,
                                                        size,
                                                        tls_session->nx_secure_tls_prf_metadata_area,
                                                        tls_session->nx_secure_tls_prf_metadata_size,
                                                        NX_NULL,
                                                        NX_NULL );
    if( status != NX_CRYPTO_SUCCESS )
    {
        TLS_WRAPPER_DEBUG( CYLF_MIDDLEWARE, CY_LOG_ERR, "nx_crypto_operation failed 0x%x\r\n", status );
        return CY_RSLT_MODULE_TLS_ERROR;
    }
    status = session_prf_method -> nx_crypto_cleanup( tls_session->nx_secure_tls_prf_metadata_area );
    if( status != NX_CRYPTO_SUCCESS )
    {
        TLS_WRAPPER_DEBUG( CYLF_MIDDLEWARE, CY_LOG_ERR, "nx_crypto_cleanup failed 0x%x\r\n", status);
        return CY_RSLT_MODULE_TLS_ERROR;
    }
    return CY_RSLT_SUCCESS;
}

/*
 * @func  : cy_tls_get_mppe_key
 *
 * @brief : Generate MPPE key to be used for wifi handshake.
 */
cy_rslt_t cy_tls_get_mppe_key( cy_tls_context_t *tls_context, const char* label, uint8_t *context, uint16_t context_len, uint8_t* mppe_keys, int size )
{
    cy_rslt_t result                    = CY_RSLT_MODULE_TLS_UNSUPPORTED;
    NX_SECURE_TLS_SESSION *tls_session  = &tls_context->context;

#if (NX_SECURE_TLS_TLS_1_3_ENABLED)
    if (tls_context->tls_v13)
    {
        result = get_tls13_mppe_key(tls_session, label, context, context_len, mppe_keys, size);
    }
    else
#endif
    {
        result = get_tls12_mppe_key(tls_session, label, mppe_keys, size);
    }

    if (result != CY_RSLT_SUCCESS)
    {
        TLS_WRAPPER_DEBUG( CYLF_MIDDLEWARE, CY_LOG_ERR, "MPPE Key generation failed with error 0x%x\r\n", result);
        return result;
    }

#ifdef ENABLE_TLS_WRAPPER_DUMP
    TLS_WRAPPER_DEBUG( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "EAP-TLS key material is: \n");
    tls_dump_bytes(mppe_keys,size);
    TLS_WRAPPER_DEBUG( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "\n");
#endif
    return CY_RSLT_SUCCESS;
}

/*
 * @func  : cy_tls_init_context
 *
 * @brief : Initialize TLS context.
 */
cy_rslt_t cy_tls_init_context( cy_tls_context_t* tls_context, cy_tls_identity_t* identity, char* peer_cn )
{
    static bool netx_tls_init_done = false;

    if( tls_context == NULL )
    {
        TLS_WRAPPER_DEBUG( CYLF_MIDDLEWARE, CY_LOG_ERR, "[%s][%d] Invalid TLS context\r\n", __func__, __LINE__ );
        return CY_RSLT_MODULE_TLS_BADARG;
    }

    if( netx_tls_init_done == false )
    {
        nx_secure_tls_initialize();
        netx_tls_init_done =  true;
    }

    tls_context->identity = identity;
    tls_context->peer_cn  = peer_cn;

    return CY_RSLT_SUCCESS;
}

/*
 * @func  : cy_tls_deinit_context
 *
 * @brief : Deinitialize TLS context.
 */
cy_rslt_t cy_tls_deinit_context( cy_tls_context_t* tls_context )
{
    UINT error;

    if( tls_context == NULL )
    {
        TLS_WRAPPER_DEBUG( CYLF_MIDDLEWARE, CY_LOG_ERR, "Invalid TLS context\r\n" );
        return CY_RSLT_MODULE_TLS_BADARG;
    }

    if( tls_context->tls_handshake_successful == true )
    {
        /* Cleanup TLS session. */
        (void)nx_secure_tls_session_end( &tls_context->context, NX_WAIT_FOREVER );

        error = nx_secure_tls_session_delete( &tls_context->context );
        if ( error != NX_SUCCESS )
        {
            TLS_WRAPPER_DEBUG( CYLF_MIDDLEWARE, CY_LOG_ERR, "nx_secure_tls_session_delete failed 0x%x\r\n", error );
        }
        tls_context->tls_handshake_successful = false;
    }
    mem_free( (void**)&tls_context->tls_metadata );
    mem_free( (void**)&tls_context->tls_packet_buffer );
    mem_free( (void**)&tls_context->certificate_buffer );

    return CY_RSLT_SUCCESS;
}

/*
 * @func  : cy_tls_init_root_ca_certificates
 *
 * @brief : Initialize and add ca certificate to the trust chain
 *
 */
cy_rslt_t cy_tls_init_root_ca_certificates( cy_tls_context_t* context, const char* trusted_ca_certificates, const uint32_t length )
{
    INT error;
    UINT der_cert_len = 0;

    if( context == NULL || trusted_ca_certificates == NULL || length == 0 )
    {
        return CY_RSLT_MODULE_TLS_BADARG;
    }

    /* Allocate memory NetXSecure X509 format certificate */
    if( context->root_ca_certificates == NULL )
    {
        context->root_ca_certificates = (cy_x509_crt_t*)mem_calloc( sizeof(cy_x509_crt_t), 1 );
        if( context->root_ca_certificates == NULL )
        {
            return CY_RSLT_MODULE_TLS_OUT_OF_HEAP_SPACE;
        }
    }

    /* Allocate memory for DER formated rootCA certificate */
    context->root_ca_cert_der = mem_calloc( length, 1 );
    if( context->root_ca_cert_der == NULL )
    {
        mem_free( (void**)&context->root_ca_certificates );
        return CY_RSLT_MODULE_TLS_OUT_OF_HEAP_SPACE;
    }
    /* Convert PEM to DER */
    der_cert_len = length;
    error = cy_tls_convert_pem_to_der( (const unsigned char *)trusted_ca_certificates, length, CY_TLS_PEM_TYPE_CERT, context->root_ca_cert_der, &der_cert_len );
    if( error != 0 )
    {
        TLS_WRAPPER_DEBUG( CYLF_MIDDLEWARE, CY_LOG_ERR,
                "[%s][%d] cy_tls_convert_pem_to_der failed 0x%x\r\n", error, __func__, __LINE__ );

        mem_free( (void**)&context->root_ca_certificates );
        mem_free( (void**)&context->root_ca_cert_der );

        return CY_RSLT_MODULE_TLS_PARSE_CERTIFICATE;
    }

    /* Initialize the RootCA Certificate */
    error = nx_secure_x509_certificate_initialize( context->root_ca_certificates, context->root_ca_cert_der, der_cert_len, NULL, 0, NULL, 0, NX_SECURE_X509_KEY_TYPE_NONE );
    if( error != NX_SUCCESS )
    {
        TLS_WRAPPER_DEBUG( CYLF_MIDDLEWARE, CY_LOG_ERR,
                "[%s][%d] nx_secure_x509_certificate_initialize failed 0x%x\r\n", error, __func__, __LINE__);

        mem_free( (void**)&context->root_ca_certificates );
        mem_free( (void**)&context->root_ca_cert_der );

        return CY_RSLT_MODULE_TLS_PARSE_CERTIFICATE;
    }
    return CY_RSLT_SUCCESS;
}

/*
 * @func  : cy_tls_deinit_root_ca_certificates
 *
 * @brief : Deinitialize the root ca certificates.
 *
 */
cy_rslt_t cy_tls_deinit_root_ca_certificates( cy_tls_context_t* context )
{
    if( context->root_ca_certificates != NULL )
    {
        TLS_WRAPPER_DEBUG( CYLF_MIDDLEWARE, CY_LOG_DEBUG,
                "[%s][%d] Freeing root ca certificates\r\n", __func__, __LINE__ );
        mem_free( (void**)&context->root_ca_certificates );
    }

    if( context->root_ca_cert_der != NULL )
    {
        TLS_WRAPPER_DEBUG( CYLF_MIDDLEWARE, CY_LOG_DEBUG,
                "[%s][%d] Freeing root ca der certificates\r\n", __func__, __LINE__ );
        mem_free( (void**)&context->root_ca_cert_der );
    }
    return CY_RSLT_SUCCESS;
}

/*
 * @func  : cy_tls_init_identity
 *
 * @brief : Intialize TLS identity.
 */
cy_rslt_t cy_tls_init_identity( cy_tls_identity_t* identity, const char* private_key,
                                const uint32_t key_length, const uint8_t* certificate_data,
                                uint32_t certificate_length )
{
    cy_rslt_t res;

    if( identity == NULL )
    {
        TLS_WRAPPER_DEBUG( CYLF_MIDDLEWARE, CY_LOG_ERR,
                "[%s][%d] Invalid identity \n", __func__, __LINE__ );
        return CY_RSLT_MODULE_TLS_BADARG;
    }

    memset( identity, 0, sizeof( cy_tls_identity_t ) );

    res = tls_load_certificate_key( identity, certificate_data, certificate_length, private_key, key_length );
    if( res != CY_RSLT_SUCCESS )
    {
        TLS_WRAPPER_DEBUG( CYLF_MIDDLEWARE, CY_LOG_ERR,
                "[%s][%d]  Failed to load certificate & private key \n", __func__, __LINE__ );
        return CY_RSLT_MODULE_TLS_PARSE_KEY;
    }
    if( certificate_data == NULL || private_key == NULL )
    {
        identity->is_client_auth = 0;
    }
    else
    {
        identity->is_client_auth = 1;
    }
    return CY_RSLT_SUCCESS;
}

/*
 * @func  : cy_tls_deinit_identity
 *
 * @brief : Deinitialize the identity.
 */
cy_rslt_t cy_tls_deinit_identity(cy_tls_identity_t* identity)
{
    if( identity == NULL )
    {
        TLS_WRAPPER_DEBUG( CYLF_MIDDLEWARE, CY_LOG_ERR, "[%s][%d] Invalid identity \n", __func__, __LINE__ );
        return CY_RSLT_MODULE_TLS_BADARG;
    }
    mem_free( (void**)&identity->certificate_der );
    mem_free( (void**)&identity->private_key_der );

    return CY_RSLT_SUCCESS;
}

/*
 * @func  : cy_tls_receive_eap_packet
 *
 * @brief : Receive EAP packet from the tls session
 */
cy_rslt_t cy_tls_receive_eap_packet( supplicant_workspace_t* supplicant, supplicant_packet_t* packet )
{
    UINT error;

    error = nx_secure_tls_session_receive(&supplicant->tls_context->context, (NX_PACKET**)packet, NX_TIMEOUT(1000));

    if (error != NX_SUCCESS)
    {
        return CY_RSLT_ENTERPRISE_SECURITY_TLS_ERROR;
    }
    return CY_RSLT_SUCCESS;
}

/*
 * @func  : cy_tls_free_eap_packet
 *
 * @brief : Free-up packet.
 */
void cy_tls_free_eap_packet (void *packet)
{
    NX_PACKET *pkt = (NX_PACKET*)packet;

    if ( pkt )
    {
        nx_packet_release( pkt );
    }
}

/*
 * @func  : cy_crypto_get_random
 *
 * @brief : Generate random values and store in buffer
 */
cy_rslt_t cy_crypto_get_random( cy_tls_context_t *context, void* buffer, uint16_t buffer_length )
{
    uint32_t random_value;
    uint16_t i;
    uint16_t loop_len;
    uint8_t  *pBuf = (uint8_t*)buffer;

    CY_TLS_UNUSED_PARAM(context);

    if( buffer == NULL || buffer_length == 0 )
    {
        return CY_RSLT_MODULE_TLS_BADARG;
    }

    loop_len = buffer_length;
    if( buffer_length % 4 )
    {
        loop_len--;
    }

    for( i=0; i < loop_len; i += 4)
    {
        /* coverity[dont_call]
         * rand should not be used for security-related applications, because linear congruential algorithms are too easy to break.
         * However this is the current random function available for NetXSecure.
         */
        random_value = (uint32_t)NX_RAND();

        pBuf[i]   = (uint8_t)(random_value);
        pBuf[i+1] = (uint8_t)(random_value >> 8);
        pBuf[i+2] = (uint8_t)(random_value >> 16);
        pBuf[i+3] = (uint8_t)(random_value >> 24);
    }
    /* Fill for the remaining bytes */
    for( ; i < buffer_length; i++ )
    {
        /* coverity[dont_call]
         * rand should not be used for security-related applications, because linear congruential algorithms are too easy to break.
         * However this is the current random function available for NetXSecure.
         */
        pBuf[i] = (uint8_t)NX_RAND();
    }
    return CY_RSLT_SUCCESS;
}

/*
 * @func  : cy_tls_init_workspace_context
 *
 * @brief : Initialize workspace context
 */
void cy_tls_init_workspace_context( cy_tls_context_t *context )
{
    CY_TLS_UNUSED_PARAM(context);
}

/*
 * @func  : cy_tls_get_versions
 *
 * @brief : This function is used to get the negotiated TLS version
 */
cy_rslt_t cy_tls_get_versions(cy_tls_context_t* tls_context, uint8_t *major_version, uint8_t *minor_version)
{
    if(tls_context == NULL || major_version == NULL || minor_version == NULL)
    {
        return CY_RSLT_MODULE_TLS_BADARG;
    }

    *major_version = (UCHAR)((tls_context->context.nx_secure_tls_protocol_version & 0xFF00) >> 8);
    *minor_version = (UCHAR)(tls_context->context.nx_secure_tls_protocol_version & 0x00FF);

    return CY_RSLT_SUCCESS;
}

/*
 * @func  : cy_tls_calculate_overhead
 *
 * @brief : Calculates the maximium amount of payload that can fit in a given sized buffer
 */
cy_rslt_t cy_tls_calculate_overhead( void *work, cy_tls_context_t* tls_context, uint16_t available_space, uint16_t* header, uint16_t* footer)
{
    UINT status;
    USHORT iv_size = 0;
    cy_tls_workspace_t* context;

    CY_TLS_UNUSED_PARAM(work);

    if( tls_context == NULL || header == NULL || footer == NULL )
    {
        return CY_RSLT_MODULE_TLS_BADARG;
    }

    *header = 0;
    *footer = 0;

    context = &tls_context->context;

    if( context->nx_secure_tls_client_state == NX_SECURE_TLS_CLIENT_STATE_SERVERHELLO_DONE ||
                    context->nx_secure_tls_client_state == NX_SECURE_TLS_CLIENT_STATE_HANDSHAKE_FINISHED )
    {
        *header = NX_SECURE_TLS_RECORD_HEADER_SIZE;

        status = _nx_secure_tls_session_iv_size_get(context, &iv_size);
        if( NX_SUCCESS != status )
        {
            return CY_RSLT_ENTERPRISE_SECURITY_TLS_ERROR;
        }

        if (iv_size > 0)
        {
            *footer += iv_size;
        }
    }
    return CY_RSLT_SUCCESS;
}

/*
 * @func  : cy_tls_netx_encrypt_packet
 *
 * @brief : Encrypt the packet data
 */
static UINT cy_tls_netx_encrypt_packet(NX_SECURE_TLS_SESSION *tls_session, NX_PACKET *send_packet, ULONG wait_option)
{
    UINT       status;
    UINT       message_length;
    UCHAR     *mac_secret;
    UCHAR     *record_header;
    UCHAR      record_hash[NX_SECURE_TLS_MAX_HASH_SIZE];
    UINT       hash_length;
    UCHAR     *hash_data;
    ULONG      hash_data_length;
    ULONG      length;
    USHORT     iv_size = 0;
    NX_PACKET *current_packet;
    UCHAR      record_type = NX_SECURE_TLS_APPLICATION_DATA;

    /* Length of the data in the packet. */
    length = send_packet -> nx_packet_length;

    /* Get transmit mutex first. */
    status = tx_mutex_get(&(tls_session -> nx_secure_tls_session_transmit_mutex), TX_WAIT_FOREVER);
    if (status)
    {
        /* Unable to send due to another thread is still transmitting. */
        return(NX_SECURE_TLS_TRANSMIT_LOCKED);
    }

    /* See if this is an active session, we need to account for the IV if the session cipher
        uses one. TLS 1.3 does not use an explicit IV so don't add it.*/
    if (tls_session -> nx_secure_tls_local_session_active
    #if (NX_SECURE_TLS_TLS_1_3_ENABLED)
        && !tls_session->nx_secure_tls_1_3
    #endif
        )
    {

        /* Get the size of the IV used by the session cipher. */
        status = _nx_secure_tls_session_iv_size_get(tls_session, &iv_size);

        if (status != NX_SUCCESS)
        {
            tx_mutex_put(&(tls_session -> nx_secure_tls_session_transmit_mutex));
            return(status);
        }

        /* Ensure there is enough room for the IV data.  */
        if ((ULONG)(send_packet -> nx_packet_prepend_ptr - send_packet -> nx_packet_data_start) < iv_size)
        {
            /* Return an invalid packet error.  */
            tx_mutex_put(&(tls_session -> nx_secure_tls_session_transmit_mutex));
            return(NX_SECURE_TLS_INVALID_PACKET);
        }

        /* Back off the pointer to the point before the IV data allocation
            (can be 0). Increases length since we are moving the prepend pointer. */
        send_packet -> nx_packet_prepend_ptr -= iv_size;
        send_packet -> nx_packet_length += iv_size;
    }

    /* Ensure there is enough room for the record header.  */
    if ((ULONG)(send_packet -> nx_packet_prepend_ptr - send_packet -> nx_packet_data_start) < NX_SECURE_TLS_RECORD_HEADER_SIZE)
    {
        /* Return an invalid packet error.  */
        tx_mutex_put(&(tls_session -> nx_secure_tls_session_transmit_mutex));
        return(NX_SECURE_TLS_INVALID_PACKET);
    }

    /* Get a pointer to our record header which is now right after the prepend pointer. */
    record_header = send_packet -> nx_packet_prepend_ptr - NX_SECURE_TLS_RECORD_HEADER_SIZE;

    /* Build the TLS record header. */
    record_header[0] = record_type;

    /* Set the version number. */
    record_header[1] = (UCHAR)((tls_session -> nx_secure_tls_protocol_version & 0xFF00) >> 8);
    record_header[2] = (UCHAR)(tls_session -> nx_secure_tls_protocol_version & 0x00FF);

    /* Set the length of the record prior to hashing and encryption - this is because
        the hashing is done on the record as if it were not encrypted so any additional
        padding or IVs, etc. added to the length would invalidate the hash. We update
        the length following the encryption below. */
    message_length = length;
    record_header[3] = (UCHAR)((length & 0xFF00) >> 8);
    record_header[4] = (UCHAR)(length & 0x00FF);

    /* If the session is active, hash and encrypt the record payload using
        the session keys and chosen ciphersuite. */
    if (tls_session -> nx_secure_tls_local_session_active)
    {
        /*************************************************************************************************************/
        NX_ASSERT(tls_session -> nx_secure_tls_session_ciphersuite != NX_NULL)

    #if (NX_SECURE_TLS_TLS_1_3_ENABLED)
        /* TLS 1.3 records have the record type appended in a single byte. */
        if(tls_session->nx_secure_tls_1_3)
        {
            /* If in a TLS 1.3 encrypted session, write the message type to the end. */
            status = nx_packet_data_append(send_packet, (UCHAR*)(&record_type), 1,
                                            tls_session -> nx_secure_tls_packet_pool, NX_TIMEOUT(1000));

            if(status != NX_SUCCESS)
            {
                tx_mutex_put(&(tls_session -> nx_secure_tls_session_transmit_mutex));
                return(status);
            }

            /* Record header type is APPLICATION DATA for all TLS 1.3 encrypted records. */
            record_type = NX_SECURE_TLS_APPLICATION_DATA;
            record_header[0] = record_type;
        }
    #endif

        /* TLS 1.3 does uses AEAD instead of per-record hash MACs. */
        if (
    #if (NX_SECURE_TLS_TLS_1_3_ENABLED)
            !tls_session->nx_secure_tls_1_3 &&
    #endif
            tls_session -> nx_secure_tls_session_ciphersuite -> nx_secure_tls_hash -> nx_crypto_operation)
        {
            /* We are a client, so use the server's MAC secret. */
            mac_secret = tls_session -> nx_secure_tls_key_material.nx_secure_tls_client_write_mac_secret;

            /* Account for large records that exceed the packet size and are chained in multiple packets
                such as large certificate messages with multiple certificates.
                tls_session->nx_secure_hash_mac_metadata_area is persistent across the following calls, so it's important
                to not do anything that might change the contents of that buffer until the hash is calculated after the loop! */
            current_packet = send_packet;

            /* Initialize the hash routine with our MAC secret, sequence number, and header. */
            status = _nx_secure_tls_record_hash_initialize(tls_session, tls_session -> nx_secure_tls_local_sequence_number,
                                                            record_header, 5, &hash_length, mac_secret);

            /* Check return from hash routine initialization. */
            if (status != NX_SUCCESS)
            {
                tx_mutex_put(&(tls_session -> nx_secure_tls_session_transmit_mutex));
                return(status);
            }

            /* Start the hash data after the header and IV. */
            hash_data = current_packet -> nx_packet_prepend_ptr + iv_size;
            hash_data_length = (ULONG)(current_packet -> nx_packet_append_ptr - current_packet -> nx_packet_prepend_ptr) - iv_size;

            /* Walk packet chain. */
            do
            {
                /* Update the hash with the data. */
                status = _nx_secure_tls_record_hash_update(tls_session, hash_data,
                                                            (UINT)hash_data_length);

                /* Advance the packet pointer to the next packet in the chain. */
                current_packet = current_packet -> nx_packet_next;
                if (current_packet != NX_NULL)
                {
                    hash_data = current_packet -> nx_packet_prepend_ptr;
                    hash_data_length = (ULONG)(current_packet -> nx_packet_append_ptr - current_packet -> nx_packet_prepend_ptr);
                }
            } while (current_packet != NX_NULL);

            /* Generate the hash on the plaintext data. */
            _nx_secure_tls_record_hash_calculate(tls_session, record_hash, &hash_length);


            /* Append the hash to the plaintext data in the last packet before encryption. */
            status = nx_packet_data_append(send_packet, record_hash, hash_length,
                                            tls_session -> nx_secure_tls_packet_pool, NX_TIMEOUT(1000));

    #ifdef NX_SECURE_KEY_CLEAR
            NX_SECURE_MEMSET(record_hash, 0, sizeof(record_hash));
    #endif /* NX_SECURE_KEY_CLEAR  */
        }


        /*************************************************************************************************************/
        /***** ENCRYPTION *****/

        status = _nx_secure_tls_record_payload_encrypt(tls_session, send_packet, tls_session -> nx_secure_tls_local_sequence_number, record_type);

        if (status != NX_SUCCESS)
        {
            tx_mutex_put(&(tls_session -> nx_secure_tls_session_transmit_mutex));
            return(status);
        }

        /*************************************************************************************************************/

        /* Increment the sequence number. */
        if ((tls_session -> nx_secure_tls_local_sequence_number[0] + 1) == 0)
        {
            /* Check for overflow of the 32-bit number. */
            tls_session -> nx_secure_tls_local_sequence_number[1]++;

            if (tls_session -> nx_secure_tls_local_sequence_number[1] == 0)
            {

                /* Check for overflow of the 64-bit unsigned number. As it should not reach here
                    in practical, we return a general error to prevent overflow theoretically. */
                tx_mutex_put(&(tls_session -> nx_secure_tls_session_transmit_mutex));
                return(NX_NOT_SUCCESSFUL);
            }
        }
        tls_session -> nx_secure_tls_local_sequence_number[0]++;
    }

    /* The encryption above may have changed the payload length, so get the length from
        the packet and use it to update the record header. */
    message_length = send_packet -> nx_packet_length;

    /* Set the length of the record. */
    record_header[3] = (UCHAR)((message_length & 0xFF00) >> 8);
    record_header[4] = (UCHAR)(message_length & 0x00FF);

    /* Adjust packet length */
    send_packet -> nx_packet_prepend_ptr -= NX_SECURE_TLS_RECORD_HEADER_SIZE;
    send_packet -> nx_packet_length += NX_SECURE_TLS_RECORD_HEADER_SIZE;

    tx_mutex_put(&(tls_session -> nx_secure_tls_session_transmit_mutex));

    return NX_SUCCESS;
}

/*
 * @func  : cy_tls_encrypt_data
 *
 * @brief : Encrypt the given data and store in out buffer.
 */
cy_rslt_t cy_tls_encrypt_data(cy_tls_context_t* tls_context, uint8_t* out, uint8_t* in, uint32_t* in_out_length)
{
    cy_rslt_t result;
    NX_PACKET *packet = NULL;
    NX_PACKET_POOL *pool;
    UINT error;

    if (tls_context == NULL || tls_context->tls_handshake_successful == false)
    {
        return CY_RSLT_ENTERPRISE_SECURITY_TLS_ERROR;
    }

    /* Get packet pool to allocate a packet */
    result = cy_network_get_packet_pool(CY_NETWORK_PACKET_TX, (void *)&pool);
    if (result != CY_RSLT_SUCCESS)
    {
        TLS_WRAPPER_DEBUG( CYLF_MIDDLEWARE, CY_LOG_ERR, "Get packet pool failed with error 0x%lx\n", result);
        return CY_RSLT_ENTERPRISE_SECURITY_TLS_ERROR;
    }

    /* Get a packet from the pool to send encrypted data. */
    error = nx_secure_tls_packet_allocate(&tls_context->context, pool, &packet, CY_TLS_DEFAULT_WAIT);
    if (error != NX_SUCCESS)
    {
        TLS_WRAPPER_DEBUG( CYLF_MIDDLEWARE, CY_LOG_ERR, "TX packet allocation failed with error 0x%lx\n", error);
        return CY_RSLT_ENTERPRISE_SECURITY_TLS_ERROR;
    }

    /* Populate the packet with input data. */
    error = nx_packet_data_append(packet, (void *)(in), *in_out_length, pool, CY_TLS_DEFAULT_WAIT);
    if (error != NX_SUCCESS)
    {
        TLS_WRAPPER_DEBUG( CYLF_MIDDLEWARE, CY_LOG_ERR, "nx_packet_data_append failed with error 0x%lx\n", error);
        nx_secure_tls_packet_release(packet);
        return CY_RSLT_ENTERPRISE_SECURITY_TLS_ERROR;
    }

    /* Perform encryption */
    error = cy_tls_netx_encrypt_packet(&tls_context->context, packet, CY_TLS_DEFAULT_WAIT);
    if (error != NX_SUCCESS)
    {
        TLS_WRAPPER_DEBUG( CYLF_MIDDLEWARE, CY_LOG_ERR, "Packet encrypt failed with error 0x%lx\n", error);
        nx_secure_tls_packet_release(packet);
        return CY_RSLT_ENTERPRISE_SECURITY_TLS_ERROR;
    }

    /* Copy to out buffer */
    *in_out_length = packet->nx_packet_length;
    memcpy(out, packet->nx_packet_prepend_ptr, packet->nx_packet_length);

    /* Release the packet */
    nx_secure_tls_packet_release(packet);
    return CY_RSLT_SUCCESS;
}
