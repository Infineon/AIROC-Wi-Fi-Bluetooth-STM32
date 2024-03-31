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

/**
 * @file cy_aws_ota_crypto.c
 *  Implements cryptographic APIs for AWS OTA.
 */

#include <string.h>
#include "cy_ota_crypto.h"
#include "cy_aws_iot_sdk_port_log.h"
#include "cy_aws_port_mbedtls_version.h"

/**
 * Internal signature verification context structure
 */
typedef struct SignatureVerificationState
{
    long xAsymmetricAlgorithm;
    long xHashAlgorithm;
    union
    {
        mbedtls_sha1_context xSHA1Context;
        mbedtls_sha256_context xSHA256Context;
    } xSHA_Context;
} SignatureVerificationState_t, *SignatureVerificationStatePtr_t;

/*-----------------------------------------------------------*/
/**
 * Verifies a cryptographic signature based on the signer
 * certificate, hash algorithm, and the data that was signed.
 */
static cy_rslt_t cy_crypto_verify_sign( char * pcSignerCertificate,
                                        size_t xSignerCertificateLength,
                                        long xHashAlgorithm,
                                        uint8_t * pucHash,
                                        size_t xHashLength,
                                        uint8_t * pucSignature,
                                        size_t xSignatureLength )
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    mbedtls_x509_crt xCertCtx;
    mbedtls_md_type_t xMbedHashAlg = MBEDTLS_MD_SHA256;

    memset( &xCertCtx, 0, sizeof( mbedtls_x509_crt ) );

    /* Map the hash algorithm */
    if( CY_CRYPTO_HASH_ALGORITHM_SHA1 == xHashAlgorithm )
    {
        xMbedHashAlg = MBEDTLS_MD_SHA1;
    }

    /* Decode and create a certificate context */
    mbedtls_x509_crt_init( &xCertCtx );

    if( mbedtls_x509_crt_parse( &xCertCtx, ( const unsigned char * ) pcSignerCertificate, xSignerCertificateLength ) == 0 )
    {
        /* Verify the signature using the public key from the decoded certificate */
        if( mbedtls_pk_verify( &xCertCtx.pk, xMbedHashAlg, pucHash, xHashLength, pucSignature, xSignatureLength ) != 0 )
        {
            result = CY_RSLT_AWS_IOT_PORT_ERROR_CRYPTO_VERIFY_SIGN_FAIL;
            cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Failed to verify public key.\r\n" );
        }
    }
    else
    {
        result = CY_RSLT_AWS_IOT_PORT_ERROR_CRYPTO_VERIFY_SIGN_FAIL;
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Failed to parse mbedTLS x509 certificate.\r\n" );
    }

    /* Clean-up */
    mbedtls_x509_crt_free( &xCertCtx );

    return result;
}

/*-----------------------------------------------------------*/
/**
 * @brief Creates signature verification context.
 */
cy_rslt_t cy_crypto_sign_verification_start( void **ppvContext,
                                             long xAsymmetricAlgorithm,
                                             long xHashAlgorithm )
{
    SignatureVerificationState_t * pxCtx = NULL;

    if( ppvContext == NULL )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Invalid argument to cy_crypto_sign_verification_start. \r\n" );
        return CY_RSLT_AWS_IOT_PORT_ERROR_BADARG;
    }

    /* Allocate the context  */
    if( NULL == ( pxCtx = ( SignatureVerificationStatePtr_t ) malloc( sizeof( *pxCtx ) ) ) )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Memory allocation failed in cy_crypto_sign_verification_start. \r\n" );
        return CY_RSLT_AWS_IOT_PORT_ERROR_OUT_OF_MEMORY;
    }

    *ppvContext = pxCtx;

    /* Store the algorithm identifiers */
    pxCtx->xAsymmetricAlgorithm = xAsymmetricAlgorithm;
    pxCtx->xHashAlgorithm = xHashAlgorithm;

    /* Initialize the requested hash type */
    if( CY_CRYPTO_HASH_ALGORITHM_SHA1 == pxCtx->xHashAlgorithm )
    {
        mbedtls_sha1_init( &(pxCtx->xSHA_Context.xSHA1Context) );

#if MBEDTLS_VERSION_MAJOR == MBEDTLS_VERSION_MAJOR_3
        ( void ) mbedtls_sha1_starts( &(pxCtx->xSHA_Context.xSHA1Context) );
#else
        ( void ) mbedtls_sha1_starts_ret( &(pxCtx->xSHA_Context.xSHA1Context) );
#endif
    }
    else
    {
        mbedtls_sha256_init( &(pxCtx->xSHA_Context.xSHA256Context) );
#if MBEDTLS_VERSION_MAJOR == MBEDTLS_VERSION_MAJOR_3
        ( void ) mbedtls_sha256_starts( &(pxCtx->xSHA_Context.xSHA256Context), 0 );
#else
        ( void ) mbedtls_sha256_starts_ret( &(pxCtx->xSHA_Context.xSHA256Context), 0 );
#endif
    }

    return CY_RSLT_SUCCESS;
}

/*-----------------------------------------------------------*/
/**
 * @brief Adds bytes to an in-progress hash for subsequent signature
 * verification.
 */
void cy_crypto_sign_verification_update( void *pvContext,
                                         const uint8_t *pucData,
                                         size_t xDataLength )
{
    SignatureVerificationState_t *pxCtx = NULL;

    if( (pvContext == NULL) || (pucData == NULL) || (xDataLength == 0) )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Invalid argument to cy_crypto_sign_verification_update. \r\n" );
    }
    else
    {
        pxCtx = ( SignatureVerificationStatePtr_t ) pvContext; /*lint !e9087 Allow casting void* to other types. */

        /* Add the data to the hash of the requested type */
        if( CY_CRYPTO_HASH_ALGORITHM_SHA1 == pxCtx->xHashAlgorithm )
        {
#if MBEDTLS_VERSION_MAJOR == MBEDTLS_VERSION_MAJOR_3
            ( void ) mbedtls_sha1_update( &(pxCtx->xSHA_Context.xSHA1Context), pucData, xDataLength );
#else
            ( void ) mbedtls_sha1_update_ret( &(pxCtx->xSHA_Context.xSHA1Context), pucData, xDataLength );
#endif
        }
        else
        {
#if MBEDTLS_VERSION_MAJOR == MBEDTLS_VERSION_MAJOR_3
            ( void ) mbedtls_sha256_update( &(pxCtx->xSHA_Context.xSHA256Context), pucData, xDataLength );
#else
            ( void ) mbedtls_sha256_update_ret( &(pxCtx->xSHA_Context.xSHA256Context), pucData, xDataLength );
#endif
        }
    }
}

/*-----------------------------------------------------------*/
/**
 * @brief Performs signature verification on a cryptographic hash.
 */
cy_rslt_t cy_crypto_sign_verification_final( void * pvContext,
                                             char * pcSignerCertificate,
                                             size_t xSignerCertificateLength,
                                             uint8_t *pucSignature,
                                             size_t xSignatureLength )
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    if( pvContext == NULL )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Invalid argument to cy_crypto_sign_verification_final. \r\n" );
        result = CY_RSLT_AWS_IOT_PORT_ERROR_BADARG;
    }
    else
    {
        SignatureVerificationStatePtr_t pxCtx = ( SignatureVerificationStatePtr_t ) pvContext;
        /* Reserve enough space for the larger of SHA1 or SHA256 results. */
        uint8_t ucSHA1or256[ CY_CRYPTO_SHA256_DIGEST_BYTES ];
        uint8_t *pucHash = NULL;
        size_t xHashLength = 0;

        if( ( pcSignerCertificate != NULL ) &&
            ( pucSignature != NULL ) &&
            ( xSignerCertificateLength > 0UL ) &&
            ( xSignatureLength > 0UL ) )
        {
            /* Finish the hash */
            if( CY_CRYPTO_HASH_ALGORITHM_SHA1 == pxCtx->xHashAlgorithm )
            {
#if MBEDTLS_VERSION_MAJOR == MBEDTLS_VERSION_MAJOR_3
                ( void ) mbedtls_sha1_finish( &(pxCtx->xSHA_Context.xSHA1Context), ucSHA1or256 );
#else
                ( void ) mbedtls_sha1_finish_ret( &(pxCtx->xSHA_Context.xSHA1Context), ucSHA1or256 );
#endif
                pucHash = ucSHA1or256;
                xHashLength = CY_CRYPTO_SHA1_DIGEST_BYTES;
            }
            else
            {
#if MBEDTLS_VERSION_MAJOR == MBEDTLS_VERSION_MAJOR_3
                ( void ) mbedtls_sha256_finish( &(pxCtx->xSHA_Context.xSHA256Context), ucSHA1or256 );
#else
                ( void ) mbedtls_sha256_finish_ret( &(pxCtx->xSHA_Context.xSHA256Context), ucSHA1or256 );
#endif
                pucHash = ucSHA1or256;
                xHashLength = CY_CRYPTO_SHA256_DIGEST_BYTES;
            }

            /* Verify the signature */
            result = cy_crypto_verify_sign( pcSignerCertificate, xSignerCertificateLength,
                                            pxCtx->xHashAlgorithm, pucHash,
                                            xHashLength, pucSignature, xSignatureLength );
        }
        else
        {
            /* Allow function to be called with only the context pointer for cleanup after a failure. */
        }

        /* Clean-up */
        if( pxCtx != NULL )
        {
            free( pxCtx );
            pxCtx = NULL;
        }
    }

    return result;
}

/*-----------------------------------------------------------*/
/**
 * @brief Validate Certificate using the signer certificate
 */
cy_rslt_t cy_crypto_validate_cert( char *pCertificate,
                                   size_t xCertificateLength,
                                   char * pSignerCertificate,
                                   size_t xSignerCertificateLength )
{
    mbedtls_x509_crt    cacert;
    mbedtls_x509_crt    cert;
    cy_rslt_t           result  = CY_RSLT_AWS_IOT_PORT_ERROR_GENERAL;
    int                 ret     = 0;
    uint32_t            flags   = 0;

    if( ( pCertificate == NULL ) || ( pSignerCertificate == NULL ) )
    {
        return ( CY_RSLT_AWS_IOT_PORT_ERROR_BADARG );
    }

    mbedtls_x509_crt_init( &cacert );
    mbedtls_x509_crt_init( &cert );

    ret = mbedtls_x509_crt_parse( &cert, (const unsigned char *) pCertificate,
                                  xCertificateLength );
    if( ret != 0 )
    {
        goto cleanup;
    }

    ret = mbedtls_x509_crt_parse( &cacert, (const unsigned char *) pSignerCertificate,
                                  xSignerCertificateLength );
    if( ret != 0 )
    {
        goto cleanup;
    }

    ret = mbedtls_x509_crt_verify( &cert, &cacert, NULL, NULL, &flags, NULL, NULL );
    if( ret != 0 )
    {
        result = CY_RSLT_AWS_IOT_PORT_ERROR_CRYPTO_VERIFY_SIGN_FAIL;
        goto cleanup;
    }
    result = CY_RSLT_SUCCESS;

cleanup:
    mbedtls_x509_crt_free( &cacert );
    mbedtls_x509_crt_free( &cert );
    return ( result );
}


/*-----------------------------------------------------------*/
