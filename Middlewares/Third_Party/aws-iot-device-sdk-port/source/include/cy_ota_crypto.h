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
 * @file cy_ota_crypto.h
 *  Function declarations for cryptographic APIs for AWS OTA.
 */

#ifndef CY_OTA_CRYPTO_H_
#define CY_OTA_CRYPTO_H_

#include "cyabs_rtos.h"
#include "cy_aws_iot_port_error.h"
#include "version.h"

#ifdef COMPONENT_MBEDTLS

#include "mbedtls/platform.h"
#include "mbedtls/sha256.h"
#include "mbedtls/sha1.h"
#include "mbedtls/pk.h"
#include "mbedtls/x509_crt.h"

#endif
/**
 * Commonly used buffer sizes for storing cryptographic hash computation results.
 */
#define CY_CRYPTO_SHA1_DIGEST_BYTES              ( 20 )
#define CY_CRYPTO_SHA256_DIGEST_BYTES            ( 32 )

/**
 * Library-independent cryptographic algorithm identifiers.
 */
#define CY_CRYPTO_HASH_ALGORITHM_SHA1            ( 1 )
#define CY_CRYPTO_HASH_ALGORITHM_SHA256          ( 2 )
#define CY_CRYPTO_ASYMMETRIC_ALGORITHM_RSA       ( 1 )
#define CY_CRYPTO_ASYMMETRIC_ALGORITHM_ECDSA     ( 2 )

/**
 * Initializes digital signature verification.
 *
 * @param[out] ppvContext Opaque context structure.
 * @param[in] xAsymmetricAlgorithm Cryptographic public key cryptosystem.
 * @param[in] xHashAlgorithm Cryptographic hash algorithm that was used for signing.
 *
 * @return CY_RSLT_SUCCESS if initialization succeeds, or CY_OTA_CRYPTO_FAIL otherwise.
 */
cy_rslt_t cy_crypto_sign_verification_start( void ** ppvContext,
                                             long xAsymmetricAlgorithm,
                                             long xHashAlgorithm );

/**
 * Updates a cryptographic hash computation with the specified byte array.
 *
 * @param[in] pvContext Opaque context structure.
 * @param[in] pucData Byte array that was signed.
 * @param[in] xDataLength Length in bytes of data that was signed.
 */
void cy_crypto_sign_verification_update( void * pvContext,
                                         const uint8_t * pucData,
                                         size_t xDataLength );

/**
 * @brief Verifies a digital signature computation using the public key from the
 * specified certificate.
 *
 * @param[in] pvContext Opaque context structure.
 * @param[in] pucSignerCertificate Base64 and DER encoded X.509 certificate of the
 * signer.
 * @param[in] xSignerCertificateLength Length in bytes of the certificate.
 * @param[in] pucSignature Digital signature result to verify.
 * @param[in] xSignatureLength in bytes of digital signature result.
 *
 * @return CY_RSLT_SUCCESS if the signature is correct or CY_OTA_CRYPTO_FAIL if the signature is invalid.
 */
cy_rslt_t cy_crypto_sign_verification_final( void * pvContext,
                                             char * pcSignerCertificate,
                                             size_t xSignerCertificateLength,
                                             uint8_t * pucSignature,
                                             size_t xSignatureLength );

/**
 * Validates the certificate using the signer certificate.
 *
 * @param[in] pCertificate       Certificate to validate
 * @param[in] xCertificateLength Length in bytes of the certificate
 * @param[in] pSignerCertificate Signer certificate
 * @param[in] pSignerCertificate Length in bytes of the signer certificate
 *
 * @return CY_RSLT_SUCCESS if certificate verification succeeds, or failure code otherwise.
 */
cy_rslt_t cy_crypto_validate_cert( char *pCertificate,
                                   size_t xCertificateLength,
                                   char * pSignerCertificate,
                                   size_t xSignerCertificateLength );

#endif /* ifndef CY_OTA_CRYPTO_H_ */
