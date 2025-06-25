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

#include "cy_enterprise_security_error.h"
#include "cy_enterprise_security_log.h"
#include "cy_mschapv2.h"
#include "cy_md4.h"

#include "cy_tls_abstraction.h"
#include "cy_rtos_abstraction.h"

#ifdef COMPONENT_NETXSECURE
#include "nx_crypto_des.h"
#include "nx_crypto_sha1.h"
extern NX_CRYPTO_METHOD crypto_method_des;
extern NX_CRYPTO_METHOD crypto_method_sha1;
#endif

/******************************************************
 *                      Macros
 ******************************************************/
#define CY_SUPPLICANT_MSCHAPV2_DEBUG  cy_enterprise_security_log_msg

/******************************************************
 *                      Constants
 ******************************************************/
#define SHA1_LENGTH (20)

/******************************************************
 *                Functions Definations
 ******************************************************/

cy_rslt_t mschap_challenge_hash(uint8_t* peer_challenge, uint8_t* authenticator_challenge, char* user_name, uint8_t* challenge)
{
    uint8_t hash_value[SHA1_LENGTH];
#ifdef COMPONENT_MBEDTLS
    mbedtls_sha1_context sha1_ctx;

    CY_SUPPLICANT_MSCHAPV2_DEBUG(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "\r\n %s %s %d \r\n",__FILE__,__FUNCTION__,__LINE__);

    mbedtls_sha1_init( &sha1_ctx );
    mbedtls_sha1_starts_ret( &sha1_ctx );
    mbedtls_sha1_update_ret( &sha1_ctx, (unsigned char *)peer_challenge, 16 );
    mbedtls_sha1_update_ret( &sha1_ctx, (unsigned char *)authenticator_challenge, 16 );
    mbedtls_sha1_update_ret( &sha1_ctx, (unsigned char *)user_name, strlen((const char*)user_name) );
    mbedtls_sha1_finish_ret( &sha1_ctx, (unsigned char *)hash_value );
    mbedtls_sha1_free( &sha1_ctx );

#elif defined (COMPONENT_NETXSECURE)
    UINT status;
    UCHAR *meta_ptr;
    VOID *handler = NX_NULL;
    NX_CRYPTO_METHOD *method = &crypto_method_sha1;
    UINT metadata_sz = method->nx_crypto_metadata_area_size;

    meta_ptr = (UCHAR*)cy_rtos_malloc(metadata_sz);
    if (meta_ptr == NULL)
    {
        CY_SUPPLICANT_MSCHAPV2_DEBUG(CYLF_MIDDLEWARE, CY_LOG_ERR, "Meta PTR allocation Failure\n");
        return CY_RSLT_ENTERPRISE_SECURITY_TLS_ERROR;
    }

    status = method -> nx_crypto_init((NX_CRYPTO_METHOD*)method,
                                        NULL,
                                        0,
                                        &handler,
                                        meta_ptr,
                                        metadata_sz);
    if (status != NX_CRYPTO_SUCCESS)
    {
        CY_SUPPLICANT_MSCHAPV2_DEBUG(CYLF_MIDDLEWARE, CY_LOG_ERR, "SHA1: Crypto Init Failed\n");
        cy_rtos_free(meta_ptr);
        return(status);
    }


   /* Hash init. */
   status = method -> nx_crypto_operation(NX_CRYPTO_HASH_INITIALIZE,
                                        handler,
                                        method,
                                        NULL,
                                        0,
                                        NULL,
                                        0,
                                        NULL,
                                        NULL,
                                        0,
                                        meta_ptr,
                                        metadata_sz,
                                        NX_NULL, NX_NULL);
    if (status != NX_CRYPTO_SUCCESS)
    {
        CY_SUPPLICANT_MSCHAPV2_DEBUG(CYLF_MIDDLEWARE, CY_LOG_ERR, "SHA1: Hash Init Failed\n");
        cy_rtos_free(meta_ptr);
        return(status);
    }

    status = method -> nx_crypto_operation(NX_CRYPTO_HASH_UPDATE,
                                        handler,
                                        method,
                                        NULL,
                                        0,
                                        (UCHAR *)peer_challenge,
                                        16,
                                        NULL,
                                        NULL,
                                        0,
                                        meta_ptr,
                                        metadata_sz,
                                        NX_NULL, NX_NULL);
    if (status != NX_CRYPTO_SUCCESS)
    {
        cy_rtos_free(meta_ptr);
        return(status);
    }

    status = method -> nx_crypto_operation(NX_CRYPTO_HASH_UPDATE,
                                    handler,
                                    method,
                                    NULL,
                                    0,
                                    (UCHAR *)authenticator_challenge,
                                    16,
                                    NULL,
                                    NULL,
                                    0,
                                    meta_ptr,
                                    metadata_sz,
                                    NX_NULL, NX_NULL);
    if (status != NX_CRYPTO_SUCCESS)
    {
        cy_rtos_free(meta_ptr);
        return(status);
    }

    status = method -> nx_crypto_operation(NX_CRYPTO_HASH_UPDATE,
                                    handler,
                                    method,
                                    NULL,
                                    0,
                                    (UCHAR *)user_name,
                                    strlen((const CHAR*)user_name),
                                    NULL,
                                    NULL,
                                    0,
                                    meta_ptr,
                                    metadata_sz,
                                    NX_NULL, NX_NULL);
    if (status != NX_CRYPTO_SUCCESS)
    {
        cy_rtos_free(meta_ptr);
        return(status);
    }

    status = method -> nx_crypto_operation(NX_CRYPTO_HASH_CALCULATE,
                                    handler,
                                    method,
                                    NULL,
                                    0,
                                    NULL,
                                    0,
                                    NULL,
                                    hash_value,
                                    SHA1_LENGTH,
                                    meta_ptr,
                                    metadata_sz,
                                    NX_NULL, NX_NULL);
    if (status != NX_CRYPTO_SUCCESS)
    {
        CY_SUPPLICANT_MSCHAPV2_DEBUG(CYLF_MIDDLEWARE, CY_LOG_ERR, "SHA1: Hash Calculate Failed\n");
        cy_rtos_free(meta_ptr);
        return(status);
    }

    if (method -> nx_crypto_cleanup)
    {
        status = method -> nx_crypto_cleanup(meta_ptr);
    }
    cy_rtos_free(meta_ptr);
#endif
    memcpy( challenge, hash_value, 8);
    return CY_RSLT_SUCCESS;
}

cy_rslt_t mschap_nt_password_hash( char* password, uint16_t length, uint8_t* password_hash )
{
    cy_rslt_t ret = cy_md4_func((uint8_t*)password, length, (uint8_t*)password_hash);
    return ret;
}

cy_rslt_t mschap_permute_key(uint8_t* key56, uint8_t* key64)
{
    int i, shift;
    key64[0] = key56[0] & 0xFE;
    CY_SUPPLICANT_MSCHAPV2_DEBUG(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "\r\n %s %s %d \r\n",__FILE__,__FUNCTION__,__LINE__);

    for (i = 1, shift = 1; i < 8; i++, shift++)
    {
        key64[i] = 0;
        key64[i] = ( key56[i-1] << (8-shift) ) | ( key56[i] >> shift);
    }

    return CY_RSLT_SUCCESS;
}



cy_rslt_t mschap_des_encrypt( uint8_t* clear, uint8_t* key, uint8_t* cypher)
{
    uint8_t key64[ 8 ];
#ifdef COMPONENT_MBEDTLS
    mbedtls_des_context des_ctx;

    CY_SUPPLICANT_MSCHAPV2_DEBUG(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "\r\n %s %s %d \r\n",__FILE__,__FUNCTION__,__LINE__);

    mschap_permute_key( key, key64 );
    mbedtls_des_init( &des_ctx );
    mbedtls_des_setkey_enc( &des_ctx, key64 );
    mbedtls_des_crypt_ecb(&des_ctx, clear, cypher);
    mbedtls_des_free( &des_ctx );

#elif defined (COMPONENT_NETXSECURE)
    UINT status;
    UCHAR *meta_ptr;
    VOID *handler = NX_NULL;
    NX_CRYPTO_METHOD *method = &crypto_method_des;
    UINT metadata_sz = method->nx_crypto_metadata_area_size;

    CY_SUPPLICANT_MSCHAPV2_DEBUG(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "\r\n %s %s %d \r\n",__FILE__,__FUNCTION__,__LINE__);

    mschap_permute_key( key, key64 );

    meta_ptr = (UCHAR*)cy_rtos_malloc(metadata_sz);
    if(meta_ptr == NULL)
    {
        CY_SUPPLICANT_MSCHAPV2_DEBUG(CYLF_MIDDLEWARE, CY_LOG_ERR, "Meta PTR allocation Failure\n");
        return CY_RSLT_ENTERPRISE_SECURITY_TLS_ERROR;
    }

    status = method -> nx_crypto_init((NX_CRYPTO_METHOD*)method,
                                                        key64,
                                                        NX_CRYPTO_DES_KEY_LEN_IN_BITS,
                                                        &handler,
                                                        meta_ptr,
                                                        metadata_sz);
    if (status != NX_CRYPTO_SUCCESS)
    {
        CY_SUPPLICANT_MSCHAPV2_DEBUG(CYLF_MIDDLEWARE, CY_LOG_ERR, "DES: Crypto Init Failure\n");
        cy_rtos_free(meta_ptr);
        return(status);
    }

    status = _nx_crypto_des_encrypt((NX_CRYPTO_DES*)meta_ptr, clear, cypher, 1);
    if (status != NX_CRYPTO_SUCCESS)
    {
        CY_SUPPLICANT_MSCHAPV2_DEBUG(CYLF_MIDDLEWARE, CY_LOG_ERR, "DES: Encrypt Failure\n");
        cy_rtos_free(meta_ptr);
        return(status);
    }

    if (method -> nx_crypto_cleanup)
    {
        status = method -> nx_crypto_cleanup(meta_ptr);
    }
    cy_rtos_free(meta_ptr);
#endif
    return CY_RSLT_SUCCESS;
}

cy_rslt_t mschap_challenge_response( uint8_t* challenge, uint8_t* nt_password_hash, uint8_t* nt_response )
{
    int pwd_index =0, rsp_index = 0;
    uint8_t  z_password_hash[21 + 1]={0};
    CY_SUPPLICANT_MSCHAPV2_DEBUG(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "\r\n %s %s %d \r\n",__FILE__,__FUNCTION__,__LINE__);

    memcpy(z_password_hash, nt_password_hash, 16);

    mschap_des_encrypt( challenge, &z_password_hash[pwd_index], &nt_response[rsp_index] );
    pwd_index += 7;
    rsp_index += 8;
    mschap_des_encrypt( challenge, &z_password_hash[pwd_index], &nt_response[rsp_index] );
    pwd_index += 7;
    rsp_index += 8;
    mschap_des_encrypt( challenge, &z_password_hash[pwd_index], &nt_response[rsp_index] );

    return CY_RSLT_SUCCESS;
}

cy_rslt_t mschap_generate_nt_response(uint8_t* authenticator_challenge, uint8_t* peer_challenge, char* user_name, char* password, uint16_t password_length, uint8_t* nt_response)
{
    uint8_t challenge[8];
    uint8_t password_hash[16];
    CY_SUPPLICANT_MSCHAPV2_DEBUG(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "\r\n %s %s %d \r\n",__FILE__,__FUNCTION__,__LINE__);

    memset(password_hash, 0, sizeof(password_hash));
    memset(challenge, 0, sizeof(challenge));

    mschap_challenge_hash( peer_challenge, authenticator_challenge, user_name, challenge);
    mschap_nt_password_hash( password, password_length, password_hash );
    mschap_challenge_response( challenge, password_hash, nt_response );

    return CY_RSLT_SUCCESS;
}

cy_rslt_t mschap_process_packet( mschapv2_packet_t *packet, supplicant_workspace_t *workspace )
{
    supplicant_phase2_state_t* phase2 = (supplicant_phase2_state_t*)workspace->ptr_phase2;

    CY_SUPPLICANT_MSCHAPV2_DEBUG(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "\r\n %s %d \r\n",__FUNCTION__,__LINE__);

    if ( packet->opcode == MSCHAPV2_OPCODE_CHALLENGE )
    {
        mschapv2_response_packet_t*  response;
        supplicant_packet_t          response_packet    = NULL;
        mschapv2_challenge_packet_t* challenge          = (mschapv2_challenge_packet_t*) packet;
        uint16_t                     packet_size        = sizeof( mschapv2_response_packet_t ) + phase2->identity_length -1;
        uint8_t                      peer_challenge[16] = {0};
        uint16_t                     aligned_length;

        CY_SUPPLICANT_MSCHAPV2_DEBUG(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "\r\n %s %d \r\n",__FUNCTION__,__LINE__);

        cy_crypto_get_random( workspace->tls_context, peer_challenge, sizeof( peer_challenge ) );

        if (workspace->eap_type == CY_ENTERPRISE_SECURITY_EAP_TYPE_PEAP)
        {
            supplicant_create_peap_response_packet( &response_packet, CY_ENTERPRISE_SECURITY_EAP_TYPE_MSCHAPV2, packet_size, workspace->tls_length_overhead, workspace);
        }
        else if(workspace->eap_type == CY_ENTERPRISE_SECURITY_EAP_TYPE_TTLS)
        {
            supplicant_create_ttls_response_packet( &response_packet, CY_ENTERPRISE_SECURITY_EAP_TYPE_MSCHAPV2, packet_size, workspace->tls_length_overhead, workspace);
        }
        if ( response_packet == NULL )
        {
            CY_SUPPLICANT_MSCHAPV2_DEBUG(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "\r\n %s %d \r\n",__FUNCTION__,__LINE__);
            return CY_RSLT_ENTERPRISE_SECURITY_SUPPLICANT_UNPROCESSED;
        }
        response = (mschapv2_response_packet_t *)supplicant_host_get_data( workspace->interface->whd_driver,response_packet );
        response->opcode     = 2;
        response->id         = challenge->id;
        response->value_size = 0x31;
        memcpy( response->peer_challenge, peer_challenge, sizeof(peer_challenge));
        memset( response->reserved, 0, sizeof(response->reserved ));
        memcpy( response->name, phase2->identity, phase2->identity_length);
        SUPPLICANT_WRITE_16_BE( &aligned_length, packet_size );
        response->length = aligned_length;
        CY_SUPPLICANT_MSCHAPV2_DEBUG(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "\r\n %s %d \r\n",__FUNCTION__,__LINE__);

        CY_SUPPLICANT_MSCHAPV2_DEBUG(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "\r\nphase2->identity %s len %d \r\n", phase2->identity, phase2->identity_length);
        /* coverity[overrun-buffer-val]
            FALSE-POSITIVE: peap->password_length never exceeds 32. So parameter 'ilen' passed to md4_update() won't be more than 32.
            Hence, 'ilen' >= 64 condition will never be true. And 'input' will not be indexed at byte position 63 as reported by coverity */
        mschap_generate_nt_response( challenge->challenge, peer_challenge, (char*)phase2->identity, (char*)phase2->password, phase2->password_length, response->nt_reponse);

        if (workspace->eap_type == CY_ENTERPRISE_SECURITY_EAP_TYPE_PEAP)
        {
            CY_SUPPLICANT_MSCHAPV2_DEBUG(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "\r\n %s %d \r\n",__FUNCTION__,__LINE__);
            supplicant_send_peap_response_packet( &response_packet, workspace );
        }
        else if(workspace->eap_type == CY_ENTERPRISE_SECURITY_EAP_TYPE_TTLS)
        {
            CY_SUPPLICANT_MSCHAPV2_DEBUG(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "\r\n %s %d \r\n",__FUNCTION__,__LINE__);
            supplicant_send_ttls_response_packet( &response_packet, workspace );
        }
    }
    else if ( packet->opcode == MSCHAPV2_OPCODE_SUCCESS )
    {
        mschapv2_success_response_packet_t* response;
        supplicant_packet_t                       response_packet = NULL;
        uint16_t                            packet_size   =  sizeof( mschapv2_success_response_packet_t );

        CY_SUPPLICANT_MSCHAPV2_DEBUG(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "\r\n %s %d \r\n",__FUNCTION__,__LINE__);

        if (workspace->eap_type == CY_ENTERPRISE_SECURITY_EAP_TYPE_PEAP)
        {
            CY_SUPPLICANT_MSCHAPV2_DEBUG(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "\r\n %s %d \r\n",__FUNCTION__,__LINE__);
            supplicant_create_peap_response_packet( &response_packet, CY_ENTERPRISE_SECURITY_EAP_TYPE_MSCHAPV2, packet_size, workspace->tls_length_overhead, workspace);
        }
        else if(workspace->eap_type == CY_ENTERPRISE_SECURITY_EAP_TYPE_TTLS)
        {
            supplicant_create_ttls_response_packet( &response_packet, CY_ENTERPRISE_SECURITY_EAP_TYPE_MSCHAPV2, packet_size, workspace->tls_length_overhead, workspace);
        }

        if ( response_packet == NULL )
        {
            CY_SUPPLICANT_MSCHAPV2_DEBUG(CYLF_MIDDLEWARE, CY_LOG_ERR, "\r\n %s %d \r\n",__FUNCTION__,__LINE__);
            return CY_RSLT_ENTERPRISE_SECURITY_SUPPLICANT_UNPROCESSED;
        }

        response = (mschapv2_success_response_packet_t *)supplicant_host_get_data( workspace->interface->whd_driver,response_packet );
        response->opcode = 3;

        if (workspace->eap_type == CY_ENTERPRISE_SECURITY_EAP_TYPE_PEAP)
        {
            CY_SUPPLICANT_MSCHAPV2_DEBUG(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "\r\n %s %d \r\n",__FUNCTION__,__LINE__);
            supplicant_send_peap_response_packet( &response_packet, workspace );
        }
        else if(workspace->eap_type == CY_ENTERPRISE_SECURITY_EAP_TYPE_TTLS)
        {
            supplicant_send_ttls_response_packet( &response_packet, workspace );
            phase2->result = CY_RSLT_SUCCESS;
        }
    }
    else if ( packet->opcode == MSCHAPV2_OPCODE_FAILURE )
    {
        CY_SUPPLICANT_MSCHAPV2_DEBUG(CYLF_MIDDLEWARE, CY_LOG_ERR, "\r\n %s %d \r\n",__FUNCTION__,__LINE__);
    }
    else
    {
        CY_SUPPLICANT_MSCHAPV2_DEBUG(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "\r\n %s %d \r\n",__FUNCTION__,__LINE__);
        return CY_RSLT_ENTERPRISE_SECURITY_SUPPLICANT_UNPROCESSED;
    }
    CY_SUPPLICANT_MSCHAPV2_DEBUG(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "\r\n %s %d \r\n",__FUNCTION__,__LINE__);

    return CY_RSLT_SUCCESS;
}
