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

#include "cy_tls_abstraction.h"
#include "cy_rtos_abstraction.h"
#include "cy_supplicant_structures.h"
#include "cy_supplicant_process_et.h"
#include "cy_enterprise_security_internal.h"
#include "cy_enterprise_security_log.h"
#include "cy_enterprise_security_error.h"
#include "whd_wifi_api.h"
#include "cybsp_wifi.h"

/******************************************************
 *              Macros
 ******************************************************/

/******************************************************
 *              Variables
 ******************************************************/

/******************************************************
 *              Function definitions
 ******************************************************/
static whd_security_t convert_to_whd_security_type(cy_enterprise_security_auth_t supplicant_security);

cy_rslt_t cy_enterprise_security_create(cy_enterprise_security_t *handle, cy_enterprise_security_parameters_t *ent_parameters)
{
    cy_supplicant_instance_t *supplicant_instance;

    if( ent_parameters == NULL || handle == NULL)
    {
         cy_enterprise_security_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Enterprise Security create parameters are NULL.\n");
         return CY_RSLT_ENTERPRISE_SECURITY_BADARG;
    }

    *handle = NULL;
    supplicant_instance = malloc( sizeof( cy_supplicant_instance_t ) );
    if( supplicant_instance == NULL )
    {
        cy_enterprise_security_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Malloc for Supplicant instance failed.\n");
        return CY_RSLT_ENTERPRISE_SECURITY_NOMEM;
    }

    /* Clear supplicant instance data. */
    memset( supplicant_instance, 0x00, sizeof( cy_supplicant_instance_t ) );

    if ( ent_parameters->ca_cert )
    {
        supplicant_instance->tls_security.ca_cert = ent_parameters->ca_cert;
        supplicant_instance->tls_security.ca_cert_len = strlen( ent_parameters->ca_cert );
    }
    if ( ent_parameters->client_cert )
    {
        supplicant_instance->tls_security.cert = ent_parameters->client_cert;
        supplicant_instance->tls_security.cert_len = strlen( ent_parameters->client_cert );
    }
    if ( ent_parameters->client_key )
    {
        supplicant_instance->tls_security.key = ent_parameters->client_key;
        supplicant_instance->tls_security.key_len = strlen( ent_parameters->client_key );
    }

    /* Copy SSID */
    strncpy( supplicant_instance->ssid, ent_parameters->ssid, SSID_NAME_SIZE );
    supplicant_instance->ssid[ SSID_NAME_SIZE - 1 ] = '\0';

    /* Copy EAP identity */
    strncpy( supplicant_instance->outer_eap_identity, (const char *)ent_parameters->outer_eap_identity, CY_ENTERPRISE_SECURITY_MAX_IDENTITY_LENGTH );
    supplicant_instance->outer_eap_identity[ CY_ENTERPRISE_SECURITY_MAX_IDENTITY_LENGTH - 1 ] = '\0';
    supplicant_instance->outer_eap_identity_length = strlen( supplicant_instance->outer_eap_identity ) + 1;

    /* Copy EAP and Auth type */
    supplicant_instance->eap_type = ent_parameters->eap_type;
    supplicant_instance->auth_type = ent_parameters->auth_type;

    if( ent_parameters->eap_type == CY_ENTERPRISE_SECURITY_EAP_TYPE_PEAP )
    {
        supplicant_instance->phase2_config.tunnel_auth_type = ent_parameters->phase2.tunnel_auth_type;

        /* Copy PEAP identiy */
        strncpy( supplicant_instance->phase2_config.tunnel_protocol.peap.inner_identity.identity,
                ent_parameters->phase2.inner_identity, CY_ENTERPRISE_SECURITY_MAX_IDENTITY_LENGTH );
        supplicant_instance->phase2_config.tunnel_protocol.peap.inner_identity.identity[ CY_ENTERPRISE_SECURITY_MAX_IDENTITY_LENGTH - 1 ] = '\0';
        supplicant_instance->phase2_config.tunnel_protocol.peap.inner_identity.identity_length =
                strlen( supplicant_instance->phase2_config.tunnel_protocol.peap.inner_identity.identity ) + 1;

        /* Copy PEAP password */
        strncpy( supplicant_instance->phase2_config.tunnel_protocol.peap.inner_identity.password,
                ent_parameters->phase2.inner_password, CY_ENTERPRISE_SECURITY_MAX_PASSWORD_LENGTH );
        supplicant_instance->phase2_config.tunnel_protocol.peap.inner_identity.password[ CY_ENTERPRISE_SECURITY_MAX_PASSWORD_LENGTH - 1 ] = '\0';
        supplicant_instance->phase2_config.tunnel_protocol.peap.inner_identity.password_length =
                strlen( supplicant_instance->phase2_config.tunnel_protocol.peap.inner_identity.password ) + 1;
    }
    else if( ent_parameters->eap_type == CY_ENTERPRISE_SECURITY_EAP_TYPE_TTLS )
    {
        supplicant_instance->phase2_config.tunnel_auth_type = ent_parameters->phase2.tunnel_auth_type;
        supplicant_instance->phase2_config.tunnel_protocol.eap_ttls.inner_eap_type = ent_parameters->phase2.inner_eap_type;

        /* Copy EAP-TTLS identiy */
        strncpy( supplicant_instance->phase2_config.tunnel_protocol.eap_ttls.inner_identity.identity,
                ent_parameters->phase2.inner_identity, CY_ENTERPRISE_SECURITY_MAX_IDENTITY_LENGTH );
        supplicant_instance->phase2_config.tunnel_protocol.eap_ttls.inner_identity.identity[ CY_ENTERPRISE_SECURITY_MAX_IDENTITY_LENGTH - 1 ] = '\0';
        supplicant_instance->phase2_config.tunnel_protocol.eap_ttls.inner_identity.identity_length =
                strlen( supplicant_instance->phase2_config.tunnel_protocol.eap_ttls.inner_identity.identity ) + 1;

        /* Copy EAP-TTLS password */
        strncpy( supplicant_instance->phase2_config.tunnel_protocol.eap_ttls.inner_identity.password,
                ent_parameters->phase2.inner_password, CY_ENTERPRISE_SECURITY_MAX_PASSWORD_LENGTH );
        supplicant_instance->phase2_config.tunnel_protocol.eap_ttls.inner_identity.password[ CY_ENTERPRISE_SECURITY_MAX_PASSWORD_LENGTH - 1 ] = '\0';
        supplicant_instance->phase2_config.tunnel_protocol.eap_ttls.inner_identity.password_length =
                strlen( supplicant_instance->phase2_config.tunnel_protocol.eap_ttls.inner_identity.password ) + 1;
    }

    *handle = (void *)supplicant_instance;
    return CY_RSLT_SUCCESS;
}

cy_rslt_t cy_enterprise_security_delete(cy_enterprise_security_t *handle)
{
    cy_supplicant_instance_t *supplicant_obj;

    if( handle == NULL || *handle == NULL )
    {
        cy_enterprise_security_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "\n Enterprise Security handle cannot be NULL.\n" );
        return CY_RSLT_ENTERPRISE_SECURITY_BADARG;
    }

    supplicant_obj = (cy_supplicant_instance_t *)(*handle);

    /* Free the TLS session stored for session resumption */
    supplicant_free_tls_session(&supplicant_obj->saved_session);

    /* Clear supplicant instance data. */
    memset( supplicant_obj, 0x00, sizeof( cy_supplicant_instance_t ) );
    free( *handle );
    *handle = NULL;

    return CY_RSLT_SUCCESS;
}

cy_rslt_t cy_supplicant_free( cy_supplicant_instance_t *supplicant_instance )
{
    if( supplicant_instance->tls_identity != NULL )
    {
        cy_rtos_free( supplicant_instance->tls_identity );
        supplicant_instance->tls_identity = NULL;
    }

    if( supplicant_instance->tls_context != NULL )
    {
        cy_rtos_free( supplicant_instance->tls_context );
        supplicant_instance->tls_context = NULL;
    }

    if( supplicant_instance->supplicant_core.supplicant_workspace != NULL )
    {
        cy_rtos_free( supplicant_instance->supplicant_core.supplicant_workspace );
        supplicant_instance->supplicant_core.supplicant_workspace = NULL;
    }

    return CY_RSLT_SUCCESS;
}

cy_rslt_t cy_supplicant_alloc( cy_supplicant_instance_t *supplicant_instance )
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    if( supplicant_instance == NULL )
    {
        cy_enterprise_security_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Supplicant instance cannot be NULL.\n");
        return CY_RSLT_ENTERPRISE_SECURITY_BADARG;
    }

    supplicant_instance->supplicant_core.supplicant_workspace = cy_rtos_malloc( sizeof( supplicant_workspace_t ) );
    if( supplicant_instance->supplicant_core.supplicant_workspace == NULL )
    {
        cy_enterprise_security_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Failed to allocate memory for supplicant workspace.\n");
        return CY_RSLT_ENTERPRISE_SECURITY_NOMEM;
    }

    memset( supplicant_instance->supplicant_core.supplicant_workspace, 0, sizeof( supplicant_workspace_t ) );

    supplicant_instance->tls_context = cy_rtos_malloc( sizeof( cy_tls_context_t ) );
    if( supplicant_instance->tls_context == NULL )
    {
        cy_enterprise_security_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Failed to allocate memory for TLS context.\n");
        result = CY_RSLT_ENTERPRISE_SECURITY_NOMEM;
        goto SUPPLICANT_WORKSPACE_CLEAN;
    }

    memset( supplicant_instance->tls_context, 0, sizeof( cy_tls_context_t ) );

    supplicant_instance->tls_context->session = &supplicant_instance->saved_session;

    supplicant_instance->tls_identity = cy_rtos_malloc( sizeof(cy_tls_identity_t ) );
    if( supplicant_instance->tls_identity == NULL )
    {
        cy_enterprise_security_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Failed to allocate memory for TLS identity.\n");
        result = CY_RSLT_ENTERPRISE_SECURITY_NOMEM;
        goto TLS_CONTEXT_CLEAN;
    }

    memset( supplicant_instance->tls_identity,0,sizeof(cy_tls_identity_t ) );

    return result;

TLS_CONTEXT_CLEAN:
    cy_rtos_free( supplicant_instance->tls_context );
    supplicant_instance->tls_context = NULL;

SUPPLICANT_WORKSPACE_CLEAN:
    cy_rtos_free( supplicant_instance->supplicant_core.supplicant_workspace );
    supplicant_instance->supplicant_core.supplicant_workspace = NULL;

    return result;
}

cy_rslt_t cy_leave_ent( cy_supplicant_instance_t *supplicant_instance )
{
    cy_rslt_t res;
    supplicant_workspace_t *supplicant_workspace = NULL;

    if( supplicant_instance == NULL )
    {
        cy_enterprise_security_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Supplicant instance cannot be NULL.\n");
        return CY_RSLT_ENTERPRISE_SECURITY_BADARG;
    }

    supplicant_workspace = supplicant_instance->supplicant_core.supplicant_workspace;
    if( supplicant_workspace == NULL )
    {
        cy_enterprise_security_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Supplicant workspace cannot be NULL.\n");
        return CY_RSLT_ENTERPRISE_SECURITY_BADARG;
    }

    cy_enterprise_security_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "Stop supplicant\n");
    res = supplicant_stop( supplicant_workspace );
    if( res != CY_RSLT_SUCCESS )
    {
        cy_enterprise_security_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Supplicant Stop failed with error = [%u]\n", (unsigned int)res);
        return res;
    }

    cy_enterprise_security_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "De-init supplicant\n");
    res = supplicant_deinit( supplicant_workspace );
    if ( res != CY_RSLT_SUCCESS )
    {
        cy_enterprise_security_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Supplicant De-init failed with error = [%u]\n", (unsigned int)res);
        return res;
    }

    return CY_RSLT_SUCCESS;
}

cy_rslt_t cy_join_ent( cy_supplicant_instance_t *supplicant_instance )
{

    whd_driver_t whd_driver = cybsp_get_wifi_driver();
    supplicant_workspace_t *supplicant_workspace = NULL;
    whd_ap_info_t details;
    whd_security_t whd_security;
    cy_rslt_t res = CY_RSLT_SUCCESS;
    supplicant_connection_info_t conn_info;

    if( supplicant_instance == NULL )
    {
        cy_enterprise_security_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Supplicant instance cannot be NULL.\n");
        return CY_RSLT_ENTERPRISE_SECURITY_BADARG;
    }

    supplicant_workspace = supplicant_instance->supplicant_core.supplicant_workspace;

    if( supplicant_workspace == NULL || whd_driver == NULL )
    {
        cy_enterprise_security_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "\r\n Supplicant workspace or whd_driver can't be NULL\r\n");
        return CY_RSLT_ENTERPRISE_SECURITY_BADARG;
    }

    /* Modify config */
    details.SSID.length = strlen( supplicant_instance->ssid );
    memcpy( (char*)details.SSID.value, supplicant_instance->ssid, details.SSID.length );
    whd_security = convert_to_whd_security_type(supplicant_instance->auth_type);
    if( whd_security == WHD_SECURITY_UNKNOWN )
    {
        cy_enterprise_security_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "\r\n Unsupported security type. \r\n");
        return CY_RSLT_ENTERPRISE_SECURITY_BADARG;
    }
    details.security = whd_security;

    conn_info.auth_type = supplicant_instance->auth_type;
    conn_info.eap_type = supplicant_instance->eap_type;
    conn_info.private_key = (uint8_t*)supplicant_instance->tls_security.key;
    conn_info.key_length = supplicant_instance->tls_security.key_len;
    conn_info.root_ca_cert_length = supplicant_instance->tls_security.ca_cert_len;
    conn_info.trusted_ca_certificates = (uint8_t*)supplicant_instance->tls_security.ca_cert;
    conn_info.user_cert = (uint8_t*)supplicant_instance->tls_security.cert;
    conn_info.user_cert_length = supplicant_instance->tls_security.cert_len;
    conn_info.eap_identity = (uint8_t*)supplicant_instance->outer_eap_identity;

    if( conn_info.eap_type == (eap_type_t) CY_ENTERPRISE_SECURITY_EAP_TYPE_PEAP )
    {
        conn_info.tunnel_auth_type = supplicant_instance->phase2_config.tunnel_auth_type;

        conn_info.user_name = (uint8_t*) supplicant_instance->phase2_config.tunnel_protocol.peap.inner_identity.identity;
        conn_info.password =  (uint8_t*) supplicant_instance->phase2_config.tunnel_protocol.peap.inner_identity.password;
    }
    else if( conn_info.eap_type == (eap_type_t) CY_ENTERPRISE_SECURITY_EAP_TYPE_TTLS )
    {
        conn_info.tunnel_auth_type = supplicant_instance->phase2_config.tunnel_auth_type;

        conn_info.user_name = (uint8_t*) supplicant_instance->phase2_config.tunnel_protocol.eap_ttls.inner_identity.identity;
        conn_info.password =  (uint8_t*) supplicant_instance->phase2_config.tunnel_protocol.eap_ttls.inner_identity.password;

        conn_info.inner_eap_type = supplicant_instance->phase2_config.tunnel_protocol.eap_ttls.inner_eap_type;
    }

    conn_info.interface = supplicant_instance->interface;

    conn_info.tls_session = supplicant_instance->tls_context->session;
    conn_info.tls_identity = supplicant_instance->tls_identity;
    conn_info.context = supplicant_instance->tls_context;

    cy_tls_init_workspace_context( conn_info.context );

    conn_info.context->root_ca_certificates = NULL;

    conn_info.interface = whd_driver->iflist[ 0 ];
    cy_enterprise_security_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "Initialize supplicant\n");
    res = supplicant_init( supplicant_workspace, &conn_info );
    if( res != CY_RSLT_SUCCESS )
    {
        cy_enterprise_security_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Unable to initialize supplicant. Error = [%u]\n", (unsigned int)res);
        return res;
    }

    cy_enterprise_security_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "Start supplicant\n");
    res = supplicant_start( supplicant_workspace );
    if( res != CY_RSLT_SUCCESS )
    {
        cy_enterprise_security_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Unable to start supplicant. Error = [%u]\n", (unsigned int)res);
        return res;
    }

    return CY_RSLT_SUCCESS;
}

static whd_security_t convert_to_whd_security_type(cy_enterprise_security_auth_t supplicant_security)
{
    switch(supplicant_security)
    {
        case CY_ENTERPRISE_SECURITY_AUTH_TYPE_WPA_AES:
            return WHD_SECURITY_WPA_AES_ENT;

        case CY_ENTERPRISE_SECURITY_AUTH_TYPE_WPA_MIXED:
            return WHD_SECURITY_WPA_MIXED_ENT;

        case CY_ENTERPRISE_SECURITY_AUTH_TYPE_WPA2_AES:
            return WHD_SECURITY_WPA2_AES_ENT;

        case CY_ENTERPRISE_SECURITY_AUTH_TYPE_WPA2_MIXED:
            return WHD_SECURITY_WPA2_MIXED_ENT;

#ifdef ENABLE_ENTPS_FEATURE
        case CY_ENTERPRISE_SECURITY_AUTH_TYPE_WPA3_AES:
            return WHD_SECURITY_WPA3_ENT;

        case CY_ENTERPRISE_SECURITY_AUTH_TYPE_WPA3_192BIT:
            return WHD_SECURITY_WPA3_192BIT_ENT;

        case CY_ENTERPRISE_SECURITY_AUTH_TYPE_WPA3_AES_CCMP:
            return WHD_SECURITY_WPA3_ENT_AES_CCMP;
#endif

        default:
            return WHD_SECURITY_UNKNOWN;
    }
}
