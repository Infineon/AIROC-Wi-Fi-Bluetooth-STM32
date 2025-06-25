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
 *  Implements user functions for joining/leaving enterprise security network.
 *
 *  This file provides end-user functions which allow joining or leaving
 *  enterprise security network.
 *
 */

#include "cy_enterprise_security_log.h"
#include "cy_enterprise_security_internal.h"
#include "cy_wifi_abstraction.h"
#include "cy_supplicant_core_constants.h"
#include "cy_supplicant_process_et.h"
#include "cy_wcm.h"

extern cy_wcm_ip_setting_t* static_ip_settings;

/* This API is used for internal purpose to pass the static IP settings */
cy_rslt_t cy_enterprise_security_set_static_ip( cy_wcm_ip_setting_t* ip_settings )
{
    static_ip_settings = ip_settings;
    return CY_RSLT_SUCCESS;
}

cy_rslt_t cy_enterprise_security_join( cy_enterprise_security_t handle )
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    cy_supplicant_instance_t *supplicant_instance;

    if( handle == NULL )
    {
        cy_enterprise_security_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Enterprise Security handle is NULL.\n" );
        return CY_RSLT_ENTERPRISE_SECURITY_BADARG;
    }

    supplicant_instance = (cy_supplicant_instance_t *)handle;

    if( is_wifi_connected() == WIFI_CONNECTED )
    {
        cy_enterprise_security_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Already connected to Wi-Fi network.\n");
        return CY_RSLT_ENTERPRISE_SECURITY_ALREADY_CONNECTED;
    }

    result = cy_supplicant_alloc( supplicant_instance );
    if ( result != CY_RSLT_SUCCESS )
    {
        cy_enterprise_security_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "ERROR: cy_supplicant_alloc failed with error = [%u]\n", (unsigned int)result);
        return result;
    }

    wifi_on_ent();

    result = cy_join_ent( supplicant_instance );
    if( result != CY_RSLT_SUCCESS )
    {
        cy_enterprise_security_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "ERROR: cy_join_ent failed with error = [%u]\n", (unsigned int)result);
        cy_supplicant_free( supplicant_instance );
        return result;
    }

    result = connect_ent( supplicant_instance->ssid, strlen( supplicant_instance->ssid ) + 1, NULL, 0, supplicant_instance->auth_type );
    if( result != CY_RSLT_SUCCESS )
    {
        cy_leave_ent( supplicant_instance );
        cy_supplicant_free( supplicant_instance );
        cy_enterprise_security_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "ERROR: connect failed with error = [%u]\n", (unsigned int)result);
        return result;
    }

    cy_enterprise_security_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "Successfully joined Enterprise Security network.\r\n");
    return CY_RSLT_SUCCESS;
}

cy_rslt_t cy_enterprise_security_leave(cy_enterprise_security_t handle)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    cy_supplicant_instance_t *supplicant_instance;

    if( handle == NULL )
    {
        cy_enterprise_security_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Enterprise Security handle is NULL.\n" );
        return CY_RSLT_ENTERPRISE_SECURITY_BADARG;
    }

    supplicant_instance = (cy_supplicant_instance_t *)handle;

    if( is_wifi_connected() != WIFI_CONNECTED )
    {
        cy_enterprise_security_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Not connected to any Wi-Fi network.\n");
        return CY_RSLT_ENTERPRISE_SECURITY_NOT_CONNECTED;
    }

    result = cy_leave_ent( supplicant_instance );
    if( result !=  CY_RSLT_SUCCESS )
    {
        cy_enterprise_security_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "ERROR: cy_leave_ent failed with error = [%u]\n", (unsigned int)result);
        cy_supplicant_free( supplicant_instance );
        return result;
    }

    result = disconnect_ent();
    if( result != CY_RSLT_SUCCESS )
    {
        cy_enterprise_security_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "ERROR: disconnect failed with error = [%u]\n", (unsigned int)result);
        cy_supplicant_free( supplicant_instance );
        return result;
    }

    (void) cy_supplicant_free( supplicant_instance );
    cy_enterprise_security_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "Successfully left Enterprise Security network.\r\n");

    static_ip_settings = NULL;

    return CY_RSLT_SUCCESS;
}
