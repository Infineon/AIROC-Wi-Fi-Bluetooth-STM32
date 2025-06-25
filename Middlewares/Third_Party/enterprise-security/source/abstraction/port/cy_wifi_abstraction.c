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
 *  Implements functions for controlling the Wi-Fi system in AnyCloud using WCM
 *
 *  This file provides functions which allow actions such as turning on,
 *  joining Wi-Fi networks, getting the Wi-Fi connection status, etc
 *
 */
#include "cy_enterprise_security_log.h"
#include "cy_enterprise_security_error.h"
#include "cy_wifi_abstraction.h"
#include "cy_wcm.h"

#define ENTERPRISE_SECURITY_IPV4_ADDR_SIZE           4

cy_wcm_ip_setting_t* static_ip_settings = NULL;
static cy_wcm_config_t wcm_config;

#ifdef ENABLE_ENTERPRISE_SECURITY_LOGS
static void print_ip4(uint32_t ip);
#endif

void wifi_on_ent( void )
{
    cy_rslt_t res;

    wcm_config.interface = CY_WCM_INTERFACE_TYPE_STA;

    res = cy_wcm_init(&wcm_config);
    if( res != CY_RSLT_SUCCESS )
    {
        cy_enterprise_security_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Wi-Fi module failed to initialize, err=%u\r\n", (unsigned int)res);
        return;
    }

    cy_enterprise_security_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Wi-Fi module initialized.\r\n");
}

cy_rslt_t connect_ent( const char *ssid, uint8_t ssid_length,
                 const char *password, uint8_t password_length,
                 cy_enterprise_security_auth_t auth_type )
{
    cy_rslt_t res;
    cy_wcm_connect_params_t connect_params;
    cy_wcm_ip_address_t ip_addr;

    memset(&connect_params, 0, sizeof(cy_wcm_connect_params_t));

    /* validate input parameters */
    if( ssid == NULL || strlen(ssid) == 0 || strlen(ssid) > CY_ENTERPRISE_SECURITY_MAX_SSID_LENGTH )
    {
        cy_enterprise_security_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Invalid SSID!\n");
        return CY_RSLT_ENTERPRISE_SECURITY_BADARG;
    }

    /* Setup parameters. */
    memcpy(connect_params.ap_credentials.SSID, ssid, strlen(ssid) + 1);

    if( auth_type == CY_ENTERPRISE_SECURITY_AUTH_TYPE_WPA_AES )
    {
        connect_params.ap_credentials.security = CY_WCM_SECURITY_WPA_AES_ENT;
    }
    else if( auth_type == CY_ENTERPRISE_SECURITY_AUTH_TYPE_WPA_MIXED )
    {
        connect_params.ap_credentials.security = CY_WCM_SECURITY_WPA_MIXED_ENT;
    }
    else if( auth_type == CY_ENTERPRISE_SECURITY_AUTH_TYPE_WPA2_AES )
    {
        connect_params.ap_credentials.security = CY_WCM_SECURITY_WPA2_AES_ENT;
    }
    else if( auth_type == CY_ENTERPRISE_SECURITY_AUTH_TYPE_WPA2_MIXED )
    {
        connect_params.ap_credentials.security = CY_WCM_SECURITY_WPA2_MIXED_ENT;
    }
    else if( auth_type == CY_ENTERPRISE_SECURITY_AUTH_TYPE_WPA2_FBT )
    {
        cy_enterprise_security_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "The auth type is not supported \r\n");
        return CY_RSLT_ENTERPRISE_SECURITY_BADARG;
    }
#ifdef ENABLE_ENTPS_FEATURE
    else if( auth_type == CY_ENTERPRISE_SECURITY_AUTH_TYPE_WPA3_AES )
    {
        connect_params.ap_credentials.security = CY_WCM_SECURITY_WPA3_ENT;
    }
    else if( auth_type == CY_ENTERPRISE_SECURITY_AUTH_TYPE_WPA3_AES_CCMP )
    {
        connect_params.ap_credentials.security = CY_WCM_SECURITY_WPA3_ENT_AES_CCMP;
    }
    else if( auth_type == CY_ENTERPRISE_SECURITY_AUTH_TYPE_WPA3_192BIT )
    {
        connect_params.ap_credentials.security = CY_WCM_SECURITY_WPA3_192BIT_ENT;
    }
#endif
    else
    {
        cy_enterprise_security_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "The auth type is invalid\r\n");
        return CY_RSLT_ENTERPRISE_SECURITY_BADARG;
    }

    connect_params.band = CY_WCM_WIFI_BAND_ANY; // No band is set, so set it to auto.

    if(static_ip_settings != NULL)
    {
        connect_params.static_ip_settings = static_ip_settings;
    }

    res = cy_wcm_connect_ap(&connect_params, &ip_addr);
    if( res != CY_RSLT_SUCCESS )
    {
        cy_enterprise_security_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Wi-Fi unable to connect, err=%u\r\n", (unsigned int)res);
        return CY_RSLT_ENTERPRISE_SECURITY_JOIN_ERROR;
    }

    cy_enterprise_security_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "Wi-Fi Connected to AP.\r\n");
#ifdef ENABLE_ENTERPRISE_SECURITY_LOGS
    print_ip4(ip_addr.ip.v4);
#endif
    return CY_RSLT_SUCCESS;
}

cy_rslt_t disconnect_ent( void )
{
    cy_rslt_t res = cy_wcm_disconnect_ap();
    if( res != CY_RSLT_SUCCESS )
    {
        cy_enterprise_security_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Not Successfully Disconnected from AP, err=%u.\r\n", (unsigned int)res);
        return CY_RSLT_ENTERPRISE_SECURITY_LEAVE_ERROR;
    }

    cy_enterprise_security_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "Successfully Disconnected from AP.\r\n");
    return CY_RSLT_SUCCESS;
}

wifi_connection_status_t is_wifi_connected( void )
{
    if( cy_wcm_is_connected_to_ap() == 1 )
    {
        return WIFI_CONNECTED;
    }
    else
    {
        return WIFI_NOT_CONNECTED;
    }
}

#ifdef ENABLE_ENTERPRISE_SECURITY_LOGS
static void print_ip4(uint32_t ip)
{
    unsigned char bytes[ENTERPRISE_SECURITY_IPV4_ADDR_SIZE];

    (void)bytes;

    bytes[0] = ip & 0xFF;
    bytes[1] = (ip >> 8) & 0xFF;
    bytes[2] = (ip >> 16) & 0xFF;
    bytes[3] = (ip >> 24) & 0xFF;

    cy_enterprise_security_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "IP Address acquired: %d.%d.%d.%d\n", bytes[0], bytes[1], bytes[2], bytes[3]);
}
#endif // ENABLE_ENTERPRISE_SECURITY_LOGS
