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
#include "cy_eap.h"

/******************************************************
 *                      Macros
 ******************************************************/
#define EAP_WRAPPER_DEBUG  cy_enterprise_security_log_msg

/******************************************************
 *               Function Definitions
 ******************************************************/
void supplicant_send_eapol_packet(supplicant_packet_t packet, supplicant_workspace_t* workspace, eapol_packet_type_t type, uint16_t content_size )
{
    eapol_packet_t* header = (eapol_packet_t*) supplicant_host_get_data( workspace->interface->whd_driver,packet );
    memcpy( header->ethernet.ether_dhost, &workspace->authenticator_mac_address, sizeof(whd_mac_t) );
    memcpy( header->ethernet.ether_shost, &workspace->supplicant_mac_address, sizeof(whd_mac_t) );
    header->ethernet.ether_type = supplicant_host_hton16( ETHER_TYPE_802_1X );
    header->eapol.version = 1;
    header->eapol.type    = type;
    header->eapol.length  = supplicant_host_hton16( content_size );
    supplicant_host_send_packet( workspace->supplicant_host_workspace, packet, content_size + sizeof(eapol_packet_header_t));
}

cy_rslt_t supplicant_send_eapol_start( supplicant_workspace_t *workspace )
{
    supplicant_packet_t packet;
    cy_rslt_t result;

    result = supplicant_host_create_packet(workspace->interface->whd_driver, &packet, sizeof(eapol_packet_t) );
    if ( result != CY_RSLT_SUCCESS )
    {
        EAP_WRAPPER_DEBUG(CYLF_MIDDLEWARE, CY_LOG_ERR, " ERR: #### Sending EAPOL start packet\r\n" );
        return CY_RSLT_ENTERPRISE_SECURITY_SUPPLICANT_ERROR;
    }

    if (packet == 0)
    {
        return CY_RSLT_ENTERPRISE_SECURITY_SUPPLICANT_ERROR;
    }
    EAP_WRAPPER_DEBUG(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Sending EAPOL start\r\n");
    supplicant_send_eapol_packet( packet, workspace, EAPOL_START, 0);

    return CY_RSLT_SUCCESS;
}

void supplicant_send_eap_response_packet( supplicant_workspace_t* workspace, eap_type_t eap_type, uint8_t* data, uint16_t data_length )
{
    supplicant_packet_t packet;
    eap_packet_t* header;
    cy_rslt_t result;
    uint16_t      aligned_length;

    result = supplicant_host_create_packet( workspace->interface->whd_driver,&packet, sizeof(eap_packet_t) + data_length );
    if ( result != CY_RSLT_SUCCESS )
    {
        EAP_WRAPPER_DEBUG(CYLF_MIDDLEWARE, CY_LOG_ERR, " ERR: #### Sending EAP Response\r\n" );
        return;
    }

    header             = ( eap_packet_t* ) supplicant_host_get_data(workspace->interface->whd_driver,packet );
    header->eap.code   = EAP_CODE_RESPONSE;
    header->eap.id     = workspace->last_received_id;
    SUPPLICANT_WRITE_16_BE( &aligned_length, ( sizeof( eap_header_t ) + data_length ) );
    header->eap.length = aligned_length;
    header->eap.type   = eap_type;
    memcpy( header->data, data, data_length );
    EAP_WRAPPER_DEBUG(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Sending EAP Response\r\n");

    supplicant_send_eapol_packet( packet, workspace, EAP_PACKET, sizeof( eap_header_t ) + data_length );
    EAP_WRAPPER_DEBUG(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "EAP Response sent\r\n");
}

cy_rslt_t supplicant_send_zero_length_eap_tls_packet( supplicant_workspace_t* workspace )
{
    supplicant_packet_t     packet;
    eap_tls_packet_t* header;
    uint16_t          aligned_length;

    cy_rslt_t result = supplicant_host_create_packet( workspace->interface->whd_driver, &packet, sizeof(eap_tls_packet_t) );
    if ( result != CY_RSLT_SUCCESS )
    {
        return CY_RSLT_ENTERPRISE_SECURITY_SUPPLICANT_ERROR;
    }

    header                = ( eap_tls_packet_t* ) supplicant_host_get_data( workspace->interface->whd_driver,packet );
    header->eap.code      = EAP_CODE_RESPONSE;
    header->eap.id        = workspace->last_received_id;
    SUPPLICANT_WRITE_16_BE( &aligned_length, (sizeof(eap_header_t) + sizeof(eap_tls_header_t)) );
    header->eap.length = aligned_length;
    header->eap_tls.flags = 0;
    header->eap.type      = workspace->eap_type;
    EAP_WRAPPER_DEBUG(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Sending zero length EAP-TLS packet\r\n");
    supplicant_send_eapol_packet( packet, workspace, EAP_PACKET, (sizeof(eap_header_t) + sizeof(eap_tls_header_t)) );

    return CY_RSLT_SUCCESS;
}

cy_rslt_t supplicant_send_eap_tls_fragment( supplicant_workspace_t* workspace, supplicant_packet_t packet )
{
    eap_tls_packet_t*  header = ( eap_tls_packet_t* ) supplicant_host_get_data( workspace->interface->whd_driver,packet );

    /* We set the EAP ID here because fragmentation occurs before we know what the EAP ID will be */
    header->eap.id      = workspace->last_received_id;
    uint16_t packet_length = supplicant_host_get_packet_size( workspace->interface->whd_driver,packet );
    EAP_WRAPPER_DEBUG(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Outgoing eap tls packet_length = [%u]\n", (unsigned int)packet_length);
    supplicant_send_eapol_packet( packet, workspace, EAP_PACKET, packet_length - sizeof(cy_ether_header_t) - sizeof(eapol_header_t) );

    return CY_RSLT_SUCCESS;
}
