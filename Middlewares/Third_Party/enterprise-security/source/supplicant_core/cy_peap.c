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
#include "cy_peap.h"

/******************************************************
 *                      Macros
 ******************************************************/
#define CY_SUPPLICANT_PEAP_DEBUG  cy_enterprise_security_log_msg

/******************************************************
 *               Function Definations
 ******************************************************/
supplicant_packet_t supplicant_create_peap_response_packet( supplicant_packet_t* packet, eap_type_t eap_type, uint16_t data_length, uint8_t length_field_overhead, supplicant_workspace_t* workspace )
{
    eap_tls_packet_t*   header = NULL;
    cy_rslt_t           result;
    tls_record_t*       record = NULL;
    peap_header_t*      peap_header = NULL;
    uint16_t            header_space = 0;
    uint16_t            footer_pad_space = 0;

    supplicant_tls_calculate_overhead( workspace, data_length, &header_space, &footer_pad_space );

    result = supplicant_host_create_packet(  workspace->interface->whd_driver,packet, sizeof(eap_tls_packet_t) + sizeof(peap_header_t) + length_field_overhead + data_length + header_space + footer_pad_space + 10 );
    if ( result != CY_RSLT_SUCCESS )
    {
        return NULL;
    }

    header                = ( eap_tls_packet_t* ) supplicant_host_get_data( workspace->interface->whd_driver,*packet );

    header->eap.code      = EAP_CODE_RESPONSE;
    header->eap.id        = workspace->last_received_id;
    header->eap.type      = CY_ENTERPRISE_SECURITY_EAP_TYPE_PEAP;
    if ( length_field_overhead > 0 )
    {
        header->eap_tls.flags = EAP_TLS_FLAG_LENGTH_INCLUDED; /* Always include length until such time that we need to support fragmentation on transmit of PEAP packets */
    }
    else
    {
        header->eap_tls.flags = 0;
    }

    record                = (tls_record_t*) ( header->data + length_field_overhead );

    record->type          = CY_TLS_RECORD_TYPE_APPLICATION_DATA;

    cy_tls_get_versions(workspace->tls_context, &record->major_version, &record->minor_version);

    record->length        = htobe16( sizeof( peap_header_t ) + data_length );
    peap_header           = ( peap_header_t* ) record->message;
    peap_header->type     = eap_type;

    /* set data point to inner eap packet after all headers */

    supplicant_inner_packet_set_data(  workspace->interface->whd_driver,packet, sizeof(eap_tls_packet_t) + sizeof(peap_header_t) + sizeof(tls_record_header_t) + length_field_overhead - 1 );

    return packet;
}

void supplicant_send_peap_response_packet( supplicant_packet_t* packet, supplicant_workspace_t* workspace )
{
    eap_tls_packet_t*   header;
    tls_record_t*       record;
    uint32_t            data_length;
    uint8_t             length_field_overhead = 0;
    uint16_t            aligned_length;

    /* return to eap_tls_packet_t header */
    supplicant_inner_packet_set_data( workspace->interface->whd_driver,packet, -1 * (sizeof(eap_tls_packet_t) + sizeof(peap_header_t) + sizeof(tls_record_header_t) + length_field_overhead - 1));

    header                = ( eap_tls_packet_t* ) supplicant_host_get_data(workspace->interface->whd_driver, *packet );


    if ( header->eap_tls.flags &  EAP_TLS_FLAG_LENGTH_INCLUDED )
    {
        length_field_overhead = 4;
    }
    record                = (tls_record_t*) ( header->data + length_field_overhead );

    data_length           = htobe16(record->length);

    cy_tls_encrypt_data(workspace->tls_context, (uint8_t*)record, record->message, &data_length);

    if ( length_field_overhead )
    {
        SUPPLICANT_WRITE_32_BE( &header->data, data_length );
    }

    data_length  += sizeof( eap_header_t ) + sizeof(eap_tls_header_t) + length_field_overhead;
    SUPPLICANT_WRITE_16_BE( &aligned_length, data_length );
    header->eap.length = aligned_length;

    supplicant_send_eapol_packet( *packet, workspace, EAP_PACKET, data_length );
}

cy_rslt_t supplicant_process_peap_event(supplicant_workspace_t* workspace, supplicant_packet_t packet)
{
    supplicant_phase2_state_t* peap = NULL;
    peap_packet_t* peap_packet = NULL;

    CY_SUPPLICANT_PEAP_DEBUG(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "\r\n IN PROCESS PEAP EVENT \r\n");

    peap = (supplicant_phase2_state_t*) workspace->ptr_phase2;

    peap_packet = (peap_packet_t*) supplicant_host_get_data(workspace->interface->whd_driver, packet );


    if ( peap_packet->type == CY_ENTERPRISE_SECURITY_EAP_TYPE_IDENTITY )
    {
        CY_SUPPLICANT_PEAP_DEBUG(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "\r\n  %s %d \r\n",__FUNCTION__,__LINE__);

        /* this is probably an extension packet and this is not the type but rather the code */
        eap_header_t* eap_packet = (eap_header_t*) peap_packet;
        if ( eap_packet->type == 33 )
        {
            supplicant_packet_t response_packet;
            uint8_t* data;
            peap_extention_request_t* request = (peap_extention_request_t*) peap_packet;
            peap_extention_response_t* response;
            uint16_t aligned_length;
            CY_SUPPLICANT_PEAP_DEBUG(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "\r\n  %s %d \r\n",__FUNCTION__,__LINE__);

            supplicant_create_peap_response_packet( &response_packet, CY_ENTERPRISE_SECURITY_EAP_TYPE_IDENTITY, sizeof(peap_extention_response_t) - sizeof(peap_header_t), workspace->tls_length_overhead, workspace );
            data = supplicant_host_get_data( workspace->interface->whd_driver,response_packet );
            CY_SUPPLICANT_PEAP_DEBUG(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Supplicant peap received extension success\n");

            response = (peap_extention_response_t*) ( data - sizeof(peap_header_t) );
            response->header.code = 2;
            response->header.id = request->header.id;
            response->header.type = request->header.type;
            SUPPLICANT_WRITE_16_BE( &aligned_length, sizeof(peap_extention_response_t) );
            response->header.length = aligned_length;
            memcpy( &response->avp[ 0 ], &request->avp[ 0 ], sizeof( response->avp[ 0 ] ) );
            peap->result = CY_RSLT_ENTERPRISE_SECURITY_SUPPLICANT_ABORTED;
            supplicant_send_peap_response_packet( &response_packet, workspace );
        }
        else
        {
            /* Peap packets doens't include Code, ID, or length, they are all obtained from EAP packet */
            supplicant_packet_t response_packet;
            uint8_t * data;
            CY_SUPPLICANT_PEAP_DEBUG(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "\r\n  %s %d \r\n",__FUNCTION__,__LINE__);

            CY_SUPPLICANT_PEAP_DEBUG(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Supplicant peap received EAP ID Request\n");

            peap->sub_stage = SUPPLICANT_EAP_IDENTITY;
            supplicant_create_peap_response_packet( &response_packet, CY_ENTERPRISE_SECURITY_EAP_TYPE_IDENTITY, peap->identity_length, workspace->tls_length_overhead, workspace );
            data = supplicant_host_get_data(workspace->interface->whd_driver, response_packet );
            memcpy( data, peap->identity, peap->identity_length );
            supplicant_send_peap_response_packet( &response_packet, workspace );
        }
    }
    else if ( ( peap_packet->type != peap->eap_type ) && ( peap->sub_stage == SUPPLICANT_EAP_IDENTITY ) )
    {
        CY_SUPPLICANT_PEAP_DEBUG(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Supplicant received PEAP packet Type %u\n", peap_packet->type);

        peap->sub_stage = SUPPLICANT_EAP_NAK;
    }
    else if ( ( peap_packet->type == peap->eap_type ) && ( ( peap->sub_stage == SUPPLICANT_EAP_IDENTITY ) || ( peap->sub_stage == SUPPLICANT_EAP_NAK ) || ( peap->sub_stage == SUPPLICANT_EAP_METHOD ) ) )
    {
        CY_SUPPLICANT_PEAP_DEBUG(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Supplicant received required EAP Type %u\n", peap->eap_type);

        if ( peap->eap_type == CY_ENTERPRISE_SECURITY_EAP_TYPE_MSCHAPV2 )
        {
            CY_SUPPLICANT_PEAP_DEBUG(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Process MSCHAP packet\n");
            mschap_process_packet( (mschapv2_packet_t*) peap_packet->data, workspace );
            peap->sub_stage = SUPPLICANT_EAP_METHOD;
        }
    }
    return CY_RSLT_SUCCESS;
}
