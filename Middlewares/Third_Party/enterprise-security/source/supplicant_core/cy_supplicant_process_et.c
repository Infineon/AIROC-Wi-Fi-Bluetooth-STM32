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
#include "cy_supplicant_process_et.h"
#include "cy_supplicant_host.h"
#include "cy_eap.h"
#include "cy_tls_abstraction.h"
#include "whd.h"
#include "whd_events_int.h"
#include "cy_ttls.h"
#include "cyabs_rtos.h"

/******************************************************
 *                    Macros
 ******************************************************/

/* Supplicant library allocates thread stack memory, Recommended to keep this macro enabled */
#define RTOS_USE_STATIC_THREAD_STACK

#define CY_SUPPLICANT_PROCESS_ET_INFO cy_enterprise_security_log_msg
#define CY_SUPPLICANT_PROCESS_ET_BYTES( x ) //printf x

//#define ENABLE_SUPPLICANT_DUMP_BYTES

#ifdef ENABLE_SUPPLICANT_DUMP_BYTES
static void          supplicant_dump_bytes                     ( const uint8_t* bptr, uint32_t len );
#else
#define supplicant_dump_bytes( bptr, len )
#endif


#define IF_TO_WORKSPACE( interface )   ( active_supplicant_workspaces[ WHD_STA_ROLE ] )     /* STA is the only currently supported interface. */

/* Labels */
#define CY_EAP_TTLS_LABEL       "ttls keying material"
#define CY_EAP_TLS_LABEL        "client EAP encryption"
#define CY_EAP_TLS_V13_LABEL    "EXPORTER_EAP_TLS_Key_Material"

/******************************************************
 *              Function Prototypes
 ******************************************************/
extern cy_rslt_t cy_tls_receive_eap_packet( supplicant_workspace_t* supplicant, supplicant_packet_t* packet );

static cy_rslt_t supplicant_send_eap_tls_packet( supplicant_workspace_t* workspace, tls_agent_event_message_t* tls_agent_message, uint32_t timeout );

/******************************************************
 *              Global variables
 ******************************************************/

supplicant_workspace_t* active_supplicant_workspaces[SUPPLICANT_WORKSPACE_ARRAY_SIZE] = { 0 };
const whd_event_num_t supplicant_events[] = { WLC_E_LINK, WLC_E_DEAUTH_IND, WLC_E_DISASSOC_IND, WLC_E_NONE };

/******************************************************
 *              Function Definations
 ******************************************************/

#ifdef ENABLE_SUPPLICANT_DUMP_BYTES
static void supplicant_dump_bytes(const uint8_t* bptr, uint32_t len)
{
    uint32_t i = 0;

    for (i = 0; i < len; )
    {
        if ((i & 0x0f) == 0)
        {
            CY_SUPPLICANT_PROCESS_ET_BYTES( ( "\r\n" ) );
        }
        else if ((i & 0x07) == 0)
        {
            CY_SUPPLICANT_PROCESS_ET_BYTES( (" ") );
        }
        CY_SUPPLICANT_PROCESS_ET_BYTES( ( "%02x ", bptr[i++] ) );
    }
    CY_SUPPLICANT_PROCESS_ET_BYTES( ( "\r\n" ) );
}
#endif /*ENABLE_SUPPLICANT_DUMP_BYTES*/

tls_agent_packet_t* supplicant_receive_eap_tls_packet( void* workspace_in, uint32_t* new_length, uint32_t timeout )
{
    supplicant_workspace_t*     workspace = (supplicant_workspace_t*) workspace_in;
    tls_agent_event_message_t   message;
    supplicant_rtos_workspace_t* tls_agent_host_workspace = (supplicant_rtos_workspace_t*)workspace->tls_agent.tls_agent_host_workspace;
    cy_rslt_t result = cy_rtos_get_queue(&tls_agent_host_workspace->event_queue, &message, timeout, 0 );

    if(result == CY_RTOS_TIMEOUT )
    {
        return NULL;
    }
    if ( result != CY_RSLT_SUCCESS )
    {
        return NULL;
    }
    if ( message.event_type == TLS_AGENT_EVENT_EAPOL_PACKET )
    {
       *new_length = message.length;
    }
    else
    {
        cy_rtos_delay_milliseconds( 10 );
    }

    return message.data.packet;
}

cy_rslt_t supplicant_tls_agent_finish_connect( supplicant_workspace_t* workspace )
{
    uint8_t mppe_keys[128]      = {0};
    uint8_t *key                = NULL;
    uint8_t key_len             = 0;
    cy_rslt_t result            = CY_RSLT_SUCCESS;
    const char *label           = NULL;
    uint8_t eap_tls13_context   = CY_ENTERPRISE_SECURITY_EAP_TYPE_NONE;
    uint8_t *context            = NULL;
    uint16_t context_len        = 0;

    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_INFO, "TLS handshake completed successfully\r\n");

    if (workspace->tls_context->tls_v13)
    {
        eap_tls13_context = workspace->eap_type;
        context = &eap_tls13_context;
        context_len = 1;
        label = CY_EAP_TLS_V13_LABEL;
    }
    else
    {
        if(workspace->eap_type == CY_ENTERPRISE_SECURITY_EAP_TYPE_TTLS)
        {
            label = CY_EAP_TTLS_LABEL;
        }
        else
        {
            label = CY_EAP_TLS_LABEL;
        }
    }

    /* Calculate MPPE KEY */
    result = cy_tls_get_mppe_key( workspace->tls_context, label, context, context_len, mppe_keys, SIZEOF_MPPE_KEYS);
    if ( result != CY_RSLT_SUCCESS )
    {
        CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_ERR, "Failed to generate MPPE KEY\r\n");
        return result;
    }

    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "MPPE keys:\r\n");
    supplicant_dump_bytes( mppe_keys, SIZEOF_MPPE_KEYS );

    if ( workspace->auth_type == CY_ENTERPRISE_SECURITY_AUTH_TYPE_WPA2_FBT )
    {
        /* Set the PMK which is the second 32 bytes of the MPPE keys */
        key = mppe_keys + PMK_LEN;
        key_len = PMK_LEN;
    }
#ifdef ENABLE_ENTPS_FEATURE
    else if ( workspace->auth_type == CY_ENTERPRISE_SECURITY_AUTH_TYPE_WPA3_192BIT )
    {
        key = mppe_keys;
        key_len = PMK_LEN_192BIT;
    }
#endif
    else
    {
        /* Set the PMK which is the first 32 bytes of the MPPE keys */
        key = mppe_keys;
        key_len = PMK_LEN;
    }

    result  = supplicant_set_pmk( workspace->interface,key, key_len );
    if ( result != CY_RSLT_SUCCESS )
    {
        CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_ERR, "Failed to write PMK to radio firmware\r\n");
        return result;
    }

    if( workspace->tls_context->resume != 1 )
    {
        if( workspace->eap_type == CY_ENTERPRISE_SECURITY_EAP_TYPE_TTLS )
        {
            if(workspace->tls_context->tls_v13)
            {
                /* With TLS1.3, server will send an explicit request for TTLS. We should be sending identity as
                 * response to that request. Wait for that packet from server
                 */
                tls_agent_packet_t* packet        = NULL;
                uint32_t length                   = 0;

                packet = supplicant_receive_eap_tls_packet( workspace, &length, SUPPLICANT_TIMEOUT_PHASE2_START );
                if( packet != NULL )
                {
                    supplicant_host_free_packet(workspace->interface->whd_driver, packet );
                }
            }
            supplicant_init_ttls_phase2_handshake( workspace );
        }
        else
        {
            result = supplicant_send_zero_length_eap_tls_packet( workspace );
        }
    }

    return result;
}

cy_rslt_t supplicant_outgoing_pop( void* workspace, supplicant_event_message_t* message )
{
    supplicant_host_workspace_t* supplicant_host = (supplicant_host_workspace_t*)workspace;
    cy_rslt_t result;

    result = cy_rtos_get_queue( &supplicant_host->outgoing_packet_queue, message, SUPPLICANT_TIMEOUT, 0 );

    if(result != CY_RSLT_SUCCESS)
    {
        return CY_RSLT_ENTERPRISE_SECURITY_SUPPLICANT_ERROR;
    }

    return CY_RSLT_SUCCESS;
}

cy_rslt_t supplicant_outgoing_push( void* workspace, supplicant_event_message_t* message )
{
    cy_rslt_t result;
    supplicant_host_workspace_t* supplicant_host = (supplicant_host_workspace_t*)workspace;
    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_INFO, "Push outgoing packet to queue\r\n");
    result = cy_rtos_put_queue( &supplicant_host->outgoing_packet_queue, message, SUPPLICANT_NEVER_TIMEOUT, 0 );
    if(result != CY_RSLT_SUCCESS)
    {
        whd_buffer_release( supplicant_host->host_workspace.interface->whd_driver, (whd_buffer_t) message->data.packet, WHD_NETWORK_TX );
        return CY_RSLT_ENTERPRISE_SECURITY_SUPPLICANT_ERROR;
    }

    return CY_RSLT_SUCCESS;
}

cy_rslt_t supplicant_host_get_tls_data( supplicant_workspace_t* workspace,supplicant_packet_t eapol_packet, uint16_t offset, uint8_t** data, uint16_t* fragment_available_data_length, uint16_t *total_available_data_length )
{
    uint16_t packet_length = supplicant_host_get_packet_size(workspace->interface->whd_driver, eapol_packet );

    *data = supplicant_host_get_data( workspace->interface->whd_driver, eapol_packet ) + offset;
    *total_available_data_length    = packet_length - offset;
    *fragment_available_data_length = *total_available_data_length;

    return CY_RSLT_SUCCESS;
}

cy_rslt_t supplicant_fragment_and_queue_eap_response( supplicant_workspace_t *workspace )
{
    supplicant_packet_t          packet;
    eap_tls_packet_t*            header;
    uint32_t                     length_to_be_fragmented;
    uint32_t                     eap_tls_length;
    uint8_t*                     data;
    supplicant_event_message_t   message;
    uint32_t                     packet_length;
    uint32_t                     length_field_overhead = 0;

    length_to_be_fragmented = workspace->data_end - workspace->data_start;
    eap_tls_length          = length_to_be_fragmented;
    data                    = workspace->buffer;
    workspace->have_packet  = 0; /* Buffer should be consumed after this */

    /* This is the start of an EAP fragment chain */
    length_field_overhead = 4;

    while (length_to_be_fragmented != 0)
    {
        uint16_t amount_to_copy = 0;
        cy_rslt_t result;
        uint16_t aligned_length;

        amount_to_copy = (uint16_t) SUPPLICANT_DEFS_MIN(length_to_be_fragmented, ( 1024 - length_field_overhead ) );
        length_to_be_fragmented -= amount_to_copy;
        packet_length  = sizeof(eap_tls_packet_t) - 1 + length_field_overhead + amount_to_copy;
        result = supplicant_host_create_packet( workspace->interface->whd_driver,&packet, packet_length );
        if ( result != CY_RSLT_SUCCESS )
        {
            CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_ERR, "Unable to create eapol packet\r\n");
            return result;
        }

        header = ( eap_tls_packet_t* ) supplicant_host_get_data(  workspace->interface->whd_driver,packet );
        header->eap.code      = EAP_CODE_RESPONSE;
        header->eap.type      = workspace->eap_type;
        header->eap.id        = workspace->last_received_id;
        header->eap_tls.flags = 0;
        SUPPLICANT_WRITE_16_BE( &aligned_length, (sizeof(eap_header_t) + sizeof(eap_tls_header_t) + length_field_overhead + amount_to_copy) );
        header->eap.length = aligned_length;
        memcpy( &header->data[0] + length_field_overhead, data, amount_to_copy );
        supplicant_host_set_packet_size( workspace->interface->whd_driver,packet, packet_length );

        /* If the length field is included then this is the first (possibly only) fragment */
        if ( length_field_overhead == 4 )
        {
            header->eap_tls.flags = EAP_TLS_FLAG_LENGTH_INCLUDED;
            SUPPLICANT_WRITE_32_BE( &header->data, eap_tls_length );
            if ( length_to_be_fragmented != 0 )
            {
                header->eap_tls.flags |= EAP_TLS_FLAG_MORE_FRAGMENTS;
            }

            result = supplicant_queue_message_packet( workspace, SUPPLICANT_EVENT_PACKET_TO_SEND, packet );
            if ( result != CY_RSLT_SUCCESS )
            {
                CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_ERR, "Sending EAPOL packet failed\r\n");
                return result;
            }
            length_field_overhead = 0;

        }
        else /* The length field is not included */
        {
            if ( length_to_be_fragmented != 0 )
            {
                header->eap_tls.flags |= EAP_TLS_FLAG_MORE_FRAGMENTS;
            }
            message.event_type  = SUPPLICANT_EVENT_PACKET_TO_SEND;
            message.data.packet = packet;

            result = supplicant_outgoing_push((void*) workspace->supplicant_host_workspace, &message );
            if ( result != CY_RSLT_SUCCESS )
            {
                CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_ERR, "Sending EAPOL fragment failed\r\n");
                return result;
            }
        }

        if ( length_to_be_fragmented != 0 )
        {
            data += amount_to_copy;
        }
    }
    return CY_RSLT_SUCCESS;
}

/* Free a previously create phase2 work space. */
cy_rslt_t supplicant_phase2_deinit( supplicant_workspace_t* workspace )
{
    supplicant_phase2_workspace_t* phase2_workspace = workspace->ptr_phase2;
    whd_result_t res = CY_RSLT_SUCCESS;

    if ( phase2_workspace != NULL )
    {
        CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_INFO, "Terminate PEAP thread\r\n");

        if ( phase2_workspace->thread_stack != NULL )
        {
            CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "MEM : FREE : Free PEAP thread stack\r\n");
            supplicant_host_free( phase2_workspace->thread_stack );
            phase2_workspace->thread_stack = NULL;
        }

        CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "MEM : FREE : Free PEAP workspace\r\n");
        supplicant_host_free( workspace->ptr_phase2 );
        workspace->ptr_phase2 = NULL;
    }

    return res;
}

cy_rslt_t supplicant_phase2_stop( supplicant_workspace_t* workspace )
{
    supplicant_phase2_workspace_t* phase2_workspace = workspace->ptr_phase2;

    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_INFO, "Stop supplicant_peap_thread\r\n");
    if ( (phase2_workspace != NULL) && (phase2_workspace->state.result != CY_RSLT_ENTERPRISE_SECURITY_SUPPLICANT_NOT_STARTED) )
    {
        CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Signal abort to Phase2 thread\r\n");
        phase2_workspace->state.result = CY_RSLT_ENTERPRISE_SECURITY_SUPPLICANT_ABORTED;
        CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Wait for PEAP thread to exit\r\n");
        cy_rtos_join_thread( &phase2_workspace->thread );
        CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "PEAP thread exited\r\n");
    }

    return CY_RSLT_SUCCESS;
}

cy_rslt_t supplicant_init_state(supplicant_workspace_t* workspace, eap_type_t eap_type )
{
    workspace->eap_type                 = eap_type;
    workspace->current_sub_stage        = SUPPLICANT_EAP_START;

    workspace->eap_handshake_start_time = 0;

    return CY_RSLT_SUCCESS;
}

cy_rslt_t supplicant_tls_agent_init( tls_agent_workspace_t* workspace )
{
    supplicant_rtos_workspace_t* host_workspace;

    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "[%s()] : L%d : MEM : MALLOC : Allocating TLS Agent host workspace\r\n", __FUNCTION__, __LINE__);
    host_workspace = supplicant_host_malloc("tls_agent workspace", sizeof(supplicant_rtos_workspace_t));
    if (host_workspace == NULL)
    {
        CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_ERR, " ERROR : [%s()] : L%d : Failed to allocate workspace for TLS Agent\r\n", __FUNCTION__, __LINE__);
        return CY_RSLT_ENTERPRISE_SECURITY_NOMEM;
    }
    memset(host_workspace, 0, sizeof(supplicant_rtos_workspace_t));
    workspace->tls_agent_host_workspace = host_workspace;

#ifdef RTOS_USE_STATIC_THREAD_STACK
    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "[%s()] : L%d : MEM : MALLOC : Allocating TLS Agent thread stack\r\n", __FUNCTION__, __LINE__);
    host_workspace->thread_stack = supplicant_host_malloc("tls agent stack", TLS_AGENT_THREAD_STACK_SIZE);
    if (host_workspace->thread_stack == NULL)
    {
        CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_ERR,
            "ERROR : Failed to allocate thread stack for TLS Agent\r\n");
        supplicant_host_free(host_workspace);
        host_workspace = NULL;
        return CY_RSLT_ENTERPRISE_SECURITY_NOMEM;
    }
    memset( host_workspace->thread_stack, 0, TLS_AGENT_THREAD_STACK_SIZE );
#else
    host_workspace->thread_stack = NULL;
#endif

    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, " [%s()] : L%d : Init event queue for TLS Agent\r\n", __FUNCTION__, __LINE__);
    cy_rtos_init_queue( &host_workspace->event_queue,  15 , sizeof(tls_agent_event_message_t)) ;

    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_INFO, " [%s()] : L%d : TLS Agent init done!\r\n", __FUNCTION__, __LINE__);

    return CY_RSLT_SUCCESS;
}

cy_rslt_t supplicant_tls_agent_deinit( tls_agent_workspace_t* workspace )
{
    supplicant_rtos_workspace_t* host_workspace = workspace->tls_agent_host_workspace;

    if ( host_workspace != NULL )
    {
        CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "[%s()] : L%d : Waiting for TLS Agent thread to join\r\n", __FUNCTION__, __LINE__);

        cy_rtos_join_thread(&host_workspace->thread);
        CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "[%s()] : L%d : TLS Agent thread joined\r\n", __FUNCTION__, __LINE__);

        CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "[%s()] : L%d : Deinit TLS Agent thread queue\r\n", __FUNCTION__, __LINE__);
        cy_rtos_deinit_queue( &host_workspace->event_queue );

        if ( host_workspace->thread_stack != NULL )
        {
            CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "[%s()] : L%d : MEM : FREE : Freeing TLS Agent thread stack\r\n", __FUNCTION__, __LINE__);
            supplicant_host_free( host_workspace->thread_stack );
            host_workspace->thread_stack = NULL;
        }

        CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "[%s()] : L%d : MEM : FREE : Freeing TLS Agent host workspace\r\n", __FUNCTION__, __LINE__);
        supplicant_host_free( workspace->tls_agent_host_workspace );
        workspace->tls_agent_host_workspace = NULL;
        CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_INFO, " [%s()] : L%d : TLS Agent de-init done!\r\n", __FUNCTION__, __LINE__);
    }

    return CY_RSLT_SUCCESS;
}

void supplicant_eap_handshake_cleanup( supplicant_workspace_t* workspace )
{
    /* Check if clean up is already done */
    if ( workspace->eap_handshake_start_time == 0 )
    {
        CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Already cleaned up\r\n");
        return;
    }

    /*** Free the resources and reset the states ***/
    if ( workspace->eap_type == CY_ENTERPRISE_SECURITY_EAP_TYPE_PEAP || workspace->eap_type == CY_ENTERPRISE_SECURITY_EAP_TYPE_TTLS)
    {
        /* Stop and terminate PEAP thread, since EAP handshake is over */
        CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Stop PEAP thread\r\n");
        supplicant_phase2_stop( workspace );
        CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Terminate PEAP thread\r\n");
        supplicant_phase2_deinit( workspace );
    }

    /* Stop and terminate TLS agent thread, since EAP handshake is over */
    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_INFO, "[%s()] : L%d : Deinit Supplicant agent\r\n", __FUNCTION__, __LINE__);
    supplicant_tls_agent_deinit( &workspace->tls_agent );

    /* Clean the session as we no longer need it */
    cy_tls_session_cleanup(workspace->tls_context);

    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_INFO, "Reset supplicant states\r\n");
    supplicant_init_state( workspace, workspace->eap_type );

    return;
}

cy_rslt_t supplicant_tls_calculate_overhead( supplicant_workspace_t* workspace, uint16_t available_space, uint16_t* header, uint16_t* footer )
{
    return cy_tls_calculate_overhead(  workspace, workspace->tls_context, available_space, header, footer );
}

static cy_rslt_t supplicant_send_eap_tls_packet( supplicant_workspace_t* workspace, tls_agent_event_message_t* tls_agent_message, uint32_t timeout )
{
    supplicant_rtos_workspace_t* tls_agent_host_workspace = (supplicant_rtos_workspace_t*) workspace->tls_agent.tls_agent_host_workspace;
    cy_rslt_t result = cy_rtos_put_queue(&tls_agent_host_workspace->event_queue, tls_agent_message, SUPPLICANT_NEVER_TIMEOUT, 0);
    if ( result != CY_RSLT_SUCCESS )
    {
        CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_ERR, "Supplicant unable to push packet to tls agent queue\r\n");
        supplicant_host_free_packet( workspace->interface->whd_driver,tls_agent_message->data.packet );
    }

    return CY_RSLT_SUCCESS;
}

void supplicant_tls_agent_thread( cy_thread_arg_t arg )
{
    supplicant_workspace_t* workspace = (supplicant_workspace_t*)arg;
    supplicant_rtos_workspace_t* host = (supplicant_rtos_workspace_t*)workspace->tls_agent.tls_agent_host_workspace;
    supplicant_event_message_t message;

    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "\r\nTLS handshake start..\r\n");
    if ( cy_tls_generic_start_tls_with_ciphers( workspace->tls_context, workspace, TLS_VERIFICATION_REQUIRED ) != CY_RSLT_SUCCESS )
    {
        CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_ERR, "TLS handshake failed with error\r\n");
        workspace->supplicant_result = CY_RSLT_ENTERPRISE_SECURITY_SUPPLICANT_ABORTED;
    }
    else
    {
        CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "TLS Agent finish connect start..\r\n");
        if ( supplicant_tls_agent_finish_connect( workspace ) == CY_RSLT_SUCCESS)
        {
            CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "TLS Agent finish connect completed\r\n");
        }
        else
        {
            CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_ERR, "TLS Agent finish connect failed\r\n");
        }
    }
    /* Clean up left over messages in the event queue */
    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Cleanup TLS Agent event queue\r\n");
    while ( cy_rtos_get_queue( &host->event_queue, &message, 0, 0 ) == CY_RSLT_SUCCESS )
    {
        if (message.event_type == SUPPLICANT_EVENT_EAPOL_PACKET_RECEIVED || message.event_type == SUPPLICANT_EVENT_PACKET_TO_SEND )
        {
            supplicant_host_free_packet(workspace->interface->whd_driver,message.data.packet);
        }
    }
    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_INFO, "Cleanup of TLS Agent event queue done!\r\n");

    cy_rtos_exit_thread();
}

cy_rslt_t supplicant_tls_agent_start( supplicant_workspace_t* workspace )
{
    supplicant_rtos_workspace_t* host_workspace = (supplicant_rtos_workspace_t*) workspace->tls_agent.tls_agent_host_workspace;

    workspace->have_packet = 0;
    return cy_rtos_create_thread(&host_workspace->thread, supplicant_tls_agent_thread, "tls_agent", host_workspace->thread_stack, TLS_AGENT_THREAD_STACK_SIZE, CY_RTOS_PRIORITY_ABOVENORMAL, (cy_thread_arg_t) workspace);
}

cy_rslt_t supplicant_get_result( supplicant_workspace_t* workspace )
{
    return workspace->supplicant_result;
}


void supplicant_set_identity( supplicant_workspace_t* workspace, const uint8_t* eap_identity, uint32_t eap_identity_length )
{
    workspace->outer_eap_identity_length = eap_identity_length;
    memcpy( workspace->outer_eap_identity, eap_identity, SUPPLICANT_DEFS_MIN( eap_identity_length, sizeof(workspace->outer_eap_identity) ) );
}

void supplicant_set_inner_identity( supplicant_workspace_t* workspace, eap_type_t eap_type, void* inner_identity )
{
    eap_type_t inner_eap_type = workspace->inner_eap_type;
    if( eap_type == CY_ENTERPRISE_SECURITY_EAP_TYPE_PEAP ||
            ( (eap_type == CY_ENTERPRISE_SECURITY_EAP_TYPE_TTLS && (inner_eap_type == CY_ENTERPRISE_SECURITY_EAP_TYPE_MSCHAPV2 || inner_eap_type == CY_ENTERPRISE_SECURITY_EAP_TYPE_LEAP) ) ) )
    {
        supplicant_phase2_state_t*        phase2        = (supplicant_phase2_state_t*) workspace->ptr_phase2;
        supplicant_inner_identity_t*      identity = (supplicant_inner_identity_t*) inner_identity;

        if ( phase2 == NULL )
            return;

        phase2->identity_length = SUPPLICANT_DEFS_MIN( identity->identity_length, sizeof(phase2->identity));
        phase2->password_length = SUPPLICANT_DEFS_MIN( identity->password_length, sizeof(phase2->password));
        memcpy( phase2->identity, identity->identity, phase2->identity_length );
        memcpy( phase2->password, identity->password, phase2->password_length );
    }
}

/* This function is called by the WHD thread so do not print from here unless the WHD thread stack size has been increased by 4K to allow for printing. */
void* supplicant_external_event_handler( whd_interface_t ifp, const whd_event_header_t* event_header, const uint8_t* event_data, /*@returned@*/ void* handler_user_data )
{
    supplicant_workspace_t* workspace = (supplicant_workspace_t*) handler_user_data;

    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_INFO, "[%s()] : L%d : Event type = [%u], Status = [%u], Reason = [%u], Flags = [%u]\r\n", __FUNCTION__, __LINE__, (unsigned int)event_header->event_type, (unsigned int)event_header->status,(unsigned int)event_header->reason, (unsigned int)event_header->flags);
    switch ( event_header->event_type )
    {
        case WLC_E_DEAUTH_IND:
        case WLC_E_DISASSOC_IND:
        {
            break;
        }

        case WLC_E_LINK:
            CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_INFO, "Supplicant received link event\r\n");
            if ( ( event_header->flags & WLC_EVENT_MSG_LINK ) != 0 )
            {
                if ( workspace->current_sub_stage == SUPPLICANT_EAP_START )
                {
                    workspace->current_main_stage = SUPPLICANT_INITIALISING;
                    cy_rtos_get_time( &workspace->start_time );

                    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "[%s()] : L%d : Start timer to wait for EAPOL ID request\r\n", __FUNCTION__, __LINE__);
                    supplicant_host_start_timer( workspace->supplicant_host_workspace, EAPOL_PACKET_TIMEOUT ); /* Start a timer to wait for EAP ID Request */
                }

            }
            break;

        default:
            break;
    }

    return handler_user_data;
}

cy_rslt_t supplicant_management_set_event_handler( supplicant_workspace_t* workspace, cy_bool_t enable )
{
    whd_result_t result;
    static uint16_t events = 0xFF;

    if ( enable == CY_TRUE )
    {
        result = whd_wifi_set_event_handler(( whd_interface_t ) workspace->interface,(const uint32_t*)supplicant_events, supplicant_external_event_handler, workspace, &events );

    }
    else
    {
        result = whd_wifi_deregister_event_handler( ( whd_interface_t ) workspace->interface, events );
    }

    if ( result != WHD_SUCCESS )
    {
        CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_ERR, "Error setting supplicant event handler %d\r\n", (unsigned int)result);
    }

    return (cy_rslt_t) result;
}

/* Create a Phase2 workspace to run EAP in it. */
cy_rslt_t supplicant_phase2_init( supplicant_workspace_t* workspace, eap_type_t type )
{
    supplicant_phase2_workspace_t* phase2_workspace;

    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Start supplicant_phase2_init\r\n");
    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "[%s()] : L%d : MEM : MALLOC : Allocating PEAP workspace\r\n", __FUNCTION__, __LINE__);
    phase2_workspace = supplicant_host_malloc( "supplicant phase2 workspace ", sizeof(supplicant_phase2_workspace_t) );
    if ( phase2_workspace == NULL )
    {
        CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_ERR, "Can't allocate memory for peap workspace, out of heap space!\r\n");
        return CY_RSLT_ENTERPRISE_SECURITY_NOMEM;
    }
    memset( phase2_workspace, 0, sizeof(supplicant_phase2_workspace_t) );
    workspace->ptr_phase2 = phase2_workspace;

#ifdef RTOS_USE_STATIC_THREAD_STACK
    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "[%s()] : L%d : MEM : MALLOC : Allocating Phase2 thread stack\r\n", __FUNCTION__, __LINE__);
    phase2_workspace->thread_stack = supplicant_host_malloc( "supplicant Phase2 stack", SUPPLICANT_THREAD_STACK_SIZE );
    if ( phase2_workspace->thread_stack == NULL )
    {
        supplicant_host_free( workspace->ptr_phase2 );
        workspace->ptr_phase2 = NULL;
        CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_ERR, "Can't allocate memory for Phase2 thread stack, out of heap space!\r\n");
        return CY_RSLT_ENTERPRISE_SECURITY_NOMEM;
    }
    memset( phase2_workspace->thread_stack, 0, SUPPLICANT_THREAD_STACK_SIZE );
#else
    phase2_workspace->thread_stack = NULL;
#endif

    phase2_workspace->state.eap_type   = type;
    phase2_workspace->state.result     = CY_RSLT_ENTERPRISE_SECURITY_SUPPLICANT_NOT_STARTED;
    phase2_workspace->state.main_stage = SUPPLICANT_INITIALISING;
    phase2_workspace->state.sub_stage  = SUPPLICANT_EAP_START;

    supplicant_set_inner_identity( workspace, workspace->eap_type, &workspace->inner_identity );

    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_INFO, "supplicant_phase2_init done\r\n");

    return CY_RSLT_SUCCESS;
}

cy_rslt_t supplicant_inner_packet_set_data( whd_driver_t whd_driver,supplicant_packet_t* packet, int32_t size )
{
    return (cy_rslt_t) whd_buffer_add_remove_at_front( whd_driver,(whd_buffer_t *) packet, size );
}

void supplicant_phase2_thread( cy_thread_arg_t arg )
{
    supplicant_workspace_t*      workspace      = (supplicant_workspace_t*) arg;
    supplicant_phase2_workspace_t* phase2_workspace = workspace->ptr_phase2;
    cy_rslt_t (* phase2_event_handler ) (supplicant_workspace_t* , supplicant_packet_t) = NULL;

    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_INFO, "\r\nsupplicant_peap_thread started...\r\n");

    if (workspace->eap_type == CY_ENTERPRISE_SECURITY_EAP_TYPE_PEAP)
    {
        phase2_event_handler = supplicant_process_peap_event;
    }
    else if (workspace->eap_type == CY_ENTERPRISE_SECURITY_EAP_TYPE_TTLS)
    {
        phase2_event_handler = supplicant_process_ttls_phase2_event;
    }
    else
    {
        CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "The eap type is not supported\r\n");
        phase2_workspace->state.result = CY_RSLT_ENTERPRISE_SECURITY_SUPPLICANT_ERROR;
    }

    /* Wait until TLS is done or phase2 state is aborted */
    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_INFO, "Wait for TLS handshake to finish or abort\r\n");
#ifdef COMPONENT_MBEDTLS
    while ( workspace->tls_context->context.state != MBEDTLS_SSL_HANDSHAKE_OVER &&
            phase2_workspace->state.result != CY_RSLT_ENTERPRISE_SECURITY_SUPPLICANT_ABORTED )
#elif defined (COMPONENT_NETXSECURE)
    while ( workspace->tls_context->tls_handshake_successful != true &&
            phase2_workspace->state.result != CY_RSLT_ENTERPRISE_SECURITY_SUPPLICANT_ABORTED )
#endif
    {
        cy_rtos_delay_milliseconds( 10 );
    }


    if ( phase2_workspace->state.result == CY_RSLT_ENTERPRISE_SECURITY_SUPPLICANT_ABORTED )
    {
        CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Phase2 is aborted\r\n");
    }
    else
    {
        CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_INFO, "TLS handshake finished. Start PEAP processing loop\r\n");
    }

    while ( phase2_workspace->state.result == CY_RSLT_ENTERPRISE_SECURITY_SUPPLICANT_IN_PROGRESS )
    {
        cy_rslt_t result;
        supplicant_packet_t packet;
#ifdef COMPONENT_MBEDTLS
        uint8_t data[1024]= {0};
#endif
        supplicant_buffer_t buffer;
        memset(&buffer, 0, sizeof(buffer));
#ifdef COMPONENT_MBEDTLS
        buffer.payload = data;
        packet =(supplicant_packet_t) &buffer;
#endif

        CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Waiting to receive EAP packet\r\n");

        result = cy_tls_receive_eap_packet( workspace, &packet );

        if ( result == CY_RSLT_SUCCESS )
        {
            phase2_event_handler( workspace, packet );
            cy_tls_free_eap_packet(packet);
        }
        else
        {
            cy_rtos_delay_milliseconds( 10 );
        }
    }
    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_INFO, "supplicant_phase2_thread end\r\n");

    cy_rtos_exit_thread();
}

cy_rslt_t supplicant_phase2_start( supplicant_workspace_t* workspace )
{
    supplicant_phase2_workspace_t* phase2_workspace = workspace->ptr_phase2;

    /* validating input parameters */
    if(workspace->eap_type == CY_ENTERPRISE_SECURITY_EAP_TYPE_TTLS)
    {
        if(workspace->tunnel_auth_type != CY_ENTERPRISE_SECURITY_TUNNEL_TYPE_EAP)
        {
            return CY_RSLT_ENTERPRISE_SECURITY_SUPPLICANT_ERROR;
        }
        if(!( workspace->inner_eap_type == CY_ENTERPRISE_SECURITY_EAP_TYPE_LEAP || workspace->inner_eap_type == CY_ENTERPRISE_SECURITY_EAP_TYPE_MSCHAPV2))
        {
            return CY_RSLT_ENTERPRISE_SECURITY_SUPPLICANT_ERROR;
        }
    }
    else if(workspace->eap_type != CY_ENTERPRISE_SECURITY_EAP_TYPE_PEAP)
    {
        return CY_RSLT_ENTERPRISE_SECURITY_SUPPLICANT_ERROR;
    }

    phase2_workspace->state.result = CY_RSLT_ENTERPRISE_SECURITY_SUPPLICANT_IN_PROGRESS;
    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_INFO, "Start supplicant_phase2_start\r\n");
    return (cy_rslt_t) cy_rtos_create_thread( &phase2_workspace->thread, supplicant_phase2_thread, "phase2", phase2_workspace->thread_stack, SUPPLICANT_THREAD_STACK_SIZE, CY_RTOS_PRIORITY_ABOVENORMAL, (cy_thread_arg_t) workspace );
}

cy_rslt_t supplicant_process_event(supplicant_workspace_t* workspace, supplicant_event_message_t* message)
{
    eap_packet_t*             packet =  NULL;
    tls_agent_event_message_t tls_agent_message;
    supplicant_event_message_t      outgoing_packet;
    uint16_t len;

    // CY_SUPPLICANT_PROCESS_ET_INFO(( "[%s()] : L%d : BESL Event Type = [%s]\r\n", __FUNCTION__, __LINE__, SUPPLICANT_EVENT_to_string(message->event_type) ));
    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_INFO, "[%s()] : L%d : BESL Event Type = [%d]\r\n", __FUNCTION__, __LINE__, message->event_type );
    /* Process the event */
    switch ( message->event_type )
    {
        case SUPPLICANT_EVENT_ABORT_REQUESTED:
            CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_INFO, "[%s()] : L%d : SUPPLICANT_EVENT_ABORT_REQUESTED\r\n", __FUNCTION__, __LINE__);
            workspace->supplicant_result = CY_RSLT_ENTERPRISE_SECURITY_SUPPLICANT_ABORTED;
            break;

        case SUPPLICANT_EVENT_EAPOL_PACKET_RECEIVED:
            CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_INFO, "[%s()] : L%d : SUPPLICANT_EVENT_EAPOL_PACKET_RECEIVED\r\n", __FUNCTION__, __LINE__);
            packet = (eap_packet_t*)whd_buffer_get_current_piece_data_pointer(workspace->interface->whd_driver,message->data.packet);

            /* Check for identity request */
            if ( packet->eap.code == EAP_CODE_REQUEST )
            {
                workspace->last_received_id = packet->eap.id;
                if ( packet->eap.type == CY_ENTERPRISE_SECURITY_EAP_TYPE_IDENTITY )
                {
                    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Trying to cleanup before starting a fresh handshake...\r\n");
                    supplicant_eap_handshake_cleanup( workspace );

                    if (workspace->current_sub_stage == SUPPLICANT_EAP_START )
                    {
                        CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_INFO, "[%s()] : L%d : SUPPLICANT_EAP_START\r\n", __FUNCTION__, __LINE__);
                        supplicant_get_bssid(  workspace->interface,&workspace->authenticator_mac_address );
                    }

                    cy_rtos_get_time( &workspace->eap_handshake_start_time );
                    cy_rtos_get_time( &workspace->start_time  );
                    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Supplicant received EAP ID Request a time = [%lu]\r\n", workspace->eap_handshake_start_time);

                    /* Stop the EAPOL start timer */
                    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "[%s()] : L%d : Stop wait timer registered for EAPOL ID request\r\n", __FUNCTION__, __LINE__);
                    supplicant_host_stop_timer( workspace->supplicant_host_workspace );
                    workspace->current_sub_stage = SUPPLICANT_EAP_IDENTITY;
                    supplicant_send_eap_response_packet( workspace, CY_ENTERPRISE_SECURITY_EAP_TYPE_IDENTITY, (uint8_t*)&workspace->outer_eap_identity, workspace->outer_eap_identity_length );
                    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "[%s()] : L%d : Sent EAP ID Response\r\n", __FUNCTION__, __LINE__);
                }
                else if ( ( packet->eap.type != workspace->eap_type ) && ( workspace->current_sub_stage == SUPPLICANT_EAP_IDENTITY ) )
                {
                    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Supplicant received EAP Type %u\r\n", packet->eap.type);
                    workspace->current_sub_stage = SUPPLICANT_EAP_NAK;
                    supplicant_send_eap_response_packet( workspace, CY_ENTERPRISE_SECURITY_EAP_TYPE_NAK, (uint8_t*)&workspace->eap_type, 1 );

                }
                else if ( ( packet->eap.type == workspace->eap_type ) && ( ( workspace->current_sub_stage == SUPPLICANT_EAP_IDENTITY ) || ( workspace->current_sub_stage == SUPPLICANT_EAP_NAK ) ) )
                {
                    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Supplicant received required EAP Type %u\r\n", packet->eap.type);

                    workspace->current_sub_stage = SUPPLICANT_EAP_METHOD;
                    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "[%s()] : L%d : Supplicant agent init\r\n", __FUNCTION__, __LINE__);
                    if ( supplicant_tls_agent_init( &workspace->tls_agent ) != CY_RSLT_SUCCESS )
                    {

                        CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Supplicant TLS agent initialisation failed\r\n");
                    }
                    else
                    {
                        CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "[%s()] : L%d : Supplicant agent start\r\n", __FUNCTION__, __LINE__);
                        if ( supplicant_tls_agent_start( workspace ) != CY_RSLT_SUCCESS )
                        {

                            CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_ERR, "Supplicant TLS agent failed to start\r\n");
                        }
                        CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_INFO, "[%s()] : L%d : Supplicant agent start completed\r\n", __FUNCTION__, __LINE__);

                    }
                }
                else if ( ( packet->eap.type == workspace->eap_type ) && ( workspace->current_sub_stage == SUPPLICANT_EAP_METHOD ) )
                {
                    eap_tls_packet_t* eap_tls_packet;
                    int32_t          header_overhead = sizeof( eap_tls_packet_t ) - 1;
                    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "[%s()] : L%d : Should be a EAPOL packet for TLS/PEAP/Other. workspace->eap_type = [%d]\r\n", __FUNCTION__, __LINE__, workspace->eap_type);
                    if ( ( workspace->eap_type == CY_ENTERPRISE_SECURITY_EAP_TYPE_TLS ) || ( workspace->eap_type == CY_ENTERPRISE_SECURITY_EAP_TYPE_PEAP ) || ( workspace->eap_type == CY_ENTERPRISE_SECURITY_EAP_TYPE_TTLS ))
                    {
                        eap_tls_packet = (eap_tls_packet_t*)packet;

                        /* Send a zero length EAP Response if there are more fragments to be received */
                        if ( eap_tls_packet->eap_tls.flags & EAP_TLS_FLAG_MORE_FRAGMENTS )
                        {
                            supplicant_send_zero_length_eap_tls_packet( workspace );
                        }
                        workspace->tls_length_overhead = 0;
                        if ( eap_tls_packet->eap_tls.flags & EAP_TLS_FLAG_LENGTH_INCLUDED )
                        {
                            header_overhead += 4;
                            /* This is introduced as a work around for the conflict in handling the length flag of
                             * TLS record headers. Some servers seem to crank if the length is present while others go
                             * wrong when the length is not present ( see wpa_supplicant comments for Windows Server 2008 NPS
                             * (eap_tls_common.c)). As a workaround, WICED will detect what flag type the server is using and
                             * imitate that. This is most pronounced in PEAP.
                             */
                            workspace->tls_length_overhead = 4;
                        }

                        len = supplicant_host_hton16( eap_tls_packet->eap.length );
                        CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "EAP packet received with EAP ID = [%u], len = %d \r\n", (unsigned int)eap_tls_packet->eap.id, len);
                        supplicant_dump_bytes((uint8_t*)&eap_tls_packet->eap, SUPPLICANT_READ_16_BE( (uint8_t *)&eap_tls_packet->eap.length ));

                        /* For EAP-TTLS, allowed EAP packet with length 6 to be send to TLS agent. This message is required in case of
                         * TLS1.3, which act as the request for Phase2 start. In EAP-TTLS case, TLS agent will wait for this packet
                         * after the TLS handshake complete to start Phase2.
                         */
                        if ( supplicant_host_hton16( eap_tls_packet->eap.length ) > 6
                                || ( supplicant_host_hton16( eap_tls_packet->eap.length ) == 6
                                        && workspace->eap_type == CY_ENTERPRISE_SECURITY_EAP_TYPE_TTLS ) )
                        {
                            CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "[%s()] : L%d : Move the packet start to the start of TLS data\r\n", __FUNCTION__, __LINE__);

                            /* Move the packet data pointer so it points at the start of TLS data */
                            supplicant_host_consume_bytes(workspace->interface->whd_driver,&message->data.packet, header_overhead);

                            /* Push the current packet to the TLS agent */
                            tls_agent_message.event_type  = TLS_AGENT_EVENT_EAPOL_PACKET;
                            tls_agent_message.data.packet = message->data.packet;

                            if ( len - (header_overhead - sizeof(eapol_packet_header_t)) > 0)
                            {
                                tls_agent_message.length = len - (header_overhead - sizeof(eapol_packet_header_t));
                            }
                            else
                            {
                               tls_agent_message.length = 0;
                            }
                            CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "[%s()] : L%d : Send the TLS packet to TLS Agent\r\n", __FUNCTION__, __LINE__);

                            supplicant_send_eap_tls_packet( workspace, &tls_agent_message, SUPPLICANT_NEVER_TIMEOUT );

                            /* Break so the packet is not freed by this thread */
                            break;
                        }
                        else
                        {
                            CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "[%s()] : L%d : Received a Zero length EAP request\r\n", __FUNCTION__, __LINE__);
                            /* It's a zero length EAP request */
                            cy_rslt_t result = supplicant_outgoing_pop( workspace->supplicant_host_workspace, &outgoing_packet );
                            if ( result == CY_RSLT_SUCCESS )
                            {
                                CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "[%s()] : L%d : Calling supplicant_send_eap_tls_fragment()\r\n", __FUNCTION__, __LINE__);
                                supplicant_send_eap_tls_fragment( workspace, outgoing_packet.data.packet );
                            }
                            else
                            {
                                CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_ERR, "Problem with outgoing packet dequeue %u    \r\n", (unsigned int)result);
                            }
                        }
                    }
                }
            }
            /* Check for EAP fail */
            else if ( ( packet->eap.code == EAP_CODE_FAILURE ) || ( packet->eap.code == EAP_CODE_SUCCESS ) )
            {
                CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Received EAP %s!\r\n", ( packet->eap.code == EAP_CODE_SUCCESS ) ? "Success" : "Failed");
                workspace->supplicant_result = ( packet->eap.code == EAP_CODE_SUCCESS ) ? CY_RSLT_SUCCESS : CY_RSLT_ENTERPRISE_SECURITY_EAP_ERROR;

                CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Free the resources and reset the state\r\n");
                supplicant_eap_handshake_cleanup( workspace );

                workspace->current_main_stage = SUPPLICANT_INITIALISED;
            }
            supplicant_host_free_packet(workspace->interface->whd_driver, message->data.packet );

            break;

        case SUPPLICANT_EVENT_TIMER_TIMEOUT:
            CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Supplicant event timer timeout\r\n");
            if ( ( workspace->current_main_stage == SUPPLICANT_INITIALISING ) && ( workspace->current_sub_stage == SUPPLICANT_EAP_START ) )
            {
                supplicant_get_bssid( workspace->interface, &workspace->authenticator_mac_address );
                CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "BSSID is: %02X:%02X:%02X:%02X:%02X:%02X\r\n", workspace->authenticator_mac_address.octet[0],
                        workspace->authenticator_mac_address.octet[1],
                        workspace->authenticator_mac_address.octet[2],
                        workspace->authenticator_mac_address.octet[3],
                        workspace->authenticator_mac_address.octet[4],
                        workspace->authenticator_mac_address.octet[5]);
                if ( !NULL_MAC( workspace->authenticator_mac_address.octet ))
                {
                    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "[%s()] : L%d : Send EAPOL Start request\r\n", __FUNCTION__, __LINE__);
                    supplicant_send_eapol_start( workspace );
                }
            }
            break;

        case SUPPLICANT_EVENT_PACKET_TO_SEND:
            CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Supplicant event packet to send\r\n");
            supplicant_send_eap_tls_fragment( workspace, message->data.packet );

            if ( workspace->eap_type == CY_ENTERPRISE_SECURITY_EAP_TYPE_PEAP  &&
#ifdef COMPONENT_MBEDTLS
                    ( workspace->tls_context->context.state  >= MBEDTLS_SSL_CLIENT_FINISHED )
#elif defined (COMPONENT_NETXSECURE)
                    ( workspace->tls_context->context.nx_secure_tls_client_state == NX_SECURE_TLS_CLIENT_STATE_SERVERHELLO_DONE  ||
                      workspace->tls_context->context.nx_secure_tls_client_state == NX_SECURE_TLS_CLIENT_STATE_HANDSHAKE_FINISHED )
#endif
            )
            {
                CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Initialize PEAP thread. workspace->tunnel_auth_type %d\r\n", workspace->tunnel_auth_type);
                supplicant_phase2_init(workspace, CY_ENTERPRISE_SECURITY_EAP_TYPE_MSCHAPV2); //workspace->tunnel_auth_type );

                CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_INFO, "Start PEAP thread.\r\n");
                if (supplicant_phase2_start(workspace) != CY_RSLT_SUCCESS)
                {
                    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_ERR, "Supplicant PEAP failed to start\r\n");
                }
            }
            break;

        default:
            CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "[%s()] : L%d : CY_SUPPLICANT_UNPROCESSED\r\n", __FUNCTION__, __LINE__);
            return CY_RSLT_ENTERPRISE_SECURITY_SUPPLICANT_UNPROCESSED;
            break;
    }

    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_INFO, "\r\n[%s()] : L%d : Returning successfully from %s\r\n", __FUNCTION__, __LINE__, __FUNCTION__);

    return CY_RSLT_SUCCESS;
}

void supplicant_thread_main( cy_thread_arg_t arg )
{
    cy_time_t                    current_time;
    cy_rslt_t                    result;
    supplicant_event_message_t   message;
    supplicant_workspace_t*      workspace = (supplicant_workspace_t*)arg;
    supplicant_rtos_workspace_t* host = &((supplicant_host_workspace_t*)workspace->supplicant_host_workspace)->host_workspace;

    workspace->supplicant_result = CY_RSLT_ENTERPRISE_SECURITY_SUPPLICANT_IN_PROGRESS;

    /* Now that our queue is initialized we can flag the workspace as active */
    IF_TO_WORKSPACE( workspace->interface->role ) = workspace;

    cy_rtos_get_time( &workspace->start_time );

    while ( workspace->supplicant_result != CY_RSLT_ENTERPRISE_SECURITY_SUPPLICANT_ABORTED )
    {
        uint32_t time_to_wait;

        CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_INFO, "[%s()] : L%d : In processing loop...\r\n", __FUNCTION__, __LINE__);

        cy_rtos_get_time ( &workspace->eap_handshake_current_time );

        if ( ( workspace->eap_handshake_start_time != 0 ) && ( ( workspace->eap_handshake_current_time - workspace->eap_handshake_start_time ) >= EAP_HANDSHAKE_TIMEOUT_IN_MSEC ) )
        {
            CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_INFO, "EAP Handshake timed out...\r\n");
            supplicant_eap_handshake_cleanup( workspace );
        }

        /* Update EAP timeout values based on EAP handshake state */
        if ( workspace->current_main_stage != SUPPLICANT_INITIALISED )
        {
            cy_rtos_get_time( &current_time );

            if (( current_time - workspace->start_time ) >= SUPPLICANT_HANDSHAKE_ATTEMPT_TIMEOUT )
            {
                CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Total handshake timeout expired. Quit processing\r\n");
                workspace->supplicant_result = CY_RSLT_ENTERPRISE_SECURITY_SUPPLICANT_ABORTED;
                continue;
            }

            CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Calculate the wait time\r\n");
            time_to_wait = ( SUPPLICANT_HANDSHAKE_ATTEMPT_TIMEOUT ) - ( current_time - workspace->start_time );
            if ( host->timer_timeout != 0 )
            {
                CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Timer timeout is enabled. Find the min wait time again\r\n");
                time_to_wait = SUPPLICANT_DEFS_MIN( time_to_wait, host->timer_timeout - (current_time - host->timer_reference));
            }
        }
        else
        {
            time_to_wait = EAPOL_PACKET_TIMEOUT;
        }

        CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Waiting for EAPOL packet\r\n");
        if ( cy_rtos_get_queue( &host->event_queue, &message, time_to_wait, 0 ) != CY_RSLT_SUCCESS )
        {
            /* Create a timeout message to process */
            CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_INFO, "\r\nEAPOL packet wait timed out\r\n");
            message.event_type = SUPPLICANT_EVENT_TIMER_TIMEOUT;
            message.data.value = 0;
        }

        /* Process the message */
        CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "\r\nProcess EAPOL packet\r\n");

        result = supplicant_process_event( workspace, &message );
        if ( result != CY_RSLT_SUCCESS )
        {
            CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_ERR, "Supplicant error %u\r\n", (unsigned int)result);
        }
    }

    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Terminate TLS agent and PEAP threads, if running\r\n");
    supplicant_eap_handshake_cleanup( workspace );
}

void supplicant_thread( cy_thread_arg_t arg )
{
    supplicant_thread_main( arg );

    cy_rtos_exit_thread();
}
/* This function is called by the WHD thread so do not print from here unless the WHD thread stack size has been increased by 4K to allow for printing. */
void supplicant_eapol_packet_handler( whd_interface_t interface, whd_buffer_t buffer )
{
    supplicant_workspace_t* workspace;

    if ( interface->role == WHD_STA_ROLE )
    {
        workspace = IF_TO_WORKSPACE( interface->role );

        if ( workspace == NULL )
        {
            whd_buffer_release( interface->whd_driver,buffer, WHD_NETWORK_RX );
        }
        else
        {
            supplicant_queue_message_packet( workspace, SUPPLICANT_EVENT_EAPOL_PACKET_RECEIVED, buffer );
        }
    }
    else
    {
        CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_ERR, "EAPOL packet arriving on incorrect interface %u\r\n", (unsigned int) interface);
    }
}
cy_rslt_t supplicant_init(supplicant_workspace_t* workspace, supplicant_connection_info_t *conn_info)
{
    supplicant_host_workspace_t* supplicant_host_workspace = NULL;
    cy_rslt_t result = CY_RSLT_SUCCESS;
    cy_rslt_t cy_result = CY_RSLT_SUCCESS;
    /* validating ttls input parameters */
    if(conn_info->eap_type == CY_ENTERPRISE_SECURITY_EAP_TYPE_TTLS)
    {
        if(conn_info->tunnel_auth_type != CY_ENTERPRISE_SECURITY_TUNNEL_TYPE_EAP)
        {
            return CY_RSLT_ENTERPRISE_SECURITY_SUPPLICANT_ERROR;
        }
        if(!( conn_info->inner_eap_type == CY_ENTERPRISE_SECURITY_EAP_TYPE_LEAP || conn_info->inner_eap_type == CY_ENTERPRISE_SECURITY_EAP_TYPE_MSCHAPV2))
        {
            return CY_RSLT_ENTERPRISE_SECURITY_SUPPLICANT_ERROR;
        }
    }

    /* key/cert is mandatory for EAP-TLS */
    if ( conn_info->eap_type == CY_ENTERPRISE_SECURITY_EAP_TYPE_TLS
            && ( conn_info->private_key == NULL || conn_info->user_cert == NULL ))
    {
        CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_ERR, "Invalid Key/Certificate for EAP-TLS\r\n");
        return CY_RSLT_ENTERPRISE_SECURITY_SUPPLICANT_ERROR;
    }

    if ( conn_info->private_key && conn_info->user_cert )
    {
        cy_result = cy_tls_init_identity( conn_info->tls_identity, (char*) conn_info->private_key, conn_info->key_length, conn_info->user_cert, conn_info->user_cert_length );
    }
    else
    {
        cy_result = cy_tls_init_identity( conn_info->tls_identity, NULL, 0, NULL, 0 );
    }

    if ( cy_result != CY_RSLT_SUCCESS )
    {
        return CY_RSLT_ENTERPRISE_SECURITY_SUPPLICANT_ERROR;
    }

    workspace->tls_context = conn_info->context;
    cy_result = cy_tls_init_root_ca_certificates( workspace->tls_context,(char*) conn_info->trusted_ca_certificates, conn_info->root_ca_cert_length );
    if ( cy_result != CY_RSLT_SUCCESS )
    {
        result = CY_RSLT_ENTERPRISE_SECURITY_SUPPLICANT_ERROR;
        goto EXIT_FROM_TLS_INIT_CONTEXT;
    }

    cy_result = cy_tls_init_context( conn_info->context, conn_info->tls_identity, NULL );
    if ( cy_result != CY_RSLT_SUCCESS )
    {
        result = CY_RSLT_ENTERPRISE_SECURITY_SUPPLICANT_ERROR;
        goto EXIT_FROM_TLS_INIT_IDENTITY;
    }

    /* Register the EAPOL packet receive handler */
    result = (cy_rslt_t) suppliant_emac_register_eapol_packet_handler( supplicant_eapol_packet_handler );
    if ( result != CY_RSLT_SUCCESS )
    {
        CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_ERR, "ERROR : EAPOL hndlr registration failed with error = [%d]\r\n", result);
        goto EXIT_FROM_TLS_INIT_ROOT_CA;
    }

    /* Allocate memory for the supplicant host workspace */
    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_INFO, "[%s()] : L%d : MEM : MALLOC : Allocating supplicant host workspace\r\n", __FUNCTION__, __LINE__);
    supplicant_host_workspace = supplicant_host_calloc( "supplicant host", 1, sizeof(supplicant_host_workspace_t) );
    if ( supplicant_host_workspace == NULL )
    {
        result = CY_RSLT_ENTERPRISE_SECURITY_NOMEM;
        CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_ERR, "ERROR : Couldn't allocate supplicant host workspace\r\n");
        goto EXIT_FROM_EAPOL_RECV_HANDLER_REGISTRATION;
    }
    workspace->supplicant_host_workspace = supplicant_host_workspace;

    /* Allocate memory for the EAP defragmentation buffer */
    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_INFO, "[%s()] : L%d : MEM : MALLOC : Allocating supplicant defragmentation buffer\r\n", __FUNCTION__, __LINE__);
    workspace->buffer = supplicant_host_calloc( "supplicant buffer", 1, SUPPLICANT_BUFFER_SIZE  );
    if ( workspace->buffer == NULL )
    {
        result = CY_RSLT_ENTERPRISE_SECURITY_NOMEM;
        CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_ERR, "ERROR : Couldn't allocate EAPOL packet defragmentation buffer\r\n");
        goto EXIT_FROM_FREEING_SUPPLICANT_HOST_WORKSPACE;
    }
    workspace->buffer_size = SUPPLICANT_BUFFER_SIZE;
    workspace->auth_type = conn_info->auth_type;

    /* Allocate memory for the supplicant host thread stack */
#ifdef RTOS_USE_STATIC_THREAD_STACK
    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_INFO, "[%s()] : L%d : MEM : MALLOC : Allocating supplicant thread stack\r\n", __FUNCTION__, __LINE__);
    supplicant_host_workspace->host_workspace.thread_stack = supplicant_host_calloc("supplicant thread stack", 1, SUPPLICANT_THREAD_STACK_SIZE);
    if (supplicant_host_workspace->host_workspace.thread_stack == NULL)
    {
        result = CY_RSLT_ENTERPRISE_SECURITY_NOMEM;
        goto EXIT_FROM_FREEING_SUPPLICANT_EAP_DEFRAG_BUFFER;
    }
#else
    supplicant_host_workspace->host_workspace.thread_stack = NULL;
#endif

    supplicant_host_workspace->host_workspace.interface = conn_info->interface;
    workspace->interface = conn_info->interface;
    whd_wifi_get_mac_address( (whd_interface_t) workspace->interface ,(whd_mac_t*) &workspace->supplicant_mac_address );

    supplicant_init_state( workspace, conn_info->eap_type );
    workspace->current_main_stage = SUPPLICANT_INITIALISING;
    workspace->supplicant_result = CY_RSLT_ENTERPRISE_SECURITY_SUPPLICANT_NOT_STARTED;

    if(workspace->eap_type == CY_ENTERPRISE_SECURITY_EAP_TYPE_TTLS)
    {
        workspace->inner_eap_type = conn_info->inner_eap_type;
        workspace->tunnel_auth_type = conn_info->tunnel_auth_type;
    }
    else if (workspace->eap_type == CY_ENTERPRISE_SECURITY_EAP_TYPE_PEAP)
    {
        workspace->inner_eap_type = conn_info->inner_eap_type;
        workspace->tunnel_auth_type = conn_info->tunnel_auth_type;
    }

    cy_result = cy_rtos_init_queue( &supplicant_host_workspace->host_workspace.event_queue, 15, sizeof(supplicant_event_message_t) );
    if ( cy_result != CY_RSLT_SUCCESS )
    {
        result = CY_RSLT_ENTERPRISE_SECURITY_SUPPLICANT_ERROR;
        goto EXIT_FROM_FREEING_SUPPLICANT_EAP_DEFRAG_BUFFER;
    }

    cy_result = cy_rtos_init_queue( &supplicant_host_workspace->outgoing_packet_queue, 10, sizeof(supplicant_event_message_t) );
    if ( cy_result != CY_RSLT_SUCCESS )
    {
        result = CY_RSLT_ENTERPRISE_SECURITY_SUPPLICANT_ERROR;
        goto EXIT_FROM_DEINIT_HOST_WORKSPACE_EVENT_QUEUE;
    }
    supplicant_enable_tls( workspace, conn_info->context );
    supplicant_set_identity( workspace, conn_info->eap_identity, strlen( (char*) conn_info->eap_identity ) );

    if ( conn_info->eap_type == CY_ENTERPRISE_SECURITY_EAP_TYPE_PEAP || conn_info->eap_type == CY_ENTERPRISE_SECURITY_EAP_TYPE_TTLS )
    {
        /* Preserve user name and password for PEAP and EAP-TTLS*/
        /* Convert the password from ASCII to UTF16 */
        int i;
        uint8_t* password = (uint8_t*) conn_info->password;
        uint8_t* unicode = (uint8_t*) workspace->inner_identity.password;

        int tem_len = strlen( (char*) conn_info->password );

        CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_INFO, "Initialize inner identity and password\r\n");

        for ( i = 0; i <= tem_len; i++ )
        {
            *unicode++ = *password++;
            *unicode++ = '\0';
        }
        workspace->inner_identity.password_length = 2 * ( i - 1 );

        workspace->inner_identity.identity_length = SUPPLICANT_DEFS_MIN( sizeof( workspace->inner_identity.identity ), strlen( (char* )conn_info->user_name ) );
        memcpy( workspace->inner_identity.identity, conn_info->user_name, workspace->inner_identity.identity_length );
    }

    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "set supplicant event handlers\r\n");
    if ( supplicant_management_set_event_handler( workspace, CY_TRUE ) != CY_RSLT_SUCCESS )
    {
        CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_ERR, "Supplicant unable to set management event handler.\r\n");
        goto EXIT_FROM_DEINIT_HOST_WORKSPACE_OUTGOING_PACKET_QUEUE;
    }
    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "set supplicant event handlers end\r\n");

    return CY_RSLT_SUCCESS;

EXIT_FROM_DEINIT_HOST_WORKSPACE_OUTGOING_PACKET_QUEUE:
    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Deinitializing host workspace outgoing packet queue\r\n");
    cy_rtos_deinit_queue( &supplicant_host_workspace->outgoing_packet_queue );

EXIT_FROM_DEINIT_HOST_WORKSPACE_EVENT_QUEUE:
    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Deinitializing host workspace event queue\r\n");
    cy_rtos_deinit_queue( &supplicant_host_workspace->host_workspace.event_queue );

EXIT_FROM_FREEING_SUPPLICANT_EAP_DEFRAG_BUFFER:
#ifdef RTOS_USE_STATIC_THREAD_STACK
    if ( supplicant_host_workspace->host_workspace.thread_stack != NULL )
    {
        CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Freeing thread stack\r\n");
        supplicant_host_free( supplicant_host_workspace->host_workspace.thread_stack );
        supplicant_host_workspace->host_workspace.thread_stack = NULL;
    }
#endif /*RTOS_USE_STATIC_THREAD_STACK*/
    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Freeing eap defragmentation buffer\r\n");
    supplicant_host_free( workspace->buffer );
    workspace->buffer = NULL;

EXIT_FROM_FREEING_SUPPLICANT_HOST_WORKSPACE:
    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Freeing supplicant host workspace\r\n");
    supplicant_host_free( workspace->supplicant_host_workspace );
    workspace->supplicant_host_workspace = NULL;

EXIT_FROM_EAPOL_RECV_HANDLER_REGISTRATION:
    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Deregister Eapol receive handler\r\n");
    cy_ent_sec_unregister_eapol_packet_handler( );

EXIT_FROM_TLS_INIT_ROOT_CA:
    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Deinitialize TLS ROOT CA\r\n");
    cy_tls_deinit_root_ca_certificates(workspace->tls_context );

EXIT_FROM_TLS_INIT_CONTEXT:
    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Deinitialize TLS context\r\n");
    cy_tls_deinit_context( conn_info->context );

EXIT_FROM_TLS_INIT_IDENTITY:
    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Deinitialize TLS identity\r\n");
    cy_tls_deinit_identity( conn_info->tls_identity );

return result;
}

cy_rslt_t supplicant_deinit( supplicant_workspace_t* workspace )
{
    supplicant_host_workspace_t* supplicant_host_workspace = (supplicant_host_workspace_t*) workspace->supplicant_host_workspace;
    supplicant_event_message_t message;

    /* Check if supplicant is started before de-initializing it */
    if ( workspace->supplicant_result == CY_RSLT_ENTERPRISE_SECURITY_SUPPLICANT_NOT_STARTED )
    {
        CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "supplicant is not started\r\n");
        return CY_RSLT_ENTERPRISE_SECURITY_SUPPLICANT_ERROR;
    }

    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Unregister EAPOL handler\r\n");
    cy_ent_sec_unregister_eapol_packet_handler( );

    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Reset supplicant event handlers\r\n");
    if ( supplicant_management_set_event_handler( workspace, CY_FALSE ) != CY_RSLT_SUCCESS )
    {
        CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_ERR, "Supplicant unable to set management event handler.\r\n");
    }

    if(supplicant_host_workspace == NULL)
    {
        return CY_RSLT_SUCCESS;
    }

    /* Wait for supplicant thread to exit */
    cy_rtos_join_thread(&supplicant_host_workspace->host_workspace.thread);
    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Delete supplicant thread %p \r\n",supplicant_host_workspace->host_workspace.thread);

    /** Queues clean up **/
    /* Clean up left over messages in the event and outgoing packet queues */
    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Cleanup supplicant event queue\r\n");
    while ( cy_rtos_get_queue( &supplicant_host_workspace->host_workspace.event_queue, &message, SUPPLICANT_NO_WAIT, 0 ) == CY_RSLT_SUCCESS )
    {
        if ( message.event_type == SUPPLICANT_EVENT_EAPOL_PACKET_RECEIVED || message.event_type == SUPPLICANT_EVENT_PACKET_TO_SEND )
        {
            supplicant_host_free_packet(workspace->interface->whd_driver, message.data.packet );
        }
    }
    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Cleanup supplicant outgoing packet queue\r\n");
    while ( cy_rtos_get_queue( &( (supplicant_host_workspace_t*) workspace->supplicant_host_workspace )->outgoing_packet_queue, &message, SUPPLICANT_NO_WAIT, 0 ) == CY_RSLT_SUCCESS )
    {
        if ( message.event_type == SUPPLICANT_EVENT_PACKET_TO_SEND )
        {
            supplicant_host_free_packet(workspace->interface->whd_driver, message.data.packet );
        }
    }

    if ( supplicant_host_workspace != NULL )
    {
        IF_TO_WORKSPACE( supplicant_host_workspace->host_workspace.interface->role ) = NULL;

        CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Deinit and free host queues\r\n");
        cy_rtos_deinit_queue( &supplicant_host_workspace->host_workspace.event_queue );
        cy_rtos_deinit_queue( &supplicant_host_workspace->outgoing_packet_queue );
        CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_INFO, "Deinited and free host queues\r\n");

        if ( supplicant_host_workspace->host_workspace.thread_stack != NULL )
        {
            CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_INFO, "MEM : FREE : Free supplicant thread stack\r\n");
            supplicant_host_free( supplicant_host_workspace->host_workspace.thread_stack );
            supplicant_host_workspace->host_workspace.thread_stack = NULL;
        }

        CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_INFO, "MEM : FREE : Free supplicant host workspace\r\n");
        supplicant_host_free( supplicant_host_workspace );
        workspace->supplicant_host_workspace = NULL;
    }
    workspace->supplicant_result = CY_RSLT_ENTERPRISE_SECURITY_SUPPLICANT_NOT_STARTED;

    if ( workspace->buffer != NULL )
    {
        CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_INFO, "MEM : FREE : Free supplicant defragmentation buffer\r\n");
        supplicant_host_free( workspace->buffer );
        workspace->buffer = NULL;
    }

    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Deinitialize TLS context, cert and identity\r\n");
    cy_tls_deinit_root_ca_certificates(workspace->tls_context);
    cy_tls_deinit_identity( workspace->tls_context->identity );
    cy_tls_deinit_context( workspace->tls_context );

    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_INFO, "Exit success!\r\n");

    return CY_RSLT_SUCCESS;
}

cy_rslt_t supplicant_stop( supplicant_workspace_t* workspace )
{
    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Supplicant result = [%d]\r\n", workspace->supplicant_result);
    if (workspace->supplicant_result != CY_RSLT_ENTERPRISE_SECURITY_SUPPLICANT_NOT_STARTED)
    {
        CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Signal supplicant thread to exit\r\n");
        supplicant_queue_message_packet(workspace,SUPPLICANT_EVENT_ABORT_REQUESTED, NULL);

        CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Wait for supplicant thread to exit\r\n\r\n");
        cy_rtos_delay_milliseconds(10);
    }
    CY_SUPPLICANT_PROCESS_ET_INFO(CYLF_MIDDLEWARE, CY_LOG_INFO, "Supplicant stopped.\r\n\r\n");
    return CY_RSLT_SUCCESS;
}

cy_rslt_t supplicant_enable_tls( supplicant_workspace_t* supplicant, void* context )
{
    supplicant->tls_context = context;
    return CY_RSLT_SUCCESS;
}

cy_rslt_t supplicant_start( supplicant_workspace_t* workspace )
{
    supplicant_rtos_workspace_t* host_workspace = &((supplicant_host_workspace_t*) workspace->supplicant_host_workspace)->host_workspace;
    return cy_rtos_create_thread(&host_workspace->thread, supplicant_thread, "supplicant", host_workspace->thread_stack, SUPPLICANT_THREAD_STACK_SIZE, CY_RTOS_PRIORITY_ABOVENORMAL, (cy_thread_arg_t)workspace);
}

void supplicant_free_tls_session( cy_tls_session_t* session )
{
    /* If session is not NULL, free it from memory. */
    if(session != NULL)
    {
#ifdef COMPONENT_MBEDTLS
        mbedtls_ssl_session_free(session);
#endif
    }
}
