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

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "cy_type_defs.h"
#include "cy_supplicant_core_constants.h"
#include "cy_supplicant_structures.h"
#include "cy_supplicant_host.h"
#include "cy_eap.h"
#include "cyabs_rtos.h"

/******************************************************
 *                  Constants
 ******************************************************/
#ifdef ENABLE_ENTERPRISE_SECURITY_LOGS
#define SUPPLICANT_THREAD_STACK_SIZE         ( 6*1024 + 3*1024 ) /* Extra 3k for debug logs */
#define TLS_AGENT_THREAD_STACK_SIZE          ( 6*1024 + 3*1024 ) /* Extra 3k for debug logs */
#else
#define SUPPLICANT_THREAD_STACK_SIZE         ( 6*1024 )
#define TLS_AGENT_THREAD_STACK_SIZE          ( 6*1024 )
#endif
#define SUPPLICANT_BUFFER_SIZE               ( 3500 )
#define SUPPLICANT_WORKSPACE_ARRAY_SIZE      ( 3 )
#define WLC_EVENT_MSG_LINK                   ( 0x01 )
#define EAPOL_PACKET_TIMEOUT                 ( 5000 )  /* Milliseconds */
#define SUPPLICANT_HANDSHAKE_ATTEMPT_TIMEOUT ( 30000 ) /* Milliseconds */
#define EAP_HANDSHAKE_TIMEOUT_IN_MSEC        ( 25000 ) /* Milliseconds */

/******************************************************
 *                 Macros
 ******************************************************/

/******************************************************
 *              Function Prototypes
 ******************************************************/

cy_rslt_t supplicant_start( supplicant_workspace_t* workspace );
cy_rslt_t supplicant_init_state(supplicant_workspace_t* workspace, eap_type_t eap_type );
void      supplicant_set_identity        ( supplicant_workspace_t* workspace, const uint8_t* eap_identity, uint32_t eap_identity_length );
void      supplicant_set_inner_identity  ( supplicant_workspace_t* workspace, eap_type_t eap_type, void* inner_identity );
cy_rslt_t supplicant_management_set_event_handler( supplicant_workspace_t* workspace, cy_bool_t enable );
cy_rslt_t supplicant_start( supplicant_workspace_t* workspace );
cy_rslt_t supplicant_stop( supplicant_workspace_t* workspace );
cy_rslt_t supplicant_init(supplicant_workspace_t* workspace, supplicant_connection_info_t *conn_info);
cy_rslt_t supplicant_deinit( supplicant_workspace_t* workspace );
cy_rslt_t supplicant_tls_calculate_overhead( supplicant_workspace_t* workspace, uint16_t available_space, uint16_t* header, uint16_t* footer );
cy_rslt_t supplicant_inner_packet_set_data( whd_driver_t whd_driver,supplicant_packet_t* packet, int32_t size );
cy_rslt_t supplicant_process_peap_event(supplicant_workspace_t* workspace, supplicant_packet_t packet);
cy_rslt_t supplicant_phase2_init( supplicant_workspace_t* workspace, eap_type_t type );
cy_rslt_t supplicant_phase2_start( supplicant_workspace_t* workspace );
cy_rslt_t supplicant_enable_tls( supplicant_workspace_t* supplicant, void* context );
void      supplicant_phase2_thread( cy_thread_arg_t arg );
void      supplicant_free_tls_session( cy_tls_session_t* session );
tls_agent_packet_t* supplicant_receive_eap_tls_packet( void* workspace_in, uint32_t* new_length, uint32_t timeout );
cy_rslt_t supplicant_host_get_tls_data(supplicant_workspace_t*, supplicant_packet_t, uint16_t, uint8_t**, uint16_t*, uint16_t*);
cy_rslt_t supplicant_fragment_and_queue_eap_response( supplicant_workspace_t *workspace );
cy_rslt_t supplicant_outgoing_push( void* workspace, supplicant_event_message_t* message );

#ifdef __cplusplus
} /*extern "C" */
#endif
