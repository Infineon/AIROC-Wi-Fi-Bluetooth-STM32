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

#include "cy_tls_abstraction.h"
#include "cy_supplicant_host.h"

/******************************************************
 *                      Macros
 ******************************************************/
#define SUPPLICANT_MAX_IDENTITY_LENGTH  (32)
#define SUPPLICANT_MAX_PASSWORD_LENGTH  (64)

#define SUPPLICANT_DEFS_MIN(x, y)       ((x) < (y) ? (x) : (y))

/******************************************************
 *                 Type Definitions
 ******************************************************/
typedef void* tls_agent_packet_t;

/******************************************************
 *                Unpacked Structures
 ******************************************************/
typedef uint32_t             tls_packet_t;

typedef struct
{
    supplicant_rtos_workspace_t host_workspace;
    cy_queue_t                  outgoing_packet_queue;
} supplicant_host_workspace_t;

typedef struct
{
    tls_agent_event_t        event_type;
    union
    {
        tls_agent_packet_t   packet;
        uint32_t value;
    } data;
    uint16_t length;
} tls_agent_event_message_t;

typedef struct
{
    void*                    tls_agent_host_workspace;
} tls_agent_workspace_t;

typedef struct supplicant_phase2_state_s
{
    eap_type_t              eap_type;
    cy_rslt_t               result;
    supplicant_main_stage_t main_stage;
    uint8_t                 sub_stage;

    uint8_t                 identity[SUPPLICANT_MAX_IDENTITY_LENGTH];
    uint8_t                 identity_length;

    uint8_t                 password[SUPPLICANT_MAX_PASSWORD_LENGTH];
    uint8_t                 password_length;

    uint8_t                 request_id;
    uint8_t                 leap_ap_challenge[LEAP_CHALLENGE_LEN];  /* Used only in eap-ttls with Cisco-LEAP inner authentication*/
} supplicant_phase2_state_t;

typedef struct supplicant_inner_identity_s
{
    uint8_t                 identity[SUPPLICANT_MAX_IDENTITY_LENGTH];
    uint8_t                 identity_length;
    /* in Windows password is UNICODE */
    uint8_t                 password[SUPPLICANT_MAX_PASSWORD_LENGTH];
    uint8_t                 password_length;
}supplicant_inner_identity_t;

typedef struct supplicant_phase2_workspace_s* supplicant_phase2_workspace_ptr_t;

typedef struct supplicant_connection_info_
{
    whd_interface_t               interface;                /* Network interface type*/

    cy_tls_identity_t            *tls_identity;             /* Identity for the secure connection */
    cy_tls_session_t             *tls_session;              /* Used to store TLS session information */
    cy_tls_context_t             *context;                  /* TLS context required for secure connection */

    uint8_t*                      trusted_ca_certificates;  /* Root-CA certificate */
    uint32_t                      root_ca_cert_length;      /* Root-CA certificate length */

    eap_type_t                    eap_type;                 /* EAP type */
    cy_enterprise_security_auth_t auth_type;                /* Security Auth type */
    uint8_t*                      eap_identity;             /* EAP identity string, 'null' terminated */

    uint8_t*                      user_name;                /* [Used incase of PEAP and EAP_TTLS] User name string, 'null' terminated */
    uint8_t*                      password;                 /* [Used incase of PEAP and EAP_TTLS] Password string, 'null' terminated */

    uint8_t*                      user_cert;                /* [Used incase of EAP-TLS and optionally in EAP_TTLS] Client certificate */
    uint32_t                      user_cert_length;         /* [Used incase of EAP-TLS and optionally in EAP_TTLS] Client certificate length */

    supplicant_tunnel_auth_type_t tunnel_auth_type;         /* tunnel authentication type EAP, CHAP etc..*/
    eap_type_t                    inner_eap_type;           /* [Used incase of EAP_TTLS] inner eap type*/

    uint8_t*                      private_key;              /* [Used incase of EAP-TLS] Client private key */
    uint32_t                      key_length;               /* [Used incase of EAP-TLS] Client private key length */
} supplicant_connection_info_t;

typedef struct
{
    eap_type_t                    eap_type;
    void*                         supplicant_host_workspace;
    whd_interface_t               interface;
    cy_rslt_t                     supplicant_result;

    /* State machine stages */
    supplicant_main_stage_t       current_main_stage;
    uint8_t                       current_sub_stage;
    cy_enterprise_security_auth_t auth_type;

    /* The ID of the last received packet we should use when replying */
    uint8_t                       last_received_id;

    uint8_t                       have_packet;
    cy_time_t                     start_time;
    cy_time_t                     eap_handshake_start_time;
    cy_time_t                     eap_handshake_current_time;

    whd_mac_t                     supplicant_mac_address;
    whd_mac_t                     authenticator_mac_address;
    char                          outer_eap_identity[32];
    uint8_t                       outer_eap_identity_length;

    cy_tls_context_t*             tls_context;
    tls_agent_workspace_t         tls_agent;
    uint8_t                       tls_length_overhead;
    uint8_t*                      buffer;
    uint32_t                      buffer_size;
    uint8_t*                      data_start;
    uint8_t*                      data_end;

    supplicant_tunnel_auth_type_t tunnel_auth_type;
    eap_type_t                    inner_eap_type;
    supplicant_inner_identity_t   inner_identity;
    supplicant_phase2_workspace_ptr_t ptr_phase2;
    uint16_t                      cipher_flags;
} supplicant_workspace_t;

struct supplicant_phase2_workspace_s
{
    supplicant_phase2_state_t state;
    cy_thread_t               thread;
    void*                     thread_stack;
};
typedef struct supplicant_phase2_workspace_s supplicant_phase2_workspace_t;

#ifdef __cplusplus
} /*extern "C" */
#endif
