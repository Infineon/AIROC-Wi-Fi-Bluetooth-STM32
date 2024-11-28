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
#include "whd.h"
#include "whd_int.h"
#include "whd_wifi_api.h"
#include "whd_buffer_api.h"
#include "whd_wlioctl.h"
#include "whd_types.h"
#include "whd_types_int.h"
#include "whd_wlioctl.h"
#include "cy_enterprise_security.h"

/******************************************************
 *                      Macros & Inlines
 ******************************************************/

#define ALWAYS_INLINE_PRE      //_Pragma( "inline=forced" )
#define ALWAYS_INLINE


#ifndef htobe16   /* This is defined in POSIX platforms */
ALWAYS_INLINE_PRE static inline ALWAYS_INLINE uint16_t htobe16(uint16_t v)
{
    return (uint16_t)(((v&0x00FF) << 8) | ((v&0xFF00)>>8));
}
#endif /* ifndef htobe16 */

#ifndef htobe32   /* This is defined in POSIX platforms */
ALWAYS_INLINE_PRE static inline ALWAYS_INLINE uint32_t htobe32(uint32_t v)
{
    return (uint32_t)(((v&0x000000FF) << 24) | ((v&0x0000FF00) << 8) | ((v&0x00FF0000) >> 8) | ((v&0xFF000000) >> 24));
}
#endif /* ifndef htobe32 */

/******************************************************
 *                    Constants
 ******************************************************/

#define SIZEOF_RANDOM                (64)
#define SIZEOF_SESSION_MASTER        (48)
#define SIZEOF_MPPE_KEYS             (128)
#define PMK_LEN                      (32)
#define PMK_LEN_192BIT               (48)

#define EAP_TLS_FLAG_LENGTH_INCLUDED (0x80)
#define EAP_TLS_FLAG_MORE_FRAGMENTS  (0x40)

#define LEAP_VERSION                 (1)
#define LEAP_CHALLENGE_LEN           (8)
#define LEAP_RESPONSE_LEN            (24)
#define LEAP_KEY_LEN                 (16)

#define AVP_CODE_EAP_MESSAGE         (79)
#define AVP_FLAG_MANDATORY_MASK      (0x40)
#define AVP_FLAG_VENDOR_MASK         (0x80)
#define AVP_LENGTH_SIZE              (3)

#define EAP_MTU_SIZE                 (1020)

#define ETHERNET_ADDRESS_LENGTH      (6)

/**
 * Semaphore wait time constants
 */
#define SUPPLICANT_NEVER_TIMEOUT        (0xFFFFFFFF)
#define SUPPLICANT_WAIT_FOREVER         (0xFFFFFFFF)
#define SUPPLICANT_TIMEOUT              (5000)
#define SUPPLICANT_TIMEOUT_PHASE2_START (2000)
#define SUPPLICANT_NO_WAIT              (0)

/******************************************************
 *                   Enumerations
 ******************************************************/

/**
 * Supplicant Tunnel Eap Types
 */
typedef cy_enterprise_security_tunnel_t supplicant_tunnel_auth_type_t;

/**
 * Supplicant Eap Types
 */
typedef cy_enterprise_security_eap_type_t eap_type_t;

typedef enum
{
    TLS_AGENT_EVENT_EAPOL_PACKET,
    TLS_AGENT_EVENT_ABORT_REQUESTED,
} tls_agent_event_t;

/* High level states
 * INITIALISING ( scan, join )
 * EAP_HANDSHAKE ( go through EAP state machine )
 * WPS_HANDSHAKE ( go through WPS state machine )
 */
typedef enum
{
    SUPPLICANT_INITIALISING,
    SUPPLICANT_INITIALISED,
    SUPPLICANT_IN_EAP_METHOD_HANDSHAKE,
    SUPPLICANT_CLOSING_EAP,
} supplicant_main_stage_t;

typedef enum
{
    SUPPLICANT_EAP_START         = 0,    /* (EAP start ) */
    SUPPLICANT_EAP_IDENTITY      = 1,    /* (EAP identity request, EAP identity response) */
    SUPPLICANT_EAP_NAK           = 2,
    SUPPLICANT_EAP_METHOD        = 3,
} supplicant_state_machine_stage_t;

typedef enum
{
    SUPPLICANT_LEAP_IDENTITY              = SUPPLICANT_EAP_IDENTITY,
    SUPPLICANT_LEAP_RESPOND_CHALLENGE     = 2,
    SUPPLICANT_LEAP_REQUEST_CHALLENGE     = 3,
    SUPPLICANT_LEAP_DONE                  = 4,
} supplicant_leap_state_machine_t;

typedef enum
{
    EAP_CODE_REQUEST  = 1,
    EAP_CODE_RESPONSE = 2,
    EAP_CODE_SUCCESS  = 3,
    EAP_CODE_FAILURE  = 4
} eap_code_t;

/**
 * EAPOL types
 */
typedef enum
{
    EAP_PACKET                   = 0,
    EAPOL_START                  = 1,
    EAPOL_LOGOFF                 = 2,
    EAPOL_KEY                    = 3,
    EAPOL_ENCAPSULATED_ASF_ALERT = 4
} eapol_packet_type_t;

/*
 * MSCHAPV2 codes
 */
typedef enum
{
    MSCHAPV2_OPCODE_CHALLENGE       = 1,
    MSCHAPV2_OPCODE_RESPONSE        = 2,
    MSCHAPV2_OPCODE_SUCCESS         = 3,
    MSCHAPV2_OPCODE_FAILURE         = 4,
    MSCHAPV2_OPCODE_CHANGE_PASSWORD = 7,
} mschapv2_opcode_t;

#ifdef __cplusplus
} /*extern "C" */
#endif
