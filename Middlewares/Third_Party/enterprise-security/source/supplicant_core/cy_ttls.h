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

#ifndef MW_ENTERPRISE_SECURITY_SRC_SUPPLICANT_CORE_CY_TTLS_H_
#define MW_ENTERPRISE_SECURITY_SRC_SUPPLICANT_CORE_CY_TTLS_H_

#include "cy_type_defs.h"
#include "cy_supplicant_core_constants.h"
#include "cy_supplicant_structures.h"
#include "cy_supplicant_host.h"
#include "cy_eap.h"
#include "cy_tls_abstraction.h"
#include "cy_mschapv2.h"
#include "cy_supplicant_process_et.h"

/******************************************************
 *              Packed Structures
 ******************************************************/
#pragma pack(1)

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint32_t avp_code;
    uint8_t flags;
    uint8_t avp_length[3];
} avp_header_t;

typedef struct
{
    uint8_t Version;
    uint8_t reserved;
    uint8_t count;
} leap_header;

typedef struct
{
    avp_header_t avp_header;
    uint8_t data[1];
} avp_packet_t;

#pragma pack()

/******************************************************
 *              Function Prototypes
 ******************************************************/
cy_rslt_t supplicant_process_ttls_phase2_event(supplicant_workspace_t* workspace, supplicant_packet_t packet);
void      supplicant_send_ttls_response_packet( supplicant_packet_t* packet, supplicant_workspace_t* workspace );
supplicant_packet_t supplicant_create_ttls_response_packet( supplicant_packet_t* packet, eap_type_t eap_type, uint16_t data_length, uint8_t length_field_overhead, supplicant_workspace_t* workspace );
cy_rslt_t supplicant_init_ttls_phase2_handshake(supplicant_workspace_t* workspace);

#ifdef __cplusplus
} /*extern "C" */
#endif

#endif /* MW_ENTERPRISE_SECURITY_SRC_SUPPLICANT_CORE_CY_TTLS_H_ */
