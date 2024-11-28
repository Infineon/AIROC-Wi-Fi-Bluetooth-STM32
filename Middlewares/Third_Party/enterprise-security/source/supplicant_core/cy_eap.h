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

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                   Typedef structures
 ******************************************************/
#pragma pack(1)

typedef struct
{
    uint8_t ether_dhost[ETHERNET_ADDRESS_LENGTH];
    uint8_t ether_shost[ETHERNET_ADDRESS_LENGTH];
    uint16_t ether_type;
} cy_ether_header_t;

typedef struct
{
    uint8_t version;
    uint8_t type;
    uint16_t length;
} eapol_header_t;

typedef struct
{
    cy_ether_header_t ethernet;
    eapol_header_t eapol;
} eapol_packet_header_t;

typedef struct
{
    cy_ether_header_t ethernet;
    eapol_header_t eapol;
    uint8_t data[1];
} eapol_packet_t;

typedef struct
{
    uint8_t code;
    uint8_t id;
    uint16_t length;
    uint8_t type;
} eap_header_t;

typedef struct
{
    cy_ether_header_t ethernet;
    eapol_header_t eapol;
    eap_header_t eap;
    uint8_t data[1];
} eap_packet_t;

typedef struct
{
    uint8_t flags;
} eap_tls_header_t;

typedef struct
{
    cy_ether_header_t ethernet;
    eapol_header_t eapol;
    eap_header_t eap;
    eap_tls_header_t eap_tls;
    uint8_t data[1]; /* Data starts with a length of TLS data field or TLS data depending on the flags field */
} eap_tls_packet_t;

typedef struct
{
    uint16_t type;
    uint16_t length;
    uint8_t value[1];
} avp_request_t;

typedef struct
{
    uint16_t type;
    uint16_t length;
    uint16_t status;
} avp_result_t;

typedef struct
{
    uint8_t type;
    uint8_t major_version;
    uint8_t minor_version;
    uint16_t length;
    uint8_t message[1];
} tls_record_t;

/* Helper structure to create TLS record */
typedef struct
{
    uint8_t type;
    uint8_t major_version;
    uint8_t minor_version;
    uint16_t length;
} tls_record_header_t;

#pragma pack()

/******************************************************
 *               Function Prototypes
 ******************************************************/
cy_rslt_t supplicant_send_eapol_start               ( supplicant_workspace_t* workspace );
void      supplicant_send_eap_response_packet       ( supplicant_workspace_t* workspace, eap_type_t eap_type, uint8_t* data, uint16_t data_length );
cy_rslt_t supplicant_send_zero_length_eap_tls_packet( supplicant_workspace_t* workspace );
cy_rslt_t supplicant_send_eap_tls_fragment          ( supplicant_workspace_t* workspace, supplicant_packet_t packet );
void      supplicant_send_eapol_packet              ( supplicant_packet_t packet, supplicant_workspace_t* workspace, eapol_packet_type_t type, uint16_t content_size );

#ifdef __cplusplus
} /*extern "C" */
#endif
