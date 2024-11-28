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
#include "cy_supplicant_structures.h"
#include "cy_peap.h"
#include "cy_ttls.h"

/******************************************************
 *                  Typedef structures
 ******************************************************/
#pragma pack(1)
typedef struct
{
    uint8_t     opcode;
    uint8_t     id;
    uint16_t    length;
}mschapv2_header_t;

typedef struct
{
    uint8_t     opcode;
    uint8_t     id;
    uint16_t    length;
    uint8_t     data[1];
}mschapv2_packet_t;

typedef struct
{
    uint8_t     opcode;
    uint8_t     id;
    uint16_t    length;
    uint8_t     value_size;
    uint8_t     challenge[16];
    uint8_t     name[1];
}mschapv2_challenge_packet_t;

typedef struct
{
    uint8_t     opcode;
    uint8_t     id;
    uint16_t    length;
    uint8_t     value_size;
    uint8_t     peer_challenge[16];
    uint8_t     reserved[8];
    uint8_t     nt_reponse[24];
    uint8_t     flags;
    uint8_t     name[1];
}mschapv2_response_packet_t;

typedef struct
{
    uint8_t     opcode;
    uint8_t     id;
    uint16_t    length;
    uint8_t     message[1];
}mschapv2_success_request_packet_t;

typedef struct
{
    uint8_t     opcode;
}mschapv2_success_response_packet_t;

typedef mschapv2_success_request_packet_t mschapv2_failure_request_packet_t;

typedef mschapv2_success_response_packet_t mschapv2_failure_response_packet_t;

#pragma pack()

/******************************************************
 *              Function Prototypes
 ******************************************************/
cy_rslt_t mschap_challenge_hash           ( uint8_t* peer_challenge, uint8_t* authenticator_challenge, char* user_name, uint8_t* challenge);
cy_rslt_t mschap_nt_password_hash         ( char* password, uint16_t length, uint8_t* password_hash );
cy_rslt_t mschap_permute_key              ( uint8_t* key56, uint8_t* key64);
cy_rslt_t mschap_des_encrypt              ( uint8_t* clear, uint8_t* key, uint8_t* cypher);
cy_rslt_t mschap_challenge_response       ( uint8_t* challenge, uint8_t* nt_password_hash, uint8_t* nt_response );
cy_rslt_t mschap_generate_nt_response     ( uint8_t* authenticator_challenge, uint8_t* peer_challenge,char* user_name, char* password, uint16_t password_length, uint8_t* nt_response);
cy_rslt_t mschap_process_packet           ( mschapv2_packet_t *packet, supplicant_workspace_t *workspace );

#ifdef __cplusplus
} /*extern "C" */
#endif
