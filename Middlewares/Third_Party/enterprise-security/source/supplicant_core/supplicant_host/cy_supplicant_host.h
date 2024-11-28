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

#include "whd.h"

/*TODO : Move to Abstraction */
#ifdef COMPONENT_LWIP
#include "lwip/pbuf.h"
#endif

#ifdef COMPONENT_NETXDUO
#include <nx_api.h>
#endif
/******************************************************
 *                      Macros
 ******************************************************/
#define SUPPLICANT_HOST_IS_ALIGNED

#ifdef SUPPLICANT_HOST_IS_ALIGNED

#define SUPPLICANT_READ_16(ptr)              ((uint16_t)(((uint8_t*)ptr)[0] + (((uint8_t*)ptr)[1] << 8)))
#define SUPPLICANT_READ_16_BE(ptr)           ((uint16_t)(((uint8_t*)ptr)[1] + (((uint8_t*)ptr)[0] << 8)))
#define SUPPLICANT_READ_32(ptr)              ((uint32_t)(((uint8_t*)ptr)[0] + ((((uint8_t*)ptr)[1] << 8)) + (((uint8_t*)ptr)[2] << 16) + (((uint8_t*)ptr)[3] << 24)))
#define SUPPLICANT_READ_32_BE(ptr)           ((uint32_t)(((uint8_t*)ptr)[3] + ((((uint8_t*)ptr)[2] << 8)) + (((uint8_t*)ptr)[1] << 16) + (((uint8_t*)ptr)[0] << 24)))
#define SUPPLICANT_WRITE_16(ptr, value)      do { ((uint8_t*)ptr)[0] = (uint8_t)value; ((uint8_t*)ptr)[1]=(uint8_t)(value>>8); } while(0)
#define SUPPLICANT_WRITE_16_BE(ptr, value)   do { ((uint8_t*)ptr)[1] = (uint8_t)value; ((uint8_t*)ptr)[0]=(uint8_t)(value>>8); } while(0)
#define SUPPLICANT_WRITE_32(ptr, value)      do { ((uint8_t*)ptr)[0] = (uint8_t)value; ((uint8_t*)ptr)[1]=(uint8_t)(value>>8); ((uint8_t*)ptr)[2]=(uint8_t)(value>>16); ((uint8_t*)ptr)[3]=(uint8_t)(value>>24); } while(0)
#define SUPPLICANT_WRITE_32_BE(ptr, value)   do { ((uint8_t*)ptr)[3] = (uint8_t)value; ((uint8_t*)ptr)[2]=(uint8_t)(value>>8); ((uint8_t*)ptr)[1]=(uint8_t)(value>>16); ((uint8_t*)ptr)[0]=(uint8_t)(value>>24); } while(0)

#else

#define SUPPLICANT_READ_16(ptr)            ((uint16_t*)ptr)[0]
#define SUPPLICANT_READ_32(ptr)            ((uint32_t*)ptr)[0]
#define SUPPLICANT_WRITE_16(ptr, value)    ((uint16_t*)ptr)[0] = value
#define SUPPLICANT_WRITE_16_BE(ptr, value) ((uint16_t*)ptr)[0] = htobe16(value)
#define SUPPLICANT_WRITE_32(ptr, value)    ((uint32_t*)ptr)[0] = value
#define SUPPLICANT_WRITE_32_BE(ptr, value) ((uint32_t*)ptr)[0] = htobe32(value)

/* Prevents errors about strict aliasing */
static inline ALWAYS_INLINE uint16_t SUPPLICANT_READ_16_BE(uint8_t* ptr_in)
{
    uint16_t* ptr = (uint16_t*)ptr_in;
    uint16_t  v   = *ptr;
    return (uint16_t)(((v&0x00FF) << 8) | ((v&0xFF00)>>8));
}

static inline ALWAYS_INLINE uint32_t SUPPLICANT_READ_32_BE(uint8_t* ptr_in)
{
    uint32_t* ptr = (uint32_t*)ptr_in;
    uint32_t  v   = *ptr;
    return (uint32_t)(((v&0x000000FF) << 24) | ((v&0x0000FF00) << 8) | ((v&0x00FF0000) >> 8) | ((v&0xFF000000) >> 24));
}

#endif

/******************************************************
 *                    Constants
 ******************************************************/
#define SUPPLICANT_NEVER_TIMEOUT    (0xFFFFFFFF)

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    SUPPLICANT_EVENT_NO_EVENT,
    SUPPLICANT_EVENT_TIMER_TIMEOUT,
    SUPPLICANT_EVENT_ABORT_REQUESTED,
    SUPPLICANT_EVENT_EAPOL_PACKET_RECEIVED,
    SUPPLICANT_EVENT_RECEIVED_IDENTITY_REQUEST,
    SUPPLICANT_EVENT_COMPLETE,
    SUPPLICANT_EVENT_PACKET_TO_SEND,
} supplicant_event_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef void* supplicant_packet_t;


/*TODO : Move to Abstraction */
#ifdef COMPONENT_LWIP
typedef struct pbuf supplicant_buffer_t;
#endif

#ifdef COMPONENT_NETXDUO
typedef NX_PACKET supplicant_buffer_t;
#endif

typedef void (*cy_ent_sec_eapol_packet_handler_t)(whd_interface_t interface, whd_buffer_t buffer);
/******************************************************
 *                  Typedef Structures
 ******************************************************/

typedef union
{
    supplicant_packet_t packet;
    uint32_t value;
} supplicant_event_message_data_t;

typedef struct
{
    supplicant_event_t event_type;
    supplicant_event_message_data_t data;
    uint16_t length;
} supplicant_event_message_t;

typedef struct
{
    cy_thread_t thread;
    void* thread_stack;
    cy_queue_t event_queue;
    cy_time_t timer_reference;
    cy_time_t timer_timeout;
    whd_interface_t interface;
} supplicant_rtos_workspace_t;


/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

/* Endian management functions */
uint32_t supplicant_host_hton32     (uint32_t intlong);
uint16_t supplicant_host_hton16     (uint16_t intshort);
uint32_t supplicant_host_hton32_ptr (uint8_t* in, uint8_t* out);
uint16_t supplicant_host_hton16_ptr (uint8_t* in, uint8_t* out);

void* supplicant_host_malloc        ( const char* name, uint32_t size );
void* supplicant_host_calloc        ( const char* name, uint32_t num, uint32_t size );
void  supplicant_host_free          ( void* p );

/* Packet functions */
cy_rslt_t supplicant_host_create_packet   ( whd_driver_t whd_driver,supplicant_packet_t* packet, uint16_t size );
void      supplicant_host_consume_bytes   ( whd_driver_t whd_driver,supplicant_packet_t* packet, int32_t number_of_bytes);
uint8_t*  supplicant_host_get_data        ( whd_driver_t whd_driver, supplicant_packet_t packet );
cy_rslt_t supplicant_host_set_packet_size ( whd_driver_t whd_driver, supplicant_packet_t packet, uint16_t packet_length );
uint16_t  supplicant_host_get_packet_size ( whd_driver_t whd_driver,supplicant_packet_t packet );
void      supplicant_host_free_packet     ( whd_driver_t whd_driver, supplicant_packet_t packet );
void      supplicant_host_send_packet     ( void* workspace, supplicant_packet_t packet, uint16_t size );
cy_rslt_t supplicant_host_leave           ( whd_interface_t interface );
void      supplicant_host_start_timer     ( void* workspace, uint32_t timeout );
void      supplicant_host_stop_timer      ( void* workspace );
uint32_t  supplicant_host_get_timer       ( void* workspace );
cy_rslt_t supplicant_queue_message_packet ( void* workspace, supplicant_event_t type, supplicant_packet_t packet );
cy_rslt_t supplicant_queue_message_uint   ( void* workspace, supplicant_event_t type, uint32_t value );
void      supplicant_get_bssid            ( whd_interface_t interface, whd_mac_t* mac );
cy_rslt_t supplicant_set_passphrase       ( whd_interface_t interface, const uint8_t* security_key, uint8_t key_length );
cy_rslt_t supplicant_set_pmk              ( whd_interface_t interface, const uint8_t* security_key, uint8_t key_length );
cy_rslt_t suppliant_emac_register_eapol_packet_handler(cy_ent_sec_eapol_packet_handler_t eapol_packet_handler);
void      supplicant_host_hex_bytes_to_chars( char* cptr, const uint8_t* bptr, uint32_t blen );
cy_rslt_t supplicant_queue_message        ( void* host_workspace, supplicant_event_t type, supplicant_event_message_data_t data );
cy_rslt_t cy_ent_sec_register_eapol_packet_handler(cy_ent_sec_eapol_packet_handler_t eapol_packet_handler);
void      cy_ent_sec_unregister_eapol_packet_handler(void);

#ifdef __cplusplus
} /*extern "C" */
#endif
