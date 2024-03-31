/*
 * Copyright 2023, Cypress Semiconductor Corporation (an Infineon company) or
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

/**
 *  @file cy_ota_os_timer.h
 *  Function declarations for OTA OS and timer functional Interface APIs for AnyCloud framework.
 */

#ifndef CY_OTA_OS_TIMER_H_
#define CY_OTA_OS_TIMER_H_

/* Standard library include. */
#include <stdint.h>
#include <string.h>

/* OTA library interface include. */
#include "ota_os_interface.h"

/**
 * AWS OTA Event Context structure.
 */

struct OtaEventContext
{
    cy_queue_t ota_event_queue;    /* The queue control handle. */
    OtaTimerCallback_t ota_timer_callback; /* OTA App Timer callback. */
};

/**
 * Initialize the OTA events mechanism for AnyCloud framework.
 * @note AWS OTA library calls this API. API signature is defined in ota_os_interface.h of AWS OTA library.
 *
 * @param ota_event_ctx [in]        : Pointer to the OTA event context. AWS OTA library creates and initializes this OTA event context.
 *
 * @return OtaOsStatus_t            : OtaOsStatus_t, OtaOsSuccess if success, OtaOsEventQueueCreateFailed on failure.
 */
OtaOsStatus_t cy_awsport_ota_event_init( OtaEventContext_t *ota_event_ctx );

/**
 * Sends an OTA event to OTA library event handler on AnyCloud framework.
 * @note AWS OTA library calls this API. API signature is defined in ota_os_interface.h of AWS OTA library.
 *
 * @param ota_event_ctx [in]        : Pointer to the OTA event context. AWS OTA library creates and initializes this OTA event context.
 * @param event_msg [in]            : Event to be sent to the OTA handler.
 * @param timeout [in]              : The maximum amount of time (msec) the task should block.
 *
 * @return OtaOsStatus_t            : OtaOsStatus_t, OtaOsSuccess if success, OtaOsEventQueueSendFailed on failure.
 */
OtaOsStatus_t cy_awsport_ota_event_send( OtaEventContext_t *ota_event_ctx, const void *event_msg, unsigned int timeout );

/**
 * Receives next OTA event from the pending OTA events list on AnyCloud framework.
 * @note AWS OTA library calls this API. API signature is defined in ota_os_interface.h of AWS OTA library.
 *
 * @param ota_event_ctx [in]        : Pointer to the OTA event context. AWS OTA library creates and initializes this OTA event context.
 * @param event_msg [in]            : Pointer to store message.
 * @param timeout [in]              : The maximum amount of time (msec) the task should block.
 *
 * @return OtaOsStatus_t            : OtaOsStatus_t, OtaOsSuccess if success, OtaOsEventQueueReceiveFailed on failure.
 */
OtaOsStatus_t cy_awsport_ota_event_receive( OtaEventContext_t *ota_event_ctx, void *event_msg, uint32_t timeout );

/**
 * Deinitialize the OTA events mechanism and frees any resources used for OTA events.
 * @note AWS OTA library calls this API. API signature is defined in ota_os_interface.h of AWS OTA library.
 *
 * @param ota_event_ctx [in]        : Pointer to the OTA event context. AWS OTA library creates and initializes this OTA event context.
 *
 * @return OtaOsStatus_t            : OtaOsStatus_t, OtaOsSuccess if success, OtaOsEventQueueDeleteFailed on failure.
 */
OtaOsStatus_t cy_awsport_ota_event_deinit( OtaEventContext_t *ota_event_ctx );

/**
 * Create and starts the OTA timers, or resets it if it is already started.
 * @note AWS OTA library calls this API. API signature is defined in ota_os_interface.h of AWS OTA library.
 *
 * @param ota_timer_id [in]         : Timer ID of type otaTimerId_t, defined in ota_os_interface.h of AWS OTA library.
 * @param timer_name [in]           : Timer name.
 * @param timeout [in]              : Timeout for the timer.
 * @param callback [in]             : Callback to be called when timer expires.
 *
 * @return OtaOsStatus_t            : OtaOsStatus_t, OtaOsSuccess if success, OtaOsTimerCreateFailed on failure.
 */
OtaOsStatus_t cy_awsport_ota_timer_create_start( OtaTimerId_t ota_timer_id, const char * const timer_name,
                                                 const uint32_t timeout, OtaTimerCallback_t callback );

/**
 * Stops the OTA timers.
 * @note AWS OTA library calls this API. API signature is defined in ota_os_interface.h of AWS OTA library.
 *
 * @param ota_timer_id [in]         : Timer ID of type otaTimerId_t, defined in ota_os_interface.h of AWS OTA library.
 *
 * @return OtaOsStatus_t            : OtaOsStatus_t, OtaOsSuccess if success, OtaOsTimerStopFailed on failure.
 */
OtaOsStatus_t cy_awsport_ota_timer_stop( OtaTimerId_t ota_timer_id );

/**
 * Delete the OTA timers.
 * @note AWS OTA library calls this API. API signature is defined in ota_os_interface.h of AWS OTA library.
 *
 * @param ota_timer_id [in]         : Timer ID of type otaTimerId_t, defined in ota_os_interface.h of AWS OTA library.
 *
 * @return OtaOsStatus_t            : OtaOsStatus_t, OtaOsSuccess if success, OtaOsTimerDeleteFailed on failure.
 */
OtaOsStatus_t cy_awsport_ota_timer_delete( OtaTimerId_t ota_timer_id );

/**
 * Allocates the requested bytes of memory and returns a pointer to it.
 * @note AWS OTA library calls this API. API signature is defined in ota_os_interface.h of AWS OTA library.
 *
 * @param size [in]                 : Size of the memory block in bytes.
 *
 * @return                          : Returns a pointer to the allocated memory, or NULL if the request fails.
 */
void * cy_awsport_ota_malloc( size_t size );

/**
 * Deallocates the memory which is previously allocated.
 * @note AWS OTA library calls this API. API signature is defined in ota_os_interface.h of AWS OTA library.
 *
 * @param ptr [in]                  : Pointer to the memory block which needs to be deallocated.
 *                                    If a null pointer is passed as an argument, no action occurs.
 *
 * @return                          : None.
 */
void cy_awsport_ota_free( void *ptr );

#endif /* ifndef CY_OTA_OS_TIMER_H_ */
