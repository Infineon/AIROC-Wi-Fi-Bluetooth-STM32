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
 * @file cy_aws_ota_os_timer.c
 *  Implements OS and timer functional interface APIs for AWS OTA.
 *
 */

#include <string.h>
#include <stdlib.h>
#include "cyabs_rtos.h"
#include "cy_ota_os_timer.h"
#include "cy_aws_iot_sdk_port_log.h"

#include "ota.h"
#include "ota_private.h"

#ifndef OTA_NUM_MSG_Q_ENTRIES
#define OTA_NUM_MSG_Q_ENTRIES            ( 20 )
#endif

/* OTA event context */
OtaEventContext_t aws_ota_event_context;

struct local_timer
{
    bool created;
    cy_timer_t timer;
};

/* OTA Timer handles.*/
static struct local_timer aws_ota_timer[ OtaNumOfTimers ];

/* OTA Timer callback's. */
static void awsport_ota_request_timer_callback( void *arg );
static void awsport_ota_selftest_timer_callback( void *arg );

void ( * timer_callback[ OtaNumOfTimers ] )( void *arg ) = { awsport_ota_request_timer_callback,
                                                             awsport_ota_selftest_timer_callback };

/*-----------------------------------------------------------*/
static void awsport_ota_selftest_timer_callback( void *arg )
{
    ( void ) arg;
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Self-test expired within %u ms\n\r", otaconfigSELF_TEST_RESPONSE_WAIT_MS );
    if( aws_ota_event_context.ota_timer_callback != NULL )
    {
        aws_ota_event_context.ota_timer_callback( OtaSelfTestTimer );
    }
    else
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_WARNING, "Self-test timer event not handled.\n\r" );
    }
}

/*-----------------------------------------------------------*/
static void awsport_ota_request_timer_callback( void *arg )
{
    ( void ) arg;

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Request timer expired in %u ms \n\r", otaconfigFILE_REQUEST_WAIT_MS );

    if( aws_ota_event_context.ota_timer_callback != NULL )
    {
        aws_ota_event_context.ota_timer_callback( OtaRequestTimer );
    }
    else
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_WARNING, "Request timer event not handled.\n\r" );
    }
}

/*-----------------------------------------------------------*/
OtaOsStatus_t cy_awsport_ota_event_init( OtaEventContext_t *ota_event_ctx )
{
    OtaOsStatus_t aws_ota_os_status = OtaOsSuccess;
    cy_rslt_t result = CY_RSLT_SUCCESS;

    ( void ) ota_event_ctx;

    result = cy_rtos_init_queue( &(aws_ota_event_context.ota_event_queue), OTA_NUM_MSG_Q_ENTRIES, sizeof( OtaEventMsg_t ) );
    if( result != CY_RSLT_SUCCESS )
    {
        aws_ota_os_status = OtaOsEventQueueCreateFailed;
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "cy_rtos_init_queue failed with Error : [0x%X] \n\r", (unsigned int)result );
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Failed to create OTA Event Queue.\n\r" );
    }
    else
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_INFO, "OTA Event Queue created.\n\r" );
    }

    return aws_ota_os_status;
}

/*-----------------------------------------------------------*/
OtaOsStatus_t cy_awsport_ota_event_send( OtaEventContext_t *ota_event_ctx, const void *event_msg, unsigned int timeout )
{
    OtaOsStatus_t aws_ota_os_status = OtaOsSuccess;
    cy_rslt_t result = CY_RSLT_SUCCESS;

    ( void ) ota_event_ctx;
    ( void ) timeout;

    if( event_msg == NULL )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Invalid argument to cy_awsport_ota_event_send. \n\r" );
        return OtaOsEventQueueSendFailed;
    }

    /* Send the event to OTA event queue.*/
    result = cy_rtos_put_queue( &(aws_ota_event_context.ota_event_queue), (void *)event_msg, timeout, false );
    if( result != CY_RSLT_SUCCESS )
    {
        aws_ota_os_status = OtaOsEventQueueSendFailed;
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "cy_rtos_put_queue failed with Error : [0x%X] \n\r", (unsigned int)result );
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Failed to send event to OTA Event Queue.\n\r" );
    }
    else
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "OTA Event Sent.\n\r" );
    }

    return aws_ota_os_status;
}

/*-----------------------------------------------------------*/
OtaOsStatus_t cy_awsport_ota_event_receive( OtaEventContext_t *ota_event_ctx, void *event_msg, uint32_t timeout )
{
    OtaOsStatus_t aws_ota_os_status = OtaOsSuccess;
    cy_rslt_t result = CY_RSLT_SUCCESS;
    uint8_t buff[sizeof( OtaEventMsg_t )];

    ( void ) ota_event_ctx;
    ( void ) timeout;

    if( event_msg == NULL )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Invalid argument to cy_awsport_ota_event_receive. \n\r" );
        return OtaOsEventQueueReceiveFailed;
    }

    result = cy_rtos_get_queue( &(aws_ota_event_context.ota_event_queue), (void *)&buff, CY_RTOS_NEVER_TIMEOUT, false );
    if( result != CY_RSLT_SUCCESS )
    {
        aws_ota_os_status = OtaOsEventQueueReceiveFailed;
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "cy_rtos_get_queue failed with Error : [0x%X] \n\r", (unsigned int)result );
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Failed to receive event from OTA Event Queue.\n\r" );
    }
    else
    {
        /* copy the data from local buffer.*/
        memcpy( event_msg, buff, sizeof( OtaEventMsg_t ) );
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "OTA Event received.\n\r" );
    }

    return aws_ota_os_status;
}

/*-----------------------------------------------------------*/
OtaOsStatus_t cy_awsport_ota_event_deinit( OtaEventContext_t *ota_event_ctx )
{
    OtaOsStatus_t aws_ota_os_status = OtaOsSuccess;
    cy_rslt_t result = CY_RSLT_SUCCESS;
    ( void ) ota_event_ctx;

    /* Deinit the event queue.*/
    result = cy_rtos_deinit_queue( &(aws_ota_event_context.ota_event_queue) );
    if( result != CY_RSLT_SUCCESS )
    {
        aws_ota_os_status = OtaOsEventQueueDeleteFailed;
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "cy_rtos_get_queue failed with Error : [0x%X] \n\r", (unsigned int)result );
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Failed to delete OTA Event Queue.\n\r" );
    }
    else
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_WARNING, " OTA Event Queue was not initialized. \n\r" );
    }
    return aws_ota_os_status;
}

/*-----------------------------------------------------------*/
OtaOsStatus_t cy_awsport_ota_timer_create_start( OtaTimerId_t ota_timer_id, const char * const timer_name,
                                                 const uint32_t timeout, OtaTimerCallback_t callback )
{
    OtaOsStatus_t aws_ota_timer_status = OtaOsSuccess;
    cy_rslt_t result = CY_RSLT_SUCCESS;

    if( (callback == NULL) || (ota_timer_id >= OtaNumOfTimers) )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Invalid parameter to OTA Timer start. \n\r" );
        return OtaOsTimerCreateFailed;
    }

    /* Set OTA library callback. */
    aws_ota_event_context.ota_timer_callback = callback;

    /* If timer is not created.*/
    if( aws_ota_timer[ ota_timer_id ].created == false )
    {
        /* Create the timer. */
        result = cy_rtos_init_timer( &(aws_ota_timer[ ota_timer_id ].timer), CY_TIMER_TYPE_ONCE,
                                     ( cy_timer_callback_t )timer_callback[ ota_timer_id ],
                                     ( cy_timer_callback_arg_t )NULL );
        if( result != CY_RSLT_SUCCESS )
        {
            aws_ota_timer_status = OtaOsTimerCreateFailed;
            cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "cy_rtos_init_timer failed with Error : [0x%X] \n\r", (unsigned int)result );
            cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Failed to create OTA timer.\n\r" );
        }
        else
        {
            aws_ota_timer[ ota_timer_id ].created = true;
            cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_INFO, "OTA Timer created.\n\r" );
            /* Start the timer. */
            result = cy_rtos_start_timer( &(aws_ota_timer[ ota_timer_id ].timer), timeout );
            if( result != CY_RSLT_SUCCESS )
            {
                aws_ota_timer_status = OtaOsTimerStartFailed;
                cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "cy_rtos_start_timer failed with Error : [0x%X] \n\r", (unsigned int)result );
                cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Failed to start OTA timer.\n\r" );
            }
            else
            {
                cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_INFO, "OTA Timer started.\n\r" );
            }
        }
    }
    else
    {
        /* Reset the timer. */
        result = cy_rtos_start_timer( &(aws_ota_timer[ ota_timer_id ].timer), timeout );
        if( result != CY_RSLT_SUCCESS )
        {
            aws_ota_timer_status = OtaOsTimerRestartFailed;
            cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "cy_rtos_start_timer failed with Error : [0x%X] \n\r", (unsigned int)result );
            cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Failed to set OTA timer timeout.\n\r" );
        }
        else
        {
            cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "OTA Timer restarted." );
        }
    }
    return aws_ota_timer_status;
}

/*-----------------------------------------------------------*/
OtaOsStatus_t cy_awsport_ota_timer_stop( OtaTimerId_t ota_timer_id )
{
    OtaOsStatus_t aws_ota_timer_status = OtaOsSuccess;
    cy_rslt_t result = CY_RSLT_SUCCESS;

    if( ota_timer_id >= OtaNumOfTimers )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Invalid parameter to OTA Timer Stop. \n\r" );
        return OtaOsTimerStopFailed;
    }

    if( aws_ota_timer[ ota_timer_id ].created )
    {
        /* Stop the timer. */
        result = cy_rtos_stop_timer( &(aws_ota_timer[ ota_timer_id ].timer) );
        if( result != CY_RSLT_SUCCESS )
        {
            aws_ota_timer_status = OtaOsTimerStopFailed;
            cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "cy_rtos_stop_timer failed with Error : [0x%X] \n\r", (unsigned int)result );
            cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Failed to stop OTA timer.\n\r" );
        }
        else
        {
            cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_INFO, "OTA Timer Stopped for Timerid = %i. \n\r", ota_timer_id );
        }
    }
    else
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_WARNING, "OTA Timer handle NULL for Timerid = %i, can't stop. \n\r", ota_timer_id );
        aws_ota_timer_status = OtaOsTimerStopFailed;
    }

    return aws_ota_timer_status;
}

/*-----------------------------------------------------------*/
OtaOsStatus_t cy_awsport_ota_timer_delete( OtaTimerId_t ota_timer_id )
{
    OtaOsStatus_t aws_ota_timer_status = OtaOsSuccess;
    cy_rslt_t result = CY_RSLT_SUCCESS;

    if( ota_timer_id >= OtaNumOfTimers )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Invalid parameter to OTA Timer delete. \n\r" );
        return OtaOsTimerDeleteFailed;
    }

    if( aws_ota_timer[ ota_timer_id ].created )
    {
        /* Delete the timer. */
        result = cy_rtos_deinit_timer( &(aws_ota_timer[ ota_timer_id ].timer) );
        if( result != CY_RSLT_SUCCESS )
        {
            aws_ota_timer_status = OtaOsTimerDeleteFailed;
            cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "cy_rtos_deinit_timer failed with Error : [0x%X] \n\r", (unsigned int)result );
            cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Failed to delete OTA timer.\n\r" );
        }
        else
        {
            aws_ota_timer[ ota_timer_id ].created = false;
            cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_INFO, "OTA Timer deleted. \n\r" );
        }
    }
    else
    {
        aws_ota_timer_status = OtaOsTimerDeleteFailed;
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_WARNING, "OTA Timer handle NULL for Timerid = %i, can't delete. \n\r", ota_timer_id );
    }

    return aws_ota_timer_status;
}

/*-----------------------------------------------------------*/
void * cy_awsport_ota_malloc( size_t size )
{
    void *pvReturn = NULL;
    if( size <= 0 )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "cy_awsport_ota_malloc called with size = %u . \n\r",  size );
        goto exit;
    }
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "cy_awsport_ota_malloc called with size = %u . \n\r",  size );
    pvReturn = malloc( size );
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Allocated memory = %p \n\r", pvReturn );

exit :
    if( pvReturn == NULL )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Malloc failed..!!!\n\r" );
    }
    return pvReturn;
}

/*-----------------------------------------------------------*/
void cy_awsport_ota_free( void *ptr )
{
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_INFO, "cy_awsport_ota_free called with memory address = %p . \n\r",  ptr );
    if( ptr!= NULL )
    {
        free( ptr );
        ptr = NULL;
    }
}

/*-----------------------------------------------------------*/
