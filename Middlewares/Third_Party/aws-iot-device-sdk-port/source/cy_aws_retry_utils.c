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
 * @file cy_aws_retry_utils.c
 * Utility implementation of backoff logic, used for attempting retries of failed processes.
 */

/* Standard includes. */
#include <stdlib.h>
#include <time.h>
#include "cy_result.h"
#include "cyhal.h"
#include "cy_retry_utils.h"
#include "cy_aws_iot_sdk_port_log.h"
#include "clock.h"

#ifdef CY_TFM_PSA_SUPPORTED
#include "psa/crypto.h"
#endif

#ifdef COMPONENT_4390X
extern cy_rslt_t cy_prng_get_random( void* buffer, uint32_t buffer_length );
#endif

#ifdef COMPONENT_CAT5
extern uint32_t thread_ap_rbg_rand(void);
#endif

/*-----------------------------------------------------------*/
#ifndef CY_TFM_PSA_SUPPORTED
#ifndef COMPONENT_4390X
#ifndef COMPONENT_CAT5
static int trng_get_bytes( cyhal_trng_t *obj, uint8_t *output, size_t length, size_t *output_length )
{
    uint32_t offset = 0;
    /* If output is not word-aligned, write partial word */
    uint32_t prealign = (uint32_t)( (uintptr_t)output % sizeof(uint32_t) );
    if( prealign != 0 )
    {
        uint32_t value = cyhal_trng_generate( obj );
        uint32_t count = sizeof(uint32_t) - prealign;
        memmove(&output[0], &value, count);
        offset += count;
    }
    /* Write aligned full words */
    for( ; offset < length - (sizeof(uint32_t) - 1u); offset += sizeof(uint32_t) )
    {
        *(uint32_t *)(&output[offset]) = cyhal_trng_generate( obj );
    }
    /* Write partial trailing word if requested */
    if( offset < length )
    {
        uint32_t value = cyhal_trng_generate( obj );
        uint32_t count = length - offset;
        memmove( &output[offset], &value, count );
        offset += count;
    }
    *output_length = offset;
    return 0;
}
#endif
#endif
#endif
int generate_random_number( void *buffer, size_t buffer_length, size_t *output_length )
{
#ifdef CY_TFM_PSA_SUPPORTED
    psa_status_t status = psa_crypto_init();
    if( status != PSA_SUCCESS )
    {
        return -1;
    }

    status = psa_generate_random( buffer, buffer_length );
    if( status != PSA_SUCCESS )
    {
        return -1;
    }

    *output_length = buffer_length;
#elif defined(COMPONENT_4390X)
    /* 4390X kits does not have TRNG module. Get the random
     * number from wifi-mw-core internal PRNG API. */
    cy_rslt_t result;
    result = cy_prng_get_random(buffer, buffer_length);
    if(result != CY_RSLT_SUCCESS)
    {
        return -1;
    }
    *output_length = buffer_length;
#elif defined(COMPONENT_CAT5)
    *((uint32_t *)buffer) = thread_ap_rbg_rand();
    *output_length = 4;
#else
    uint8_t *p = buffer;
    size_t length = 0;

    cyhal_trng_t obj;
    cy_rslt_t result;

    result = cyhal_trng_init( &obj );
    if( result != CY_RSLT_SUCCESS )
    {
        return -1;
    }

    (void)trng_get_bytes( &obj, p, buffer_length, (size_t*) &length );

    *output_length = length;
    cyhal_trng_free( &obj );
#endif
    return 0;
}

static uint32_t cy_rand( void )
{
    int ret = 0;
    uint16_t  r[2];
    size_t    output_length = 0;
    uint32_t  random_num = 0;

    /* Generate a random number between 1 and 9999999 */
    while ( random_num == 0 )
    {
        ret = generate_random_number( (void *)r,  4, &output_length );
        if( ret != 0 )
        {
            break;
        }
        random_num = (uint32_t)(r[0] * r[1]) % 9999999;
    }
    return random_num;
}

RetryUtilsStatus_t RetryUtils_BackoffAndSleep( RetryUtilsParams_t *pRetryParams )
{
    RetryUtilsStatus_t status = RetryUtilsRetriesExhausted;
    uint32_t backoff_delay = 0;

    if( pRetryParams == NULL )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "\nInvalid pRetryParams for RetryUtils_BackoffAndSleep..!\n" );
        return status;
    }

    /* If CY_MQTT_MAX_RETRY_ATTEMPT is set to 0, try forever. */
    if( ( pRetryParams->attemptsDone < CY_BACKOFF_MAX_RETRY_ATTEMPTS ) ||
        ( 0 == CY_BACKOFF_MAX_RETRY_ATTEMPTS ) )
    {
        /* Choose a random value for back-off time between 0 and the max jitter value. */
        backoff_delay = (uint32_t)(cy_rand() % pRetryParams->nextJitterMax);

        /*  Wait for the backoff time to expire before the next retry. */
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_INFO, "Wait for backoff time %ul for the next retry!\n", backoff_delay );
        Clock_SleepMs( backoff_delay );

        /* Increment backoff counts. */
        pRetryParams->attemptsDone++;

        /* Double the max jitter value for the next retry attempt, only
         * if the new value will be less than the max backoff time value. */
        if( pRetryParams->nextJitterMax < ( MAX_RETRY_BACKOFF_SECONDS / 2U ) )
        {
            pRetryParams->nextJitterMax += pRetryParams->nextJitterMax;
        }
        else
        {
            pRetryParams->nextJitterMax = MAX_RETRY_BACKOFF_SECONDS;
        }

        status = RetryUtilsSuccess;
    }
    else
    {
        /* When max retry attempts are exhausted, let the application know by
         * returning RetryUtilsRetriesExhausted. The application may choose to
         * restart the retry process after calling RetryUtils_ParamsReset(). */
        status = RetryUtilsRetriesExhausted;
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_INFO, "Max retry attempts are exhausted, calling RetryUtils_ParamsReset()!\n" );
        RetryUtils_ParamsReset( pRetryParams );
    }

    return status;
}

/*-----------------------------------------------------------*/
void RetryUtils_ParamsReset( RetryUtilsParams_t *pRetryParams )
{
    uint32_t jitter = 0;
    uint32_t time_ms = 0;

    /* Reset the attempts done to zero so that the next retry cycle can start. */
    pRetryParams->attemptsDone = 0;

    /* Get the current time to seed the pseudo random number generator. */
    time_ms = Clock_GetTimeMs();

    /* Seed the pseudo random number generator with miliseconds. */
    srand( time_ms );

    /* Calculate jitter value using a random number. */
    jitter = (uint32_t)( cy_rand() % MAX_JITTER_VALUE_SECONDS );
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_INFO, "Calculated jitter value %lu\n", jitter );

    /* Reset the backoff value to the initial timeout value plus jitter. */
    pRetryParams->nextJitterMax = INITIAL_RETRY_BACKOFF_SECONDS + jitter;
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_INFO, "Calculated next jitter max value %lu\n", pRetryParams->nextJitterMax );
}

/*-----------------------------------------------------------*/
void cy_aws_initialize_backoff_params(  BackoffAlgorithmParams_t * pContext,
                                        uint16_t backOffBase,
                                        uint16_t maxBackOff,
                                        uint32_t maxAttempts )
{
    if(pContext != NULL)
    {
        BackoffAlgorithm_InitializeParams( &( pContext->context ), backOffBase, maxBackOff, maxAttempts );
        pContext->nextAllowedTickCount = 0U;
    }
}

/*-----------------------------------------------------------*/
cy_rslt_t cy_aws_get_backoff_status( BackoffAlgorithmParams_t * pContext,
                                     bool *inBackOff )
{
    cy_rslt_t result = CY_RSLT_TYPE_ERROR;

    if( pContext != NULL && inBackOff != NULL )
    {
        uint32_t currentTickCount = Clock_GetTimeMs();

        result = CY_RSLT_SUCCESS;

        *inBackOff = false;

         /* Handle time roll over case */
        if( ( currentTickCount < pContext->nextAllowedTickCount ) &&
            ( ( pContext->nextAllowedTickCount - currentTickCount ) > pContext->context.maxBackoffDelay ) )
        {
            pContext->nextAllowedTickCount = currentTickCount;
        }

        /* Time has not elapsed. We are in Backoff */
        if( pContext->nextAllowedTickCount > currentTickCount )
        {
            *inBackOff = true;
        }
    }
    return ( result );
}

/*-----------------------------------------------------------*/
cy_rslt_t cy_aws_calculate_next_backoff( BackoffAlgorithmParams_t * pContext )
{
    cy_rslt_t result          = CY_RSLT_SUCCESS;
    uint16_t nextRetryBackoff = 0;
    uint32_t currentTickCount = Clock_GetTimeMs();

    ( void ) BackoffAlgorithm_GetNextBackoff( &( pContext->context ),
                                                cy_rand(),
                                                &nextRetryBackoff );
    pContext->nextAllowedTickCount = nextRetryBackoff + currentTickCount;

    return ( result );
}

/*-----------------------------------------------------------*/
