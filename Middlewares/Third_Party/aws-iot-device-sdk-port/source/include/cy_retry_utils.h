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
 * @file cy_retry_utils.h
 * Declaration of the exponential backoff retry logic utility functions and constants.
 */

#ifndef CY_RETRY_UTILS_H_
#define CY_RETRY_UTILS_H_

/* Standard include. */
#include <stdint.h>

#include "backoff_algorithm.h"
/**
 * @brief Max number of retry attempts. Set this value to 0 if the client must
 * retry forever.
 */
#ifndef CY_BACKOFF_MAX_RETRY_ATTEMPTS
#define CY_BACKOFF_MAX_RETRY_ATTEMPTS               4U
#endif

/**
 * @brief Initial fixed backoff value in seconds between two successive
 * retries. A random jitter value is added to every backoff value.
 */
#define INITIAL_RETRY_BACKOFF_SECONDS    1U

/**
 * @brief Max backoff value in seconds.
 */
#define MAX_RETRY_BACKOFF_SECONDS        128U

/**
 * @brief Max jitter value in seconds.
 */
#define MAX_JITTER_VALUE_SECONDS         5U

/**
 * @brief Status for @ref RetryUtils_BackoffAndSleep.
 */
typedef enum RetryUtilsStatus
{
    RetryUtilsSuccess = 0,     /**< @brief The function returned successfully after sleeping. */
    RetryUtilsRetriesExhausted /**< @brief The function exhausted all retry attempts. */
} RetryUtilsStatus_t;

/**
 * @brief Represents parameters required for retry logic.
 */
typedef struct RetryUtilsParams
{
    /**
     * @brief The cumulative count of backoff delay cycles completed
     * for retries.
     */
    uint32_t attemptsDone;

    /**
     * @brief The max jitter value for backoff time in retry attempt.
     */
    uint32_t nextJitterMax;
} RetryUtilsParams_t;

/**
 * @brief Represents parameters Backoff Algorithm.
 */
typedef struct BackoffAlgorithmParams
{
    /**
     * @brief Backoff Algorithm context.
     */
    BackoffAlgorithmContext_t context;

    /**
     * @brief Next allowed tick count.
     */
    uint32_t nextAllowedTickCount;

} BackoffAlgorithmParams_t;


/**
 * @brief Resets the retry timeout value and number of attempts.
 * This function must be called by the application before a new retry attempt.
 *
 * @param[in, out] pRetryParams Structure containing attempts done and timeout
 * value.
 */
/* @[define_retryutils_paramsreset] */
void RetryUtils_ParamsReset( RetryUtilsParams_t * pRetryParams );
/* @[define_retryutils_paramsreset] */

/**
 * @brief Simple platform specific exponential backoff function. The application
 * must use this function between retry failures to add exponential delay.
 * This function will block the calling task for the current timeout value.
 *
 * @param[in, out] pRetryParams Structure containing retry parameters.
 *
 * @return #RetryUtilsSuccess after a successful sleep, #RetryUtilsRetriesExhausted
 * when all attempts are exhausted.
 */
/* @[define_retryutils_backoffandsleep] */
RetryUtilsStatus_t RetryUtils_BackoffAndSleep( RetryUtilsParams_t * pRetryParams );
/* @[define_retryutils_backoffandsleep] */

/**
 * @brief Initializes the context for using backoff algorithm. The parameters
 * are required for calculating the next retry backoff delay.
 *
 * @param[out] pContext The context to initialize with parameters required
 * for the next backoff delay calculation function.
 * @param[in] maxBackOff The maximum backoff delay (in milliseconds) between
 * consecutive retry attempts.
 * @param[in] backOffBase The base value (in milliseconds) of backoff delay to
 * use in the exponential backoff and jitter model.
 * @param[in] maxAttempts The maximum number of retry attempts. Set the value to
 * 0 to retry for ever.
 */
/* @[define_cy_aws_initialize_backoff_params] */
void cy_aws_initialize_backoff_params(  BackoffAlgorithmParams_t * pContext,
                                        uint16_t backOffBase,
                                        uint16_t maxBackOff,
                                        uint32_t maxAttempts );
/**
 * @brief Checks whether the context is in back off period. If yes, inBackOff will
 * be set to true, false otherwise.
 *
 * @param[in, out] pContext Structure containing parameters for the Backoff.
 * @param[in, out] inBackOff Indicating whether the context is still in backoff.
 *
 * @return CY_RSLT_SUCCESS if the back off calculation is success.
 */
/* @[define_cy_aws_get_backoff_status] */
cy_rslt_t cy_aws_get_backoff_status( BackoffAlgorithmParams_t * pContext,
                                     bool *inBackOff );

/**
 * @brief After a failure of an operation that needs to be retried, the application
 * should use this function to obtain the backoff delay value for the next retry,
 * and then wait for the backoff time period before retrying the operation.
 *
 * @param[in, out] pContext Structure containing parameters for the next backoff
 * value calculation.
 *
 * @return cy_rslt_t after a successful calculation of next backoff.
 */
/* @[define_cy_aws_calculate_next_backoff] */
cy_rslt_t cy_aws_calculate_next_backoff( BackoffAlgorithmParams_t * pContext );

#endif /* ifndef CY_RETRY_UTILS_H_ */
