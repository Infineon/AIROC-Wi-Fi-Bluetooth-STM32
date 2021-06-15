/*
 * $ Copyright Cypress Semiconductor $
 */

/** @file
 *
 *  \addtogroup timer Timer Management Services
 *  \ingroup HardwareDrivers
 *
 *  @{
 *
 * Defines the interfaces for Timer Management Services
 */

#ifndef _WICED_TIMER_H_
#define _WICED_TIMER_H_

#include "wiced_result.h"

typedef void *wiced_timer_callback_arg_t;                   /**< Timer callback argument type */
typedef wiced_timer_callback_arg_t WICED_TIMER_PARAM_TYPE;  /**< To avoid recompiling older apps */

/**
 * Function prototype for the timer call back.
 * @param[in]    cb_params      :Timer callback function
 */
typedef void (wiced_timer_callback_t)(wiced_timer_callback_arg_t cb_params);

/** Timer structure.
 * 
 *  NOTE: this structure is used internally by the wiced stack. Applications MUST NOT
 *        modify any of the elements of this structure.
 *
 *        Timer control block memory MUST be peristant from when the timer is initialized,
 *        using wiced_init_timer(), till it is de-initialized, using wiced_deinit_timer().
 */
typedef struct _wiced_timer_t
{
    struct _wiced_timer_t       *p_next;   /**< internal, next pointer to \ref wiced_timer_t */
    wiced_timer_callback_t      *p_cback;  /**< internal, the callback function */
    wiced_timer_callback_arg_t  cb_arg;    /**< internal, the callback argument */
    uint32_t                    pi;        /**< internal, timer flags */
    uint64_t                    tt;        /**< internal, timer timeout */
} wiced_timer_t;

/**
 * Defines the wiced timer types. These timers are system tick driven and a
 * systick is 1 millisecond.So the minimum timer resolution supported is
 * 1 millisecond
 */
typedef enum
{
    WICED_MILLI_SECONDS_TIMER,
    WICED_SECONDS_TIMER,
    WICED_MILLI_SECONDS_PERIODIC_TIMER,
    WICED_SECONDS_PERIODIC_TIMER,
} wiced_timer_type_e;

#ifdef __cplusplus
extern "C"
{
#endif

/**  Initializes the timer
 *
 *@param[in]    p_timer         :Pointer to the timer structure
 *@param[in]    p_timer_cb      :Timer callback function to be invoked on timer expiry
 *@param[in]    cb_arg          :Parameter to be passed to the timer callback function which
 *                                              gets invoked on timer expiry,if any
 *@param[in]    timer_type      :Shows if the timer is milliseconds or seconds, and if periodic or not
 *
 * @return   wiced_result_t
 */
wiced_result_t wiced_init_timer (wiced_timer_t* p_timer, wiced_timer_callback_t *p_timer_cb,
                                 wiced_timer_callback_arg_t cb_arg, wiced_timer_type_e timer_type);

/** Starts a timer
 *
 * @param[in]    p_timer                :Pointer to the timer structure
 * @param[in]    timeout_ms             :timeout in milliseconds
 *
 * @return       wiced_result_t
 */
wiced_result_t wiced_start_timer (wiced_timer_t* p_timer,uint32_t timeout_ms);

/** Stops a timer
 *
 * @param[in]    p_timer      :Pointer to the timer structure
 *
 * @return       wiced_result_t
 */
wiced_result_t wiced_stop_timer (wiced_timer_t *p_timer);

/**  Checks if a timer is in use
*
*@param[in]    p_timer                  :Pointer to the timer structure
*
* @return   TRUE if the timer is in use and FALSE if the timer is not in use
*/
wiced_bool_t wiced_is_timer_in_use (wiced_timer_t *p_timer);

/** Deinitialize a timer instance and stops the timer if it is running
 *
 * @param[in]    p_timer                :Pointer to the timer
 *
 * @return       wiced_result_t
 */
wiced_result_t wiced_deinit_timer (wiced_timer_t* p_timer);

/** @} */
#ifdef __cplusplus
}
#endif

#endif // _WICED_TIMER_H_

