/*
 * $ Copyright 2023-YEAR Infineon Technologies $
 */

/** @file
 *
 * Provide utility function for serializing
 *
 */

#ifndef __WICED_BT_SERIALIZE__
#define __WICED_BT_SERIALIZE__

#include "wiced_bt_dev.h"
#include "wiced_data_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
*  @addtogroup    app_utils   Application Utility Functions
*
*  @{
*/

/**
* Called by applications to serialize the execution of an application function in the BT stack context.
* This function should not be called from ISR context.
*
* @param[in] p_func   Function to be called in the BT stack context
* @param[in] param:   Parameter to be passed
*
* @returns  WICED_BT_SUCCESS if success else error reason.
*/
wiced_result_t wiced_bt_serialize_function (wiced_bt_serialized_app_func_t p_func, void *param);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* __WICED_BT_SERIALIZE__ */

/**@} */
