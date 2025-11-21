/*******************************************************************************
* \file cybt_serialize.c
*
*
* \brief
* Provides some application utility functions.
*
********************************************************************************
* \copyright
* Copyright 2018-2019 Cypress Semiconductor Corporation
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/
#include "wiced_bt_version.h"

#include "cyabs_rtos.h"
#include "cybt_result.h"
#include "cybt_platform_task.h"
#include "cybt_platform_interface.h"

/**
*  @addtogroup    app_utils   Application Utility Functions
*
*  @{
*/

// The maximum number of outstanding serialized calls allowed
#ifndef SERIALIZE_APP_QUEUE_SIZE
    #define SERIALIZE_APP_QUEUE_SIZE   (4)
#endif

typedef struct
{
    cy_queue_t   serialQ;
    wiced_bool_t bInitialized;
} CYBT_SER_Q_t;

typedef struct
{
    wiced_bt_serialized_app_func_t   pf;        // App function pointer
    void                             *pp;       // App parameter
} CYBT_SER_Q_ENTRY_t;


CYBT_SER_Q_t serial_q_struct = {0};

/**
* Called by applications to serialize the execution of an application function in the BT_Task context
* This function is safe to be called from ISR.
*
* @param[in] p_func   Function to be called in the BT stack context
* @param[in] param:   Parameter to be passed
*
* @returns  WICED_BT_SUCCESS if success else error reason.
*/
wiced_result_t wiced_bt_serialize_function_from_isr (wiced_bt_serialized_app_func_t p_func, void *param)
{
    CYBT_SER_Q_ENTRY_t   entry;

    // First time called, initialize the queue
    if (!serial_q_struct.bInitialized)
    {
        if (cy_rtos_init_queue(&serial_q_struct.serialQ, SERIALIZE_APP_QUEUE_SIZE, sizeof (CYBT_SER_Q_ENTRY_t)) == CY_RSLT_SUCCESS)
            serial_q_struct.bInitialized = WICED_TRUE;
        else
            return WICED_QUEUE_ERROR;
    }

    entry.pf = p_func;
    entry.pp = param;

    if (cy_rtos_put_queue(&serial_q_struct.serialQ, &entry, 0, false) == CY_RSLT_SUCCESS)
    {
        cybt_send_msg_to_hci_rx_task(BT_IND_TO_APP_SERIALIZATION, WICED_FALSE);
        return (WICED_BT_SUCCESS);
    }
    else
    {
        return (WICED_QUEUE_ERROR);
    }
}

/**
* Call back to the application in the BT stack context
*
* @param[in] void
*
* @returns  void
*/
void cybt_call_app_in_stack_context (void)
{
    CYBT_SER_Q_ENTRY_t   entry;

    if (cy_rtos_get_queue(&serial_q_struct.serialQ, &entry, 0, false) == CY_RSLT_SUCCESS)
        (entry.pf)(entry.pp);
}

/**@} */
