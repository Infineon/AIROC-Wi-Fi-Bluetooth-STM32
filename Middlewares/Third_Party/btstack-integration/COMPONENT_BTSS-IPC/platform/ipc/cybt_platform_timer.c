/***************************************************************************//**
* \file cybt_platform_freertos.c
*
* \brief
* Implementation for BT porting interface on FreeRTOS
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
#include "cyabs_rtos.h"
#include "cybt_platform_task.h"
#include "cybt_platform_interface.h"
#include "cybt_platform_trace.h"
#include "cybt_platform_internal.h"

/******************************************************************************
 *                                Constants
 ******************************************************************************/
#define NOT_IN_ISR             (false)
#define IN_ISR                 (true)
#define ENABLE_EVENT           (true)
#define DISABLE_EVENT          (false)

/*****************************************************************************
 *                           Type Definitions
 *****************************************************************************/


/******************************************************************************
 *                           Variables Definitions
 ******************************************************************************/

static cy_timer_t stack_timer;
/******************************************************************************
 *                          Function Declarations
 ******************************************************************************/
extern cybt_result_t cybt_platform_msg_to_bt_task(const uint16_t msg, bool is_from_isr);

/******************************************************************************
 *                           Function Definitions
 ******************************************************************************/
uint64_t cybt_platform_get_tick_count_us(void)
{
    static cy_time_t last_time_in_ms = 0;
    static uint64_t abs_time_cnt_in_us_hi = 0;
    cy_time_t cur_time_in_ms;
    uint64_t cur_time_in_ms64 = 0;
    uint64_t cur_time_in_us64 = 0;

    cy_rtos_get_time(&cur_time_in_ms);

    if(cur_time_in_ms < last_time_in_ms)
    {
        abs_time_cnt_in_us_hi += 0x100000000;
    }

    last_time_in_ms = cur_time_in_ms;

    cur_time_in_ms64 = cur_time_in_ms + abs_time_cnt_in_us_hi;
    cur_time_in_us64 = (cur_time_in_ms64 * 1000);
    return (cur_time_in_us64);
}

void cybt_platform_set_next_timeout(uint64_t abs_tick_us_to_expire)
{
    uint64_t curr_time_in_us = cybt_platform_get_tick_count_us();
    uint64_t time_to_expire_in_us = abs_tick_us_to_expire - curr_time_in_us;

    if(abs_tick_us_to_expire <= curr_time_in_us)
    {
        cybt_platform_msg_to_bt_task(BT_EVT_TIMEOUT, NOT_IN_ISR);
        return;
    }

    {
        cy_rslt_t result;
        cy_time_t next_timeout = (cy_time_t)(time_to_expire_in_us/1000);

        /* No need to stop this timer, internally FREE-RTOS restarting the timer
         * Leaving reminder (~1ms), Its ok verified */
        result = cy_rtos_start_timer(&stack_timer, next_timeout);
        if(CY_RSLT_SUCCESS != result)
        {
            MAIN_TRACE_DEBUG("timer failed to start %u\n", next_timeout);
        }
    }
}

static void platform_stack_timer_callback(cy_timer_callback_arg_t arg)
{
    /* Will be called from TmrSVC rtos thread context and not from ISR context */
    cybt_platform_msg_to_bt_task(BT_EVT_TIMEOUT, NOT_IN_ISR);
}

void cybt_platform_stack_timer_init(void)
{
    cy_rtos_init_timer(&stack_timer, CY_TIMER_TYPE_ONCE, platform_stack_timer_callback, (cy_timer_callback_arg_t)NULL);
}

void cybt_platform_stack_timer_deinit(void)
{
    cy_rtos_deinit_timer(&stack_timer);
}
