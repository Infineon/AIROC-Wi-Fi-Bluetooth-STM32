/*******************************************************************************
* \file cybt_platform_freertos.c
*
* \brief
* Implementation for BT porting interface on FreeRTOS
*
********************************************************************************
* \copyright
* Copyright 2018-2021 Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.
*
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

// Updated cybt_platform_log_print function to use ENABLE_DEBUG_UART macro & enable COMPONENT_55X. 
// Using Enter Download Mode command method instead of autobaud

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "wiced_data_types.h"
#include "platform_hal_wrapper.h"

#include "cybt_platform_interface.h"
#include "cybt_platform_task.h"
#include "cybt_platform_trace.h"
#include "cybt_platform_util.h"

#if (!defined (CY_USING_HAL) && (CY_CFG_PWR_SYS_IDLE_MODE == CY_CFG_PWR_MODE_DEEPSLEEP))
#include "mtb_syspm_callbacks.h"
#endif
/******************************************************************************
 *                                Constants
 ******************************************************************************/

#define HCI_SEMAPHORE_MAX_COUNT  (1)
#define HCI_SEMAPHORE_INIT_COUNT (0)

#if( configUSE_TICKLESS_IDLE != 0 )
#define PLATFORM_SLEEP_IDLE_TIMEOUT_MS     (3000)

#define SLEEP_TASK_NAME               "sleep_task"
#define SLEEP_TASK_STACK_SIZE         (1024)
#define SLEEP_TASK_PRIORITY           (CY_RTOS_PRIORITY_REALTIME)
#define SLEEP_TASK_QUEUE_COUNT        (32)
#define SLEEP_TASK_QUEUE_ITEM_SIZE    (sizeof(uint8_t))

#define SLEEP_ACT_START_IDLE_TIMER    (0x20)
#define SLEEP_ACT_STOP_IDLE_TIMER     (0x40)
#define SLEEP_ACT_EXIT_SLEEP_TASK     (0xFF)
#endif

#if (!defined (CY_USING_HAL) && (CY_CFG_PWR_SYS_IDLE_MODE == CY_CFG_PWR_MODE_DEEPSLEEP))
/* Default syspm callback configuration elements */
#define SYSPM_SKIP_MODE         (0U)
#define SYSPM_CALLBACK_ORDER    (1U)
#endif
#define COMPONENT_55X

/*****************************************************************************
 *                           Type Definitions
 *****************************************************************************/
#if( configUSE_TICKLESS_IDLE != 0 )
typedef uint8_t sleep_action_t;
#endif

/******************************************************************************
 *                           Variables Definitions
 ******************************************************************************/
extern hci_uart_cb_t   hci_uart_cb;

#if (!defined (CY_USING_HAL) && (CY_CFG_PWR_SYS_IDLE_MODE == CY_CFG_PWR_MODE_DEEPSLEEP))
extern cy_stc_scb_uart_context_t  CYBSP_BT_UART_context;  /* BT UART context */
extern mtb_async_transfer_context_t CYBSP_BT_UART_async_tx_context; /* BT UART async context */
#endif

#if( configUSE_TICKLESS_IDLE != 0 )
// This timer is only used in active mode,
// hence it's implemented by cy_rtos_timer API .
cy_timer_t      platform_sleep_idle_timer;
bool            platform_sleep_lock = false;

cy_thread_t     sleep_timer_task;
cy_queue_t      sleep_timer_task_queue;
#endif

static cy_timer_t stack_timer;
/******************************************************************************
 *                          Function Declarations
 ******************************************************************************/
#if( configUSE_TICKLESS_IDLE != 0 )
void cybt_idle_timer_cback(cy_timer_callback_arg_t arg);

cybt_result_t cybt_send_action_to_sleep_task(sleep_action_t action);
void cybt_sleep_timer_task(cy_thread_arg_t arg);
#endif

/******************************************************************************
 *                           Function Definitions
 ******************************************************************************/
static void platform_stack_timer_callback(cy_timer_callback_arg_t arg)
{
    cybt_platform_sleep_lock();

    cybt_send_msg_to_hci_rx_task(BT_IND_TO_BTS_TIMER, true);

    cybt_platform_sleep_unlock();
}


void *cybt_platform_malloc(uint32_t req_size)
{
    return malloc((size_t) req_size);
}

void cybt_platform_free(void *p_mem_block)
{
    free(p_mem_block);
}

void cybt_platform_disable_irq(void)
{
    __disable_irq();
}

void cybt_platform_enable_irq(void)
{
    __enable_irq();
}

/* BT UART deepsleep callback parameters  */
#if (!defined (CY_USING_HAL) && (CY_CFG_PWR_SYS_IDLE_MODE == CY_CFG_PWR_MODE_DEEPSLEEP))
/* Context reference structure for Debug UART */
static mtb_syspm_uart_deepsleep_context_t platform_syspm_ds_context =
{
    .uart_context       = &CYBSP_BT_UART_context,
    .async_context      = &CYBSP_BT_UART_async_tx_context,
    .tx_pin =
    {
        .port           = CYBSP_BT_UART_TX_PORT,
        .pinNum         = CYBSP_BT_UART_TX_PIN,
        .hsiom          = CYBSP_BT_UART_TX_HSIOM
    },
    .rts_pin =
    {
        .port           = CYBSP_BT_UART_RTS_PORT,
        .pinNum         = CYBSP_BT_UART_RTS_PIN,
        .hsiom          = CYBSP_BT_UART_RTS_HSIOM
    }
};

/* SysPm callback parameter structure for Debug UART */
static cy_stc_syspm_callback_params_t platform_syspm_cb_params =
{
    .context            = &platform_syspm_ds_context,
    .base               = CYBSP_BT_UART_HW
};

/* SysPm callback structure for BT UART */
static cy_stc_syspm_callback_t platform_syspm_cb =
{
    .callback           = &mtb_syspm_scb_uart_deepsleep_callback,
    .skipMode           = SYSPM_SKIP_MODE,
    .type               = CY_SYSPM_DEEPSLEEP,
    .callbackParams     = &platform_syspm_cb_params,
    .prevItm            = NULL,
    .nextItm            = NULL,
    .order              = SYSPM_CALLBACK_ORDER
};
#endif

void cybt_platform_init(void)
{
#if( configUSE_TICKLESS_IDLE != 0 )
    cy_rtos_init_timer(&platform_sleep_idle_timer,
                       CY_TIMER_TYPE_ONCE,
                       cybt_idle_timer_cback,
                       0
                      );
    MAIN_TRACE_DEBUG("cybt_platform_init(): platform_sleep_idle_timer = 0x%x", &platform_sleep_idle_timer);
#endif

    memset(&hci_uart_cb, 0, sizeof(hci_uart_cb_t));

    cy_rtos_init_timer(&stack_timer, CY_TIMER_TYPE_ONCE, platform_stack_timer_callback, (cy_timer_callback_arg_t)NULL);

#if( configUSE_TICKLESS_IDLE != 0 )
    cy_rtos_init_queue(&sleep_timer_task_queue,
                       SLEEP_TASK_QUEUE_COUNT,
                       SLEEP_TASK_QUEUE_ITEM_SIZE
                      );

    cy_rtos_create_thread(&sleep_timer_task,
                          cybt_sleep_timer_task,
                          SLEEP_TASK_NAME,
                          NULL,
                          SLEEP_TASK_STACK_SIZE,
                          SLEEP_TASK_PRIORITY,
                          (cy_thread_arg_t) 0
                         );
#endif
}

void cybt_platform_deinit(void)
{
    MAIN_TRACE_DEBUG("cybt_platform_deinit()");

#if( configUSE_TICKLESS_IDLE != 0 )
    cybt_send_action_to_sleep_task(SLEEP_ACT_EXIT_SLEEP_TASK);
#endif

    cy_rtos_deinit_timer(&stack_timer);

#if( configUSE_TICKLESS_IDLE != 0 )
    cy_rtos_deinit_timer(&platform_sleep_idle_timer);
#endif
}

void cybt_platform_sleep_lock(void)
{
#if( configUSE_TICKLESS_IDLE != 0 )
    cybt_platform_disable_irq();

    if(false == platform_sleep_lock)
    {
        platform_hal_syspm_lock_deepsleep_wrapper();
        platform_sleep_lock = true;
    }

    cybt_send_action_to_sleep_task(SLEEP_ACT_STOP_IDLE_TIMER);
    cybt_platform_enable_irq();
#endif
}

void cybt_platform_sleep_unlock(void)
{
#if( configUSE_TICKLESS_IDLE != 0 )
    cybt_platform_disable_irq();

    if(true == platform_sleep_lock)
    {
        cybt_send_action_to_sleep_task(SLEEP_ACT_START_IDLE_TIMER);
        cybt_platform_enable_irq();
    }
    else
    {
        cybt_platform_enable_irq();
    }
#endif
}

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
        // Already expired...
        cybt_send_msg_to_hci_rx_task(BT_IND_TO_BTS_TIMER, true);

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

__attribute__((weak)) cybt_result_t cybt_debug_uart_send_trace(uint16_t length, uint8_t* p_data)
{
    return CYBT_SUCCESS;
}

void cybt_platform_log_print(const char *fmt_str, ...)
{
    char buffer[CYBT_TRACE_BUFFER_SIZE];
    va_list ap;
    int len;
    cy_time_t time;

    cy_rtos_get_time(&time);
    va_start(ap, fmt_str);
    len = vsnprintf(buffer, CYBT_TRACE_BUFFER_SIZE, fmt_str, ap);
    va_end(ap);
#ifdef ENABLE_DEBUG_UART
    cybt_debug_uart_send_trace(len, (uint8_t*)buffer);
#else
    printf("[%u] %s\r\n", (unsigned int)time, buffer);
    UNUSED_VARIABLE(len);
#endif // DISABLE_DEBUG_UART_PRINT
}

void cybt_platform_assert_bt_wake(void)
{
    bool wake_polarity=platform_hal_gpio_dev_wake_polarity_wrapper();

    if(true == cybt_platform_get_sleep_mode_status())
    {
        platform_hal_gpio_write_wrapper(DEVICE_WAKE_PIN, wake_polarity);
    }
}

void cybt_platform_deassert_bt_wake(void)
{
    bool wake_polarity=platform_hal_gpio_dev_wake_polarity_wrapper();

    if(true == cybt_platform_get_sleep_mode_status())
    {
        platform_hal_gpio_write_wrapper(DEVICE_WAKE_PIN, !wake_polarity);
    }
}

#ifdef COMPONENT_55X
cybt_result_t cybt_platform_enter_download_mode(void)
{
#define ENTER_DOWNLOAD_MODE_RES_LEN 29
    static uint8_t enter_download_mode[] = {0x01, 0xED, 0xFF, 0x14, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xED, 0xFF, 0x01, 0x00};
    size_t tx_size = sizeof(enter_download_mode);
    uint8_t rx_data[ENTER_DOWNLOAD_MODE_RES_LEN];
    uint8_t total_read_bytes = 0;
    size_t required_len = ENTER_DOWNLOAD_MODE_RES_LEN;
    cy_rslt_t res = cyhal_uart_write(&hci_uart_cb.hal_obj,
                                        (void *) enter_download_mode,
                                        &tx_size
                                       );
    if (res)
        return res;
    while(total_read_bytes < ENTER_DOWNLOAD_MODE_RES_LEN)
    {
        cy_rtos_delay_milliseconds(10);
        res = cyhal_uart_read(&hci_uart_cb.hal_obj, (uint8_t *) rx_data, &required_len);
        if (res)
            return res;
        if (required_len == 0)
            continue;
        total_read_bytes += required_len;
        required_len = ENTER_DOWNLOAD_MODE_RES_LEN-total_read_bytes;
    }
    return CYBT_SUCCESS;
}
#endif // COMPONENT_55X

cybt_result_t cybt_platform_hci_open(void *p_arg)
{
    cybt_result_t result;
    UNUSED_VARIABLE(p_arg);

#ifdef COMPONENT_55X
    uint32_t platform_baud_download=platform_hal_get_fw_download_baud_wrapper();
#endif

    if(true == hci_uart_cb.inited)
    {
        return  CYBT_SUCCESS;
    }

    memset(&hci_uart_cb, 0, sizeof(hci_uart_cb_t));

    cy_rtos_init_semaphore(&hci_uart_cb.tx_complete,
                           HCI_SEMAPHORE_MAX_COUNT,
                           HCI_SEMAPHORE_INIT_COUNT
                          );
    cy_rtos_init_semaphore(&hci_uart_cb.rx_complete,
                           HCI_SEMAPHORE_MAX_COUNT,
                           HCI_SEMAPHORE_INIT_COUNT
                          );
    cy_rtos_init_mutex(&hci_uart_cb.tx_atomic);
    cy_rtos_init_mutex(&hci_uart_cb.rx_atomic);

#if( configUSE_TICKLESS_IDLE != 0 )
    if((CYBT_SLEEP_MODE_ENABLED == platform_hal_get_sleep_mode_enabled_wrapper()))
    {
        result = platform_hal_gpio_init_wrapper(HOST_WAKE_PIN);
        CYBT_RSLT_CHECK(result);

        result = platform_hal_gpio_register_callback_wrapper(HOST_WAKE_PIN);
        CYBT_RSLT_CHECK(result);

        result = platform_hal_gpio_enable_event_wrapper(HOST_WAKE_PIN);
        CYBT_RSLT_CHECK(result);
    }
#endif

    if(CYBT_SLEEP_MODE_ENABLED == platform_hal_get_sleep_mode_enabled_wrapper())
    {
        result = platform_hal_gpio_init_wrapper(DEVICE_WAKE_PIN);
        CYBT_RSLT_CHECK(result);

        result = platform_hal_gpio_write_wrapper(DEVICE_WAKE_PIN,false);
        CYBT_RSLT_CHECK(result);
        cy_rtos_delay_milliseconds(100);
    }

    result = platform_hal_gpio_init_wrapper(BT_POWER_PIN);
    CYBT_RSLT_CHECK(result);


    result = platform_hal_gpio_write_wrapper(BT_POWER_PIN,true);
    CYBT_RSLT_CHECK(result);
    cy_rtos_delay_milliseconds(500);

    result = platform_hal_uart_init_wrapper();
    CYBT_RSLT_CHECK(result);

#ifdef COMPONENT_55X
    result = platform_hal_uart_set_baud_wrapper(platform_baud_download);
#else
    result = platform_hal_uart_set_baud_wrapper(HCI_UART_DEFAULT_BAUDRATE);
#endif
    CYBT_RSLT_CHECK(result);

    result = platform_hal_uart_enable_flow_control_wrapper();
    CYBT_RSLT_CHECK(result);

#ifdef COMPONENT_55X
    cybt_platform_enter_download_mode();
#endif // COMPONENT_55X

    platform_hal_uart_register_callback_wrapper(true);
    platform_hal_uart_enable_event_wrapper(true);

    HCIDRV_TRACE_DEBUG("hci_open(): Wait CTS low");

    while(true == platform_hal_gpio_read_wrapper(BT_UART_CTS))
    {
        cy_rtos_delay_milliseconds(10);
    }

#if (!defined (CY_USING_HAL) && (CY_CFG_PWR_SYS_IDLE_MODE == CY_CFG_PWR_MODE_DEEPSLEEP))
    /* UART SysPm callback registration for BT UART */
    Cy_SysPm_RegisterCallback(&platform_syspm_cb);
#endif /* (CY_CFG_PWR_SYS_IDLE_MODE == CY_CFG_PWR_MODE_DEEPSLEEP) */

    hci_uart_cb.inited = true;

    HCIDRV_TRACE_DEBUG("hci_open(): Done");

    return  CYBT_SUCCESS;
}


cybt_result_t cybt_platform_hci_set_baudrate(uint32_t baudrate)
{
    cybt_result_t result = platform_hal_uart_set_baud_wrapper(baudrate);
    if(false == hci_uart_cb.inited)
    {
        HCIDRV_TRACE_ERROR("set_baudrate(): UART is NOT initialized");
        return  CYBT_ERR_HCI_NOT_INITIALIZE;
    }

    return  result;
}

cybt_result_t cybt_platform_hci_write(hci_packet_type_t type,
                                                  uint8_t          *p_data,
                                                  uint32_t         length
                                                 )
{
    cy_rslt_t result;
    cybt_result_t cybt_result =  CYBT_SUCCESS;

    if(false == hci_uart_cb.inited)
    {
        HCIDRV_TRACE_ERROR("hci_write(): UART is NOT initialized");
        return  CYBT_ERR_HCI_NOT_INITIALIZE;
    }

    result = cy_rtos_get_mutex(&hci_uart_cb.tx_atomic, CY_RTOS_NEVER_TIMEOUT);
    if(CY_RSLT_SUCCESS != result)
    {
        HCIDRV_TRACE_ERROR("hci_write(): Get mutex error (0x%x)\n", result);
        return  CYBT_ERR_HCI_GET_TX_MUTEX_FAILED;
    }

    cybt_platform_sleep_lock();
    cybt_platform_assert_bt_wake();

    cybt_result = platform_hal_uart_write_async_wrapper((void *) p_data,(size_t) length);
    CYBT_RSLT_CHECK(cybt_result);
    if(CYBT_SUCCESS == cybt_result)
    {
        cy_rtos_get_semaphore(&hci_uart_cb.tx_complete, CY_RTOS_NEVER_TIMEOUT, false);
    }

    cybt_platform_deassert_bt_wake();
    cybt_platform_sleep_unlock();

    cy_rtos_set_mutex(&hci_uart_cb.tx_atomic);

    return cybt_result;
}

cybt_result_t cybt_platform_hci_read(hci_packet_type_t type,
                                                uint8_t           *p_data,
                                                uint32_t          *p_length,
                                                uint32_t          timeout_ms
                                               )
{
    uint32_t  req_len = *p_length;
    cy_rslt_t result;
    cybt_result_t cybt_result;

    if(false == hci_uart_cb.inited)
    {
        HCIDRV_TRACE_ERROR("hci_read(): UART is NOT initialized");
        return  CYBT_ERR_HCI_NOT_INITIALIZE;
    }

    if (0 == req_len)
    {
        HCIDRV_TRACE_ERROR("hci_read(): Required length = 0\n");
        return  CYBT_ERR_BADARG;
    }

    result = cy_rtos_get_mutex(&hci_uart_cb.rx_atomic, timeout_ms);
    if(CY_RSLT_SUCCESS != result)
    {
        HCIDRV_TRACE_ERROR("hci_read(): Get mutex error (0x%x)\n", result);
        return  CYBT_ERR_HCI_GET_RX_MUTEX_FAILED;
    }

    cybt_platform_sleep_lock();

    if(0 < timeout_ms)
    {
        cybt_result = platform_hal_uart_read_async_wrapper((void *) p_data,(size_t) req_len);
        if(CYBT_SUCCESS == cybt_result)
        {
            result = cy_rtos_get_semaphore(&hci_uart_cb.rx_complete,
                                           timeout_ms,
                                           false
                                          );
            if(CY_RSLT_SUCCESS != result)
            {
                #if defined (CY_USING_HAL)
                HCIDRV_TRACE_ERROR("hci_read(): failed (0x%x), read size = %d",
                                   result,
                                   hci_uart_cb.hal_obj.context.rxBufIdx
                                  );
                #else
                HCIDRV_TRACE_ERROR("hci_read(): failed (0x%x), read size = %d",
                                   result,
                                   hci_uart_cb.CYBSP_BT_UART_hal_obj.context->rxBufIdx
                                  );
                #endif

                if(CY_RTOS_TIMEOUT == result)
                {
                    cybt_result =  CYBT_ERR_TIMEOUT;
                }
                else
                {
                    cybt_result =  CYBT_ERR_GENERIC;
                }
                #if defined (CY_USING_HAL)
                *p_length = hci_uart_cb.hal_obj.context.rxBufIdx;
                #else
                *p_length = hci_uart_cb.CYBSP_BT_UART_hal_obj.context->rxBufIdx;
                #endif

                cybt_result = platform_hal_uart_read_abort_wrapper();
            }
            else
            {
        cybt_result = CYBT_SUCCESS;
            }
        }
        else
        {
            cybt_platform_sleep_unlock();
            cy_rtos_set_mutex(&hci_uart_cb.rx_atomic);
            return cybt_result;
        }
    }
    else
    {
        cybt_result = platform_hal_uart_read_wrapper((void *) p_data,(size_t *)p_length);
        CYBT_RSLT_CHECK(cybt_result);
    }

    cybt_platform_sleep_unlock();
    cy_rtos_set_mutex(&hci_uart_cb.rx_atomic);

    return cybt_result;

}

cybt_result_t cybt_platform_hci_close(void)
{
    cybt_result_t result;

    if(false == hci_uart_cb.inited)
    {
        HCIDRV_TRACE_ERROR("hci_close(): Not inited\n");
        return  CYBT_ERR_HCI_NOT_INITIALIZE;
    }

    platform_hal_uart_register_callback_wrapper(false);
    platform_hal_uart_enable_event_wrapper(false);

    if(CYBT_SLEEP_MODE_ENABLED == platform_hal_get_sleep_mode_enabled_wrapper())
    {
        result = platform_hal_gpio_enable_event_wrapper(HOST_WAKE_PIN);
        CYBT_RSLT_CHECK(result);

        result = platform_hal_gpio_register_callback_wrapper(HOST_WAKE_PIN);
        CYBT_RSLT_CHECK(result);
    }

    result = platform_hal_gpio_free_wrapper(DEVICE_WAKE_PIN);
    CYBT_RSLT_CHECK(result);

    result = platform_hal_gpio_free_wrapper(HOST_WAKE_PIN);
    CYBT_RSLT_CHECK(result);

    cy_rtos_deinit_mutex(&hci_uart_cb.tx_atomic);
    cy_rtos_deinit_mutex(&hci_uart_cb.rx_atomic);
    cy_rtos_deinit_semaphore(&hci_uart_cb.tx_complete);
    cy_rtos_deinit_semaphore(&hci_uart_cb.rx_complete);

    result = platform_hal_gpio_write_wrapper(BT_POWER_PIN,false);
    CYBT_RSLT_CHECK(result);

    result = platform_hal_gpio_free_wrapper(BT_POWER_PIN);
    CYBT_RSLT_CHECK(result);

    memset(&hci_uart_cb, 0, sizeof(hci_uart_cb_t));

    return  CYBT_SUCCESS;
}

#if( configUSE_TICKLESS_IDLE != 0 )
void cybt_idle_timer_cback(cy_timer_callback_arg_t arg)
{
    cybt_platform_disable_irq();

    if (platform_sleep_lock == true)
    {
        platform_hal_syspm_unlock_deepsleep_wrapper();
        platform_sleep_lock = false;
    }

    cybt_platform_enable_irq();
}

cybt_result_t cybt_send_action_to_sleep_task(sleep_action_t action)
{
    bool is_from_isr = (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0;

    cy_rslt_t result = cy_rtos_put_queue(&sleep_timer_task_queue,
                                         (void *)&action,
                                         0,
                                         is_from_isr
                                        );
    UNUSED_VARIABLE(is_from_isr);

    if(CY_RSLT_SUCCESS != result)
    {
        CY_ASSERT(0);
        return CYBT_ERR_SEND_QUEUE_FAILED;
    }

    return CYBT_SUCCESS;
}

void cybt_sleep_timer_task(cy_thread_arg_t arg)
{
    cy_rslt_t result;
    sleep_action_t action = 0;

    while(1)
    {
        result = cy_rtos_get_queue(&sleep_timer_task_queue,
                                   (void *)&action,
                                   CY_RTOS_NEVER_TIMEOUT,
                                   false
                                  );

        if(CY_RSLT_SUCCESS != result)
        {
            MAIN_TRACE_WARNING("sleep_task(): queue error (0x%x)",
                                result
                              );
            continue;
        }

        if (SLEEP_ACT_EXIT_SLEEP_TASK == action)
        {
            cy_rtos_deinit_queue(&sleep_timer_task_queue);
            break;
        }

        switch(action)
        {
            case SLEEP_ACT_START_IDLE_TIMER:
                cy_rtos_start_timer(&platform_sleep_idle_timer,
                                    PLATFORM_SLEEP_IDLE_TIMEOUT_MS
                                   );
                break;
            case SLEEP_ACT_STOP_IDLE_TIMER:
                {
                    bool is_timer_running = false;
                    cy_rslt_t result;

                    result = cy_rtos_is_running_timer(&platform_sleep_idle_timer,
                                                      &is_timer_running
                                                     );
                    if(CY_RSLT_SUCCESS == result && true == is_timer_running)
                    {
                        cy_rtos_stop_timer(&platform_sleep_idle_timer);
                    }
                }
                break;
            default:
                MAIN_TRACE_ERROR("sleep_task(): Unknown action = 0x%x", action);
                break;
        }
    }
    cy_rtos_exit_thread();
}

void cybt_platform_terminate_sleep_thread(void)
{
    cy_rslt_t cy_result;

    cy_result = cy_rtos_join_thread(&sleep_timer_task);
    if(CY_RSLT_SUCCESS != cy_result)
    {
        MAIN_TRACE_ERROR("terminate Sleep thread failed 0x%x\n", cy_result);
    }
}

#endif
