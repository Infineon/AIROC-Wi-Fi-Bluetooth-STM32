/***************************************************************************//**
* \file platform_hal_wrapper.c
*
* \brief
* Implementation for HAL wrapper functions
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

#include <stdlib.h>
#include <stdio.h>
#include "platform_hal_wrapper.h"

#if defined (CY_USING_HAL)
/******************************************************************************
 *                                Constants
 ******************************************************************************/

/*****************************************************************************
 *                           Type Definitions
 *****************************************************************************/

/******************************************************************************
 *                           Variables Definitions
 ******************************************************************************/
platform_hal_gpio_config_t gpio_config;
hci_uart_cb_t   hci_uart_cb;

/******************************************************************************
 *                          Function Declarations
 ******************************************************************************/
static void cybt_uart_rx_not_empty(void);
static void cybt_uart_tx_done_irq(void);
static void cybt_uart_rx_done_irq(void);
static void cybt_uart_irq_handler(void *handler_arg, cyhal_uart_event_t event);
const char *get_gpio_pin_name_from_pin_id(bt_gpio_id_t gpio_id);
void platform_hal_host_wake_irq_handler(void *callback_arg, cyhal_gpio_event_t event);
cyhal_gpio_t platform_hal_gpio_get_pin_from_id(bt_gpio_id_t gpio_id);

/******************************************************************************
 *                           Function Definitions
 ******************************************************************************/

/******************************************************************************
*                           HAL GPIO wrappers
******************************************************************************/
cybt_result_t platform_hal_gpio_init_wrapper(bt_gpio_id_t gpio_id)
{
    cy_rslt_t result;
    cyhal_gpio_t gpio_pin = platform_hal_gpio_get_pin_from_id(gpio_id);

    if(NC != gpio_pin)
    {
        //Get the required configurations of gpio for initializing
        platform_hal_gpio_set_config(gpio_id);

        result = cyhal_gpio_init(gpio_pin,
                                 gpio_config.pin_direction,
                                 gpio_config.pin_drive_mode,
                                 gpio_config.pin_init_val
                                 );

        if(CY_RSLT_SUCCESS != result)
        {
            //CYHAL_HWMGR_RSLT_ERR_INUSE returned if the gpio is already initialized by the device configurator or application.
            if(CYHAL_HWMGR_RSLT_ERR_INUSE == result)
            {
                HCIDRV_TRACE_ERROR("[%s] - %s is already initialized - status: (0x%x)", __FUNCTION__,
                                                                                        get_gpio_pin_name_from_pin_id(gpio_id),
                                                                                        result);
            }
            else
            {
                HCIDRV_TRACE_ERROR("[%s] - %s Initialization failed - status: (0x%x)",__FUNCTION__,
                                                                            get_gpio_pin_name_from_pin_id(gpio_id),
                                                                            result);
                return CYBT_ERR_GPIO_INIT_FAILED;
            }
        }

        return CYBT_SUCCESS;
    }
    else
    {
        HCIDRV_TRACE_ERROR("[%s] failed - GPIO: %s, Code: (0x%x)",  __FUNCTION__,
                                                                    get_gpio_pin_name_from_pin_id(gpio_id),
                                                                    CYBT_ERR_GPIO_NC_FAIL)
        return CYBT_ERR_GPIO_NC_FAIL;
    }
}

bool platform_hal_gpio_read_wrapper(bt_gpio_id_t gpio_id)
{
    bool read_val = false;
    cyhal_gpio_t gpio_pin = platform_hal_gpio_get_pin_from_id(gpio_id);

    if(NC != gpio_pin)
    {
        read_val = cyhal_gpio_read(gpio_pin);
    }
    else
    {
        HCIDRV_TRACE_ERROR("[%s] failed - GPIO: %s, Code: (0x%x)",__FUNCTION__,
                                                                        get_gpio_pin_name_from_pin_id(gpio_id),
                                                                    CYBT_ERR_GPIO_NC_FAIL)
    }

    return read_val;
}

cybt_result_t platform_hal_gpio_write_wrapper(bt_gpio_id_t gpio_id, bool value)
{
    cyhal_gpio_t gpio_pin = platform_hal_gpio_get_pin_from_id(gpio_id);

    if(NC != gpio_pin)
    {
        cyhal_gpio_write(gpio_pin, value);
        return CYBT_SUCCESS;
    }
    else
    {
        HCIDRV_TRACE_ERROR("[%s] failed - GPIO: %s, Code: (0x%x)",  __FUNCTION__,
                                                                    get_gpio_pin_name_from_pin_id(gpio_id),
                                                                    CYBT_ERR_GPIO_NC_FAIL)
        return CYBT_ERR_GPIO_NC_FAIL;
    }
}

cybt_result_t platform_hal_gpio_free_wrapper(bt_gpio_id_t gpio_id)
{
    cyhal_gpio_t gpio_pin = platform_hal_gpio_get_pin_from_id(gpio_id);

    if(NC != gpio_pin)
    {
        cyhal_gpio_free(gpio_pin);
        return CYBT_SUCCESS;
    }
    else
    {
        HCIDRV_TRACE_ERROR("[%s] failed - GPIO: %s, Code: (0x%x)",  __FUNCTION__,
                                                                    get_gpio_pin_name_from_pin_id(gpio_id),
                                                                    CYBT_ERR_GPIO_NC_FAIL)
        return CYBT_ERR_GPIO_NC_FAIL;
    }
}

void platform_hal_host_wake_irq_handler(void *callback_arg, cyhal_gpio_event_t event)
{
    const cybt_platform_config_t *p_bt_platform_cfg = cybt_platform_get_config();

    switch(event)
    {
    case CYHAL_GPIO_IRQ_RISE:
        if(CYBT_WAKE_ACTIVE_HIGH == p_bt_platform_cfg->controller_config.sleep_mode.host_wake_polarity)
        {
            cybt_platform_sleep_lock();
        }
        else
        {
            cybt_platform_sleep_unlock();
        }
        break;
    case CYHAL_GPIO_IRQ_FALL:
        if(CYBT_WAKE_ACTIVE_LOW == p_bt_platform_cfg->controller_config.sleep_mode.host_wake_polarity)
        {
            cybt_platform_sleep_lock();
        }
        else
        {
            cybt_platform_sleep_unlock();
        }
        break;
    default:
        break;
    }
}

cybt_result_t platform_hal_gpio_register_callback_wrapper(bt_gpio_id_t gpio_id)
{
    cyhal_gpio_t gpio_pin = platform_hal_gpio_get_pin_from_id(gpio_id);
    const cybt_platform_config_t *p_bt_platform_cfg = cybt_platform_get_config();

    if(NC != gpio_pin)
    {
#if( configUSE_TICKLESS_IDLE != 0 )
    #if (CYHAL_API_VERSION >= 2)
        static cyhal_gpio_callback_data_t cb_data = { .callback = platform_hal_host_wake_irq_handler, .callback_arg = NULL };
        cyhal_gpio_register_callback(p_bt_platform_cfg->controller_config.sleep_mode.host_wakeup_pin, &cb_data);
    #else
        cyhal_gpio_register_callback(p_bt_platform_cfg->controller_config.sleep_mode.host_wakeup_pin,
                                     platform_hal_host_wake_irq_handler,
                                     NULL
                                     );
    #endif
#else
    #if (CYHAL_API_VERSION >= 2)
        cyhal_gpio_register_callback(p_bt_platform_cfg->controller_config.sleep_mode.host_wakeup_pin, NULL);
    #else
        cyhal_gpio_register_callback(p_bt_platform_cfg->controller_config.sleep_mode.host_wakeup_pin,
                                     NULL,
                                     NULL
                                     );
    #endif
#endif
        return CYBT_SUCCESS;
    }
    else
    {
        HCIDRV_TRACE_ERROR("[%s] failed - GPIO: %s, Code: (0x%x)",  __FUNCTION__,
                                                                    get_gpio_pin_name_from_pin_id(gpio_id),
                                                                    CYBT_ERR_GPIO_NC_FAIL)
        return CYBT_ERR_GPIO_NC_FAIL;
    }
}

cybt_result_t platform_hal_gpio_enable_event_wrapper(bt_gpio_id_t gpio_id)
{
    cyhal_gpio_t gpio_pin = platform_hal_gpio_get_pin_from_id(gpio_id);
    const cybt_platform_config_t *p_bt_platform_cfg = cybt_platform_get_config();

    if(NC != gpio_pin)
    {
#if( configUSE_TICKLESS_IDLE != 0 )
        cyhal_gpio_enable_event(p_bt_platform_cfg->controller_config.sleep_mode.host_wakeup_pin,
                                CYHAL_GPIO_IRQ_BOTH,
                                CYHAL_ISR_PRIORITY_DEFAULT,
                                true
                               );
#else
        cyhal_gpio_enable_event(p_bt_platform_cfg->controller_config.sleep_mode.host_wakeup_pin,
                                CYHAL_GPIO_IRQ_NONE,
                                CYHAL_ISR_PRIORITY_DEFAULT,
                                true
                               );
#endif
        return CYBT_SUCCESS;
    }
    else
    {
        HCIDRV_TRACE_ERROR("[%s] failed - GPIO: %s, Code: (0x%x)",  __FUNCTION__,
                                                                    get_gpio_pin_name_from_pin_id(gpio_id),
                                                                    CYBT_ERR_GPIO_NC_FAIL)
        return CYBT_ERR_GPIO_NC_FAIL;
    }
}

uint32_t platform_hal_get_task_mem_pool_size_wrapper(void)
{
    const cybt_platform_config_t *p_bt_platform_cfg = cybt_platform_get_config();

    return (p_bt_platform_cfg->task_mem_pool_size);
}

bool platform_hal_gpio_host_wake_polarity_wrapper(void)
{
    bool polarity = false;
    const cybt_platform_config_t *p_bt_platform_cfg = cybt_platform_get_config();

    if(p_bt_platform_cfg->controller_config.sleep_mode.host_wake_polarity)
    {
    polarity = true;
    }

    return polarity;
}

bool platform_hal_gpio_dev_wake_polarity_wrapper(void)
{
    const cybt_platform_config_t *p_bt_platform_cfg = cybt_platform_get_config();

    if(CYBT_WAKE_ACTIVE_LOW==p_bt_platform_cfg->controller_config.sleep_mode.device_wake_polarity)
    {
        return false;
    }
    else
    {
        return true;
    }
}

bool platform_hal_get_sleep_mode_enabled_wrapper(void)
{
    bool sleep_mode=false;
    const cybt_platform_config_t *p_bt_platform_cfg = cybt_platform_get_config();

    if(p_bt_platform_cfg->controller_config.sleep_mode.sleep_mode_enabled)
    {
        sleep_mode = true;
    }

    return sleep_mode;
}

void platform_hal_gpio_set_config(bt_gpio_id_t gpio_id)
{
    switch(gpio_id)
    {
    case BT_POWER_PIN:
        gpio_config.pin_direction = BT_POWER_PIN_DIRECTION;
        gpio_config.pin_drive_mode = BT_POWER_PIN_DRIVE_MODE;
        gpio_config.pin_init_val = BT_POWER_PIN_INIT_VALUE;
        break;
    case HOST_WAKE_PIN:
        gpio_config.pin_direction = CYHAL_GPIO_DIR_INPUT;
        gpio_config.pin_drive_mode = CYHAL_GPIO_DRIVE_NONE;
        gpio_config.pin_init_val = false;
        break;
    case DEVICE_WAKE_PIN:
        gpio_config.pin_direction = CYHAL_GPIO_DIR_OUTPUT;
        gpio_config.pin_drive_mode = CYHAL_GPIO_DRIVE_STRONG;
        gpio_config.pin_init_val = false;
        break;
    case BT_UART_RTS:
        gpio_config.pin_direction = CYHAL_GPIO_DIR_OUTPUT;
        gpio_config.pin_drive_mode = CYHAL_GPIO_DRIVE_STRONG;
        gpio_config.pin_init_val = true;
        break;
    default:
        break;
    }
}

const char *get_gpio_pin_name_from_pin_id(bt_gpio_id_t gpio_id)
{
    switch ( (int)gpio_id )
    {
    CASE_RETURN_STR(BT_POWER_PIN)
    CASE_RETURN_STR(HOST_WAKE_PIN)
    CASE_RETURN_STR(DEVICE_WAKE_PIN)
    CASE_RETURN_STR(BT_UART_TX)
    CASE_RETURN_STR(BT_UART_RX)
    CASE_RETURN_STR(BT_UART_RTS)
    CASE_RETURN_STR(BT_UART_CTS)
    }
    return "UNKNOWN_GPIO";
}

cyhal_gpio_t platform_hal_gpio_get_pin_from_id(bt_gpio_id_t gpio_id)
{
    cyhal_gpio_t gpio_pin = NC;
    const cybt_platform_config_t *p_bt_platform_cfg = cybt_platform_get_config();

    switch(gpio_id)
    {
    case BT_POWER_PIN:
        gpio_pin = p_bt_platform_cfg->controller_config.bt_power_pin;
        break;
    case HOST_WAKE_PIN:
        gpio_pin = p_bt_platform_cfg->controller_config.sleep_mode.host_wakeup_pin;
        break;
    case DEVICE_WAKE_PIN:
        gpio_pin = p_bt_platform_cfg->controller_config.sleep_mode.device_wakeup_pin;
        break;
    case BT_UART_TX:
        gpio_pin = p_bt_platform_cfg->hci_config.hci.hci_uart.uart_tx_pin;
        break;
    case BT_UART_RX:
        gpio_pin = p_bt_platform_cfg->hci_config.hci.hci_uart.uart_rx_pin;
        break;
    case BT_UART_RTS:
        gpio_pin = p_bt_platform_cfg->hci_config.hci.hci_uart.uart_rts_pin;
        break;
    case BT_UART_CTS:
        gpio_pin = p_bt_platform_cfg->hci_config.hci.hci_uart.uart_cts_pin;
        break;
    }
    return gpio_pin;
}

/******************************************************************************
*                           HAL UART wrappers
******************************************************************************/
cybt_result_t platform_hal_uart_init_wrapper(void)
{
    cy_rslt_t result;
    cyhal_uart_cfg_t bt_uart_cfg = {0};
    const cybt_platform_config_t *p_bt_platform_cfg = cybt_platform_get_config();

    bt_uart_cfg.data_bits = p_bt_platform_cfg->hci_config.hci.hci_uart.data_bits;
    bt_uart_cfg.stop_bits = p_bt_platform_cfg->hci_config.hci.hci_uart.stop_bits;
    bt_uart_cfg.parity = p_bt_platform_cfg->hci_config.hci.hci_uart.parity;
    bt_uart_cfg.rx_buffer = NULL;
    bt_uart_cfg.rx_buffer_size = 0;

#if (CYHAL_API_VERSION >= 2)
    result = cyhal_uart_init(&hci_uart_cb.hal_obj,
                          p_bt_platform_cfg->hci_config.hci.hci_uart.uart_tx_pin,
                              p_bt_platform_cfg->hci_config.hci.hci_uart.uart_rx_pin,
                              p_bt_platform_cfg->hci_config.hci.hci_uart.uart_cts_pin,
                              p_bt_platform_cfg->hci_config.hci.hci_uart.uart_rts_pin,
                              NULL,
                              &bt_uart_cfg
                            );
#else
    result = cyhal_uart_init(&hci_uart_cb.hal_obj,
                              p_bt_platform_cfg->hci_config.hci.hci_uart.uart_tx_pin,
                              p_bt_platform_cfg->hci_config.hci.hci_uart.uart_rx_pin,
                              NULL,
                              &bt_uart_cfg
                            );
#endif

    if(CY_RSLT_SUCCESS != result)
    {
        HCIDRV_TRACE_ERROR("hci_open(): init error (0x%x)", result);
        return  CYBT_ERR_HCI_INIT_FAILED;
    }

    return CYBT_SUCCESS;
}

uint32_t platform_hal_get_feature_baud_wrapper(void)
{
    uint32_t baud_rate;
    const cybt_platform_config_t *p_bt_platform_cfg = cybt_platform_get_config();

    baud_rate = p_bt_platform_cfg->hci_config.hci.hci_uart.baud_rate_for_feature;

    return baud_rate;
}

uint32_t platform_hal_get_fw_download_baud_wrapper(void)
{
    uint32_t baud_rate;
    const cybt_platform_config_t *p_bt_platform_cfg = cybt_platform_get_config();

    baud_rate = p_bt_platform_cfg->hci_config.hci.hci_uart.baud_rate_for_fw_download;

    return baud_rate;
}


cybt_result_t platform_hal_uart_set_baud_wrapper(uint32_t baudrate)
{
    cy_rslt_t result;
    uint32_t actual_baud_rate;

    result = cyhal_uart_set_baud(&hci_uart_cb.hal_obj,
                                  baudrate,
                                  &actual_baud_rate
                                  );

    if(CY_RSLT_SUCCESS != result)
    {
        HCIDRV_TRACE_ERROR("hci_open(): Set baud rate failed (0x%x)",result);
        return  CYBT_ERR_HCI_SET_BAUDRATE_FAILED;
    }
    else
    {
        HCIDRV_TRACE_DEBUG("Baud rate set - Req = %d, actual = %d", baudrate, actual_baud_rate);
        return CYBT_SUCCESS;
    }
}

cybt_result_t platform_hal_uart_enable_flow_control_wrapper(void)
{
    cy_rslt_t result;
    const cybt_platform_config_t *p_bt_platform_cfg = cybt_platform_get_config();

    if(true == p_bt_platform_cfg->hci_config.hci.hci_uart.flow_control)
    {
#if (CYHAL_API_VERSION >= 2)
       result = cyhal_uart_enable_flow_control(&hci_uart_cb.hal_obj, true, true);
#else
       result = cyhal_uart_set_flow_control(&hci_uart_cb.hal_obj,
                                           p_bt_platform_cfg->hci_config.hci.hci_uart.uart_cts_pin,
                                           p_bt_platform_cfg->hci_config.hci.hci_uart.uart_rts_pin
                                          );
#endif

        if(CY_RSLT_SUCCESS != result)
        {
            HCIDRV_TRACE_ERROR("hci_open(): Set flow control failed (0x%x)",result);
            return  CYBT_ERR_HCI_SET_FLOW_CTRL_FAILED;
        }

        return CYBT_SUCCESS;
    }
    else
    {
        HCIDRV_TRACE_ERROR("hci_open(): Flow control not enabled - (0x%x)",CYBT_ERR_HCI_SET_FLOW_CTRL_FAILED);
        return CYBT_ERR_HCI_SET_FLOW_CTRL_FAILED;
    }
}

static void cybt_uart_rx_not_empty(void)
{
    cyhal_uart_enable_event(&hci_uart_cb.hal_obj,
                            CYHAL_UART_IRQ_RX_NOT_EMPTY,
                            CYHAL_ISR_PRIORITY_DEFAULT,
                            false
                           );

    cybt_send_msg_to_hci_rx_task(BT_IND_TO_HCI_DATA_READY_UNKNOWN, true);
}

static void cybt_uart_tx_done_irq(void)
{
    cy_rtos_set_semaphore(&hci_uart_cb.tx_complete, true);
}

static void cybt_uart_rx_done_irq(void)
{
    cy_rtos_set_semaphore(&hci_uart_cb.rx_complete, true);
}

static void cybt_uart_irq_handler(void *handler_arg, cyhal_uart_event_t event)
{
    switch(event)
    {
        case CYHAL_UART_IRQ_RX_NOT_EMPTY:
            cybt_uart_rx_not_empty();
            break;
        case CYHAL_UART_IRQ_TX_DONE:
            cybt_uart_tx_done_irq();
            break;
        case CYHAL_UART_IRQ_RX_DONE:
            cybt_uart_rx_done_irq();
            break;
        default:
            break;
    }
}

void platform_hal_uart_register_callback_wrapper(uint8_t hci_state)
{
    if(hci_state==0)
    {
         cyhal_uart_register_callback(&hci_uart_cb.hal_obj,
                                         NULL,
                                         NULL
                                        );
    }
    else
    {
        cyhal_uart_register_callback(&hci_uart_cb.hal_obj,
                                          cybt_uart_irq_handler,
                                          NULL
                                          );
    }

}

void platform_hal_uart_enable_event_wrapper(uint8_t hci_state)
{
    cyhal_uart_event_t enable_irq_event = (cyhal_uart_event_t)(CYHAL_UART_IRQ_RX_DONE
                                               | CYHAL_UART_IRQ_TX_DONE
                                               | CYHAL_UART_IRQ_RX_NOT_EMPTY
                                              );

    if(hci_state==0)
    {
        cyhal_uart_enable_event(&hci_uart_cb.hal_obj,
                                enable_irq_event,
                                CYHAL_ISR_PRIORITY_DEFAULT,
                                false
                               );
    }
    else
    {
        cyhal_uart_enable_event(&hci_uart_cb.hal_obj,
                                    enable_irq_event,
                                    CYHAL_ISR_PRIORITY_DEFAULT,
                                    true
                                   );
    }

}

void cybt_platform_hci_irq_rx_data_ind(bool enable)
{
    cyhal_uart_enable_event(&hci_uart_cb.hal_obj,
                            CYHAL_UART_IRQ_RX_NOT_EMPTY,
                            CYHAL_ISR_PRIORITY_DEFAULT,
                            enable
                           );
}

cybt_result_t platform_hal_uart_write_async_wrapper(void *tx, size_t length)
{
    cy_rslt_t result;

    result = cyhal_uart_write_async(&hci_uart_cb.hal_obj,
                                        (void *) tx,
                                        (size_t) length
                                       );
    if(CY_RSLT_SUCCESS != result)
    {
        HCIDRV_TRACE_ERROR("hci_write(): failure (0x%x)\n", result);
        return  CYBT_ERR_HCI_WRITE_FAILED;
    }

    return CYBT_SUCCESS;
}

cybt_result_t platform_hal_uart_read_async_wrapper(void *rx, size_t length)
{
    cy_rslt_t result;

    result = cyhal_uart_read_async(&hci_uart_cb.hal_obj,
                                           (void *) rx,
                                           (size_t) length
                                          );
    if(CY_RSLT_SUCCESS != result)
    {
        HCIDRV_TRACE_ERROR("hci_read_async(): failure code = 0x%x\n", result);
        return  CYBT_ERR_HCI_READ_FAILED;
    }

    return CYBT_SUCCESS;
}

cybt_result_t platform_hal_uart_read_abort_wrapper(void)
{
    cy_rslt_t result;

    result=cyhal_uart_read_abort(&hci_uart_cb.hal_obj);

    if(CY_RSLT_SUCCESS != result)
    {
        HCIDRV_TRACE_ERROR("hci_read_abort(): failure code = 0x%x\n", result);
        return  CYBT_ERR_HCI_ABORT_FAILED;
    }

    return CYBT_SUCCESS;
}

cybt_result_t platform_hal_uart_read_wrapper(void *rx, size_t *rx_length)
{
    cy_rslt_t result;

    result = cyhal_uart_read(&hci_uart_cb.hal_obj,
                                     (void *) rx,
                                     (size_t *)rx_length
                                    );
    if(CY_RSLT_SUCCESS != result)
    {
        HCIDRV_TRACE_ERROR("hci_read(): failure code = 0x%x\n", result);
        return  CYBT_ERR_HCI_READ_FAILED;
    }

    return CYBT_SUCCESS;
}

/******************************************************************************
*                           HAL Deep sleep wrappers
******************************************************************************/
void platform_hal_syspm_lock_deepsleep_wrapper(void)
{
    cyhal_syspm_lock_deepsleep();
}

void platform_hal_syspm_unlock_deepsleep_wrapper(void)
{
    cyhal_syspm_unlock_deepsleep();
}
#endif
