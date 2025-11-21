/*******************************************************************************
* \file platform_hal_wrapper.h
*
* \brief
* HAL wrapper functions declarations
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

#ifndef CYBT_PLATFORM_HAL_WRAPPER_H
#define CYBT_PLATFORM_HAL_WRAPPER_H

#include <stdint.h>
#include <stdbool.h>

#if defined (CY_USING_HAL)
#include "cyhal_gpio.h"
#include "cyhal_uart.h"
#include "cyhal_syspm.h"
#else
#include "cycfg.h"
#include "cybsp_bt_config.h"
#include "mtb_hal.h"
#endif

#include "wiced_data_types.h"
#include "cybt_result.h"
#include "cybt_platform_config.h"
#include "cybt_platform_interface.h"
#include "cybt_platform_util.h"
#include "cybt_platform_task.h"
#include "cybt_platform_trace.h"

/******************************************************************************
 *                                Constants
 ******************************************************************************/
#define CYBT_RSLT_CHECK(x)    if(x) {return x;}
#define CASE_RETURN_STR(enum_val)          case enum_val: return #enum_val;

#if defined (CY_USING_HAL)
#ifndef XMC7200D_E272K8384
#define BT_POWER_PIN_DIRECTION  CYHAL_GPIO_DIR_OUTPUT
#define BT_POWER_PIN_DRIVE_MODE CYHAL_GPIO_DRIVE_PULLUP
#define BT_POWER_PIN_INIT_VALUE true
#else
#define BT_POWER_PIN_DIRECTION  CYHAL_GPIO_DIR_OUTPUT
#define BT_POWER_PIN_DRIVE_MODE CYHAL_GPIO_DRIVE_STRONG
#define BT_POWER_PIN_INIT_VALUE false
#endif
#endif

#ifndef CYHAL_HWMGR_RSLT_ERR_INVALID
#define CYHAL_HWMGR_RSLT_ERR_INVALID    CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR, CYHAL_RSLT_MODULE_HWMGR, 0)
#endif // !CYHAL_HWMGR_RSLT_ERR_INVALID
#ifndef CYHAL_HWMGR_RSLT_ERR_INUSE
#define CYHAL_HWMGR_RSLT_ERR_INUSE      CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR, CYHAL_RSLT_MODULE_HWMGR, 1)
#endif // !CYHAL_HWMGR_RSLT_ERR_INUSE

/*****************************************************************************
 *                           Type Definitions
 *****************************************************************************/
typedef struct
{
    bool            inited;
    #if defined (CY_USING_HAL)
    cyhal_uart_t    hal_obj;
    #else
    mtb_hal_uart_t  CYBSP_BT_UART_hal_obj; /* BT UART HAL object */
    #endif
    cy_semaphore_t  tx_complete;
    cy_semaphore_t  rx_complete;
    cy_mutex_t      tx_atomic;
    cy_mutex_t      rx_atomic;
} hci_uart_cb_t;

#if defined (CY_USING_HAL)
typedef struct
{
    cyhal_gpio_direction_t pin_direction;
    cyhal_gpio_drive_mode_t pin_drive_mode;
    bool pin_init_val;
}platform_hal_gpio_config_t;
#else
enum
{
    GPIO_PORT,
    GPIO_PIN
};
typedef uint8_t bt_gpio_type;
#endif

enum
{
    BT_POWER_PIN,
    HOST_WAKE_PIN,
    DEVICE_WAKE_PIN,
    BT_UART_TX,
    BT_UART_RX,
    BT_UART_RTS,
    BT_UART_CTS
};
typedef uint8_t bt_gpio_id_t;

#ifdef __cplusplus
extern "C"
{
#endif

/*****************************************************************************
 *                           Function Declarations
 ****************************************************************************/
/*HAL GPIO wrappers*/
cybt_result_t platform_hal_gpio_init_wrapper(bt_gpio_id_t gpio_id);
bool platform_hal_gpio_read_wrapper(bt_gpio_id_t gpio_id);
cybt_result_t platform_hal_gpio_write_wrapper(bt_gpio_id_t gpio_id, bool value);
cybt_result_t platform_hal_gpio_free_wrapper(bt_gpio_id_t gpio_id);
cybt_result_t platform_hal_gpio_register_callback_wrapper(bt_gpio_id_t gpio_id);
cybt_result_t platform_hal_gpio_enable_event_wrapper(bt_gpio_id_t gpio_id);
bool platform_hal_gpio_dev_wake_polarity_wrapper(void);
bool platform_hal_gpio_host_wake_polarity_wrapper(void);
void platform_hal_gpio_set_config(bt_gpio_id_t gpio_id);
uint32_t platform_hal_get_task_mem_pool_size_wrapper(void);

bool platform_hal_get_sleep_mode_enabled_wrapper(void);
uint32_t platform_hal_get_fw_download_baud_wrapper(void);
uint32_t platform_hal_get_feature_baud_wrapper(void);

/*HAL UART wrappers*/
cybt_result_t platform_hal_uart_init_wrapper(void);
cybt_result_t platform_hal_uart_set_baud_wrapper(uint32_t baudrate);
cybt_result_t platform_hal_uart_enable_flow_control_wrapper();
void platform_hal_uart_register_callback_wrapper(uint8_t hci_state);
void platform_hal_uart_enable_event_wrapper(uint8_t hci_state);
cybt_result_t platform_hal_uart_write_async_wrapper(void *tx, size_t length);
cybt_result_t platform_hal_uart_write_wrapper(void *tx, size_t length);
cybt_result_t platform_hal_uart_read_async_wrapper(void *rx, size_t length);
cybt_result_t platform_hal_uart_read_abort_wrapper();
cybt_result_t platform_hal_uart_read_wrapper(void *rx, size_t *rx_length);


/*HAL Deep sleep wrappers*/
void platform_hal_syspm_lock_deepsleep_wrapper(void);
void platform_hal_syspm_unlock_deepsleep_wrapper(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

/**@} */
#endif //CYBT_PLATFORM_HAL_WRAPPER_H
