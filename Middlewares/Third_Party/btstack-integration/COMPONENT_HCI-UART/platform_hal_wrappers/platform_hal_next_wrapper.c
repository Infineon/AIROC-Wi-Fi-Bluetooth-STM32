/***************************************************************************//**
* \file platform_hal_wrapper.c
*
* \brief
* Implementation for HAL NEXT wrapper functions
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

#if !defined (CY_USING_HAL)
/******************************************************************************
 *                                Constants
 ******************************************************************************/
#define CYBSP_BT_UART_IRQ_SOURCE   (CYBSP_BT_UART_IRQ)
#define HOST_WAKE_INTERRUPT_PRIORITY  (7U)

/*****************************************************************************
 *                           Type Definitions
 *****************************************************************************/

/******************************************************************************
 *                           Variables Definitions
 ******************************************************************************/
hci_uart_cb_t   hci_uart_cb;
cy_stc_scb_uart_context_t  CYBSP_BT_UART_context;  /* BT UART context */
mtb_async_transfer_context_t CYBSP_BT_UART_async_tx_context; /* BT UART async context */

/* HAL NEXT GPIO Objects */
static mtb_hal_gpio_t                host_wakeup_pin_hal_obj;        /** GPIO HAL Object for HOST_WAKE pin */
static mtb_hal_gpio_t                device_wakeup_pin_hal_obj;      /** GPIO HAL Object for DEVICE_WAKE pin */
static mtb_hal_gpio_t                bt_power_pin_hal_obj;           /** GPIO HAL Object for BT_POWER_PIN */
static mtb_hal_gpio_t                bt_uart_cts_hal_obj;            /** GPIO HAL Object for UART CTS PIN*/
static mtb_hal_gpio_t                bt_uart_rts_hal_obj;            /** GPIO HAL Object for UART RTS PIN*/

/* Interrupt config structure */
cy_stc_sysint_t host_wake_intr_cfg =
{
	CYBSP_BT_HOST_WAKE_IRQ,       /* Interrupt source */
	HOST_WAKE_INTERRUPT_PRIORITY   /* Interrupt priority */
};
/******************************************************************************
 *                          Function Declarations
 ******************************************************************************/
mtb_hal_gpio_t *platform_hal_gpio_obj_from_id(bt_gpio_id_t gpio_id);
void platform_hal_host_wake_irq_handler(void *callback_arg, mtb_hal_gpio_event_t  event);

static void cybt_uart_rx_not_empty(void);
static void cybt_uart_tx_done_irq(void);
static void cybt_uart_rx_done_irq(void);
static void cybt_uart_irq_handler(void *handler_arg, mtb_hal_uart_event_t event);

void mw_uart_process_interrupt(mtb_hal_uart_t* uart);
void cybsb_bt_uart_interrupt_handler(void);

static void host_wake_interrupt_handler(void);
/******************************************************************************
 *                           Function Definitions
 ******************************************************************************/

/******************************************************************************
*                           HAL NEXT GPIO wrappers
******************************************************************************/
cybt_result_t platform_hal_gpio_init_wrapper(bt_gpio_id_t gpio_id)
{
    mtb_hal_gpio_t *gpio_obj = platform_hal_gpio_obj_from_id(gpio_id);

    platform_hal_gpio_free_wrapper(gpio_id);

    switch(gpio_id)
    {
    case BT_POWER_PIN:
        Cy_GPIO_Pin_FastInit(CYBSP_BT_POWER_PORT, CYBSP_BT_POWER_PIN, CY_GPIO_DM_PULLUP, 1, HSIOM_SEL_GPIO);
        mtb_hal_gpio_setup(gpio_obj, CYBSP_BT_POWER_PORT_NUM, CYBSP_BT_POWER_PIN);
        break;

    case HOST_WAKE_PIN:
        Cy_GPIO_Pin_FastInit(CYBSP_BT_HOST_WAKE_PORT, CYBSP_BT_HOST_WAKE_PIN, CY_GPIO_DM_HIGHZ, 0, HSIOM_SEL_GPIO);
        mtb_hal_gpio_setup(gpio_obj, CYBSP_BT_HOST_WAKE_PORT_NUM, CYBSP_BT_HOST_WAKE_PIN);

        Cy_SysInt_Init(&host_wake_intr_cfg, &host_wake_interrupt_handler); /* Initialize the interrupt and register interrupt callback */
        NVIC_EnableIRQ(host_wake_intr_cfg.intrSrc); /* Enable the interrupt in the NVIC */

        break;
    case DEVICE_WAKE_PIN:
        Cy_GPIO_Pin_FastInit(CYBSP_BT_DEVICE_WAKE_PORT, CYBSP_BT_DEVICE_WAKE_PIN, CY_GPIO_DM_STRONG, 0, HSIOM_SEL_GPIO);
        mtb_hal_gpio_setup(gpio_obj, CYBSP_BT_DEVICE_WAKE_PORT_NUM, CYBSP_BT_DEVICE_WAKE_PIN);
        break;

    case BT_UART_RTS:
        Cy_GPIO_Pin_FastInit(CYBSP_BT_UART_RTS_PORT, CYBSP_BT_UART_RTS_PIN, CY_GPIO_DM_STRONG, 1, HSIOM_SEL_GPIO);
        mtb_hal_gpio_setup(gpio_obj, CYBSP_BT_UART_RTS_PORT_NUM, CYBSP_BT_UART_RTS_PIN);
        break;

    case BT_UART_CTS:
        //CTS will be initialized during UART init process. No need to initialize as GPIO.
        mtb_hal_gpio_setup(gpio_obj, CYBSP_BT_UART_CTS_PORT_NUM, CYBSP_BT_UART_CTS_PIN);
        break;
    }

    return CYBT_SUCCESS;
}

bool platform_hal_gpio_read_wrapper(bt_gpio_id_t gpio_id)
{
    bool read_val = false;
    mtb_hal_gpio_t *gpio_obj = platform_hal_gpio_obj_from_id(gpio_id);

    read_val = mtb_hal_gpio_read(gpio_obj);

    return read_val;
}

cybt_result_t platform_hal_gpio_write_wrapper(bt_gpio_id_t gpio_id, bool value)
{
    mtb_hal_gpio_t *gpio_obj = platform_hal_gpio_obj_from_id(gpio_id);

    mtb_hal_gpio_write(gpio_obj, value);

    return CYBT_SUCCESS;
}

cybt_result_t platform_hal_gpio_free_wrapper(bt_gpio_id_t gpio_id)
{
    switch(gpio_id)
    {
    case BT_POWER_PIN:
        Cy_GPIO_Pin_FastInit(CYBSP_BT_POWER_PORT, CYBSP_BT_POWER_PIN, CY_GPIO_DM_ANALOG, 0, HSIOM_SEL_GPIO);
        break;
    case HOST_WAKE_PIN:
        Cy_GPIO_Pin_FastInit(CYBSP_BT_HOST_WAKE_PORT, CYBSP_BT_HOST_WAKE_PIN, CY_GPIO_DM_ANALOG, 0, HSIOM_SEL_GPIO);
        break;
    case DEVICE_WAKE_PIN:
        Cy_GPIO_Pin_FastInit(CYBSP_BT_DEVICE_WAKE_PORT, CYBSP_BT_DEVICE_WAKE_PIN, CY_GPIO_DM_ANALOG, 0, HSIOM_SEL_GPIO);
        break;
    case BT_UART_RTS:
        Cy_GPIO_Pin_FastInit(CYBSP_BT_UART_RTS_PORT, CYBSP_BT_UART_RTS_PIN, CY_GPIO_DM_ANALOG, 0, HSIOM_SEL_GPIO);
        break;
    case BT_UART_CTS:
        Cy_GPIO_Pin_FastInit(CYBSP_BT_UART_CTS_PORT, CYBSP_BT_UART_CTS_PIN, CY_GPIO_DM_ANALOG, 0, HSIOM_SEL_GPIO);
        break;
    }
    return CYBT_SUCCESS;
}

void platform_hal_host_wake_irq_handler(void *callback_arg, mtb_hal_gpio_event_t  event)
{
    switch(event)
    {
    case MTB_HAL_GPIO_IRQ_RISE:
        if(CYBT_WAKE_ACTIVE_HIGH == CYCFG_BT_HOST_WAKE_IRQ_EVENT)
        {
            cybt_platform_sleep_lock();
        }
        else
        {
            cybt_platform_sleep_unlock();
        }
        break;
    case MTB_HAL_GPIO_IRQ_FALL:
        if(CYBT_WAKE_ACTIVE_LOW == CYCFG_BT_HOST_WAKE_IRQ_EVENT)
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
    mtb_hal_gpio_t *gpio_obj = platform_hal_gpio_obj_from_id(gpio_id);

#if( configUSE_TICKLESS_IDLE != 0 )
    mtb_hal_gpio_register_callback( gpio_obj,
                                    platform_hal_host_wake_irq_handler,
                                    NULL);
#else
    mtb_hal_gpio_register_callback( gpio_obj,
                                    NULL,
                                    NULL);
#endif

    return CYBT_SUCCESS;
}

cybt_result_t platform_hal_gpio_enable_event_wrapper(bt_gpio_id_t gpio_id)
{
    mtb_hal_gpio_t *gpio_obj = platform_hal_gpio_obj_from_id(gpio_id);

#if( configUSE_TICKLESS_IDLE != 0 )
    mtb_hal_gpio_enable_event(  gpio_obj,
                                MTB_HAL_GPIO_IRQ_BOTH,
                                true);
#else
    mtb_hal_gpio_enable_event(  gpio_obj,
                                MTB_HAL_GPIO_IRQ_NONE,
                                true);
#endif

    return CYBT_SUCCESS;
}

uint32_t platform_hal_get_task_mem_pool_size_wrapper(void)
{
    return CYBSP_BT_PLATFORM_CFG_MEM_POOL_BYTES;
}

bool platform_hal_gpio_host_wake_polarity_wrapper(void)
{
    bool polarity = false;

    if(CYCFG_BT_HOST_WAKE_IRQ_EVENT)
    {
        polarity = true;
    }

    return polarity;
}

bool platform_hal_gpio_dev_wake_polarity_wrapper(void)
{
    bool polarity = false;

    if(CYCFG_BT_DEV_WAKE_POLARITY)
    {
        polarity = true;
    }

    return polarity;
}

bool platform_hal_get_sleep_mode_enabled_wrapper(void)
{
    bool sleep_mode = false;

    if(CYBSP_BT_PLATFORM_CFG_SLEEP_MODE_LP_ENABLED)
    {
        sleep_mode = true;
    }

    return sleep_mode;
}

static void host_wake_interrupt_handler(void)
{
	mtb_hal_gpio_process_interrupt(&host_wakeup_pin_hal_obj);
}

mtb_hal_gpio_t *platform_hal_gpio_obj_from_id(bt_gpio_id_t gpio_id)
{
    switch(gpio_id)
    {
    case BT_POWER_PIN:
        return &bt_power_pin_hal_obj;
        break;
    case HOST_WAKE_PIN:
        return &host_wakeup_pin_hal_obj;
        break;
    case DEVICE_WAKE_PIN:
        return &device_wakeup_pin_hal_obj;
        break;
    case BT_UART_RTS:
        return &bt_uart_rts_hal_obj;
        break;
    case BT_UART_CTS:
        return &bt_uart_cts_hal_obj;
        break;
    default:
        return NULL;
        break;
    }
}

/******************************************************************************
*                           HAL NEXT UART wrappers
******************************************************************************/
cybt_result_t platform_hal_uart_init_wrapper(void)
{
    cy_rslt_t result;

    platform_hal_gpio_init_wrapper(BT_UART_CTS);

    Cy_GPIO_Pin_Init(CYBSP_BT_UART_CTS_PORT, CYBSP_BT_UART_CTS_PIN, &CYBSP_BT_UART_CTS_config);
    Cy_GPIO_Pin_Init(CYBSP_BT_UART_RTS_PORT, CYBSP_BT_UART_RTS_PIN, &CYBSP_BT_UART_RTS_config);

    result = (cy_rslt_t)Cy_SCB_UART_Init(CYBSP_BT_UART_HW, &CYBSP_BT_UART_config, &CYBSP_BT_UART_context);
    if(CY_RSLT_SUCCESS != result)
    {
        HCIDRV_TRACE_ERROR("[%s] - SCB UART init error (0x%x)",__FUNCTION__, result);
        return  CYBT_ERR_HCI_INIT_FAILED;
    }

    Cy_SCB_UART_Enable(CYBSP_BT_UART_HW);

    cy_stc_sysint_t intr_cfg_1 = {.intrSrc = CYBSP_BT_UART_IRQ_SOURCE, .intrPriority = 7u};
    Cy_SysInt_Init(&intr_cfg_1, cybsb_bt_uart_interrupt_handler);
    NVIC_EnableIRQ(CYBSP_BT_UART_IRQ);

    result = mtb_hal_uart_setup(&hci_uart_cb.CYBSP_BT_UART_hal_obj, &CYBSP_BT_UART_hal_config, &CYBSP_BT_UART_context, NULL);
    if(CY_RSLT_SUCCESS != result)
    {
        HCIDRV_TRACE_ERROR("[%s] - UART setup error (0x%x)",__FUNCTION__, result);
        return  CYBT_ERR_HCI_INIT_FAILED;
    }

    result = mtb_hal_uart_config_async(&hci_uart_cb.CYBSP_BT_UART_hal_obj,&CYBSP_BT_UART_async_tx_context);
    if(CY_RSLT_SUCCESS != result)
    {
        HCIDRV_TRACE_ERROR("[%s] - UART asyn config error (0x%x)",__FUNCTION__, result);
        return  CYBT_ERR_HCI_INIT_FAILED;
    }

    return CYBT_SUCCESS;
}

uint32_t platform_hal_get_feature_baud_wrapper(void)
{
    uint32_t baud_rate;

    baud_rate = CYBSP_BT_PLATFORM_CFG_BAUD_FEATURE;

    return baud_rate;
}

uint32_t platform_hal_get_fw_download_baud_wrapper(void)
{
    uint32_t baud_rate;

    baud_rate = CYBSP_BT_PLATFORM_CFG_BAUD_DOWNLOAD;

    return baud_rate;
}

cybt_result_t platform_hal_uart_set_baud_wrapper(uint32_t baudrate)
{
    cy_rslt_t result;
    uint32_t actual_baud_rate;

    result = mtb_hal_uart_set_baud(&hci_uart_cb.CYBSP_BT_UART_hal_obj,
                                baudrate,
                                &actual_baud_rate
                               );

    if(CY_RSLT_SUCCESS != result)
    {
        HCIDRV_TRACE_ERROR("[%s] - Set baud rate failed (0x%x)", __FUNCTION__, result);
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

    if(CYBSP_BT_PLATFORM_CFG_FLOW_CONTROL_ENABLE)
    {
        result = mtb_hal_uart_enable_cts_flow_control(&hci_uart_cb.CYBSP_BT_UART_hal_obj, true);
        if(CY_RSLT_SUCCESS != result)
        {
            HCIDRV_TRACE_ERROR("[%s] - Set flow control failed (0x%x)",__FUNCTION__, result);
            return  CYBT_ERR_HCI_SET_FLOW_CTRL_FAILED;
        }

        return CYBT_SUCCESS;
    }
    else
    {
        HCIDRV_TRACE_ERROR("[%s] - Flow control not enabled - (0x%x)",__FUNCTION__, CYBT_ERR_HCI_SET_FLOW_CTRL_FAILED);
        return CYBT_ERR_HCI_SET_FLOW_CTRL_FAILED;
    }
}

static void cybt_uart_rx_not_empty(void)
{
    mtb_hal_uart_enable_event(&hci_uart_cb.CYBSP_BT_UART_hal_obj,
                            MTB_HAL_UART_IRQ_RX_NOT_EMPTY,
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

void cybt_uart_irq_handler(void *handler_arg, mtb_hal_uart_event_t event)
{
    switch(event)
    {
        case MTB_HAL_UART_IRQ_RX_NOT_EMPTY:
            cybt_uart_rx_not_empty();
            break;
        case MTB_HAL_UART_IRQ_TX_TRANSMIT_IN_FIFO:
            mtb_hal_uart_enable_event(  &hci_uart_cb.CYBSP_BT_UART_hal_obj,
                                        MTB_HAL_UART_IRQ_TX_EMPTY,
                                        true);
            break;
        case MTB_HAL_UART_IRQ_TX_EMPTY:
            mtb_hal_uart_enable_event(  &hci_uart_cb.CYBSP_BT_UART_hal_obj,
                                        MTB_HAL_UART_IRQ_TX_EMPTY,
                                        false);
            cybt_uart_tx_done_irq();
            break;
        case MTB_HAL_UART_IRQ_RX_DONE:
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
    mtb_hal_uart_register_callback(&hci_uart_cb.CYBSP_BT_UART_hal_obj,
                                        NULL,
                                        NULL);

    }
    else
    {
    mtb_hal_uart_register_callback(&hci_uart_cb.CYBSP_BT_UART_hal_obj,
                                       cybt_uart_irq_handler,
                                       NULL);

    }
}

void platform_hal_uart_enable_event_wrapper(uint8_t hci_state)
{
    mtb_hal_uart_event_t enable_irq_event = (mtb_hal_uart_event_t)(MTB_HAL_UART_IRQ_RX_DONE
                                                   | MTB_HAL_UART_IRQ_TX_TRANSMIT_IN_FIFO
                                                   | MTB_HAL_UART_IRQ_RX_NOT_EMPTY
                                                   );
    if(hci_state==0)
    {
    mtb_hal_uart_enable_event(  &hci_uart_cb.CYBSP_BT_UART_hal_obj,
                                    enable_irq_event,
                                    false);
    }
    else
    {

    mtb_hal_uart_enable_event(  &hci_uart_cb.CYBSP_BT_UART_hal_obj,
                                    enable_irq_event,
                                    true);


    }
}

void cybt_platform_hci_irq_rx_data_ind(bool enable)
{
    mtb_hal_uart_enable_event(&hci_uart_cb.CYBSP_BT_UART_hal_obj,
                                MTB_HAL_UART_IRQ_RX_NOT_EMPTY,
                                enable
                               );
}

cybt_result_t platform_hal_uart_write_async_wrapper(void *tx, size_t length)
{
    cy_rslt_t result;

    result = mtb_hal_uart_write_async(&hci_uart_cb.CYBSP_BT_UART_hal_obj,
                                        (void *) tx,
                                        (size_t) length
                                       );
    if(CY_RSLT_SUCCESS != result)
    {
        HCIDRV_TRACE_ERROR("hci_write_async(): failure code = (0x%x)\n", result);
        return  CYBT_ERR_HCI_WRITE_FAILED;
    }

    return CYBT_SUCCESS;
}

cybt_result_t platform_hal_uart_read_async_wrapper(void *rx, size_t length)
{
    cy_rslt_t result;

    result = mtb_hal_uart_read_async(&hci_uart_cb.CYBSP_BT_UART_hal_obj,
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

    result=mtb_hal_uart_read_abort(&hci_uart_cb.CYBSP_BT_UART_hal_obj);

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

    result = mtb_hal_uart_read(&hci_uart_cb.CYBSP_BT_UART_hal_obj,
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

void mw_uart_process_interrupt(mtb_hal_uart_t* uart)
{
    mtb_hal_uart_process_interrupt(uart);
}

void cybsb_bt_uart_interrupt_handler(void)
{
    mw_uart_process_interrupt(&hci_uart_cb.CYBSP_BT_UART_hal_obj);
}

/******************************************************************************
*                           HAL NEXT Deep sleep wrappers
******************************************************************************/
void platform_hal_syspm_lock_deepsleep_wrapper(void)
{
    mtb_hal_syspm_lock_deepsleep();
}

void platform_hal_syspm_unlock_deepsleep_wrapper(void)
{
    mtb_hal_syspm_unlock_deepsleep();
}

#endif //CY_USING_HAL
