/*
 * Copyright 2020, Cypress Semiconductor Corporation or a subsidiary of
 * Cypress Semiconductor Corporation. All Rights Reserved.
 *
 * This software, associated documentation and materials ("Software"),
 * is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
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


/** @file
 *
 */
#include "cycfg.h"
#include "cyhal_uart.h"
#include "cyhal_gpio.h"

#include "bt_mfg_test.h"
#include "bt_bus.h"

/*******************************************************
 *                      Macros
 ******************************************************/
/* Macro for waiting until bus is ready */
#define BT_BUS_WAIT_UNTIL_READY() \
do \
{ \
    while ( bt_bus_is_ready( ) == false ) \
    { \
        cy_rtos_delay_milliseconds( 10 ); \
    } \
} while ( 0 )

/* Verify if WICED Platform API returns success.
 * Otherwise, returns the error code immediately.
 * Assert in DEBUG build.
 */
#define RETURN_IF_FAILURE(x) \
do \
{ \
    cy_rslt_t _result = (x); \
    if ( _result != CY_RSLT_SUCCESS ) \
    { \
        return false; \
    } \
} while( 0 )

/* Macro for checking of bus is initialised */
#define IS_BUS_INITIALISED() \
do \
{ \
    if ( bus_initialised == false ) \
    { \
        return false; \
    } \
}while ( 0 )

/*******************************************************
 *                    Constants
 ******************************************************/
#define HCI_UART_DEFAULT_BAUDRATE (115200)
#define HCI_UART_3M_BAUDATE (3000000)

#define HCI_SEMAPHORE_MAX_COUNT  (1)
#define HCI_SEMAPHORE_INIT_COUNT (0)

cy_semaphore_t tx_complete;
cy_semaphore_t rx_complete;
cy_mutex_t     tx_atomic;
cy_mutex_t     rx_atomic;

/*******************************************************
 *                   Enumerations
 ******************************************************/

/*******************************************************
 *                 Type Definitions
 ******************************************************/

/*******************************************************
 *                    Structures
 ******************************************************/
/**
 *  The HCI UART configuration, including:
 *  * hardware pin assignment
 *  * baud rate
 *  * data format
 *  * flow control support
 */
typedef struct
{
    cyhal_gpio_t        uart_tx_pin;    /**< Uart TXD pin */
    cyhal_gpio_t        uart_rx_pin;    /**< Uart RXD pin */
    cyhal_gpio_t        uart_rts_pin;   /**< Uart RTS pin */
    cyhal_gpio_t        uart_cts_pin;   /**< Uart CTS pin */
    uint32_t            baud_rate;      /**< Uart baud rate */
    uint32_t            data_bits;      /**< the size of data bits */
    uint32_t            stop_bits;      /**< the size of stop bits */
    cyhal_uart_parity_t parity;         /**< parity check control */
    bool                flow_control;   /**< flow control status */
} hci_uart_config_t;

/*******************************************************
 *               Static Function Declarations
 ******************************************************/
static bool bluetooth_uart_init(void);


/*******************************************************
 *               Variable Definitions
 ******************************************************/
static volatile bool bus_initialised = false;

static const hci_uart_config_t uart_config =
{
    .uart_tx_pin  = CYBSP_BT_UART_TX,
    .uart_rx_pin  = CYBSP_BT_UART_RX,
    .uart_rts_pin = CYBSP_BT_UART_RTS,
    .uart_cts_pin = CYBSP_BT_UART_CTS,

    .baud_rate    = 115200,

    .data_bits    = 8,
    .stop_bits    = 1,
    .parity       = CYHAL_UART_PARITY_NONE,
    .flow_control = true,
};

cyhal_uart_t uart_hal_obj;

/******************************************************
*               Function Definitions
******************************************************/
bool bt_bus_init(void)
{
    bool init_result = true;
    if (bus_initialised == false)
    {
        init_result = bluetooth_uart_init();
        if (init_result == true)
        {
            bus_initialised = true;
        }
    }

    return init_result;
}


/***************************************************************************************************
 * bt_bus_transmit
 **************************************************************************************************/
bool bt_bus_transmit(const uint8_t* data_out, uint32_t size)
{
    cy_rslt_t result;
    bool      return_status = false;

    IS_BUS_INITIALISED();

    BT_BUS_WAIT_UNTIL_READY();

    result = cy_rtos_get_mutex(&tx_atomic, CY_RTOS_NEVER_TIMEOUT);
    if (CY_RSLT_SUCCESS != result)
    {
        APP_TRACE_DEBUG("hci_write(): Get mutex error (0x%x)\n", result);
        return false;
    }

    result = cyhal_uart_write_async(&uart_hal_obj, (void*)data_out, size);

    if (CY_RSLT_SUCCESS == result)
    {
        result = cy_rtos_get_semaphore(&tx_complete,
                                       CY_RTOS_NEVER_TIMEOUT,
                                       false
                                       );
    }

    if (CY_RSLT_SUCCESS == result)
    {
        return_status = true;
    }

    cy_rtos_set_mutex(&tx_atomic);

    return return_status;
}


/***************************************************************************************************
 * bt_bus_receive
 **************************************************************************************************/
bool bt_bus_receive(uint8_t* data_in, uint32_t size, uint32_t timeout_ms)
{
    cy_rslt_t result;
    bool      return_status = false;

    IS_BUS_INITIALISED();

    result = cy_rtos_get_mutex(&rx_atomic, timeout_ms);
    if (CY_RSLT_SUCCESS != result)
    {
        APP_TRACE_DEBUG("hci_read(): Get mutex error (0x%x)\n", result);
        return false;
    }

    result = cyhal_uart_read_async(&uart_hal_obj, (uint8_t*)data_in, size);

    if (CY_RSLT_SUCCESS == result)
    {
        result = cy_rtos_get_semaphore(&rx_complete,
                                       timeout_ms,
                                       false
                                       );
    }
    else
    {
        cy_rtos_set_mutex(&rx_atomic);
        return false;
    }

    if (CY_RSLT_SUCCESS == result)
    {
        return_status = true;
    }
    else
    {
        cyhal_uart_read_abort(&uart_hal_obj);
    }

    cy_rtos_set_mutex(&rx_atomic);
    return return_status;
}


/***************************************************************************************************
 * bt_bus_is_ready
 **************************************************************************************************/
bool bt_bus_is_ready(void)
{
    return (bus_initialised == false) ?
           false : ((cyhal_gpio_read(uart_config.uart_cts_pin) == true) ? false : true);
}


/***************************************************************************************************
 * cybt_uart_rx_not_empty
 **************************************************************************************************/
static void cybt_uart_rx_not_empty(void)
{
    cyhal_uart_enable_event(&uart_hal_obj,
                            CYHAL_UART_IRQ_RX_NOT_EMPTY,
                            CYHAL_ISR_PRIORITY_DEFAULT,
                            false
                            );
}


/***************************************************************************************************
 * cybt_uart_tx_done_irq
 **************************************************************************************************/
static void cybt_uart_tx_done_irq(void)
{
    cy_rtos_set_semaphore(&tx_complete, true);
}


/***************************************************************************************************
 * cybt_uart_rx_done_irq
 **************************************************************************************************/
static void cybt_uart_rx_done_irq(void)
{
    cy_rtos_set_semaphore(&rx_complete, true);
}


/***************************************************************************************************
 * cybt_uart_irq_handler
 **************************************************************************************************/
static void cybt_uart_irq_handler(void* handler_arg, cyhal_uart_event_t event)
{
    switch (event)
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


/***************************************************************************************************
 * bluetooth_uart_init
 **************************************************************************************************/
bool bluetooth_uart_init(void)
{
    cyhal_uart_event_t enable_irq_event = (cyhal_uart_event_t)(CYHAL_UART_IRQ_RX_DONE
                                                               | CYHAL_UART_IRQ_TX_DONE
                                                               | CYHAL_UART_IRQ_RX_NOT_EMPTY
                                                               );
    uint32_t         actual_baud_rate;
    cy_rslt_t        result;
    cyhal_uart_cfg_t bt_uart_cfg = { 0 };

    cy_rtos_init_semaphore(&tx_complete,
                           HCI_SEMAPHORE_MAX_COUNT,
                           HCI_SEMAPHORE_INIT_COUNT
                           );
    cy_rtos_init_semaphore(&rx_complete,
                           HCI_SEMAPHORE_MAX_COUNT,
                           HCI_SEMAPHORE_INIT_COUNT
                           );

    cy_rtos_init_mutex(&tx_atomic);
    cy_rtos_init_mutex(&rx_atomic);

    bt_uart_cfg.data_bits      = uart_config.data_bits;
    bt_uart_cfg.stop_bits      = uart_config.stop_bits;
    bt_uart_cfg.parity         = uart_config.parity;
    bt_uart_cfg.rx_buffer      = NULL;
    bt_uart_cfg.rx_buffer_size = 0;

    result = cyhal_uart_init(&uart_hal_obj,
                             uart_config.uart_tx_pin,
                             uart_config.uart_rx_pin,
                             uart_config.uart_cts_pin,
                             uart_config.uart_rts_pin,
                             NULL,
                             &bt_uart_cfg
                             );
    if (CY_RSLT_SUCCESS != result)
    {
        APP_TRACE_DEBUG("hci_uart_init(): init error (0x%x)\n", result);
    }

    result = cyhal_uart_set_baud(&uart_hal_obj,
                                 HCI_UART_DEFAULT_BAUDRATE,
                                 &actual_baud_rate
                                 );
    if (CY_RSLT_SUCCESS != result)
    {
        APP_TRACE_DEBUG("hci_uart_init(): Set baud rate failed (0x%x)\n",
                        result
                        );
        return false;
    }
    APP_TRACE_DEBUG("hci_uart_init(): act baud rate  = %d\n", actual_baud_rate);

    if (true == uart_config.flow_control)
    {
        result= cyhal_uart_enable_flow_control(&uart_hal_obj, true, true);
        if (CY_RSLT_SUCCESS != result)
        {
            APP_TRACE_DEBUG("hci_uart_init(): Set flow control failed (0x%x)\n",
                            result
                            );
            return false;
        }
    }

    cyhal_uart_register_callback(&uart_hal_obj,
                                 cybt_uart_irq_handler,
                                 NULL
                                 );

    cyhal_uart_enable_event(&uart_hal_obj,
                            enable_irq_event,
                            CYHAL_ISR_PRIORITY_DEFAULT,
                            true
                            );

    APP_TRACE_DEBUG("hci_open(): Wait CTS low\n");
    while (true == cyhal_gpio_read(uart_config.uart_cts_pin))
    {
        cy_rtos_delay_milliseconds(10);
    }

    return true;
}
