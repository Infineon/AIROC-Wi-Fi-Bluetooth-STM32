/*******************************************************************************
* File Name: cyhal_uart.c
*
* Description:
* Provides a high level interface for interacting with the STM32 UART. This is
* a wrapper around the lower level PDL API.
*
*******************************************************************************
* \copyright
* Copyright 2021 Cypress Semiconductor Corporation
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
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
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/

#include <stdlib.h>
#include <string.h>
#include "cyhal_uart.h"
#include "stm32_cyhal_uart_ex.h"
#include "cyhal_syspm.h"
#include "cyhal_hw_types.h"

#include "stm32_cyhal_common.h"

#if defined(HAL_UART_MODULE_ENABLED)

#if defined(__cplusplus)
extern "C"
{
#endif

/*******************************************************************************
*      Private macros
*******************************************************************************/

#define CYHAL_UART_OVERSAMPLE                 (12UL)
#define CYHAL_UART_OVERSAMPLE_MIN             (8UL)
#define CYHAL_UART_OVERSAMPLE_MAX             (16UL)
#define CYHAL_UART_MAX_INSTANCES              (3)

#define CYHAL_REPEAT_OPERATION                (10)

/*******************************************************************************
*      Private types
*******************************************************************************/

typedef struct stm32_cyhal_uart_structs_t
{
    UART_HandleTypeDef* huart;
    cyhal_gpio_t        associate_pin;
    cyhal_uart_t*       occupied_obj;
} stm32_cyhal_uart_structs_t;


/*******************************************************************************
*      Private variables
*******************************************************************************/

static stm32_cyhal_uart_structs_t _cyhal_uart_structs[CYHAL_UART_MAX_INSTANCES] =
    { 0 }; /* init with zeros */


/*******************************************************************************
*      Private functions
*******************************************************************************/

static uint32_t _stm32_cyhal_uart_convert_parity(cyhal_uart_parity_t parity);
static uint32_t _stm32_cyhal_uart_convert_stopbits(uint8_t stopbits);
static uint32_t _stm32_cyhal_uart_convert_wordlength(uint8_t wordlength);
void _stm32_cyhal_uart_enable_irq(USART_TypeDef* instance, uint32_t priority, bool enable);


static UART_HandleTypeDef* _stm32_cyhal_uart_alloc_hw(cyhal_uart_t* obj, cyhal_gpio_t tx,
                                                      cyhal_gpio_t rx);
static void _stm32_cyhal_uart_free_hw(cyhal_uart_t* obj);
cyhal_uart_t* _stm32_cyhal_uart_get_obj(UART_HandleTypeDef* huart);
void cyhal_uart_rx_not_empty_irq_handler(UART_HandleTypeDef* huart);


//--------------------------------------------------------------------------------------------------
// cyhal_uart_init
//--------------------------------------------------------------------------------------------------
cy_rslt_t cyhal_uart_init(cyhal_uart_t* obj, cyhal_gpio_t tx, cyhal_gpio_t rx,
                          const cyhal_clock_t* clk, const cyhal_uart_cfg_t* cfg)
{
    cy_rslt_t status = CY_RSLT_SUCCESS;

    /* Check the parameters */
    assert_param(NULL != obj);
    assert_param(NULL != cfg);
    assert_param(!((NC == tx) && (NC == rx)));

    /* Clean uart object */
    memset(obj, 0, sizeof(cyhal_uart_t));

    /* Try to allocate uart instance from _cyhal_uart_structs table */
    if ((obj->huart = _stm32_cyhal_uart_alloc_hw(obj, tx, rx)) != NULL)
    {
        /* Copy current configuration */
        UART_InitTypeDef current_init = obj->huart->Init;

        /* Set common parameters from cyhal_uart_cfg_t */
        obj->huart->Init.WordLength = _stm32_cyhal_uart_convert_wordlength(cfg->data_bits);
        obj->huart->Init.StopBits   = _stm32_cyhal_uart_convert_stopbits(cfg->stop_bits);
        obj->huart->Init.Parity     = _stm32_cyhal_uart_convert_parity(cfg->parity);

        /* NOTE: skip uart initialize in following cases
         *   -- uart is initialized (state != HAL_UART_STATE_RESET) and
         *   -- init parameters is the same as in input configuration (cfg)
         */
        if ((obj->huart->gState != HAL_UART_STATE_RESET) &&
            (memcmp(&current_init, &obj->huart->Init, sizeof(UART_InitTypeDef))))
        {
            /* Initialize the UART mode according to the specified parameters */
            status = HAL_UART_Init(obj->huart) ? CYHAL_UART_RSLT_ERR_HAL_ERROR :
                     CY_RSLT_SUCCESS;
        }
    }

    if (status != CY_RSLT_SUCCESS)
    {
        cyhal_uart_free(obj);
    }
    return status;
}


//--------------------------------------------------------------------------------------------------
// cyhal_uart_free
//--------------------------------------------------------------------------------------------------
void cyhal_uart_free(cyhal_uart_t* obj)
{
    /* Check the parameters */
    assert_param(NULL != obj);

    /* De-initialize the UART peripheral */
    HAL_UART_DeInit(obj->huart);

    /* Free from uart resource table */
    _stm32_cyhal_uart_free_hw(obj);
}


//--------------------------------------------------------------------------------------------------
// cyhal_uart_set_baud
//--------------------------------------------------------------------------------------------------
cy_rslt_t cyhal_uart_set_baud(cyhal_uart_t* obj, uint32_t baudrate, uint32_t* actualbaud)
{
    /* Check the parameters */
    assert_param(NULL != obj);
    *actualbaud = baudrate;
    if (baudrate == obj->huart->Init.BaudRate)
    {
        return CY_RSLT_SUCCESS;
    }

    /* Set new baud rate value */
    obj->huart->Init.BaudRate = baudrate;

    return (UART_SetConfig(obj->huart) ? CYHAL_UART_RSLT_ERR_HAL_ERROR : CY_RSLT_SUCCESS);
}


//--------------------------------------------------------------------------------------------------
// cyhal_uart_configure
//--------------------------------------------------------------------------------------------------
cy_rslt_t cyhal_uart_configure(cyhal_uart_t* obj, const cyhal_uart_cfg_t* cfg)
{
    /* Check the parameters */
    assert_param(NULL != obj);
    assert_param(NULL != cfg);

    obj->huart->Init.Parity     = _stm32_cyhal_uart_convert_parity(cfg->parity);
    obj->huart->Init.StopBits   = _stm32_cyhal_uart_convert_stopbits(cfg->stop_bits);
    obj->huart->Init.WordLength = _stm32_cyhal_uart_convert_wordlength(cfg->data_bits);

    return (UART_SetConfig(obj->huart) ? CYHAL_UART_RSLT_ERR_HAL_ERROR : CY_RSLT_SUCCESS);
}


//--------------------------------------------------------------------------------------------------
// cyhal_uart_set_flow_control
//--------------------------------------------------------------------------------------------------
cy_rslt_t cyhal_uart_set_flow_control(cyhal_uart_t* obj, cyhal_gpio_t cts, cyhal_gpio_t rts)
{
    /* Check the parameters */
    assert_param(NULL != obj);

    if ((cts != NC) && (rts != NC))
    {
        obj->huart->Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
    }
    else if ((cts == NC) && (rts != NC))
    {
        obj->huart->Init.HwFlowCtl = UART_HWCONTROL_RTS;
    }
    else if ((cts != NC) && (rts == NC))
    {
        obj->huart->Init.HwFlowCtl = UART_HWCONTROL_CTS;
    }
    else
    {
        obj->huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    }

    return (UART_SetConfig(obj->huart) ? CYHAL_UART_RSLT_ERR_HAL_ERROR : CY_RSLT_SUCCESS);
}


//--------------------------------------------------------------------------------------------------
// cyhal_uart_write
//--------------------------------------------------------------------------------------------------
cy_rslt_t cyhal_uart_write(cyhal_uart_t* obj, void* tx, size_t* tx_length)
{
    HAL_StatusTypeDef status;
    status = HAL_UART_Transmit(obj->huart, tx, *tx_length, 0xFFFF);

    if (status == HAL_BUSY)
    {
        return CYHAL_SYSPM_RSLT_ERR_PM_PENDING;
    }
    else
    {
        return (status ? CYHAL_UART_RSLT_ERR_HAL_ERROR : CY_RSLT_SUCCESS);
    }
}


//--------------------------------------------------------------------------------------------------
// cyhal_uart_read
//--------------------------------------------------------------------------------------------------
cy_rslt_t cyhal_uart_read(cyhal_uart_t* obj, void* rx, size_t* rx_length)
{
    HAL_StatusTypeDef status;
    status = HAL_UART_Receive(obj->huart, rx, *rx_length, 0u);

    if (status == HAL_TIMEOUT)
    {
        *rx_length = *rx_length - obj->huart->RxXferCount;
    }

    if (status == HAL_BUSY)
    {
        return CYHAL_SYSPM_RSLT_ERR_PM_PENDING;
    }
    else
    {
        return (((status == HAL_TIMEOUT) || (status == HAL_OK)) ?
                CY_RSLT_SUCCESS : CYHAL_UART_RSLT_ERR_HAL_ERROR);
    }
}


//--------------------------------------------------------------------------------------------------
// cyhal_uart_write_abort
//--------------------------------------------------------------------------------------------------
cy_rslt_t cyhal_uart_write_abort(cyhal_uart_t* obj)
{
    /* Check the parameters */
    assert_param(NULL != obj);

    return (HAL_UART_AbortTransmit(obj->huart) ? CYHAL_UART_RSLT_ERR_HAL_ERROR : CY_RSLT_SUCCESS);
}


//--------------------------------------------------------------------------------------------------
// cyhal_uart_read_abort
//--------------------------------------------------------------------------------------------------
cy_rslt_t cyhal_uart_read_abort(cyhal_uart_t* obj)
{
    /* Check the parameters */
    assert_param(NULL != obj);

    return (HAL_UART_AbortReceive(obj->huart) ? CYHAL_UART_RSLT_ERR_HAL_ERROR : CY_RSLT_SUCCESS);
}


//--------------------------------------------------------------------------------------------------
// cyhal_uart_write_async
//--------------------------------------------------------------------------------------------------
cy_rslt_t cyhal_uart_write_async(cyhal_uart_t* obj, void* tx, size_t length)
{
    HAL_StatusTypeDef status;
    uint32_t repeat_operation = CYHAL_REPEAT_OPERATION;

    /* Check the parameters */
    assert_param(NULL != obj);
    assert_param(NULL != tx);
    assert_param(0u != length);

    do
    {
        if (obj->huart->hdmatx == NULL)
        {
            status = HAL_UART_Transmit_IT(obj->huart, tx, length);
        }
        else
        {
            status = HAL_UART_Transmit_DMA(obj->huart, tx, length);
        }
    } while ((status != CY_RSLT_SUCCESS) && (repeat_operation--));

    if (status == HAL_BUSY)
    {
        return CYHAL_SYSPM_RSLT_ERR_PM_PENDING;
    }
    else
    {
        return (status ? CYHAL_UART_RSLT_ERR_HAL_ERROR : CY_RSLT_SUCCESS);
    }
}


//--------------------------------------------------------------------------------------------------
// cyhal_uart_read_async
//--------------------------------------------------------------------------------------------------
cy_rslt_t cyhal_uart_read_async(cyhal_uart_t* obj, void* rx, size_t length)
{
    HAL_StatusTypeDef status;
    uint32_t repeat_operation = CYHAL_REPEAT_OPERATION;

    /* Check the parameters */
    assert_param(NULL != obj);
    assert_param(NULL != rx);
    assert_param(0u != length);

    do
    {
        if (obj->huart->hdmarx == NULL)
        {
            status = HAL_UART_Receive_IT(obj->huart, rx, length);
        }
        else
        {
            status = HAL_UART_Receive_DMA(obj->huart, rx, length);
        }
    } while ((status != CY_RSLT_SUCCESS) && (repeat_operation--));

    /* NOTE: keep context.rxBufIdx to have BWC with bluetooth-free (PSoC implementation) */
    obj->context.rxBufIdx = obj->huart->NbRxDataToProcess;

    if (status == HAL_BUSY)
    {
        return CYHAL_SYSPM_RSLT_ERR_PM_PENDING;
    }
    else
    {
        return (status ? CYHAL_UART_RSLT_ERR_HAL_ERROR : CY_RSLT_SUCCESS);
    }
}


//--------------------------------------------------------------------------------------------------
// cyhal_uart_is_tx_active
//--------------------------------------------------------------------------------------------------
bool cyhal_uart_is_tx_active(cyhal_uart_t* obj)
{
    HAL_UART_StateTypeDef state;

    state = HAL_UART_GetState(obj->huart);

    /* NOTE: it will also return TRUE in case of both - TX AND RX are busy simultaneously */
    return ((state & HAL_UART_STATE_BUSY_TX) == HAL_UART_STATE_BUSY_TX);
}


//--------------------------------------------------------------------------------------------------
// cyhal_uart_is_rx_active
//--------------------------------------------------------------------------------------------------
bool cyhal_uart_is_rx_active(cyhal_uart_t* obj)
{
    HAL_UART_StateTypeDef state;

    state = HAL_UART_GetState(obj->huart);

    /* NOTE: it will also return TRUE in case of both - TX AND RX are busy simultaneously */
    return ((state & HAL_UART_STATE_BUSY_RX) == HAL_UART_STATE_BUSY_RX);
}


//--------------------------------------------------------------------------------------------------
// Callback functions
// NOTE: For UART callbacks, USE_HAL_UART_REGISTER_CALLBACKS must be enabled in STM32CubeMx
//--------------------------------------------------------------------------------------------------

#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)

//--------------------------------------------------------------------------------------------------
// cyhal_uart_tx_complete_callback
//--------------------------------------------------------------------------------------------------
void cyhal_uart_tx_complete_callback(UART_HandleTypeDef* huart)
{
    cyhal_uart_t* obj = _stm32_cyhal_uart_get_obj(huart);
    if (obj != NULL)
    {
        ((cyhal_uart_event_callback_t)obj->callback)(obj->callback_arg, CYHAL_UART_IRQ_TX_DONE);
    }
}


//--------------------------------------------------------------------------------------------------
// cyhal_uart_rx_complete_callback
//--------------------------------------------------------------------------------------------------
void cyhal_uart_rx_complete_callback(UART_HandleTypeDef* huart)
{
    cyhal_uart_t* obj = _stm32_cyhal_uart_get_obj(huart);
    if (obj != NULL)
    {
        ((cyhal_uart_event_callback_t)obj->callback)(obj->callback_arg, CYHAL_UART_IRQ_RX_DONE);
    }

    if ((obj->irq & CYHAL_UART_IRQ_RX_NOT_EMPTY) != 0u)
    {
        /* Restore the RxISR function pointer. RxISR is clean in default RX interrupt
         * handler, when Rx complete Set own RX handler to catch IRQ_RX_NOT_EMPTY
         * interrupt */
        obj->huart->RxISR = cyhal_uart_rx_not_empty_irq_handler;

        /* Enable the UART Parity Error interrupt and Data Register Not Empty interrupt */
        SET_BIT(obj->huart->Instance->CR1, USART_CR1_PEIE | USART_CR1_RXNEIE_RXFNEIE);
    }
}


//--------------------------------------------------------------------------------------------------
// cyhal_uart_error_callback
//--------------------------------------------------------------------------------------------------
void cyhal_uart_error_callback(UART_HandleTypeDef* huart)
{
    cyhal_uart_t* obj = _stm32_cyhal_uart_get_obj(huart);
    if (obj != NULL)
    {
        if (obj->huart->RxState == HAL_UART_STATE_BUSY_RX)
        {
            ((cyhal_uart_event_callback_t)obj->callback)(obj->callback_arg,
                                                         CYHAL_UART_IRQ_RX_ERROR);
        }
        else
        {
            ((cyhal_uart_event_callback_t)obj->callback)(obj->callback_arg,
                                                         CYHAL_UART_IRQ_TX_ERROR);
        }
    }
}


//--------------------------------------------------------------------------------------------------
// cyhal_uart_rx_fifo_full_callback
//--------------------------------------------------------------------------------------------------
void cyhal_uart_rx_fifo_full_callback(UART_HandleTypeDef* huart)
{
    cyhal_uart_t* obj = _stm32_cyhal_uart_get_obj(huart);
    if (obj != NULL)
    {
        ((cyhal_uart_event_callback_t)obj->callback)(obj->callback_arg, CYHAL_UART_IRQ_RX_FULL);
    }
}


//--------------------------------------------------------------------------------------------------
// cyhal_uart_tx_fifo_empty_callback
//--------------------------------------------------------------------------------------------------
void cyhal_uart_tx_fifo_empty_callback(UART_HandleTypeDef* huart)
{
    cyhal_uart_t* obj = _stm32_cyhal_uart_get_obj(huart);
    if (obj != NULL)
    {
        ((cyhal_uart_event_callback_t)obj->callback)(obj->callback_arg, CYHAL_UART_IRQ_TX_EMPTY);
    }
}


#endif /* (USE_HAL_UART_REGISTER_CALLBACKS == 1) */

//--------------------------------------------------------------------------------------------------
// cyhal_uart_rx_not_empty_irq_handler
//--------------------------------------------------------------------------------------------------
void cyhal_uart_rx_not_empty_irq_handler(UART_HandleTypeDef* huart)
{
    cyhal_uart_t* obj = _stm32_cyhal_uart_get_obj(huart);
    ((cyhal_uart_event_callback_t)obj->callback)(obj->callback_arg,
                                                 CYHAL_UART_IRQ_RX_NOT_EMPTY);
}


//--------------------------------------------------------------------------------------------------
// cyhal_uart_register_callback
//--------------------------------------------------------------------------------------------------
void cyhal_uart_register_callback(cyhal_uart_t* obj, cyhal_uart_event_callback_t callback,
                                  void* callback_arg)
{
    /* Check the parameters */
    assert_param(NULL != obj);
    assert_param(NULL != callback);

    /* Store callback info */
    obj->callback     = (void*)callback;
    obj->callback_arg = callback_arg;
}


//--------------------------------------------------------------------------------------------------
// cyhal_uart_enable_event
//--------------------------------------------------------------------------------------------------
void cyhal_uart_enable_event(cyhal_uart_t* obj, cyhal_uart_event_t event,
                             uint8_t intr_priority, bool enable)
{
    #if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
    pUART_CallbackTypeDef p_callback;

    /* Register CYHAL_UART_IRQ_TX_DONE callback */
    if ((event & CYHAL_UART_IRQ_TX_DONE) != CYHAL_UART_IRQ_NONE)
    {
        p_callback = &cyhal_uart_tx_complete_callback;
        if (enable)
        {
            obj->irq |= CYHAL_UART_IRQ_TX_DONE;
            HAL_UART_RegisterCallback(obj->huart, HAL_UART_TX_COMPLETE_CB_ID, p_callback);
        }
        else
        {
            obj->irq &= ~CYHAL_UART_IRQ_TX_DONE;
            HAL_UART_UnRegisterCallback(obj->huart, HAL_UART_TX_COMPLETE_CB_ID);
        }
    }

    /* Register CYHAL_UART_IRQ_RX_DONE callback */
    if ((event & CYHAL_UART_IRQ_RX_DONE) != CYHAL_UART_IRQ_NONE)
    {
        p_callback = &cyhal_uart_rx_complete_callback;
        if (enable)
        {
            obj->irq |= CYHAL_UART_IRQ_RX_DONE;
            HAL_UART_RegisterCallback(obj->huart, HAL_UART_RX_COMPLETE_CB_ID, p_callback);
        }
        else
        {
            obj->irq &= ~CYHAL_UART_IRQ_RX_DONE;
            HAL_UART_UnRegisterCallback(obj->huart, HAL_UART_RX_COMPLETE_CB_ID);
        }
    }

    /* Register CYHAL_UART_IRQ_TX_ERROR / CYHAL_UART_IRQ_RX_ERROR callback */
    if ((event & (CYHAL_UART_IRQ_TX_ERROR | CYHAL_UART_IRQ_RX_ERROR)) !=
        CYHAL_UART_IRQ_NONE)
    {
        p_callback = &cyhal_uart_error_callback;
        if (enable)
        {
            obj->irq |= CYHAL_UART_IRQ_TX_ERROR;
            HAL_UART_RegisterCallback(obj->huart, HAL_UART_ERROR_CB_ID, p_callback);
        }
        else
        {
            obj->irq &= ~CYHAL_UART_IRQ_TX_ERROR;
            HAL_UART_UnRegisterCallback(obj->huart, HAL_UART_ERROR_CB_ID);
        }
    }

    /* Register CYHAL_UART_IRQ_RX_FULL callback */
    if ((event & CYHAL_UART_IRQ_RX_FULL) != CYHAL_UART_IRQ_NONE)
    {
        p_callback = &cyhal_uart_rx_fifo_full_callback;
        if (enable)
        {
            obj->irq |= CYHAL_UART_IRQ_RX_FULL;
            HAL_UART_RegisterCallback(obj->huart, HAL_UART_RX_FIFO_FULL_CB_ID, p_callback);
        }
        else
        {
            obj->irq &= ~CYHAL_UART_IRQ_RX_FULL;
            HAL_UART_UnRegisterCallback(obj->huart, HAL_UART_RX_FIFO_FULL_CB_ID);
        }
    }

    /* Register / un-register CYHAL_UART_IRQ_TX_EMPTY callback */
    if ((event & CYHAL_UART_IRQ_TX_EMPTY) != CYHAL_UART_IRQ_NONE)
    {
        p_callback = &cyhal_uart_tx_fifo_empty_callback;

        if (enable)
        {
            obj->irq |= CYHAL_UART_IRQ_TX_EMPTY;
            HAL_UART_RegisterCallback(obj->huart, HAL_UART_TX_FIFO_EMPTY_CB_ID, p_callback);
        }
        else
        {
            obj->irq &= ~CYHAL_UART_IRQ_TX_EMPTY;
            HAL_UART_UnRegisterCallback(obj->huart, HAL_UART_TX_FIFO_EMPTY_CB_ID);
        }
    }

    /* Register / un-register CYHAL_UART_IRQ_RX_NOT_EMPTY callback */
    if ((event & CYHAL_UART_IRQ_RX_NOT_EMPTY) != CYHAL_UART_IRQ_NONE)
    {
        if (enable)
        {
            obj->irq |= CYHAL_UART_IRQ_RX_NOT_EMPTY;
            /* Update the RxISR function pointer. Set own RX handler to catch
             * IRQ_RX_NOT_EMPTY interrupt */
            obj->huart->RxISR = cyhal_uart_rx_not_empty_irq_handler;

            /* Enable the UART Parity Error interrupt and Data Register Not Empty interrupt */
            SET_BIT(obj->huart->Instance->CR1, USART_CR1_PEIE | USART_CR1_RXNEIE_RXFNEIE);
        }
        else
        {
            obj->irq &= ~CYHAL_UART_IRQ_RX_NOT_EMPTY;
            CLEAR_BIT(obj->huart->Instance->CR1, USART_CR1_PEIE | USART_CR1_RXNEIE_RXFNEIE);
        }
    }

    /* Configure uart interrupt. Do not disable interrupt if we have have some
     * enabled  obj->irq */
    if ((enable) || (!enable && (obj->irq == 0u)))
    {
        _stm32_cyhal_uart_enable_irq(obj->huart->Instance, intr_priority, enable);
    }

    #else /* if (USE_HAL_UART_REGISTER_CALLBACKS == 1) */
    /* NOTE: USE_HAL_UART_REGISTER_CALLBACKS must be enabled */
    assert_param(false);
    #endif /*  (USE_HAL_UART_REGISTER_CALLBACKS == 1) */
}


//--------------------------------------------------------------------------------------------------
// Private functions
//--------------------------------------------------------------------------------------------------

static uint32_t _stm32_cyhal_uart_convert_parity(cyhal_uart_parity_t parity)
{
    switch (parity)
    {
        case CYHAL_UART_PARITY_NONE:
            return UART_PARITY_NONE;

        case CYHAL_UART_PARITY_EVEN:
            return UART_PARITY_EVEN;

        case CYHAL_UART_PARITY_ODD:
            return UART_PARITY_ODD;

        default:
            assert_param(false); /* wrong value */
            return 0xFFFF;
    }
}


//--------------------------------------------------------------------------------------------------
// _stm32_cyhal_uart_convert_stopbits
//--------------------------------------------------------------------------------------------------
static uint32_t _stm32_cyhal_uart_convert_stopbits(uint8_t stopbits)
{
    switch (stopbits)
    {
        case 1:
            return UART_STOPBITS_1;

        case 2:
            return UART_STOPBITS_2;

        default:
            assert_param(false); /* wrong value */
            return 0xFFFF;
    }
}


//--------------------------------------------------------------------------------------------------
// _stm32_cyhal_uart_convert_wordlength
//--------------------------------------------------------------------------------------------------
static uint32_t _stm32_cyhal_uart_convert_wordlength(uint8_t wordlength)
{
    switch (wordlength)
    {
        case 7:
            return UART_WORDLENGTH_7B;

        case 8:
            return UART_WORDLENGTH_8B;

        case 9:
            return UART_WORDLENGTH_9B;

        default:
            assert_param(false); /* wrong value */
            return 0xFFFF;
    }
}


//--------------------------------------------------------------------------------------------------
// _stm32_cyhal_uart_enable_irq
//--------------------------------------------------------------------------------------------------
void _stm32_cyhal_uart_enable_irq(USART_TypeDef* instance, uint32_t priority, bool enable)
{
    IRQn_Type IRQn = (IRQn_Type)0;


    #if defined (LPUART1)
    if (instance == LPUART1)
    {
        IRQn = LPUART1_IRQn;
    }
    else
    #endif

    #if defined (USART1)
    if (instance == USART1)
    {
        IRQn = USART1_IRQn;
    }
    else
    #endif

    #if defined (USART2)
    if (instance == USART2)
    {
        IRQn = USART2_IRQn;
    }
    else
    #endif

    #if defined (USART3)
    if (instance == USART3)
    {
        IRQn = USART3_IRQn;
    }
    else
    #endif

    #if defined (UART4)
    if (instance == UART4)
    {
        IRQn = UART4_IRQn;
    }
    else
    #endif

    #if defined (UART5)
    if (instance == UART5)
    {
        IRQn = UART5_IRQn;
    }
    else
    #endif

    #if defined (USART6)
    if (instance == USART6)
    {
        IRQn = USART6_IRQn;
    }
    else
    #endif

    #if defined (UART7)
    if (instance == UART7)
    {
        IRQn = UART7_IRQn;
    }
    else
    #endif

    #if defined (UART8)
    if (instance == UART8)
    {
        IRQn = UART8_IRQn;
    }
    else
    #endif
    {
        assert_param(false); /* wrong instance */
    }

    if (enable)
    {
        HAL_NVIC_SetPriority(IRQn, priority, 0);
        HAL_NVIC_EnableIRQ(IRQn);
    }
    else
    {
        HAL_NVIC_DisableIRQ(IRQn);
    }
}


//--------------------------------------------------------------------------------------------------
// _stm32_cyhal_uart_alloc_hw
//--------------------------------------------------------------------------------------------------
static UART_HandleTypeDef* _stm32_cyhal_uart_alloc_hw(cyhal_uart_t* obj, cyhal_gpio_t tx,
                                                      cyhal_gpio_t rx)
{
    UART_HandleTypeDef* handle = NULL;

    /* Go through _cyhal_uart_structs and find uart handle by TX/RX pins */
    for (uint32_t i = 0u; i < CYHAL_UART_MAX_INSTANCES; i++)
    {
        if (((tx != NC) && (tx == _cyhal_uart_structs[i].associate_pin)) ||
            ((rx != NC) && (rx == _cyhal_uart_structs[i].associate_pin)))
        {
            if (_cyhal_uart_structs[i].occupied_obj == NULL)
            {
                handle                              =  _cyhal_uart_structs[i].huart;
                _cyhal_uart_structs[i].occupied_obj = obj;
                break;
            }
        }
    }
    return (handle);
}


//--------------------------------------------------------------------------------------------------
// _stm32_cyhal_uart_free_hw
//--------------------------------------------------------------------------------------------------
static void _stm32_cyhal_uart_free_hw(cyhal_uart_t* obj)
{
    /* Go through _cyhal_uart_structs and find uart handle by TX/RX pins */
    for (uint32_t i = 0u; i < CYHAL_UART_MAX_INSTANCES; i++)
    {
        if (obj->huart == _cyhal_uart_structs[i].huart)
        {
            _cyhal_uart_structs[i].occupied_obj = NULL;
            break;
        }
    }
}


//--------------------------------------------------------------------------------------------------
// _stm32_cyhal_uart_get_obj
//--------------------------------------------------------------------------------------------------
cyhal_uart_t* _stm32_cyhal_uart_get_obj(UART_HandleTypeDef* huart)
{
    for (uint32_t i = 0u; i < CYHAL_UART_MAX_INSTANCES; i++)
    {
        if (_cyhal_uart_structs[i].occupied_obj->huart == huart)
        {
            return (_cyhal_uart_structs[i].occupied_obj);
        }
    }
    return NULL;
}


//--------------------------------------------------------------------------------------------------
// _stm32_cyhal_uart_hw_init
//--------------------------------------------------------------------------------------------------
uint32_t _stm32_cyhal_uart_hw_init(UART_HandleTypeDef* huart, cyhal_gpio_t associate_pin)
{
    for (uint32_t i = 0u; i < CYHAL_UART_MAX_INSTANCES; i++)
    {
        if (_cyhal_uart_structs[i].huart == NULL)
        {
            _cyhal_uart_structs[i].huart         = huart;
            _cyhal_uart_structs[i].associate_pin = associate_pin;
            return 0;
        }
    }
    return 1; /* all structs are occupied already */
}


#if defined(__cplusplus)
}
#endif /* defined(__cplusplus) */

#endif /* defined(HAL_UART_MODULE_ENABLED) */
