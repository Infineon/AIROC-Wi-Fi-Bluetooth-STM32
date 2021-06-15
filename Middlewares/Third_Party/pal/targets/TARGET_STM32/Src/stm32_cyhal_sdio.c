/***************************************************************************************************
 * \file stm32_cyhal_sdio.c
 *
 * \brief
 * Provides a high level interface for interacting with STM32 SDIO.
 * This is a wrapper around the lower level STM32 SDIO HAL API.
 *
 ***************************************************************************************************
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
 **************************************************************************************************/

#if defined(__cplusplus)
extern "C"
{
#endif

#include "cyhal_sdio.h"
#include "stm32_cyhal_sdio_ex.h"
#include "stdio.h"
#include "string.h"

#if defined(HAL_SD_MODULE_ENABLED)


/***************************************************************************************************
 *      Private macros
 **************************************************************************************************/

/* Number of cycles for read/write operation complete */
#define _CYHAL_SDIO_RW_RETRY_CYCLES             (1000U)

#define _CYHAL_SDIO_400KHZ                      (400000U)

/* Masks for errors in an R5 response */
#define SDMMC_R5_COM_CRC_ERROR                  ((uint32_t)0x00008000U)
#define SDMMC_R5_ILLEGAL_COMMAND                ((uint32_t)0x00004000U)
#define SDMMC_R5_IO_CURRENT_STATE               ((uint32_t)0x00003000U)
#define SDMMC_R5_ERROR                          ((uint32_t)0x00000400U)
#define SDMMC_R5_FUNCTION_NUMBER                ((uint32_t)0x00000200U)
#define SDMMC_R5_OUT_OF_RANGE                   ((uint32_t)0x00000100U)
#define SDMMC_R5_ERRORBITS                      (SDMMC_R5_COM_CRC_ERROR   | \
                                                 SDMMC_R5_ILLEGAL_COMMAND | \
                                                 SDMMC_R5_ERROR           | \
                                                 SDMMC_R5_FUNCTION_NUMBER | \
                                                 SDMMC_R5_OUT_OF_RANGE)
/* Data Block Size */
#define _CYHAL_SDIO_DATA_BLOCK_SIZE_2B          (2U)
#define _CYHAL_SDIO_DATA_BLOCK_SIZE_4B          (4U)
#define _CYHAL_SDIO_DATA_BLOCK_SIZE_8B          (8U)
#define _CYHAL_SDIO_DATA_BLOCK_SIZE_16B         (16U)
#define _CYHAL_SDIO_DATA_BLOCK_SIZE_32B         (32U)
#define _CYHAL_SDIO_DATA_BLOCK_SIZE_64B         (64U)
#define _CYHAL_SDIO_DATA_BLOCK_SIZE_128B        (128U)
#define _CYHAL_SDIO_DATA_BLOCK_SIZE_256B        (256U)
#define _CYHAL_SDIO_DATA_BLOCK_SIZE_512B        (512U)

/* Set default SDIO priority */
#if !defined (CYHAL_SDIO_IRQ_PRIORITY)
    #define CYHAL_SDIO_IRQ_PRIORITY             (5U)
#endif /* !defined (CYHAL_SDIO_IRQ_PRIORITY) */

/* SDIO DMA buffer used in cyhal_sdio_bulk_transfer function. The default value
 * is 1568 bytes, it required by WHD as max backplane transfer size.
 * Overwrite _CYHAL_SDIO_DMA_BUFFER_SIZE in cybsp.h if need to increase the
 * SDIO DMA buffer size */
#if !defined (_CYHAL_SDIO_DMA_BUFFER_SIZE)
    #define _CYHAL_SDIO_DMA_BUFFER_SIZE         (1568 / 4)  /* size in words */
#endif /* !defined (_CYHAL_SDIO_DMA_BUFFER_SIZE) */


/***************************************************************************************************
 *      Private variables
 **************************************************************************************************/

static SD_HandleTypeDef* _cyhal_sdio_handle = NULL;

/***************************************************************************************************
 *      Private functions
 **************************************************************************************************/

static uint32_t _stm32_sdio_cmd_rw_extended(SDMMC_TypeDef* SDMMCx, uint32_t argument,
                                            uint32_t* response);
static uint32_t _stm32_sdio_cmd_send_op_cond(SDMMC_TypeDef* SDMMCx);

static uint32_t _stm32_sdio_cmd_rw_direct(SDMMC_TypeDef* SDMMCx, uint32_t argument,
                                          uint32_t* response);
static uint32_t _stm32_sdio_convert_block_size(uint16_t block_size);
static uint32_t _stm32_sdio_find_optimal_block_size(uint32_t data_size);

static void _stm32_sdio_enable_irq(const SD_TypeDef* instance, uint32_t priority, bool en_irq);
static void _stm32_sdio_enable_hw_block(cyhal_sdio_t* obj);
static void _stm32_sdio_disable_hw_block(const cyhal_sdio_t* obj);

static uint32_t _stm32_sdio_get_cmd_resp4(SDMMC_TypeDef* SDMMCx);
static uint32_t _stm32_sdio_get_cmd_resp5(SDMMC_TypeDef* SDMMCx, uint8_t SD_CMD, uint32_t* data);

static uint32_t _stm32_safe_divide(uint32_t num, uint32_t denom);


/***************************************************************************************************
 * stm32_cypal_sdio_hw_init
 **************************************************************************************************/
uint32_t stm32_cypal_sdio_hw_init(SD_HandleTypeDef* hsd)
{
    /* Check the parameters */
    assert_param(NULL != hsd);

    _cyhal_sdio_handle = hsd;
    return 0;
}


/***************************************************************************************************
 * cyhal_sdio_init
 **************************************************************************************************/
cy_rslt_t cyhal_sdio_init(cyhal_sdio_t* obj, cyhal_gpio_t cmd, cyhal_gpio_t clk, cyhal_gpio_t data0,
                          cyhal_gpio_t data1, cyhal_gpio_t data2, cyhal_gpio_t data3)
{
    (void)cmd;
    (void)clk;
    (void)data0;
    (void)data1;
    (void)data2;
    (void)data3;
    uint32_t sdmmc_clk;

    /* Check the parameters */
    assert_param(NULL != obj);

    /* Update obj sd handle */
    obj->hsd = _cyhal_sdio_handle;

    /* Check the SD handle allocation */
    if (obj->hsd == NULL)
    {
        return (cy_rslt_t)HAL_ERROR;
    }

    /* Init the low level hardware : GPIO, CLOCK, CORTEX...etc */
    if (obj->hsd->State == HAL_SD_STATE_RESET)
    {
        /* Allocate lock resource and initialize it */
        obj->hsd->Lock = HAL_UNLOCKED;

        HAL_SD_MspInit(obj->hsd);
    }

    /* Default SDMMC peripheral configuration for initialization */
    obj->hsd->Init.ClockEdge           = SDMMC_CLOCK_EDGE_RISING;
    obj->hsd->Init.ClockPowerSave      = SDMMC_CLOCK_POWER_SAVE_DISABLE;
    obj->hsd->Init.BusWide             = SDMMC_BUS_WIDE_4B;
    obj->hsd->Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;

    /* Init Clock should be less or equal to 400Khz */
    sdmmc_clk = HAL_RCCEx_GetPeriphCLKFreq(STM32_RCC_PERIPHCLK_SDMMC);

    obj->hsd->Init.ClockDiv = _stm32_safe_divide(sdmmc_clk, 2U * _CYHAL_SDIO_400KHZ);
    obj->hsd->State         = HAL_SD_STATE_BUSY;

    /* Enable SDIO block */
    _stm32_sdio_enable_hw_block(obj);

    /* Initialize the error code */
    obj->hsd->ErrorCode = HAL_SD_ERROR_NONE;

    /* store object in handle context */
    obj->hsd->Context = (uint32_t)obj;

    /* Initialize the SD state */
    obj->hsd->State = HAL_SD_STATE_READY;

    /* Enable interrupts */
    _stm32_sdio_enable_irq(obj->hsd->Instance, CYHAL_SDIO_IRQ_PRIORITY, true);
    return CY_RSLT_SUCCESS;
}


/***************************************************************************************************
 * cyhal_sdio_configure
 **************************************************************************************************/
cy_rslt_t cyhal_sdio_configure(cyhal_sdio_t* obj, const cyhal_sdio_cfg_t* config)
{
    uint32_t clk_freq;

    /* Check the parameters */
    assert_param(NULL != obj);

    /* Update obj sd handle */
    if (obj->hsd == NULL)
    {
        obj->hsd = _cyhal_sdio_handle;
    }

    /* Do not change frequency if requested value is zero */
    if (config->frequencyhal_hz != 0u)
    {
        /* Override the SDMMC Clock frequency Configuration if defined */
        #ifdef SDMMC_CLK_FREQ_OVERRIDE
        clk_freq = SDMMC_CLK_FREQ_OVERRIDE;
        #else
        clk_freq = config->frequencyhal_hz;
        #endif
        obj->frequencyhal_hz = clk_freq;

        /* Calculate SDMMC Clock divider. */
        /* ClockDiv = SDMMC input clock / (2 * Expected SDMMC Clock) */
        uint32_t sdmmc_clk = HAL_RCCEx_GetPeriphCLKFreq(STM32_RCC_PERIPHCLK_SDMMC);

        obj->hsd->Init.ClockDiv = _stm32_safe_divide(sdmmc_clk, 2U * clk_freq);

        /* Reset and Enable SDIO block */
        _stm32_sdio_enable_hw_block(obj);
    }

    /* Do not change block size if requested value is zero */
    if (config->block_size != 0u)
    {
        obj->block_size = config->block_size;
    }

    return CY_RSLT_SUCCESS;
}


/***************************************************************************************************
 * cyhal_sdio_send_cmd
 **************************************************************************************************/
cy_rslt_t cyhal_sdio_send_cmd(const cyhal_sdio_t* obj, cyhal_transfer_t direction,
                              cyhal_sdio_command_t command, uint32_t argument, uint32_t* response)
{
    (void)direction;
    uint32_t ret;

    /* Check the parameters */
    assert_param(obj != NULL);
    assert_param(obj->hsd != NULL);

    switch (command)
    {
        /* CMD0 */
        case CYHAL_SDIO_CMD_GO_IDLE_STATE:
            ret = SDMMC_CmdGoIdleState(obj->hsd->Instance);
            break;

        /* CMD3 */
        case CYHAL_SDIO_CMD_SEND_RELATIVE_ADDR:
        {
            uint16_t resp;
            assert_param(response != NULL);
            ret       = SDMMC_CmdSetRelAdd(obj->hsd->Instance, &resp);
            *response = resp;
            break;
        }

        /* CMD5 */
        case CYHAL_SDIO_CMD_IO_SEND_OP_COND:
            ret = _stm32_sdio_cmd_send_op_cond(obj->hsd->Instance);
            break;

        /* CMD7 */
        case CYHAL_SDIO_CMD_SELECT_CARD:
            ret = SDMMC_CmdSelDesel(obj->hsd->Instance, (uint64_t)argument << 16);
            break;

        /* CMD52 */
        case CYHAL_SDIO_CMD_IO_RW_DIRECT:
            /* this one already has  != NULL check inside */
            ret = _stm32_sdio_cmd_rw_direct(obj->hsd->Instance, argument, response);
            break;

        /* CMD53 */
        case CYHAL_SDIO_CMD_IO_RW_EXTENDED:
            /* this one already has  != NULL check inside */
            ret = _stm32_sdio_cmd_rw_extended(obj->hsd->Instance, argument, response);
            break;

        case CYHAL_SDIO_CMD_GO_INACTIVE_STATE:
        default:
            ret = CYHAL_SDIO_RSLT_ERR_BAD_PARAM;
            break;
    }
    return ret;
}


/***************************************************************************************************
 * cyhal_sdio_bulk_transfer
 **************************************************************************************************/
cy_rslt_t cyhal_sdio_bulk_transfer(cyhal_sdio_t* obj, cyhal_transfer_t direction, uint32_t argument,
                                   const uint32_t* data, uint16_t length, uint32_t* response)
{
    cy_rslt_t result = CYHAL_SDIO_RSLT_CANCELED;
    CYHAL_ALIGN_DMA_BUFFER(static uint32_t  _temp_dma_buffer[_CYHAL_SDIO_DMA_BUFFER_SIZE]);

    /* Check data buffer if aligned on 32-bytes (needed for cache maintenance purpose),
     * and Length is multiple by block size. If NOT, use internal buffer for DMA */
    bool use_temp_dma_buffer = (((uint32_t)data % _CYHAL_DMA_BUFFER_ALIGN_BYTES) != 0u) ||
                               ((length % obj->block_size) != 0u);

    /* Check the parameters */
    assert_param(NULL != obj);
    assert_param(0u != length);
    assert_param(!use_temp_dma_buffer || (length <= sizeof(_temp_dma_buffer)));

    SDMMC_DataInitTypeDef config;
    sdio_cmd_argument_t   arg = { .value = argument };

    uint32_t  timeout = _CYHAL_SDIO_RW_RETRY_CYCLES;
    uint32_t  number_of_blocks;
    uint32_t  block_size;
    uint32_t  flags;
    uint32_t* p_dma_buffer = NULL;

    if (obj->hsd->State == HAL_SD_STATE_READY)
    {
        /* Block size not initialized */
        if (obj->block_size == 0U)
        {
            obj->block_size = _CYHAL_SDIO_DATA_BLOCK_SIZE_64B;
        }

        /* Block mode */
        if (length >= obj->block_size)
        {
            block_size       = obj->block_size;
            number_of_blocks = (length + block_size - 1u) / block_size;
        }
        /* Byte mode */
        else
        {
            block_size       = _stm32_sdio_find_optimal_block_size(length);
            number_of_blocks = 1UL;
        }

        /* Set the CMD53 byte mode size to be the same as the block size */
        if (arg.cmd53.block_mode == 0U)
        {
            if (_stm32_sdio_find_optimal_block_size(arg.cmd53.count) <
                _CYHAL_SDIO_DATA_BLOCK_SIZE_512B)
            {
                arg.cmd53.count = _stm32_sdio_find_optimal_block_size(arg.cmd53.count);
            }
            else
            {
                arg.cmd53.count = 0u;
            }
        }

        obj->hsd->ErrorCode = HAL_SD_ERROR_NONE;
        obj->hsd->State     = HAL_SD_STATE_BUSY;

        /* Initialize data control register */
        obj->hsd->Instance->DCTRL = 0u;

        if (use_temp_dma_buffer)
        {
            if ((length == 0) || (length > sizeof(_temp_dma_buffer)))
            {
                obj->hsd->State = HAL_SD_STATE_READY;
                return CYHAL_SDIO_BAD_ARGUMENT;
            }

            /* Using internal buffer */
            p_dma_buffer = _temp_dma_buffer;

            /* Clean internal buffer and copy data */
            (void)memcpy((void*)_temp_dma_buffer, (void*)data, length);

            if (length != sizeof(_temp_dma_buffer))
            {
                (void)memset((void*)&((uint8_t*)_temp_dma_buffer)[length], 0,
                             sizeof(_temp_dma_buffer) - length);
            }
        }
        else
        {
            /* Use app data buffer */
            p_dma_buffer = (uint32_t*)data;
        }

        /* Maintenance D-cache for DMA buffers */
        #if defined(_CYHAL_DCACHE_MAINTENANCE)
        if (direction == CYHAL_WRITE)
        {
            SCB_CleanDCache_by_Addr((uint32_t*)p_dma_buffer, block_size * number_of_blocks);
        }
        else
        {
            /* Cache-Invalidate the output from DMA */
            SCB_InvalidateDCache_by_Addr((uint32_t*)p_dma_buffer, block_size * number_of_blocks);
        }
        #endif /* if defined(_CYHAL_DCACHE_MAINTENANCE) */

        /* DMA configuration (use single buffer) */
        obj->hsd->Instance->IDMACTRL  = SDMMC_ENABLE_IDMA_SINGLE_BUFF;
        obj->hsd->Instance->IDMABASE0 = (uint32_t)p_dma_buffer;

        /* Configure the SD DPSM (Data Path State Machine) */
        config.DataTimeOut = SDMMC_DATATIMEOUT;
        config.TransferDir = (direction == CYHAL_WRITE) ? SDMMC_TRANSFER_DIR_TO_CARD :
                             SDMMC_TRANSFER_DIR_TO_SDMMC;
        config.TransferMode  = SDMMC_TRANSFER_MODE_BLOCK;
        config.DPSM          = SDMMC_DPSM_DISABLE;
        config.DataLength    = block_size * number_of_blocks;
        config.DataBlockSize = _stm32_sdio_convert_block_size((uint16_t)block_size);

        obj->hsd->Instance->DCTRL |= SDMMC_DCTRL_SDIOEN; /* SD I/O enable functions */
        (void)SDMMC_ConfigData(obj->hsd->Instance, &config);

        __SDMMC_CMDTRANS_ENABLE(obj->hsd->Instance);

        /* Call CMD53 (CYHAL_SDIO_CMD_IO_RW_EXTENDED) */
        result = cyhal_sdio_send_cmd(obj, direction, CYHAL_SDIO_CMD_IO_RW_EXTENDED,
                                     arg.value, response);

        /* Check error from CMD53 */
        if (result != HAL_SD_ERROR_NONE)
        {
            /* Clear all the static flags */
            __HAL_SD_CLEAR_FLAG(obj->hsd, SDMMC_STATIC_FLAGS);
            obj->hsd->State = HAL_SD_STATE_READY;
            return result;
        }

        /* Set polling flags */
        flags = SDMMC_FLAG_DCRCFAIL | SDMMC_FLAG_DTIMEOUT | SDMMC_FLAG_DATAEND |
                SDMMC_FLAG_TXUNDERR | SDMMC_FLAG_RXOVERR;

        /* Poll on SDMMC flags */
        while ((!__HAL_SD_GET_FLAG(obj->hsd, flags)) && (timeout > 0u))
        {
            /* returning ERROR code for timeout */
            HAL_Delay(1U);
            timeout--;
            if (timeout == 0u)
            {
                obj->hsd->State = HAL_SD_STATE_READY;
                return CYHAL_SDIO_RET_CMD_TIMEOUT;
            }
        }

        /* Check for SDMMC interrupt flags */
        if (__HAL_SD_GET_FLAG(obj->hsd, SDMMC_FLAG_DATAEND) != RESET)
        {
            __HAL_SD_CLEAR_FLAG(obj->hsd, SDMMC_FLAG_DATAEND);
        }
        else if (__HAL_SD_GET_FLAG(obj->hsd, SDMMC_FLAG_DCRCFAIL | SDMMC_FLAG_DTIMEOUT |
                                   SDMMC_FLAG_RXOVERR | SDMMC_FLAG_TXUNDERR) != RESET)
        {
            /* Set Error code */
            if (__HAL_SD_GET_FLAG(obj->hsd, SDMMC_FLAG_DCRCFAIL) != RESET)
            {
                obj->hsd->ErrorCode |= HAL_SD_ERROR_DATA_CRC_FAIL;
            }
            if (__HAL_SD_GET_FLAG(obj->hsd, SDMMC_FLAG_DTIMEOUT) != RESET)
            {
                obj->hsd->ErrorCode |= HAL_SD_ERROR_DATA_TIMEOUT;
            }
            if (__HAL_SD_GET_FLAG(obj->hsd, SDMMC_FLAG_RXOVERR) != RESET)
            {
                obj->hsd->ErrorCode |= HAL_SD_ERROR_RX_OVERRUN;
            }
            if (__HAL_SD_GET_FLAG(obj->hsd, SDMMC_FLAG_TXUNDERR) != RESET)
            {
                obj->hsd->ErrorCode |= HAL_SD_ERROR_TX_UNDERRUN;
            }
        }
        else
        {
            /* Nothing to do */
        }

        /* Set SDIO to init state */
        __HAL_SD_DISABLE_IT(obj->hsd, SDMMC_IT_IDMABTC);
        __SDMMC_CMDTRANS_DISABLE(obj->hsd->Instance);

        obj->hsd->Instance->DLEN     = 0;
        obj->hsd->Instance->DCTRL    = SDMMC_DCTRL_SDIOEN;
        obj->hsd->Instance->IDMACTRL = SDMMC_DISABLE_IDMA;
        obj->hsd->Instance->CMD      = 0;
        obj->hsd->Instance->DCTRL   |= SDMMC_DCTRL_FIFORST;

        /* Clear all the static flags */
        __HAL_SD_CLEAR_FLAG(obj->hsd, SDMMC_STATIC_DATA_FLAGS);

        /* Copy received data to user */
        if ((direction == CYHAL_READ) && (use_temp_dma_buffer) && (!obj->hsd->ErrorCode))
        {
            (void)memcpy((void*)data, (void*)_temp_dma_buffer, (size_t)length);
        }

        /* Set the SD state to ready to be able to start again the process */
        _cyhal_sdio_handle->State = HAL_SD_STATE_READY;

        /* This interrupt is disabled in interrupt handler so need to enable it here */
        if (0U != ((uint32_t)CYHAL_SDIO_CARD_INTERRUPT & obj->irq))
        {
            __HAL_SD_ENABLE_IT(obj->hsd, SDMMC_IT_SDIOIT);
        }
    }
    else
    {
        obj->hsd->ErrorCode |= HAL_SD_ERROR_BUSY;
        return CYHAL_SDIO_RSLT_ERR_PM_PENDING;
    }

    if (obj->hsd->ErrorCode == HAL_SD_ERROR_NONE)
    {
        result = CY_RSLT_SUCCESS;
    }
    else
    {
        result = CYHAL_SDIO_RET_NO_SP_ERRORS; /**< Non-specific error code*/
    }
    return result;
}


/***************************************************************************************************
 * cyhal_sdio_register_callback
 **************************************************************************************************/
void cyhal_sdio_register_callback(cyhal_sdio_t* obj, cyhal_sdio_event_callback_t callback,
                                  void* callback_arg)
{
    /* Check the parameters */
    assert_param(NULL != obj);
    assert_param(NULL != callback);

    /* Store callback info */
    obj->callback     = (void*)callback;
    obj->callback_arg = callback_arg;
}


/***************************************************************************************************
 * cyhal_sdio_enable_event
 **************************************************************************************************/
void cyhal_sdio_enable_event(cyhal_sdio_t* obj, cyhal_sdio_event_t event, uint8_t intr_priority,
                             bool enable)
{
    (void)intr_priority;
    if (enable)
    {
        __HAL_SD_ENABLE_IT(obj->hsd, SDMMC_IT_SDIOIT);
        obj->irq |= (uint32_t)event;
    }
    else
    {
        __HAL_SD_DISABLE_IT(obj->hsd, SDMMC_IT_SDIOIT);
        obj->irq &= ~(uint32_t)event;
    }
}


/***************************************************************************************************
 * stm32_cyhal_sdio_irq_handler
 **************************************************************************************************/
void stm32_cyhal_sdio_irq_handler(void)
{
    if ((_cyhal_sdio_handle != NULL) &&
        (__HAL_SD_GET_FLAG(_cyhal_sdio_handle, SDMMC_STA_SDIOIT) != RESET))
    {
        cyhal_sdio_t* obj = (cyhal_sdio_t*)_cyhal_sdio_handle->Context;
        ((cyhal_sdio_event_callback_t)obj->callback)(obj->callback_arg, CYHAL_SDIO_CARD_INTERRUPT);

        /* Clear the interrupt */
        __HAL_SD_CLEAR_FLAG(_cyhal_sdio_handle, SDMMC_FLAG_SDIOIT);

        /* Mask interrupt, to be unmasked later by Tx Path */
        __HAL_SD_DISABLE_IT(_cyhal_sdio_handle, SDMMC_IT_SDIOIT);

        /* Set IRQ Flag which will be used for setting IRQ MASK back */
        obj->irq |= (uint32_t)CYHAL_SDIO_CARD_INTERRUPT;
    }
}


/***************************************************************************************************
 * Private Functions
 **************************************************************************************************/

/***************************************************************************************************
 * _stm32_safe_divide
 **************************************************************************************************/
static uint32_t _stm32_safe_divide(uint32_t num, uint32_t denom)
{
    /* Safe divide */
    uint32_t divres;

    assert_param(num != 0u);
    assert_param(denom != 0u);
    divres = num / denom;
    if ((num % denom) >= (denom>>1))
    {
        divres++;
    }
    return divres;
}


/***************************************************************************************************
 * _stm32_sdio_enable_hw_block
 **************************************************************************************************/
static void _stm32_sdio_enable_hw_block(cyhal_sdio_t* obj)
{
    /* Disable/reset SDIO Block */
    _stm32_sdio_disable_hw_block(obj);

    /* Enable the SDIO Clock */
    #if defined (SDMMC1)
    if (obj->hsd->Instance == SDMMC1)
    {
        __HAL_RCC_SDMMC1_CLK_ENABLE();
    }
    #endif
    #if defined (SDMMC2)
    else
    {
        __HAL_RCC_SDMMC2_CLK_ENABLE();
    }
    #endif

    /* Initialize SDMMC peripheral interface with default configuration */
    (void)SDMMC_Init(obj->hsd->Instance, obj->hsd->Init);

    /* Set Power State to ON */
    (void)SDMMC_PowerState_ON(obj->hsd->Instance);

    /* Wait clock */
    (void)SDMMC_SetSDMMCReadWaitMode(obj->hsd->Instance, SDMMC_READ_WAIT_MODE_CLK);


    uint32_t sdmmc_clk = HAL_RCCEx_GetPeriphCLKFreq(STM32_RCC_PERIPHCLK_SDMMC) /
                         (2U * obj->hsd->Init.ClockDiv);
    if (sdmmc_clk != 0U)
    {
        /* Wait 74 Cycles: required power up waiting time
         * before starting the SD initialization sequence */
        HAL_Delay(1U + (74U * 1000U / (sdmmc_clk)));
    }
    else
    {
        HAL_Delay(2U);
    }
}


/***************************************************************************************************
 * _stm32_sdio_disable_hw_block
 **************************************************************************************************/
static void _stm32_sdio_disable_hw_block(const cyhal_sdio_t* obj)
{
    /* Reset SDIO Block */
    (void)SDMMC_PowerState_OFF(obj->hsd->Instance);
    #if defined (SDMMC1)
    if (obj->hsd->Instance == SDMMC1)
    {
        __HAL_RCC_SDMMC1_FORCE_RESET();
        __HAL_RCC_SDMMC1_RELEASE_RESET();
    }
    #endif
    #if defined (SDMMC2)
    else
    {
        __HAL_RCC_SDMMC2_FORCE_RESET();
        __HAL_RCC_SDMMC2_RELEASE_RESET();
    }
    #endif
}


/***************************************************************************************************
 * _stm32_sdio_find_optimal_block_size
 **************************************************************************************************/
static  uint32_t _stm32_sdio_find_optimal_block_size(uint32_t data_size)
{
    if (data_size > (uint32_t)_CYHAL_SDIO_DATA_BLOCK_SIZE_256B)
    {
        return _CYHAL_SDIO_DATA_BLOCK_SIZE_512B;
    }
    if (data_size > (uint32_t)_CYHAL_SDIO_DATA_BLOCK_SIZE_128B)
    {
        return _CYHAL_SDIO_DATA_BLOCK_SIZE_256B;
    }
    if (data_size > (uint32_t)_CYHAL_SDIO_DATA_BLOCK_SIZE_64B)
    {
        return _CYHAL_SDIO_DATA_BLOCK_SIZE_128B;
    }
    if (data_size > (uint32_t)_CYHAL_SDIO_DATA_BLOCK_SIZE_32B)
    {
        return _CYHAL_SDIO_DATA_BLOCK_SIZE_64B;
    }
    if (data_size > (uint32_t)_CYHAL_SDIO_DATA_BLOCK_SIZE_16B)
    {
        return _CYHAL_SDIO_DATA_BLOCK_SIZE_32B;
    }
    if (data_size > (uint32_t)_CYHAL_SDIO_DATA_BLOCK_SIZE_8B)
    {
        return _CYHAL_SDIO_DATA_BLOCK_SIZE_16B;
    }
    if (data_size > (uint32_t)_CYHAL_SDIO_DATA_BLOCK_SIZE_4B)
    {
        return _CYHAL_SDIO_DATA_BLOCK_SIZE_8B;
    }
    if (data_size > (uint32_t)_CYHAL_SDIO_DATA_BLOCK_SIZE_2B)
    {
        return _CYHAL_SDIO_DATA_BLOCK_SIZE_4B;
    }
    return _CYHAL_SDIO_DATA_BLOCK_SIZE_4B;
}


/***************************************************************************************************
 * _stm32_sdio_convert_block_size
 **************************************************************************************************/
static uint32_t _stm32_sdio_convert_block_size(uint16_t block_size)
{
    switch (block_size)
    {
        case 1:
            return SDMMC_DATABLOCK_SIZE_1B;

        case 2:
            return SDMMC_DATABLOCK_SIZE_2B;

        case 4:
            return SDMMC_DATABLOCK_SIZE_4B;

        case 8:
            return SDMMC_DATABLOCK_SIZE_8B;

        case 16:
            return SDMMC_DATABLOCK_SIZE_16B;

        case 32:
            return SDMMC_DATABLOCK_SIZE_32B;

        case 64:
            return SDMMC_DATABLOCK_SIZE_64B;

        case 128:
            return SDMMC_DATABLOCK_SIZE_128B;

        case 256:
            return SDMMC_DATABLOCK_SIZE_256B;

        case 512:
            return SDMMC_DATABLOCK_SIZE_512B;

        case 1024:
            return SDMMC_DATABLOCK_SIZE_1024B;

        case 2048:
            return SDMMC_DATABLOCK_SIZE_2048B;

        case 4096:
            return SDMMC_DATABLOCK_SIZE_4096B;

        case 8192:
            return SDMMC_DATABLOCK_SIZE_8192B;

        case 16384:
            return SDMMC_DATABLOCK_SIZE_16384B;

        default:
            assert_param(false); /* wrong value */
            return SDMMC_DATABLOCK_SIZE_512B;
    }
}


/***************************************************************************************************
 * _stm32_sdio_enable_irq
 **************************************************************************************************/
static void _stm32_sdio_enable_irq(const SD_TypeDef* instance, uint32_t priority, bool en_irq)
{
    IRQn_Type IRQn = (IRQn_Type)0;

    #if defined (SDMMC1)
    if (instance == SDMMC1)
    {
        IRQn = SDMMC1_IRQn;
    }
    else
    #endif

    #if defined (SDMMC2)
    if (instance == SDMMC2)
    {
        IRQn = SDMMC2_IRQn;
    }
    else
    #endif
    {
        assert_param(false); /* wrong instance */
    }

    if (en_irq)
    {
        HAL_NVIC_SetPriority(IRQn, priority, 0);
        HAL_NVIC_EnableIRQ(IRQn);
    }
    else
    {
        HAL_NVIC_DisableIRQ(IRQn);
    }
}


/***************************************************************************************************
 * _stm32_sdio_get_cmd_resp4
 **************************************************************************************************/
static uint32_t _stm32_sdio_get_cmd_resp4(SDMMC_TypeDef* SDMMCx)
{
    uint32_t sta_reg;
    /* 8 is the number of required instructions cycles for the below loop statement.
       The SDMMC_CMDTIMEOUT is expressed in ms */
    uint32_t count = SDMMC_CMDTIMEOUT * (SystemCoreClock / 8U /1000U);

    do
    {
        if (count-- == 0U)
        {
            return SDMMC_ERROR_TIMEOUT;
        }
        sta_reg = SDMMCx->STA;
    } while(((sta_reg & (SDMMC_FLAG_CCRCFAIL | SDMMC_FLAG_CMDREND | SDMMC_FLAG_CTIMEOUT)) == 0U) ||
            ((sta_reg & SDMMC_FLAG_CMDACT) != 0U));

    if (__SDMMC_GET_FLAG(SDMMCx, SDMMC_FLAG_CTIMEOUT))
    {
        __SDMMC_CLEAR_FLAG(SDMMCx, SDMMC_FLAG_CTIMEOUT);

        return SDMMC_ERROR_CMD_RSP_TIMEOUT;
    }
    else
    {
        /* Clear all the static flags */
        __SDMMC_CLEAR_FLAG(SDMMCx, SDMMC_STATIC_CMD_FLAGS);
    }

    return SDMMC_ERROR_NONE;
}


/***************************************************************************************************
 * _stm32_sdio_cmd_send_op_cond
 **************************************************************************************************/
static uint32_t _stm32_sdio_cmd_send_op_cond(SDMMC_TypeDef* SDMMCx)
{
    SDMMC_CmdInitTypeDef sdmmc_cmdinit;
    uint32_t             errorstate;

    sdmmc_cmdinit.Argument         = 0U;
    sdmmc_cmdinit.CmdIndex         = SDMMC_CMD_SDMMC_SEN_OP_COND;
    sdmmc_cmdinit.Response         = SDMMC_RESPONSE_SHORT;
    sdmmc_cmdinit.WaitForInterrupt = SDMMC_WAIT_NO;
    sdmmc_cmdinit.CPSM             = SDMMC_CPSM_ENABLE;
    (void)SDMMC_SendCommand(SDMMCx, &sdmmc_cmdinit);

    /* Check for error conditions */
    errorstate = _stm32_sdio_get_cmd_resp4(SDMMCx);

    return errorstate;
}


/***************************************************************************************************
 * _stm32_sdio_get_cmd_resp5
 **************************************************************************************************/
static uint32_t _stm32_sdio_get_cmd_resp5(SDMMC_TypeDef* SDMMCx, uint8_t SD_CMD, uint32_t* data)
{
    uint32_t response_r1;
    uint32_t sta_reg;

    /* 8 is the number of required instructions cycles for the below loop statement.
       The SDMMC_CMDTIMEOUT is expressed in ms */
    uint32_t count = SDMMC_CMDTIMEOUT * (SystemCoreClock / 8U /1000U);

    do
    {
        if (count-- == 0U)
        {
            return SDMMC_ERROR_TIMEOUT;
        }
        sta_reg = SDMMCx->STA;
    } while(((sta_reg & (SDMMC_FLAG_CCRCFAIL | SDMMC_FLAG_CMDREND | SDMMC_FLAG_CTIMEOUT)) == 0U) ||
            ((sta_reg & SDMMC_FLAG_CMDACT) != 0U));

    if (__SDMMC_GET_FLAG(SDMMCx, SDMMC_FLAG_CTIMEOUT))
    {
        __SDMMC_CLEAR_FLAG(SDMMCx, SDMMC_FLAG_CTIMEOUT);

        return SDMMC_ERROR_CMD_RSP_TIMEOUT;
    }
    else if (__SDMMC_GET_FLAG(SDMMCx, SDMMC_FLAG_CCRCFAIL))
    {
        __SDMMC_CLEAR_FLAG(SDMMCx, SDMMC_FLAG_CCRCFAIL);

        return SDMMC_ERROR_CMD_CRC_FAIL;
    }
    else
    {
        /* Nothing to do */
    }

    /* Check response received is of desired command */
    if (SDMMC_GetCommandResponse(SDMMCx) != SD_CMD)
    {
        return SDMMC_ERROR_CMD_CRC_FAIL;
    }

    /* Clear all the static flags */
    __SDMMC_CLEAR_FLAG(SDMMCx, SDMMC_STATIC_CMD_FLAGS);

    /* We have received response, retrieve it.  */
    response_r1 = SDMMC_GetResponse(SDMMCx, SDMMC_RESP1);

    if ((response_r1 & SDMMC_R5_ERRORBITS) == SDMMC_ALLZERO)
    {
        if (data != NULL)
        {
            *data = response_r1;
        }
        return SDMMC_ERROR_NONE;
    }
    else if ((response_r1 & SDMMC_R5_COM_CRC_ERROR) == SDMMC_R5_COM_CRC_ERROR)
    {
        return SDMMC_ERROR_COM_CRC_FAILED;
    }
    else if ((response_r1 & SDMMC_R5_ILLEGAL_COMMAND) == SDMMC_R5_ILLEGAL_COMMAND)
    {
        return SDMMC_ERROR_ILLEGAL_CMD;
    }
    else if ((response_r1 & SDMMC_R5_ERROR) == SDMMC_R5_ERROR)
    {
        return SDMMC_ERROR_GENERAL_UNKNOWN_ERR;
    }
    else if ((response_r1 & SDMMC_R5_FUNCTION_NUMBER) == SDMMC_R5_FUNCTION_NUMBER)
    {
        return SDMMC_ERROR_INVALID_PARAMETER;
    }
    else if ((response_r1 & SDMMC_R5_OUT_OF_RANGE) == SDMMC_R5_OUT_OF_RANGE)
    {
        return SDMMC_ERROR_ADDR_OUT_OF_RANGE;
    }
    else
    {
        /* Nothing to do */
    }

    /* Should not get here, but this is needed to make the compiler happy */
    return SDMMC_ERROR_GENERAL_UNKNOWN_ERR;
}


/***************************************************************************************************
 * _stm32_sdio_cmd_rw_direct
 **************************************************************************************************/
static uint32_t _stm32_sdio_cmd_rw_direct(SDMMC_TypeDef* SDMMCx, uint32_t argument,
                                          uint32_t* response)
{
    SDMMC_CmdInitTypeDef sdmmc_cmdinit;
    uint32_t             errorstate;

    sdmmc_cmdinit.Argument         = argument;
    sdmmc_cmdinit.CmdIndex         = SDMMC_CMD_SDMMC_RW_DIRECT;
    sdmmc_cmdinit.Response         = SDMMC_RESPONSE_SHORT;
    sdmmc_cmdinit.WaitForInterrupt = SDMMC_WAIT_NO;
    sdmmc_cmdinit.CPSM             = SDMMC_CPSM_ENABLE;
    (void)SDMMC_SendCommand(SDMMCx, &sdmmc_cmdinit);

    /* Check for error conditions */
    errorstate = _stm32_sdio_get_cmd_resp5(SDMMCx, SDMMC_CMD_SDMMC_RW_DIRECT, response);
    return errorstate;
}


/***************************************************************************************************
 * _stm32_sdio_cmd_rw_extended
 **************************************************************************************************/
static uint32_t _stm32_sdio_cmd_rw_extended(SDMMC_TypeDef* SDMMCx, uint32_t argument,
                                            uint32_t* response)
{
    SDMMC_CmdInitTypeDef sdmmc_cmdinit;
    uint32_t             errorstate;

    sdmmc_cmdinit.Argument         = argument;
    sdmmc_cmdinit.CmdIndex         = SDMMC_CMD_SDMMC_RW_EXTENDED;
    sdmmc_cmdinit.Response         = SDMMC_RESPONSE_SHORT;
    sdmmc_cmdinit.WaitForInterrupt = SDMMC_WAIT_NO;
    sdmmc_cmdinit.CPSM             = SDMMC_CPSM_ENABLE;
    (void)SDMMC_SendCommand(SDMMCx, &sdmmc_cmdinit);

    /* Check for error conditions */
    errorstate = _stm32_sdio_get_cmd_resp5(SDMMCx, SDMMC_CMD_SDMMC_RW_EXTENDED, response);
    return errorstate;
}


#if defined(__cplusplus)
}
#endif

#endif /* defined(HAL_SDMMM_MODULE_ENABLED) */
