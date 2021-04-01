/***************************************************************************//**
* \file cyhal_spi.c
*
* \brief
* Implementation for SPI abstraction
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
#include "cyhal_gpio.h"
#include "stm32_cyhal_gpio_ex.h"
#include "cyhal_spi.h"
#include "stm32_cyhal_spi_ex.h"
#include "stm32_cyhal_common.h"


#if defined(HAL_SPI_MODULE_ENABLED)

/*******************************************************************************
*      Private globals
*******************************************************************************/

static SPI_HandleTypeDef* wifi_spi;
static GPIO_TypeDef*      spi_cs_gpio_port = NULL;
static uint16_t           spi_cs_gpio_pin;


//--------------------------------------------------------------------------------------------------
// _stm32_cyhal_spi_cs_init
//--------------------------------------------------------------------------------------------------
void stm32_cypal_spi_hw_init(SPI_HandleTypeDef* hspi, cyhal_gpio_t spi_cs)
{
    wifi_spi = hspi;

    if (spi_cs != NC)
    {
        spi_cs_gpio_port = CYHAL_GET_PORT(spi_cs);
        spi_cs_gpio_pin  = CYHAL_GET_PIN(spi_cs);

        cyhal_gpio_init(spi_cs, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_OPENDRAINDRIVESHIGH, true);
    }
}


//--------------------------------------------------------------------------------------------------
// cyhal_spi_transfer
//--------------------------------------------------------------------------------------------------
cy_rslt_t cyhal_spi_transfer(cyhal_spi_t* obj, const uint8_t* tx, size_t tx_length, uint8_t* rx,
                             size_t rx_length, uint8_t write_fill)
{
    (void)(obj);

    if ((tx == NULL) && (rx == NULL))
    {
        return CY_RSLT_SUCCESS;
    }

    if (spi_cs_gpio_port != NULL)
    {
        HAL_GPIO_WritePin(spi_cs_gpio_port, spi_cs_gpio_pin, GPIO_PIN_RESET);
    }

    if ((tx != NULL) && (rx != NULL))
    {
        if (tx_length != rx_length)
        {
            // NOTE:  WPRINT_WHD_INFO(("write_fill not currently supported\n"));
        }
        return HAL_SPI_TransmitReceive_DMA(wifi_spi, (uint8_t*)tx, (uint8_t*)rx,
                                           (tx_length > rx_length) ? tx_length : rx_length);
    }
    else if (tx != NULL)
    {
        return HAL_SPI_Transmit_DMA(wifi_spi, (uint8_t*)tx, tx_length);
    }
    else if (rx != NULL)  /* currently do one at a time */
    {
        return HAL_SPI_Receive_DMA(wifi_spi, (uint8_t*)rx, rx_length);
    }

    return CY_RSLT_SUCCESS;
}


//--------------------------------------------------------------------------------------------------
// spi_transfer_complete
//--------------------------------------------------------------------------------------------------
static void spi_transfer_complete(void)
{
    if (spi_cs_gpio_port != NULL)
    {
        HAL_GPIO_WritePin(spi_cs_gpio_port, spi_cs_gpio_pin, GPIO_PIN_SET);
    }
}


//--------------------------------------------------------------------------------------------------
// HAL_SPI_RxCpltCallback
//--------------------------------------------------------------------------------------------------
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef* hspi)
{
    spi_transfer_complete();
}


//--------------------------------------------------------------------------------------------------
// HAL_SPI_TxCpltCallback
//--------------------------------------------------------------------------------------------------
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi)
{
    spi_transfer_complete();
}


//--------------------------------------------------------------------------------------------------
// HAL_SPI_TxRxCpltCallback
//--------------------------------------------------------------------------------------------------
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi)
{
    spi_transfer_complete();
}


// TODO: rework callback usage. Need to use HAL_SPI_RegisterCallback, instead of Legacy weak SPI
//       callback functions.
//--------------------------------------------------------------------------------------------------
// HAL_SPI_ErrorCallback
//--------------------------------------------------------------------------------------------------
/*
   void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi)
   {
   }
 */


#endif /* defined(HAL_SPI_MODULE_ENABLED) */
