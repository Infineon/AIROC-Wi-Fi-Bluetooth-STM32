/**
  ******************************************************************************
  * File Name          : cybsp.h
  * Description        : This file provides code for the configuration
  *                      of the cybsp.h instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CYBSP__H__
#define __CYBSP__H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Defines ------------------------------------------------------------------*/
#define false 0
#define true 1

/* WIFI interface types */
#define CYBSP_SDIO_INTERFACE    (0)
#define CYBSP_SPI_INTERFACE     (1)
#define CYBSP_M2M_INTERFACE     (2)
#define CYBSP_USB_INTERFACE     (3)

/**
	MiddleWare name : Infineon.AIROC-Wi-Fi-Bluetooth-STM32.1.6.1
	MiddleWare fileName : cybsp.h
	MiddleWare version :
*/
/*----------   -----------*/
#define CYBSP_BT_UART_CTS      PD3

/*----------   -----------*/
#define CY_WIFI_COUNTRY_CUSTOM      WHD_COUNTRY_XX

/*----------   -----------*/
#define CYBSP_BT_UART_TX      PD5

/*----------   -----------*/
#define CYBSP_USER_BTN      PC13

/*----------   -----------*/
#define COUNTRY_CUSTOM_ENABLE      false

/*----------   -----------*/
#define CYBSP_BT_POWER      PG8

/*----------   -----------*/
#define COMMAND_CONSOLE_UART_TX      NC

/*----------   -----------*/
#define COMMAND_CONSOLE_UART_RX      NC

/*----------   -----------*/
#define CYBSP_WIFI_HOST_WAKE      NC

/*----------   -----------*/
#define CYHAL_ISR_PRIORITY_DEFAULT      7

/*----------   -----------*/
#define SDMMC_CLK_FREQ_OVERRIDE      0

/*----------   -----------*/
#define CYBSP_WIFI_WL_REG_ON      PD0

/*----------   -----------*/
#define CYBSP_BT_DEVICE_WAKE      NC

/*----------   -----------*/
#define CYBSP_BT_HOST_WAKE      NC

/*----------   -----------*/
#define CYBSP_WIFI_HOST_WAKE_IRQ_EVENT      CYHAL_GPIO_IRQ_RISE

/*----------   -----------*/
#define CYHAL_UART_MAX_INSTANCES      3

/*----------   -----------*/
#define CYBSP_BT_UART_RX      PD6

/*----------   -----------*/
#define CY_WIFI_COUNTRY      WHD_COUNTRY_UNITED_STATES

/*----------   -----------*/
#define CYBSP_BT_UART_RTS      PD4

/*----------   -----------*/
#define _CYHAL_SDIO_DMA_BUFFER_SIZE      0

#if COUNTRY_CUSTOM_ENABLE == 1
#if defined(CY_WIFI_COUNTRY)
#undef CY_WIFI_COUNTRY
#define CY_WIFI_COUNTRY CY_WIFI_COUNTRY_CUSTOM
#endif
#endif

#if !defined(CYBSP_WIFI_INTERFACE_TYPE)
#define CYBSP_WIFI_INTERFACE_TYPE 	(CYBSP_SDIO_INTERFACE)
#endif

#ifdef __cplusplus
}
#endif
#endif /*__ CYBSP__H_H */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
