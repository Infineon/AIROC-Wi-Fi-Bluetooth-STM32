/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l5xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32l562e_discovery_ospi.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
/* Exported macro -----------------------------------------------------*/
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void MX_SDMMC1_SD_Init(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LCD_BL_PWM_Pin GPIO_PIN_1
#define LCD_BL_PWM_GPIO_Port GPIOE
#define LED_GREEN_Pin GPIO_PIN_12
#define LED_GREEN_GPIO_Port GPIOG
#define SDIO_CMD_Pin GPIO_PIN_2
#define SDIO_CMD_GPIO_Port GPIOD
#define SDIO_D3_Pin GPIO_PIN_11
#define SDIO_D3_GPIO_Port GPIOC
#define SDIO_CLK_Pin GPIO_PIN_12
#define SDIO_CLK_GPIO_Port GPIOC
#define SDIO_D2_Pin GPIO_PIN_10
#define SDIO_D2_GPIO_Port GPIOC
#define USER_BUTTON_Pin GPIO_PIN_13
#define USER_BUTTON_GPIO_Port GPIOC
#define LED_RED_Pin GPIO_PIN_3
#define LED_RED_GPIO_Port GPIOD
#define GYRO_ACC_INT_Pin GPIO_PIN_3
#define GYRO_ACC_INT_GPIO_Port GPIOF
#define UCPD_DBn_Pin GPIO_PIN_5
#define UCPD_DBn_GPIO_Port GPIOB
#define USART1_TX_Pin GPIO_PIN_9
#define USART1_TX_GPIO_Port GPIOA
#define USART1_RX_Pin GPIO_PIN_10
#define USART1_RX_GPIO_Port GPIOA
#define LCD_TE_Pin GPIO_PIN_8
#define LCD_TE_GPIO_Port GPIOA
#define SDIO_DETECT_Pin GPIO_PIN_2
#define SDIO_DETECT_GPIO_Port GPIOF
#define SDIO_D1_Pin GPIO_PIN_9
#define SDIO_D1_GPIO_Port GPIOC
#define SDIO_D0_Pin GPIO_PIN_8
#define SDIO_D0_GPIO_Port GPIOC
#define LCD_PWR_ON_Pin GPIO_PIN_0
#define LCD_PWR_ON_GPIO_Port GPIOH
#define WIFI_WL_REG_ON_Pin GPIO_PIN_5
#define WIFI_WL_REG_ON_GPIO_Port GPIOF
#define BLE_INT_Pin GPIO_PIN_6
#define BLE_INT_GPIO_Port GPIOG
#define BLE_RSTN_Pin GPIO_PIN_8
#define BLE_RSTN_GPIO_Port GPIOG
#define BLE_CSN_Pin GPIO_PIN_5
#define BLE_CSN_GPIO_Port GPIOG
#define LCD_RST_Pin GPIO_PIN_14
#define LCD_RST_GPIO_Port GPIOF
#define STMOD_SEL_12_Pin GPIO_PIN_11
#define STMOD_SEL_12_GPIO_Port GPIOF
#define AUDIO_RESETN_Pin GPIO_PIN_1
#define AUDIO_RESETN_GPIO_Port GPIOG
#define WIFI_HOST_WAKE_Pin GPIO_PIN_13
#define WIFI_HOST_WAKE_GPIO_Port GPIOB
#define WIFI_HOST_WAKE_EXTI_IRQn EXTI13_IRQn
#define UCPD_FLT_Pin GPIO_PIN_14
#define UCPD_FLT_GPIO_Port GPIOB
#define VBUS_SENSE_Pin GPIO_PIN_4
#define VBUS_SENSE_GPIO_Port GPIOA
#define STMOD_SEL_34_Pin GPIO_PIN_12
#define STMOD_SEL_34_GPIO_Port GPIOF
#define CTP_RST_Pin GPIO_PIN_15
#define CTP_RST_GPIO_Port GPIOF

/* USER CODE BEGIN Private defines */
#define CY_STORAGE_WIFI_DATA ".whd_fw"
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
