/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32u5xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cy_tls.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

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
#define GYRO_ACC_INT_Pin GPIO_PIN_15
#define GYRO_ACC_INT_GPIO_Port GPIOH
#define CYBSP_WIFI_WL_REG_ON_Pin GPIO_PIN_4
#define CYBSP_WIFI_WL_REG_ON_GPIO_Port GPIOB
#define SDCARD_DETECT_Pin GPIO_PIN_0
#define SDCARD_DETECT_GPIO_Port GPIOI
#define CAMERA_PLUG_Pin GPIO_PIN_10
#define CAMERA_PLUG_GPIO_Port GPIOG
#define CAMERA_XSDN_Pin GPIO_PIN_3
#define CAMERA_XSDN_GPIO_Port GPIOI
#define CAMERA_RSTI_Pin GPIO_PIN_2
#define CAMERA_RSTI_GPIO_Port GPIOI
#define USER_BUTTON_Pin GPIO_PIN_13
#define USER_BUTTON_GPIO_Port GPIOC
#define FMC_A22_LCD_RS_Pin GPIO_PIN_6
#define FMC_A22_LCD_RS_GPIO_Port GPIOE
#define UCPD_FLT_Pin GPIO_PIN_8
#define UCPD_FLT_GPIO_Port GPIOB
#define LED_GREEN_Pin GPIO_PIN_7
#define LED_GREEN_GPIO_Port GPIOB
#define LED_RED_Pin GPIO_PIN_7
#define LED_RED_GPIO_Port GPIOH
#define LCD_BL_CTRL_Pin GPIO_PIN_8
#define LCD_BL_CTRL_GPIO_Port GPIOA
#define IBUS_SENSE_Pin GPIO_PIN_0
#define IBUS_SENSE_GPIO_Port GPIOA
#define VBUS_SENSE_Pin GPIO_PIN_5
#define VBUS_SENSE_GPIO_Port GPIOA
#define CYBSP_BT_POWER_Pin GPIO_PIN_11
#define CYBSP_BT_POWER_GPIO_Port GPIOB
#define MFX_IRQ_OUT_Pin GPIO_PIN_5
#define MFX_IRQ_OUT_GPIO_Port GPIOC
#define MFX_WAKEUP_Pin GPIO_PIN_11
#define MFX_WAKEUP_GPIO_Port GPIOF
#define WIFI_WL_REG_ON_Pin GPIO_PIN_4
#define WIFI_WL_REG_ON_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
