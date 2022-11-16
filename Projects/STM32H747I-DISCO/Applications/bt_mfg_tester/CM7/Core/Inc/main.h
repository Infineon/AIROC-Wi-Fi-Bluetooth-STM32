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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */


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

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CYBSP_BT_UART_TX_Pin GPIO_PIN_12
#define CYBSP_BT_UART_TX_GPIO_Port GPIOA
#define CYBSP_BT_UART_RX_Pin GPIO_PIN_11
#define CYBSP_BT_UART_RX_GPIO_Port GPIOA
#define WIFI_WL_REG_ON_Pin GPIO_PIN_6
#define WIFI_WL_REG_ON_GPIO_Port GPIOC
#define CYBSP_BT_POWER_Pin GPIO_PIN_13
#define CYBSP_BT_POWER_GPIO_Port GPIOD
#define CYBSP_BT_UART_CTS_Pin GPIO_PIN_15
#define CYBSP_BT_UART_CTS_GPIO_Port GPIOB
#define CYBSP_BT_UART_RTS_Pin GPIO_PIN_14
#define CYBSP_BT_UART_RTS_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
