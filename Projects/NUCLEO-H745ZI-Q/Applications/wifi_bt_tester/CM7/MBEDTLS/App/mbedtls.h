/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : mbedtls.h
  * Description        : This file provides code for the configuration
  *                      of the mbedtls instances.
  ******************************************************************************
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
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __mbedtls_H
#define __mbedtls_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "mbedtls_config.h"
#include "mbedtls/ssl.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/debug.h"
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* Global variables ---------------------------------------------------------*/

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/* MBEDTLS init function */
void MX_MBEDTLS_Init(void);

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

#ifdef __cplusplus
}
#endif
#endif /*__mbedtls_H */

/**
  * @}
  */

/**
  * @}
  */
