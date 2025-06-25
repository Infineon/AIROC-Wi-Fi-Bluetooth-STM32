/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    custom_rng.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    20-June-2017
  * @brief   mbedtls alternate entropy data function skeleton to be completed by the user.
  *
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

#include "mbedtls.h"

#ifdef MBEDTLS_ENTROPY_HARDWARE_ALT

#include "main.h"
#include "string.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_rng.h"
#include "mbedtls/entropy_poll.h"

extern RNG_HandleTypeDef hrng;

int mbedtls_hardware_poll( void *Data, unsigned char *Output, size_t Len, size_t *oLen )
{
/* USER CODE BEGIN custom_rng */
	uint32_t index;
	uint32_t randomValue;

	for (index = 0; index < Len/4; index++)
	{
		if (HAL_RNG_GenerateRandomNumber(&hrng, &randomValue) == HAL_OK)
		{
			*oLen += 4;
			memset(&(Output[index * 4]), (int)randomValue, 4);
		}
		else
		{
			Error_Handler();
		}
	}
	return 0;
/* USER CODE END custom_rng */
}

#endif /*MBEDTLS_ENTROPY_HARDWARE_ALT*/
