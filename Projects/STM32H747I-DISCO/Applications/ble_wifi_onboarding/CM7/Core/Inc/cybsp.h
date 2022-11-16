/***********************************************************************************************//**
 * \file cybsp.h
 *
 * \brief
 * Basic API for setting up boards containing a STM32 MCU.
 *
 ***************************************************************************************************
 * \copyright
 * Copyright 2018-2020 Cypress Semiconductor Corporation
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 **************************************************************************************************/

#pragma once

/** These names are explicitly referenced in the support libraries */
#define  CYBSP_WIFI_WL_REG_ON             PC6
#define  CYBSP_WIFI_HOST_WAKE             PJ13


/* This are used by the bluetooth-freertos library */
#define  CYBSP_BT_POWER                   PD13
#define  CYBSP_BT_DEVICE_WAKE             NC
#define  CYBSP_BT_HOST_WAKE               NC

#define  CYBSP_BT_UART_TX                 PA12
#define  CYBSP_BT_UART_RX                 PA11
#define  CYBSP_BT_UART_RTS                PB14
#define  CYBSP_BT_UART_CTS                PB15

#define  CYBSP_WIFI_HOST_WAKE_IRQ_EVENT   (CYHAL_GPIO_IRQ_RISE)
#define  CYHAL_ISR_PRIORITY_DEFAULT       (7)


/** User button */
#define  CYBSP_USER_BTN                   PK2

#define CYHAL_UART_MAX_INSTANCES          (3)
