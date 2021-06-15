/***********************************************************************************************//**
 * \file stm32_cyhal_gpio_pin.h
 *
 * \brief
 * Provides a high level interface for interacting with the GPIO on Cypress devices.
 * This interface abstracts out the chip specific details. If any chip specific
 * functionality is necessary, or performance is critical the low level functions
 * can be used directly.
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

#pragma once

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

#include "stm32_cyhal_common.h"

/** Gets a pin definition from the provided port and pin numbers */
#define CYHAL_GET_GPIO(port, pin) \
    ((((uint32_t)GPIO_GET_INDEX(port)) << 16U) | ((uint32_t)(pin)))

/** Definitions for all of the pins that are bonded out on in the STM32 devices. */
typedef enum
{
    NC   = (uint32_t) 0x0FFFFFFF,              //!< No Connect/Invalid Pin

    #if defined(GPIOA)
    PA0  = CYHAL_GET_GPIO(GPIOA, GPIO_PIN_0),  //!< Port A Pin 0
    PA1  = CYHAL_GET_GPIO(GPIOA, GPIO_PIN_1),  //!< Port A Pin 1
    PA2  = CYHAL_GET_GPIO(GPIOA, GPIO_PIN_2),  //!< Port A Pin 2
    PA3  = CYHAL_GET_GPIO(GPIOA, GPIO_PIN_3),  //!< Port A Pin 3
    PA4  = CYHAL_GET_GPIO(GPIOA, GPIO_PIN_4),  //!< Port A Pin 4
    PA5  = CYHAL_GET_GPIO(GPIOA, GPIO_PIN_5),  //!< Port A Pin 5
    PA6  = CYHAL_GET_GPIO(GPIOA, GPIO_PIN_6),  //!< Port A Pin 6
    PA7  = CYHAL_GET_GPIO(GPIOA, GPIO_PIN_7),  //!< Port A Pin 7
    PA8  = CYHAL_GET_GPIO(GPIOA, GPIO_PIN_8),  //!< Port A Pin 8
    PA9  = CYHAL_GET_GPIO(GPIOA, GPIO_PIN_9),  //!< Port A Pin 9
    PA10 = CYHAL_GET_GPIO(GPIOA, GPIO_PIN_10), //!< Port A Pin 10
    PA11 = CYHAL_GET_GPIO(GPIOA, GPIO_PIN_11), //!< Port A Pin 11
    PA12 = CYHAL_GET_GPIO(GPIOA, GPIO_PIN_12), //!< Port A Pin 12
    PA13 = CYHAL_GET_GPIO(GPIOA, GPIO_PIN_13), //!< Port A Pin 13
    PA14 = CYHAL_GET_GPIO(GPIOA, GPIO_PIN_14), //!< Port A Pin 14
    PA15 = CYHAL_GET_GPIO(GPIOA, GPIO_PIN_15), //!< Port A Pin 15
    #endif // if defined(GPIOA)
    #if defined(GPIOB)
    PB0  = CYHAL_GET_GPIO(GPIOB, GPIO_PIN_0),  //!< Port B Pin 0
    PB1  = CYHAL_GET_GPIO(GPIOB, GPIO_PIN_1),  //!< Port B Pin 1
    PB2  = CYHAL_GET_GPIO(GPIOB, GPIO_PIN_2),  //!< Port B Pin 2
    PB3  = CYHAL_GET_GPIO(GPIOB, GPIO_PIN_3),  //!< Port B Pin 3
    PB4  = CYHAL_GET_GPIO(GPIOB, GPIO_PIN_4),  //!< Port B Pin 4
    PB5  = CYHAL_GET_GPIO(GPIOB, GPIO_PIN_5),  //!< Port B Pin 5
    PB6  = CYHAL_GET_GPIO(GPIOB, GPIO_PIN_6),  //!< Port B Pin 6
    PB7  = CYHAL_GET_GPIO(GPIOB, GPIO_PIN_7),  //!< Port B Pin 7
    PB8  = CYHAL_GET_GPIO(GPIOB, GPIO_PIN_8),  //!< Port B Pin 8
    PB9  = CYHAL_GET_GPIO(GPIOB, GPIO_PIN_9),  //!< Port B Pin 9
    PB10 = CYHAL_GET_GPIO(GPIOB, GPIO_PIN_10), //!< Port B Pin 10
    PB11 = CYHAL_GET_GPIO(GPIOB, GPIO_PIN_11), //!< Port B Pin 11
    PB12 = CYHAL_GET_GPIO(GPIOB, GPIO_PIN_12), //!< Port B Pin 12
    PB13 = CYHAL_GET_GPIO(GPIOB, GPIO_PIN_13), //!< Port B Pin 13
    PB14 = CYHAL_GET_GPIO(GPIOB, GPIO_PIN_14), //!< Port B Pin 14
    PB15 = CYHAL_GET_GPIO(GPIOB, GPIO_PIN_15), //!< Port B Pin 15
    #endif // if defined(GPIOB)
    #if defined(GPIOC)
    PC0  = CYHAL_GET_GPIO(GPIOC, GPIO_PIN_0),  //!< Port C Pin 0
    PC1  = CYHAL_GET_GPIO(GPIOC, GPIO_PIN_1),  //!< Port C Pin 1
    PC2  = CYHAL_GET_GPIO(GPIOC, GPIO_PIN_2),  //!< Port C Pin 2
    PC3  = CYHAL_GET_GPIO(GPIOC, GPIO_PIN_3),  //!< Port C Pin 3
    PC4  = CYHAL_GET_GPIO(GPIOC, GPIO_PIN_4),  //!< Port C Pin 4
    PC5  = CYHAL_GET_GPIO(GPIOC, GPIO_PIN_5),  //!< Port C Pin 5
    PC6  = CYHAL_GET_GPIO(GPIOC, GPIO_PIN_6),  //!< Port C Pin 6
    PC7  = CYHAL_GET_GPIO(GPIOC, GPIO_PIN_7),  //!< Port C Pin 7
    PC8  = CYHAL_GET_GPIO(GPIOC, GPIO_PIN_8),  //!< Port C Pin 8
    PC9  = CYHAL_GET_GPIO(GPIOC, GPIO_PIN_9),  //!< Port C Pin 9
    PC10 = CYHAL_GET_GPIO(GPIOC, GPIO_PIN_10), //!< Port C Pin 10
    PC11 = CYHAL_GET_GPIO(GPIOC, GPIO_PIN_11), //!< Port C Pin 11
    PC12 = CYHAL_GET_GPIO(GPIOC, GPIO_PIN_12), //!< Port C Pin 12
    PC13 = CYHAL_GET_GPIO(GPIOC, GPIO_PIN_13), //!< Port C Pin 13
    PC14 = CYHAL_GET_GPIO(GPIOC, GPIO_PIN_14), //!< Port C Pin 14
    PC15 = CYHAL_GET_GPIO(GPIOC, GPIO_PIN_15), //!< Port C Pin 15
    #endif // if defined(GPIOC)
    #if defined(GPIOD)
    PD0  = CYHAL_GET_GPIO(GPIOD, GPIO_PIN_0),  //!< Port D Pin 0
    PD1  = CYHAL_GET_GPIO(GPIOD, GPIO_PIN_1),  //!< Port D Pin 1
    PD2  = CYHAL_GET_GPIO(GPIOD, GPIO_PIN_2),  //!< Port D Pin 2
    PD3  = CYHAL_GET_GPIO(GPIOD, GPIO_PIN_3),  //!< Port D Pin 3
    PD4  = CYHAL_GET_GPIO(GPIOD, GPIO_PIN_4),  //!< Port D Pin 4
    PD5  = CYHAL_GET_GPIO(GPIOD, GPIO_PIN_5),  //!< Port D Pin 5
    PD6  = CYHAL_GET_GPIO(GPIOD, GPIO_PIN_6),  //!< Port D Pin 6
    PD7  = CYHAL_GET_GPIO(GPIOD, GPIO_PIN_7),  //!< Port D Pin 7
    PD8  = CYHAL_GET_GPIO(GPIOD, GPIO_PIN_8),  //!< Port D Pin 8
    PD9  = CYHAL_GET_GPIO(GPIOD, GPIO_PIN_9),  //!< Port D Pin 9
    PD10 = CYHAL_GET_GPIO(GPIOD, GPIO_PIN_10), //!< Port D Pin 10
    PD11 = CYHAL_GET_GPIO(GPIOD, GPIO_PIN_11), //!< Port D Pin 11
    PD12 = CYHAL_GET_GPIO(GPIOD, GPIO_PIN_12), //!< Port D Pin 12
    PD13 = CYHAL_GET_GPIO(GPIOD, GPIO_PIN_13), //!< Port D Pin 13
    PD14 = CYHAL_GET_GPIO(GPIOD, GPIO_PIN_14), //!< Port D Pin 14
    PD15 = CYHAL_GET_GPIO(GPIOD, GPIO_PIN_15), //!< Port D Pin 15
    #endif // if defined(GPIOD)
    #if defined(GPIOE)
    PE0  = CYHAL_GET_GPIO(GPIOE, GPIO_PIN_0),  //!< Port E Pin 0
    PE1  = CYHAL_GET_GPIO(GPIOE, GPIO_PIN_1),  //!< Port E Pin 1
    PE2  = CYHAL_GET_GPIO(GPIOE, GPIO_PIN_2),  //!< Port E Pin 2
    PE3  = CYHAL_GET_GPIO(GPIOE, GPIO_PIN_3),  //!< Port E Pin 3
    PE4  = CYHAL_GET_GPIO(GPIOE, GPIO_PIN_4),  //!< Port E Pin 4
    PE5  = CYHAL_GET_GPIO(GPIOE, GPIO_PIN_5),  //!< Port E Pin 5
    PE6  = CYHAL_GET_GPIO(GPIOE, GPIO_PIN_6),  //!< Port E Pin 6
    PE7  = CYHAL_GET_GPIO(GPIOE, GPIO_PIN_7),  //!< Port E Pin 7
    PE8  = CYHAL_GET_GPIO(GPIOE, GPIO_PIN_8),  //!< Port E Pin 8
    PE9  = CYHAL_GET_GPIO(GPIOE, GPIO_PIN_9),  //!< Port E Pin 9
    PE10 = CYHAL_GET_GPIO(GPIOE, GPIO_PIN_10), //!< Port E Pin 10
    PE11 = CYHAL_GET_GPIO(GPIOE, GPIO_PIN_11), //!< Port E Pin 11
    PE12 = CYHAL_GET_GPIO(GPIOE, GPIO_PIN_12), //!< Port E Pin 12
    PE13 = CYHAL_GET_GPIO(GPIOE, GPIO_PIN_13), //!< Port E Pin 13
    PE14 = CYHAL_GET_GPIO(GPIOE, GPIO_PIN_14), //!< Port E Pin 14
    PE15 = CYHAL_GET_GPIO(GPIOE, GPIO_PIN_15), //!< Port E Pin 15
    #endif // if defined(GPIOE)
    #if defined(GPIOF)
    PF0  = CYHAL_GET_GPIO(GPIOF, GPIO_PIN_0),  //!< Port F Pin 0
    PF1  = CYHAL_GET_GPIO(GPIOF, GPIO_PIN_1),  //!< Port F Pin 1
    PF2  = CYHAL_GET_GPIO(GPIOF, GPIO_PIN_2),  //!< Port F Pin 2
    PF3  = CYHAL_GET_GPIO(GPIOF, GPIO_PIN_3),  //!< Port F Pin 3
    PF4  = CYHAL_GET_GPIO(GPIOF, GPIO_PIN_4),  //!< Port F Pin 4
    PF5  = CYHAL_GET_GPIO(GPIOF, GPIO_PIN_5),  //!< Port F Pin 5
    PF6  = CYHAL_GET_GPIO(GPIOF, GPIO_PIN_6),  //!< Port F Pin 6
    PF7  = CYHAL_GET_GPIO(GPIOF, GPIO_PIN_7),  //!< Port F Pin 7
    PF8  = CYHAL_GET_GPIO(GPIOF, GPIO_PIN_8),  //!< Port F Pin 8
    PF9  = CYHAL_GET_GPIO(GPIOF, GPIO_PIN_9),  //!< Port F Pin 9
    PF10 = CYHAL_GET_GPIO(GPIOF, GPIO_PIN_10), //!< Port F Pin 10
    PF11 = CYHAL_GET_GPIO(GPIOF, GPIO_PIN_11), //!< Port F Pin 11
    PF12 = CYHAL_GET_GPIO(GPIOF, GPIO_PIN_12), //!< Port F Pin 12
    PF13 = CYHAL_GET_GPIO(GPIOF, GPIO_PIN_13), //!< Port F Pin 13
    PF14 = CYHAL_GET_GPIO(GPIOF, GPIO_PIN_14), //!< Port F Pin 14
    PF15 = CYHAL_GET_GPIO(GPIOF, GPIO_PIN_15), //!< Port F Pin 15
    #endif // if defined(GPIOF)
    #if defined(GPIOG)
    PG0  = CYHAL_GET_GPIO(GPIOG, GPIO_PIN_0),  //!< Port G Pin 0
    PG1  = CYHAL_GET_GPIO(GPIOG, GPIO_PIN_1),  //!< Port G Pin 1
    PG2  = CYHAL_GET_GPIO(GPIOG, GPIO_PIN_2),  //!< Port G Pin 2
    PG3  = CYHAL_GET_GPIO(GPIOG, GPIO_PIN_3),  //!< Port G Pin 3
    PG4  = CYHAL_GET_GPIO(GPIOG, GPIO_PIN_4),  //!< Port G Pin 4
    PG5  = CYHAL_GET_GPIO(GPIOG, GPIO_PIN_5),  //!< Port G Pin 5
    PG6  = CYHAL_GET_GPIO(GPIOG, GPIO_PIN_6),  //!< Port G Pin 6
    PG7  = CYHAL_GET_GPIO(GPIOG, GPIO_PIN_7),  //!< Port G Pin 7
    PG8  = CYHAL_GET_GPIO(GPIOG, GPIO_PIN_8),  //!< Port G Pin 8
    PG9  = CYHAL_GET_GPIO(GPIOG, GPIO_PIN_9),  //!< Port G Pin 9
    PG10 = CYHAL_GET_GPIO(GPIOG, GPIO_PIN_10), //!< Port G Pin 10
    PG11 = CYHAL_GET_GPIO(GPIOG, GPIO_PIN_11), //!< Port G Pin 11
    PG12 = CYHAL_GET_GPIO(GPIOG, GPIO_PIN_12), //!< Port G Pin 12
    PG13 = CYHAL_GET_GPIO(GPIOG, GPIO_PIN_13), //!< Port G Pin 13
    PG14 = CYHAL_GET_GPIO(GPIOG, GPIO_PIN_14), //!< Port G Pin 14
    PG15 = CYHAL_GET_GPIO(GPIOG, GPIO_PIN_15), //!< Port G Pin 15
    #endif // if defined(GPIOG)
    #if defined(GPIOH)
    PH0  = CYHAL_GET_GPIO(GPIOH, GPIO_PIN_0),  //!< Port H Pin 0
    PH1  = CYHAL_GET_GPIO(GPIOH, GPIO_PIN_1),  //!< Port H Pin 1
    PH2  = CYHAL_GET_GPIO(GPIOH, GPIO_PIN_2),  //!< Port H Pin 2
    PH3  = CYHAL_GET_GPIO(GPIOH, GPIO_PIN_3),  //!< Port H Pin 3
    PH4  = CYHAL_GET_GPIO(GPIOH, GPIO_PIN_4),  //!< Port H Pin 4
    PH5  = CYHAL_GET_GPIO(GPIOH, GPIO_PIN_5),  //!< Port H Pin 5
    PH6  = CYHAL_GET_GPIO(GPIOH, GPIO_PIN_6),  //!< Port H Pin 6
    PH7  = CYHAL_GET_GPIO(GPIOH, GPIO_PIN_7),  //!< Port H Pin 7
    PH8  = CYHAL_GET_GPIO(GPIOH, GPIO_PIN_8),  //!< Port H Pin 8
    PH9  = CYHAL_GET_GPIO(GPIOH, GPIO_PIN_9),  //!< Port H Pin 9
    PH10 = CYHAL_GET_GPIO(GPIOH, GPIO_PIN_10), //!< Port H Pin 10
    PH11 = CYHAL_GET_GPIO(GPIOH, GPIO_PIN_11), //!< Port H Pin 11
    PH12 = CYHAL_GET_GPIO(GPIOH, GPIO_PIN_12), //!< Port H Pin 12
    PH13 = CYHAL_GET_GPIO(GPIOH, GPIO_PIN_13), //!< Port H Pin 13
    PH14 = CYHAL_GET_GPIO(GPIOH, GPIO_PIN_14), //!< Port H Pin 14
    PH15 = CYHAL_GET_GPIO(GPIOH, GPIO_PIN_15), //!< Port H Pin 15
    #endif // if defined(GPIOH)
    #if defined(GPIOI)
    PI0  = CYHAL_GET_GPIO(GPIOI, GPIO_PIN_0),  //!< Port I Pin 0
    PI1  = CYHAL_GET_GPIO(GPIOI, GPIO_PIN_1),  //!< Port I Pin 1
    PI2  = CYHAL_GET_GPIO(GPIOI, GPIO_PIN_2),  //!< Port I Pin 2
    PI3  = CYHAL_GET_GPIO(GPIOI, GPIO_PIN_3),  //!< Port I Pin 3
    PI4  = CYHAL_GET_GPIO(GPIOI, GPIO_PIN_4),  //!< Port I Pin 4
    PI5  = CYHAL_GET_GPIO(GPIOI, GPIO_PIN_5),  //!< Port I Pin 5
    PI6  = CYHAL_GET_GPIO(GPIOI, GPIO_PIN_6),  //!< Port I Pin 6
    PI7  = CYHAL_GET_GPIO(GPIOI, GPIO_PIN_7),  //!< Port I Pin 7
    PI8  = CYHAL_GET_GPIO(GPIOI, GPIO_PIN_8),  //!< Port I Pin 8
    PI9  = CYHAL_GET_GPIO(GPIOI, GPIO_PIN_9),  //!< Port I Pin 9
    PI10 = CYHAL_GET_GPIO(GPIOI, GPIO_PIN_10), //!< Port I Pin 10
    PI11 = CYHAL_GET_GPIO(GPIOI, GPIO_PIN_11), //!< Port I Pin 11
    PI12 = CYHAL_GET_GPIO(GPIOI, GPIO_PIN_12), //!< Port I Pin 12
    PI13 = CYHAL_GET_GPIO(GPIOI, GPIO_PIN_13), //!< Port I Pin 13
    PI14 = CYHAL_GET_GPIO(GPIOI, GPIO_PIN_14), //!< Port I Pin 14
    PI15 = CYHAL_GET_GPIO(GPIOI, GPIO_PIN_15), //!< Port I Pin 15
    #endif // if defined(GPIOI)
    #if defined(GPIOJ)
    PJ0  = CYHAL_GET_GPIO(GPIOJ, GPIO_PIN_0),  //!< Port J Pin 0
    PJ1  = CYHAL_GET_GPIO(GPIOJ, GPIO_PIN_1),  //!< Port J Pin 1
    PJ2  = CYHAL_GET_GPIO(GPIOJ, GPIO_PIN_2),  //!< Port J Pin 2
    PJ3  = CYHAL_GET_GPIO(GPIOJ, GPIO_PIN_3),  //!< Port J Pin 3
    PJ4  = CYHAL_GET_GPIO(GPIOJ, GPIO_PIN_4),  //!< Port J Pin 4
    PJ5  = CYHAL_GET_GPIO(GPIOJ, GPIO_PIN_5),  //!< Port J Pin 5
    PJ6  = CYHAL_GET_GPIO(GPIOJ, GPIO_PIN_6),  //!< Port J Pin 6
    PJ7  = CYHAL_GET_GPIO(GPIOJ, GPIO_PIN_7),  //!< Port J Pin 7
    PJ8  = CYHAL_GET_GPIO(GPIOJ, GPIO_PIN_8),  //!< Port J Pin 8
    PJ9  = CYHAL_GET_GPIO(GPIOJ, GPIO_PIN_9),  //!< Port J Pin 9
    PJ10 = CYHAL_GET_GPIO(GPIOJ, GPIO_PIN_10), //!< Port J Pin 10
    PJ11 = CYHAL_GET_GPIO(GPIOJ, GPIO_PIN_11), //!< Port J Pin 11
    PJ12 = CYHAL_GET_GPIO(GPIOJ, GPIO_PIN_12), //!< Port J Pin 12
    PJ13 = CYHAL_GET_GPIO(GPIOJ, GPIO_PIN_13), //!< Port J Pin 13
    PJ14 = CYHAL_GET_GPIO(GPIOJ, GPIO_PIN_14), //!< Port J Pin 14
    PJ15 = CYHAL_GET_GPIO(GPIOJ, GPIO_PIN_15), //!< Port J Pin 15
    #endif // if defined(GPIOJ)
    #if defined(GPIOK)
    PK0  = CYHAL_GET_GPIO(GPIOK, GPIO_PIN_0),  //!< Port K Pin 0
    PK1  = CYHAL_GET_GPIO(GPIOK, GPIO_PIN_1),  //!< Port K Pin 1
    PK2  = CYHAL_GET_GPIO(GPIOK, GPIO_PIN_2),  //!< Port K Pin 2
    PK3  = CYHAL_GET_GPIO(GPIOK, GPIO_PIN_3),  //!< Port K Pin 3
    PK4  = CYHAL_GET_GPIO(GPIOK, GPIO_PIN_4),  //!< Port K Pin 4
    PK5  = CYHAL_GET_GPIO(GPIOK, GPIO_PIN_5),  //!< Port K Pin 5
    PK6  = CYHAL_GET_GPIO(GPIOK, GPIO_PIN_6),  //!< Port K Pin 6
    PK7  = CYHAL_GET_GPIO(GPIOK, GPIO_PIN_7),  //!< Port K Pin 7
    PK8  = CYHAL_GET_GPIO(GPIOK, GPIO_PIN_8),  //!< Port K Pin 8
    PK9  = CYHAL_GET_GPIO(GPIOK, GPIO_PIN_9),  //!< Port K Pin 9
    PK10 = CYHAL_GET_GPIO(GPIOK, GPIO_PIN_10), //!< Port K Pin 10
    PK11 = CYHAL_GET_GPIO(GPIOK, GPIO_PIN_11), //!< Port K Pin 11
    PK12 = CYHAL_GET_GPIO(GPIOK, GPIO_PIN_12), //!< Port K Pin 12
    PK13 = CYHAL_GET_GPIO(GPIOK, GPIO_PIN_13), //!< Port K Pin 13
    PK14 = CYHAL_GET_GPIO(GPIOK, GPIO_PIN_14), //!< Port K Pin 14
    PK15 = CYHAL_GET_GPIO(GPIOK, GPIO_PIN_15)  //!< Port K Pin 15
    #endif // if defined(GPIOK)
} cyhal_gpio_def_t;


#ifdef __cplusplus
}
#endif /* __cplusplus */
