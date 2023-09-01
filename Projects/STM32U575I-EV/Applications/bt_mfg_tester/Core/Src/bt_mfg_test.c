/******************************************************************************
 * File Name:   bt_mfg_test.c
 *
 * Description: This is the source code for the top-level Bluetooth
 *              manufacturing test functions.
 *
 * Related Document: See readme.txt
 *
 *
 *******************************************************************************
 * (c) 2019-2020, Cypress Semiconductor Corporation. All rights reserved.
 *******************************************************************************
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
#include "main.h"
#include "cyhal.h"
#include "cyhal_gpio.h"


#include "stm32_cyhal_gpio_ex.h"
#include "stm32_cyhal_sdio_ex.h"
#include "stm32_cyhal_uart_ex.h"

#include "bt_mfg_test.h"
#include "bt_transport.h"
#include "bt_bus.h"
#include "bt_firmware.h"

#include <stdio.h>

/******************************************************************************
* Macros
******************************************************************************/
#define BAUDRATE_3MBPS       (3000000)
#define BAUDRATE_115KBPS     (115200)

/* Task parameters for MfgTest App Task. */
#define MFG_TEST_TX_TASK_PRIORITY    CY_RTOS_PRIORITY_NORMAL
#define MFG_TEST_RX_TASK_PRIORITY    CY_RTOS_PRIORITY_ABOVENORMAL
#define MFG_TEST_TASK_STACK_SIZE     (1024)

#define TRANSPORT_TASK_PRIORITY      CY_RTOS_PRIORITY_NORMAL
#define TRANSPORT_TASK_STACK_SIZE    (4096)

/*******************************************************
 *                    Structures
 ******************************************************/

/*******************************************************************************
 * Global Variables
 ******************************************************************************/

extern UART_HandleTypeDef hlpuart1;
extern UART_HandleTypeDef huart1;

#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
/***************************************************************************
* Function Name: __io_putchar (GCC)
***************************************************************************/
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 0xFFFF);
    return ch;
}


/***************************************************************************
* Function Name: getchar
***************************************************************************/
int getchar(void)
{
    int8_t ch = -1;

    while (HAL_UART_Receive(&huart1, (uint8_t*)&ch, 1, HAL_MAX_DELAY) != HAL_OK)
    {
    }

    return ch;
}


#elif defined (__ICCARM__) /* IAR */
    #include <yfuns.h>

/***************************************************************************
* Function Name: __write (IAR)
***************************************************************************/
size_t __write(int handle, const unsigned char* buffer, size_t size)
{
    size_t nChars = 0;
    /* This template only writes to "standard out", for all other file
     * handles it returns failure. */
    if (handle != _LLIO_STDOUT)
    {
        return (_LLIO_ERROR);
    }
    if (buffer != NULL)
    {
        for (/* Empty */; nChars < size; ++nChars)
        {
            HAL_UART_Transmit(&huart1, (uint8_t*)buffer, 1, 0xFFFF);
            ++buffer;
        }
    }
    return (nChars);
}


/***************************************************************************
* Function Name: __read
***************************************************************************/
size_t __read(int handle, unsigned char* buffer, size_t size)
{
    HAL_StatusTypeDef rslt;

    // This template only reads from "standard in", for all other file handles it returns failure.
    if ((handle != _LLIO_STDIN) || (buffer == NULL))
    {
        return (_LLIO_ERROR);
    }
    else
    {
        rslt = HAL_UART_Receive(&huart1, (uint8_t*)buffer, 1, HAL_MAX_DELAY);
        return (CY_RSLT_SUCCESS == rslt) ? 1 : 0;
    }
}


/***************************************************************************************************
 * __aeabi_read_tp
 **************************************************************************************************/
void* __aeabi_read_tp(void)
{
    return 0;
}


#endif /* __GNUC__ */

/***************************************************************************************************
 * bt_power_init
 **************************************************************************************************/
bool bt_power_init(cyhal_gpio_t power_pin)
{
    if (CY_RSLT_SUCCESS != cyhal_gpio_init(power_pin,
                                           CYHAL_GPIO_DIR_OUTPUT,
                                           CYHAL_GPIO_DRIVE_PULLUP,
                                           1
                                           ))
    {
        return false;
    }

    cyhal_gpio_write(power_pin, true);
    cy_rtos_delay_milliseconds(500);

    return true;
}


/*******************************************************************************
 * Function Name: bt_mfgtest_task
 *******************************************************************************
 * Summary:
 *  Task for handling initialization BT/UART and patchram download
 *
 * Parameters:
 *  void *pvParameters : Task parameter defined during task creation (unused)
 *
 * Return:
 *  void
 *
 ******************************************************************************/
void bt_mfgtest_task(void* pvParameters)
{
    extern void bt_tx_transport_task(cy_thread_arg_t arg);
    extern void bt_rx_transport_task(cy_thread_arg_t arg);

    extern const uint8_t brcm_patchram_buf[];
    extern const int     brcm_patch_ram_length;

    printf("=============================================\r\n");
    printf("BT MfgTest Application\r\n");
    printf("=============================================\r\n");

    printf("bt_mfgtest_task\r\n");

    /* Set I/O to No Buffering */
    setvbuf(stdin, NULL, _IONBF, 0);

    /* Set I/O to No Buffering */
    setvbuf(stdout, NULL, _IONBF, 0);

    if (stm32_cypal_uart_hw_init(&hlpuart1, CYBSP_BT_UART_TX) != 0)
    {
        printf("Error Init UART failed\r\n");
        return;
    }

    /* Turn on BT */
    if (!bt_power_init(CYBSP_BT_POWER))
    {
        printf("Error Init power pin failed\r\n");
        return;
    }

    /* Enable HCI UART */
    if (!bt_bus_init())
    {
        printf("Error initialising BT bus\r\n");
        return;
    }

    /* patchram */
    if (!bt_firmware_download(brcm_patchram_buf, brcm_patch_ram_length))
    {
        printf("Error downloading HCI firmware\r\n");
        return;
    }

    printf("Firmware downloaded. You can start do wmbt test\n\r");

    bt_trasport_mempool_init();

    /* transport Tx thread - Grab message from PC and pass it over to the controller */
    xTaskCreate(bt_tx_transport_task, "Transport Tx task", TRANSPORT_TASK_STACK_SIZE,
                NULL, MFG_TEST_TX_TASK_PRIORITY, NULL);

    /* transport Rx thread - Grab message from controller and pass it over to the PC */
    xTaskCreate(bt_rx_transport_task, "Transport Rx task", TRANSPORT_TASK_STACK_SIZE,
                NULL, MFG_TEST_RX_TASK_PRIORITY, NULL);

    /* Cleanup section for various operations. */
    vTaskDelete(NULL);
}


/***************************************************************************************************
 * Function Name: HAL_GPIO_EXTI_Callback
 ***************************************************************************************************
 * Summary: This Function Overwrite EXTI Callback Function in stm32h7xx_hal_gpio.c
 * and calls GPIO callback function for that particular GPIO.
 *
 * Parameters:
 *  uint16_t GPIO_Pin: GPIO Pin number
 *
 * Return:
 *  void
 *
 **************************************************************************************************/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    stm32_cyhal_gpio_irq_handler(GPIO_Pin);
}
