/***************************************************************************************************
 * File Name: mfg_test_client_task.c
 *
 * Description: This file contains functions that perform network related tasks
 * like connecting to an AP, scanning for APs, connection event callbacks, and
 * utility function for printing scan results.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "cyhal_sdio.h"
#include "stm32_cyhal_sdio_ex.h"
#include "stm32_cyhal_gpio_ex.h"
#include "wifi_bt_if.h"
#include "cy_wcm.h"
#include "cyabs_rtos.h"
#include "whd_wifi_api.h"
#include "whd_types.h"
#include "whd_types_int.h"
#include "whd_int.h"
#include "cy_utils.h"
#include "mfg_test_common_api.h"


/* Private typedef -----------------------------------------------------------*/
typedef struct
{
    uint32_t result_count;
} wcm_scan_data_t;

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/


/* Macro to get the Wifi Mfg Tester application version. */
#define GET_WIFI_MFG_VER(str) #str
#define GET_WIFI_MFG_STRING(str) GET_WIFI_MFG_VER(str)
#define GET_WIFI_MFG_VER_STRING  GET_WIFI_MFG_STRING(WIFI_MFG_VER)

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
cy_rslt_t cy_wcm_get_whd_interface(cy_wcm_interface_t interface_type, whd_interface_t* whd_iface);
extern UART_HandleTypeDef huart1;
/* Private user code ---------------------------------------------------------*/
SD_HandleTypeDef SDHandle = { .Instance = SDMMC1 };

#define IOCTL_MED_LEN   (512)
static unsigned char buf[IOCTL_MED_LEN] = { 0 };


/***************************************************************************************************
 * Function Name: mfg_test_client_task
 ***************************************************************************************************
 * Summary: This task initializes the Wi-Fi device, Wi-Fi transport, lwIP
 * network stack, and issues a scan for the available networks. A scan filter
 * is applied depending on the value of scan_filter_mode_select. After starting
 * the scan, it waits for the notification from the scan callback for completion
 * of the scan. It then waits for a delay specified by SCAN_DELAY_MS
 * before repeating the process.
 *
 * Parameters:
 *  void* arg: Task parameter defined during task creation (unused).
 *
 * Return:
 *  void
 *
 **************************************************************************************************/
void mfg_test_client_task(void* argument)
{
    cy_rslt_t       result;
    cy_wcm_config_t wcm_config = { .interface = CY_WCM_INTERFACE_TYPE_STA };
    whd_interface_t sta_interface = NULL;


    /* Set I/O to No Buffering */
    setvbuf(stdin, NULL, _IONBF, 0);

    /* Set I/O to No Buffering */
    setvbuf(stdout, NULL, _IONBF, 0);

    if (stm32_cypal_wifi_sdio_init(&SDHandle) != CY_RSLT_SUCCESS)
    {
        printf("\r\n    ERROR: stm32_cypal_wifi_sdio_init failed\r\n\r\n");
        Error_Handler();
    }

    /* wcm init */
    result = cy_wcm_init(&wcm_config);
    if (CY_RSLT_SUCCESS != result)
    {
        printf("Failed to initialize Wi-Fi\r\n");
        Error_Handler();
    }

    result = cy_wcm_get_whd_interface(CY_WCM_INTERFACE_TYPE_STA, &sta_interface);
    if (result != CY_RSLT_SUCCESS)
    {
        printf("\r\nWifi STA Interface Not found, ");
        printf(" Wi-Fi Connection Manager initialization failed!\r\n");
        return;
    }

    wl_set_sta_interface_handle(sta_interface);

    printf("\r\nWi-Fi Connection Manager initialized.\r\n");

    while (true)
    {
        memset(buf, 0, sizeof(buf));
        wl_remote_command_handler(buf);
    }
}


/***************************************************************************************************
 * Function Name: SDMMC1_IRQHandler
 ***************************************************************************************************
 * Summary: This Function handles SDMMC Interrupt and calls STM hal layer
 * function which handles interrupt masking and calling sdio event callback
 * function.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 **************************************************************************************************/
void SDMMC1_IRQHandler(void)
{
    stm32_cyhal_sdio_irq_handler();
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
