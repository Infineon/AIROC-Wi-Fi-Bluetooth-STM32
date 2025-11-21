/***************************************************************************************************
 * File Name: console_task.c
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
#include "cyhal_sdio.h"
#include "cyhal_uart.h"
#include "stm32_cyhal_sdio_ex.h"
#include "stm32_cyhal_gpio_ex.h"
#include "stm32_cyhal_uart_ex.h"
#include "stm32_cyhal_trng_ex.h"
#include "wifi_bt_if.h"
#include "cy_wcm.h"
#include "cyabs_rtos.h"
#include "whd_wifi_api.h"
#include "whd_types.h"
#include "whd_types_int.h"
#include "whd_int.h"
#include "cy_utils.h"
#include "command_console.h"
#include "bt_utility.h"
#include "iperf_utility.h"
#include "wifi_utility.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
#ifndef WIFI_SSID
#define WIFI_SSID                        "WIFI_SSID"
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD                    "WIFI_PASSWORD"
#endif
#define WIFI_BAND                        CY_WCM_WIFI_BAND_ANY
#define CMD_CONSOLE_MAX_WIFI_RETRY_COUNT 15
#define IP_STR_LEN                       16

#define CY_RSLT_ERROR                    (-1)

#define CONSOLE_COMMAND_MAX_PARAMS       (32)
#define CONSOLE_COMMAND_MAX_LENGTH       (85)
#define CONSOLE_COMMAND_HISTORY_LENGTH   (10)

/* Private variables ---------------------------------------------------------*/
/* Connection parameters to the Wi-Fi connection manager (WCM). */
static cy_wcm_config_t wcm_config;
static cy_wcm_connect_params_t conn_params;
const char* console_delimiter_string = " ";
static char command_buffer[CONSOLE_COMMAND_MAX_LENGTH];
static char command_history_buffer[CONSOLE_COMMAND_MAX_LENGTH * CONSOLE_COMMAND_HISTORY_LENGTH];

/* Private function prototypes -----------------------------------------------*/
void app_init(void);
extern UART_HandleTypeDef  huart1;
extern RNG_HandleTypeDef hrng;

/* Private user code ---------------------------------------------------------*/
SD_HandleTypeDef SDHandle = { .Instance = SDMMC2 };
cyhal_uart_t command_console_uart_obj;


/***************************************************************************************************
 * get_ip_string
 **************************************************************************************************/
static void get_ip_string(char* buffer, uint32_t ip)
{
    sprintf(buffer, "%lu.%lu.%lu.%lu",
            (unsigned long)(ip) & 0xFF,
            (unsigned long)(ip >>  8) & 0xFF,
            (unsigned long)(ip >> 16) & 0xFF,
            (unsigned long)(ip >> 24) & 0xFF);
}


/***************************************************************************************************
 * connect_wifi
 **************************************************************************************************/
cy_rslt_t connect_wifi()
{
    cy_rslt_t res;

    const char* ssid = WIFI_SSID;
    const char* key = WIFI_PASSWORD;
    cy_wcm_wifi_band_t band = WIFI_BAND;
    int retry_count = 0;
    cy_wcm_ip_address_t ip_addr;
    char ipstr[IP_STR_LEN];

    memset(&conn_params, 0, sizeof(cy_wcm_connect_params_t));

    while (1)
    {
        /*
         * Join to WIFI AP
         */
        memcpy(&conn_params.ap_credentials.SSID, ssid, strlen(ssid) + 1);
        memcpy(&conn_params.ap_credentials.password, key, strlen(key) + 1);
        conn_params.ap_credentials.security = CY_WCM_SECURITY_WPA2_AES_PSK;
        conn_params.band = band;

        res = cy_wcm_connect_ap(&conn_params, &ip_addr);
        cy_rtos_delay_milliseconds(500);

        if (res != CY_RSLT_SUCCESS)
        {
            retry_count++;
            if (retry_count >= CMD_CONSOLE_MAX_WIFI_RETRY_COUNT)
            {
                printf("Exceeded max WiFi connection attempts\n");
                return CY_RSLT_ERROR;
            }
            printf("Connection to WiFi network failed. Retrying...\n");
            continue;
        }
        else
        {
            printf("Successfully joined wifi network '%s , result = %ld'\n", ssid, (long)res);
            get_ip_string(ipstr, ip_addr.ip.v4);
            printf("IP Address %s assigned\n", ipstr);
            break;
        }
    }

    return CY_RSLT_SUCCESS;
}


/***************************************************************************************************
 * command_console_add_command
 **************************************************************************************************/
cy_rslt_t command_console_add_command(void)
{
    cy_command_console_cfg_t console_cfg;
    cy_rslt_t result = CY_RSLT_SUCCESS;

    cyhal_uart_cfg_t command_console_uart_cfg;

    command_console_uart_cfg.data_bits = 8u;
    command_console_uart_cfg.parity    = CYHAL_UART_PARITY_NONE;
    command_console_uart_cfg.stop_bits = 1u;

    result = cyhal_uart_init(&command_console_uart_obj, COMMAND_CONSOLE_UART_TX,
                             COMMAND_CONSOLE_UART_RX, NC, NC, NULL, &command_console_uart_cfg);
    if (result != CY_RSLT_SUCCESS)
    {
        printf("Error in initializing UART for console library : %ld \n", (long)result);
        return CY_RSLT_ERROR;
    }

    result = cyhal_uart_set_baud(&command_console_uart_obj, 115200, NULL);
    if (result != CY_RSLT_SUCCESS)
    {
        printf("Error in configuration UART for console library : %ld \n", (long)result);
        return CY_RSLT_ERROR;
    }

    printf("executing command_console_add_remove_command \n");
    console_cfg.serial             = (void*)&command_console_uart_obj;
    console_cfg.line_len           = sizeof(command_buffer);
    console_cfg.buffer             = command_buffer;
    console_cfg.history_len        = CONSOLE_COMMAND_HISTORY_LENGTH;
    console_cfg.history_buffer_ptr = command_history_buffer;
    console_cfg.delimiter_string   = console_delimiter_string;
    console_cfg.params_num         = CONSOLE_COMMAND_MAX_PARAMS;
    console_cfg.thread_priority    = CY_RTOS_PRIORITY_NORMAL;
    console_cfg.delimiter_string   = " ";

    /* Initialize command console library */
    result = cy_command_console_init(&console_cfg);
    if (result != CY_RSLT_SUCCESS)
    {
        printf("Error in initializing command console library : %ld \n", (long)result);
        return CY_RSLT_ERROR;
    }

    /* Initialize Wi-Fi utility and add Wi-Fi commands */
    result = wifi_utility_init();
    if (result != CY_RSLT_SUCCESS)
    {
        printf("Error in initializing command console library : %ld \n", (long)result);
        return CY_RSLT_ERROR;
    }

    /* Initialize IPERF utility and add IPERF commands */
    iperf_utility_init(&wcm_config.interface);

    #if !defined(DISABLE_COMMAND_CONSOLE_BT)
    /* Initialize Bluetooth utility and add BT commands */
    bt_utility_init();
    #endif /* !defined(DISABLE_COMMAND_CONSOLE_BT) */

    return CY_RSLT_SUCCESS;
}


/* For Nucleo-H745ZI, Nucelo-H563ZI, and Nucleo-U575ZI */
#define SDMMC_D0        PC4
#define SDMMC_D1        PC5
#define SDMMC_D2        PC0
#define SDMMC_D3        PE4
#define SDMMC_DATA_DELAY 10

/***************************************************************************************************
 * toggle_sdmmc_data
 **************************************************************************************************/
void toggle_sdmmc_data(void)
{
    cyhal_gpio_init(SDMMC_D0, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_PULLUP, false);
    cyhal_gpio_init(SDMMC_D1, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_PULLUP, false);
    cyhal_gpio_init(SDMMC_D2, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_PULLUP, false);
    cyhal_gpio_init(SDMMC_D3, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_PULLUP, false);
    cyhal_system_delay_ms(SDMMC_DATA_DELAY);
    cyhal_gpio_write(SDMMC_D0, true);
    cyhal_gpio_write(SDMMC_D1, true);
    cyhal_gpio_write(SDMMC_D2, true);
    cyhal_gpio_write(SDMMC_D3, true);
    cyhal_system_delay_ms(SDMMC_DATA_DELAY);
    cyhal_gpio_free(SDMMC_D0);
    cyhal_gpio_free(SDMMC_D1);
    cyhal_gpio_free(SDMMC_D2);
    cyhal_gpio_free(SDMMC_D3);
}


/***************************************************************************************************
 * Function Name: console_task
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
void console_task(ULONG id)
{
    cy_rslt_t       result = CY_RSLT_SUCCESS;
    cy_wcm_config_t wcm_config;
    wcm_config.interface = CY_WCM_INTERFACE_TYPE_STA;

    printf("Command console application\r\n\n");

    /* STM32 CYPAL init */
    #if !defined(DISABLE_COMMAND_CONSOLE_BT)
    if (stm32_cypal_bt_init(&huart2, &hlptim1) != CY_RSLT_SUCCESS)
    {
        printf("\r\n    ERROR: stm32_cypal_bt_init failed\r\n\r\n");
        Error_Handler();
    }
    #endif /* DISABLE_COMMAND_CONSOLE_BT */

    /* Workaround for Nucleo144-M.2 Adapter */
    //toggle_sdmmc_data();

    if (stm32_cypal_wifi_sdio_init(&SDHandle) != CY_RSLT_SUCCESS)
    {
        printf("\r\n    ERROR: stm32_cypal_wifi_sdio_init failed\r\n\r\n");
        Error_Handler();
    }

    if (stm32_cypal_trng_hw_init(&hrng) != CY_RSLT_SUCCESS)
    {
        printf("\r\n    ERROR: stm32_cypal_trng_hw_init failed\r\n\r\n");
        Error_Handler();
    }

    /* Allocate UART resource for command-console library */
    if (stm32_cypal_uart_hw_init(&huart1, COMMAND_CONSOLE_UART_TX) != CY_RSLT_SUCCESS)
    {
        printf("\r\n    ERROR: stm32_cypal_uart_hw_init failed\r\n\r\n");
        Error_Handler();
    }

    /* wcm init */
    result = cy_wcm_init(&wcm_config);
    if (CY_RSLT_SUCCESS != result)
    {
        printf("Failed to initialize Wi-Fi\r\n");
        Error_Handler();
    }
    printf("WCM Initialized\n");

    /* Connect to an AP for which credentials are specified */
    #if defined(WIFI_CONNECT_ENABLE) && (WIFI_CONNECT_ENABLE == 1)
    connect_wifi();
    #endif /* defined(WIFI_CONNECT_ENABLE) && (WIFI_CONNECT_ENABLE == 1) */

    command_console_add_command();

    while (1)
    {
        cy_rtos_delay_milliseconds(500);
    }
}
