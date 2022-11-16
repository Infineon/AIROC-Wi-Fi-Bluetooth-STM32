/***************************************************************************************************
 * File Name: wifi_task.c
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
#include <lwip/tcpip.h>
#include <lwip/api.h>
#include "cyabs_rtos.h"
#include "whd_wifi_api.h"
#include "whd_types.h"
#include "whd_types_int.h"
#include "whd_int.h"
#include "cy_utils.h"

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
    uint32_t result_count;
} wcm_scan_data_t;

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Wi-Fi Connection related parameters */
#define WIFI_CONNECT_ENABLE                 (0) /* Enable/Disable Wi-Fi Connect */
#define WIFI_SSID                           "WIFI_SSID"
#define WIFI_PASSWORD                       "WIFI_PASSWORD"
#define WIFI_SECURITY                       CY_WCM_SECURITY_WPA2_AES_PSK
#define MAX_WIFI_RETRY_COUNT                (3u)
#define WIFI_CONN_RETRY_INTERVAL_MSEC       (100u)

/* The delay in milliseconds between successive scans.*/
#define SCAN_DELAY_MS                           (3000u)

#define PRINT_SCAN_TEMPLATE() \
    printf("\r\n--------------------------------------------------" \
           "--------------------------------------------------\r\n" \
           "  #                  SSID                  RSSI   Channel       " \
           "MAC Address              Security\r\n" \
           "--------------------------------------------------" \
           "--------------------------------------------------\r\n");

/* Private variables ---------------------------------------------------------*/
/* Connection parameters to the Wi-Fi connection manager (WCM). */
cy_wcm_connect_params_t connect_param;
cy_wcm_ip_address_t     ip_addr;
wcm_scan_data_t         scan_data;
cy_wcm_mac_t            last_bssid;

/* Private function prototypes -----------------------------------------------*/
void app_init(void);
void scan_result_callback(cy_wcm_scan_result_t* result_ptr, void* user_data,
                          cy_wcm_scan_status_t status);
const char* security_to_str(whd_security_t security);
void ospi_configure(void);
void wifi_connect(void);
extern osThreadId_t       WiFi_TaskHandle;
extern UART_HandleTypeDef huart1;
/* Private user code ---------------------------------------------------------*/
SD_HandleTypeDef SDHandle = { .Instance = SDMMC1 };

/***************************************************************************************************
 * Function Name: WiFiTask
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
void WiFiTask(void* argument)
{
    cy_rslt_t       result = CY_RSLT_SUCCESS;
    cy_wcm_config_t wcm_config;
    wcm_config.interface = CY_WCM_INTERFACE_TYPE_STA;

    /* Wait for the user button press to continue */
    app_init();
    ospi_configure();
    if (stm32_cypal_wifi_sdio_init(&SDHandle) != CY_RSLT_SUCCESS)
    {
        printf("\r\n    ERROR: Init failed\r\n\r\n");
        Error_Handler();
    }

    /* wcm init */
    result = cy_wcm_init(&wcm_config);
    if (CY_RSLT_SUCCESS != result)
    {
        printf("Failed to initialize Wi-Fi\r\n");
        Error_Handler();
    }

    #if WIFI_CONNECT_ENABLE
    /* Wi-Fi connect */
    wifi_connect();

    #else /* Wi-Fi Scanning in loop */
    {
        cy_wcm_scan_filter_t scan_filter =
        {
            .mode             = CY_WCM_SCAN_FILTER_TYPE_RSSI,
            .param.rssi_range = CY_WCM_SCAN_RSSI_FAIR
        };

        while (true)
        {
            /* Reset the count everytime before scanning */
            scan_data.result_count = 0;
            memset(last_bssid, 0, sizeof(cy_wcm_mac_t));

            /* Print out the scan results*/
            PRINT_SCAN_TEMPLATE();

            /* Start the scan */
            result =  cy_wcm_start_scan(scan_result_callback, &scan_data, &scan_filter);
            if (CY_RSLT_SUCCESS == result)
            {
                xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
            }
            /* Add Delay before starting the scan again */
            vTaskDelay(pdMS_TO_TICKS(SCAN_DELAY_MS));
        }
    }
    #endif // if WIFI_CONNECT_ENABLE
}


/***************************************************************************************************
 * Function Name: app_init
 ***************************************************************************************************
 * Summary: This Function handles the user button press event which helps the
 * user to connect the CYW43xxx to microSD card slot before starting the app.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 **************************************************************************************************/
void app_init()
{
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);

    printf(" ******************* WiFi-Scan app ******************* \r\n\r\n");
    printf("    Push blue button or send any symbol via serial terminal to continue...\r\n\r\n");

    char read_terminal = 0;

    /* Wait for User push-button press before starting the Communication */
    while ((HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin) != GPIO_PIN_SET) &&
           (read_terminal == 0))
    {
        HAL_UART_Receive(&huart1, (uint8_t*)&read_terminal, 1, 1);
        HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
        HAL_Delay(100);
    }

    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);

    if (HAL_GPIO_ReadPin(SDIO_DETECT_GPIO_Port, SDIO_DETECT_Pin) == GPIO_PIN_RESET)
    {
        printf("    CYW43xxx detected \r\n\r\n");
    }
    else
    {
        printf("    ERROR: CYW43xxx not detected in microSD card slot \r\n");
        Error_Handler();
    }
}


/***************************************************************************************************
 * Function Name: ospi_configure
 ***************************************************************************************************
 * Summary: This Function configures OSPI in memory mapped mode
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 **************************************************************************************************/
void ospi_configure()
{
    /* OCTOSPI Configuration Start */

    BSP_OSPI_NOR_Info_t pQSPI_Info;
    uint8_t             status;
    BSP_OSPI_NOR_Init_t init;
    init.InterfaceMode=MX25LM51245G_OPI_MODE;
    init.TransferRate = MX25LM51245G_DTR_TRANSFER;
    status            = BSP_OSPI_NOR_Init(0, &init);
    if (status != BSP_ERROR_NONE)
    {
        printf("\r\n    ERROR: BSP_QSPI_Init() failed \r\n");
        Error_Handler();
    }
    pQSPI_Info.FlashSize          = (uint32_t)0x00;
    pQSPI_Info.EraseSectorSize    = (uint32_t)0x00;
    pQSPI_Info.EraseSectorsNumber = (uint32_t)0x00;
    pQSPI_Info.ProgPageSize       = (uint32_t)0x00;
    pQSPI_Info.ProgPagesNumber    = (uint32_t)0x00;

    /* Read the QSPI memory info */
    BSP_OSPI_NOR_GetInfo(0, &pQSPI_Info);

    /*##-6-Memory Mapped Mode ###############################################*/
    status = BSP_OSPI_NOR_EnableMemoryMappedMode(0);
    if (status != BSP_ERROR_NONE)
    {
        printf("\r\n    ERROR: BSP_OSPI_NOR_EnableMemoryMappedMode() failed \r\n");
        Error_Handler();
    }
    /* OCTOSPI Configuration End */
}


/***************************************************************************************************
 * Function Name: scan_check_bssid_in_list
 ***************************************************************************************************
 * Summary: This Function removes duplicate entries in scan results.
 *
 * Parameters:
 *  cy_wcm_scan_result_t *result_ptr: Pointer to the scan result
 *
 * Return:
 *  bool
 *
 **************************************************************************************************/
bool scan_check_bssid_in_list(cy_wcm_scan_result_t* result_ptr)
{
    bool present = false;
    if (memcmp(result_ptr->BSSID, last_bssid, sizeof(cy_wcm_mac_t)) == 0)
    {
        /* Already existing BSSID, ignore the result */
        present = true;
        return present;
    }
    return present;
}


/***************************************************************************************************
 * Function Name: scan_result_callback
 ***************************************************************************************************
 * Summary: The callback function which accumulates the scan results. After
 * completing the scan, it sends a task notification to scan_task.
 *
 * Parameters:
 *  cy_wcm_scan_result_t *result_ptr: Pointer to the scan result
 *  void *user_data: User data.
 *  cy_wcm_scan_status_t status: Status of scan completion.
 *
 * Return:
 *  void
 *
 **************************************************************************************************/
void scan_result_callback(cy_wcm_scan_result_t* result_ptr, void* user_data,
                          cy_wcm_scan_status_t status)
{
    wcm_scan_data_t* scan_data = (wcm_scan_data_t*)user_data;

    if (scan_data != NULL)
    {
        if (status == CY_WCM_SCAN_COMPLETE)
        {
            xTaskNotify(WiFi_TaskHandle, 0, eNoAction);
        }
        else
        {
            if ((result_ptr != NULL) && (status == CY_WCM_SCAN_INCOMPLETE))
            {
                if ((strlen((const char*)result_ptr->SSID) != 0) &&
                    (scan_check_bssid_in_list(result_ptr) == false))
                {
                    scan_data->result_count++;
                    /* Copy BSSID to last bssid which will be used in scan_check_bssid_in_list to
                       avoid
                     * repeating BSSID in scan results
                     */
                    memcpy(&last_bssid, &result_ptr->BSSID[0], sizeof(last_bssid));
                    printf(" %2ld   %-32s     %4d     %3d      %02X:%02X:%02X:%02X:%02X:%02X"
                           "         %-15s\r\n",
                           scan_data->result_count, result_ptr->SSID,
                           result_ptr->signal_strength, result_ptr->channel,
                           result_ptr->BSSID[0], result_ptr->BSSID[1],
                           result_ptr->BSSID[2], result_ptr->BSSID[3],
                           result_ptr->BSSID[4], result_ptr->BSSID[5],
                           security_to_str((whd_security_t)result_ptr->security));
                }
            }
        }
    }
}


/***************************************************************************************************
 * Function Name: security_to_str
 ***************************************************************************************************
 * Summary: This Function returns string for each of WHD security types which
 * will be used in scan results.
 * Parameters:
 *  whd_security_t security: WHD security type enum
 *
 * Return:
 *  char *
 *
 **************************************************************************************************/
const char* security_to_str(whd_security_t security)
{
    switch (security)
    {
        case WHD_SECURITY_OPEN:
            return "OPEN";

        case WHD_SECURITY_WEP_PSK:
            return "WEP_PSK";

        case WHD_SECURITY_WEP_SHARED:
            return "WEP_SHARED";

        case WHD_SECURITY_WPA_TKIP_PSK:
            return "WPA_TKIP_PSK";

        case WHD_SECURITY_WPA_AES_PSK:
            return "WPA_AES_PSK";

        case WHD_SECURITY_WPA_MIXED_PSK:
            return "WPA_MIXED_PSK";

        case WHD_SECURITY_WPA2_AES_PSK:
            return "WPA2_AES_PSK";

        case WHD_SECURITY_WPA2_TKIP_PSK:
            return "WPA2_TKIP_PSK";

        case WHD_SECURITY_WPA2_MIXED_PSK:
            return "WPA2_MIXED_PSK";

        case WHD_SECURITY_WPA2_FBT_PSK:
            return "WPA2_FBT_PSK";

        case WHD_SECURITY_WPA3_SAE:
            return "WPA3_SAE";

        case WHD_SECURITY_WPA3_WPA2_PSK:
            return "WPA3_WPA2_PSK";

        case WHD_SECURITY_WPA_TKIP_ENT:
            return "WPA_TKIP_ENTERPRISE";

        case WHD_SECURITY_WPA_AES_ENT:
            return "WPA_AES_ENTERPRISE";

        case WHD_SECURITY_WPA_MIXED_ENT:
            return "WPA_MIXED_ENTERPRISE";

        case WHD_SECURITY_WPA2_TKIP_ENT:
            return "WPA2_TKIP_ENTERPRISE";

        case WHD_SECURITY_WPA2_AES_ENT:
            return "WPA2_AES_ENTERPRISE";

        case WHD_SECURITY_WPA2_MIXED_ENT:
            return "WPA2_MIXED_ENTERPRISE";

        case WHD_SECURITY_WPA2_FBT_ENT:
            return "WPA2_FBT_ENTERPRISE";

        case WHD_SECURITY_IBSS_OPEN:
            return "IBSS_OPEN";

        case WHD_SECURITY_WPS_SECURE:
            return "WPS_SECURE";

        case WHD_SECURITY_FORCE_32_BIT:
        case WHD_SECURITY_UNKNOWN:
        default:
            return "UNKNOWN_SECURITY";
    }
}


/***************************************************************************************************
 * print_ip4
 **************************************************************************************************/
static void print_ip4(uint32_t ip, char* str)
{
    unsigned char bytes[4];
    bytes[0] = ip & 0xFF;
    bytes[1] = (ip >> 8) & 0xFF;
    bytes[2] = (ip >> 16) & 0xFF;
    bytes[3] = (ip >> 24) & 0xFF;
    printf("%s addr = %d.%d.%d.%d\r\n", str, bytes[0], bytes[1], bytes[2], bytes[3]);
}


/***************************************************************************************************
 * Function Name: wifi_connect
 ***************************************************************************************************
 * Summary: This function executes a connect to the AP. The maximum number of
 * times it attempts to connect to the AP is specified by MAX_RETRY_COUNT. Then
 * ping to Gateway IP address.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 **************************************************************************************************/
void wifi_connect()
{
    cy_rslt_t                   result = CY_RSLT_SUCCESS;
    cy_wcm_connect_params_t     connect_param;
    cy_wcm_ip_address_t         ip_address;
    cy_wcm_ip_address_t         gateway_addr;
    uint32_t                    elapsed_time_ms;
    cy_wcm_associated_ap_info_t ap_info;

    memset(&connect_param, 0, sizeof(cy_wcm_connect_params_t));
    memset(&ip_address, 0, sizeof(cy_wcm_ip_address_t));
    memcpy(connect_param.ap_credentials.SSID, WIFI_SSID, sizeof(WIFI_SSID));
    memcpy(connect_param.ap_credentials.password, WIFI_PASSWORD, sizeof(WIFI_PASSWORD));
    connect_param.ap_credentials.security = WIFI_SECURITY;

    while (true)
    {
        printf("\r\n***** Connecting to '%s' AP ***** \r\n", connect_param.ap_credentials.SSID);

        /* Attempt to connect to Wi-Fi until a connection is made or
         * MAX_WIFI_RETRY_COUNT attempts have been made.
         */
        for (uint32_t conn_retries = 0; conn_retries < MAX_WIFI_RETRY_COUNT; conn_retries++)
        {
            result = cy_wcm_connect_ap(&connect_param, &ip_address);
            if (result == CY_RSLT_SUCCESS)
            {
                printf("Successfully connected to Wi-Fi network '%s'.\r\n",
                       connect_param.ap_credentials.SSID);
                break;
            }
            printf("Connection to Wi-Fi network failed with error code %d."
                   "Retrying in %d ms...\r\n", (int)result, WIFI_CONN_RETRY_INTERVAL_MSEC);
            vTaskDelay(pdMS_TO_TICKS(WIFI_CONN_RETRY_INTERVAL_MSEC));
        }
        if (result != CY_RSLT_SUCCESS)
        {
            printf("Wi-FI connection failed even with multiple retries \r\n");
            return;
        }
        /* Get RSSI */
        result = cy_wcm_get_associated_ap_info(&ap_info);
        if (result != CY_RSLT_SUCCESS)
        {
            printf("cy_wcm_get_associated_ap_info failed with result %ld \r\n", result);
            return;
        }
        printf("RSSI : %d \r\n", ap_info.signal_strength);

        /* Get IPv4 address */
        result = cy_wcm_get_ip_addr(CY_WCM_INTERFACE_TYPE_STA, &ip_address);
        if (result != CY_RSLT_SUCCESS)
        {
            printf("cy_wcm_get_ip_addr failed with result %ld \r\n", result);
            return;
        }
        print_ip4(ip_address.ip.v4, "IPV4");

        /* Get gateway address */
        result = cy_wcm_get_gateway_ip_address(CY_WCM_INTERFACE_TYPE_STA, &gateway_addr);
        if (result != CY_RSLT_SUCCESS)
        {
            printf("cy_wcm_get_gateway_ip_address failed with result %ld \r\n", result);
            return;
        }

        print_ip4(gateway_addr.ip.v4, "Pinging to gateway IPV4");
        vTaskDelay(2000);

        /* Send PING request with 3000ms ping timeout */
        result = cy_wcm_ping(CY_WCM_INTERFACE_TYPE_STA, &gateway_addr, 3000, &elapsed_time_ms);
        switch (result)
        {
            case CY_RSLT_SUCCESS:
                printf("Ping was successful time elapsed = %lu ms\r\n", elapsed_time_ms);
                break;

            case CY_RSLT_WCM_WAIT_TIMEOUT:
                printf("Ping timeout!\r\n");
                break;

            default:
                printf("Ping failed !! Module %lx Code %lx\r\n", CY_RSLT_GET_MODULE(
                           result), CY_RSLT_GET_CODE(result));
                break;
        }

        printf("Disconnecting ... \r\n");
        result = cy_wcm_disconnect_ap();
        if (result != CY_RSLT_SUCCESS)
        {
            printf("cy_wcm_disconnect_ap failed with result %ld \r\n", result);
        }
        /* Delay before Connecting back to AP */
        vTaskDelay(pdMS_TO_TICKS(SCAN_DELAY_MS));
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
