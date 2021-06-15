/***************************************************************************************************
 * File Name:   wifi_task.c
 *
 * Description: This file contains the task definition for initializing the
 * Wi-Fi device, connecting to the AP, disconnecting from AP, scanning and
 * EEPROM related functionality.
 *
 * Related Document: See Readme.md
 *
 ***************************************************************************************************
 * (c) 2021, Cypress Semiconductor Corporation. All rights reserved.
 ***************************************************************************************************
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

#include <FreeRTOS.h>
#include <task.h>
#include "cybsp.h"
#include "stdio.h"
#include "cy_wcm.h"
#include "wifi_task.h"
#include "cycfg_gatt_db.h"
#include "wiced_bt_stack.h"
#include "app_utils.h"
#include "cycfg_gap.h"

#include "stdio.h"
#include "stdbool.h"

#include "cy_utils.h"


/***************************************************************************************************
 *                             Global Variables
 **************************************************************************************************/
/* WCM related structures */
cy_wcm_connect_params_t wifi_conn_param;

wifi_details_t wifi_details;

/***************************************************************************************************
 *                        Extern Functions and Variables
 **************************************************************************************************/
/* Maintains the connection id of the current connection */
extern uint16_t conn_id;

/* This variable is set to true when button callback is received and
 * data is present in EMEEPROM. It is set to false after the WiFi Task
 * processes Disconnection notification. It is used to check button
 * interrupt while the device is trying to connect to WiFi
 */
extern bool button_pressed;

/* Task Handles for WiFi Task */
extern TaskHandle_t wifi_task_handle;

extern gatt_db_lookup_table_t* app_get_attribute(uint16_t handle);

/***************************************************************************************************
 * Function Name: wifi_task
 ***************************************************************************************************
 * Summary:
 * This function initializes the WCM module and connects/disconnects to the WIFI AP
 *
 **************************************************************************************************/
void wifi_task(void* arg)
{
    cy_rslt_t result;

    /* Variable to use for filtering WiFi scan results */
    cy_wcm_scan_filter_t scan_filter;

    /* Notification values received from other tasks */
    uint32_t ulNotifiedValue;

    /* WCM configuration and IP address variables */
    cy_wcm_config_t     wifi_config;
    cy_wcm_ip_address_t ip_address;

    wifi_config.interface = CY_WCM_INTERFACE_TYPE_STA;

    /* Initialize WCM */
    printf("\n");
    result = cy_wcm_init(&wifi_config);
    if (CY_RSLT_SUCCESS != result)
    {
        printf("Failed to initialize Wi-Fi\n");
        CY_ASSERT(0);
    }

    while (true)
    {
        /* Wait for a notification */
        xTaskNotifyWait(0x00,       /* Don't clear any notification bits on entry. */
                        UINT32_MAX,  /* Reset the notification value to 0 on exit. */
                        &ulNotifiedValue,  /* Notified value pass out in
                                              ulNotifiedValue. */
                        portMAX_DELAY);    /* Block indefinitely. */

        /* Check if the WiFi credentials needs to be taken from GATT DB or EMEEPROM */
        if ((NOTIF_GATT_DB == ulNotifiedValue) || (NOTIF_EMEEPROM == ulNotifiedValue))
        {
            /* Set the WiFi Connection parameters structure to 0 before copying
             * data */
            memset(&wifi_conn_param, 0, sizeof(cy_wcm_connect_params_t));

            /* Credentials present in EMEEPROM */
            if (NOTIF_EMEEPROM == ulNotifiedValue)
            {
                printf("Getting credentials from EMEEPROM\n");
                memcpy(wifi_conn_param.ap_credentials.SSID, &wifi_details.wifi_ssid[0],
                       wifi_details.ssid_len);

                memcpy(wifi_conn_param.ap_credentials.password, &wifi_details.wifi_password[0],
                       wifi_details.password_len);
            }
            /* Credentials obtained through BLE */
            else if (NOTIF_GATT_DB == ulNotifiedValue)
            {
                /* Copy the SSID and password type from GATT DB */
                gatt_db_lookup_table_t* puAttribute;

                /* Get the right address for the WiFi SSID handle in Gatt DB */
                puAttribute = app_get_attribute(HDLC_CUSTOM_SERVICE_WIFI_SSID_VALUE);

                memcpy(wifi_conn_param.ap_credentials.SSID, puAttribute->p_data,
                       puAttribute->cur_len);

                /* Get the right address for the WiFi password handle in Gatt DB */
                puAttribute = app_get_attribute(HDLC_CUSTOM_SERVICE_WIFI_PASSWORD_VALUE);

                memcpy(wifi_conn_param.ap_credentials.password, puAttribute->p_data,
                       puAttribute->cur_len);

                memcpy(&wifi_details.wifi_ssid[0], wifi_conn_param.ap_credentials.SSID,
                       strlen((char*)wifi_conn_param.ap_credentials.SSID));

                memcpy(&wifi_details.wifi_password[0], wifi_conn_param.ap_credentials.password,
                       strlen((char*)wifi_conn_param.ap_credentials.password));

                wifi_details.ssid_len     = strlen((char*)wifi_conn_param.ap_credentials.SSID);
                wifi_details.password_len = strlen((char*)wifi_conn_param.ap_credentials.password);

                #if EEPROM_SUPORT
                /* Write data to EEPROM. */
                // TODO: add code to write wifi_details to STM32 flash storage (internal, external
                // flash)
                #endif /* EEPROM_SUPORT */
            }
            memset(&scan_filter, 0, sizeof(cy_wcm_scan_filter_t));

            /* Configure the scan filter for SSID */
            scan_filter.mode = CY_WCM_SCAN_FILTER_TYPE_SSID;
            memcpy(scan_filter.param.SSID, wifi_conn_param.ap_credentials.SSID,
                   strlen((char*)wifi_conn_param.ap_credentials.SSID));

            printf("Starting scan with SSID: %s\n", scan_filter.param.SSID);
            result = cy_wcm_start_scan(scan_callback, NULL, &scan_filter);

            if (result)
            {
                printf("Start scan failed\n");
            }
        }
        /* If the scan is complete then proceed for connection */
        else if (NOTIF_SCAN_COMPLETE == ulNotifiedValue)
        {
            /* Variable to track the number of connection retries to the Wi-Fi AP */
            uint8_t conn_retries = 0;

            /* Join the Wi-Fi AP */
            result = CY_RSLT_TYPE_ERROR;

            while ((result != CY_RSLT_SUCCESS) && (conn_retries < MAX_CONNECTION_RETRIES))
            {
                /* If button ISR is received then skip connecting to WiFi */
                if (button_pressed)
                {
                    break;
                }
                printf("\nTrying to connect SSID: %s, Password: %s\n",
                       wifi_conn_param.ap_credentials.SSID,
                       wifi_conn_param.ap_credentials.password);

                /* Connect to the given AP */
                result = cy_wcm_connect_ap(&wifi_conn_param, &ip_address);
                conn_retries++;
            }

            if (result == CY_RSLT_SUCCESS)
            {
                printf("Successfully joined the Wi-Fi network\n");

                /* Update GATT DB about disconnection */
                app_custom_service_wifi_connection[0] = true;

                /* Check if the connection is active, notifications are enabled the connection
                   request had
                 * come from BLE client and then send the notification
                 */
                if ((conn_id != 0) &&
                    ((app_custom_service_wifi_connection_client_char_config[0] &
                      GATT_CLIENT_CONFIG_NOTIFICATION)))
                {
                    wiced_bt_gatt_server_send_notification(
                        conn_id,
                        HDLC_CUSTOM_SERVICE_WIFI_CONNECTION_VALUE,
                        sizeof(app_custom_service_wifi_connection[0]),
                        app_custom_service_wifi_connection,
                        NULL);
                }
                else /* Notification not sent */
                {
                    printf("Notification not sent\n");
                }
            }
            else /* WiFi connection failed */
            {
                /* Update GATT DB about unsuccessful connection */
                app_custom_service_wifi_connection[0] = false;

                /* Send notification for unsuccessful connection */
                /* Check if the connection is active, notifications are enabled the connection
                   request had
                 * come from BLE client
                 */
                if ((conn_id != 0) &&
                    ((app_custom_service_wifi_connection_client_char_config[0] &
                      GATT_CLIENT_CONFIG_NOTIFICATION)))
                {
                    wiced_bt_gatt_server_send_notification(
                        conn_id,
                        HDLC_CUSTOM_SERVICE_WIFI_CONNECTION_VALUE,
                        sizeof(app_custom_service_wifi_connection[0]),
                        app_custom_service_wifi_connection,
                        NULL);
                }
                else /* Notification not sent */
                {
                    printf("Notification not sent\n");
                }

                printf("Failed to join Wi-Fi network\n");

                if ((0 == conn_id) &&
                    (BTM_BLE_ADVERT_OFF == wiced_bt_ble_get_current_advert_mode()))
                {
                    printf("Starting BLE ADV. Connect to BLE and provide"
                           "proper credentials\n");
                    /* Set the advertising params and make the device discoverable */
                    result = wiced_bt_ble_set_raw_advertisement_data(CY_BT_ADV_PACKET_DATA_SIZE,
                                                                     cy_bt_adv_packet_data);
                    if (WICED_SUCCESS != result)
                    {
                        printf("Set ADV data failed\n");
                    }

                    wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL);
                }
            }
        }
        /* Task notification for disconnection */
        else if ((NOTIF_DISCONNECT_GATT_DB == ulNotifiedValue) ||
                 (NOTIF_DISCONNECT_BTN == ulNotifiedValue))
        {
            if (NOTIF_DISCONNECT_BTN == ulNotifiedValue)
            {
                printf("Deleting Wi-Fi data from EMEEPROM\n");
                /* Set the data to 0*/
                memset(&wifi_details, 0, sizeof(wifi_details));
                #if EEPROM_SUPORT
                /* Write data to EEPROM. */
                // TODO: add code to read wifi_details from STM32 flash storage (internal, external
                // flash)
                #endif /* EEPROM_SUPORT */
            }

            printf("Disconnecting Wi-Fi\n");
            result = cy_wcm_disconnect_ap();
            if (result == CY_RSLT_SUCCESS)
            {
                /* Update GATT DB about disconnection */
                app_custom_service_wifi_connection[0] = false;

                printf("Successfully disconnected from AP\n");
                /* Start ADV if the device is not already connected or advertising */
                if ((0 == conn_id) &&
                    (BTM_BLE_ADVERT_OFF == wiced_bt_ble_get_current_advert_mode()))
                {
                    printf("Starting BLE ADV. Connect to BLE and provide "
                           "proper credentials\n");

                    result = wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL);

                    if (WICED_SUCCESS != result)
                    {
                        printf("Failed to start ADV");
                    }
                }
                else /* Connection is active */
                {
                    /* Send notification for successful disconnection */
                    /* Check if the connection is active, notifications are enabled the
                       disconnection request had
                     * come from BLE client
                     */
                    if ((conn_id != 0) &&
                        ((app_custom_service_wifi_connection_client_char_config[0] &
                          GATT_CLIENT_CONFIG_NOTIFICATION) &&
                         (NOTIF_DISCONNECT_GATT_DB == ulNotifiedValue)))
                    {
                        wiced_bt_gatt_server_send_notification(
                            conn_id,
                            HDLC_CUSTOM_SERVICE_WIFI_CONNECTION_VALUE,
                            sizeof(app_custom_service_wifi_connection[0]),
                            app_custom_service_wifi_connection,
                            NULL);
                    }
                    else /* Notification not sent */
                    {
                        printf("Notification not sent\n");
                    }
                }
            }
            else /* Disconnection failed */
            {
                printf("Failed to disconnect\n");
            }

            /* Set the variable as false as the disconnection was done */
            button_pressed = false;
        }
    }
}


/***************************************************************************************************
 * Function Name: scan_callback
 ***************************************************************************************************
 * Summary: The callback function which accumulates the scan results.
 * After completing the scan, it sends a task notification to wifi_task.
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
void scan_callback(cy_wcm_scan_result_t* result_ptr, void* user_data, cy_wcm_scan_status_t status)
{
    if ((strlen((const char*)result_ptr->SSID) != 0) && (status == CY_WCM_SCAN_INCOMPLETE))
    {
        wifi_conn_param.ap_credentials.security = result_ptr->security;
        printf("%-32s\t", result_ptr->SSID);
        printf(get_wifi_security_name(result_ptr->security));
        printf("\n");
    }

    if ((CY_WCM_SCAN_COMPLETE == status))
    {
        xTaskNotifyFromISR(wifi_task_handle, NOTIF_SCAN_COMPLETE, eSetValueWithOverwrite, pdFALSE);
    }
}


/* [] END OF FILE */
