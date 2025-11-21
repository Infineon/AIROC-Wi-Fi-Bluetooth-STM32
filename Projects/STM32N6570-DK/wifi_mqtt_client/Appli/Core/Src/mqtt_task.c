/******************************************************************************
 * File Name:   mqtt_task.c
 *
 * Description: This file contains the task that handles initialization &
 *              connection of Wi-Fi and the MQTT client. The task then starts
 *              the subscriber and the publisher tasks. The task also implements
 *              reconnection mechanisms to handle WiFi and MQTT disconnections.
 *              The task also handles all the cleanup operations to gracefully
 *              terminate the Wi-Fi and MQTT connections in case of any failure.
 *
 * Related Document: See README.md
 *
 *
 *******************************************************************************
 * Copyright 2020-2023, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
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
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 *******************************************************************************/

#include "cyhal.h"
#include "cybsp.h"

/* FreeRTOS header files */
#include "FreeRTOS.h"
#include "task.h"

/* Task header files */
#include "mqtt_task.h"
#include "subscriber_task.h"
#include "publisher_task.h"

/* Configuration file for Wi-Fi and MQTT client */
#include "wifi_config.h"
#include "mqtt_client_config.h"

/* Middleware libraries */
#include "cy_wcm.h"

#include "cy_mqtt_api.h"
#include "clock.h"

/* LwIP header files */
#include "lwip/netif.h"

#include "stm32_cyhal_sdio_ex.h"
#include "wifi_bt_if.h"
#include "stm32_cyhal_trng_ex.h"

/******************************************************************************
* Macros
******************************************************************************/
/* Queue length of a message queue that is used to communicate the status of
 * various operations.
 */
#define MQTT_TASK_QUEUE_LENGTH           (3u)

/* Time in milliseconds to wait before creating the publisher task. */
#define TASK_CREATION_DELAY_MS           (2000u)

/* Flag Masks for tracking which cleanup functions must be called. */
#define WCM_INITIALIZED                  (1lu << 0)
#define WIFI_CONNECTED                   (1lu << 1)
#define LIBS_INITIALIZED                 (1lu << 2)
#define BUFFER_INITIALIZED               (1lu << 3)
#define MQTT_INSTANCE_CREATED            (1lu << 4)
#define MQTT_CONNECTION_SUCCESS          (1lu << 5)
#define MQTT_MSG_RECEIVED                (1lu << 6)

/*String that describes the MQTT handle that is being created in order to uniquely identify it*/
#define MQTT_HANDLE_DESCRIPTOR            "MQTThandleID"

/* Macro to check if the result of an operation was successful and set the
 * corresponding bit in the status_flag based on 'init_mask' parameter. When
 * it has failed, print the error message and return the result to the
 * calling function.
 */
#define CHECK_RESULT(result, init_mask, error_message ...)      \
                     do                                        \
                     {                                         \
                         if ((int)result == CY_RSLT_SUCCESS)   \
                         {                                     \
                             status_flag |= init_mask;         \
                         }                                     \
                         else                                  \
                         {                                     \
                             printf(error_message);            \
                             return result;                    \
                         }                                     \
                     } while(0)

/******************************************************************************
 * Global Variables
 *******************************************************************************/
/* MQTT connection handle. */
cy_mqtt_t mqtt_connection;

/* Queue handle used to communicate results of various operations - MQTT
 * Publish, MQTT Subscribe, MQTT connection, and Wi-Fi connection between tasks
 * and callbacks.
 */
QueueHandle_t mqtt_task_q;

/* Flag to denote initialization status of various operations. */
uint32_t status_flag;

/* Pointer to the network buffer needed by the MQTT library for MQTT send and
 * receive operations.
 */
uint8_t* mqtt_network_buffer = NULL;

/******************************************************************************
 * Function Prototypes
 *******************************************************************************/
static cy_rslt_t wifi_connect(void);
static cy_rslt_t mqtt_init(void);
static cy_rslt_t mqtt_connect(void);

static void mqtt_event_callback(cy_mqtt_t mqtt_handle, cy_mqtt_event_t event, void* user_data);
static void cleanup(void);
void print_heap_usage(char* msg);

#if GENERATE_UNIQUE_CLIENT_ID
static cy_rslt_t mqtt_get_unique_client_identifier(char* mqtt_client_identifier);
#endif /* GENERATE_UNIQUE_CLIENT_ID */

SD_HandleTypeDef SDHandle = { .Instance = SDMMC2 };
extern RNG_HandleTypeDef   hrng;
void Error_Handler(void);

/* For Nucleo-H745ZI, Nucelo-H563ZI, and Nucleo-U575ZI */
#define SDMMC_D0    PC8
#define SDMMC_D1    PC9
#define SDMMC_D2    PC10
#define SDMMC_D3    PC11
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


/******************************************************************************
* Function Name: mqtt_client_task
******************************************************************************
* Summary:
*  Task for handling initialization & connection of Wi-Fi and the MQTT client.
*  The task also creates and manages the subscriber and publisher tasks upon
*  successful MQTT connection. The task also handles the WiFi and MQTT
*  connections by initiating reconnection on the event of disconnections.
*
* Parameters:
*  void *pvParameters : Task parameter defined during task creation (unused)
*
* Return:
*  void
*
******************************************************************************/
void mqtt_client_task(void* pvParameters)
{
    /* Structures that store the data to be sent/received to/from various
     * message queues.
     */
    mqtt_task_cmd_t mqtt_status;
    subscriber_data_t subscriber_q_data;
    publisher_data_t publisher_q_data;

    /* Configure the Wi-Fi interface as a Wi-Fi STA (i.e. Client). */
    cy_wcm_config_t config = { .interface = CY_WCM_INTERFACE_TYPE_STA };

    /* To avoid compiler warnings */
    (void)pvParameters;

    /* Create a message queue to communicate with other tasks and callbacks. */
    mqtt_task_q = xQueueCreate(MQTT_TASK_QUEUE_LENGTH, sizeof(mqtt_task_cmd_t));

    /* Workaround for Nucleo144-M.2 Adapter */
    toggle_sdmmc_data();

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

    /* Initialize the Wi-Fi Connection Manager and jump to the cleanup block
     * upon failure.
     */
    if (CY_RSLT_SUCCESS != cy_wcm_init(&config))
    {
        printf("\nWi-Fi Connection Manager initialization failed!\n");
        goto exit_cleanup;
    }

    /* Set the appropriate bit in the status_flag to denote successful
     * WCM initialization.
     */
    status_flag |= WCM_INITIALIZED;
    printf("\nWi-Fi Connection Manager initialized.\n");

    /* Initiate connection to the Wi-Fi AP and cleanup if the operation fails. */
    if (CY_RSLT_SUCCESS != wifi_connect())
    {
        goto exit_cleanup;
    }

    /* Set-up the MQTT client and connect to the MQTT broker. Jump to the
     * cleanup block if any of the operations fail.
     */
    if ((CY_RSLT_SUCCESS != mqtt_init()) || (CY_RSLT_SUCCESS != mqtt_connect()))
    {
        goto exit_cleanup;
    }

    /* Create the subscriber task and cleanup if the operation fails. */
    if (pdPASS != xTaskCreate(subscriber_task, "Subscriber task", SUBSCRIBER_TASK_STACK_SIZE,
                              NULL, SUBSCRIBER_TASK_PRIORITY, &subscriber_task_handle))
    {
        printf("Failed to create the Subscriber task!\n");
        goto exit_cleanup;
    }

    /* Wait for the subscribe operation to complete. */
    vTaskDelay(pdMS_TO_TICKS(TASK_CREATION_DELAY_MS));

    /* Create the publisher task and cleanup if the operation fails. */
    if (pdPASS != xTaskCreate(publisher_task, "Publisher task", PUBLISHER_TASK_STACK_SIZE,
                              NULL, PUBLISHER_TASK_PRIORITY, &publisher_task_handle))
    {
        printf("Failed to create Publisher task!\n");
        goto exit_cleanup;
    }

    print_heap_usage("mqtt_client_task: subscriber & publisher tasks created\n");

    while (true)
    {
        /* Wait for results of MQTT operations from other tasks and callbacks. */
        if (pdTRUE == xQueueReceive(mqtt_task_q, &mqtt_status, portMAX_DELAY))
        {
            /* In this code example, the disconnection from the MQTT Broker or
             * the Wi-Fi network is handled by the case 'HANDLE_DISCONNECTION'.
             *
             * The publish and subscribe failures (`HANDLE_MQTT_PUBLISH_FAILURE`
             * and `HANDLE_MQTT_SUBSCRIBE_FAILURE`) does not initiate
             * reconnection in this example, but they can be handled as per the
             * application requirement in the following swich cases.
             */
            switch (mqtt_status)
            {
                case HANDLE_MQTT_PUBLISH_FAILURE:
                {
                    /* Handle Publish Failure here. */
                    break;
                }

                case HANDLE_MQTT_SUBSCRIBE_FAILURE:
                {
                    /* Handle Subscribe Failure here. */
                    break;
                }

                case HANDLE_DISCONNECTION:
                {
                    /* Deinit the publisher before initiating reconnections. */
                    publisher_q_data.cmd = PUBLISHER_DEINIT;
                    xQueueSend(publisher_task_q, &publisher_q_data, portMAX_DELAY);

                    /* Although the connection with the MQTT Broker is lost,
                     * call the MQTT disconnect API for cleanup of threads and
                     * other resources before reconnection.
                     */
                    cy_mqtt_disconnect(mqtt_connection);

                    /* Check if Wi-Fi connection is active. If not, update the
                     * status flag and initiate Wi-Fi reconnection.
                     */
                    if (cy_wcm_is_connected_to_ap() == 0)
                    {
                        status_flag &= ~(WIFI_CONNECTED);
                        printf("\nInitiating Wi-Fi Reconnection...\n");
                        if (CY_RSLT_SUCCESS != wifi_connect())
                        {
                            goto exit_cleanup;
                        }
                    }

                    printf("\nInitiating MQTT Reconnection...\n");
                    if (CY_RSLT_SUCCESS != mqtt_connect())
                    {
                        goto exit_cleanup;
                    }

                    /* Initiate MQTT subscribe post the reconnection. */
                    subscriber_q_data.cmd = SUBSCRIBE_TO_TOPIC;
                    xQueueSend(subscriber_task_q, &subscriber_q_data, portMAX_DELAY);

                    /* Initialize Publisher post the reconnection. */
                    publisher_q_data.cmd = PUBLISHER_INIT;
                    xQueueSend(publisher_task_q, &publisher_q_data, portMAX_DELAY);
                    break;
                }

                default:
                    break;
            }
        }
    }

    /* Cleanup section: Delete subscriber and publisher tasks and perform
     * cleanup for various operations based on the status_flag.
     */
exit_cleanup:
    printf("\nTerminating Publisher and Subscriber tasks...\n");
    if (subscriber_task_handle != NULL)
    {
        vTaskDelete(subscriber_task_handle);
    }
    if (publisher_task_handle != NULL)
    {
        vTaskDelete(publisher_task_handle);
    }
    cleanup();
    printf("\nCleanup Done\nTerminating the MQTT task...\n\n");
    vTaskDelete(NULL);
}


/******************************************************************************
* Function Name: wifi_connect
******************************************************************************
* Summary:
*  Function that initiates connection to the Wi-Fi Access Point using the
*  specified SSID and PASSWORD. The connection is retried a maximum of
*  'MAX_WIFI_CONN_RETRIES' times with interval of 'WIFI_CONN_RETRY_INTERVAL_MS'
*  milliseconds.
*
* Parameters:
*  void
*
* Return:
*  cy_rslt_t : CY_RSLT_SUCCESS upon a successful Wi-Fi connection, else an
*              error code indicating the failure.
*
******************************************************************************/
static cy_rslt_t wifi_connect(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    cy_wcm_connect_params_t connect_param;
    cy_wcm_ip_address_t ip_address;

    /* Check if Wi-Fi connection is already established. */
    if (cy_wcm_is_connected_to_ap() == 0)
    {
        /* Configure the connection parameters for the Wi-Fi interface. */
        memset(&connect_param, 0, sizeof(cy_wcm_connect_params_t));
        memcpy(connect_param.ap_credentials.SSID, WIFI_SSID, sizeof(WIFI_SSID));
        memcpy(connect_param.ap_credentials.password, WIFI_PASSWORD, sizeof(WIFI_PASSWORD));
        connect_param.ap_credentials.security = WIFI_SECURITY;

        printf("\nWi-Fi Connecting to '%s'\n", connect_param.ap_credentials.SSID);

        /* Connect to the Wi-Fi AP. */
        for (uint32_t retry_count = 0; retry_count < MAX_WIFI_CONN_RETRIES; retry_count++)
        {
            result = cy_wcm_connect_ap(&connect_param, &ip_address);

            if (result == CY_RSLT_SUCCESS)
            {
                printf("\nSuccessfully connected to Wi-Fi network '%s'.\n",
                       connect_param.ap_credentials.SSID);

                /* Set the appropriate bit in the status_flag to denote
                 * successful Wi-Fi connection, print the assigned IP address.
                 */
                status_flag |= WIFI_CONNECTED;
                if (ip_address.version == CY_WCM_IP_VER_V4)
                {
                    printf("IPv4 Address Assigned: %s\n\n",
                           ip4addr_ntoa((const ip4_addr_t*)&ip_address.ip.v4));
                }
                else if (ip_address.version == CY_WCM_IP_VER_V6)
                {
                    printf("IPv6 Address Assigned: %s\n\n",
                           ip6addr_ntoa((const ip6_addr_t*)&ip_address.ip.v6));
                }
                return result;
            }

            printf(
                "Wi-Fi Connection failed. Error code:0x%0X. Retrying in %d ms. Retries left: %d\n",
                (int)result, WIFI_CONN_RETRY_INTERVAL_MS,
                (int)(MAX_WIFI_CONN_RETRIES - retry_count - 1));
            vTaskDelay(pdMS_TO_TICKS(WIFI_CONN_RETRY_INTERVAL_MS));
        }

        printf("\nExceeded maximum Wi-Fi connection attempts!\n");
        printf("Wi-Fi connection failed after retrying for %d mins\n\n",
               (int)(WIFI_CONN_RETRY_INTERVAL_MS * MAX_WIFI_CONN_RETRIES) / 60000u);
    }
    return result;
}


/******************************************************************************
* Function Name: mqtt_init
******************************************************************************
* Summary:
*  Function that initializes the MQTT library and creates an instance for the
*  MQTT client. The network buffer needed by the MQTT library for MQTT send
*  send and receive operations is also allocated by this function.
*
* Parameters:
*  void
*
* Return:
*  cy_rslt_t : CY_RSLT_SUCCESS on a successful initialization, else an error
*              code indicating the failure.
*
******************************************************************************/
static cy_rslt_t mqtt_init(void)
{
    /* Variable to indicate status of various operations. */
    cy_rslt_t result = CY_RSLT_SUCCESS;

    /* Initialize the MQTT library. */
    result = cy_mqtt_init();
    CHECK_RESULT(result, LIBS_INITIALIZED, "\nMQTT library initialization failed!\n");

    /* Allocate buffer for MQTT send and receive operations. */
    mqtt_network_buffer = (uint8_t*)pvPortMalloc(sizeof(uint8_t) * MQTT_NETWORK_BUFFER_SIZE);
    if (mqtt_network_buffer == NULL)
    {
        result = ~CY_RSLT_SUCCESS;
    }
    CHECK_RESULT(result, BUFFER_INITIALIZED, "Network Buffer allocation failed!\n\n");

    /* Create the MQTT client instance. */
    result = cy_mqtt_create(mqtt_network_buffer, MQTT_NETWORK_BUFFER_SIZE,
                            security_info, &broker_info, MQTT_HANDLE_DESCRIPTOR,
                            &mqtt_connection);

    CHECK_RESULT(result, MQTT_INSTANCE_CREATED, "\nMQTT instance creation failed!\n");
    if (CY_RSLT_SUCCESS == result)
    {
        /* Register a MQTT event callback */
        result = cy_mqtt_register_event_callback(mqtt_connection,
                                                 (cy_mqtt_callback_t)mqtt_event_callback, NULL);
        if (CY_RSLT_SUCCESS == result)
        {
            printf("\nMQTT library initialization successful.\n");
        }
    }
    return result;
}


/******************************************************************************
* Function Name: mqtt_connect
******************************************************************************
* Summary:
*  Function that initiates MQTT connect operation. The connection is retried
*  a maximum of 'MAX_MQTT_CONN_RETRIES' times with interval of
*  'MQTT_CONN_RETRY_INTERVAL_MS' milliseconds.
*
* Parameters:
*  void
*
* Return:
*  cy_rslt_t : CY_RSLT_SUCCESS upon a successful MQTT connection, else an
*              error code indicating the failure.
*
******************************************************************************/
static cy_rslt_t mqtt_connect(void)
{
    /* Variable to indicate status of various operations. */
    cy_rslt_t result = CY_RSLT_SUCCESS;

    /* MQTT client identifier string. */
    char mqtt_client_identifier[(MQTT_CLIENT_IDENTIFIER_MAX_LEN + 1)] = MQTT_CLIENT_IDENTIFIER;

    /* Configure the user credentials as a part of MQTT Connect packet */
    if (strlen(MQTT_USERNAME) > 0)
    {
        connection_info.username = MQTT_USERNAME;
        connection_info.password = MQTT_PASSWORD;
        connection_info.username_len = sizeof(MQTT_USERNAME) - 1;
        connection_info.password_len = sizeof(MQTT_PASSWORD) - 1;
    }

    /* Generate a unique client identifier with 'MQTT_CLIENT_IDENTIFIER' string
     * as a prefix if the `GENERATE_UNIQUE_CLIENT_ID` macro is enabled.
     */
    #if GENERATE_UNIQUE_CLIENT_ID
    result = mqtt_get_unique_client_identifier(mqtt_client_identifier);
    CHECK_RESULT(result, 0, "Failed to generate unique client identifier for the MQTT client!\n");
    #endif /* GENERATE_UNIQUE_CLIENT_ID */

    /* Set the client identifier buffer and length. */
    connection_info.client_id = mqtt_client_identifier;
    connection_info.client_id_len = strlen(mqtt_client_identifier);

    printf("\n'%.*s' connecting to MQTT broker '%.*s'...\n",
           connection_info.client_id_len,
           connection_info.client_id,
           broker_info.hostname_len,
           broker_info.hostname);

    for (uint32_t retry_count = 0; retry_count < MAX_MQTT_CONN_RETRIES; retry_count++)
    {
        if (cy_wcm_is_connected_to_ap() == 0)
        {
            printf(
                "\nUnexpectedly disconnected from Wi-Fi network! \nInitiating Wi-Fi reconnection...\n");
            status_flag &= ~(WIFI_CONNECTED);

            /* Initiate Wi-Fi reconnection. */
            result = wifi_connect();
            if (CY_RSLT_SUCCESS != result)
            {
                return result;
            }
        }

        /* Establish the MQTT connection. */
        result = cy_mqtt_connect(mqtt_connection, &connection_info);

        if (result == CY_RSLT_SUCCESS)
        {
            printf("MQTT connection successful.\r\n");

            /* Set the appropriate bit in the status_flag to denote successful
             * MQTT connection, and return the result to the calling function.
             */
            status_flag |= MQTT_CONNECTION_SUCCESS;
            return result;
        }

        printf(
            "\nMQTT connection failed with error code 0x%0X. \nRetrying in %d ms. Retries left: %d\n",
            (int)result, MQTT_CONN_RETRY_INTERVAL_MS,
            (int)(MAX_MQTT_CONN_RETRIES - retry_count - 1));
        vTaskDelay(pdMS_TO_TICKS(MQTT_CONN_RETRY_INTERVAL_MS));
    }

    printf("\nExceeded maximum MQTT connection attempts\n");
    printf("MQTT connection failed after retrying for %d mins\n\n",
           (int)(MQTT_CONN_RETRY_INTERVAL_MS * MAX_MQTT_CONN_RETRIES) / 60000u);
    return result;
}


/******************************************************************************
* Function Name: mqtt_event_callback
******************************************************************************
* Summary:
*  Callback invoked by the MQTT library for events like MQTT disconnection,
*  incoming MQTT subscription messages from the MQTT broker.
*    1. In case of MQTT disconnection, the MQTT client task is communicated
*       about the disconnection using a message queue.
*    2. When an MQTT subscription message is received, the subscriber callback
*       function implemented in subscriber_task.c is invoked to handle the
*       incoming MQTT message.
*
* Parameters:
*  cy_mqtt_t mqtt_handle : MQTT handle corresponding to the MQTT event (unused)
*  cy_mqtt_event_t event : MQTT event information
*  void *user_data : User data pointer passed during cy_mqtt_create() (unused)
*
* Return:
*  void
*
******************************************************************************/
static void mqtt_event_callback(cy_mqtt_t mqtt_handle, cy_mqtt_event_t event, void* user_data)
{
    cy_mqtt_publish_info_t* received_msg;
    mqtt_task_cmd_t mqtt_task_cmd;

    (void)mqtt_handle;
    (void)user_data;

    switch (event.type)
    {
        case CY_MQTT_EVENT_TYPE_DISCONNECT:
        {
            /* Clear the status flag bit to indicate MQTT disconnection. */
            status_flag &= ~(MQTT_CONNECTION_SUCCESS);

            /* MQTT connection with the MQTT broker is broken as the client
             * is unable to communicate with the broker. Set the appropriate
             * command to be sent to the MQTT task.
             */
            printf("\nUnexpectedly disconnected from MQTT broker!\n");
            mqtt_task_cmd = HANDLE_DISCONNECTION;

            /* Send the message to the MQTT client task to handle the
             * disconnection.
             */
            xQueueSend(mqtt_task_q, &mqtt_task_cmd, portMAX_DELAY);
            break;
        }

        case CY_MQTT_EVENT_TYPE_SUBSCRIPTION_MESSAGE_RECEIVE:
        {
            status_flag |= MQTT_MSG_RECEIVED;

            /* Incoming MQTT message has been received. Send this message to
             * the subscriber callback function to handle it.
             */

            received_msg = &(event.data.pub_msg.received_message);

            mqtt_subscription_callback(received_msg);
            break;
        }

        default:
        {
            /* Unknown MQTT event */
            printf("\nUnknown Event received from MQTT callback!\n");
            break;
        }
    }
}


#if GENERATE_UNIQUE_CLIENT_ID
/******************************************************************************
* Function Name: mqtt_get_unique_client_identifier
******************************************************************************
* Summary:
*  Function that generates unique client identifier for the MQTT client by
*  appending a timestamp to a common prefix 'MQTT_CLIENT_IDENTIFIER'.
*
* Parameters:
*  char *mqtt_client_identifier : Pointer to the string that stores the
*                                 generated unique identifier
*
* Return:
*  cy_rslt_t : CY_RSLT_SUCCESS on successful generation of the client
*              identifier, else a non-zero value indicating failure.
*
******************************************************************************/
static cy_rslt_t mqtt_get_unique_client_identifier(char* mqtt_client_identifier)
{
    cy_rslt_t status = CY_RSLT_SUCCESS;

    /* Check for errors from snprintf. */
    if (0 > snprintf(mqtt_client_identifier,
                     (MQTT_CLIENT_IDENTIFIER_MAX_LEN + 1),
                     MQTT_CLIENT_IDENTIFIER "%lu",
                     (long unsigned int)Clock_GetTimeMs()))
    {
        status = ~CY_RSLT_SUCCESS;
    }

    return status;
}


#endif /* GENERATE_UNIQUE_CLIENT_ID */

/******************************************************************************
* Function Name: cleanup
******************************************************************************
* Summary:
*  Function that invokes the deinit and cleanup functions for various
*  operations based on the status_flag.
*
* Parameters:
*  void
*
* Return:
*  void
*
******************************************************************************/
static void cleanup(void)
{
    cy_rslt_t status = CY_RSLT_SUCCESS;

    /* Disconnect the MQTT connection if it was established. */
    if (status_flag & MQTT_CONNECTION_SUCCESS)
    {
        status = cy_mqtt_disconnect(mqtt_connection);

        if (status == CY_RSLT_SUCCESS)
        {
            printf("Disconnected from the MQTT Broker...\n");
        }
        else
        {
            printf("MQTT disconnect API failed unexpectedly.\n");
        }
    }
    /* Delete the MQTT instance if it was created. */
    if (status_flag & MQTT_INSTANCE_CREATED)
    {
        status = cy_mqtt_delete(mqtt_connection);

        if (status == CY_RSLT_SUCCESS)
        {
            printf("Removed MQTT connection info from stack...\n");
        }
        else
        {
            printf("MQTT delete API failed unexpectedly.\n");
        }
    }
    /* Deallocate the network buffer. */
    if (status_flag & BUFFER_INITIALIZED)
    {
        vPortFree((void*)mqtt_network_buffer);
    }
    /* Deinit the MQTT library. */
    if (status_flag & LIBS_INITIALIZED)
    {
        status = cy_mqtt_deinit();

        if (status == CY_RSLT_SUCCESS)
        {
            printf("Deinitialized MQTT stack...\n");
        }
        else
        {
            printf("MQTT deinit API failed unexpectedly.\n");
        }
    }
    /* Disconnect from Wi-Fi AP. */
    if (status_flag & WIFI_CONNECTED)
    {
        status = cy_wcm_disconnect_ap();

        if (status == CY_RSLT_SUCCESS)
        {
            printf("Disconnected from the Wi-Fi AP!\n");
        }
        else
        {
            printf("WCM disconnect AP failed unexpectedly.\n");
        }
    }
    /* De-initialize the Wi-Fi Connection Manager. */
    if (status_flag & WCM_INITIALIZED)
    {
        status = cy_wcm_deinit();

        if (status == CY_RSLT_SUCCESS)
        {
            printf("Deinitialized Wifi connection...\n");
        }
        else
        {
            printf("WCM deinit API failed unexpectedly.\n");
        }
    }
}


/***************************************************************************************************
 * _gettimeofday
 **************************************************************************************************/
int _gettimeofday(struct timeval* tv, void* timezone)
{
    cy_time_t time_ms;

    (void)timezone; /* Unused parameter */
    (void)cy_rtos_get_time(&time_ms);
    tv->tv_sec =  (time_ms / 1000);
    tv->tv_usec = (time_ms - (tv->tv_sec * 1000)) * 1000;

    return 0;
}


/* [] END OF FILE */
