/***************************************************************************************************
 * File Name:   app.c
 *
 * Description: This is the source code for the WiFi Onboarding Using BLE
 *              project for ModusToolbox.
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

#include <app_utils.h>
#include "cybsp.h"

#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_memory.h"
#include "cycfg_bt_settings.h"
#include "cycfg_gap.h"
#include "app_platform_cfg.h"
#include "wiced_bt_stack.h"
#include "cycfg_gatt_db.h"

#include "cy_utils.h"
#include "wifi_task.h"
#include "cyabs_rtos.h"
/***************************************************************************************************
 *                                Constants
 **************************************************************************************************/

/* Priority for the button interrupt */
#define GPIO_INTERRUPT_PRIORITY   (7)

/* Number of Advertisement elements */
#define ADV_ELEMENTS              (3)

/* LE Key Size */
#define MAX_KEY_SIZE              (0x10)

/* App buffer heap */
#define APP_BUFFER_HEAP           (0x400)

#define DEFAULT_PRIORITY         20
#define DEFAULT_MEMORY_SIZE      1024
/***************************************************************************************************
 *                             Global Variables
 **************************************************************************************************/

/* Maintains the connection id of the current connection */
uint16_t conn_id = 0;

/* This variable is set to true when button callback is received and
 * data is present in EM. It is set to false after the WiFi Task
 * processes Disconnection notification. It is used to check button
 * interrupt while the device is trying to connect to WiFi
 */
volatile bool button_pressed = false;

extern cy_event_t  wifi_event;

/* This enables RTOS aware debugging. */
volatile int uxTopUsedPriority;

/***************************************************************************************************
 *                              Function Prototypes
 **************************************************************************************************/
static wiced_result_t app_management_callback(wiced_bt_management_evt_t event,
                                              wiced_bt_management_evt_data_t* p_event_data);
static wiced_bt_gatt_status_t app_gatts_callback(wiced_bt_gatt_evt_t event,
                                                 wiced_bt_gatt_event_data_t* p_data);
static void application_init(void);
static void gpio_interrupt_handler(void* handler_arg, cyhal_gpio_event_t event);

static uint8_t* app_alloc_buffer(uint16_t len);
static void app_free_buffer(uint8_t* p_data);


typedef void (* pfn_free_buffer_t)(uint8_t*);


/***************************************************************************************************
 * Function Name: main
 ***************************************************************************************************
 * Summary:
 * This is the main function for CM7 CPU
 *    1. Configures the button for interrupt
 *    2. Initializes platform configuration
 *    3. Initializes BT stack and heap
 *    4. Creates WiFi connect and disconnect tasks
 *    5. Starts the RTOS scheduler
 *
 * Return:
 *  int
 *
 **************************************************************************************************/
void application_start(void)
{
    cyhal_gpio_callback_data_t callback_data =
    {
        .callback = gpio_interrupt_handler
    };

    /* Configure GPIO interrupt */
    cyhal_gpio_register_callback(CYBSP_USER_BTN, &callback_data);

    printf("***************************\n"
           "Wi-Fi Onboarding Using BLE ThreadX\n"
           "***************************\n\n");

    #if EEPROM_SUPORT
    /* Initialize the EMEEPROM. */

    // TODO: add code to initialize STM32 flash storage (internal, external flash)

    #endif /* EEPROM_SUPORT */


    /* Configure platform specific settings for Bluetooth */
    cybt_platform_config_init(&bt_platform_cfg_settings);

    /* Initialize the Bluetooth stack with a callback function and stack
     * configuration structure */
    if (WICED_SUCCESS != wiced_bt_stack_init(app_management_callback, &wiced_bt_cfg_settings))
    {
        printf("Error initializing BT stack\n");
        CY_ASSERT(0);
    }

    /* Create a buffer heap, make it the default heap.  */
    if (NULL == wiced_bt_create_heap("app", NULL, APP_BUFFER_HEAP, NULL, WICED_TRUE))
    {
        printf("Error creating a buffer heap\n");
        CY_ASSERT(0);
    }
}


/***************************************************************************************************
 * Function Name: application_init
 ***************************************************************************************************
 * Summary:
 * This function is called from the BTM enabled event
 *    1. Initializes and registers the GATT DB
 *    2. Sets pairable mode to true
 *    3. Sets ADV data and starts advertising
 *
 **************************************************************************************************/
void application_init(void)
{
    wiced_result_t         result;
    wiced_bt_gatt_status_t gatt_status;

    /* Enabling button interrupt */
    cyhal_gpio_enable_event(CYBSP_USER_BTN, CYHAL_GPIO_IRQ_FALL,
                            GPIO_INTERRUPT_PRIORITY, true);

    /* Register with stack to receive GATT callback */
    gatt_status = wiced_bt_gatt_register(app_gatts_callback);
    printf("\nGATT status:\t");
    printf(get_bt_gatt_status_name(gatt_status));
    printf("\n");

    /*  Inform the stack to use our GATT database */
    gatt_status =  wiced_bt_gatt_db_init(gatt_database, gatt_database_len, NULL);

    /* Allow peer to pair */
    wiced_bt_set_pairable_mode(WICED_TRUE, false);

    result = wiced_bt_ble_set_raw_advertisement_data(CY_BT_ADV_PACKET_DATA_SIZE,
                                                     cy_bt_adv_packet_data);
    if (WICED_SUCCESS != result)
    {
        printf("Set ADV data failed\n");
    }

    #if EEPROM_SUPORT
    /* Determine if the EEPROM has a previously stored WiFi SSID value.
     * If it does, use the stored credentials to connect to WiFi.
     * Otherwise, start BLE advertisements so that the user can use BLE
     * to enter the WiFi credentials. */

    // TODO: add code to read wifi_details from STM32 flash storage (internal, external flash)

    if (0 != wifi_details.ssid_len)
    {
        printf("Data present in EMEEPROM\n");
        /* Unblock WiFi task with notification value indicating the WiFi credentials
         * should be taken from EMEEPROM
         */
        //xTaskNotify(wifi_task_handle, NOTIF_EMEEPROM, eSetValueWithOverwrite);
    }
    else /* WiFi credentials not found in EMEEPROM */
    #endif /* EEPROM_SUPORT */
    {
        printf("Data not present in EMEEPROM\n");
        wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_LOW,
                                      BLE_ADDR_PUBLIC, NULL);
    }
}


/***************************************************************************************************
 * Function Name: app_management_callback
 ***************************************************************************************************
 * Summary:
 * This function handles the BT stack events.
 *
 * Parameters:
 *  wiced_bt_management_evt_t: event code
 *  p_event_data: Pointer to the event data
 *
 * Return:
 *  wiced_result_t: Result
 *
 **************************************************************************************************/
wiced_result_t app_management_callback(wiced_bt_management_evt_t event,
                                       wiced_bt_management_evt_data_t* p_event_data)
{
    wiced_result_t            result = WICED_BT_SUCCESS;
    wiced_bt_device_address_t bda    = { 0 };

    printf("Bluetooth Management Event: \t");
    printf(get_bt_event_name(event));
    printf("\n");

    switch (event)
    {
        case BTM_ENABLED_EVT:
            /* Initialize the application */
            wiced_bt_set_local_bdaddr((uint8_t*)cy_bt_device_address, BLE_ADDR_PUBLIC);
            /* Bluetooth is enabled */
            wiced_bt_dev_read_local_addr(bda);
            printf("Local Bluetooth Address: ");
            print_bd_address(bda);

            application_init();
            break;

        case BTM_DISABLED_EVT:
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
            p_event_data->pairing_io_capabilities_ble_request.local_io_cap =
                BTM_IO_CAPABILITIES_NONE;

            p_event_data->pairing_io_capabilities_ble_request.oob_data = BTM_OOB_NONE;

            p_event_data->pairing_io_capabilities_ble_request.auth_req = BTM_LE_AUTH_REQ_SC;

            p_event_data->pairing_io_capabilities_ble_request.max_key_size = MAX_KEY_SIZE;

            p_event_data->pairing_io_capabilities_ble_request.init_keys =
                BTM_LE_KEY_PENC |
                BTM_LE_KEY_PID |
                BTM_LE_KEY_PCSRK |
                BTM_LE_KEY_LENC;

            p_event_data->pairing_io_capabilities_ble_request.resp_keys =
                BTM_LE_KEY_PENC|
                BTM_LE_KEY_PID|
                BTM_LE_KEY_PCSRK|
                BTM_LE_KEY_LENC;
            break;

        case BTM_PAIRING_COMPLETE_EVT:
            if (WICED_SUCCESS == p_event_data->pairing_complete.pairing_complete_info.ble.status)
            {
                printf("Pairing Complete: SUCCESS\n");
            }
            else /* Pairing Failed */
            {
                printf("Pairing Complete: FAILED\n");
            }
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
            /* Paired Device Link Keys update */
            result = WICED_SUCCESS;
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
            /* Paired Device Link Keys Request */
            result = WICED_BT_ERROR;
            break;

        case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
            /* Local identity Keys Update */
            result = WICED_SUCCESS;
            break;

        case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
            /* Local identity Keys Request */
            result = WICED_BT_ERROR;
            break;

        case BTM_ENCRYPTION_STATUS_EVT:
            if (WICED_SUCCESS == p_event_data->encryption_status.result)
            {
                printf("Encryption Status Event: SUCCESS\n");
            }
            else /* Encryption Failed */
            {
                printf("Encryption Status Event: FAILED\n");
            }
            break;

        case BTM_SECURITY_REQUEST_EVT:
            wiced_bt_ble_security_grant(p_event_data->security_request.bd_addr,
                                        WICED_BT_SUCCESS);
            break;

        case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
            printf("\n");
            printf("Advertisement state changed to ");
            printf(get_bt_advert_mode_name(p_event_data->ble_advert_state_changed));
            printf("\n");
            break;

        default:
            break;
    }

    return result;
}


/***************************************************************************************************
 * Function Name: app_get_attribute
 ***************************************************************************************************
 * Summary:
 * This function searches through the GATT DB to point to the attribute
 * corresponding to the given handle
 *
 * Parameters:
 *  uint16_t handle: Handle to search for in the GATT DB
 *
 * Return:
 *  gatt_db_lookup_table_t *: Pointer to the correct attribute in the GATT DB
 *
 **************************************************************************************************/
gatt_db_lookup_table_t* app_get_attribute(uint16_t handle)
{
    /* Search for the given handle in the GATT DB and return the pointer to the
       correct attribute */
    uint8_t array_index = 0;

    for (array_index = 0; array_index < app_gatt_db_ext_attr_tbl_size; array_index++)
    {
        if (app_gatt_db_ext_attr_tbl[array_index].handle == handle)
        {
            return (&app_gatt_db_ext_attr_tbl[array_index]);
        }
    }
    return NULL;
}


/***************************************************************************************************
 * Function Name: app_gatts_req_read_handler
 ***************************************************************************************************
 * Summary:
 * This function handles the GATT read request events from the stack
 *
 * Parameters:
 *  uint16_t conn_id: Connection ID
 *  wiced_bt_gatt_read_t * p_read_data: Read data structure
 *
 * Return:
 *  wiced_bt_gatt_status_t: GATT result
 *
 **************************************************************************************************/
wiced_bt_gatt_status_t app_gatts_req_read_handler(uint16_t conn_id,
                                                  wiced_bt_gatt_opcode_t opcode,
                                                  wiced_bt_gatt_read_t* p_read_req,
                                                  uint16_t len_requested)
{
    gatt_db_lookup_table_t* puAttribute;
    uint16_t                attr_len_to_copy;
    uint8_t*                from;

    if ((puAttribute = app_get_attribute(p_read_req->handle)) == NULL)
    {
        printf("[%s]  attr not found handle: 0x%04x\n", __FUNCTION__, p_read_req->handle);
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->handle,
                                            WICED_BT_GATT_INVALID_HANDLE);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    attr_len_to_copy = puAttribute->cur_len;

    printf("[%s] conn_id: %d handle:0x%04x offset:%d len:%d\n",
           __FUNCTION__, conn_id, p_read_req->handle, p_read_req->offset, attr_len_to_copy);

    if (p_read_req->offset >= puAttribute->cur_len)
    {
        printf("[%s] offset:%d larger than attribute length:%d\n", __FUNCTION__,
               p_read_req->offset, puAttribute->cur_len);

        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->handle,
                                            WICED_BT_GATT_INVALID_OFFSET);
        return (WICED_BT_GATT_INVALID_HANDLE);
    }

    int to_send = MIN(len_requested, attr_len_to_copy - p_read_req->offset);

    from = ((uint8_t*)puAttribute->p_data) + p_read_req->offset;

    wiced_bt_gatt_server_send_read_handle_rsp(conn_id, opcode, to_send, from, NULL);

    return WICED_BT_GATT_SUCCESS;
}


/***************************************************************************************************
 * Function Name: app_gatts_req_write_handler
 ***************************************************************************************************
 * Summary:
 * This function handles the GATT write request events from the stack
 *
 * Parameters:
 *  uint16_t conn_id: Connection ID
 *  wiced_bt_gatt_write_t * p_data: Write data structure
 *
 * Return:
 *  wiced_bt_gatt_status_t: GATT result
 *
 **************************************************************************************************/
wiced_bt_gatt_status_t app_gatts_req_write_handler(uint16_t conn_id,
                                                   wiced_bt_gatt_opcode_t opcode,
                                                   wiced_bt_gatt_write_req_t* p_data)
{
    wiced_bt_gatt_status_t  result = WICED_BT_GATT_SUCCESS;
    uint8_t*                p_attr = p_data->p_val;
    gatt_db_lookup_table_t* puAttribute;

    printf("GATT write handler: handle:0x%X len:%d\n",
           p_data->handle, p_data->val_len);

    /* Get the right address for the handle in Gatt DB */
    if (NULL == (puAttribute = app_get_attribute(p_data->handle)))
    {
        printf("\nWrite Handle attr not found. Handle:0x%X\n", p_data->handle);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    switch (p_data->handle)
    {
        /* Write request for the WiFi SSID characteristic. Copy the incoming data
           to the WIFI SSID variable */
        case HDLC_CUSTOM_SERVICE_WIFI_SSID_VALUE:
            memset(app_custom_service_wifi_ssid, 0, strlen((char*)app_custom_service_wifi_ssid));
            memcpy(app_custom_service_wifi_ssid, p_attr, p_data->val_len);
            puAttribute->cur_len = p_data->val_len;
            printf("Wi-Fi SSID: %s\n", app_custom_service_wifi_ssid);
            break;

        /* Write request for the WiFi password characteristic. Accept the password.
           Copy the incoming data to the WIFI Password variable */
        case HDLC_CUSTOM_SERVICE_WIFI_PASSWORD_VALUE:
            memset(app_custom_service_wifi_password, 0,
                   strlen((char*)app_custom_service_wifi_password));
            memcpy(app_custom_service_wifi_password, p_attr, p_data->val_len);
            puAttribute->cur_len = p_data->val_len;
            printf("Wi-Fi Password: %s\n", app_custom_service_wifi_password);
            break;

        /* Write request for the connection characteristic. Copy the incoming data
           to the WIFI connection variable. Based on the value either start connect
           or disconnect procedure */
        case HDLC_CUSTOM_SERVICE_WIFI_CONNECTION_VALUE:
            app_custom_service_wifi_connection[0] = p_attr[0];
            puAttribute->cur_len                  = p_data->val_len;

            /* Connection message */
            if (app_custom_service_wifi_connection[0])
            {
                /* Unblock WiFiConnect task with notification value indicating the WiFi credentials
                 * should be taken from GATT DB
                 */
                cy_rtos_setbits_event(&wifi_event, NOTIF_GATT_DB, 0);
            }
            else /* Disconnection message */
            {
                cy_rtos_setbits_event(&wifi_event, NOTIF_DISCONNECT_GATT_DB, 0);
            }
            break;

        /* Notification for connection characteristic. If enabled, notification can
         * be sent to the client if the connection was successful or not
         */
        case HDLD_CUSTOM_SERVICE_WIFI_CONNECTION_CLIENT_CHAR_CONFIG:
            app_custom_service_wifi_connection_client_char_config[0] = p_attr[0];
            app_custom_service_wifi_connection_client_char_config[1] = p_attr[1];
            break;

        default:
            printf("Write GATT Handle not found\n");
            result = WICED_BT_GATT_INVALID_HANDLE;
            break;
    }

    return result;
}


/***************************************************************************************************
 * Function Name: app_gatt_connect_callback
 ***************************************************************************************************
 * Summary:
 * This function handles the GATT connect request events from the stack
 *
 * Parameters:
 *  wiced_bt_gatt_connection_status_t *p_conn_status: Connection or disconnection
 *
 * Return:
 *  wiced_bt_gatt_status_t: GATT result
 *
 **************************************************************************************************/
wiced_bt_gatt_status_t app_gatt_connect_callback(wiced_bt_gatt_connection_status_t* p_conn_status)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;
    wiced_result_t         result;

    /* Check whether it is a connect event or disconnect event. If the device
       has been disconnected then restart advertisement */
    if (NULL != p_conn_status)
    {
        if (p_conn_status->connected)
        {
            /* Device got connected */
            printf("\nConnected: Peer BD Address: ");
            print_bd_address(p_conn_status->bd_addr);
            printf("\n");
            conn_id = p_conn_status->conn_id;
        }
        else /* Device got disconnected */
        {
            printf("\nDisconnected: Peer BD Address: ");
            print_bd_address(p_conn_status->bd_addr);
            printf("\n");

            printf("Reason for disconnection: \t");
            printf(get_bt_gatt_disconn_reason_name(p_conn_status->reason));
            printf("\n");

            conn_id = 0;

            result = wiced_bt_ble_set_raw_advertisement_data(CY_BT_ADV_PACKET_DATA_SIZE,
                                                             cy_bt_adv_packet_data);
            if (WICED_SUCCESS != result)
            {
                printf("Set ADV data failed\n");
            }

            wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL);
        }
        status = WICED_BT_GATT_SUCCESS;
    }

    return status;
}


/*
 * Process write read-by-type request from peer device
 */
wiced_bt_gatt_status_t apps_gatts_req_read_by_type_handler(uint16_t conn_id,
                                                           wiced_bt_gatt_opcode_t opcode,
                                                           wiced_bt_gatt_read_by_type_t* p_read_req,
                                                           uint16_t len_requested)
{
    gatt_db_lookup_table_t* puAttribute;
    uint16_t                attr_handle = p_read_req->s_handle;
    uint8_t*                p_rsp       = app_alloc_buffer(len_requested);
    uint8_t                 pair_len    = 0;
    int                     used        = 0;

    if (p_rsp == NULL)
    {
        printf("[%s]  no memory len_requested: %d!!\n", __FUNCTION__, len_requested);
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, attr_handle,
                                            WICED_BT_GATT_INSUF_RESOURCE);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Read by type returns all attributes of the specified type,
     * between the start and end handles */
    while (WICED_TRUE)
    {
        /* Add your code here */
        attr_handle = wiced_bt_gatt_find_handle_by_type(attr_handle, p_read_req->e_handle,
                                                        &p_read_req->uuid);

        if (attr_handle == 0)
        {
            break;
        }

        if ((puAttribute = app_get_attribute(attr_handle)) == NULL)
        {
            wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->s_handle,
                                                WICED_BT_GATT_ERR_UNLIKELY);
            app_free_buffer(p_rsp);
            return WICED_BT_GATT_INVALID_HANDLE;
        }

        {
            int filled = wiced_bt_gatt_put_read_by_type_rsp_in_stream(p_rsp + used,
                                                                      len_requested - used,
                                                                      &pair_len,
                                                                      attr_handle,
                                                                      puAttribute->cur_len,
                                                                      puAttribute->p_data);
            if (filled == 0)
            {
                break;
            }
            used += filled;
        }

        /* Increment starting handle for next search to one past current */
        attr_handle++;
    }

    if (used == 0)
    {
        printf("[%s]  attr not found  start_handle: 0x%04x  end_handle: 0x%04x  Type: 0x%04x\n",
               __FUNCTION__, p_read_req->s_handle, p_read_req->e_handle,
               p_read_req->uuid.uu.uuid16);

        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->s_handle,
                                            WICED_BT_GATT_INVALID_HANDLE);
        app_free_buffer(p_rsp);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Send the response */
    wiced_bt_gatt_server_send_read_by_type_rsp(conn_id, opcode, pair_len, used, p_rsp,
                                               (wiced_bt_gatt_app_context_t)app_free_buffer);

    return WICED_BT_GATT_SUCCESS;
}


/***************************************************************************************************
 * Function Name: app_gatts_req_cb
 ***************************************************************************************************
 * Summary:
 * This function redirects the GATT attribute requests to the appropriate functions
 *
 * Parameters:
 *  wiced_bt_gatt_attribute_request_t *p_data: GATT request data structure
 *
 * Return:
 *  wiced_bt_gatt_status_t: GATT result
 *
 **************************************************************************************************/

wiced_bt_gatt_status_t app_gatts_req_cb(wiced_bt_gatt_attribute_request_t* p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;
    switch (p_data->opcode)
    {
        case GATT_REQ_READ:
            result = app_gatts_req_read_handler(p_data->conn_id, p_data->opcode,
                                                &p_data->data.read_req,
                                                p_data->len_requested);
            break;

        case GATT_REQ_WRITE:
            result =
                app_gatts_req_write_handler(p_data->conn_id, p_data->opcode,
                                            &(p_data->data.write_req));
            if ((p_data->opcode == GATT_REQ_WRITE) && (result == WICED_BT_GATT_SUCCESS))
            {
                wiced_bt_gatt_write_req_t* p_write_request = &p_data->data.write_req;
                wiced_bt_gatt_server_send_write_rsp(p_data->conn_id, p_data->opcode,
                                                    p_write_request->handle);
            }
            break;

        case GATT_REQ_MTU:
            printf("req_mtu: %d\n", p_data->data.remote_mtu);
            wiced_bt_gatt_server_send_mtu_rsp(p_data->conn_id,
                                              p_data->data.remote_mtu,
                                              wiced_bt_cfg_settings.p_ble_cfg->ble_max_rx_pdu_size);
            result = WICED_BT_GATT_SUCCESS;
            break;

        case GATT_REQ_READ_BY_TYPE:
            result = apps_gatts_req_read_by_type_handler(p_data->conn_id, p_data->opcode,
                                                         &p_data->data.read_by_type,
                                                         p_data->len_requested);
            break;

        default:
            printf("-> %d\n", p_data->opcode);

            break;
    }
    return result;
}


/***************************************************************************************************
 * Function Name: app_gatts_callback
 ***************************************************************************************************
 * Summary:
 * This function redirects the GATT requests to the appropriate functions
 *
 * Parameters:
 *  wiced_bt_gatt_attribute_request_t *p_data: GATT request data structure
 *
 * Return:
 *  wiced_bt_gatt_status_t: GATT result
 *
 **************************************************************************************************/
wiced_bt_gatt_status_t app_gatts_callback(wiced_bt_gatt_evt_t event,
                                          wiced_bt_gatt_event_data_t* p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;

    switch (event)
    {
        case GATT_CONNECTION_STATUS_EVT:
            result = app_gatt_connect_callback(&p_data->connection_status);
            break;

        case GATT_ATTRIBUTE_REQUEST_EVT:
            result = app_gatts_req_cb(&p_data->attribute_request);
            break;


        case GATT_GET_RESPONSE_BUFFER_EVT:
            p_data->buffer_request.buffer.p_app_rsp_buffer = app_alloc_buffer(
                p_data->buffer_request.len_requested);
            p_data->buffer_request.buffer.p_app_ctxt =
                (wiced_bt_gatt_app_context_t)app_free_buffer;
            result = WICED_BT_GATT_SUCCESS;
            break;

        case GATT_APP_BUFFER_TRANSMITTED_EVT:
        {
            pfn_free_buffer_t pfn_free = (pfn_free_buffer_t)p_data->buffer_xmitted.p_app_ctxt;

            /* If the buffer is dynamic, the context will point to a function to free it. */
            if (pfn_free)
            {
                pfn_free(p_data->buffer_xmitted.p_app_data);
            }

            result = WICED_BT_GATT_SUCCESS;
        } break;

        default:
            break;
    }
    return result;
}


/***************************************************************************************************
 * Function Name: gpio_interrupt_handler
 **************************************************************************************************
 * Summary:
 *  GPIO interrupt service routine. This function detects button presses
 *  and passes a notification to the WiFi task to disconnect from WiFi and
 *  delete EMEEPROM data.
 *
 *
 * Parameters:
 *  void *callback_arg : pointer to variable passed to the ISR
 *  cyhal_gpio_event_t event : GPIO event type
 *
 **************************************************************************************************/
void gpio_interrupt_handler(void* handler_arg, cyhal_gpio_event_t event)
{
    /* Notify the WiFi task to disconnect */
    button_pressed = true;
    cy_rtos_setbits_event(&wifi_event, NOTIF_DISCONNECT_BTN, 0);
}


/***************************************************************************************************
 * app_alloc_buffer
 **************************************************************************************************/
static uint8_t* app_alloc_buffer(uint16_t len)
{
    uint8_t* p = (uint8_t*)wiced_bt_get_buffer(len);
    printf("[%s] len %d alloc 0x%x \n", __FUNCTION__, len, (unsigned int)p);

    return p;
}


/***************************************************************************************************
 * app_free_buffer
 **************************************************************************************************/
static void app_free_buffer(uint8_t* p_data)
{
    wiced_bt_free_buffer(p_data);

    printf("[%s] 0x%x \n", __FUNCTION__, (unsigned int)p_data);
}


/* [] END OF FILE */
