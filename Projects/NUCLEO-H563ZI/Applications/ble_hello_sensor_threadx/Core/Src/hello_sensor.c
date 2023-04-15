/***************************************************************************************************
 * File Name: hello_sensor.c
 *
 * This code example demonstrates the implementation of a simple Bluetooth Stack
 * functionality in GAP Peripheral role. During initialization the app registers
 * with LE stack to receive various notifications including bonding complete,
 * connection status change and peer write. Peer device can also write in to client
 * configuration descriptor of the notification characteristic.
 *
 * Features demonstrated
 * - GATT database and Device configuration initialization
 * - Registration with LE stack for various events
 * - Sending data to the client
 * - Processing write requests from the client
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

#include <string.h>
#include "wiced_bt_dev.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_uuid.h"
#include "wiced_result.h"
#include "hello_sensor.h"
#include "wiced_bt_stack.h"
#include "wiced_memory.h"
#include "hello_sensor.h"

#include "wiced_bt_types.h"
#include "wiced_timer.h"

#include "cybt_platform_trace.h"

/* Redirect WICED_BT_TRACE to APP_TRACE_API (from bluetooth-freertos) */
#undef WICED_BT_TRACE
#define WICED_BT_TRACE cybt_platform_log_print


/***************************************************************************************************
 *                                Constants
 **************************************************************************************************/
#define HELLO_SENSOR_GATTS_MAX_CONN     1
#define APP_BUFFER_HEAP                 0x1000

/***************************************************************************************************
 *                                Structures
 **************************************************************************************************/
typedef struct
{
    wiced_bt_device_address_t remote_addr;   /* remote peer device address */

    uint32_t timer_count;            /* timer count */
    uint32_t fine_timer_count;       /* fine timer count */
    uint16_t conn_id;                /* connection ID referenced by the stack */
    uint16_t peer_mtu;               /* peer MTU */
    uint8_t  num_to_write;           /* num msgs to send, incr on each button intr */
    uint8_t  flag_indication_sent;   /* indicates waiting for ack/cfm */
    uint8_t  flag_stay_connected;    /* stay connected or disconnect after all messages are sent */
    uint8_t  battery_level;          /* dummy battery level */
} hello_sensor_state_t;

/* Host information saved in  NVRAM */
typedef WICED_BT_STRUCT_PACKED
{
    wiced_bt_device_address_t  bdaddr;              /* BD address of the bonded host */
    uint16_t  characteristic_client_configuration;  /* Current value of the client configuration
                                                       descriptor */
    uint8_t   number_of_blinks;                     /* Sensor config, number of times to blink the
                                                       LEd when button is pushed. */
} host_info_t;

typedef struct
{
    uint16_t handle;
    uint16_t attr_len;
    void*    p_attr;
} attribute_t;

typedef void (* pfn_free_buffer_t)(uint8_t*);

extern const  wiced_bt_cfg_settings_t wiced_bt_cfg_settings;
extern wiced_bt_cfg_ble_t             hello_sensor_cfg_ble;
extern wiced_bt_cfg_gatt_t            hello_sensor_cfg_gatt;
extern uint32_t config_VS_Write(uint16_t config_item_id, uint32_t len, uint8_t* buf);
extern uint32_t config_VS_Read(uint16_t config_item_id, uint32_t len, uint8_t* buf);
static uint8_t* hello_sensor_alloc_buffer(uint16_t len);
static void hello_sensor_free_buffer(uint8_t* p_data);



/***************************************************************************************************
 *                                Variables Definitions
 **************************************************************************************************/
/*
 * This is the GATT database for the Hello Sensor application.  It defines
 * services, characteristics and descriptors supported by the sensor.  Each
 * attribute in the database has a handle, (characteristic has two, one for
 * characteristic itself, another for the value).  The handles are used by
 * the peer to access attributes, and can be used locally by application for
 * example to retrieve data written by the peer.  Definition of characteristics
 * and descriptors has GATT Properties (read, write, notify...) but also has
 * permissions which identify if and how peer is allowed to read or write
 * into it.  All handles do not need to be sequential, but need to be in
 * ascending order.
 */
const uint8_t hello_sensor_gatt_database[]=
{
    /* Declare mandatory GATT service */
    PRIMARY_SERVICE_UUID16(HANDLE_HSENS_GATT_SERVICE,
                           UUID_SERVICE_GATT),

    /* Declare mandatory GAP service. Device Name and Appearance are mandatory */
    /* characteristics of GAP service */
    PRIMARY_SERVICE_UUID16(HANDLE_HSENS_GAP_SERVICE,
                           UUID_SERVICE_GAP),

    /* Declare mandatory GAP service characteristic: Dev Name */
    CHARACTERISTIC_UUID16(HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_NAME,
                          HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_NAME_VAL,
                          UUID_CHARACTERISTIC_DEVICE_NAME,
                          GATTDB_CHAR_PROP_READ,
                          GATTDB_PERM_READABLE),

    /* Declare mandatory GAP service characteristic: Appearance */
    CHARACTERISTIC_UUID16(HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_APPEARANCE,
                          HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL,
                          UUID_CHARACTERISTIC_APPEARANCE,
                          GATTDB_CHAR_PROP_READ,
                          GATTDB_PERM_READABLE),

    /* Declare proprietary Hello Service with 128 byte UUID */
    PRIMARY_SERVICE_UUID128(HANDLE_HSENS_SERVICE,
                            UUID_HELLO_SERVICE),

    /* Declare characteristic used to notify/indicate change */
    CHARACTERISTIC_UUID128(HANDLE_HSENS_SERVICE_CHAR_NOTIFY,
                           HANDLE_HSENS_SERVICE_CHAR_NOTIFY_VAL,
                           UUID_HELLO_CHARACTERISTIC_NOTIFY,
                           GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_NOTIFY |
                           GATTDB_CHAR_PROP_INDICATE,
                           GATTDB_PERM_READABLE),

    /* Declare client characteristic configuration descriptor */
    /* Value of the descriptor can be modified by the client */
    /* Value modified shall be retained during connection and across connection */
    /* for bonded devices.  Setting value to 1 tells this application to send notification */
    /* when value of the characteristic changes.  Value 2 is to allow indications. */
    CHAR_DESCRIPTOR_UUID16_WRITABLE(HANDLE_HSENS_SERVICE_CHAR_CFG_DESC,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ),

    /* Declare characteristic Hello Configuration */
    /* The configuration consists of 1 bytes which indicates how many times to */
    /* blink the LED when user pushes the button. */
    CHARACTERISTIC_UUID128_WRITABLE(HANDLE_HSENS_SERVICE_CHAR_BLINK,
                                    HANDLE_HSENS_SERVICE_CHAR_BLINK_VAL,
                                    UUID_HELLO_CHARACTERISTIC_CONFIG,
                                    GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_WRITE,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_CMD |
                                    GATTDB_PERM_WRITE_REQ),

    /* Declare Device info service */
    PRIMARY_SERVICE_UUID16(HANDLE_HSENS_DEV_INFO_SERVICE,
                           UUID_SERVICE_DEVICE_INFORMATION),

    /* Handle 0x4e: characteristic Manufacturer Name, handle 0x4f characteristic value */
    CHARACTERISTIC_UUID16(HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MFR_NAME,
                          HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MFR_NAME_VAL,
                          UUID_CHARACTERISTIC_MANUFACTURER_NAME_STRING,
                          GATTDB_CHAR_PROP_READ,
                          GATTDB_PERM_READABLE),

    /* Handle 0x50: characteristic Model Number, handle 0x51 characteristic value */
    CHARACTERISTIC_UUID16(HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MODEL_NUM,
                          HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MODEL_NUM_VAL,
                          UUID_CHARACTERISTIC_MODEL_NUMBER_STRING,
                          GATTDB_CHAR_PROP_READ,
                          GATTDB_PERM_READABLE),

    /* Handle 0x52: characteristic System ID, handle 0x53 characteristic value */
    CHARACTERISTIC_UUID16(HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_SYSTEM_ID,
                          HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_SYSTEM_ID_VAL,
                          UUID_CHARACTERISTIC_SYSTEM_ID,
                          GATTDB_CHAR_PROP_READ,
                          GATTDB_PERM_READABLE),

    /* Declare Battery service */
    PRIMARY_SERVICE_UUID16(HANDLE_HSENS_BATTERY_SERVICE,
                           0x180F),

    /* Handle 0x62: characteristic Battery Level, handle 0x63 characteristic value */
    CHARACTERISTIC_UUID16(HANDLE_HSENS_BATTERY_SERVICE_CHAR_LEVEL,
                          HANDLE_HSENS_BATTERY_SERVICE_CHAR_LEVEL_VAL,
                          UUID_CHARACTERISTIC_BATTERY_LEVEL,
                          GATTDB_CHAR_PROP_READ,
                          GATTDB_PERM_READABLE),
};

/* GAP Service characteristic Device Name */
uint8_t hello_sensor_device_name[] = "Hello";

uint8_t hello_sensor_appearance_name[2]     = { BIT16_TO_8(APPEARANCE_GENERIC_TAG) };

/* Notification Name */
char    hello_sensor_char_notify_value[]    = { 'H', 'e', 'l', 'l', 'o', ' ', '0' };
char    hello_sensor_char_mfr_name_value[]  = { 'C', 'y', 'p', 'r', 'e', 's', 's', 0, };
char    hello_sensor_char_model_num_value[] = { '1', '2', '3', '4', 0, 0, 0, 0 };
uint8_t hello_sensor_char_system_id_value[] = { 0xbb, 0xb8, 0xa1, 0x80, 0x5f, 0x9f, 0x91, 0x71 };

/* Holds the global state of the hello sensor application */
hello_sensor_state_t hello_sensor_state;

/* Holds the host info saved in the NVRAM */
host_info_t   hello_sensor_hostinfo;
wiced_timer_t hello_sensor_second_timer;
wiced_timer_t hello_sensor_ms_timer;
wiced_timer_t hello_sensor_conn_idle_timer;

/* LED timer and counters */
wiced_timer_t hello_sensor_led_timer;
uint8_t       hello_sensor_led_blink_count = 0;
uint16_t      hello_sensor_led_on_ms       = 0;
uint16_t      hello_sensor_led_off_ms      = 0;


/* Attribute list of the hello sensor */
attribute_t gauAttributes[] =
{
    {
        HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_NAME_VAL,
        sizeof(hello_sensor_device_name),
        hello_sensor_device_name
    },
    {
        HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL,
        sizeof(hello_sensor_appearance_name),
        hello_sensor_appearance_name
    },
    {
        HANDLE_HSENS_SERVICE_CHAR_NOTIFY_VAL,
        sizeof(hello_sensor_char_notify_value),
        hello_sensor_char_notify_value
    },
    {
        HANDLE_HSENS_SERVICE_CHAR_CFG_DESC,
        2,
        (void*)&hello_sensor_hostinfo.characteristic_client_configuration
    },
    {
        HANDLE_HSENS_SERVICE_CHAR_BLINK_VAL,
        1,
        &hello_sensor_hostinfo.number_of_blinks
    },
    {
        HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MFR_NAME_VAL,
        sizeof(hello_sensor_char_mfr_name_value),
        hello_sensor_char_mfr_name_value
    },
    {
        HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MODEL_NUM_VAL,
        sizeof(hello_sensor_char_model_num_value),
        hello_sensor_char_model_num_value
    },
    {
        HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_SYSTEM_ID_VAL,
        sizeof(hello_sensor_char_system_id_value),
        hello_sensor_char_system_id_value
    },
    {
        HANDLE_HSENS_BATTERY_SERVICE_CHAR_LEVEL_VAL,
        1,
        &hello_sensor_state.battery_level
    }
};

static wiced_result_t hello_sensor_management_cback(wiced_bt_management_evt_t event,
                                                    wiced_bt_management_evt_data_t* p_event_data);
static wiced_bt_gatt_status_t hello_sensor_gatts_callback(wiced_bt_gatt_evt_t event,
                                                          wiced_bt_gatt_event_data_t* p_data);
static void hello_sensor_set_advertisement_data(void);
static void hello_sensor_send_message(void);
static void hello_sensor_gatts_increment_notify_value(void);
static void hello_sensor_timeout(WICED_TIMER_PARAM_TYPE count);
static void hello_sensor_fine_timeout(WICED_TIMER_PARAM_TYPE finecount);
static void hello_sensor_smp_bond_result(uint8_t result);
static void hello_sensor_encryption_changed(wiced_result_t result, uint8_t* bd_addr);
static void hello_sensor_application_init(void);
static void hello_sensor_conn_idle_timeout(WICED_TIMER_PARAM_TYPE arg);
static void hello_sensor_load_keys_for_address_resolution(void);

/***************************************************************************************************
 *                          Function Definitions
 **************************************************************************************************/

/***************************************************************************************************
 * Function Name: print_bd_address()
 ***************************************************************************************************
 * Summary:
 *   This is the utility function that prints the address of the Bluetooth device
 *
 * Parameters:
 *   wiced_bt_device_address_t bdadr                : Bluetooth address
 *
 * Return:
 *  void
 *
 **************************************************************************************************/
void print_bd_address(wiced_bt_device_address_t bdadr)
{
    printf("%02X:%02X:%02X:%02X:%02X:%02X",
           bdadr[0], bdadr[1], bdadr[2], bdadr[3], bdadr[4], bdadr[5]);
}


/***************************************************************************************************
 *  Entry point to the application. Set device configuration and start BT
 *  stack initialization.  The actual application initialization will happen
 *  when stack reports that BT device is ready.
 **************************************************************************************************/
void application_start(void)
{
    WICED_BT_TRACE("Hello Sensor Start");

    /* Configure trace */
    cybt_platform_set_trace_level(CYBT_TRACE_ID_MAIN, CYBT_TRACE_LEVEL_MAX);

    /* Register call back and configuration with stack */
    wiced_bt_stack_init(hello_sensor_management_cback, &wiced_bt_cfg_settings);

    /* Create a buffer heap, make it the default heap.  */
    wiced_bt_create_heap("app", NULL, APP_BUFFER_HEAP, NULL, WICED_TRUE);
}


/***************************************************************************************************
 * This function is executed in the BTM_ENABLED_EVT management callback.
 **************************************************************************************************/
void hello_sensor_application_init(void)
{
    wiced_bt_gatt_status_t gatt_status;
    wiced_result_t         result;
    wiced_bt_device_address_t bda = { 0 };

    WICED_BT_TRACE("hello_sensor_application_init\n");

    /* Register with stack to receive GATT callback */
    gatt_status = wiced_bt_gatt_register(hello_sensor_gatts_callback);
    WICED_BT_TRACE("wiced_bt_gatt_register: %d\n", gatt_status);

    /*  Tell stack to use our GATT database */
    gatt_status =
        wiced_bt_gatt_db_init(hello_sensor_gatt_database, sizeof(hello_sensor_gatt_database), NULL);

    WICED_BT_TRACE("wiced_bt_gatt_db_init %d\n", gatt_status);

    /* Starting the app timers , seconds timer and the ms timer  */
    if (wiced_init_timer(&hello_sensor_second_timer, hello_sensor_timeout, 0,
                         WICED_SECONDS_PERIODIC_TIMER) == WICED_SUCCESS)
    {
        wiced_start_timer(&hello_sensor_second_timer, HELLO_SENSOR_APP_TIMEOUT_IN_SECONDS);
    }
    if (wiced_init_timer(&hello_sensor_ms_timer, hello_sensor_fine_timeout, 0,
                         WICED_MILLI_SECONDS_PERIODIC_TIMER) == WICED_SUCCESS)
    {
        //wiced_start_timer( &hello_sensor_ms_timer, HELLO_SENSOR_APP_FINE_TIMEOUT_IN_MS );
    }

    wiced_init_timer(&hello_sensor_conn_idle_timer, hello_sensor_conn_idle_timeout, 0,
                     WICED_SECONDS_TIMER);

    /* Load previous paired keys for address resolution */
    hello_sensor_load_keys_for_address_resolution();

    /* Allow peer to pair */
    wiced_bt_set_pairable_mode(WICED_TRUE, 0);

    /* Set the advertising params and make the device discoverable */
    hello_sensor_set_advertisement_data();

    result =  wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);
    WICED_BT_TRACE("wiced_bt_start_advertisements %d\n", result);

    /*
     * Set flag_stay_connected to remain connected after all messages are sent
     * Reset flag to 0, to disconnect
     */
    hello_sensor_state.flag_stay_connected = 1;

    wiced_bt_dev_read_local_addr(bda);
    printf("Local Bluetooth Address: ");
    print_bd_address(bda);
    printf("\n\r");
}


/***************************************************************************************************
 * Setup advertisement data with 16 byte UUID and device name
 **************************************************************************************************/
void hello_sensor_set_advertisement_data(void)
{
    wiced_bt_ble_advert_elem_t adv_elem[3];

    uint8_t num_elem = 0;
    uint8_t flag     = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;

    uint8_t hello_service_uuid[LEN_UUID_128] = { UUID_HELLO_SERVICE };

    adv_elem[num_elem].advert_type = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].len         = sizeof(uint8_t);
    adv_elem[num_elem].p_data      = &flag;
    num_elem++;

    adv_elem[num_elem].advert_type = BTM_BLE_ADVERT_TYPE_128SRV_COMPLETE;
    adv_elem[num_elem].len         = LEN_UUID_128;
    adv_elem[num_elem].p_data      = hello_service_uuid;
    num_elem++;

    adv_elem[num_elem].advert_type = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len         =
        (uint8_t)strlen((const char*)wiced_bt_cfg_settings.device_name);
    adv_elem[num_elem].p_data      = (uint8_t*)wiced_bt_cfg_settings.device_name;
    num_elem++;

    wiced_bt_ble_set_raw_advertisement_data(num_elem, adv_elem);
}


/***************************************************************************************************
 * This function is invoked when advertisements stop.  If we are configured to stay connected,
 * disconnection was caused by the peer, start low advertisements, so that peer can connect
 * when it wakes up
 **************************************************************************************************/
void hello_sensor_advertisement_stopped(void)
{
    wiced_result_t result;

    if (hello_sensor_state.flag_stay_connected && !hello_sensor_state.conn_id)
    {
        result =  wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL);
        WICED_BT_TRACE("wiced_bt_start_advertisements: %d\n", result);
    }
    else
    {
        WICED_BT_TRACE("ADV stop\n");
    }

    UNUSED_VARIABLE(result);
}


/***************************************************************************************************
 * The function invoked on timeout of app seconds timer.
 **************************************************************************************************/
void hello_sensor_timeout(WICED_TIMER_PARAM_TYPE count)
{
    hello_sensor_state.timer_count++;
    /* print for first 10 seconds, then once every 10 seconds thereafter  */
    if ((hello_sensor_state.timer_count <= 10) || (hello_sensor_state.timer_count % 10 == 0))
    {
        WICED_BT_TRACE("hello_sensor_timeout: %d, ft:%d \n",
                       hello_sensor_state.timer_count, hello_sensor_state.fine_timer_count);

        /* send notification */
        if (hello_sensor_state.conn_id != 0)
        {
            hello_sensor_send_message();
        }
    }
}


/***************************************************************************************************
 * The function invoked on timeout of app milliseconds fine timer
 **************************************************************************************************/
void hello_sensor_fine_timeout(WICED_TIMER_PARAM_TYPE finecount)
{
    hello_sensor_state.fine_timer_count++;
}


/***************************************************************************************************
 * Process SMP bonding result. If we successfully paired with the
 * central device, save its BDADDR in the NVRAM and initialize
 * associated data
 **************************************************************************************************/
void hello_sensor_smp_bond_result(uint8_t result)
{
    WICED_BT_TRACE("hello_sensor, bond result: %d\n", result);

    /* Bonding success */
    if (result == WICED_BT_SUCCESS)
    {
        /* Pack the data to be stored into the hostinfo structure */
        memcpy(hello_sensor_hostinfo.bdaddr, hello_sensor_state.remote_addr, BD_ADDR_LEN);

        #ifdef NVRAM_SUPPORT
        wiced_result_t status;
        status = config_VS_Write(HELLO_SENSOR_VS_ID, sizeof(hello_sensor_hostinfo),
                                 (uint8_t*)&hello_sensor_hostinfo);
        WICED_BT_TRACE("NVRAM write status: %d\n", status);
        #endif
    }
}


/***************************************************************************************************
 * Process notification from stack that encryption has been set. If connected
 * client is registered for notification or indication, it is a good time to
 * send it out
 **************************************************************************************************/
void hello_sensor_encryption_changed(wiced_result_t result, uint8_t* bd_addr)
{
    WICED_BT_TRACE("encryp change res: %x ", result);

    /* Connection has been encrypted meaning that we have correct/paired device
     * restore values in the database
     */
    #ifdef NVRAM_SUPPORT
    result = config_VS_Read(HELLO_SENSOR_VS_ID, sizeof(hello_sensor_hostinfo),
                            (uint8_t*)&hello_sensor_hostinfo);
    #endif

    /* If there are outstanding messages that we could not send out because
     * connection was not up and/or encrypted, send them now.  If we are sending
     * indications, we can send only one and need to wait for ack.
     * */
    while ((hello_sensor_state.num_to_write != 0) && !hello_sensor_state.flag_indication_sent)
    {
        hello_sensor_state.num_to_write--;
        hello_sensor_send_message();
    }

    /* If configured to disconnect after delivering data, start idle timeout
     * to do disconnection
     * */
    if ((!hello_sensor_state.flag_stay_connected) && !hello_sensor_state.flag_indication_sent)
    {
        if (wiced_is_timer_in_use(&hello_sensor_conn_idle_timer))
        {
            wiced_stop_timer(&hello_sensor_conn_idle_timer);
            wiced_start_timer(&hello_sensor_conn_idle_timer,
                              HELLO_SENSOR_CONN_IDLE_TIMEOUT_IN_SECONDS);
        }
    }
}


/**************************************************************************************************
* Function on connection idle timeout initiates the gatt disconnect
**************************************************************************************************/
void hello_sensor_conn_idle_timeout(WICED_TIMER_PARAM_TYPE arg)
{
    WICED_BT_TRACE("hello_sensor_conn_idle_timeout\n");

    /* Initiating the gatt disconnect */
    wiced_bt_gatt_disconnect(hello_sensor_state.conn_id);
}


/***************************************************************************************************
 * hello_sensor bt/ble link management callback
 **************************************************************************************************/
wiced_result_t hello_sensor_management_cback(wiced_bt_management_evt_t event,
                                             wiced_bt_management_evt_data_t* p_event_data)
{
    wiced_bt_dev_encryption_status_t* p_status;
    wiced_bt_dev_ble_pairing_info_t*  p_info;
    wiced_bt_ble_advert_mode_t*       p_mode;
    wiced_result_t                    result = WICED_BT_SUCCESS;

    WICED_BT_TRACE("hello_sensor_management_cback: %x\n", event);

    switch (event)
    {
        /* Bluetooth  stack enabled */
        case BTM_ENABLED_EVT:
            hello_sensor_application_init();
            break;

        case BTM_DISABLED_EVT:
            break;

        case BTM_USER_CONFIRMATION_REQUEST_EVT:
            WICED_BT_TRACE("Numeric_value: %d \n",
                           p_event_data->user_confirmation_request.numeric_value);
            wiced_bt_dev_confirm_req_reply(WICED_BT_SUCCESS,
                                           p_event_data->user_confirmation_request.bd_addr);
            break;

        case BTM_PASSKEY_NOTIFICATION_EVT:
            WICED_BT_TRACE("PassKey Notification. Key %d \n",
                           p_event_data->user_passkey_notification.passkey);
            wiced_bt_dev_confirm_req_reply(WICED_BT_SUCCESS,
                                           p_event_data->user_passkey_notification.bd_addr);
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
            p_event_data->pairing_io_capabilities_ble_request.local_io_cap =
                BTM_IO_CAPABILITIES_NONE;
            p_event_data->pairing_io_capabilities_ble_request.oob_data = BTM_OOB_NONE;
            p_event_data->pairing_io_capabilities_ble_request.auth_req =
                BTM_LE_AUTH_REQ_SC_BOND;
            p_event_data->pairing_io_capabilities_ble_request.max_key_size = 0x10;
            p_event_data->pairing_io_capabilities_ble_request.init_keys =
                BTM_LE_KEY_PENC| BTM_LE_KEY_PID| BTM_LE_KEY_PCSRK| BTM_LE_KEY_LENC;
            p_event_data->pairing_io_capabilities_ble_request.resp_keys =
                BTM_LE_KEY_PENC| BTM_LE_KEY_PID| BTM_LE_KEY_PCSRK| BTM_LE_KEY_LENC;
            break;

        case BTM_PAIRING_COMPLETE_EVT:
            p_info =  &p_event_data->pairing_complete.pairing_complete_info.ble;
            WICED_BT_TRACE("Pairing Complete: %x ", p_info->reason);
            hello_sensor_smp_bond_result(p_info->reason);
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
            /* save keys to NVRAM */
            #ifdef NVRAM_SUPPORT
        {
            uint8_t* p_keys;
            p_keys = (uint8_t*)&p_event_data->paired_device_link_keys_update;
            result =
                config_VS_Write(HELLO_SENSOR_PAIRED_KEYS_VS_ID,
                                sizeof(wiced_bt_device_link_keys_t), p_keys);
            WICED_BT_TRACE("keys save to NVRAM %B result: %d \n", p_keys, result);
        }
            #endif
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
            /* read keys from NVRAM */
            #ifdef NVRAM_SUPPORT
            p_keys = (uint8_t*)&p_event_data->paired_device_link_keys_request;
            result =
                config_VS_Read(HELLO_SENSOR_PAIRED_KEYS_VS_ID, sizeof(wiced_bt_device_link_keys_t),
                               p_keys);
            WICED_BT_TRACE("keys read from NVRAM %B result: %d \n", p_keys, result);
            #endif
            break;

        case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
            #ifdef NVRAM_SUPPORT
            /* save keys to NVRAM */
            p_keys = (uint8_t*)&p_event_data->local_identity_keys_update;
            result =
                config_VS_Write(HELLO_SENSOR_LOCAL_KEYS_VS_ID,
                                sizeof(wiced_bt_local_identity_keys_t),
                                p_keys);
            WICED_BT_TRACE("local keys save to NVRAM result: %d \n", result);
            #endif
            break;

        case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
            #ifdef NVRAM_SUPPORT
            /* read keys from NVRAM */
            p_keys = (uint8_t*)&p_event_data->local_identity_keys_request;
            result =
                config_VS_Read(HELLO_SENSOR_LOCAL_KEYS_VS_ID,
                               sizeof(wiced_bt_local_identity_keys_t),
                               p_keys);
            WICED_BT_TRACE("local keys read from NVRAM result: %d \n", result);
            #endif
            break;

        case BTM_ENCRYPTION_STATUS_EVT:
            p_status = &p_event_data->encryption_status;
            WICED_BT_TRACE("Encryption Status Event: res %x", p_status->result);
            hello_sensor_encryption_changed(p_status->result, p_status->bd_addr);
            break;

        case BTM_SECURITY_REQUEST_EVT:
            wiced_bt_ble_security_grant(p_event_data->security_request.bd_addr, WICED_BT_SUCCESS);
            break;

        case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
            p_mode = &p_event_data->ble_advert_state_changed;
            WICED_BT_TRACE("Advertisement State Change: %x\n", *p_mode);
            if (*p_mode == BTM_BLE_ADVERT_OFF)
            {
                hello_sensor_advertisement_stopped();
            }
            break;

        default:
            break;
    }

    return result;
}


/***************************************************************************************************
 * Check if client has registered for notification/indication
 * and send message if appropriate
 **************************************************************************************************/
void hello_sensor_send_message(void)
{
    WICED_BT_TRACE("[%s] CCC:%d\n", __FUNCTION__,
                   hello_sensor_hostinfo.characteristic_client_configuration);

    /* If client has not registered for indication or notification, no action */
    if (hello_sensor_hostinfo.characteristic_client_configuration == 0)
    {
        return;
    }

    /* Increment the last byte of the hello sensor notify value */
    hello_sensor_gatts_increment_notify_value();

    if (hello_sensor_hostinfo.characteristic_client_configuration &
        GATT_CLIENT_CONFIG_NOTIFICATION)
    {
        uint8_t* p_attr = (uint8_t*)hello_sensor_char_notify_value;

        wiced_bt_gatt_server_send_notification(hello_sensor_state.conn_id,
                                               HANDLE_HSENS_SERVICE_CHAR_NOTIFY_VAL,
                                               sizeof(hello_sensor_char_notify_value), p_attr,
                                               NULL);
    }
    else
    {
        if (!hello_sensor_state.flag_indication_sent)
        {
            uint8_t* p_attr = (uint8_t*)hello_sensor_char_notify_value;

            hello_sensor_state.flag_indication_sent = TRUE;

            wiced_bt_gatt_server_send_indication(hello_sensor_state.conn_id,
                                                 HANDLE_HSENS_SERVICE_CHAR_NOTIFY_VAL,
                                                 sizeof(hello_sensor_char_notify_value), p_attr,
                                                 NULL);
        }
    }
}


/***************************************************************************************************
 * Find attribute description by handle
 **************************************************************************************************/
attribute_t* hello_sensor_find_by_handle(uint16_t handle)
{
    int i;
    for (i = 0; i < sizeof(gauAttributes) / sizeof(gauAttributes[0]); i++)
    {
        if (gauAttributes[i].handle == handle)
        {
            return (&gauAttributes[i]);
        }
    }
    return NULL;
}


/***************************************************************************************************
 * Process Read request or read blob from peer device
 **************************************************************************************************/
wiced_bt_gatt_status_t hello_sensor_gatts_req_read_handler(uint16_t conn_id,
                                                           wiced_bt_gatt_opcode_t opcode,
                                                           wiced_bt_gatt_read_t* p_read_req,
                                                           uint16_t len_requested)
{
    attribute_t* puAttribute;
    uint16_t     attr_len_to_copy;
    uint8_t*     from;

    if ((puAttribute = hello_sensor_find_by_handle(p_read_req->handle)) == NULL)
    {
        WICED_BT_TRACE("[%s]  attr not found handle: 0x%04x\n", __FUNCTION__, p_read_req->handle);
        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->handle,
                                            WICED_BT_GATT_INVALID_HANDLE);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Dummy battery value read increment */
    if (p_read_req->handle == HANDLE_HSENS_BATTERY_SERVICE_CHAR_LEVEL_VAL)
    {
        if (hello_sensor_state.battery_level++ > 100)
        {
            hello_sensor_state.battery_level = 0;
        }
    }

    attr_len_to_copy = puAttribute->attr_len;

    WICED_BT_TRACE("[%s] conn_id: %d handle:0x%04x offset:%d len:%d\n", __FUNCTION__, conn_id,
                   p_read_req->handle, p_read_req->offset, attr_len_to_copy);

    if (p_read_req->offset >= puAttribute->attr_len)
    {
        WICED_BT_TRACE("[%s] offset:%d larger than attribute length:%d\n", __FUNCTION__,
                       p_read_req->offset, puAttribute->attr_len);

        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->handle,
                                            WICED_BT_GATT_INVALID_OFFSET);
        return (WICED_BT_GATT_INVALID_HANDLE);
    }

    int to_send = MIN(len_requested, attr_len_to_copy - p_read_req->offset);

    from = ((uint8_t*)puAttribute->p_attr) + p_read_req->offset;

    wiced_bt_gatt_server_send_read_handle_rsp(conn_id, opcode, to_send, from, NULL);

    return WICED_BT_GATT_SUCCESS;
}


/***************************************************************************************************
 * Process write read-by-type request from peer device
 **************************************************************************************************/
wiced_bt_gatt_status_t hello_sensor_gatts_req_read_by_type_handler(
    uint16_t conn_id, wiced_bt_gatt_opcode_t opcode, wiced_bt_gatt_read_by_type_t* p_read_req,
    uint16_t len_requested)
{
    attribute_t* puAttribute;
    uint16_t     attr_handle = p_read_req->s_handle;
    uint8_t*     p_rsp       = hello_sensor_alloc_buffer(len_requested);
    uint8_t      pair_len    = 0;
    int          used        = 0;

    if (p_rsp == NULL)
    {
        WICED_BT_TRACE("[%s]  no memory len_requested: %d!!\n", __FUNCTION__, len_requested);

        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, attr_handle,
                                            WICED_BT_GATT_INSUF_RESOURCE);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Read by type returns all attributes of the specified type, between the start and end handles
     */
    while (WICED_TRUE)
    {
        /* Add your code here */
        attr_handle = wiced_bt_gatt_find_handle_by_type(attr_handle, p_read_req->e_handle,
                                                        &p_read_req->uuid);

        if (attr_handle == 0)
        {
            break;
        }

        if ((puAttribute = hello_sensor_find_by_handle(attr_handle)) == NULL)
        {
            WICED_BT_TRACE("[%s]  found type but no attribute ??\n", __FUNCTION__);
            wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->s_handle,
                                                WICED_BT_GATT_ERR_UNLIKELY);
            hello_sensor_free_buffer(p_rsp);
            return WICED_BT_GATT_INVALID_HANDLE;
        }

        {
            int filled = wiced_bt_gatt_put_read_by_type_rsp_in_stream(p_rsp + used,
                                                                      len_requested - used,
                                                                      &pair_len,
                                                                      attr_handle,
                                                                      puAttribute->attr_len,
                                                                      puAttribute->p_attr);
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
        WICED_BT_TRACE(
            "[%s]  attr not found  start_handle: 0x%04x  end_handle: 0x%04x  Type: 0x%04x\n",
            __FUNCTION__, p_read_req->s_handle, p_read_req->e_handle,
            p_read_req->uuid.uu.uuid16);

        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, p_read_req->s_handle,
                                            WICED_BT_GATT_INVALID_HANDLE);
        hello_sensor_free_buffer(p_rsp);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Send the response */
    wiced_bt_gatt_server_send_read_by_type_rsp(conn_id, opcode, pair_len, used, p_rsp,
                                               (wiced_bt_gatt_app_context_t)
                                               hello_sensor_free_buffer);

    return WICED_BT_GATT_SUCCESS;
}


/***************************************************************************************************
 * Process write read multi request from peer device
 **************************************************************************************************/
wiced_bt_gatt_status_t hello_sensor_gatts_req_read_multi_handler(
    uint16_t conn_id, wiced_bt_gatt_opcode_t opcode, wiced_bt_gatt_read_multiple_req_t* p_read_req,
    uint16_t len_requested)
{
    attribute_t* puAttribute;
    uint8_t*     p_rsp = hello_sensor_alloc_buffer(len_requested);
    int          used  = 0;
    int          xx;
    uint16_t     handle = wiced_bt_gatt_get_handle_from_stream(p_read_req->p_handle_stream, 0);

    if (p_rsp == NULL)
    {
        WICED_BT_TRACE("[%s]  no memory len_requested: %d!!\n", __FUNCTION__, len_requested);

        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, handle, WICED_BT_GATT_INSUF_RESOURCE);

        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Read by type returns all attributes of the specified type, between the start and end handles
     */
    for (xx = 0; xx < p_read_req->num_handles; xx++)
    {
        handle = wiced_bt_gatt_get_handle_from_stream(p_read_req->p_handle_stream, xx);
        if ((puAttribute = hello_sensor_find_by_handle(handle)) == NULL)
        {
            WICED_BT_TRACE("[%s]  no handle 0x%04xn", __FUNCTION__, handle);
            wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, *p_read_req->p_handle_stream,
                                                WICED_BT_GATT_ERR_UNLIKELY);
            hello_sensor_free_buffer(p_rsp);
            return WICED_BT_GATT_INVALID_HANDLE;
        }

        {
            int filled = wiced_bt_gatt_put_read_multi_rsp_in_stream(opcode, p_rsp + used,
                                                                    len_requested - used,
                                                                    puAttribute->handle,
                                                                    puAttribute->attr_len,
                                                                    puAttribute->p_attr);
            if (!filled)
            {
                break;
            }
            used += filled;
        }
    }

    if (used == 0)
    {
        WICED_BT_TRACE("[%s] no attr found\n", __FUNCTION__);

        wiced_bt_gatt_server_send_error_rsp(conn_id, opcode, *p_read_req->p_handle_stream,
                                            WICED_BT_GATT_INVALID_HANDLE);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Send the response */
    wiced_bt_gatt_server_send_read_multiple_rsp(conn_id, opcode, used, p_rsp,
                                                (wiced_bt_gatt_app_context_t)
                                                hello_sensor_free_buffer);

    return WICED_BT_GATT_SUCCESS;
}


/***************************************************************************************************
 * Process write request or write command from peer device
 **************************************************************************************************/
wiced_bt_gatt_status_t hello_sensor_gatts_req_write_handler(uint16_t conn_id,
                                                            wiced_bt_gatt_opcode_t opcode,
                                                            wiced_bt_gatt_write_req_t* p_data)
{
    wiced_bt_gatt_status_t result    = WICED_BT_GATT_SUCCESS;
    uint8_t*               p_attr    = p_data->p_val;
    uint8_t                nv_update = WICED_FALSE;

    WICED_BT_TRACE("[%s] conn_id:%d hdl:0x%x opcode:%d offset:%d len:%d\n ", __FUNCTION__, conn_id,
                   p_data->handle, opcode, p_data->offset, p_data->val_len);

    switch (p_data->handle)
    {
        /* By writing into Characteristic Client Configuration descriptor
         * peer can enable or disable notification or indication */
        case HANDLE_HSENS_SERVICE_CHAR_CFG_DESC:
            if (p_data->val_len != 2)
            {
                return WICED_BT_GATT_INVALID_ATTR_LEN;
            }
            hello_sensor_hostinfo.characteristic_client_configuration =
                p_attr[0] | (p_attr[1] << 8);

            WICED_BT_TRACE("[%s] CCCD changed to: 0x%04x\n ", __FUNCTION__,
                           hello_sensor_hostinfo.characteristic_client_configuration);

            nv_update = WICED_TRUE;
            break;

        case HANDLE_HSENS_SERVICE_CHAR_BLINK_VAL:
            if (p_data->val_len != 1)
            {
                return WICED_BT_GATT_INVALID_ATTR_LEN;
            }
            hello_sensor_hostinfo.number_of_blinks = p_attr[0];
            if (hello_sensor_hostinfo.number_of_blinks != 0)
            {
                WICED_BT_TRACE("[%s] num blinks: %d\n", __FUNCTION__,
                               hello_sensor_hostinfo.number_of_blinks);
                nv_update = WICED_TRUE;
            }
            break;

        default:
            result = WICED_BT_GATT_INVALID_HANDLE;
            break;
    }

    if (nv_update)
    {
        #ifdef NVRAM_SUPPORT
        int bytes_written =
            config_VS_Write(HELLO_SENSOR_VS_ID, sizeof(hello_sensor_hostinfo),
                            (uint8_t*)&hello_sensor_hostinfo);
        WICED_BT_TRACE("[%s]  NVRAM write: %d", __FUNCTION__, bytes_written);
        #endif
    }

    return result;
}


/***************************************************************************************************
 * Write Execute Procedure
 **************************************************************************************************/
wiced_bt_gatt_status_t hello_sensor_gatts_req_write_exec_handler(
    uint16_t conn_id, wiced_bt_gatt_exec_flag_t exec_falg)
{
    WICED_BT_TRACE("write exec: flag:%d\n", exec_falg);
    return WICED_BT_GATT_SUCCESS;
}


/***************************************************************************************************
 * Process MTU request from the peer
 **************************************************************************************************/
wiced_bt_gatt_status_t hello_sensor_gatts_req_mtu_handler(uint16_t conn_id, uint16_t mtu)
{
    WICED_BT_TRACE("req_mtu: %d\n", mtu);
    wiced_bt_gatt_server_send_mtu_rsp(conn_id, mtu, hello_sensor_cfg_ble.ble_max_rx_pdu_size);
    return WICED_BT_GATT_SUCCESS;
}


/***************************************************************************************************
 * Process indication confirm. If client wanted us to use indication instead of
 * notifications we have to wait for confirmation after every message sent.
 * For example if user pushed button twice very fast
 * we will send first message, then
 * wait for confirmation, then
 * send second message, then
 * wait for confirmation and
 * if configured start idle timer only after that.
 **************************************************************************************************/
wiced_bt_gatt_status_t hello_sensor_gatts_req_conf_handler(uint16_t conn_id, uint16_t handle)
{
    WICED_BT_TRACE("hello_sensor_indication_cfm, conn %d hdl %d\n", conn_id, handle);

    if (!hello_sensor_state.flag_indication_sent)
    {
        WICED_BT_TRACE("Hello: Wrong Confirmation!");
        return WICED_BT_GATT_SUCCESS;
    }

    hello_sensor_state.flag_indication_sent = 0;

    /* We might need to send more indications */
    if (hello_sensor_state.num_to_write)
    {
        hello_sensor_state.num_to_write--;
        hello_sensor_send_message();
    }
    /* if we sent all messages, start connection idle timer to disconnect */
    if (!hello_sensor_state.flag_stay_connected && !hello_sensor_state.flag_indication_sent)
    {
        if (wiced_is_timer_in_use(&hello_sensor_conn_idle_timer))
        {
            wiced_stop_timer(&hello_sensor_conn_idle_timer);
            wiced_start_timer(&hello_sensor_conn_idle_timer,
                              HELLO_SENSOR_CONN_IDLE_TIMEOUT_IN_SECONDS);
        }
    }

    return WICED_BT_GATT_SUCCESS;
}


/***************************************************************************************************
 * This function is invoked when connection is established
 **************************************************************************************************/
wiced_bt_gatt_status_t hello_sensor_gatts_connection_up(wiced_bt_gatt_connection_status_t* p_status)
{
    wiced_result_t result;

    WICED_BT_TRACE("hello_sensor_conn_up id:%x\n:", p_status->conn_id);

    /* Update the connection handler.  Save address of the connected device. */
    hello_sensor_state.conn_id = p_status->conn_id;
    memcpy(hello_sensor_state.remote_addr, p_status->bd_addr, BD_ADDR_LEN);

    /* Stop advertising */
    result =  wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF, 0, NULL);

    WICED_BT_TRACE("Stopping Advertisements %x\n", result);

    /* Stop idle timer */
    wiced_stop_timer(&hello_sensor_conn_idle_timer);

    /* Saving host info in NVRAM  */
    memcpy(hello_sensor_hostinfo.bdaddr, p_status->bd_addr, BD_ADDR_LEN);
    hello_sensor_hostinfo.characteristic_client_configuration = 0;
    hello_sensor_hostinfo.number_of_blinks                    = 0;

    #ifdef NVRAM_SUPPORT
    {
        uint32_t bytes_written = 0;
        bytes_written = config_VS_Write(HELLO_SENSOR_VS_ID, sizeof(hello_sensor_hostinfo),
                                        (uint8_t*)&hello_sensor_hostinfo);

        WICED_BT_TRACE("NVRAM write %d\n", bytes_written);
        UNUSED_VARIABLE(bytes_written);
    }
    #endif

    return WICED_BT_GATT_SUCCESS;
}


/***************************************************************************************************
 * This function is invoked when connection is lost
 **************************************************************************************************/
wiced_bt_gatt_status_t hello_sensor_gatts_connection_down(
    wiced_bt_gatt_connection_status_t* p_status)
{
    wiced_result_t result;

    WICED_BT_TRACE("connection_down conn_id:%x reason:%x\n", p_status->conn_id, p_status->reason);

    /* Resetting the device info */
    memset(hello_sensor_state.remote_addr, 0, 6);
    hello_sensor_state.conn_id = 0;

    /*
     * If we are configured to stay connected, disconnection was
     * caused by the peer, start low advertisements, so that peer
     * can connect when it wakes up
     */
    if (hello_sensor_state.flag_stay_connected)
    {
        result =  wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL);
        WICED_BT_TRACE("wiced_bt_start_advertisements %x\n", result);
    }

    UNUSED_VARIABLE(result);

    return WICED_BT_SUCCESS;
}


/***************************************************************************************************
 * Connection up/down event
 **************************************************************************************************/
wiced_bt_gatt_status_t hello_sensor_gatts_conn_status_cb(
    wiced_bt_gatt_connection_status_t* p_status)
{
    if (p_status->connected)
    {
        return hello_sensor_gatts_connection_up(p_status);
    }

    return hello_sensor_gatts_connection_down(p_status);
}


/***************************************************************************************************
 * Process GATT request from the peer
 **************************************************************************************************/
wiced_bt_gatt_status_t hello_sensor_gatts_req_cb(wiced_bt_gatt_attribute_request_t* p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;

    WICED_BT_TRACE("hello_sensor_gatts_req_cb. conn %d, opcode %d\n", p_data->conn_id,
                   p_data->opcode);

    switch (p_data->opcode)
    {
        case GATT_REQ_READ:
        case GATT_REQ_READ_BLOB:
            result = hello_sensor_gatts_req_read_handler(p_data->conn_id, p_data->opcode,
                                                         &p_data->data.read_req,
                                                         p_data->len_requested);
            break;

        case GATT_REQ_READ_BY_TYPE:
            result = hello_sensor_gatts_req_read_by_type_handler(p_data->conn_id, p_data->opcode,
                                                                 &p_data->data.read_by_type,
                                                                 p_data->len_requested);
            break;

        case GATT_REQ_READ_MULTI:
        case GATT_REQ_READ_MULTI_VAR_LENGTH:
            result = hello_sensor_gatts_req_read_multi_handler(p_data->conn_id, p_data->opcode,
                                                               &p_data->data.read_multiple_req,
                                                               p_data->len_requested);
            break;

        case GATT_REQ_WRITE:
        case GATT_CMD_WRITE:
        case GATT_CMD_SIGNED_WRITE:
            result =
                hello_sensor_gatts_req_write_handler(p_data->conn_id, p_data->opcode,
                                                     &(p_data->data.write_req));
            if ((p_data->opcode == GATT_REQ_WRITE) && (result == WICED_BT_GATT_SUCCESS))
            {
                wiced_bt_gatt_write_req_t* p_write_request = &p_data->data.write_req;
                wiced_bt_gatt_server_send_write_rsp(p_data->conn_id, p_data->opcode,
                                                    p_write_request->handle);
            }
            break;

        case GATT_REQ_EXECUTE_WRITE:
            hello_sensor_gatts_req_write_exec_handler(p_data->conn_id,
                                                      p_data->data.exec_write_req.exec_write);
            wiced_bt_gatt_server_send_execute_write_rsp(p_data->conn_id, p_data->opcode);
            break;

        case GATT_REQ_MTU:
            result = hello_sensor_gatts_req_mtu_handler(p_data->conn_id, p_data->data.remote_mtu);
            break;

        case GATT_HANDLE_VALUE_CONF:
            result = hello_sensor_gatts_req_conf_handler(p_data->conn_id,
                                                         p_data->data.confirm.handle);
            break;

        default:
            break;
    }

    return result;
}


/***************************************************************************************************
 * Callback for various GATT events.  As this application performs only as a GATT server, some of
 * the events are ommitted.
 **************************************************************************************************/
wiced_bt_gatt_status_t hello_sensor_gatts_callback(wiced_bt_gatt_evt_t event,
                                                   wiced_bt_gatt_event_data_t* p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;

    switch (event)
    {
        case GATT_CONNECTION_STATUS_EVT:
            result = hello_sensor_gatts_conn_status_cb(&p_data->connection_status);
            break;

        case GATT_ATTRIBUTE_REQUEST_EVT:
            result = hello_sensor_gatts_req_cb(&p_data->attribute_request);
            break;

        case GATT_GET_RESPONSE_BUFFER_EVT:
            p_data->buffer_request.buffer.p_app_rsp_buffer = hello_sensor_alloc_buffer(
                p_data->buffer_request.len_requested);
            p_data->buffer_request.buffer.p_app_ctxt =
                (wiced_bt_gatt_app_context_t)hello_sensor_free_buffer;
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
 * Keep number of the button pushes in the last byte of the Hello message.
 * That will guarantee that if client reads it, it will have correct data.
 **************************************************************************************************/
void hello_sensor_gatts_increment_notify_value(void)
{
    /* Getting the last byte */
    int  last_byte = sizeof(hello_sensor_char_notify_value) - 1;
    char c         = hello_sensor_char_notify_value[last_byte];

    c++;
    if ((c < '0') || (c > '9'))
    {
        c = '0';
    }
    hello_sensor_char_notify_value[last_byte] = c;
}


/***************************************************************************************************
 * hello_sensor_load_keys_for_address_resolution
 **************************************************************************************************/
static void hello_sensor_load_keys_for_address_resolution(void)
{
    #ifdef NVRAM_SUPPORT

    wiced_bt_device_link_keys_t link_keys;
    wiced_result_t              result;
    uint8_t*                    p;

    memset(&link_keys, 0, sizeof(wiced_bt_device_link_keys_t));
    p      = (uint8_t*)&link_keys;
    result = config_VS_Read(HELLO_SENSOR_PAIRED_KEYS_VS_ID, sizeof(wiced_bt_device_link_keys_t), p);

    if (result == 0)
    {
        result = wiced_bt_dev_add_device_to_address_resolution_db(&link_keys);
    }
    WICED_BT_TRACE("hello_sensor_load_keys_for_address_resolution %B result:%d \n", p, result);
    #endif // ifdef NVRAM_SUPPORT
}


/***************************************************************************************************
 * hci_control_proc_rx_cmd
 **************************************************************************************************/
uint32_t hci_control_proc_rx_cmd(uint8_t* p_buffer, uint32_t length)
{
    return 0;
}


/***************************************************************************************************
 * hello_sensor_alloc_buffer
 **************************************************************************************************/
static uint8_t* hello_sensor_alloc_buffer(uint16_t len)
{
    uint8_t* p = (uint8_t*)wiced_bt_get_buffer(len);
    WICED_BT_TRACE("[%s] len %d alloc 0x%x", __FUNCTION__, len, p);

    return p;
}


/***************************************************************************************************
 * hello_sensor_free_buffer
 **************************************************************************************************/
static void hello_sensor_free_buffer(uint8_t* p_data)
{
    wiced_bt_free_buffer(p_data);

    WICED_BT_TRACE("[%s] 0x%x", __FUNCTION__, p_data);
}
