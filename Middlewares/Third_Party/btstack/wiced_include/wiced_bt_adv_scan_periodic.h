/*
 * Copyright 2024-2025, Cypress Semiconductor Corporation or
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
 */

/** @file
 *
 * AIROC Bluetooth Low Energy (LE) Functions for extended adv and scan
 *
 */
#ifndef __WICED_BT_ADV_SCAN_PERIODIC_H__
#define __WICED_BT_ADV_SCAN_PERIODIC_H__

#ifndef WICED_BLE_ENABLE_EXTENDED_ADV_API
#define WICED_BLE_ENABLE_EXTENDED_ADV_API 1
#endif

#if (WICED_BLE_ENABLE_EXTENDED_ADV_API == 1)

/**
 * This section contains LE Periodic advertisement and scanning defines, structures and functions.
 * @note Applications using extended and periodic mode advertising and scanning shall use only extended mode APIs
 * cannot invoke any of the Legacy mode APIs for advertisement and scanning
 *
 * @addtogroup  wicedbt_Periodic   LE Periodic Mode Advertising and Scanning
 *
 * @ingroup     wicedbt
 * @{
 */


/** Periodic adv property */
enum wiced_ble_padv_prop_e
{
    /**< Speicify Tx power in periodic adv events */
    WICED_BLE_PADV_PROPERTY_INCLUDE_TX_POWER = (1 << 6)
};
/** Periodic adv property (see #wiced_ble_padv_prop_e) */
typedef uint16_t wiced_ble_padv_prop_t;


/** Options used in create sync to periodic adv command
 * The Options parameter is used to determine whether the Periodic Advertiser List is
 * used, whether HCI_LE_Periodic_Advertising_Report events for this periodic advertising
 * train are initially enabled or disabled, and whether duplicate reports are filtered or not.
 * If the Periodic Advertiser List is not used, the Advertising_SID, Advertiser Address_Type,
 * and Advertiser Address parameters specify the periodic advertising device to listen to;
 * otherwise they shall be ignored.
*/
enum wiced_ble_padv_sync_options_e
{
    /** Use the Advertising_SID, Advertising_Address_Type, and Advertising Address parameters specified
     * in create sync command to determine which advertiser to listen to */
    WICED_BLE_PADV_CREATE_SYNC_OPTION_IGNORE_PA_LIST = 0,

    /** Use the Periodic Advertiser List to determine which advertiser to listen to.*/
    WICED_BLE_PADV_CREATE_SYNC_OPTION_USE_PA_LIST = 1,

    /**
    * Disable receiving periodic advertising reports
    */
    WICED_BLE_PADV_CREATE_SYNC_OPTION_DISABLE_REPORTING = 2,

    /**
    * Enable duplicate filtering for the periodic advertising reports
    */
    WICED_BLE_PADV_CREATE_SYNC_OPTION_ENABLE_DUPLICATE_FILTERING = 4
};

/** Options used in create periodic sync to periodic adv command (see #wiced_ble_padv_sync_options_e)*/
typedef uint8_t wiced_ble_padv_sync_options_t;

/* @cond PAWR_API
   APIs for Periodic Advertising with Response*/

/** Maximum PAWR Subevent data len */
#define WICED_BT_MAX_PAWR_SUBEVENT_DATA_LEN 251

/** Periodic Advertising with Response (PAWR) Subevent Data Request Event Data */
typedef struct
{
    /** advertisement set handle */
    wiced_ble_ext_adv_handle_t adv_handle;
    /** first subevent number*/
    uint8_t subevent_start;
    /** number of subevents that data is requested for */
    uint8_t subevent_start_data_count;
} wiced_ble_padv_subevent_data_req_event_data_t;

/** Periodic Advertising with Response (PAWR) Response Report Event Data */
typedef struct
{
    uint8_t adv_handle;    /**< advertisement set handle */
    uint8_t subevent;      /**< subevent number */
    uint8_t tx_status;     /**< if 1, AUX_SYNC_SUBEVENT_IND pkt was transmitted, else no */
    uint8_t tx_power;      /**< tx power */
    uint8_t rssi;          /**< rssi */
    uint8_t cte_type;      /**< constant tone extension */
    uint8_t response_slot; /**< response slot */
    uint8_t data_status;   /**< data status */
    uint16_t data_length;  /**< data length */
    uint8_t *p_data;       /**< data in the event */
} wiced_ble_padv_rsp_report_event_data_t;

/* @endcond */


/* @cond PAWR_API
   APIs for Periodic Advertising with Response*/
/** Configuration for Periodic Advertising with Response (PAWR) subevent indication data
**  which is sent by the central device at the start of each subevent
*/
typedef struct
{
    uint8_t subevent_num;     /**< The subevent number */
    uint8_t rsp_slot_start;   /**< Response slot start */
    uint8_t rsp_slot_count;   /**< Response slot count */
    uint16_t subevent_data_length; /**< Length of the subevent indication data  */
    uint8_t subevent_data[1];      /**< Start of the subevent data of \p subevent_data_length  */
} wiced_ble_padv_subevent_data_t;

/** Configuration for Periodic Advertising with Response (PAWR) response data*/
typedef struct
{
    uint16_t req_event;    /**< Request Event */
    uint8_t req_subevent;  /**< Request Subevent */
    int8_t rsp_subevent;   /**< Response Subevent */
    uint8_t rsp_slot;      /**< Response Slot */
    uint16_t rsp_data_len; /**< Response data length */
    uint8_t *p_data;       /**< Response data  */
} wiced_ble_padv_subevent_rsp_data_t;

/** Parameters for LE Set Periodic Advertising Parameter command */
typedef struct
{
    /** Minimum Periodic Advertising Interval  */
    uint16_t adv_int_min;
    /** Maximum Periodic Advertising Interval  */
    uint16_t adv_int_max;
    /** Advertising properties */
    wiced_ble_padv_prop_t adv_properties;
    /** Number of subevents, applies to PAWR */
    uint8_t subevent_num;
    /** Interval between subevents,
     * applies to PAWR valid only if \p subevent_num is not zero
     */
    uint8_t subevent_interval;
    /** Response slot delay,
     * applies to PAWR valid only if \p subevent_num is not zero
     */
    uint8_t rsp_slot_delay;
    /** Response slot spacing,
     * applies to PAWR valid only if \p subevent_num is not zero
     */
    uint8_t rsp_slot_spacing;
    /** Number of subevent response slots,
     * applies to PAWR valid only if \p subevent_num is not zero
     */
    uint8_t rsp_slot_num;
} wiced_ble_padv_params_t;

/**< @endcond */

/** Peridic Adv Create Sync parameters */
typedef struct
{
    wiced_ble_padv_sync_options_t options;   /**< Options for the sync cmd */
    wiced_ble_ext_adv_sid_t adv_sid;        /**< adv sid to identify periodic adv */
    wiced_bt_ble_address_type_t adv_addr_type; /**< peer address type  */
    wiced_bt_device_address_t adv_addr;        /**< peer address */
    uint16_t skip;         /**< max number of periodic adv events that can be skipped after a successful receive */
    uint16_t sync_timeout; /**< synchronization timeout for the PA train,
                      range : 0xA - 0x4000,
                      Time = sync_timeout * 10ms => 100ms to 163.84s */
    uint8_t sync_cte_type; /**< bit 0 - Do not sync to packets with an AoA Constant Tone Extension
                                bit 1 - Do not sync to packets with an AoD Constant Tone Extension with 1 μs slots
                                bit 2 - Do not sync to packets with an AoD Constant Tone Extension with 2 μs slots
                                bit 3 - Do not sync to packets with a type 3 Constant Tone Extension*/
} wiced_ble_padv_create_sync_params_t;

/** Sync_Handle to be used to identify the periodic advertiser. Range: 0x0000-0x0EFF
 * */
typedef uint16_t wiced_ble_padv_sync_handle_t;

/** Advertiser clock accuracy */
enum wiced_ble_padv_clock_accuracy_e
{
    PERIODIC_ADVERTISER_CLK_ACCURACY_500PPM, /**< Advertiser clock accuracy 500 ppm */
    PERIODIC_ADVERTISER_CLK_ACCURACY_250PPM, /**< Advertiser clock accuracy 250 ppm */
    PERIODIC_ADVERTISER_CLK_ACCURACY_150PPM, /**< Advertiser clock accuracy 150 ppm */
    PERIODIC_ADVERTISER_CLK_ACCURACY_100PPM, /**< Advertiser clock accuracy 100 ppm */
    PERIODIC_ADVERTISER_CLK_ACCURACY_75PPM,  /**< Advertiser clock accuracy 75 ppm */
    PERIODIC_ADVERTISER_CLK_ACCURACY_50PPM,  /**< Advertiser clock accuracy 50 ppm */
    PERIODIC_ADVERTISER_CLK_ACCURACY_30PPM,  /**< Advertiser clock accuracy 30 ppm */
    PERIODIC_ADVERTISER_CLK_ACCURACY_20PPM,  /**< Advertiser clock accuracy 20 ppm */
};
/** Advertiser clock accuracy (see #wiced_ble_padv_clock_accuracy_e) */
typedef uint8_t wiced_ble_padv_clock_accuracy_t;

/**
 * Periodic Advertising Sync Established Event Data
 * Sync established to periodic advertiser event data format.
 * (The LE Periodic Advertising Sync Established event indicates that the
 * Controller has received the first periodic advertising packet from an advertiser
 * after the LE_Periodic_Advertising_Create_Sync Command has been sent to
 * the Controller.)
*/
typedef struct
{
    /** HCI status */
    uint8_t status;
    /** sync handle */
    wiced_ble_padv_sync_handle_t sync_handle;
    /** advertisement set identifier */
    wiced_ble_ext_adv_sid_t adv_sid;
    /** advertiser address type */
    wiced_bt_ble_address_type_t adv_addr_type;
    /** advertiser address */
    wiced_bt_device_address_t adv_addr;
    /** advertiser phy */
    wiced_ble_ext_adv_phy_t adv_phy;
    /** Periodic adv interval */
    uint16_t periodic_adv_int;
    /** advertiser clock accuracy */
    wiced_ble_padv_clock_accuracy_t advertiser_clock_accuracy;
    /** number of subevents, valid only for PAWR */
    uint8_t num_subevents;
    /** subevent interval, valid only for PAWR*/
    uint8_t subevent_interval;
    /** response slot delay, valid only for PAWR*/
    uint8_t response_slot_delay;
    /** response slot spacing, valid only for PAWR*/
    uint8_t response_slot_spacing;
} wiced_ble_padv_sync_established_event_data_t;

/** Periodic Adv Sync Transfer Received Event Data */
typedef struct
{
    /** Periodic Adv Sync Data */
    wiced_ble_padv_sync_established_event_data_t sync_data;
    /** connection handle */
    wiced_bt_ble_connection_handle_t conn_handle;
    /** Service Data value provided by the peer device */
    uint16_t service_data;
} wiced_ble_padv_sync_transfer_event_data_t;

/** Periodic advertising report data format */
typedef struct
{
    /** sync handle */
    wiced_ble_padv_sync_handle_t sync_handle;
    /** tx power */
    uint8_t tx_power;
    /** rssi */
    uint8_t rssi;
    /** constant tone extension */
    uint8_t cte_type;
    /** Periodic Event counter, valid only for PAWR*/
    uint16_t periodic_evt_counter;
    /** Subevent number, valid only for PAWR */
    uint8_t sub_event;
    /** data status */
    uint8_t data_status;
    /** Length of the subevent indication data  */
    uint16_t data_length;
    /** Data received from a Periodic Advertising packet  */
    uint8_t * p_data;
} wiced_ble_padv_report_event_data_t;

/** Mode used in Periodic Advertising Sync Transfer Parameters */
enum wiced_ble_padv_sync_transfer_mode_e
{
    /** No attempt is made to synchronize to the Periodic Advertising (PA) and no
     * HCI_LE_Periodic_Advertising_Sync_Transfer_Received event is sent to the Host (default)
     * */
    WICED_BLE_PERIODIC_IGNORE_PA_SYNC_TRANSFER_EVT,
    /** An HCI_LE_Periodic_Advertising_Sync_Transfer_Received event is sent to the Host.
     * HCI_LE_Periodic_Advertising_Report events will be disabled.
     * */
    WICED_BLE_PERIODIC_ENABLE_PA_SYNC_TRANSFER_DISABLE_PA_REPORT_EVT,
    /** An HCI_LE_Periodic_Advertising_Sync_Transfer_Received event is sent to the Host.
     * HCI_LE_Periodic_Advertising_Report events will be enabled.
     * */
    WICED_BLE_PERIODIC_ENABLE_PA_SYNC_TRANSFER_ENABLE_PA_REPORT_EVT,
};
/** Mode used in create periodic sync to periodic adv command
 * (see #wiced_ble_padv_sync_transfer_mode_e)
 * */
typedef uint8_t wiced_ble_padv_sync_transfer_mode_t;

/** Periodic adv sync transfer params*/
typedef struct
{
    /** The number of periodic advertising packets that can be skipped after a successful receive */
    uint16_t skip;
    /** Sync timeout value */
    uint16_t sync_timeout;
    /** Periodic Adv sync transfer mode */
    wiced_ble_padv_sync_transfer_mode_t mode;

    /** bit 0 - Do not sync to packets with an AoA Constant Tone Extension
    * bit 1 - Do not sync to packets with an AoD Constant Tone Extension with 1 μs slots
    * bit 2 - Do not sync to packets with an AoD Constant Tone Extension with 2 μs slots
    * bit 3 - Do not sync to packets with a type 3 Constant Tone Extension */
    uint8_t sync_cte_type;
} wiced_ble_padv_sync_transfer_param_t; /**< Periodic adv sync transfer params*/


/**
* Function         wiced_ble_padv_set_adv_params
*                  This API is called on a central to set the PAWR parameters
*
* @param[in]  adv_handle    Handle of the Advertising Set
* @param[out] p_adv_params  Pointer to p_adv_params
* @return          wiced_result_t
*                  WICED_BT_SUCCESS contents of features are valid
*                  WICED_BT_ERROR   otherwise.
*/
wiced_bt_dev_status_t wiced_ble_padv_set_adv_params(wiced_ble_ext_adv_handle_t adv_handle,
                                                    wiced_ble_padv_params_t *p_adv_params);

/**
 * Sends the HCI command to write the periodic adv data
 *
 * @param[in]    adv_handle        advertisement set handle
 * @param[in]    adv_data_length   periodic data length
 * @param[in]    p_adv_data        pointer to the periodic data
 *
 * @return          wiced_bt_dev_status_t
 * <b> WICED_BT_ILLEGAL_VALUE </b> : If paramer is wrong \n
 * <b> WICED_BT_UNSUPPORTED </b>   : If command not supported \n
 * <b> WICED_BT_NO_RESOURCES </b>  : If no memory to issue the command \n
 * <b> WICED_BT_SUCCESS </b>       : If successful\n
 *
 */
wiced_bt_dev_status_t wiced_ble_padv_set_adv_data(wiced_ble_ext_adv_handle_t adv_handle,
                                                  uint16_t adv_data_length,
                                                  uint8_t *p_adv_data);

/**
 * Sends the HCI command to enable/disable periodic advertisements

 * @param[in]       adv_handle  - handle of the advertising set
 * @param[in]       enable      - true to enable, false to disable
 *
 * @return          wiced_bt_dev_status_t
 * <b> WICED_BT_ILLEGAL_VALUE </b> : If paramer is wrong \n
 * <b> WICED_BT_UNSUPPORTED </b>   : If command not supported \n
 * <b> WICED_BT_NO_RESOURCES </b>  : If no memory to issue the command \n
 * <b> WICED_BT_SUCCESS </b>       : If successful\n
 *
 */
wiced_bt_dev_status_t wiced_ble_padv_enable_adv(wiced_ble_ext_adv_handle_t adv_handle, wiced_bool_t enable);

/**
 * Sends the HCI command to synchronize with periodic advertising from an advertiser and begin receiving periodic
 * advertising packets.
 *
 * @param[in] p_sync_params : Parameters for sync to periodic adv
 *
 * @return wiced_bt_dev_status_t
 *
 * <b> WICED_BT_UNSUPPORTED </b>   : If command not supported \n
 * <b> WICED_BT_NO_RESOURCES </b>  : If no memory to issue the command \n
 * <b> WICED_BT_SUCCESS </b>       : If successful\n
 *
 */
wiced_bt_dev_status_t wiced_ble_padv_create_sync(wiced_ble_padv_create_sync_params_t *p_sync_params);

/**
 * Sends HCI command to cancel the create sync command while it is pending.
 *
 *
 * @return          wiced_bt_dev_status_t
 * <b> WICED_BT_UNSUPPORTED </b>   : If command not supported \n
 * <b> WICED_BT_NO_RESOURCES </b>  : If no memory to issue the command \n
 * <b> WICED_BT_SUCCESS </b>       : If successful\n
 *
 */
wiced_bt_dev_status_t wiced_ble_padv_cancel_sync(void);

/**
 * Sends the HCI command to stop reception of periodic advertising identified by the sync_handle
 *
 * @param[in]       sync_handle      - Sync handle received in WICED_BLE_PERIODIC_ADV_SYNC_ESTABLISHED_EVENT.
 *
 * @return          wiced_bt_dev_status_t
 *
 * <b> WICED_BT_UNSUPPORTED </b>   : If command not supported \n
 * <b> WICED_BT_NO_RESOURCES </b>  : If no memory to issue the command \n
 * <b> WICED_BT_SUCCESS </b>       : If successful \n
 *
 */
wiced_bt_dev_status_t wiced_ble_padv_terminate_sync(uint16_t sync_handle);

/**
 * Sends the HCI command to add the given advertiser to Periodic Advertiser list.
 *
 * Note : Caller shall not attempt to add more than max list size
 *       Shall not attempt to call this API, while create to periodic sync command is pending.
 *
 * @param[in]      advertiser_addr_type : Periodic advertiser addr type
 * @param[in]      advetiser_addr       : Periodic advertiser addr
 * @param[in]      adv_sid              : Periodic advertiser sid
 *
 * @return          wiced_bt_dev_status_t
 * <b> WICED_BT_ILLEGAL_VALUE </b> : If paramer is wrong \n
 * <b> WICED_BT_UNSUPPORTED </b>   : If command not supported \n
 * <b> WICED_BT_NO_RESOURCES </b>  : If no memory to issue the command \n
 * <b> WICED_BT_PENDING </b>       : If command queued to send down \n
 * <b> WICED_BT_SUCCESS </b>       : If successful\n
 *
 */
wiced_bt_dev_status_t wiced_ble_padv_add_device_to_list(wiced_bt_ble_address_type_t advertiser_addr_type,
                                                        wiced_bt_device_address_t advetiser_addr,
                                                        wiced_ble_ext_adv_sid_t adv_sid);

/**
 * Sends the HCI command to remove the given advertiser from Periodic Advertiser list.
 *
 * Note : Shall not attempt to call this API, while create to periodic sync command is pending.
 *
 * @param[in]      advertiser_addr_type : Periodic advertiser addr type
 * @param[in]      advetiser_addr       : Periodic advertiser addr
 * @param[in]      adv_sid              : Periodic advertiser sid
 *
 * @return          wiced_bt_dev_status_t
 * <b> WICED_BT_ILLEGAL_VALUE </b> : If paramer is wrong \n
 * <b> WICED_BT_UNSUPPORTED </b>   : If command not supported \n
 * <b> WICED_BT_NO_RESOURCES </b>  : If no memory to issue the command \n
 * <b> WICED_BT_SUCCESS </b>       : If successful\n
 *
 */
wiced_bt_dev_status_t wiced_ble_padv_remove_device_from_list(wiced_bt_ble_address_type_t advertiser_addr_type,
                                                             wiced_bt_device_address_t advetiser_addr,
                                                             wiced_ble_ext_adv_sid_t adv_sid);

/**
 * Sends the HCI command to remove to remove all devices from the the Periodic Advertisers list.
 *
 * Note : Shall not attempt to call this API, while create to periodic sync command is pending.
 *
 * @return          wiced_bt_dev_status_t
 * <b> WICED_BT_UNSUPPORTED </b>   : If command not supported \n
 * <b> WICED_BT_NO_RESOURCES </b>  : If no memory to issue the command \n
 * <b> WICED_BT_SUCCESS </b>       : If successful\n
 *
 */
wiced_bt_dev_status_t wiced_ble_padv_clear_list(void);

/**
 * Sends the HCI command to enable or disable receiving periodic ADV data for a sync handle.
 *
 * @param[in]       sync_handle : Sync handle
 * @param[in]       enable      : Boolean for enable/disable.
 *
 * @return          wiced_bt_dev_status_t
 * <b> WICED_BT_UNSUPPORTED </b>   : If command not supported \n
 * <b> WICED_BT_NO_RESOURCES </b>  : If no memory to issue the command \n
 * <b> WICED_BT_SUCCESS </b>       : If command queued to send down \n
 *
 */
wiced_bt_dev_status_t wiced_ble_padv_set_rcv_enable(wiced_ble_padv_sync_handle_t sync_handle, wiced_bool_t enable);

/**
 * Sends the HCI command to send synchronization information about the periodic advertising train identified by the Sync_Handle parameter to given device
 *
 * @param[in]       peer_bda        - Peer Bluetooth Address
 * @param[in]       service_data    - Service Data value
 * @param[in]       sync_handle : Sync handle
 *
 * @return          wiced_bt_dev_status_t
 * <b> WICED_BT_UNKNOWN_ADDR </b> :  If Unknown remote BD address \n
 * <b> WICED_BT_NO_RESOURCES </b>  : If no memory to issue the command \n
 * <b> WICED_BT_SUCCESS </b>       : If command queued to send down \n
 *
 */
wiced_bt_dev_status_t wiced_ble_padv_transfer_sync(wiced_bt_device_address_t peer_bda,
                                                   uint16_t service_data,
                                                   wiced_ble_padv_sync_handle_t sync_handle);

/**
 * Sends the HCI command  to send synchronization information about the periodic advertising in an advertising
 * set to the connected device.
 *
 * @param[in]       peer_bda        - Peer Bluetooth Address
 * @param[in]       service_data    - Service Data value
 * @param[in]       adv_handle  - handle of the advertising set
 *
 * @return          wiced_bt_dev_status_t
 * <b> WICED_BT_UNKNOWN_ADDR </b> :  If Unknown remote BD address \n
 * <b> WICED_BT_NO_RESOURCES </b>  : If no memory to issue the command \n
 * <b> WICED_BT_SUCCESS </b>       : If command queued to send down \n
 *
 */
wiced_bt_dev_status_t wiced_ble_padv_transfer_set_info(wiced_bt_device_address_t peer_bda,
                                                       uint16_t service_data,
                                                       wiced_ble_ext_adv_handle_t adv_handle);

/**
 * Sends the HCI command to set synchronize periodic transfer parameter
 *
 * @param[in]       peer_bda        - Peer Bluetooth Address
 * @param[in]       p_st            - sync transfer params
 *
 * @return          wiced_bt_dev_status_t
 * <b> WICED_BT_UNKNOWN_ADDR </b> :  If Unknown remote BD address \n
 * <b> WICED_BT_NO_RESOURCES </b>  : If no memory to issue the command \n
 * <b> WICED_BT_SUCCESS </b>       : If successful\n
 *
 */
wiced_bt_dev_status_t wiced_ble_padv_set_sync_transfer_params(wiced_bt_device_address_t peer_bda,
                                                              wiced_ble_padv_sync_transfer_param_t *p_st);


/**
 * Sends the HCI command to set Default synchronize periodic transfer parameter
 *
 * @param[in] p_st - sync transfer params
 *
 * @return          wiced_bt_dev_status_t
 * <b> WICED_BT_NO_RESOURCES </b>  : If no memory to issue the command \n
 * <b> WICED_BT_SUCCESS </b>       : If successful\n
 *
 */
wiced_bt_dev_status_t
wiced_ble_padv_set_default_sync_transfer_params( wiced_ble_padv_sync_transfer_param_t *p_st);


/* @cond PAWR_API
   APIs for Periodic Advertising with Response*/

/**
* Function         wiced_ble_padv_set_subevent_data
*
*                  This API is used by the Host to set the data for one or more subevents of PAwR in reply to an
*                  HCI_LE_Periodic_Advertising_Subevent_Data_Request event. The data for a subevent shall be transmitted only once.
*
* @param[in] adv_handle        Handle of the Advertising Set
* @param[in] num_subevents     Number of subevent data in the command
* @param[in] p_se_data         Pointer to the subevent data
* @note:
* @return          wiced_bt_dev_status_t
*                  WICED_BT_SUCCESS contents of features are valid
*                  WICED_BT_ERROR   otherwise.
*/
wiced_bt_dev_status_t wiced_ble_padv_set_subevent_data(wiced_ble_ext_adv_handle_t adv_handle,
                                                       uint8_t num_subevents,
                                                       wiced_ble_padv_subevent_data_t *p_se_data);

/**
* Function         wiced_ble_padv_configure_subevent_response_data
*
*                  This API is used by the Host to configure the reassembly of periodic subevent response data
*                  segments. Invoking this API is optional. Application may use this API in case it expects the
*                  subevent responses to come in with data_status=1 (partial).
*                  In case this API is invoked, the stack will reassemble the incoming data segments and send up
*                  the #WICED_BT_BLE_PAWR_RSP_REPORT_EVENT
*
* @param[in] max_response_len  max total response length expected across mutiple data reports for a subevent
* @param[in] max_responses     max number of subevent responses to be tracked
*
* @return          wiced_bt_dev_status_t
*                  WICED_BT_SUCCESS contents of features are valid
*                  WICED_BT_ERROR   otherwise.
*/
wiced_bt_dev_status_t wiced_ble_padv_configure_subevent_response_data_reassembly(uint16_t max_response_len,
                                                                                 uint8_t max_responses);

/**
* Function         wiced_ble_padv_set_subevent_rsp_data
*
*                  This API is used by the Host to set the data for a response slot in a specific subevent of the PAwR
*                  identified by the \p sync_handle. The data for a response slot shall be transmitted only once.
*
* @param[in]  sync_handle    Handle of the synchronized advertising train
* @param[out] p_rsp_data  Pointer to p_rsp_data
* @return          wiced_result_t
*                  WICED_BT_SUCCESS contents of features are valid
*                  WICED_BT_ERROR   otherwise.
*/
wiced_bt_dev_status_t wiced_ble_padv_set_subevent_rsp_data(uint16_t sync_handle,
                                                           wiced_ble_padv_subevent_rsp_data_t *p_rsp_data);

/**
* Function         wiced_ble_padv_set_sync_subevent
*
*                  This API is called to instruct the Controller to synchronize with a subset of the subevents
*                  within a PAwR train identified by the \p sync_handle, listen for packets sent by the peer device
*                  and pass any received data up to the Host
*
* @param[in]  sync_handle     Handle of the synchronized periodic ADV train
* @param[in]  properties      Properties of the synchronized periodic ADV train
* @param[in]  num_subevents   Number of subevents
* @param[in]  p_subevents     Pointer to an array of subevents
*
* @return          wiced_result_t
*                  WICED_BT_SUCCESS contents of features are valid
*                  WICED_BT_ERROR   otherwise.
*/
wiced_bt_dev_status_t wiced_ble_padv_set_sync_subevent(uint16_t sync_handle,
                                                       uint16_t properties,
                                                       uint8_t num_subevents,
                                                       uint8_t *p_subevents);

/* @endcond */

/**
* API allocates an object which can reassemble a periodic adv packet on the periodic adv scanner.
* upto a length of \p max_adv_len
* Application can invoke this API in the #WICED_BLE_PERIODIC_ADV_SYNC_ESTABLISHED_EVENT event on successful
* sync establishment. The object created can be reused for subsequent calls to append the periodic adv reports.
* The object is freed by the stack in case the periodic sync is lost or terminated.
* @note This API allocates the reassembly buffer + header from the application default heap created using a call
* to \ref wiced_bt_create_heap (b_make_default =1)
*
* @param [in] sync_handle Handle of the synchronized periodic ADV train
* @param [in] max_adv_len Max length to reassemble
*
* @return wiced_result_t
*
*/
wiced_result_t wiced_ble_padv_alloc_segment_assembler(wiced_ble_padv_sync_handle_t sync_handle, uint16_t max_adv_len);

/**@} wicedbt */
#endif // WICED_BLE_ENABLE_EXTENDED_ADV_API
#endif // __WICED_BT_ADV_SCAN_PERIODIC_H__
