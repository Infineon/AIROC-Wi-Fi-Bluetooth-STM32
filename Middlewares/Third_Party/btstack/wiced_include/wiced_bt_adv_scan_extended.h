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
 * AIROC Bluetooth Low Energy (LE) Functions for Extended Advertisement and Scan
 *
 */
#ifndef __WICED_BT_ADV_SCAN_EXTENDED_H__
#define __WICED_BT_ADV_SCAN_EXTENDED_H__

#ifndef WICED_BLE_ENABLE_EXTENDED_ADV_API
#define WICED_BLE_ENABLE_EXTENDED_ADV_API 1
#endif

#if (WICED_BLE_ENABLE_EXTENDED_ADV_API == 1)

#if (WICED_BLE_ENABLE_LEGACY_EXTENDED_API_ERROR_CHECK)
#if (WICED_BLE_ENABLE_LEGACY_ADV_API == 1)
#error "Cannot enable legacy and extended adv together"
#endif
#endif


/**
 * This section contains LE Extended advertisement and scanning defines, structures and functions.
 * @note Applications using extended and periodic mode advertising and scanning shall use only extended mode APIs
 * cannot invoke any of the Legacy mode APIs for advertisement and scanning
 *
 * @addtogroup  wicedbt_Extended   LE Extended Mode Advertising and Scanning
 *
 * @ingroup     wicedbt
 *
 * @{
 */

/** Extended Scan result event type */
enum
{
    BTM_BLE_EXT_EVT_CONNECTABLE_ADVERTISEMENT = 0x1, /**< Connectable adv report */
    BTM_BLE_EXT_EVT_SCANNABLE_ADVERTISEMENT = 0x2,   /**< Scannable adv report */
    BTM_BLE_EXT_EVT_DIRECTED_ADVERTISEMENT = 0x4,    /**< Directed adv report */
    BTM_BLE_EXT_EVT_SCAN_RESPONSE = 0x8,             /**< Scan response */
    BTM_BLE_EXT_EVT_LEGACY_ADV_PDU = 0x10,           /**< Legacy PDU adv report */
    BTM_BLE_EXT_EVT_DATA_STATUS = 0x60,              /**< Extended adv report data status */
};
typedef uint16_t wiced_ble_ext_adv_evt_type_mask_t; /**< Extended adv report event type */

#define IS_BTM_BLE_EXT_EVT_COMPLETE_DATA(evt)                                                                          \
    (((evt)&BTM_BLE_EXT_EVT_DATA_STATUS) == 0) /**< Complete data packet received */
#define IS_BTM_BLE_EXT_EVT_INCOMPLETE_DATA(evt)                                                                        \
    (((evt)&BTM_BLE_EXT_EVT_DATA_STATUS) == 1) /**< Incomplete data packet received, more to come */
#define IS_BTM_BLE_EXT_EVT_TRUNCATED_DATA(evt)                                                                         \
    (((evt)&BTM_BLE_EXT_EVT_DATA_STATUS) == 2) /**< Truncated data packet received, no more to come */


/** LE PHY bit mask */
enum
{
    WICED_BLE_EXT_ADV_PHY_1M_BIT = (1 << 0),       /**< Bit mask to specify for LE1M PHY */
    WICED_BLE_EXT_ADV_PHY_2M_BIT = (1 << 1),       /**< Bit mask to specify for LE2M PHY */
    WICED_BLE_EXT_ADV_PHY_LE_CODED_BIT = (1 << 2), /**< Bit mask to specify for LE coded PHY */
};
/** LE phy mask to be used for extended advertisement */
typedef uint8_t wiced_ble_ext_adv_phy_mask_t;

/** Advertising event properties: Describes the type of advertising event that is being configured and its basic properties */
enum
{
    /** Connectable ADV */
    WICED_BLE_EXT_ADV_EVENT_PROPERTY_CONNECTABLE_ADV = (1 << 0),
    /** Scannable ADV */
    WICED_BLE_EXT_ADV_EVENT_PROPERTY_SCANNABLE_ADV = (1 << 1),
    /** Low duty cycle directed advertisement */
    WICED_BLE_EXT_ADV_EVENT_PROPERTY_DIRECTED_ADV = (1 << 2),
    /** 3.75 ms Adv Interval, only valid in legacy ADV */
    WICED_BLE_EXT_ADV_EVENT_PROPERTY_HIGH_DUTY_DIRECTED_CONNECTABLE_ADV = (1 << 3),
    /** Legacy Advertisement. Adv data cannot be more than 31 bytes.*/
    WICED_BLE_EXT_ADV_EVENT_PROPERTY_LEGACY_ADV = (1 << 4),
    /** Omits advertisers address from all PDUs */
    WICED_BLE_EXT_ADV_EVENT_PROPERTY_ANONYMOUS_ADV = (1 << 5),
    /** Include Tx power in ext ADV pdus */
    WICED_BLE_EXT_ADV_EVENT_PROPERTY_INCLUDE_TX_POWER = (1 << 6),
    /** Other bits RFU */
};
/** LE extended advertisement event property for setting extended adv params*/
typedef uint16_t wiced_ble_ext_adv_event_property_t;

/** Advertisement set handle to identify adv set b/n host and controller */
enum
{
    WICED_BLE_EXT_ADV_HANDLE_MIN = 0x00, /**< min advertisement set handle value */
    WICED_BLE_EXT_ADV_HANDLE_MAX = 0xef, /**< max advertisement set handle value */
};


/** Value to configure to receive scan request recived notification */
enum wiced_ble_ext_adv_scan_req_notification_setting_e
{
    WICED_BLE_EXT_ADV_SCAN_REQ_NOTIFY_DISABLE = 0x00, /**< Do not send Notification on scan request */
    WICED_BLE_EXT_ADV_SCAN_REQ_NOTIFY_ENABLE = 0x01,  /**< Send Notification on scan request */
};
/** Enable or disable notification value (see #wiced_ble_ext_adv_scan_req_notification_setting_e) */
typedef uint8_t wiced_ble_ext_adv_scan_req_notification_setting_t;

/** Advertisement duration configuration for specified adv handle */
typedef struct
{
    /** advertisement set handle */
    wiced_ble_ext_adv_handle_t adv_handle;

    /** 0 = No advertising duration. Advertising to continue until the Host disables it.
     * 0xXXXX = Range: 0x0001 - 0xFFFF (10 ms to 655,350 ms)
     */
    uint16_t adv_duration;

    /** 0xXX: Maximum number of extended advertising events the Controller shall
     * attempt to send prior to disabling the extended advertising set even if
     * the adv_duration parameter has not expired.
     * 0x00: No maximum number of advertising events
     */
    uint8_t max_ext_adv_events;
} wiced_ble_ext_adv_duration_config_t;

/** Extended scan results */
typedef struct
{
    /** extended scan evt type */
    wiced_ble_ext_adv_evt_type_mask_t ext_evt_type;
    /** LE Address type */
    uint8_t ble_addr_type;
    /** Device address */
    wiced_bt_device_address_t remote_bd_addr;
    /** Primary PHY, valid only if \p is_extended is True */
    uint8_t primary_phy;
    /** Secondary PHY, valid only if \p is_extended is True */
    uint8_t secondary_phy;
    /** advertisement security ID, valid only if \p is_extended is True */
    uint8_t adv_sid;
    /** Tx power, valid only if \p is_extended is True */
    uint8_t tx_power;
    /** Set to #BTM_INQ_RES_IGNORE_RSSI, if not valid */
    int8_t rssi;
    /** Periodic advertisement interval, valid only if \p is_extended is True */
    uint16_t periodic_adv_interval;
    /** Directed address type, valid only if \p is_extended is True */
    uint8_t direct_addr_type;
    /** Directed address, valid only if \p is_extended is True */
    wiced_bt_device_address_t direct_bda;
    /** length of the adv_data */
    uint16_t adv_len;
} wiced_ble_ext_scan_results_t;


/**
 * Callback for receiving extended scan results
 *
 * Extended Scan result callback (from calling #wiced_ble_ext_scan_enable)
 *
 * @param p_scan_result   : scan result data (NULL indicates end of scanning)
 * @param adv_data_len    : length of the pointer pointed to by \p p_adv_data
 * @param p_adv_data      : Advertisement data (parse using \ref wiced_ble_adv_data_search)
 *
 * @return void
 */
typedef void(wiced_ble_ext_scan_result_cback_t)(wiced_ble_ext_scan_results_t *p_scan_result,
                                                uint16_t adv_data_len,
                                                uint8_t *p_adv_data);

/** Extended scan duplicate filter policy */
enum wiced_ble_ext_scan_filter_duplicate_e
{
    /** send all advertisements received from advertisers*/
    WICED_BLE_EXT_SCAN_FILTER_DUPLICATE_DISABLE,
    /** duplicate advertisements should not be sent until scan disabled */
    WICED_BLE_EXT_SCAN_FILTER_DUPLICATE_ENABLE,
    /** filter duplicate adv during a single scan duration.
     * Period should be non zero on using this option */
    WICED_BLE_EXT_SCAN_FILTER_DUPLICATE_ENABLE_RESET_ON_SCAN_PERIOD,
};
/** Extended scan duplicate filter policy (see #wiced_ble_ext_scan_filter_duplicate_e) */
typedef uint8_t wiced_ble_ext_scan_filter_duplicate_t;

/** Initiator filter policy enums  used in extended create connection command */
enum wiced_ble_ext_conn_initiator_filter_policy_e
{
    /** Filter Accept List is not used to determine which advertiser to connect to.
     * Peer_Address_Type and Peer_Address shall be used
     * */
    WICED_BT_BLE_IGNORE_FILTER_ACCEPT_LIST_FOR_CONNS = 0,
    /** Filter Accept List is used to determine which advertiser to connect to.
     * Peer_Address_Type and Peer_Address shall be ignored.
     * */
    WICED_BT_BLE_USE_FILTER_ACCEPT_LIST_FOR_CONNS = 1,
};
/** Initiator filter policy used. (see #wiced_ble_ext_conn_initiator_filter_policy_e) */
typedef uint8_t wiced_ble_ext_conn_initiator_filter_policy_t;

/** Scanning filter policy enums used in set extended scan parameters command */
enum wiced_ble_ext_scanning_filter_policy_e
{
    WICED_BLE_EXT_SCAN_BASIC_UNFILTERED_SP = WICED_BLE_SCAN_BASIC_UNFILTERED_SP,    /**< Basic unfiltered scanning policy */
    WICED_BLE_EXT_SCAN_BASIC_FILTERED_SP = WICED_BLE_SCAN_BASIC_FILTERED_SP,      /**< Basic filtered scanning policy  */
    WICED_BLE_EXT_SCAN_EXTENDED_UNFILTERED_SP = WICED_BLE_SCAN_EXTENDED_UNFILTERED_SP, /**< Extended unfiltered scanning policy */
    WICED_BLE_EXT_SCAN_EXTENDED_FILTERED_SP = WICED_BLE_SCAN_EXTENDED_FILTERED_SP,   /**< Extended filtered scanning policy  */
};

/** Phy adv options to be set in #wiced_ble_ext_adv_set_params */
enum wiced_ble_ext_adv_phy_options_e
{
    /** The Host has no preferred or required coding when transmitting on the LE Coded PHY  */
    WICED_BLE_EXT_ADV_PHY_OPTIONS_NO_PREFERENCE = 0,
    /** The Host prefers that S=2 coding be used when transmitting on the LE Coded PHY */
    WICED_BLE_EXT_ADV_PHY_OPTIONS_PREFER_S2 = 1,
    /** The Host prefers that S=8 coding be used when transmitting on the LE Coded PHY */
    WICED_BLE_EXT_ADV_PHY_OPTIONS_PREFER_S8 = 2,
    /** The Host requires that S=2 coding be used when transmitting on the LE Coded PHY */
    WICED_BLE_EXT_ADV_PHY_OPTIONS_REQUIRE_S2 = 3,
    /** The Host requires that S=8 coding be used when transmitting on the LE Coded PHY */
    WICED_BLE_EXT_ADV_PHY_OPTIONS_REQUIRE_S8 = 4,
};
/** Phy adv options to be set in #wiced_ble_ext_adv_set_params.
 * (see #wiced_ble_ext_adv_phy_options_e)*/
typedef uint8_t wiced_ble_ext_adv_phy_options_t;

/** Parameters for extended adv */
typedef struct
{
    /** Bit mask to specify connectable, scannable, low duty, high duty, directed, legacy adv */
    wiced_ble_ext_adv_event_property_t event_properties;
    /** Min primary adv interval Range : 0x000020 to 0xFFFFFF(20 ms to 10, 485.759375 s) */
    uint32_t primary_adv_int_min;

    /** Max primary adv interval Range : 0x000020 to 0xFFFFFF(20 ms to 10, 485.759375 s) */
    uint32_t primary_adv_int_max;

    /** LE advertisement channel map(see #wiced_bt_ble_advert_chnl_map_e) */
    wiced_bt_ble_advert_chnl_map_t primary_adv_channel_map;

    /** Ignored in case of anonymous adv.See \p event_properties */
    wiced_ble_own_address_options_t own_addr_type;

    /** Peer device address type */
    wiced_bt_ble_address_type_t peer_addr_type;

    /** Peer device address */
    wiced_bt_device_address_t peer_addr;

    /** Adv filter policy */
    wiced_bt_ble_advert_filter_policy_t adv_filter_policy;

    /** Adv TX Power - 127 to + 126. 127 means host has no preference */
    int8_t adv_tx_power;

    /** Primary_adv_phy Phy used to transmit ADV packets on Primary ADV channels */
    wiced_ble_ext_adv_phy_t primary_adv_phy;

    /** Secondary_adv_max_skip Valid only in case of extended ADV.Range 0 to FF.Maximum advertising events controller
        can skip before sending auxiliary adv packets on the secondary adv channel */
    uint8_t secondary_adv_max_skip;

    /** Secondary_adv_phy Phy used to transmit ADV packets on secondary ADV channels .
     * Valid only in case of extended ADV */
    wiced_ble_ext_adv_phy_t secondary_adv_phy;

    /** Advertisement set identifier is the value to be transmitted in extended ADV PDUs */
    wiced_ble_ext_adv_sid_t adv_sid;

    /** scan_request_not scan request received notification enable/ disable */
    wiced_ble_ext_adv_scan_req_notification_setting_t scan_request_not;

    /** primary phy adv options */
    wiced_ble_ext_adv_phy_options_t primary_phy_opts;

    /** secondary phy adv options */
    wiced_ble_ext_adv_phy_options_t secondary_phy_opts;
} wiced_ble_ext_adv_params_t;

/** Phy options for Extended Create Connection */
typedef struct
{
    uint16_t scan_int;       /**< Range N: 0x0004 to 0xFFFF.Time = N * 0.625 ms. Time Range: 2.5 ms to 40.959375 s */
    uint16_t scan_window;    /**< Range N: 0x0004 to 0xFFFF.Time = N * 0.625 ms. Time Range: 2.5 ms to 40.959375 s */
    uint16_t min_conn_int;   /**< Range N: 0x0006 to 0x0C80 Time = N * 1.25 ms Time Range: 7.5 ms to 4 s */
    uint16_t max_conn_int;   /**< Range N: 0x0006 to 0x0C80 Time = N * 1.25 ms Time Range: 7.5 ms to 4 s */
    uint16_t conn_latency;   /**< Range N: 0x0000 to 0x01F3 */
    uint16_t supervision_to; /**< Range N: 0x000A to 0x0C80 Time = N * 10 ms Time Range: 100 ms to 32 s */
    uint16_t min_ce_len;     /**< Range N: 0x0000 � 0xFFFF. Time = N * 0.625 ms */
    uint16_t max_ce_len;     /**< Range N: 0x0000 � 0xFFFF. Time = N * 0.625 ms */
} wiced_ble_ext_conn_cfg_phy_options_t;

/** Extended ADV connection configuration structure */
typedef struct
{
    /** advertising handle, Range: 0x00 to 0xEF or 0xFF.
     * @note adv_handle shall be set to 0xff in case adv_handle parameter is not to be used */
    uint8_t adv_handle;
    /** subevent,Range: 0x00 to 0x7F or 0xFF.
     * @note sub_event shall be set to 0xff in case sub_event parameter is not to be used */
    uint8_t sub_event;
    /** timeout in secs for creating the connection */
    uint8_t timeout_secs;
    /** Initiator filter policy */
    wiced_ble_ext_conn_initiator_filter_policy_t init_filter_policy;
    /** initiator address type */
    wiced_ble_own_address_options_t own_addr_type;
    /** peer address type */
    wiced_bt_ble_address_type_t peer_addr_type;
    /** peer address */
    wiced_bt_device_address_t peer_addr;
    /** The PHY(s) bit mask on which the advertising packets should be received on the primary advertising channel
     * and the PHYs for which connection parameters have been specified
     * Bit 0 = 1: Scan connectable advertisements on LE 1M PHY with the connection parameters provided
     * Bit 1 = 1: Scan connectable advertisements on LE 1M PHY with the connection parameters provided
     * Bit 2 = 1: Scan connectable advertisements on LE Coded PHY with the connection parameters provided
     * */
    wiced_ble_ext_adv_phy_mask_t initiating_phys;

    /** Phy options for extended create connection */
    wiced_ble_ext_conn_cfg_phy_options_t phy_options[WICED_BLE_EXT_ADV_NUM_PHYS];
} wiced_ble_ext_conn_cfg_t;

#define IS_EXT_ADV_REPORT_CONNECTABLE_ADV(x) (x & (1 << 0))        /**< adv is connectable */
#define IS_EXT_ADV_REPORT_SCANNABLE_ADV(x) (x & (1 << 1))          /**< adv is scannable */
#define IS_EXT_ADV_REPORT_DIRECTED_ADV(x) (x & (1 << 2))           /**< directed adv */
#define IS_EXT_ADV_REPORT_SCAN_RSP(x) (x & (1 << 3))               /**< scan response */
#define IS_EXT_ADV_REPORT_LEGACY_ADV(x) (x & (1 << 4))             /**< legacy adv */
#define IS_EXT_ADV_REPORT_DATA_STATUS_INCOMPLETE(x) (x & (1 << 5)) /**< adv data incomplete, more data to come */
#define IS_EXT_ADV_REPORT_DATA_STATUS_TRUNCATED(x) (x & (2 << 5))  /**< Incomplete, data truncated, no more to come */
/** Bit mask to identify the type of the adv received in extended adv report.
 * (see \ref wiced_ble_ext_scan_results_t.ext_evt_type field )
 * */
typedef uint16_t wiced_ble_ext_adv_report_event_mask_t;

/** Min and Max possible number of reports in LE extended adv report event */
enum wiced_ble_ext_adv_report_count_e
{
    ADV_REP_EVT_COUNT_MIN = 1,  /**< min number of reports in LE extended adv report event */
    ADV_REP_EVT_COUNT_MAX = 10, /**< max number of reports in LE extended adv report event */
};
typedef uint8_t
    wiced_ble_ext_adv_report_count_t; /**< Min and Max reports (see wiced_ble_ext_adv_report_count_e)*/

/** scan request received event data format */
typedef struct
{
    wiced_ble_ext_adv_handle_t adv_handle;      /**< advertisement set handle */
    wiced_bt_ble_address_type_t scanner_addr_type; /**< Scanner address type */
    wiced_bt_device_address_t scanner_address;     /**< Scanner address */
} wiced_ble_ext_scan_req_received_event_data_t;

/** BIGInfo report */
typedef struct
{
    uint16_t sync_handle; /**< Sync_Handle identifying the periodic advertising train (Range: 0x0000 to 0x0EFF) */
    uint8_t num_bis;      /**< Value of the Num_BIS subfield of the BIGInfo field */
    uint8_t number_of_subevents;        /**< Value of the NSE subfield of the BIGInfo field */
    uint16_t iso_interval;              /**< Value of the ISO_Interval subfield of the BIGInfo field */
    uint8_t burst_number;               /**< Value of the BN subfield of the BIGInfo field */
    uint8_t pretransmission_offset;     /**< Value of the PTO subfield of the BIGInfo field */
    uint8_t immediate_repetition_count; /**< Value of the IRC subfield of the BIGInfo field */
    uint16_t max_pdu;                /**< Value of the Max_PDU subfield of the BIGInfo field in the Advertising PDU */
    uint32_t sdu_interval;           /**< Value of the SDU_Interval subfield of the BIGInfo field */
    uint16_t max_sdu;                /**< Value of the Max_SDU subfield of the BIGInfo field in the Advertising PDU */
    wiced_ble_isoc_phy_t phy;         /**< The transmitter PHY of packets */
    wiced_ble_isoc_framing_t framing; /**< Framing parameter */
    wiced_ble_isoc_encryption_t encryption; /**< BIG carries encrypted or unencrypted data */
} wiced_ble_biginfo_adv_report_t;


/** structure for scan parameter per phy */
typedef struct
{
    uint8_t scan_type;      /**< scan type, set 0 for passive, 1 for active */
    uint16_t scan_interval; /**< Range N: 0x0004 to 0xFFFF.Time = N * 0.625 ms. Time Range: 2.5 ms to 40.959375 s*/
    uint16_t scan_window;   /**< Range N: 0x0004 to 0xFFFF.Time = N * 0.625 ms. Time Range: 2.5 ms to 40.959375 s*/
} wiced_ble_ext_scan_phy_params_t;

/** Configuration for extended scanning */
typedef struct
{
    /** Own LE Address type */
    wiced_ble_own_address_options_t own_addr_type;
    /** scan filter policy */
    wiced_ble_scanning_filter_policy_t scan_filter_policy;
    /** Indicates PHY(s) to receive the primary advertising channel.*/
    wiced_ble_ext_adv_phy_mask_t scanning_phys;

    /** scan parameters for 1M Phy */
    wiced_ble_ext_scan_phy_params_t sp_1m;
    /** scan parameters for LE Coded phy */
    wiced_ble_ext_scan_phy_params_t sp_le_coded_phy;
} wiced_ble_ext_scan_params_t;

/** Extended scanning enable params*/
typedef struct
{
    /** filter_duplicates Set 1 to filter duplicates, 0 to allow duplicates */
    uint8_t filter_duplicates;
    /** scan duration, Set 0 to scan continuously till explicit disable, else
     * Range N : 0x0001 to 0xFFFF.
     * Time = N * 0.625 ms.Time Range : 10 ms to 655.35 s */
    uint16_t scan_duration;
    /** scan period, Set 0 to scan continuously, else
     * Range N : 0x0001 to 0xFFFF.
     * Time = N * 1.28 s.Time Range : 1.28 s to 83.884 s  */
    uint16_t scan_period;
} wiced_ble_ext_scan_enable_params_t;

/** extended adv set terminated event data format. This event generated asynchronously by the
 * controller when adv set get terminated either adv duration expires or connection being created */
typedef struct
{
    /** HCI status */
    uint8_t status;
    /** advertisement set handle */
    wiced_ble_ext_adv_handle_t adv_handle;
    /** connection handle. The conn_handle parameter is only valid when advertising
     * ends because a connection was created */
    wiced_bt_ble_connection_handle_t conn_handle;
    /** number of completed extended advertising events the Controller had transmitted
     * when either the duration elapsed or the maximum number of extended advertising
     * events was reached; otherwise it shall be set to zero. */
    uint8_t num_completed_ext_adv_events;
} wiced_ble_ext_adv_set_terminated_event_data_t;


/** Data for the command complete event */
typedef struct
{
    uint16_t cmd_opcode; /**< Command opcode */
    uint8_t cmd_status;  /**< Command status */
} wiced_ble_cmd_cmplt_event_data_t;

/** union of events data */
typedef union
{
    /** Data for WICED_BLE_PERIODIC_ADV_SYNC_ESTABLISHED_EVENT*/
    wiced_ble_padv_sync_established_event_data_t sync_establish;
    /** Data for WICED_BLE_PERIODIC_ADV_REPORT_EVENT*/
    wiced_ble_padv_report_event_data_t periodic_adv_report;
    /** Data for WICED_BLE_PERIODIC_ADV_SYNC_LOST_EVENT*/
    wiced_ble_padv_sync_handle_t sync_handle;
    /** Data for WICED_BT_BLE_ADV_SET_TERMINATED_EVENT*/
    wiced_ble_ext_adv_set_terminated_event_data_t adv_set_terminated;
    /** Data for WICED_BT_BLE_SCAN_REQUEST_RECEIVED_EVENT*/
    wiced_ble_ext_scan_req_received_event_data_t scan_req_received;
    /** Data for WICED_BT_BLE_BIGINFO_ADV_REPORT_EVENT*/
    wiced_ble_biginfo_adv_report_t biginfo_adv_report;
    /** Data for WICED_BLE_PERIODIC_ADV_SYNC_TRANSFER_EVENT */
    wiced_ble_padv_sync_transfer_event_data_t sync_transfer;
    /** Data for WICED_BLE_EXT_COMMAND_CMPLT_EVENT*/
    wiced_ble_cmd_cmplt_event_data_t cmd_cmplt;

    /* @cond PAWR_API APIs for Periodic Advertising with Response*/

    /** Data for WICED_BT_BLE_PAWR_SUBEVENT_DATA_REQ_EVENT*/
    wiced_ble_padv_subevent_data_req_event_data_t pawr_data_req;
    /** Data for WICED_BT_BLE_PAWR_RSP_REPORT_EVENT*/
    wiced_ble_padv_rsp_report_event_data_t pawr_rsp_report;
    /* @endcond */
} wiced_ble_ext_adv_event_data_t;

/** ADV extension events to the application */
typedef enum
{
    /** Advertising set terminated becaue either connection being created or adv timeout.
     * \ref wiced_ble_ext_adv_set_terminated_event_data_t */
    WICED_BT_BLE_ADV_SET_TERMINATED_EVENT,
    /** scan request received event.  \ref wiced_ble_ext_scan_req_received_event_data_t */
    WICED_BT_BLE_SCAN_REQUEST_RECEIVED_EVENT,
    /** BIGInfo adv report event.  \ref wiced_ble_biginfo_adv_report_t */
    WICED_BT_BLE_BIGINFO_ADV_REPORT_EVENT,
    /** command complete Event.  \ref wiced_ble_cmd_cmplt_event_data_t */
    WICED_BLE_EXT_COMMAND_CMPLT_EVENT,

    /** Sync established to periodic advertiser's periodic advertisement.
     * \ref wiced_ble_padv_sync_established_event_data_t */
    WICED_BLE_PERIODIC_ADV_SYNC_ESTABLISHED_EVENT,
    /** Periodic adv report.  \ref wiced_ble_padv_report_event_data_t */
    WICED_BLE_PERIODIC_ADV_REPORT_EVENT,
    /** Periodic sync lost event.  \ref wiced_ble_padv_sync_handle_t */
    WICED_BLE_PERIODIC_ADV_SYNC_LOST_EVENT,
    /** Periodic Adv Sync Transfer Event.  \ref wiced_ble_padv_sync_transfer_event_data_t */
    WICED_BLE_PERIODIC_ADV_SYNC_TRANSFER_EVENT,

    /* @cond PAWR_API APIs for Periodic Advertising with Response*/
    /** PAWR event on advertiser to request for subevent data
     * \ref wiced_ble_padv_subevent_data_req_event_data_t */
    WICED_BT_BLE_PAWR_SUBEVENT_DATA_REQ_EVENT,
    /** PAWR event on advertiser to report the subevent response data
     * \ref wiced_ble_padv_rsp_report_event_data_t */
    WICED_BT_BLE_PAWR_RSP_REPORT_EVENT,
    /* @endcond */
} wiced_ble_ext_adv_event_t;

#ifdef __cplusplus
extern "C"
{
#endif


/**
 * Check if the local Bluetooth controller supports extended advertising
 * @return wiced_bool_t
 *
 */
    wiced_bool_t wiced_ble_ext_adv_support(void);

    /**
 * Sends the HCI command to set the parameters for extended advetisement
 *
 * @param[in]  adv_handle: Advertisement set handle
 * @param[in]  p_params:   Extended adv parameters
 *
 * @return          wiced_bt_dev_status_t
 *
 * <b> WICED_BT_SUCCESS </b>       : If all extended adv params are set successfully\n
 * <b> WICED_BT_ILLEGAL_VALUE </b> : If paramer is wrong \n
 * <b> WICED_BT_UNSUPPORTED </b>   : If command not supported \n
 * <b> WICED_BT_NO_RESOURCES </b>  : If no memory to issue the command \n
 * <b> WICED_BT_PENDING </b>       : If command queued to send down \n
 */
    wiced_bt_dev_status_t wiced_ble_ext_adv_set_params(wiced_ble_ext_adv_handle_t adv_handle,
                                                       const wiced_ble_ext_adv_params_t *p_params);

    /**
 * Sends HCI command to set the random address for an adv set
 *
 * @param[in] adv_handle  - handle of the advertising set
 * @param[in] random_addr - random address to use for this set
 *
 * @return          wiced_bt_dev_status_t
 *
 * <b> WICED_BT_SUCCESS </b>       : If random addr is set successfully\n
 * <b> WICED_BT_ILLEGAL_VALUE </b> : If paramer is wrong \n
 * <b> WICED_BT_UNSUPPORTED </b>   : If command not supported \n
 * <b> WICED_BT_NO_RESOURCES </b>  : If no memory to issue the command \n
 * <b> WICED_BT_PENDING </b>       : If command queued to send down \n
 *
 */
    wiced_bt_dev_status_t wiced_ble_ext_adv_set_random_address(wiced_ble_ext_adv_handle_t adv_handle,
                                                               wiced_bt_device_address_t random_addr);


    /**
 * Sends HCI command to write the extended adv data
 * @note This API allows sending data formatted with \ref wiced_ble_adv_data_build.
 * @note  This API cannot be used for advertising sets with event_properties that
 * do not support advertising with data viz., #WICED_BLE_EXT_ADV_EVENT_PROPERTY_DIRECTED_ADV,
 *  #WICED_BLE_EXT_ADV_EVENT_PROPERTY_HIGH_DUTY_DIRECTED_CONNECTABLE_ADV
 *
 * @param[in] adv_handle  - handle of the advertising set
 * @param[in] data_len    - length of the adv data to use for this set
 * @param[in] p_data      - pointer to the adv data to use for this set
 *
 * @return          wiced_bt_dev_status_t
 * <b> WICED_BT_SUCCESS </b>       : If all extended adv data set successfully\n
 * <b> WICED_BT_ILLEGAL_VALUE </b> : If paramer is wrong \n
 * <b> WICED_BT_UNSUPPORTED </b>   : If command not supported \n
 * <b> WICED_BT_NO_RESOURCES </b>  : If no memory to issue the command \n
 * <b> WICED_BT_PENDING </b>       : If command queued to send down \n
 */
    wiced_bt_dev_status_t wiced_ble_ext_adv_set_adv_data(wiced_ble_ext_adv_handle_t adv_handle,
                                                         uint16_t data_len,
                                                         uint8_t *p_data);

    /**
 * Sends HCI command to write the extended scan rsp data
 *
 * @param[in] adv_handle  - handle of the advertising set
 * @param[in] data_len    - length of the scan response data to use for this set
 * @param[in] p_data      - pointer to the scan response data to use for this set
 *
 * @return          wiced_bt_dev_status_t
 *
 * <b> WICED_BT_SUCCESS </b>       : If all extended scan response data set successfully\n
 * <b> WICED_BT_ILLEGAL_VALUE </b> : If paramer is wrong \n
 * <b> WICED_BT_UNSUPPORTED </b>   : If command not supported \n
 * <b> WICED_BT_NO_RESOURCES </b>  : If no memory to issue the command \n
 * <b> WICED_BT_PENDING </b>       : If command queued to send down \n
 *
 */
    wiced_bt_dev_status_t wiced_ble_ext_adv_set_scan_rsp_data(wiced_ble_ext_adv_handle_t adv_handle,
                                                              uint16_t data_len,
                                                              uint8_t *p_data);

/**
 * Sends the HCI command to enable/disable extended advertisements
 *
 * @param[in]       enable    - true to enable, false to disable
 * @param[in]       num_sets  - number of sets to enable, unused if disabling
 * @param[in]       p_dur     - pointer to adv handle(s) and duration configuration
 *
 * @return          wiced_bt_dev_status_t
 *
 * <b> WICED_BT_ILLEGAL_VALUE </b> : If paramer is wrong \n
 * <b> WICED_BT_UNSUPPORTED </b>   : If command not supported \n
 * <b> WICED_BT_NO_RESOURCES </b>  : If no memory to issue the command \n
 * <b> WICED_BT_SUCCESS </b>       : If successful\n
 *
 */
    wiced_bt_dev_status_t wiced_ble_ext_adv_enable(uint8_t enable,
                                                   uint8_t num_sets,
                                                   wiced_ble_ext_adv_duration_config_t *p_dur);

    /**
 * Sends the HCI command to remove an extended advertisement set (which is currently not advertising)
 *
 * @param[in]       adv_handle    - handle to advertisement set
 *
 * @return          wiced_bt_dev_status_t
 *
 * <b> WICED_BT_ILLEGAL_VALUE </b> : If paramer is wrong \n
 * <b> WICED_BT_UNSUPPORTED </b>   : If command not supported \n
 * <b> WICED_BT_NO_RESOURCES </b>  : If no memory to issue the command \n
 * <b> WICED_BT_SUCCESS </b>       : If successful\n
 *
 */
    wiced_bt_dev_status_t wiced_ble_ext_adv_remove_adv_set(wiced_ble_ext_adv_handle_t adv_handle);

    /**
 * Sends the HCI command to remove all extended advertisement sets which are currently not advertising
 *
 * @return          wiced_bt_dev_status_t
 *
 * <b> WICED_BT_UNSUPPORTED </b>   : If command not supported \n
 * <b> WICED_BT_NO_RESOURCES </b>  : If no memory to issue the command \n
 * <b> WICED_BT_SUCCESS </b>       : If successfuly\n
 *
 */
    wiced_bt_dev_status_t wiced_ble_ext_adv_clear_adv_sets(void);

    /**
 * Register the extended scan callback to receive the advertisement reports.
 *
 * @param[in] p_ext_scan_cback  - pointer to scan result callback
 *
 * @return          wiced_bt_dev_status_t
 *
 * <b> WICED_BT_UNSUPPORTED </b>   : If command not supported \n
 * <b> WICED_BT_NO_RESOURCES </b>  : If no memory to issue the command \n
 * <b> WICED_BT_SUCCESS </b>       : If successful\n
 *
 */
    wiced_bt_dev_status_t wiced_ble_ext_scan_register_cb(wiced_ble_ext_scan_result_cback_t *p_ext_scan_cback);

        /**
 * Application may enable reassembly of incoming partial advertisement data segments with this API.
 * The stack will reassemble the incoming chained ADV segments upto \p max_ext_adv_len.
 * @note Inorder to use the reassembly feature for extended adv reports, the application is required to create a default
 * heap using \ref wiced_bt_create_heap. The heap should be sized greater than
 * \p max_ext_adv_len * \p max_ext_scan_partial_pkt_q_count. BTSTACK manages allocation and free of the reassembled buffers
 * from this heap.
 *
 * @param[in] max_ext_adv_len - max adv length expected.
                                The stack code will attempt to reassemble adv data reports upto this length
 * @param[in] max_ext_scan_partial_pkt_q_count - max queue size to store the partial packets of the advertisement chain.
 *
 * @return          wiced_bt_dev_status_t
 *
 * <b> WICED_BT_UNSUPPORTED </b>   : If command not supported \n
 * <b> WICED_BT_NO_RESOURCES </b>  : If no memory to issue the command \n
 * <b> WICED_BT_SUCCESS </b>       : If successful\n
 *
 */
    wiced_bt_dev_status_t wiced_ble_ext_scan_configure_reassembly(uint16_t max_ext_adv_len,
                                                                  uint16_t max_ext_scan_partial_pkt_q_count);

    /**
 * Set extended scanning parameters
 *
 * @param[in] p_scp:  pointer to scan configuration
 *
 * @return          wiced_bt_dev_status_t

 * <b> WICED_BT_ILLEGAL_VALUE </b> : If paramer is wrong \n
 * <b> WICED_BT_UNSUPPORTED </b>   : If command not supported \n
 * <b> WICED_BT_NO_RESOURCES </b>  : If no memory to issue the command \n
 * <b> WICED_BT_SUCCESS </b>       : If successful\n
 *
 */
    wiced_bt_dev_status_t wiced_ble_ext_scan_set_params(const wiced_ble_ext_scan_params_t *p_scp);

    /**
 * Start extended scanning
 *
 * @param[in] enable:  Set 1 to start/enable scanning, 0 to stop/disable
 * @param[in] p_sce:  Scanning enable parameters
 *
 * @return          wiced_bt_dev_status_t

 * <b> WICED_BT_ILLEGAL_VALUE </b> : If paramer is wrong \n
 * <b> WICED_BT_UNSUPPORTED </b>   : If command not supported \n
 * <b> WICED_BT_NO_RESOURCES </b>  : If no memory to issue the command \n
 * <b> WICED_BT_SUCCESS </b>       : If successful\n
 *
 */
    wiced_bt_dev_status_t wiced_ble_ext_scan_enable(wiced_bool_t enable, const wiced_ble_ext_scan_enable_params_t *p_sce);

    /**
 * Creates a connection using extended HCI commands
 *
 * @param[in]       p_ext_conn_cfg    - pointer to connection configuration
 *
 * @note The adv_handle and sub_event parameters of connection configuration shall be set to 0xFF if
 *        p_ext_conn_cfg->adv_handle and p_ext_conn_cfg->sub_event parameters are not to be used
 *
 * @return          wiced_bt_dev_status_t
 *
 * <b> WICED_BT_UNSUPPORTED </b>   : If command not supported \n
 * <b> WICED_BT_NO_RESOURCES </b>  : If no memory to issue the command \n
 * <b> WICED_BT_SUCCESS </b>       : If successful\n
 *
 */
    wiced_bt_dev_status_t wiced_ble_ext_create_connection(wiced_ble_ext_conn_cfg_t *p_ext_conn_cfg);

   /**
* Cancel a ble connection
*
* @return          wiced_bt_dev_status_t
*
* <b> WICED_BT_UNSUPPORTED </b>   : If command not supported \n
* <b> WICED_BT_NO_RESOURCES </b>  : If no memory to issue the command \n
* <b> WICED_BT_SUCCESS </b>       : If successful\n
*
*/
   wiced_bt_dev_status_t wiced_ble_cancel_connection (void);

    /**
 * Callback wiced_ble_ext_adv_event_cback_t
 *
 * Adv extension command status, command complete event and LE adv extension meta event callback
 *
 * @param event  : Event type (see wiced_ble_periodic_event_t)
 * @param p_data : Event data (see wiced_ble_ext_adv_event_data_t)
 *
 * @return void
 */
    typedef void (*wiced_ble_ext_adv_event_cback_t)(wiced_ble_ext_adv_event_t event,
                                                    wiced_ble_ext_adv_event_data_t *p_data);

    /**
 * Register an application callback function to receive extended, periodic advertising events and PAWR command failed status event.
 *
 * @param[in]       p_app_adv_ext_event_cb      - pointer to function to receive extended adv events.
 *
 * @return          wiced_bt_dev_status_t
 * <b> WICED_BT_UNSUPPORTED </b>   : If command not supported \n
 * <b> WICED_BT_NO_RESOURCES </b>  : If no memory to issue the command \n
 * <b> WICED_BT_PENDING </b>       : If command queued to send down \n
 * <b> WICED_BT_SUCCESS </b>       : If successful\n
 *
 */
    void wiced_ble_ext_adv_register_cback(wiced_ble_ext_adv_event_cback_t p_app_adv_ext_event_cb);

#ifdef __cplusplus
} // extern "C"
#endif

/**@} wicedbt */

#endif // WICED_BLE_ENABLE_EXTENDED_ADV_API
#endif // __WICED_BT_ADV_SCAN_EXTENDED_H__
