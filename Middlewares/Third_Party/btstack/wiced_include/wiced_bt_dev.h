/*
 * Copyright 2016-2025, Cypress Semiconductor Corporation or
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
 * Bluetooth Management (BTM) Application Programming Interface
 *
 * The BTM consists of several management entities:
 *      1. Device Control - controls the local device
 *      2. Device Discovery - manages inquiries, discover database
 *      3. ACL Channels - manages ACL connections (BR/EDR and LE)
 *      4. SCO Channels - manages SCO connections
 *      5. Security - manages all security functionality
 *      6. Power Management - manages park, sniff, hold, etc.
 *
 */

#ifndef  __WICED_BT_DEV_H__
#define  __WICED_BT_DEV_H__

#include "wiced_bt_types.h"
#include "hcidefs.h"
#include "wiced_bt_sco.h"
#include "wiced_bt_dev_utils.h"

/**
 * This section consists of several management entities:
 *      - Device Control - controls the local device
 *      - Device Discovery - manages inquiries, discover database
 * @if DUAL_MODE
 *      - ACL Channels - manages ACL connections (BR/EDR and LE)
 *      - SCO Channels - manages SCO connections
 *      - Power Management - manages park, sniff, hold, etc.
 * @else
 *      - ACL Channels - manages LE ACL connections
 *      - Security - manages all security functionality
 * @endif
 *
 * @addtogroup  wicedbt_DeviceManagement    Device Management
 *
 * @ingroup     wicedbt
 *
 * @{
 */
/****************************************************************************/

/** RSSI value not supplied (ignore it) */
#define BTM_INQ_RES_IGNORE_RSSI     0x7f
/** Passed to BTM_SetScanConfig() to ignore */
#define BTM_SCAN_PARAM_IGNORE       0


/** @cond DUAL_MODE */

/** Structure for local address extendend API
 @note #wiced_bt_dev_read_local_addr_ext API function sets private_addr_type and private_addr only if LE privacy is set to true */
typedef struct
{
    wiced_bool_t                    is_static_rand_addr_used;   /**< True if static random address is used */
    wiced_bool_t                    is_privacy_enabled;         /**< True LE Privacy is enabled */
    wiced_bt_ble_address_type_t     private_addr_type;          /**< Private address type*/
    wiced_bt_device_address_t       private_addr;               /**< Private address */
    wiced_bt_device_address_t       local_addr;                 /**< Local Bluetooth Address */
} wiced_bt_dev_local_addr_ext_t;

/** Default Discovery Window (in 0.625 msec intervals) */
#define BTM_DEFAULT_DISC_WINDOW     0x0012
/** Default Discovery Interval (in 0.625 msec intervals) */
#define BTM_DEFAULT_DISC_INTERVAL   0x0800

/** Default Connection Window  */
#define BTM_DEFAULT_CONN_WINDOW     0x0012
/** Default Connection Interval  */
#define BTM_DEFAULT_CONN_INTERVAL   0x0800

/** BR-EDR Discoverable modes */
enum wiced_bt_discoverability_mode_e {
    BTM_NON_DISCOVERABLE            = 0,        /**< Non discoverable */
    BTM_LIMITED_DISCOVERABLE        = 1,        /**< Limited BR/EDR discoverable */
    BTM_GENERAL_DISCOVERABLE        = 2         /**< General BR/EDR discoverable */
};

/** BR/EDR Connectable modes */
enum wiced_bt_connectability_mode_e {
    BTM_NON_CONNECTABLE             = 0,        /**< Not connectable */
    BTM_CONNECTABLE                 = 1         /**< BR/EDR connectable */
};

/** Inquiry modes
  * @note These modes are associated with the inquiry active values */
enum wiced_bt_inquiry_mode_e {
    BTM_INQUIRY_NONE                = 0,        /**< Stop inquiry */
    BTM_GENERAL_INQUIRY             = 0x01,     /**< General inquiry */
    BTM_LIMITED_INQUIRY             = 0x02,     /**< Limited inquiry */
    BTM_BR_INQUIRY_MASK             = (BTM_GENERAL_INQUIRY | BTM_LIMITED_INQUIRY) /**< BR Inquiry Mask */
};

/** Discoverable Mask */
#define BTM_DISCOVERABLE_MASK       (BTM_LIMITED_DISCOVERABLE|BTM_GENERAL_DISCOVERABLE)
/** Max Value for Discoverable */
#define BTM_MAX_DISCOVERABLE        BTM_GENERAL_DISCOVERABLE
/** Connectable Mask */
#define BTM_CONNECTABLE_MASK        (BTM_NON_CONNECTABLE | BTM_CONNECTABLE)

/** Standard Scan Type : Device listens for the duration of the scan window*/
#define BTM_SCAN_TYPE_STANDARD      0
/** Interlaces Scan Type : Device performed two back to back scans */
#define BTM_SCAN_TYPE_INTERLACED    1

/** Inquiry results  */
#define BTM_INQ_RESULT              0
/** Inquiry results with RSSI  */
#define BTM_INQ_RESULT_WITH_RSSI    1
/** Extended Inquiry results */
#define BTM_INQ_RESULT_EXTENDED     2

/** Inquiry Filter Condition types (see wiced_bt_dev_inq_parms_t) */
enum wiced_bt_dev_filter_cond_e {
    BTM_CLR_INQUIRY_FILTER          = 0,                            /**< No inquiry filter */
    BTM_FILTER_COND_DEVICE_CLASS    = HCI_FILTER_COND_DEVICE_CLASS, /**< Filter on device class */
    BTM_FILTER_COND_BD_ADDR         = HCI_FILTER_COND_BD_ADDR,      /**< Filter on device addr */
};

/** BTM service definitions (used for storing EIR data to bit mask refer eir_uuid_mask in #wiced_bt_dev_inquiry_scan_result_t) */
enum
{
    BTM_EIR_UUID_SERVCLASS_SERIAL_PORT,         /**< Serial Port Service Index */
    BTM_EIR_UUID_SERVCLASS_DIALUP_NETWORKING,   /**< Dialup Networking Service Index */
    BTM_EIR_UUID_SERVCLASS_IRMC_SYNC,           /**< IRMC SYNC Service Index */
    BTM_EIR_UUID_SERVCLASS_OBEX_OBJECT_PUSH,    /**< OBEX Object Push Service Index */
    BTM_EIR_UUID_SERVCLASS_OBEX_FILE_TRANSFER,  /**< OBEX File Transfer Service Index */
    BTM_EIR_UUID_SERVCLASS_IRMC_SYNC_COMMAND,   /**< IRMC SYNC Command Service Index */
    BTM_EIR_UUID_SERVCLASS_HEADSET,             /**< Headset Service Index */
    BTM_EIR_UUID_SERVCLASS_AUDIO_SOURCE,        /**< Audio Source Service Index */
    BTM_EIR_UUID_SERVCLASS_AUDIO_SINK,          /**< Audio Sink Service Index */
    BTM_EIR_UUID_SERVCLASS_AV_REM_CTRL_TARGET,  /**< AVRCP TG Service Index */
    BTM_EIR_UUID_SERVCLASS_AV_REMOTE_CONTROL,   /**< AVRCP CT Service Index */
    BTM_EIR_UUID_SERVCLASS_HEADSET_AUDIO_GATEWAY,   /**< Headset AG Service Index */
    BTM_EIR_UUID_SERVCLASS_DIRECT_PRINTING,     /**< Direct Printing Service Index */
    BTM_EIR_UUID_SERVCLASS_HF_HANDSFREE,        /**< HF Handsfree Service Index */
    BTM_EIR_UUID_SERVCLASS_AG_HANDSFREE,        /**< AG Handsfree Service Index */
    BTM_EIR_UUID_SERVCLASS_HUMAN_INTERFACE,     /**< Human Interface Service Index */
    BTM_EIR_UUID_SERVCLASS_SAP,                 /**< SAP Service Index */
    BTM_EIR_UUID_SERVCLASS_PBAP_PCE,            /**< PBAP PCE Service Index */
    BTM_EIR_UUID_SERVCLASS_PBAP_PSE,            /**< PBAP PSE Service Index */
    BTM_EIR_UUID_SERVCLASS_PHONE_ACCESS,        /**< Phone Access Service Index */
    BTM_EIR_UUID_SERVCLASS_HEADSET_HS,          /**< Headset HS Service Index */
    BTM_EIR_UUID_SERVCLASS_PNP_INFORMATION,     /**< PNP Information Service Index */
    BTM_EIR_UUID_SERVCLASS_MESSAGE_ACCESS,      /**< Message Access Service Index */
    BTM_EIR_UUID_SERVCLASS_MESSAGE_NOTIFICATION,    /**< Message Notification Service Index */
    BTM_EIR_UUID_SERVCLASS_HDP_SOURCE,          /**<HDP Source Service Index */
    BTM_EIR_UUID_SERVCLASS_HDP_SINK,            /**< HDP Sink Service Index */
    BTM_EIR_MAX_SERVICES                        /**< Max Service Index */
};

/** HCI role definitions */
#define HCI_ROLE_CENTRAL 0x00 /**< device role central */
#define HCI_ROLE_PERIPHERAL 0x01 /**< device role peripheral */
#define HCI_ROLE_UNKNOWN    0xff  /**< device role unknown */
typedef uint8_t wiced_bt_dev_role_t; /**< device role for the connection */

/***************************
 *  Device Discovery Types
 ****************************/
/** Class of Device inquiry filter */
typedef struct
{
    wiced_bt_dev_class_t            dev_class;          /**< class of device */
    wiced_bt_dev_class_t            dev_class_mask;     /**< class of device filter mask */
} wiced_bt_dev_cod_cond_t;

/** Inquiry filter */
typedef union
{
    wiced_bt_device_address_t       bdaddr_cond;        /**< bluetooth address filter */
    wiced_bt_dev_cod_cond_t         cod_cond;           /**< class of device filter */
} wiced_bt_dev_inq_filt_cond_t;

/** Inquiry Parameters */
typedef struct
{
    uint8_t                         mode;               /**< Inquiry mode (see #wiced_bt_inquiry_mode_e) */
    uint8_t                         duration;           /**< Inquiry duration (1.28 sec increments) */
    uint8_t                         filter_cond_type;   /**< Inquiry filter type  (see #wiced_bt_dev_filter_cond_e) */
    wiced_bt_dev_inq_filt_cond_t    filter_cond;        /**< Inquiry filter */
} wiced_bt_dev_inq_parms_t;

/** Inquiry Results */
typedef struct
{
    uint16_t                        clock_offset;                           /**< Clock offset */
    wiced_bt_device_address_t       remote_bd_addr;                         /**< Device address */
    wiced_bt_dev_class_t            dev_class;                              /**< Class of device */
    uint8_t                         page_scan_rep_mode;                     /**< Page scan repetition mode */
    uint8_t                         page_scan_per_mode;                     /**< Page scan per mode */
    uint8_t                         page_scan_mode;                         /**< Page scan mode */
    int8_t                          rssi;                                   /**< Receive signal strength index (#BTM_INQ_RES_IGNORE_RSSI, if not available) */
    uint32_t                        eir_uuid_mask;          /**< Bit mask of EIR UUIDs */
    wiced_bool_t                    eir_complete_list;                      /**< TRUE if EIR array is complete */
} wiced_bt_dev_inquiry_scan_result_t;

/** RSSI Result  (in response to #wiced_bt_dev_read_rssi)  */
typedef struct
{
    wiced_result_t                  status;             /**< Status of the operation */
    uint8_t                         hci_status;         /**< Status from controller (Refer Spec 5.0 Vol 2 Part D Error Codes) */
    int8_t                          rssi;               /**< RSSI in dB */
    wiced_bt_device_address_t       rem_bda;            /**< Remote BD address */
} wiced_bt_dev_rssi_result_t;

/** Structure returned with remote name request */
typedef struct
{
    uint16_t                        status;             /**< Status of the operation. BTM_SUCCESS for success otherwise BTM_BAD_VALUE_RET*/
    wiced_bt_device_address_t       bd_addr;            /**< Remote BD address */
    uint16_t                        length;             /**< Device name Length */
    wiced_bt_remote_name_t          remote_bd_name;     /**< Remote device name */
}wiced_bt_dev_remote_name_result_t;

/** Structure returned with switch role request */
typedef struct
{
    uint8_t                         status;             /**< Status of the operation. (Refer Spec 5.0 Vol 2 Part D Error Codes) */
    wiced_bt_dev_role_t             role;               /**< HCI_ROLE_CENTRAL or HCI_ROLE_PERIPHERAL */
    wiced_bt_device_address_t       bd_addr;            /**< Remote BD address involved with the switch */
} wiced_bt_dev_switch_role_result_t;

/** Structure returned on setup qos or flow spec complete */
typedef struct
{
    uint8_t status;           /**< status of the operation (Refer Spec 5.0 Vol 2 Part D Error Codes) */
    wiced_bt_device_address_t bd_addr; /**< Remote BD address */
    uint8_t unused;           /**< unused */
    uint8_t service_type;     /**< service type (NO_TRAFFIC, BEST_EFFORT, or GUARANTEED) */
    uint32_t token_rate;      /**< token rate (bytes/second) */
    uint32_t peak_bandwidth;  /**< peak bandwidth (bytes/second)*/
    uint32_t latency;         /**< latency (microseconds) */
    uint32_t delay_variation; /**< delay variation (microseconds) */
} wiced_bt_flow_spec_cmpl_evt_t;


/** @endcond */

/*****************************************************************************
 *  SECURITY MANAGEMENT
 *****************************************************************************/

/** security flags for current BR/EDR link */
enum wiced_bt_sec_flags_e
{
    BTM_SEC_LINK_ENCRYPTED                       = 0x01,                 /**< Link encrypted */
    BTM_SEC_LINK_PAIRED_WITHOUT_MITM             = 0x02,                 /**< Paired without man-in-the-middle protection */
    BTM_SEC_LINK_PAIRED_WITH_MITM                = 0x04                  /**< Link with man-in-the-middle protection */
};

/** Variable Pin Type */
#define BTM_PIN_TYPE_VARIABLE       HCI_PIN_TYPE_VARIABLE
/** Fix Length Pin Type */
#define BTM_PIN_TYPE_FIXED          HCI_PIN_TYPE_FIXED

/** Security key data length (used by wiced_bt_device_link_keys_t structure) */
#ifndef BTM_SECURITY_KEY_DATA_LEN
#define BTM_SECURITY_KEY_DATA_LEN       132
#endif

/** Local security key data length (used by wiced_bt_local_identity_keys_t structure) */
#ifndef BTM_SECURITY_LOCAL_KEY_DATA_LEN
#define BTM_SECURITY_LOCAL_KEY_DATA_LEN 65
#endif

/** Pairing IO Capabilities */
enum wiced_bt_dev_io_cap_e {
    BTM_IO_CAPABILITIES_DISPLAY_ONLY                   = 0,  /**< Display Only        */
    BTM_IO_CAPABILITIES_DISPLAY_AND_YES_NO_INPUT       = 1,  /**< Display Yes/No      */
    BTM_IO_CAPABILITIES_KEYBOARD_ONLY                  = 2,  /**< Keyboard Only       */
    BTM_IO_CAPABILITIES_NONE                           = 3,  /**< No Input, No Output */
    BTM_IO_CAPABILITIES_BLE_DISPLAY_AND_KEYBOARD_INPUT = 4,  /**< Keyboard display (For LE SMP) */
    BTM_IO_CAPABILITIES_MAX                            = 5,  /**< Max value for IO capability */
};

typedef uint8_t wiced_bt_dev_io_cap_t;          /**< IO capabilities */

/** BR/EDR Authentication requirement */
enum wiced_bt_dev_auth_req_e {
    BTM_AUTH_SINGLE_PROFILE_NO = 0,                     /**< MITM Protection Not Required - Single Profile/non-bonding. Numeric comparison with automatic accept allowed */
    BTM_AUTH_SINGLE_PROFILE_YES = 1,                    /**< MITM Protection Required - Single Profile/non-bonding. Use IO Capabilities to determine authentication procedure */
    BTM_AUTH_ALL_PROFILES_NO = 2,                       /**< MITM Protection Not Required - All Profiles/dedicated bonding. Numeric comparison with automatic accept allowed */
    BTM_AUTH_ALL_PROFILES_YES = 3,                      /**< MITM Protection Required - All Profiles/dedicated bonding. Use IO Capabilities to determine authentication procedure */
    BTM_AUTH_SINGLE_PROFILE_GENERAL_BONDING_NO = 4,     /**< MITM Protection Not Required - Single Profiles/general bonding. Numeric comparison with automatic accept allowed */
    BTM_AUTH_SINGLE_PROFILE_GENERAL_BONDING_YES = 5,    /**< MITM Protection Required - Single Profiles/general bonding. Use IO Capabilities to determine authentication procedure */
};
typedef uint8_t wiced_bt_dev_auth_req_t;                /**< BR/EDR authentication requirement (see #wiced_bt_dev_auth_req_e) */

/** LE Authentication requirement */
enum wiced_bt_dev_le_auth_req_e {
    BTM_LE_AUTH_REQ_NO_BOND =       0x00,                                               /**< Not required - No Bond */
    BTM_LE_AUTH_REQ_BOND =          0x01,                                               /**< Required - General Bond */
    BTM_LE_AUTH_REQ_MITM =          0x04,                                               /**< MITM required - Auth Y/N */
    BTM_LE_AUTH_REQ_SC =            0x08,                                               /**< LE Secure Connection or legacy, no MITM, no Bonding */
    BTM_LE_AUTH_REQ_KP =            0x10,                                               /**< Keypress supported Y/N */
    BTM_LE_AUTH_REQ_H7 =            0x20,                                               /**< Key derivation function H7 supported Y/N */
    BTM_LE_AUTH_REQ_SC_BOND =       (BTM_LE_AUTH_REQ_SC|BTM_LE_AUTH_REQ_BOND),          /**< LE Secure Connection or legacy, no MITM, Bonding */
    BTM_LE_AUTH_REQ_SC_MITM =       (BTM_LE_AUTH_REQ_SC|BTM_LE_AUTH_REQ_MITM),          /**< LE Secure Connection or legacy, MITM, no Bonding */
    BTM_LE_AUTH_REQ_SC_MITM_BOND =  (BTM_LE_AUTH_REQ_SC|BTM_LE_AUTH_REQ_MITM|BTM_LE_AUTH_REQ_BOND),    /**< LE Secure Connection or legacy , MITM, Bonding */
    BTM_LE_AUTH_REQ_MASK =          0x3D                                                /**< Auth Request Mask */
};
typedef uint8_t wiced_bt_dev_le_auth_req_t;             /**< LE authentication requirement (see #wiced_bt_dev_le_auth_req_e) */

/** LE Security key level */
#define SMP_SEC_NONE                 0            /**< Security Key Level: None */
#define SMP_SEC_UNAUTHENTICATE      (1 << 0)      /**< Security Key Level: key not authenticated */
#define SMP_SEC_AUTHENTICATED       (1 << 2)      /**< Security Key Level: key authenticated */
typedef uint8_t wiced_bt_smp_sec_level_t;    /**< LE Security key level */

/** Public key */
typedef struct
{
    BT_OCTET32  x;              /**< X cordinate value */
    BT_OCTET32  y;              /**< Y cordinate value */
} wiced_bt_public_key_t;

 /** OOB Data status */
#ifndef BTM_OOB_STATE
#define BTM_OOB_STATE
/** OOB Data status */
enum wiced_bt_dev_oob_data_e
{
    BTM_OOB_NONE,                                       /**< No OOB data */
    BTM_OOB_PRESENT_192,                                /**< OOB data present (from the P-192 public key) */
    BTM_OOB_PRESENT_256,                                /**< OOB data present (from the P-256 public key) */
    BTM_OOB_PRESENT_192_256,                            /**< OOB data present (from the P-192 and P-256 public keys) */
    BTM_OOB_UNKNOWN                                     /**< OOB data unknown */
};
#endif
typedef uint8_t wiced_bt_dev_oob_data_t;                /**< OOB data (see #wiced_bt_dev_oob_data_e) */

/** Data for #BTM_SMP_SC_LOCAL_OOB_DATA_NOTIFICATION_EVT */
typedef struct
{
    wiced_bool_t                present;                /**< TRUE if local oob is present */
    BT_OCTET16                  randomizer;             /**< randomizer */
    BT_OCTET16                  commitment;             /**< commitment */

    wiced_bt_ble_address_t      addr_sent_to;           /**< peer address sent to */
    BT_OCTET32                  private_key_used;       /**< private key */
    wiced_bt_public_key_t       public_key_used;        /**< public key */
} wiced_bt_smp_sc_local_oob_t;

/** @cond DUAL_MODE */

/** Data type for IO capabalities response (#BTM_PAIRING_IO_CAPABILITIES_BR_EDR_RESPONSE_EVT) */
typedef struct
{
    wiced_bt_device_address_t   bd_addr;                /**< Peer address */
    wiced_bt_dev_io_cap_t       io_cap;                 /**< Peer IO capabilities, #wiced_bt_dev_io_cap_t */
    wiced_bt_dev_oob_data_t     oob_data;               /**< OOB data present at peer device for the local device */
    wiced_bt_dev_auth_req_t     auth_req;               /**< Authentication required for peer device */
} wiced_bt_dev_bredr_io_caps_rsp_t;

/** Pairing user passkey request  (#BTM_PASSKEY_REQUEST_EVT event data type) */
typedef struct
{
    wiced_bt_device_address_t   bd_addr;            /**< peer address       */
} wiced_bt_dev_user_key_req_t;

/** Pairing keypress types */
enum wiced_bt_dev_passkey_entry_type_e
{
    BTM_PASSKEY_ENTRY_STARTED,          /**< passkey entry started */
    BTM_PASSKEY_DIGIT_ENTERED,          /**< passkey digit entered */
    BTM_PASSKEY_DIGIT_ERASED,           /**< passkey digit erased */
    BTM_PASSKEY_DIGIT_CLEARED,          /**< passkey cleared */
    BTM_PASSKEY_ENTRY_COMPLETED         /**< passkey entry completed */
};
typedef uint8_t wiced_bt_dev_passkey_entry_type_t;  /**< Bluetooth pairing keypress value (see #wiced_bt_dev_passkey_entry_type_e)  */

/** Data associated with the information received from the peer via OOB interface */
typedef struct
{
    wiced_bool_t                present;                /**< TRUE if local oob is present */
    BT_OCTET16                  randomizer;             /**< randomizer */
    BT_OCTET16                  commitment;             /**< commitment */
    wiced_bt_ble_address_t      addr_received_from;     /**< peer address */
} wiced_bt_smp_sc_peer_oob_data_t;

/** Data for #wiced_bt_smp_sc_oob_reply */
typedef struct
{
    wiced_bt_smp_sc_local_oob_t     local_oob_data;     /**< My OOB  sent to peer out of band */
    wiced_bt_smp_sc_peer_oob_data_t peer_oob_data;      /**< Peer OOB received out of band */
}wiced_bt_smp_sc_oob_data_t;

/** SCO link type */
#define BTM_LINK_TYPE_SCO           HCI_LINK_TYPE_SCO       /**< Link type SCO */
#define BTM_LINK_TYPE_ESCO          HCI_LINK_TYPE_ESCO      /**< Link type eSCO */
/** SCO link type */
typedef uint8_t wiced_bt_sco_type_t;

/** SCO connected event related data */
typedef struct {
    uint16_t    sco_index;                  /**< SCO index */
} wiced_bt_sco_connected_t;

/** SCO disconnected event related data */
typedef struct {
    uint16_t    sco_index;                  /**< SCO index */
} wiced_bt_sco_disconnected_t;

/**  SCO connect request event related data */
typedef struct {
    uint16_t                    sco_index;      /**< SCO index */
    wiced_bt_device_address_t   bd_addr;        /**< Peer bd address */
    wiced_bt_dev_class_t        dev_class;      /**< Peer device class */
    wiced_bt_sco_type_t         link_type;      /**< SCO link type */
} wiced_bt_sco_connection_request_t;

/** SCO connection change event related data */
typedef struct {
    uint16_t                    sco_index;          /**< SCO index */
    uint16_t                    rx_pkt_len;         /**< RX packet length */
    uint16_t                    tx_pkt_len;         /**< TX packet length */
    wiced_bt_device_address_t   bd_addr;            /**< Peer bd address */
    uint8_t                     hci_status;         /**< HCI status */
    uint8_t                     tx_interval;        /**< TX interval */
    uint8_t                     retrans_windows;    /**< Retransmission windows */
} wiced_bt_sco_connection_change_t;

/** @endcond */

/** Type of OOB data required  */
#ifndef BTM_OOB_REQ_TYPE
#define BTM_OOB_REQ_TYPE
/** Type of OOB data required  */
enum wiced_bt_dev_oob_data_req_type_e
{
    BTM_OOB_INVALID_TYPE,                               /**< Invalid OOB Type */
    BTM_OOB_PEER,                                       /**< Peer OOB data requested */
    BTM_OOB_LOCAL,                                      /**< Local OOB data requested */
    BTM_OOB_BOTH                                        /**< Both local and peer OOB data requested */
};
#endif
typedef uint8_t wiced_bt_dev_oob_data_req_type_t;         /**< OOB data type requested (see #wiced_bt_dev_oob_data_req_type_t) */


/** data type for #BTM_SMP_REMOTE_OOB_DATA_REQUEST_EVT */
typedef struct
{
    wiced_bt_device_address_t           bd_addr;        /**< peer address */
} wiced_bt_smp_remote_oob_req_t;

/** data type for #BTM_SMP_SC_REMOTE_OOB_DATA_REQUEST_EVT */
typedef struct
{
    wiced_bt_device_address_t           bd_addr;        /**< peer address */
    wiced_bt_dev_oob_data_req_type_t    oob_type;       /**< requested oob data types (#BTM_OOB_PEER, #BTM_OOB_LOCAL, or #BTM_OOB_BOTH) */
} wiced_bt_smp_sc_remote_oob_req_t;

/** Pairing keypress notification (#BTM_KEYPRESS_NOTIFICATION_EVT event data type) */
typedef struct
{
    wiced_bt_device_address_t           bd_addr;        /**< peer address       */
    wiced_bt_dev_passkey_entry_type_t   keypress_type;  /**< type of keypress   */
} wiced_bt_dev_user_keypress_t;

/** Data for pairing passkey notification (#BTM_PASSKEY_NOTIFICATION_EVT event data type) */
typedef struct
{
    wiced_bt_device_address_t   bd_addr;            /**< peer address       */
    uint32_t                    passkey;            /**< passkey            */
} wiced_bt_dev_user_key_notif_t;


/** BR/EDR pairing complete information */
typedef struct
{
    uint8_t         status;                 /**< status of the simple pairing process (see defintions for HCI status codes) */
} wiced_bt_dev_br_edr_pairing_info_t;

/** Data for pairing confirmation request (#BTM_USER_CONFIRMATION_REQUEST_EVT event data type) */
typedef struct
{
    wiced_bt_device_address_t   bd_addr;                            /**< peer address */
    uint32_t                    numeric_value;                      /**< numeric value for comparison (if "just_works", do not show this number to UI) */
    wiced_bool_t                just_works;                         /**< TRUE, if using "just works" association model */
    wiced_bt_dev_auth_req_t     local_authentication_requirements;  /**< Authentication requirement for local device */
    wiced_bt_dev_auth_req_t     remote_authentication_requirements; /**< Authentication requirement for peer device */
} wiced_bt_dev_user_cfm_req_t;

/** LE pairing complete information */
typedef struct
{
    wiced_result_t                    status;                 /**< status of the simple pairing process   */
    uint8_t                           reason;                 /**< failure reason (see #wiced_bt_smp_status_t) */
    wiced_bt_smp_sec_level_t          sec_level;              /**< 0 - None, 1- Unauthenticated Key, 4-Authenticated Key  */
    wiced_bool_t                      is_pair_cancel;         /**< True if cancelled, else False   */
    wiced_bt_device_address_t         resolved_bd_addr;       /**< Resolved address (if remote device using private address) */
    wiced_bt_ble_address_type_t       resolved_bd_addr_type;  /**< Resolved addr type of bonded device */
} wiced_bt_dev_ble_pairing_info_t;


/** Transport dependent pairing complete information */
typedef union
{
    wiced_bt_dev_br_edr_pairing_info_t  br_edr;         /**< BR/EDR pairing complete information */
    wiced_bt_dev_ble_pairing_info_t     ble;            /**< LE pairing complete information */
} wiced_bt_dev_pairing_info_t;

/** Pairing complete notification (#BTM_PAIRING_COMPLETE_EVT event data type) */
typedef struct
{
    wiced_bt_device_address_ptr_t bd_addr;               /**< peer address           */
    wiced_bt_transport_t          transport;              /**< BT_TRANSPORT_BR_EDR or BT_TRANSPORT_LE */
    wiced_bt_dev_pairing_info_t   pairing_complete_info;  /**< Transport dependent pairing complete information */
} wiced_bt_dev_pairing_cplt_t;

/** Security/authentication failure status  (used by #BTM_SECURITY_FAILED_EVT notication) */
typedef struct
{
    wiced_bt_device_address_t   bd_addr;                /**< [in]  Peer address */
    wiced_result_t              status;                 /**< Status of the operation */
    uint8_t                     hci_status;             /**< Status from controller */
} wiced_bt_dev_security_failed_t;

/** Security request (#BTM_SECURITY_REQUEST_EVT event data type) */
typedef struct
{
    wiced_bt_device_address_t      bd_addr;             /**< peer address           */
} wiced_bt_dev_security_request_t;

/** SMP key distribution mask */
enum wiced_bt_dev_le_key_type_e
{
    SMP_SEC_KEY_TYPE_ENC = (1 << 0),  /**< encryption key */
    SMP_SEC_KEY_TYPE_ID = (1 << 1),   /**< identity key */
    SMP_SEC_KEY_TYPE_CSRK = (1 << 2), /**< Peripheral CSRK */
    SMP_SEC_KEY_TYPE_LK = (1 << 3),   /**< BR/EDR link key */
};
typedef uint8_t wiced_bt_dev_le_key_type_t; /**< SMP key distribution mask, see #wiced_bt_dev_le_key_type_e*/

#define BTM_LE_KEY_PENC  SMP_SEC_KEY_TYPE_ENC  /**< @deprecated, encryption information of peer device */
#define BTM_LE_KEY_PID  SMP_SEC_KEY_TYPE_ID    /**< @deprecated identity key of the peer device */
#define BTM_LE_KEY_PCSRK SMP_SEC_KEY_TYPE_CSRK /**< @deprecated peer SRK */
#define BTM_LE_KEY_PLK SMP_SEC_KEY_TYPE_LK     /**< @deprecated peer link key */
#define BTM_LE_KEY_LENC SMP_SEC_KEY_TYPE_LK    /**< @deprecated peer link key */

/** Scan duty cycle (used for #BTM_BLE_SCAN_STATE_CHANGED_EVT ) */
#ifndef BTM_BLE_SCAN_TYPE
#define BTM_BLE_SCAN_TYPE
/** Scan duty cycle (used for #BTM_BLE_SCAN_STATE_CHANGED_EVT ) */
enum wiced_bt_ble_scan_type_e
{
    BTM_BLE_SCAN_TYPE_NONE,                 /**< Stop scanning */
    BTM_BLE_SCAN_TYPE_HIGH_DUTY,            /**< General inquiry high duty cycle scan */
    BTM_BLE_SCAN_TYPE_LOW_DUTY,             /**< General inquiry low duty cycle scan */
    BTM_BLE_SCAN_TYPE_LIMITED_HIGH_DUTY,    /**< Limited inquiry high duty cycle scan */
    BTM_BLE_SCAN_TYPE_LIMITED_LOW_DUTY      /**< Limited inquiry low duty cycle scan */
};
#endif
typedef uint8_t wiced_bt_ble_scan_type_t;   /**< scan type (see #wiced_bt_ble_scan_type_e) */


/** SMP Pairing status codes */
enum wiced_bt_smp_status_e
{
    SMP_SUCCESS                 = 0,    /**< Success */
    SMP_PASSKEY_ENTRY_FAIL      = 0x01, /**< Passkey entry failed */
    SMP_OOB_FAIL                = 0x02, /**< OOB failed */
    SMP_PAIR_AUTH_FAIL          = 0x03, /**< Authentication failed */
    SMP_CONFIRM_VALUE_ERR       = 0x04, /**< Value confirmation failed */
    SMP_PAIR_NOT_SUPPORT        = 0x05, /**< Not supported */
    SMP_ENC_KEY_SIZE            = 0x06, /**< Encryption key size failure */
    SMP_INVALID_CMD             = 0x07, /**< Invalid command */
    SMP_PAIR_FAIL_UNKNOWN       = 0x08, /**< Unknown failure */
    SMP_REPEATED_ATTEMPTS       = 0x09, /**< Repeated attempts */
    SMP_INVALID_PARAMETERS      = 0x0A, /**< Invalid parameters  */
    SMP_DHKEY_CHK_FAIL          = 0x0B, /**< DH Key check failed */
    SMP_NUMERIC_COMPAR_FAIL     = 0x0C, /**< Numeric comparison failed */
    SMP_BR_PAIRING_IN_PROGR     = 0x0D, /**< BR paIring in progress */
    SMP_XTRANS_DERIVE_NOT_ALLOW = 0x0E, /**< Cross transport key derivation not allowed */
    SMP_ERR_CODE_KEY_REJECTED   = 0x0F, /**< Device chose not to accept a distributed key*/
    SMP_ERR_CODE_BUSY           = 0x10, /**< Device is not ready to perform a pairing procedure*/
    SMP_MAX_FAIL_RSN_PER_SPEC = SMP_ERR_CODE_BUSY, /**< SMP Max Fail Reason as per spec */

    /* bte smp status codes */
    SMP_PAIR_INTERNAL_ERR       = (SMP_MAX_FAIL_RSN_PER_SPEC + 0x01), /**< Internal error */
    SMP_UNKNOWN_IO_CAP          = (SMP_MAX_FAIL_RSN_PER_SPEC + 0x02), /**< unknown IO capability, unable to decide associatino model */
    SMP_INIT_FAIL               = (SMP_MAX_FAIL_RSN_PER_SPEC + 0x03), /**< Initialization failed */
    SMP_CONFIRM_FAIL            = (SMP_MAX_FAIL_RSN_PER_SPEC + 0x04), /**< Confirmation failed */
    SMP_BUSY                    = (SMP_MAX_FAIL_RSN_PER_SPEC + 0x05), /**< Busy */
    SMP_ENC_FAIL                = (SMP_MAX_FAIL_RSN_PER_SPEC + 0x06), /**< Encryption failed */
    SMP_STARTED                 = (SMP_MAX_FAIL_RSN_PER_SPEC + 0x07), /**< Started */
    SMP_RSP_TIMEOUT             = (SMP_MAX_FAIL_RSN_PER_SPEC + 0x08), /**< Response timeout */
    SMP_FAIL                    = (SMP_MAX_FAIL_RSN_PER_SPEC + 0x09), /**< Generic failure */
    SMP_CONN_TOUT               = (SMP_MAX_FAIL_RSN_PER_SPEC + 0x0A), /**< Connection timeout */
};
/** SMP Pairing status (see #wiced_bt_smp_status_e) */
typedef uint8_t wiced_bt_smp_status_t;

/** LE keys */
typedef struct
{
    BT_OCTET16       ir;   /**< IR Key */
    BT_OCTET16       irk;  /**< IRK Key  */
    BT_OCTET16       dhk;  /**< DHK Key */
}wiced_bt_local_id_keys;


/** LE identity key for local device (used by #BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT and #BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT notification) */
typedef struct {
    uint8_t            key_type_mask;  /**< The type of the key (#BTM_BLE_KEY_TYPE_ID or #BTM_BLE_KEY_TYPE_ER) */
    wiced_bt_local_id_keys id_keys;  /**< Local ID Keys    */
    BT_OCTET16       er;             /**< LE encryption key  */
} wiced_bt_local_identity_keys_t;

/* Key types that may be set in the key_type_mask field */
#define BTM_BLE_KEY_TYPE_ID         1       /**< Identity resolving key */
#define BTM_BLE_KEY_TYPE_ER         2       /**< Encryption root key    */



/** LE connection parameter update event related data */
typedef struct
{
    wiced_bt_hci_err_code_t     status;             /**< connection parameters update status */
    wiced_bt_device_address_t   bd_addr;            /**< peer bd address */
    uint16_t                    conn_interval;      /**< updated connection interval ( in 0.625msec ) */
    uint16_t                    conn_latency;       /**< updated connection latency */
    uint16_t                    supervision_timeout;/**< updated supervision timeout */
} wiced_bt_ble_connection_param_update_t;

/** LE connection parameter request event related data */
typedef struct
{
    uint8_t deny;                      /**< allow or deny request, set 0 to allow, 1 to deny */
    wiced_bt_device_address_t bd_addr; /**< peer bd address */
    uint16_t min_interval;             /**< requested min connection interval */
    uint16_t max_interval;             /**< requested max connection interval */
    uint16_t conn_latency;             /**< requested connection latency */
    uint16_t supervision_timeout;      /**< requested supervision timeout */
    uint16_t min_ce_len;               /**< min connection event length preferred */
    uint16_t max_ce_len;               /**< max connection event length preferred */
} wiced_bt_ble_connection_param_request_t;


/** LE Physical link update event related data */
typedef struct
{
    uint8_t status;                    /**< status of the event, ignored when the structure is used to set */
    wiced_bt_device_address_t bd_addr; /**< peer bd address, set to 0 if default */
    uint16_t subrate_min;     /**< Minimum subrate factor allowed in requests by a Peripheral,
                                   Range: 0x0001 to 0x01F4,
                                   Default: 0x0001
                                   */
    uint16_t subrate_max;     /**< Maximum subrate factor allowed in requests by a Peripheral,
                                   Range: 0x0001 to 0x01F4,
                                   Default: 0x0001
                                   */
    uint16_t latency;         /**< Maximum Peripheral latency allowed in requests by a Peripheral,
                                   in units of subrated connection intervals,
                                   Range: 0x0000 to 0x01F3,
                                   Default: 0x0000
                                   */
    uint16_t continuation_number; /**< Minimum number of underlying connection events to remain active
                                       after a packet containing a Link Layer PDU with a non-zero Length field
                                       is sent or received in requests by a Peripheral,
                                       Range: 0x0000 to 0x01F3,
                                       Default: 0x0000
                                       */
    uint16_t supervision_timeout; /**< Maximum supervision timeout allowed in requests by a Peripheral,
                                       Range: 0x000A to 0x0C80,
                                       Time = N × 10 ms,
                                       Time Range: 100 ms to 32 s,
                                       Default: 0x0C80
                                       */
} wiced_bt_ble_conn_subrate_t;


/** BLE Physical link update event related data */
typedef struct
{
    wiced_bt_hci_err_code_t      status;      /**< LE Phy update status */
    wiced_bt_device_address_t    bd_address;  /**< peer BD address*/
    uint8_t                      tx_phy;      /**< Transmitter PHY, values: 1=1M, 2=2M, 3=LE coded */
    uint8_t                      rx_phy;      /**< Receiver PHY, values: 1=1M, 2=2M, 3=LE coded */
} wiced_bt_ble_phy_update_t;

/** LE data length update event related data */
typedef struct
{
    wiced_bt_device_address_t bd_address; /**< peer BD address*/
    uint16_t max_tx_octets; /**< Max number of bytes to be sent in an LLData PDU */
    uint16_t max_tx_time;   /**< Max time that the local controller will take to send a LL packet in a LLData PDU */
    uint16_t max_rx_octets; /**< Max number of bytes to be received in an LLData PDU */
    uint16_t max_rx_time;   /**< Max time that the local controller expects to receive a LL packet in a LLData PDU */
} wiced_bt_ble_phy_data_length_update_t;

/** LE Multi adv opcodes returned */
typedef enum
{
    SET_ADVT_PARAM_MULTI     = 1,            /**< Opcode as a result of calling wiced_set_multi_advertisement_params*/
    SET_ADVT_DATA_MULTI      = 2,            /**< Opcode as a result of calling wiced_set_multi_advertisement_data*/
    SET_SCAN_RESP_DATA_MULTI = 3,            /**< Opcode as a result of calling wiced_set_multi_advertisement_scan_response_data*/
    SET_RANDOM_ADDR_MULTI    = 4,            /**< Opcode as a result of calling wiced_set_multi_advertisements_random_address*/
    SET_ADVT_ENABLE_MULTI    = 5             /**< Opcode as a result of calling wiced_start_multi_advertisements*/
} wiced_bt_multi_adv_opcodes_t;

/** LE Multi adv VSC response data */
typedef struct
{
    wiced_bt_multi_adv_opcodes_t    opcode;      /**< Multi adv vendor specifiv opcode */
    wiced_bt_hci_err_code_t         status;      /**< status of the operation received from controller, 0 - Success. Check the HCI error codes Vol 1, Part F, Table 1.1 Error codes */
} wiced_bt_ble_multi_adv_response_t;

/** Power Management status codes */
#ifndef BTM_PM_STATUS_CODES
#define BTM_PM_STATUS_CODES
/** Power Management status */
enum wiced_bt_dev_power_mgmt_status_e
{
    BTM_PM_STS_ACTIVE = HCI_MODE_ACTIVE,    /**< Active */
    BTM_PM_STS_HOLD   = HCI_MODE_HOLD,      /**< Hold */
    BTM_PM_STS_SNIFF  = HCI_MODE_SNIFF,     /**< Sniff */
    BTM_PM_STS_PARK   = HCI_MODE_PARK,      /**< Park */
    BTM_PM_STS_SSR,                         /**< Sniff subrating notification */
    BTM_PM_STS_PENDING,                     /**< Pending (waiting for status from controller) */
    BTM_PM_STS_ERROR                        /**< Error (controller returned error) */
};
#endif

/** Power Management state */
#define WICED_POWER_STATE_ACTIVE  BTM_PM_STS_ACTIVE  /**< Active */
#define WICED_POWER_STATE_SNIFF   BTM_PM_STS_SNIFF   /**< Sniff */
#define WICED_POWER_STATE_SSR     BTM_PM_STS_SSR     /**< Sniff subrating notification */
#define WICED_POWER_STATE_PENDING BTM_PM_STS_PENDING /**< Pending (waiting for status from controller) */
#define WICED_POWER_STATE_ERROR   BTM_PM_STS_ERROR   /**< Error (controller returned error) */

typedef uint8_t wiced_bt_dev_power_mgmt_status_t; /**< Power management status (see #wiced_bt_dev_power_mgmt_status_e) */

/*BR channel map*/
#define BTM_AFH_CHNL_MAP_SIZE    HCI_AFH_CHANNEL_MAP_LEN        /**< Channel Map Length */

typedef uint8_t wiced_bt_br_chnl_map_t[BTM_AFH_CHNL_MAP_SIZE];  /**< Array of Channel Map Length */

/** Bluetooth Management event */
#ifndef BTM_MANAGEMENT_EVT
#define BTM_MANAGEMENT_EVT
/** Bluetooth Management events used in #wiced_bt_management_cback_t.
  * @note Some of the events are BR/EDR events which are available only in dual mode (BR/EDR+LE) operation
  * @note Return values of the management events
  * Return values of notification and status events are typically not checked, unless explicitly mentioned.
  * Return values of events requesting information from app should be WICED_BT_ERROR for cases where the app
  * does not handle the events. For the request events which are handled by the app, indicate WICED_BT_SUCCESS
  * as per the specific requirements of the event as documented below.
  *
  */
enum wiced_bt_management_evt_e {
    /* Bluetooth status events */

    /**
     * Event notifies Bluetooth controller and host stack is enabled.
     * Event data: \ref wiced_bt_management_evt_data_t.enabled
     * Indicates the stack is up. Application can now start calling bluetooth AIROC Bluetooth APIs
     */
    BTM_ENABLED_EVT,                    /* 0, 0x0 */

    /**
     * Event notifies Bluetooth controller and host stack disabled.
     * Event data: NULL
     */
    BTM_DISABLED_EVT,                   /* 1, 0x1 */

    /**
     * Event notifies Power management status change.
     * Event data: \ref wiced_bt_management_evt_data_t.power_mgmt_notification
     */
    BTM_POWER_MANAGEMENT_STATUS_EVT,    /* 2, 0x2 */

    /**
     * Event notifies Bluetooth controller and host stack re-enabled.
     * Event data: \ref wiced_bt_management_evt_data_t.enabled
     * @note : Not used
     */
    BTM_RE_START_EVT,                   /* 3, 0x3 */

    /* Security events */
    /**
     * Event requests app for the PIN to be used for pairing (legacy pairing only).
     * Event data: \ref wiced_bt_management_evt_data_t.pin_request
     */
    BTM_PIN_REQUEST_EVT,                /* 4, 0x4 */

    /**
     * Event requests user confirmation for the numeric value to continue the
     * App is expected to respond with using #wiced_bt_dev_confirm_req_reply typically
     * by confirming via a display to the user
     * Event data: \ref wiced_bt_management_evt_data_t.user_confirmation_request
     */
    BTM_USER_CONFIRMATION_REQUEST_EVT,  /* 5, 0x5 */

    /**
     * Event notifies user passkey app
     * App is expected to display the passkey to the user
     * Event data: \ref wiced_bt_management_evt_data_t.user_passkey_notification
     */
    BTM_PASSKEY_NOTIFICATION_EVT,       /* 6, 0x6 */

    /**
     * Event requests user passkey from app
     * @if DUAL_MODE
     * (respond using #wiced_bt_dev_pass_key_req_reply).
     * Application is expected to respond with the passkey for pairing
     * with \ref wiced_bt_dev_pass_key_req_reply
     * @endif
     * Event data: \ref wiced_bt_management_evt_data_t.user_passkey_request
     */
    BTM_PASSKEY_REQUEST_EVT,            /* 7, 0x7 */

    /**
     * Event notifies keypress notification event to app
     * Event data: \ref wiced_bt_management_evt_data_t.user_keypress_notification
     */
    BTM_KEYPRESS_NOTIFICATION_EVT,      /* 8, 0x8 */

    /**
     * Event requests BR/EDR IO capabilities for BR/EDR pairing from app
     * Event data: \ref wiced_bt_management_evt_data_t.pairing_io_capabilities_br_edr_request
     * App is expected to fill in it's BR/EDR IO capabilities into the incoming
     * \ref wiced_bt_management_evt_data_t.pairing_io_capabilities_br_edr_request structure member
     * @note  BR/EDR Only
     */
    BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT, /* 9, 0x9 */

    /**
     * Event notifies received IO capabilities response for BR/EDR pairing.
     * Event data: \ref wiced_bt_management_evt_data_t.pairing_io_capabilities_br_edr_response
     * @note  BR/EDR Only
     */
    BTM_PAIRING_IO_CAPABILITIES_BR_EDR_RESPONSE_EVT, /* 10, 0xA */

    /**
     * Event requests LE IO capabilities for LE pairing from app.
     * Peripheral can check peer io capabilities in event data before updating with local io capabilities.
     * Event data: \ref wiced_bt_management_evt_data_t.pairing_io_capabilities_ble_request
     */
    BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT,    /* 11, 0xB */

    /**
     * Event notifies simple pairing complete to app
     * Event data: \ref wiced_bt_management_evt_data_t.pairing_complete
     */
    BTM_PAIRING_COMPLETE_EVT,                       /* 12, 0xC */

    /**
     * Event notifies encryption status change to app
     * Event data: \ref wiced_bt_management_evt_data_t.encryption_status
     */
    BTM_ENCRYPTION_STATUS_EVT,                      /* 13, 0xD */

    /**
     * Event requests app to allow stack to continue Security procedures/pairing to continue with
     * the peer. App needs to respond with #wiced_bt_ble_security_grant
     * App is expected to either allow or deny the incoming pairing request based on it's state
     * Event data: \ref wiced_bt_management_evt_data_t.security_request
     */
    BTM_SECURITY_REQUEST_EVT,                       /* 14, 0xE */

    /**
     * Event notifies Security procedure/authentication failed to app
     * Event data:  \ref wiced_bt_management_evt_data_t.security_failed
     */
    BTM_SECURITY_FAILED_EVT,                        /* 15, 0xF */

    /**
     * Event notifies security procedure aborted locally, or unexpected link drop.
     * Event data: \ref wiced_bt_management_evt_data_t.security_aborted
     */
    BTM_SECURITY_ABORTED_EVT,                       /* 16, 0x10 */

    /**
     * Event notifies result of reading local OOB data from the controller
     * Event data: \ref wiced_bt_management_evt_data_t.read_local_oob_data_complete
     * @note  BR/EDR Only
     */
    BTM_READ_LOCAL_OOB_DATA_COMPLETE_EVT,           /* 17, 0x11 */

    /**
     * Event requests OOB data of the remote device from app
     * Event data: \ref wiced_bt_management_evt_data_t.remote_oob_data_request
     * @note  BR/EDR Only
     */
    BTM_REMOTE_OOB_DATA_REQUEST_EVT,                /* 18, 0x12 */

    /**
     * Event notifies app with the updated remote device link keys in this event.
     * App is expected to store device_link_keys to  NV memory.
     * This is the place to verify that the correct link key has been generated.
     * Event data: \ref wiced_bt_management_evt_data_t.paired_device_link_keys_update
     */
    BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT,         /* 19, 0x13 */

    /**
     * Event requests for stored remote device link keys from app (restore device_link_keys from NV memory).
     * If available then fill the stored keys into \p wiced_bt_management_evt_data_t.paired_device_link_keys_request
     * and return WICED_BT_SUCCESS else WICED_BT_ERROR
     * Event data: \ref wiced_bt_management_evt_data_t.paired_device_link_keys_request
     */
    BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT,        /* 20, 0x14 */

    /**
     * Event notifies updated local identity key to the app (store local_identity_keys to NV memory).
     * App is expected to store the identity key to  NV memory.
     * Event data: \ref wiced_bt_management_evt_data_t.local_identity_keys_update
     */
    BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT,             /* 21, 0x15 */

    /**
     * Event requests local identity key from app (get local_identity_keys from NV memory).
     * If available then fill the local key into \p wiced_bt_management_evt_data_t.local_identity_keys_request
     * and return WICED_BT_SUCCESS else WICED_BT_ERROR.
     * Event data: \ref wiced_bt_management_evt_data_t.local_identity_keys_request
     */
    BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT,            /* 22, 0x16 */

    /**
     * Event notifies LE scan state change to app
     * Event data: \ref wiced_bt_management_evt_data_t.ble_scan_state_changed
     */
    BTM_BLE_SCAN_STATE_CHANGED_EVT,                 /* 23, 0x17 */

    /**
     * Event notifies LE advertisement state change to app
     * Event data: \ref wiced_bt_management_evt_data_t.ble_advert_state_changed
     */
    BTM_BLE_ADVERT_STATE_CHANGED_EVT,               /* 24, 0x18 */

    /* LE Secure Connection events */
    /**
     * Event requests SMP remote oob data. Reply using wiced_bt_smp_oob_data_reply.
     * Event data: \ref wiced_bt_management_evt_data_t.smp_remote_oob_data_request
     */
    BTM_SMP_REMOTE_OOB_DATA_REQUEST_EVT,            /* 25, 0x19 */

    /**
     * Event requests LE secure connection remote oob data request. Reply using wiced_bt_smp_sc_oob_reply.
     * Event data: \ref wiced_bt_management_evt_data_t.smp_sc_remote_oob_data_request
     */
    BTM_SMP_SC_REMOTE_OOB_DATA_REQUEST_EVT,         /* 26, 0x1A */

    /**
     * Event notifies LE secure connection local OOB data (#wiced_bt_smp_create_local_sc_oob_data) returned by the stack
     * The app is expected to copy the data into it's memory and share out of band with the peer
     * @if DUAL_MODE To build the data to be shared, app can use \ref wiced_bt_dev_build_oob_data @endif
     * Event data: \ref wiced_bt_management_evt_data_t.p_smp_sc_local_oob_data
     */
    BTM_SMP_SC_LOCAL_OOB_DATA_NOTIFICATION_EVT,     /* 27, 0x1B */

    /**
     * Event notfies SCO connected to app
     * Event data: \ref wiced_bt_management_evt_data_t.sco_connected
     * @note  BR/EDR Only
     */
    BTM_SCO_CONNECTED_EVT,                          /* 28, 0x1C */

    /**
     * Event notifies SCO disconnected event to app
     * Event data: \ref wiced_bt_management_evt_data_t.sco_disconnected
     * @note  BR/EDR Only
     */
    BTM_SCO_DISCONNECTED_EVT,                       /* 29, 0x1D */

    /**
     * Event notifies SCO connection request to app
     * App is expected to reply with @cond DUAL_MODE #wiced_bt_sco_accept_connection @endcond
     * Event data: \ref wiced_bt_management_evt_data_t.sco_connection_request
     * @note  BR/EDR Only
     */
    BTM_SCO_CONNECTION_REQUEST_EVT,                 /* 30, 0x1E */

    /**
     * Event notifies SCO connection change to app
     * Event data: \ref wiced_bt_management_evt_data_t.sco_connection_change
     * @note  BR/EDR Only
     */
    BTM_SCO_CONNECTION_CHANGE_EVT,                  /* 31, 0x1F */


    /**
     * Event notifies LE connection parameter update to app
     * Event data: \ref wiced_bt_management_evt_data_t.ble_connection_param_update
     */
    BTM_BLE_CONNECTION_PARAM_UPDATE,                /* 32, 0x20 */

    /**
     * Event notifies LE Physical link update to app
     * Event data: \ref wiced_bt_management_evt_data_t.ble_phy_update_event
     */
    BTM_BLE_PHY_UPDATE_EVT,                         /* 33, 0x21 */

    /**
     * Event notifies Bluetooth device wake has been deasserted. Used for Host Stack Use Case.
     */
    BTM_LPM_STATE_LOW_POWER,                        /* 34, 0x22 */

    /**
     * Event notifies Multi adv command status event used for the status of the command sent
     * Event data: \ref wiced_bt_management_evt_data_t.ble_multi_adv_response_event
     */
    BTM_MULTI_ADVERT_RESP_EVENT,                    /* 35, 0x23 */

    /**
     * Event to notify change in the data length and timeout configured for Rx and Tx on the
     * LE link
     * Event data: \ref wiced_bt_management_evt_data_t.ble_data_length_update_event
     */
    BTM_BLE_DATA_LENGTH_UPDATE_EVENT,               /* 36, 0x24 */

    /**
     * Event to notify subrate change event
     * BLE link
     * Event data: \ref wiced_bt_management_evt_data_t.ble_subrate_change_event
     */
    BTM_BLE_SUBRATE_CHANGE_EVENT,                   /* 37, 0x25 */

    /**
     * Event to notify change in the device address
     * Event data: \ref wiced_bt_management_evt_data_t.ble_addr_update_event
     */
    BTM_BLE_DEVICE_ADDRESS_UPDATE_EVENT,            /* 38, 0x26 */

    /**
     * Event to notify change channel selection algorithm for the connection
     * Event data: \ref wiced_bt_management_evt_data_t.ble_channel_sel_algo_event
     */
    BTM_BLE_CHANNEL_SELECTION_ALGO_EVENT, /* 39, 0x27*/

    /**
     * Event to notify Maximum Advertising data length supported
     * Event data: \ref wiced_bt_management_evt_data_t.max_adv_data_len
     */
    BTM_BLE_READ_MAX_ADV_DATA_LEN_EVENT,            /* 40, 0x28 */

    /**
     * Event to notify Setup QoS Complete or Flow Specication complete
     * Event data: \ref wiced_bt_management_evt_data_t.br_flow_spec_event
     */
    BTM_BR_ACL_FLOW_SPEC_COMPLETE_EVENT,           /* 41, 0x29 */

    /**
     * Event to allow application to allow/deny the incoming connection parameter update
     * request.
     * To allow the request set \ref wiced_bt_ble_connection_param_request_t.deny to 0.
     * To deny the request set \ref wiced_bt_ble_connection_param_request_t.deny 1
     * Event data: \ref wiced_bt_management_evt_data_t.ble_connection_param_request
     */
    BTM_BLE_CONNECTION_PARAM_REQUEST_EVENT,        /* 42, 0x3A */

#if SMP_CATB_CONFORMANCE_TESTER == TRUE
    /**
     * The Secure Connections support information of the peer device.
     */
    BTM_SMP_SC_PEER_INFO_EVT,                        /* 43, 0x2B */
#endif
};
#endif
typedef uint8_t wiced_bt_management_evt_t;          /**< Bluetooth management events (see #wiced_bt_management_evt_e) */

/** Device enabled (used by #BTM_ENABLED_EVT) */
typedef struct {
    wiced_result_t          status;                     /**< Status */
} wiced_bt_dev_enabled_t;

/** Device disabled (used by #BTM_DISABLED_EVT) */
typedef struct {
    uint8_t          reason; /**< Reason for #BTM_DISABLED_EVT */
}wiced_bt_dev_disabled_t;

/** Remote device information (used by #BTM_PIN_REQUEST_EVT, #BTM_SECURITY_ABORTED_EVT) */
typedef struct {
    wiced_bt_device_address_t  *bd_addr;   /**< BD Address of remote */
    wiced_bt_dev_class_t       *dev_class; /**< peer class of device */
    uint8_t                    *bd_name;   /**< BD Name of remote */
} wiced_bt_dev_name_and_class_t;

/** Change in power management status  (used by #BTM_POWER_MANAGEMENT_STATUS_EVT notication) */
typedef struct {
    wiced_bt_device_address_t           bd_addr;        /**< BD Address of remote */
    wiced_bt_dev_power_mgmt_status_t    status;         /**< PM status */
    uint16_t                            value;          /**< Additional mode data */
    uint8_t                             hci_status;     /**< HCI status */
} wiced_bt_power_mgmt_notification_t;

/** Encryption status change (used by #BTM_ENCRYPTION_STATUS_EVT) */
typedef struct {
    uint8_t                 *bd_addr;    /**< BD Address of remote */
    wiced_bt_transport_t    transport;   /**< #BT_TRANSPORT_BR_EDR or #BT_TRANSPORT_LE */
    void                    *p_ref_data; /**< Optional data passed in by #wiced_bt_dev_set_encryption */
    wiced_result_t          result;      /**< Result of the operation */
} wiced_bt_dev_encryption_status_t;

/** Local OOB data #BTM_READ_LOCAL_OOB_DATA_COMPLETE_EVT */
typedef struct
{
    wiced_result_t status;             /**< Status */
    wiced_bool_t is_extended_oob_data; /**< TRUE if extended OOB data */

    BT_OCTET16 c_192; /**< Simple Pairing Hash C derived from the P-192 public key */
    BT_OCTET16 r_192; /**< Simple Pairing Randomnizer R associated with the P-192 public key */
    BT_OCTET16 c_256; /**< Simple Pairing Hash C derived from the P-256 public key
                        (valid only if \p is_extended_oob_data=TRUE) */
    BT_OCTET16 r_256; /**< Simple Pairing Randomnizer R associated with the P-256 public key
                        (valid only if \p is_extended_oob_data=TRUE) */
} wiced_bt_dev_local_oob_t;

/** Data for #BTM_REMOTE_OOB_DATA_REQUEST_EVT */
typedef struct {
    wiced_bt_device_address_t   bd_addr;                /**< BD Address of remote */
    wiced_bool_t                extended_oob_data;      /**< TRUE if requesting extended OOB (P-256) */
} wiced_bt_dev_remote_oob_t;

/** BR/EDR Pairing IO Capabilities (to be filled by application callback on #BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT) */
typedef struct
{
    wiced_bt_device_address_t   bd_addr;     /**< [in] BD Address of remote   */
    wiced_bt_dev_io_cap_t       local_io_cap;/**< local IO capabilities (to be filled by app) */
    wiced_bt_dev_oob_data_t     oob_data;    /**< OOB data present at peer device for the local device (to be filled by app)  */
    wiced_bt_dev_auth_req_t     auth_req;    /**< Authentication required for peer device (to be filled by app) */
    wiced_bool_t                is_orig;     /**< [in] TRUE, if local device initiated the pairing process    */
} wiced_bt_dev_bredr_io_caps_req_t;

/** LE Pairing IO Capabilities (to be filled by application callback on #BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT) */
typedef struct
{
    wiced_bt_device_address_t bd_addr;      /**< [in] BD Address of remote   */
    wiced_bt_dev_io_cap_t local_io_cap;     /**< local IO capabilities (to be filled by application callback) */
    uint8_t oob_data;                       /**< OOB data present (locally) for the peer device */
    wiced_bt_dev_le_auth_req_t auth_req;    /**< Authentication request (for local device) */
    uint8_t max_key_size;                   /**< Max encryption key size */
    wiced_bt_dev_le_key_type_t init_keys;   /**< Keys to be distributed, bit mask*/
    wiced_bt_dev_le_key_type_t resp_keys;   /**< keys to be distributed, bit mask*/
} wiced_bt_dev_ble_io_caps_req_t;

/** Paired device LE Keys  */
typedef struct
{
    BT_OCTET16               irk;            /**< peer diverified identity root */
    BT_OCTET16               pltk;           /**< peer long term key */
    BT_OCTET16               pcsrk;          /**< peer SRK peer device used to secured sign local data  */

    BT_OCTET16               lltk;           /**< local long term key */
    BT_OCTET16               lcsrk;          /**< local SRK peer device used to secured sign local data  */

    BT_OCTET8                rand;           /**< random vector for LTK generation */
    uint16_t                 ediv;           /**< LTK diversifier of this Peripheral device */
    uint16_t                 div;            /**< local DIV  to generate local LTK=d1(ER,DIV,0) and CSRK=d1(ER,DIV,1)  */
    wiced_bt_smp_sec_level_t sec_level;      /**< local pairing security level */
    uint8_t                  key_size;       /**< key size of the LTK delivered to peer device */
    uint8_t                  srk_sec_level;  /**< security property of peer SRK for this device */
    uint8_t                  local_csrk_sec_level;  /**< security property of local CSRK for this device */

    uint32_t                 counter;        /**< peer sign counter for verifying rcv signed cmd */
    uint32_t                 local_counter;  /**< local sign counter for sending signed write cmd*/
}wiced_bt_ble_keys_t;

/** Paired Device Link key data */
typedef struct
{
    /* BR/EDR key */
    uint8_t br_edr_key_type;        /**<  BR/EDR Link Key type */
    wiced_bt_link_key_t br_edr_key; /**<  BR/EDR Link Key */

    /* LE Keys */
    uint8_t le_keys_available_mask;            /**<  Mask of available LE keys */
    wiced_bt_ble_address_type_t ble_addr_type; /**<  LE address type: public or random address */
    wiced_bt_ble_keys_t le_keys;               /**<  LE keys */
} wiced_bt_device_sec_keys_t;

/** Paired device link key notification (used by #BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT notication) */
typedef struct
{
    wiced_bt_device_address_t   bd_addr;         /**< [in] BD Address of remote
                                                 @note For LE transport
                                                    1. device_addr_type for this bd_addr is in \ref wiced_bt_device_sec_keys_t.ble_addr_type
                                                    2. if the remote device has shared an IRK, this address will be of type \ref BLE_ADDR_PUBLIC_ID or
                                                       \ref BLE_ADDR_RANDOM_ID which may be different from the address used in the connection
                                                       complete event (\ref GATT_CONNECTION_STATUS_EVT).
                                                    3. Application must use the address in this structure only for storing/retrieving the keys. For any
                                                       other API which requires the bd_addr, application must continue to use the address reported in the
                                                       \ref GATT_CONNECTION_STATUS_EVT.                                                 */

    wiced_bt_device_sec_keys_t  key_data;        /**< [in/out] Key data */
    wiced_bt_device_address_t   conn_addr;       /**< [in] BD Address remote used to originate the first connection */
} wiced_bt_device_link_keys_t;

/**
 * .Enumeration of known link policy settings value assignments of the \ref wiced_bt_link_policy_settings_t
 */
enum  wiced_bt_link_policy_settings_values_e
{
    WICED_ENABLE_ROLE_SWITCH  = 0x01,       /**< Enable Role Switch */
    WICED_ENABLE_HOLD_MODE    = 0x02,       /**< Enable Hold mode */
    WICED_ENABLE_SNIFF_MODE   = 0x04        /**<Enable Sniff mode */
};

/** Link Policy Settings type (see #wiced_bt_link_policy_settings_values_e) */
typedef uint16_t wiced_bt_link_policy_settings_t[1];

/** advertisement type (used when calling #wiced_bt_start_advertisements) */
#ifndef BTM_BLE_ADVERT_MODE
#define BTM_BLE_ADVERT_MODE
/** advertisement type (used when calling #wiced_bt_start_advertisements) */
enum wiced_bt_ble_advert_mode_e
{
    BTM_BLE_ADVERT_OFF,                 /**< Stop advertising */
    BTM_BLE_ADVERT_DIRECTED_HIGH,       /**< Directed advertisement (high duty cycle) */
    BTM_BLE_ADVERT_DIRECTED_LOW,        /**< Directed advertisement (low duty cycle) */
    BTM_BLE_ADVERT_UNDIRECTED_HIGH,     /**< Undirected advertisement (high duty cycle) */
    BTM_BLE_ADVERT_UNDIRECTED_LOW,      /**< Undirected advertisement (low duty cycle) */
    BTM_BLE_ADVERT_NONCONN_HIGH,        /**< Non-connectable advertisement (high duty cycle) */
    BTM_BLE_ADVERT_NONCONN_LOW,         /**< Non-connectable advertisement (low duty cycle) */
    BTM_BLE_ADVERT_DISCOVERABLE_HIGH,   /**< discoverable advertisement (high duty cycle) */
    BTM_BLE_ADVERT_DISCOVERABLE_LOW     /**< discoverable advertisement (low duty cycle) */
};
#endif
typedef uint8_t wiced_bt_ble_advert_mode_t;   /**< Advertisement type (see #wiced_bt_ble_advert_mode_e) */

/** scan mode used in initiating */
#ifndef BTM_BLE_CONN_MODE
#define BTM_BLE_CONN_MODE
/** scan mode used in initiating */
enum wiced_bt_ble_conn_mode_e
{
    BLE_CONN_MODE_OFF,                  /**< Stop initiating */
    BLE_CONN_MODE_LOW_DUTY,             /**< slow connection scan parameter */
    BLE_CONN_MODE_HIGH_DUTY             /**< fast connection scan parameter */
};
#endif
typedef uint8_t wiced_bt_ble_conn_mode_t;       /**< Conn mode (see #wiced_bt_ble_conn_mode_e) */

#if SMP_CATB_CONFORMANCE_TESTER == TRUE
typedef struct
{
    wiced_bool_t             peer_sc_is_set;     /* Secure Connection    */
    wiced_bool_t             peer_kp_n_is_set;   /* KeyPress Notification */
} wiced_bt_ble_sc_peer_info;
#endif

/** HCI trace types  */
typedef enum
{
    HCI_TRACE_EVENT, /**< HCI event data from controller to the host */
    HCI_TRACE_COMMAND, /**< HCI command data from host to controller */
    HCI_TRACE_INCOMING_ACL_DATA,/**< HCI incoming acl data */
    HCI_TRACE_OUTGOING_ACL_DATA,/**< HCI outgoing acl data */
    HCI_TRACE_INCOMING_ISO_DATA,/**< HCI incoming ISO data */
    HCI_TRACE_OUTGOING_ISO_DATA,/**< HCI outgoing ISO data */
    HCI_TRACE_INCOMING_SCO_DATA,/**< HCI incoming sco data */
    HCI_TRACE_OUTGOING_SCO_DATA /**< HCI outgoing sco data */
}wiced_bt_hci_trace_type_t;

/** LE ACL connection handle */
typedef uint16_t wiced_bt_ble_connection_handle_t;

/** Event on update of random device address */
typedef struct
{
    uint8_t status; /**< status of the change address command */
    wiced_bt_device_address_t bdaddr; /**< current private bluetooth address */
}wiced_bt_ble_device_addr_update_t;

/** LE channel selection algorithms */
enum wiced_bt_ble_channel_sel_algo_e
{
    LE_CHANNEL_SEL_ALGO_1_USED, /**< LE channel selection algorithm#1 used */
    LE_CHANNEL_SEL_ALGO_2_USED, /**< LE channel selection algorithm#2 used */
};
typedef uint8_t
    wiced_bt_ble_channel_sel_algo_t; /**< LE channel algorithm selection (see #wiced_bt_ble_channel_sel_algo_e) */

/** Channel selection algorithm event data format */
typedef struct
{
    wiced_bt_ble_connection_handle_t connection_handle; /**< LE ACL connection handle */
    wiced_bt_ble_channel_sel_algo_t channel_sel_algo;   /**< LE channel selection algorithm used for this connection*/
    /* remaining RFU */
} wiced_bt_ble_channel_sel_algo_event_data_t;


/** Structure definitions for Bluetooth Management (#wiced_bt_management_cback_t) event notifications */
typedef union
{
    /* Bluetooth status event data types*/
    wiced_bt_dev_enabled_t                  enabled;                            /**< Data for #BTM_ENABLED_EVT */
    wiced_bt_dev_disabled_t                 disabled;                           /**< Data for #BTM_DISABLED_EVT */
    wiced_bt_power_mgmt_notification_t      power_mgmt_notification;            /**< Data for #BTM_POWER_MANAGEMENT_STATUS_EVT */

    /* Security event data types */
    wiced_bt_dev_name_and_class_t           pin_request;                        /**< Data for #BTM_PIN_REQUEST_EVT */
    wiced_bt_dev_user_cfm_req_t             user_confirmation_request;          /**< Data for #BTM_USER_CONFIRMATION_REQUEST_EVT */
    wiced_bt_dev_user_key_notif_t           user_passkey_notification;          /**< Data for #BTM_PASSKEY_NOTIFICATION_EVT */
    wiced_bt_dev_user_key_req_t             user_passkey_request;               /**< Data for #BTM_PASSKEY_REQUEST_EVT */
    wiced_bt_dev_user_keypress_t            user_keypress_notification;         /**< Data for #BTM_KEYPRESS_NOTIFICATION_EVT */
    wiced_bt_dev_bredr_io_caps_req_t        pairing_io_capabilities_br_edr_request; /**< Data for #BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT */
    wiced_bt_dev_bredr_io_caps_rsp_t        pairing_io_capabilities_br_edr_response;/**< Data for #BTM_PAIRING_IO_CAPABILITIES_BR_EDR_RESPONSE_EVT */
    wiced_bt_dev_ble_io_caps_req_t          pairing_io_capabilities_ble_request;    /**< Data for #BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT */
    wiced_bt_dev_pairing_cplt_t             pairing_complete;                   /**< Data for #BTM_PAIRING_COMPLETE_EVT */
    wiced_bt_dev_encryption_status_t        encryption_status;                  /**< Data for #BTM_ENCRYPTION_STATUS_EVT */
    wiced_bt_dev_security_request_t         security_request;                   /**< Data for #BTM_SECURITY_REQUEST_EVT */
    wiced_bt_dev_security_failed_t          security_failed;                    /**< Data for #BTM_SECURITY_FAILED_EVT See #wiced_bt_dev_security_failed_t */
    wiced_bt_dev_name_and_class_t           security_aborted;                   /**< Data for BTM_SECURITY_ABORTED_EVT */

    wiced_bt_dev_local_oob_t                read_local_oob_data_complete;       /**< Data for #BTM_READ_LOCAL_OOB_DATA_COMPLETE_EVT */
    wiced_bt_dev_remote_oob_t               remote_oob_data_request;            /**< Data for #BTM_REMOTE_OOB_DATA_REQUEST_EVT */

    wiced_bt_device_link_keys_t             paired_device_link_keys_update;     /**< Data for #BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT */
    wiced_bt_device_link_keys_t             paired_device_link_keys_request;    /**< Data for #BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT */
    wiced_bt_local_identity_keys_t          local_identity_keys_update;         /**< Data for #BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT */
    wiced_bt_local_identity_keys_t          local_identity_keys_request;        /**< Data for #BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT */

    wiced_bt_ble_scan_type_t                ble_scan_state_changed;             /**< Data for #BTM_BLE_SCAN_STATE_CHANGED_EVT */
    wiced_bt_ble_advert_mode_t              ble_advert_state_changed;           /**< Data for #BTM_BLE_ADVERT_STATE_CHANGED_EVT */

    wiced_bt_smp_remote_oob_req_t           smp_remote_oob_data_request;        /**< Data for #BTM_SMP_REMOTE_OOB_DATA_REQUEST_EVT */
    wiced_bt_smp_sc_remote_oob_req_t        smp_sc_remote_oob_data_request;     /**< Data for #BTM_SMP_SC_REMOTE_OOB_DATA_REQUEST_EVT */
    wiced_bt_smp_sc_local_oob_t             *p_smp_sc_local_oob_data;           /**< Data for #BTM_SMP_SC_LOCAL_OOB_DATA_NOTIFICATION_EVT */

    wiced_bt_sco_connected_t                sco_connected;                      /**< Data for #BTM_SCO_CONNECTED_EVT */
    wiced_bt_sco_disconnected_t             sco_disconnected;                   /**< Data for #BTM_SCO_DISCONNECTED_EVT */
    wiced_bt_sco_connection_request_t       sco_connection_request;             /**< Data for #BTM_SCO_CONNECTION_REQUEST_EVT */
    wiced_bt_sco_connection_change_t        sco_connection_change;              /**< Data for #BTM_SCO_CONNECTION_CHANGE_EVT */
    wiced_bt_ble_connection_param_update_t  ble_connection_param_update;        /**< Data for #BTM_BLE_CONNECTION_PARAM_UPDATE */
    wiced_bt_ble_phy_update_t               ble_phy_update_event;               /**< Data for #BTM_BLE_PHY_UPDATE_EVT */
    wiced_bt_ble_multi_adv_response_t       ble_multi_adv_response_event;       /**< Response status update event for the multiadv command #BTM_MULTI_ADVERT_RESP_EVENT*/
    wiced_bt_ble_phy_data_length_update_t   ble_data_length_update_event;       /**< Data for #BTM_BLE_DATA_LENGTH_UPDATE_EVENT*/
    wiced_bt_ble_conn_subrate_t             ble_subrate_change_event;           /**< Data for #BTM_BLE_SUBRATE_CHANGE_EVENT */
    wiced_bt_ble_device_addr_update_t       ble_addr_update_event;              /**< Data for #BTM_BLE_DEVICE_ADDRESS_UPDATE_EVENT */
    wiced_bt_ble_channel_sel_algo_event_data_t ble_channel_sel_algo_event;      /**< Data for #BTM_BLE_CHANNEL_SELECTION_ALGO_EVENT*/
    uint16_t                                max_adv_data_len;                   /**< Data for #BTM_BLE_READ_MAX_ADV_DATA_LEN_EVENT*/
    wiced_bt_flow_spec_cmpl_evt_t           br_flow_spec_event;                 /**< Data for #BTM_BR_ACL_FLOW_SPEC_COMPLETE_EVENT*/
    wiced_bt_ble_connection_param_request_t ble_connection_param_request;       /**< Data for BTM_BLE_DEVICE_ADDRESS_UPDATE_EVENT  */
#if SMP_CATB_CONFORMANCE_TESTER == TRUE
    wiced_bt_ble_sc_peer_info               smp_sc_peer_info;                   /* Data for #BTM_SMP_SC_PEER_INFO_EVT */
#endif

} wiced_bt_management_evt_data_t;

/**
 * Bluetooth Management callback
 *
 * Callback for Bluetooth Management event notifications.
 * Registered using wiced_bt_stack_init()
 *
 * @param event             : Event ID
 * @param p_event_data      : Event data
 *
 * @return Status of event handling
 */
typedef wiced_result_t (wiced_bt_management_cback_t) (wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);

/**
 * Connection status change callback.
 * Callback for Bluetooth Management event notifications.
 * Registered using wiced_bt_register_connection_status_change()
 *
 * @param[in] bd_addr       : BD Address of remote
 * @if DUAL_MODE
 * @param[in] p_features    : BR/EDR Peer feature
 * @else
 * @param[in] p_features    : unused (NULL)
 * @endif
 * @param[in] is_connected  : TRUE if connected
 * @param[in] handle        : Connection handle
 * @param[in] transport     : BT_TRANSPORT_BR_EDR or BT_TRANSPORT_LE
 * @param[in] reason        : status for acl connection change \n
 * <b>  HCI_SUCCESS \n
 * HCI_ERR_PAGE_TIMEOUT \n
 * HCI_ERR_MEMORY_FULL \n
 * HCI_ERR_CONNECTION_TOUT \n
 * HCI_ERR_PEER_USER \n
 * HCI_ERR_CONN_CAUSE_LOCAL_HOST \n
 * HCI_ERR_LMP_RESPONSE_TIMEOUT \n
 * HCI_ERR_CONN_FAILED_ESTABLISHMENT </b>
 *
 */
typedef void (wiced_bt_connection_status_change_cback_t) (wiced_bt_device_address_t bd_addr, uint8_t *p_features, wiced_bool_t is_connected, uint16_t handle, wiced_bt_transport_t transport, uint8_t reason);  /**<   connection status change callback */

/**
 * Inquiry result callback.
 *
 * @param p_inquiry_result  : Inquiry result data (NULL if inquiry is complete)
 * @param p_eir_data        : Extended inquiry response data
 *
 */
typedef void (wiced_bt_inquiry_result_cback_t) (wiced_bt_dev_inquiry_scan_result_t *p_inquiry_result, uint8_t *p_eir_data); /**<   inquiry result callback */

/**
 * Remote name result callback.
 *
 * @param p_remote_name_result          : Remote name result data
 *
 * @return void
 */
typedef void (wiced_bt_remote_name_cback_t) (wiced_bt_dev_remote_name_result_t *p_remote_name_result); /**<   remote name result callback */

/**
 * Vendor event handler callback
 *
 * @param len               : input data length
 * @param p                 : input data
 */
typedef void (wiced_bt_dev_vse_callback_t)(uint8_t len, uint8_t *p);

/**
 * HCI trace callback
 *
 * Callback for HCI traces
 * Registered using wiced_bt_dev_register_hci_trace()
 *
 * @param[in] type       : Trace type
 * @param[in] length : Length of the trace data
 * @param[in] p_data  : Pointer to the data
 *
 * @return void
 */
typedef void (wiced_bt_hci_trace_cback_t)(wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data);

/**@} wicedbt */

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *               Function Declarations
 ******************************************************/
/****************************************************************************/

/****************************************************************************/
/**
 * @cond DUAL_MODE
 * BR/EDR (Bluetooth Basic Rate / Enhanced Data Rate) Functions.
 * This module provids different set of BR/EDR API for discovery, inquiry
 * ACL & SCO Connection, Data transfer, BR/EDR Security etc.

 *
 * @addtogroup  wicedbt_bredr    BR/EDR (Bluetooth Basic Rate / Enhanced Data Rate)
 * @ingroup     wicedbt_DeviceManagement
 *
 * @{
 */
/****************************************************************************/

/****************************************************************************/
/**
 * This module provided various Bluetooth security functionality such as authorisation, authentication and encryption
 *
 * @addtogroup  wiced_bredr_api    Bluetooth BR/EDR API
 * @ingroup     wicedbt_bredr
 *
 * @{
 */
/****************************************************************************/

/**
 * Begin BR/EDR inquiry for peer devices.
 *
 * @param[in]       p_inqparms              : inquiry parameters
 * @param[in]       p_inquiry_result_cback  : inquiry results callback
 *
 * @return          wiced_result_t
 *
 * <b> WICED_BT_PENDING </b>        : if successfully initiated \n
 * <b> WICED_BT_BUSY </b>           : if already in progress \n
 * <b> WICED_BT_ILLEGAL_VALUE </b>  : if parameter(s) are out of range \n
 * <b> WICED_BT_NO_RESOURCES </b>   : if could not allocate resources to start the command \n
 * <b> WICED_BT_WRONG_MODE </b>     : if the device is not up
 */
wiced_result_t  wiced_bt_start_inquiry (wiced_bt_dev_inq_parms_t *p_inqparms, wiced_bt_inquiry_result_cback_t *p_inquiry_result_cback);

/**
 *
 * Cancel inquiry
 *
 * @return
 *
 * <b> WICED_BT_SUCCESS </b>       : if successful \n
 * <b> WICED_BT_NO_RESOURCES </b>  : if could not allocate a message buffer \n
 * <b> WICED_BT_WRONG_MODE </b>    : if the device is not up.
 */
wiced_result_t wiced_bt_cancel_inquiry(void);

/**
 * Read the local device address
 *
 * @param[out]      bd_addr        : Local bd address
 *
 * @return          void
 *
 */
void wiced_bt_dev_read_local_addr (wiced_bt_device_address_t bd_addr);

/**
 * Read the extended local device address information
 *
 * @param[out]      data        : data pointer in which stack will populate the address information.(refer #wiced_bt_dev_local_addr_ext_t )
 *
 * @return          void
 *
 */
void wiced_bt_dev_read_local_addr_ext(wiced_bt_dev_local_addr_ext_t *data);


/**
 *
 * Set advanced connection parameters for subsequent BR/EDR connections
 * (remote clock offset, page scan mode,  and other information obtained during inquiry)
 *
 * If not called, then default connection parameters will be used.
 *
 * @param[in]       p_inquiry_scan_result : Inquiry scan result (from #wiced_bt_dev_inquiry_scan_result_t)
 *
 * @return          wiced_result_t
 *
 * <b> WICED_BT_SUCCESS </b> : on success; \n
 * <b> WICED_BT_FAILED  </b> : if an error occurred
 */
wiced_result_t wiced_bt_dev_set_advanced_connection_params (wiced_bt_dev_inquiry_scan_result_t *p_inquiry_scan_result);

/**
 *
 * Send a vendor specific HCI command to the controller.
 *
 * @param[in]       opcode              : Opcode of vendor specific command
 * @param[in]       param_len           : Length of parameter buffer
 * @param[in]       p_param_buf         : Parameters
 * @param[in]       p_cback             : Callback for command complete
 *
 * @return
 *
 * <b> WICED_BT_SUCCESS </b>   : Command sent. Does not expect command complete event. (command complete callback param is NULL) \n
 * <b> WICED_BT_PENDING </b>   : Command sent. Waiting for command complete event. \n
 * <b> WICED_BT_BUSY    </b>   : Command not sent. Waiting for command complete event for prior command.
 *
 */
wiced_result_t wiced_bt_dev_vendor_specific_command (uint16_t opcode, uint8_t param_len, uint8_t *p_param_buf,
                                wiced_bt_dev_vendor_specific_command_complete_cback_t *p_cback);

/**
 * Set discoverability for BR/EDR devices
 *
 * @note            The duration must be less than or equal to the interval.
 *
 * @param[in]       inq_mode        : Discoverability mode (see #wiced_bt_discoverability_mode_e )
 * @param[in]       duration        : Duration (in 0.625 msec intervals). <b>BTM_DEFAULT_DISC_WINDOW</b>, or range: <b>0x0012 ~ 0x1000 </b> (11.25 ~ 2560 msecs)
 * @param[in]       interval        : Interval (in 0.625 msec intervals). <b>BTM_DEFAULT_DISC_INTERVAL</b>, or range: <b>0x0012 ~ 0x1000 </b> (11.25 ~ 2560 msecs)
 *
 * @return
 *
 * <b> WICED_BT_SUCCESS </b>       : If successful \n
 * <b> WICED_BT_BUSY </b>          : If a setting of the filter is already in progress \n
 * <b> WICED_BT_NO_RESOURCES </b>  : If couldn't get a memory pool buffer \n
 * <b> WICED_BT_ILLEGAL_VALUE </b> : If a bad parameter was detected \n
 * <b> WICED_BT_WRONG_MODE </b>    : If the device is not up
 */
wiced_result_t  wiced_bt_dev_set_discoverability (uint8_t inq_mode, uint16_t duration,
                                                    uint16_t interval);

/**
 *
 * Set connectablilty for BR/EDR devices
 *
 * @note            The duration (window parameter) must be less than or equal to the interval.
 *
 * @param[in]       page_mode       : Connectability mode (see #wiced_bt_connectability_mode_e )
 * @param[in]       window          : Duration (in 0.625 msec intervals). <b>BTM_DEFAULT_CONN_WINDOW</b>, or range: <b>0x0012 ~ 0x1000 </b> (11.25 ~ 2560 msecs)
 * @param[in]       interval        : Interval (in 0.625 msec intervals). <b>BTM_DEFAULT_CONN_INTERVAL</b>, or range: <b>0x0012 ~ 0x1000 </b> (11.25 ~ 2560 msecs)
 *
 * @return
 *
 * <b> WICED_BT_SUCCESS </b>       :  If successful \n
 * <b> WICED_BT_ILLEGAL_VALUE </b> :  If a bad parameter is detected \n
 * <b> WICED_BT_NO_RESOURCES </b>  :  If could not allocate a message buffer \n
 * <b> WICED_BT_WRONG_MODE </b>    :  If the device is not up
 */
wiced_result_t wiced_bt_dev_set_connectability (uint8_t page_mode, uint16_t window,
                                                      uint16_t interval);
/**
 *
 * Register callback for connection status change
 *
 *
 * @param[in]       p_wiced_bt_connection_status_change_cback - Callback for connection status change
 *
 * @return          wiced_result_t
 *
 * <b> WICED_BT_SUCCESS </b>  : on success; \n
 * <b> WICED_BT_FAILED </b>   : if an error occurred
 */
wiced_result_t wiced_bt_dev_register_connection_status_change(wiced_bt_connection_status_change_cback_t *p_wiced_bt_connection_status_change_cback);

/**
 *
 * Set a connection into sniff mode.
 *
 * @param[in]       remote_bda      : Link for which to put into sniff mode
 * @param[in]       min_period      : Minimum sniff period (range 0x0006 to 0x0540) (in 0.625 msec)
 * @param[in]       max_period      : Maximum sniff period (range 0x0006 to 0x0540) (in 0.625 msec)
 * @param[in]       attempt         : Number of attempts for switching to sniff mode (range 0x0001 – 0x7FFF) (in 0.625 msec)
 * @param[in]       timeout         : Timeout for attempting to switch to sniff mode (range 0x0000 – 0x7FFF) (in 0.625 msec)
 *
 * @return          <b> WICED_BT_PENDING </b> if successfully initiated, otherwise error
 */
wiced_result_t wiced_bt_dev_set_sniff_mode (wiced_bt_device_address_t remote_bda, uint16_t min_period,
                                             uint16_t max_period, uint16_t attempt,
                                             uint16_t timeout);


/**
 * Take a connection out of sniff mode.
 * A check is made if the connection is already in sniff mode,
 * and if not, the cancel sniff mode is ignored.
 *
 * @return          <b> WICED_BT_PENDING </b> if successfully initiated, otherwise error
 *
 */
wiced_result_t wiced_bt_dev_cancel_sniff_mode (wiced_bt_device_address_t remote_bda);


/**
 *
 * Set sniff subrating parameters for an active connection
 *
 * @param[in]       remote_bda          : device address of desired ACL connection
 * @param[in]       max_latency         : maximum latency (in 0.625ms units) (range: 0x0002-0xFFFE)
 * @param[in]       min_remote_timeout  : minimum remote timeout (in 0.625ms units) (range: 0x0000 – 0xFFFE)
 * @param[in]       min_local_timeout   : minimum local timeout (in 0.625ms units) (range: 0x0000 – 0xFFFE)
 *
 * @return
 *
 * <b> WICED_BT_SUCCESS </b>        : on success; \n
 * <b> WICED_BT_ILLEGAL_ACTION </b> : if an error occurred
 */
wiced_result_t wiced_bt_dev_set_sniff_subrating (wiced_bt_device_address_t remote_bda, uint16_t max_latency,
                              uint16_t min_remote_timeout, uint16_t min_local_timeout);

/**
 *
 * Get Receive Signal Strenth Index (RSSI) for the requested link
 *
 * @param[in]       remote_bda      : BD address of connection to read rssi
 * @param[in]       transport       : Transport type
 * @param[in]       p_cback         : Result callback (#wiced_bt_dev_rssi_result_t will be passed to the callback)
 *
 * @note For a BR/EDR Controller, A positive RSSI value shall indicate how many dB the RSSI is above the
 * upper limit, a negative value shall indicate how many dB the RSSI is below the
 * lower limit, and zero shall indicate that the RSSI is inside the range.
 * For an LE transport, the RSSI parameter returns the absolute received signal
 * strength value in dBm to ±6 dB accuracy.
 *
 * @return
 *
 * <b> WICED_BT_PENDING </b>      : if command issued to controller. \n
 * <b> WICED_BT_NO_RESOURCES </b> : if couldn't allocate memory to issue command \n
 * <b> WICED_BT_UNKNOWN_ADDR </b> : if no active link with bd addr specified \n
 * <b> WICED_BT_BUSY </b>         : if command is already in progress
 *
 */
wiced_result_t wiced_bt_dev_read_rssi (wiced_bt_device_address_t remote_bda, wiced_bt_transport_t transport, wiced_bt_dev_cmpl_cback_t *p_cback);

/**
 *
 * Write EIR data to controller.
 *
 * @param[in]       p_buff   : EIR data as per the spec (Spec 5.0 Vol 3 Part C, Section 8 )
 * @param[in]       len      : Total Length of EIR data being passed
 *
 * @return
 * <b> WICED_BT_SUCCESS </b>      : if successful \n
 * <b> WICED_BT_NO_RESOURCES </b> : if couldn't allocate memory to issue command \n
 * <b> WICED_BT_UNSUPPORTED </b>  : if local device cannot support request \n
 *
 */
wiced_result_t wiced_bt_dev_write_eir (uint8_t *p_buff, uint16_t len);

/**
 *
 * This function is called to switch the role between Central and
 * Peripheral.  If role is already set it will do nothing. If the
 * command was initiated, the callback function is called upon
 * completion.
 *
 * @param[in]       remote_bd_addr      : BD address of remote device
 * @param[in]       new_role            : New role (HCI_ROLE_CENTRAL or HCI_ROLE_PERIPHERAL)
 * @param[in]       p_cback             : Result callback (#wiced_bt_dev_switch_role_result_t will be passed to the callback)
 *
 * @return      wiced_result_t
 *
 */
wiced_result_t wiced_bt_dev_switch_role( wiced_bt_device_address_t remote_bd_addr, wiced_bt_dev_role_t new_role, wiced_bt_dev_cmpl_cback_t *p_cback );

/**
*
* This function is called to send HCI_SET_AFH_CHANNELS command
* to BR/EDR controller.
*
* Channel n is bad = 0.
* Channel n is unknown = 1.
* The most significant bit is reserved and shall be set to 0.
* At least 20 channels shall be marked as unknown.
*
* @param  afh_channel_map     : AFH Host Channel Classification array
*
* @return
* <b> WICED_BT_UNSUPPORTED </b>  : if feature does not supported \n
* <b> WICED_BT_WRONG_MODE </b>   : if device is in wrong mode \n
* <b> WICED_BT_NO_RESOURCES </b> : if device does not have buffers to process the request
*
*/
wiced_bt_dev_status_t wiced_bt_dev_set_afh_channel_classification(const wiced_bt_br_chnl_map_t afh_channel_map);

/**
*
* Get Bluetooth Friendly name from remote device.
*
* @param[in]       bd_addr  : Peer bd address
* @param[in]       p_remote_name_result_cback  : remote name result callback
*
* @return          wiced_result_t
*
* <b> WICED_BT_PENDING </b>      : if successfully initiated \n
* <b> WICED_BT_BUSY </b>         : if already in progress \n
* <b> WICED_BT_ILLEGAL_VALUE </b>: if parameter(s) are out of range \n
* <b> WICED_BT_NO_RESOURCES </b> : if could not allocate resources to start the command \n
* <b> WICED_BT_WRONG_MODE </b>   : if the device is not up.
**/
wiced_result_t  wiced_bt_dev_get_remote_name (wiced_bt_device_address_t bd_addr, wiced_bt_remote_name_cback_t *p_remote_name_result_cback);

/**
*
*  Retrieves the Class of Device of a peer BT device.
*
* @param[in]       bdaddr  : Peer bd address
* @param[out]      p_cod    : Class of Device of a peer BT device
*
* @return          wiced_result_t
*
* @Note : Below API applicable only for acceptor role. For Initiator role, please use EIR data to get the peer class of device.
**/
wiced_result_t wiced_bt_dev_get_device_class(wiced_bt_device_address_t bdaddr, wiced_bt_dev_class_t* p_cod);

/**
 * This function is called to set the Link Policy for remote device
 *
 * @param[in]       remote_bda      : remote device's address
 * @param[in]       settings        : the policy setting value(from #wiced_bt_link_policy_settings_values_e)
 *
 * @return          wiced_result_t
*/
wiced_result_t wiced_bt_dev_set_link_policy(wiced_bt_device_address_t remote_bda,
        wiced_bt_link_policy_settings_t settings);

/**
*
* Application can invoke this function to enable the coex functionality
*
* @param[in]       seci_baud_rate       : SECI baud rate. Ensure to set a valid baud rate which will be used
*                                         for the SECI communication between Bluetooth and WLAN chip. Maximum supported
*                                         value is up to 4M
*
* @return          wiced_result_t
*
**/
wiced_result_t wiced_bt_coex_enable( uint32_t seci_baud_rate );

/**
 *
 * Application can invoke this function to disable the coex functionality
 *
 * @return         void
 *
 */
void wiced_bt_coex_disable( void );

/**
 *
 * Application can invoke this function to change the device name in Controller.
 *
 * @return         void
 *
 */
wiced_result_t wiced_bt_btm_set_device_name(char* p_name);

/**
*
* This function is called to set the channel assessment mode on or off
*
* @param[in]       enable_or_disable :  Enable or disable AFH channel assessment
*
* @return          wiced_result_t
*
**/
wiced_result_t wiced_bt_dev_set_afh_channel_assessment(wiced_bool_t enable_or_disable);

/**
 *
 * This function is called to set the packet types used for
 * a specific SCO connection and for all connections.
 *
 * @param[in]       sco_inx: Specific connection and -1 for all connections.
 * @param[in]       pkt_types: One or more of the following \n
 * <b> BTM_SCO_PKT_TYPES_MASK_HV1 \n
 *  BTM_SCO_PKT_TYPES_MASK_HV2 \n
 *  BTM_SCO_PKT_TYPES_MASK_HV3 \n
 *  BTM_SCO_PKT_TYPES_MASK_EV3 \n
 *  BTM_SCO_PKT_TYPES_MASK_EV4 \n
 *  BTM_SCO_PKT_TYPES_MASK_EV5 \n
 *  BTM_SCO_PKT_TYPES_MASK_NO_2_EV3 \n
 *  BTM_SCO_PKT_TYPES_MASK_NO_3_EV3 \n
 *  BTM_SCO_PKT_TYPES_MASK_NO_2_EV5 \n
 *  BTM_SCO_PKT_TYPES_MASK_NO_3_EV5 \n
 *  BTM_SCO_LINK_ALL_MASK </b>   - enables all supported types
 *
 * @return          wiced_bt_dev_status_t
 */
wiced_bt_dev_status_t wiced_bt_sco_setPacketTypes(int16_t sco_inx, uint16_t pkt_types);

/**
 * This function is set the ACL packet types that
 * the device supports for specific connection and all connections if bd address with all zeros.
 *
 * @param[in]     remote_bda: BD Address for specific conection and for all connections it should be all 0's.
 * @param[in]     pkt_types: Packet types supported by the device.
 *                           One or more of the following (bitmask): \n
 * <b> BTM_ACL_PKT_TYPES_MASK_DM1 \n
 * BTM_ACL_PKT_TYPES_MASK_DH1 \n
 * BTM_ACL_PKT_TYPES_MASK_DM3 \n
 * BTM_ACL_PKT_TYPES_MASK_DH3 \n
 * BTM_ACL_PKT_TYPES_MASK_DM5 \n
 * BTM_ACL_PKT_TYPES_MASK_DH5 \n
 * BTM_ACL_PKT_TYPES_MASK_NO_2_DH1 \n
 * BTM_ACL_PKT_TYPES_MASK_NO_3_DH1 \n
 * BTM_ACL_PKT_TYPES_MASK_NO_2_DH3 \n
 * BTM_ACL_PKT_TYPES_MASK_NO_3_DH3 \n
 * BTM_ACL_PKT_TYPES_MASK_NO_2_DH5 </b>
 *
 * @return          wiced_result_t
 */
wiced_result_t wiced_bt_dev_setAclPacketTypes(wiced_bt_device_address_t remote_bda, uint16_t pkt_types);

/**
* @}
*/

/**
 *
 * This module provided various Bluetooth BR/EDR security functionality such as authorisation, authentication and encryption.
 *
 * @addtogroup  br_edr_sec_api_functions        BR/EDR Security Function
 * @ingroup     wicedbt_bredr
 *
 * @note General Security APIs are listed in \ref ble_common_sec_api_functions section.
 * @{
 */

/**
 * PIN code reply (use in response to <b>BTM_PIN_REQUEST_EVT </b> in #wiced_bt_management_cback_t)
 *
 *  @param[in]      bd_addr     : Address of the device for which PIN was requested
 *  @param[in]      res         : result of the operation WICED_BT_SUCCESS if success
 *  @param[in]      pin_len     : length in bytes of the PIN Code
 *  @param[in]      p_pin       : pointer to array with the PIN Code
 *
 * @return          void
 * @note            BR/EDR Only
 */
void wiced_bt_dev_pin_code_reply (wiced_bt_device_address_t bd_addr, wiced_result_t res, uint8_t pin_len, uint8_t *p_pin);

/**
 *
 * Provide the pairing passkey (in response to <b>BTM_PASSKEY_REQUEST_EVT </b> of #wiced_bt_management_cback_t)
 *
 * @param[in]       res           : result of the operation WICED_BT_SUCCESS if success
 * @param[in]       bd_addr       : Address of the peer device
 * @param[in]       passkey       : numeric value in the range of 0 - 999999(0xF423F).
 *
 * @return          void
 * @note            BR/EDR only
 */
void wiced_bt_dev_pass_key_req_reply(wiced_result_t res, wiced_bt_device_address_t bd_addr, uint32_t passkey);

/**
 *
 * Read the local OOB data from controller (for sending
 * to peer device over oob message). When
 * operation is completed, local OOB data will be
 * provided via #BTM_READ_LOCAL_OOB_DATA_COMPLETE_EVT.
 *
 * @note            BR/EDR Only
 */
wiced_result_t wiced_bt_dev_read_local_oob_data(void);

/**
 *
 * Provide the remote OOB extended data for Simple Pairing
 * in response to #BTM_REMOTE_OOB_DATA_REQUEST_EVT
 *
 * @param[in]       res                     : response reply
 * @param[in]       bd_addr                 : Address of the peer device
 * @param[in]       is_extended_oob_data    : TRUE if extended OOB data (set according to #BTM_REMOTE_OOB_DATA_REQUEST_EVT request)
 * @param[in]       c_192                   : simple pairing Hash C derived from the P-192 public key.
 * @param[in]       r_192                   : simple pairing Randomizer R associated with the P-192 public key.
 * @param[in]       c_256                   : simple pairing Hash C derived from the P-256 public key (if is_extended_oob_data=TRUE)
 * @param[in]       r_256                   : simple pairing Randomizer R associated with the P-256 public key (if is_extended_oob_data=TRUE)
 *
 * @note            BR/EDR Only
 */
void wiced_bt_dev_remote_oob_data_reply (wiced_result_t res, wiced_bt_device_address_t bd_addr,
                                              wiced_bool_t is_extended_oob_data,
                                              BT_OCTET16 c_192, BT_OCTET16 r_192,
                                              BT_OCTET16 c_256, BT_OCTET16 r_256);

/**
 * Build the OOB data block to be used to send OOB extended
 * data over OOB (non-Bluetooth) link.
 *
 * @param[out]      p_data                  : OOB data block location
 * @param[in]       max_len                 : OOB data block size
 * @param[in]       is_extended_oob_data    : TRUE if extended OOB data (for Secure Connections)
 * @param[in]       c_192                   : simple pairing Hash C derived from the P-192 public key.
 * @param[in]       r_192                   : simple pairing Randomizer R associated with the P-192 public key.
 * @param[in]       c_256                   : simple pairing Hash C derived from the P-256 public key (if is_extended_oob_data=TRUE)
 * @param[in]       r_256                   : simple pairing Randomizer R associated with the P-256 public key (if is_extended_oob_data=TRUE)
 *
 * @return          Number of bytes put into OOB data block.
 *
 * @note            BR/EDR Only
 */
uint16_t wiced_bt_dev_build_oob_data(uint8_t *p_data, uint16_t max_len,
                                          wiced_bool_t is_extended_oob_data,
                                          BT_OCTET16 c_192, BT_OCTET16 r_192,
                                          BT_OCTET16 c_256, BT_OCTET16 r_256);

/**
 *
 * This function is called to provide the OOB data for
 * SMP in response to #BTM_SMP_REMOTE_OOB_DATA_REQUEST_EVT
 *
 *  @param[in]      bd_addr     - Address of the peer device
 *  @param[in]      res         - result of the operation WICED_BT_SUCCESS if success
 *  @param[in]      len         - oob data length
 *  @param[in]      p_data      - oob data
 *
 * @note            BR/EDR Only
 */
void wiced_bt_smp_oob_data_reply(wiced_bt_device_address_t bd_addr, wiced_result_t res, uint8_t len, uint8_t *p_data);

/**
 *
 * Provide the SC OOB data for SMP in response to
 * #BTM_SMP_SC_REMOTE_OOB_DATA_REQUEST_EVT
 *
 * @param[in]       p_oob_data  : oob data
 *
 */
void wiced_bt_smp_sc_oob_reply (wiced_bt_smp_sc_oob_data_t *p_oob_data);

/**
 *
 * This function is called to parse the OOB data payload
 * received over OOB (non-Bluetooth) link
 *
 * @param[in]       p_data  : oob data
 * @param[in]       eir_tag : EIR Data type( version5.0, Volume 3, Part C Section 5.2.2.7 )
 * @param[out]      p_len   : the length of the data with the given EIR Data type
 *
 * @return          The beginning of the data with the given EIR Data type.
 *                  NULL, if the tag is not found.
 * @note            BR/EDR Only
 */
uint8_t* wiced_bt_dev_read_oob_data(uint8_t *p_data, uint8_t eir_tag, uint8_t *p_len);

/**
 *
 * Disable Bluetooth secure connection
 *
 * @note This utility is used for LRAC application to disable the Bluetooth secure connection only.
 *       If the interference issue is fixed, this utility may be removed
 *        This utility shall be called before the Bluetooth stack is initialized
 *        (by calling app_bt_init()).
 */
void wiced_bt_dev_lrac_disable_secure_connection(void);

/**
 *@}
 *@} wicedbt_DeviceManagement
 *
 */
/* @endcond*/

/****************************************************************************/
/**
 * This sections provides Bluetooth utilities functions related to trace, local bda, tx power etc.
 *
 * @addtogroup  wicedbt_utility    Utilities
 * @ingroup     wicedbt_DeviceManagement
 *
 * @{
 */
/****************************************************************************/

/**
 *
 * Register to get the hci traces
 *
 * @param[in]      p_cback        : Callback for hci traces
 *
 * @return          void
 *
 */
void wiced_bt_dev_register_hci_trace( wiced_bt_hci_trace_cback_t* p_cback );

/**
 *
 * Update the hci trace mode
 *
 * @param[in]      enable    : TRUE to enable HCI traces, FALSE to disable
 *
 * @return          void
 *
 */
void wiced_bt_dev_update_hci_trace_mode(wiced_bool_t enable);

/**
 *
 * Update the debug trace mode
 *
 * @param[in]      enable    : TRUE to enable debug traces, FALSE to disable
 *
 * @return          void
 *
 */
void wiced_bt_dev_update_debug_trace_mode(wiced_bool_t enable);

/**
 * Set Local Bluetooth Device Address.
 * The application has to set a valid address (Static/Random) by calling this function.\n
 * If this function is not called, the default address is typically a controller assigned address(Bluetooth device part number),
 * which is same for particular device type.For example, all CYW43012 devcies will typically have the same default address.
 *
 * The application can set a non-resolvable or static random address by setting the addr_type to \ref BLE_ADDR_RANDOM.
 * For non-resolvable addresses the top two bits of the bd_addr shall be set to 'b00' and atleast one of the remaining 46 shall be non-zero.
 * For static random addresses the top two bits of the bd_addr are required to be 'b11' and atleast one of the remaining 46 shall be non-zero.
 *
 * @param[in]       bd_addr    : device address to use
 * @note bd_addr[0] is the MSB,
 *       for non-resolvable address, set bd_addr[0] &= ~0xC0
 *       for static random address,  set bd_addr[0] |= 0xC0
 * @param[in]       addr_type  : device address type , should be BLE_ADDR_RANDOM or BLE_ADDR_PUBLIC \n
 *
 * @return          wiced_result_t
 *
 * WICED_BT_ILLEGAL_VALUE : if invalid device address specified \n
 * WICED_BT_NO_RESOURCES  : if couldn't allocate memory to issue command \n
 * WICED_BT_SUCCESS       : if local bdaddr is set successfully \n
 *
 * @note            BD_Address must be in Big Endian format
 *
 * Example:
 * <PRE>    Data         | AB | CD | EF | 01 | 23 | 45 | </PRE>
 * <PRE>    Address      | 0  | 1  | 2  | 3  | 4  | 5  | </PRE>
 *                       For above example it will set AB:CD:EF:01:23:45 bd address
 */
wiced_result_t wiced_bt_set_local_bdaddr( wiced_bt_device_address_t  bd_addr , wiced_bt_ble_address_type_t addr_type);

/**
 *
 * This function is called to get the role of the local device
 * for the ACL connection with the specified remote device
 *
 * @param[in]       remote_bd_addr      : BD address of remote device
 * @param[in]       transport               : BT_TRANSPORT_BR_EDR or BT_TRANSPORT_LE
 *
 * @param[out]     p_role       : Role of the local device
 *
 * @return      WICED_BT_UNKNOWN_ADDR if no active link with bd addr specified
 *
 */
wiced_result_t wiced_bt_dev_get_role(wiced_bt_device_address_t remote_bd_addr,
                                     wiced_bt_dev_role_t      *p_role,
                                     wiced_bt_transport_t      transport);
/**
*
* Enable or disable pairing
*
* @param[in]       allow_pairing        : (TRUE or FALSE) whether or not the device allows pairing.
* @param[in]       connect_only_paired   : (TRUE or FALSE) whether or not to only allow paired devices to connect.
*                                         <b> Applicable only for BR/EDR </b>
*
* @return          void
*
*/
void wiced_bt_set_pairable_mode(uint8_t allow_pairing, uint8_t connect_only_paired);

/**
 *
 * Application can register Vendor-Specific HCI event callback
 *
 * @param[in]      cb       : callback function to register
 *
 * @return         WICED_SUCCESS
 *                 WICED_ERROR if out of usage
 */
wiced_result_t wiced_bt_dev_register_vse_callback(wiced_bt_dev_vse_callback_t cb);

/**
 *
 * Application can deregister Vendor-Specific HCI event callback
 *
 * @param[in]      cb       : callback function to deregister
 *
 * @return         WICED_SUCCESS
 *                 WICED_ERROR if the input callback function was not registered yet
 */
wiced_result_t wiced_bt_dev_deregister_vse_callback(wiced_bt_dev_vse_callback_t cb);

/**
 *
 * This API is called to get the statistics for an ACL link
 *
 * Limitation       This API works when there is only one ACL connection
 *
 * @param[in]       bda               : bluetooth device address of desired link quality statistics
 * @param[in]       transport         : Tranport type LE/BR-EDR
 * @param[in]       action            : WICED_CLEAR_LINK_QUALITY_STATS = reset the link quality statistics to 0,
 *                                                  WICED_READ_LINK_QUALITY_STATS = read link quality statistics,
 *                                                  WICED_READ_THEN_CLEAR_LINK_QUALITY_STATS = read link quality statistics, then clear it
 * @param[in]       p_cback           : Result callback (#wiced_bt_dev_cmpl_cback_t will be passed to the callback)
 *
 * @return
 *
 * <b> WICED_BT_SUCCESS </b>      : If successful \n
 * <b> WICED_BT_PENDING </b>      : If command succesfully sent down \n
 * <b> WICED_BT_BUSY </b>         : If already in progress \n
 * <b> WICED_BT_NO_RESORCES </b>  : If no memory/buffers available to sent down to controller \n
 * <b> WICED_BT_UNKNOWN_ADDR </b> : If given BD_ADDRESS is invalid \n
 *
 * @note   Callback function argument is a pointer of type wiced_bt_lq_stats_result_t
 *
 *
 */
wiced_bt_dev_status_t wiced_bt_dev_link_quality_stats(wiced_bt_device_address_t bda, wiced_bt_transport_t transport,
                uint8_t action, wiced_bt_dev_cmpl_cback_t *p_cback);

#ifdef USE_WICED_HCI
/**
 * MCU host push all the saved NVRAM informatoin mainly paired device Info
 *
 * @param[in]       paired_device_info : Remote device address, Link key
 *
 * @return          WICED_BT_SUCCESS if successful
 *
 */
wiced_result_t wiced_bt_dev_push_nvram_data(wiced_bt_device_link_keys_t *paired_device_info);
#endif

/**@} wicedbt_utility */

/**
 * @if DUAL_MODE
 * Bluetooth generic security API.
 *
 * @addtogroup  ble_common_sec_api_functions        Generic Security API
 * @ingroup     br_edr_sec_api_functions
 * @ingroup     btm_ble_sec_api_functions
 * @else
 * Bluetooth LE Security Functions.
 * @ingroup  btm_ble_sec_api_functions
 * @endif
 * @{
 */

/**
 *
 * Bond with peer device. If the connection is already up, but not secure, pairing is attempted.
 *
 *  @note           PIN parameters are only needed when bonding with legacy devices (pre-2.1 Core Spec)
 *
 *  @param[in]      bd_addr         : Peer device bd address to pair with.
 *  @param[in]      bd_addr_type    : BLE_ADDR_PUBLIC or BLE_ADDR_RANDOM (applies to LE devices only)
 *  @param[in]      transport       : BT_TRANSPORT_BR_EDR or BT_TRANSPORT_LE
 *  @param[in]      pin_len         : Length of input parameter p_pin (0 if not used).
 *  @param[in]      p_pin           : Pointer to Pin Code to use (NULL if not used).
 *
 *  @return
 *
 * <b> WICED_BT_PENDING </b> : if successfully initiated, \n
 * <b> WICED_BT_SUCCESS </b> : if already paired to the device, else error code
 */
wiced_result_t wiced_bt_dev_sec_bond (wiced_bt_device_address_t bd_addr, wiced_bt_ble_address_type_t bd_addr_type, wiced_bt_transport_t transport, uint8_t pin_len, uint8_t *p_pin);

/**
 *
 * Pair with peer device(dont store the keys). If the connection is already up, but not secure, pairing is attempted.
 *
 *  @note           PIN parameters are only needed when bonding with legacy devices (pre-2.1 Core Spec)
 *
 *  @param[in]      bd_addr         : Peer device bd address to pair with.
 *  @param[in]      bd_addr_type    : BLE_ADDR_PUBLIC or BLE_ADDR_RANDOM (applies to LE devices only)
 *  @param[in]      transport       : BT_TRANSPORT_BR_EDR or BT_TRANSPORT_LE
 *  @param[in]      pin_len         : Length of input parameter p_pin (0 if not used).
 *  @param[in]      p_pin           : Pointer to Pin Code to use (NULL if not used).
 *
 *  @return
 *
 * <b> WICED_BT_PENDING </b> : if successfully initiated, \n
 * <b> WICED_BT_SUCCESS </b> : if already paired to the device, else error code
 */
wiced_result_t wiced_bt_dev_sec_pair_without_bonding(wiced_bt_device_address_t bd_addr,
                                                     wiced_bt_ble_address_type_t bd_addr_type,
                                                     wiced_bt_transport_t transport,
                                                     uint8_t pin_len,
                                                     uint8_t *p_pin);

/**
 *
 * Cancel an ongoing bonding process with peer device.
 *
 *  @param[in]      bd_addr         : Peer device bd address to pair with.
 *
 *  @return
 *
 * <b> WICED_BT_PENDING </b> : if cancel initiated, \n
 * <b> WICED_BT_SUCCESS </b> : if cancel has completed already, else error code.
 */
wiced_result_t wiced_bt_dev_sec_bond_cancel (wiced_bt_device_address_t bd_addr);


/**
 * Encrypt the specified connection.
 *
 *  @param[in]      bd_addr         : Address of peer device
 *  @param[in]      transport       : BT_TRANSPORT_BR_EDR or BT_TRANSPORT_LE
 *  @param[in]      p_ref_data      : Encryption type \ref wiced_bt_ble_sec_action_type_t
 *
 * @return
 *
 * <b> WICED_BT_SUCCESS </b>     : already encrypted \n
 * <b> WICED_BT_PENDING </b>     : Status is notified using <b>BTM_ENCRYPTION_STATUS_EVT </b> of #wiced_bt_management_cback_t.\n
 * <b> WICED_BT_WRONG_MODE </b>  : connection not up. \n
 * <b> WICED_BT_BUSY </b>        : security procedures are currently active
 */
wiced_result_t wiced_bt_dev_set_encryption (wiced_bt_device_address_t bd_addr, wiced_bt_transport_t transport, void *p_ref_data);


/**
 *
 * Confirm the numeric value for pairing (in response to <b>BTM_USER_CONFIRMATION_REQUEST_EVT </b> of #wiced_bt_management_cback_t)
 *
 * @param[in]       res           : result of the operation WICED_BT_SUCCESS if success
 * @param[in]       bd_addr       : Address of the peer device
 *
 * @return          void
 */
void wiced_bt_dev_confirm_req_reply(wiced_result_t res, wiced_bt_device_address_t bd_addr);

/**
 *
 * Inform remote device of keypress during pairing.
 *
 * Used during the passkey entry by a device with KeyboardOnly IO capabilities
 * (typically a HID keyboard device).
 *
 * @param[in]       bd_addr : Address of the peer device
 * @param[in]       type    : notification type
 *
 */
void wiced_bt_dev_send_key_press_notif(wiced_bt_device_address_t bd_addr, wiced_bt_dev_passkey_entry_type_t type);

/**
 * remove bonding with remote device with assigned bd_addr
 * @deprecated Stack does not store any bonding information. For applications which use BLE mode invoke
 * \ref wiced_bt_dev_remove_device_from_address_resolution_db to remove the device from the resolving list.
 *
 * @param[in] bd_addr : bd_addr of remote device to be removed from bonding list
 *
 * @return Returns WICED_SUCCESS
 *
 */
wiced_result_t wiced_bt_dev_delete_bonded_device(wiced_bt_device_address_t bd_addr);

/**
 *
 * Get security flags for the device
 *
 * @param[in]       bd_addr         : peer address
 * @param[out]      p_sec_flags  : security flags (see #wiced_bt_sec_flags_e)
 *
 * @return          TRUE if successful
 *
 */
wiced_bool_t wiced_bt_dev_get_security_state(wiced_bt_device_address_t bd_addr, uint8_t *p_sec_flags);

/**
* @}
*/

/**
 * @addtogroup  btm_ble_sec_api_functions        LE Security
 * @if DUAL_MODE
 * @ingroup     btm_ble_api_functions
 *
 * Bluetooth LE security API (authorisation, authentication and encryption)
 *
 * @note General Security APIs are listed in \ref ble_common_sec_api_functions section.
 * @else
 * LE Security API.
 * @ingroup   wicedbt_DeviceManagement
 * @endif
 * @{
 */

/**
 *
 * add link key information to internal address resolution db
 *
 * @param[in]      p_link_keys    : link keys information stored in application side
 *
 * @return          wiced_result_t
 *
 */
wiced_result_t wiced_bt_dev_add_device_to_address_resolution_db(wiced_bt_device_link_keys_t *p_link_keys);


/**
 *
 * remove link key information from internal address resolution db
 *
 * @param[in]      p_link_keys    : link keys information stored in application side
 *
 * @return          wiced_result_t
 *
 */
wiced_result_t wiced_bt_dev_remove_device_from_address_resolution_db(wiced_bt_device_link_keys_t *p_link_keys);

/**
 * get the acl connection handle for bdaddr
 *
 * @param[in] bdaddr: device identity address
 * @param[in] transport: connection transport
 *
 * @return : acl connection handle
 */
uint16_t wiced_bt_dev_get_acl_conn_handle(wiced_bt_device_address_t bdaddr, wiced_bt_transport_t transport);

/**@} btm_ble_sec_api_functions */

/** @cond DUAL_MODE */
/**
 * Quality of Service (QoS) setup
 *
 * @param[in] remote_bda: peer device bdaddr
 * @param[in] p_flow: qos setup parameters
 * @param[in] p_cb: callback
 *
 * @return wiced_result_t
 */
wiced_result_t wiced_bt_dev_qos_setup_by_bda(wiced_bt_device_address_t remote_bda,
                                             wiced_bt_flow_spec_t *p_flow,
                                             wiced_bt_dev_cmpl_cback_t *p_cb);

/**
 * Quality of Service (QoS) setup
 *
 * @param[in] connection_handle: acl connection handle
 * @param[in] p_flow: qos setup parameters
 * @param[in] p_cb: callback
 *
 * @return wiced_result_t
 */
wiced_result_t wiced_bt_dev_qos_setup_by_handle(uint16_t connection_handle,
                                                wiced_bt_flow_spec_t *p_flow,
                                                wiced_bt_dev_cmpl_cback_t *p_cb);
/**
 * @endcond // DUAL_MODE
*/

#ifdef __cplusplus
}
#endif

#endif //__WICED_BT_DEV_H__
