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
#ifndef __WICED_BT_ADV_SCAN_COMMON_H__
#define __WICED_BT_ADV_SCAN_COMMON_H__

#include "wiced_bt_cfg.h"
#include "wiced_bt_dev.h"

/**
 * This section contains some of the common defines and structures used for LE advertising,
 * scanning, and link connection management.
 *
 * @addtogroup  wicedbt_Common   Common LE Advertisement, Scan and Connection defines
 *
 * @ingroup     wicedbt
 *
 * @{
 */


/** default advertising channel map */
#ifndef BTM_BLE_DEFAULT_ADVERT_CHNL_MAP
#define BTM_BLE_DEFAULT_ADVERT_CHNL_MAP (BTM_BLE_ADVERT_CHNL_37 | BTM_BLE_ADVERT_CHNL_38 | BTM_BLE_ADVERT_CHNL_39)
#endif

/** Advertising filter policy */
enum wiced_bt_ble_advert_filter_policy_e
{
    /**< Process scan and connection requests from all devices (i.e., the Filter Accept List is not in use) (default) */
    BTM_BLE_ADV_POLICY_ACCEPT_CONN_AND_SCAN = 0x00,
    /**< Process connection requests from all devices and only scan requests from devices that are in the Filter Accept List. */
    BTM_BLE_ADV_POLICY_ACCEPT_CONN_FILTER_SCAN = 0x01,
    /**< Process scan requests from all devices and only connection requests from devices that are in the Filter Accept List */
    BTM_BLE_ADV_POLICY_FILTER_CONN_ACCEPT_SCAN = 0x02,
    /**< Process scan and connection requests only from devices in the Filter Accept List. */
    BTM_BLE_ADV_POLICY_FILTER_CONN_FILTER_SCAN = 0x03,
    /**< Max Adv filter value */
    BTM_BLE_ADV_POLICY_MAX
};
/** Advertising filter policy (see wiced_bt_ble_advert_filter_policy_e) */
typedef uint8_t wiced_bt_ble_advert_filter_policy_t;

/** default advertising filter policy */
#define BTM_BLE_ADVERT_FILTER_DEFAULT BTM_BLE_ADV_POLICY_ACCEPT_CONN_AND_SCAN

#define BTM_BLE_ADVERT_INTERVAL_MIN 0x0020 /**< adv parameter Min value */
#define BTM_BLE_ADVERT_INTERVAL_MAX 0x4000 /**< adv parameter Max value */

#define BTM_BLE_SCAN_INTERVAL_MIN 0x0004 /**< Scan interval minimum value */
#define BTM_BLE_SCAN_INTERVAL_MAX 0x4000 /**< Scan interval miximum value */
#define BTM_BLE_SCAN_WINDOW_MIN 0x0004   /**< Scan window minimum value */
#define BTM_BLE_SCAN_WINDOW_MAX 0x4000   /**< Scan window maximum value */
#define BTM_BLE_CONN_INTERVAL_MIN 0x0006 /**< Connection interval minimum value */
#define BTM_BLE_CONN_INTERVAL_MAX 0x0C80 /**< Connection interval maximum value */
#define BTM_BLE_CONN_LATENCY_MAX 500     /**< Maximum Connection Latency */
#define BTM_BLE_CONN_SUP_TOUT_MIN 0x000A /**< Minimum Supervision Timeout */
#define BTM_BLE_CONN_SUP_TOUT_MAX 0x0C80 /**< Maximum Supervision Timeout */
#define BTM_BLE_CONN_PARAM_UNDEF 0xffff  /**< use this value when a specific value not to be overwritten */
#define BTM_BLE_CONN_SUP_TOUT_DEF 700    /**< Default Supervision Timeout */

/* default connection parameters if not configured, use GAP recommend value for auto connection */
/** default scan interval
 *  30 ~ 60 ms (use 60)  = 96 *0.625
 */
#define BTM_BLE_SCAN_FAST_INTERVAL 96

/** default scan window (in .625ms slots) for background auto connections
 * 30 ms = 48 *0.625
 */
#define BTM_BLE_SCAN_FAST_WINDOW 48

/** default scan interval used in reduced power cycle (background scanning)
 *  1.28 s   = 2048 *0.625
 */
#define BTM_BLE_SCAN_SLOW_INTERVAL_1 2048

/** default scan window used in reduced power cycle (background scanning)
 *   11.25 ms = 18 *0.625
 */
#define BTM_BLE_SCAN_SLOW_WINDOW_1 18

/** default scan interval used in reduced power cycle (background scanning)
 *  2.56 s   = 4096 *0.625
 */
#define BTM_BLE_SCAN_SLOW_INTERVAL_2 4096

/** default scan window used in reduced power cycle (background scanning)
 *  22.5 ms = 36 *0.625
 */
#define BTM_BLE_SCAN_SLOW_WINDOW_2 36

#define BTM_BLE_POLICY_REJECT_ALL 0x00 /**< relevant to both */
#define BTM_BLE_POLICY_ALLOW_SCAN 0x01 /**< relevant to advertiser */
#define BTM_BLE_POLICY_ALLOW_CONN 0x02 /**< relevant to advertiser */
#define BTM_BLE_POLICY_ALLOW_ALL 0x03  /**< relevant to both */

/* ADV data flag bit definition used for BTM_BLE_ADVERT_TYPE_FLAG */
#define BTM_BLE_LIMITED_DISCOVERABLE_FLAG (0x01 << 0) /**< Limited Discoverable */
#define BTM_BLE_GENERAL_DISCOVERABLE_FLAG (0x01 << 1) /**< General Discoverable */
#define BTM_BLE_BREDR_NOT_SUPPORTED (0x01 << 2)       /**< BR/EDR Not Supported */
/* 4.1 spec adv flag for simultaneous BR/EDR+LE connection support (see) */
#define BTM_BLE_SIMULTANEOUS_DUAL_MODE_TO_SAME_DEVICE_CONTROLLER_SUPPORTED                                             \
    (0x01 << 3) /**< Simultaneous LE and BR/EDR to Same Device Capable (Controller). */
#define BTM_BLE_SIMULTANEOUS_DUAL_MODE_TO_SAME_DEVICE_HOST_SUPPORTED                                                   \
    (0x01 << 4)                                      /**< Simultaneous LE and BR/EDR to Same Device Capable (Host). */
#define BTM_BLE_NON_LIMITED_DISCOVERABLE_FLAG (0x00) /**< Non Discoverable */
#define BTM_BLE_ADVERT_FLAG_MASK                                                                                       \
    (BTM_BLE_LIMITED_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED |                                                 \
     BTM_BLE_GENERAL_DISCOVERABLE_FLAG)                                       /**< LE adverisement mask */
#define BTM_BLE_LIMITED_DISCOVERABLE_MASK (BTM_BLE_LIMITED_DISCOVERABLE_FLAG) /**< LE Limited discovery mask*/


/**
 * Advertisement data types
 * */
enum wiced_bt_ble_advert_type_e
{
    BTM_BLE_ADVERT_TYPE_FLAG = 0x01,                     /**< Advertisement flags */
    BTM_BLE_ADVERT_TYPE_16SRV_PARTIAL = 0x02,            /**< List of supported services - 16 bit UUIDs (partial) */
    BTM_BLE_ADVERT_TYPE_16SRV_COMPLETE = 0x03,           /**< List of supported services - 16 bit UUIDs (complete) */
    BTM_BLE_ADVERT_TYPE_32SRV_PARTIAL = 0x04,            /**< List of supported services - 32 bit UUIDs (partial) */
    BTM_BLE_ADVERT_TYPE_32SRV_COMPLETE = 0x05,           /**< List of supported services - 32 bit UUIDs (complete) */
    BTM_BLE_ADVERT_TYPE_128SRV_PARTIAL = 0x06,           /**< List of supported services - 128 bit UUIDs (partial) */
    BTM_BLE_ADVERT_TYPE_128SRV_COMPLETE = 0x07,          /**< List of supported services - 128 bit UUIDs (complete) */
    BTM_BLE_ADVERT_TYPE_NAME_SHORT = 0x08,               /**< Short name */
    BTM_BLE_ADVERT_TYPE_NAME_COMPLETE = 0x09,            /**< Complete name */
    BTM_BLE_ADVERT_TYPE_TX_POWER = 0x0A,                 /**< TX Power level  */
    BTM_BLE_ADVERT_TYPE_DEV_CLASS = 0x0D,                /**< Device Class */
    BTM_BLE_ADVERT_TYPE_SIMPLE_PAIRING_HASH_C = 0x0E,    /**< Simple Pairing Hash C */
    BTM_BLE_ADVERT_TYPE_SIMPLE_PAIRING_RAND_C = 0x0F,    /**< Simple Pairing Randomizer R */
    BTM_BLE_ADVERT_TYPE_SM_TK = 0x10,                    /**< Security manager TK value */
    BTM_BLE_ADVERT_TYPE_SM_OOB_FLAG = 0x11,              /**< Security manager Out-of-Band data */
    BTM_BLE_ADVERT_TYPE_INTERVAL_RANGE = 0x12,           /**< Peripheral connection interval range */
    BTM_BLE_ADVERT_TYPE_SOLICITATION_SRV_UUID = 0x14,    /**< List of solicitated services - 16 bit UUIDs */
    BTM_BLE_ADVERT_TYPE_128SOLICITATION_SRV_UUID = 0x15, /**< List of solicitated services - 128 bit UUIDs */
    BTM_BLE_ADVERT_TYPE_SERVICE_DATA = 0x16,             /**< Service data - 16 bit UUID */
    BTM_BLE_ADVERT_TYPE_PUBLIC_TARGET = 0x17,            /**< Public target address */
    BTM_BLE_ADVERT_TYPE_RANDOM_TARGET = 0x18,            /**< Random target address */
    BTM_BLE_ADVERT_TYPE_APPEARANCE = 0x19,               /**< Appearance */
    BTM_BLE_ADVERT_TYPE_ADVERT_INTERVAL = 0x1a,          /**< Advertising interval */
    BTM_BLE_ADVERT_TYPE_LE_BD_ADDR = 0x1b,               /**< LE device bluetooth address */
    BTM_BLE_ADVERT_TYPE_LE_ROLE = 0x1c,                  /**< LE role */
    BTM_BLE_ADVERT_TYPE_256SIMPLE_PAIRING_HASH = 0x1d,   /**< Simple Pairing Hash C-256 */
    BTM_BLE_ADVERT_TYPE_256SIMPLE_PAIRING_RAND = 0x1e,   /**< Simple Pairing Randomizer R-256 */
    BTM_BLE_ADVERT_TYPE_32SOLICITATION_SRV_UUID = 0x1f,  /**< List of solicitated services - 32 bit UUIDs */
    BTM_BLE_ADVERT_TYPE_32SERVICE_DATA = 0x20,           /**< Service data - 32 bit UUID */
    BTM_BLE_ADVERT_TYPE_128SERVICE_DATA = 0x21,          /**< Service data - 128 bit UUID */
    BTM_BLE_ADVERT_TYPE_CONN_CONFIRM_VAL = 0x22,         /**< LE Secure Connections Confirmation Value */
    BTM_BLE_ADVERT_TYPE_CONN_RAND_VAL = 0x23,            /**< LE Secure Connections Random Value */
    BTM_BLE_ADVERT_TYPE_URI = 0x24,                      /**< URI */
    BTM_BLE_ADVERT_TYPE_INDOOR_POS = 0x25,               /**< Indoor Positioning */
    BTM_BLE_ADVERT_TYPE_TRANS_DISCOVER_DATA = 0x26,      /**< Transport Discovery Data */
    BTM_BLE_ADVERT_TYPE_SUPPORTED_FEATURES = 0x27,       /**< LE Supported Features */
    BTM_BLE_ADVERT_TYPE_UPDATE_CH_MAP_IND = 0x28,        /**< Channel Map Update Indication */
    BTM_BLE_ADVERT_TYPE_PB_ADV = 0x29,                   /**< PB-ADV */
    BTM_BLE_ADVERT_TYPE_MESH_MSG = 0x2A,                 /**< Mesh Message */
    BTM_BLE_ADVERT_TYPE_MESH_BEACON = 0x2B,              /**< Mesh Beacon */
    BTM_BLE_ADVERT_TYPE_PSRI = 0x2E,                     /**< Generic Audio Provate Set Random Identifier */
    BTM_BLE_ADVERT_TYPE_EAD = 0x31,                      /**< Encrypted Advertising Data */
    BTM_BLE_ADVERT_TYPE_3D_INFO_DATA = 0x3D,             /**< 3D Information Data */
    BTM_BLE_ADVERT_TYPE_MANUFACTURER = 0xFF              /**< Manufacturer data */
};
/** LE advertisement data type (see #wiced_bt_ble_advert_type_e) */
typedef uint8_t wiced_bt_ble_advert_type_t;

/** Handle value of the advertisement set */
typedef uint8_t wiced_ble_ext_adv_handle_t;


/** The Advertising set identifier(SID) is used to uniquely identify adv sets from advertiser.
    SID the value to be transmitted in the advertising SID subfield of the ADI field of the Extended ADV PDUs */
enum
{
    WICED_BLE_EXT_ADV_SID_MIN = 0x00, /**< min SID value */
    WICED_BLE_EXT_ADV_SID_MAX = 0x0f, /**< max SID value */
};
/** SID value */
typedef uint8_t wiced_ble_ext_adv_sid_t;

/** LE Phy to be used for extended advertisement */
enum
{
    WICED_BLE_EXT_ADV_PHY_1M = 0x1,       /**< advetiser advertisement PHY is LE 1M */
    WICED_BLE_EXT_ADV_PHY_2M = 0x2,       /**< advetiser advertisement PHY is LE 2M */
    WICED_BLE_EXT_ADV_PHY_LE_CODED = 0x3, /**< advetiser advertisement PHY is LE Coded (for long range) */
    WICED_BLE_EXT_ADV_NUM_PHYS = 0x3      /**< 3 PHYs are defined */
};
/** LE Phy type for extended advertisement */
typedef uint8_t wiced_ble_ext_adv_phy_t;

/** ISOC LE PHY */
enum wiced_ble_isoc_phy_e
{
    WICED_BLE_ISOC_LE_1M_PHY = 1, /**< ISOC Phy set to 1M */
    WICED_BLE_ISOC_LE_2M_PHY = 2, /**< ISOC Phy set to 2M */
    WICED_BLE_ISOC_LE_CODED = 4,  /**< ISOC Phy set to coded */
};
typedef uint8_t wiced_ble_isoc_phy_t; /**< ISOC LE PHY (see #wiced_ble_isoc_phy_e) */

/** ISOC Framing types */
enum wiced_ble_isoc_framing_e
{
    WICED_BLE_ISOC_UNFRAMED = 0, /**< Unframed */
    WICED_BLE_ISOC_FRAMED = 1    /**< Framed */
};
typedef uint8_t wiced_ble_isoc_framing_t; /**< ISOC Framing types (see #wiced_ble_isoc_framing_e) */

/** Broadcast ISOC Encryption */
enum wiced_ble_isoc_encryption_e
{
    WICED_BLE_ISOC_UNENCRYPTED = 0, /**< ISOC unencrypted */
    WICED_BLE_ISOC_ENCRYPTED = 1,   /**< ISOC encrypted */
};
typedef uint8_t wiced_ble_isoc_encryption_t; /**< ISOC Encryption (see #wiced_ble_isoc_encryption_e) */

/** Own address options for adv, scan, create connection */
enum wiced_ble_own_address_options_e
{
    /** Use public address, see \ref wiced_bt_set_local_bdaddr */
    WICED_BLE_OWN_ADDR_PUBLIC = 0,
    /** Use random address, see \ref wiced_bt_set_local_bdaddr.
    * For advertising with extended APIs, random address set on the adv handle is used
    */
    WICED_BLE_OWN_ADDR_RANDOM = 1,
    /** Controller generates RPA with local IRK entry in resolving list pointed by peer address and peer address type.
    * If no entry found, use public address see \ref wiced_bt_set_local_bdaddr
    */
    WICED_BLE_OWN_ADDR_GENERATE_RPA_OR_USE_PUBLIC_ADDR = 2,
    /** Controller generates RPA with local IRK entry in resolving list pointed by peer address and peer address type.
    * If no entry found, use random address see \ref wiced_bt_set_local_bdaddr
    * For advertising with extended APIs, random address set on the adv handle is used
    */
    WICED_BLE_OWN_ADDR_GENERATE_RPA_OR_USE_RANDOM_ADDR = 3
};
/** Own address options for adv, scan, create connection, see \ref wiced_ble_own_address_options_e*/
typedef uint8_t wiced_ble_own_address_options_t;

/** Scanning filter policy enums used in set scan parameters command */
enum wiced_ble_scanning_filter_policy_e
{
    WICED_BLE_SCAN_BASIC_UNFILTERED_SP = 0,    /**< Basic unfiltered scanning policy */
    WICED_BLE_SCAN_BASIC_FILTERED_SP = 1,      /**< Basic filtered scanning policy  */
    WICED_BLE_SCAN_EXTENDED_UNFILTERED_SP = 2, /**< Extended unfiltered scanning policy */
    WICED_BLE_SCAN_EXTENDED_FILTERED_SP = 3,   /**< Extended filtered scanning policy  */
};
/** Scanning filter policy used. (see #wiced_ble_scanning_filter_policy_e) */
typedef uint8_t wiced_ble_scanning_filter_policy_t;

/** Advertisement report context for the call to get offset and length of the searched adv type
 * #wiced_bt_ble_advert_type_t
 */
typedef struct
{
    /** pointer to advertisement data of length \p adv_len */
    uint8_t *p_adv;
    /** length of the adv pointed to by \p p_adv */
    uint16_t adv_len;
    /** offset of the data read in the previous call to \ref wiced_ble_adv_data_search */
    uint16_t offset;
} wiced_bt_adv_ctx_t;

/** Advertisement element */
typedef struct
{
    uint8_t *p_data;                        /**< Advertisement data */
    uint16_t len;                           /**< Advertisement length */
    wiced_bt_ble_advert_type_t advert_type; /**< Advertisement data type */
} wiced_bt_ble_advert_elem_t;

/**
 * Parse advertising data (returned from scan results callback \ref wiced_bt_ble_scan_result_cback_t or
 * \ref wiced_ble_ext_scan_result_cback_t ).
 * Look for specified advertisement data type.
 *
 * @param[in]  p_ctx  : context data for the advertisement data, len, offset read and type.
 *                      p_ctx->read_offset is incremented by this API on invocation of the API
 * @param[in]  type :   advertisement data type to search for
 * @param[out] p_length : length of advertisement data (if found)
 *
 * @return pointer to start of requested advertisement data (if found). NULL if requested data type not found.
 */
uint8_t *wiced_ble_adv_data_search(wiced_bt_adv_ctx_t *p_ctx,
                                   wiced_bt_ble_advert_type_t type,
                                   uint16_t * p_length);
/**
 * Build an advertisement or scan data packet
 *
 * @note: This API does not write data to the controller
 *
 * @param[in] p_ctxt : advertisement context, contains the pointer and length of buffer to be filled
 *                     The p_ctx->offset variable is updated by this call by the length of adv data written out
 *                     to p_ctx->p_adv
 * @param[in] p_elem : advertisement element of type, length, value
 *
 * @return  0 in case p_elem cannot be written to the buffer in \p p_ctxt, else (p_elem->len + 2)
 *
 */
uint16_t wiced_ble_adv_data_build(wiced_bt_adv_ctx_t *p_ctxt, wiced_bt_ble_advert_elem_t *p_elem);

/**@} wicedbt */
#endif // __WICED_BT_ADV_SCAN_COMMON_H__
