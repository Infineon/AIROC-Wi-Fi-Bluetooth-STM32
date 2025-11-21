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
 * AIROC Bluetooth Low Energy (LE) Functions for LE link management
 *
 */
#ifndef __WICED_BT_BLE_CONN_API_H__
#define __WICED_BT_BLE_CONN_API_H__

/**
 * This section contains the connection related defines and structures used in LE link management.
 *
 * @addtogroup  wicedbt_Conn   LE Connection
 *
 * @ingroup     wicedbt
 *
 * @{
 */

/** default connection interval min
 *  recommended min: 30ms  = 24 * 1.25
 */
#define BTM_BLE_CONN_INTERVAL_MIN_DEF 24

/** default connection interval max
 * recommended max: 50 ms = 56 * 1.25
 */
#define BTM_BLE_CONN_INTERVAL_MAX_DEF 40

/** default Peripheral latency */
#define BTM_BLE_CONN_PERIPHERAL_LATENCY_DEF 0

/** default supervision timeout */
#define BTM_BLE_CONN_TIMEOUT_DEF 2000

/** LE Signature
 *  LE data signature length 8 Bytes + 4 bytes counter
 */
#define BTM_BLE_AUTH_SIGNATURE_SIZE 12
/** Device address (see #BTM_BLE_AUTH_SIGNATURE_SIZE) */
typedef uint8_t wiced_dev_ble_signature_t[BTM_BLE_AUTH_SIGNATURE_SIZE];


/**
 * LE encryption method
 */
enum wiced_bt_ble_sec_action_type_e
{
    BTM_BLE_SEC_NONE,            /**< No encryption */
    BTM_BLE_SEC_ENCRYPT,         /**< encrypt the link using current key */
    BTM_BLE_SEC_ENCRYPT_NO_MITM, /**< encryption without MITM */
    BTM_BLE_SEC_ENCRYPT_MITM     /**< encryption with MITM*/
};
/** LE security type. refer #wiced_bt_ble_sec_action_type_e
 * */
typedef uint8_t wiced_bt_ble_sec_action_type_t;

/**
 * security settings used with L2CAP LE COC
 * */
enum wiced_bt_ble_sec_flags_e
{
    BTM_SEC_LE_LINK_ENCRYPTED = 0x01,           /**< Link encrypted */
    BTM_SEC_LE_LINK_PAIRED_WITHOUT_MITM = 0x02, /**< Paired without man-in-the-middle protection */
    BTM_SEC_LE_LINK_PAIRED_WITH_MITM = 0x04     /**< Link with man-in-the-middle protection */
};

/** LE preferred connection parameters */
typedef struct
{
    uint16_t conn_interval_min;        /**< minimum connection interval */
    uint16_t conn_interval_max;        /**< maximum connection interval */
    uint16_t conn_latency;             /**< connection latency */
    uint16_t conn_supervision_timeout; /**< connection supervision timeout */
    uint16_t min_ce_length;            /**< minimum connection events */
    uint16_t max_ce_length;            /**< maximum connection events */
} wiced_bt_ble_pref_conn_params_t;


/** Initiator filter policy for legacy connections */
enum wiced_ble_legacy_initiator_filter_policy_e
{
    WICED_BLE_LEGACY_INITIATOR_DO_NOT_USE_FILTER_LIST = 0, /**< Do not use filter list, use the peer address and
                                                           * peer addr type instead
                                                           */
    WICED_BLE_LEGACY_INITIATOR_USE_FILTER_LIST = 1         /**< Use the filter list, ignore peer address and peer type */
} ;
/** Initiator filter policy for legacy connections, check \ref wiced_ble_legacy_initiator_filter_policy_e */
typedef uint8_t wiced_ble_legacy_initiator_filter_policy_t;

/** Create Connection parameter for create a legacy LE ACL connection */
typedef struct
{
    /** Time interval from when the Controller started its lastLE scan until it begins the subsequent LE scan.
     * Range: 0x0004 to 0x4000 Time = N × 0.625 ms Time Range: 2.5 ms to 10.24 s */
    uint16_t le_scan_interval;
    /** Amount of time for the duration of the LE scan. LE_Scan_Window shall be less than or equal to LE_Scan_Interval
     * Range: 0x0004 to 0x4000 Time = N × 0.625 ms Time Range: 2.5 ms to 10.24 s*/
    uint16_t le_scan_window;
    /** Initiator filter policy */
    wiced_ble_legacy_initiator_filter_policy_t initiator_filter_policy;
    /** peer address type */
    wiced_bt_ble_address_type_t peer_address_type;
    /** peer address */
    wiced_bt_device_address_t peer_address;
    /** Option to determine own/peer public, random address or generated RPA to be used for initiating the connection */
    uint8_t own_address_type;
    /** Preferred connection parameters */
    wiced_bt_ble_pref_conn_params_t conn_params;
} wiced_ble_legacy_create_conn_t;

#define BTM_BLE_PREFER_1M_PHY 0x01   /**< LE 1M PHY preference */
#define BTM_BLE_PREFER_2M_PHY 0x02   /**< LE 2M PHY preference */
#define BTM_BLE_PREFER_LELR_PHY 0x04 /**< LE LELR PHY preference */

/**  Host preferences on PHY.
 *  bit field that indicates the transmitter PHYs that
 *  the Host prefers the Controller to use.Bit number 3 -7 reserved for future.
 */
typedef uint8_t wiced_bt_ble_host_phy_preferences_t;

#define BTM_BLE_PREFER_NO_LELR 0x0000 /**< No preferred coding */
#define BTM_BLE_PREFER_LELR_S2 0x0001 /**< Preferred coding is S=2, 500 kb/s */
#define BTM_BLE_PREFER_LELR_S8 0x0002 /**< Preferred coding is S=8, 125 kb/s */

/**  The PHY_options parameter is a bit field that allows the Host to specify options
 *    for LE long range PHY. Default connection is with no LE coded PHY.The Controller may override any
 *    preferred coding (S2 coded phy for 512k speed and s8 coded phy for 128K) for
 *    transmitting on the LE Coded PHY.
 *    The Host may specify a preferred coding even if it prefers not to use the LE
 *    Coded transmitter PHY since the Controller may override the PHY preference.
 *    Bit 2-15 reserved for future use.
 *  @note  These preferences applicable only when BTM_BLE_PREFER_LELR_PHY flag is set
 */
typedef uint16_t wiced_bt_ble_lelr_phy_preferences_t;

/** Host PHY preferences */
typedef struct
{
    wiced_bt_device_address_t remote_bd_addr;     /**< Peer Device address */
    wiced_bt_ble_host_phy_preferences_t tx_phys;  /**< Host preference among the TX PHYs */
    wiced_bt_ble_host_phy_preferences_t rx_phys;  /**< Host preference among the RX PHYs */
    wiced_bt_ble_lelr_phy_preferences_t phy_opts; /**< Host preference on LE coded PHY */
} wiced_bt_ble_phy_preferences_t;

/** LE connection parameteres */
typedef struct
{
    wiced_bt_dev_role_t role;     /**< Connection role 0: Central  1: Peripheral */
    uint16_t conn_interval;       /**< Connection interval in slots */
    uint16_t conn_latency;        /**< Connection latency */
    uint16_t supervision_timeout; /**< Supervision timeout */
} wiced_bt_ble_conn_params_t;


/** Privacy mode
 * refer Spec version 5.0 Vol 3 Part C Section 10.7 privacy feature
 */
enum wiced_bt_ble_privacy_e
{
    BTM_BLE_PRIVACY_MODE_NETWORK, /**< network privacy mode*/
    BTM_BLE_PRIVACY_MODE_DEVICE   /**< device privacy mode*/
};
/** LE Privacy mode. See #wiced_bt_ble_privacy_e
 * */
typedef uint8_t wiced_bt_ble_privacy_mode_t;

/** Encryption Data Key Material structure */
typedef struct
{
    wiced_bt_link_key_t session_key; /**< Session key */
    wiced_bt_iv_t iv;                /**< Initialization Vector */
} wiced_bt_ble_key_material_t;

#define BTM_AFH_CHNL_MAP_SIZE HCI_AFH_CHANNEL_MAP_LEN /**< AFH channel map size */
#define BLE_CHANNEL_MAP_LEN 5                         /**< AFH Channel Map len */
/** LE Channel Map */
typedef uint8_t wiced_bt_ble_chnl_map_t[BLE_CHANNEL_MAP_LEN];


/**
 * Callback wiced_bt_ble_read_phy_complete_callback_t
 *
 * read phy complete callback (from calling #wiced_bt_ble_read_phy)
 *
 * @param p_phy_result             : read phys result
 *
 * @return Nothing
 */
typedef void(wiced_bt_ble_read_phy_complete_callback_t)(wiced_bt_ble_phy_update_t *p_phy_result);

#ifdef __cplusplus
extern "C"
{
#endif

    /**
    * API to create a legacy LE ACL connection
    * The completion of the connection is reported with the \ref GATT_CONNECTION_STATUS_EVT
    * @param[in] p_legacy_conn_cfg Pointer to the connection configuration
    *
    * @return wiced_result_t
    */
    wiced_result_t wiced_ble_legacy_create_connection(wiced_ble_legacy_create_conn_t *p_legacy_conn_cfg);

/**
 * To read LE connection parameters based on connection address received in gatt connection up indication.
 *
 * @param[in]       remote_bda          : remote device address.
 * @param[in]       p_conn_parameters   : Connection Parameters
 *
 * @return          wiced_result_t \n
 *
 * <b> WICED_BT_ILLEGAL_VALUE </b> : if p_conn_parameters is NULL. \n
 * <b> WICED_BT_UNKNOWN_ADDR </b>  : if device address is bad. \n
 * <b> WICED_BT_SUCCESS </b> otherwise.
 *
 */
    wiced_result_t wiced_bt_ble_get_connection_parameters(wiced_bt_device_address_t remote_bda,
                                                          wiced_bt_ble_conn_params_t *p_conn_parameters);

/**
 * @addtogroup  btm_ble_conn_filter_accept_list_functions        Connection and Filter Accept List
 * @ingroup     btm_ble_api_functions
 *
 * This section provides functions for LE connection related and Filter Accept List operations.
 *
 * @{
 */

/**
 *
 *
 * @param[in]       bd_addr: remote device address.
 * @param[in]       addr_type   : remote device address type .
 *
 * @return  wiced_result_t
 *
 */
    wiced_result_t wiced_ble_add_to_filter_accept_list(wiced_bt_device_address_t bd_addr,
                                                       wiced_bt_ble_address_type_t addr_type);
/**
 *
 * Remove device from scanner filter accept list
 * @note This command shall not be used when
 *      • any advertising filter policy uses the Filter Accept List and advertising is enabled,
 *      • the scanning filter policy uses the Filter Accept List and scanning is enabled, or
 *      • the initiator filter policy uses the Filter Accept List and an HCI_LE_Create_Connection
 *        or HCI_LE_Extended_Create_Connection command is pending.
 *
 * @param[in]       bd_addr: remote device address.
 * @param[in]       addr_type   : remote device address type .
 *
 * @return       wiced_result_t
 *
 */
    wiced_result_t wiced_ble_remove_from_filter_accept_list(wiced_bt_device_address_t bd_addr,
                                                            wiced_bt_ble_address_type_t addr_type);

/**
 *
 * Request clearing filter Accept List in controller side
 *
 *
 * @return          TRUE if request of clear is sent to controller side
 *
 */
    wiced_result_t wiced_ble_clear_filter_accept_list(void);

/**
 *
 * Returns size of Filter Accept List size in controller side
 *
 *
 * @return          size of Filter Accept List in current controller
 *
 */
    uint16_t wiced_ble_get_filter_accept_list_size(void);

/**@} btm_ble_conn_filter_accept_list_functions */



/**
 * @addtogroup  btm_ble_phy_functions        Phy
 * @ingroup     btm_ble_api_functions
 *
 * This section provides functionality to read and update PHY.
 * @{
 */

/**
 * Host to read the current transmitter PHY and receiver PHY on the connection identified by the remote bdaddr.
 * phy results notified using #wiced_bt_ble_read_phy_complete_callback_t callback
 *
 * @param[in]       remote_bd_addr                   - remote device address
 * @param[in]       p_read_phy_complete_callback     - read phy complete callback
 *
 * @return          wiced_result_t \n
 *
 * <b> WICED_BT_SUCCESS </b>        : if the request was successfully sent to HCI. \n
 * <b> WICED_BT_UNKNOWN_ADDR </b>   : if device address does not correspond to a connected remote device \n
 * <b> WICED_BT_ILLEGAL_VALUE </b>  : if p_read_phy_complete_callback is NULL \n
 * <b> WICED_BT_NO_RESOURCES </b>   : if could not allocate resources to start the command
 *
 */
wiced_bt_dev_status_t wiced_bt_ble_read_phy(wiced_bt_device_address_t remote_bd_addr,
                                            wiced_bt_ble_read_phy_complete_callback_t *p_read_phy_complete_callback);

/**
 * Host to configure default transmitter phy and receiver phy to
 * be used for all subsequent connections over the LE transport.
 *
 *
 * @param[in]       phy_preferences      - Phy preferences
 *
 * Note : remote_bd_addr field of the phy_preferences is ignored.
 *
 * @return          wiced_result_t
 *
 * <b> WICED_BT_SUCCESS </b>        : if the request was successfully sent to HCI. \n
 * <b> WICED_BT_ILLEGAL_VALUE </b>  : if phy_preferences is NULL \n
 * <b> WICED_BT_NO_RESOURCES </b>   : if could not allocate resources to start the command
 *
 */
wiced_bt_dev_status_t wiced_bt_ble_set_default_phy(wiced_bt_ble_phy_preferences_t *phy_preferences);

/**
 * Host to configure the LE link to 1M or 2M and LE coding to be used
 *
 * @param[in]       phy_preferences      - Phy preferences
 *
 * @return          wiced_result_t \n
 *
 * <b> WICED_BT_SUCCESS </b>        : if the request was successfully sent to HCI. \n
 * <b> WICED_BT_ILLEGAL_VALUE </b>  : if phy_preferences is NULL \n
 * <b> WICED_BT_UNKNOWN_ADDR </b>   : if device address does not correspond to a connected remote device \n
 * <b> WICED_BT_NO_RESOURCES </b>   : if could not allocate resources to start the command
 *
 */
wiced_result_t wiced_bt_ble_set_phy(wiced_bt_ble_phy_preferences_t *phy_preferences);

/**
 * Set channel classification for the available 40 channels.
 *
 * Channel n is bad = 0.
 * Channel n is unknown = 1.
 *
 * At least one channel shall be marked as unknown.
 *
 * @param[in]       ble_channel_map
 *
 * @return          wiced_result_t \n
 *
 * <b> WICED_BT_SUCCESS </b>        if successfully initiated \n
 * <b> WICED_BT_NO_RESOURCES </b>   if could not allocate resources to start the command
 */
wiced_result_t wiced_bt_ble_set_channel_classification(const wiced_bt_ble_chnl_map_t ble_channel_map);
/**@} btm_ble_phy_functions */


/**
 * @ingroup     btm_ble_sec_api_functions
 *
 * @{
 */

/**
 *
 * Grant or deny access.  Used in response to an BTM_SECURITY_REQUEST_EVT event.
 *
 * @param[in]       bd_addr     : peer device bd address.
 * @param[in]       res         : WICED_BT_SUCCESS to grant access;
                                  WICED_BT_UNSUPPORTED , if local device does not allow pairing;
                                  WICED_BT_REPEATED_ATTEMPTS otherwise
 *
 * @return          <b> None </b>
 *
 */
void wiced_bt_ble_security_grant(wiced_bt_device_address_t bd_addr, wiced_bt_dev_status_t res);

/**
 * Sign the data using AES128 CMAC algorith.
 *
 * @param[in]       bd_addr: target device the data to be signed for.
 * @param[in]       p_text: signing data
 * @param[in]       len: length of the signing data
 * @param[in]       signature: output parameter where data signature is going to be stored
 *
 * @return          TRUE if signing successful, otherwise FALSE.
 *
 */
wiced_bool_t wiced_bt_ble_data_signature(wiced_bt_device_address_t bd_addr,
                                         uint8_t *p_text,
                                         uint16_t len,
                                         wiced_dev_ble_signature_t signature);

/**
 * Verify the data signature
 *
 * @param[in]       bd_addr: target device the data to be signed for.
 * @param[in]       p_orig:  original data before signature.
 * @param[in]       len: length of the signing data
 * @param[in]       counter: counter used when doing data signing
 * @param[in]       p_comp: signature to be compared against.
 *
 * @return          TRUE if signature verified correctly; otherwise FALSE.
 *
 */
wiced_bool_t wiced_bt_ble_verify_signature(
    wiced_bt_device_address_t bd_addr, uint8_t *p_orig, uint16_t len, uint32_t counter, uint8_t *p_comp);

/**
 * Get security mode 1 flags and encryption key size for LE peer.
 *
 * @param[in]       bd_addr         : peer address
 * @param[out]      p_le_sec_flags  : security flags (see #wiced_bt_ble_sec_flags_e)
 * @param[out]      p_le_key_size   : encryption key size
 *
 * @return          TRUE if successful
 *
 */
wiced_bool_t wiced_bt_ble_get_security_state(wiced_bt_device_address_t bd_addr,
                                             uint8_t *p_le_sec_flags,
                                             uint8_t *p_le_key_size);

/**
 * Updates privacy mode if device is already available in controller resolving list
 *
 * @param[in]       remote_bda      -remote device address received during connection up
 * @param[in]       rem_bda_type    -remote device address type received during connection up
 * @param[in]       privacy_mode    - privacy mode (see \ref wiced_bt_ble_privacy_mode_t)
 *
 * @return          wiced_bt_dev_status_t \n
 * <b> WICED_BT_ILLEGAL_VALUE </b>  : if paramer is wrong \n
 * <b> WICED_BT_UNSUPPORTED </b>    : if command not supported \n
 * <b> WICED_BT_UNKNOWN_ADDR </b>   : if bd_addr is wrong \n
 * <b> WICED_BT_ILLEGAL_ACTION </b> : if device not added to resolving list or peer irk is not valid \n
 * <b> WICED_BT_ERROR </b>      : error while processing the command \n
 * <b> WICED_BT_SUCCESS</b>     : if command started
 *
 */
wiced_bt_dev_status_t wiced_bt_ble_set_privacy_mode(wiced_bt_device_address_t remote_bda,
                                                    wiced_bt_ble_address_type_t rem_bda_type,
                                                    wiced_bt_ble_privacy_mode_t privacy_mode);

/**
 * Function         wiced_ble_private_device_address_resolution
 *
 *                  This API verifies whether given device address is Resolvable Private Address or not
 *
 * @param rpa       LE Resolvable Private Address
 * @param irk       LE IRK
 * @return          wiced_result_t
 *                  WICED_BT_SUCCESS the identity of device address has been resolved.
 *                  WICED_BT_ERROR   otherwise.
 */
wiced_result_t wiced_ble_private_device_address_resolution(wiced_bt_device_address_t rpa, BT_OCTET16 irk);

/**
 * Function wiced_bt_ble_address_resolution_list_clear_and_disable
 *
 * This API clears the address resolution list and disables the address resolution feature.
 *
 * @return          wiced_result_t
 *                  WICED_BT_SUCCESS if address resolution list is cleared and adress resolution feature is disabled.
 *                  WICED_BT_ERROR   otherwise.
 */
wiced_result_t wiced_bt_ble_address_resolution_list_clear_and_disable(void);
/**@} btm_ble_sec_api_functions */

/**
 * Function         wiced_bt_ble_read_le_features
 *
 *                  This API returns the features supported by the \p bda
 *
 * @param[in]  bda       Device address pointer, pass NULL for local device
 * @param[out] features  Pointer to store the supported features
 * @return          wiced_result_t
 *                  WICED_BT_SUCCESS contents of features are valid
 *                  WICED_BT_ERROR   otherwise.
 */
wiced_result_t wiced_bt_ble_read_le_features(wiced_bt_device_address_t bda, wiced_bt_features_t features);

/**
* Function wiced_bt_ble_set_host_features
*
*          This API is called to set the supported host features
*
* @param[in]  feature   bit position of required feature
* @param[in]  bit_value Value to enable or disable Advertising Coding Selection Host Support feature bit
*
* @return          wiced_result_t
*                  WICED_BT_SUCCESS Advertising Coding Selection Host Support feature bit is modified
*                  WICED_BT_ERROR   otherwise.
*/
wiced_bt_dev_status_t wiced_bt_ble_set_host_features(wiced_bt_ble_feature_bit_t feature, uint8_t bit_value);

/**
* Function wiced_bt_ble_encrypt_adv_packet
*
*    This API is called to encrypt advertising data. Encrypts data at \p p_plaintext of length \p payload_len into
*    \p p_encrypted of length \p payload_len
*
* @param[in]  p_key         session key (16 bytes, little endian)
* @param[in]  p_iv          initialization vector (8 bytes, little endian)
* @param[in]  p_randomizer  randomizer (5 bytes, little endian)
* @param[in]  p_plaintext   plaintext to be encoded
* @param[out] p_encrypted   encrypted output
* @param[in/out]  payload_len   plaintext/encrypted data len
* @param[out] p_mic         pointer to MIC, Message Integrity Check, derived from the data
*
* @note the value returned in \p p_mic has to be appended to the end of the encrypted data
*
* @return          wiced_result_t
*                  WICED_BT_SUCCESS If adv_packet is encrypted successfully
*                  WICED_BT_ERROR   otherwise
*
*/
wiced_result_t wiced_bt_ble_encrypt_adv_packet(uint8_t *p_key,
                                               uint8_t *p_iv,
                                               uint8_t *p_randomizer,
                                               const uint8_t *p_plaintext,
                                               uint8_t *p_encrypted,
                                               int payload_len,
                                               uint32_t *p_mic);

/**
* Function wiced_bt_ble_decrypt_adv_packet
*
*    This API is called to decrypt advertising data. Decrypts data at \p p_encrypted of length \p coded_len into
*    \p p_plaintext of length \p coded_len
*
* @param[in]  p_key        session key (16 bytes, little endian)
* @param[in]  p_iv         initialization vector (8 bytes, little endian)
* @param[in]  p_randomizer randomizer (5 bytes, little endian)
* @param[in]  p_encrypted  encrypted data input
* @param[out] p_plaintext  plaintext output
* @param[in/out]  coded_len    encrypted data len
* @param[out] p_mic        pointer to MIC, Message Integrity Check, derived from the data
*
* @note the value returned in \p p_mic can be used to check against the value in the received encrypted data
*
* @return          wiced_result_t
*                  WICED_BT_SUCCESS If adv_packet is decrypted successfully
*                  WICED_BT_ERROR   otherwise
*/
wiced_result_t wiced_bt_ble_decrypt_adv_packet(uint8_t *p_key,
                                               uint8_t *p_iv,
                                               uint8_t *p_randomizer,
                                               const uint8_t *p_encrypted,
                                               uint8_t *p_plaintext,
                                               int coded_len,
                                               uint32_t *p_mic);

/**
 * This API allows to set maximum transmission payload size and maximum packet transmission time
 * to be used for LL DATA PDUs on a given connection
 *
 * @param[in]       bd_addr  - bd_Addr for which the LL Data PDU need to set
 * @param[in]       tx_pdu_length -  maximum LL Data PDU on this connection.
 * @param[in]       tx_time -  maximum number of microseconds taken to transmit a packet on this connection(Range 0x0148 to 0x4290).
 *
 * @return          wiced_bt_dev_status_t
 *
 * <b> WICED_BT_SUCCESS </b>       : If  command sent successfully\n
 * <b> WICED_BT_ERROR </b>         : If there is no connection exists \n
 * <b> WICED_BT_WRONG_MODE </b>   : If command not supported \n
 *
 *  @note if tx_time = 0, then a default value applies based on the max supported by the controller HCI version.

 */

wiced_bt_dev_status_t wiced_bt_ble_set_data_packet_length(wiced_bt_device_address_t bd_addr,
                                                          uint16_t tx_pdu_length,
                                                          uint16_t tx_time);

/**
 *
 * Create local LE SC (secure connection) OOB data. When
 * operation is completed, local OOB data will be
 * provided via BTM_SMP_SC_LOCAL_OOB_DATA_NOTIFICATION_EVT.
 *
 * @param[in]       bd_addr         : remote device address for the OOB data
 * @param[in]       bd_addr_type    : device address type of address \p bd_addr
 *
 * @return          TRUE: creation of local SC OOB data set started.
 *
 */
wiced_bool_t wiced_bt_smp_create_local_sc_oob_data(wiced_bt_device_address_t bd_addr,
                                                   wiced_bt_ble_address_type_t bd_addr_type);
#ifdef __cplusplus
} // extern "C"
#endif

/**@} wicedbt */
#endif // __WICED_BT_BLE_CONN_API_H__
