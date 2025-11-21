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
 * AIROC Bluetooth Low Energy (LE) Channel Sounding Functions
 *
 */
#ifndef __WICED_BLE_CS_H__
#define __WICED_BLE_CS_H__

#include "wiced_bt_types.h"
#include "wiced_result.h"

/**
 * This section contains LE Channel Sounding defines, structures and functions.
 * @note Applications using channel sounding are expected to use extended mode ADV and Scan APIs and
 * cannot invoke any of the Legacy mode APIs for advertisement and scanning
 *
 * @addtogroup  wicedbt_ChannelSounding   LE Channel Sounding
 *
 * @ingroup     wicedbt
 *
 * @{
 */

/** Structure for channel sounding capabilities */
typedef struct
{
    /** ACL connection handle */
    uint16_t acl_conn_handle;
    /** Number of CS configurations supported per connection, range 0x1 to 0x4 */
    uint8_t num_config_supported;
    /** If value set to
     * 0 - Support for both a fixed number of consecutive CS procedures and for an indefinite number of CS
     *     procedures until termination
     * If value set to (1 to 0xffff) - Maximum number of consecutive CS procedures supported.*/
    uint16_t max_consecutive_procedures_supported;
    /** Number of antennas supported */
    uint8_t num_antennas_supported;
    /** Maximum number of antenna paths supported */
    uint8_t max_antenna_paths_supported;
    /** Roles supported. Bit 0 - Initiator, Bit 1 - Reflector */
    uint8_t roles_supported;
    /** Optional CS modes supported. Bit 0 if set means Mode-3 supported */
    uint8_t modes_supported;
    /** The RTT_Capability, RTT_AA_Only_N, RTT_Sounding_N, and RTT_Random_Payload_N parameters indicate the time-of-flight
    accuracy as described in BT Core Specification ver 6.0, [Vol 6] Part B, Section 2.4.2.44 */
    /** Bits 0-2 valid. Refer BT Core Specification ver 6.0, [Vol 4], Part E, Section 7.8.132 */
    uint8_t rtt_capability;
    /** If value set to
     * 0 - RTT AA Only not supported, else if value (1 - 0xff) - Specifies Number of CS steps of single packet exchanges
     * needed to satisfy the precision requirements */
    uint8_t rtt_aa_only_n;
    /** If value set to
     * 0 - RTT Sounding not supported, else if value (1 - 0xff) - Specifies Number of CS steps of single packet exchanges
     * needed to satisfy the precision requirements */
    uint8_t rtt_sounding_n;
    /** If value set to
     * 0 - RTT Random Payload not supported, else if value (1 - 0xff) - Specifies Number of CS steps of single packet
     * exchanges needed to satisfy the precision requirements */
    uint8_t rtt_random_payload_n;
    /** The nadm_sounding_capability and nadm_random_capability indicate the support by the remote controller for
    reporting Normalized Attack Detector Metric (NADM) when a CS_SYNC with a sounding sequence or random sequence is
    received.*/
    /** If Bit 0 set - Support for Phase-based Normalized Attack Detector Metric when a CS_SYNC with sounding sequence is received */
    uint16_t nadm_sounding_capability;
    /** If Bit 0 set - Support for Phase-based Normalized Attack Detector Metric when a CS_SYNC with random sequence is received*/
    uint16_t nadm_random_capability;
    /** Bit 1 - LE 2M Phy supported Bit 2 - LE 2M 2BT Phy supported  */
    uint8_t cs_sync_phys_supported;
    /** Bit 1: CS with no transmitter Frequency Actuation Error
     * Bit 2: CS Channel Selection Algorithm #3c
     * Bit 3: CS phase-based ranging from RTT sounding sequence */
    uint16_t subfeatures_supported;
    /** The t_ip1_times_supported, t_ip2_times_supported, t_fcs_times_supported, t_pm_times_supported, and
         t_sw_time_supported parameters indicate the supported optional time durations used in CS steps as
         described in BT Core Specification ver 6.0, [Vol 6] Part H, Section 4.3 */
    /** Bit 0: 10 us supported, Bit 1: 20 us supported, Bit 2: 30 us supported, Bit 3: 40 us supported,
     * Bit 4: 50 us supported, Bit 5: 60 us supported, Bit 6: 80 us supported */
    uint16_t t_ip1_times_supported;
    /** Bit 0: 10 us supported, Bit 1: 20 us supported, Bit 2: 30 us supported, Bit 3: 40 us supported,
     * Bit 4: 50 us supported, Bit 5: 60 us supported, Bit 6: 80 us supported */
    uint16_t t_ip2_times_supported;
    /** Bit 0: 15 us supported, Bit 1: 20 us supported, Bit 2: 30 us supported, Bit 3: 40 us supported,
     * Bit 4: 50 us supported, Bit 5: 60 us supported, Bit 6: 80 us supported, Bit 7: 100 us supported,
     * Bit 8: 120 us supported */
    uint16_t t_fcs_times_supported;
    /** Bit 0: 10 us supported, Bit 1: 20 us supported */
    uint16_t t_pm_times_supported;
    /** Time in microseconds for the antenna switch period of the CS tones,
     * Allowed values : 0x00, 0x01, 0x02, 0x04, or 0x0A */
    uint8_t t_sw_time_supported;
    /** Bit 0: 18 dB supported, Bit 1: 21 dB supported, Bit 2: 24 dB supported, Bit 3: 27 dB supported,
     * Bit 4: 30 dB supported */
    uint8_t tx_snr_capability;
} wiced_ble_cs_capabilites_t;

/** Structure for setting channel sounding default settings */
typedef struct
{
    /** ACL connection handle */
    uint16_t acl_conn_handle;
    /** CS Role to enable or disable.  Bit 0 - Initiator enabled, Bit 1 - Reflector enabled */
    uint8_t role_enable;
    /**
     * The antenna identifier to be used for transmitting and receiving CS_SYNC packets
     * if set as (0x01 to 0x04) - Antenna identifier to be used for CS_SYNC packets by the local Controller
     * if set to 0xFE - Antennas to be used, in repetitive order from 0x01 to 0x04, for CS_SYNC packets by
     * the local Controller
     * if set to 0xFF - Host does not have a recommendation
     */
    uint8_t cs_sync_antenna_selection;
    /** maximum output power, EIRP (Effective Isotropic Radiated Power), to be used for all CS transmissions
     */
    uint8_t max_tx_power;
} wiced_ble_cs_default_settings_t;

#define WICED_BLE_CS_FAE_TABLE_SIZE 72 /**< Size of the CS FAE (Frequency Actuation Error) Table */
#define WICED_BLE_CS_CHANNEL_MAP_SIZE 10 /**< Size of the CS Channel Map */

/** CS FAE (Frequency Actuation Error) table  */
typedef uint8_t wiced_ble_cs_fae_table_t[WICED_BLE_CS_FAE_TABLE_SIZE];
/** CS Channel map
* 10 octet value to set the CS channel map, This parameter contains 80 1-bit fields. The nth such field (in the
* range 0 to 78) contains the value for CS  channel index n.
* Channel n is enabled for CS procedure = 1
* Channel n is disabled for CS procedure = 0
* Channels n = 0, 1, 23, 24, 25, 77, and 78 shall be ignored and shall be set to zero.
* At least 15 channels shall be enabled.
* The most significant bit (bit 79) is reserved for future use.
*/
typedef uint8_t wiced_ble_cs_channel_map_t[WICED_BLE_CS_CHANNEL_MAP_SIZE];

/** Structure for the FAE table */
typedef struct
{
    uint16_t acl_conn_handle; /**< ACL connection handle */
    wiced_ble_cs_fae_table_t table; /**< FAE table data */
} wiced_ble_cs_fae_table_data_t;

/** Structure to set the CS config */
typedef struct
{
    /**
     * ACL connection handle
     */
    uint16_t acl_conn_handle;
    /** CS Config Id identifying CS procedure to be created or updated */
    uint8_t config_id;
    /**
     * If set to 0 - CS configuration set to local controller only If set to 1 - CS configuration set in both
     * local and remote
     */
    uint8_t create_context;
    /**
     * If set to 0x1: Mode-1, 0x2: Mode-2, 0x3:Mode-3
     */
    uint8_t main_mode_type;
    /**
     * If set to 0x1: Mode-1, 0x2: Mode-2, 0x3:Mode-3
     */
    uint8_t sub_mode_type;
    /**
     * Valid values (0x2 to 0xFF) : Minimum number of CS main mode steps to be executed before a submode step
     * is executed
     */
    uint8_t min_main_mode_steps;
    /**
     * Valid values (0x2 to 0xFF) : Maximum number of CS main mode steps to be executed before a submode step
     * is executed
     */
    uint8_t max_main_mode_steps;
    /**
     * Valid values (0x0 to 0x3) : The number of main mode steps taken from the end of the last CS subevent
     * to be repeated at the beginning of the current CS subevent directly after the last mode-0 step of that event
     */
    uint8_t main_mode_repetition;
    /**
     * Valid values (0x1 to 0x3) : Number of CS mode-0 steps to be included at the beginning of each CS subevent
     */
    uint8_t mode_0_steps;
    /**
     * Set 0 - Initiator, 1 - Reflector
     */
    uint8_t role;
    /**
     * 0x00 RTT AA-only, 0x01 RTT with 32-bit sounding sequence, 0x02 RTT with 96-bit sounding sequence,
     * 0x03 RTT with 32-bit random sequence, 0x04 RTT with 64-bit random sequence,
     * 0x05 RTT with 96-bit random sequence, 0x06 RTT with 128-bit random sequence,
     * All other values Reserved for future use
     */
    uint8_t rtt_type;
    /**
     * 0x01 LE 1M PHY, 0x02 LE 2M PHY, 0x03 LE 2M 2BT PHY
     */
    uint8_t cs_sync_phy;
    /**
     * CS Channel map
     */
    wiced_ble_cs_channel_map_t channel_map;
    /**
     * The number of times the map represented by the channel_map field is to be cycled through for non-mode-0
     * steps within a CS procedure Valid value 0x1-0xFF
     */
    uint8_t channel_map_repetition;
    /**
     * Value 0 : Use Channel Selection Algorithm #3b for non-mode-0 CS steps
     * Value 1: Use Channel Selection Algorithm #3c for non-mode-0 CS steps
     */
    uint8_t channel_selection_type;
    /**
     * Value 0: Use Hat shape for user-specified channel sequence
     * Value 1: Use X shape for user-specified channel sequence
     */
    uint8_t ch3c_shape;
    /**
     * Values (0x2 - 0x8) Number of channels skipped in each rising and falling sequence
     */
    uint8_t ch3c_jump;
    /**
     * Reserved, shall be set to 0
     */
    uint8_t reserved;
} wiced_ble_cs_config_t;

/** CS Procedure parameters */
typedef struct
{
    /**
     * ACL connection handle
     */
    uint16_t acl_conn_handle;
    /**
     * CS Config Id to identity the procedure, range 0 to 3
     */
    uint8_t config_id;
    /**
     * Maximum duration for each CS procedure,
     * Range: 0x1 - 0xFFFF, Time = Nx0.625ms, Time Range: 0.625ms to 40.959375 s
     */
    uint16_t max_procedure_len;
    /** Minimum number of connection events between consecutive CS procedures
     * Range: 0x0001 to 0xFFFF
     */
    uint16_t min_procedure_interval;
    /**
     * Maximum number of connection events between consecutive CS procedures
     * Range: 0x0001 to 0xFFFF
     */
    uint16_t max_procedure_interval;
    /** If set to 0: CS procedures to continue until disabled, else,
     * if set to (1 - 0xFFFF): Maximum number of CS procedures to be scheduled
     */
    uint16_t max_procedure_count;
    /** Minimum suggested duration for each CS subevent in microseconds
     * Range: 1250 us to 4 s
     */
    uint32_t min_subevent_len;
    /**
     * Maximum suggested duration for each CS subevent in microseconds,
     * Range: 1250 us to 4 s
     */
    uint32_t max_subevent_len;
    /**
     * Antenna Configuration Index as described in BT Core Specification ver 6.0, [Vol 6] Part A, Section 5.3,
     * Valid values : 0x0 - 0x7
     */
    uint8_t tone_antenna_config_selection;
    /**
     * 0x01 LE 1M PHY, 0x02 LE 2M PHY, 0x03 LE Coded PHY with S=8 data coding, 0x04 LE Coded PHY with S=2 data coding
     */
    uint8_t phy;
    /**
     * 0xXX - Transmit power delta, in signed dB, to indicate the recommended difference between the remote device's
     * power level for the CS tones and RTT packets and the existing power level for the PHY indicated by the PHY
     * parameter
     * If set to 0x80 - Host does not have a recommendation for transmit power delta
     */
    uint8_t tx_power_delta;
    /**
     * Bit 0: Use first ordered antenna element, Bit 1: Use second ordered antenna element,
     * Bit 2: Use third ordered antenna element, Bit 3: Use fourth ordered antenna element
     */
    uint8_t preferred_peer_antenna;
    /**
     * Value 0x00 SNR control adjustment of 18 dB, Value 0x01 SNR control adjustment of 21 dB,
     * Value 0x02 SNR control adjustment of 24 dB, Value 0x03 SNR control adjustment of 27 dB,
     * Value 0x04 SNR control adjustment of 30 dB,  Value 0xFF SNR control is not to be applied
     */
    uint8_t snr_control_initiator;
    /**
     * Value 0x00 SNR control adjustment of 18 dB, Value 0x01 SNR control adjustment of 21 dB,
     * Value 0x02 SNR control adjustment of 24 dB, Value 0x03 SNR control adjustment of 27 dB,
     * Value 0x04 SNR control adjustment of 30 dB, Value 0xFF SNR control is not to be applied
     */
    uint8_t snr_control_reflector;
} wiced_ble_cs_procedure_params_t;

/** Structure to return the CS Configuration complete event */
typedef struct
{
    /**
     * ACL connection handle
     */
    uint16_t acl_conn_handle;
    /**
     * CS Config Id identifying CS procedure
     */
    uint8_t config_id;
    /**
     * Value 0x00 CS configuration is removed
     * Value 0x01 CS configuration is created
     */
    uint8_t action;
    /**
     * If set to 0x1: Mode-1, 0x2: Mode-2, 0x3:Mode-3
     */
    uint8_t main_mode_type;
    /**
     * If set to 0x1: Mode-1, 0x2: Mode-2, 0x3:Mode-3
     */
    uint8_t sub_mode_type;
    /** Valid values (0x2 to 0xFF) : Minimum number of CS main mode steps to be executed before a submode
     * step is executed
     */
    uint8_t min_main_mode_steps;
    /**
     * Valid values (0x2 to 0xFF) : Maximum number of CS main mode steps to be
     * executed before a submode step is executed
     */
    uint8_t max_main_mode_steps;
    /** Valid values (0x0 to 0x3) : The number of main mode steps taken from the end of the last CS subevent
     * to be repeated at the beginning of the current CS subevent directly after the last mode-0 step of
     * that event
     */
    uint8_t main_mode_repetition;
    /**
     * Valid values (0x1 to 0x3) : Number of CS mode-0 steps to be included at the beginning of each CS subevent
     */
    uint8_t mode_0_steps;
    /**
     * Set Value 0 - Initiator, Value 1 - Reflector
     */
    uint8_t role;
    /**
     * 0x00 RTT AA-only, 0x01 RTT with 32-bit sounding sequence, 0x02 RTT with 96-bit sounding sequence,
     * 0x03 RTT with 32-bit random sequence, 0x04 RTT with 64-bit random sequence,
     * 0x05 RTT with 96-bit random sequence, 0x06 RTT with 128-bit random sequence
     * All other values Reserved for future use
     */
    uint8_t rtt_type;
    /**
     * 0x01 LE 1M PHY, 0x02 LE 2M PHY, 0x03 LE 2M 2BT PHY
     */
    uint8_t cs_sync_phy;
    /**
     * CS Channel map
     */
    wiced_ble_cs_channel_map_t channel_map;
    /**
     * The number of times the map represented by the channel_map field is to be cycled through for non-mode-0
     * steps within a CS procedure
     * Valid value 0x1-0xFF
     */
    uint8_t channel_map_repetition;
    /**
     * Value 0 : Use Channel Selection Algorithm #3b for non-mode-0 CS steps
     * Value 1: Use Channel Selection Algorithm #3c for non-mode-0 CS steps
     */
    uint8_t channel_selection_type;
    /**
     * Value 0: Use Hat shape for user-specified channel sequence
     * Value 1: Use X shape for user-specified channel sequence
     */
    uint8_t ch3c_shape;
    /**
     * Values (0x2 - 0x8) Number of channels skipped in each rising and falling sequence
     */
    uint8_t ch3c_jump;
    /**
     * Reserved, shall be set to 0
     */
    uint8_t reserved;
    /**
     * Interlude time in microseconds between the RTT packets,
     * Values : 0x0A, 0x14, 0x1E, 0x28, 0x32, 0x3C, 0x50, or 0x91
     */
    uint8_t t_ip1_time;
    /**
     * Interlude time in microseconds between the CS tones
     * Values : 0x0A, 0x14, 0x1E, 0x28, 0x32, 0x3C, 0x50, or 0x91
     */
    uint8_t t_ip2_time;
    /**
     * Time in microseconds for frequency changes
     * Values : 0x0F, 0x14, 0x1E, 0x28, 0x32, 0x3C, 0x50, 0x64, 0x78, or 0x96
     */
    uint8_t t_fcs_time;
    /**
     * Time in microseconds for the phase measurement period of the CS tones Values: 0x0A, 0x14, or 0x28
     */
    uint8_t t_pm_time;
} wiced_ble_cs_config_complete_t;

/** Structure to return the CS Procedure enable complete event */
typedef struct
{
    /** ACL connection handle */
    uint16_t acl_conn_handle;
    /**
     * CS Config Id identifying CS procedure
     */
    uint8_t config_id;
    /**
     * CS Procedure State
     * Value 0: CS procedures are disabled
     * Value 1: CS procedures are enabled
     */
    uint8_t state;
    /**
     * Antenna Configuration Index as described in BT Core Specification ver 6.0, [Vol 6] Part A, Section 5.3,
     * Values : 0x00 - 0x07
     */
    uint8_t tone_antenna_config_selection;
    /**
     * Transmit power level used for CS procedure, Range: -127 to 20, Units: dBm,
     * If value = 0x7F - Transmit power level is unavailable
     */
    uint8_t selected_tx_power;
    /**
     * Duration for each CS subevent in microseconds, Range: 1250 us to 4 s
     */
    uint32_t subevent_len;
    /**
     * Number of CS subevents anchored off the same ACL connection event
     */
    uint8_t subevents_per_event;
    /**
     * Time between consecutive CS subevents anchored off the same ACL connection event. Units: 0.625 ms
     */
    uint16_t subevent_interval;
    /**
     * Number of ACL connection events between consecutive CS event anchor points
     */
    uint16_t event_interval;
    /**
     * Number of ACL connection events between consecutive CS procedure anchor points
     */
    uint16_t procedure_interval;
    /**
     * Number of CS procedures to be scheduled, Range: 0x0001 to 0xFFFF
     * If set to 0 - CS procedures to continue until disabled, else,
     */
    uint16_t procedure_count;
    /**
     * Maximum duration for each CS procedure,
     * Range: 0x0001 to 0xFFFF, Time = N × 0.625 ms, Time range: 0.625 ms to 40.959375 s
     */
    uint16_t max_procedure_len;
} wiced_ble_cs_procedure_enable_complete_t;

/** CS events to the application */

/** CS step data results */
typedef struct
{
    uint8_t *p_data; /**< step data sets*/
    uint8_t len;     /**< length of \p p_data */
} wiced_ble_cs_subevent_result_to_app_t;

/** CS Subevent continue result hdr */
typedef struct
{
    uint16_t acl_conn_handle; /**< ACL connection handle */
    uint8_t config_id;        /**< CS config identifier */
    uint8_t procedure_done_status; /**< procedure status*/
    uint8_t subevent_done_status;  /**< subevent status */
    uint8_t abort_reason;          /**< abort reason */
    uint8_t num_antenna_paths;     /**< number of antenna paths used */
    uint8_t num_steps_reported;    /**< num steps reported */
} wiced_ble_cs_subevent_continue_result_hdr_t;

/** CS Subevent result hdr */
typedef struct
{
    wiced_ble_cs_subevent_continue_result_hdr_t cntn; /**< subevent hdr */
    uint16_t start_acl_conn_event_counter;               /**< start of the acl conn event counter */
    uint16_t procedure_counter;                          /**< procedure counter */
    uint16_t frequency_compensation;                     /**< frequence compensation */
    uint8_t reference_power_level;                       /**< reference power level */
} wiced_ble_cs_subevent_result_hdr_t;

/** union of data for CS events */
typedef union
{
    /** Data for #WICED_BLE_CS_READ_LOCAL_CAPABILITIES_COMPLETE and
     * #WICED_BLE_CS_READ_REMOTE_CAPABILITIES_COMPLETE */
    wiced_ble_cs_capabilites_t cs_capabilities;
    /** Data for #WICED_BLE_CS_READ_REMOTE_FAE_TABLE_COMPLETE*/
    wiced_ble_cs_fae_table_data_t fae_table;
    /** Data for #WICED_BLE_CS_SET_SECURITY_ENABLE_COMPLETE and
     * #WICED_BLE_CS_WRITE_CACHED_CAPABILITIES_CMD_COMPLETE*/
    uint16_t acl_conn_handle;
    /** Data for #WICED_BLE_CS_CONFIG_COMPLETE*/
    wiced_ble_cs_config_complete_t config_cmplt;
    /** Data for #WICED_BLE_CS_PROCEDURE_ENABLE_COMPLETE_EVT*/
    wiced_ble_cs_procedure_enable_complete_t proc_enable;
    /** Data for #WICED_BLE_CS_SUBEVENT_RESULT_EVT and #WICED_BLE_CS_SUBEVENT_RESULT_CONTINUE_EVT*/
    wiced_ble_cs_subevent_result_to_app_t cs_subevent_data;
} wiced_ble_cs_event_data_t;

/** CS Events enumeration */
enum wiced_ble_cs_events_e
{
    /**
     * CS Read local capabilities complete event.
     * Event data in #wiced_ble_cs_event_data_t.cs_capabilities
     */
    WICED_BLE_CS_READ_LOCAL_CAPABILITIES_COMPLETE,
    /**
     * CS Read Remote capabilities complete event.
     * Event data in #wiced_ble_cs_event_data_t.cs_capabilities
     */
    WICED_BLE_CS_READ_REMOTE_CAPABILITIES_COMPLETE,
    /**
     * CS write cached capabilities cmd complete event,
     * Event data in #wiced_ble_cs_event_data_t.acl_conn_handle
     */
    WICED_BLE_CS_WRITE_CACHED_CAPABILITIES_CMD_COMPLETE,
    /**
     * CS write default settings complete event,
     * Event data in #wiced_ble_cs_event_data_t.acl_conn_handle
     */
    WICED_BLE_CS_WRITE_DEFAULT_SETTINGS_COMPLETE,
    /**
     * CS read remote FAE table complete event,
     * Event data in #wiced_ble_cs_event_data_t.fae_table
     */
    WICED_BLE_CS_READ_REMOTE_FAE_TABLE_COMPLETE,
    /**
     * CS write cached FAE table complete event,
     * Event data in #wiced_ble_cs_event_data_t.acl_conn_handle
     */
    WICED_BLE_CS_WRITE_CACHED_REMOTE_FAE_TABLE_COMPLETE,
    /**
     * CS security enable complete event,
     * Event data in #wiced_ble_cs_event_data_t.acl_conn_handle
     */
    WICED_BLE_CS_SET_SECURITY_ENABLE_COMPLETE,
    /**
     * CS config complete event,
     * Event data in #wiced_ble_cs_event_data_t.config_cmplt
     */
    WICED_BLE_CS_CONFIG_COMPLETE,
    /**
     * CS set channel classification complete event,
     * Event data : NULL
     */
    WICED_BLE_CS_SET_CHANNEL_CLASSIFICATION_COMPLETE,
    /**
     * CS Procedure enable command complete event,
     * Event data in #wiced_ble_cs_event_data_t.acl_conn_handle
     */
    WICED_BLE_CS_PROCEDURE_ENABLE_CMD_COMPLETE,
    /**
     * CS Procedure enable complete evt.
     * Event Data: #wiced_ble_cs_event_data_t.proc_enable
     */
    WICED_BLE_CS_PROCEDURE_ENABLE_COMPLETE_EVT,
    /**
     * CS Subevent result event.
     * Event Data: #wiced_ble_cs_event_data_t.cs_subevent_data
     */
    WICED_BLE_CS_SUBEVENT_RESULT_EVT,
    /**
     * CS Subevent result continue event.
     * Event Data: #wiced_ble_cs_event_data_t.cs_subevent_data
     */
    WICED_BLE_CS_SUBEVENT_RESULT_CONTINUE_EVT
};
/** Channel sounding events (see #wiced_ble_cs_events_e)*/
typedef uint8_t wiced_ble_cs_events_t;

/**
 * Callback wiced_ble_cs_events_cb_t
 *
 * Adv extension command status, command complete event and LE adv extension meta event callback
 *
 * @param event  : Event type (see #wiced_ble_cs_events_t)
 * @param p_data : Event data (see #wiced_ble_cs_event_data_t)
 *
 * @return Nothing
 */
typedef void (*wiced_ble_cs_events_cb_t)(wiced_ble_cs_events_t event,
                                            uint8_t status,
                                            wiced_ble_cs_event_data_t *p_data);

/**
 * Register the callback for receiving CS events
 * @param pf_cs_cb : Function ptr to receive CS events
 *
 * @return wiced_result_t
 */
wiced_result_t wiced_ble_cs_register(wiced_ble_cs_events_cb_t pf_cs_cb);

/**
 * Read local supported CS capabilities
 * The local capabilities read are returned in the registered callback, with event set to
 *   #WICED_BLE_CS_READ_LOCAL_CAPABILITIES_COMPLETE
 *
 * @return wiced_result_t
 */
wiced_result_t wiced_ble_cs_read_local_supported_capabilities(void);

/**
 * Read remote supported CS capabilities
 * The remote capabilities read are returned in the registered callback, with event set to
 *   #WICED_BLE_CS_READ_REMOTE_CAPABILITIES_COMPLETE
 * @param acl_conn_handle : ACL connection handle
 *
 * @return wiced_result_t
 */
wiced_result_t wiced_ble_cs_read_remote_supported_capabilities(uint16_t acl_conn_handle);

/**
 * Write cached remote supported CS capabilities
 * #WICED_BLE_CS_WRITE_CACHED_CAPABILITIES_CMD_COMPLETE is generated on completion of this API.
 *
 * @param p_cap : Cached remote capabilities, previously received from the remote device
 *
 * @return wiced_result_t
 */
wiced_result_t wiced_ble_cs_write_cached_remote_capabilities(wiced_ble_cs_capabilites_t *p_cap);

/**
 * Enable CS security for the connection
 * #WICED_BLE_CS_SET_SECURITY_ENABLE_COMPLETE event is generated on completion of CS security enable for the
 * connection
 *
 * @param acl_conn_handle : ACL connection handle
 *
 * @return wiced_result_t
 */
wiced_result_t wiced_ble_cs_security_enable(uint16_t acl_conn_handle);

/**
 * Write default CS settings for the connection
 * #WICED_BLE_CS_WRITE_DEFAULT_SETTINGS_COMPLETE is generated on completion of this API.
 *
 * @param p_def : Default settings to be applied for the connection
 *
 * @return wiced_result_t
 */
wiced_result_t wiced_ble_cs_set_default_settings(wiced_ble_cs_default_settings_t *p_def);

/**
 * Read remote FAE table for the connection
 * #WICED_BLE_CS_READ_REMOTE_FAE_TABLE_COMPLETE event is generated on read complete
 *
 * @param acl_conn_handle : ACL connection handle
 *
 * @return wiced_result_t
 */
wiced_result_t wiced_ble_cs_read_remote_fae_table(uint16_t acl_conn_handle);

/**
 * Write cached remote FAE table
 * #WICED_BLE_CS_WRITE_CACHED_REMOTE_FAE_TABLE_COMPLETE is generated on completion of this API.
 *
 * @param p_tab : Cached FAE table, previously received from the remote device
 *
 * @return wiced_result_t
 */
wiced_result_t wiced_ble_cs_write_cached_remote_fae_table(wiced_ble_cs_fae_table_data_t *p_tab);

/**
 * Create/Update CS Config for config id
 * #WICED_BLE_CS_CONFIG_COMPLETE is generated on completion of this API.
 *
 * @param p_cfg : Cached FAE table, previously received from the remote device
 *
 * @return wiced_result_t
 */
wiced_result_t wiced_ble_cs_create_config(wiced_ble_cs_config_t *p_cfg);

/**
 * Remove CS Config for config id
 * #WICED_BLE_CS_CONFIG_COMPLETE is generated on completion of this API.
 *
 * @param acl_conn_handle : ACL connection handle
 * @param config_id : CS config identifier
 *
 * @return wiced_result_t
 */
wiced_result_t wiced_ble_cs_remove_config(uint16_t acl_conn_handle, uint8_t config_id);

/**
 * Set the CS channel classification
 * #WICED_BLE_CS_SET_CHANNEL_CLASSIFICATION_COMPLETE is generated on completion of this API.
 *
 * @param channel_map : Channel map to be used
 *
 * @return wiced_result_t
 */
wiced_result_t wiced_ble_cs_set_channel_classification(wiced_ble_cs_channel_map_t channel_map);

/**
 * Set the CS procedure parameters
 * #WICED_BLE_CS_PROCEDURE_ENABLE_CMD_COMPLETE is generated on completion of this API.
 * @note This API can be invoked only if the host has created a configuration using the
 * \ref wiced_ble_cs_create_config API
 *
 * @param p_param : Procedure paramters to be set
 *
 * @return wiced_result_t
 */
wiced_result_t wiced_ble_cs_set_procedure_params(wiced_ble_cs_procedure_params_t *p_param);

/**
 * Enable/Disable CS procedure
 * #WICED_BLE_CS_PROCEDURE_ENABLE_COMPLETE_EVT is generated on completion of this API.
 * @note This API can be invoked only if the host has created a configuration using the
 * \ref wiced_ble_cs_create_config API
 *
 * @param acl_conn_handle : ACL connection handle
 * @param config_id: CS config identifier
 * @param enable: Set 1 to enable the procedure, 0 to disable
 *
 * @return wiced_result_t
 */
wiced_result_t wiced_ble_cs_set_procedure_enable(uint16_t acl_conn_handle, uint8_t config_id, uint8_t enable);

/**
 * Utility function to return an ascii string for the event
 * @param evt: CS event received
 *
 * @return event_string
 */
const char *wiced_ble_cs_get_event_str(wiced_ble_cs_events_t evt);

/**
 * Utility function to read the subevent result
 * @param[out] p_hdr: On return this contains the parsed data from the event
 * @param[in] p_data : Received subevent data
 * @param[in] len : length of the data pointed to by \p p_data
 *
 * @return number of bytes read from the stream
 */

int wiced_ble_cs_read_subevent_result_event_hdr_from_stream(
    wiced_ble_cs_subevent_result_hdr_t *p_hdr, uint8_t *p_data, int len);

/**
 * Utility function to read the subevent continue result
 * @param[out] p_hdr: On return this contains the parsed data from the event
 * @param[in] p_data : Received subevent data
 * @param[in] len : length of the data pointed to by \p p_data
 *
 * @return number of bytes read from the stream
 */
int wiced_ble_cs_read_subevent_cont_result_event_hdr_from_stream(
    wiced_ble_cs_subevent_continue_result_hdr_t *p_hdr, uint8_t *p_data, int len);

/**@} wicedbt */
#endif
