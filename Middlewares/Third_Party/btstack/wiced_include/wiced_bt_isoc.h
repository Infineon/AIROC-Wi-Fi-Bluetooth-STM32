/*
 * Copyright 2021-2025, Cypress Semiconductor Corporation or
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
 * AIROC Bluetooth Low Energy (LE) Isochronous function
 *
 *  Some Acronyms and Abbreviations :
 *      CIS   -  Connected Isochronous Stream
 *      CIG   -  Connected Isochronous Group
 *      BIS   -  Broadcast Isochronous Stream
 *      BIG   -  Broadcast Isochronous Group
 *      ISO   -  Isochronous
 *      ISOAL -  Isochronous Adaption Layer
 */
#ifndef __WICED_BT_ISOC_H__
#define __WICED_BT_ISOC_H__


#include "wiced_bt_types.h"
#include "wiced_result.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_adv_scan_common.h"

/**
 * @defgroup  wicedbt_isoc        Isochronous (ISOC)
 *
 * This section describes the API's to use ISOC functionality.
 *
*/

/**
 * @addtogroup  wicedbt_isoc_defs         ISOC data types and macros
 * @ingroup     wicedbt_isoc
 *
 * @{
*/
/*****************************************************************************
**  constants
*****************************************************************************/

#define WICED_BLE_ISOC_MIN_TRANSPORT_LATENCY      0x0005     /**< ISOC Minimum Latency */
#define WICED_BLE_ISOC_MAX_TRANSPORT_LATENCY      0x0FA0     /**< ISOC Maximum Latency */

/**< Data_Path_Direction */
typedef enum
{
    WICED_BLE_ISOC_DPD_INPUT = 0,     // ISO driver is source (Input data path (Host to Controller))
    WICED_BLE_ISOC_DPD_OUTPUT,        // ISO driver is sink (Output data path (Controller to Host))
    WICED_BLE_ISOC_DPD_MAX_DIRECTIONS // must be last
} wiced_ble_isoc_data_path_direction_t;

/**< Data path direction bits */
typedef enum
{
    WICED_BLE_ISOC_DPD_UNUSED = 0, /**< unused value */
    WICED_BLE_ISOC_DPD_INPUT_BIT = 1, /**< Input bit enabled */
    WICED_BLE_ISOC_DPD_OUTPUT_BIT = 2, /**< Output bit enabled */
    WICED_BLE_ISOC_DPD_INPUT_OUTPUT_BIT = WICED_BLE_ISOC_DPD_INPUT_BIT | WICED_BLE_ISOC_DPD_OUTPUT_BIT, /**< both input and output enabled */
    WICED_BLE_ISOC_DPD_RESERVED                                       /**< reserved */
} wiced_ble_isoc_data_path_bit_t;

#define ISOC_SET_DATA_PATH_DIR(var, dir) (var |= (1 << dir)) /**< Macro to set direction bits */
#define ISOC_GET_DATA_PATH_DIR(var, dir) (var & (1 << dir))  /**< Macro to get direction bits */
#define ISOC_CLEAR_DATA_PATH_DIR(var, dir) (var &= ~(1 << dir)) /**< Macro to clear direction bits */

/**< Data path identifier */
typedef enum
{
    WICED_BLE_ISOC_DPID_HCI = 0, /**< Data path HCI */
    WICED_BLE_ISOC_DPID_DIABLED = 0xFF /**< Data path disabled */
} wiced_ble_isoc_data_path_id_t;

/** ISOC packing methods */
enum wiced_ble_isoc_packing_e
{
    WICED_BLE_ISOC_SEQUENTIAL_PACKING = 0, /**< Sequential packing */
    WICED_BLE_ISOC_INTERLEAVED_PACKING = 1 /**< Interleaved packing */
};
typedef uint8_t wiced_ble_isoc_packing_t; /**< ISOC packing methods (see #wiced_ble_isoc_packing_e) */


/** ISOC Events */
enum wiced_ble_isoc_event_e
{
    WICED_BLE_ISOC_SET_CIG_CMD_COMPLETE_EVT,  /**< SET CIG Command Response, \ref wiced_ble_isoc_set_cig_cmd_status_evt_t */
    WICED_BLE_ISOC_CIS_REQUEST_EVT,           /**< CIS connection Requested, \ref wiced_ble_isoc_cis_request_evt_t */
    WICED_BLE_ISOC_CIS_ESTABLISHED_EVT,       /**< CIS connection established, \ref wiced_ble_isoc_cis_established_evt_t */
    WICED_BLE_ISOC_SCA_EVT,                   /**< Sleep Clock Accuracy, \ref wiced_ble_isoc_sca_event_evt_t */
    WICED_BLE_ISOC_CIS_DISCONNECTED_EVT,      /**< CIS disconnected, \ref wiced_ble_isoc_cis_disconnect_evt_t */
    WICED_BLE_ISOC_DATA_PATH_SETUP_EVT,       /**< CIS Data path status, \ref wiced_ble_isoc_setup_data_path_evt_t */
    WICED_BLE_ISOC_DATA_PATH_REMOVED_EVT,     /**< CIS Data path status, \ref wiced_ble_isoc_removed_data_path_evt_t */
    WICED_BLE_ISOC_BIG_CREATED_EVT,           /**< BIG Connected, \ref wiced_ble_isoc_create_big_cmpl_evt_t */
    WICED_BLE_ISOC_BIG_SYNC_ESTABLISHED_EVT,  /**< BIG Sync Established, \ref wiced_ble_isoc_big_sync_established_evt_t */
    WICED_BLE_ISOC_BIG_TERMINATED_EVT,        /**< BIG Terminated, \ref wiced_ble_isoc_terminated_evt_t */
    WICED_BLE_ISOC_BIG_SYNC_LOST_EVT,         /**< BIG Sync Lost, \ref wiced_ble_isoc_terminated_evt_t */
};
typedef uint8_t wiced_ble_isoc_event_t; /**< ISOC Events (see #wiced_ble_isoc_event_e) */

/**
 * ISOC stream configuration
 */
typedef struct
{
    uint8_t max_cis; /**< Max number of simultaneous CIS(Connected Isochronous Streams) across all CIGs(Connected Isochronous Group)s */
    uint8_t max_bis; /**< Max number of simultaneous BIS(Broadcast Isochronous Streams) across all BIGs(Broadcast Isochronous Group)s */
} wiced_ble_isoc_cfg_t;

/** ISOC Set CIG Command Status data
*  Returned with \ref WICED_BLE_ISOC_SET_CIG_CMD_COMPLETE_EVT event
*/
typedef struct
{
    uint8_t status;                       /**< CIG Establishment Status  (0 = Success).
                                               Refer Core Spec v5.2 [Vol 1] Part F, Controller Error Codes */
    uint8_t cig_id;                       /**< CIG ID */
    uint8_t cis_count;                    /**< CIS Count */
    uint16_t *cis_connection_handle_list; /**< CIS Connection Handle List */
} wiced_ble_isoc_set_cig_cmd_status_evt_t;


/** ISOC CIS Request event data
* Returned with \ref WICED_BLE_ISOC_CIS_REQUEST_EVT event
*/
typedef struct
{
    uint16_t    cis_conn_handle;        /**< CIS Connection Handle */
    uint16_t    acl_conn_handle;        /**< ACL Connection Handle */
    uint8_t     cig_id;                 /**< CIG ID */
    uint8_t     cis_id;                 /**< CIS ID */
}wiced_ble_isoc_cis_t;


/** ISOC CIS Established event data
* Returned with \ref WICED_BLE_ISOC_CIS_ESTABLISHED_EVT event
*/
typedef struct
{
    uint8_t              status;           /**< CIG Establishment Status  (0 = Success). Refer Core Spec v5.2 [Vol 1] Part F, Controller Error Codes */
    wiced_ble_isoc_cis_t  cis;               /**< CIS information */
    uint32_t             cig_sync_delay;   /**< CIG Sync Delay in microseconds */
    uint32_t             cis_sync_delay;   /**< CIS Sync Delay in microseconds */
    uint32_t             latency_c_to_p;   /**< Maximum time, in microseconds, for an SDU to be transported from the central Controller to peripheral Controller */
    uint32_t             latency_p_to_c;   /**< Maximum time, in microseconds, for an SDU to be transported from the peripheral Controller to central Controller */
    wiced_ble_isoc_phy_t  phy_c_to_p;       /**< The transmitter PHY of packets from the central */
    wiced_ble_isoc_phy_t  phy_p_to_c;       /**< The transmitter PHY of packets from the peripheral */
    uint8_t              nse;              /**< Maximum Number of Subevent in each isochronous event */
    uint8_t              bn_c_to_p;        /**< Burst number for central to peripheral transmission */
    uint8_t              bn_p_to_c;        /**< Burst number for peripheral to central transmission */
    uint8_t              ft_c_to_p;        /**< Flush timeout, in multiples of the ISO_Interval for central to peripheral transmission */
    uint8_t              ft_p_to_c;        /**< Flush timeout, in multiples of the ISO_Interval for peripheral to central transmission */
    uint16_t             max_pdu_c_to_p;   /**< Maximum size, in bytes, of an SDU from the central’s Host */
    uint16_t             max_pdu_p_to_c;   /**< Maximum size, in octets, of an SDU from the peripheral’s Host */
    uint16_t             iso_interval;     /**< Time between two consecutive CIS anchor points */
} wiced_ble_isoc_cis_established_evt_t;


/** ISOC Peer SCA data
* Returned with \ref WICED_BLE_ISOC_SCA_EVT event
*/
typedef struct
{
    uint8_t status; /**< SCA Status (0 = Success). Refer Core Spec v5.2 [Vol 1] Part F, Controller Error Codes*/
    uint16_t acl_conn_handle;           /**< ACL Connection Handle */
    wiced_bt_device_address_t peer_bda; /**< Peer Bluetooth Address */
    uint8_t sca;                        /**< Sleep Clock Accuracy value */
} wiced_ble_isoc_sca_event_evt_t;

/** ISOC CIS disconnect event data
* Returned with \ref WICED_BLE_ISOC_CIS_DISCONNECTED_EVT event
*/
typedef struct
{
    wiced_ble_isoc_cis_t cis;  /**< CIS Info */
    uint8_t reason;           /**< Disconnection Reason */
} wiced_ble_isoc_cis_disconnect_evt_t;

/** ISOC setup data path event data
* Returned with \ref WICED_BLE_ISOC_DATA_PATH_SETUP_EVT event
*/
typedef struct
{
    uint8_t status;     /**< Data path Status  (0 = Success). Refer Core Spec v5.2 [Vol 1] Part F, Controller Error Codes */
    uint16_t conn_hdl;  /**< CIS/BIS Connection Handle  */
    void *p_app_ctx;    /**< Application callback context */
} wiced_ble_isoc_setup_data_path_evt_t;

/** ISOC remove data path event data
* Returned with \ref WICED_BLE_ISOC_DATA_PATH_REMOVED_EVT event
*/
typedef struct
{
    uint8_t status; /**< Data path Status  (0 = Success). Refer Core Spec v5.2 [Vol 1] Part F, Controller Error Codes */
    uint16_t conn_hdl; /**< CIS/BIS Connection Handle  */
} wiced_ble_isoc_removed_data_path_evt_t;

/** ISOC BIG Sync Establishment data */
typedef struct
{
    uint8_t        status;                       /**< Create BIG Status (0 = Success). Refer Core Spec v5.2 [Vol 1] Part F, Controller Error Codes */
    uint8_t        big_handle;                   /**< BIG Handle */
    uint32_t       trans_latency;                /**< The maximum delay time, in microseconds, for transmission of SDUs of all BISes in a BIG event */
    uint8_t        number_of_subevents;          /**< The number of subevents in each BIS event in the BIG */
    uint8_t        burst_number;                 /**< The number of new payloads in each BIS event */
    uint8_t        pretransmission_offset;       /**< Offset used for pre-transmissions */
    uint8_t        immediate_repetition_count;   /**< The number of times a payload is transmitted in a BIS event */
    uint16_t       max_pdu;                      /**< Maximum size, in octets, of the payload */
    uint16_t       iso_interval;                 /**< The time between two consecutive BIG anchor points. Time = N * 1.25 ms */
    uint8_t        num_bis;                      /**< Total number of BISes in the BIG */
    uint16_t       *bis_conn_hdl_list;           /**< The connection handles of the BISes */
}wiced_ble_isoc_big_sync_established_evt_t;

/** ISOC BIG Command Status data
* Returned with \ref WICED_BLE_ISOC_BIG_CREATED_EVT event
*/
typedef struct
{
    wiced_ble_isoc_big_sync_established_evt_t sync_data; /**< BIG Sync Data */
    uint32_t sync_delay;                                /**< BIG Sync Delay in microseconds */
    wiced_ble_isoc_phy_t phy;                            /**< The transmitter PHY of packets */
} wiced_ble_isoc_create_big_cmpl_evt_t;


/** ISOC BIG Terminate/Sync_Lost event data
* Returned with \ref WICED_BLE_ISOC_BIG_TERMINATED_EVT and
* \ref WICED_BLE_ISOC_BIG_SYNC_LOST_EVT events
*/
typedef struct
{
    uint8_t big_handle; /**< BIG Handle */
    uint8_t reason;     /**< Reason for termination. Refer Core Spec v5.2 [Vol 1] Part F, Controller Error Codes */
} wiced_ble_isoc_terminated_evt_t;


/** ISOC event data */
typedef union
{
    wiced_ble_isoc_set_cig_cmd_status_evt_t cig_status_data;        /**< CIG Command Status */
    wiced_ble_isoc_cis_established_evt_t cis_established_data;      /**< CIS Established */
    wiced_ble_isoc_cis_t cis_request;                               /**< CIS Request     */
    wiced_ble_isoc_sca_event_evt_t sca_data;                        /**< Sleep Clock Accuracy */
    wiced_ble_isoc_cis_disconnect_evt_t cis_disconnect;             /**< CIS Disconnect  */
    wiced_ble_isoc_setup_data_path_evt_t datapath;                  /**< Data Path Status (setup/remove) */
    wiced_ble_isoc_create_big_cmpl_evt_t create_big;                /**< Create BIG Command Status */
    wiced_ble_isoc_terminated_evt_t terminate_big;                  /**< Terminate BIG Command Status */
    wiced_ble_isoc_big_sync_established_evt_t big_sync_established; /**< BIG Sync Established data */
    wiced_ble_isoc_terminated_evt_t big_sync_lost;                  /**< BIG Sync Lost Data */
} wiced_ble_isoc_event_data_t;

/** ISOC CIS Configuration */
typedef struct
{
    /** CIS Id : ZERO if not created*/
    uint8_t             cis_id;
    /** Maximum size, in bytes, of an SDU from the central’s Host, Valid Range 0x000 to 0xFFF */
    uint16_t            max_sdu_c_to_p;
    /** Maximum size, in octets, of an SDU from the peripheral’s Host Valid Range 0x000 to 0xFFF*/
    uint16_t            max_sdu_p_to_c;
    /** The transmitter PHY of packets from the central */
    wiced_ble_isoc_phy_t phy_c_to_p;
    /** The transmitter PHY of packets from the peripheral */
    wiced_ble_isoc_phy_t phy_p_to_c;
    /** Maximum number of times every CIS Data PDU should be retransmitted from the central to peripheral */
    uint8_t             rtn_c_to_p;
    /** Maximum number of times every CIS Data PDU should be retransmitted from the peripheral to central */
    uint8_t             rtn_p_to_c;
} wiced_ble_isoc_cis_config_t;

typedef struct
{
    /** CIS Id : ZERO if not created*/
    uint8_t cis_id;
    /** Maximum number of subevents in each CIS event */
    uint8_t nse;
    /** Maximum size, in bytes, of an SDU from the master□~@~Ys Host Valid Range 0x000 to 0xFFF*/
    uint16_t max_sdu_c_to_p;

    /** Maximum size, in octets, of an SDU from the slave□~@~Ys Host Valid Range 0x000 to 0xFFF*/
    uint16_t max_sdu_p_to_c;
    /** Maximum size, in bytes, of an SDU from the master□~@~Ys Host Valid Range 0x000 to 0xFFF*/
    uint16_t max_pdu_c_to_p;
    /** Maximum size, in octets, of an SDU from the slave□~@~Ys Host Valid Range 0x000 to 0xFFF*/
    uint16_t max_pdu_p_to_c;
    /** The transmitter PHY of packets from the central to peripheral */
    wiced_ble_isoc_phy_t phy_c_to_p;
    /** The transmitter PHY of packets from the peripheral to the central */
    wiced_ble_isoc_phy_t phy_p_to_c;
    /** Maximum number of times every CIS Data PDU should be retransmitted from the master to slave */
    uint8_t bn_c_to_p;
    /** Maximum number of times every CIS Data PDU should be retransmitted from the slave to master */
    uint8_t bn_p_to_c;
} wiced_bt_ble_cis_config_test_t;

/** ISOC CIG Configuration for setting the CIG parameters */
typedef struct
{
    /** CIG ID */
    uint8_t cig_id;
    /**
     * Time interval in microseconds between the start of consecutive SDUs from the central’s Host
     * for all the CISes in the CIG
     */
    uint32_t sdu_interval_c_to_p;
    /**
     * Time interval in microseconds between the start of consecutive SDUs from the peripheral’s
     * Host for all the CISes in the CIG.
     */
    uint32_t sdu_interval_p_to_c;
    /**
     * The Worst_Case_SCA parameter shall be the worst-case sleep clock accuracy of all
     * the Peripherals that will participate in the CIG.
     */
    uint8_t worst_case_sca;
    /** Maximum time, in microseconds, for an SDU to be transported from the central Controller
     * to peripheral Controller
     */
    uint16_t max_trans_latency_c_to_p;
    /**
     * Maximum time, in microseconds, for an SDU to be transported from the peripheral Controller
     * to central Controller
     */
    uint16_t max_trans_latency_p_to_c;
    /** Packing method  */
    wiced_ble_isoc_packing_t packing;
    /** Framing parameter */
    wiced_ble_isoc_framing_t framing;
    /**
     * Total number of CISes in the CIG being added or modified Valid Range 0x00 to 0x10
     */
    uint8_t cis_count;
    /** CIS configurations */
    wiced_ble_isoc_cis_config_t *p_cis_config_list;
} wiced_ble_isoc_cig_param_t;

typedef struct
{
    /** CIG ID if known */
    uint8_t cig_id;
    /** Time interval in microseconds between the start of consecutive SDUs from the master□~@~Ys Host for all the CISes in the CIG */
    uint32_t sdu_interval_c_to_p;
    /** Time interval in microseconds between the start of consecutive SDUs from the slave□~@~Ys Host for all the CISes in the CIG. */
    uint32_t sdu_interval_p_to_c;
    /** The flush timeout in multiples of ISO_Interval for each payload sent from the master to slave. */
    uint8_t ft_c_to_p;
    /** The flush timeout in multiples of ISO_Interval for each payload sent from the slave to master. */
    uint8_t ft_p_to_c;
    /** Time between consecutive CIS anchor points */
    uint16_t iso_interval;
    /**
     * The Worst_Case_SCA parameter shall be the worst-case sleep clock accuracy of all
     * the Peripherals that will participate in the CIG.
     */
    uint8_t worst_case_sca;
    /** Packing method  */
    wiced_ble_isoc_packing_t packing;
    /** Framing parameter */
    wiced_ble_isoc_framing_t framing;
    /** Total number of CISes in the CIG being added or modified Valid Range 0x00 to 0x10 */
    uint8_t cis_count;
    /** CIS configurations */
    wiced_bt_ble_cis_config_test_t *p_cis_config_list;
} wiced_ble_isoc_cig_param_test_t;

/** ISOC CIS Configuration for setting the CIG parameters */
typedef struct
{
    /** CIS Id */
    uint8_t cis_id;
    /** Maximum number of subevents in each CIS event */
    uint8_t nse;
    /** Maximum size, in bytes, of an SDU from the central’s Host Valid Range 0x000 to 0xFFF*/
    uint16_t max_sdu_c_to_p;
    /** Maximum size, in octets, of an SDU from the peripheral’s Host Valid Range 0x000 to 0xFFF*/
    uint16_t max_sdu_p_to_c;
    /** Maximum size, in bytes, of an SDU from the central’s Host Valid Range 0x000 to 0xFFF*/
    uint16_t max_pdu_c_to_p;
    /** Maximum size, in octets, of an SDU from the peripheral’s Host Valid Range 0x000 to 0xFFF*/
    uint16_t max_pdu_p_to_c;
    /** The transmitter PHY of packets from the central */
    wiced_ble_isoc_phy_t phy_c_to_p;
    /** The transmitter PHY of packets from the peripheral */
    wiced_ble_isoc_phy_t phy_p_to_c;
    /** Maximum number of times every CIS Data PDU should be retransmitted from the central to peripheral */
    uint8_t bn_c_to_p;
    /** Maximum number of times every CIS Data PDU should be retransmitted from the peripheral to central */
    uint8_t bn_p_to_c;
} wiced_ble_isoc_cis_config_test_t;

/** Parameters for CIS creation */
typedef struct
{
    uint16_t cis_conn_handle; /**< CIS connection handle */
    uint16_t acl_conn_handle; /**< ACL connection handle */
} wiced_ble_isoc_cis_acl_t;

/** ISOC BIG Configuration for setting BIG parameters */
typedef struct
{
    uint8_t big_handle;                 /**< BIG handle to identify BIG */
    uint8_t adv_handle;                 /**< ADV handle to identify periodic adv train */
    uint8_t num_bis;                    /**< Number of BIS in the BIG */
    uint32_t sdu_interval;              /**< BIG SDU interval */
    uint16_t max_sdu;                   /**< Max size of SDU in octets */
    uint16_t max_trans_latency;         /**< Max transport latency */
    uint8_t rtn;                        /**< Retransmission number */
    wiced_ble_isoc_phy_t phy;            /**< Phy used for ISOC */
    wiced_ble_isoc_packing_t packing;    /**< Packing method */
    wiced_ble_isoc_framing_t framing;    /**< Framing method */
    wiced_ble_isoc_encryption_t encrypt; /**< Encryption used */
    uint8_t broadcast_code[16];         /**< Broadcast code if encryption enabled */
} wiced_ble_isoc_create_big_param_t;

/** ISOC BIG Configuration for setting BIG test parameters */
typedef struct
{
    uint8_t big_handle;                 /**< BIG handle to identify BIG */
    uint16_t sync_handle;               /**< Periodic train identifier */
    wiced_ble_isoc_encryption_t encrypt; /**< Identifies if Broadcast code status, 0 = invalid, 1 = valid */
    uint8_t broadcast_code[16];         /**< Broadcast code */
    uint8_t max_sub_events;             /**< Max sub events */
    uint16_t big_sync_timeout;          /**< BIG Sync timeout */
    uint8_t num_bis;                    /**< Num BIS in the \p bis_idx_list */
    uint8_t *bis_idx_list;              /**< BIS list of size \p num_bis */
} wiced_ble_isoc_big_create_sync_t;


/**< Structure received for tx broadcast sync complete */
typedef struct
{
    /** status of Read Tx Sync command, Refer Core Spec v5.4 [Vol 1] Part F, Controller Error Codes */
    uint8_t status;
    /** CIS/BIS Connection Handle  */
    uint16_t isoc_conn_hdl;
    /** Packet sequence number of an SDU  */
    uint16_t psn;
    /** The CIG reference point or BIG anchor point of a transmitted SDU (in microseconds) */
    uint32_t tx_timestamp;
    /** Time offset that is associated with a transmitted SDU (in microseconds) */
    uint32_t time_offset;
} wiced_ble_isoc_read_tx_sync_complete_t;

/**< Command parameters for \ref wiced_ble_isoc_setup_data_path */
typedef struct
{
    /** CIS/BIS  Connection handle */
    uint16_t isoc_conn_hdl;
    /** Data path to the setup */
    wiced_ble_isoc_data_path_direction_t data_path_dir;
    /** Data path identifier */
    wiced_ble_isoc_data_path_id_t data_path_id;
    /** Controller delay in microseconds, Range: 0x000000 to 0x3D0900 */
    uint32_t controller_delay;
    /** Set the Codec for the data path */
    uint8_t codec_id[5];
    /** Length of the Codec Specific Configuration pointed by \p p_csc*/
    uint8_t csc_length;
    /** Codec Specific Configuration */
    uint8_t *p_csc;
    /** Application provided context, returned to the application */
    void *p_app_ctx;
} wiced_ble_isoc_setup_data_path_info_t;

/** ISOC Data callbacks */
/**
* Callback for receiving ISOC data
*
* @param[in] p_data : incoming ISOC data
* @param[in] length : length of the data
*/
typedef void (*wiced_ble_isoc_rx_cb_t)(uint8_t *p_data, uint32_t length);

/**
* Callback for receiving number of completed packets
*
* @param[in] p_buf : incoming number of completed packets event
*
* @return wiced_bool_t, return FALSE in case any of the connection handles
* in the event are not handled/(ISOC)
*/
typedef wiced_bool_t (*wiced_ble_isoc_num_complete_cb_t)(uint8_t *p_buf);


/******************************************************
 *               Function Declarations
 *
 ******************************************************/
#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief Event callback for ISOC event notifications. Registered using #wiced_ble_isoc_init
 *
 * @param event_id     : ISOC event ID
 * @param p_event_data : ISOC event data
 *
*/
typedef void wiced_ble_isoc_cback_t(wiced_ble_isoc_event_t event_id, wiced_ble_isoc_event_data_t *p_event_data);

/**
 * @brief Event callback to return the read TX Sync information
 *
 * @param p_sync : Sync information
 *
*/
typedef void (wiced_ble_isoc_read_tx_sync_complete_cback_t)(wiced_ble_isoc_read_tx_sync_complete_t *p_sync);

/** @} wicedbt_isoc_defs         */

/**
 * @addtogroup  wicedbt_isoc_functions   Isochronous (ISOC) functions
 * @ingroup     wicedbt_isoc
 *
 * Isochronous(ISOC) Functions.
 *
 * @{
 */

/**
 * @brief ISOC Register event callback handler
 *
 * @param[in] p_cfg : ISOC stream configuration
 * @param[in] isoc_cb  : ISOC event callback
 */
void wiced_ble_isoc_init(wiced_ble_isoc_cfg_t *p_cfg, wiced_ble_isoc_cback_t isoc_cb);

/**
 * @brief Register ISO data event callbacks
 * @note Only for hosted applications, where the application is required to encode/decode the incoming/outgoing ISOC Data
 *
 * @param[in] rx_data_cb Callback upon receiving ISO data
 * @param[in] num_complete_cb Callback after transmitting ISO data
 */
void wiced_ble_isoc_register_data_cb(wiced_ble_isoc_rx_cb_t rx_data_cb, wiced_ble_isoc_num_complete_cb_t num_complete_cb);

/**
 * @brief Called by Central's host to create/set the parameters of one or more CISes that are associated with a CIG
 * identified with the cig_id in the Controller.
 * If the CIG_ID does not exist, then the Controller creates a new CIG. The commandmodifies or adds CIS configurations
 * to the CIG and updates all the parameters that apply to the CIG.
 * The \ref WICED_BLE_ISOC_SET_CIG_CMD_COMPLETE_EVT event is generated to return the status of the API
 *
 * @param[in] p_cig_params: CIG parameter (@ref wiced_ble_isoc_cig_param_t)
 *
 * @return wiced_result_t
 */
wiced_result_t wiced_ble_isoc_central_set_cig_param(wiced_ble_isoc_cig_param_t *p_cig_params);

/**
 * @brief Called by Central's host to create/set the parameters of one or more CISes that are associated with a CIG
 * identified with the cig_id in the Controller.
 * If the CIG_ID does not exist, then the Controller creates a new CIG. The commandmodifies or adds CIS configurations
 * to the CIG and updates all the parameters that apply to the CIG.
 * The \ref WICED_BLE_ISOC_SET_CIG_CMD_COMPLETE_EVT event is generated to return the status of the API
 *
 * @param[in] p_cig_params: CIG parameter (@ref wiced_ble_isoc_cig_param_t)
 *
 * @return wiced_result_t
 */
wiced_result_t wiced_ble_isoc_central_set_cig_param_test(wiced_ble_isoc_cig_param_test_t *p_cig_params);

/**
 * @brief Called by Central's host to create one or more CIS channels. The \ref WICED_BLE_ISOC_CIS_ESTABLISHED_EVT event
 * is generated on completion of the CIS establishment procedure
 *
 * @param[in] cis_count: Specifies the number of cis/acl pairs pointed to by \p p_ca
 * @param[in] p_ca : CIS/ACL connection handle pairs to create CIS
 *
 * @return wiced_result_t
 */
wiced_result_t wiced_ble_isoc_central_create_cis(int cis_count, wiced_ble_isoc_cis_acl_t *p_ca);

/**
 * @brief Called by the Peripheral to accept the CIS (Connected Isochronous Stream) connection request received in the
 * \ref WICED_BLE_ISOC_CIS_REQUEST_EVT event
 * The \ref WICED_BLE_ISOC_CIS_ESTABLISHED_EVT event is generated on completion of the CIS establishment procedure
 *
 * @param[in] p_req : Request data received in \req WICED_BLE_ISOC_CIS_REQUEST_EVT event
 * return wiced_result_t
 *
 */
wiced_result_t wiced_ble_isoc_peripheral_accept_cis(wiced_ble_isoc_cis_t *p_req);

/**
 * @brief Called by the Peripheral to reject the CIS (Connected Isochronous Stream) connection request received in the
 * \ref WICED_BLE_ISOC_CIS_REQUEST_EVT event
 *
 * @param[in]       p_req  : Request data received in \ref WICED_BLE_ISOC_CIS_REQUEST_EVT event
 * @param[in]       reason : Reject Reason
 * @return wiced_result_t
 */
wiced_result_t wiced_ble_isoc_peripheral_reject_cis(wiced_ble_isoc_cis_t *p_req, uint8_t reason);

/**
 * @brief Disconnect CIS (Connected Isochronous Stream) connection. The \ref WICED_BLE_ISOC_CIS_DISCONNECTED_EVT event will be
 *  received by the application on completion of the disconnection procedure
 *
 * @param[in] cis_conn_handle : CIS connection handle received in the \ref WICED_BLE_ISOC_CIS_ESTABLISHED_EVT event
 * @return    status
 */
wiced_result_t wiced_ble_isoc_disconnect_cis(uint16_t cis_conn_handle);

/**
 * @brief Invoked by the Central host to remove the CIG (Connected Isochronous Group) and associated
 * CISs (Connected Isochronous Stream). The call is not allowed to be invoked of any of the associated CIS are in the
 * connected (established) state.
 * @note: It is recommended to invoke this command after disconnecting the last connected CIS inorder to clean up the
 * state variables in the Controller
 *
 * @param[in] cig_id  : CIG ID
 * @return wiced_result_t
 */
wiced_result_t wiced_ble_isoc_central_remove_cig(uint8_t cig_id);

/**
 * @brief Get the CIS connection handle using CIG and CIS identifiers
 *
 * @param cig_id CIG identifier
 * @param cis_id CIS identifier
 * @return uint16_t CIS connection handle
 */
uint16_t wiced_ble_isoc_central_get_cis_conn_handle(uint8_t cig_id, uint8_t cis_id);

/**
 * @brief Get the CIS connection handle using ACL connection handle, CIG and CIS identifiers
 *
 * @param cig_id       :CIG identifier
 * @param cis_id       :CIS identifier
 * @param acl_conn_hdl :ACL connection handle
 * @return uint16_t CIS connection handle
 */
uint16_t wiced_ble_isoc_get_cis_conn_handle(uint8_t cig_id, uint8_t cis_id, uint16_t acl_conn_hdl);

/**
 * @brief Get CIS connection status by CIS conn id
 *
 * @param cis_conn_hdl CIS connection handle
 * @return TRUE if CIS connection exists
 */
wiced_bool_t wiced_ble_isoc_is_cis_connected_with_conn_hdl(uint16_t cis_conn_hdl);

/**
 *
 * @brief Request for Sleep Clock Accuracy. The \ref WICED_BLE_ISOC_SCA_EVT event will be received on
 * completion of the request
 *
 * @param[in] peer_bda : Peer Bluetooth Address
 *
 * @return wiced_result_t
 */
wiced_result_t wiced_ble_isoc_central_request_peer_sca(wiced_bt_device_address_t peer_bda);

/**
 * @brief Called by the Central host (Broadcaster) to create BIG with provided parameters
 *
 * @param p_big_param Number of BIS, SDU Interval, Packing, Framing
 * @return wiced_result_t
 */
wiced_result_t wiced_ble_isoc_central_create_big(wiced_ble_isoc_create_big_param_t *p_big_param);

/**
 * @brief terminate a BIG identified by the BIG_Handle
 *
 * @param big_handle Used to identify the BIG
 * @param reason Reason the BIG is terminated.
 * @return wiced_result_t
 */
wiced_result_t wiced_ble_isoc_central_terminate_big(uint8_t big_handle, uint8_t reason);

/**
 * @brief Sync to the BIS stream described by wiced_ble_isoc_big_create_sync_t
 *
 * @param p_create_sync sync_handle, list of BIS indices
 * @return wiced_bool_t TRUE if successful
 */
wiced_result_t wiced_ble_isoc_peripheral_big_create_sync(wiced_ble_isoc_big_create_sync_t *p_create_sync);

/**
 * @brief Stop synchronizing or cancel the process of synchronizing to
 * the BIG identified by the BIG_Handle
 *
 * @param big_handle Used to identify the BIG
 * @return wiced_result_t wiced_result_t WICED_SUCCESS if successful
 */
wiced_result_t wiced_ble_isoc_peripheral_big_terminate_sync(uint8_t big_handle);

/**
 * wiced_ble_isoc_is_bis_created
 *
 * @param[in]       handle  : BIS Connection handle
 * @return     TRUE if BIG exists
 */
wiced_bool_t wiced_ble_isoc_is_bis_created(uint16_t bis_conn_handle);

/**
 * @brief This function is used to request the Controller to configure the data transport path in a given direction
 * between the Controller and the Host.
 * @note This command is used only ihe offloading use cases, which engage the data path identified by the
 * \p data_path_id parameter defined by the specific controller
 *
 * @param data_path_dir : Direction to be configured
 * @param data_path_id  : Transport data path
 *
 * @return wiced_result_t
 */

wiced_result_t wiced_ble_isoc_configure_data_path(wiced_ble_isoc_data_path_direction_t data_path_dir,
                                                 wiced_ble_isoc_data_path_id_t data_path_id);

/**
 * @brief Set's up the codec parameters and configuration for the CIS/BIS connection handle.
 * For the audio offload use cases (encoding/decoding in the controller), invoke \ref wiced_ble_isoc_configure_data_path
 * to configure the data path \p p_param->data_path_dir direction for data path identifier \p p_param->data_path_id
 *
 * @param p_param : Data path parameters
 * @return wiced_result_t
 */
wiced_result_t wiced_ble_isoc_setup_data_path(wiced_ble_isoc_setup_data_path_info_t *p_param);

/**
 * @brief Remove data path setup for a CIS/BIS
 *
 * @param isoc_conn_hdl CIS/BIS Connection handle
 * @param data_path_dir_bitfield see #wiced_ble_isoc_data_path_bit_t
 *                               bit 0: Remove Input data path
 *                               bit 1: Remove output data path
 * @return wiced_bool_t TRUE if successful in sending the command
 */
wiced_bool_t wiced_ble_isoc_remove_data_path(uint16_t isoc_conn_hdl, wiced_ble_isoc_data_path_bit_t data_path_dir_bitfield);

/**
 * @brief Get status of the ISO CIS/BIS data path
 *
 * @param cig_id CIG identifier
 * @param cis_id CIS identifier
 * @param data_path_dir INPUT/OUTPUT see #wiced_ble_isoc_data_path_direction_t
 * @return wiced_bool_t
 */
wiced_bool_t wiced_ble_isoc_is_data_path_active(uint8_t cig_id,
                                               uint8_t cis_id,
                                               wiced_ble_isoc_data_path_direction_t data_path_dir);


/**
  * This function writes ISO buffer to the lower layer (or controller in the Hosted Stack)
  *
  * Called by application to send the isoc data.
  *
  * @param[in] p_data    : Pointer to the buffer
  * @param[in] len       : Length of data at p_data
  *
  * @return : wiced_bool_t TRUE if successful otherwise FALSE
  */

 wiced_bool_t wiced_ble_isoc_write_data_to_lower(uint8_t *p_data, uint16_t len);

/** This function returns ISOC buffer size */

 uint16_t wiced_ble_isoc_get_max_data_pkt_len(void);

 /**
  * @brief This function is used to read TX Timestamp and Time offset of a transmitted
  * SDU identified by the packet sequence number on the Central or Peripheral.
  *
  * @param isoc_conn_hdl: CIS/BIS Connection handle
  * @param p_cback      : Callabck for receiving return parameters
  *
  * @return             : WICED_SUCCESS if successful
  */
 wiced_result_t wiced_ble_isoc_read_tx_sync(uint16_t isoc_conn_hdl,
                                           wiced_ble_isoc_read_tx_sync_complete_cback_t *p_cback);


/**@} wicedbt_isoc_functions */

#ifdef __cplusplus
}
#endif

#endif //__WICED_BT_ISOC_H__
