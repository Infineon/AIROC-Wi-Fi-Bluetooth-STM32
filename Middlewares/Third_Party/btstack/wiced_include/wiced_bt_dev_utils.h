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

#ifndef __WICED_BT_DEV_UTILS_H__
#define __WICED_BT_DEV_UTILS_H__

/**
 * @addtogroup wicedbt_dev_utils Wiced Dev Utils
 * @{
 */

/**
 * Asynchronous operation complete callback.
 *
 * @param p_data            : Operation dependent data
 *
 * @return void
 */
typedef void(wiced_bt_dev_cmpl_cback_t)(void *p_data);

/** Structure returned with Vendor Specific Command complete callback */
typedef struct
{
    uint16_t opcode;      /**< Vendor specific command opcode */
    uint16_t param_len;   /**< Return parameter length        */
    uint8_t *p_param_buf; /**< Return parameter buffer (Contains Command Specific data) */
} wiced_bt_dev_vendor_specific_command_complete_params_t;

/**
 * Vendor specific command complete.
 *
 * @param p_command_complete_params : Command complete parameters.
 *
 * @return void
 */

typedef void(wiced_bt_dev_vendor_specific_command_complete_cback_t)(
    wiced_bt_dev_vendor_specific_command_complete_params_t *p_command_complete_params);


/****************************************************************************/

/** TX Power Result   (in response to #wiced_bt_dev_read_tx_power) */
typedef struct
{
    wiced_result_t                  status;             /**< Status of the operation */
    uint8_t                         hci_status;         /**< Status from controller (Refer Spec 5.0 Vol 2 Part D Error Codes) */
    int8_t                          tx_power;           /**< TX power in dB */
    wiced_bt_device_address_t       rem_bda;            /**< Remote BD address */
} wiced_bt_tx_power_result_t;

/** Transmit Power Range Result Buffer */
typedef struct
{
    uint8_t status;                    /**< Command status, see list of HCI Error codes in core spec */
    int8_t max_tx_power;               /**< Maximum Transmit power in dB */
    int8_t min_tx_power;               /**< Miminum Transmit power in dB */
} wiced_bt_transmit_power_range_res_buf_t;

/** Transmit Power Range Result (in response to #wiced_bt_set_transmit_power_range) */
typedef struct
{
    uint16_t opcode;    /**< Vendor specific command opcode */
    uint16_t param_len; /**< Return parameter length        */
    wiced_bt_transmit_power_range_res_buf_t
        *p_param_buf; /**< Return parameter buffer */
} wiced_bt_set_transmit_power_range_result_t;

/** TX Power Result (in response to #wiced_bt_ble_set_adv_tx_power) */
typedef struct
{
    uint16_t    un_used1;                     /**< Unused */
    uint16_t    un_used2;                     /**< Unused */
    uint8_t     *p_param_buf;                 /**< Command status, see list of HCI Error codes in core spec*/
} wiced_bt_set_adv_tx_power_result_t;

/**
 * @endcond // DUAL_MODE
*/

/** BR packets statistics details */
typedef struct
{
    uint16_t    null_count;        /**< No.of NULL packets received/transmitted */
    uint16_t    poll_count;        /**< No.of POLL packets received/transmitted */
    uint16_t    dm1_count;         /**< No.of DM1 packets received/transmitted  */
    uint16_t    dh1_count;         /**< No.of DH1 packets received/transmitted  */
    uint16_t    dv_count;          /**< No.of DV packets received/transmitted   */
    uint16_t    aux1_count;        /**< No.of AUX1 packets received/transmitted */
    uint16_t    dm3_count;         /**< No.of DM3 packets received/transmitted  */
    uint16_t    dh3_count;         /**< No.of DH3 packets received/transmitted  */
    uint16_t    dm5_count;         /**< No.of DM5 packets received/transmitted  */
    uint16_t    dh5_count;         /**< No.of DH5 packets received/transmitted  */
} wiced_bt_dev_br_packet_types_t;

/** EDR packets statistics types details */
typedef struct
{
    uint16_t    null_count;        /**< No.of NULL packets received/transmitted  */
    uint16_t    poll_count;        /**< No.of POLL packets received/transmitted  */
    uint16_t    dm1_count;         /**< No.of DM1 packets received/transmitted   */
    uint16_t    _2_dh1_count;      /**< No.of 2DH1 packets received/transmitted  */
    uint16_t    _3_dh1_count;      /**< No.of 3DH1 packets received/transmitted  */
    uint16_t    _2_dh3_count;      /**< No.of 2DH3 packets received/transmitted  */
    uint16_t    _3_dh3_count;      /**< No.of 3DH3 packets received/transmitted  */
    uint16_t    _2_dh5_count;      /**< No.of 2DH5 packets received/transmitted  */
    uint16_t    _3_dh5_count;      /**< No.of 3DH5 packets received/transmitted  */
    uint16_t    not_used;          /**< Not Used */
}wiced_bt_dev_edr_packet_types_t;

/** BR SCO packets statistics types details */
typedef struct
{
    uint16_t null_count;    /**< No.of NULL packets received/transmitted  */
    uint16_t poll_count;    /**< No.of POLL packets received/transmitted  */
    uint16_t hv1;           /**< No.of HV1 packets received/transmitted   */
    uint16_t hv2;           /**< No.of HV2 packets received/transmitted   */
    uint16_t hv3;           /**< No.of HV3 packets received/transmitted   */
    uint16_t dv;            /**< No.of DV packets received/transmitted    */
    uint16_t ev3;           /**< No.of EV3 packets received/transmitted   */
    uint16_t ev4;           /**< No.of EV4 packets received/transmitted   */
    uint16_t ev5;           /**< No.of EV5 packets received/transmitted   */
} wiced_bt_dev_br_sco_packet_types_t;

/** EDR SCO packets statistics types details */
typedef struct
{
    uint16_t null_count;    /**< No.of NULL packets received/transmitted  */
    uint16_t poll_count;    /**< No.of POLL packets received/transmitted  */
    uint16_t _2_ev3;        /**< No.of 2EV3 packets received/transmitted   */
    uint16_t _3_ev3;        /**< No.of 3EV3 packets received/transmitted   */
    uint16_t _2_ev5;        /**< No.of 2EV5 packets received/transmitted   */
    uint16_t _3_ev5;        /**< No.of 3EV5 packets received/transmitted   */
} wiced_bt_dev_esco_packet_types_t;



/** BR/EDR packet types detail statistics */
typedef union
{
    uint16_t  array[10];               /**< Statistic Arrary */
    /** BR packets statistics details */
    wiced_bt_dev_br_packet_types_t   br_packet_types;      /**< BR packet statastics  */
    wiced_bt_dev_edr_packet_types_t  edr_packet_types;     /**< EDR packet statastics  */
    wiced_bt_dev_br_sco_packet_types_t sco_packet_types;        /**< basic SCO packet statastics  */
    wiced_bt_dev_esco_packet_types_t edr_sco_types;  /**< EDR SCO packet statastics  */
} wiced_bt_br_edr_pkt_type_stats;

/** BR/EDR link statistics */
typedef struct
{
    wiced_bt_br_edr_pkt_type_stats rx_pkts;  /**< Received packets details */
    wiced_bt_br_edr_pkt_type_stats tx_pkts;  /**< Transmitted packets details */
    uint32_t rx_acl_bytes;        /**< Total Received ACL bytes */
    uint32_t tx_acl_bytes;        /**< Total Transmitted ACL bytes */
    uint16_t hec_errs;           /**< hecErrs packet count */
    uint16_t crc_errs;           /**< crcErrs packet count */
    uint16_t seqn_repeat;        /**< seqnRepeat packet count */
    uint16_t acl_soft_reset;      /**< aclSoftReset packet count */
    uint16_t test_mode_tx_pkts;    /**< testModeTxPkts packet count */
    uint16_t test_mode_rx_pkts;    /**< testModeRxPkts packet count */
    uint16_t test_mode_errors;    /**< testModeErrors packet count */
}wiced_bt_dev_lq_br_edr_stats;

/** Link statistics for ACL/SCO connections */
typedef struct
{
    uint32_t re_transmit_count;     /**< Retransmit packet count */
    uint16_t re_transmit_percent;   /**< Retransmit percent */
    uint16_t packet_error_rate;     /**< Packet error rate */
}wiced_bt_dev_lq_acl_stats;

/** LE link statistics for LE connections */
typedef struct
{
    uint32_t  tx_pkt_cnt;           /**< Transmit packet count */
    uint32_t  tx_acked_cnt;         /**< Transmit packet acknowledged count */
    uint32_t  rx_good_pkt_cnt;      /**< Received good packet count */
    uint32_t  rx_good_bytes;        /**< Received good byte count */
    uint32_t  rx_all_pkt_sync_to;   /**< All received packet sync timeout count */
    uint32_t  rx_all_pkt_crc_err;   /**< All received packet crc error count */
    uint32_t  sft_rst_cnt;          /**< Number of LE_CONN task software reset counts */
    uint32_t  evt_flow_off_cnt;    /**< Number of event flow off counts. */
    uint32_t  evt_alarm_hi_tsk_prioty_cnt; /**< Number of LE_CONN task alarm to high priority counts. */
}wiced_bt_dev_lq_le_stats;


/** LE link statistics for LE connections */
typedef struct
{
    wiced_bt_dev_lq_acl_stats acl_lq_stats;  /**< Link statistics for ACL connections */
    wiced_bt_dev_lq_le_stats le_lq_stats;   /**< Link statistics for LE connections */
}wiced_bt_dev_lq_le_acl_stats;

/** Link Quality statistics action type */
enum wiced_bt_dev_lq_conn_type_e
{
    WICED_LQ_CONN_TYPE_BLE = 0,      /**< connection_type BLE*/
    WICED_LQ_CONN_TYPE_ACL = 1,      /**< connection_type BR/EDR*/
    WICED_LQ_CONN_TYPE_SCO_ESCO = 2, /**<  connection_type ACL */
};
typedef uint8_t wiced_bt_dev_lq_conn_type_t; /**< Link Quality Statistic Action ( see #wiced_bt_dev_lq_conn_type_e ) */

/** Link Quality statistics action type */
enum wiced_bt_dev_lq_action_e
{
    WICED_CLEAR_LQ_STATS            = 0,  /**< clear link quality stats */
    WICED_READ_LQ_STATS             = 1,  /**< read link quality stats */
    WICED_READ_THEN_CLEAR_LQ_STATS  = 2,  /**< read then clear link quality stats */
};
typedef uint8_t wiced_bt_lq_action_t;   /**< Link Quality Statistic Action ( see #wiced_bt_dev_lq_action_e ) */

/** LQ Quality Result (in response to wiced_bt_dev_lq_stats) */
typedef struct
{
    uint8_t   status;             /**< event status */
    uint16_t  conn_handle;        /**< connection handle of link quality stats */
    wiced_bt_lq_action_t   action;             /**< see #wiced_bt_dev_lq_action_e for options */
    wiced_bt_dev_lq_conn_type_t   conn_type;             /**< Type of connection */
    /** LQ Quality Statistics */
    union
    {
        wiced_bt_dev_lq_br_edr_stats br_edr_stats; /**< br edr statistics */
        wiced_bt_dev_lq_le_acl_stats     lq_le_acl_stats;     /**< le statistics */
    }wiced_bt_lq_stats;
} wiced_bt_lq_stats_result_t;

/**
 * @}
 */

#ifdef __cplusplus
extern "C"
{
#endif

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
 * Verify if the given bd_addr matches with the local bd_addr
 *
 * @param[in]      bd_addr    :  remote bd address
 *
 * @return         TRUE if bd_addr is same as local_bd_addr,FALSE otherwise
 *
 */
wiced_bool_t wiced_bt_dev_bdaddr_is_local(wiced_bt_device_address_t bd_addr);

/**
*
* Compare two BD address
*
* @param[in]      bd_addr_1    :  bd address
* @param[in]      bd_addr_2    :  bd address to be compared with bd_addr_1
*
* Returns         TRUE if both bd_addr are same,
*                 FALSE if different
*
*/
wiced_bool_t wiced_bt_dev_bdaddr_is_same(wiced_bt_device_address_t bd_addr_1, wiced_bt_device_address_t bd_addr_2);

/**
* Is controller address resolution enabled
*
*
* Returns         TRUE if enabled
*
*/
wiced_bool_t wiced_bt_dev_is_address_resolution_enabled(void);

/**
* Is device privacy supported
*
*
* Returns         TRUE if supported
*
*/
wiced_bool_t wiced_bt_dev_is_privacy_supported(void);

/**
*  This function turns OFF/ON SMP over BR/EDR (i.e. link keys crosspairing SC BR/EDR->SC LE) for the remote device.
*  If mode is set to TRUE then the crosspairing will not happen.
*
* @param[in]  mode :  Set to TRUE to disable support for smp on br.
*
* Returns void
*
*/
void wiced_bt_dev_set_no_smp_on_br(wiced_bool_t mode);

/**
 *  Command to set the tx power on link
 *  This command will adjust the transmit power attenuation on a per connection basis.
 *
 * @param[in]       bd_addr       : peer address
 *                                  To set Adv Tx power keep bd_addr NULL
 * @param[in]       power          :  power value in db
 * @param[in]       p_cb           :  Result callback (wiced_bt_set_adv_tx_power_result_t will be passed to the callback)
 *
 * @return          wiced_result_t
 *
 **/
wiced_result_t wiced_bt_set_tx_power ( wiced_bt_device_address_t bd_addr , int8_t power, wiced_bt_dev_vendor_specific_command_complete_cback_t *p_cb );

/**
 *  Command to set the range of transmit power on the link
 *  This command will set minimum and maximum values for transmit power on a per connection basis.
 *
 * @note            This API is supported by generic target of CYW20829B0 and CYW89829B0 controllers only.
 *
 * @param[in]       bd_addr       : peer address of device
 * @param[in]       max_tx_power  : maximum power value in db
 * @param[in]       min_tx_power  : minimum power value in db
 * @param[in]       p_cb          : Result callback (wiced_bt_set_transmit_power_range_result_t will be passed to the callback)
 *
 * @return          wiced_result_t
 *
 **/
wiced_result_t wiced_bt_set_transmit_power_range ( wiced_bt_device_address_t bd_addr,
                                                   int8_t max_tx_power,
                                                   int8_t min_tx_power,
                                                   wiced_bt_dev_vendor_specific_command_complete_cback_t *p_cb);

/**
 * Read the transmit power for the requested link
 *
 * @param[in]       remote_bda      : BD address of connection to read tx power
 * @param[in]       transport       : Transport type
 * @param[in]       p_cback         : Result callback (wiced_bt_tx_power_result_t will be passed to the callback)
 *
 * @return
 *
 * <b> WICED_BT_PENDING </b>      : if command issued to controller. \n
 * <b> WICED_BT_NO_RESOURCES </b> : if couldn't allocate memory to issue command \n
 * <b> WICED_BT_UNKNOWN_ADDR </b> : if no active link with bd addr specified \n
 * <b> WICED_BT_BUSY </b>         : if command is already in progress
 *
 */
wiced_result_t wiced_bt_dev_read_tx_power (wiced_bt_device_address_t remote_bda, wiced_bt_transport_t transport,
                                            wiced_bt_dev_cmpl_cback_t *p_cback);
    /**
    * @}
    */

#ifdef __cplusplus
}
#endif


#endif //__WICED_BT_DEV_UTILS_H__
