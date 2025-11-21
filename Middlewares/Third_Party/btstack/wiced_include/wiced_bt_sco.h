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
 * Bluetooth Synchronous Connection Oriented Channel Application Programming Interface
 *
 */
#ifndef __WICED_BT_SCO_H__
#define __WICED_BT_SCO_H__



/**
 * @cond DUAL_MODE
 * @addtogroup  sco      Synchronous Connection Oriented (SCO) Channel
 * @ingroup     wicedbt_sco
 *
 * @{
 */


/******************************************************
 *              Constants
 ******************************************************/
#define WICED_BT_SCO_CONNECTION_ACCEPT              0x00 /**< Status accept connection */
#define WICED_BT_SCO_CONNECTION_REJECT_RESOURCES    0x0D /**< Status reject connection due to no resources */
#define WICED_BT_SCO_CONNECTION_REJECT_SECURITY     0x0E /**< Status reject connection due to security */
#define WICED_BT_SCO_CONNECTION_REJECT_DEVICE       0x0F /**< Status reject connection from peer */

#ifndef WICED_SCO_PKT_TYPES_MASK
#define WICED_INVALID_SCO_INDEX           0xFFFF    /**< Default SCO index */
#define WICED_SCO_LINK_ALL_PKT_MASK       0x003F    /**< SCO packet type all */
#define WICED_SCO_PKT_TYPES_MASK_HV3      0x0004    /**< SCO packet type HV3 */
#define WICED_SCO_PKT_TYPES_MASK_EV3      0x0008    /**< SCO packet type EV3 */
#define WICED_SCO_PKT_TYPES_MASK_EV4      0x0010    /**< SCO packet type EV4 */
#define WICED_SCO_PKT_TYPES_MASK_EV5      0x0020    /**< SCO packet type EV5 */
#define WICED_SCO_PKT_TYPES_MASK_NO_2_EV3 0x0040    /**< SCO packet type 2-EV3 */
#define WICED_SCO_PKT_TYPES_MASK_NO_3_EV3 0x0080    /**< SCO packet type 3-EV3 */
#define WICED_SCO_PKT_TYPES_MASK_NO_2_EV5 0x0100    /**< SCO packet type 2-EV5 */
#define WICED_SCO_PKT_TYPES_MASK_NO_3_EV5 0x0200    /**< SCO packet type 3-EV5 */
#endif

#define WICED_BT_SCO_NUM_CODECS     2       /**< Number of SCO codecs. This is only used as size of default SCO params array.*/
#define WICED_BT_SCO_IDX_CVSD       0       /**< eSCO setting for CVSD (S4)*/
#define WICED_BT_SCO_IDX_MSBC       1       /**< eSCO setting for mSBC/LC3 (T2) */
#define WICED_BT_SCO_IDX_NONE WICED_BT_SCO_NUM_CODECS /**< Limit index */

typedef uint8_t wiced_bt_sco_idx_t;   /**< SCO codec index */

/** SCO route path */
typedef enum
{
     WICED_BT_SCO_OVER_HCI,         /**< SCO over HCI to AIROC stack in the host */
     WICED_BT_SCO_OVER_PCM    /**< Not supported yet. PCM data config for routing over I2S/PCM interface */
}wiced_bt_sco_route_path_t;

/******************************************************
 *              Type Definitions
 ******************************************************/
#define  WICED_BT_SCO_DATA_CB_GET_LENGTH(ltch_len)   ((ltch_len>>8)&0xff)	/**< SCO data callback length */

/** Call back function for pcm data transfer, ltch_len = (length)<<8|(sco_channel) */
typedef void (wiced_bt_sco_data_cb_t) (uint16_t sco_channel, uint16_t length, uint8_t *p_data);


/** Subset for the enhanced setup/accept synchronous connection parameters
 * See BT 4.1 or later HCI spec for details */
typedef struct
{
#define CODEC_NARROW_BAND       0               /* to signify use of narrow band eg. CVSD */
#define CODEC_WIDE_BAND         1               /* to signify use of wide band eg. mSBC */
#define CODEC_SUPER_WIDE_BAND   2               /* to signify use of super wide band eg. LC3 */
#define CODEC_IS_NARROW_BAND(use_wbs)       (CODEC_NARROW_BAND == use_wbs)          /* wbs_used to indicate narrow band*/
#define CODEC_IS_WIDE_BAND(use_wbs)         (CODEC_WIDE_BAND == use_wbs)            /* wbs_used to indicate wide band*/
#define CODEC_IS_SUPER_WIDE_BAND(use_wbs)   (CODEC_SUPER_WIDE_BAND == use_wbs)      /* wbs_used to indicate super wide band*/


    uint16_t    max_latency;                   /**< Maximum latency (0x4-0xFFFE in msecs) */
    uint16_t    packet_types;                  /**< Packet Types */
    uint8_t     retrans_effort;                /**< 0x00-0x02, 0xFF don't care */
    uint8_t     use_wbs;                       /**< CODEC_WIDE_BAND to signify wide band, CODEC_SUPER_WIDE_BAND for super wide band, CODEC_NARROW_BAND for narrow band */
} wiced_bt_sco_params_t;

/** SCO path config */
typedef struct
{
    wiced_bt_sco_route_path_t    path;           /**< sco routing path see #wiced_bt_sco_route_path_t*/
    uint8_t                      vsc_route;      /**< Vendor Specific Route */
    wiced_bt_sco_data_cb_t       *p_sco_data_cb; /**< If not NULL and route is APP_CB, callback function called for incoming pcm data */
}wiced_bt_voice_path_setup_t;

/** Coding format */
typedef struct
{
    uint8_t   coding_format;      /**< Coding Format see hcidefs.h */
    uint16_t  company_id;         /**< Company ID from BT SIG Assigned Numbers */
    uint16_t  vs_codec_id;        /**< Vendor Specific Codec ID:  */
} wiced_bt_coding_format;

/** Enhanced ESCO params */
typedef struct
{
    uint32_t tx_bw;                           /**< TX BW (# of octets/second) */
    uint32_t rx_bw;                           /**< RX BW (# of octets/second) */
    wiced_bt_coding_format tx_cf;             /**< TX coding format */
    wiced_bt_coding_format rx_cf;             /**< RX coding format */
    uint16_t tx_codec_frame_size;             /**< TX CODEC frame size (OTA frame size) */
    uint16_t rx_codec_frame_size;             /**< RX CODEC frame size (OTA frame size) */
    uint32_t input_bandwidth;                 /**< Input BW (nominal rate octets/sec) */
    uint32_t output_bandwidth;                /**< Output BW (nominal rate octets/sec) */
    wiced_bt_coding_format input_cf;          /**< Input coding format */
    wiced_bt_coding_format output_cf;         /**< Output coding format */
    uint16_t input_coded_data_size;           /**< Input coded data size (in bits) */
    uint16_t output_coded_data_size;          /**< Output coded data size (in bits) */
    uint8_t input_pcm_data_format;            /**< Input PCM data format (see hcidefs.h) */
    uint8_t output_pcm_data_format;           /**< Output PCM data format (see hcidefs.h) */
    uint8_t input_pcm_payload_msb_position;   /**< Input PCM sample payload MSB position */
    uint8_t output_pcm_payload_msb_position;  /**< Output PCM sample payload MSB position */
    uint8_t input_data_path;                  /**< 0x00 - HCI, or 0x01-0xFE for VS) */
    uint8_t output_data_path;                 /**< 0x00 - HCI, or 0x01-0xFE for VS) */
    uint8_t input_transport_unit_size;        /**< Input transport unit size */
    uint8_t output_transport_unit_size;       /**< Output transport unit size */
    uint16_t max_latency;                     /**< Maximum latency (0x4-0xFFFE in msecs) */
    uint16_t packet_types;                    /**< Packet Types */
    uint8_t retrans_effort;                   /**< 0x00-0x02, 0xFF don't care */
} wiced_bt_enh_esco_params_t;

/******************************************************
 *              Function Declarations
 ******************************************************/
#ifdef __cplusplus
extern "C"
{
#endif

/**
 *The wiced_sco_lib.a was required to included before we link this function.
 *Creates a synchronous connection oriented connection as initiator.
 *
 *  @param[in]  bd_addr                 : Peer bd_addr
 *  @param[in]  p_params                : Pointer to the SCO parameter structure
 *  @param[out] p_sco_index             : SCO index returned
 *
 *  @return     <b> WICED_BT_UNKNOWN_ADDR </b>      : Create connection failed, ACL connection is not up
 *              <b> WICED_BT_BUSY </b>              : Create connection failed, another SCO is being
 *                                                    conncted to the same BD address
 *              <b> WICED_BT_WRONG_MODE </b>        : Create connection failed, wrong mode
 *              <b> WICED_BT_NO_RESOURCES </b>      : Create connection failed, max SCO limit has been
 *                                                    reached
 *              <b> WICED_BT_PENDING </b>            : Create connection successfully, "p_sco_index" is returned
 */
wiced_bt_dev_status_t wiced_bt_sco_create_as_initiator (wiced_bt_device_address_t bd_addr,
                                                        uint16_t *p_sco_index,
                                                        wiced_bt_sco_params_t *p_params);

/**
 * Creates a synchronous connection oriented connection as acceptor.
 *
 *  @param[in]  remote_bda              :  remote device bd_addr
 *  @param[out] p_sco_index             : SCO index returned
 *
 *  @return     <b> WICED_BT_UNKNOWN_ADDR </b>      : Create connection failed, ACL connection is not up or
 *                                                    address is invalid
 *              <b> WICED_BT_BUSY </b>              : Create connection failed, a SCO connection is already
 *                                                    conncted to the same BD address
 *              <b> WICED_BT_WRONG_MODE </b>        : Create connection failed, link in park mode or
 *                                                    automatic un-park is not supported
 *              <b> WICED_BT_NO_RESOURCES </b>      : Create connection failed, max SCO limit has been
 *                                                    reached
 *              <b> WICED_BT_PENDING </b>            : Create connection successfully, "p_sco_index" is returned
 */
wiced_bt_dev_status_t wiced_bt_sco_create_as_acceptor_ex (wiced_bt_device_address_t remote_bda, uint16_t *p_sco_index);

/**   defined this macro for backward compatiblity */
#define wiced_bt_sco_create_as_acceptor(p_sco_index) wiced_bt_sco_create_as_acceptor_ex(NULL, p_sco_index)


/**
 * Removes a specific synchronous connection oriented connection.
 *
 *  @param[in]  sco_index                           : SCO index to remove
 *
 *  @return     <b> WICED_BT_UNKNOWN_ADDR </b>      : Remove connection failed, invalid SCO index
 *              <b> WICED_BT_NO_RESOURCES </b>      : Remove connection failed, no resource
 *              <b> WICED_BT_SUCCESS </b>           : Remove connection successfully, device is still
 *                                                    listening for incoming connection
 *              <b> WICED_BT_PENDING </b>           : Remove connection successfully
 */
wiced_bt_dev_status_t wiced_bt_sco_remove (uint16_t sco_index);

/**
 *
 *The wiced_sco_lib.a was required to be included before we link this function.
 *Called to handle (e)SCO connection request event (wiced_bt_sco_connect_request_event).
 *
 *
 *  @param[in]  sco_index           : SCO index
 *
 *  @param[in]  hci_status          : WICED_BT_SCO_CONNECTION_ACCEPT              0x00
 *                                    WICED_BT_SCO_CONNECTION_REJECT_RESOURCES    0x0D
 *                                    WICED_BT_SCO_CONNECTION_REJECT_SECURITY     0x0E
 *                                    WICED_BT_SCO_CONNECTION_REJECT_DEVICE       0x0F
 *  @param[in]  p_params            : Pointer to the SCO parameter structure
 *
 */
void wiced_bt_sco_accept_connection (uint16_t sco_index, uint8_t hci_status,
                                     wiced_bt_sco_params_t *p_params);


/**
 *
 *The wiced_sco_lib.a was required to include before we link this function.
 *Configure the SCO routing path.
 *
 *  @param[in]  pData                 : To setup SCO routing path
 *
 * @return
 *              WICED_BT_SUCCESS    : Config success.
 *              WICED_BT_PENDING    : Command sent. Waiting for command complete event.
 *              WICED_BT_BUSY       : Command not sent. Waiting for command complete event for prior command.
 */
wiced_bt_dev_status_t wiced_bt_sco_setup_voice_path(wiced_bt_voice_path_setup_t *pData);


/**
 *
 *The wiced_sco_lib.a was required to include before we link this function.
 *This function write SCO data to a specified instance
 *
 *  @param[in]  sco_inx             : sco connection instance
 *  @param[in]  p_data              : Pointer to  data
 *  @param[in]  len                 : Length of data at p_data
 *
 * @return
 *              WICED_BT_SUCCESS            : data write is successful.
 *              WICED_BT_ERROR              : generic error
 *              WICED_BT_UNKNOWN_ADDR       : unknown SCO connection instance.
 */

wiced_bt_dev_status_t wiced_bt_sco_write_buffer (uint16_t sco_inx, uint8_t *p_data, uint8_t len);



/**
 *
 *The wiced_sco_lib.a was required to include before we link this function.
 *Send pcm data to internal audio pipe.
 *
 *
 *  @param[in]  sco_index     : SCO index to send the stream
 *  @param[in]  p_pcmsrc        : Audio stream source.
 *  @param[in]  len             : Length of stream.
 *
 *  @return                     : Return the legth of non transmited stream.
 */
uint16_t wiced_bt_sco_output_stream( uint16_t sco_index, uint8_t* p_pcmsrc,uint16_t len );

/**
 *
 *The wiced_voice_path.a was required to include before we link this function.
 *To turn off the PCM/I2S hardware clock
 *This function needs to be called after the application turns off (or mutes audio data in/out of) the codec
 *
 *  @return                     : None
 */

void  wiced_bt_sco_turn_off_pcm_clock( void );

/**
 * Read the esco parametes
 *
 * @param[in] index: index to SCO codec
 * @param[out] p_enh_esco_params:  application allocated buffer pointer to read the parameters
 *
 * @return : None
 */
void wiced_bt_read_esco_parameters(wiced_bt_sco_idx_t index, wiced_bt_enh_esco_params_t  *p_enh_esco_params);

/**
 * update the default esco parametes, This API needs to be called before initiating SCO connection
 *
 * @param[in] index: SCO Codec index to update the parameters
 * @param[in] p_enh_esco_params:  pointer to sco parameters
 *
 * @return : None
 */

void wiced_bt_config_esco_parameters(wiced_bt_sco_idx_t index, wiced_bt_enh_esco_params_t *p_enh_esco_params);

#ifdef __cplusplus
}
#endif

/** @} sco */
/* @endcond*/

#endif //__WICED_BT_SCO_H__
