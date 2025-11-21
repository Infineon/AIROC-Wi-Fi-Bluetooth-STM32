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
 * MPEG-D, USAC A2DP Application Programming Interface
 *
 */
#ifndef __WICED_BT_A2D_MDU_H__
#define __WICED_BT_A2D_MDU_H__

/**
 * @cond DUAL_MODE
 * @addtogroup  wicedbt_a2dp_mpeg_d_usac    MPEG-D,USAC Support
 * @ingroup     wicedbt_av_a2d_helper
 * This section describes A2DP MPEG-D,USAC Audio codec API
 * @{
 */

/*****************************************************************************
**  Constants
*****************************************************************************/

/** the LOSC of MPEG_D, USAC media codec capabilitiy */
#define A2D_MDU_INFO_LEN            9

/** for Codec Specific Information Element */

/**
 * @anchor A2D_MDU_OBJ
 * @name MPEG-D,USAC Object
 * @{
 */
#define A2D_MDU_IE_OBJ_MSK          0xC0        /**< b7-b6 object type */
#define A2D_MDU_IE_OBJ_USAC         0x80        /**< b7: MPEG-D USAC WITH MPEG-D DRC */
/** @} A2D_MDU_OBJ */

/**
 * @anchor A2D_MDU_SF
 * @name MPEG-D USAC Sampling Frequency
 * @{
 */

#define A2D_MDU_IE_SAMP_FREQ_MSK    0x3FFFFFF0  /**< sampling frequency Full */
#define A2D_MDU_IE_SAMP_FREQ0_MSK   0x3F000000  /**< sampling frequency Mask (octet 0) */
#define A2D_MDU_IE_SAMP_FREQ12_MSK  0x00FFFF00  /**< sampling frequency (octet 1/2) */
#define A2D_MDU_IE_SAMP_FREQ3_MSK   0x000000F0  /**< sampling frequency (octet 3) */

#define A2D_MDU_IE_SAMP_FREQ_7      0x20000000  /**< b5:octet0 7350 Hz */
#define A2D_MDU_IE_SAMP_FREQ_80     0x10000000  /**< b4:octet0 8000 Hz */
#define A2D_MDU_IE_SAMP_FREQ_8      0x08000000  /**< b3:octet0 8820 Hz */
#define A2D_MDU_IE_SAMP_FREQ_9      0x04000000  /**< b2:octet0 9600 Hz */
#define A2D_MDU_IE_SAMP_FREQ_11     0x02000000  /**< b1:octet0 11025Hz */
#define A2D_MDU_IE_SAMP_FREQ_117    0x01000000  /**< b0:octet0 11760Hz */
#define A2D_MDU_IE_SAMP_FREQ_12     0x00800000  /**< b7:octet1 12kHz */
#define A2D_MDU_IE_SAMP_FREQ_128    0x00400000  /**< b6:octet1  12.8kHz */
#define A2D_MDU_IE_SAMP_FREQ_14     0x00200000  /**< b5:octet1  14.7kHz */
#define A2D_MDU_IE_SAMP_FREQ_16     0x00100000  /**< b4:octet1  16kHz */
#define A2D_MDU_IE_SAMP_FREQ_17     0x00080000  /**< b3:octet1  17640Hz */
#define A2D_MDU_IE_SAMP_FREQ_19     0x00040000  /**< b2:octet1  19.2kHz */
#define A2D_MDU_IE_SAMP_FREQ_22     0x00020000  /**< b1:octet1  22.05kHz */
#define A2D_MDU_IE_SAMP_FREQ_24     0x00010000  /**< b0:octet1  24kHz */
#define A2D_MDU_IE_SAMP_FREQ_29     0x00008000  /**< b7:octet2  29.4kHz */
#define A2D_MDU_IE_SAMP_FREQ_32     0x00004000  /**< b6:octet2  32kHz */
#define A2D_MDU_IE_SAMP_FREQ_35     0x00002000  /**< b5:octet2  35.28kHz */
#define A2D_MDU_IE_SAMP_FREQ_38     0x00001000  /**< b4:octet2  38.4kHz */
#define A2D_MDU_IE_SAMP_FREQ_44     0x00000800  /**< b3:octet2  44.1kHz */
#define A2D_MDU_IE_SAMP_FREQ_48     0x00000400  /**< b2:octet2  48kHz */
#define A2D_MDU_IE_SAMP_FREQ_58     0x00000200  /**< b1:octet2  58.8kHz */
#define A2D_MDU_IE_SAMP_FREQ_64     0x00000100  /**< b0:octet2  64kHz */
#define A2D_MDU_IE_SAMP_FREQ_70     0x00000080  /**< b7:octet3  70.56kHz */
#define A2D_MDU_IE_SAMP_FREQ_76     0x00000040  /**< b6:octet3  76.8kHz */
#define A2D_MDU_IE_SAMP_FREQ_88     0x00000020  /**< b5:octet3  88.2kHz */
#define A2D_MDU_IE_SAMP_FREQ_96     0x00000010  /**< b4:octet3  96kHz */
/** @} A2D_MDU_SF */

/**
 * @anchor A2D_MDU_CH_MD
 * @name MPEG-D USAC Channel
 * @{
 */
#define A2D_MDU_IE_CHNL_MSK         0x0F        /**< b3-b0 channels (octet 3) */
#define A2D_MDU_IE_CHNL_1           0x08        /**< b3: 1 channel */
#define A2D_MDU_IE_CHNL_2           0x04        /**< b2: 2 channels */
/** @} A2D_MDU_CH_MD */

#define A2D_MDU_IE_VBR_MSK          0x80        /**< b7: VBR (octet 4)*/

/**
 * @anchor A2D_MDU_BITRATE
 * @name MPEG-D USAC Bitrate
 * @{
 */
#define A2D_MDU_IE_BITRATE4_MSK     0x7F0000    /**< octect 4*/
#define A2D_MDU_IE_BITRATE56_MSK    0x00FFFF    /**< octect5, 6*/
#define A2D_MDU_IE_BITRATE_MSK      0x7FFFFF    /**< b6-b0 of octect 4, all of octect5, 6*/
/** @} A2D_MDU_BITRATE */


/*****************************************************************************
**  Type Definitions
*****************************************************************************/

/** data type for the MPEG-2, 4 AAC Codec Information Element*/
typedef struct
{
    uint32_t        samp_freq;              /**< Sampling frequency \ref A2D_MDU_SF*/
    uint32_t        bitrate;                /**< Bit rate index \ref A2D_MDU_BITRATE*/
    uint8_t         obj_type;               /**< Object type \ref A2D_MDU_OBJ*/
    uint8_t         chnl;                   /**< Channel mode \ref A2D_MDU_CH_MD*/
    uint8_t         vbr;                    /**< Variable Bit Rate */
} wiced_bt_a2d_mdu_cie_t;

/*****************************************************************************
**  External Function Declarations
*****************************************************************************/
#ifdef __cplusplus
extern "C"
{
#endif
/**
 *
 * This function is called by an application to build
 * the MPEG-2, 4 AAC Media Codec Capabilities byte sequence
 * beginning from the LOSC octet.
 *
 * @param[in]  media_type : Indicates Audio, or Multimedia.
 * @param[in]  p_ie       : MPEG-2, 4 AAC Codec Information Element information.
 * @param[out] p_result   : the resulting codec info byte sequence.
 *
 * @return    <b> A2D_SUCCESS </b> if function execution succeeded. \n
 *            <b> Error status code </b>, otherwise.
 */
wiced_bt_a2d_status_t wiced_bt_a2d_bld_mdu_info(uint8_t media_type, wiced_bt_a2d_mdu_cie_t *p_ie,
                                               uint8_t *p_result);

/**
 *
 * This function is called by an application to parse
 * the MPEG-2, 4 AAC Media Codec Capabilities byte sequence
 * beginning from the LOSC octet.
 *
 * @param[in]  p_info   :  the byte sequence to parse.
 * @param[in]  for_caps :  TRUE, if the byte sequence is for get capabilities response.
 * @param[out] p_ie     :  MPEG-2, 4 AAC Codec Information Element information.
 *
 * @return     <b> A2D_SUCCESS </b> if function execution succeeded. \n
 *             <b> Error status code </b>, otherwise.
 */
wiced_bt_a2d_status_t wiced_bt_a2d_pars_mdu_info(wiced_bt_a2d_mdu_cie_t *p_ie,
                                                    uint8_t *p_info,
                                               wiced_bool_t for_caps);

/** @} wicedbt_a2dp_mpeg_2_4 */
/* @endcond*/

#ifdef __cplusplus
}
#endif

#endif //__WICED_BT_A2D_M24_H__
