/*
 * Copyright 2020, Cypress Semiconductor Corporation or a subsidiary of
 * Cypress Semiconductor Corporation. All Rights Reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software"), is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
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

#ifndef LIBS_WCM_INCLUDE_CY_WCM_ERROR_H_
#define LIBS_WCM_INCLUDE_CY_WCM_ERROR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "cy_result.h"
#include "cy_result_mw.h"

/**
 * @defgroup generic_wcm_defines WCM results/error codes
 * @ingroup group_wcm_macros
 * Cypress middleware APIs return results of type cy_rslt_t and consist of three parts:
 * - module base
 * - type
 * - error code
 *
 * \par Result Format
 *
   \verbatim
              Module base                   Type    Library specific error code
      +-----------------------------------+------+------------------------------+
      |CY_RSLT_MODULE_WCM_BASE            | 0x2  |           Error Code         |
      +-----------------------------------+------+------------------------------+
                14-bits                    2-bits            16-bits

   See the macro section of this document for library-specific error codes.
   \endverbatim
 *
 * The data structure cy_rslt_t is part of cy_result.h located in <core_lib/include>.
 *
 * Module base: This base is derived from CY_RSLT_MODULE_MIDDLEWARE_BASE (defined in cy_result.h) and is an offset of the CY_RSLT_MODULE_MIDDLEWARE_BASE.
 *              The details of the offset and the middleware base are defined in cy_result_mw.h, that is part of [GitHub connectivity-utilities] (https://github.com/cypresssemiconductorco/connectivity-utilities)
 *              For example, Wi-Fi Connection Manager (WCM) uses CY_RSLT_MODULE_WCM_BASE as the module base.
 *
 * Type: This type is defined in cy_result.h and can be one of CY_RSLT_TYPE_FATAL, CY_RSLT_TYPE_ERROR, CY_RSLT_TYPE_WARNING, or CY_RSLT_TYPE_INFO. AWS library error codes are of type CY_RSLT_TYPE_ERROR.
 *
 * Library-specific error code: These error codes are library-specific and defined in the macro section.
 *
 * Helper macros used for creating the library-specific result are provided as part of cy_result.h.
 * \{
 */
/** Generic wcm base error code */
#define CY_RSLT_WCM_ERR_BASE                   CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR, CY_RSLT_MODULE_WCM_BASE, 0)

/** WCM Error Codes */
#define CY_RSLT_WCM_WAIT_TIMEOUT                           (CY_RSLT_WCM_ERR_BASE + 1)  /**< Wait timeout.                                 */
#define CY_RSLT_WCM_BAD_NETWORK_PARAM                      (CY_RSLT_WCM_ERR_BASE + 2)  /**< Bad network parameters.                       */
#define CY_RSLT_WCM_BAD_SSID_LEN                           (CY_RSLT_WCM_ERR_BASE + 3)  /**< Bad SSID length.                              */
#define CY_RSLT_WCM_SECURITY_NOT_SUPPORTED                 (CY_RSLT_WCM_ERR_BASE + 4)  /**< Security not supported.                       */
#define CY_RSLT_WCM_BAD_PASSPHRASE_LEN                     (CY_RSLT_WCM_ERR_BASE + 5)  /**< Bad passphrase length.                        */
#define CY_RSLT_WCM_BAD_ARG                                (CY_RSLT_WCM_ERR_BASE + 6)  /**< Bad argument.                                 */
#define CY_RSLT_WCM_INTERFACE_NOT_SUPPORTED                (CY_RSLT_WCM_ERR_BASE + 7)  /**< Interface type not supported.                 */
#define CY_RSLT_WCM_MUTEX_ERROR                            (CY_RSLT_WCM_ERR_BASE + 8)  /**< Mutex error.                                  */
#define CY_RSLT_WCM_STA_DISCONNECT_ERROR                   (CY_RSLT_WCM_ERR_BASE + 9)  /**< STA disconnect error.                         */
#define CY_RSLT_WCM_STA_NETWORK_DOWN                       (CY_RSLT_WCM_ERR_BASE + 10) /**< STA network is down.                          */
#define CY_RSLT_WCM_BSP_INIT_ERROR                         (CY_RSLT_WCM_ERR_BASE + 11) /**< CY BSP initialization error.                  */
#define CY_RSLT_WCM_BSP_DEINIT_ERROR                       (CY_RSLT_WCM_ERR_BASE + 12) /**< CY BSP error while de-initialization.         */
#define CY_RSLT_WCM_NO_ACTIVE_SCAN                         (CY_RSLT_WCM_ERR_BASE + 13) /**< No active scan running currently.             */
#define CY_RSLT_WCM_SCAN_IN_PROGRESS                       (CY_RSLT_WCM_ERR_BASE + 14) /**< Scan in progress.                             */
#define CY_RSLT_WCM_SCAN_ERROR                             (CY_RSLT_WCM_ERR_BASE + 15) /**< Scan error.                                   */
#define CY_RSLT_WCM_STOP_SCAN_ERROR                        (CY_RSLT_WCM_ERR_BASE + 16) /**< Stop scan error.                              */
#define CY_RSLT_WCM_BAND_NOT_SUPPORTED                     (CY_RSLT_WCM_ERR_BASE + 17) /**< BAND not supported.                           */
#define CY_RSLT_WCM_OUT_OF_MEMORY                          (CY_RSLT_WCM_ERR_BASE + 18) /**< WCM out of memory error.                      */
#define CY_RSLT_WCM_CHANNEL_ERROR                          (CY_RSLT_WCM_ERR_BASE + 19) /**< Error in retrieving the Wi-Fi channel.        */
#define CY_RSLT_WCM_NETIF_DOES_NOT_EXIST                   (CY_RSLT_WCM_ERR_BASE + 20) /**< Network interface does not exist.             */
#define CY_RSLT_WCM_ARP_REQUEST_FAILURE                    (CY_RSLT_WCM_ERR_BASE + 21) /**< Error returned for ARP request failure.       */
#define CY_RSLT_WCM_IPV6_GLOBAL_ADDRESS_NOT_SUPPORTED      (CY_RSLT_WCM_ERR_BASE + 22) /**< IPv6 global IP not supported.                 */
#define CY_RSLT_WCM_IPV6_INTERFACE_NOT_READY               (CY_RSLT_WCM_ERR_BASE + 23) /**< IPV6 interface not ready.                     */
#define CY_RSLT_WCM_PING_FAILURE                           (CY_RSLT_WCM_ERR_BASE + 24) /**< Failure in executing ping.                    */
#define CY_RSLT_WCM_PING_REQUEST_TIMEOUT                   (CY_RSLT_WCM_ERR_BASE + 25) /**< Ping request timed out.                       */
#define CY_RSLT_WCM_STATIC_IP_NOT_SUPPORTED                (CY_RSLT_WCM_ERR_BASE + 26) /**< Static IP address not supported for IPv6.     */
#define CY_RSLT_WCM_BAD_STATIC_IP                          (CY_RSLT_WCM_ERR_BASE + 27) /**< Bad Static IP address.                        */
#define CY_RSLT_WCM_SECONDARY_INTERFACE_ERROR              (CY_RSLT_WCM_ERR_BASE + 28) /**< Error in bringing up the secondary interface. */
#define CY_RSLT_WCM_AP_NETWORK_BRINGUP_ERROR               (CY_RSLT_WCM_ERR_BASE + 29) /**< AP network bring up error.                    */
#define CY_RSLT_WCM_AP_BAD_CHANNEL                         (CY_RSLT_WCM_ERR_BASE + 30) /**< Bad AP channel number.                        */
#define CY_RSLT_WCM_AP_IE_REMOVAL_ERROR                    (CY_RSLT_WCM_ERR_BASE + 31) /**< AP IE removal error.                          */
#define CY_RSLT_WCM_INVALID_IE                             (CY_RSLT_WCM_ERR_BASE + 32) /**< Invalid IE.                                   */
#define CY_RSLT_WCM_AP_NOT_UP                              (CY_RSLT_WCM_ERR_BASE + 33) /**< SoftAP is not started.                        */
#define CY_RSLT_WCM_INTERFACE_NOT_UP                       (CY_RSLT_WCM_ERR_BASE + 34) /**< Interface is not initialized.                 */
#define CY_RSLT_WCM_NOT_INITIALIZED                        (CY_RSLT_WCM_ERR_BASE + 35) /**< WCM not initialized.                          */

/** WPS Error Codes */
#define CY_RSLT_WCM_WPS_PBC_OVERLAP                        (CY_RSLT_WCM_ERR_BASE + 36) /**< WPS PBC overlap.                              */
#define CY_RSLT_WCM_WPS_ERROR_RECEIVED_WEP_CREDENTIALS     (CY_RSLT_WCM_ERR_BASE + 37) /**< WPS received incorrect credentials.           */
#define CY_RSLT_WCM_WPS_FAILED                             (CY_RSLT_WCM_ERR_BASE + 38) /**< WPS PBC/PIN mode failed.                      */

/** DHCP Error Code */
#define CY_RSLT_WCM_DHCP_TIMEOUT                           (CY_RSLT_WCM_ERR_BASE + 39)  /**< DHCP timeout.                                */

/** \} error codes */

#ifdef __cplusplus
} /* extern C */
#endif

#endif /* LIBS_WCM_INCLUDE_CY_WCM_ERROR_H_ */
