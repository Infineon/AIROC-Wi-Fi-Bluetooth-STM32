/*
 * Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
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
 *  Defines the Enterprise Security Interface Error codes.
 *
 */

#ifndef INCLUDED_CY_ENTERPRISE_SECURITY_ERROR_H_
#define INCLUDED_CY_ENTERPRISE_SECURITY_ERROR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "cy_result.h"
#include "cy_result_mw.h"

/**
 * \defgroup enterprise_security_results Enterprise Security results/error codes
 * @ingroup enterprise_security_defines
 *
 * Enterprise Security library APIs return results of type cy_rslt_t and consist of three parts:
 * - module base
 * - type
 * - error code
 *
 * \par Result Format
 *
   \verbatim
                   Module base                   Type    Library-specific error code
      +----------------------------------------+------+------------------------------+
      |CY_RSLT_MODULE_ENTERPRISE_SECURITY_BASE | 0x2  |           Error Code         |
      +----------------------------------------+------+------------------------------+
                       14 bits                  2 bits            16 bits

   See the macro section of this document for library-specific error codes.
   \endverbatim
 *
 * The data structure cy_rslt_t is part of cy_result.h located in <core_lib/include>
 *
 * Module base: This base is derived from CY_RSLT_MODULE_MIDDLEWARE_BASE (defined in cy_result.h) and is an offset of the CY_RSLT_MODULE_MIDDLEWARE_BASE.
 *              The details of the offset and the middleware base are defined in cy_result_mw.h, which is part of the [GitHub connectivity-utilities] (https://github.com/infineon/connectivity-utilities) repo.
 *              For example, the Enterprise Security library uses CY_RSLT_MODULE_ENTERPRISE_SECURITY_BASE as the module base.
 *
 * Type: This type is defined in cy_result.h and can be one of CY_RSLT_TYPE_FATAL, CY_RSLT_TYPE_ERROR, CY_RSLT_TYPE_WARNING, or CY_RSLT_TYPE_INFO.
 *
 * Library-specific error code: These error codes are library-specific and defined in the macro section.
 *
 * Helper macros used for creating the library-specific result are provided as part of cy_result.h.
 * \{
 */

/** Enterprise Security base error code */
#define CY_RSLT_ENTERPRISE_SECURITY_ERR_BASE                         CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR, CY_RSLT_MODULE_ENTERPRISE_SECURITY_BASE, 0)

/** Enterprise Security error codes */
#define CY_RSLT_ENTERPRISE_SECURITY_ERROR                            ( CY_RSLT_ENTERPRISE_SECURITY_ERR_BASE + 1 )   /**< Generic error.                 */
#define CY_RSLT_ENTERPRISE_SECURITY_BADARG                           ( CY_RSLT_ENTERPRISE_SECURITY_ERR_BASE + 2 )   /**< Bad argument.                  */
#define CY_RSLT_ENTERPRISE_SECURITY_NOMEM                            ( CY_RSLT_ENTERPRISE_SECURITY_ERR_BASE + 3 )   /**< Out of resources.              */
#define CY_RSLT_ENTERPRISE_SECURITY_TIMEOUT                          ( CY_RSLT_ENTERPRISE_SECURITY_ERR_BASE + 4 )   /**< Out of resources.              */
#define CY_RSLT_ENTERPRISE_SECURITY_ALREADY_CONNECTED                ( CY_RSLT_ENTERPRISE_SECURITY_ERR_BASE + 5 )   /**< Already connected to AP.       */
#define CY_RSLT_ENTERPRISE_SECURITY_NOT_CONNECTED                    ( CY_RSLT_ENTERPRISE_SECURITY_ERR_BASE + 6 )   /**< Not connected to any AP.       */
#define CY_RSLT_ENTERPRISE_SECURITY_JOIN_ERROR                       ( CY_RSLT_ENTERPRISE_SECURITY_ERR_BASE + 7 )   /**< Error joining AP.              */
#define CY_RSLT_ENTERPRISE_SECURITY_LEAVE_ERROR                      ( CY_RSLT_ENTERPRISE_SECURITY_ERR_BASE + 8 )   /**< Error leaving AP.              */
#define CY_RSLT_ENTERPRISE_SECURITY_TLS_ERROR                        ( CY_RSLT_ENTERPRISE_SECURITY_ERR_BASE + 9 )   /**< TLS Error.                     */
#define CY_RSLT_ENTERPRISE_SECURITY_SUPPLICANT_ERROR                 ( CY_RSLT_ENTERPRISE_SECURITY_ERR_BASE + 10 )  /**< Supplicant error.              */
#define CY_RSLT_ENTERPRISE_SECURITY_SUPPLICANT_IN_PROGRESS           ( CY_RSLT_ENTERPRISE_SECURITY_ERR_BASE + 11 )  /**< Supplicant is in progress.     */
#define CY_RSLT_ENTERPRISE_SECURITY_SUPPLICANT_ABORTED               ( CY_RSLT_ENTERPRISE_SECURITY_ERR_BASE + 12 )  /**< Supplicant aborted.            */
#define CY_RSLT_ENTERPRISE_SECURITY_SUPPLICANT_NOT_STARTED           ( CY_RSLT_ENTERPRISE_SECURITY_ERR_BASE + 13 )  /**< Supplicant not yet started.    */
#define CY_RSLT_ENTERPRISE_SECURITY_SUPPLICANT_UNPROCESSED           ( CY_RSLT_ENTERPRISE_SECURITY_ERR_BASE + 14 )  /**< Unprocessed event.             */
#define CY_RSLT_ENTERPRISE_SECURITY_EAP_ERROR                        ( CY_RSLT_ENTERPRISE_SECURITY_ERR_BASE + 15 )  /**< EAP error.                     */
#define CY_RSLT_ENTERPRISE_SECURITY_UTIL_ERROR                       ( CY_RSLT_ENTERPRISE_SECURITY_ERR_BASE + 16 )  /**< Util error.                    */

/** \} enterprise_security_defines */

#ifdef __cplusplus
} /*extern "C" */
#endif

#endif /* ifndef INCLUDED_CY_ENTERPRISE_SECURITY_ERROR_H_ */
