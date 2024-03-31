/*
 * Copyright 2023, Cypress Semiconductor Corporation (an Infineon company) or
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

/**
 *  @file cy_aws_iot_port_error.h
 *  Defines the AWS IoT device SDK port library interface error codes.
 */

#ifndef CY_AWS_IOT_PORT_ERROR_H_
#define CY_AWS_IOT_PORT_ERROR_H_

#include "cy_result.h"
#include "cy_result_mw.h"

/**
 * \defgroup group_aws_iot_sdk_port_results AWS IoT SDK port results/error codes
 * @ingroup group_aws_iot_sdk_port_macros
 *
 * AWS IoT SDK port library APIs return results of type cy_rslt_t and consist of three parts:
 * - module base
 * - type
 * - error code
 *
 * \par Result Format
 *
   \verbatim
              Module base                   Type    Library-specific error code
      +-----------------------------------+------+------------------------------+
      |  CY_RSLT_AWS_IOT_PORT_ERROR_BASE  | 0x2  |           Error Code         |
      +-----------------------------------+------+------------------------------+
                14 bits                    2 bits            16 bits

   See the macro section of this document for library-specific error codes.
   \endverbatim
 *
 * The data structure cy_rslt_t is part of cy_result.h located in <core_lib/include>
 *
 * Module base: This base is derived from CY_RSLT_MODULE_MIDDLEWARE_BASE (defined in cy_result.h) and is an offset of the CY_RSLT_MODULE_MIDDLEWARE_BASE.
 *              The details of the offset and the middleware base are defined in cy_result_mw.h, which is part of the [GitHub connectivity-utilities] (https://github.com/Infineon/connectivity-utilities) repo.
 *              For example, the AWS IoT SDK port library uses CY_RSLT_AWS_IOT_PORT_ERROR_BASE as the module base.
 *
 * Type: This type is defined in cy_result.h and can be one of CY_RSLT_TYPE_FATAL, CY_RSLT_TYPE_ERROR, CY_RSLT_TYPE_WARNING, or CY_RSLT_TYPE_INFO. AWS library error codes are of type CY_RSLT_TYPE_ERROR.
 *
 * Library-specific error code: These error codes are library-specific and defined in the macro section.
 *
 * Helper macros used for creating the library-specific result are provided as part of cy_result.h.
 * \{
 */

/** AWS IoT port library error code base. */
#define CY_RSLT_AWS_IOT_PORT_ERROR_BASE    CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR, CY_RSLT_MODULE_MIDDLEWARE_AWS, 0) /**< AWS library error code base. */

/* General error codes */
#define CY_RSLT_AWS_IOT_PORT_ERROR_UNSUPPORTED           (CY_RSLT_AWS_IOT_PORT_ERROR_BASE +  1) /**< Unsupported feature.                  */
#define CY_RSLT_AWS_IOT_PORT_ERROR_GENERAL               (CY_RSLT_AWS_IOT_PORT_ERROR_BASE +  2) /**< Generic error.                        */
#define CY_RSLT_AWS_IOT_PORT_ERROR_BADARG                (CY_RSLT_AWS_IOT_PORT_ERROR_BASE +  3) /**< Bad argument.                         */
#define CY_RSLT_AWS_IOT_PORT_ERROR_OUT_OF_MEMORY         (CY_RSLT_AWS_IOT_PORT_ERROR_BASE +  4) /**< Out of Memory error.                  */

/* Storage related error codes */
#define CY_RSLT_AWS_IOT_PORT_ERROR_INIT_STORAGE          (CY_RSLT_AWS_IOT_PORT_ERROR_BASE +  5) /**< MQTT init failed.                     */
#define CY_RSLT_AWS_IOT_PORT_ERROR_OPEN_STORAGE          (CY_RSLT_AWS_IOT_PORT_ERROR_BASE +  6) /**< Could not open local storage.         */
#define CY_RSLT_AWS_IOT_PORT_ERROR_READ_STORAGE          (CY_RSLT_AWS_IOT_PORT_ERROR_BASE +  7) /**< Could not Read from local storage.    */
#define CY_RSLT_AWS_IOT_PORT_ERROR_WRITE_STORAGE         (CY_RSLT_AWS_IOT_PORT_ERROR_BASE +  8) /**< Could not Write to local storage.     */
#define CY_RSLT_AWS_IOT_PORT_ERROR_CLOSE_STORAGE         (CY_RSLT_AWS_IOT_PORT_ERROR_BASE +  9) /**< Close local storage error.            */

/* Crypto related error codes */
#define CY_RSLT_AWS_IOT_PORT_ERROR_CRYPTO_VERIFY_SIGN_FAIL     (CY_RSLT_AWS_IOT_PORT_ERROR_BASE +  10) /**< Sign verify failed.            */


#endif /* ifndef CY_AWS_IOT_PORT_ERROR_H_ */
