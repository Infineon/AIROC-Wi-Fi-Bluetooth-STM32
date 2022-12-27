/*
 * Copyright 2022, Cypress Semiconductor Corporation (an Infineon company) or
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

/** WiFi-Cert interface header file */
#include "cy_result.h"

/**
 * \defgroup group_wpa3_ext_supplicant_wificert_interface_function Functions
 */

/**
 * \addtogroup group_wpa3_ext_supplicant_wificert_interface_function
 * \{
 * * The WPA3 External Supplicant H2E commit derive PT for SSID and Passphrase.
 * * The below API is a blocking call
 */


/** This function initializes the WPA3 External Supplicant middleware library by initializing following
 *   * i.   Initializes the WPA3 External Supplicant
 *   * ii.  Initializes the WAP3 Crypto Context information
 *   * iii. Loads the Mbed TLS ECP Group
 *
 * @return cy_rslt_t : CY_RSLT_SUCCESS
 *                   : WPA3_EXT_SUPP_ERROR
  *******************************************************************************/
cy_rslt_t wpa3_supplicant_h2e_pfn_list_derive_pt (uint8_t *ssid, uint8_t ssid_len, uint8_t *passphrase, uint8_t passphrase_len, uint8_t *output, uint8_t outlen );

/** \} group_wpa3_ext_supplicant_wificert_interface_function */
