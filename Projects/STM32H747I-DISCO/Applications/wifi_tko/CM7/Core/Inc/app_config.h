/******************************************************************************
 * File Name: app_config.h
 *
 * Description: This file includes the user configurable macros required by the
 * example to connect to an AP and configure the parameters for suspending the
 * network stack.
 *
 * Related Document: See README.md
 *
 ********************************************************************************
 * Copyright 2020-2021, Cypress Semiconductor Corporation (an Infineon company) or
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
 *******************************************************************************/


/*******************************************************************************
* Include guard
*******************************************************************************/
#ifndef APP_CONFIG_H_
#define APP_CONFIG_H_

#include "cy_wcm.h"
#include "cybsp.h"

/* Wi-Fi Credentials: Modify WIFI_SSID and WIFI_PASSWORD to match your Wi-Fi network
 * Credentials.
 */
#if  !defined(WIFI_SSID)
   #define WIFI_SSID                       "WIFI_SSID"
#endif /* (WIFI_SSID) */
#if  !defined(WIFI_PASSWORD)
   #define WIFI_PASSWORD                   "WIFI_PASSWORD"
#endif /* (WIFI_PASSWORD) */

/* Security type of the Wi-Fi access point. See 'cy_wcm_security_t' structure
 * in "cy_wcm.h" for more details.
 */
#define WIFI_SECURITY_TYPE                CY_WCM_SECURITY_WPA2_AES_PSK

/* This macro specifies the interval in milliseconds that the device monitors
   the network for inactivity. If the network is inactive for duration lesser
   than INACTIVE_WINDOW_MS in this interval, the MCU does not suspend the network
   stack and informs the calling function that the MCU wait period timed out
   while waiting for network to become inactive.
 */
#define NETWORK_INACTIVE_INTERVAL_MS      (300)

/* This macro specifies the continuous duration in milliseconds for which the
   network has to be inactive. If the network is inactive for this duaration,
   the MCU will suspend the network stack. Now, the MCU will not need to service
   the network timers which allows it to stay longer in sleep/deepsleep.
 */
#define NETWORK_INACTIVE_WINDOW_MS        (200)

/*
 * Delay between subsequent calls to suspending the network stack.
 * This is a safe delay which helps in preventing the race conditions
 * that might occur when activating and de-activating the offload.
 */
#define NETWORK_SUSPEND_DELAY_MS          (100)

/*
 * Enable(1) or Disable(0) the Host TCP Keepalive via ENABLE_HOST_TCP_KEEPALIVE.
 * It is disabled by default.
 */
#define ENABLE_HOST_TCP_KEEPALIVE         (0)

#endif /* APP_CONFIG_H_ */


/* [] END OF FILE */
