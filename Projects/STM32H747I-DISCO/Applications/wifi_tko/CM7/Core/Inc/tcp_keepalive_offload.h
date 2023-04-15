/******************************************************************************
 * File Name:   tcp_keepalive_offload.h
 *
 * Description: This file is the public interface of tcp_keepalive_offload.c
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

#ifndef TCP_KEEPALIVE_OFFLOAD_H
#define TCP_KEEPALIVE_OFFLOAD_H

/* FreeRTOS header files */
#include <FreeRTOS.h>
#include <task.h>
#include "cy_utils.h"
#include <cy_lpa_wifi_ol.h>


/*******************************************************************************
 * Macros
 ********************************************************************************/
#define MAX_WIFI_RETRY_COUNT                     (3)
#define NULL_IP_ADDRESS                          "0.0.0.0"

#define CHECK_RESULT(x)                          \
    do { if (CY_RSLT_SUCCESS != x) { CY_ASSERT(0); } } while(0);
#define APP_INFO(x)                              do { printf("Info: "); printf x; } while(0);
#define ERR_INFO(x)                              do { printf("Error: "); printf x; } while(0);

#define PRINT_AND_ASSERT(result, msg, args ...)   do                                \
                                                 {                                  \
                                                     if (CY_RSLT_SUCCESS != result) \
                                                     {                              \
                                                         ERR_INFO((msg, ## args));  \
                                                         CY_ASSERT(0);              \
                                                     }                              \
                                                 } while(0);

#define TCP_SOCKET_ERROR_DELAY_MS                (2000)

/*******************************************************************************
 * Function Prototypes
 ********************************************************************************/
cy_rslt_t wifi_connect(void);
void network_idle_task(void* arg);
cy_rslt_t tcp_socket_connection_start(void);
const ol_desc_t* find_my_tko_descriptor(const char* name);

#endif /* TCP_KEEPALIVE_OFFLOAD_H */


/* [] END OF FILE */
