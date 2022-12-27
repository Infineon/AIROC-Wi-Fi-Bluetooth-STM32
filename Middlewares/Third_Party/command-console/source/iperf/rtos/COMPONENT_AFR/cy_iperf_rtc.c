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

#include "cy_rtc.h"
#include "cy_iperf_rtc.h"
#include "cyhal_rtc.h"
#include "mktime.h"
#include <time.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

static cyhal_rtc_t cy_rtc;

void rtc_init(void)
{
    if (CY_RSLT_SUCCESS != cyhal_rtc_init(&cy_rtc)) {
        printf("Error in cyhal_rtc_init \n");
    }
}

void rtc_free(void)
{
    cyhal_rtc_free(&cy_rtc);
}

int rtc_isenabled(void)
{
    return cyhal_rtc_is_enabled(&cy_rtc) ? 1 : 0;
}

time_t rtc_read(void)
{
    struct tm rtc_time;
    if (CY_RSLT_SUCCESS != cyhal_rtc_read(&cy_rtc, &rtc_time)) {
    	printf("Error in cyhal_rtc_read \n");
    }
    time_t seconds;
    if (!_rtc_maketime(&rtc_time, &seconds, RTC_FULL_LEAP_YEAR_SUPPORT)) {
    	printf("Error in rtc_maketime \n");
    }
    return seconds;
}

void rtc_write(time_t t)
{
    struct tm rtc_time;
    if (!_rtc_localtime(t, &rtc_time, RTC_FULL_LEAP_YEAR_SUPPORT)) {
    	printf("Error in rtc_localtime \n");
    }
    if (CY_RSLT_SUCCESS != cyhal_rtc_write(&cy_rtc, &rtc_time)) {
    	printf("Error in cyhal_rtc_write \n");
    }
}

#ifdef __cplusplus
}
#endif
