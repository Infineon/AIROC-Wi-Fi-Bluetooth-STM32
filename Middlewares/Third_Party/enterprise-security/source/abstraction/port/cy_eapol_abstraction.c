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
 *  Implements functions for registering and unregistering for eapol packets with
 *  the underlying WHD layer.
 */
#include "cy_enterprise_security_log.h"
#include "cy_enterprise_security_error.h"
#include "cy_supplicant_host.h"
#include "cy_wifimwcore_eapol.h"

cy_rslt_t cy_ent_sec_register_eapol_packet_handler(cy_ent_sec_eapol_packet_handler_t eapol_packet_handler)
{
    if(cy_wifimwcore_eapol_register_receive_handler((cy_wifimwcore_eapol_packet_handler_t)eapol_packet_handler) != CY_RSLT_SUCCESS)
    {
        cy_enterprise_security_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Failed to register eapol receive handler.\n");
        return CY_RSLT_ENTERPRISE_SECURITY_ERROR;
    }

    cy_enterprise_security_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Successfully registered eapol packet handler.\r\n");
    return CY_RSLT_SUCCESS;
}

void cy_ent_sec_unregister_eapol_packet_handler(void)
{
    cy_wifimwcore_eapol_register_receive_handler(NULL);
    cy_enterprise_security_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Successfully unregistered eapol packet handler.\r\n");
}
