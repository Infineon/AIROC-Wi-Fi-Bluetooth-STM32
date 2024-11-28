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
 *  Prototypes of functions for controlling enterprise security network
 */

#pragma once

#include "WhdSTAInterface.h"
#ifdef __cplusplus
extern "C" {
#endif
#include "cy_enterprise_security.h"

#ifdef __cplusplus
}
#endif


/**
 * \addtogroup enterprise_security_class
 * \{
 */

/**
 * EnterpriseSecurity class
 *
 * @brief
 * Defines Enterprise Security class with methods to join/leave an enterprise network.
 */
class EnterpriseSecurity : public WhdSTAInterface
{
public:
    /**
     * EnterpriseSecurity constructor
     *
     * @param[in]  ent_parameters : Pointer to \ref cy_enterprise_security_parameters_t structure,
     *                              initialized by the caller with the details required for establishing connection with the enterprise network.
     */
    EnterpriseSecurity(cy_enterprise_security_parameters_t *ent_parameters);

    /**
     * EnterpriseSecurity destructor
     */
    ~EnterpriseSecurity();

    /**
     * Joins an enterprise security network (802.1x Access point)
     *
     * @return cy_rslt_t  : CY_RSLT_SUCCESS - on success, an error code otherwise.
     *                      Error codes returned by this function are: \n
     *                     \ref CY_RSLT_ENTERPRISE_SECURITY_BADARG \n
     *                     \ref CY_RSLT_ENTERPRISE_SECURITY_ALREADY_CONNECTED \n
     *                     \ref CY_RSLT_ENTERPRISE_SECURITY_NOMEM \n
     *                     \ref CY_RSLT_ENTERPRISE_SECURITY_JOIN_ERROR \n
     *                     \ref CY_RSLT_ENTERPRISE_SECURITY_SUPPLICANT_ERROR
     */
    cy_rslt_t join( void );

    /**
     * Leaves an Enterprise security network (802.1x Access point)
     *
     * @return cy_rslt_t  : CY_RSLT_SUCCESS - on success, an error code otherwise.
     *                      Error codes returned by this function are: \n
     *                     \ref CY_RSLT_ENTERPRISE_SECURITY_BADARG \n
     *                     \ref CY_RSLT_ENTERPRISE_SECURITY_NOT_CONNECTED \n
     *                     \ref CY_RSLT_ENTERPRISE_SECURITY_LEAVE_ERROR \n
     *                     \ref CY_RSLT_ENTERPRISE_SECURITY_SUPPLICANT_ERROR
     */
    cy_rslt_t leave( void );

private:
    cy_enterprise_security_t handle; /**< Pointer to store the Enterprise Security instance handle for internal use. */
};

/** \} enterprise_security_class */
