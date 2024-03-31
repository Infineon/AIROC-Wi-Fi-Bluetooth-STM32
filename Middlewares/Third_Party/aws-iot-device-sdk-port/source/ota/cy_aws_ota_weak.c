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
 * @file cy_aws_ota_weak.c
 *
 */

#include <string.h>
#include <stdlib.h>
#include "cyabs_rtos.h"
#include "cy_ota_storage.h"
#include "cy_ota_os_timer.h"


#if defined(__ICCARM__)
#define CY_OTA_WEAK_FUNCTION        __WEAK
#define CY_OTA_ATTR_PACKED(struct)  __packed struct
#elif defined(__GNUC__) || defined(__clang__) || defined(__CC_ARM)
#define CY_OTA_WEAK_FUNCTION        __attribute__((weak))
#define CY_OTA_ATTR_PACKED(struct)  struct __attribute__((packed))
#else
#define CY_OTA_WEAK_FUNCTION        __attribute__((weak))
#define CY_OTA_ATTR_PACKED(struct)  struct __attribute__((packed))
#endif  /* defined(__ICCARM__) */

#define CY_OTA_UNUSED_ARG(arg)      (void)(arg)

/*
 * Retrieve the OTA signer certificate
 */
CY_OTA_WEAK_FUNCTION uint8_t * cy_awsport_get_ota_signer_certificate( cy_ota_type_t ota_type,
                                                                      const uint8_t * const pucCertName,
                                                                      uint32_t * const ota_signer_cert_size)
{
    uint8_t* pucCert = NULL;

    *ota_signer_cert_size = sizeof( signingcredentialSIGNING_CERTIFICATE_PEM );

    pucCert = ( uint8_t * )cy_awsport_ota_malloc( *ota_signer_cert_size );
    if( pucCert == NULL )
    {
        /* clear signer cert size */
        *ota_signer_cert_size = 0;
    }
    else
    {
       memcpy( pucCert, signingcredentialSIGNING_CERTIFICATE_PEM, *ota_signer_cert_size );
    }
    CY_OTA_UNUSED_ARG( pucCertName );
    return pucCert;
}

/*-----------------------------------------------------------*/
