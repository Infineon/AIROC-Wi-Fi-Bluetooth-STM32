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
 * @file cy_aws_ota_storage.c
 *  Implements storage interface APIs for AWS OTA.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cy_ota_storage.h"
#include "cy_ota_crypto.h"
#include "cy_aws_iot_sdk_port_log.h"

/* RTOS includes */
#include "cyabs_rtos.h"

/* Utility API's needs */

#include "cy_ota_utils.h"
#include "ota_appversion32.h"

#ifdef CY_OTA_FLASH_SUPPORT
/* Added for Flash Read */
#include "untar.h"
#include "flash_map_backend/flash_map_backend.h"
#include "sysflash/sysflash.h"
#include "bootutil/bootutil.h"
#include "bootutil_priv.h"
#include "cy_smif_psoc6.h"
#endif

/* Added the below defines to resolve ARMC6 linker errors. Check this issue with Amazon team. */
#if defined(__ARMCC_VERSION)
#ifndef CY_OTA_FLASH_SUPPORT
// Version of the app
#ifndef APP_VERSION_MAJOR
#define APP_VERSION_MAJOR    1
#endif

#ifndef APP_VERSION_MINOR
#define APP_VERSION_MINOR    0
#endif

#ifndef APP_VERSION_BUILD
#define APP_VERSION_BUILD    0
#endif

/* Firmware version. */
const AppVersion32_t appFirmwareVersion =
{
    .u.x.major = APP_VERSION_MAJOR,
    .u.x.minor = APP_VERSION_MINOR,
    .u.x.build = APP_VERSION_BUILD,
};
#endif //CY_OTA_FLASH_SUPPORT

void static_assert(int p, void *a)
{
     return;
}

#endif // __ARMCC_VERSION

/* File type in the tarball */
#define CY_FILE_TYPE_SPE                    "SPE"     /* Secure Programming Environment (TFM) code type               */
#define CY_FILE_TYPE_NSPE                   "NSPE"    /* Non-Secure Programming Environment (application) code type   */

typedef struct cy_awsport_ota_flash_ctx
{
    const OtaFileContext_t *ota_fs_ctx;                /**< OTA PAL layer handle. Used to keep track of system context between calls from the OTA Agent. */
    cy_ota_type_t           ota_type;                  /**< OTA type. Value indicates its Host OTA or Device OTA. */
    uint32_t                ota_flash_start_address;   /**< OTA flash region starting address. */
    uint32_t                ota_flash_read_offset;     /**< Current read offset for OTA/HOTA image. */
    cy_mutex_t              ota_sync_mutex;            /**< Mutex for synchronizing OTA calls. */
} cy_awsport_ota_flash_ctx_t;

cy_awsport_ota_flash_ctx_t cy_aws_ota_ctx;

FILE *open_memstream(char **bufptr, size_t *lenptr);

/**
 * Specify the OTA signature algorithm we support on this platform.
 */
const char OTA_JsonFileSignatureKey[ OTA_FILE_SIG_KEY_STR_MAX_LENGTH ] = "sig-sha256-ecdsa";

#ifdef CY_OTA_FLASH_SUPPORT

/**
 *  Current OTA Image State
 */
static OtaImageState_t current_ota_image_state = OtaImageStateUnknown;

/**
 * This FLASH write block used, if a block is < Block size to satisfy requirements of flash_area_write().
 */
static uint8_t block_buffer[ CY_FLASH_SIZEOF_ROW ];

/**
 * Flag to denote this is a tar file. This flag is used on subsequent chunks of file to know how to handle the data.
 */
int ota_is_tar_archive = 0;

/**
 * To keep track of the last signature check value for cy_awsport_ota_pal_set_platform_imageState
 */
static OtaPalStatus_t ota_last_sig_check;

/**
 * Signature Verification Context
 */
void *ota_sig_verify_context = NULL;

/**
 * Signature Certificate size
 */
uint32_t ota_signer_cert_size = 0;

/**
 * Pointer to Signer Certificate
 */
uint8_t  *ota_signer_cert = NULL;

/**
 * Context structure for parsing the tar file
 */
cy_untar_context_t  cy_ota_untar_context;
#endif

/*-----------------------------------------------------------*/
FILE *open_memstream(char **bufptr, size_t *lenptr)
{
    return ((FILE *)NULL);
}

/*-----------------------------------------------------------*/
#ifdef CY_OTA_FLASH_SUPPORT
/**
 * Write different sized chunks in 512 byte blocks
 */
static int ota_write_data_to_flash( const struct flash_area *fap, uint32_t offset, uint8_t * const source, uint32_t size )
{
    uint32_t bytes_to_write;
    uint32_t curr_offset;
    uint8_t *curr_src;

    bytes_to_write = size;
    curr_offset = offset;
    curr_src = source;

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s() start ....\n\r", __func__ );
    while( bytes_to_write > 0 )
    {
        uint32_t chunk_size = bytes_to_write;
        if( chunk_size > CY_FLASH_SIZEOF_ROW )
        {
            chunk_size = CY_FLASH_SIZEOF_ROW;
        }

        /* This should only happen on last part of last 4k chunk */
        if( chunk_size % CY_FLASH_SIZEOF_ROW )
        {
            /* we will read a 512 byte block, write out data into the block, then write the whole block */
            if( flash_area_read( fap, curr_offset, block_buffer, sizeof(block_buffer) ) != 0 )
            {
                cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "flash_area_read() failed.\n\r" );
                return -1;
            }
            memcpy( block_buffer, curr_src, chunk_size );

            if( flash_area_write( fap, curr_offset, block_buffer, sizeof(block_buffer) ) != 0 )
            {
                cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "flash_area_write() failed.\n\r" );
                return -1;
            }
        }
        else
        {
            if( flash_area_write( fap, curr_offset, curr_src, chunk_size ) != 0 )
            {
                cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "flash_area_write() failed.\n\r" );
                return -1;
            }
        }

        curr_offset += chunk_size;
        curr_src += chunk_size;
        bytes_to_write -= chunk_size;
    }

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s() end ....\n\r", __func__ );
    return 0;
}

/*-----------------------------------------------------------*/
/**
 * Set pending for the files contained in the TAR archive we just validated.
 */
static int ota_untar_set_pending( void )
{
    int i = 0;
    int image = 0;

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s() start ....\n\r", __func__ );
    for( i = 0; i < cy_ota_untar_context.num_files_in_json; i++ )
    {
        if( strncmp( cy_ota_untar_context.files[i].type, CY_FILE_TYPE_SPE, strlen(CY_FILE_TYPE_SPE) ) == 0 )
        {
            image = 1;  /* The TFM code, cm0 */
            cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "FILE TYPE %d: %s \n", i, cy_ota_untar_context.files[i].type );
        }
        else if( strncmp(cy_ota_untar_context.files[i].type, CY_FILE_TYPE_NSPE, strlen(CY_FILE_TYPE_NSPE) ) == 0 )
        {
            image = 0;  /* The application code, cm4 */
            cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "FILE TYPE %d: %s \n", i, cy_ota_untar_context.files[i].type );
        }
        else
        {
            /* unknown file type */
            cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "BAD FILE TYPE %d: %s \n", i, cy_ota_untar_context.files[i].type );
            continue;
        }
        boot_set_pending( image, 0 );
    }

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s() end ....\n\r", __func__ );
    return CY_UNTAR_SUCCESS;
}

/*-----------------------------------------------------------*/
cy_untar_result_t ota_untar_write_callback( cy_untar_context_ptr ctxt, uint16_t file_index, uint8_t *buffer,
                                            uint32_t file_offset, uint32_t chunk_size, void *cb_arg )
{
    int image = 0;
    const struct flash_area *fap;
    OtaFileContext_t * const C = (OtaFileContext_t *)cb_arg;

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s() start ....\n\r", __func__ );
    if( (ctxt == NULL) || (buffer == NULL) || (C == NULL) )
    {
        return CY_UNTAR_ERROR;
    }

    if( strncmp( ctxt->files[file_index].type, CY_FILE_TYPE_SPE, strlen(CY_FILE_TYPE_SPE) ) == 0 )
    {
        image = 1;  /* The TFM code, cm0 */
    }
    else if( strncmp( ctxt->files[file_index].type, CY_FILE_TYPE_NSPE, strlen(CY_FILE_TYPE_NSPE) ) == 0 )
    {
        image = 0;  /* The application code, cm4 */
    }
    else
    {
        /* unknown file type */
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "BAD FILE TYPE : %s. \n\r", ctxt->files[file_index].type );
        return CY_UNTAR_ERROR;
    }

    if( flash_area_open( FLASH_AREA_IMAGE_SECONDARY( image ), &fap ) != 0 )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "flash_area_open(%d) failed.\n\r", image );
        return CY_UNTAR_ERROR;
    }

    if( ota_write_data_to_flash( fap, file_offset, buffer, chunk_size ) != 0 )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "ota_write_data_to_flash() failed.\n\r" );
        flash_area_close( fap );
        return CY_UNTAR_ERROR;
    }

    flash_area_close( fap );
    return CY_UNTAR_SUCCESS;
}

/*-----------------------------------------------------------*/
/**
 * Initialization routine for handling tarball OTA file
 */
cy_rslt_t ota_untar_init_context( cy_untar_context_t *ctxt, OtaFileContext_t * const C )
{
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s() start ....\n\r", __func__ );
    if( cy_untar_init( ctxt, ota_untar_write_callback, (void *)C ) == CY_RSLT_SUCCESS )
    {
        ota_is_tar_archive  = 1;
        return CY_RSLT_SUCCESS;
    }
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s() end ....\n\r", __func__ );
    return CY_RSLT_TYPE_ERROR;
}

/*-----------------------------------------------------------*/
/**
 * Read the specified signer certificate from the filesystem into a local buffer.
 * The allocated memory returned becomes the property of the caller who is responsible for freeing it.
 *
 * This function is called from ota_pal_file_sign_check_init(). It should be implemented if signature
 * verification is not offloaded to non-volatile memory io function.
 *
 * Returns a pointer to the signer certificate in the file system.
 * NULL if the certificate cannot be read.
 * This returned pointer is the responsibility of the caller; if the memory is allocated the caller must free it.
 */
static uint8_t * ota_pal_read_assume_certificate( const uint8_t * const pucCertName, uint32_t * const ota_signer_cert_size )
{
    return ( cy_awsport_get_ota_signer_certificate( cy_aws_ota_ctx.ota_type, pucCertName, ota_signer_cert_size ) );
}

/*-----------------------------------------------------------*/
/*
 * Checks whether signature check can be skipped for the current OTA in progress
 *
 * Returns true if signature check to be skipped, false otherwise
 */
static bool ota_pal_skip_sign_check( OtaFileContext_t * const C )
{
    bool skip_sign_check = false;

    if( C != NULL )
    {
        switch( C->fileType )
        {
            case CY_AWS_IOT_OTA_TYPE_HOTA_CERT:
            case CY_AWS_IOT_OTA_TYPE_HOST_OTA:
                /* Skip validation if the signer certificate is NULL */
                if( ota_signer_cert == NULL )
                {
                   skip_sign_check = true;
                }
                break;

            default:
                break;
        }
    }
    return ( skip_sign_check );
}


/*-----------------------------------------------------------*/
/**
 * De Initialize the signature checking data
 */
static void ota_pal_file_sign_check_deinit( void )
{
    ota_sig_verify_context = NULL;
    ota_signer_cert_size = 0;

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s() start ....\n\r", __func__ );
    if( ota_signer_cert != NULL )
    {
        vPortFree( ota_signer_cert );
        ota_signer_cert = NULL;
    }
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s() end ....\n\r", __func__ );
}

/*-----------------------------------------------------------*/
/**
 * Initialize the signature checking for tarball OTA.
 * We keep track of the interim values for subsequent WriteBlock calls.
 * This function is used for a tarball OTA file.
 */
static OtaPalStatus_t ota_pal_file_sign_check_init( OtaFileContext_t * const C )
{
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s() start ....\n\r", __func__ );
    if( C == NULL )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Parameter check failed: Input is NULL.\n\r" );
        return OtaPalNullFileContext;
    }

    /* Reset globals for incremental sign check and handle re-entrant calls */
    ota_pal_file_sign_check_deinit();

    /* Verify an ECDSA-SHA256 signature. */
    if( cy_crypto_sign_verification_start( &ota_sig_verify_context, CY_CRYPTO_ASYMMETRIC_ALGORITHM_ECDSA,
                                           CY_CRYPTO_HASH_ALGORITHM_SHA256 ) != CY_RSLT_SUCCESS )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "cy_crypto_sign_verification_start() failed.\n\r" );
        return OtaPalSignatureCheckFailed;
    }

    ota_signer_cert = ota_pal_read_assume_certificate( ( const uint8_t * const ) C->pCertFilepath, &ota_signer_cert_size );
    if( ota_signer_cert == NULL )
    {
        if ( ota_pal_skip_sign_check( C ) != true )
        {
            cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "ota_pal_read_assume_certificate() failed.\n\r" );
            return OtaPalBadSignerCert;
        }
    }
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s() end ....\n\r", __func__ );
    return OtaPalSuccess;
}

/*-----------------------------------------------------------*/
/**
 * Do the final Signature Check for tarball OTA.
 * This function is used for a tarball OTA file.
 * This function is called from cy_awsport_ota_flash_close_receive_file().
 */
static OtaPalStatus_t ota_pal_file_sign_check_final( OtaFileContext_t * const C )
{
    OtaPalStatus_t   result = OtaPalSuccess;

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s() start ....\n\r", __func__ );
    if ( (C == NULL) || (C != cy_aws_ota_ctx.ota_fs_ctx) || (ota_sig_verify_context == NULL) )
    {
        /* Free the signer certificate that we now own after prvReadAndAssumeCertificate(). */
        ota_pal_file_sign_check_deinit();
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "ota_pal_file_sign_check_final failed\n\r" );
        return OtaPalBadSignerCert;
    }

    if( ( ota_pal_skip_sign_check( C ) != true ) && ( ( ota_signer_cert == NULL ) || ( ota_signer_cert_size == 0 ) ) )
    {
        ota_pal_file_sign_check_deinit();
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "ota_pal_file_sign_check_final failed\n\r" );
        return OtaPalBadSignerCert;
    }

    if( cy_crypto_sign_verification_final( ota_sig_verify_context, ( char * ) ota_signer_cert, ota_signer_cert_size,
                                           C->pSignature->data, C->pSignature->size ) != CY_RSLT_SUCCESS )
    {
        if( ota_pal_skip_sign_check( C ) != true )
        {
            cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "cy_crypto_sign_verification_final() failed.\n\r" );
            result = OtaPalSignatureCheckFailed;
        }
    }
    ota_last_sig_check = result;

    /* Free the signer certificate that we now own after prvReadAndAssumeCertificate(). */
    ota_pal_file_sign_check_deinit();
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s() end ....\n\r", __func__ );
    return result;
}

/*-----------------------------------------------------------*/
/**
 * Incremental update of the signature for tarball OTA.
 * This function is used for a tarball OTA file.
 * This function is called from cy_awsport_ota_pal_write_block().
 */
static OtaPalStatus_t ota_pal_file_sign_check_step( OtaFileContext_t * const C, uint8_t * buffer, uint32_t size )
{
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s() start ....\n\r", __func__ );
    if( (C == NULL) || (C != cy_aws_ota_ctx.ota_fs_ctx ) || (ota_sig_verify_context == NULL) )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, " Bad Arguments...!\n\r" );
        /* Free the signer certificate that we now own after prvReadAndAssumeCertificate(). */
        ota_pal_file_sign_check_deinit();
        if( ota_sig_verify_context == NULL )
        {
            return OtaPalBadSignerCert;
        }
        return OtaPalSignatureCheckFailed;
    }
    if( ota_pal_skip_sign_check( C ) != true )
    {
        cy_crypto_sign_verification_update( ota_sig_verify_context, buffer, size );
    }
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s() end ....\n\r", __func__ );
    return OtaPalSuccess;
}

/*-----------------------------------------------------------*/
/**
 * Erase the second slot to prepare for writing OTA image file.
 */
static int ota_erase_slot_two( void )
{
    const struct flash_area *fap;
    int image;

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s() start ....\n\r", __func__ );
    /* MCUBLOOT_IMAGE_NUMBER == 2, will erase both Secondary slot 1 and 2 */
    for( image = 0; image < MCUBOOT_IMAGE_NUMBER; image++ )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Start flash_area_open for %d...\n\r.", image );
        if( flash_area_open( FLASH_AREA_IMAGE_SECONDARY(image), &fap ) != 0 )
        {
            cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "flash_area_open(FLASH_AREA_IMAGE_SECONDARY(%d)) failed...\n\r", image );
            return -1;
        }
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Completed flash_area_open ...\n\r." );
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Start flash_area_erase ...\n\r." );
        if( flash_area_erase( fap, 0, fap->fa_size ) != 0 )
        {
            cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "flash_area_erase(fap, 0) (image:%d) failed...\n", image );
            return -1;
        }
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Completed flash_area_erase ...\n\r." );
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Start flash_area_close ...\n\r." );
        flash_area_close( fap );
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_INFO, "Completed flash_area_close ...\n\r." );
    }

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s() end ....\n\r", __func__ );
    return 0;
}
#endif

/*-----------------------------------------------------------*/
cy_rslt_t cy_awsport_ota_flash_init( void )
{
    cy_rslt_t cy_result = CY_RSLT_SUCCESS;

    (void)open_memstream(NULL, NULL);
#ifdef CY_OTA_FLASH_SUPPORT
    bool ota_mutex_init_status = false;

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s() start ....\n\r", __func__ );

    /* Clear the MQTT handle data. */
    memset( &cy_aws_ota_ctx, 0x00, sizeof( cy_awsport_ota_flash_ctx_t ) );

    cy_result = cy_rtos_init_mutex2( &(cy_aws_ota_ctx.ota_sync_mutex), false );
    if( cy_result != CY_RSLT_SUCCESS )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "\n Creating OTA mutex %p. failed", cy_aws_ota_ctx.ota_sync_mutex );
        return cy_result;
    }

    ota_mutex_init_status = true;

#ifdef CY_BOOT_USE_EXTERNAL_FLASH
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "psoc6_qspi_init started...\n\r" );
    cy_result = psoc6_qspi_init();
    if( cy_result != CY_RSLT_SUCCESS )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "psoc6_qspi_init() FAILED!!\n\r" );
        cy_result = CY_RSLT_AWS_IOT_PORT_ERROR_INIT_STORAGE;
        goto exit;
    }
    else
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "psoc6_qspi_init completed...\n\r ");
    }
#endif /* CY_BOOT_USE_EXTERNAL_FLASH */

exit :
    if( cy_result != CY_RSLT_SUCCESS )
    {
        if( ota_mutex_init_status == true )
        {
            (void)cy_rtos_deinit_mutex( &(cy_aws_ota_ctx.ota_sync_mutex) );
            ota_mutex_init_status = false;
        }
    }

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s() end ....\n\r", __func__ );
#else
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Flash support is not added\n" );
#endif
    return cy_result;
}

/*-----------------------------------------------------------*/
OtaPalStatus_t cy_awsport_ota_flash_abort( OtaFileContext_t * const C )
{
    cy_rslt_t        cy_result = CY_RSLT_SUCCESS;
    OtaPalStatus_t   result = OtaPalSuccess;

#ifdef CY_OTA_FLASH_SUPPORT
    const struct flash_area *fap;

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s() start ....\n\r", __func__ );

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Acquiring OTA Mutex %p \n\r", cy_aws_ota_ctx.ota_sync_mutex );
    cy_result = cy_rtos_get_mutex( &(cy_aws_ota_ctx.ota_sync_mutex), CY_RTOS_NEVER_TIMEOUT );
    if( cy_result != CY_RSLT_SUCCESS )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Acquiring OTA Mutex %p failed with Error : [0x%X] \n\r", cy_aws_ota_ctx.ota_sync_mutex, (unsigned int)result );
        return OTA_PAL_COMBINE_ERR( OtaPalAbortFailed, 0 );
    }
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Acquired OTA Mutex %p \n\r", cy_aws_ota_ctx.ota_sync_mutex );

    if ( (C == NULL) || (C != cy_aws_ota_ctx.ota_fs_ctx) )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Invalid file context.\n\r" );
        (void)cy_rtos_set_mutex( &(cy_aws_ota_ctx.ota_sync_mutex) );
        return OTA_PAL_COMBINE_ERR( OtaPalNullFileContext, 0 );
    }

    if( C->pFile == NULL )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Invalid file handle.\n\r" );
        (void)cy_rtos_set_mutex( &(cy_aws_ota_ctx.ota_sync_mutex) );
        return OTA_PAL_COMBINE_ERR( OtaPalNullFileContext, 0 );
    }

    fap = (const struct flash_area *)C->pFile;
    if( fap != NULL )
    {
        flash_area_close( fap );
    }

    /* reset our globals */
    C->pFile = NULL;
    cy_aws_ota_ctx.ota_fs_ctx = NULL;
    cy_aws_ota_ctx.ota_type = CY_AWS_IOT_OTA_TYPE_INVALID;

    /* Free the signer certificate that we now own after prvReadAndAssumeCertificate(). */
    ota_pal_file_sign_check_deinit();

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Releasing OTA Mutex %p \n\r", cy_aws_ota_ctx.ota_sync_mutex );
    cy_result = cy_rtos_set_mutex( &(cy_aws_ota_ctx.ota_sync_mutex) );
    if( cy_result != CY_RSLT_SUCCESS )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Failed to release OTA Mutex %p failed with Error : [0x%X] \n\r", cy_aws_ota_ctx.ota_sync_mutex, (unsigned int)cy_result );
        return OTA_PAL_COMBINE_ERR( OtaPalAbortFailed, 0 );
    }
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "\nReleased OTA Mutex %p ", cy_aws_ota_ctx.ota_sync_mutex );

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s() end ....\n\r", __func__ );
#else
    (void)cy_result;
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Flash support is not added\n" );
#endif

    return OTA_PAL_COMBINE_ERR( result, 0 );
}

/*-----------------------------------------------------------*/
OtaPalStatus_t cy_awsport_ota_flash_create_receive_file( OtaFileContext_t * const C )
{
    cy_rslt_t        cy_result = CY_RSLT_SUCCESS;

#ifdef CY_OTA_FLASH_SUPPORT
    const struct     flash_area *fap;

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s() start ....\n\r", __func__ );

    if( C == NULL )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Invalid file context.\n\r" );
        return OTA_PAL_COMBINE_ERR( OtaPalNullFileContext, 0 );
    }

    if( C->pFilePath == NULL )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_WARNING, "Invalid file path provided.\n\r" );
    }

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_INFO, "Complete OTA image size : %ld bytes \n\r", C->fileSize );

    /* prepare the slot for writing */
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_INFO, "Erasing slot 2 started...\n\r" );
    ota_erase_slot_two();
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_INFO, "Erasing slot 2 completed.\n\r" );

    /* Must set something into lFileHandle and we use for MQTT downloads */
    if( flash_area_open( FLASH_AREA_IMAGE_SECONDARY(0), &fap ) != 0 )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "OTA receive file creation failed...\n\r " );
        return OTA_PAL_COMBINE_ERR( OtaPalRxFileCreateFailed, 0 );
    }

    /* NOTE: FileHandle MUST be non-NULL of the OTA Agent will error out */
    C->pFile = (uint8_t *)fap;

    /* initialize these for checking later */
    ota_last_sig_check = OtaPalUninitialized;
    ota_sig_verify_context = NULL;
    ota_signer_cert = NULL;
    ota_signer_cert_size = 0;

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Acquiring OTA Mutex %p \n\r", cy_aws_ota_ctx.ota_sync_mutex );
    cy_result = cy_rtos_get_mutex( &(cy_aws_ota_ctx.ota_sync_mutex), CY_RTOS_NEVER_TIMEOUT );
    if( cy_result != CY_RSLT_SUCCESS )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Acquiring OTA Mutex %p failed with Error : [0x%X] \n\r", cy_aws_ota_ctx.ota_sync_mutex, (unsigned int)cy_result );
        return OTA_PAL_COMBINE_ERR( OtaPalRxFileCreateFailed, 0 );
    }
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Acquired OTA Mutex %p \n\r", cy_aws_ota_ctx.ota_sync_mutex );

    cy_aws_ota_ctx.ota_fs_ctx = C;
    cy_aws_ota_ctx.ota_flash_start_address = fap->fa_off;
    cy_aws_ota_ctx.ota_type = (cy_ota_type_t)C->fileType;

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Updated OTA context... \n\r" );
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Releasing OTA Mutex %p \n\r", cy_aws_ota_ctx.ota_sync_mutex );
    cy_result = cy_rtos_set_mutex( &(cy_aws_ota_ctx.ota_sync_mutex) );
    if( cy_result != CY_RSLT_SUCCESS )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Failed to release OTA Mutex %p failed with Error : [0x%X] \n\r", cy_aws_ota_ctx.ota_sync_mutex, (unsigned int)cy_result );
        return OTA_PAL_COMBINE_ERR( OtaPalRxFileCreateFailed, 0 );
    }
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "\nReleased OTA Mutex %p ", cy_aws_ota_ctx.ota_sync_mutex );
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_INFO, "OTA receive file created.\n\r" );

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s() end ....\n\r", __func__ );
#else
    (void)cy_result;
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Flash support is not added\n" );
#endif

    return OTA_PAL_COMBINE_ERR( OtaPalSuccess, 0 );
}

/*-----------------------------------------------------------*/
OtaPalStatus_t cy_awsport_ota_flash_close_receive_file( OtaFileContext_t * const C )
{
    cy_rslt_t        cy_result = CY_RSLT_SUCCESS;
    OtaPalStatus_t   result = OtaPalSuccess;

#ifdef CY_OTA_FLASH_SUPPORT
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s() start ....\n\r", __func__ );

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Acquiring OTA Mutex %p \n\r", cy_aws_ota_ctx.ota_sync_mutex );
    cy_result = cy_rtos_get_mutex( &(cy_aws_ota_ctx.ota_sync_mutex), CY_RTOS_NEVER_TIMEOUT );
    if( cy_result != CY_RSLT_SUCCESS )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Acquiring OTA Mutex %p failed with Error : [0x%X] \n\r", cy_aws_ota_ctx.ota_sync_mutex, (unsigned int)result );
        result = OtaPalFileClose;
        goto _exit_CloseFile;
    }
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Acquired OTA Mutex %p \n\r", cy_aws_ota_ctx.ota_sync_mutex );

    if ( (C == NULL) || (C != cy_aws_ota_ctx.ota_fs_ctx ) || (C->pFile == NULL))
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Invalid file context.\n\r" );
        result = OtaPalFileClose;
        goto _exit_CloseFile;
    }

    /* we got the data, try to verify it */
    if( C->pSignature == NULL )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "OTA signature structure is NULL.\n\r" );
        result = OtaPalSignatureCheckFailed;
        goto _exit_CloseFile;
    }

    /* For TAR files, we must use HTTP transport so that the file blocks come down
     * in sequential order - we need start of TAR (components.json) first, and we
     * do not store all of the blocks. Signature must be checked in sequential order.
     */
    if( ota_is_tar_archive == 1 )
    {
        /* Do the final check for signature */
        if( ota_pal_file_sign_check_final(C) != OtaPalSuccess )
        {
            cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "TAR ota_pal_file_sign_check_final() failed.\n\r" );
            ota_erase_slot_two();
            result = OtaPalSignatureCheckFailed;
        }
    }
    else
    {
        /* Non-tar OTA, we downloaded a single file, might have been MQTT.
         * If MQTT, the file blocks may come in non-sequential order.
         * We need to run signature check across entire downloaded file at once.
         */
        const struct flash_area *fap = NULL;

        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_INFO, "BINARY file, check whole file for signature.\n\r" );
        fap = (const struct flash_area *)C->pFile;
        if( fap == NULL )
        {
            cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "ERROR: fap == NULL.\n\r" );
            result = OtaPalSignatureCheckFailed;
            goto _exit_CloseFile;
        }

        result = ota_pal_file_sign_check_init( C );
        if( result == OtaPalSuccess )
        {
            uint32_t addr;
            addr = 0;
            while( addr < C->fileSize )
            {
                uint32_t toread = C->fileSize - addr;
                /* Re-use coalesce buffer as we have completed download (it is not used for non-TAR OTA) */
                if( toread > sizeof(cy_ota_untar_context.coalesce_buffer) )
                {
                    toread = sizeof( cy_ota_untar_context.coalesce_buffer );
                }

                if( flash_area_read( fap, addr, cy_ota_untar_context.coalesce_buffer, toread ) < 0 )
                {
                    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "flash_area_read() failed for signature check.\n\r" );
                    result = OtaPalSignatureCheckFailed;
                    goto _exit_CloseFile;
                }

                cy_crypto_sign_verification_update( ota_sig_verify_context, cy_ota_untar_context.coalesce_buffer, toread );
                addr += toread;
            }
            if( ota_pal_file_sign_check_final( C ) != OtaPalSuccess )
            {
                cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "ota_pal_file_sign_check_final() failed.\n\r" );
                ota_erase_slot_two();
                result = OtaPalSignatureCheckFailed;
            }
        }
        else
        {
            cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "ota_pal_file_sign_check_init() failed.\n\r" );
            ota_erase_slot_two();
            result = OtaPalSignatureCheckFailed;
        }
    }
    if(result == OtaPalSuccess)
    {
        uint32_t off;
        int rc = 0;
        const struct flash_area *fap;

        fap = (const struct flash_area *)C->pFile;

        /* For Device OTA, Remove the boot magic if present.
         * Boot magic is added when the user applies image.
         */
        if(fap != NULL && C->fileType == CY_AWS_IOT_OTA_TYPE_DEVICE_OTA)
        {
            uint8_t magic_erase[BOOT_MAGIC_SZ];

            off = fap->fa_size - BOOT_MAGIC_SZ;
            memset( &magic_erase, 0xFF, BOOT_MAGIC_SZ );

            /* Clear the magic num if any */
            rc = flash_area_write( fap, off, &magic_erase, BOOT_MAGIC_SZ );
            if( rc != 0 )
            {
                cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Failed to remove boot_magic.\n" );
            }
        }
    }

_exit_CloseFile:
    if ( C != NULL && C->pFile != NULL )
    {
        const struct flash_area *fap;
        fap = (const struct flash_area *)C->pFile;
        flash_area_close( fap );
    }

    /* Free the signer certificate that we now own after prvReadAndAssumeCertificate(). */
    ota_pal_file_sign_check_deinit();

    if( result != OtaPalSuccess )
    {
        current_ota_image_state = OtaImageStateUnknown;
        cy_aws_ota_ctx.ota_type = CY_AWS_IOT_OTA_TYPE_INVALID;
    }

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Releasing OTA Mutex %p \n\r", cy_aws_ota_ctx.ota_sync_mutex );
    cy_result = cy_rtos_set_mutex( &(cy_aws_ota_ctx.ota_sync_mutex) );
    if( cy_result != CY_RSLT_SUCCESS )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Failed to release OTA Mutex %p failed with Error : [0x%X] \n\r", cy_aws_ota_ctx.ota_sync_mutex, (unsigned int)cy_result );
        result = OtaPalFileClose;
    }
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "\nReleased OTA Mutex %p ", cy_aws_ota_ctx.ota_sync_mutex );

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s() end ....\n\r", __func__ );
#else
    (void)cy_result;
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Flash support is not added\n" );
#endif

    return OTA_PAL_COMBINE_ERR( result, 0 );
}

/*-----------------------------------------------------------*/
int16_t cy_awsport_ota_flash_write_block( OtaFileContext_t * const C, uint32_t ulOffset,
                                          uint8_t * const pcData, uint32_t ulBlockSize )
{
    cy_rslt_t        cy_result = CY_RSLT_SUCCESS;

#ifdef CY_OTA_FLASH_SUPPORT
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s() start ....\n\r", __func__ );

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Acquiring OTA Mutex %p \n\r", cy_aws_ota_ctx.ota_sync_mutex );
    cy_result = cy_rtos_get_mutex( &(cy_aws_ota_ctx.ota_sync_mutex), CY_RTOS_NEVER_TIMEOUT );
    if( cy_result != CY_RSLT_SUCCESS )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Acquiring OTA Mutex %p failed with Error : [0x%X] \n\r", cy_aws_ota_ctx.ota_sync_mutex, (unsigned int)cy_result );
        return -1;
    }
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Acquired OTA Mutex %p \n\r", cy_aws_ota_ctx.ota_sync_mutex );

    if( (C == NULL) || (C != cy_aws_ota_ctx.ota_fs_ctx ) )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Invalid file context.\n\r" );
        (void)cy_rtos_set_mutex( &(cy_aws_ota_ctx.ota_sync_mutex) );
        return -1;
    }

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "ulOffset = %ld \n\r", ulOffset );
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "ulBlockSize = %ld \n\r", ulBlockSize );
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_INFO, "Number of blocks remaining in OTA image : %ld \n\r", C->blocksRemaining );

    /* we need to check some things when we receive the first block */
    if( ulOffset == 0UL )
    {
        /* Initialize file signature checking */
        if( ota_pal_file_sign_check_init( C ) != OtaPalSuccess )
        {
            cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "ota_pal_file_sign_check_init() FAILED.\n\r" );
            (void)cy_rtos_set_mutex( &(cy_aws_ota_ctx.ota_sync_mutex) );
            return -1;
        }

        /*
         * Check for incoming tarball (as opposed to a single file OTA)
         */
        if( cy_is_tar_header( pcData, ulBlockSize ) == CY_UNTAR_SUCCESS )
        {
            if( ota_untar_init_context( &cy_ota_untar_context, C ) != CY_RSLT_SUCCESS )
            {
                cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "ota_untar_init_context() FAILED! \n\r" );
                (void)cy_rtos_set_mutex( &(cy_aws_ota_ctx.ota_sync_mutex) );
                return -1;
            }
        }
    }

    /* Treat a tar file differently from a "normal" OTA */
    if( ota_is_tar_archive != 0 )
    {
        uint32_t consumed = 0;

        /* check the signature incrementally over every block received */
        if( ota_pal_file_sign_check_step( C, pcData, ulBlockSize) != OtaPalSuccess )
        {
            cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "FileSignatureCheckStep() offset:%ld FAILED!\n", ulOffset );
            (void)cy_rtos_set_mutex( &(cy_aws_ota_ctx.ota_sync_mutex) );
            return -1;
        }

        while( consumed < ulBlockSize )
        {
            cy_untar_result_t result;
            result = cy_untar_parse( &cy_ota_untar_context, (ulOffset + consumed), &pcData[consumed], (ulBlockSize- consumed), &consumed );
            if( (result == CY_UNTAR_ERROR) || (result == CY_UNTAR_INVALID))
            {
                cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "cy_untar_parse() FAIL. consumed: %ld block size: %ld result : %ld!\n", consumed, ulBlockSize, result );
                (void)cy_rtos_set_mutex( &(cy_aws_ota_ctx.ota_sync_mutex) );
                return -1;
            }
            /* Yield for a bit */
            cy_rtos_delay_milliseconds( 1 );
        }

        /* with the tarball we get a version - check if it is > current so we can bail early */
#ifdef CY_TEST_APP_VERSION_IN_TAR
        if( cy_ota_untar_context.version[0] != 0 )
        {
            /* example version string "<major>.<minor>.<build>" */
            uint16_t major = 0;
            uint16_t minor = 0;
            uint16_t build = 0;
            char *dot;
            major = atoi( cy_ota_untar_context.version );
            dot = strstr( cy_ota_untar_context.version, "." );
            if( dot != NULL )
            {
                dot++;
                minor = atoi( dot );
                dot = strstr( dot, "." );
                if( dot != NULL )
                {
                    dot++;
                    build = atoi( dot );

                    if( (major < APP_VERSION_MAJOR) ||
                          ( (major == APP_VERSION_MAJOR) &&
                            (minor < APP_VERSION_MINOR)) ||
                          ( (major == APP_VERSION_MAJOR) &&
                            (minor == APP_VERSION_MINOR) &&
                            (build <= APP_VERSION_BUILD)) )
                    {
                        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "OTA image version %d.%d.%d <= current %d.%d.%d-- bail!\n\r",
                                       major, minor, build, APP_VERSION_MAJOR, APP_VERSION_MINOR, APP_VERSION_BUILD );
                        (void)cy_rtos_set_mutex( &(cy_aws_ota_ctx.ota_sync_mutex) );
                        return -1;
                    }
                }
            }
        }
#endif  /* CY_TEST_APP_VERSION_IN_TAR */
    }
    else
    {
        /* non-tarball OTA here */
        const struct flash_area *fap;
        fap = (const struct flash_area *)C->pFile;
        if( fap == NULL )
        {
            cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "flash_area_pointer is NULL.\n\r" );
            (void)cy_rtos_set_mutex( &(cy_aws_ota_ctx.ota_sync_mutex) );
            return -1;
        }

        if( ota_write_data_to_flash( fap, ulOffset, pcData, ulBlockSize ) == -1 )
        {
            (void)cy_rtos_set_mutex( &(cy_aws_ota_ctx.ota_sync_mutex) );
            return -1;
        }
    }

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Releasing OTA Mutex %p \n\r", cy_aws_ota_ctx.ota_sync_mutex );
    cy_result = cy_rtos_set_mutex( &(cy_aws_ota_ctx.ota_sync_mutex) );
    if( cy_result != CY_RSLT_SUCCESS )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Failed to release OTA Mutex %p failed with Error : [0x%X] \n\r", cy_aws_ota_ctx.ota_sync_mutex, (unsigned int)cy_result );
    }
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "\nReleased OTA Mutex %p ", cy_aws_ota_ctx.ota_sync_mutex );

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s() end ....\n\r", __func__ );
#else
    (void)cy_result;
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Flash support is not added\n" );
#endif

    return ulBlockSize;
}

/*-----------------------------------------------------------*/
OtaPalStatus_t cy_awsport_ota_flash_activate_newimage( OtaFileContext_t * const C )
{
    OtaPalMainStatus_t result = OtaPalSuccess;

#ifdef CY_OTA_FLASH_SUPPORT
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s() start ....\n\r", __func__ );

    /* Mark this new OTA image as pending, it will be the permanent bootable image going forward. */
    if( (ota_is_tar_archive != 0) && (cy_ota_untar_context.magic == CY_UNTAR_CONTEXT_MAGIC) )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_INFO, "TAR ota_pal_file_sign_check_final() GOOD.\n\r" );
        ota_untar_set_pending();
    }
    else
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_INFO, "BIN ota_pal_file_sign_check_final() GOOD.\n\r" );
        /* Non-tar only has type 0 (NSPE) update */
        boot_set_pending( 0, 0 );
    }

    result = cy_awsport_ota_flash_reset_device( C );
    if( OTA_PAL_MAIN_ERR( result ) != OtaPalSuccess )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_INFO, "cy_awsport_ota_flash_reset_device failed.\n\r" );
    }

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s() end ....\n\r", __func__ );
#else
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Flash support is not added\n" );
#endif

    return result;
}

/*-----------------------------------------------------------*/
OtaPalStatus_t cy_awsport_ota_flash_set_platform_imagestate( OtaFileContext_t * const C,
                                                             OtaImageState_t eState )
{
    OtaPalStatus_t   result = OtaPalSuccess;

#ifdef CY_OTA_FLASH_SUPPORT
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s() start ....\n\r", __func__ );

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Current OTA image state : %d \n\r", (int)current_ota_image_state );
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "New OTA image state : %d \n\r", (int)eState );

    if( eState == OtaImageStateUnknown || eState > OtaLastImageState )
    {
        return OTA_PAL_COMBINE_ERR( OtaPalBadImageState, 0 );
    }

    if( cy_aws_ota_ctx.ota_fs_ctx == NULL )
    {
        /* We are not currently loading an OTA image. This state change pertains to Slot 0 image */
        switch( eState )
        {
            case OtaImageStateAccepted:
                /* Mark Slot 0 image as valid. */
                /* We need to know if the last check was good... */
                if( ota_last_sig_check == OtaPalSuccess )
                {
                    boot_set_confirmed();
                    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_INFO, "Boot Set confirmed.\n\r" );
                }
                else
                {
                    result = OtaPalCommitFailed;
                }
                break;
            case OtaImageStateRejected:
                /* we haven't closed the file, and the OTA Agent has rejected the download */
                result = OtaPalSuccess;
                break;
            case OtaImageStateAborted:
                /* We are not actively downloading, this pertains to an invalid download job
                 * or an aborted job. We erase secondary slot when we start a new download,
                 * and do not consider secondary slot ready unless download completes and
                 * signature checked. nothing to do here, but set our state to Accepted so
                 * we store it for GetImageState().
                 */
                eState = OtaImageStateAccepted;
                break;
            case OtaImageStateTesting:
                break;
            default:
                result = OtaPalBadImageState;
                break;
        }
    }
    else
    {
        /* We are in the middle of a new download
         * Abort or Reject refers to the new download, not the primary slot.
         * Nothing to do here.
         */
        if( eState == OtaImageStateAccepted )
        {
            if( cy_aws_ota_ctx.ota_fs_ctx->pFile != NULL )
            {
                result = OtaPalCommitFailed;
            }
        }
    }

    /* keep track of the state OTA Agent sent. */
    current_ota_image_state = eState;
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_INFO, "Current OTA image eSTATE : %d \n\r", eState );

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s() end ....\n\r", __func__ );
#else
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Flash support is not added\n" );
#endif

    return OTA_PAL_COMBINE_ERR( result, 0 );
}

/*-----------------------------------------------------------*/
OtaPalImageState_t cy_awsport_ota_flash_get_platform_imagestate( OtaFileContext_t * const C )
{
    OtaPalImageState_t result = OtaPalImageStateUnknown;

#ifdef CY_OTA_FLASH_SUPPORT
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s() start ....\n\r", __func__ );

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_INFO, "\n\rCurrent OTA image eSTATE : %d \n\r", current_ota_image_state );

    /**
     * After swap/copy of secondary slot to primary slot, boot_swap_type() returns NONE.
     * It does not reflect the fact we may be in self-test mode.
     * Use the saved value from SetImageState() to report our status.
     */
    if( current_ota_image_state == OtaImageStateTesting )
    {
        /* in self-test, report Pending. */
        result = OtaPalImageStatePendingCommit;
    }
    else if( (current_ota_image_state == OtaImageStateRejected ) ||
             (current_ota_image_state == OtaImageStateAborted ) )
    {
        result = OtaPalImageStateInvalid;
    }
    else if( current_ota_image_state == OtaImageStateAccepted )
    {
        result = OtaPalImageStateValid;
    }
    else
    {
        result = OtaPalImageStateUnknown;
    }

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_INFO, "Platform Image state : %d \n\r", (int)result );

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s() end ....\n\r", __func__ );
#else
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Flash support is not added\n" );
#endif

    return result;
}

/*-----------------------------------------------------------*/
OtaPalStatus_t cy_awsport_ota_flash_reset_device( OtaFileContext_t * const C )
{
    ( void ) C;

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s() start ....\n\r", __func__ );

#ifdef CY_OTA_FLASH_SUPPORT
    /* we want to wait a bit when in DEBUG builds so the logging mechanism can finish before resetting. */
    cy_rtos_delay_milliseconds( 1000UL );
    /* Really not an error. Wanted to print restart message with highest priority. */
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "\n\n\rRESETTING NOW....... !!!!\n\r" );
    cy_rtos_delay_milliseconds( 1000UL );
    NVIC_SystemReset();
#endif

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s() end ....\n\r", __func__ );
    return OTA_PAL_COMBINE_ERR( OtaPalSuccess, 0 );
}

/*-----------------------------------------------------------*/
cy_rslt_t cy_awsport_ota_flash_image_validate( void )
{
    int16_t boot_result = 0;

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s() start ....\n\r", __func__ );

#ifdef CY_OTA_FLASH_SUPPORT
    /* Mark Image in Primary Slot as valid.   */
    boot_result = boot_set_confirmed();
    if( boot_result != 0 )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s() end ....\n\r", __func__ );
        return CY_RSLT_AWS_IOT_PORT_ERROR_GENERAL;
    }

#else
    (void)boot_result;
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Flash support is not added\n" );
#endif

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s() end ....\n\r", __func__ );
    return CY_RSLT_SUCCESS;
}

/*-----------------------------------------------------------*/
cy_rslt_t cy_awsport_ota_remove_boot_magic(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

#ifdef CY_OTA_FLASH_SUPPORT
    const struct flash_area *fap;
    int rc = 0;
    uint32_t off = 0;

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s() start ....\n\r", __func__ );
    rc = flash_area_open( FLASH_AREA_IMAGE_SECONDARY(0), &fap );
    if(rc != 0)
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Failed to get flash handle.\n" );
        result = CY_RSLT_AWS_IOT_PORT_ERROR_GENERAL;
    }
    else
    {
        uint8_t magic_erase[BOOT_MAGIC_SZ];

        off = fap->fa_size - BOOT_MAGIC_SZ;
        memset( &magic_erase, 0xFF, BOOT_MAGIC_SZ );

        rc = flash_area_write( fap, off, &magic_erase, BOOT_MAGIC_SZ );
        if( rc != 0 )
        {
            cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Failed to remove boot_magic.\n" );
            result = CY_RSLT_AWS_IOT_PORT_ERROR_GENERAL;
        }
        flash_area_close( fap );
    }
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s() end ....\n\r", __func__ );
#else
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Flash support is not added\n" );
#endif
    return result;
}

/*-----------------------------------------------------------*/
void * cy_awsport_ota_flash_get_handle( void )
{
    cy_rslt_t    cy_result = CY_RSLT_SUCCESS;
    void         *flash_handle = NULL;

#ifdef CY_OTA_FLASH_SUPPORT
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s() start ....\n\r", __func__ );

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Acquiring OTA Mutex %p \n\r", cy_aws_ota_ctx.ota_sync_mutex );
    cy_result = cy_rtos_get_mutex( &(cy_aws_ota_ctx.ota_sync_mutex), CY_RTOS_NEVER_TIMEOUT );
    if( cy_result != CY_RSLT_SUCCESS )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Acquiring OTA Mutex %p failed with Error : [0x%X] \n\r", cy_aws_ota_ctx.ota_sync_mutex, (unsigned int)cy_result );
        flash_handle = NULL;
        goto exit;
    }
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Acquired OTA Mutex %p \n\r", cy_aws_ota_ctx.ota_sync_mutex );

    if( cy_aws_ota_ctx.ota_fs_ctx != NULL )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "OTA flash storage handle = %p \n\r", cy_aws_ota_ctx.ota_fs_ctx );
        flash_handle = ( void *)cy_aws_ota_ctx.ota_fs_ctx;
    }
    else
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_INFO, "OTA flash storage is NULL...!!! \n\r" );
        flash_handle = NULL;
        goto exit;
    }

exit :
    if( cy_result == CY_RSLT_SUCCESS )
    {
        (void)cy_rtos_set_mutex( &(cy_aws_ota_ctx.ota_sync_mutex) );
    }

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s() end ....\n\r", __func__ );
#else
    (void)cy_result;
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Flash support is not added\n" );
#endif

    return flash_handle;
}

/*-----------------------------------------------------------*/
cy_rslt_t cy_awsport_ota_flash_read( uint32_t bytes_to_read, uint8_t * const read_buf,
                                     uint32_t read_buf_size, uint32_t *bytes_received )
{
    cy_rslt_t    cy_result = CY_RSLT_SUCCESS;

#ifdef CY_OTA_FLASH_SUPPORT
    uint8_t      flash_id = 0;
    int          rc = 0;
    uint32_t     invalid_bytes = 0;
    const struct flash_area     *fap = NULL;

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s() start ....\n\r", __func__ );

    if( (read_buf == NULL) || (bytes_to_read < 0) )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Invalid arg to read.\n\r" );
        return CY_RSLT_AWS_IOT_PORT_ERROR_BADARG;
    }

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Acquiring OTA Mutex %p \n\r", cy_aws_ota_ctx.ota_sync_mutex );
    cy_result = cy_rtos_get_mutex( &(cy_aws_ota_ctx.ota_sync_mutex), CY_RTOS_NEVER_TIMEOUT );
    if( cy_result != CY_RSLT_SUCCESS )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Acquiring OTA Mutex %p failed with Error : [0x%X] \n\r", cy_aws_ota_ctx.ota_sync_mutex, (unsigned int)cy_result );
        return CY_RSLT_AWS_IOT_PORT_ERROR_READ_STORAGE;
    }
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Acquired OTA Mutex %p \n\r", cy_aws_ota_ctx.ota_sync_mutex );

    if( cy_aws_ota_ctx.ota_type == CY_AWS_IOT_OTA_TYPE_DEVICE_OTA )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Read is not allowed for device OTA \n\r" );
        (void)cy_rtos_set_mutex( &(cy_aws_ota_ctx.ota_sync_mutex) );
        return CY_RSLT_AWS_IOT_PORT_ERROR_READ_STORAGE;
    }

    flash_id = FLASH_AREA_IMAGE_SECONDARY(0);

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Current read offset = %ld \n\r", cy_aws_ota_ctx.ota_flash_read_offset );
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Requested bytes = %ld \n\r", bytes_to_read );

    /* Always read from secondary slot */
    if( flash_area_open( flash_id, &fap ) != 0 )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "flash_area_open(FLASH_AREA_IMAGE_SECONDARY(0) ) failed\r\n" );
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Releasing OTA Mutex %p \n\r", cy_aws_ota_ctx.ota_sync_mutex );
        (void)cy_rtos_set_mutex( &(cy_aws_ota_ctx.ota_sync_mutex) );
        return CY_RSLT_AWS_IOT_PORT_ERROR_OPEN_STORAGE;
    }

    /* read into the chunk_info buffer */
    rc = flash_area_read( fap, cy_aws_ota_ctx.ota_flash_read_offset, read_buf, bytes_to_read );
    if( rc != 0 )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "flash_area_read() failed result:%d \n\r", __func__, rc );
        flash_area_close( fap );
        (void)cy_rtos_set_mutex( &(cy_aws_ota_ctx.ota_sync_mutex) );
        return CY_RSLT_AWS_IOT_PORT_ERROR_READ_STORAGE;
    }

    if( (cy_aws_ota_ctx.ota_flash_start_address + fap->fa_size) < (cy_aws_ota_ctx.ota_flash_start_address + cy_aws_ota_ctx.ota_flash_read_offset + bytes_to_read) )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_INFO, "Reading beyond HOTA storage space...!!! \n\r" );
        invalid_bytes = cy_aws_ota_ctx.ota_flash_start_address + cy_aws_ota_ctx.ota_flash_read_offset + bytes_to_read;
        invalid_bytes -=  (cy_aws_ota_ctx.ota_flash_start_address + fap->fa_size);
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_INFO, "Last %ld bytes in read buffer are invalid...!!! \n\r", invalid_bytes );
    }

    *bytes_received = bytes_to_read - invalid_bytes;

    flash_area_close( fap );

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Releasing OTA Mutex %p \n\r", cy_aws_ota_ctx.ota_sync_mutex );
    cy_result = cy_rtos_set_mutex( &(cy_aws_ota_ctx.ota_sync_mutex) );
    if( cy_result != CY_RSLT_SUCCESS )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Failed to release OTA Mutex %p failed with Error : [0x%X] \n\r", cy_aws_ota_ctx.ota_sync_mutex, (unsigned int)cy_result );
        cy_result = CY_RSLT_AWS_IOT_PORT_ERROR_READ_STORAGE;
    }
    else
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "\nReleased OTA Mutex %p ", cy_aws_ota_ctx.ota_sync_mutex );
    }

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s() end ....\n\r", __func__ );
#else
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Flash support is not added\n" );
#endif

    return cy_result;
}

/*-----------------------------------------------------------*/
cy_rslt_t cy_awsport_ota_flash_seek( uint32_t offset )
{
    cy_rslt_t cy_result = CY_RSLT_SUCCESS;

#ifdef CY_OTA_FLASH_SUPPORT
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s() start ....\n\r", __func__ );

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Acquiring OTA Mutex %p \n\r", cy_aws_ota_ctx.ota_sync_mutex );
    cy_result = cy_rtos_get_mutex( &(cy_aws_ota_ctx.ota_sync_mutex), CY_RTOS_NEVER_TIMEOUT );
    if( cy_result != CY_RSLT_SUCCESS )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Acquiring OTA Mutex %p failed with Error : [0x%X] \n\r", cy_aws_ota_ctx.ota_sync_mutex, (unsigned int)cy_result );
        return CY_RSLT_AWS_IOT_PORT_ERROR_READ_STORAGE;
    }
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Acquired OTA Mutex %p \n\r", cy_aws_ota_ctx.ota_sync_mutex );

    cy_aws_ota_ctx.ota_flash_read_offset = offset;

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Releasing OTA Mutex %p \n\r", cy_aws_ota_ctx.ota_sync_mutex );
    cy_result = cy_rtos_set_mutex( &(cy_aws_ota_ctx.ota_sync_mutex) );
    if( cy_result != CY_RSLT_SUCCESS )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Failed to release OTA Mutex %p failed with Error : [0x%X] \n\r", cy_aws_ota_ctx.ota_sync_mutex, (unsigned int)cy_result );
        return cy_result;
    }
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "\nReleased OTA Mutex %p ", cy_aws_ota_ctx.ota_sync_mutex );

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s() end ....\n\r", __func__ );
#else
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Flash support is not added\n" );
#endif

    return cy_result;
}

/*-----------------------------------------------------------*/
cy_rslt_t cy_awsport_ota_flash_erase( uint8_t flash_id )
{
    cy_rslt_t cy_result = CY_RSLT_SUCCESS;

#ifdef CY_OTA_FLASH_SUPPORT
    const struct flash_area *fap;

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s() start ....\n\r", __func__ );

    cy_aws_ota_ctx.ota_type = CY_AWS_IOT_OTA_TYPE_INVALID;

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Start flash_area_open for %d...\n\r.", flash_id );
    if( flash_area_open( flash_id, &fap ) != 0 )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "flash_area_open(FLASH_AREA_IMAGE_SECONDARY(%d)) failed...\n\r", flash_id );
        return CY_RSLT_AWS_IOT_PORT_ERROR_OPEN_STORAGE;
    }
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Completed flash_area_open ...\n\r." );
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Start flash_area_erase ...\n\r." );
    if( flash_area_erase( fap, 0, fap->fa_size ) != 0 )
    {
        cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "flash_area_erase(fap, 0) (flash_id:%d) failed...\n", flash_id );
        return CY_RSLT_AWS_IOT_PORT_ERROR_GENERAL;
    }
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Completed flash_area_erase ...\n\r." );
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Start flash_area_close ...\n\r." );
    flash_area_close( fap );
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_INFO, "Completed flash_area_close ...\n\r." );

    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_DEBUG, "%s() end ....\n\r", __func__ );
#else
    cy_ap_log_msg( CYLF_MIDDLEWARE, CY_LOG_ERR, "Flash support is not added\n" );
#endif
    return cy_result;
}
/*-----------------------------------------------------------*/
