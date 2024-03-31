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
 *  @file cy_ota_storage.h
 *  Function declarations for OTA functional storage Interface APIs for AnyCloud framework.
 */

#ifndef CY_OTA_STORAGE_H_
#define CY_OTA_STORAGE_H_

#include "cy_aws_iot_port_error.h"

/* OTA includes */
#include "ota.h"
#include "ota_config.h"

typedef enum cy_ota_type
{
    CY_AWS_IOT_OTA_TYPE_INVALID = 0,
    CY_AWS_IOT_OTA_TYPE_DEVICE_OTA = 101,
    CY_AWS_IOT_OTA_TYPE_OTA_CERT = 103,
    CY_AWS_IOT_OTA_TYPE_ROOT_CERT = 107,
    CY_AWS_IOT_OTA_TYPE_HOST_OTA = 202,
    CY_AWS_IOT_OTA_TYPE_HOTA_CERT = 204
} cy_ota_type_t;

/**
 * Code signing certificate is used for OTA image signing.
 */
static const char signingcredentialSIGNING_CERTIFICATE_PEM[] = AWS_IOT_OTA_SIGNING_CERT;

/**
 * Initializes external flash required for AWS OTA.
 *
 * @return cy_rslt_t                : CY_RSLT_SUCCESS on success; error codes otherwise.
 *
 */
cy_rslt_t cy_awsport_ota_flash_init( void );

/**
 * Abort an OTA transfer.
 *
 * Aborts access to an existing open file represented by the OTA file context C. This is
 * only valid for jobs that started successfully.
 *
 * @note The input OtaFileContext_t C is checked for NULL by the OTA agent before this
 * function is called.
 * This function may be called before the file is opened, so the file pointer C->fileHandle
 * may be NULL when this function is called.
 *
 * @param C [in]                    : OTA file context information.
 *
 * @return The OTA PAL layer error code combined with the MCU specific error code. See OTA Agent
 * error codes information in ota.h.
 *
 * The file pointer will be set to NULL after this function returns.
 * OtaPalSuccess is returned when aborting access to the open file was successful.
 * OtaPalFileAbort is returned when aborting access to the open file context was unsuccessful.
 */
OtaPalStatus_t cy_awsport_ota_flash_abort( OtaFileContext_t * const C );

/**
 * Create a new receive file for the data chunks as they come in.
 *
 * @note Opens the file indicated in the OTA file context in the MCU file system.
 *
 * @note The previous image may be present in the designated image download partition or file, so the
 * partition or file must be completely erased or overwritten in this routine.
 *
 * @note The input OtaFileContext_t C is checked for NULL by the OTA agent before this
 * function is called.
 * The device file path is a required field in the OTA job document, so C->pFilePath is
 * checked for NULL by the OTA agent before this function is called.
 *
 * @param C [in]                    : OTA file context information.
 *
 * @return The OTA PAL layer error code combined with the MCU specific error code. See OTA Agent
 * error codes information in ota.h.
 *
 * OtaPalSuccess is returned when file creation is successful.
 * OtaPalRxFileTooLarge is returned if the file to be created exceeds the device's non-volatile memory size constraints.
 * OtaPalBootInfoCreateFailed is returned if the bootloader information file creation fails.
 * OtaPalRxFileCreateFailed is returned for other errors creating the file in the device's non-volatile memory.
 */
OtaPalStatus_t cy_awsport_ota_flash_create_receive_file( OtaFileContext_t * const C );

/* Authenticate and close the underlying receive file in the specified OTA context.
 *
 * @note The input OtaFileContext_t C is checked for NULL by the OTA agent before this
 * function is called. This function is called only at the end of block ingestion.
 * cy_awsport_ota_pal_create_receive_file() must succeed before this function is reached, so
 * C->fileHandle(or C->pFile) is never NULL.
 * The certificate path on the device is a required job document field in the OTA Agent,
 * so C->pCertFilepath is never NULL.
 * The file signature key is required job document field in the OTA Agent, so C->pSignature will
 * never be NULL.
 *
 * If the signature verification fails, file close should still be attempted.
 *
 * @param C [in]                    : OTA file context information.
 *
 * @return The OTA PAL layer error code combined with the MCU specific error code. See OTA Agent
 * error codes information in ota.h.
 *
 * OtaPalSuccess is returned on success.
 * OtaPalSignatureCheckFailed is returned when cryptographic signature verification fails.
 * OtaPalBadSignerCert is returned for errors in the certificate itself.
 * OtaPalFileClose is returned when closing the file fails.
 */
OtaPalStatus_t cy_awsport_ota_flash_close_receive_file( OtaFileContext_t * const C );

/**
 * Write a block of data to the specified file at the given offset.
 *
 * @note The input OtaFileContext_t C is checked for NULL by the OTA agent before this
 * function is called.
 * The file pointer/handle C->pFile, is checked for NULL by the OTA agent before this
 * function is called.
 * pData is checked for NULL by the OTA agent before this function is called.
 * blockSize is validated for range by the OTA agent before this function is called.
 * offset is validated by the OTA agent before this function is called.
 *
 * @param C [in]                    : OTA file context information.
 * @param ulOffset [in]             : Byte offset to write to from the beginning of the file.
 * @param pcData [in]               : Pointer to the byte array of data to write.
 * @param ulBlockSize [in]          : blockSize The number of bytes to write.
 *
 * @return The number of bytes written on a success, or a negative error code from the platform
 * abstraction layer.
 */
int16_t cy_awsport_ota_flash_write_block( OtaFileContext_t * const C, uint32_t ulOffset,
                                          uint8_t * const pcData, uint32_t ulBlockSize );

/**
 * Activate the newest MCU image received via OTA.
 *
 * This function activates the newest firmware received via OTA. It is typically just a reset of
 * the device.
 *
 * @note This function SHOULD not return. If it does, the platform does not support
 * an automatic reset or an error occurred.
 *
 * @return The OTA PAL layer error code combined with the MCU specific error code. See OTA Agent
 * error codes information in ota.h.
 */
OtaPalStatus_t cy_awsport_ota_flash_activate_newimage( OtaFileContext_t * const C );

/**
 * Reset the device.
 *
 * This function shall reset the MCU and cause a reboot of the system.
 *
 * @note This function SHOULD not return. If it does, the platform does not support
 * an automatic reset or an error occurred.
 *
 * @param C [in]                    : OTA file context information.
 *
 * @return The OTA PAL layer error code combined with the MCU specific error code. See OTA Agent
 * error codes information in ota.h.
 */

OtaPalStatus_t cy_awsport_ota_flash_reset_device( OtaFileContext_t * const C );

/**
 * Attempt to set the state of the OTA update image.
 *
 * Do whatever is required by the platform to Accept/Reject the OTA update image (or bundle).
 * Refer to the PAL implementation to determine what happens on your platform.
 *
 * @param C [in]                    : OTA file context information.
 * @param eState [in]               : The desired state of the OTA update image.
 *
 * @return The OtaPalStatus_t error code combined with the MCU specific error code. See ota.h for
 *         OTA major error codes and your specific PAL implementation for the sub error code.
 *
 * Major error codes returned are:
 *
 *   OtaPalSuccess on success.
 *   OtaPalBadImageState: if you specify an invalid OtaImageState_t. No sub error code.
 *   OtaPalAbortFailed: failed to roll back the update image as requested by OtaImageStateAborted.
 *   OtaPalRejectFailed: failed to roll back the update image as requested by OtaImageStateRejected.
 *   OtaPalCommitFailed: failed to make the update image permanent as requested by OtaImageStateAccepted.
 */
OtaPalStatus_t cy_awsport_ota_flash_set_platform_imagestate( OtaFileContext_t * const C,
                                                             OtaImageState_t eState );

/**
 * Get the state of the OTA update image.
 *
 * We read this at OTA_Init time and when the latest OTA job reports itself in self
 * test. If the update image is in the "pending commit" state, we start a self test
 * timer to assure that we can successfully connect to the OTA services and accept
 * the OTA update image within a reasonable amount of time (user configurable). If
 * we don't satisfy that requirement, we assume there is something wrong with the
 * firmware and automatically reset the device, causing it to roll back to the
 * previously known working code.
 *
 * If the update image state is not in "pending commit," the self test timer is
 * not started.
 *
 * @param C [in]                    : OTA file context information.
 *
 * @return An OtaPalImageState_t. One of the following:
 *   OtaPalImageStatePendingCommit (the new firmware image is in the self test phase)
 *   OtaPalImageStateValid         (the new firmware image is already committed)
 *   OtaPalImageStateInvalid       (the new firmware image is invalid or non-existent)
 *
 *   NOTE: OtaPalImageStateUnknown should NEVER be returned and indicates an implementation error.
 */
OtaPalImageState_t cy_awsport_ota_flash_get_platform_imagestate( OtaFileContext_t * const C );

/**
 * The application has validated the new OTA image.
 * This call must be after reboot and MCUboot has copied the new application
 * to the primary slot.
 *
 * @return cy_rslt_t                : CY_RSLT_SUCCESS on success; error codes otherwise.
 *
 */
cy_rslt_t cy_awsport_ota_flash_image_validate( void );

/**
 * Sets offset value for reading OTA image file.
 *
 * @param offset [in]               : offset value.
 *
 * @return cy_rslt_t                : CY_RSLT_SUCCESS on success; error codes otherwise.
 */
cy_rslt_t cy_awsport_ota_flash_seek( uint32_t offset );

/**
 * Reads requested bytes of data from offset which is set using /refcy_awsport_ota_flash_seek.
 *
 * Start address of flash is calculated based on OTA type.
 *
 * @param bytes_to_read [in]        : Number of bytes to read from OTA file.
 * @param read_buf [in]             : OTA read buffer.
 * @param read_buf_size [in]        : Size of OTA read buffer.
 * @param bytes_received [out]      : Number of bytes read from flash.
 *
 * @return cy_rslt_t                : CY_RSLT_SUCCESS on success; error codes otherwise.
 */
cy_rslt_t cy_awsport_ota_flash_read( uint32_t bytes_to_read, uint8_t * const read_buf,
                                     uint32_t read_buf_size, uint32_t *bytes_received );

/**
 * Erase OTA flash area.
 *
 * @param flash_id [in]             : Flash storage ID.
 *
 * @return cy_rslt_t                : CY_RSLT_SUCCESS on success; error codes otherwise.
 */
cy_rslt_t cy_awsport_ota_flash_erase( uint8_t flash_id );

/**
 * Gets the Flash handle required for AWS OTA.
 *
 * @return void *                   : Flash handle on success; NULL otherwise.
 *
 */
void * cy_awsport_ota_flash_get_handle( void );

/**
 * Gets Signer certificate for the ota_type
 *
 * @param ota_type [in]             : Specify OTA type.
 * @param pucCertName [in]          : Certificate name.
 * @param ota_signer_cert_size [out]: Size of the signer certificate.
 *
 * @return uint8_t *                : Signer certificate on success; NULL otherwise.
 *
 */
uint8_t * cy_awsport_get_ota_signer_certificate( cy_ota_type_t ota_type,
                                                 const uint8_t * const pucCertName,
                                                 uint32_t * const ota_signer_cert_size);

/**
 * Remove boot magic code from image in external flash.
 *
 * @return cy_rslt_t                : CY_RSLT_SUCCESS on success; error codes otherwise.
 */
cy_rslt_t cy_awsport_ota_remove_boot_magic(void);

#endif /* ifndef CY_OTA_STORAGE_H_ */
