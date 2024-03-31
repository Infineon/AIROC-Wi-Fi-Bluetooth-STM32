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
 * @file ota_config.h
 * @brief OTA user configurable settings.
 */

#ifndef OTA_CONFIG_H_
#define OTA_CONFIG_H_

/**
 * @brief Log base 2 of the size of the file data block message (excluding the
 * header).
 *
 * <b>Possible values:</b> Any unsigned 32 integer. <br>
 * <b>Default value:</b> '12'
 */
#define otaconfigLOG2_FILE_BLOCK_SIZE    12UL


/**
 * @brief Size of the file data block message (excluding the header).
 *
 */
#define otaconfigFILE_BLOCK_SIZE                ( 1UL << otaconfigLOG2_FILE_BLOCK_SIZE )

/**
 * @brief Milliseconds to wait for the self test phase to succeed before we
 * force reset.
 *
 * <b>Possible values:</b> Any unsigned 32 integer. <br>
 * <b>Default value:</b> '16000'
 */
#define otaconfigSELF_TEST_RESPONSE_WAIT_MS     16000U

/**
 * @brief Milliseconds to wait before requesting data blocks from the OTA
 * service if nothing is happening.
 *
 * @note The wait timer is reset whenever a data block is received from the OTA
 * service so we will only send the request message after being idle for this
 * amount of time.
 *
 * <b>Possible values:</b> Any unsigned 32 integer. <br>
 * <b>Default value:</b> '10000'
 */
#define otaconfigFILE_REQUEST_WAIT_MS           10000U

/**
 * @brief The maximum allowed length of the thing name used by the OTA agent.
 *
 * @note AWS IoT requires Thing names to be unique for each device that
 * connects to the broker. Likewise, the OTA agent requires the developer to
 * construct and pass in the Thing name when initializing the OTA agent. The
 * agent uses this size to allocate static storage for the Thing name used in
 * all OTA base topics. Namely $aws/things/thingName
 *
 * <b>Possible values:</b> Any unsigned 32 integer. <br>
 * <b>Default value:</b> '64'
 */
#define otaconfigMAX_THINGNAME_LEN              64U

/**
 * @brief The maximum number of data blocks requested from OTA streaming
 * service.
 *
 * @note This configuration parameter is sent with data requests and represents
 * the maximum number of data blocks the service will send in response. The
 * maximum limit for this must be calculated from the maximum data response
 * limit (128 KB from service) divided by the block size. For example if block
 * size is set as 1 KB then the maximum number of data blocks that we can
 * request is 128/1 = 128 blocks. Configure this parameter to this maximum
 * limit or lower based on how many data blocks response is expected for each
 * data requests.
 *
 * <b>Possible values:</b> Any unsigned 32 integer value greater than 0. <br>
 * <b>Default value:</b> '1'
 */
#define otaconfigMAX_NUM_BLOCKS_REQUEST         8U

/**
 * @brief The maximum number of requests allowed to send without a response
 * before we abort.
 *
 * @note This configuration parameter sets the maximum number of times the
 * requests are made over the selected communication channel before aborting
 * and returning error.
 *
 * <b>Possible values:</b> Any unsigned 32 integer. <br>
 * <b>Default value:</b> '32'
 */
#define otaconfigMAX_NUM_REQUEST_MOMENTUM       32U

/**
 * @brief How frequently the device will report its OTA progress to the cloud.
 *
 * @note Device will update the job status with the number of blocks it has received every certain
 * number of blocks it receives. For example, 64 means device will update job status every 64 blocks
 * it receives.
 *
 * <b>Possible values:</b> Any unsigned 32 integer. <br>
 * <b>Default value:</b> '64'
 */
#define otaconfigOTA_UPDATE_STATUS_FREQUENCY    64U

/**
 * @brief The number of data buffers reserved by the OTA agent.
 *
 * @note This configurations parameter sets the maximum number of static data
 * buffers used by the OTA agent for job and file data blocks received.
 *
 * <b>Possible values:</b> Any unsigned 32 integer. <br>
 * <b>Default value:</b> '1'
 */
#define otaconfigMAX_NUM_OTA_DATA_BUFFERS       10U

/**
 * @brief Flag to enable booting into updates that have an identical or lower
 * version than the current version.
 *
 * @note Set this configuration parameter to '1' to disable version checks.
 * This allows updates to an identical or lower version. This is provided for
 * testing purpose and it's recommended to always update to higher version and
 * keep this configuration disabled.
 *
 * <b>Possible values:</b> Any unsigned 32 integer. <br>
 * <b>Default value:</b> '0'
 */
#ifndef otaconfigAllowDowngrade
    #define otaconfigAllowDowngrade    0U
#endif

/**
 * @brief The file type id received in the job document.
 *
 * @note The file type id received in the job document that allows devices
 * to identify the type of file received from the cloud. This configuration
 * defines the file type id used for firmware updates. If this is changed
 * then the updated value must be used while creating firmware update jobs.
 *
 */
#define configOTA_FIRMWARE_UPDATE_FILE_TYPE_ID    101U

/**
 * @brief The protocol selected for OTA control operations.
 *
 * @note This configurations parameter sets the default protocol for all the
 * OTA control operations like requesting OTA job, updating the job status etc.
 *
 * @note Only MQTT is supported at this time for control operations.
 *
 * <b>Possible values:</b> OTA_CONTROL_OVER_MQTT <br>
 * <b>Default value:</b> 'OTA_CONTROL_OVER_MQTT'
 */
#define configENABLED_CONTROL_PROTOCOL    ( OTA_CONTROL_OVER_MQTT )

/**
 * @brief The protocol selected for OTA data operations.
 *
 * @note This configurations parameter sets the protocols selected for the data
 * operations like requesting file blocks from the service.
 *
 * <b>Possible values:</b><br>
 * Enable data over MQTT - ( OTA_DATA_OVER_MQTT ) <br>
 * Enable data over HTTP - ( OTA_DATA_OVER_HTTP ) <br>
 * Enable data over both MQTT & HTTP - ( OTA_DATA_OVER_MQTT | OTA_DATA_OVER_HTTP ) <br>
 * <b>Default value:</b> 'OTA_DATA_OVER_MQTT'
 */
#define configENABLED_DATA_PROTOCOLS    ( OTA_DATA_OVER_MQTT )

/**
 * @brief The preferred protocol selected for OTA data operations.
 *
 * @note Primary data protocol will be the protocol used for downloading file
 * if more than one protocol is selected while creating OTA job.
 *
 * <b>Possible values:</b><br>
 * Data over MQTT - ( OTA_DATA_OVER_MQTT ) <br>
 * Data over HTTP - ( OTA_DATA_OVER_HTTP ) <br>
 * <b>Default value:</b>  'OTA_DATA_OVER_MQTT'
 */
#define configOTA_PRIMARY_DATA_PROTOCOL    ( OTA_DATA_OVER_MQTT )

/**
 * Code signing certificate is used for OTA image signing.
 * If a platform does not support a file system the signing certificate can be pasted here.
 */
#ifndef AWS_IOT_OTA_SIGNING_CERT
#define AWS_IOT_OTA_SIGNING_CERT               "Add OTA signing cert here"
#endif

#endif /* OTA_CONFIG_H_ */
