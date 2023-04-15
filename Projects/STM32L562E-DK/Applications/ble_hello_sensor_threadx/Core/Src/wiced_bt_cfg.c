/***************************************************************************************************
 * \file wiced_bt_cfg.c
 *
 * \brief
 * This file contains functions that perform unit tests on boards
 *
 ***************************************************************************************************
 * \copyright
 * Copyright 2021 Cypress Semiconductor Corporation
 * This software, including source code, documentation and related materials
 * ("Software"), is owned by Cypress Semiconductor Corporation or one of its
 * subsidiaries ("Cypress") and is protected by and subject to worldwide patent
 * protection (United States and foreign), United States copyright laws and
 * international treaty provisions. Therefore, you may use this Software only
 * as provided in the license agreement accompanying the software package from
 * which you obtained this Software ("EULA").
 *
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software source
 * code solely for use in connection with Cypress's integrated circuit products.
 * Any reproduction, modification, translation, compilation, or representation
 * of this Software except as specified above is prohibited without the express
 * written permission of Cypress.
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
 * including Cypress's product in a High Risk Product, the manufacturer of such
 * system or application assumes all risk of such use and in doing so agrees to
 * indemnify Cypress against all liability.
 **************************************************************************************************/

/** @file
 *
 * Runtime Bluetooth stack configuration parameters
 *
 */
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"

wiced_bt_cfg_ble_scan_settings_t hello_sensor_cfg_scan_settings =
{
    /**< BLE scan mode (BTM_BLE_SCAN_MODE_PASSIVE, BTM_BLE_SCAN_MODE_ACTIVE, or
         BTM_BLE_SCAN_MODE_NONE ) */
    .scan_mode                    = BTM_BLE_SCAN_MODE_ACTIVE,


    /* Advertisement scan configuration */

    /**< High duty scan interval */
    .high_duty_scan_interval      = WICED_BT_CFG_DEFAULT_HIGH_DUTY_SCAN_INTERVAL,

    /**< High duty scan window */
    .high_duty_scan_window        = WICED_BT_CFG_DEFAULT_HIGH_DUTY_SCAN_WINDOW,

    /**< High duty scan duration in seconds ( 0 for infinite ) */
    .high_duty_scan_duration      = 5,

    /**< Low duty scan interval  */
    .low_duty_scan_interval       = WICED_BT_CFG_DEFAULT_LOW_DUTY_SCAN_INTERVAL,

    /**< Low duty scan window */
    .low_duty_scan_window         = WICED_BT_CFG_DEFAULT_LOW_DUTY_SCAN_WINDOW,

    /**< Low duty scan duration in seconds ( 0 for infinite ) */
    .low_duty_scan_duration       = 5,


    /* Connection scan configuration */

    /**< High duty cycle connection scan interval */
    .high_duty_conn_scan_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_CONN_SCAN_INTERVAL,

    /**< High duty cycle connection scan window */
    .high_duty_conn_scan_window   = WICED_BT_CFG_DEFAULT_HIGH_DUTY_CONN_SCAN_WINDOW,

    /**< High duty cycle connection duration in seconds ( 0 for infinite ) */
    .high_duty_conn_duration      = 30,

    /**< Low duty cycle connection scan interval */
    .low_duty_conn_scan_interval  = WICED_BT_CFG_DEFAULT_LOW_DUTY_CONN_SCAN_INTERVAL,

    /**< Low duty cycle connection scan window */
    .low_duty_conn_scan_window    = WICED_BT_CFG_DEFAULT_LOW_DUTY_CONN_SCAN_WINDOW,

    /**< Low duty cycle connection duration in seconds ( 0 for infinite ) */
    .low_duty_conn_duration       = 30,


    /* Connection configuration */

    /**< Minimum connection interval */
    .conn_min_interval            = WICED_BT_CFG_DEFAULT_CONN_MIN_INTERVAL,

    /**< Maximum connection interval */
    .conn_max_interval            = WICED_BT_CFG_DEFAULT_CONN_MAX_INTERVAL,

    /**< Connection latency */
    .conn_latency                 = WICED_BT_CFG_DEFAULT_CONN_LATENCY,

    /**< Connection link supervision timeout */
    .conn_supervision_timeout     = WICED_BT_CFG_DEFAULT_CONN_SUPERVISION_TIMEOUT
};


const wiced_bt_cfg_ble_advert_settings_t hello_sensor_cfg_adv_settings =
{
    /**< Advertising channel map ( mask of BTM_BLE_ADVERT_CHNL_37, BTM_BLE_ADVERT_CHNL_38,
         BTM_BLE_ADVERT_CHNL_39 ) */
    .channel_map                     = BTM_BLE_ADVERT_CHNL_37 |
                                       BTM_BLE_ADVERT_CHNL_38 |
                                       BTM_BLE_ADVERT_CHNL_39,

    /**< High duty undirected connectable minimum advertising interval */
    .high_duty_min_interval          = WICED_BT_CFG_DEFAULT_HIGH_DUTY_ADV_MIN_INTERVAL,

    /**< High duty undirected connectable maximum advertising interval */
    .high_duty_max_interval          = WICED_BT_CFG_DEFAULT_HIGH_DUTY_ADV_MAX_INTERVAL,

    /**< High duty undirected connectable advertising duration in seconds ( 0 for infinite ) */
    .high_duty_duration              = 30,

    /**< Low duty undirected connectable minimum advertising interval */
    .low_duty_min_interval           = 1024,

    /**< Low duty undirected connectable maximum advertising interval */
    .low_duty_max_interval           = 1024,

    /**< Low duty undirected connectable advertising duration in seconds ( 0 for infinite ) */
    .low_duty_duration               = 60,

    /**< High duty directed connectable minimum advertising interval */
    .high_duty_directed_min_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_DIRECTED_ADV_MIN_INTERVAL,

    /**< High duty directed connectable maximum advertising interval */
    .high_duty_directed_max_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_DIRECTED_ADV_MAX_INTERVAL,

    /**< Low duty directed connectable minimum advertising interval */
    .low_duty_directed_min_interval  = WICED_BT_CFG_DEFAULT_LOW_DUTY_DIRECTED_ADV_MIN_INTERVAL,

    /**< Low duty directed connectable maximum advertising interval */
    .low_duty_directed_max_interval  = WICED_BT_CFG_DEFAULT_LOW_DUTY_DIRECTED_ADV_MAX_INTERVAL,

    /**< Low duty directed connectable advertising duration in seconds ( 0 for infinite) */
    .low_duty_directed_duration      = 30,

    /**< High duty non-connectable minimum advertising interval */
    .high_duty_nonconn_min_interval  = WICED_BT_CFG_DEFAULT_HIGH_DUTY_NONCONN_ADV_MIN_INTERVAL,

    /**< High duty non-connectable maximum advertising interval */
    .high_duty_nonconn_max_interval  = WICED_BT_CFG_DEFAULT_HIGH_DUTY_NONCONN_ADV_MAX_INTERVAL,

    /**< High duty non-connectable advertising duration in seconds ( 0 for infinite) */
    .high_duty_nonconn_duration      = 30,

    /**< Low duty non-connectable minimum advertising interval */
    .low_duty_nonconn_min_interval   = WICED_BT_CFG_DEFAULT_LOW_DUTY_NONCONN_ADV_MIN_INTERVAL,

    /**< Low duty non-connectable maximum advertising interval */
    .low_duty_nonconn_max_interval   = WICED_BT_CFG_DEFAULT_LOW_DUTY_NONCONN_ADV_MAX_INTERVAL,

    /**< Low duty non-connectable advertising duration in seconds ( 0 for infinite) */
    .low_duty_nonconn_duration       = 0
};


wiced_bt_cfg_ble_t hello_sensor_cfg_ble =
{
    .ble_max_simultaneous_links   = 1,
    .ble_max_rx_pdu_size          = 65,

    .p_ble_scan_cfg               = &hello_sensor_cfg_scan_settings, /**< */
    .p_ble_advert_cfg             = &hello_sensor_cfg_adv_settings,
    .appearance                   = APPEARANCE_GENERIC_TAG, /**< GATT appearance (see
                                                               gatt_appearance_e) */

    /**< LE Address Resolution DB settings - effective only for pre 4.2 controller*/
    .host_addr_resolution_db_size = 5,

    /**< Interval of  random address refreshing - secs */
    .rpa_refresh_timeout          = WICED_BT_CFG_DEFAULT_RANDOM_ADDRESS_NEVER_CHANGE
};

wiced_bt_cfg_gatt_t hello_sensor_cfg_gatt =
{
    .max_db_service_modules = 0,   /**< Maximum number of service modules in the DB*/
    .max_eatt_bearers       = 0,   /**< Maximum number of allowed gatt bearers */
};

wiced_bt_cfg_settings_t wiced_bt_cfg_settings =
{
    .device_name = (uint8_t*)"hello",     /**< Local device name ( NULL terminated ) */
    .p_ble_cfg   = &hello_sensor_cfg_ble,
    .p_gatt_cfg  = &hello_sensor_cfg_gatt,
};
