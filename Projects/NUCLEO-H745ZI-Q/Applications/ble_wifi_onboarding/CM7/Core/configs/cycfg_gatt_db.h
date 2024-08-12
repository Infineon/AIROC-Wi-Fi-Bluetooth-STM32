/***************************************************************************//**
* File Name: cycfg_gatt_db.h
*
* Description:
* Definitions for constants used in the device's GATT database and function
* prototypes.
*
********************************************************************************
* Copyright 2021 Cypress Semiconductor Corporation
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#if !defined(CYCFG_GATT_DB_H)
#define CYCFG_GATT_DB_H

#include "stdint.h"

#define __UUID_SERVICE_GENERIC_ACCESS                                0x1800
#define __UUID_CHARACTERISTIC_DEVICE_NAME                            0x2A00
#define __UUID_CHARACTERISTIC_APPEARANCE                             0x2A01
#define __UUID_SERVICE_GENERIC_ATTRIBUTE                             0x1801
#define __UUID_SERVICE_CUSTOM_SERVICE                                0x5B, 0x19, 0xBA, 0xE4, 0xE4, 0x52, 0xA9, 0x96, 0xF1, 0x4A, 0x84, 0xC8, 0x09, 0x4D, 0xC0, 0x21
#define __UUID_CHARACTERISTIC_CUSTOM_SERVICE_WIFI_SSID               0x63, 0x97, 0xCA, 0x92, 0x5F, 0x02, 0x1E, 0xB9, 0x3D, 0x4A, 0x31, 0x6B, 0x43, 0x00, 0x50, 0x1E
#define __UUID_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION            0x2901
#define __UUID_CHARACTERISTIC_CUSTOM_SERVICE_WIFI_PASSWORD           0x64, 0x97, 0xCA, 0x92, 0x5F, 0x02, 0x1E, 0xB9, 0x3D, 0x4A, 0x31, 0x6B, 0x43, 0x00, 0x50, 0x1E
#define __UUID_CHARACTERISTIC_CUSTOM_SERVICE_WIFI_CONNECTION         0x65, 0x97, 0xCA, 0x92, 0x5F, 0x02, 0x1E, 0xB9, 0x3D, 0x4A, 0x31, 0x6B, 0x43, 0x00, 0x50, 0x1E
#define __UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION        0x2902

/* Service Generic Access */
#define HDLS_GAP                                                     0x01
/* Characteristic Device Name */
#define HDLC_GAP_DEVICE_NAME                                         0x02
#define HDLC_GAP_DEVICE_NAME_VALUE                                   0x03
/* Characteristic Appearance */
#define HDLC_GAP_APPEARANCE                                          0x04
#define HDLC_GAP_APPEARANCE_VALUE                                    0x05

/* Service Generic Attribute */
#define HDLS_GATT                                                    0x06

/* Service Custom Service */
#define HDLS_CUSTOM_SERVICE                                          0x07
/* Characteristic WIFI SSID */
#define HDLC_CUSTOM_SERVICE_WIFI_SSID                                0x08
#define HDLC_CUSTOM_SERVICE_WIFI_SSID_VALUE                          0x09
/* Descriptor Characteristic User Description */
#define HDLD_CUSTOM_SERVICE_WIFI_SSID_CHAR_USER_DESCRIPTION          0x0A
/* Characteristic WIFI PASSWORD */
#define HDLC_CUSTOM_SERVICE_WIFI_PASSWORD                            0x0B
#define HDLC_CUSTOM_SERVICE_WIFI_PASSWORD_VALUE                      0x0C
/* Descriptor Characteristic User Description */
#define HDLD_CUSTOM_SERVICE_WIFI_PASSWORD_CHAR_USER_DESCRIPTION      0x0D
/* Characteristic WIFI CONNECTION */
#define HDLC_CUSTOM_SERVICE_WIFI_CONNECTION                          0x0E
#define HDLC_CUSTOM_SERVICE_WIFI_CONNECTION_VALUE                    0x0F
/* Descriptor Client Characteristic Configuration */
#define HDLD_CUSTOM_SERVICE_WIFI_CONNECTION_CLIENT_CHAR_CONFIG       0x10
/* Descriptor Characteristic User Description */
#define HDLD_CUSTOM_SERVICE_WIFI_CONNECTION_CHAR_USER_DESCRIPTION    0x11

/* External Lookup Table Entry */
typedef struct
{
    uint16_t handle;
    uint16_t max_len;
    uint16_t cur_len;
    uint8_t  *p_data;
} gatt_db_lookup_table_t;

/* External definitions */
extern const uint8_t  gatt_database[];
extern const uint16_t gatt_database_len;
extern gatt_db_lookup_table_t app_gatt_db_ext_attr_tbl[];
extern const uint16_t app_gatt_db_ext_attr_tbl_size;
extern uint8_t app_gap_device_name[];
extern const uint16_t app_gap_device_name_len;
extern uint8_t app_gap_appearance[];
extern const uint16_t app_gap_appearance_len;
extern uint8_t app_custom_service_wifi_ssid[];
extern const uint16_t app_custom_service_wifi_ssid_len;
extern uint8_t app_custom_service_wifi_ssid_char_user_description[];
extern const uint16_t app_custom_service_wifi_ssid_char_user_description_len;
extern uint8_t app_custom_service_wifi_password[];
extern const uint16_t app_custom_service_wifi_password_len;
extern uint8_t app_custom_service_wifi_password_char_user_description[];
extern const uint16_t app_custom_service_wifi_password_char_user_description_len;
extern uint8_t app_custom_service_wifi_connection[];
extern const uint16_t app_custom_service_wifi_connection_len;
extern uint8_t app_custom_service_wifi_connection_client_char_config[];
extern const uint16_t app_custom_service_wifi_connection_client_char_config_len;
extern uint8_t app_custom_service_wifi_connection_char_user_description[];
extern const uint16_t app_custom_service_wifi_connection_char_user_description_len;

#endif /* CYCFG_GATT_DB_H */

