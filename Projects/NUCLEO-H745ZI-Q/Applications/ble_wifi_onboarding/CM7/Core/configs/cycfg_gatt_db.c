/***************************************************************************//**
* File Name: cycfg_gatt_db.c
*
* Description:
* BLE device's GATT database and device configuration.
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

#include "cycfg_gatt_db.h"
#include "wiced_bt_uuid.h"
#include "wiced_bt_gatt.h"

/*************************************************************************************
* GATT server definitions
*************************************************************************************/

const uint8_t gatt_database[] = 
{
    /* Primary Service: Generic Access */
    PRIMARY_SERVICE_UUID16 (HDLS_GAP, __UUID_SERVICE_GENERIC_ACCESS),
        /* Characteristic: Device Name */
        CHARACTERISTIC_UUID16 (HDLC_GAP_DEVICE_NAME, HDLC_GAP_DEVICE_NAME_VALUE, __UUID_CHARACTERISTIC_DEVICE_NAME, GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE),
        /* Characteristic: Appearance */
        CHARACTERISTIC_UUID16 (HDLC_GAP_APPEARANCE, HDLC_GAP_APPEARANCE_VALUE, __UUID_CHARACTERISTIC_APPEARANCE, GATTDB_CHAR_PROP_READ, GATTDB_PERM_READABLE),

    /* Primary Service: Generic Attribute */
    PRIMARY_SERVICE_UUID16 (HDLS_GATT, __UUID_SERVICE_GENERIC_ATTRIBUTE),

    /* Primary Service: Custom Service */
    PRIMARY_SERVICE_UUID128 (HDLS_CUSTOM_SERVICE, __UUID_SERVICE_CUSTOM_SERVICE),
        /* Characteristic: WIFI SSID */
        CHARACTERISTIC_UUID128_WRITABLE (HDLC_CUSTOM_SERVICE_WIFI_SSID, HDLC_CUSTOM_SERVICE_WIFI_SSID_VALUE, __UUID_CHARACTERISTIC_CUSTOM_SERVICE_WIFI_SSID, GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_WRITE, GATTDB_PERM_VARIABLE_LENGTH | GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ),
            /* Descriptor: Characteristic User Description */
            CHAR_DESCRIPTOR_UUID16 (HDLD_CUSTOM_SERVICE_WIFI_SSID_CHAR_USER_DESCRIPTION, __UUID_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION, GATTDB_PERM_READABLE),
        /* Characteristic: WIFI PASSWORD */
        CHARACTERISTIC_UUID128_WRITABLE (HDLC_CUSTOM_SERVICE_WIFI_PASSWORD, HDLC_CUSTOM_SERVICE_WIFI_PASSWORD_VALUE, __UUID_CHARACTERISTIC_CUSTOM_SERVICE_WIFI_PASSWORD, GATTDB_CHAR_PROP_WRITE, GATTDB_PERM_VARIABLE_LENGTH | GATTDB_PERM_WRITE_REQ | GATTDB_PERM_AUTH_WRITABLE),
            /* Descriptor: Characteristic User Description */
            CHAR_DESCRIPTOR_UUID16 (HDLD_CUSTOM_SERVICE_WIFI_PASSWORD_CHAR_USER_DESCRIPTION, __UUID_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION, GATTDB_PERM_READABLE),
        /* Characteristic: WIFI CONNECTION */
        CHARACTERISTIC_UUID128_WRITABLE (HDLC_CUSTOM_SERVICE_WIFI_CONNECTION, HDLC_CUSTOM_SERVICE_WIFI_CONNECTION_VALUE, __UUID_CHARACTERISTIC_CUSTOM_SERVICE_WIFI_CONNECTION, GATTDB_CHAR_PROP_READ | GATTDB_CHAR_PROP_WRITE | GATTDB_CHAR_PROP_NOTIFY, GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ),
            /* Descriptor: Client Characteristic Configuration */
            CHAR_DESCRIPTOR_UUID16_WRITABLE (HDLD_CUSTOM_SERVICE_WIFI_CONNECTION_CLIENT_CHAR_CONFIG, __UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION, GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ),
            /* Descriptor: Characteristic User Description */
            CHAR_DESCRIPTOR_UUID16 (HDLD_CUSTOM_SERVICE_WIFI_CONNECTION_CHAR_USER_DESCRIPTION, __UUID_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION, GATTDB_PERM_READABLE),
};

/* Length of the GATT database */
const uint16_t gatt_database_len = sizeof(gatt_database);

/*************************************************************************************
 * GATT Initial Value Arrays
 ************************************************************************************/
 
uint8_t app_gap_device_name[]                                      = {'b', 'l', 'e', 'P', 'r', 'o', 'v', '\0', };
uint8_t app_gap_appearance[]                                       = {0x00, 0x00, };
uint8_t app_custom_service_wifi_ssid[]                             = {'\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', };
uint8_t app_custom_service_wifi_ssid_char_user_description[]       = {'W', 'i', 'F', 'i', ' ', 'S', 'S', 'I', 'D', };
uint8_t app_custom_service_wifi_password[]                         = {'\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', };
uint8_t app_custom_service_wifi_password_char_user_description[]   = {'W', 'i', 'F', 'i', ' ', 'P', 'a', 's', 's', 'w', 'o', 'r', 'd', };
uint8_t app_custom_service_wifi_connection[]                       = {0x00, };
uint8_t app_custom_service_wifi_connection_client_char_config[]    = {0x00, 0x00, };
uint8_t app_custom_service_wifi_connection_char_user_description[] = {'W', 'i', 'F', 'i', ' ', 'C', 'o', 'n', 'n', 'e', 'c', 't', 'i', 'o', 'n', ' ', 'C', 'o', 'n', 't', 'r', 'o', 'l', };
 
 /************************************************************************************
 * GATT Lookup Table
 ************************************************************************************/
 
gatt_db_lookup_table_t app_gatt_db_ext_attr_tbl[] =
{
    /* { attribute handle,                                       maxlen, curlen, attribute data } */
    { HDLC_GAP_DEVICE_NAME_VALUE,                                8,      8,      app_gap_device_name },
    { HDLC_GAP_APPEARANCE_VALUE,                                 2,      2,      app_gap_appearance },
    { HDLC_CUSTOM_SERVICE_WIFI_SSID_VALUE,                       32,     32,     app_custom_service_wifi_ssid },
    { HDLD_CUSTOM_SERVICE_WIFI_SSID_CHAR_USER_DESCRIPTION,       9,      9,      app_custom_service_wifi_ssid_char_user_description },
    { HDLC_CUSTOM_SERVICE_WIFI_PASSWORD_VALUE,                   63,     63,     app_custom_service_wifi_password },
    { HDLD_CUSTOM_SERVICE_WIFI_PASSWORD_CHAR_USER_DESCRIPTION,   13,     13,     app_custom_service_wifi_password_char_user_description },
    { HDLC_CUSTOM_SERVICE_WIFI_CONNECTION_VALUE,                 1,      1,      app_custom_service_wifi_connection },
    { HDLD_CUSTOM_SERVICE_WIFI_CONNECTION_CLIENT_CHAR_CONFIG,    2,      2,      app_custom_service_wifi_connection_client_char_config },
    { HDLD_CUSTOM_SERVICE_WIFI_CONNECTION_CHAR_USER_DESCRIPTION, 23,     23,     app_custom_service_wifi_connection_char_user_description },
};

/* Number of Lookup Table entries */
const uint16_t app_gatt_db_ext_attr_tbl_size = (sizeof(app_gatt_db_ext_attr_tbl) / sizeof(gatt_db_lookup_table_t));

/* Number of GATT initial value arrays entries */
const uint16_t app_gap_device_name_len = (sizeof(app_gap_device_name));
const uint16_t app_gap_appearance_len = (sizeof(app_gap_appearance));
const uint16_t app_custom_service_wifi_ssid_len = (sizeof(app_custom_service_wifi_ssid));
const uint16_t app_custom_service_wifi_ssid_char_user_description_len = (sizeof(app_custom_service_wifi_ssid_char_user_description));
const uint16_t app_custom_service_wifi_password_len = (sizeof(app_custom_service_wifi_password));
const uint16_t app_custom_service_wifi_password_char_user_description_len = (sizeof(app_custom_service_wifi_password_char_user_description));
const uint16_t app_custom_service_wifi_connection_len = (sizeof(app_custom_service_wifi_connection));
const uint16_t app_custom_service_wifi_connection_client_char_config_len = (sizeof(app_custom_service_wifi_connection_client_char_config));
const uint16_t app_custom_service_wifi_connection_char_user_description_len = (sizeof(app_custom_service_wifi_connection_char_user_description));

