/***************************************************************************//**
* File Name: cycfg_gap.c
*
* Description:
* BLE device's GAP configuration.
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

#include "cycfg_gap.h"

/* Device address */
const wiced_bt_device_address_t cy_bt_device_address = {0x00, 0xA0, 0x50, 0x00, 0x00, 0x00};

const uint8_t cy_bt_adv_packet_elem_0[1] = { 0x06 };
const uint8_t cy_bt_adv_packet_elem_1[7] = { 0x62, 0x6C, 0x65, 0x50, 0x72, 0x6F, 0x76 };
const uint8_t cy_bt_adv_packet_elem_2[16] = { 0x5B, 0x19, 0xBA, 0xE4, 0xE4, 0x52, 0xA9, 0x96, 0xF1, 0x4A, 0x84, 0xC8, 0x09, 0x4D, 0xC0, 0x21 };
wiced_bt_ble_advert_elem_t cy_bt_adv_packet_data[] = 
{
    /* Flags */
    {
        .advert_type = BTM_BLE_ADVERT_TYPE_FLAG, 
        .len = 1, 
        .p_data = (uint8_t*)cy_bt_adv_packet_elem_0, 
    },
    /* Complete local name */
    {
        .advert_type = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE, 
        .len = 7, 
        .p_data = (uint8_t*)cy_bt_adv_packet_elem_1, 
    },
    /* Complete list of 128-bit UUIDs available */
    {
        .advert_type = BTM_BLE_ADVERT_TYPE_128SRV_COMPLETE, 
        .len = 16, 
        .p_data = (uint8_t*)cy_bt_adv_packet_elem_2, 
    },
};

