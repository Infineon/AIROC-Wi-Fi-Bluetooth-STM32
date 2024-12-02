/*
 * Copyright (c) 2019, Cypress Semiconductor Corporation, All Rights Reserved
 * SPDX-License-Identifier: LicenseRef-PBL
 *
 * This file and the related binary are licensed under the
 * Permissive Binary License, Version 1.0 (the "License");
 * you may not use these files except in compliance with the License.
 *
 * You may obtain a copy of the License here:
 * LICENSE-permissive-binary-license-1.0.txt and at
 * https://www.mbed.com/licenses/PBL-1.0
 *
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "wiced_resource.h"

#ifndef WLAN_MFG_FIRMWARE
#if defined(CY_STORAGE_WIFI_DATA)
CY_SECTION_WHD(CY_STORAGE_WIFI_DATA) __attribute__((used))
#endif
const unsigned char wifi_firmware_clm_blob_data[1040] = {
        66, 76, 79, 66, 60, 0, 0, 0, 15, 77, 97, 178, 1, 0, 0, 0, 2, 0, 0, 0,
        0, 0, 0, 0, 60, 0, 0, 0, 208, 3, 0, 0, 183, 154, 30, 166, 0, 0, 0, 0,
        0, 0, 0, 0, 12, 4, 0, 0, 4, 0, 0, 0, 247, 164, 240, 22, 0, 0, 0, 0,
        67, 76, 77, 32, 68, 65, 84, 65, 0, 0, 18, 0, 2, 0, 77, 117, 114, 97,
        116, 97, 50, 71, 70, 46, 79, 117, 116, 100, 111, 111, 114, 83, 84, 65,
        49, 46, 51, 54, 46, 49, 0, 0, 0, 0, 0, 0, 0, 0, 84, 2, 0, 0, 67, 108,
        109, 73, 109, 112, 111, 114, 116, 58, 32, 49, 46, 52, 55, 46, 49, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 118, 49, 32, 50, 52, 47, 48, 52,
        47, 49, 57, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 16, 1, 0,
        0, 10, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 12, 0, 1, 2, 3, 4, 5,
        6, 7, 8, 9, 10, 11, 4, 0, 1, 2, 3, 8, 4, 5, 6, 7, 8, 9, 10, 11, 1, 1,
        1, 11, 1, 13, 2, 2, 2, 3, 2, 10, 3, 3, 4, 4, 4, 8, 5, 8, 9, 9, 10, 10,
        11, 11, 36, 36, 36, 144, 36, 165, 40, 60, 56, 60, 56, 116, 64, 64, 100,
        100, 100, 140, 100, 144, 104, 116, 104, 136, 132, 136, 132, 144, 132,
        165, 140, 140, 144, 161, 149, 165, 165, 165, 0, 1, 1, 1, 2, 1, 15, 2,
        18, 27, 2, 21, 30, 1, 22, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 36, 64, 4, 100, 144, 4, 149, 165, 4, 50, 48, 50, 52,
        45, 48, 52, 45, 50, 50, 32, 48, 49, 58, 48, 57, 58, 51, 51, 0, 0, 8,
        70, 17, 0, 58, 19, 0, 46, 20, 0, 70, 23, 0, 70, 25, 0, 54, 28, 0, 70,
        29, 0, 54, 31, 0, 0, 2, 64, 21, 0, 50, 30, 0, 0, 1, 70, 22, 0, 0, 8,
        58, 13, 0, 70, 16, 0, 58, 19, 0, 46, 20, 0, 70, 24, 0, 54, 28, 0, 70,
        29, 0, 54, 31, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 255, 1, 30, 1,
        0, 9, 66, 0, 2, 54, 0, 3, 70, 4, 3, 74, 5, 2, 74, 8, 3, 70, 10, 3, 66,
        11, 3, 66, 12, 2, 58, 12, 3, 2, 0, 255, 1, 20, 2, 0, 2, 68, 2, 2, 74,
        2, 3, 0, 0, 255, 0, 0, 0, 2, 0, 255, 1, 23, 2, 0, 1, 74, 2, 1, 1, 0,
        255, 1, 30, 1, 0, 9, 66, 0, 2, 54, 0, 3, 70, 4, 3, 74, 5, 2, 74, 8,
        3, 70, 10, 3, 66, 11, 3, 66, 12, 2, 58, 12, 3, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8, 50,
        0, 0, 58, 3, 0, 66, 6, 0, 70, 7, 0, 74, 9, 0, 70, 10, 0, 66, 11, 0,
        50, 12, 0, 0, 1, 74, 2, 0, 0, 1, 74, 2, 0, 0, 8, 50, 0, 0, 58, 3, 0,
        66, 6, 0, 70, 7, 0, 74, 9, 0, 70, 10, 0, 66, 11, 0, 50, 12, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 10, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21,
        8, 4, 5, 6, 7, 8, 9, 10, 11, 20, 3, 0, 0, 128, 0, 0, 0, 174, 0, 0, 0,
        0, 0, 0, 0, 238, 0, 0, 0, 64, 2, 0, 0, 120, 1, 0, 0, 76, 3, 0, 0, 248,
        1, 0, 0, 45, 1, 0, 0, 28, 3, 0, 0, 192, 3, 0, 0, 180, 3, 0, 0, 255,
        80, 129, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 136, 0, 0, 0, 0, 0, 0, 0, 25,
        1, 0, 0, 120, 0, 0, 0, 56, 2, 0, 0, 232, 1, 0, 0, 104, 0, 0, 0, 0, 1,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 112, 0, 0, 0, 8, 1, 0,
        0, 240, 1, 0, 0, 224, 1, 0, 0, 112, 1, 0, 0, 200, 3, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 188, 3, 0, 0, 5, 0, 0, 0, 36,
        3, 0, 0, 35, 110, 0, 2, 0, 255, 255, 0, 67, 65, 0, 0, 1, 0, 0, 0, 68,
        69, 0, 1, 2, 1, 1, 32, 74, 80, 0, 3, 3, 2, 2, 0, 85, 83, 0, 4, 4, 3,
        3, 0, 0, 0, 255, 0, 0, 0, 4, 2, 255, 3, 24, 18, 24, 26, 30, 30, 0, 8,
        70, 17, 1, 58, 19, 1, 50, 20, 1, 70, 23, 1, 70, 25, 1, 58, 28, 1, 70,
        29, 1, 62, 31, 1, 5, 1, 255, 2, 30, 21, 14, 30, 0, 2, 68, 21, 1, 50,
        30, 1, 6, 2, 255, 1, 30, 22, 0, 1, 70, 22, 1, 3, 2, 255, 2, 24, 14,
        30, 30, 0, 8, 58, 13, 1, 70, 16, 1, 58, 19, 1, 50, 20, 1, 70, 24, 1,
        58, 28, 1, 70, 29, 1, 62, 31, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 13, 1,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 14, 168, 0, 0
};
const resource_hnd_t wifi_firmware_clm_blob = { RESOURCE_IN_MEMORY, 1040, {.mem = { (const char *) wifi_firmware_clm_blob_data }}};
#endif /* !WLAN_MFG_FIRMWARE */
