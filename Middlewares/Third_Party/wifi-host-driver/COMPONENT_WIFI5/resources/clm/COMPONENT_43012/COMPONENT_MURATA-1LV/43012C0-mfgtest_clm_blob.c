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

#ifdef WLAN_MFG_FIRMWARE
#if defined(CY_STORAGE_WIFI_DATA)
CY_SECTION_WHD(CY_STORAGE_WIFI_DATA) __attribute__((used))
#endif
const unsigned char wifi_mfg_firmware_clm_blob_data[1412] = {
        66, 76, 79, 66, 60, 0, 0, 0, 51, 146, 35, 251, 1, 0, 0, 0, 2, 0, 0,
        0, 0, 0, 0, 0, 60, 0, 0, 0, 68, 5, 0, 0, 157, 16, 112, 62, 0, 0, 0,
        0, 0, 0, 0, 0, 128, 5, 0, 0, 4, 0, 0, 0, 147, 68, 77, 121, 0, 0, 0,
        0, 67, 76, 77, 32, 68, 65, 84, 65, 0, 0, 18, 0, 2, 0, 77, 117, 114,
        97, 116, 97, 46, 73, 67, 87, 45, 83, 84, 77, 51, 50, 0, 0, 0, 0, 49,
        46, 51, 54, 46, 49, 0, 0, 0, 0, 0, 0, 0, 0, 204, 2, 0, 0, 67, 108, 109,
        73, 109, 112, 111, 114, 116, 58, 32, 49, 46, 52, 55, 46, 49, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 118, 49, 32, 50, 51, 47, 49, 50, 47, 48,
        56, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
        0, 140, 2, 0, 0, 18, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17,
        18, 19, 20, 21, 38, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39,
        40, 41, 42, 43, 44, 45, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 60,
        61, 62, 63, 64, 65, 66, 67, 68, 69, 35, 97, 0, 0, 1, 0, 0, 2, 35, 110,
        0, 3, 0, 255, 255, 0, 67, 65, 0, 1, 2, 1, 1, 2, 68, 69, 0, 2, 3, 2,
        2, 33, 74, 80, 0, 4, 4, 3, 3, 33, 85, 83, 0, 5, 5, 4, 4, 2, 38, 62,
        8, 102, 142, 8, 151, 175, 8, 0, 1, 86, 1, 0, 0, 1, 74, 0, 0, 0, 1, 74,
        1, 0, 0, 1, 74, 1, 0, 0, 1, 74, 0, 0, 3, 11, 1, 1, 11, 1, 13, 36, 36,
        36, 48, 36, 64, 36, 140, 36, 144, 36, 165, 40, 60, 52, 140, 52, 144,
        64, 64, 100, 100, 100, 140, 104, 104, 108, 165, 149, 165, 0, 0, 0, 3,
        0, 0, 0, 192, 2, 0, 0, 42, 42, 42, 58, 42, 122, 42, 171, 58, 58, 106,
        106, 106, 122, 122, 138, 122, 171, 138, 138, 155, 155, 0, 0, 1, 0, 0,
        0, 172, 2, 0, 0, 1, 0, 0, 0, 48, 2, 0, 0, 2, 0, 255, 1, 30, 1, 0, 1,
        86, 1, 1, 1, 0, 255, 1, 30, 0, 0, 1, 74, 0, 1, 2, 0, 255, 1, 20, 1,
        0, 2, 68, 1, 2, 74, 1, 3, 0, 0, 255, 0, 0, 0, 2, 0, 255, 1, 23, 1, 0,
        1, 74, 1, 1, 1, 0, 255, 1, 30, 0, 0, 1, 74, 0, 1, 1, 0, 0, 0, 52, 5,
        0, 0, 1, 0, 0, 0, 55, 5, 0, 0, 10, 12, 13, 14, 15, 16, 17, 18, 19, 20,
        21, 38, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42,
        43, 44, 45, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 60, 61, 62, 63,
        64, 65, 66, 67, 68, 69, 8, 4, 5, 6, 7, 8, 9, 10, 11, 3, 2, 2, 1, 1,
        1, 1, 0, 6, 2, 5, 1, 1, 1, 4, 2, 4, 1, 1, 1, 2, 5, 4, 2, 2, 1, 1, 1,
        8, 2, 2, 1, 1, 1, 1, 10, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 12,
        0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 4, 0, 1, 2, 3, 8, 4, 5, 6, 7,
        8, 9, 10, 11, 5, 2, 2, 18, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
        16, 17, 18, 19, 20, 21, 38, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37,
        38, 39, 40, 41, 42, 43, 44, 45, 48, 49, 50, 51, 52, 53, 54, 55, 56,
        57, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 50, 48, 50, 51, 45, 49,
        50, 45, 49, 51, 32, 48, 49, 58, 51, 57, 58, 52, 48, 0, 0, 0, 0, 3, 0,
        0, 0, 226, 0, 0, 0, 1, 13, 1, 0, 2, 0, 0, 0, 76, 1, 0, 0, 42, 58, 16,
        106, 138, 16, 155, 171, 16, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 2, 2,
        0, 2, 0, 0, 0, 156, 1, 0, 0, 6, 0, 0, 0, 178, 0, 0, 0, 36, 64, 4, 100,
        144, 4, 149, 165, 4, 0, 0, 0, 112, 0, 0, 0, 44, 1, 0, 0, 7, 1, 0, 0,
        0, 0, 0, 0, 244, 3, 0, 0, 172, 1, 0, 0, 92, 1, 0, 0, 140, 3, 0, 0, 235,
        0, 0, 0, 48, 4, 0, 0, 184, 2, 0, 0, 164, 2, 0, 0, 4, 5, 0, 0, 255, 80,
        129, 5, 252, 4, 0, 0, 0, 0, 0, 0, 12, 5, 0, 0, 52, 1, 0, 0, 0, 0, 0,
        0, 120, 0, 0, 0, 51, 2, 0, 0, 0, 0, 0, 0, 10, 2, 0, 0, 0, 0, 0, 0, 109,
        2, 0, 0, 44, 5, 0, 0, 132, 2, 0, 0, 40, 4, 0, 0, 24, 4, 0, 0, 236, 3,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 220, 3, 0, 0, 32, 4, 0,
        0, 228, 3, 0, 0, 16, 4, 0, 0, 60, 5, 0, 0, 104, 0, 0, 0, 0, 0, 0, 0,
        176, 2, 0, 0, 144, 2, 0, 0, 0, 0, 255, 0, 0, 0, 5, 2, 255, 1, 30, 7,
        0, 1, 86, 7, 2, 5, 2, 255, 3, 23, 3, 24, 10, 30, 16, 0, 1, 70, 7, 2,
        4, 1, 255, 3, 23, 4, 30, 13, 14, 16, 0, 3, 70, 3, 2, 68, 9, 2, 50, 16,
        2, 3, 1, 255, 2, 23, 4, 30, 13, 0, 1, 70, 5, 2, 5, 2, 255, 2, 24, 6,
        30, 16, 0, 1, 70, 7, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 1, 5, 2, 5, 16, 1, 7, 1,
        0, 0, 0, 231, 1, 0, 0, 4, 0, 0, 0, 238, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 152,
        2, 0, 0, 4, 6, 62, 2, 0, 70, 8, 0, 62, 11, 0, 54, 12, 0, 70, 14, 0,
        86, 15, 0, 5, 1, 86, 4, 0, 8, 3, 62, 2, 0, 86, 9, 0, 32, 10, 0, 4, 1,
        70, 7, 0, 5, 8, 48, 1, 0, 58, 6, 0, 62, 7, 0, 44, 9, 0, 40, 10, 0, 62,
        12, 0, 54, 13, 0, 62, 14, 0, 8, 5, 38, 0, 0, 28, 4, 0, 36, 5, 0, 62,
        7, 0, 58, 10, 0, 4, 6, 70, 3, 0, 46, 3, 1, 60, 9, 0, 46, 9, 1, 50, 16,
        0, 20, 16, 1, 5, 4, 62, 3, 0, 56, 3, 1, 32, 15, 0, 20, 15, 1, 8, 4,
        62, 2, 0, 54, 2, 1, 32, 10, 0, 16, 10, 1, 4, 1, 70, 5, 0, 5, 3, 59,
        2, 0, 60, 8, 0, 62, 11, 0, 8, 2, 58, 1, 0, 62, 6, 0, 4, 1, 70, 7, 0,
        5, 8, 48, 1, 0, 58, 6, 0, 62, 7, 0, 44, 9, 0, 40, 10, 0, 62, 12, 0,
        54, 13, 0, 62, 14, 0, 8, 5, 38, 0, 0, 28, 4, 0, 36, 5, 0, 62, 7, 0,
        58, 10, 0, 2, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 11, 38,
        38, 38, 46, 38, 134, 38, 159, 38, 175, 46, 46, 54, 54, 54, 62, 62, 62,
        102, 102, 102, 134, 110, 126, 134, 134, 142, 159, 151, 159, 1, 0, 0,
        0, 4, 1, 0, 0, 0, 2, 2, 0, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 168,
        0, 0
};
const resource_hnd_t wifi_mfg_firmware_clm_blob = { RESOURCE_IN_MEMORY, 1412, {.mem = { (const char *) wifi_mfg_firmware_clm_blob_data }}};
#endif /* WLAN_MFG_FIRMWARE */
