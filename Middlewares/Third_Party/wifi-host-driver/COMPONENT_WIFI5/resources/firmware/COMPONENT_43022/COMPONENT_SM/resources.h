/*
 * Copyright 2024, Cypress Semiconductor Corporation (an Infineon company)
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
 */
#ifndef INCLUDED_RESOURCES_H_
#define INCLUDED_RESOURCES_H_
#include "wiced_resource.h"

#ifndef WLAN_MFG_FIRMWARE
extern const resource_hnd_t wifi_firmware_image;
extern const unsigned char wifi_firmware_image_data[463372];
#endif /* !WLAN_MFG_FIRMWARE */
#ifdef WLAN_MFG_FIRMWARE
extern const resource_hnd_t wifi_mfg_firmware_image;
extern const unsigned char wifi_mfg_firmware_image_data[482348];
#endif /* WLAN_MFG_FIRMWARE */

#endif /* ifndef INCLUDED_RESOURCES_H_ */