/*******************************************************************************
 * File Name: cycfg_connectivity_wifi.c
 *
 * Description:
 * Connectivity Wi-Fi configuration
 * This file was automatically generated and should not be modified.
 * Tools Package 2.4.0.5972
 * mtb-pdl-cat1 2.4.1.17937
 * personalities 6.0.0.0
 * udd 3.0.0.2024
 *
 ********************************************************************************
 * Copyright 2022 Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.
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
 ********************************************************************************/

#include "cycfg_connectivity_wifi.h"

#define CYCFG_TKO_OL_ENABLED (1u)

static tko_ol_t tko_ol_0_ctxt;
static const cy_tko_ol_cfg_t cy_tko_ol_cfg_0 =
{
    .interval        = 5,
    .retry_interval  = 3,
    .retry_count     = 3,
    .ports[0] =
    {
        .local_port  = 3353,
        .remote_port = 3360,
        "192.168.43.228"
    },
    .ports[1] =
    {
        .local_port  = 0,
        .remote_port = 0,
        "0.0.0.0"
    },
    .ports[2] =
    {
        .local_port  = 0,
        .remote_port = 0,
        "0.0.0.0"
    },
    .ports[3] =
    {
        .local_port  = 0,
        .remote_port = 0,
        "0.0.0.0"
    },
};
static const ol_desc_t ol_list_0[] =
{
    [0u] = { "TKO", &cy_tko_ol_cfg_0, &tko_ol_fns, &tko_ol_0_ctxt },
    [1u] = { NULL, NULL,  NULL,        NULL           },
};

/***************************************************************************************************
 * cycfg_get_default_ol_list
 **************************************************************************************************/
const ol_desc_t* cycfg_get_default_ol_list(void)
{
    return &ol_list_0[0];
}
