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

/** @file
 *  Header for whd_driver structure
 */

#ifndef INCLUDED_WHD_INT_H
#define INCLUDED_WHD_INT_H

#include "whd_thread.h"
#ifndef PROTO_MSGBUF
#include "whd_sdpcm.h"
#include "whd_cdc_bdc.h"
#else
#include "whd_msgbuf.h"
#endif /* PROTO_MSGBUF */
#include "whd_chip.h"
#include "whd_ap.h"
#include "whd_debug.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* Forward declarations */
struct whd_proto;   /* device communication protocol info */
struct whd_ram_shared_info;   /* device ram shared info */
struct whd_msgbuf;   /* device msg buffer info */

typedef struct
{
    uint32_t tx_total; /* Total number of TX packets sent from WHD */
    uint32_t rx_total; /* Total number of RX packets received at WHD */
    uint32_t tx_no_mem; /* Number of times WHD could not send due to no buffer */
    uint32_t rx_no_mem; /* Number of times WHD could not receive due to no buffer */
    uint32_t tx_fail; /* Number of times TX packet failed */
    uint32_t no_credit; /* Number of times WHD could not send due to no credit */
    uint32_t flow_control; /* Number of times WHD Flow control is enabled */
    uint32_t internal_host_buffer_fail_with_timeout; /* Internal host buffer get failed after timeout */
} whd_stats_t;

#define WHD_INTERFACE_MAX 3
typedef enum
{
    WHD_INVALID_ROLE           = 0,
    WHD_STA_ROLE               = 1,         /**< STA or Client Interface     */
    WHD_AP_ROLE                = 2,         /**< softAP Interface  */
    WHD_P2P_ROLE               = 3,         /**< P2P Interface  */
} whd_interface_role_t;

/* The level of bus communication with the dongle */
typedef enum
{
    WHD_PROTO_BCDC,
    WHD_PROTO_MSGBUF,
} whd_bus_protocol_type_t;

struct whd_interface
{
    whd_driver_t whd_driver;
    uint8_t ifidx;
    uint8_t bsscfgidx;

    char if_name[WHD_MSG_IFNAME_MAX];
    whd_interface_role_t role;
    whd_mac_t mac_addr;
    uint8_t event_reg_list[WHD_EVENT_ENTRY_MAX];
    whd_bool_t state;
#if defined(WHD_CSI_SUPPORT)
    struct whd_csi_info *csi_info;
#endif /* defined(WHD_CSI_SUPPORT) */
};
struct whd_bt_dev
{
    void     *bt_data;
    void (*bt_int_cb)(void *data);
    uint32_t bt_use_count;
    whd_bool_t intr;
};

struct whd_bt_info
{
    uint32_t bt_buf_reg_addr;
    uint32_t host_ctrl_reg_addr;
    uint32_t bt_ctrl_reg_addr;
    uint32_t wlan_buf_addr;
};

struct whd_driver
{
    whd_interface_t iflist[WHD_INTERFACE_MAX];
    uint8_t if2ifp[WHD_INTERFACE_MAX];

    /* Bus variables */
    struct whd_bus_info *bus_if;
    struct whd_bus_priv *bus_priv;
    struct whd_bus_common_info *bus_common_info;
    struct whd_proto *proto;
    whd_bt_dev_t bt_dev;

    whd_buffer_funcs_t *buffer_if;
    whd_netif_funcs_t *network_if;
    whd_resource_source_t *resource_if;
    uint8_t *aligned_addr;

    whd_bool_t bus_gspi_32bit;

    whd_bus_protocol_type_t proto_type;
    whd_thread_info_t thread_info;
    whd_error_info_t error_info;
#ifndef PROTO_MSGBUF
    whd_sdpcm_info_t sdpcm_info;
#endif /* PROTO_MSGBUF */
    whd_internal_info_t internal_info;
    whd_ap_int_info_t ap_info;
    whd_chip_info_t chip_info;

    whd_stats_t whd_stats;
    whd_country_code_t country;

    whd_ioctl_log_t whd_ioctl_log[WHD_IOCTL_LOG_SIZE];
    int whd_ioctl_log_index;
    cy_semaphore_t whd_log_mutex;

    struct whd_ram_shared_info *ram_shared;
    struct whd_msgbuf *msgbuf;
    uint16_t (*read_ptr)(struct whd_driver *whd_driver, uint32_t mem_offset);
    void (*write_ptr)(struct whd_driver *whd_driver, uint32_t mem_offset, uint16_t value);
    cy_semaphore_t host_suspend_mutex;
    uint8_t ack_d2h_suspend; /* Flag to check D3 wake from CM33(Host) */
    uint8_t dma_index_sz;
#ifdef CYCFG_ULP_SUPPORT_ENABLED
    uint32_t ds_exit_in_progress;
    deepsleep_cb_info_t ds_cb_info;
#endif

#ifdef PROTO_MSGBUF
    cy_timer_t rxbuf_update_timer;
    bool update_buffs;
    bool force_rx_read;
#endif

#if defined(COMPONENT_CAT5) && !defined(WHD_DISABLE_PDS)
    /* Callback to be registered to the syspm module for Low Power */
    cyhal_syspm_callback_data_t whd_syspm_cb_data;
    bool pds_sleep_allow;
    uint32_t lock_sleep;
    cy_mutex_t sleep_mutex;
#endif /* defined(COMPONENT_CAT5) && !defined(WHD_DISABLE_PDS) */

};

whd_result_t whd_add_interface(whd_driver_t whd_driver, uint8_t bsscfgidx, uint8_t ifidx,
                               const char *name, whd_mac_t *mac_addr,  whd_interface_t *ifpp);

whd_result_t whd_add_primary_interface(whd_driver_t whd_driver, whd_interface_t *ifpp);

whd_interface_t whd_get_primary_interface(whd_driver_t whd_driver);

whd_interface_t whd_get_interface(whd_driver_t whd_driver, uint8_t ifidx);

#ifdef __cplusplus
} /* extern "C" */
#endif
#endif /* INCLUDED_WHD_INT_H */
