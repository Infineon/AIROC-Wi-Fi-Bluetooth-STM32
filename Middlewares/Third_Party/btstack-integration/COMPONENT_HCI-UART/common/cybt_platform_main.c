/*******************************************************************************
* \file cybt_platform_main.c
*
* \brief
* This file provides functions for WICED BT stack initialization.
*
********************************************************************************
* \copyright
* Copyright 2018-2021 Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.
*
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

#include "wiced_bt_dev.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_stack_platform.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_version.h"

#include "cybt_platform_task.h"
#include "cybt_platform_trace.h"
#include "cybt_platform_config.h"
#include "cybt_platform_interface.h"
#include "platform_hal_wrapper.h"
#ifndef ENABLE_SMP_SERVER_MODULE
#define ENABLE_SMP_SERVER_MODULE 1
#endif
#ifndef ENABLE_SMP_CLIENT_MODULE
#define ENABLE_SMP_CLIENT_MODULE 1
#endif
#ifndef ENABLE_CREATE_LOCAL_KEYS
#define ENABLE_CREATE_LOCAL_KEYS 1
#endif
#ifndef ENABLE_HOST_RPA_GENERATION
#define ENABLE_HOST_RPA_GENERATION 0
#endif

#ifdef PROVIDE_INITIAL_SETUP_DATA_TO_STACK
extern wiced_bt_stack_init_cmd_data_t ctrl_data_for_stack_init;
#endif

wiced_result_t cybt_core_management_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);
/******************************************************************************
 *                                Constants
 ******************************************************************************/
#define HCI_VSC_WRITE_SLEEP_MODE             (0xFC27)
#define HCI_VSC_WRITE_SLEEP_MODE_LENGTH      (12)

#define BT_SLEEP_MODE_UART                   (1)
#define BT_SLEEP_THRESHOLD_HOST              (1)
#define BT_SLEEP_THRESHOLD_HOST_CONTROLLER   (1)
#define BT_SLEEP_ALLOW_HOST_SLEEP_DURING_SCO (1)
#define BT_SLEEP_COMBINE_SLEEP_MODE_AND_LPM  (1)
#define BT_SLEEP_ENABLE_UART_TXD_TRISTATE    (0)
#define BT_SLEEP_PULSED_HOST_WAKE            (0)
#define BT_SLEEP_SLEEP_GUARD_TIME            (0)
#define BT_SLEEP_WAKEUP_GUARD_TIME           (0)
#define BT_SLEEP_TXD_CONFIG                  (1)
#define BT_SLEEP_BT_WAKE_IDLE_TIME           (50)

pf_wiced_exception pf_platform_exception = NULL;

extern wiced_result_t host_stack_platform_smp_adapter_init();
extern wiced_result_t app_initialize_btstack_modules(void);
#ifdef COMPONENT_BLESS_IPC
int g_host_rpa_gen = 1;
#else
int g_host_rpa_gen = ENABLE_HOST_RPA_GENERATION;
#endif

/*****************************************************************************
 *                           Type Definitions
 *****************************************************************************/
typedef struct
{
    wiced_bt_management_cback_t   *p_app_management_callback;
#if defined (CY_USING_HAL)
    const cybt_platform_config_t  *p_bt_platform_cfg;
#endif
    bool                          is_sleep_mode_enabled;
} cybt_platform_main_cb_t;

/******************************************************************************
 *                           Variables Definitions
 ******************************************************************************/
cybt_platform_main_cb_t cybt_main_cb = {0};


/******************************************************************************
 *                          Function Declarations
 ******************************************************************************/
extern void host_stack_platform_interface_init(void);
extern const wiced_bt_cfg_settings_t *BTU_GetConfigSettings(void);

/******************************************************************************
 *                           Function Definitions
 ******************************************************************************/
void bt_sleep_status_cback (wiced_bt_dev_vendor_specific_command_complete_params_t *p_command_complete_params)
{
    MAIN_TRACE_DEBUG("bt_sleep_status_cback(): status = 0x%x",
                     p_command_complete_params->p_param_buf[0]);

    if(HCI_SUCCESS == p_command_complete_params->p_param_buf[0])
    {
        cybt_main_cb.is_sleep_mode_enabled = true;
    }
    else
    {
        cybt_main_cb.is_sleep_mode_enabled = false;
    }
}

bool bt_enable_sleep_mode(void)
{
    wiced_result_t result;
    uint8_t        sleep_vsc[HCI_VSC_WRITE_SLEEP_MODE_LENGTH];

    MAIN_TRACE_DEBUG("bt_enable_sleep_mode()");

    sleep_vsc[0] = BT_SLEEP_MODE_UART;
    sleep_vsc[1] = BT_SLEEP_THRESHOLD_HOST;
    sleep_vsc[2] = BT_SLEEP_THRESHOLD_HOST_CONTROLLER;
#if defined (CY_USING_HAL)
    sleep_vsc[3] = cybt_main_cb.p_bt_platform_cfg->controller_config.sleep_mode.device_wake_polarity;
    sleep_vsc[4] = cybt_main_cb.p_bt_platform_cfg->controller_config.sleep_mode.host_wake_polarity;
#else
    sleep_vsc[3] = platform_hal_gpio_dev_wake_polarity_wrapper();
    sleep_vsc[4] = platform_hal_gpio_host_wake_polarity_wrapper();
#endif
    sleep_vsc[5] = BT_SLEEP_ALLOW_HOST_SLEEP_DURING_SCO;
    sleep_vsc[6] = BT_SLEEP_COMBINE_SLEEP_MODE_AND_LPM;
    sleep_vsc[7] = BT_SLEEP_ENABLE_UART_TXD_TRISTATE;
    sleep_vsc[8] = 0;
    sleep_vsc[9] = 0;
    sleep_vsc[10] = 0;
    sleep_vsc[11] = BT_SLEEP_PULSED_HOST_WAKE;

    result = wiced_bt_dev_vendor_specific_command(HCI_VSC_WRITE_SLEEP_MODE,
                                                  HCI_VSC_WRITE_SLEEP_MODE_LENGTH,
                                                  sleep_vsc,
                                                  bt_sleep_status_cback
                                                 );
    if(WICED_BT_PENDING != result)
    {
        MAIN_TRACE_DEBUG("bt_enable_sleep_mode(): Fail to send vsc (0x%x)", result);
        return false;
    }

    return true;
}

/*
 * On stack initalization complete this call back gets called
*/
void wiced_post_stack_init_cback( void )
{
    MAIN_TRACE_DEBUG("wiced_post_stack_init_cback");

#ifndef DISABLE_DEFAULT_BTSTACK_INIT
    app_initialize_btstack_modules(); // initialize all layers
#endif

    wiced_bt_issue_btm_enabled_evt(cybt_core_management_cback);


    if(platform_hal_get_sleep_mode_enabled_wrapper())
    {
        bool status = bt_enable_sleep_mode();

        if(false == status)
        {
            MAIN_TRACE_ERROR("wiced_post_stack_init_cback(): Fail to init sleep mode");
        }
    }
    else
    {
        MAIN_TRACE_ERROR("wiced_post_stack_init_cback(): BT sleep mode is NOT enabled");
    }
}

/*
* This call back gets called for each HCI event.
*/
wiced_bool_t wiced_stack_event_handler_cback (uint8_t *p_event)
{
    return WICED_FALSE;
}

static void write_local_keys_to_stack(wiced_bt_local_identity_keys_t *p_keys)
{
    wiced_bt_features_t features = {0};

    wiced_bt_ble_read_le_features(NULL, features);
    int use_host_generation = (features[0] & (1 << 6)) ? 0 : 1;

    if (g_host_rpa_gen)
    {
        use_host_generation = 1;
    }

    MAIN_TRACE_DEBUG("[%s] host %d 0x%x", __FUNCTION__, use_host_generation, wiced_bt_get_btm_startup_flags());

    if (use_host_generation)
    {
#ifndef DISABLE_HOST_RPA_GENERATION
        wiced_ble_init_host_private_addr_generation(p_keys);
#endif
    }
    else
    {
#ifndef DISABLE_CTLR_RPA_GENERATION
        wiced_ble_init_ctlr_private_addr_generation(p_keys);
#endif
    }
    wiced_bt_issue_btm_enabled_evt(cybt_core_management_cback);
}

static wiced_result_t init_layers(void)
{
    /* handle in porting layer */
	wiced_result_t res = WICED_BT_SUCCESS;
#if (ENABLE_SMP_SERVER_MODULE == 1)
    res = wiced_bt_smp_server_module_init();
    if(res == WICED_BT_SUCCESS)
#endif
	{
#if (ENABLE_SMP_CLIENT_MODULE == 1)
    res = wiced_bt_smp_client_module_init();
#endif
	}

#if (ENABLE_CREATE_LOCAL_KEYS == 1)
    {
        wiced_bt_local_identity_keys_t keys = {0};

        wiced_ble_read_local_identity_keys_from_app(&keys);

        MAIN_TRACE_DEBUG("[%s] mask 0x%x", __FUNCTION__, keys.key_type_mask);

        if (keys.key_type_mask != 0)
        {
            write_local_keys_to_stack(&keys);
        }
        else
        {
            wiced_ble_create_local_identity_keys();
        }
    }
#endif
    return res;
}

#if (defined(__GNUC__) || defined(__ARMCC_VERSION) || defined(__llvm__))
extern __attribute__((weak)) wiced_result_t app_initialize_btstack_modules(void)
{
     return init_layers();
}
#endif

#if defined(__ICCARM__)
extern wiced_result_t app_initialize_btstack_modules(void);
#pragma weak app_initialize_btstack_modules=init_layers
#endif

wiced_result_t cybt_core_management_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_result_t result = WICED_BT_SUCCESS;
    int send_to_app = 1;

    switch(event)
    {
        case BTM_ENABLED_EVT:
            wiced_bt_init_resolution(); // To be removed, only required for non-privacy controllers
        break;
#if (ENABLE_CREATE_LOCAL_KEYS == 1)
        case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
        {
            write_local_keys_to_stack(&p_event_data->local_identity_keys_update);
        }
        break;
        case BTM_BLE_DEVICE_ADDRESS_UPDATE_EVENT:
            wiced_bt_issue_btm_enabled_evt(cybt_core_management_cback);
            break;
#endif
        default:
            break;
    }

    if(send_to_app && cybt_main_cb.p_app_management_callback)
    {
        result = cybt_main_cb.p_app_management_callback(event, p_event_data);
    }

    return result;
}

void cybt_core_stack_init(void)
{
    wiced_result_t result;

    /* Start the stack */
    wiced_bt_stack_init_internal(cybt_core_management_cback,
                                 wiced_post_stack_init_cback,
                                 wiced_stack_event_handler_cback
                                 );

    result = host_stack_platform_smp_adapter_init();
    if(WICED_SUCCESS != result)
	{
		MAIN_TRACE_ERROR("host_stack_platform_smp_adapter_init(): failed, result = 0x%x", result);
	}
}

wiced_result_t wiced_bt_stack_init(wiced_bt_management_cback_t *p_bt_management_cback,
                                   const wiced_bt_cfg_settings_t *p_bt_cfg_settings
                                   )
{
    wiced_result_t result;

    MAIN_TRACE_DEBUG("wiced_bt_stack_init()");

    cybt_main_cb.is_sleep_mode_enabled = false;

    cybt_platform_init();

    cybt_main_cb.p_app_management_callback = p_bt_management_cback;

    host_stack_platform_interface_init();

    wiced_bt_enable_stack_default_flow();

    /* Configure the stack */
    if(0 == wiced_bt_set_stack_config(p_bt_cfg_settings))
        return WICED_BT_ERROR;

    result = (wiced_result_t)cybt_platform_task_init((void *)p_bt_cfg_settings);

    return result;
}

wiced_result_t wiced_bt_stack_deinit( void )
{
    wiced_result_t result;
    extern cy_thread_t cybt_task[BT_TASK_NUM];
#if( configUSE_TICKLESS_IDLE != 0 )
    extern cy_thread_t sleep_timer_task;
    extern void cybt_platform_terminate_sleep_thread(void);
#endif
    cy_thread_t cur_thread_handle;

    if(CY_RSLT_SUCCESS != cy_rtos_get_thread_handle(&cur_thread_handle))
    {
        MAIN_TRACE_ERROR("wiced_bt_stack_deinit(): Fail to deinit stack.");
        return WICED_BT_ERROR;
    }

    if((cur_thread_handle == cybt_task[BT_TASK_ID_HCI_RX]) || (cur_thread_handle == cybt_task[BT_TASK_ID_HCI_TX])
    #if( configUSE_TICKLESS_IDLE != 0 )
        || (cur_thread_handle == sleep_timer_task)
    #endif
        )
    {
        /** This API is used to terminate all BT-own tasks and release all related resources.
         *  According to the current design, the OS task cannot be deleted by itself.
            In other words, the application must call wiced_stack_deinit( ) in the task context other than BT-own ones.
         */
        MAIN_TRACE_ERROR("Please call this API- wiced_bt_stack_deinit() in application thread.");
        return WICED_BT_ERROR;
    }
    else
    {
        result = (wiced_result_t)cybt_platform_task_deinit();

        if(WICED_BT_SUCCESS != result)
            return result;

        cybt_platform_deinit();

        cybt_platform_terminate_hci_rx_thread();

        cybt_platform_terminate_hci_tx_thread();

    #if( configUSE_TICKLESS_IDLE != 0 )
        cybt_platform_terminate_sleep_thread();
    #endif
    }

    return result;
}

bool cybt_platform_get_sleep_mode_status(void)
{
    return cybt_main_cb.is_sleep_mode_enabled;
}

#if defined (CY_USING_HAL)
void cybt_platform_config_init(const cybt_platform_config_t *p_bt_platform_cfg)
{
    MAIN_TRACE_DEBUG("cybt_platform_config_init()");

    cybt_main_cb.p_bt_platform_cfg = p_bt_platform_cfg;
}

const cybt_platform_config_t* cybt_platform_get_config(void)
{
    return cybt_main_cb.p_bt_platform_cfg;
}
#endif

void wiced_set_exception_callback(pf_wiced_exception pf_handler)
{
	pf_platform_exception = pf_handler;
}
