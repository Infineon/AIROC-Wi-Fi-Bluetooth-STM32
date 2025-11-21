/***************************************************************************//**
* \file cybt_platform_freertos.c
*
* \brief
* Implementation for BT porting interface on FreeRTOS
*
********************************************************************************
* \copyright
* Copyright 2018-2019 Cypress Semiconductor Corporation
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

#include <string.h>
#include "cy_ipc_bt.h"
#include "cybt_platform_trace.h"
#include "cyabs_rtos.h"
#include "cybt_platform_task.h"
#include "cybt_platform_interface.h"
#include "cybt_platform_internal.h"
#include "cybt_platform_config.h"
#include "cybt_platform_util.h"
#include "wiced_bt_stack_platform.h"

/******************************************************************************
 *                                Constants
 ******************************************************************************/
#define NOT_IN_ISR              (false)
#define IN_ISR                  (true)

/* MCU IPC interupt */
#define IPC1_INTERRUPT          (btss_interrupt_btss_ipc_IRQn)
#define IPC1_PRIORITY           (5u)      /* IPC1 interrupt priority */

/* MCU IPC channel definition */
#define MCU_IPC_HCI_UL          (0)       /* For HCI  MCU -> BLE */
#define MCU_IPC_HCI_DL          (1)       /* For HCI  BLE -> MCU */
#define MCU_IPC_HPC_UL          (2)       /* For HPC  MCU -> BLE */
#define MCU_IPC_HPC_DL          (3)       /* For HPC  BLE -> MCU */

/* Interrupt Structure definition */
#define IPC_INTR_BLE            (0)
#define IPC_INTR_MCU            (1)

#define MAX_IPC_RETRIES         (20)

#define RECORD_IPC_STATS

/*****************************************************************************
 *                           Type Definitions
 *****************************************************************************/
typedef struct
{
    bool            inited;
    cy_semaphore_t  boot_fully_up;
    /* Context for MCU IPC */
    cy_stc_ipc_bt_context_t ipc_context;
} hci_interface_t;

typedef uint16_t bt_task_event_t;

#ifdef RECORD_IPC_STATS
typedef struct {
    uint32_t get_buf_fail_count; /* Number of get buffer requests that failed. This does not count number of retries performed for same request.  */
    uint32_t get_buf_max_retry_count; /* Maximum number of failed retries permormed for get buffer request. 0 <= MAX_IPC_RETRIES */
    uint32_t write_fail_count; /* Number of write requests that failed. This does not count number of retries performed for same request.  */
    uint32_t write_max_retry_count; /* Maximum number of failed retries permormed for write request. 0 <= MAX_IPC_RETRIES */
}cybt_ipc_stats_t;
#endif

/******************************************************************************
 *                           Callback Prototypes
 ******************************************************************************/
cy_en_syspm_status_t cybt_platform_syspm_DeepSleepCallback(cy_stc_syspm_callback_params_t* callbackParams,
                                                 cy_en_syspm_callback_mode_t mode);

/******************************************************************************
 *                           Variables Definitions
 ******************************************************************************/
static hci_interface_t   hci_cb;
static bt_task_event_t timer_msg = BT_EVT_TIMEOUT;
static bt_task_event_t hci_core_msg = BT_EVT_CORE_HCI;
static bt_task_event_t hci_boot_msg = BT_EVT_TASK_BOOT_COMPLETES;
static bt_task_event_t shutdown_msg = BT_EVT_TASK_SHUTDOWN;
static bt_task_event_t buffer_msg = BT_EVT_CORE_HCI_WRITE_BUF_AVAIL;
static bt_task_event_t reset_msg = BT_EVT_TASK_RESET_COMPLETE;

#ifdef RECORD_IPC_STATS
static cybt_ipc_stats_t ipc_stats;
#endif

static cy_stc_syspm_callback_params_t cybt_syspm_callback_param = { NULL, NULL };
static cy_stc_syspm_callback_t        cybt_syspm_callback       =
{
    .callback       = &cybt_platform_syspm_DeepSleepCallback,
    .type           = (cy_en_syspm_callback_type_t) CY_SYSPM_MODE_DEEPSLEEP_RAM,
    .callbackParams = &cybt_syspm_callback_param,
    .order          = 253u,
};


/******************************************************************************
 *                          Function Declarations
 ******************************************************************************/
extern cybt_result_t cybt_platform_msg_to_bt_task(const uint16_t msg, bool is_from_isr);
extern void cybt_platform_exception_handler(uint16_t error, uint8_t *ptr, uint32_t length);

/******************************************************************************
 *                           Function Definitions
 ******************************************************************************/
static void reset_btss(void)
{
    HCIDRV_TRACE_DEBUG("BTSS Reset\n");

    /* It will assert reset ( release group 3 IP ) */
    (void)Cy_SysClk_PeriGroupSetSlaveCtl(3, CY_SYSCLK_PERI_GROUP_SL_CTL2, 0xFFFFFFFFU); /* Suppress a compiler warning about unused return value */

    /* It will de-assert reset ( release group 3 IP ) */
    (void)Cy_SysClk_PeriGroupSetSlaveCtl(3, CY_SYSCLK_PERI_GROUP_SL_CTL2, 0x0U); /* Suppress a compiler warning about unused return value */

    HCIDRV_TRACE_DEBUG("BTSS Reset Complete\n");
}

BTSTACK_PORTING_SECTION_BEGIN
static void notify_rx_not_empty(uint32_t * msg)
{
    cybt_platform_msg_to_bt_task(BT_EVT_CORE_HCI, IN_ISR);
    return;
}
BTSTACK_PORTING_SECTION_END

static void notify_write_buffer_available(cy_en_btipc_buftype_t type)
{
    UNUSED_VARIABLE(type);
    cybt_platform_msg_to_bt_task(BT_EVT_CORE_HCI_WRITE_BUF_AVAIL, IN_ISR);
    return;
}

/* IPC IRQ Handler for MCU */
BTSTACK_PORTING_SECTION_BEGIN
static void ipc_irq_handler_mcu(void)
{
    CONTROLLER_SLEEP(LOCK); // before Cy_BTIPC_IRQ_Handler
    Cy_BTIPC_IRQ_Handler(&hci_cb.ipc_context);
    CONTROLLER_SLEEP(UNLOCK); // after Cy_BTIPC_IRQ_Handler
}
BTSTACK_PORTING_SECTION_END

/* Release callback for MCU */
static void release_on_tx_done(void)
{

}

#ifdef NOTIFY_IPC_PM
/* HPC Notify callback to MCU for power management */
static void notify_callback_mcu_pm(uint32_t * msg)
{
    HCIDRV_TRACE_DEBUG("MCU: HPC Power Manager  0x%lx, 0x%lx\n",msg[0], msg[1]);
    return;
}
#endif

/* HPC Notify callback to MCU for boot type in init process */
static void notify_callback_mcu_longmsg(uint32_t * msg)
{
    /* Don't add any prints in ISR Context. It may lead to crash */
    uint8_t bootType = *(((uint8_t*)msg)+2);
    switch(bootType)
    {
        case CY_BT_IPC_BOOT_CONFIG_WAIT:
            cybt_platform_msg_to_bt_task(BT_EVT_TASK_BOOT_COMPLETES, IN_ISR);
            break;
        case CY_BT_IPC_BOOT_FULLY_UP:
#ifdef FPGA_TEST_PLATFORM
            /* On FPGA, controller does not send CY_BT_IPC_BOOT_CONFIG_WAIT and directly sends CY_BT_IPC_BOOT_FULLY_UP */
            cybt_platform_msg_to_bt_task(BT_EVT_TASK_BOOT_COMPLETES, IN_ISR);
#else

#if (defined(COMPONENT_CYW20829B0) || defined(COMPONENT_CYW89829B0))
            Cy_SysClk_ClkHfSetSource(0U, CY_SYSCLK_CLKHF_IN_CLKPATH0); /* restore to original frequency */
#endif // (defined(COMPONENT_CYW20829B0) || defined(COMPONENT_CYW89829B0))

            cy_rtos_set_semaphore(&hci_cb.boot_fully_up, IN_ISR);
#endif /* FPGA_TEST_PLATFORM */
            break;
        default:
            break;
    }
    return;
}

#ifdef NOTIFY_IPC_TRNG
/* HPC Notify callback to MCU for TRNG */
static void notify_callback_mcu_trng(uint32_t * msg)
{
    HCIDRV_TRACE_DEBUG("MCU: HPC TRNG indication: 0x%lx, 0x%lx\n",msg[0], msg[1]);
    return;
}
#endif

BTSTACK_PORTING_SECTION_BEGIN
static cybt_result_t ipc_hci_release_buffer(uint32_t *p_msg)
{
    uint32_t time_out = 2000UL; //2ms
    cy_en_btipcdrv_status_t ipc_status;
    uint8_t ipc_status_buf[4]={0};

    do
    {
        ipc_status = Cy_BTIPC_HCI_RelBuffer(&hci_cb.ipc_context, p_msg);

        if(CY_BT_IPC_DRV_SUCCESS == ipc_status)
            return CYBT_SUCCESS;

        RELBUF_DELAY(); // default it will be dummy

        time_out--;
    } while (time_out > 0);

    HCIDRV_TRACE_ERROR("[%s] Release buffer failure: 0x%x", __FUNCTION__, ipc_status);

    UINT32_TO_BE_FIELD(ipc_status_buf,ipc_status);
    cybt_platform_exception_handler(CYBT_PORTING_HCI_IPC_REL_BUFFER, (uint8_t *)ipc_status_buf, sizeof(ipc_status_buf)/sizeof(ipc_status_buf[0]));

    /* App wants to continue on error, we can proceed */
    return CYBT_ERR_HCI_IPC_REL_BUFFER_FAILED;
}
BTSTACK_PORTING_SECTION_END

void cybt_platform_hci_wait_for_boot_fully_up(bool is_from_isr)
{
    cy_rtos_get_semaphore(&hci_cb.boot_fully_up, CY_RTOS_NEVER_TIMEOUT, is_from_isr);
}

BTSTACK_PORTING_SECTION_BEGIN
uint8_t *cybt_platform_hci_get_buffer(hci_packet_type_t pti, uint32_t size)
{
    static uint8_t short_msg[MAX_SHORT_MESG_LENGTH] = {0};
    cy_en_btipcdrv_status_t ipc_status;
    uint8_t *p_txbuffer = &short_msg[0];
    cy_en_btipc_hcipti_t hci_pti = (cy_en_btipc_hcipti_t) pti;

    /* Sleep lock to get and access shared buffer.
     * It will be unlocked in cybt_platform_hci_write() */
    CONTROLLER_SLEEP(LOCK); // before get_ipc_write_buffer and ipc_write

    if ( (size > MAX_SHORT_MESG_LENGTH) || (hci_pti == CY_BT_IPC_HCI_ISO) || (hci_pti == CY_BT_IPC_HCI_ACL) )
    {
        int retries = 0;

        do
        {
            ipc_status = Cy_BTIPC_HCI_GetWriteBufPtr(&hci_cb.ipc_context, hci_pti, (void**)&p_txbuffer, size);

            if (CY_BT_IPC_DRV_SUCCESS != ipc_status)
            {
                /* adding a delay before retrying */
                {
                    volatile int delay = 1000;
                    while(delay--);
                }
                //HCIDRV_TRACE_ERROR("MCU Error: IPC Get Buffer to BLE failed 0x%x!\n", ipc_status);
#ifdef RECORD_IPC_STATS
                if (!retries)
                    ++ipc_stats.get_buf_fail_count;

                if (retries > ipc_stats.get_buf_max_retry_count){
                    ipc_stats.get_buf_max_retry_count = retries;
                }
#endif
            }
            else {
                return (p_txbuffer);
            }
        }while(++retries < MAX_IPC_RETRIES);

        /* Unlocking here since no buffer available */
        CONTROLLER_SLEEP(UNLOCK); // if no ipc_write_buffer
        return (NULL);
    }
    return (p_txbuffer);
}
BTSTACK_PORTING_SECTION_END

void cybt_platform_hci_post_stack_init(void)
{
    Cy_BTIPC_Buffer_RegisterCb(&hci_cb.ipc_context, notify_write_buffer_available);
}

BTSTACK_PORTING_SECTION_BEGIN
cybt_result_t cybt_platform_msg_to_bt_task(const uint16_t msg, bool is_from_isr)
{
    cybt_result_t result;
    switch(msg)
    {
        case BT_EVT_TIMEOUT:
            result = cybt_send_msg_to_bt_task(&timer_msg, is_from_isr);
            break;
        case BT_EVT_CORE_HCI:
            result = cybt_send_msg_to_bt_task(&hci_core_msg, is_from_isr);
            break;
        case BT_EVT_TASK_BOOT_COMPLETES:
            result = cybt_send_msg_to_bt_task(&hci_boot_msg, is_from_isr);
            break;
        case BT_EVT_TASK_SHUTDOWN:
            result = cybt_send_msg_to_bt_task(&shutdown_msg, is_from_isr);
            result = cybt_send_msg_to_bt_task(&reset_msg, is_from_isr);
            break;
        case BT_EVT_CORE_HCI_WRITE_BUF_AVAIL:
            result = cybt_send_msg_to_bt_task(&buffer_msg, is_from_isr);
            break;
        default:
            result = CYBT_ERR_BADARG;
            break;
    }
    return (result);
}
BTSTACK_PORTING_SECTION_END

cy_en_syspm_status_t cybt_platform_syspm_DeepSleepCallback(cy_stc_syspm_callback_params_t* callbackParams,
                                                 cy_en_syspm_callback_mode_t mode)
{
    cy_en_syspm_status_t retVal = CY_SYSPM_FAIL;

    CY_UNUSED_PARAMETER(callbackParams);

    switch (mode)
    {
        case CY_SYSPM_CHECK_READY:
        {
            retVal = CY_SYSPM_SUCCESS;
            break;
        }

        case CY_SYSPM_CHECK_FAIL:
        {
            retVal = CY_SYSPM_SUCCESS;
            break;
        }

        case CY_SYSPM_BEFORE_TRANSITION:
        {
            retVal = CY_SYSPM_SUCCESS;
            break;
        }

        case CY_SYSPM_AFTER_TRANSITION:
        {
            Cy_BTSS_PowerDep(true);
            Cy_BTIPC_WarmInit(&hci_cb.ipc_context, (cy_stc_ipc_bt_config_t *) 0xFFFFFFFF);
            Cy_BTSS_PowerDep(false);

            retVal = CY_SYSPM_SUCCESS;
            break;
        }

        default:
            break;
    }

    return retVal;
}

cybt_result_t cybt_platform_hci_open(void *p_arg)
{
    cy_en_btipcdrv_status_t ipc_status;
    uint32_t interruptState;

    /* MCU IPC Config */
    cy_stc_ipc_bt_config_t btIpcHciConfig_mcu = {
        .ulChannelHCI = MCU_IPC_HCI_UL, /* HCI Uplink channel from MCU to BLE */
        .dlChannelHCI = MCU_IPC_HCI_DL, /* HCI Downlink channel from BLE to MCU */
        .ulChannelHPC = MCU_IPC_HPC_UL, /* HPC Uplink channel from MCU to BLE */
        .dlChannelHPC = MCU_IPC_HPC_DL, /* HPC Downlink channel from BLE to MCU */

        .intStuctureSelf = IPC_INTR_MCU, /* Interrupt Structure 1 is used for MCU */
        .intStucturePeer = IPC_INTR_BLE, /* Interrupt Structure 0 is used for BLE */

        .ipcIntConfig.intrSrc = IPC1_INTERRUPT,
        .ipcIntConfig.intrPriority = IPC1_PRIORITY,

        /* This handler will be removed later once simulations are done */
        .irqHandlerPtr = ipc_irq_handler_mcu,
        /* This callback will be removed later once simulations are done */
        .internal_hpc_notify_cb = Cy_BTIPC_HPC_Notify,

        .ulReleaseCallbackPtr = release_on_tx_done,
        .bufCallbackPtr  = NULL, // will register once host stack init is done
    };

    UNUSED_VARIABLE(p_arg);

    if(true == hci_cb.inited)
    {
        return  CYBT_SUCCESS;
    }

    /* host power domain: CY_PD_PDCM_MAIN (MAIN Power domain)
     * destination power doamin: CY_PD_PDCM_BTSS (BTSS Power domain)
     * cy_pd_pdcm_set_dependency() will create dependency settting of host on destination.
     * which indirectly means host will not go into deep sleep until destination is running.*/
    (void)cy_pd_pdcm_set_dependency(CY_PD_PDCM_MAIN, CY_PD_PDCM_BTSS); /* Suppress a compiler warning about unused return value */

    memset(&hci_cb, 0, sizeof(hci_interface_t));

    cy_rtos_init_semaphore(&hci_cb.boot_fully_up, 1, 0);

    interruptState = Cy_SysLib_EnterCriticalSection();
    reset_btss();

    CONTROLLER_SLEEP(LOCK); // Cy_BTIPC_Init lock
    ipc_status = Cy_BTIPC_Init(&hci_cb.ipc_context, &btIpcHciConfig_mcu);
    CONTROLLER_SLEEP(UNLOCK); // Cy_BTIPC_Init unlock

    if (ipc_status)
    {
        HCIDRV_TRACE_ERROR("MCU Error: IPC HCI Init failed !\n");
        CY_ASSERT(0);
    }

    /* MCU Register HCI notify callback */
    ipc_status = Cy_BTIPC_HCI_RegisterCb(&hci_cb.ipc_context, notify_rx_not_empty);
    if (ipc_status)
    {
        HCIDRV_TRACE_ERROR("MCU Error: IPC HCI Notify Cb register failed 0x%x!\n", ipc_status);
        CY_ASSERT(0);
    }

    /* MCU Register HPC callback for power management */
    {
        cy_stc_ipc_hcp_cb_t notifyCallbackParam[] = {
#ifdef NOTIFY_IPC_PM
            {.hpcNotifyCallbackPtr = notify_callback_mcu_pm, .msgType = CY_BT_IPC_HPC_PM},
#endif
            {.hpcNotifyCallbackPtr = notify_callback_mcu_longmsg, .msgType = CY_BT_IPC_HPC_LONG},
#ifdef NOTIFY_IPC_TRNG
            {.hpcNotifyCallbackPtr = notify_callback_mcu_trng, .msgType = CY_BT_IPC_HPC_REQTRNG},
#endif
        };

        for (int i = 0; i < sizeof(notifyCallbackParam)/sizeof(notifyCallbackParam[0]); i++)
        {
            if(notifyCallbackParam[i].hpcNotifyCallbackPtr){
            ipc_status = Cy_BTIPC_HPC_RegisterCb(&hci_cb.ipc_context, &notifyCallbackParam[i]);
                if (ipc_status)
                {
                    HCIDRV_TRACE_ERROR("MCU Error: IPC register %d failed 0x%x!\n", i, ipc_status);
                    CY_ASSERT(0);
                }
            }
        }
    }

    Cy_SysLib_ExitCriticalSection(interruptState);

    if (!Cy_SysPm_RegisterCallback(&cybt_syspm_callback))
    {
    	HCIDRV_TRACE_ERROR("MCU Error: Syspm Callback registering failed 0x%x!\n", ipc_status);
        CY_ASSERT(0);
    }

    hci_cb.inited = true;

    HCIDRV_TRACE_DEBUG("hci_open(): Done");

    return  CYBT_SUCCESS;
}

BTSTACK_PORTING_SECTION_BEGIN
cybt_result_t cybt_platform_hci_write(hci_packet_type_t pti,
                                      uint8_t *p_data,
                                      uint32_t length
                                      )
{
    cybt_result_t status =  CYBT_SUCCESS;
    cy_en_btipcdrv_status_t ipc_status;
    int retries = 0;

    do
    {
        ipc_status = Cy_BTIPC_HCI_Write(&hci_cb.ipc_context, (cy_en_btipc_hcipti_t)pti, p_data, length);

        if (ipc_status)
        {
            /* adding a delay before retrying */
            {
                volatile int delay = 1000;
                while(delay--);
            }

            status = CYBT_ERR_HCI_WRITE_FAILED;

#ifdef RECORD_IPC_STATS
			if (!retries)
				++ipc_stats.write_fail_count;

			if (retries > ipc_stats.write_max_retry_count){
				ipc_stats.write_max_retry_count = retries;
            }
#endif
        }else{
            status = CYBT_SUCCESS;
            break;
        }
    }while(++retries < MAX_IPC_RETRIES);

    /* Sleep unlock which was already locked in cybt_platform_hci_get_buffer */
    CONTROLLER_SLEEP(UNLOCK); // after get_ipc_write_buffer and ipc_write
    return status;
}
BTSTACK_PORTING_SECTION_END

BTSTACK_PORTING_SECTION_BEGIN
uint16_t cybt_platfrom_hci_get_rx_fifo_count(void)
{
    if(!hci_cb.inited)
        return 0;
    return (Cy_BTIPC_HCI_FIFOCount(&hci_cb.ipc_context));
}
BTSTACK_PORTING_SECTION_END

BTSTACK_PORTING_SECTION_BEGIN
cybt_result_t cybt_platform_hci_read(void *event,
                                     hci_packet_type_t *pti,
                                     uint8_t  *p_data,
                                     uint32_t *p_length
                                     )
{
    cybt_result_t status =  CYBT_SUCCESS;
    uint8_t *ptr_rx_hci = NULL;
    uint32_t *p_msg = NULL;

    UNUSED_VARIABLE(event);

    if(!(hci_cb.inited && pti && p_data && p_length))
    {
        HCIDRV_TRACE_ERROR("hci_read(): UART is NOT initialized");
        return  CYBT_ERR_HCI_NOT_INITIALIZE;
    }

    CONTROLLER_SLEEP(LOCK); // before ipc_read

    /* cybt_platform_hci_read only be called when FIFO count is non-zero.
     * And NULL check already handling in the beginning.
     * Hence no need to check return values for below IPC API's */
    Cy_BTIPC_HCI_FIFOGet(&hci_cb.ipc_context, &p_msg, 0);
    Cy_BTIPC_HCI_getPTI ((cy_en_btipc_hcipti_t *)pti, p_length, p_msg);
    Cy_BTIPC_HCI_GetReadBufPtr (&hci_cb.ipc_context, (void**)&ptr_rx_hci, (size_t*)p_length);

    memcpy((uint8_t*)p_data, ptr_rx_hci, *p_length);
    status = ipc_hci_release_buffer(p_msg);

    CONTROLLER_SLEEP(UNLOCK); // after ipc_read

    return status;
}
BTSTACK_PORTING_SECTION_END

cybt_result_t cybt_platform_hci_close(void)
{
    cybt_result_t status = CYBT_SUCCESS;
    if(false == hci_cb.inited)
    {
        HCIDRV_TRACE_ERROR("hci_close(): Not inited\n");
        return  CYBT_ERR_HCI_NOT_INITIALIZE;
    }

    if(CY_BT_IPC_DRV_SUCCESS != Cy_BTIPC_Deinit(&hci_cb.ipc_context))
    {
        HCIDRV_TRACE_ERROR("hci_close(): Cy_BTIPC_Deinit failed\n");
        status = CYBT_ERR_HCI_IPC_DEINIT_FAILED;
    }

    if (!Cy_SysPm_UnregisterCallback(&cybt_syspm_callback))
    {
        HCIDRV_TRACE_ERROR("MCU Error: Syspm Callback Unregistering failed!\n");
        CY_ASSERT(0);
    }

    cy_rtos_deinit_semaphore(&hci_cb.boot_fully_up);

    memset(&hci_cb, 0, sizeof(hci_interface_t));

    return status;
}

BTSTACK_PORTING_SECTION_BEGIN
uint16_t cybt_platform_get_event_id(void *event)
{
    bt_task_event_t *evt = (bt_task_event_t *)event;
    return (*evt);
}
BTSTACK_PORTING_SECTION_END

cybt_result_t cybt_platform_hci_set_baudrate(uint32_t baudrate)
{
    /* This function is not applicable if HCI communication is using IPC instead UART
     * Only reason to keeping this function is to maintain compatibility
     */

    UNUSED_VARIABLE(baudrate);
    return  CYBT_SUCCESS;
}

BTSTACK_PORTING_SECTION_BEGIN
bool cybt_platform_hci_process_if_coredump(uint8_t *p_data, uint32_t length)
{
    if (0xF4FF != *((uint16_t *)&p_data[0]))
    {
        /* Packet raw data will be like this
         * fff41b03107907020000001800000004000081f0d80020000000000000000000
         * 0xFFF4 opcode, 0x1B DBFW dump event subcode, 0x03 dumpp type, 0x10 length, 0x79 check sum */
        return false;
    }

    /* You may get multiple core dump packets don't assert here */
    cybt_platform_exception_handler(CYBT_CONTROLLER_CORE_DUMP, p_data, length);
    return true;
}
BTSTACK_PORTING_SECTION_END

#ifdef RECORD_IPC_STATS
cybt_result_t cybt_platform_hci_get_ipc_stats(cybt_ipc_stats_t *p_stats)
{
	p_stats->get_buf_fail_count = ipc_stats.get_buf_fail_count;
	p_stats->get_buf_max_retry_count = ipc_stats.get_buf_max_retry_count;
	p_stats->write_fail_count = ipc_stats.write_fail_count;
	p_stats->write_max_retry_count = ipc_stats.write_max_retry_count;
	return  CYBT_SUCCESS;
}
#endif

BTSTACK_PORTING_SECTION_BEGIN
void cybt_platform_btss_get_trng(uint8_t *p_rand, uint8_t *p_len)
{
    static bool trng_init = false;
	uint32_t rand_num;
	uint8_t loop_cnt = 0, remainder = 0;

    if (!trng_init)
    {
        Cy_Cryptolite_Trng_Init(CRYPTOLITE, NULL);

        trng_init = true;
    }
    
    loop_cnt = ((*p_len) / 4);
    remainder = ((*p_len) % 4);

    for(uint8_t i = 0; i < loop_cnt; i++)
    {
    	Cy_Cryptolite_Trng(CRYPTOLITE, &rand_num);
    	memcpy((p_rand + (i * 4)), (uint8_t *)&rand_num, sizeof(rand_num));
    }

    if(remainder)
    {
    	Cy_Cryptolite_Trng(CRYPTOLITE, &rand_num);
    	memcpy((p_rand + (loop_cnt * 4)), (uint8_t *)&rand_num, remainder);
    }
}
BTSTACK_PORTING_SECTION_END
