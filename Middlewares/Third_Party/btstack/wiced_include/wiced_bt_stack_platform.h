/*
 * Copyright 2019-2025, Cypress Semiconductor Corporation or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */
/** @file
 *
 * Runtime Bluetooth configuration parameters
 *
 */
 /**
 * @addtogroup  wiced_bt_platform_group Bluetooth Stack Platform Interface
 *
 * Interface between Stack and platform.
 *
 * @{
 */

#ifndef __WICED_BT_STACK_PLATFORM_H__
#define __WICED_BT_STACK_PLATFORM_H__


#include <stdarg.h>
#include "wiced_bt_types.h"
#include "wiced_data_types.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_dev.h"


/**< Stack Exception definitions  */
#define CYBT_STACK_BASE_EXCEPTION        0xF000                         /**< Stack BASE exception */
#define CYBT_STACK_BUF_CORRUPTED         CYBT_STACK_BASE_EXCEPTION + 1  /**< Indicates a buffer/memory corruption issue */
#define CYBT_STACK_NOT_BUF_OWNER         CYBT_STACK_BASE_EXCEPTION + 2  /**< Signifies a lack of buffer ownership by the current process */
#define CYBT_STACK_FREEBUF_BAD_QID       CYBT_STACK_BASE_EXCEPTION + 3  /**< Occurs when attempting to free a buffer with an invalid queue ID */
#define CYBT_STACK_FREEBUF_BUF_LINKED    CYBT_STACK_BASE_EXCEPTION + 4  /**< Denotes an error with a linked buffer during the free operation */
#define CYBT_STACK_SEND_MSG_BAD_DEST     CYBT_STACK_BASE_EXCEPTION + 5  /**< Indicates a bad destination for a message send operation */
#define CYBT_STACK_SEND_MSG_BUF_LINKED   CYBT_STACK_BASE_EXCEPTION + 6  /**< Represents an issue with a linked buffer during the message send operation */
#define CYBT_STACK_ENQUEUE_BUF_LINKED    CYBT_STACK_BASE_EXCEPTION + 7  /**< Indicates a linked buffer error during the enqueue operation */
#define CYBT_STACK_DELETE_POOL_BAD_QID   CYBT_STACK_BASE_EXCEPTION + 8  /**< Occurs when attempting to delete a pool with an invalid queue ID */
#define CYBT_STACK_BUF_SIZE_TOOBIG       CYBT_STACK_BASE_EXCEPTION + 9  /**< Signifies that the buffer size exceeds the maximum allowed size */
#define CYBT_STACK_BUF_SIZE_ZERO         CYBT_STACK_BASE_EXCEPTION + 10 /**< Indicates an attempt to create a buffer with zero size */
#define CYBT_STACK_ADDR_NOT_IN_BUF       CYBT_STACK_BASE_EXCEPTION + 11 /**< Denotes an address that is not within the buffer range */
#define CYBT_STACK_OUT_OF_BUFFERS        CYBT_STACK_BASE_EXCEPTION + 12 /**< Occurs when there are no available buffers */
#define CYBT_STACK_GETPOOLBUF_BAD_QID    CYBT_STACK_BASE_EXCEPTION + 13 /**< Signifies an invalid queue ID during the get pool buffer operation */
#define CYBT_STACK_POOLBUF_BAD_SIZE      CYBT_STACK_BASE_EXCEPTION + 14 /**< Indicates an error with the size of the pool buffer */
#define CYBT_STACK_NO_INTERFACE          CYBT_STACK_BASE_EXCEPTION + 15 /**< Signifies the absence of a required interface */
#define CYBT_STACK_BAD_TRANSPORT         CYBT_STACK_BASE_EXCEPTION + 16 /**< Denotes a problem with the transport assignement (BLE / BREDR) */
#define CYBT_STACK_NO_MEMORY             CYBT_STACK_BASE_EXCEPTION + 17 /**< Indicates a memory allocation failure */
#define CYBT_STACK_NO_BUF                CYBT_STACK_BASE_EXCEPTION + 18 /**< Represents the absence of available buffers */
#define CYBT_STACK_MAX_EXCEPTION         CYBT_STACK_NO_BUF + 1          /**< Stack MAX exception */

typedef struct _aes_context_t aes_context; /**< Forward definition */

/** AIROC Bluetooth Stack Platform */
typedef struct
{
    /**
     * Stack exception callback
     */
    pf_wiced_exception pf_exception;

    /**
     * Platform function to allocate memory
     *
     * Called by stack code to allocate memory from the OS/Platform.
     * Implementing function is expected to return memory allocated from
     * the OS/Platform
     *
     * @param[in] size    : Size of memory to be allocated
     *
     * @return : Pointer to allocated memory
     */
    void * (*pf_os_malloc)(uint32_t size);

    /**
     * Platform memory free
     *
     * Called by stack code to free memory back to the OS/Platform.
     * Implementing function is expected to free the memory allocated
     * using pf_os_malloc (refer #pf_os_malloc ) call from the OS/Platform
     *
     * @param[in] p_mem    : Ptr to memory to be freed
     *
     * @return : None
     */
    void   (*pf_os_free)(void* p_mem);

    /**
     * Platform function to get tick count
     *
     * Called by stack timer code to get the free running 64 bit tick count
     *
     * @param[in] None
     *
     * @return : 64 bit current tick count
     */
    uint64_t (*pf_get_tick_count_64)(void);

    /**
     * Platform function to set the next timeout
     *
     * Called by stack timer code set the next timeout
     *
     * @param[in] abs_tick_count : 64 bit tick count instant at which the timeout has to occur
     *
     * @return : void
     */
    void   (*pf_set_next_timeout)(uint64_t  abs_tick_count);

    /** Stack lock */
    wiced_bt_lock_t stack_lock;

    /**
     * Platform function to get ACL buffer to send to lower
     *
     * Called by stack to get a buffer to fill in the data to be sent to 'transport' (LE or BR/EDR)
     * of 'size'
     *
     * @param[in] transport : Transport on which the buffer is to be sent
     * @param[in] size      : Size of the buffer
     *
     * @return : Pointer to buffer which will be filled with data
     */
    uint8_t       *(*pf_get_acl_to_lower_buffer)(wiced_bt_transport_t transport, uint32_t size);

    /**
     * Platform function to write ACL buffer to lower
     *
     * Called by stack to send the buffer allocated using pf_get_acl_to_lower_buffer
     * after filling it with the data to send.
     *
     * @param[in] transport : Transport on which the buffer is to be sent
     * @param[in] p_data    : Pointer received using pf_get_acl_to_lower_buffer
     * @param[in] len       : Length of data at p_data
     *
     * @return : wiced_result_t
     */
    wiced_result_t (*pf_write_acl_to_lower)(wiced_bt_transport_t transport, uint8_t* p_data, uint16_t len);

    /**
     * Platform function to write ISO buffer to lower
     *
     * Called by stack to send the buffer allocated using pf_get_acl_to_lower_buffer
     * after filling it with the data to send.
     *
     * @param[in] p_data    : Pointer received using pf_get_acl_to_lower_buffer
     * @param[in] len       : Length of data at p_data
     *
     * @return : wiced_result_t
     */
    wiced_result_t (*pf_write_iso_to_lower)(uint8_t* p_data, uint16_t len);

    /**
     * Platform function to write CMD buffer to lower
     *
     * Called by stack to send HCI CMD buffer to lower
     *
     * @param[in] p_cmd   : Pointer to HCI CMD data
     * @param[in] cmd_len : Length of data at p_cmd
     *
     * @return : wiced_result_t
     */
    wiced_result_t (*pf_write_cmd_to_lower)(uint8_t * p_cmd, uint16_t cmd_len);

     /**
     * Platform function to get SCO buffer to send to lower
     *
     * Called by stack to get a SCO buffer to fill in the data to be sent to HCI
     * of 'size'
     *
     * @param[in] size      : Size of the buffer
     *
     * @return : Pointer to buffer which will be filled with data
     */
    uint8_t       *(*pf_get_sco_to_lower_buffer)(uint32_t size);

    /**
     * Platform function to write SCO buffer to lower
     *
     * Called to send SCO CMD buffer to lower
     *
     * @param[in] p_sco_data   : Pointer to SCO data
     * @param[in] len      : Length of data at p_data
     *
     * @return : wiced_result_t
     */
    wiced_result_t (*pf_write_sco_to_lower)(uint8_t* p_sco_data, uint8_t len);

    /**
     * Callback function to trace HCI messages
     *
     * Called by stack to allow application to trace the HCI messages.
     * Application/Porting code is expected to treat received data as read only, and make a
     * copy of the data to reference it outside of the callback
     *
     * @param[in] type   : HCI event data type
     * @param[in] len    : Length of data at p_data
     * @param[in] p_data : Pointer to data
     *
     * @return : void
     */
    void (*pf_hci_trace_cback_t)(wiced_bt_hci_trace_type_t type, uint16_t len, uint8_t* p_data);

    /**
     * Callback function to dump out trace messages
     * This interface function can be NULL if no debug tracing is supported
     *
     * Called by stack to allow application to write debug trace messages
     *
     * @param[in] p_trace_buf   : Pointer to the trace buffer
     * @param[in] trace_buf_len : Length of the trace buffer
     * @param[in] trace_type    : Type of trace message
     *
     * @return : void
     */
    void (*pf_debug_trace)(char *p_trace_buf, int trace_buf_len, wiced_bt_trace_type_t trace_type);

    /** trace_buffer_len : Trace buffer len */
    int   trace_buffer_len;
    /**
     * trace_buffer     : Pointer to the trace buffer
     * Applications can set this to NULL to disable traces
     */
    char* trace_buffer;


    /**
     * Used for additional controller initialization by the porting layer to be performed
     * after the HCI reset. Can be set to NULL if no additional initialization required
     */
    void (*pf_patch_download)(void);

    /**
    * Set platform specific Random generator function
    * @note If platform does not support TRNG function then set this parameter to NULL
    * @return : wiced_result_t
    * @param[in] p_rand  : buffer to store generated random number
    * @param[in] p_len  : size of the buffer
    */
    void (*pf_get_trng)(uint8_t *p_rand, uint8_t *p_len);

    /**
     *set is_legacy_bless_controller to 1 for only BLESS controllers.
     *This is used while sending BLESS vendor specific commands.
     */
    uint32_t is_legacy_bless_controller:1 ;

} wiced_bt_stack_platform_t;

/** SMP encryption  */
typedef struct
{
	/**
	 * Callback function to generate local public key
	 *
	 * Called by stack to generate local public key by processing local private key
	 *
	 * @param[in] private_key   : Pointer to local private key
	 * @param[out] public_key : Pointer to generated public key
	 *
	 * @return : wiced_result_t
	 */
	wiced_result_t (*pf_smp_process_private_key)(const BT_OCTET32 *private_key, wiced_bt_public_key_t *public_key);

	/**
	 * Callback function to generate dhkey
	 *
	 * Called by stack to generate diffie-hellman key using peer public key and local private key
	 *
	 * @param[in] private_key   : Pointer to local private key
	 * @param[in] public_key : Pointer to peer public key
	 * @param[in] dhkey : Pointer to generated dhkey
	 *
	 * @return : wiced_result_t
	 */
	wiced_result_t (*pf_smp_compute_dhkey)(const BT_OCTET32 * private_key, const wiced_bt_public_key_t* public_key, BT_OCTET32 * dhkey);

	/**
	 * Callback function to validate peer public key
	 *
	 * Called by stack to validate peer public key
	 *
	 * @param[in] public_key : Pointer to peer public key
	 *
	 * @return : wiced_result_t
	 */
	wiced_result_t (*pf_smp_ecdh_validate_pub_key)(const wiced_bt_public_key_t* public_key);

	/**
	 * Callback function to get a pointer to active aes_context
	 *
	 * Callback function to get a pointer to active aes_context for encryption operation
	 *
	 * @return : pointer to aes_context
	 */
	aes_context* (*pf_smp_aes_get_context)();

	/**
	 * Callback function to set key for encryption
	 *
	 * Called by stack to provide key details to be used for encryption
	 *
	 * @param[in] key : Pointer to key
	 *
	 * @param[in] keylen : length of the key
	 *
	 * @param[in] p_ctx  : Pointer to active aes_context
	 *
	 * @return : wiced_result_t
	 */
	wiced_result_t (*pf_smp_aes_set_key)(const unsigned char key[], uint8_t keylen, aes_context * p_ctx);

	/**
	 * Callback function to do the encryption
	 *
	 * Called by stack to encrypt given data
	 *
	 * @param[in] in : Pointer to input data to be encrypted. Input data is always 16 bytes in length.
	 *
	 * @param[out] out : Pointer to encrypted data. Encrypted data is always 16 bytes in length.
	 *
	 * @param[in] p_ctx  : Pointer to aes_context
	 *
	 * @return : wiced_result_t
	 */
	wiced_result_t (*pf_smp_aes_encrypt)(const unsigned char in[],
										 unsigned char out[],
										 aes_context * p_ctx);

	/**
	 * Callback function to finish encryption
	 *
	 * Called by stack to finish encrypting a message
	 *
	 * @param[in] p_ctx  : Pointer to aes_context
	 *
	 * @param[in] output_external : Buffer where the output is to be written
	 *
	 * @param[out] output_size : Size of the \p output_external buffer in bytes.
	 *
	 * @param[out] output_length : On success, the number of bytes that make up the returned output
	 *
	 * @return : wiced_result_t
	 */
	wiced_result_t (*pf_smp_aes_finish)(aes_context * p_ctx,
										uint8_t *output_external,
										size_t output_size,
										size_t *output_length);
} wiced_bt_smp_adapter_t;

/** Version information for the local Controller. */
typedef struct
{
    uint8_t     hci_version;    /**< Version of the HCI layer supported by the Controller  */
    uint16_t    hci_revision;   /**< Revision of the HCI implementation in the Controller */
    uint8_t     lmp_version;    /**< Version of the Current LMP supported by the Controller */
    uint16_t    manufacturer;   /**< Company identifier for the manufacturer of the Controller */
    uint16_t    lmp_subversion; /**< Subversion of the Current LMP in the Controller */
} wiced_bt_hci_version_info_t;

/** LE configuration parameters */

typedef struct
{
    uint16_t max_adv_data_len;          /**< Max ADV data length supported in the controller.   */
    uint16_t default_le_tx_data_length; /**< Suggested Max Tx Octets for BLE links */
    uint16_t hcit_ble_acl_data_size;    /**< Max BLE ACL data size for BLE */
    uint16_t hcit_ble_iso_data_size;    /**< Max ISO data size */
    uint8_t hcit_ble_acl_buf_num;       /**< Number of BLE acl buffer size */
    uint8_t hcit_ble_iso_buf_num;       /**< Num ISO buffers */
    uint8_t periodic_adv_list_size;     /**< Periodic ADV list size supported by the controller */
    uint8_t num_sets_supported;         /**< Number of ADV sets supported by the controller.    */
    uint8_t rl_max_entries;             /**< Resolving list max size */
    uint8_t wl_max_entries;             /**< Filter Accept List management */
} wiced_bt_ble_init_data_t;

/** device configuration parameters */

typedef struct
{
    wiced_bt_hci_version_info_t local_version;                             /**< Local Version Information */
    wiced_bt_features_t local_lmp_features[HCI_EXT_FEATURES_PAGE_MAX + 1]; /**< change this to pointer  */
    wiced_bt_features_t local_le_features;               /**< Local LE Supported features mask for the device */
    uint16_t hcit_br_acl_data_size;                         /**< Max ACL data size for BR/EDR  */
    uint8_t hcit_br_acl_buf_num;                         /**< Number of BR/EDR acl buffer size */
    uint8_t supported_cmds[HCI_NUM_SUPP_COMMANDS_BYTES]; /**< change this to pointer  */
} wiced_bt_dev_init_data_t;

/** BTSTACK initialization configuration parameters */
typedef struct
{
    wiced_bt_ble_init_data_t le_info;  /**< LE Read stack setup data */
    wiced_bt_dev_init_data_t dev_info; /**< Device related setup data */
} wiced_bt_stack_init_cmd_data_t;


/**
 * Initialize the platform interfaces, by providing porting functions specific to
 * the underlying platform.
 *
 * @return   <b> WICED_BT_SUCCESS </b> : on success; \n
 *           <b> WICED_BT_ERROR  </b>  : if an error occurred
 */
extern wiced_result_t wiced_bt_stack_platform_initialize(wiced_bt_stack_platform_t * platform_interfaces);

/**
 * Called by the porting layer to process the incoming ACL data received from the
 * remote bluetooth device
 *
 * @param[in] pData  : Pointer to the ACL data to be processed
 * @param[in] length : Length of the ACL data buffer
 *
 * @return    void
 */
extern void wiced_bt_process_acl_data(uint8_t* pData, uint32_t length);

/**
 * Called by the porting layer to process the incoming HCI events from the local
 * bluetooth controller
 *
 * @param[in] pData  : Pointer to the HCI Events to be processed
 * @param[in] length : Length of the event buffer
 * @return    void
 */
extern void wiced_bt_process_hci_events(uint8_t* pData, uint32_t length);

/**
 * Called by the porting layer to process the incoming  SCO data received from the
 * remote bluetooth device
 *
 * @param[in] pData  : Pointer to the SCO data to be processed
  * @param[in] length : Length of the SCO data buffer
 * @return    void
 */

extern void wiced_bt_process_sco_data(uint8_t *pData, uint32_t length);

/**
 * Called by the porting layer to process the incoming  ISOC data received from the
 * remote bluetooth device
 *
 * @param[in] pData  : Pointer to the ISOC data to be processed
 * @param[in] length : Length of the ISOC data buffer
 * @return    void
 */
extern void wiced_bt_process_isoc_data(uint8_t *pData, uint32_t length);

 /**
 * Called by the porting layer on expiry of the timer to process pending timers
 *
 * @return    void
 */
extern void wiced_bt_process_timer(void);


/**
 * Called by the lower layer transport driver to restart sending ACL data to the controller
 * Note: Porting layer API.
 *     This API is expected to be invoked by the lower layer transport driver, to restart
 *     transfers from the stack to the controller.
 *     The lower tx layer is expected to have space for atleast one complete ACL buffer
 *     Typically used in cases where the lower Tx has lesser number of buffers than allowed by controller
 */
extern void wiced_bt_stack_indicate_lower_tx_complete(void);

/**
 * Called by the porting layer to complete/continue the reset process
 * Typically called after downloading firmware patches to the controller
 *
 * @return    void
 */
extern void wiced_bt_continue_reset(void);

/**
* Set the stack config. Invoked by the porting layer
*
* @param[in] p_bt_new_cfg_settings : Stack configuration settings
*
* @return   0 if there is any error in the configuration otherwise the dynamic
*           memory size requirements of the stack for the configuration.
*
*
*/
extern uint32_t wiced_bt_set_stack_config(const wiced_bt_cfg_settings_t* p_bt_new_cfg_settings);

/**
* Get the stack config set by the application
*
* @return pointer to application config settings
*
*/
extern const wiced_bt_cfg_settings_t *wiced_bt_get_stack_config(void);

/**
* Function prototype for the post Stack Init Callback.
*/
typedef void (*wiced_bt_internal_post_stack_init_cb)(void);


/**
* Function prototype for the HCI event monitor function that the application may suppply.
* The application MUST return TRUE if the it handled the event and does not want the stack to
* process the event. If the application returns FALSE, the stack will process the event.
*/
typedef wiced_bool_t (*wiced_bt_internal_stack_evt_handler_cb)(uint8_t* p_event);

/**
* Internal stack init
*
* @param[in] mgmt_cback : Application Bluetooth Management callback
* @param[in] post_stack_cb : Internal post stack init callback
* @param[in] evt_handler_cb : Internal stack event handler
*
* @return    Dynamic memory size requirements of the stack for the configuration
*
*/
void wiced_bt_stack_init_internal(wiced_bt_management_cback_t mgmt_cback,
    wiced_bt_internal_post_stack_init_cb post_stack_cb,
    wiced_bt_internal_stack_evt_handler_cb evt_handler_cb);

/**
 * This function blocks until all de-initialisation procedures are complete.
 * It is recommended that the application disconnect any outstanding connections prior to invoking this function.
 *
 * @return    None
 */
void wiced_bt_stack_shutdown(void);

/**
* Set the initial setup data to configure the stack. Invoked by the porting layer
*
* @param[in] p_init_setup_data : intial setup data
*
*@note Call this API if want to reduce the stack size.
* This API needs to be invoked prior to \ref wiced_bt_stack_init_internal API.
* Do not call API \ref wiced_bt_enable_stack_default_flow if calling this API.
*
* @return   wiced_result_t
*           WICED_SUCCESS if Success.
*           WICED_BADARG if called with NULL arguement.
*/

wiced_result_t wiced_bt_set_stack_initial_setup_data(wiced_bt_stack_init_cmd_data_t *p_init_setup_data);

/**
 * Called by the porting layer to enable the stack to send the initial setup
 * commands(refer Version 5.4 | Vol 6, Part D 2.1 INITIAL SETUP) to the controller.
 *
 *@note Call this API if required to get the intial setup information from the controller.
* Do not call API \ref wiced_bt_set_stack_initial_setup_data if calling this API.\n
* This API needs to be invoked prior to \ref wiced_bt_stack_init_internal API.
 * @return    void
 */

void wiced_bt_enable_stack_default_flow(void);

/**
 * Called by the porting layer to set platform specific or custom SMP encryption
 *
 * @param[in] p_smp_adapter : SMP Adaptor configurations and callbacks
 *
 * @return   <b> WICED_SUCCESS </b> : on success; \n
 *           <b> WICED_ERROR  </b>  : if an error occurred
 *
 */
wiced_result_t wiced_bt_platform_set_smp_adapter(const wiced_bt_smp_adapter_t *p_smp_adapter);

/**
 * Called by the porting layer to set default SMP encryption
 *
 * @return   <b> WICED_SUCCESS </b> : on success; \n
 *           <b> WICED_ERROR  </b>  : if an error occurred
 */
wiced_result_t wiced_bt_set_default_smp_adapter(void);

/**
* Called by porting layer to get the stored local keys from the app.
* If application returns WICED_SUCCESS, the keys are written to the stack to start
* Controller based address resolution with \ref wiced_ble_init_ctlr_private_addr_generation or
* Host based address resolution with \ref wiced_ble_init_host_private_addr_generation
* Else, \ref wiced_ble_create_local_identity_keys call needs to be invoked
*/
wiced_result_t wiced_ble_read_local_identity_keys_from_app(wiced_bt_local_identity_keys_t *p_local_keys);


/**
 * Create new local keys to be used for device privacy
 * The new local keys generated are returned to the application in \ref BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT
 * The porting layer can start,
* Controller based address resolution with \ref wiced_ble_init_ctlr_private_addr_generation or
* Host based address resolution with \ref wiced_ble_init_host_private_addr_generation
*
* @return  wiced_result_t
 *
 */
wiced_result_t wiced_ble_create_local_identity_keys(void);

/**
 * API to write back saved local keys and init host based address generation.
 * The local keys sent down in this call are used to generate the local RPA (Resolvable
 * Private Address). The RPA generated is required to be refreshed periodically by the host based on the
 * suggested \ref wiced_bt_cfg_ble_t.rpa_refresh_timeout member of wiced_bt_cfg_settings_t.p_ble_cfg
 *
 * @param[in] p_local_keys : device local keys generated from a previous call to
 *  \ref wiced_ble_create_local_identity_keys
 *
 * @return  wiced_result_t
 *
 */
wiced_result_t wiced_ble_init_host_private_addr_generation(wiced_bt_local_identity_keys_t *p_local_keys);

/**
 * API to write back saved local keys and controller based address generation.
 * The local keys sent down in this call are used to generate the local RPA (Resolvable
 * Private Address). The RPA generated is required to be refreshed periodically by the controller based on the
 * suggested \ref wiced_bt_cfg_ble_t.rpa_refresh_timeout member of wiced_bt_cfg_settings_t.p_ble_cfg
 *
 * @param[in] p_local_keys : device local keys generated from a previous call to
 * \ref wiced_ble_create_local_identity_keys
 *
 * @return  wiced_result_t
 *
 */
wiced_result_t wiced_ble_init_ctlr_private_addr_generation(wiced_bt_local_identity_keys_t *p_local_keys);

/**
* Helper API to issue BTM_ENABLED_EVT
* @param p_app_management_callback Management callback set by the application
*/
wiced_bool_t wiced_bt_issue_btm_enabled_evt(wiced_bt_management_cback_t p_app_management_callback);

/**
* Helper API to get the startup flags
* @return uint32_t value
*/
uint32_t wiced_bt_get_btm_startup_flags(void);

/**@} wiced_bt_platform_group */

#endif //__WICED_BT_STACK_PLATFORM_H__
