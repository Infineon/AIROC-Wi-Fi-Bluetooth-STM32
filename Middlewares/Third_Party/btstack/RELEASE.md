# Bluetooth Stack Library

## Known Issues/Limitations
Following are the limitations when using host based address resolution (only applies to the 4343 controller chip):
    Since whitelists are implemented purely in the controller, advertisement whitelist filtering will not work if a peer is using Resolvable Private Addresses (since the address cannot be resolved in the controller). To be safe, applications should not use the whitelist at all, but at a minimum not use it if paired with a private peer device.
    There is a very small window when the host stack gets an ADV, resolves the address and tries to connect to the private address, but during that time the peripheral has changed the address. This might very occasionally cause a connection attempt to fail.
    If the device is acting as a central it should not enable privacy since if a peripheral sends a directed connectable ADV, the controller would not be able to match the RPA and the connection will fail.

## Changelog
## V4.1.4
### BTSTACK4.1.4 contains following updates and fixes -
 - Bug fix to restart ATT indication and request queues on GATT TX complete
 - Synchronize updating controller generated RPA with changes in the advertisement data
 - Documentation update of wiced_bt_ble_encrypt_adv_packet and wiced_bt_ble_decrypt_adv_packet APIs
 - Added Support for SWB (LC3) in SCO
    - In  wiced_bt_sco_params_t, use_wbs field changed from bool to uint8. This allows SCO to now support Narrow band, wide band as well as super wide band
    - For newly added LC3, the coding format is set as transparent
 - A2DP 1.4 support
    - MPEG 2,4 parser updated
        - DRC is optional feature in MPEG 2,4. Current we do not support DRC
    - Added parser for MPEG-D USAC
    - Added mdu to wiced_bt_a2dp_codec_info_t

## V4.1.3
### Fixes in BTSTACK4.1.3
 - Fix to not update the random address if rpa_refresh_timeout is set to 0

## V4.1.2
### Fixes in BTSTACK4.1.2
 - Fix for PSOC + BLESS controller not responding to scan request from a private device

## V4.1.0
### BTSTACK4.1 contains following enhancements and fixes -
 - Creation of stack libraries using LLVM toolchain
### Advertisement and scan API Updates
  - wiced_ble_ext_scan_set_config is removed. Use **wiced_ble_ext_scan_register_cb** to register extended scan result callback.
  - Optionally use **wiced_ble_ext_scan_configure_reassembly** to reassemble partial extended adv data reports
  - Structure member data_len of **wiced_ble_padv_rsp_report_event_data_t** is renamed to data_length
  - Structure member max_periodic_adv_len of **wiced_ble_padv_create_sync_params_t** is removed. Optionally use **wiced_ble_padv_alloc_segment_assembler** to reassemble partial periodic adv data segments
  - Structure member data of **wiced_ble_padv_report_event_data_t** is renamed to p_data
### Privacy APIs
  - Deprecated API wiced_bt_dev_delete_bonded_device. **Stack does not store any bonding information**. For applications which use BLE mode, invoke **wiced_bt_dev_remove_device_from_address_resolution_db** to remove the device from the resolving list.
### Background connection APIs
  - Background connections can be initiated by central devices by adding peripheral device addresses to the filter list.
  - To add to the filter list use **wiced_ble_add_to_filter_accept_list**
  - To remove a device from the filter list use **wiced_ble_remove_from_filter_accept_list**
  - Initiate a background connection with the filter list use **wiced_ble_legacy_create_connection** (legacy connection), **wiced_ble_ext_create_connection** (extended)
  - The following APIs are no longer supported
    - wiced_bt_ble_set_background_connection_type
    - wiced_bt_ble_update_background_connection_device
    - wiced_bt_ble_update_advertising_filter_accept_list
    - wiced_bt_ble_update_scanner_filter_list
    - wiced_bt_ble_clear_filter_accept_list
    - wiced_bt_ble_get_filter_accept_list_size
    - wiced_bt_gatt_listen
### wiced_ble_key_distribution_t
  - Corrected the enumerations for **wiced_ble_key_distribution_e**
### Setting the local device address
- Use **wiced_bt_set_local_bdaddr** to setup the **BLE_ADDR_PUBLIC** or **BLE_ADDR_RANDOM** address.
- If BLE_ADDR_RANDOM is used, ensure that the MSB bits are set to b'11' for static addresses and b'00' for non-resolvable addresses
- Addresses set using this API are used during scanning, advertising, create connection, periodic advertising
### Random address management
- BTSTACK4.1 adds APIs to allow application control while creating IRK (Identity Resolving Key).
- Use **wiced_ble_create_local_identity_keys** to create new keys, which are returned to the app in **BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT**
- In case the application already has previously created local keys, use
  - **wiced_ble_init_host_private_addr_generation** for controllers which do not support controller based privacy
  - **wiced_ble_init_ctlr_private_addr_generation** for controllers which support controller based privacy
- These APIs are invoked in the porting layer, but can be called from the application by setting ```-DENABLE_CREATE_LOCAL_KEYS=0``` in the application makefile


## V4.0.0
BTSTACK4.0 is BT6.0 certified. DN: [Q359398](https://qualification.bluetooth.com/ListingDetails/291872)

BTSTACK4.0 contains the following enhancements -
 - Major update for Extended/Periodic/PAWR advertising and scanning APIs
 - Major update for ISOC APIs
 - API change \ref wiced_bt_l2cap_update_ble_conn_params
 - Legacy advertising, scanning, connection APIs are unaffected by this release.
 - Applications are expected to either use legacy or extended advertisement, scan APIs
 - To create a legacy LE connection use \ref wiced_bt_gatt_le_connect
 - To create a extended LE connection use \ref wiced_ble_ext_create_connection
 - To detect errors related to mixing of legacy and extended APIs add the following to the application makefile and fix the reported warnings
   - Using only legacy APIs
     ```
     DEFINES+=WICED_BLE_ENABLE_LEGACY_EXTENDED_API_ERROR_CHECK
     DEFINES+=WICED_BLE_ENABLE_EXTENDED_ADV_API=0
     DEFINES+=WICED_BLE_ENABLE_LEGACY_ADV_API=1
     ```
   - Using only extended APIs
     ```
     DEFINES+=WICED_BLE_ENABLE_LEGACY_EXTENDED_API_ERROR_CHECK
     DEFINES+=WICED_BLE_ENABLE_EXTENDED_ADV_API=1
     DEFINES+=WICED_BLE_ENABLE_LEGACY_ADV_API=0
     ```

## V3.9.2
BTSTACK3.9.2 is a patch release with following enhancements -
 - Modified wiced_bt_dev_read_tx_power() to send HCI_Read_Transmit_Power_Level HCI command

## V3.9.1
BTSTACK3.9.1 is a patch release with following enhancements -
 - Improved Doxygen API content and formatting

## V3.9.0
BTSTACK3.9 contains following enhancements and fixes -
 - Added a new wiced API, wiced_bt_set_transmit_power_range() (See API documentation for details)
 - Added a fix to disallow signed write command on EATT channel as per BT Core Spec

## V3.8.2
BTSTACK3.8.2 is a patch release with following enhancement -
 - Added a new wiced API, wiced_bt_ble_set_data_packet_length(), to set maximum transmission payload size and maximum packet transmission time to be used for LL DATA PDUs on a given connection

## V3.8.1
BTSTACK3.8.1 is a patch release with following enhancements and fixes -
 - Added new wiced API, wiced_ble_isoc_read_tx_sync(), to send HCI_LE_Read_ISO_TX_Sync HCI command
 - Fixed the issue in AIROC™ BT/BLE stack deinitialization
 - Properly set maximum transmission payload size to be used for LL data PDUs
 - Changes to stop PAwR extended connection when Enhanced connection complete error is received
 - Corrected enum value of WICED_BLE_ISOC_DPD_INPUT_OUTPUT_BIT
 - Changes for optimization of acl link allocation in stack
 - Updated wiced_memory_get_free_bytes() and wiced_bt_ble_cache_ext_conn_config() API documentation
 - Added support to configure input and output coding format for SCO connection using newly added wiced_bt_read_esco_parameters() and wiced_bt_config_esco_parameters() APIs

## V3.8.0
BTSTACK3.8 is BT5.4 certified. QDID: 219623.
This release contains bug fixes and and an enhancement listed below.
 - Support for PAwR added, refer to wiced_bt_ble.h for details of interfaces
 - Added utility functions for Advertisement encryption and decryption, see wiced_bt_ble_encrypt_adv_packet and wiced_bt_ble_decrypt_adv_packet
 - Implementation of BT Core spec erratum 22240
 - Added API wiced_bt_app_serialize_function that can be called by applications to serialize the execution of an application function in the BT stack context
 - Updates to optimize the code size for Dual-mode stack
 - Support for connection subrate request procedure, added the API wiced_bt_l2cap_subrate_request and related events
 - Added API wiced_bt_ble_notify_on_device_address_change to notify the application on device address change via BTM_BLE_DEVICE_ADDRESS_UPDATE_EVENT event.
 - Other bug fixes and documentation enhancements.

## V3.7.2
BTSTACK3.7.2 is a patch release with following fix -
 - Fix an issue where GATT congestion release notification was not sent by Server side

## V3.7.1
BTSTACK3.7.1 is a patch release with following enhancements -
 - Add v2 support for HCI_LE_Set_Extended_Advertising_Parameters present in BT Core Spec 5.4

## V3.7.0
BTSTACK3.7 contains following enhancements and fixes -
 - Enhancements for lowering stack library code size. Only the part of the stack code that is used by application is included in the linked image.
   - Applications which do not create GATT/ACL connections or those which do not need SMP may override the default initializations done in the stack by defining the macro **DISABLE_DEFAULT_BTSTACK_INIT** in the applicatin Makefile.
       #Set DISABLE_DEFAULT_BTSTACK_INIT=1
       DEFINES+=DISABLE_DEFAULT_BTSTACK_INIT=1
   - GATT Server applications which need to implement GATT Robust Caching will need to invoke **wiced_bt_gatt_server_enable_caching** in the **BTM_ENABLED_EVT**
   - GATT applications work with signed data will need to invoke **wiced_bt_gatt_enable_signing** in the **BTM_ENABLED_EVT**
 - New APIs added (See API documentation for details)
   - wiced_bt_gatt_server_enable_caching
   - wiced_bt_gatt_server_enable_signing
 - Fix to Correct macro names used for S=2 and S=8 coding
 - Fix for A2DP/SNK/AVP/BI-20-C, A2DP/SNK/AVP/BI-10-C failures
 - Fix issue setting default Tx power with CY BLESS controller.
 - Updated document for following APIs -
   - wiced_bt_l2cap_enable_update_ble_conn_params
   - wiced_bt_avdt_write_req
   - wiced_bt_l2cap_update_ble_conn_params
 - L2CAP/ECFC/BI-08-C is under discussion/debug with PTS (see TSE#17703).

## V3.6.1
BTSTACK3.6.1 is a patch release with the following fixes -
 - Fix to check for ISOC support in controller before issuing read_buf_size_v2 HCI command.
 - Fix to mask Enhanced Connection Complete Event v2 HCI event in the HCI event mask as this is not handled in stack.

## V3.6.0
BTSTACK3.6 is BT5.3 certified. QDID: [196979](https://launchstudio.bluetooth.com/ListingDetails/164219).
Below test cases fail and are under discussion/debug with PTS.
 - L2CAP/COS/CED/BI-02-C, L2CAP/ECFC/BI-08-C, refer to TSE#17703
 - L2CAP/ECFC/BI-09-C, refer to TSE#18208

This release contains bug fixes and and an enhancement listed below.
 - Support HCI_LE_Read_buffer_size [v2].
 - Fix to return correct BD Address in the encryption status event callback.
 - Documentation update for wiced_bt_rfcomm_read_data().
 - The known issue from BTSTACK3.5 related to the handling of directed advertisement flags in the LE Set Extended Adv commands, is handled by changes in the application to set the correct address type to start directed adv.

## V3.5.0
BTSTACK3.5 release contains bug fixes and and an enhancement listed below.
 - Updated API description for wiced_bt_gatt_client_send_read_by_type().
 - Fix to issue where setting adv parameters was allowing incorrect adv type to be set for directed advertisements.
 - Fix for an issue in wiced_bt_delete_heap().
 - Fixed an issue to return correct connection handle when there is an ISOC CIS disconnection and added reason field.
 - Fix to avoid clearing of CIS data path info from stack during CIS disconnection.
 - Added sections in the code for aiding placement of btstack data path in the linker file.
 - Creation of stack libraries using ARM and IAR toolchains.

### Known issues in V3.5.0
 - The issue is related to the handling of directed advertisement flags in the LE Set Extended Adv commands. Android expects the device to send directed advertisement packets with its
   Random Private Address, whereas the device issues the same with the remotes public address.
   The workaround for the issue is to use undirected advertisements for applications to continue with their development when using a controller that supports Extended Advertisements.

## V3.4.0
BTSTACK3.4 release contains bug fixes and and an enhancement listed below.
 - Stack now conforms to PTS8.1.1. Below are the test cases that fail and are under discussion/debug with PTS,
   - TSE#12608 --> GATT/CL/GPM/BV-12-C
   - TSE#17703 --> L2CAP/COS/CED/BI-02-C, L2CAP/ECFC/BI-08-C
   - TSE#17581 --> L2CAP/ECFC/BV-38-C
   - TSE#18208 --> L2CAP/ECFC/BI-09-C
 - Added data type and macros for HCI roles. See wiced_bt_dev.h
 - Added new event BTM_BLE_DATA_LENGTH_UPDATE_EVENT to notify change in the data length and timeout configured for Rx and Tx on the BLE link. See wiced_bt_dev.h.
 - Added new API wiced_bt_dev_sec_pair_without_bonding() for pairing but without bonding. See wiced_bt_dev.h.
 - Added new APIs wiced_bt_dev_is_address_resolution_enabled and wiced_bt_dev_is_privacy_supported to query the support status for controller based address resolution and device privacy mode, respectively. See wiced_bt_dev_utils.h.
 - GATT status code updated with error code for improper configuration of CCCD (WICED_BT_GATT_CCCD_IMPROPER_CONFIGURED). See wiced_bt_gatt.h.
 - Added APIs to get heap and pool statistics in wiced_memory.h.


## V3.3.0
BTSTACK3.3 release contains bug fixes and and an enhancement listed below.
 - Fix for GAP conformance test case GAP/SEC/SEM/BV-14-C.
 - Fix for an issue where Maximum possible multi-adv connections value was wrong and adv instance creation was failing.
 - Update the documentation for wiced_bt_ble_security_grant() API.
 - Removed unused platform API wiced_bt_stack_platform_deinit().
 - Added memory management APIs to allocate free long term memory blocks, wiced_memory_alloc_long_term_mem_block() and wiced_memory_free_long_term_mem_block() in wiced_memory.h


## V3.2.0
BTSTACK3.2 contains below updates to the Bluetooth host stack:
 - New APIs added
   - wiced_bt_l2cap_deregister_fixed_channel() API to deregister a fixed L2CAP channel (see also wiced_bt_l2cap_register_fixed_channel().
   - wiced_bt_dev_update_debug_trace_mode() to enable/disable debug traces.
   - APIs for ECRB (Enhanced Credit Based L2CAP) channels management.
   - wiced_ble_private_device_address_resolution() that checks if a private address is resolvable with the given IRK.
   - wiced_bt_dev_get_acl_conn_handle() to get connection handle given a peer BD Address.
 - Bug fixes and other updates
   - The stack internally sets the default BLE data length to 251 at start-up, i.e., enables BLE Data Length Extension. Also, pairing is disabled by default so that application can enable using wiced_bt_set_pairable_mode(),
   - Fix for crash during LE-COC data transfer.
   - Fix for issue where extended advertisement was enabled before random address was set.
   - Fix to update just_works field correctly in BTM_USER_CONFIRMATION_REQUEST_EVT.

### Known issues in V3.2.0
 - Documentation bug - in the documentation the API wiced_bt_ble_security_grant(), possible values for argument 'res' is mentioned as BTM_SUCCESS, BTM_MODE_UNSUPPORTED and BTM_REPEATED_ATTEMPTS. Please read them as WICED_BT_SUCCESS, WICED_BT_UNSUPPORTED and WICED_BT_REPEATED_ATTEMPTS, respectively.

## V3.1.0
BTSTACK3.1 is BT5.2 certified. Certification includes EATT and ISOC features, QDID: [172247](https://launchstudio.bluetooth.com/ListingDetails/134246).
 - APIs added in this release:
   - API to get number of GATT packets in the tx queue: int wiced_bt_gatt_get_num_queued_tx_packets()
   - API to get num packets in L2C queue: wiced_bt_l2cap_get_num_queued_tx_packets()
   - API for checkecking whether a connection id is still in operation/connected: wiced_bt_gatt_status_t wiced_bt_gatt_validate_conn_id()
 - Minor documentation updates are done including the issue in wiced_bt_gatt.h using the term clcb_operation instead of operation that was present in BTSTACK3.0.
 - Bug fixes
   - In GATT Disconnection event connection id always set to 0x8000.
   - Fix crash in API wiced_bt_remove_from_queue when dequeuing from an empty queue.
   - Fix for incorrect discovery_type value in GATT_DISCOVERY_CPLT_EVT.
   - Fix to allow back to back operations on event complete in gatt read, read multiple, read_by_type_rsp.

## V3.0.0
This release contains major updates BTSTACK interfaces requiring modifications in existing applications in order for them to work with BTSTACK3.0. Please see the **[Migration Guide](https://infineon.github.io/btstack/BTSTACK_2.0_to_3.0_API_Migration_Guide.htm)** for more details on how to migrate your application written for BTSTACK1.X or BTSTACK2.0 to BTSTACK3.0.
 - Stack configuration parameters (wiced_bt_cfg_settings_t) are updated for better readability and to remove parameters that are not necessary, providing APIs where requred. Stack APIs such as GATT profile APIs have been modified for ease of use.
 - BT SIG has updated the naming of a number of entities (e.g., roles of bluetooth devices), the stack interfaces and internal code has been updated accordingly.
 - Stack Config Structure - Changes to stack config settings structure has been done to keep the config elements to a minimum and to be able to seperate BLE configuration elements from BR/EDR configuration elements for ease of use and readability. Parameters that can't be modified using config structure are allowed to be modified using APIs where required.
As part of this change, the wiced_bt_cfg_settings_t and associated structures are changed.
 - GATT APIs - GATT APIs have been updated to simplify for ease of use of the APIs.
   - Updated APIs let application determine and allocate the memory required for GATT data transfers.
   - Provides a single interface to respond to a request by eliminating multiple paths that was present earlier (by API call, implicit response by return in callback).
   - On the GATT server, applications are expected to invoke specific response APIs or the error response to respond to incoming GATT requests
 - Better Congestion handling
 - Improved overall memory usage for GATT applications

### Known issues in V3.0.0
- In GATT Disconnection event connection id always set to 0x8000.
  Workaround to identify the terminated connection is to check the BD ADDR in the event data.
- Documentation - in wiced_bt_gatt.h, the term clcb_operation should be read as 'operation' in the following structure definition
  - wiced_bt_gatt_discovery_complete_t, in documentation of member status.
  - wiced_bt_gatt_operation_complete_t, in documentation of member op.

## V2.0.0
 - The BT Host Stack is BT5.2 certified, QDID: 160340.
 - Extended Advertisement feature is supported.
 - EATT is supported.
 - Documentation improvements.
 - Documentation fixes.

## V1.5.0
This release is an update over BTSTACK1.4 release.
 - New API in wiced_bt_stack_Platform.h, wiced_bt_stack_indicate_lower_tx_complete() has been added. This is called by the lower layer transport driver to restart sending ACL data to the controller when buffers are agian available with the lower layer transport driver after having run out.
 - The stack now supports Host based address resolution when using controllers that do not support address resolution.
 - API wiced_bt_stack_shutdown in wiced_bt_stack_platform.h, to be called by porting layer to clean-up stack context data when wiced_bt_stack_deinit() is called by application.


## V1.4.0
BTSTACK v1.40 has bug fixes and enhancements.
 - New interface in wiced_bt_stack_platform.h, which defines porting layer facing interfaces, to disable stack traces and identify trace type (DEBUG, ERROR, etc.,).
 - New API to get expected dynamic memory required in stack based on a given wiced_bt_cfg_settings_t is added (wiced_bt_stack_get_dynamic_memory_size_for_config).
- GATT Caching:
  - Perform OTFK (On-The-Fly_Key hash generation for the database. This reduces the intermediate memory requirements for calculating the hash)
  - Support for split database added
- wiced_bt_delete_heap API to delete application created heaps added

## V1.3.0
- Initial public release of Bluetooth Host stack
