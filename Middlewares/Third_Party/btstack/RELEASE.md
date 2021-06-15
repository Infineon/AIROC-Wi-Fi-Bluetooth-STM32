# Bluetooth Stack Library

## Known Issues/Limitations
Following are the limitations when using host based address resolution (only applies to the 4343 controller chip):
    Since whitelists are implemented purely in the controller, advertisement whitelist filtering will not work if a peer is using Resolvable Private Addresses (since the address cannot be resolved in the controller). To be safe, applications should not use the whitelist at all, but at a minimum not use it if paired with a private peer device.
    There is a very small window when the host stack gets an ADV, resolves the address and tries to connect to the private address, but during that time the peripheral has changed the address. This might very occasionally cause a connection attempt to fail.
    If the device is acting as a central it should not enable privacy since if a peripheral sends a directed connectable ADV, the controller would not be able to match the RPA and the connection will fail.

### Known issues in V3.0.0
- In GATT Disconnection event connection id always set to 0x8000.
  Workaround to identify the terminated connection is to check the BD ADDR in the event data.
- Documentation - in wiced_bt_gatt.h, the term clcb_operation should be read as 'operation' in the following structure definition
  - wiced_bt_gatt_discovery_complete_t, in documentation of member status.
  - wiced_bt_gatt_operation_complete_t, in documentation of member op.

## Changelog

### V3.0.0
This release contains major updates BTSTACK interfaces requiring modifications in existing applications in order for them to work with BTSTACK3.0. Please see the **[Migration Guide](https://cypresssemiconductorco.github.io/btstack/BTSTACK_2.0_to_3.0_API_Migration_Guide.htm)** for more details on how to migrate your application written for BTSTACK1.X or BTSTACK2.0 to BTSTACK3.0.
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

### V2.0.0
 - The BT Host Stack is BT5.2 certified, QDID: 160340.
 - Extended Advertisement feature is supported.
 - EATT is supported.
 - Documentation improvements.
 - Documentation fixes.

### V1.5.0
This release is an update over BTSTACK1.4 release.
 - New API in wiced_bt_stack_Platform.h, wiced_bt_stack_indicate_lower_tx_complete() has been added. This is called by the lower layer transport driver to restart sending ACL data to the controller when buffers are agian available with the lower layer transport driver after having run out.
 - The stack now supports Host based address resolution when using controllers that do not support address resolution.
 - API wiced_bt_stack_shutdown in wiced_bt_stack_platform.h, to be called by porting layer to clean-up stack context data when wiced_bt_stack_deinit() is called by application.


### V1.4.0
BTSTACK v1.40 has bug fixes and enhancements.
 - New interface in wiced_bt_stack_platform.h, which defines porting layer facing interfaces, to disable stack traces and identify trace type (DEBUG, ERROR, etc.,).
 - New API to get expected dynamic memory required in stack based on a given wiced_bt_cfg_settings_t is added (wiced_bt_stack_get_dynamic_memory_size_for_config).
- GATT Caching:
  - Perform OTFK (On-The-Fly_Key hash generation for the database. This reduces the intermediate memory requirements for calculating the hash)
  - Support for split database added
- wiced_bt_delete_heap API to delete application created heaps added

### V1.3.0
- Initial public release of Bluetooth Host stack
