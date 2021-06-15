### Cypress WICED BT/BLE Host Stack solution
Cypress WICED BT/BLE stack provides Bluetooth functionality with high performance and less resource consumption. It targets on IoT products, especially for embedded devices.

### What's Included?
This release of Cypress WICED BT/BLE stack solution package includes as following:
* Cypress Bluetotoh chip firmware (CYW4343W and CYW43012)
* WICED BLE stack library and API headers (by btstack.lib)
* Platform and Operation System porting layer

#### v3.0.0
* Support Bluetooth stack version latest-v3.x

#### v2.0.0
* Software Thread Architecture change (HCI_TX and HCI_RX tasks are introduced, replace HCI and BT tasks in earlier versions)
* Cypress Bluetooth chip 4373 support
* Update firmware to support Mesh
* Support Bluetooth stack version v2.0 which API changed

#### v1.3.0
* Fixed Co-Existence Security Vulnerabilities
* Support WICED Bluetooth stack version 1.5

#### v1.2.0
* Support WICED Bluetooth stack version 1.4
* Cypress Bluetooth chip 43438 support
* CLib FreeRTOS support

#### v1.1.0
* Cypress Bluetooth chip 43012 support
* Cypress ModusToolBox BT LPA support
* btstack.lib is used to fetch wiced bt/ble libraries/headers from btstack repo directly

#### v1.0.0
* Initial release supporting mbed & FreeRTOS

### Supported Software and Tools
This version of Cypress BT/BLE stack API was validated for compatibility with the following Software and Tools:

| Software and Tools                        | Version |
| :---                                      | :----:  |
| ModusToolbox Software Environment         | 2.0     |
| GCC Compiler                              | 7.4     |
| IAR Compiler                              | 8.32    |
| ARM Compiler                              | 6.11    |
| mbed OS                                   | 5.15    |
| FreeRTOS                                  | 10.0.1  |

---
© Cypress Semiconductor Corporation, 2019-2020.