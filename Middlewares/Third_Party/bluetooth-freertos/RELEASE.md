### AIROC&trade; BT/BLE Host Stack solution
AIROC&trade; BT/BLE stack provides Bluetooth&reg; functionality with high performance and less resource consumption. It targets on IoT products, especially for embedded devices.

### What's Included?
This release of AIROC&trade; BT/BLE stack solution package includes as following:
* Infineon Bluetooth&reg; chip firmware (CYW4343W and CYW43012)
* WICED BLE stack library and API headers (by btstack.lib)
* Platform and Operation System porting layer

#### v3.4.0
* Support firmware patch is stored in external flash

#### v3.3.0
* Infineon Bluetooth&reg; chip 43439 support(Murata EA 37.4MHz)

#### v3.2.0
* Enable BTSpy trace utility
* Infineon Bluetooth&reg; chip 43439 support(26MHz)
* Support HAL version v2.0 which API changed

#### v3.1.0
* Firmware folder restructure
* Fix no lptimer callback after mtb-hal-cat1 v1.6

#### v3.0.0
* Support Bluetooth&reg; stack version latest-v3.x

#### v2.0.0
* Software Thread Architecture change (HCI_TX and HCI_RX tasks are introduced, replace HCI and BT tasks in earlier versions)
* Infineon Bluetooth&reg; chip 4373 support
* Update firmware to support Mesh
* Support Bluetooth&reg; stack version v2.0 which API changed

#### v1.3.0
* Fixed Co-Existence Security Vulnerabilities
* Support WICED Bluetooth&reg; stack version 1.5

#### v1.2.0
* Support WICED Bluetooth&reg; stack version 1.4
* Infineon Bluetooth&reg; chip 43438 support
* CLib FreeRTOS support

#### v1.1.0
* Infineon Bluetooth&reg; chip 43012 support
* Infineon ModusToolbox&trade; BT LPA support
* btstack.lib is used to fetch wiced bt/ble libraries/headers from btstack repo directly

#### v1.0.0
* Initial release supporting mbed & FreeRTOS

### Supported Software and Tools
This version of Infineon BT/BLE stack API was validated for compatibility with the following Software and Tools:

| Software and Tools                        | Version |
| :---                                      | :----:  |
| ModusToolbox™ Software Environment        | 2.4.0   |
| GCC Compiler                              | 10.3.1  |
| IAR Compiler                              | 8.4     |
| ARM Compiler                              | 6.11    |
| mbed OS                                   | 5.15    |
| FreeRTOS                                  | 10.4.3  |

---
© Infineon Technologies, 2019.