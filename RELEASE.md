# STM32 Connectivity Expansion Pack 1.2.0 Release Notes
The STM32 Connectivity Expansion Pack is an extension of the CMSIS-Pack standard established by Arm.
The pack is compliant with the full CMSIS-Pack standard, with additional requirements/restrictions
on the final pack to meet the STM standard.

This pack uses libraries from the ModusToolbox environment.

## What's Included?
* STM32L5, Threadx/NetxDuo, Manufacturing tools Support Release

## What Changed?

 v1.2.0
* Added ThreadX/NetxDuo support
* Added STM32L5 support
* Added WiFi and BLE/BT Manufacturing tools
  - Bluetooth Manufacturing Test Application for FreeRTOS (bt_mfg_tester)
  - WLAN manufacturing test application for FreeRTOS (wifi_mfg_tester)
  - Tester - Wi-Fi Bluetooth Console (wifi_bt_tester)
* PAL improvements and fixes

## Supported STM32 Boards and MCU
*  STM32L562E-DK kit and STM32L5xx
*  STM32H747I-DISCO Discovery kit and STM32H7xx

## Supported Connectivity Modules

Infineon's CYW43xxx Wi-Fi-BT combo chip family:
*  CYW43012
*  CYW43439 / CYW43438 / CYW4343W
*  CYW4373 / CYW4373/E


## Example apps inside the Pack

### Wi-Fi Scan Example
This example initializes the Wi-Fi device and starts a Wi-Fi scan without any filter and prints the
results on the serial terminal.

Refer to Projects/STM32H747I-DISCO/Applications/wifi_scan/readme.txt for more details

### Wi-Fi Onboarding with Bluetooth LE Example
This example demonstrates a simultaneous usage of Wi-Fi and BLE functionality of CYW43xxx combo
devices. It uses BLE on the combo device to help connect the Wi-Fi to the AP.

Refer to Projects/STM32H747I-DISCO/Applications/ble_wifi_onboarding/readme.txt for more details

### Azure RTOS NetXDuo Wi-Fi UDP echo server
This application provides an example of Azure RTOS/NetXDuo stack usage. It shows how to develop a 
NetX UDP server to communicate with a remote client using the NetX UDP socket API.

## Compatible Software

|  Software                    | Version   |
| ---------------------------- | --------- |
| STM32 CubeMX                 |  6.5.0    |
| STM32 CubeIDE                |  1.9.0    |
| IAR Embedded Workbench IDE   |  8.50.4   |

### Supported Compiler Version

| Compiler                     | Version |
| :---                         | :----:  |
| GCC Compiler                 | 7.3     |
| IAR Compiler                 | 8.50.4  |


## Change History

| Version | Changes                                                             | Reason for Change                     |
| :----:  | :---                                                                | :----                                 |
| 1.2.0   | Added STM32L5 support.                                              | Add support of new STM32 series       |
|         | Added ThreadX/NetxDuo support.                                      | Add support for Azure RTOS            |
|         | Added WiFi and BLE Manufacturing tools support.                     | Manufacturing tools (Iperf and mfgtest) support. Documentation updates. |
| 1.1.0   | New release comprising BLE/BT support and various PAL improvements. | BLE/BT support and code example added |
| 1.0.1   | Patch release.                                                      | SDIO Buffer overrun fix.              |
| 1.0.0   | The initial version                                                 |                                       |


## Future release

* Add support of STM32U5. 


### More information
* [README.md](./README.md)
* [STM32 Connectivity Expansion Pack User Guide](./Documentation/STM32ConnectivityExpansionPack_UserGuide.pdf)
* [Cypress Semiconductor, an Infineon Technologies Company](http://www.infineon.com)
* [Infineon GitHub](https://github.com/Infineon/)
* [ModusToolbox](https://www.infineon.com/cms/en/design-support/tools/sdk/modustoolbox-software/)


-------------------------------------------------------------------------------

© Cypress Semiconductor Corporation, 2021-2022.
