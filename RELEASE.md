# STM32 Connectivity Expansion Pack 1.1.0 Release Notes
The STM32 Connectivity Expansion Pack is an extension of the CMSIS-Pack standard established by Arm.
The pack is compliant with the full CMSIS-Pack standard, with additional requirements/restrictions
on the final pack to meet the STM standard.

This pack uses libraries from the ModusToolbox environment.

## What's Included?
* BLE/BT Support Release

## What Changed?

 v1.1.0
* Added BLE/BT support
* BLE/BT with WiFi combo usage
* WiFi onboarding with BLE Code Example added
* WiFi connectivity assets have versions promoted
* PAL improvements and fixes

## Supported STM32 Boards and MCU
*  STM32H747I-DISCO Discovery kit and STM32H7xx

## Supported Connectivity Modules

Infineon's CYW43xxx Wi-Fi-BT combo chip family:
* CYW43012
* CYW43439 / CYW4343W / CYW43438
* CYW4373  / CYW4373E

## Example apps inside the Pack

### Wi-Fi Scan Example
This example initializes the Wi-Fi device and starts a Wi-Fi scan without any filter and prints the
results on the serial terminal.

Refer to Projects/STM32H747I-DISCO/Applications/wifi_scan/readme.txt for more details

### Wi-Fi Onboarding with Bluetooth LE Example
This example demonstrates a simultaneous usage of Wi-Fi and BLE functionality of CYW43xxx combo
devices. It uses BLE on the combo device to help connect the Wi-Fi to the AP.

Refer to Projects/STM32H747I-DISCO/Applications/ble_wifi_onboarding/readme.txt for more details

## Compatible Software

|  Software                    | Version   |
| ---------------------------- | --------- |
| STM32 CubeMX                 |  6.1.1    |
| STM32 CubeIDE                |  1.5.1    |
| IAR Embedded Workbench IDE   |  8.50.4   |

### Supported Compiler Version

| Compiler                     | Version |
| :---                         | :----:  |
| GCC Compiler                 | 7.3     |
| IAR Compiler                 | 8.50.4  |


## Change History

| Version | Changes                                | Reason for Change                     |
| :----:  | :---                                   | :----                                 |
| 1.1.0   | New release comprising BLE/BT support  | BLE/BT support and code example added |
|         | and various PAL improvements.          |                                       |
| 1.0.1   | Patch release.                         | SDIO Buffer overrun fix.              |
| 1.0.0   | The initial version                    |                                       |


## Future release

* Iperf and mfgtest support


### More information
* [README.md](./README.md)
* [STM32 Connectivity Expansion Pack User Guide](./Documentation/STM32ConnectivityExpansionPack_UserGuide.pdf)
* [Cypress Semiconductor, an Infineon Technologies Company](http://www.cypress.com)
* [Cypress Semiconductor GitHub](https://github.com/cypresssemiconductorco)
* [ModusToolbox](https://www.cypress.com/products/modustoolbox-software-environment)


-------------------------------------------------------------------------------

© Cypress Semiconductor Corporation, 2021.
