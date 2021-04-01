# STM32 Connectivity Expansion Pack Release Notes
The STM32 Connectivity Expansion Pack is an extension of the CMSIS-Pack standard established by Arm. The pack is compliant with the full CMSIS-Pack standard, with additional requirements/restrictions on the final pack to meet the STM standard.
This pack uses libraries from the ModusToolbox environment.

## What's Included?
* Initial Release
* Wi-Fi Scan Example running on STM32H747I-DISCO Discovery kit
* Includes below libraries and middleware:

|  Asset                                                                                     | Version   | 
| ------------------------------------------------------------------------------------------ | --------- |
| [wifi-host-driver](https://github.com/cypresssemiconductorco/wifi-host-driver)             |  1.93.0   |
| [wcm](https://github.com/cypresssemiconductorco/wifi-connection-manager)                   |  2.0.1    |
| [wifi-mw-core](https://github.com/cypresssemiconductorco/wifi-mw-core)                     |  3.0.0    |
| [whd-bsp-integration](https://github.com/cypresssemiconductorco/whd-bsp-integration)       |  1.1.1    |
| [connectivity-utilities](https://github.com/cypresssemiconductorco/connectivity-utilities) |  3.0.1    |
| [core-lib](https://github.com/cypresssemiconductorco/core-lib)                             |  1.1.4    |        
| [abstraction-rtos](https://github.com/cypresssemiconductorco/abstraction-rtos)             |  1.3.0    |
| [LwIP](https://git.savannah.nongnu.org/cgit/lwip.git)                                      |  2.1.2    |             
| PAL          	                                                                             |  1.0.0    |
| Device                                                                                     |  1.0.0    |

## What Changed?

 v1.0.0
* Initial Release with support for STM32H7xx MCUs with Infineon’s CYW43xxx Wi-Fi-BT combo chip family listed below.

## Supported STM32 Boards and MCU 
*  STM32H747I-DISCO Discovery kit and STM32H7xx

## Supported Connectivity Modules

Infineon’s CYW43xxx Wi-Fi-BT combo chip family:
* CYW43012
* CYW4343W / CYW43438
* CYW4373

## Example apps inside the Pack

### Wi-Fi Scan Example
This example initializes the Wi-Fi device and starts a Wi-Fi scan without any filter and prints the results on the serial terminal.

Refer to Projects/STM32H747I-DISCO/Applications/wifi_scan/readme.txt for more details


## Compatible Software

|  Software                    | Version   | 
| ---------------------------- | --------- |
| STM32 CubeMX                 |  6.1.1    |
| STM32 CubeIDE                |  1.5.1    |
| IAR Embedded Workbench IDE   |  8.50.4   |

### Supported Compiler Version

| Compiler                                  | Version |
| :---                                      | :----:  |
| GCC Compiler                              | 7.3     |
| IAR Compiler                              | 8.50.4  |


## Future release

* Iperf and mfgtest support
* Bluetooth Support


### More information
* [README.md](./README.md)
* [STM32 Connectivity Expansion Pack User Guide](./Documentation/STM32ConnectivityExpansionPack_UserGuide.pdf)
* [Cypress Semiconductor GitHub](https://github.com/cypresssemiconductorco)
* [ModusToolbox](https://www.cypress.com/products/modustoolbox-software-environment)
* www.cypress.com / www.infineon.com


-------------------------------------------------------------------------------

© Cypress Semiconductor Corporation, 2021.