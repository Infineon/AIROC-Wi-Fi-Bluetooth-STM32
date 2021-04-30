# STM32 Connectivity Expansion Pack

## Overview
The STM32 Connectivity Expansion Pack is an extension of the CMSIS-Pack standard established by Arm to support Wi-Fi and Bluetooth LE application on STM32 MCUs with Infineon Wireless Combo devices.

This pack is MCU- and flow-independent, meaning you can choose the compiler toolchain and IDE, and then adapt the source code to any STM microcontroller offering sufficient resources (serial interfaces and memory) and performance to run the application. You can select and configure the pack in the STM32CubeMX tool, make choices appropriate for your design, such as which CYW43xxx device to use, and then generate a project from your selection.

This pack uses libraries from the ModusToolbox environment. For more details, refer to https://www.cypress.com/products/modustoolbox.


## Dependencies
Wi-Fi applications require all the assets mentioned in the Wi-Fi group of the expansion pack to be selected along with the appropriate CYW43xxx device selected in the Device drop-down menu in STM32CubeMX.

## Expansion Pack Contents
The STM32 Connectivity Expansion Pack includes the following libraries and middleware:

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
| PAL          	                                                                             |  1.0.1    |
| Device                                                                                     |  1.0.1    |

## Additional details
The pack is compliant with the full CMSIS-Pack standard, with additional requirements/restrictions on the final pack to meet the STM standard.

## More information
* [RELEASE.md](./RELEASE.md)
* [STM32 Connectivity Expansion Pack User Guide](./Documentation/STM32ConnectivityExpansionPack_UserGuide.pdf)
* [Cypress Semiconductor GitHub](https://github.com/cypresssemiconductorco)
* [ModusToolbox](https://www.cypress.com/products/modustoolbox-software-environment)
* www.cypress.com / www.infineon.com

------

All other trademarks or registered trademarks referenced herein are the property of their respective owners.

![Banner](./Documentation/images/Banner.png)

-------------------------------------------------------------------------------

© Cypress Semiconductor Corporation, 2021.