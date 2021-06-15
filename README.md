# STM32 Connectivity Expansion Pack 1.1.0

## Overview
The STM32 Connectivity Expansion Pack is an extension of the CMSIS-Pack standard established by Arm
to support Wi-Fi and BLE application on STM32 MCUs with Infineon Wireless Combo devices.

This pack is MCU and flow-independent, meaning the user can choose the compiler toolchain and IDE,
and adapt the source code to any STM microcontroller offering sufficient resources (serial
interfaces and memory) and performance to run the application.

This pack uses libraries from the ModusToolbox environment. For more details, refer to
https://www.cypress.com/products/modustoolbox.

You can select and configure the pack in the STM32CubeMX tool, make choices appropriate for your
design, such as which CYW43xxx device to use, and then generate a project from your selection.

## Dependencies
Wi-Fi application requires all the assets mentioned in the wifi group of expansion pack to be
selected along with proper CYW43xxx selected in the Device dropbox in STM32CubeMX.

## Expansion Pack Contents
The STM32 Connectivity Expansion Pack uses below assets:

|  Asset                                                                                     | Version |
| ------------------------------------------------------------------------------------------ | ------- |
| [btstack](https://github.com/cypresssemiconductorco/btstack)                               |  3.0.0  |
| [bluetooth-freertos](https://github.com/cypresssemiconductorco/bluetooth-freertos)         |  3.0.0  |
| [wifi-host-driver](https://github.com/cypresssemiconductorco/wifi-host-driver)             |  1.94.0 |
| [wcm](https://github.com/cypresssemiconductorco/wifi-connection-manager)                   |  2.0.3  |
| [wifi-mw-core](https://github.com/cypresssemiconductorco/wifi-mw-core)                     |  3.1.0  |
| [whd-bsp-integration](https://github.com/cypresssemiconductorco/whd-bsp-integration)       |  1.2.0  |
| [connectivity-utilities](https://github.com/cypresssemiconductorco/connectivity-utilities) |  3.0.2  |
| [core-lib](https://github.com/cypresssemiconductorco/core-lib)                             |  1.2.0  |
| [abstraction-rtos](https://github.com/cypresssemiconductorco/abstraction-rtos)             |  1.4.0  |
| [LwIP](https://git.savannah.nongnu.org/cgit/lwip.git)                                      |  2.1.2  |
| PAL                                                                                        |  1.1.0  |
| Device                                                                                     |  1.0.0  |

## Additional details
The pack is compliant with the full CMSIS-Pack standard, with additional requirements/restrictions
on the final pack to meet the STM standard.

## More information
* [RELEASE.md](./RELEASE.md)
* [STM32 Connectivity Expansion Pack User Guide](./Documentation/STM32ConnectivityExpansionPack_UserGuide.pdf)
* [Cypress Semiconductor, an Infineon Technologies Company](http://www.cypress.com)
* [Cypress Semiconductor GitHub](https://github.com/cypresssemiconductorco)
* [ModusToolbox](https://www.cypress.com/products/modustoolbox-software-environment)

------

All other trademarks or registered trademarks referenced herein are the property of their respective
owners.

![Banner](images/Banner.png)

-------------------------------------------------------------------------------

© Cypress Semiconductor Corporation, 2021.
