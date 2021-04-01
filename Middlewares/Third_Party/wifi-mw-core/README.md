# Wi-Fi Middleware Core Library
This repo comprises core components needed for Wi-Fi connectivity support. The library bundles FreeRTOS, lwIP TCP/IP stack, and mbed TLS for security, Wi-Fi Host Driver (WHD), Secure Sockets interface, configuration files, and associated code to bind these components together.

The ModusToolbox™ Wi-Fi code examples download this library automatically, so you don't need to.

## Features and Functionality
The following components are part of this library. These components are bundled as *.lib* entries; each *.lib* entry points to the respective repositories where they are hosted. All except lwIP and mbed TLS libraries are hosted on Cypress' GitHub repository. lwIP and mbed TLS libraries are hosted on their respective external repositories.

- **Wi-Fi Host Driver(WHD)** - Embedded Wi-Fi host driver that provides a set of APIs to interact with Cypress WLAN chips. See [Wi-Fi Host Driver(WHD)](https://github.com/cypresssemiconductorco/wifi-host-driver).

- **FreeRTOS** - FreeRTOS kernel, distributed as standard C source files with the configuration header file, for use with PSoC 6 MCU. This FreeRTOS library is based on publicly available FreeRTOS library version 10.0.1.37. See [FreeRTOS](https://github.com/cypresssemiconductorco/freertos) web site for details.

- **CLib FreeRTOS Support Library** - This library provides the necessary hooks to make C library functions such as malloc and free thread-safe. This implementation is specific to FreeRTOS; this library is required for building your application. See the [CLib FreeRTOS Support Library](https://github.com/cypresssemiconductorco/clib-support) web site for details.

- **lwIP** - A Lightweight open-source TCP/IP stack, version: 2.1.2. See the [lwIP](https://savannah.nongnu.org/projects/lwip/) web site for details.

   **Note**: Using this library in a project will cause lwIP to be downloaded on your computer. It is your responsibility to understand and accept the lwIP license.

- **mbed TLS** - An open source, portable, easy-to-use, readable and flexible SSL library that has cryptographic capabilities, version: 2.16.6. See the [mbed TLS](https://tls.mbed.org/) web site for details.

   **Note**: Using this library in a project will cause mbed TLS to be downloaded on your computer. It is your responsibility to understand and accept the mbed TLS license and regional use restrictions (including abiding by all applicable export control laws).

- **RTOS Abstraction Layer** - The RTOS Abstraction APIs allow middleware to be written to be RTOS-aware, but without depending on any particular RTOS. See the [RTOS Abstraction Layer](https://github.com/cypresssemiconductorco/abstraction-rtos) repository for details.

- **Secure Sockets** - Network abstraction APIs for the underlying lwIP network stack and mbed TLS security library. The Secure Sockets Library eases application development by exposing a socket-like interface for both secure and non-secure socket communication. See the [Secure Sockets](https://github.com/cypresssemiconductorco/secure-sockets) repository for details.

- **Predefined configuration files** - For FreeRTOS, lwIP, and mbed TLS for typical embedded IoT use cases. See **Quick Start** section for details.

- **Associated Glue Layer Between lwIP and WHD** 

This library is designed to work with PSoC 6 kits with Wi-Fi capability, supported through the ModusToolbox software environment. 

## Supported Platforms
This library and its features are supported on the following PSoC 6 platforms:

- [PSoC 6 Wi-Fi-BT Prototyping Kit (CY8CPROTO-062-4343W)](https://www.cypress.com/documentation/development-kitsboards/psoc-6-wi-fi-bt-prototyping-kit-cy8cproto-062-4343w)

- [PSoC 62S2 Wi-Fi BT Pioneer Kit (CY8CKIT-062S2-43012)](https://www.cypress.com/documentation/development-kitsboards/psoc-62s2-wi-fi-bt-pioneer-kit-cy8ckit-062s2-43012)

- [PSoC 6 WiFi-BT Pioneer Kit (CY8CKIT-062-WiFi-BT)](https://www.cypress.com/documentation/development-kitsboards/psoc-6-wifi-bt-pioneer-kit-cy8ckit-062-wifi-bt)

## Quick Start
1. A set of pre-defined configuration files have been bundled with this library for FreeRTOS, lwIP, and mbed TLS. These files are located in the *configs* folder.

   You should do the following:

   - Copy *FreeRTOSConfig.h*, *lwipopts.h*, and *mbedtls_user_config.h* files from the *configs* directory to the top-level code example directory in the project.

   - Configure the `MBEDTLS_USER_CONFIG_FILE` C macro to *mbedtls_user_config.h* in the Makefile to provide the user configuration to the mbed TLS library. The Makefile entry would look like as follows:

       ```
       DEFINES+=MBEDTLS_USER_CONFIG_FILE='"mbedtls_user_config.h"'
       ```

   - Add the `CYBSP_WIFI_CAPABLE` build configuration to enable Wi-Fi functionality. The Makefile entry would look like as follows:

       ```
       DEFINES+=CYBSP_WIFI_CAPABLE
       ```

   - Add the `CY_RTOS_AWARE` build configuration to inform the HAL that an RTOS environment is being used. The Makefile entry would look like as follows:

       ```
       DEFINES+=CY_RTOS_AWARE
       ```

   - Application which wants to use Automatic Private IP Addressing (Auto IP) should enable LWIP_AUTOIP and LWIP_DHCP_AUTOIP_COOP in lwipopts.h like as follows:

       ```
       #define AUTOIP 1
       #define LWIP_DHCP_AUTOIP_COOP 1
       ```

2. Secure Sockets, lwIP, and mbed TLS libraries contain reference and test applications. To ensure that these applications do not conflict with the code examples, a *.cyignore* file is also included with this library.

3. Add the following to COMPONENTS in the code example project's Makefile - `FREERTOS`, `PSOC6HAL`, `LWIP`, `MBEDTLS`, and either `4343W` or `43012` depending on the platform.

   For example, if your target is CY8CKIT-062S2-43012, the Makefile entry would look like as follows:

   ```
   COMPONENTS=FREERTOS PSOC6HAL LWIP MBEDTLS 43012
   ```
4. wifi-mw-core library disables all the debug log messages by default. To enable log messages, the application must perform the following:
 - Add `ENABLE_WIFI_MIDDLEWARE_LOGS` macro to the *DEFINES* in the code example's Makefile. The Makefile entry would look like as follows:
  ```
  DEFINES+=ENABLE_WIFI_MIDDLEWARE_LOGS
  ```
 - Call the `cy_log_init()` function provided by the *cy-log* module. cy-log is part of the *connectivity-utilities* library. See [connectivity-utilities library API documentation](https://cypresssemiconductorco.github.io/connectivity-utilities/api_reference_manual/html/group__logging__utils.html) for cy-log details.

## Additional Information
* [Wi-Fi Middleware Core RELEASE.md](./RELEASE.md)

* [Wi-Fi Middleware Core API Reference Guide](https://cypresssemiconductorco.github.io/wifi-mw-core/api_reference_manual/html/index.html)

* [ModusToolbox Software Environment, Quick Start Guide, Documentation, and Videos](https://www.cypress.com/products/modustoolbox-software-environment)

* [lwIP](https://savannah.nongnu.org/projects/lwip/)

* [mbed TLS](https://tls.mbed.org/)

* [Wi-Fi Middleware Core version](./version.txt)
