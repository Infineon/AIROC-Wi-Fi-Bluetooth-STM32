# Wi-Fi middleware core library

This repo comprises core components needed for Wi-Fi connectivity support. The library bundles FreeRTOS, lwIP TCP/IP stack, mbed TLS for security, mbedTLS crypto acceleration module, Wi-Fi host driver (WHD), secure sockets interface, configuration files, and associated code to bind these components together.

The ModusToolbox™ Wi-Fi code examples download this library automatically, so you don't need to.

## Features and functionality

The following components are part of this library. These components are bundled as *.lib* entries; each *.lib* entry points to the respective repositories where they are hosted. All except lwIP and mbed TLS libraries are hosted on Cypress GitHub repository. lwIP and mbed TLS libraries are hosted on their respective external repositories.

- **Wi-Fi Host Driver (WHD):** Embedded Wi-Fi host driver that provides a set of APIs to interact with Cypress WLAN chips. See [Wi-Fi host driver (WHD)](https://github.com/cypresssemiconductorco/wifi-host-driver).

- **FreeRTOS for Infineon MCUs:** FreeRTOS kernel, distributed as standard C source files with the configuration header file, for use with the Infineon MCUs. See 
[Readme](https://github.com/cypresssemiconductorco/freertos/README.md) for details.

- **CLib FreeRTOS support library:** This library provides the necessary hooks to make C library functions such as malloc and free thread-safe. This implementation is specific to FreeRTOS; this library is required for building your application. See the [CLib FreeRTOS support library](https://github.com/cypresssemiconductorco/clib-support) web site for details.

- **lwIP:** A Lightweight open-source TCP/IP stack, version: 2.1.2. See the [lwIP](https://savannah.nongnu.org/projects/lwip/) web site for details.

   **Note:** Using this library in a project will cause lwIP to be downloaded on your computer. It is your responsibility to understand and accept the lwIP license.

- **mbed TLS:** An open-source, portable, easy-to-use, readable and flexible SSL library that has cryptographic capabilities, version: 2.25.0. See the [mbed TLS](https://tls.mbed.org/) web site for details.

   **Note:** Using this library in a project will cause mbed TLS to be downloaded on your computer. It is your responsibility to understand and accept the mbed TLS license and regional use restrictions (including abiding by all applicable export control laws).

- **RTOS abstraction layer:** The RTOS abstraction APIs allow middleware to be written to be RTOS-aware, but without depending on any particular RTOS. See the [RTOS abstraction layer](https://github.com/cypresssemiconductorco/abstraction-rtos) repository for details.

- **Secure sockets:** Network abstraction APIs for the underlying lwIP network stack and mbed TLS security library. The secure sockets library eases application development by exposing a socket-like interface for both secure and non-secure socket communication. See the [Secure sockets](https://github.com/cypresssemiconductorco/secure-sockets) repository for details.

- **Predefined configuration files:** For FreeRTOS, lwIP, and mbed TLS for typical embedded IoT use cases. See **Quick Start** section for details.

- **Glue layer between lwIP and WHD:** This library is designed to work with PSoC&trade; 6 MCU kits with Wi-Fi capability, supported through the ModusToolbox&trade; software environment.


## Supported platforms

This library and its features are supported on the following PSoC&trade; 6 platforms:

- [PSoC&trade; 6 Wi-Fi Bluetooth&reg; prototyping kit (CY8CPROTO-062-4343W)](https://www.cypress.com/documentation/development-kitsboards/psoc-6-wi-fi-bt-prototyping-kit-cy8cproto-062-4343w)

- [PSoC&trade; 62S2 Wi-Fi Bluetooth&reg; pioneer kit (CY8CKIT-062S2-43012)](https://www.cypress.com/documentation/development-kitsboards/psoc-62s2-wi-fi-bt-pioneer-kit-cy8ckit-062s2-43012)

- [PSoC&trade; 6 Wi-Fi Bluetooth&reg; pioneer kit (CY8CKIT-062-WiFi-BT)](https://www.cypress.com/documentation/development-kitsboards/psoc-6-wifi-bt-pioneer-kit-cy8ckit-062-wifi-bt)

- [PSoC&trade; 64S0S2 Wi-Fi Bluetooth&reg; pioneer kit (CY8CKIT-064S0S2-4343W)](https://www.cypress.com/documentation/development-kitsboards/psoc-64-standard-secure-aws-wi-fi-bt-pioneer-kit-cy8ckit)

- [PSoC&trade; 62S2 evaluation kit (CY8CEVAL-062S2-LAI-4373M2)](https://www.cypress.com/documentation/development-kitsboards/psoc-62s2-evaluation-kit-cy8ceval-062s2)

- [CYW954907AEVAL1F Evaluation Kit(CYW954907AEVAL1F)](https://www.cypress.com/documentation/development-kitsboards/cyw954907aeval1f-evaluation-kit)

- [CYW943907AEVAL1F Evaluation Kit(CYW943907AEVAL1F)](https://www.cypress.com/documentation/development-kitsboards/cyw943907aeval1f-evaluation-kit)

- [PSoC&trade; 62S2 evaluation kit (CY8CEVAL-062S2-MUR-43439M2)](https://www.cypress.com/documentation/development-kitsboards/psoc-62s2-evaluation-kit-cy8ceval-062s2)

## Quick start

A set of pre-defined configuration files have been bundled with this library for lwIP, and mbed TLS. These files are located in the *configs* folder.

You should do the following:

1. Copy *lwipopts.h*, and *mbedtls_user_config.h* files from the *configs* directory to the top-level code example directory in the project.

2. Configure the `MBEDTLS_USER_CONFIG_FILE` C macro to *mbedtls_user_config.h* in the Makefile to provide the user configuration to the mbed TLS library. The Makefile entry should look like as follows:

    ```
    DEFINES+=MBEDTLS_USER_CONFIG_FILE='"mbedtls_user_config.h"'
    ```

3. Add the `CYBSP_WIFI_CAPABLE` build configuration to enable the Wi-Fi functionality. The Makefile entry should look like as follows:

    ```
    DEFINES+=CYBSP_WIFI_CAPABLE
    ```

4. Add the `CY_RTOS_AWARE` build configuration to inform the HAL that an RTOS environment is being used. The Makefile entry should look like as follows:

    ```
    DEFINES+=CY_RTOS_AWARE
    ```

5. If your application uses automatic private IP addressing (Auto IP), enable `LWIP_AUTOIP` and `LWIP_DHCP_AUTOIP_COOP` in *lwipopts.h* like as follows:

    ```
    #define AUTOIP 1
    #define LWIP_DHCP_AUTOIP_COOP 1
    ```

6. Add the following to `COMPONENTS` in the code example project's Makefile: `FREERTOS`, `LWIP`, and `MBEDTLS`.

   For example:

   ```
   COMPONENTS=FREERTOS LWIP MBEDTLS
   ```
   **Note:** `PSOC6HAL` and either `43012` or `4343W` are necessary for the library depending on the platform used, but these are already included in the BSP's Makefile. Therefore, you do not need to include them here again.

7. By default, the wifi-mw-core library enables mbedtls alternate implementation for the cryptographic operations supported by cy-mbedtls-acceleration module. Do the following to disable the mbedtls alternate implementation:

   ```
   DEFINES+=DISABLE_MBEDTLS_ACCELERATION
   ```

8. The cy-mbedtls-acceleration module included in wifi-mw-core library, enables mbed TLS ALT configurations for the crypto operations supported by the platform. Even if all the crypto operations under mbedTLS ALT config are not supported by the platform, the cy-mbedtls-acceleration module enables the mbedTLS ALT config. For example, even though all the eliptic curves are not supported, it enables MBEDTLS_ECP_ALT. In such cases, it is user's responsibility to disable the mbed TLS ALT config to use the mbed TLS software crypto. For example, if the user enables a cipher-suite that involes in eliptic curve crypto operation not supported by cy-mbedtls-acceleration, then he need to disable the MBEDTLS_ECP_ALT config. To know the supported hardware crypto operations, See [mbedTLS Crypto acceleration for CAT1A, CAT1B & CAT1C MCUs](https://github.com/Infineon/cy-mbedtls-acceleration) documentation.

9. The wifi-mw-core library disables all debug log messages by default. Do the following to enable log messages:

   1. Add the `ENABLE_WIFI_MIDDLEWARE_LOGS` macro to the *DEFINES* in the code example's Makefile. The Makefile entry should look like as follows:
       ```
       DEFINES+=ENABLE_WIFI_MIDDLEWARE_LOGS
       ```
   2. Call the `cy_log_init()` function provided by the *cy-log* module. cy-log is part of the *connectivity-utilities* library. See [connectivity-utilities library API documentation](https://cypresssemiconductorco.github.io/connectivity-utilities/api_reference_manual/html/group__logging__utils.html) for cy-log details.

Secure sockets, lwIP, and mbed TLS libraries contain reference and test applications. To ensure that these applications do not conflict with the code examples, a *.cyignore* file is also included with this library.


## Additional information

- [Wi-Fi middleware core RELEASE.md](./RELEASE.md)

- [Wi-Fi middleware core API reference guide](https://cypresssemiconductorco.github.io/wifi-mw-core/api_reference_manual/html/index.html)

- [ModusToolbox&trade; software environment, quick start guide, documentation, and videos](https://www.cypress.com/products/modustoolbox-software-environment)

- [lwIP](https://savannah.nongnu.org/projects/lwip/)

- [mbedTLS Crypto acceleration for CAT1A, CAT1B & CAT1C MCUs](https://github.com/Infineon/cy-mbedtls-acceleration)

- [mbed TLS](https://tls.mbed.org/)

- [Wi-Fi middleware core version](./version.xml)
