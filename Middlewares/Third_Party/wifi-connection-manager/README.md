# Wi-Fi Connection Manager (WCM)
WCM is a library which helps application developers to manage Wi-Fi Connectivity. The library provides a set of APIs that can be used to establish and monitor Wi-Fi connections on Cypress platforms that support Wi-Fi connectivity.

The library APIs are thread-safe. The library monitors the Wi-Fi connection and can notifies connection state changes through an event notification mechanism. The library also provides APIs to connect to a Wi-Fi network using Wi-Fi Protected Setup (WPS) methods.

## Features and Functionality
The current implementation has the following features and functionality:
* Supports WPA3 personal network security.
* Supports STA, SoftAP, and Concurrent(simultaneous Soft-AP + STA) modes.
* Support for WPS - Enrollee role.
* Exposes Wi-Fi APIs to scan, join, and leave the Wi-Fi network.
* Connection monitoring: Monitors active connections and link events. Automatically reconnects to the AP if the connection with the AP is lost intermittently. Notifies the connection state change through event notification registration mechanism.
* This library is part of AnyCloud framework that supports connectivity applications based on FreeRTOS, lwIP, and mbed TLS.
* The library is built on top of the abstraction-rtos library that provides the RTOS abstraction API for FreeRTOS.

## Supported Platforms
This library and its features are supported on the following Infineon platforms:
* [PSoC&trade; 6 Wi-Fi BT Prototyping Kit (CY8CPROTO-062-4343W)](https://www.cypress.com/documentation/development-kitsboards/psoc-6-wi-fi-bt-prototyping-kit-cy8cproto-062-4343w)

* [PSoC&trade; 62S2 Wi-Fi BT Pioneer Kit (CY8CKIT-062S2-43012)](https://www.cypress.com/documentation/development-kitsboards/psoc-62s2-wi-fi-bt-pioneer-kit-cy8ckit-062s2-43012)

* [PSoC&trade; 6 WiFi-BT Pioneer Kit (CY8CKIT-062-WiFi-BT)](https://www.cypress.com/documentation/development-kitsboards/psoc-6-wifi-bt-pioneer-kit-cy8ckit-062-wifi-bt)

* [PSoC&trade; 62S2 evaluation kit (CY8CEVAL-062S2-LAI-4373M2)](https://www.cypress.com/documentation/development-kitsboards/psoc-62s2-evaluation-kit-cy8ceval-062s2)

* [CYW954907AEVAL1F Evaluation Kit(CYW954907AEVAL1F)](https://www.cypress.com/documentation/development-kitsboards/cyw954907aeval1f-evaluation-kit)

* [CYW943907AEVAL1F Evaluation Kit(CYW943907AEVAL1F)](https://www.cypress.com/documentation/development-kitsboards/cyw943907aeval1f-evaluation-kit)

* [PSoC&trade; 62S2 evaluation kit (CY8CEVAL-062S2-MUR-43439M2)](https://www.cypress.com/documentation/development-kitsboards/psoc-62s2-evaluation-kit-cy8ceval-062s2)

## Dependent Libraries
This library depends on the following:
* [wifi-core-freertos-lwip-mbedtls](https://github.com/Infineon/wifi-core-freertos-lwip-mbedtls)
* [Wi-Fi Host Driver](https://github.com/Infineon/wifi-host-driver)

## Quick Start
* To use wifi-connection-manager library for FreeRTOS, lwIP, and mbed TLS, the application should pull [wifi-core-freertos-lwip-mbedtls](https://github.com/Infineon/wifi-core-freertos-lwip-mbedtls) library which will internally pull wifi-connection-manager, FreeRTOS, lwIP, mbed TLS and other dependent modules.
To pull wifi-core-freertos-lwip-mbedtls create the following *.mtb* file
   - *wifi-core-freertos-lwip-mbedtls.mtb:* https://github.com/Infineon/wifi-core-freertos-lwip-mbedtls#latest-v1.X#$$ASSET_REPO$$/wifi-core-freertos-lwip-mbedtls/latest-v1.X

* For existing Wi-Fi Connection Manager version 2.X users a [porting guide](https://Infineon.github.io/wifi-connection-manager/porting_guide.md) is available to migrate to Wi-Fi Connection Manager version 3.0

* A set of pre-defined configuration files have been bundled with the wifi-core-freertos-lwip-mbedtls library for FreeRTOS, lwIP, and mbed TLS. The developer is expected to review the configuration and make adjustments. See the "Quick Start" section in [README.md](https://github.com/Infineon/wifi-core-freertos-lwip-mbedtls/blob/master/README.md).

* A set of COMPONENTS must be defined in the code example project's Makefile for this library. See the "Quick Start" section in [README.md](https://github.com/Infineon/wifi-core-freertos-lwip-mbedtls/blob/master/README.md).

* The WCM library disables all the debug log messages by default. To enable log messages, the application must perform the following:

 - Add the `ENABLE_WCM_LOGS` macro to the *DEFINES* in the code example's Makefile. The Makefile entry would look like as follows:
   ```
   DEFINES+=ENABLE_WCM_LOGS
   ```
 
 - Call the `cy_log_init()` function provided by the *cy-log* module. cy-log is part of the *connectivity-utilities* library. 
 
 - See [connectivity-utilities library API documentation](https://Infineon.github.io/connectivity-utilities/api_reference_manual/html/group__logging__utils.html).

* WPS is disabled by default. WPS uses Mbed TLS security stack. Enable the following components for WPS.
  ```
  COMPONENTS+=WPS MBEDTLS
  ```

## Additional Information
* [Wi-Fi Connection Manager RELEASE.md](./RELEASE.md)

* [Wi-Fi Connection Manager API Documentation](https://Infineon.github.io/wifi-connection-manager/api_reference_manual/html/index.html)

* [Porting guide for Wi-Fi Connection Manager version 3.0](https://Infineon.github.io/wifi-connection-manager/porting_guide.md)

* [Connectivity Utilities API documentation - for cy-log details](https://Infineon.github.io/connectivity-utilities/api_reference_manual/html/group__logging__utils.html)

* [ModusToolbox&trade; Software Environment, Quick Start Guide, Documentation, and Videos](https://www.cypress.com/products/modustoolbox-software-environment)

* [Wi-Fi Connection Manager version](./version.xml)

* [ModusToolbox&trade; AnyCloud code examples](https://github.com/Infineon?q=mtb-example-anycloud%20NOT%20Deprecated)
