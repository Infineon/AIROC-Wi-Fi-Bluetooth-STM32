# Enterprise Security Middleware Library

## Overview
This library provides the capability for Infineon's best in class Wi-Fi enabled PSoC&trade; 6 devices to connect to enterprise Wi-Fi networks. This library implements a collection of the most commonly used Extensible Authentication Protocols (EAP) that are used in enterprise networks.

## Features
This section provides details on the list of enterprise security Wi-Fi features supported by this library:
* Supports the following EAP security protocols:
    * EAP-TLS
    * PEAPv0 with MSCHAPv2
    * EAP-TTLS with EAP-MSCHAPv2 (Phase 2 tunnel authentication supports only EAP methods)
* Supports TLS session (session ID based) resumption
* Supports 'PEAP Fast reconnect' (applicable only for PEAP protocol)
* Supports roaming across APs in the enterprise network (vanilla roaming)
* Supports TLS versions 1.0, 1.1, and 1.2

**Note**: Deviations for **ThreadX** :
* Does not support TLS session resumption.
* Supports TLS1.3 in addition to TLS 1.0, 1.1 and 1.2. Only ECDSA key and certificate is supported with TLS1.3.

## Supported Enterprise Security Modes

* WPA
* WPA2
* WPA3 Transition
* WPA3 Enterprise only
* WPA3 192Bit

**Note**: WPA3 Enterprise Modes are supported for ThreadX only.

## Supported Frameworks

This library supports the following frameworks:

* ModusToolbox&trade; environment: In this environment the Enterprise Security library uses the [abstraction-rtos](https://github.com/infineon/abstraction-rtos) library that provides the RTOS abstraction API and the [wcm](https://github.com/infineon/wifi-connection-manager) library for network functions.

## Supported Platforms
### ModusToolbox&trade;
* [PSoC&trade; 6 WiFi-BT Prototyping Kit (CY8CPROTO-062-4343W)](https://www.cypress.com/documentation/development-kitsboards/psoc-6-wi-fi-bt-prototyping-kit-cy8cproto-062-4343w)
* [PSoC&trade; 62S2 Wi-Fi BT Pioneer Kit (CY8CKIT-062S2-43012)](https://www.cypress.com/documentation/development-kitsboards/psoc-62s2-wi-fi-bt-pioneer-kit-cy8ckit-062s2-43012)
* [PSoC&trade; 62S2 evaluation kit (CY8CEVAL-062S2-MUR-43439M2)](https://www.cypress.com/documentation/development-kitsboards/psoc-62s2-evaluation-kit-cy8ceval-062s2)
* [CYW955913EVK-01 Wi-Fi Bluetooth&reg; Prototyping Kit (CYW955913EVK-01)](https://www.infineon.com/CYW955913EVK-01)

## Dependencies
This section provides the list of dependent libraries required for this middleware library to work:

### ModusToolbox&trade;
* [Wi-Fi Connection Manager](https://github.com/infineon/wifi-connection-manager)
* [nx-secure-wpa3](https://github.com/Infineon/nx-secure-wpa3) (For ThreadX only)

## RADIUS Servers
This library has been verified with enterprise Wi-Fi networks configured using the following RADIUS server(s):
* FreeRadius 3.0.15

## Quick Start
This library is supported on ModusToolbox&trade; environments. Enterprise security library supports 2 combinations "FreeRTOS, lwIP, Mbed TLS" and "ThreadX, NetxDuo, NetxSecure". The section below provides information on how to build the library in these environments.

### FreeRTOS, lwIP, and Mbed TLS
* To use enterprise-security library on Wi-Fi kits with FreeRTOS, lwIP, and Mbed TLS combination, the application should pull [wifi-core-freertos-lwip-mbedtls](https://github.com/Infineon/wifi-core-freertos-lwip-mbedtls) library which will internally pull all dependent modules.
To pull wifi-core-freertos-lwip-mbedtls create the following *.mtb* file in deps folder.
   - *wifi-core-freertos-lwip-mbedtls.mtb:*
      `https://github.com/Infineon/wifi-core-freertos-lwip-mbedtls#latest-v1.X#$$ASSET_REPO$$/wifi-core-freertos-lwip-mbedtls/latest-v1.X`

* A set of pre-defined configuration files for FreeRTOS, lwIP, and Mbed TLS combination is bundled in wifi-core-freertos-lwip-mbedtls library for Wi-Fi kits. The developer is expected to review the configuration and make adjustments.

1. Make the following changes to the default mbed TLS configurations in mbedtls_user_config.h:
   - Enable the following flags:
     `MBEDTLS_DES_C`, `MBEDTLS_MD4_C`, `MBEDTLS_MD5_C`, `MBEDTLS_SHA1_C`, `MBEDTLS_SSL_PROTO_TLS1`, `MBEDTLS_SSL_PROTO_TLS1_1`, and `MBEDTLS_SSL_EXPORT_KEYS`
   - Disable the following flags:
     `MBEDTLS_POLY1305_C`, `MBEDTLS_CHACHAPOLY_C`, and `MBEDTLS_CHACHA20_C`

2. Define the following COMPONENTS in the application's Makefile for the Enterprise Security library.
  ```
    COMPONENTS=FREERTOS PSOC6HAL MBEDTLS LWIP WCM
  ```

3. By default, the macro `MBEDTLS_HAVE_TIME_DATE` is undefined in mbedtls_user_config.h. If the application wishes to perform time and date validation on the certificate, then enable the `MBEDTLS_HAVE_TIME_DATE` flag in mbedtls_user_config.h.

### ThreadX, NetX Duo, and NetX Secure

* To use enterprise-security library with Wi-Fi kits on ThreadX, NetxDuo, and NetxSecure combination, the application should pull wifi-core-threadx-netxduo-netxsecure library which will internall pull all dependent modules except nx-secure-wpa3.

    * To pull wifi-core-threadx-netxduo-netxsecure create the following *.mtb* file in deps folder.
      *wifi-core-threadx-netxduo-netxsecure.mtb:*
      `mtb://wifi-core-threadx-netxduo-netxsecure#latest-v1.X#$$ASSET_REPO$$/wifi-core-threadx-netxduo-netxsecure/latest-v1.X`

    * To pull nx-secure-wpa3 create the following *.mtb* file in deps folder.
      *nx-secure-wpa3.mtb:*
      `mtb://nx-secure-wpa3#latest-v1.X#$$ASSET_REPO$$/nx-secure-wpa3/latest-v1.X`

* A set of pre-defined configuration files is bundled in wifi-core-threadx-netxduo-netxsecure library for Wi-Fi kits. The developer is expected to review the configuration and make adjustments.

    1. Define the following COMPONENTS in the application's Makefile for the Enterprise Security library.
    ```
    COMPONENTS=THREADX NETXDUO NETXSECURE NETXSECURE_WPA3
    DISABLE_COMPONENTS+=NETXSECURE_ROM
    ```
### Logging

Enterprise Security library disables all the debug log messages by default. To enable log messages, the application must perform the following:
  - Add `ENABLE_ENTERPRISE_SECURITY_LOGS` macro to the *DEFINES* in the application's Makefile. The Makefile entry would look as follows:
     ```
       DEFINES+=ENABLE_ENTERPRISE_SECURITY_LOGS
     ```
  - Call the `cy_log_init()` function provided by the *cy-log* module. cy-log is part of the *connectivity-utilities* library. See [connectivity-utilities library API documentation](https://infineon.github.io/connectivity-utilities/api_reference_manual/html/group__logging__utils.html) for cy-log details.

### Additional Information
* [Enterprise Security RELEASE.md](./RELEASE.md)
* [Enterprise Security API reference guide](https://infineon.github.io/enterprise-security/api_reference_manual/html/index.html)
* [ModusToolbox&trade; Software Environment, Quick Start Guide, Documentation, and Videos](https://www.cypress.com/products/modustoolbox-software-environment)
* [Enterprise Security library version](./version.xml)
