# Wi-Fi middleware core library

## What's iIncluded?

See the [README.md](./README.md) for a complete description of the Wi-Fi Middleware Core Library.

## Changelog

### v3.4.0
- Integrated mbedTLS Crypto acceleration module to support hardware crypto.

### v3.3.0
- Added support for CY8CEVAL-062S2-MUR-43439M2 kit

### v3.2.0
- Added support for CYW943907AEVAL1F and CYW954907AEVAL1F kits

### v3.1.1
- Upgraded mbed TLS version to v2.25

### v3.1.0
- Updated FreeRTOS and LwIP reference config files

- Added support for ARMC6 build

- Added a function for notifying network activity to low power assistant (LPA) middleware library


### v3.0.0
- Added support for SoftAP network interface

- Added support for simultaneous working of multiple network interfaces such as STA and AP (Concurrent mode)

- Added internal DHCP server support for SoftAP mode

- Added support for automatic private IP addressing (Auto IP) for STA mode


### v2.1.0
- Added support for IPv6 link-local address

- Updated lwIP and FreeRTOS reference config files


### v2.0.0
- Updated to include secure sockets library and connectivity utilities

- Upgraded mbed TLS to v2.16.6. See [release notes](https://tls.mbed.org/tech-updates/releases/mbedtls-2.16.6-and-2.7.15-released).


### v1.0.0
- Initial release for Wi-Fi middleware core

- Adds support for Wi-Fi host driver, lwIP TCP/IP stack, and mbed TLS security for TLS

### Supported software and tools

This version of the library was validated for compatibility with the following software and tools:

| Software and tools                                             | Version |
| :---                                                           | :----:  |
| ModusToolbox&trade; software environment                       | 2.4     |
| - ModusToolbox&trade; device configurator                      | 3.10    |
| - ModusToolbox&trade; CAPSENSE&trade; configurator/tuner tools | 4.0     |
| PSoC&trade; 6 peripheral driver library (PDL)                  | 2.2.1   |
| GCC compiler                                                   | 10.3.1  |
| IAR compiler                                                   | 8.32    |
| Arm&reg; compiler 6                                            | 6.14    |


## Additional information

- [Wi-Fi middleware core README.md](./README.md)

- [Wi-Fi middleware core API reference guide](https://cypresssemiconductorco.github.io/wifi-mw-core/api_reference_manual/html/index.html)

- [ModusToolbox&trade; software environment, quick start guide, documentation, and videos](https://www.cypress.com/products/modustoolbox-software-environment)
