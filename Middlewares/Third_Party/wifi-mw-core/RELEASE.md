# Wi-Fi Middleware Core Library

## What's Included?
See the [README.md](./README.md) for a complete description of the Wi-Fi Middleware Core Library.

## Changelog
### v3.0.0
* Added support for SoftAP network interface.
* Added support for simultaneous working of multiple network interfaces such as STA and AP (Concurrent mode).
* Added internal DHCP server support for SoftAP mode.
* Added support for Automatic Private IP Addressing (Auto IP) for STA mode.

### v2.1.0
* IPv6 link local address support added
* Updated lwIP and FreeRTOS reference config files

### v2.0.0
* Updated to include Secure Sockets Library and Connectivity Utilities
* Upgraded mbed TLS to v2.16.6. See [release notes](https://tls.mbed.org/tech-updates/releases/mbedtls-2.16.6-and-2.7.15-released).

### v1.0.0
* Initial release for Wi-Fi Middleware Core
* Adds support for Wi-Fi Host Driver, lwIP TCP/IP stack, and mbed TLS security for TLS

### Supported Software and Tools
This version of the library was validated for compatibility with the following software and tools:

| Software and Tools                                      | Version |
| :---                                                    | :----:  |
| ModusToolbox Software Environment                       | 2.2     |
| - ModusToolbox Device Configurator                      | 2.20    |
| - ModusToolbox CapSense Configurator / Tuner tools      | 3.10    |
| PSoC 6 Peripheral Driver Library (PDL)                  | 2.0.0   |
| GCC Compiler                                            | 9.3.1   |
| IAR Compiler                                            | 8.32    |

## Additional Information
* [Wi-Fi Middleware Core README.md](./README.md)
* [Wi-Fi Middleware Core API Reference Guide](https://cypresssemiconductorco.github.io/wifi-mw-core/api_reference_manual/html/index.html)
* [ModusToolbox Software Environment, Quick Start Guide, Documentation, and Videos](https://www.cypress.com/products/modustoolbox-software-environment)
* [Wi-Fi Middleware Core version](./version.txt)
