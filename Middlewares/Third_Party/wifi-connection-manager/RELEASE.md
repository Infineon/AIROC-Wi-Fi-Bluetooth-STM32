# Wi-Fi Connection Manager (WCM)

## What's Included?
See the [README.md](./README.md) for a complete description of the Wi-Fi Connection Manager.

## Known Issues
| Problem | Workaround |
| ------- | ---------- |
| The `cy_wcm_deinit` API does not bring down the network stack as the default underlying lwIP stack does not have an implementation for deinit. Therefore, the expectation is that `cy_wcm_init` and `cy_wcm_deinit` APIs should be invoked only once. | No workaround. Support will be added in a future release. |
| cy_wcm_connect_ap API does not connect to AP, if MAC address is specified as connection parameter.| No workaround. This issue will be fixed in future release. |
| IAR 9.30 toolchain throws build errors on Debug mode, if application explicitly includes iar_dlmalloc.h file | Add '--advance-heap' to LDFLAGS in application Makefile. |

## Changelog

### v3.0.0
* Wifi-connection-manager is updated to be network stack agnostic.
* Added support for CM0P core.
* Updated Documentation.

### v2.3.0
* Added support for WPA3-EXT SAE Supplicant

### v2.2.0
* Added support for CY8CEVAL-062S2-MUR-43439M2 kit

### v2.1.0
* Added support for CYW943907AEVAL1F and CYW954907AEVAL1F kits

### v2.0.4
* Documentation updates.
* Minor updates and improvements added in connect to AP and WPS APIs.
* General bug fixes.

### v2.0.3
* Added coverity fixes.

### v2.0.2
* Fixed passphrase length check for enterprise security auth types.

### v2.0.1
* Minor fixes added in ping API, and in re-connection logic for statically assigned IP.
* WEP security support is disabled in connect AP API.
* Documentation updates.

### v2.0.0
* Added support for Soft-AP and concurrent (simultaneous Soft-AP + STA) modes.
* Documentation updates.

### v1.1.0
* Introduced APIs to get the following:
  - Connected AP information
  - WLAN interface statistics
  - IPv6 link-local address
  - Gateway IP address
  - Netmask IP address
  - Gateway MAC address
* Introduced API to ping IPv4 address
* Minor documentation updates

### v1.0.1
* Code snippets added to the documentation

### v1.0.0
* Initial release for Wi-Fi Connection Manager

* Includes support for Wi-Fi Protected Setup (WPS) - Enrollee role

* Exposes Wi-Fi APIs to scan, join, and leave a Wi-Fi network.

* Connection monitoring: Monitor active connections and link events. Provides a mechanism to register for event notification. Re-authenticates the connection with the AP in case of intermittent connection loss.

### Supported Software and Tools
This version of the library was validated for compatibility with the following Software and Tools:

| Software and Tools                                         | Version |
| :---                                                       | :----:  |
| ModusToolbox&trade; Software Environment                   | 3.0     |
| - ModusToolbox&trade; Device Configurator                  | 4.0     |
| - ModusToolbox&trade; CapSense Configurator / Tuner tools  | 5.0     |
| PSoC 6 Peripheral Driver Library (PDL)                     | 3.0.0   |
| GCC Compiler                                               | 10.3.1  |
| IAR Compiler (only for AnyCloud)                           | 9.30    |
| Arm Compiler 6                                             | 6.16    |
