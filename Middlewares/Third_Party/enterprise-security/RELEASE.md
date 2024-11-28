# Enterprise Security Middleware Library

## What's Included?
Refer to [README.md](./README.md) for a complete description of the enterprise security library.

## Changelog

### v3.2.0
* Enabled EAP-TTLS with EAP-MSCHAPv2 for Azure RTOS.

### v3.1.0
* Enabled PEAPv0 with MSCHAPv2 for Azure RTOS.

### v3.0.0
* Enabled Enterprise security with Azure RTOS (ThreadX).
* Added support for WPA3 Enterprise security.
* Dropped support of MBED OS and AFR.

### v2.1.1
* Fixed compilation warning

### v2.1.0
* Added support for CY8CEVAL-062S2-MUR-43439M2 kit

### v2.0.1
* Fixed sending the TLS alert message when server passes certificate with Unknown CA.

### v2.0.0
* Updated interface definitions, API return type, and header file names.
* Added support for AnyCloud framework.
* Upgraded Mbed OS support to Mbed OS v6.2.0.

### v1.0.0
* Initial release for Arm Mbed OS
* Supports EAP protocols required to connect to enterprise Wi-Fi networks
    - Supports the following EAP security protocols:
        * EAP-TLS
        * PEAPv0 with MSCHAPv2
        * EAP-TTLS with EAP-MSCHAPv2 (Phase 2 tunnel authentication supports only EAP methods)
    - Supports TLS session (session ID based) resumption
    - Supports 'PEAP Fast reconnect' (applicable only for PEAP protocol)
    - Supports roaming across APs in the enterprise network (vanilla roaming)
    - Supports TLS versions 1.0, 1.1, and 1.2

## Supported Software and Tools
This version of the Middleware was validated for compatibility with the following Software and Tools:

| Software and Tools                                        | Version |
| :---                                                      | :----:  |
| ModusToolbox&trade; Software Environment                  | 3.1     |
| ModusToolbox&trade; Device Configurator                   | 4.10    |
| GCC Compiler                                              | 11.3.1  |
| IAR Compiler                                              | 9.30    |
| Arm Compiler 6                                            | 6.16    |
