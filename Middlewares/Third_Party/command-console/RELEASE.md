# Command console library

## What's included?

See the [README.md](./README.md) for a complete description of the command console library.

## Known issues
| Problem | Workaround |
| ------- | ---------- |
| Running iperf command in client UDP mode with the `-i 1` (interval) option reports a lower throughput. | Run the iPerf without the `-i` (interval) option (or) use an interval value greater than 1 second. |
| IAR 9.30 toolchain throws build errors on Debug mode, if application explicitly includes iar_dlmalloc.h file | Add '--advance-heap' to LDFLAGS in application Makefile. |

## Changelog

### v4.0.0

- Added ethernet utility support
- Added support for CM0P core
- Minor Documentation updates

### v3.2.0

- Added support for CYW954907AEVAL1F and CYW943907AEVAL1F kits

### v3.1.0

- Added support for CY8CEVAL-062S2-MUR-43439M2 kit

### v3.0.1

- Renamed Wi-Fi command names

- Build switch provided to disable command sets

### v3.0.0

- Minor documentation and BT configuration updates

- Fix added for smartcoex LE disconnection issue

- Introduced ARMC6 toolchain build support for AnyCloud

- Integrated with BTSTACK library version v3.x

### v2.0.0

- Added support of the command console library for the FreeRTOS framework

- Added Wi-Fi diagnostic commands to perform network operations

- Added BT diagnostic commands to perform Bluetooth&reg; LE operations

### v1.0.1

- Documentation updates

- For the IAR toolchain, added a fix to echo the commands as they are typed

### v1.0.0

- This is the first version of the command console library for Mbed OS and AnyCloud

## Supported software and tools

The current version of the library was validated for compatibility with the following software and tools:

| Software and tools                                      | Version |
| :---                                                    | :----:  |
| ModusToolbox&trade; software                            | 3.0     |
| PSoC&trade; 6 peripheral driver library (PDL)           | 3.0.0   |
| GCC Compiler                                            | 10.3.1  |
| IAR Compiler (only for ModusToolbox&trade;)             | 9.30    |
| Arm® Compiler 6                                         | 6.16   |
| Mbed OS                                                 | 6.2.0   |
