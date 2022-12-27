# Command console library

## Introduction

This library provides a framework to add command console support to your application. Support for Wi-Fi, iPerf, and Bluetooth Low Energy commands are bundled with this library.

[ModusToolbox&trade; AnyCloud Tester - Wi-Fi Bluetooth&reg; Console](https://github.com/Infineon/mtb-anycloud-wifi-bluetooth-tester), [Mbed OS Tester - Wi-Fi Bluetooth&reg; Console](https://github.com/Infineon/mbed-os-wifi-bluetooth-tester), and [Amazon FreeRTOS Tester - Wi-Fi Bluetooth&reg; Console](https://github.com/Infineon/afr-wifi-bluetooth-tester) applications are built over this library.

## Features

- Supports Wi-Fi commands to perform network operations such as scan and join

- Integrates iPerf 2.0 commands. iPerf is a tool for active measurements of the maximum achievable bandwidth on IP networks.

  **Note:** While iPerf provides several other commands, only a subset of commands that are relevant to embedded connectivity use cases is enabled in this library. See the [iPerf commands](#iperf-commands) section.

- Supports TCP client/server and UDP client/server on IPv4. Supports a single instance of the iPerf client and server. IPv6 support will be added in the future.

- Supports Bluetooth&reg; LE commands including Tx/Rx for measuring Bluetooth&reg; LE throughput

## Supported platforms

This library is supported on the following list of platforms/frameworks.

### ModusToolbox&trade;

- [PSoC&trade; 6 Wi-Fi Bluetooth&reg; prototyping kit (CY8CPROTO-062-4343W)](https://www.infineon.com/cms/en/product/evaluation-boards/cy8cproto-062-4343w/)

- [PSoC&trade; 62S2 Wi-Fi Bluetooth&reg; pioneer kit (CY8CKIT-062S2-43012)](https://www.infineon.com/cms/en/product/evaluation-boards/cy8ckit-062s2-43012/)

- [PSoC&trade; 62S2 evaluation kit (CY8CEVAL-062S2-LAI-4373M2)](https://www.infineon.com/cms/en/product/evaluation-boards/cy8ceval-062s2/)

- [PSoC&trade; 62S2 evaluation kit (CY8CEVAL-062S2-MUR-43439M2)](https://www.infineon.com/cms/en/product/evaluation-boards/cy8ceval-062s2/)

- [CYW943907AEVAL1F evaluation kit (CYW943907AEVAL1F)](https://www.infineon.com/cms/en/product/evaluation-boards/cyw943907aeval1f)

- [CYW954907AEVAL1F evaluation kit (CYW954907AEVAL1F)](https://www.infineon.com/cms/en/product/evaluation-boards/cyw954907aeval1f)

### Mbed OS

- [PSoC&trade; 6 Wi-Fi Bluetooth&reg; prototyping kit (CY8CPROTO-062-4343W)](https://www.infineon.com/cms/en/product/evaluation-boards/cy8cproto-062-4343w/)

- [PSoC&trade; 62S2 Wi-Fi Bluetooth&reg; pioneer kit (CY8CKIT-062S2-43012)](https://www.infineon.com/cms/en/product/evaluation-boards/cy8ckit-062s2-43012/)

### FreeRTOS

- [PSoC&trade; 6 Wi-Fi Bluetooth&reg; prototyping kit (CY8CPROTO-062-4343W)](https://www.infineon.com/cms/en/product/evaluation-boards/cy8cproto-062-4343w/)

- [PSoC&trade; 62S2 Wi-Fi Bluetooth&reg; pioneer kit (CY8CKIT-062S2-43012)](https://www.infineon.com/cms/en/product/evaluation-boards/cy8ckit-062s2-43012/)

## Supported frameworks

This library supports the following frameworks:

- **ModusToolbox&trade; environment:** In this environment the command console library uses the [abstraction-rtos](https://github.com/Infineon/abstraction-rtos) library that provides the RTOS abstraction API, and uses the [secure-sockets](https://github.com/Infineon/secure-sockets) library for implementing socket functions. For the Bluetooth&reg; LE functionality, this library uses [bluetooth-freertos](https://github.com/Infineon/bluetooth-freertos).

- **Mbed framework:** The Mbed framework is an Mbed OS-based solution. The command console library uses the [abstraction-rtos](https://github.com/Infineon/abstraction-rtos) library that provides the RTOS abstraction API, and uses the Mbed socket API for implementing socket functions. For the Bluetooth&reg; functionality, it uses the Cordio Bluetooth&reg; LE stack in Mbed OS.

- **FreeRTOS framework:** This is a FreeRTOS-based solution. The command console library uses the [abstraction-rtos](https://github.com/Infineon/abstraction-rtos) library that provides the RTOS abstraction API, and uses the FreeRTOS framework network abstraction and lwIP library for implementing socket functions. For the Bluetooth&reg; functionality, this library uses [bluetooth-freertos](https://github.com/Infineon/bluetooth-freertos).

## Dependencies

This section provides the list of dependent libraries required for this middleware library to work on FreeRTOS and Arm® Mbed OS IoT frameworks.

### ModusToolbox&trade;

- [Wi-Fi connection manager](https://github.com/Infineon/wifi-connection-manager)

- [Bluetooth&reg; stack for FreeRTOS](https://github.com/Infineon/bluetooth-freertos)

### Mbed OS

- [Arm&reg; Mbed OS 6.2.0](https://os.mbed.com/mbed-os/releases)

- [Connectivity utilities library](https://github.com/Infineon/connectivity-utilities/releases/tag/latest-v3.X)

### FreeRTOS

- [FreeRTOS](https://github.com/Infineon/amazon-freertos)

- [Bluetooth&reg; stack for FreeRTOS](https://github.com/Infineon/bluetooth-freertos)

## Quick start

This library is supported on ModusToolbox&trade;, Mbed OS, and FreeRTOS frameworks. See the following section to build the library in those frameworks.

### ModusToolbox&trade;

1. Review and make the required changes to the pre-defined configuration files.

- The configuration files are bundled with the wifi-mw-core library for FreeRTOS, lwIP, and Mbed TLS. See README.md for details.

   See the "Quick Start" section in [README.md](https://github.com/Infineon/wifi-mw-core/blob/master/README.md)(https://github.com/Infineon/wifi-core-freertos-lwip-mbedtls/blob/master/README.md).

- If the application is using bundle library then the configuration files are in the bundle library. For example if the application is using Wi-Fi core freertos lwip mbedtls bundle library, the configuration files are in wifi-core-freertos-lwip-mbedtls/configs folder. Similarly if the application is using Ethernet Core FreeRTOS lwIP mbedtls library, the configuration files are in ethernet-core-freertos-lwip-mbedtls/configs folder.

2. A set of COMPONENTS must be defined in the code example project's Makefile for this library. 

   ```
   COMPONENTS=FREERTOS MBEDTLS LWIP SECURE_SOCKETS
   ```

3. To enable the Bluetooth&reg; LE functionality, add `WICED_BLE` to the `COMPONENTS` section in the application's Makefile:

   ```
   COMPONENTS=WICED_BLE
   ```

4. Add the following compiler directives to the `DEFINES` section in the application's Makefile:

   ```
   DEFINES=HAVE_SNPRINTF
   ```

5. Tune the lwIP configuration parameters listed in application's *lwipopts.h* file, depending on the required Wi-Fi throughput and use case.

### Mbed OS

1. Add the .lib file(s) for dependent libraries:

   1. Create a folder named "deps".

   2. Create a file with the name "mbed-os.lib" and add the following line to this file. The SHA given below points to the **mbed-os-6.2.0** release:
      ```
      https://github.com/ARMmbed/mbed-os/#a2ada74770f043aff3e61e29d164a8e78274fcd4
      ```
   3. Create a file with the name "connectivity-utilities.lib" and add the following line to this file:
      ```
      https://github.com/Infineon/connectivity-utilities/#<commit-SHA-for-latest-release-v3.X>
      ```
   4. Replace `<commit-SHA-for-latest-release-v3.X>` in the above line with the commit SHA of the latest-v3.X tag available in the [connectivity utilities library - release](https://github.com/Infineon/connectivity-utilities/releases).

      Example: For tag `release-v3.0.2`

      ```
      https://github.com/Infineon/connectivity-utilities/#bbed663a71670b02d362c6f1bd69fe970ff814ec
      ```

2. Add `MBED` to the *components_add* section in the code example's JSON file. The JSON file entry would look like as follows:

   ```
   "target.components_add": ["MBED"]
   ```

3. The Bluetooth&reg; LE command utility is supported on Mbed Cordio Bluetooth&reg; LE stack and the WICED&trade; Bluetooth&reg; Stack.

   - To use the Mbed Cordio stack, add `CORDIO` to the list of components as follows:

      ```
      "target.components_add" : ["MBED", "CORDIO"]
      ```

   - To use the WICED&trade; Bluetooth&reg; stack, do the following:

     1. Add the following to the *target_overrides* section of the *mbed_app.json* file:

        ```
        "target.extra_labels_remove": ["CORDIO"],
        "target.features_remove": ["BLE"]
        ```
     2.  Add `WICED_BLE` in the list of components as follows:

         ```
         "target.components_add" : ["MBED", "WICED_BLE"]
         ```

4. Tune the lwIP configuration parameters listed in the application's *lwipopts.h* file, depending on the required Wi-Fi throughput and use case.

### FreeRTOS

All the configurations required for the command console library on FreeRTOS are set by the values defined in the *afr-wifi-bluetooth-tester/config_files/* files of the [Amazon FreeRTOS Tester - Wi-Fi Bluetooth&reg; Console](https://github.com/Infineon/afr-wifi-bluetooth-tester) application.

## Commands supported

This section lists the Wi-Fi and Bluetooth&reg; LE commands that are natively supported by the command console library.

### Wi-Fi commands

#### *Basic Wi-Fi commands*

Use the following Wi-Fi commands to perform network operations. Enter the following commands after the device boots up and the Wi-Fi module is initialized and ready.

1. Scan for nearby networks:
   ```
   > wifi_scan
	```

2. Connect to a network AP:
   ```
   > wifi_join <ssid> <security type> [password] [channel]
   ```

3. Disconnect from the network AP:
   ```
   > wifi_leave
	```

4. Get the signal strength/RSSI of the network:
    ```
    > wifi_get_rssi
    ```

5. Ping another device on the network:
    ```
    > wifi_ping <ip address>
    ```
   **Note:** This command is not supported on Mbed.
   
**Note:** Wi-Fi commands(scan/join/leave/get_rssi/ping) are retained for backward compatibility. These commands are deprecated and it will be removed in the future release.

### iPerf commands

iPerf commands are used for measuring the Wi-Fi performance/throughput. The _iperf_ sends TCP/UDP data between two peer devices to compute the Wi-Fi performance/throughput.

#### iPerf command options

The following options are supported by the iPerf software integrated into this library. On the console, type `iperf --help` for the list of options supported by the _iperf_ command.

**Note:** The `-w` command is not supported through the command line option; it can be only configured by changing the network stack configuration. See the following notes to configure the TCP window size.

```
> iperf --help

  Usage: iperf [-s|-c host] [options]
         iperf [-h|--help] [-v|--version]

   Client/Server:
   -i, --interval  #         Seconds between periodic bandwidth reports
   -l, --len       #[kmKM]   Length of buffer in bytes to read or write (Defaults: TCP=128K, v4 UDP=1470, v6 UDP=1450)
   -p, --port      #         Server port to listen on/connect to
   -u, --udp                 Use UDP rather than TCP
   -b, --bandwidth #[kmgKMG | pps]  Bandwidth to send at in bits/sec or packets per second for UDP client

   Note:
   -w, --window              TCP window size is not supported through run time option; however, you can configure the TCP window size \n
                             at the build time using the following instructions. The values given for the macros/configurations in the following  section\n
                             are just an example for reference, the actual values of the macros/configuration may vary with your mbed_app.json (or) *lwipopts.h* file.
   ```

   #### Mbed OS:

   Update the following configurations in *mbed_app.json*:

   1. For the server, modify/add the following configurations with a suitable value to change the TCP window size and pbuf pool size:

      \"lwip.tcp-wnd\": (TCP_MSS * 20)
      \"lwip.pbuf-pool-size\": 20

   2. For the client, modify the following configurations with a suitable value to change the TCP send buffer size:

      \"lwip.tcp-snd-buf\": (TCP_MSS * 20)
      \"lwip.memp-num-tcp-seg\": 127


   #### ModusToolbox&trade;:

   Update the following configurations in the application's config file (For example, *mtb-anycloud-wifi-bluetooth-tester/lwipopts.h*)

   1. For the server, modify/add the following macros with a suitable value to change the TCP window size and pbuf pool size:
      ```
      #define TCP_WND (TCP_MSS * 20)
      #define PBUF_POOL_SIZE 20
      ```

   2. For the client, modify the following macros with a suitable value to change the TCP send buffer size:

      ```
      #define TCP_SND_BUF (TCP_MSS * 20)
      #define MEMP_NUM_TCP_SEG 127
      ```

   #### FreeRTOS:

   Update the following configurations in the application's config file (For example, *afr-wifi-bluetooth-tester/config_files/lwipopts.h*).

   1. For the server, modify the following macros with a suitable value to change the TCP window size and pbuf pool size:

      ```
      #define TCP_WND (TCP_MSS * 20)
      #define PBUF_POOL_SIZE 20
      ```

   2. For the client, modify the following macro with a suitable value to change the TCP send buffer size:
      ```
      #define TCP_SND_BUF (TCP_MSS * 20)
      ```

   *Server-specific:*

   ```
   -s, --server             run in server mode
   -t, --time      #        time in seconds to listen for new connections as well as to receive traffic (default not set)
   ```

   *Client-specific:*

   ```
   -c, --client    <host>   run in client mode, connecting to <host>
   -n, --num       #[kmgKMG]    number of bytes to transmit (instead of -t)
   -t, --time      #        time in seconds to transmit for (default 10 secs)
   ```

   *Miscellaneous:*

   ```
   -x, --reportexclude [CDMSV]   exclude C(connection) D(data) M(multicast) S(settings) V(server) reports
   -y, --reportstyle C      report as a Comma-Separated Values
   -h, --help               print this message and quit
   -v, --version            print version information and quit
   ```

**Note:** On Mbed OS, the iPerf command line option `-l` has a max limit of 60 KB (per packet) for data transfer.

### iPerf setup

The following diagram shows the exact setup that should be used for measuring the Wi-Fi performance/throughput of a PSoC&trade; device using _iperf_.

![iPerf setup - Wi-Fi throughput measurement](images/iperf-setup.png)


### iPerf commands for Wi-Fi throughput measurement

Enter the following commands on the PSoC&trade; device (DUT) after the device boots up and connects to the Wi-Fi network. This section provides only the commands to be run on the DUT. When the 'client _iperf_ command' runs on the DUT, the 'server _iperf_ command' should run on the host PC (as shown in the iPerf Setup diagram), and vice versa.

1. Start iPerf as a TCP server:
	```
	> iperf -s
	```
   **Note:** On the peer iPerf device (host PC), start iPerf as a TCP client to send the TCP data.

2. Start iPerf as a TCP client:
	```
	> iperf -c <server_ip_addr> -t <time in sec>
	```
	**Note:** On the peer iPerf device (host PC), start iPerf as a TCP server.

   Sample command:
	```
	> iperf -c 192.168.0.100 -t 60
	```

3. Start iPerf as a UDP server:
	```
	> iperf -s -u
	```
   **Note:** On the peer iPerf device (host PC), start iPerf as a UDP client to send the UDP data.

4. Start iPerf as a UDP client:
	```
	> iperf -c <server_ip_addr> -t <time in sec> -u -b <band width>
	```
   **Note:** On the peer iPerf device (host PC), start iPerf as a UDP server.

   Sample command:
	```
	> iperf -c 192.168.0.100 -t 60 -u -b 50M
	```

## Tuning for optimal Wi-Fi performance

The TCP and UDP Wi-Fi performance can be tuned using the _iperf_ command options. The TCP window size option impacts the TCP throughput, and bandwidth option (`-b`) impacts the UDP throughput. You can tune the configurations either for TCP or UDP based on the details given in the following sections.

### Tuning the TCP throughput

The TCP window size is configured with a default optimal value in the Wi-Fi Bluetooth&reg; tester applications for PSoC&trade; 6 2M devices. If you are using a different PSoC&trade; device, see the [iPerf command options](#iperf-command-options) section of this document to adjust the configuration for a better TCP throughput. Note that the TCP window size of _iperf_ can be configured only at compile time.

### Tuning the UDP throughput

The iPerf command given in the [iPerf commands for Wi-Fi throughput measurement](#iperf-commands-for-wi-fi-throughput-measurement) section of this document is an ideal command for UDP throughput measurement on PSoC&trade; 6 2M devices. If you are using a different PSoC&trade; device, try different bandwidth values for the _iperf_ `-b` option to measure the UDP throughput.

For example, when you run the command `iperf -c 192.168.0.100 -t 60 -u -b 50M` to measure the UDP Tx throughput on the device, if you see the resulting throughput closer to 50 Mbps, you should run the same command again with the `-b 60M` option and measure the throughput again. If the resulting throughput is different (56 Mbps, which is lesser than 60M, for example), the max UDP Tx throughput supported by the device is 56 Mbps. On the contrary, if the resulting throughput is 27 Mbps with the `-b 50M` option, you should run the same command again with the `-b 30M` option and measure the throughput again. If the resulting throughput in this case is 28 Mbps, this is the max UDP Tx throughput supported by the PSoC&trade; device. Use this as a guideline to identify the real max UDP throughput supported by your device.

## Bluetooth&reg; LE commands

As an example, use the LE connection-oriented channel (CoC) application on a CYW20719B2Q40EVB-01 kit as the Bluetooth&reg; LE peer device to measure the Bluetooth&reg; LE throughput. Ensure that the Bluetooth&reg; LE connection-interval and LE-COC-MTU-size configurations are set to the same value in both the Bluetooth&reg; LE devices that are used for throughput measurement. To modify the Bluetooth&reg; LE connection-interval and MTU, see the [Tuning for optimal Bluetooth&reg; LE performance](#tuning-for-optimal-ble-performance) section.

The following Bluetooth&reg; LE commands are supported in this library:

1. Turn on Bluetooth&reg;:
	```
	> bt_on
	```

2. Turn off Bluetooth&reg;:
	```
	> bt_off
	```

3. Start Bluetooth&reg; LE advertisement:
	```
	> ble_start_adv
	```

4. Stop Bluetooth&reg; LE advertisement:
	```
	> ble_stop_adv
	```

5. Start Bluetooth&reg; LE scan:
	```
	> ble_start_scan
	```

6. Stop Bluetooth&reg; LE scan:
	```
	> ble_stop_scan
	```

7. Initialize LE connection-oriented channel (CoC):
	```
	> ble_coc_init
	```
   **Note:** LE CoC will be initialized with an MTU size of 100 and local Protocol Service Multiplexer (PSM) 0x19.

8. Start LE CoC advertisement:
	```
	> ble_coc_adv
	```

9. Scan and connect to a remote Bluetooth&reg; LE device running the LE CoC app on the CYW20719B2Q40EVB-01 device:
	```
	> ble_coc_scan_connect
	```

10. Disconnect an LE CoC connection:
	```
	> ble_coc_disconnect
	```

11. Send LE CoC data:
	```
	> ble_coc_send_start
	```

12. Stop sending LE CoC data:
	```
	> ble_coc_send_stop
	```

13. Get the throughput:
	```
	> ble_get_throughput
	```

14. Get the device address:
	```
	> bt_get_device_address
	```
	**Note:** This command is supported only on the WICED&trade; Bluetooth&reg; LE stack and not on the Mbed Cordio stack.

### Tuning for optimal Bluetooth&reg; LE performance

Bluetooth&reg; LE performance can be tuned using the Bluetooth&reg; LE connection-interval and LE-COC-MTU-size configurations. Ensure that the Bluetooth&reg; LE  connection-interval and LE-COC-MTU-size configurations are set to the same value in both the Bluetooth&reg; LE devices that are used for throughput measurement.

For a given MTU size, if the configured connection interval is low, the resulting throughput will be higher, and vice-versa.

### Modifying Bluetooth&reg; LE configurations on a PSoC&trade; device

**ModusToolbox&trade;:** (Infineon Bluetooth&reg; stack)

1. The connection interval value is configured in multiples of 1.25 milliseconds based on the connection interval assigned in the Bluetooth&reg; LE configuration structure.

   By default, the library sets the Bluetooth&reg; LE connection interval configuration value to 40 which computes to a connection interval time of 50 msec (40 * 1.25 msec). The connection interval can be modified through a build macro.

   Add the `BLE_CONNECTION_INTERVAL` macro to the *DEFINES* in the application's Makefile. If the connection interval needs to be set to 24, the Makefile entry would look like as follows:

    ```
    DEFINES += BLE_CONNECTION_INTERVAL=24
    ```
   For the range of connection interval that can be configured, see [Bluetooth&reg; LE connection interval](https:///infineon.github.io/btstack/ble/api_reference_manual/html/group__wiced__bt__cfg.html#ga4b1a4ba915ebe81ea7c7771f44f5cd07).

2. By default, `BLE_COC_MTU_SIZE` is set to 100 bytes; the max value that can be configured is 512 bytes. To modify this to 200, do the following:

   Add the `BLE_COC_MTU_SIZE` macro to the *DEFINES* in the application's Makefile. The Makefile entry would look like as follows:

    ```
    DEFINES += BLE_COC_MTU_SIZE=200
    ```
   
**Mbed OS:** (Cordio Bluetooth&reg; LE stack)

1. The connection interval value is configured in multiples of 1.25 milliseconds based on the connection parameter class of the Cordio stack.

   By default, the library sets the Bluetooth&reg; LE connection interval value to 40, which computes to a connection interval time of 50 ms (40 * 1.25 ms).

   To modify the connection interval, add the `BLE_CONNECTION_INTERVAL` macro to the *macros* section of the application's *mbed_app.json*. If the connection interval needs to be set to 24, the JSON entry would look like as follows:

    ```
    macros : "BLE_CONNECTION_INTERVAL=24"
    ```
   For the range of connection interval that can be configured, see [Bluetooth&reg; LE connection interval](https://os.mbed.com/docs/mbed-os/v6.7/mbed-os-api-doxy/structble_1_1_gap_1_1_preferred_connection_params__t.html#a5a859522981a2d8fd68fdf33cd6bcc2d).

2. By default, `BLE_COC_MTU_SIZE` is set to 100 bytes; the max value that can be configured is 512 bytes. To modify the this value to 200, add the `BLE_COC_MTU_SIZE` macro to the *macros* section of the application's *mbed_app.json*. The JSON entry would look like as follows:
    ```
    macros :  "BLE_COC_MTU_SIZE=200"
    ```
    
### Modifying Bluetooth&reg; LE configurations on a peer device

Do the following to modify the connection interval and MTU on the peer LE COC application running on the CYW20719B2Q40EVB-01 device:

1. The connection interval of LE COC application can be modified by changing the min and max connection interval values of BLE scan configuration structure in *le_coc_cfg.c*.By default, the connection interval of the PSoC&trade; device is set to 40. To set the same value, use the configuration given below:

    ```
    ble_scan_cfg =
    {
        ...
        /* Connection configuration */
        .conn_min_interval               = 40,
        .conn_max_interval               = 40,
        ...
    },
    ```
2. Configure the MTU using the client control tool:

   1. Right-click the LE_Connection_Oriented_Channel project on the Eclipse IDE for ModusToolbox&trade;.  Navigate to *ModusToolbox&trade;* and then select **ClientControl** from the floating window menu.

   2. Select the appropriate **WICED HCI UART** from the drop-down list and then click **Open Port**.

   3. Select the **LE COC** tab to set the required MTU size.

   4. Click **Close Port**, reset the CYW20719B2Q40EVB-01 device, and then click **Open Port** for these settings to take effect.

### Understanding the Bluetooth&reg; LE throughput

This section explains the theoretical Bluetooth&reg; LE throughput numbers achievable on a Bluetooth&reg; LE platform based on the configuration.

The theoretical Bluetooth&reg; LE throughput can be calculated using the following formula:

 **Throughput = (MTU size in bits) / (connection interval in seconds)**

The following default values are used in this library:

    - Default connection interval = 50 ms

    - Default MTU size = 100 bytes

The theoretical Bluetooth&reg; LE throughput achievable for  these default values can be calculated as follows:

   = (100 * 8) / (50 / 10^3)
   = (800 / 0.050)
   = 16,000 bps
   = 16 kbps

## Adding new console commands in an application

1. Initialize the command console framework:
	```
	cy_command_console_init
	```

   You should define the commands table and the handler along with the primary library. For example, the WHD library console commands may be placed in *WHD_utils* and located along with the WHD library. The test application will invoke the relevant function in *WHD_utils* for adding the commands.

2. Define the commands as shown in the following example:
	```
	#define MY_COMMANDS    \
		{ "mycmd1", my_command1, 0, NULL, NULL, (char *)"", (char *)"Run mycmd1 --help for usage."}, \
		{ "mycmd2", my_command2, 0, NULL, NULL, (char *)"", (char *)"Run mycmd2 --help for usage."}, \
	```

4. Define the command table:
	```
	const cy_command_console_cmd_t my_command_table[] =
	{
		MY_COMMANDS
		CMD_TABLE_END
	};
	```

5. Implement command handlers when the commands are executed on the console.

   In this example, implement `my_command1` and `my_command2` function handlers.

6. Add the command table to the console:
	```
	cy_command_console_add_table( my_command_table )
	```

7. De-register the command table:
	```
	cy_command_console_delete_table( my_command_table )
	```
## Configuring Command set based on user needs

**ModusToolbox&trade;:**

   By default, `Wifi`, `bluetooth` and `iperf` utilities are enabled. To disable any of these utilities add the following to the Makefile of the application.
   
   To disable Wi-Fi commands:
   ```
   DEFINES+=DISABLE_COMMAND_CONSOLE_WIFI
   ```
   To disable iperf command:
   ```
   DEFINES+=DISABLE_COMMAND_CONSOLE_IPERF
   ```
   To disable bluetooth commands:
   ```
   DEFINES+=DISABLE_COMMAND_CONSOLE_BT
   ```
   
**Mbed OS:**

   By default, `Wifi`, `bluetooth` and `iperf` utilities are enabled. To disable any of these utilities add the following macros to the *macros* section of the application's *mbed_app.json*. The JSON entry would look like as follows:

   To disable Wi-Fi commands:
   ```
   macros : "DISABLE_COMMAND_CONSOLE_WIFI"
   ```
   To disable iperf command:
   ```
   macros : "DISABLE_COMMAND_CONSOLE_IPERF"
   ```
   To disable bluetooth commands:
   ```
   macros : "DISABLE_COMMAND_CONSOLE_BT"
   ```

## Additional information

- [Command console RELEASE.md](./RELEASE.md)

- [Command console API reference guide](https://infineon.github.io/command-console/api_reference_manual/html/index.html)

- [ModusToolbox&trade; software, quick start guide, documentation, and videos](https://www.infineon.com/cms/en/design-support/tools/sdk/modustoolbox-software/)

- [Command console version](./version.xml)
