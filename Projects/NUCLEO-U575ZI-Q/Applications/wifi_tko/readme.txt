------------------------------
STM32H747I-DISCO : WLAN TCP keepalive offload
------------------------------
This code example demonstrates the TCP keepalive offload functionality which 
allows the WLAN device to handle TCP keepalive packets from the network on its own 
so that the host can remain longer in the low-power mode.
It employs the low power assistant (LPA) middleware library, which helps 
in developing low-power applications.

---------------------------------------
Compatible Software
---------------------------------------
STM32 CubeMX                  6.5.0
STM32 CubeIDE                 1.9.0
IAR Embedded Workbench IDE    8.50.4

---------------------
Supported Toolchains
---------------------
GCC Compiler V7.3
IAR Compiler 8.50.4

---------------------
Instructions
---------------------
1. Open wifi_tko.ioc
2. Go to Project Manager --> Project tab and select "STM32CubeIDE" or EWARM
   depending on the Compiler option needed
3. Click "Generate Code" and open the generated project
4. Modify below parameters depending on the access point used
#define WIFI_SSID                           "WIFI_SSID"
#define WIFI_PASSWORD                       "WIFI_PASSWORD"
#define WIFI_SECURITY                       CY_WCM_SECURITY_WPA2_AES_PSK
5. Modify TCP keepalive offload interval/retry_interval/retry_count, TCP socket port and TCP server port and IP address 
    static const cy_tko_ol_cfg_t cy_tko_ol_cfg_0 = 
    {
    .interval = 5,
    .retry_interval = 3,
    .retry_count = 3,
    .ports[0] = { 
            .local_port = 3353,
            .remote_port = 3360,
            "192.168.1.1"
            },
6. Build the project in the IDE selected.
7. Flash the binary and run the application.

-------------
Output
-------------

STM32H747I-DISCO: STFirmware (Fractal) V1.0.0 (Build Feb 14 2019 at 18:34:59)
CPU running at 75MHz, Peripherals at 75MHz/37MHz
WLAN MAC Address : 00:A0:50:45:13:81
WLAN Firmware    : wl0: Apr 12 2022 20:39:36 version 13.10.271.287 (760d561 CY) FWID 01-b438e2a0
WLAN CLM         : API: 18.2 Data: 9.10.0 Compiler: 1.36.1 ClmImport: 1.34.1 Creation: 2021-04-26 04:01:15 
WHD VERSION      : v2.4.0 : v2.4.0 : GCC 10.3 : 2022-08-04 17:12:02 +0800
Info: Wi-Fi initialization is successful
Info: Join to AP: SM9500
Info: Successfully joined wifi network SM9500
Info: Assigned IP address: 192.168.43.124
Info: Taking TCP Keepalive configuration from the Generated sources.
Info: Socket[0]: Created connection to IP 192.168.43.228, local port 3353, remote port 3360
Info: Skipped TCP socket connection for socket id[1]. Check the TCP Keepalive configuration.
Info: Skipped TCP socket connection for socket id[2]. Check the TCP Keepalive configuration.
Info: Skipped TCP socket connection for socket id[3]. Check the TCP Keepalive configuration.
whd_tko_toggle: Successfully enabled

Network Stack Suspended, MCU will enter DeepSleep power mode
Resuming Network Stack, Network stack was suspended for 31867ms

=====================================================
WHD Stats.. 
tx_total:73, rx_total:74, tx_no_mem:0, rx_no_mem:0
tx_fail:0, no_credit:0, flow_control:0
Bus Stats.. 
cmd52:2430, cmd53_read:393, cmd53_write:596
cmd52_fail:7, cmd53_read_fail:0, cmd53_write_fail:0
oob_intrs:0, sdio_intrs:484, error_intrs:0, read_aborts:0
=====================================================
Network is active. Resuming network stack
whd_tko_toggle: Successfully disabled
whd_tko_toggle: Successfully enabled

Network Stack Suspended, MCU will enter DeepSleep power mode
Resuming Network Stack, Network stack was suspended for 4142ms

=====================================================
