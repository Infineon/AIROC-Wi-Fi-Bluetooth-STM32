------------------------------
STM32H747I-DISCO : Wi-Fi Scan
------------------------------

The example initializes the Wi-Fi device and starts a Wi-Fi scan without any
filter and prints the results on the serial terminal.  The example starts a
scan every 3 seconds after the previous scan completes.

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
1. Open wifi_scan.ioc
2. Go to Project Manager --> Project tab and select "STM32CubeIDE" or EWARM
   depending on the Compiler option needed
3. Click "Generate Code" and open the generated project 
4. Build the project in the IDE selected.
5. Flash the binary and run the application.

-------------
Output
-------------

 ******************* WiFi-Scan app *******************

    Push blue button or send any symbol via serial terminal to continue...

    CYW43xxx detected

WLAN MAC Address : E8:E8:B7:9F:D4:90
WLAN Firmware    : wl0: Sep  9 2020 01:22:10 version 13.10.271.253 (c4c4c7c CY) FWID 01-79301bec
WLAN CLM         : API: 18.2 Data: 9.10.0 Compiler: 1.36.1 ClmImport: 1.34.1 Creation: 2020-09-09 01:19:03
WHD VERSION      : v1.93.0 : v1.93.0 : IAR 8050004 : 2020-12-21 13:24:03 +0530

----------------------------------------------------------------------------------------------------
  #                  SSID                  RSSI   Channel       MAC Address              Security
----------------------------------------------------------------------------------------------------

--------------------------------------
Wi-Fi Connect and ICMP Ping example
--------------------------------------
This example also provides option to connect to Wi-Fi Access point and send ICMP Ping to Gateway address

Modify below parameters depending on the access point used and enable this by modifying the WIFI_CONNECT_ENABLE macro.

/* Wi-Fi Connection related parameters */
#define WIFI_CONNECT_ENABLE                 (0) /* Enable/Disable Wi-Fi Connect */
#define WIFI_SSID                           "WIFI_SSID"
#define WIFI_PASSWORD                       "WIFI_PASSWORD"
#define WIFI_SECURITY                       CY_WCM_SECURITY_WPA2_AES_PSK
