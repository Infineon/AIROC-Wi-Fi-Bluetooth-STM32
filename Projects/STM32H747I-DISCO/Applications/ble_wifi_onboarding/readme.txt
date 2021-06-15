------------------------------------------------------
STM32H747I-DISCO : Wi-Fi Onboarding with Bluetooth LE
------------------------------------------------------

This example uses the STM32H7 MCU to communicate with the CYW43xxx combo
devices and control the Wi-Fi and BLE functionality.  It uses BLE on the combo
device to help connect the Wi-Fi to the AP.

BLE provides a mechanism for the device to connect to a Wi-Fi AP by providing
the Wi-Fi SSID and password in a secure manner.  The Wi-Fi credentials are
stored in EEPROM so that the device can use this data upon reset to connect to
an AP without requiring BLE intervention.  Note that the data stored in the
EEPROM is unencrypted.

---------------------------------------
Compatible Software
---------------------------------------
STM32 CubeMX                  6.1.1
STM32 CubeIDE                 1.5.1
IAR Embedded Workbench IDE    8.50.4

---------------------
Supported Toolchains
---------------------
GCC Compiler V7.3
IAR Compiler 8.50.4

---------------------
Instructions
---------------------
1. Open ble_wifi_onboarding.ioc
2. Go to Project Manager --> Project tab and select "STM32CubeIDE" or EWARM
   depending on the Compiler option needed
3. Click "Generate Code" and open the generated project 
4. Build the project in the IDE selected.
5. Flash the binary and run the application.
6. Open CySmart Mobile App on Andriod or iOS Device (Turn ON Bluetooth on the
   device)
7. Swipe down on the CySmart app home screen to start scanning for BLE
   Peripherals.  Your device ("bleProv") appears in the CySmart app home
   screen. Select your device to establish a BLE connection
8. Select the GATT DB Profile from the carousel view then select Unknown
   Service.
9. Select the attribute with the UUID ending in 63. In the ASCII field, type
   your Wi-Fi SSID in string format.
10.Select the attribute with the UUID ending in 64. In the ASCII field, type
   your Wi-Fi PASSWORD in string format.
11.Select the attribute with the UUID ending in 65. Select Notify. Write hex
   value 1 to this characteristic to connect to the WiFi network.  If the
   connection is successful then the server will send a notification with
   value 1 otherwise with value 0.

-------------
Output
-------------

***************************
Wi-Fi Onboarding Using BLE
***************************

WLAN MAC Address : E8:E8:B7:9F:D4:90
WLAN Firmware    : wl0: Apr 26 2021 04:04:15 version 13.10.271.265 (aa096f9 CY) FWID 01-29e05f8
WLAN CLM         : API: 18.2 Data: 9.10.0 Compiler: 1.36.1 ClmImport: 1.34.1 Creation: 2021-04-26 04:01:15
WHD VERSION      : v1.94.0 : v1.94.0 : GCC 7.3 : 2021-04-27 16:54:34 +0800
Bluetooth Management Event:     BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT
Bluetooth Management Event:     BTM_ENABLED_EVT
Local Bluetooth Address: 00:XX:XX:00:00:00
GATT status:    WICED_BT_GATT_SUCCESS || WICED_BT_GATT_ENCRYPTED_MITM
Data not present in EMEEPROM
Bluetooth Management Event:     BTM_BLE_ADVERT_STATE_CHANGED_EVT

Advertisement state changed to BTM_BLE_ADVERT_UNDIRECTED_LOW
[9112] wiced_post_stack_init_cback(): BT sleep mode is NOT enabled
Bluetooth Management Event:     BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT
Bluetooth Management Event:     BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT
Bluetooth Management Event:     BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT

Connected: Peer BD Address: XX:XX:XX:XX:XX:XX
Bluetooth Management Event:     BTM_BLE_ADVERT_STATE_CHANGED_EVT

Advertisement state changed to BTM_BLE_ADVERT_OFF
Bluetooth Management Event:     BTM_BLE_CONNECTION_PARAM_UPDATE
Bluetooth Management Event:     BTM_BLE_CONNECTION_PARAM_UPDATE
[app_gatts_req_read_handler] conn_id: 32768 handle:0x0003 offset:0 len:8
[app_gatts_req_read_handler] conn_id: 32768 handle:0x0005 offset:0 len:2
GATT write handler: handle:0x9 len:9
Wi-Fi SSID: XXXXXXXX
Bluetooth Management Event:     BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT
Bluetooth Management Event:     BTM_SECURITY_REQUEST_EVT
Bluetooth Management Event:     BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT
Bluetooth Management Event:     BTM_PAIRING_COMPLETE_EVT
Pairing Complete: SUCCESS
Bluetooth Management Event:     BTM_ENCRYPTION_STATUS_EVT
Encryption Status Event: SUCCESS
GATT write handler: handle:0xC len:13
Wi-Fi Password: XXXXXXXX
GATT write handler: handle:0xF len:1
Starting scan with SSID: XXXXXXXX
XXXXXXXX                               CY_WCM_SECURITY_WPA2_AES_PSK

Trying to connect SSID: XXXXXXXX, Password: XXXXXXXX
Successfully joined the Wi-Fi network
Notification not sent
