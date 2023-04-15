------------------------------------------------------
STM32H747I-DISCO : Bluetooth LE Hello Sensor
------------------------------------------------------

This code example demonstrates the implementation of a simple Bluetooth Stack
functionality in GAP Peripheral role.  During initialization the app registers
with LE stack to receive various notifications including bonding complete,
connection status change and peer write.  Peer device can also write in to
client configuration descriptor of the notification characteristic.

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
1. Open ble_hello_sensor.ioc
2. Go to Project Manager --> Project tab and select "STM32CubeIDE" or EWARM
   depending on the Compiler option needed
3. Click "Generate Code" and open the generated project 
4. Build the project in the IDE selected.
5. Flash the binary and run the application.
6. Observe the messages on the UART terminal. 
   Use the ST-LINK (CN2) COM port to view the Bluetooth stack and application
   trace messages in the terminal window.
7. Test using the CySmart mobile app, do the following 
   a. Turn ON Bluetooth on your Android or iOS device
   b. Launch the CySmart app
   c. Swipe down on the CySmart app home screen to start scanning for BLE
      Peripherals; your device ("hello") appears in the CySmart app home screen.
      Select your device to establish a BLE connection. 
   d. Check Battery by clicking on Battery Profile
   e. Select GATT DB Profile , then "Unknown Service" and Enable Sensor
      Indication/Notification by Selecting the attribute with the UUID ending
      in 26 and clicking "Indicate/Notify" Button.


-------------
Output
-------------

Hello Sensor Start
wiced_bt_stack_init()
bt_post_reset_cback()
bt_post_reset_cback(): Change baudrate (3000000) for FW downloading
bt_update_controller_baudrate(): 3000000
bt_baudrate_updated_cback(): Baudrate is updated for FW downloading
bt_update_platform_baudrate(): 3000000
bt_start_fw_download(): FW ver = CYW43012C0_003.001.015.0213.0000_Generic_UART_37_4MHz_wlcsp_ref3_sLNA
bt_patch_download_complete_cback(): status = 1
bt_fw_download_complete_cback(): Reset baudrate to 115200
bt_update_platform_baudrate(): 115200
bt_fw_download_complete_cback(): Changing baudrate to 3000000
bt_update_controller_baudrate(): 3000000
bt_baudrate_updated_cback(): Baudrate is updated for feature
bt_update_platform_baudrate(): 3000000
bt_baudrate_updated_cback(): post-reset process is done
hello_sensor_management_cback: 16

wiced_post_stack_init_cback
hello_sensor_management_cback: 0

hello_sensor_application_init

wiced_bt_gatt_register: 0

wiced_bt_gatt_db_init 0

hello_sensor_management_cback: 18

Advertisement State Change: 3

wiced_bt_start_advertisements 0

wiced_post_stack_init_cback(): BT sleep mode is NOT enabled
hello_sensor_timeout: 1, ft:0

hello_sensor_timeout: 2, ft:0
