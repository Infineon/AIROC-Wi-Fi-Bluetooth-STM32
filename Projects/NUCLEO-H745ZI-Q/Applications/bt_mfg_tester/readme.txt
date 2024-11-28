-------------------------------------------------------------------------------------
STM32H747I-DISCO : Bluetooth Manufacturing Test Application for FreeRTOS
-------------------------------------------------------------------------------------

The Bluetooth Manufacturing Test Application is used to validate the Bluetooth Firmware
and RF performance of Bluetooth BR/EDR/LE devices.

The Bluetooth MFG Application acts as a transport layer between the host "WMBT tool"
and Bluetooth Firmware. Mfg Test Application receive commands from the WMBT tool and
forwards them to the Bluetooth firmware. The Bluetooth MFG Application also relays
the response received back from Bluetooth firmware.

There are 2 parts of the functions for testing:

1. On the PC side, Application (WMBT) running on PC that will send HCI commands and
receive HCI events to PSoC board. The SIG defined BLE testing with below 3 standard
HCI commands, to test the LE HW functionalities.

   A. LE Transmitter Test Command

   B. LE Receiver Test Command

   C. LE Test End Command

2. On the PSoC side, there will be another application(MFG app) that will do

   A. Bluetooth FW download. This is to download Bluetooth firmware into the
      controller, so it can have proper controller HW and RF configured.

   B. Route the HCI commands packets received from PC app to another STM32 UART
      that with BT controller connected.

   C. App received the HCI event from BT controller and route to STM32 UART to
      host PC application.

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
Build Instructions
---------------------
1. Open wifi_mfg_tester.ioc
2. Go to Project Manager --> Project tab and select "STM32CubeIDE" or EWARM
   depending on the Compiler option needed
3. Click "Generate Code" and open the generated project 
4. Build the project in the IDE selected.
5. Flash the binary and run the application.

---------------------
Using the application
---------------------

--------------
Hardware Setup
--------------

The application running on a STM32H747I-DISCO and the test setup are shown in
the mfg-test-architecture.png file.

--------------
Software Setup
--------------

1. This application requires WMBT Tool running on a windows PC and uses UART
   port for communication with target. The pre-built executables for WMBT Tool
   are available in wmbt-tool-bin/ directory, which sync from
   https://github.com/cypresssemiconductorco/btsdk-utils and the user guide,
   https://www.cypress.com/file/298091/download.

2. IQxel tool as transmitter to send fixed count test packet which to ensure
   whatever is sent from the transmitter would received by the receiver, without
   any error.

3. Using Sniffer to ensure whatever is test packet is in same transmit channel,
   packet length and data patterns from transmitter.

4. Better to test in a shielded room to avoid interference.

---------------------
Using the Application
---------------------

1. Go to WMBT tool directory

2. Reset the Board by pressing SW1 button

3. Run the command on Windows Host for the proper BT Chip on target board.

4. Observe the output of the command

5. List of wmbt commands with BLE function which can be retrieved by typing --help
   Partial output of the command and display is below.

		Usage: wmbt reset COMx
		Usage: wmbt le_receiver_test COMx <rx_frequency>
		Usage: wmbt le_transmitter_test COMx <tx_frequency> <data_length> <data_pattern>
		Usage: wmbt le_test_end COMx
