-------------------------------------------------------------------------------------
STM32H747I-DISCO : WLAN manufacturing test application (Wifi-Mfg-Tester) for FreeRTOS
-------------------------------------------------------------------------------------

The Wifi-Mfg-Tester v3.0.1 is used to validate the WLAN firmware and radio
performance of Wi-Fi chips.

The Wifi-Mfg-Tester acts as a transport layer between the host "wl tool" and
the WLAN firmware, and receives the commands from the wl tool and forwards
them to the WLAN firmware using IOVAR/IOCTL commands. It also relays the
response received back from the WLAN firmware.

The wl tool binaries for testing the WLAN firmware are also included in this
application repository.

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

This application requires the WL tool running on a Windows PC and uses the
UART port for communication with the target. The pre-built executables for the
WL tool are available in the *wl-tool-bin/* directory.

1. Go to the WL tool directory:
   cd wl-tool-bin/

2. Reset the board by pressing the reset button.

3. Run the command on Windows host for the WLAN chip on the target board:
   wl4343x.exe --serial <port> ver`

   For example:
   wl4343x.exe --serial 139 ver

4. Observe the output of the command.

   The list of WL commands which can be retrieved by typing `--help`. Partial
   output of the command and display is as follows:

   # wl4343x.exe --serial 139 --help

   Usage: wl4343WA1.exe [-a|i <adapter>] [-h] [-d|u|x] <command> [arguments]

		-h        this message and command descriptions
		-h [cmd]  command description for cmd
		-a, -i    adapter name or number
		-d        output format signed integer
		-u        output format unsigned integer
		-x        output format hexadecimal

		ver     get version information

		cmds    generate a short list of available commands

		ioctl_echo
			check ioctl functionality

		up      reinitialize and mark adapter up (operational)

		down    reset and mark adapter down (disabled)

		out     mark adapter down but do not reset hardware(disabled)
				On dual-band cards, cards must be band-locked before use.

		clk     set board clock state. return error for set_clk attempt if the driver is not down
				0: clock off
				1: clock on

		restart Restart driver.  Driver must already be down.

		reboot  Reboot platform
