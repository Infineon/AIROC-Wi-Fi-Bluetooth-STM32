### AIROC&trade; BT/BLE Host Stack solution
AIROC&trade; BT/BLE stack provides Bluetooth&reg; functionality with high performance and less resource consumption. It targets on IoT products, especially for embedded devices.

### What's Included?
This release of AIROC&trade; BT/BLE stack solution package includes as following:
* Platform and Operating system porting layers for P6+43xx, P6+555xx, 20829, 89829 and P6+BLESS.

#### v6.2.1
btstack-integration v6.2.1 contains below changes:
* Updated porting layer to support BTSTACK v4.1.4 and later
* Added new API, wiced_bt_serialize_function_from_isr, as wiced_bt_app_serialize_function is deprecated

#### v6.2.0
btstack-integration v6.2.0 contains below changes:
* Requires BTSTACK v4.1.3 and later
* Fixed issue of application receiving a BTM_ENABLE_EVT before creation of a Random address when host random address generation is used. Affects P6+BLESS and any other platform using `wiced_ble_init_host_private_addr_generation` to enable Host Address generation.
* Added macro `ENABLE_HOST_RPA_GENERATION` to force host based address generation. Application can set `DEFINES +=ENABLE_HOST_RPA_GENERATION=1` for host based RPA generation
* Modified the IPC send message retry mechanism on P6+BLESS platform
* Updated COMPONENT_HCI-UART code to support next major version of HAL


#### v6.1.1
btstack-integration v6.1.1 contains below changes:
* Fixed an issue in P6+BLESS platform by enabling Host Address generation. Updated to support BTSTACK v4.1.1 and later

#### v6.1.0
btstack-integration v6.1.0 contains below updates:
* Updated porting layer to support BTSTACK v4.1.0 and later
* Added LLVM compiler support
* Updated to print critical error logs
* Implemented callback for returning random number
* Improved robustness of IPC communication in P6+BLESS platform

#### v6.0.0
btstack-integration v6.0.0 contains below updates:
* Updated porting layer to support BTSTACK v4.0.0 and later. Please note that porting layer is not compatible with BTSTACK v3.X.X
* Made provision for application to disable SMP server or SMP client modules of BTSTACK as per the requirement
* Made provision for application to override stack size of bt_task (BTU_TASK_STACK_SIZE) on 20829, 89829 and P6+BLESS platform
* Fixed data corruption in fragmented notification packets on P6+BLESS platform

#### v5.0.2
btstack-integration v5.0.2 is a patch release with below mentioned fixes and enhancements:
* Fixed an issue in ISOC path in P6+43xx platform
* Fixed a minor warning in code
* To control the time taken for FW patch download on P6+555xx platform, updated the code to fetch baud rate from BSP.
* Improved exception data printing for 20829/89829 platforms
* Fixed the memory leak issue observed after long time with continuous AIROC stack init and de-init operations on 20829/89829 platform

#### v5.0.1
btstack-integration v5.0.1 is a patch release with below mentioned enhancements:
* Added support for P6+555xx Platform
* Increased the controller heap size allocated in P6+BLESS platform

#### v5.0.0
btstack-integration v5.0.0 contains below updates:
* Removed controller FW patches from btstack-integration asset. Controller FW patches are now available as part of separate assets.
* Removed debug uart library, which is used for getting HCI traces and for communication with external host, from btstack-integration. It is now available as a separate airoc-hci-transport asset.
* Fixed the issue of device sometimes going in bad state after very long run
* Added support for ISOC in P6+43xx platform
* Optimized start up time in P6+43xx platform
* Changes to notify the application and restart the system after controller crash if btstack version is v4.0 or more. Applicable on 20829B0 and 89829B0 only.

#### v4.7.0
btstack-integration v4.7.0 contains below updates:
* Updated BT Controller FW patches for 20829B0 and 89829B0
* Added BT Controller FW patch for new Inventek Board ‘ISM43439-WBP-L151-EVB’ board having new module  ‘ISM43439-WBP-L151’
* Modified IRQ settings for DMA and non-DMA transfers
* Updated P6+43xx platform code to support TARGET_KIT_XMC72_EVK_MUR_43439M2
* Updated P6+43xx platform to fix an initialization issue observed with AIROC-Wi-Fi-Bluetooth-STM32 pack
* Fixed an issue observed during printing Application traces while using IAR toolchain
* Updated P6+43xx platform to use freertos timer instead of LP Timer
* Removed default callback function used for registering HCI traces on P6+43xx platform. Application now needs to register for HCI traces using wiced_bt_dev_register_hci_trace() API
* Updated BT Controller FW patch for P6+4373(Sterling LWB5Plus module)

#### v4.6.0
btstack-integration v4.6.0 contains below updates:
* Added BT Controller FW patches with ISOC and PAWR support for 20829B0
* Added BT Controller FW patches with Generic, PAWR and WBMS support for 89829B0
* Updated generic BT Controller FW patch for 20829B0
* Added support in 20829B0 for AIROC&trade; BT/BLE stack deinitialization
* Minor fix related to issue observed during AIROC&trade; BT/BLE stack deinitialization on P6+43xx platform
* Removed 20829A0 BT Controller FW patches

#### v4.5.0
btstack-integration v4.5.0 contains below updates:
* Updated BT Controller FW patch for 20829B0
* Minor modifications to adhere MISRA C compliance
* Fixes related to SCO handling in P6+43xx platform
* Changes to support DMA on 20829 and P6+BLESS platforms
* Modified debug UART for 20829 and P6+BLESS platforms

#### v4.4.0
btstack-integration v4.4.0 contains below updates:
* Added BT Controller FW patch for 43022
* Updated BT Controller FW patch for 20829B0
* Improvements to reduce start up time for 20829B0
* Added platform support for wiced_bt_app_serialize_function()
* Corrected a check related to default stack initialization

#### v4.3.1
btstack-integration v4.3.1 is a patch release with below mentioned enhancements and fixes:
* Added BT controller firmware for 20829B0
* Increased number of IPC retries
* Fixed memory alignment issue observed with ARM compiler

#### v4.3.0
btstack-integration v4.3.0 contains below updates:
* Added FW patch from Laird(LWB+) for 43439
* Added FW patches from Murata for 4373-2AE and 4373-2BC
* Added code size optimization changes in porting layer
* Added APIs to get IPC operation failure details
* Added changes to separate out HCI BTSpy traces from debug UART logs

#### v4.2.0
btstack-integration v4.2.0 contains below updates:
* Updated BT controller firmware for 20829
* Enabled FW core dump for 20829
* Added DSRAM (DeepSleep RAM) callback registration APIs for 20829
* Reduced debug UART (used for gettig HCI traces) memory size from 30K to 6K for 20829 and P6+BLESS
* Instead of using hardcoded value, read maximum number of connection count set by Application in BT configurator and use it in P6+BLESS porting layer

#### v4.1.1
btstack-integration v4.1.1 contains below update:
* Updated porting layer of P6+43xx to work with the changes done in HAL library

#### v4.1.0
btstack-integration v4.1.0 contains below updates:
* Updated BT controller firmware for 20829 and P6+BLESS
* Added ISOC support for 20829

#### v4.0.0
* Initial release of btstack-integration which has HCI-UART, BLESS-IPC & BTSS-IPC components

| Component | Hardware Platform | Description |
| :-------: | :---------------: | :---------: |
| COMPONENT_BLESS-IPC | P6 + BLESS | Represents BLESS protocol talking over IPC |
| COMPONENT_BTSS-IPC | 20829 | Represents BTSS protocol talking over IPC |
| COMPONENT_HCI-UART | P6 + 43xx | Represents HCI as the protocol talking over UART transport |

bluetooth-freertos v3.4.0 has been included as COMPONENT_HCI-UART in btstack-integration. Hence btstack-integration has initial version as 4.0.

### Supported Software and Tools
This version of Infineon BT/BLE stack API was validated for compatibility with the following Software and Tools:

| Software and Tools                        | Version |
| :---                                      | :----:  |
| ModusToolbox™ Software Environment        | 3.2.0   |
| GCC Compiler                              | 11.3.1  |
| IAR Compiler                              | 8.42.1  |
| ARM Compiler                              | 6.16    |
| FreeRTOS                                  | 10.5.0  |

---
© Infineon Technologies, 2022.
