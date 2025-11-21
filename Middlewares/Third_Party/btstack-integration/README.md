# btstack-integration Overview

The btstack-integration hosts platform adaptation layer (porting layer) between AIROC™ BT Stack and Cypress Abstraction Layers (CYHAL and CYOSAL) for different cypress hardware platforms. This layer implements/invokes the interfaces defined by BTSTACK which is a platform agnostic library, to provide OS services and to enable communication with the BT controller.

While these adaptation layer interfaces and functionality are the same, the hardware platform they run on differ in the inter processor communication mechanisms used for communication between host stack and controller.

This asset provides three components, COMPONENT_BLESS-IPC, COMPONENT_BTSS-IPC and COMPONENT_HCI-UART porting layer components for various hardware platforms (such as psoc6-bless, 20829, psoc6+43xx respectively) and IPC methods (IPC_PIPE, IPC_BTSS and UART respectively) supported. Additionally, with little or no modifications, COMPONENT_HCI-UART can be used as platform adaptation layer for the hardware platforms which use Cypress Abstraction Layers (CYHAL and CYOSAL) and FreeRTOS and where Bluetooth chip is connected over UART. Below table points to further documentation for using each of these components.

Please refer below each COMPONENT's README for more details.

|  Component's name  | Hardware platforms | IPC Method |                                          IPC Method link                                          |                              Portinglayer documentation link                              |
| :-----------------: | :----------------: | :--------: | :------------------------------------------------------------------------------------------------: | :---------------------------------------------------------------------------------------: |
| COMPONENT_BLESS-IPC |    psoc6-bless    |  IPC_PIPE  | https://infineon.github.io/mtb-pdl-cat1/pdl_api_reference_manual/html/group__group__ipc__pipe.html | https://github.com/Infineon/btstack-integration/blob/master/COMPONENT_BLESS-IPC/README.md |
| COMPONENT_BTSS-IPC |    psoc6-20829    |  IPC_BTSS  |  https://infineon.github.io/mtb-pdl-cat1/pdl_api_reference_manual/html/group__group__ipc__bt.html  | https://github.com/Infineon/btstack-integration/blob/master/COMPONENT_BTSS-IPC/README.md |
| COMPONENT_HCI-UART |     psoc6-43xx     |    UART    | https://infineon.github.io/mtb-pdl-cat1/pdl_api_reference_manual/html/group__group__scb__uart.html | https://github.com/Infineon/btstack-integration/blob/master/COMPONENT_HCI-UART/README.md |

Please refer IPC (Inter Process Communication) section of https://infineon.github.io/mtb-pdl-cat1/pdl_api_reference_manual/html/index.html for documentation of IPC mechanisms.

## Updates from code size optimization 
### Code size optimizations for Mesh/Beaconing Apps 
Applications which do not create GATT/ACL connections or those which do not need SMP may override the default initializations done in the stack by defining the macro **DISABLE_DEFAULT_BTSTACK_INIT** in the application Makefile
```
  #Define DISABLE_DEFAULT_BTSTACK_INIT
  DEFINES+=DISABLE_DEFAULT_BTSTACK_INIT
```

### Code size optimizations for SMP server and SMP client applications
By default, porting layer initialises btstack for both SMP server and SMP client. However, if the application is going to be SMP server only, then there is no need to initialise SMP client module and vice versa. This would help to save the memory involved in initialising SMP client or SMP server module depending on the configuartion.

To disable SMP server module, define ENABLE_SMP_SERVER_MODULE to 0 in application's makefile.
```
  DEFINES+=ENABLE_SMP_SERVER_MODULE=0
```

To disable SMP client module, define ENABLE_SMP_CLIENT_MODULE to 0 in applications's makefile.
```
  DEFINES+=ENABLE_SMP_CLIENT_MODULE=0
```
If SMP is not required, both SMP server and SMP client modules can be disabled in application's makefile.
```
  DEFINES+=ENABLE_SMP_SERVER_MODULE=0
  DEFINES+=ENABLE_SMP_CLIENT_MODULE=0
```

### Stack size optimization on 20829, 89829 and P6+BLESS platform
Stack size of bt_task (BTU_TASK_STACK_SIZE) is optimized to work for wide range of applications. It is possible to further optimize it by overriding it from application's makefile.
```
  DEFINES+=BTU_TASK_STACK_SIZE=0x1000
```

## Requirements for enabling GATT Robust Caching
GATT Server applications which need to implement GATT Robust Caching will need to invoke **wiced_bt_gatt_server_enable_caching** in the **BTM_ENABLED_EVT**

## Requirements for enabling GATT Data Signing
GATT applications work with signed data will need to invoke **wiced_bt_gatt_enable_signing** in the **BTM_ENABLED_EVT**

## Handling Stack, Controller and Porting layer Exceptions
AIROC Bluetooth Stack, Controller and porting layer can generate Exceptions for buffer corruption, controller crash etc. If AIROC Bluetooth Stack version is v4.0 or above, the default behaviour is to reset the system whenever an exception occurs. This is taken care by the porting layer (btstack-integration). If the application prefers to modify this behaviour, it can register an exception callback function by calling **wiced_bt_set_exceptions_callback()** and implement a custom logic in the application layer.

All controller and poring layer exceptions are handled by **cybt_platform_exception_handler()** and all stack exceptions are handled by **host_stack_exception_handler()** which are respectively defined in cybt_platform_main.c and cybt_host_stack_platform_interface.c files of porting layer. To implement a custom logic for handling the exceptions in the application layer, User can refer **platform_default_exception_handling()** and **host_stack_exception_handler()** which are present in above mentioned files.

For more specific info about exception handling functions and input parameters, please check the API documentation.

**Note:** Exception handling is currently supported on psoc6-20829 platform (i.e. CYW29829B0 and CYW89829B0 controllers) only.

## Platform HCI transport config

Besides of stack settings, HCI configuration is also required to specify,
including pins, format and UART baudrate. Please refer to cybt_platform_config.h
for more detail:

- structure **cybt_platform_config_t**
- API **cybt_platform_config_init( )**

The API **cybt_platform_config_init( )** shall be invoked prior to
**wiced_bt_stack_init( )**

## How to enable trace log?

- *Compile time definition:* Please refer to cybt_platform_trace.h which can set individual trace log level
  - Application makefile add **DEFINES+=CYBT_PLATFORM_TRACE_ENABLE=0** to disable debug log
  - Application makefile add **DEFINES+=CYBT_PLATFORM_TRACE_ENABLE=1** to enable debug log
  - Default set all category is CYBT_TRACE_LEVEL_ERROR
- *Run time update:* Dynamic set trace level by using API **cybt_platform_set_trace_level(id, level);**
  - For example: set all catoegories as debug level
    ```
    cybt_platform_set_trace_level(CYBT_TRACE_ID_ALL, CYBT_TRACE_LEVEL_DEBUG);
    ```

## How to print debug traces

Any generic terminal emulator tools such as putty/teraterm/etc. can be used for printing application or generic traces. But if the requirement is to print platform or HCI traces along with application traces, then it is adviced to use airoc-hci-transport with BTspy tool.

[The airoc-hci-transport](https://github.com/Infineon/airoc-hci-transport) is a utility library, offering a comprehensive solution for extracting debug traces and facilitating communication with external host application. This library is used along with the [BTSPY](https://github.com/Infineon/btsdk-utils) trace utility.<br />
Find more info in [airoc-hci-transport](https://github.com/Infineon/airoc-hci-transport) README.MD

NOTE: If IAR compiler is used, printf in the application calls **__write** function present in the cybt_debug_uart.c to output individual characters. As it receives one character at a time, we use a buffer (printf_buf_iar) to store the characters. The default size of the buffer is 128 (PRINTF_BUF_SIZE_IAR). Application can set it to a convenient size by definining it in the application makefile.

Note: airoc-hci-transport is supported with btstack-integration release-v5.0.0 or more.

## Debug UART TX/RX Task Priority
In DEBUG UART, the default priority set for both TX and RX tasks is CY_RTOS_PRIORITY_ABOVENORMAL. Generally, it is expected to run DEBUG UART at a lower priority compared to bt_task ensure smooth working of bluetooth activities. The default priority set for bt_task is CY_RTOS_PRIORITY_HIGH in cybt_platform_task.h
If DEBUG UART TX or RX task require more priority, it can be updated using the below MAROSs defined in cybt_debug_uart.c.


```
#define DEBUG_UART_TX_TASK_PRIORITY      (CY_RTOS_PRIORITY_ABOVENORMAL)
#define DEBUG_UART_RX_TASK_PRIORITY     (CY_RTOS_PRIORITY_ABOVENORMAL)
```

## Random address management
btstack-integration v6.1.0 or later allows application to control IRK (Identity Resolving Key).

Porting layer uses below listed APIs provided by AIROC™ BT Stack:
- **wiced_ble_create_local_identity_keys** to create new local keys. The created keys are returned in **BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT**. The returned keys are expected to be stored by the application.
- **wiced_ble_init_ctlr_private_addr_generation** to enable controller based local address generation 
- **wiced_ble_init_host_private_addr_generation** to enable host based address local address generation
- **wiced_ble_read_local_identity_keys_from_app** to read stored local keys from the app

By default, **ENABLE_CREATE_LOCAL_KEYS** MACRO is set to **1**. This allows porting layer to retrieve/create the local identity keys stored by the application and start either controller or host based local address generation/resolution.

The application can based on its requirements, choose to set **ENABLE_CREATE_LOCAL_KEYS** to **0**. In this case the application needs to invoke these APIs to create, store and initialize local address generation.

© Infineon Technologies, 2022.
