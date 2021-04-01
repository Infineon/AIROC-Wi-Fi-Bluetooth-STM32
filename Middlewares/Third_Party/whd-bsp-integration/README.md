# WiFi Host Driver Board Support Package Integration

## Overview

This library provides some convenience functions for connecting the WiFi Host Driver (WHD) library to a Board Support Package (BSP) that includes a WLAN chip. This library initializes the hardware and passes a reference to the communication interface on the board into WHD. It also sets up the LwIP based network buffers to be used for sending packets back and forth.

## Getting Started

To use this library:
1. Implement the cy_network_process_ethernet_data() function. This should be something similar to the example below.
```cpp
/* This needs to be the same item as passed to netifapi_netif_add() */
static struct netif *default_interface = NULL;

void cy_network_process_ethernet_data(whd_interface_t iface, whd_buffer_t buf)
{
    if (default_interface != NULL)
    {
        if (default_interface->input(buf, default_interface) != ERR_OK)
            cy_buffer_release(buf, WHD_NETWORK_RX);
    }
    else
    {
        cy_buffer_release(buf, WHD_NETWORK_RX);
    }
}
```
2. Include a reference to `cybsp_wifi.h`.
3. Call cybsp_wifi_init_primary() to initialize the interface. This needs to be done after having called cybsp_init().

## Features

* APIs for setting up the WHD interface with the BSP's SDIO interface.
    * Initialize a primary WiFi interface
    * Initialize an optional secondary WiFi interface
    * Cleanup interfaces
* APIs for connecting WHD to LwIP memory buffers (whd_buffer_funcs_t)
* Framework for connecting WHD to LwIP network interface (whd_netif_funcs_t)

## More information
* [API Reference Guide](https://cypresssemiconductorco.github.io/whd-bsp-integration/html/modules.html)
* [Cypress Semiconductor, an Infineon Technologies Company](http://www.cypress.com)
* [Cypress Semiconductor GitHub](https://github.com/cypresssemiconductorco)
* [ModusToolbox](https://www.cypress.com/products/modustoolbox-software-environment)

---
© Cypress Semiconductor Corporation, 2019-2020.
