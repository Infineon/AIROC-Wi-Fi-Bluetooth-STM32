/*
 * Copyright 2020, Cypress Semiconductor Corporation or a subsidiary of
 * Cypress Semiconductor Corporation. All Rights Reserved.
 *
 * This software, associated documentation and materials ("Software"),
 * is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */


/** @file
 *
 */
#include <stdio.h>
#include <stdlib.h>

#include "cycfg.h"
#include "cyhal_uart.h"
#include "cyhal_gpio.h"

#include "bt_hci_interface.h"
#include "bt_mfg_test.h"
#include "bt_bus.h"

/*******************************************************
 *                      Macros
 ******************************************************/
#define TRANSPORT_TX_TASK_ID         (0)
#define TRANSPORT_RX_TASK_ID         (1)
#define TRANSPORT_TASK_NUM           (2)

#define RX_MEM_SIZE         (1040)
#define TX_CMD_MEM_SIZE     (264)

/*******************************************************
 *                    Constants
 ******************************************************/

/*******************************************************
 *                   Enumerations
 ******************************************************/

/*******************************************************
 *                 Type Definitions
 ******************************************************/

/*******************************************************
 *                    Structures
 ******************************************************/
typedef struct
{
    uint8_t* p_rx_mem;
    uint32_t rx_mem_size;
    uint8_t* p_tx_mem;
    uint32_t tx_mem_size;
} mem_cb_t;

/*******************************************************
 *               Static Function Declarations
 ******************************************************/

/*******************************************************
 *               Variable Definitions
 ******************************************************/
mem_cb_t mem_cb = { 0 };

/*******************************************************
 *               Function Definitions
 ******************************************************/
bool bt_trasport_mempool_init(void)
{
    mem_cb.tx_mem_size = TX_CMD_MEM_SIZE;
    mem_cb.rx_mem_size = RX_MEM_SIZE;
    mem_cb.p_tx_mem    = (uint8_t*)malloc((size_t)mem_cb.tx_mem_size);
    mem_cb.p_rx_mem    = (uint8_t*)malloc((size_t)mem_cb.rx_mem_size);

    if ((mem_cb.p_tx_mem == NULL) || (mem_cb.p_rx_mem == NULL))
    {
        APP_TRACE_DEBUG("Error malloc failed\n");
        return false;
    }

    return true;
}


/***************************************************************************************************
 * read_serial_data
 **************************************************************************************************/
static void read_serial_data(uint8_t* readbuf, uint32_t len)
{
    unsigned char buf[RX_MEM_SIZE];
    uint32_t      c;
    uint32_t      i = 0;

    memset(buf, 0, sizeof(buf));
    while (i < len)
    {
        c        = getchar();
        buf[i++] = c;
    }

    if (i > 0)
    {
        memcpy(readbuf, buf, i);
    }
}


/***************************************************************************************************
 * write_serial_data
 **************************************************************************************************/
static void write_serial_data(unsigned char* write_buf, uint32_t size)
{
    uint32_t i;

    if (size == 0)
    {
        return;
    }

    for (i = 0; i < size; i++)
    {
        putchar(write_buf[i]);
    }
}


/***************************************************************************************************
 * bt_rx_transport_task
 **************************************************************************************************/
void bt_rx_transport_task(cy_thread_arg_t arg)
{
    uint32_t            content_length = 0;
    hci_event_header_t* header;
    uint8_t*            p;

    /* Grab message from controller and pass it over to the PC */
    while (1)
    {
        p = mem_cb.p_rx_mem;

        /* Read event header */
        bt_bus_receive(p, sizeof(hci_event_header_t), CY_RTOS_NEVER_TIMEOUT);

        header = (hci_event_header_t*)p;
        p     += sizeof(hci_event_header_t);

        content_length = header->content_length;

        /* Read the remaining packet */
        if (content_length > 0)
        {
            bt_bus_receive(p, content_length, CY_RTOS_NEVER_TIMEOUT);
        }

        /* Send packet to the PC */
        write_serial_data((unsigned char*)header, (sizeof(hci_event_header_t) + content_length));
    }
}


/***************************************************************************************************
 * bt_tx_transport_task
 **************************************************************************************************/
void bt_tx_transport_task(cy_thread_arg_t arg)
{
    uint32_t              content_length = 0;
    hci_command_header_t* header;
    uint8_t*              p;

    /* Grab message from PC and pass it over to the controller */
    while (1)
    {
        p = mem_cb.p_tx_mem;

        /* Read HCI header */
        read_serial_data(p, sizeof(hci_command_header_t));

        header = (hci_command_header_t*)p;
        p     += sizeof(hci_command_header_t);

        content_length = header->content_length;

        /* Read the remaining packet */
        if (content_length > 0)
        {
            read_serial_data(p, content_length);
        }

        /* Send packet to the controller */
        bt_bus_transmit((unsigned char*)header, (sizeof(hci_command_header_t) + content_length));
    }
}
