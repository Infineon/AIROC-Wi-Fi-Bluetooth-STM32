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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "cycfg.h"
#include "cyhal_uart.h"
#include "cyhal_gpio.h"

#include "bt_hci_interface.h"
#include "bt_mfg_test.h"
#include "bt_bus.h"

/*******************************************************
 *                      Macros
 ******************************************************/
/* Verify if bt_bus function returns success.
 * Otherwise, returns the error code immediately.
 * Assert in DEBUG build.
 */
#define VERIFY_RETVAL(function_call) \
do \
{ \
    bool verify_result = (function_call); \
    if ( verify_result != true ) \
    { \
        APP_TRACE_DEBUG(("[%s %d] VERIFY FAILED\n", __func__,__LINE__)); \
        return verify_result; \
    } \
} while ( 0 )

/* Verify if HCI response is expected.
 * Otherwise, returns HCI unexpected response immediately.
 * Assert in DEBUG build.
 */
#define VERIFY_RESPONSE(a, b, size) \
{ \
    if ( memcmp( (a), (b), (size) ) != 0 ) \
    { \
        APP_TRACE_DEBUG( "HCI unexpected response\n"); \
        return false; \
    } \
}

/*******************************************************
 *                    Constants
 ******************************************************/
#define DEFAULT_READ_TIMEOUT (100)
#define STABILIZATION_DELAY  (250)  /* delay before sending any new command (ms) */

/*******************************************************
 *                   Enumerations
 ******************************************************/

/*******************************************************
 *                 Type Definitions
 ******************************************************/

/*******************************************************
 *                    Structures
 ******************************************************/

/*******************************************************
 *               extern Function Declarations
 ******************************************************/


/*******************************************************
 *               Variable Definitions
 ******************************************************/
/* *SUSPEND-FORMATTING* */
static const hci_command_header_t hci_commands[] =
{
    [HCI_CMD_RESET] =
        {
            .packet_type    = (hci_packet_type_t) 0x1,
            .op_code        = HCI_CMD_OPCODE_RESET,
            .content_length = 0x0
        },
    [HCI_CMD_DOWNLOAD_MINIDRIVER] =
        {
            .packet_type    = (hci_packet_type_t) 0x1,
            .op_code        = HCI_CMD_OPCODE_DOWNLOAD_MINIDRIVER,
            .content_length = 0x0
        },
    [HCI_CMD_WRITE_RAM] =
        {
            .packet_type    = (hci_packet_type_t) 0x1,
            .op_code        = HCI_CMD_OPCODE_WRITE_RAM,
            .content_length = 0x0
        },
    [HCI_CMD_LAUNCH_RAM] =
        {
            .packet_type    = (hci_packet_type_t) 0x1,
            .op_code        = HCI_CMD_OPCODE_LAUNCH_RAM,
            .content_length = 0x0
        },
    [HCI_CMD_UPDATE_BAUDRATE] =
        {
            .packet_type    = (hci_packet_type_t) 0x1,
            .op_code        = HCI_CMD_OPCODE_UPDATE_BAUDRATE,
            .content_length = 0x6
        },
};

static const hci_event_extended_header_t expected_hci_events[] =
{
    [HCI_CMD_RESET] =
        {
            .header        = { .packet_type = (hci_packet_type_t) 0x4, .event_code = 0xE, .content_length = 0x4 },
            .total_packets = 0x1,
            .op_code       = HCI_CMD_OPCODE_RESET,
            .status        = 0x0
        },
    [HCI_CMD_DOWNLOAD_MINIDRIVER] =
        {
            .header        = { .packet_type = (hci_packet_type_t) 0x4, .event_code = 0xE, .content_length = 0x4 },
            .total_packets = 0x1,
            .op_code       = HCI_CMD_OPCODE_DOWNLOAD_MINIDRIVER,
            .status        = 0x0
        },
    [HCI_CMD_WRITE_RAM] =
        {
            .header        = { .packet_type = (hci_packet_type_t) 0x4, .event_code = 0xE, .content_length = 0x4 },
            .total_packets = 0x1,
            .op_code       = HCI_CMD_OPCODE_WRITE_RAM,
            .status        = 0x0
        },
    [HCI_CMD_LAUNCH_RAM] =
        {
            .header        = { .packet_type = (hci_packet_type_t) 0x4, .event_code = 0xE, .content_length = 0x4 },
            .total_packets = 0x1,
            .op_code       = HCI_CMD_OPCODE_LAUNCH_RAM,
            .status        = 0x0
        },
    [HCI_CMD_UPDATE_BAUDRATE] =
        {
            .header        = { .packet_type = (hci_packet_type_t) 0x4, .event_code = 0xE, .content_length = 0x4 },
            .total_packets = 0x1,
            .op_code       = HCI_CMD_OPCODE_UPDATE_BAUDRATE,
            .status        = 0x0
        },
};
/* *RESUME-FORMATTING* */

/***************************************************************************************************
 * bt_firmware_download
 **************************************************************************************************/
bool bt_firmware_download(const uint8_t* firmware_image, uint32_t size)
{
    uint8_t*                    data             = (uint8_t*)firmware_image;
    uint32_t                    remaining_length = size;
    hci_event_extended_header_t hci_event;

    /* Send 'Reset' command */
    VERIFY_RETVAL(bt_bus_transmit((const uint8_t*)&hci_commands[HCI_CMD_RESET],
                                  sizeof(hci_command_header_t)));
    VERIFY_RETVAL(bt_bus_receive((uint8_t*)&hci_event, sizeof(hci_event), CY_RTOS_NEVER_TIMEOUT));
    VERIFY_RESPONSE(&hci_event, &expected_hci_events[HCI_CMD_RESET], sizeof(hci_event));

    /* Send hci_download_minidriver command */
    VERIFY_RETVAL(bt_bus_transmit((const uint8_t*)&hci_commands[HCI_CMD_DOWNLOAD_MINIDRIVER],
                                  sizeof(hci_command_header_t)));
    VERIFY_RETVAL(bt_bus_receive((uint8_t*)&hci_event, sizeof(hci_event), CY_RTOS_NEVER_TIMEOUT));
    VERIFY_RESPONSE(&hci_event, &expected_hci_events[HCI_CMD_DOWNLOAD_MINIDRIVER],
                    sizeof(hci_event));

    /* The firmware image (.hcd format) contains a collection of hci_write_ram command + a block of
     * the image, followed by a hci_write_ram image at the end. Parse and send each individual
     * command and wait for the response.
     * This is to ensure the integrity of the firmware image sent to the bluetooth chip.
     */
    while (remaining_length)
    {
        uint32_t data_length                = data[2] + 3; /* content of data length + 2 bytes of
                                                              opcode and 1 byte of data length */
        uint8_t              residual_data  = 0;
        hci_command_opcode_t command_opcode = *(hci_command_opcode_t*)data;
        uint8_t              temp_data[259]; /* content of 1 byte of packet type + 2 bytes of opcode
                                              + 1 byte of data length + max data 255 bytes*/

        memset(&hci_event, 0, sizeof(hci_event));
        memset(temp_data, 0, sizeof(temp_data));

        /* 43438 requires the packet type before each write RAM command */
        temp_data[0] = HCI_COMMAND_PACKET;
        memcpy(&temp_data[1], data, data_length);

        /* Send hci_write_ram command.
         * The length of the data immediately follows the command opcode */
        VERIFY_RETVAL(bt_bus_transmit((const uint8_t*)temp_data, data_length + 1));
        VERIFY_RETVAL(bt_bus_receive((uint8_t*)&hci_event, sizeof(hci_event),
                                     CY_RTOS_NEVER_TIMEOUT));

        switch (command_opcode)
        {
            case HCI_CMD_OPCODE_WRITE_RAM:
                VERIFY_RESPONSE(&hci_event, &expected_hci_events[HCI_CMD_WRITE_RAM],
                                sizeof(hci_event));

                /* Update remaining length and data pointer */
                data             += data_length;
                remaining_length -= data_length;
                break;

            case HCI_CMD_OPCODE_LAUNCH_RAM:
                VERIFY_RESPONSE(&hci_event, &expected_hci_events[HCI_CMD_LAUNCH_RAM],
                                sizeof(hci_event));

                /* All responses have been read. Now let's flush residual data if any and reset
                   remaining length */
                while (bt_bus_receive(&residual_data, sizeof(residual_data),
                                      DEFAULT_READ_TIMEOUT) == true)
                {
                    ///APP_TRACE_DEBUG("residual_data=%x \n", residual_data);
                }

                remaining_length = 0;
                break;

            default:
                return false;
        }
    }

    APP_TRACE_DEBUG("patchram done\n");

    cy_rtos_delay_milliseconds(STABILIZATION_DELAY);

    /* Send 'Reset' command */
    VERIFY_RETVAL(bt_bus_transmit((const uint8_t*)&hci_commands[HCI_CMD_RESET],
                                  sizeof(hci_command_header_t)));
    VERIFY_RETVAL(bt_bus_receive((uint8_t*)&hci_event, sizeof(hci_event), CY_RTOS_NEVER_TIMEOUT));

    VERIFY_RESPONSE(&hci_event, &expected_hci_events[HCI_CMD_RESET], sizeof(hci_event));

    return true;
}
