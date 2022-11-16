/*
 * Copyright 2022, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
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

/** @file wifi_utility.cpp
 *  @brief This file contains the definition of Wi-Fi commands and implementation of the
 *  command handlers on Mbed.
 */
#ifndef DISABLE_COMMAND_CONSOLE_WIFI
#include "mbed.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "command_console.h"
#include "wifi_utility.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

#define CMD_CONSOLE_SSID_MAX_LENGTH         33 /* 32 is what 802.11 defines as longest possible name; +1 for the \0 */
#define CMD_CONSOLE_PASS_MAX_LENGTH         64

#ifndef CMD_CONSOLE_MAX_WIFI_RETRY_COUNT
#define CMD_CONSOLE_MAX_WIFI_RETRY_COUNT    5
#endif

#if defined(__ICCARM__)
#define WIFI_WEAK_FUNC            __WEAK
#elif defined(__GNUC__) || defined(__clang__) || defined(__CC_ARM)
#define WIFI_WEAK_FUNC            __attribute__((weak))
#else
#define WIFI_WEAK_FUNC           __attribute__((weak))
#endif

/******************************************************
 *               Function Declarations
 ******************************************************/
static nsapi_security_t wifi_utils_str_to_authtype(char* auth_str);
static const char* wifi_utils_authtype_to_str(nsapi_security_t sec);
static void print_scan_result(WiFiAccessPoint* accessPoints, int count);

/* Wi-Fi commands */
int join         (int argc, char* argv[], tlv_buffer_t** data);
int leave        (int argc, char* argv[], tlv_buffer_t** data);
int scan         (int argc, char* argv[], tlv_buffer_t** data);
int get_rssi     (int argc, char* argv[], tlv_buffer_t** data);
int ping         (int argc, char* argv[], tlv_buffer_t** data);

#define WIFI_COMMANDS_LIMITED_SET \
    { (char*) "join",           join,       2, NULL, NULL, (char*) "<ssid> <open|wpa|wpa2|wpa_wpa2|wpa3|wpa3_wpa2> [password] [channel]", (char*) "Join an AP.(This command is deprecated and it will be removed in the future. Please use wifi_join command)"}, \
    { (char*) "leave",          leave,      0, NULL, NULL, (char*) "", (char*) "Leave the connected AP.(This command is deprecated and it will be removed in the future. Please use wifi_leave command)"}, \
    { (char*) "scan",           scan,       0, NULL, NULL, (char*) "", (char*) "Scan all the Wi-FI AP in the vicinity.(This command is deprecated and it will be removed in the future. Please use wifi_scan command)"}, \
    { (char*) "get_rssi",       get_rssi,   0, NULL, NULL, (char*) "", (char*) "Get the received signal strength of the AP (client mode only).(This command is deprecated and it will be removed in the future. Please use wifi_get_rssi command)"}, \
    { (char*) "ping",           ping,       0, NULL, NULL, (char*) "<IP address> [timeout(ms)]", (char*) "ping to an IP address.(This command is deprecated and it will be removed in the future. Please use wifi_ping command)"}, \
    { (char*) "wifi_join",      join,       2, NULL, NULL, (char*) "<ssid> <open|wpa|wpa2|wpa_wpa2|wpa3|wpa3_wpa2> [password] [channel]", (char*) "Join an AP."}, \
    { (char*) "wifi_leave",     leave,      0, NULL, NULL, (char*) "", (char*) "Leave the connected AP."}, \
    { (char*) "wifi_scan",      scan,       0, NULL, NULL, (char*) "", (char*) "Scan all the Wi-FI AP in the vicinity."}, \
    { (char*) "wifi_get_rssi",  get_rssi,   0, NULL, NULL, (char*) "", (char*) "Get the received signal strength of the AP (client mode only)."}, \
    { (char*) "wifi_ping",      ping,       0, NULL, NULL, (char*) "<IP address> [timeout(ms)]", (char*) "ping to an IP address"}, \

/******************************************************
 *                    Constants
 ******************************************************/

static const cy_command_console_cmd_t wifi_command_table[] =
{
    WIFI_COMMANDS_LIMITED_SET
    CMD_TABLE_END
};

/******************************************************
 *               Variables Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

WIFI_WEAK_FUNC cy_rslt_t wifi_utility_init(void)
{
    cy_command_console_add_table(wifi_command_table);
    WIFI_INFO(("Wi-Fi module initialized...\n"));

    return CY_RSLT_SUCCESS;
}

int join(int argc, char* argv[], tlv_buffer_t** data)
{
    nsapi_error_t err;
    WiFiInterface* sta;
    char ssid[CMD_CONSOLE_SSID_MAX_LENGTH];
    char password[CMD_CONSOLE_PASS_MAX_LENGTH];
    nsapi_security_t sec_type;
    SocketAddress address;
    uint8_t channel = 0;
    int retry_count = 0;
    bool wifi_connected = false;
    nsapi_connection_status_t conn_status;

    sta = WiFiInterface::get_default_instance();
    if(sta == NULL)
    {
        WIFI_ERROR(("Failed to get Wi-Fi interface.\n"));
        return -1;
    }

    conn_status = sta->get_connection_status();
    WIFI_DEBUG(("Connection Status: %d\n", conn_status));
    if(conn_status == NSAPI_STATUS_LOCAL_UP || conn_status == NSAPI_STATUS_GLOBAL_UP)
    {
        WIFI_INFO(("Already connected. Leave and join again.\n"));
        return 0;
    }

    if(argc < 3)
    {
        WIFI_ERROR(("Too few arguments.\n"));
        return -1;
    }

    if(strlen(argv[1]) >= CMD_CONSOLE_SSID_MAX_LENGTH)
    {
        WIFI_ERROR(("SSID length should be lesser than [%d] bytes.\n", CMD_CONSOLE_SSID_MAX_LENGTH));
        return -1;
    }

    // ssid
    memcpy(ssid, argv[1], strlen(argv[1])+1);

    // security type
    sec_type = wifi_utils_str_to_authtype(argv[2]);
    if(sec_type == NSAPI_SECURITY_UNKNOWN)
    {
        return -1;
    }

    // password
    if(sec_type != NSAPI_SECURITY_NONE && argc > 3)
    {
        if(strlen(argv[3]) >= CMD_CONSOLE_PASS_MAX_LENGTH)
        {
            WIFI_ERROR(("Password length should be lesser than [%d] bytes.\n", CMD_CONSOLE_PASS_MAX_LENGTH));
            return -1;
        }
        memcpy(password, argv[3], strlen(argv[3])+1);
    }
    else if(sec_type == NSAPI_SECURITY_NONE)
    {
        password[0] = '\0';
    }
    else
    {
        WIFI_ERROR(("Too few arguments.\n"));
        return -1;
    }

    // channel
    if(argc > 4)
    {
        channel = atoi(argv[4]);
    }

    // connect to AP
    WIFI_INFO(("Connecting to AP please wait...\n"));
    for(retry_count = 0; retry_count < CMD_CONSOLE_MAX_WIFI_RETRY_COUNT; retry_count++)
    {
        err = sta->connect(ssid, password, sec_type, channel);
        if(err != NSAPI_ERROR_OK)
        {
            WIFI_INFO(("Failed to join AP [%d]. Retrying...\n", err));
            continue;
        }
        wifi_connected = true;
        WIFI_INFO(("Successfully connected to %s\n", argv[1]));
        break;
    }

    if(!wifi_connected)
    {
        WIFI_ERROR(("Exceeded maximum Wi-Fi connection attempts.\n"));
        return -1;
    }   

    // print IP Address
    err = sta->get_ip_address(&address);
    if(err != NSAPI_ERROR_OK)
    {
        WIFI_ERROR(("Failed to fetch IP Address. Res:%d\n", err));
        return -1;
    }
    WIFI_INFO(("IP Address: %s\n", address.get_ip_address()));

    return 0;
}

int leave(int argc, char* argv[], tlv_buffer_t** data)
{
    nsapi_error_t err;
    WiFiInterface* sta;
    nsapi_connection_status_t conn_status;
    sta = WiFiInterface::get_default_instance();
    if(sta == NULL)
    {
        WIFI_ERROR(("Failed to get Wi-Fi interface.\n"));
        return -1;
    }

    conn_status = sta->get_connection_status();
    WIFI_DEBUG(("Connection Status: %d\n", conn_status));
    if(conn_status == NSAPI_STATUS_DISCONNECTED)
    {
        WIFI_INFO(("Failed to leave. Not connected to any AP.\n"));
        return 0;
    }

    err = sta->disconnect();
    if(err != NSAPI_ERROR_OK)
    {
        WIFI_ERROR(("Failed to disconnect from AP. Res: %d\n", err));
        return -1;
    }
    WIFI_INFO(("Disconnected from AP.\n"));
    return 0;
}

int scan(int argc, char* argv[], tlv_buffer_t** data)
{
    nsapi_error_t err;
    WiFiInterface* sta;
    int count;

    sta = WiFiInterface::get_default_instance();
    if(sta == NULL)
    {
        WIFI_ERROR(("Failed to get Wi-Fi interface.\n"));
        return -1;
    }

    // get number of APs
    count = sta->scan(NULL, 0);
    if(count < 1)
    {
        WIFI_INFO(("No APs nearby.\n"));
        return 0;
    }

    WiFiAccessPoint* aps = new WiFiAccessPoint[count];
    err = sta->scan(aps, count);
    if(err < 0)
    {
        WIFI_ERROR(("Error while scanning. Res: %d", err));
        delete[] aps;
        return -1;
    }

    print_scan_result(aps, err);
    delete[] aps;

    return 0;
}

int get_rssi(int argc, char* argv[], tlv_buffer_t** data)
{
    WiFiInterface* sta;
    int8_t rssi;

    sta = WiFiInterface::get_default_instance();
    if(sta == NULL)
    {
        WIFI_ERROR(("Failed to get Wi-Fi interface.\n"));
        return -1;
    }

    rssi = sta->get_rssi();
    if(rssi == 0)
    {
        WIFI_ERROR(("Error in measuring RSSI.\n"));
        return -1;
    }

    WIFI_INFO(("RSSI: %d dBm\n", rssi));

    return 0;
}

// ping is currently not supported in Mbed
int ping(int argc, char* argv[], tlv_buffer_t** data)
{
    WIFI_INFO(("This command is currently not supported.\n"));
    return 0;
}

void print_scan_result(WiFiAccessPoint* accessPoints, int count)
{
    // print the details of APs in scan result
    WIFI_INFO(("#### Scan Results START ####\n\n"));

    WIFI_INFO(("SSID                 Security Type  RSSI(dBm)  Channel BSSID\n"));

    for(int i = 0; i < count; i++)
    {
        WIFI_INFO(("%-20s %-14s %-10d %-7d %02X:%02X:%02X:%02X:%02X:%02X\n",
        accessPoints[i].get_ssid(),
        wifi_utils_authtype_to_str(accessPoints[i].get_security()),
        accessPoints[i].get_rssi(),
        accessPoints[i].get_channel(),
        accessPoints[i].get_bssid()[0],
        accessPoints[i].get_bssid()[1],
        accessPoints[i].get_bssid()[2],
        accessPoints[i].get_bssid()[3],
        accessPoints[i].get_bssid()[4],
        accessPoints[i].get_bssid()[5]));
    }

    WIFI_INFO(("#### Scan Results END ####\n\n"));
}

nsapi_security_t wifi_utils_str_to_authtype(char* auth_str)
{
    if (strcmp(auth_str, "open") == 0)
    {
        return NSAPI_SECURITY_NONE;
    }
    else if(strcmp( auth_str, "wpa") == 0)
    {
        return NSAPI_SECURITY_WPA;
    }
    else if(strcmp(auth_str, "wpa2") == 0)
    {
        return NSAPI_SECURITY_WPA2;
    }
    else if(strcmp(auth_str, "wpa_wpa2") == 0)
    {
        return NSAPI_SECURITY_WPA_WPA2;
    }
    else if(strcmp(auth_str, "wpa3" ) == 0)
    {
        return NSAPI_SECURITY_WPA3;
    }
    else if (strcmp(auth_str, "wpa3_wpa2") == 0)
    {
        return NSAPI_SECURITY_WPA3_WPA2;
    }
    else
    {
        WIFI_ERROR(("Unsupported auth type: '%s'\r\n", auth_str));
        return NSAPI_SECURITY_UNKNOWN;
    }
}

const char* wifi_utils_authtype_to_str(nsapi_security_t sec)
{
    switch (sec)
    {
        case NSAPI_SECURITY_NONE:
            return "open";
        case NSAPI_SECURITY_WEP:
            return "wep";
        case NSAPI_SECURITY_WPA:
            return "wpa";
        case NSAPI_SECURITY_WPA2:
            return "wpa2";
        case NSAPI_SECURITY_WPA_WPA2:
            return "wpa_wpa2";
        case NSAPI_SECURITY_WPA3:
            return "wpa3";
        case NSAPI_SECURITY_UNKNOWN:
            return "Unknown";
        default:
            return "Unsupported";
    }
}

#ifdef __cplusplus
}
#endif
#endif