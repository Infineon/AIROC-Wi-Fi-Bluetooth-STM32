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

/** @file wifi_utility.c
 *  @brief This file contains the definition of Wi-Fi commands and implementation of the
 *  command handlers on Amazon FreeRTOS.
 */

/* FreeRTOS includes. */
#ifndef DISABLE_COMMAND_CONSOLE_WIFI
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "command_console.h"
#include "wifi_utility.h"
#include "iot_system_init.h"
#include "iot_logging_task.h"
#include "iot_wifi.h"
#include "iot_wifi_common.h"
#include "lwip/tcpip.h"
#include <lwip/sockets.h>
#include <lwip/igmp.h>
#include <lwip/icmp.h>
#include <lwip/prot/ip4.h>
#include <lwip/prot/ip.h>
#include <lwip/inet_chksum.h>
#include "stdio.h"
#include "stdlib.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *               Function Declarations
 ******************************************************/
int handle_join                              ( int argc, char *argv[], tlv_buffer_t** data );
int handle_leave                             ( int argc, char *argv[], tlv_buffer_t** data );
int handle_scan                              ( int argc, char *argv[], tlv_buffer_t** data );
int handle_ping                              ( int argc, char *argv[], tlv_buffer_t** data );
int handle_get_rssi                          ( int argc, char *argv[], tlv_buffer_t** data );

/******************************************************
 *                     Macros
 ******************************************************/

/* The task delay for allowing the lower priority logging task to print out Wi-Fi
 * failure status before blocking indefinitely. */
#define WIFI_UTIL_WIFI_STATUS_DELAY        pdMS_TO_TICKS( 5000 )
#define WIFI_UTIL_IPV4_ADDR_SIZE           4
#define WIFI_UTIL_PING_ID                  (0xAFAF)  /** ICMP Identifier for PING */
#define WIFI_UTIL_PING_DATA_SIZE           (64)      /** ping additional data size to include in the packet */
#define WIFI_UTIL_PING_TIMEOUT             (10000)
#define WIFI_UTIL_PING_IF_NAME_LEN         (3)
#define WIFI_UTIL_PING_RESPONSE_LEN        (64)

#ifndef CMD_CONSOLE_WIFI_NUM_SCAN_RESULT
#define CMD_CONSOLE_WIFI_NUM_SCAN_RESULT   (10)
#endif

#ifndef CMD_CONSOLE_MAX_WIFI_RETRY_COUNT
#define CMD_CONSOLE_MAX_WIFI_RETRY_COUNT    (5)
#endif

#define WIFI_COMMANDS \
    { "join",            handle_join,        2, NULL, NULL, (char*) "<ssid> <open|wpa|wpa2|wpa3> [password] [channel] "ESCAPE_SPACE_PROMPT, (char*) "Join an AP.(This command is deprecated and it will be removed in the future. Please use wifi_join command)"}, \
    { "leave",           handle_leave,       0, NULL, NULL, (char*) "", (char*) "Leave the connected AP.(This command is deprecated and it will be removed in the future. Please use wifi_leave command)"}, \
    { "scan",            handle_scan,        0, NULL, NULL, (char*) "", (char*) "Scan all the Wi-FI AP in the vicinity.(This command is deprecated and it will be removed in the future. Please use wifi_scan command)"}, \
    { "ping",            handle_ping,        1, NULL, NULL, (char*) "<IP address>", (char*) "Ping to an IP address.(This command is deprecated and it will be removed in the future. Please use wifi_ping command)"}, \
    { "get_rssi",        handle_get_rssi,    0, NULL, NULL, (char*) "", (char*) "Get the received signal strength of the AP (client mode only).(This command is deprecated and it will be removed in the future. Please use wifi_get_rssi command)"}, \
    { "wifi_join",       handle_join,        2, NULL, NULL, (char*) "<ssid> <open|wpa|wpa2|wpa3> [password] [channel] "ESCAPE_SPACE_PROMPT, (char*) "Join an AP"}, \
    { "wifi_leave",      handle_leave,       0, NULL, NULL, (char*) "", (char*) "Leave the connected AP."}, \
    { "wifi_scan",       handle_scan,        0, NULL, NULL, (char*) "", (char*) "Scan all the Wi-FI AP in the vicinity."}, \
    { "wifi_ping",       handle_ping,        1, NULL, NULL, (char*) "<IP address>", (char*) "Ping to an IP address"}, \
    { "wifi_get_rssi",   handle_get_rssi,    0, NULL, NULL, (char*) "", (char*) "Get the received signal strength of the AP (client mode only)."}, \

static const cy_command_console_cmd_t wifi_command_table[] =
{
    WIFI_COMMANDS
    CMD_TABLE_END
};

#define MAKE_IPV4_ADDRESS1(a, b, c, d) ((((uint32_t) d) << 24) | (((uint32_t) c) << 16) | (((uint32_t) b) << 8) |((uint32_t) a))

#if defined(__ICCARM__)
#define WIFI_WEAK_FUNC            __WEAK
#elif defined(__GNUC__) || defined(__clang__) || defined(__CC_ARM)
#define WIFI_WEAK_FUNC            __attribute__((weak))
#else
#define WIFI_WEAK_FUNC           __attribute__((weak))
#endif

/******************************************************
 *               Variable Definitions
 ******************************************************/

/**
 * IP Version
 */
typedef enum
{
    WIFI_UTIL_IP_VER_V4 = 4,      /**< Denotes IPv4 version. */
    WIFI_UTIL_IP_VER_V6 = 6       /**< Denotes IPv6 version. */
} wifi_util_ip_version_t;


/**
 * Structure used to receive the IP address information from \ref cy_wcm_connect_ap.
 */
typedef struct
{
    wifi_util_ip_version_t version;  /**< IP version. */
    union
    {
        uint32_t v4;     /**< IPv4 address in network byte order. */
        uint32_t v6[4];  /**< IPv6 address in network byte order. */
    } ip;                /**< IP address bytes. */
} wifi_util_ip_address_t;

struct icmp_packet
{
    struct icmp_echo_hdr hdr;
    uint8_t data[WIFI_UTIL_PING_DATA_SIZE];
};

/******************************************************
 *               Static Function Declarations
 ******************************************************/
static void ping_prepare_echo(struct icmp_packet *iecho, uint16_t len, uint16_t *ping_seq_num);
static err_t ping_send(int socket_hnd, const wifi_util_ip_address_t* address, struct icmp_packet *iecho, uint16_t *sequence_number);
static err_t ping_recv(int socket_hnd, wifi_util_ip_address_t* address, uint16_t *ping_seq_num);
static int wifi_utils_str_to_ip(char* ip_str, wifi_util_ip_address_t* ip_addr);
static err_t wifi_util_ping(wifi_util_ip_address_t* address);
static WIFISecurity_t wifi_utils_str_to_authtype(char* arg);
static char *wifi_utils_authtype_to_str(WIFISecurity_t security);

int handle_join(int argc, char *argv[], tlv_buffer_t** data)
{
    int                 result = 0;
    char*               ssid = argv[1];
    WIFISecurity_t      auth_type = wifi_utils_str_to_authtype(argv[2]);
    uint8_t*            password;
    uint8_t             key_length;
    WIFINetworkParams_t xNetworkParams;
    WIFIReturnCode_t    xWifiStatus;
    uint8_t             retry_count = 0;
    uint8_t             ucTempIp[4] = { 0 };

    if(WIFI_IsConnected())
    {
        WIFI_INFO(("Already connected. Leave and join again.\n"));
        return 0;
    }

    if( ((strcmp(argv[2], "open") == 0) && (argc > 3)) ||
        ((strcmp(argv[2], "open") != 0) && (argc < 4)) || (argc > 5))
    {
        WIFI_ERROR(("Invalid arguments\n"));
        return -1;
    }

    if(strcmp(argv[2], "open") == 0)
    {
        xNetworkParams.pcSSID = ssid;
        xNetworkParams.ucSSIDLength = strlen(ssid);
        xNetworkParams.pcPassword = NULL;
        xNetworkParams.ucPasswordLength = 0;
        xNetworkParams.xSecurity = auth_type;
        xNetworkParams.cChannel = 0;
    }
    else
    {
        if(auth_type == eWiFiSecurityNotSupported)
        {
            WIFI_ERROR(("Cannot connect due to unsupported auth type.\n"));
            return -1;
        }
        password = (uint8_t*)argv[3];
        /* Setup parameters. */
        xNetworkParams.pcSSID = ssid;
        xNetworkParams.ucSSIDLength = strlen(ssid);
        xNetworkParams.pcPassword = password;
        xNetworkParams.ucPasswordLength = strlen(password);
        xNetworkParams.xSecurity = auth_type;
        if(argc == 4)
        {
            xNetworkParams.cChannel = 0;
        }
        else
        {
            xNetworkParams.cChannel = atoi(argv[4]);
        }
    }

    WIFI_INFO(("Connecting to AP please wait...\n"));
    while(true)
    {
        xWifiStatus = WIFI_ConnectAP(&(xNetworkParams));
        if(xWifiStatus == eWiFiSuccess)
        {
            WIFI_INFO(("Successfully connected to %s\n", ssid));
            xWifiStatus = WIFI_GetIP(ucTempIp);
            if (eWiFiSuccess == xWifiStatus)
            {
                WIFI_INFO(("IP Address: %d.%d.%d.%d\n",
                                ucTempIp[ 0 ], ucTempIp[ 1 ], ucTempIp[ 2 ], ucTempIp[ 3 ]));
                break;
            }
        }
        else
        {
            retry_count++;
            if (retry_count >= CMD_CONSOLE_MAX_WIFI_RETRY_COUNT)
            {
                WIFI_INFO(("Exceeded maximum Wi-Fi connection attempts.\n"));
                result = -1;
                break;
            }
            WIFI_INFO(("Failed to join AP [%d]. Retrying...\n", xWifiStatus));
            continue;
        }
    }
    return result;
}

int handle_leave(int argc, char *argv[], tlv_buffer_t** data)
{
    WIFIReturnCode_t res;

    if(!WIFI_IsConnected())
    {
        WIFI_INFO(("Failed to leave. Not connected to any AP.\n"));
        return 0;
    }

    if((res = WIFI_Disconnect()) != eWiFiSuccess)
    {
        WIFI_ERROR(("Failed to disconnect from AP. Res:%d\n", res));
        return -1;
    }

    WIFI_INFO(("Disconnected from AP.\n"));
    return 0;
}

int handle_scan(int argc, char *argv[], tlv_buffer_t** data)
{
    WIFIReturnCode_t result;
    WIFIScanResult_t *scan_results;

    scan_results = (WIFIScanResult_t*) malloc(sizeof(WIFIScanResult_t) * CMD_CONSOLE_WIFI_NUM_SCAN_RESULT);
    if(scan_results == NULL)
    {
        WIFI_ERROR(("Heap space error while allocating scan result ptr \n"));
        return -1;
    }

    if((result = WIFI_Scan(scan_results, CMD_CONSOLE_WIFI_NUM_SCAN_RESULT)) != eWiFiSuccess)
    {
        WIFI_INFO(("Error while scanning. Res: %d \n", result));
        return -1;
    }

    WIFI_INFO(("#### Scan Results START ####\n\n"));
    WIFI_INFO(("SSID                 Security Type  RSSI(dBm)  Channel BSSID\n"));
    for(int i = 0; i < CMD_CONSOLE_WIFI_NUM_SCAN_RESULT ; i++)
    {
        if(strlen(scan_results[i].cSSID) != 0)
        {
            WIFI_INFO(("%-20s %-14s %-10d %-7d %02X:%02X:%02X:%02X:%02X:%02X\n",
                scan_results[i].cSSID,
                wifi_utils_authtype_to_str(scan_results[i].xSecurity),
                scan_results[i].cRSSI,
                scan_results[i].cChannel,
                scan_results[i].ucBSSID[0],
                scan_results[i].ucBSSID[1],
                scan_results[i].ucBSSID[2],
                scan_results[i].ucBSSID[3],
                scan_results[i].ucBSSID[4],
                scan_results[i].ucBSSID[5]));
        }
    }
    WIFI_INFO(("#### Scan Results END ####\n\n"));
    free(scan_results);

    return result;
}

int handle_ping(int argc, char *argv[], tlv_buffer_t** data)
{
    err_t res;
    wifi_util_ip_address_t ip_addr;

    if(!WIFI_IsConnected())
    {
        WIFI_INFO(("Not connected to AP.\n"));
        return 0;
    }

    if(wifi_utils_str_to_ip(argv[1], &ip_addr) == -1)
    {
        return -1;
    }

    if((res = wifi_util_ping(&ip_addr)) != ERR_OK)
    {
        WIFI_ERROR(("Ping failed. Error: %d\n", res));
        return -1;
    }
    return 0;
}


int handle_get_rssi(int argc, char *argv[], tlv_buffer_t** data)
{
    WIFI_INFO(("Not supported\n"));
    return 0;
}

WIFI_WEAK_FUNC cy_rslt_t wifi_utility_init()
{
    WIFIReturnCode_t xWifiStatus;
    cy_rslt_t res;

    cy_command_console_add_table(wifi_command_table);
    // Turn on Wi-Fi
    xWifiStatus = WIFI_On();
    if( xWifiStatus == eWiFiSuccess )
    {
        WIFI_INFO( ( "Wi-Fi module initialized...\n" ) );
        res = CY_RSLT_SUCCESS;
    }
    else
    {
        WIFI_ERROR( ( "Failed to initialize Wi-Fi module. Res:%lu\n", xWifiStatus) );
        /* Delay to allow the lower priority logging task to print the above status.
            * The while loop below will block the above printing. */
        vTaskDelay( WIFI_UTIL_WIFI_STATUS_DELAY );

        while( 1 )
        {
        }
    }
    return res;
}

WIFISecurity_t wifi_utils_str_to_authtype( char* arg )
{
    if ( strcmp( arg, "open" ) == 0 )
    {
        return eWiFiSecurityOpen;
    }
    else if ( strcmp( arg, "wpa" ) == 0 )
    {
        return eWiFiSecurityWPA;
    }
    else if ( strcmp( arg, "wpa2" ) == 0 )
    {
        return eWiFiSecurityWPA2;
    }
    else if ( strcmp( arg, "wpa3" ) == 0 )
    {
        return eWiFiSecurityWPA3;
    }
    else
    {
        WIFI_ERROR( ("Unsupported auth type: '%s'\r\n", arg) );
        return eWiFiSecurityNotSupported;
    }
}

char *wifi_utils_authtype_to_str(WIFISecurity_t security)
{
    switch(security)
    {
        case eWiFiSecurityOpen :
            return "open";

        case eWiFiSecurityWEP :
            return "wep";

        case eWiFiSecurityWPA :
            return "wpa";

        case eWiFiSecurityWPA2 :
            return "wpa2";

        case eWiFiSecurityWPA3 :
            return "wpa3";

        default:
            return "unknown";
    }
}

static int wifi_utils_str_to_ip(char* ip_str, wifi_util_ip_address_t* ip_addr)
{
    int bytes[WIFI_UTIL_IPV4_ADDR_SIZE];
    char* p;
    int i = 0;
    char* delimiter = ".";

    p = strtok (ip_str, delimiter);
    bytes[i] = atoi(p);
    i++;
    while(p != NULL && i < WIFI_UTIL_IPV4_ADDR_SIZE)
    {
        p = strtok (NULL, delimiter);
        bytes[i] = atoi(p);
        i++;
    }

    WIFI_DEBUG(("Parsed IP addr = %d.%d.%d.%d\n", bytes[0], bytes[1], bytes[2], bytes[3]));
    if(i != WIFI_UTIL_IPV4_ADDR_SIZE)
    {
        WIFI_ERROR(("Invalid IP Addr.\n"));
        return -1;
    }

    ip_addr->version = WIFI_UTIL_IP_VER_V4;
    ip_addr->ip.v4 = (uint32_t)MAKE_IPV4_ADDRESS1(bytes[0], bytes[1], bytes[2], bytes[3]);

    return 0;
}

static void ping_prepare_echo(struct icmp_packet *iecho, uint16_t len, uint16_t *ping_seq_num)
{
    int i;
    ICMPH_TYPE_SET(&iecho->hdr, ICMP_ECHO);
    ICMPH_CODE_SET(&iecho->hdr, 0);
    iecho->hdr.chksum = 0;
    iecho->hdr.id = WIFI_UTIL_PING_ID;
    iecho->hdr.seqno = htons(++(*ping_seq_num));

    /* fill the additional data buffer with some data */
    for ( i = 0; i < (int)sizeof(iecho->data); i++ )
    {
        iecho->data[i] = (uint8_t)i;
    }

    iecho->hdr.chksum = inet_chksum(iecho, len);
}

static err_t ping_send(int socket_hnd, const wifi_util_ip_address_t* address, struct icmp_packet *iecho, uint16_t *sequence_number)
{
    int                err;
    struct sockaddr_in to;

    /* Construct ping request */
    ping_prepare_echo(iecho, sizeof(struct icmp_packet), sequence_number);

    /* Send the ping request */
    to.sin_len         = sizeof( to );
    to.sin_family      = AF_INET;
    to.sin_addr.s_addr = address->ip.v4;

    err = lwip_sendto(socket_hnd, iecho, sizeof(struct icmp_packet), 0, (struct sockaddr*) &to, sizeof(to));

    return (err ? ERR_OK : ERR_VAL);
}

static err_t ping_recv(int socket_hnd, wifi_util_ip_address_t* address, uint16_t *ping_seq_num)
{
    char                  buf[WIFI_UTIL_PING_RESPONSE_LEN];
    int                   fromlen;
    int                   len;
    struct sockaddr_in    from;
    struct ip_hdr*        iphdr;
    struct icmp_echo_hdr* iecho;
    do
    {
        len = lwip_recvfrom(socket_hnd, buf, sizeof(buf), 0, (struct sockaddr*) &from, (socklen_t*) &fromlen);
        if (len >= (int) (sizeof(struct ip_hdr) + sizeof(struct icmp_echo_hdr)))
        {
            iphdr = (struct ip_hdr *) buf;
            iecho = (struct icmp_echo_hdr *) (buf + (IPH_HL(iphdr) * 4));

            if ((iecho->id == WIFI_UTIL_PING_ID) &&
                 (iecho->seqno == htons(*ping_seq_num)) &&
                 (ICMPH_TYPE(iecho) == ICMP_ER))
            {
                return ERR_OK; /* Echo reply received - return success */
            }
        }
    } while (len > 0);

    return ERR_TIMEOUT; /* No valid echo reply received before timeout */
}

static err_t wifi_util_ping(wifi_util_ip_address_t* address)
{
    cy_time_t send_time;
    cy_time_t recvd_time;
    err_t err;
    struct timeval timeout_val;
    struct icmp_packet ping_packet;
    uint16_t ping_seq_num = 0;
    int socket_for_ping = -1;
    struct netif *net_interface;
    char if_name[WIFI_UTIL_PING_IF_NAME_LEN];
    struct ifreq iface;
    uint32_t elapsed_time_ms;

    /* Open a local socket for pinging */
    socket_for_ping = lwip_socket(AF_INET, SOCK_RAW, IP_PROTO_ICMP);
    if (socket_for_ping < 0)
    {
        WIFI_ERROR(("Unable to create a socket for Ping \n"));
        goto exit;
    }

    /* convert the timeout into struct timeval */
    timeout_val.tv_sec  = (long)(WIFI_UTIL_PING_TIMEOUT / 1000);
    timeout_val.tv_usec = (long)((WIFI_UTIL_PING_TIMEOUT % 1000) * 1000);
    /* Set the receive timeout on local socket so ping will time out. */
    if((err = lwip_setsockopt(socket_for_ping, SOL_SOCKET, SO_RCVTIMEO, &timeout_val, sizeof(struct timeval))) != ERR_OK)
    {
        WIFI_ERROR(("Error while setting socket for Ping \n"));
        goto exit;
    }

    /* Bind interface to device. */
    net_interface = cy_lwip_get_interface();
    memset(&iface, 0, sizeof(iface));
    memcpy(if_name, net_interface->name, sizeof(net_interface->name));
    sprintf(&if_name[2], "%d", net_interface->num);
    memcpy(iface.ifr_name, if_name, WIFI_UTIL_PING_IF_NAME_LEN);
    if((err = lwip_setsockopt(socket_for_ping, SOL_SOCKET, SO_BINDTODEVICE, &iface, sizeof(iface))) != ERR_OK)
    {
        WIFI_ERROR(("Error while binding socket to interface \n"));
        goto exit;
    }

    /* Send a ping */
    err = ping_send(socket_for_ping, address, &ping_packet, &ping_seq_num);
    if (err != ERR_OK)
    {
        WIFI_ERROR(("Error while sending Ping request \n"));
        goto exit;
    }
    /* Record time ping was sent */
    cy_rtos_get_time(&send_time);

    /* Wait for ping reply */
    err = ping_recv(socket_for_ping, address, &ping_seq_num);
    if (err != ERR_OK)
    {
        WIFI_ERROR(("Ping response timed out \n"));
        goto exit;
    }

    /* compute the elapsed time since a ping request was initiated */
    cy_rtos_get_time(&recvd_time);
    elapsed_time_ms = (uint32_t)(recvd_time - send_time);
    WIFI_INFO(("Ping successful. Elapsed time: %d (ms) \n", elapsed_time_ms));

exit :
    /* close the socket */
    if(socket_for_ping >= 0)
    {
        lwip_close(socket_for_ping);
    }

    return err;
}

#ifdef __cplusplus
}
#endif
#endif