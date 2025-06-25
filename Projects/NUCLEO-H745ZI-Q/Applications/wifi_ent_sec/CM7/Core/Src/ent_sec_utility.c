/*
 * Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
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

/** @file
 *  Enterprise security utility code
 */

#include "command_console.h"
#include "ent_sec_utility.h"
#include "cy_wcm.h"


#include "cy_enterprise_security.h"
#include "cy_supplicant_core_constants.h"
#include "certificate.h"

#include "ent_tcp_server.h"
#include "ent_secure_tcp_client.h"
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

static int join_ent(int argc, char* argv[], tlv_buffer_t** data);
static int leave_ent(int argc, char* argv[], tlv_buffer_t** data);
static int start_echo_server(int argc, char* argv[], tlv_buffer_t** data);
static int stop_echo_server(int argc, char* argv[], tlv_buffer_t** data);
static int connect_to_server(int argc, char* argv[], tlv_buffer_t** data);
static int disconnect_server(int argc, char* argv[], tlv_buffer_t** data);

/******************************************************
*                      Macros
******************************************************/
#define WIFI_ENTERPRISE_SECURITY_COMMANDS \
        { (char*) "join_ent", join_ent, 0, NULL, NULL, (char*) "<ssid> <eap_tls> [username] [password] [eap] <wpa_aes|wpa_mixed|wpa2_aes|wpa2_mixed|wpa3_aes_ccmp|wpa3_aes_gcm|wpa3_192bit>", (char*) "Join an AP using an enterprise EAP method. DHCP assumed."}, \
        { (char*) "leave_ent", leave_ent, 0, NULL, NULL, (char*) "", (char*) "Leaves an enterprise AP and stops processing enterprise security events."}, \
        { (char*) "start_echo_server", start_echo_server, 0, NULL, NULL, (char*) "<tcp_port_number>", (char*) "Start TCP Echo server on the port number."}, \
        { (char*) "stop_echo_server", stop_echo_server, 0, NULL, NULL, (char*) "", (char*) "Stop TCP Echo server."}, \
        { (char*) "connect_to_server", connect_to_server, 0, NULL, NULL, (char*) "<IP Address> <tcp_port_number>", (char*) "Connect to TLS server on IP/Port."}, \
        { (char*) "disconnect_server", disconnect_server, 0, NULL, NULL, (char*) "", (char*) "Disconnect TLS connection."}, \


/******************************************************
*                      Structures
******************************************************/

/******************************************************
*                      Variables
******************************************************/

static const cy_command_console_cmd_t ent_sec_command_table[] =
{
    WIFI_ENTERPRISE_SECURITY_COMMANDS
    CMD_TABLE_END
};

static cy_enterprise_security_parameters_t ent_parameters;
static cy_enterprise_security_t handle;

/***************************************************************************************************
 * str_to_tunnel_authentication_type
 **************************************************************************************************/
static cy_enterprise_security_tunnel_t str_to_tunnel_authentication_type(char* arg)
{
    cy_enterprise_security_tunnel_t ret = CY_ENTERPRISE_SECURITY_TUNNEL_TYPE_NONE;
    if (strcmp(arg, "eap") == 0)
    {
        ret =  CY_ENTERPRISE_SECURITY_TUNNEL_TYPE_EAP;
    }
    else
    {
        ENT_SEC_INFO(("Unsupported Tunnel Authentication Type: '%s'\r\n", arg));
    }
    return (ret);
}


/*******************************************************************************
* Function Name: str_to_enterprise_security_type
*******************************************************************************
* Summary:
*  Convert command line string to Enterprise security type.
*
* Parameters:
* char* arg :
*
* Return:
*  cy_enterprise_security_eap_type_t ret
*
*******************************************************************************/

static cy_enterprise_security_eap_type_t str_to_enterprise_security_type(char* arg)
{
    cy_enterprise_security_eap_type_t ret = CY_ENTERPRISE_SECURITY_EAP_TYPE_NONE;

    if (strcmp(arg, "eap_tls") == 0)
    {
        ret = CY_ENTERPRISE_SECURITY_EAP_TYPE_TLS;
    }
    else if (strcmp(arg, "peap") == 0)
    {
        ret = CY_ENTERPRISE_SECURITY_EAP_TYPE_PEAP;
    }
    else if (strcmp(arg, "eap_ttls") == 0)
    {
        ret = CY_ENTERPRISE_SECURITY_EAP_TYPE_TTLS;
    }
    else if (strcmp(arg, "mschapv2") == 0)
    {
        ret = CY_ENTERPRISE_SECURITY_EAP_TYPE_MSCHAPV2;
    }
    else if (strcmp(arg, "cisco_leap") == 0)
    {
        ret = CY_ENTERPRISE_SECURITY_EAP_TYPE_LEAP;
    }
    else
    {
        ENT_SEC_INFO(("Bad EAP type: '%s'\r\n", arg));
        ret = CY_ENTERPRISE_SECURITY_EAP_TYPE_NONE;
    }
    return (ret);
}


/*******************************************************************************
* Function Name: str_to_enterprise_authtype
*******************************************************************************
* Summary:
*  Convert command line string to Enterprise auth type.
*
* Parameters:
* char* arg :
*
* Return:
*  cy_enterprise_security_auth_t ret
*
*******************************************************************************/
static cy_enterprise_security_auth_t str_to_enterprise_authtype(char* arg)
{
    cy_enterprise_security_auth_t ret = CY_ENTERPRISE_SECURITY_AUTH_TYPE_UNKNOWN;

    if (strcmp(arg, "wpa_aes") == 0)
    {
        ret = CY_ENTERPRISE_SECURITY_AUTH_TYPE_WPA_AES;
    }
    else if (strcmp(arg, "wpa_mixed") == 0)
    {
        ret = CY_ENTERPRISE_SECURITY_AUTH_TYPE_WPA_MIXED;
    }
    else if (strcmp(arg, "wpa2_aes") == 0)
    {
        ret = CY_ENTERPRISE_SECURITY_AUTH_TYPE_WPA2_AES;
    }
    else if (strcmp(arg, "wpa2_mixed") == 0)
    {
        ret = CY_ENTERPRISE_SECURITY_AUTH_TYPE_WPA2_MIXED;
    }
    else if (strcmp(arg, "wpa3_aes_gcm") == 0)
    {
        ret = CY_ENTERPRISE_SECURITY_AUTH_TYPE_WPA3_AES;
    }
    else if (strcmp(arg, "wpa3_192bit") == 0)
    {
        ret = CY_ENTERPRISE_SECURITY_AUTH_TYPE_WPA3_192BIT;
    }
    else if (strcmp(arg, "wpa3_aes_ccmp") == 0)
    {
        return CY_ENTERPRISE_SECURITY_AUTH_TYPE_WPA3_AES_CCMP;
    }
    else
    {
        ENT_SEC_INFO(("Bad auth type: '%s'\r\n", arg));
        ret = CY_ENTERPRISE_SECURITY_AUTH_TYPE_UNKNOWN;
    }
    return (ret);
}


/*******************************************************************************
* Function Name: print_ip4
*******************************************************************************
* Summary:
*  Print IP address in dotted format
*
* Parameters:
* uint32_t ip
*
* Return:
*
*******************************************************************************/
static void print_ip4(uint32_t ip)
{
    unsigned char bytes[4];

    bytes[0] = ip & 0xFF;
    bytes[1] = (ip >> 8) & 0xFF;
    bytes[2] = (ip >> 16) & 0xFF;
    bytes[3] = (ip >> 24) & 0xFF;
    ENT_SEC_INFO(("IPV4 addr = %d.%d.%d.%d\n", bytes[0], bytes[1], bytes[2], bytes[3]));
}


/*******************************************************************************
* Function Name: join_ent_internal
*******************************************************************************
* Summary:
*  Connect to enterprise security network.
*
* Parameters:
* cy_enterprise_security_parameters_t *ent_parameters
*
* Return:
*  int : ERR_CMD_OK on success, error codes otherwise
*
*******************************************************************************/
static int join_ent_internal(cy_enterprise_security_parameters_t* ent_parameters)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    cy_wcm_ip_address_t ip_addr;

    if (handle != NULL)
    {
        ENT_SEC_INFO(("Already connected to Wi-Fi network\n"));
        return ERR_CMD_OK;
    }

    result = cy_enterprise_security_create(&handle, ent_parameters);
    if (result != CY_RSLT_SUCCESS)
    {
        ENT_SEC_INFO(("Enterprise Security instance creation failed with error %u\n",
                      (unsigned int)result));
        return ERR_UNKNOWN;
    }

    result = cy_enterprise_security_join(handle);
    if (result != CY_RSLT_SUCCESS)
    {
        cy_enterprise_security_delete(&handle);
        ENT_SEC_INFO(("Enterprise Security Join failed with error %u\n", (unsigned int)result));
        return ERR_UNKNOWN;
    }

    cy_wcm_get_ip_addr(CY_WCM_INTERFACE_TYPE_STA, &ip_addr);
    ENT_SEC_INFO(("Enterprise Security Join successful\n"));
    print_ip4(ip_addr.ip.v4);

    return ERR_CMD_OK;
}


/*******************************************************************************
* Function Name: join_ent
*******************************************************************************
* Summary:
*  Parse the command line options and connect to enterprise network.
*
* Parameters:
* int argc
* char *argv[]
* tlv_buffer_t** data
*
* Return:
*  int : ERR_CMD_OK on success, error codes otherwise
*
*******************************************************************************/
static int join_ent(int argc, char* argv[], tlv_buffer_t** data)
{
    char* ssid;
    cy_enterprise_security_eap_type_t  eap_type;
    cy_enterprise_security_auth_t auth_type;
    char* username = NULL;
    char* password = NULL;

    if (argc < 4)
    {
        ENT_SEC_INFO(("Error Insufficient arguments\n"));
        return ERR_INSUFFICENT_ARGS;
    }

    ssid = argv[1];

    eap_type = str_to_enterprise_security_type(argv[2]);
    if (eap_type == CY_ENTERPRISE_SECURITY_EAP_TYPE_NONE)
    {
        ENT_SEC_INFO(("Unknown security type\n"));
        /* Fall-through: Allow invalid/unknown security type to be passed to the library. */
    }

    if ((eap_type == CY_ENTERPRISE_SECURITY_EAP_TYPE_PEAP) && (argc < 6))
    {
        return ERR_INSUFFICENT_ARGS;
    }

    if ((eap_type == CY_ENTERPRISE_SECURITY_EAP_TYPE_TTLS) && (argc < 7))
    {
        return ERR_INSUFFICENT_ARGS;
    }

    auth_type = str_to_enterprise_authtype(argv[argc - 1]);
    if (auth_type == CY_ENTERPRISE_SECURITY_AUTH_TYPE_UNKNOWN)
    {
        ENT_SEC_INFO(("Unknown auth type\n"));
        /* Fall-through: Allow invalid/unknown auth type to be passed to the library. */
    }

    ent_parameters.is_client_cert_required = 0;

    if ((eap_type == CY_ENTERPRISE_SECURITY_EAP_TYPE_PEAP) ||
        (eap_type == CY_ENTERPRISE_SECURITY_EAP_TYPE_TTLS) ||
        (eap_type == CY_ENTERPRISE_SECURITY_EAP_TYPE_TLS))
    {
        username = argv[3];

        /* Copy phase2 identity */
        strncpy(ent_parameters.phase2.inner_identity, username,
                CY_ENTERPRISE_SECURITY_MAX_IDENTITY_LENGTH);
        ent_parameters.phase2.inner_identity[CY_ENTERPRISE_SECURITY_MAX_IDENTITY_LENGTH - 1] = '\0';

        if (eap_type == CY_ENTERPRISE_SECURITY_EAP_TYPE_TLS)
        {
            ent_parameters.is_client_cert_required = 1;
        }
        else if (eap_type == CY_ENTERPRISE_SECURITY_EAP_TYPE_PEAP)
        {
            password = argv[4];
            ent_parameters.phase2.inner_eap_type = CY_ENTERPRISE_SECURITY_EAP_TYPE_MSCHAPV2;
            ent_parameters.phase2.tunnel_auth_type = CY_ENTERPRISE_SECURITY_TUNNEL_TYPE_MSCHAPV2;
            strncpy(ent_parameters.phase2.inner_password, password,
                    CY_ENTERPRISE_SECURITY_MAX_PASSWORD_LENGTH);
            ent_parameters.phase2.inner_password[CY_ENTERPRISE_SECURITY_MAX_PASSWORD_LENGTH -
                                                 1] = '\0';
        }
        else if (eap_type == CY_ENTERPRISE_SECURITY_EAP_TYPE_TTLS)
        {
            password = argv[4];
            strncpy(ent_parameters.phase2.inner_password, password,
                    CY_ENTERPRISE_SECURITY_MAX_PASSWORD_LENGTH);
            ent_parameters.phase2.inner_password[CY_ENTERPRISE_SECURITY_MAX_PASSWORD_LENGTH -
                                                 1] = '\0';

            ent_parameters.phase2.tunnel_auth_type = str_to_tunnel_authentication_type(argv[5]);
            if (ent_parameters.phase2.tunnel_auth_type == CY_ENTERPRISE_SECURITY_TUNNEL_TYPE_EAP)
            {
                ent_parameters.phase2.inner_eap_type = str_to_enterprise_security_type(argv[6]);
                if (ent_parameters.phase2.inner_eap_type == CY_ENTERPRISE_SECURITY_EAP_TYPE_NONE)
                {
                    ENT_SEC_INFO(("Unknown inner eap type\n"));
                    /* Fall-through: Allow invalid/unknown inner-eap type to be passed to the
                       library. */
                }
            }
            else
            {
                ENT_SEC_INFO(("Unsupported Tunnel Authentication Type\n"));
                /* Fall-through: Allow invalid/unknown tunnel auth type to be passed to the library.
                 */
            }

            if (argc > 7)
            {
                char* arg = NULL;
                if (ent_parameters.phase2.tunnel_auth_type ==
                    CY_ENTERPRISE_SECURITY_TUNNEL_TYPE_EAP)
                {
                    arg = argv[7];
                }
                if (strcmp(arg, "client-cert") == 0)
                {
                    ent_parameters.is_client_cert_required = 1;
                }
                else
                {
                    ent_parameters.is_client_cert_required = 0;
                }
            }
            else
            {
                ent_parameters.is_client_cert_required = 0;
            }
        }
    }

    if (ent_parameters.is_client_cert_required == 1)
    {
        if (CY_ENTERPRISE_SECURITY_AUTH_TYPE_WPA3_192BIT == auth_type)
        {
            ent_parameters.client_cert = (char*)WPA3_192BIT_USER_CERTIFICATE_STRING;
            ent_parameters.client_key = (char*)WPA3_192BIT_USER_PRIVATE_KEY_STRING;
        }
        else
        {
            ent_parameters.client_cert = (char*)WIFI_USER_CERTIFICATE_STRING;
            ent_parameters.client_key = (char*)WIFI_USER_PRIVATE_KEY_STRING;
        }
    }
    else
    {
        ent_parameters.client_cert = NULL;
        ent_parameters.client_key = NULL;
    }

    if (CY_ENTERPRISE_SECURITY_AUTH_TYPE_WPA3_192BIT == auth_type)
    {
        ent_parameters.ca_cert = (char*)WPA3_192BIT_ROOT_CERTIFICATE_STRING;
    }
    else
    {
        ent_parameters.ca_cert = (char*)WIFI_ROOT_CERTIFICATE_STRING;
    }

    strncpy(ent_parameters.ssid, ssid, SSID_NAME_SIZE);
    ent_parameters.ssid[SSID_NAME_SIZE - 1] = '\0';

    strncpy(ent_parameters.outer_eap_identity, (char*)username,
            CY_ENTERPRISE_SECURITY_MAX_IDENTITY_LENGTH);
    ent_parameters.outer_eap_identity[CY_ENTERPRISE_SECURITY_MAX_IDENTITY_LENGTH - 1] = '\0';

    ent_parameters.eap_type  = eap_type;
    ent_parameters.auth_type = auth_type;

    return join_ent_internal(&ent_parameters);
}


/*******************************************************************************
* Function Name: leave_ent
*******************************************************************************
* Summary:
*  Leave from enterprise network.
*
* Parameters:
* int argc
* char *argv[]
* tlv_buffer_t** data
*
* Return:
*  int : ERR_CMD_OK on success, error codes otherwise
*
*******************************************************************************/
static int leave_ent(int argc, char* argv[], tlv_buffer_t** data)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    result = cy_enterprise_security_leave(handle);
    if (result != CY_RSLT_SUCCESS)
    {
        ENT_SEC_INFO(("Enterprise Security Leave failed with error %u\n", (unsigned int)result));
        return ERR_UNKNOWN;
    }

    result = cy_enterprise_security_delete(&handle);
    if (result != CY_RSLT_SUCCESS)
    {
        ENT_SEC_INFO(("Enterprise Security instance deletion failed with error %u\n",
                      (unsigned int)result));
        return ERR_UNKNOWN;
    }

    ENT_SEC_INFO(("Enterprise Security Leave successful\n"));
    return ERR_CMD_OK;
}


/*******************************************************************************
* Function Name: start_echo_server
*******************************************************************************
* Summary:
*  Starts TCP Echo server on given tcp port
*
* Parameters:
* int argc
* char *argv[]
* tlv_buffer_t** data
*
* Return:
*  int : ERR_CMD_OK on success, error codes otherwise
*
*******************************************************************************/
static int start_echo_server(int argc, char* argv[], tlv_buffer_t** data)
{
    uint16_t port = 0;

    if (argc != 2)
    {
        ENT_SEC_INFO(("Error Insufficient arguments\n"));
        return ERR_INSUFFICENT_ARGS;
    }

    port = atoi(argv[1]);
    if (port == 0)
    {
        return ERR_BAD_ARG;
    }
    if (ent_tcp_server_start(port) != CY_RSLT_SUCCESS)
    {
        return ERR_UNKNOWN;
    }
    return ERR_CMD_OK;
}


/*******************************************************************************
* Function Name: stop_echo_server
*******************************************************************************
* Summary:
*  Stops TCP Echo server.
*
* Parameters:
* int argc
* char *argv[]
* tlv_buffer_t** data
*
* Return:
*  int : ERR_CMD_OK on success, error codes otherwise
*
*******************************************************************************/
static int stop_echo_server(int argc, char* argv[], tlv_buffer_t** data)
{
    ent_tcp_server_stop();
    return ERR_CMD_OK;
}


/*******************************************************************************
* Function Name: connect_to_server
*******************************************************************************
* Summary:
*  Connect to a remote TLS server.
*
* Parameters:
* int argc
* char *argv[]
* tlv_buffer_t** data
*
* Return:
*  int : ERR_CMD_OK on success, error codes otherwise
*
*******************************************************************************/
static int connect_to_server(int argc, char* argv[], tlv_buffer_t** data)
{
    char* ip = NULL;
    uint16_t port = 0;

    if (argc != 3)
    {
        ENT_SEC_INFO(("Error Insufficient arguments\n"));
        return ERR_INSUFFICENT_ARGS;
    }

    ip = argv[1];

    port = atoi(argv[2]);
    if (port == 0)
    {
        return ERR_BAD_ARG;
    }
    if (ent_secure_tcp_client_connect(ip, port) != CY_RSLT_SUCCESS)
    {
        return ERR_UNKNOWN;
    }
    return ERR_CMD_OK;
}


/*******************************************************************************
* Function Name: disconnect_server
*******************************************************************************
* Summary:
*  Disconnects from TLS server.
*
* Parameters:
* int argc
* char *argv[]
* tlv_buffer_t** data
*
* Return:
*  int : ERR_CMD_OK on success, error codes otherwise
*
*******************************************************************************/
static int disconnect_server(int argc, char* argv[], tlv_buffer_t** data)
{
    ent_secure_tcp_client_disconnect();
    return ERR_CMD_OK;
}


/*******************************************************************************
* Function Name: ent_utility_init
*******************************************************************************
* Summary:
*  Initialize enterprise utility commands
*
* Parameters:
*
* Return:
*
*******************************************************************************/
void ent_utility_init(void)
{
    cy_command_console_add_table(ent_sec_command_table);
}


#ifdef __cplusplus
}
#endif
