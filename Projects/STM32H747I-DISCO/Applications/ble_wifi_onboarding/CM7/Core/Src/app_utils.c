/***************************************************************************************************
 * File Name:   app_utils.c
 *
 * Description: This file consists of the utility functions that will help debugging and developing
 *  the
 *              applications easier with much more meaningful information.
 *
 * Related Document: See Readme.md
 *
 ***************************************************************************************************
 * (c) 2021, Cypress Semiconductor Corporation. All rights reserved.
 ***************************************************************************************************
 * This software, including source code, documentation and related materials
 * ("Software"), is owned by Cypress Semiconductor Corporation or one of its
 * subsidiaries ("Cypress") and is protected by and subject to worldwide patent
 * protection (United States and foreign), United States copyright laws and
 * international treaty provisions. Therefore, you may use this Software only
 * as provided in the license agreement accompanying the software package from
 * which you obtained this Software ("EULA").
 *
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software source
 * code solely for use in connection with Cypress's integrated circuit products.
 * Any reproduction, modification, translation, compilation, or representation
 * of this Software except as specified above is prohibited without the express
 * written permission of Cypress.
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
 * including Cypress's product in a High Risk Product, the manufacturer of such
 * system or application assumes all risk of such use and in doing so agrees to
 * indemnify Cypress against all liability.
 **************************************************************************************************/

/***************************************************************************************************
 *                                INCLUDES
 **************************************************************************************************/
#include "stdio.h"
#include "app_utils.h"

/***************************************************************************************************
 *                              FUNCTION DEFINITIONS
 **************************************************************************************************/

/***************************************************************************************************
 * Function Name: print_bd_address()
 ***************************************************************************************************
 * Summary:
 *   This is the utility function that prints the address of the Bluetooth device
 *
 * Parameters:
 *   wiced_bt_device_address_t bdadr                : Bluetooth address
 *
 * Return:
 *  void
 *
 **************************************************************************************************/
void print_bd_address(wiced_bt_device_address_t bdadr)
{
    printf("%02X:%02X:%02X:%02X:%02X:%02X",
           bdadr[0], bdadr[1], bdadr[2], bdadr[3], bdadr[4], bdadr[5]);
}


/***************************************************************************************************
 * Function Name: get_bt_event_name
 ***************************************************************************************************
 * Summary:
 * The function converts the wiced_bt_management_evt_t enum value to its
 * corresponding string literal. This will help the programmer to debug easily
 * with log traces without navigating through the source code.
 *
 * Parameters:
 *  wiced_bt_management_evt_t event: Bluetooth management event type
 *
 * Return:
 *  wiced_bt_management_evt_t
 *
 **************************************************************************************************/
const char* get_bt_event_name(wiced_bt_management_evt_t event)
{
    switch ((int)event)
    {
        CASE_RETURN_STR(BTM_ENABLED_EVT)
        CASE_RETURN_STR(BTM_DISABLED_EVT)
        CASE_RETURN_STR(BTM_POWER_MANAGEMENT_STATUS_EVT)
        CASE_RETURN_STR(BTM_PIN_REQUEST_EVT)
        CASE_RETURN_STR(BTM_USER_CONFIRMATION_REQUEST_EVT)
        CASE_RETURN_STR(BTM_PASSKEY_NOTIFICATION_EVT)
        CASE_RETURN_STR(BTM_PASSKEY_REQUEST_EVT)
        CASE_RETURN_STR(BTM_KEYPRESS_NOTIFICATION_EVT)
        CASE_RETURN_STR(BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT)
        CASE_RETURN_STR(BTM_PAIRING_IO_CAPABILITIES_BR_EDR_RESPONSE_EVT)
        CASE_RETURN_STR(BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT)
        CASE_RETURN_STR(BTM_PAIRING_COMPLETE_EVT)
        CASE_RETURN_STR(BTM_ENCRYPTION_STATUS_EVT)
        CASE_RETURN_STR(BTM_SECURITY_REQUEST_EVT)
        CASE_RETURN_STR(BTM_SECURITY_FAILED_EVT)
        CASE_RETURN_STR(BTM_SECURITY_ABORTED_EVT)
        CASE_RETURN_STR(BTM_READ_LOCAL_OOB_DATA_COMPLETE_EVT)
        CASE_RETURN_STR(BTM_REMOTE_OOB_DATA_REQUEST_EVT)
        CASE_RETURN_STR(BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT)
        CASE_RETURN_STR(BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT)
        CASE_RETURN_STR(BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT)
        CASE_RETURN_STR(BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT)
        CASE_RETURN_STR(BTM_BLE_SCAN_STATE_CHANGED_EVT)
        CASE_RETURN_STR(BTM_BLE_ADVERT_STATE_CHANGED_EVT)
        CASE_RETURN_STR(BTM_SMP_REMOTE_OOB_DATA_REQUEST_EVT)
        CASE_RETURN_STR(BTM_SMP_SC_REMOTE_OOB_DATA_REQUEST_EVT)
        CASE_RETURN_STR(BTM_SMP_SC_LOCAL_OOB_DATA_NOTIFICATION_EVT)
        CASE_RETURN_STR(BTM_SCO_CONNECTED_EVT)
        CASE_RETURN_STR(BTM_SCO_DISCONNECTED_EVT)
        CASE_RETURN_STR(BTM_SCO_CONNECTION_REQUEST_EVT)
        CASE_RETURN_STR(BTM_SCO_CONNECTION_CHANGE_EVT)
        CASE_RETURN_STR(BTM_BLE_CONNECTION_PARAM_UPDATE)
        CASE_RETURN_STR(BTM_BLE_PHY_UPDATE_EVT)
    }

    return "UNKNOWN_EVENT";
}


/***************************************************************************************************
 * Function Name: get_bt_advert_mode_name
 ***************************************************************************************************
 * Summary:
 * The function converts the wiced_bt_ble_advert_mode_t enum value to its corresponding
 * string literal. This will help the programmer to debug easily with log traces
 * without navigating through the source code.
 *
 * Parameters:
 *  wiced_bt_ble_advert_mode_t mode: Bluetooth advertisement mode type
 *
 * Return:
 *  wiced_bt_ble_advert_mode_t
 *
 **************************************************************************************************/
const char* get_bt_advert_mode_name(wiced_bt_ble_advert_mode_t mode)
{
    switch ((int)mode)
    {
        CASE_RETURN_STR(BTM_BLE_ADVERT_OFF)
        CASE_RETURN_STR(BTM_BLE_ADVERT_DIRECTED_HIGH)
        CASE_RETURN_STR(BTM_BLE_ADVERT_DIRECTED_LOW)
        CASE_RETURN_STR(BTM_BLE_ADVERT_UNDIRECTED_HIGH)
        CASE_RETURN_STR(BTM_BLE_ADVERT_UNDIRECTED_LOW)
        CASE_RETURN_STR(BTM_BLE_ADVERT_NONCONN_HIGH)
        CASE_RETURN_STR(BTM_BLE_ADVERT_NONCONN_LOW)
        CASE_RETURN_STR(BTM_BLE_ADVERT_DISCOVERABLE_HIGH)
        CASE_RETURN_STR(BTM_BLE_ADVERT_DISCOVERABLE_LOW)
    }

    return "UNKNOWN_MODE";
}


/***************************************************************************************************
 * Function Name: get_bt_gatt_disconn_reason_name
 ***************************************************************************************************
 * Summary:
 * The function converts the wiced_bt_gatt_disconn_reason_t enum value to its corresponding
 * string literal. This will help the programmer to debug easily with log traces
 * without navigating through the source code.
 *
 * Parameters:
 *  wiced_bt_gatt_disconn_reason_t reason: GATT Disconnection reason
 *
 * Return:
 *  wiced_bt_gatt_disconn_reason_t
 *
 **************************************************************************************************/
const char* get_bt_gatt_disconn_reason_name(wiced_bt_gatt_disconn_reason_t reason)
{
    switch ((int)reason)
    {
        CASE_RETURN_STR(GATT_CONN_UNKNOWN)
        CASE_RETURN_STR(GATT_CONN_L2C_FAILURE)
        CASE_RETURN_STR(GATT_CONN_TIMEOUT)
        CASE_RETURN_STR(GATT_CONN_TERMINATE_PEER_USER)
        CASE_RETURN_STR(GATT_CONN_TERMINATE_LOCAL_HOST)
        CASE_RETURN_STR(GATT_CONN_FAIL_ESTABLISH)
        CASE_RETURN_STR(GATT_CONN_LMP_TIMEOUT)
        CASE_RETURN_STR(GATT_CONN_CANCEL)
    }

    return "UNKNOWN_REASON";
}


/***************************************************************************************************
 * Function Name: get_bt_gatt_status_name
 ***************************************************************************************************
 * Summary:
 * The function converts the wiced_bt_gatt_status_t enum value to its corresponding
 * string literal. This will help the programmer to debug easily with log traces
 * without navigating through the source code.
 *
 * Parameters:
 *  wiced_bt_gatt_status_t status: GATT status
 *
 * Return:
 *  wiced_bt_gatt_status_t
 *
 **************************************************************************************************/
const char* get_bt_gatt_status_name(wiced_bt_gatt_status_t status)
{
    switch ((int)status)
    {
        CASE_RETURN_STR(WICED_BT_GATT_SUCCESS || WICED_BT_GATT_ENCRYPTED_MITM)
        CASE_RETURN_STR(WICED_BT_GATT_INVALID_HANDLE)
        CASE_RETURN_STR(WICED_BT_GATT_READ_NOT_PERMIT)
        CASE_RETURN_STR(WICED_BT_GATT_WRITE_NOT_PERMIT)
        CASE_RETURN_STR(WICED_BT_GATT_INVALID_PDU)
        CASE_RETURN_STR(WICED_BT_GATT_INSUF_AUTHENTICATION)
        CASE_RETURN_STR(WICED_BT_GATT_REQ_NOT_SUPPORTED)
        CASE_RETURN_STR(WICED_BT_GATT_INVALID_OFFSET)
        CASE_RETURN_STR(WICED_BT_GATT_INSUF_AUTHORIZATION)
        CASE_RETURN_STR(WICED_BT_GATT_PREPARE_Q_FULL)
        CASE_RETURN_STR(WICED_BT_GATT_NOT_LONG)
        CASE_RETURN_STR(WICED_BT_GATT_INSUF_KEY_SIZE)
        CASE_RETURN_STR(WICED_BT_GATT_INVALID_ATTR_LEN)
        CASE_RETURN_STR(WICED_BT_GATT_ERR_UNLIKELY)
        CASE_RETURN_STR(WICED_BT_GATT_INSUF_ENCRYPTION)
        CASE_RETURN_STR(WICED_BT_GATT_UNSUPPORT_GRP_TYPE)
        CASE_RETURN_STR(WICED_BT_GATT_INSUF_RESOURCE)
        CASE_RETURN_STR(WICED_BT_GATT_ILLEGAL_PARAMETER)
        CASE_RETURN_STR(WICED_BT_GATT_NO_RESOURCES)
        CASE_RETURN_STR(WICED_BT_GATT_INTERNAL_ERROR)
        CASE_RETURN_STR(WICED_BT_GATT_WRONG_STATE)
        CASE_RETURN_STR(WICED_BT_GATT_DB_FULL)
        CASE_RETURN_STR(WICED_BT_GATT_BUSY)
        CASE_RETURN_STR(WICED_BT_GATT_ERROR)
        CASE_RETURN_STR(WICED_BT_GATT_CMD_STARTED)
        CASE_RETURN_STR(WICED_BT_GATT_PENDING)
        CASE_RETURN_STR(WICED_BT_GATT_AUTH_FAIL)
        CASE_RETURN_STR(WICED_BT_GATT_MORE)
        CASE_RETURN_STR(WICED_BT_GATT_INVALID_CFG)
        CASE_RETURN_STR(WICED_BT_GATT_SERVICE_STARTED)
        CASE_RETURN_STR(WICED_BT_GATT_ENCRYPTED_NO_MITM)
        CASE_RETURN_STR(WICED_BT_GATT_NOT_ENCRYPTED)
        CASE_RETURN_STR(WICED_BT_GATT_CONGESTED)
        CASE_RETURN_STR(WICED_BT_GATT_WRITE_REQ_REJECTED)
        CASE_RETURN_STR(WICED_BT_GATT_CCC_CFG_ERR)
        CASE_RETURN_STR(WICED_BT_GATT_PRC_IN_PROGRESS)
        CASE_RETURN_STR(WICED_BT_GATT_OUT_OF_RANGE)
    }

    return "UNKNOWN_STATUS";
}


/***************************************************************************************************
 * Function Name: get_wifi_security_name
 ***************************************************************************************************
 * Summary:
 * The function converts the cy_wcm_security_t enum value to its corresponding
 * string literal. This will help the programmer to debug easily with log traces
 * without navigating through the source code.
 *
 * Parameters:
 *  cy_wcm_security_t name: WiFi Security Type
 *
 * Return:
 *  cy_wcm_security_t
 *
 **************************************************************************************************/
const char* get_wifi_security_name(cy_wcm_security_t name)
{
    switch ((int)name)
    {
        CASE_RETURN_STR(CY_WCM_SECURITY_OPEN)
        CASE_RETURN_STR(CY_WCM_SECURITY_WEP_SHARED)
        CASE_RETURN_STR(CY_WCM_SECURITY_WEP_PSK)
        CASE_RETURN_STR(CY_WCM_SECURITY_WPA_TKIP_PSK)
        CASE_RETURN_STR(CY_WCM_SECURITY_WPA_AES_PSK)
        CASE_RETURN_STR(CY_WCM_SECURITY_WPA_MIXED_PSK)
        CASE_RETURN_STR(CY_WCM_SECURITY_WPA2_AES_PSK)
        CASE_RETURN_STR(CY_WCM_SECURITY_WPA2_TKIP_PSK)
        CASE_RETURN_STR(CY_WCM_SECURITY_WPA2_MIXED_PSK)
        CASE_RETURN_STR(CY_WCM_SECURITY_WPA2_FBT_PSK)
        CASE_RETURN_STR(CY_WCM_SECURITY_WPA3_SAE)
        CASE_RETURN_STR(CY_WCM_SECURITY_WPA3_WPA2_PSK)
        CASE_RETURN_STR(CY_WCM_SECURITY_IBSS_OPEN)
        CASE_RETURN_STR(CY_WCM_SECURITY_WPS_SECURE)
        CASE_RETURN_STR(CY_WCM_SECURITY_UNKNOWN)
        CASE_RETURN_STR(CY_WCM_SECURITY_FORCE_32_BIT)
    }

    return "UNKNOWN SECURITY";
}


/* [] END OF FILE */
