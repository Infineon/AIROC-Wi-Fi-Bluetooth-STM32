/*
 * Copyright 2020, Cypress Semiconductor Corporation or a subsidiary of
 * Cypress Semiconductor Corporation. All Rights Reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software"), is owned by Cypress Semiconductor Corporation
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

/**
* @file cy_wcm.c
* @brief Wi-Fi connection manager (WCM) provides set of APIs that are useful
* to establish and monitor Wi-Fi connection on Cypress platforms that support Wi-Fi connectivity.
* WCM APIs are easy to use compared to WHD APIs and also provides additional features.
* Refer individual APIs for more details.
*/

#include "cy_wcm.h"
#include "cy_wcm_log.h"
#include "cy_wcm_error.h"
#include "cybsp_wifi.h"
#include "cyabs_rtos.h"
#include "cy_worker_thread.h"
#include "cy_lwip.h"
#include "cy_chip_constants.h"

/* Wi-Fi Host driver includes. */
#include "whd.h"
#include "whd_wifi_api.h"
#include "whd_network_types.h"
#include "whd_buffer_api.h"
#include "whd_wlioctl.h"

/* lwIP stack includes */
#include <lwipopts.h>
#include <lwip/dhcp.h>
#include <lwip/dns.h>
#include <lwip/inet.h>
#include <lwip/netdb.h>
#include <lwip/netif.h>
#include <lwip/netifapi.h>
#include <lwip/init.h>
#include <lwip/dhcp.h>
#include <lwip/etharp.h>
#include <lwip/tcpip.h>
#include <lwip/ethip6.h>
#include <lwip/igmp.h>
#include <lwip/icmp.h>
#include <lwip/inet_chksum.h>
#include <netif/ethernet.h>

#include "whd_debug.h"

/* While using lwip/sockets errno is required. Since IAR doesn't define errno, the below definition is required for IAR build */
#if defined(__ICCARM__)
#ifndef errno
int errno;
#endif
#endif

/**
 *  Macro for comparing MAC addresses
 */
#define CMP_MAC( a, b )  (((((unsigned char*)a)[0])==(((unsigned char*)b)[0]))&& \
                          ((((unsigned char*)a)[1])==(((unsigned char*)b)[1]))&& \
                          ((((unsigned char*)a)[2])==(((unsigned char*)b)[2]))&& \
                          ((((unsigned char*)a)[3])==(((unsigned char*)b)[3]))&& \
                          ((((unsigned char*)a)[4])==(((unsigned char*)b)[4]))&& \
                          ((((unsigned char*)a)[5])==(((unsigned char*)b)[5])))


/**
 *  Macro for checking for NULL MAC addresses
 */
#define NULL_MAC(a)  ( ( ( ( (unsigned char *)a )[0] ) == 0 ) && \
                       ( ( ( (unsigned char *)a )[1] ) == 0 ) && \
                       ( ( ( (unsigned char *)a )[2] ) == 0 ) && \
                       ( ( ( (unsigned char *)a )[3] ) == 0 ) && \
                       ( ( ( (unsigned char *)a )[4] ) == 0 ) && \
                       ( ( ( (unsigned char *)a )[5] ) == 0 ) )


#define CY_WCM_MAX_MUTEX_WAIT_TIME_MS               (120000)
#define CY_WCM_INTERFACE_TYPE_UNKNOWN               (4)
#define CY_WCM_DEFAULT_STA_CHANNEL                  (1)
#define WCM_WORKER_THREAD_PRIORITY                  (CY_RTOS_PRIORITY_ABOVENORMAL)
#define WCM_WORKER_THREAD_STACK_SIZE                (10 * 1024)
#define WCM_HANDSHAKE_TIMEOUT_MS                    (3000)
#define WLC_EVENT_MSG_LINK                          (0x01)
#define JOIN_RETRY_ATTEMPTS                         (3)
#define DEFAULT_RETRY_BACKOFF_TIMEOUT_IN_MS         (1000)
#define MAX_RETRY_BACKOFF_TIMEOUT_IN_MS             (DEFAULT_RETRY_BACKOFF_TIMEOUT_IN_MS * 32)
#define DHCP_TIMEOUT_COUNT                          (6000) /* 6000 times */
#define UNKNOWN_BAND_WIDTH                          (0)
#define ARP_WAIT_TIME_IN_MSEC                       (30000)
#define ARP_CACHE_CHECK_INTERVAL_IN_MSEC            (5)
#define PING_ID                                     (0xAFAF)  /** ICMP Identifier for PING */
#define PING_DATA_SIZE                              (64)      /** ping additional data size to include in the packet */
#define MAX_WHD_INTERFACE                           (2)
#define MAX_STA_CLIENTS                             (3)
#define TX_BIT_RATE_CONVERTER                       (500)
#define PING_IF_NAME_LEN                            (6)
#define PING_RESPONSE_LEN                           (64)

/******************************************************
 *             Structures
 ******************************************************/
typedef struct
{
    cy_wcm_scan_result_callback_t p_scan_calback;
    void*                         user_data;
    whd_scan_result_t             scan_res;
    whd_scan_status_t             scan_status;
    cy_wcm_scan_filter_t          scan_filter;
    bool                          is_scanning;
}wcm_internal_scan_t;

struct icmp_packet
{
    struct icmp_echo_hdr hdr;
    uint8_t data[PING_DATA_SIZE];
};

static wcm_internal_scan_t scan_handler;

typedef struct
{
    whd_ssid_t SSID;
    whd_mac_t sta_mac;
    cy_wcm_wifi_band_t band;
    uint8_t key[CY_WCM_MAX_PASSPHRASE_LEN];
    uint8_t keylen;
    whd_security_t security;
    ip_static_addr_t static_ip;
}wcm_ap_details;

static wcm_ap_details connected_ap_details;

typedef struct
{
  cy_wcm_event_t event;
  cy_wcm_mac_t   mac_addr;
} wcm_ap_link_event;

/******************************************************
 *               Variable Definitions
 ******************************************************/
/* Fixme: Static is removed as STA interface is used for WPS in cy_wcm_wps.c, once get_whd_interface API is available
 * it will be made to static again */

/* Array containing STA Wi-Fi driver and AP Wi-Fi driver */
whd_interface_t whd_ifs[MAX_WHD_INTERFACE];

bool is_wcm_initalized                 = false;
static cy_mutex_t wcm_mutex;
static cy_wcm_interface_t                current_interface;
static bool wcm_sta_link_up            = false;
static bool is_soft_ap_up              = false;
static bool is_sta_network_up          = false;
static bool is_ap_network_up           = false;
static whd_security_t                  sta_security_type;
static cy_worker_thread_info_t         cy_wcm_worker_thread;
static bool is_olm_initialized         = false;
static void *olm_instance              = NULL;

whd_scan_result_t scan_result;
cy_wcm_scan_result_callback_t p_scan_calback;
cy_wcm_scan_filter_t *p_scan_filter = NULL;

static cy_timer_t sta_handshake_timer;
static cy_timer_t sta_retry_timer;
static cy_wcm_event_callback_t wcm_event_handler[CY_WCM_MAXIMUM_CALLBACKS_COUNT];
static uint16_t sta_event_handler_index   = 0xFF;
static uint16_t ap_event_handler_index    = 0xFF;
static const whd_event_num_t  sta_link_events[] = {WLC_E_LINK, WLC_E_DEAUTH_IND, WLC_E_DISASSOC_IND, WLC_E_PSK_SUP, WLC_E_NONE};
static const whd_event_num_t  ap_link_events[]  = {WLC_E_DISASSOC_IND, WLC_E_ASSOC_IND, WLC_E_REASSOC_IND, WLC_E_NONE};
static bool too_many_ie_error          = false;
static bool link_up_event_received     = false;
static uint32_t retry_backoff_timeout  = DEFAULT_RETRY_BACKOFF_TIMEOUT_IN_MS;

/******************************************************
 *               Static Function Declarations
 ******************************************************/
static cy_rslt_t check_ap_credentials(const cy_wcm_connect_params_t *connect_params);
static cy_rslt_t convert_connect_params(const cy_wcm_connect_params_t *connect_params, whd_ssid_t *ssid, whd_mac_t *bssid, uint8_t **key, uint8_t *keylen, whd_security_t *security, ip_static_addr_t *static_ip_addr);
static void donothing(void *arg);
static bool is_connected_to_same_ap(const cy_wcm_connect_params_t *connect_params);
static bool check_wcm_security(cy_wcm_security_t sec);
static void internal_scan_callback(whd_scan_result_t **result_ptr, void *user_data, whd_scan_status_t status);
static void notify_scan_event(void *arg);
static void *link_events_handler(whd_interface_t ifp, const whd_event_header_t *event_header, const uint8_t *event_data, void *handler_user_data);
static void link_up(void);
static void link_down(void);
static void handshake_timeout_handler(cy_timer_callback_arg_t arg);
static void handshake_error_callback(void *arg);
static void lwip_ip_change_callback(void *arg);
static bool check_if_platform_supports_band(cy_wcm_wifi_band_t requested_band);
static void sta_link_down_handler(void* arg);
static void sta_link_up_handler(void* arg);
static void sta_link_up_renew_handler(void* arg);
void notify_ip_change(void *arg);
static cy_rslt_t network_up(whd_interface_t interface, cy_lwip_nw_interface_role_t role, ip_static_addr_t *static_ip_ptr);
static void network_down(whd_interface_t interface, cy_lwip_nw_interface_role_t role);
static void hanshake_retry_timer(cy_timer_callback_arg_t arg);
static void invoke_app_callbacks(cy_wcm_event_t event_type, cy_wcm_event_data_t* arg);
static cy_wcm_security_t whd_to_wcm_security(whd_security_t sec);
static cy_wcm_bss_type_t  whd_to_wcm_bss_type(whd_bss_type_t bss_type);
static cy_wcm_wifi_band_t whd_to_wcm_band(whd_802_11_band_t band);
static whd_security_t wcm_to_whd_security(cy_wcm_security_t sec);
static uint16_t channel_to_bandwidth(wl_chanspec_t chanspec);
static void notify_connection_status(void* arg);
#if LWIP_IPV4
static void ping_prepare_echo(struct icmp_packet *iecho, uint16_t len, uint16_t *ping_seq_num);
static err_t ping_send(int socket_hnd, const cy_wcm_ip_address_t* address, struct icmp_packet *iecho, uint16_t *sequence_number);
static err_t ping_recv(int socket_hnd, cy_wcm_ip_address_t* address, uint16_t *ping_seq_num);
#endif
static cy_rslt_t check_soft_ap_config(const cy_wcm_ap_config_t *ap_config_params);
static void read_ap_config(const cy_wcm_ap_config_t *ap_config, whd_ssid_t *ssid, uint8_t **key, uint8_t *keylen, whd_security_t *security, ip_static_addr_t *static_ip_addr);
static void* ap_link_events_handler(whd_interface_t ifp, const whd_event_header_t *event_header, const uint8_t *event_data, void *handler_user_data);
static cy_rslt_t init_whd_wifi_interface(cy_wcm_interface_t iface_type);

/******************************************************
 *               Function Definitions
 ******************************************************/
/** Offload Manager create
 *
 * @param[in]  ifp : This is whd_interface_t pointer
 *
 * @param[in]  oflds_list: This is offload list managed by Offload Manager.
 *
 * @return @ref CY_RSLT_SUCCESS if OLM create is success or failure code.
 *
 */
CYPRESS_WEAK cy_rslt_t cy_olm_create(void *ifp, void *oflds_list)
{
    cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "%s \n", __func__);
    return CY_RSLT_SUCCESS;
}

/* Offload Manager de-init called once STA disconnect with AP
 *
 * @param[in] olm: olm instance
 */
CYPRESS_WEAK void cy_olm_deinit_ols(void *olm)
{
    cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "%s \n", __func__);
}

/*
 *  Offload Manager initialization.
 *
 *  @param[in] olm : olm instance
 *
 *  @param[in] whd : whd_interface_t
 *
 *  @param[in] ip  : pointer to ipv4 address
 *
 *  @return @ref CY_RSLT_SUCCESS if success or failure code
 *
 */

CYPRESS_WEAK cy_rslt_t cy_olm_init_ols(void *olm, void *whd, void *ip)
{
    cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "%s \n", __func__);
    return CY_RSLT_SUCCESS;
}

/*
 * Function returns the Offload Manager instance
 *
 *  @return olm: olm pointer
 *
 */
CYPRESS_WEAK void *cy_get_olm_instance( void )
{
    cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "%s \n", __func__);
    return NULL;
}

cy_rslt_t cy_wcm_init(cy_wcm_config_t *config)
{
    cy_worker_thread_params_t params;
    cy_rslt_t res = CY_RSLT_SUCCESS;

    if((config == NULL) ||
       ((config->interface != CY_WCM_INTERFACE_TYPE_AP_STA) && (config->interface  != CY_WCM_INTERFACE_TYPE_STA) && (config->interface != CY_WCM_INTERFACE_TYPE_AP)))
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Error : WCM init failed due to bad arguments \n");
        return CY_RSLT_WCM_BAD_ARG;
    }

    if(is_wcm_initalized)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "WCM is already initialized \n");
        return CY_RSLT_SUCCESS;
    }

    /** Initialize TCP ip stack, LWIP init is called through tcpip_init **/
    tcpip_init(donothing, NULL) ;

    if (cy_rtos_init_mutex(&wcm_mutex) != CY_RSLT_SUCCESS)
    {
        return CY_RSLT_WCM_MUTEX_ERROR;
    }

    if((res = init_whd_wifi_interface(config->interface)) != CY_RSLT_SUCCESS)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Error : Initializing Wi-Fi interface \n");
        cy_rtos_deinit_mutex(&wcm_mutex);
        return res;
    }

    memset(&params, 0, sizeof(params));
    params.name = "WCM- Worker";
    params.priority = WCM_WORKER_THREAD_PRIORITY;
    params.stack = NULL;
    params.stack_size = WCM_WORKER_THREAD_STACK_SIZE;
    params.num_entries = 0;
    /* create a worker thread */
    if(cy_worker_thread_create(&cy_wcm_worker_thread, &params) != CY_RSLT_SUCCESS)
    {
        cy_rtos_deinit_mutex(&wcm_mutex);
        return CY_RSLT_WCM_MUTEX_ERROR;
    }

    if(config->interface != CY_WCM_INTERFACE_TYPE_AP)
    {
        cy_rtos_init_timer(&sta_handshake_timer, CY_TIMER_TYPE_ONCE, handshake_timeout_handler, 0);
        cy_rtos_init_timer(&sta_retry_timer, CY_TIMER_TYPE_ONCE, hanshake_retry_timer, 0);
        scan_handler.is_scanning = false;
        olm_instance = cy_get_olm_instance();
        if(olm_instance != NULL)
        {
            /* Offload Manager create used only for STA interface
             * not required for Soft AP interface
             */
            cy_olm_create(whd_ifs[CY_WCM_INTERFACE_TYPE_STA], NULL);
        }
        else
        {
            cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "Off loads not configured \n");
        }
        /* Register to ip change callback from LwIP, all other internal callbacks are in WCM */
        cy_lwip_register_ip_change_cb(lwip_ip_change_callback);
    }

    memset(wcm_event_handler, 0, sizeof(wcm_event_handler));
    current_interface = config->interface;
    is_wcm_initalized = true;
    return res;
}

cy_rslt_t cy_wcm_deinit()
{
    cy_rslt_t res = CY_RSLT_SUCCESS;

    if(!is_wcm_initalized)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "WCM is not initialized \n");
        return CY_RSLT_WCM_NOT_INITIALIZED;
    }

    /** Check if there are any active connections and disconnect **/
    if ((res = cy_wcm_disconnect_ap()) != CY_RSLT_SUCCESS)
    {
        wcm_sta_link_up = false;
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Error while disconnecting  \n");
        res = CY_RSLT_WCM_STA_DISCONNECT_ERROR;
    }

    if((res = cy_wcm_stop_scan()) != CY_RSLT_SUCCESS)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Error while stopping scan  \n");
    }

    if((res = cy_wcm_stop_ap()) != CY_RSLT_SUCCESS)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Error while stopping AP \n");
    }

    if ((res =cy_rtos_deinit_mutex(&wcm_mutex)) != CY_RSLT_SUCCESS)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Error while de initializing mutex \n");;
    }

    if(current_interface != CY_WCM_INTERFACE_TYPE_AP_STA)
    {
        if ((res = cybsp_wifi_deinit(whd_ifs[current_interface])) != CY_RSLT_SUCCESS)
        {
            cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Error in bringing down the interface \n");
        }
    }
    else
    {
        /* In concurrent bringing down AP interface, followed by STA interface */
        if((res = whd_deinit(whd_ifs[CY_WCM_INTERFACE_TYPE_AP])) != CY_RSLT_SUCCESS)
        {
            cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Error in bringing down AP interface \n");
        }

        if ((res = cybsp_wifi_deinit(whd_ifs[CY_WCM_INTERFACE_TYPE_STA])) != CY_RSLT_SUCCESS)
        {
            cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Error in bringing down STA interface \n");
        }
    }

    if(current_interface != CY_WCM_INTERFACE_TYPE_AP)
    {
        cy_rtos_deinit_timer(&sta_handshake_timer);
        cy_rtos_deinit_timer(&sta_retry_timer);
        retry_backoff_timeout = DEFAULT_RETRY_BACKOFF_TIMEOUT_IN_MS;
    }
    cy_worker_thread_delete(&cy_wcm_worker_thread);
    is_wcm_initalized = false;

    return res;
}

cy_rslt_t cy_wcm_start_scan(cy_wcm_scan_result_callback_t callback, void *user_data, cy_wcm_scan_filter_t *scan_filter)
{
    cy_rslt_t res = CY_RSLT_SUCCESS;
    whd_ssid_t *ssid = NULL;
    whd_mac_t *mac = NULL;
    uint32_t band;

    if(!is_wcm_initalized)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "WCM STA interface is not initialized \n");
        return CY_RSLT_WCM_NOT_INITIALIZED;
    }

    if (callback == NULL)
    {
        return CY_RSLT_WCM_BAD_ARG;
    }

    if(cy_rtos_get_mutex(&wcm_mutex, CY_WCM_MAX_MUTEX_WAIT_TIME_MS) != CY_RSLT_SUCCESS)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Unable to acquire WCM mutex \n");
        return CY_RSLT_WCM_WAIT_TIMEOUT;
    }

    if(scan_handler.is_scanning)
    {
        res = CY_RSLT_WCM_SCAN_IN_PROGRESS;
        goto exit;
    }

    /* reset previous filter and by default set band to AUTO */
    memset(&scan_handler.scan_filter, 0, sizeof(cy_wcm_scan_filter_t));
    whd_wifi_set_ioctl_value(whd_ifs[CY_WCM_INTERFACE_TYPE_STA], WLC_SET_BAND, WLC_BAND_AUTO);

    scan_handler.p_scan_calback = callback;
    scan_handler.user_data = user_data;
    if(scan_filter != NULL)
    {
        /** copy the filter setting to reuse in the internal scan handler **/
        memcpy(&scan_handler.scan_filter, scan_filter, sizeof(cy_wcm_scan_filter_t));
        switch(scan_filter->mode)
        {
            case CY_WCM_SCAN_FILTER_TYPE_SSID:
                /** Copy the SSID **/
                ssid = (whd_ssid_t*)malloc(sizeof(whd_ssid_t));
                if(ssid == NULL)
                {
                    res =  CY_RSLT_WCM_OUT_OF_MEMORY;
                    goto exit;
                }
                ssid->length = strlen((char*)scan_filter->param.SSID);
                memcpy(ssid->value, scan_filter->param.SSID, ssid->length + 1);
                break;
            case CY_WCM_SCAN_FILTER_TYPE_MAC:
                /** Copy the MAC **/
                mac = (whd_mac_t*)malloc(sizeof(whd_mac_t));
                if(mac == NULL)
                {
                    res =  CY_RSLT_WCM_OUT_OF_MEMORY;
                    goto exit;
                }
                memcpy(mac->octet, scan_filter->param.BSSID, CY_WCM_MAC_ADDR_LEN);
                break;
            case CY_WCM_SCAN_FILTER_TYPE_BAND:
                 /* check if the platform supports the requested band */
                if(!check_if_platform_supports_band(scan_filter->param.band))
                {
                    cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "band not supported \n");
                    res = CY_RSLT_WCM_BAND_NOT_SUPPORTED;
                    goto exit;
                }
                if(scan_filter->param.band == CY_WCM_WIFI_BAND_5GHZ)
                {
                    band = WLC_BAND_5G;
                }
                else if(scan_filter->param.band == CY_WCM_WIFI_BAND_2_4GHZ)
                {
                    band = WLC_BAND_2G;
                }
                else
                {
                    band = WLC_BAND_AUTO;
                }
                whd_wifi_set_ioctl_value(whd_ifs[CY_WCM_INTERFACE_TYPE_STA], WLC_SET_BAND, band);
                break;
            case CY_WCM_SCAN_FILTER_TYPE_RSSI:
                    /**
                    * Do nothing, the scan filter is copied into scan_handler
                    * which will be used in worker thread to process the received RSSI
                    */
                break;
            default:
                break;
        }
    }
    res = whd_wifi_scan(whd_ifs[CY_WCM_INTERFACE_TYPE_STA], WHD_SCAN_TYPE_ACTIVE, WHD_BSS_TYPE_ANY,
                                 ssid, mac, NULL, NULL, internal_scan_callback, &scan_result, user_data);
    if(res != CY_RSLT_SUCCESS)
    {
        res = CY_RSLT_WCM_SCAN_ERROR;
        goto exit;
    }
    scan_handler.is_scanning = true;

exit:
    /* Free the allocated memory in case of SSID or MAC based scan */
    if(ssid != NULL)
    {
        free(ssid);
        ssid = NULL;
    }
    if(mac != NULL)
    {
        free(mac);
        mac = NULL;
    }
    if (cy_rtos_set_mutex(&wcm_mutex) != CY_RSLT_SUCCESS)
    {
        res = ((res != CY_RSLT_SUCCESS) ? res : CY_RSLT_WCM_MUTEX_ERROR);
    }
    return res;
}

cy_rslt_t cy_wcm_stop_scan()
{
    cy_rslt_t res = CY_RSLT_SUCCESS;

    if(!is_wcm_initalized)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "WCM is not initialized to initialize call cy_wcm_init() \n");
        return CY_RSLT_WCM_NOT_INITIALIZED;
    }

    if(cy_rtos_get_mutex(&wcm_mutex, CY_WCM_MAX_MUTEX_WAIT_TIME_MS) != CY_RSLT_SUCCESS)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Unable to acquire WCM mutex \n");
        return CY_RSLT_WCM_WAIT_TIMEOUT;
    }

    if(!scan_handler.is_scanning)
    {
        res =  CY_RSLT_WCM_NO_ACTIVE_SCAN;
        goto exit;
    }
    scan_handler.is_scanning = false;
    if((res = whd_wifi_stop_scan(whd_ifs[CY_WCM_INTERFACE_TYPE_STA]) != CY_RSLT_SUCCESS))
    {
        res = CY_RSLT_WCM_STOP_SCAN_ERROR;
        goto exit;
    }

exit:
    if (cy_rtos_set_mutex(&wcm_mutex) != CY_RSLT_SUCCESS)
    {
        res = ((res != CY_RSLT_SUCCESS) ? res : CY_RSLT_WCM_MUTEX_ERROR);
    }
    return res;
}

cy_rslt_t cy_wcm_register_event_callback(cy_wcm_event_callback_t event_callback)
{
    uint8_t i;

    if( !is_wcm_initalized )
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "WCM is not initialized, to initialize call cy_wcm_init() \n");
        return CY_RSLT_WCM_NOT_INITIALIZED;
    }

    if( event_callback == NULL )
    {
        return CY_RSLT_WCM_BAD_ARG;
    }

    for ( i = 0; i < CY_WCM_MAXIMUM_CALLBACKS_COUNT; i++ )
    {
        if ( wcm_event_handler[i] == NULL )
        {
            wcm_event_handler[i] = event_callback;
            return CY_RSLT_SUCCESS;
        }
    }

    cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Out of CY_WCM_MAXIMUM_CALLBACKS_COUNT \r\n");
    return CY_RSLT_WCM_OUT_OF_MEMORY;
}

cy_rslt_t cy_wcm_deregister_event_callback(cy_wcm_event_callback_t event_callback)
{
    uint8_t i;

    if( !is_wcm_initalized )
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "WCM is not initialized, to initialize call cy_wcm_init() \n");
        return CY_RSLT_WCM_NOT_INITIALIZED;
    }

    if( event_callback == NULL )
    {
        return CY_RSLT_WCM_BAD_ARG;
    }

    for ( i = 0; i < CY_WCM_MAXIMUM_CALLBACKS_COUNT; i++ )
    {
        if ( wcm_event_handler[i] == event_callback )
        {
            memset( &wcm_event_handler[i], 0, sizeof( cy_wcm_event_callback_t ) );
            return CY_RSLT_SUCCESS;
        }
    }

    cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Unable to find callback to deregister \r\n");
    return CY_RSLT_WCM_BAD_ARG;
}


cy_rslt_t cy_wcm_connect_ap(const cy_wcm_connect_params_t *connect_params, cy_wcm_ip_address_t *ip_addr)
{
    cy_rslt_t res = CY_RSLT_SUCCESS;
    whd_ssid_t ssid;
    whd_mac_t bssid;
    uint8_t *key;
    uint8_t keylen;
    whd_security_t security;
    ip_static_addr_t static_ip, *static_ip_ptr;
    static_ip_ptr = NULL;
#if LWIP_IPV4
    uint32_t retry_count = 0;
    uint32_t ipv4_addr;
    ip4_addr_t* addr = NULL;
    UNUSED_VARIABLE(addr);
#endif
    uint8_t connection_status;
    struct netif *net_interface = NULL;

    if(!is_wcm_initalized)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "WCM STA interface is not initialized \n");
        return CY_RSLT_WCM_NOT_INITIALIZED;
    }

    memset(&connected_ap_details, 0, sizeof(connected_ap_details));
    if((res = check_ap_credentials(connect_params)) != CY_RSLT_SUCCESS)
    {
        return res;
    }
    if (is_connected_to_same_ap(connect_params))
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "already connected to same AP \n");
        return CY_RSLT_SUCCESS;
    }
    if (wcm_sta_link_up  && (cy_wcm_disconnect_ap() != CY_RSLT_SUCCESS))
    {
        /**
         *  Notify user disconnection error occurred and
         *  reset the is_wifi_connected flag to false
         */
        wcm_sta_link_up = false;
        return CY_RSLT_WCM_STA_DISCONNECT_ERROR;
    }
    if (cy_rtos_get_mutex(&wcm_mutex, CY_WCM_MAX_MUTEX_WAIT_TIME_MS) == CY_RSLT_SUCCESS)
    {
        whd_scan_result_t ap;
        convert_connect_params(connect_params, &ssid, &bssid, &key, &keylen, &security, &static_ip);
        sta_security_type = security;

        connection_status = CY_WCM_EVENT_CONNECTING;
        if((res = cy_worker_thread_enqueue(&cy_wcm_worker_thread, notify_connection_status, &connection_status)) != CY_RSLT_SUCCESS)
        {
            cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "L%d : %s() : ERROR : Failed to send connection status. Err = [%lu]\r\n", __LINE__, __FUNCTION__, res);
            goto exit;
        }

        if(!NULL_MAC(bssid.octet))
        {
            memset(&ap, 0, sizeof(whd_scan_result_t));
            ap.security = security;
            ap.SSID.length = ssid.length;
            ap.channel = CY_WCM_DEFAULT_STA_CHANNEL;
            memcpy(ap.SSID.value, ssid.value, ap.SSID.length + 1);
            memcpy(ap.BSSID.octet, bssid.octet, CY_WCM_MAC_ADDR_LEN);
            /*
             * If MAC address of a AP is know there is no need to populate channel or band
             * instead we can set the band to auto and invoke whd join
             */
            whd_wifi_set_ioctl_value(whd_ifs[CY_WCM_INTERFACE_TYPE_STA], WLC_SET_BAND, WLC_BAND_AUTO);
            res = whd_wifi_join_specific(whd_ifs[CY_WCM_INTERFACE_TYPE_STA], &ap, key, keylen);
        }
        else
        {
            if(connect_params->band == CY_WCM_WIFI_BAND_5GHZ)
            {
                /* check if this band is supported locally */
                if(check_if_platform_supports_band(CY_WCM_WIFI_BAND_5GHZ))
                {
                    whd_wifi_set_ioctl_value(whd_ifs[CY_WCM_INTERFACE_TYPE_STA], WLC_SET_BAND, WLC_BAND_5G);
                }
                else
                {
                    cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "band not supported \n");
                    res =  CY_RSLT_WCM_BAND_NOT_SUPPORTED;
                    connection_status = CY_WCM_EVENT_CONNECT_FAILED;
                    if((cy_worker_thread_enqueue(&cy_wcm_worker_thread, notify_connection_status, &connection_status)) != CY_RSLT_SUCCESS)
                    {
                        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "L%d : %s() : ERROR : Failed to send connection status. Err = [%lu]\r\n", __LINE__, __FUNCTION__, res);
                    }
                    goto exit;
                }
            }
            else if(connect_params->band == CY_WCM_WIFI_BAND_2_4GHZ)
            {
                whd_wifi_set_ioctl_value(whd_ifs[CY_WCM_INTERFACE_TYPE_STA], WLC_SET_BAND, WLC_BAND_2G);
            }
            else
            {
                /* If band is not specified set the band to AUTO */
                whd_wifi_set_ioctl_value(whd_ifs[CY_WCM_INTERFACE_TYPE_STA], WLC_SET_BAND, WLC_BAND_AUTO);
            }
            /** Join to Wi-Fi AP **/
            res = whd_wifi_join(whd_ifs[CY_WCM_INTERFACE_TYPE_STA], &ssid, security, key, keylen);
        }
        if (res != CY_RSLT_SUCCESS)
        {
            cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "whd_wifi join failed : %ld \n", res);
            connection_status = CY_WCM_EVENT_CONNECT_FAILED;
            if((cy_worker_thread_enqueue(&cy_wcm_worker_thread, notify_connection_status, &connection_status)) != CY_RSLT_SUCCESS)
            {
                cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "L%d : %s() : ERROR : Failed to send connection status. Err = [%lu]\r\n", __LINE__, __FUNCTION__, res);
            }
            goto exit;
        }

        /* Call Offload init after connect to AP */
        if ((is_olm_initialized == false) && ( olm_instance != NULL))
        {
            cy_olm_init_ols(olm_instance, whd_ifs[CY_WCM_INTERFACE_TYPE_STA], NULL);
            is_olm_initialized = true;
        }

        if (!is_sta_network_up)
        {
            if(connect_params->static_ip_settings != NULL)
            {
                static_ip_ptr = &static_ip;
            }

            if((res = network_up(whd_ifs[CY_WCM_INTERFACE_TYPE_STA], CY_LWIP_STA_NW_INTERFACE, static_ip_ptr)) != CY_RSLT_SUCCESS)
            {
                cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Failed to bring up the network stack\n");
                res = CY_RSLT_WCM_STA_NETWORK_DOWN;
                connection_status = CY_WCM_EVENT_CONNECT_FAILED;
                if((cy_worker_thread_enqueue(&cy_wcm_worker_thread, notify_connection_status, &connection_status)) != CY_RSLT_SUCCESS)
                {
                    cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "L%d : %s() : ERROR : Failed to send connection status. Err = [%lu]\r\n", __LINE__, __FUNCTION__, res);
                }
                goto exit;
            }
            net_interface = cy_lwip_get_interface(CY_LWIP_STA_NW_INTERFACE);
#if LWIP_IPV4
            /** wait in busy loop till dhcp starts and ip address gets assigned **/
            while (true)
            {
#if LWIP_IPV6
                ipv4_addr = net_interface->ip_addr.u_addr.ip4.addr;
                addr = &net_interface->ip_addr.u_addr.ip4;
#else
                ipv4_addr = net_interface->ip_addr.addr;
                addr = &net_interface->ip_addr;
#endif
                if (ipv4_addr != 0)
                {
                    cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "IPV4 Address %s assigned \n", ip4addr_ntoa(addr));
                    if(ip_addr != NULL)
                    {
                        ip_addr->version = CY_WCM_IP_VER_V4;
                        ip_addr->ip.v4 = ipv4_addr;
                    }
                    break;
                }
                // TO DO : get ipv6 address
                /* Delay of 10 ms */
                cy_rtos_delay_milliseconds(10);
                /* Increment count for every 10 ms */
                retry_count++;
                /* Return DHCP Timeout Error when it exceeds 6000 * 10 ms = 60 seconds */
                if (retry_count > DHCP_TIMEOUT_COUNT)
                {
                    cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "DHCP Timeout \n");
                    /* do disconnect to bring network down as DHCP failed */
                    if (cy_wcm_disconnect_ap() != CY_RSLT_SUCCESS)
                    {
                        /**
                         *  Notify user disconnection error occurred and
                         *  reset the is_wifi_connected flag to false
                         */
                        wcm_sta_link_up = false;
                        res = CY_RSLT_WCM_STA_DISCONNECT_ERROR;
                        connection_status = CY_WCM_EVENT_CONNECT_FAILED;
                        if(cy_worker_thread_enqueue(&cy_wcm_worker_thread, notify_connection_status, &connection_status) != CY_RSLT_SUCCESS)
                        {
                            cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "L%d : %s() : ERROR : Failed to send connection status. Err = [%lu]\r\n", __LINE__, __FUNCTION__, res);
                        }
                        goto exit;
                    }
                    /* Return DHCP Timeout Error when DHCP discover failed and disconnect done properly */
                    res = CY_RSLT_WCM_DHCP_TIMEOUT;
                    connection_status = CY_WCM_EVENT_CONNECT_FAILED;
                    if(cy_worker_thread_enqueue(&cy_wcm_worker_thread, notify_connection_status, &connection_status) != CY_RSLT_SUCCESS)
                    {
                        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "L%d : %s() : ERROR : Failed to send connection status. Err = [%lu]\r\n", __LINE__, __FUNCTION__, res);
                    }
                    goto exit;
                }
            }
#endif
            /* Register for Link events*/
            res = whd_management_set_event_handler(whd_ifs[CY_WCM_INTERFACE_TYPE_STA], sta_link_events, link_events_handler, NULL, &sta_event_handler_index);
            if(res != CY_RSLT_SUCCESS)
            {
                /* bring down the network and leave */
                cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Unable to register for Link events \n");
                network_down(whd_ifs[CY_WCM_INTERFACE_TYPE_STA], CY_LWIP_STA_NW_INTERFACE);
                whd_wifi_leave(whd_ifs[CY_WCM_INTERFACE_TYPE_STA]);
                connection_status = CY_WCM_EVENT_CONNECT_FAILED;
                if(cy_worker_thread_enqueue(&cy_wcm_worker_thread, notify_connection_status, &connection_status) != CY_RSLT_SUCCESS)
                {
                    cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "L%d : %s() : ERROR : Failed to send connection status. Err = [%lu]\r\n", __LINE__, __FUNCTION__, res);
                }
                goto exit;
            }

            /* save current AP credentials to reuse during retry in case handshake fails occurs */
            connected_ap_details.security = security;
            connected_ap_details.SSID.length = ssid.length;
            connected_ap_details.keylen = keylen;
            connected_ap_details.band = connect_params->band;
            memcpy(connected_ap_details.key, key, keylen+1);
            memcpy(connected_ap_details.SSID.value, ssid.value, connected_ap_details.SSID.length+1);
            memcpy(connected_ap_details.sta_mac.octet, bssid.octet, CY_WCM_MAC_ADDR_LEN);
            memcpy(&connected_ap_details.static_ip, &static_ip, sizeof(static_ip));
            connection_status = CY_WCM_EVENT_CONNECTED;
            if((res = cy_worker_thread_enqueue(&cy_wcm_worker_thread, notify_connection_status, &connection_status)) != CY_RSLT_SUCCESS)
            {
                cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "L%d : %s() : ERROR : Failed to send connection status. Err = [%lu]\r\n", __LINE__, __FUNCTION__, res);
                goto exit;
            }
            wcm_sta_link_up = true;
            /* post IP change callback */
            if((res = cy_worker_thread_enqueue(&cy_wcm_worker_thread, notify_ip_change, NULL)) != CY_RSLT_SUCCESS)
            {
                cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "L%d : %s() : Failed to notify IP change. Err = [%lu]\r\n", __LINE__, __FUNCTION__, res);
                goto exit;
            }
        }
    }
    else
    {
        res = CY_RSLT_WCM_WAIT_TIMEOUT;
        connection_status = CY_WCM_EVENT_CONNECT_FAILED;
        if(cy_worker_thread_enqueue(&cy_wcm_worker_thread, notify_connection_status, &connection_status) != CY_RSLT_SUCCESS)
        {
            cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "L%d : %s() : ERROR : Failed to send connection status. Err = [%lu]\r\n", __LINE__, __FUNCTION__, res);
        }
    }

exit:
    if (cy_rtos_set_mutex(&wcm_mutex) != CY_RSLT_SUCCESS)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Mutex release error \n");
        res = ((res != CY_RSLT_SUCCESS) ? res : CY_RSLT_WCM_MUTEX_ERROR);
    }
    return res;
}

cy_rslt_t cy_wcm_disconnect_ap()
{
    cy_rslt_t res = CY_RSLT_SUCCESS;

    if(!is_wcm_initalized)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "WCM is not initialized \n");
        return CY_RSLT_WCM_NOT_INITIALIZED;
    }

    if(cy_rtos_get_mutex(&wcm_mutex, CY_WCM_MAX_MUTEX_WAIT_TIME_MS) != CY_RSLT_SUCCESS)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Unable to acquire WCM mutex \n");
        return CY_RSLT_WCM_WAIT_TIMEOUT;
    }

    if (wcm_sta_link_up)
    {
        /* Deregister the link event handler */
        whd_wifi_deregister_event_handler(whd_ifs[CY_WCM_INTERFACE_TYPE_STA], sta_event_handler_index);
        network_down(whd_ifs[CY_WCM_INTERFACE_TYPE_STA], CY_LWIP_STA_NW_INTERFACE);
        res = whd_wifi_leave(whd_ifs[CY_WCM_INTERFACE_TYPE_STA]);
        if (res != CY_RSLT_SUCCESS)
        {
            goto exit;
        }
        wcm_sta_link_up = false;

        /* Call Offload deinit after disconnect to AP */
        if ( (is_olm_initialized == true) && (olm_instance != NULL) )
        {
           cy_olm_deinit_ols(olm_instance);
           is_olm_initialized = false;
        }
    }
exit:
    /* clear the saved ap credentials */
    memset(&connected_ap_details, 0, sizeof(connected_ap_details));
    if (cy_rtos_set_mutex(&wcm_mutex) != CY_RSLT_SUCCESS)
    {
        res = ((res != CY_RSLT_SUCCESS) ? res : CY_RSLT_WCM_MUTEX_ERROR);
    }
    return res;
}

#if LWIP_IPV4
cy_rslt_t cy_wcm_get_ip_addr(cy_wcm_interface_t interface_type, cy_wcm_ip_address_t *ip_addr, uint8_t addr_count)
{
    struct netif *net_interface  = NULL;
    uint32_t ipv4_addr;
    UNUSED_PARAMETER(addr_count);

#if LWIP_IPV4
    ip4_addr_t* addr = NULL;
    UNUSED_VARIABLE(addr);
#endif

    if(!is_wcm_initalized)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "WCM is not initialized \n");
        return CY_RSLT_WCM_NOT_INITIALIZED;
    }

    if((interface_type != CY_WCM_INTERFACE_TYPE_AP_STA) && (interface_type  != CY_WCM_INTERFACE_TYPE_STA) && (interface_type != CY_WCM_INTERFACE_TYPE_AP))
    {
        return CY_RSLT_WCM_INTERFACE_NOT_SUPPORTED;
    }

    if(ip_addr == NULL)
    {
        return CY_RSLT_WCM_BAD_ARG;
    }

    if((interface_type == CY_WCM_INTERFACE_TYPE_AP_STA) && (addr_count < MAX_WHD_INTERFACE))
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Error in size of IP address array \r\n");
        return CY_RSLT_WCM_BAD_ARG;
    }

    if(interface_type == CY_WCM_INTERFACE_TYPE_STA)
    {
        if(is_sta_network_up == false)
        {
            cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Network is not up, call cy_wcm_connect_ap API to bring network up \r\n");
            /** Network is not up and hence mem setting the ip address to zero**/
            memset(ip_addr, 0, sizeof(cy_wcm_ip_address_t));
            return CY_RSLT_WCM_STA_NETWORK_DOWN;
        }

        net_interface  = cy_lwip_get_interface(CY_LWIP_STA_NW_INTERFACE);
#if LWIP_IPV6
        ipv4_addr = net_interface->ip_addr.u_addr.ip4.addr;
        addr = &net_interface->ip_addr.u_addr.ip4;
#else
        ipv4_addr = net_interface->ip_addr.addr;
        addr = &net_interface->ip_addr;
#endif

        ip_addr->version = CY_WCM_IP_VER_V4;
        ip_addr->ip.v4 = ipv4_addr;
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "IP Address %s assigned \n", ip4addr_ntoa(addr));
        return CY_RSLT_SUCCESS;
    }
    else if(interface_type == CY_WCM_INTERFACE_TYPE_AP)
    {
        if(is_soft_ap_up == false)
        {
            cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Network is not up, call cy_wcm_start_ap API to bring up Soft AP \r\n");
            /** Network is not up and hence mem setting the ip address to zero**/
            memset(ip_addr, 0, sizeof(cy_wcm_ip_address_t));
            return CY_RSLT_WCM_AP_NOT_UP;
        }

        net_interface  = cy_lwip_get_interface(CY_LWIP_AP_NW_INTERFACE);
#if LWIP_IPV6
        ipv4_addr = net_interface->ip_addr.u_addr.ip4.addr;
        addr = &net_interface->ip_addr.u_addr.ip4;
#else
        ipv4_addr = net_interface->ip_addr.addr;
        addr = &net_interface->ip_addr;
#endif

        ip_addr->version = CY_WCM_IP_VER_V4;
        ip_addr->ip.v4 = ipv4_addr;
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "IP Address %s assigned \n", ip4addr_ntoa(addr));
        return CY_RSLT_SUCCESS;
    }
    else
    {
        /* fill the 0th index with STA IP address and 1st index with AP IP address */
        if(is_sta_network_up == false)
        {
            cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Network is not up, call cy_wcm_connect_ap API to bring network up \r\n");
            /** Network is not up and hence mem setting the ip address to zero**/
            memset(ip_addr, 0, sizeof(cy_wcm_ip_address_t));
        }

        net_interface  = cy_lwip_get_interface(CY_LWIP_STA_NW_INTERFACE);
#if LWIP_IPV6
        ipv4_addr = net_interface->ip_addr.u_addr.ip4.addr;
        addr = &net_interface->ip_addr.u_addr.ip4;
#else
        ipv4_addr = net_interface->ip_addr.addr;
        addr = &net_interface->ip_addr;
#endif
        ip_addr->version = CY_WCM_IP_VER_V4;
        ip_addr->ip.v4 = ipv4_addr;
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "STA IP Address %s assigned \n", ip4addr_ntoa(addr));

        ip_addr++;
        if(is_soft_ap_up == false)
        {
            cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Network is not up, call cy_wcm_connect_ap API to bring network up \r\n");
            /** Network is not up and hence mem setting the ip address to zero**/
            memset(ip_addr, 0, sizeof(cy_wcm_ip_address_t));
        }
        net_interface  = cy_lwip_get_interface(CY_LWIP_AP_NW_INTERFACE);
#if LWIP_IPV6
        ipv4_addr = net_interface->ip_addr.u_addr.ip4.addr;
        addr = &net_interface->ip_addr.u_addr.ip4;
#else
        ipv4_addr = net_interface->ip_addr.addr;
        addr = &net_interface->ip_addr;
#endif

        ip_addr->version = CY_WCM_IP_VER_V4;
        ip_addr->ip.v4 = ipv4_addr;
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "Soft AP IP Address %s assigned \n", ip4addr_ntoa(addr));
    }
    return CY_RSLT_SUCCESS;
}
#endif

#if LWIP_IPV6
cy_rslt_t cy_wcm_get_ipv6_addr(cy_wcm_interface_t interface_type, cy_wcm_ipv6_type_t ipv6_addr_type, cy_wcm_ip_address_t *ip_addr, uint8_t addr_count)
{
    struct netif *net_interface  = NULL;
    const ip6_addr_t* ipv6_addr  = NULL;

    if(!is_wcm_initalized)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "WCM is not initialized \n");
        return CY_RSLT_WCM_NOT_INITIALIZED;
    }

    if((interface_type != CY_WCM_INTERFACE_TYPE_AP_STA) && (interface_type  != CY_WCM_INTERFACE_TYPE_STA) && (interface_type != CY_WCM_INTERFACE_TYPE_AP))
    {
        return CY_RSLT_WCM_INTERFACE_NOT_SUPPORTED;
    }

    if(ip_addr == NULL)
    {
        return CY_RSLT_WCM_BAD_ARG;
    }

    if(ipv6_addr_type != CY_WCM_IPV6_LINK_LOCAL)
    {
        /*TODO : Need to add support for Global IPV6 address */
        return CY_RSLT_WCM_IPV6_GLOBAL_ADDRESS_NOT_SUPPORTED;
    }

    if((interface_type == CY_WCM_INTERFACE_TYPE_AP_STA) && (addr_count < MAX_WHD_INTERFACE))
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Error in size of IP address array \r\n");
        return CY_RSLT_WCM_BAD_ARG;
    }

    if(interface_type == CY_WCM_INTERFACE_TYPE_STA)
    {
        if(is_sta_network_up == false)
        {
            cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Network is not up, call cy_wcm_connect_ap API to bring network up \r\n");
            /** Network is not up and hence mem setting the ip address to zero**/
            memset(ip_addr, 0, sizeof(cy_wcm_ip_address_t));
            return CY_RSLT_WCM_STA_NETWORK_DOWN;
        }
        net_interface = cy_lwip_get_interface(CY_LWIP_STA_NW_INTERFACE);
#ifdef LWIP_IPV6
        ipv6_addr = netif_ip6_addr(net_interface, 0);
        if(ipv6_addr != NULL)
        {
            ip_addr->version = CY_WCM_IP_VER_V6;
            ip_addr->ip.v6[0] = ipv6_addr->addr[0];
            ip_addr->ip.v6[1] = ipv6_addr->addr[1];
            ip_addr->ip.v6[2] = ipv6_addr->addr[2];
            ip_addr->ip.v6[3] = ipv6_addr->addr[3];

            cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "IPV6 Address %s assigned \n", ip6addr_ntoa(netif_ip6_addr(net_interface, 0)));
        }
        else
        {
            memset(ip_addr, 0, sizeof(cy_wcm_ip_address_t));
            cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "IPV6 network not ready \n");
            return CY_RSLT_WCM_IPV6_INTERFACE_NOT_READY;
        }
#else
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERROR, "LWIP_IPV6 flag is not enabled \r\n");
        return CY_RSLT_WCM_BAD_ARG;
#endif
    }
    else if(interface_type == CY_WCM_INTERFACE_TYPE_AP)
    {
        if(is_soft_ap_up == false)
        {
            cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Network is not up, call cy_wcm_start_ap API to start SoftAP \r\n");
            /** Network is not up and hence mem setting the ip address to zero**/
            memset(ip_addr, 0, sizeof(cy_wcm_ip_address_t));
            return CY_RSLT_WCM_AP_NOT_UP;
        }
        net_interface = cy_lwip_get_interface(CY_LWIP_AP_NW_INTERFACE);
#ifdef LWIP_IPV6
        ipv6_addr = netif_ip6_addr(net_interface, 0);
        if(ipv6_addr != NULL)
        {
            ip_addr->version = CY_WCM_IP_VER_V6;
            ip_addr->ip.v6[0] = ipv6_addr->addr[0];
            ip_addr->ip.v6[1] = ipv6_addr->addr[1];
            ip_addr->ip.v6[2] = ipv6_addr->addr[2];
            ip_addr->ip.v6[3] = ipv6_addr->addr[3];
            cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "IPV6 Address %s assigned \n", ip6addr_ntoa(netif_ip6_addr(net_interface, 0)));
        }
        else
        {
            memset(ip_addr, 0, sizeof(cy_wcm_ip_address_t));
            cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "IPV6 network not ready \n");
            return CY_RSLT_WCM_IPV6_INTERFACE_NOT_READY;
        }
#else
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERROR, "LWIP_IPV6 flag is not enabled \r\n");
        return CY_RSLT_WCM_BAD_ARG;
#endif
    }
    else
    {
        /* fill the 0th index with STA IP address and 1st index with AP IP address */
        if(is_sta_network_up == false)
        {
            cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Network is not up, call cy_wcm_connect_ap API to bring network up \r\n");
            /** Network is not up and hence mem setting the ip address to zero**/
            memset(ip_addr, 0, sizeof(cy_wcm_ip_address_t));
        }
        net_interface = cy_lwip_get_interface(CY_LWIP_STA_NW_INTERFACE);
#ifdef LWIP_IPV6
        ipv6_addr = netif_ip6_addr(net_interface, 0);
        if(ipv6_addr != NULL)
        {
            ip_addr->version = CY_WCM_IP_VER_V6;
            ip_addr->ip.v6[0] = ipv6_addr->addr[0];
            ip_addr->ip.v6[1] = ipv6_addr->addr[1];
            ip_addr->ip.v6[2] = ipv6_addr->addr[2];
            ip_addr->ip.v6[3] = ipv6_addr->addr[3];

            cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "IPV6 Address %s assigned \n", ip6addr_ntoa(netif_ip6_addr(net_interface, 0)));
        }
        else
        {
            memset(ip_addr, 0, sizeof(cy_wcm_ip_address_t));
            cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "IPV6 network not ready \n");
        }
#else
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERROR, "LWIP_IPV6 flag is not enabled \r\n");
#endif
        ip_addr++;
        if(is_soft_ap_up == false)
        {
            cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Network is not up, call cy_wcm_start_ap API to start SoftAP \r\n");
            /** Network is not up and hence mem setting the ip address to zero**/
            memset(ip_addr, 0, sizeof(cy_wcm_ip_address_t));
        }
        net_interface = cy_lwip_get_interface(CY_LWIP_AP_NW_INTERFACE);
#ifdef LWIP_IPV6
        ipv6_addr = netif_ip6_addr(net_interface, 0);
        if(ipv6_addr != NULL)
        {
            ip_addr->version = CY_WCM_IP_VER_V6;
            ip_addr->ip.v6[0] = ipv6_addr->addr[0];
            ip_addr->ip.v6[1] = ipv6_addr->addr[1];
            ip_addr->ip.v6[2] = ipv6_addr->addr[2];
            ip_addr->ip.v6[3] = ipv6_addr->addr[3];
            cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "IPV6 Address %s assigned \n", ip6addr_ntoa(netif_ip6_addr(net_interface, 0)));
        }
        else
        {
            memset(ip_addr, 0, sizeof(cy_wcm_ip_address_t));
            cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "IPV6 network not ready \n");
        }
#else
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERROR, "LWIP_IPV6 flag is not enabled \r\n");
#endif
    }

    return CY_RSLT_SUCCESS;
}
#endif

cy_rslt_t cy_wcm_get_mac_addr(cy_wcm_interface_t interface_type, cy_wcm_mac_t *mac_addr, uint8_t addr_count)
{
    cy_rslt_t res = CY_RSLT_SUCCESS;
    whd_mac_t mac;

    if(!is_wcm_initalized)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "WCM is not initialized, call cy_wcm_init() \n");
        return CY_RSLT_WCM_NOT_INITIALIZED;
    }

    if((interface_type != CY_WCM_INTERFACE_TYPE_AP_STA) && (interface_type  != CY_WCM_INTERFACE_TYPE_STA) && (interface_type != CY_WCM_INTERFACE_TYPE_AP))
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Interface not supported \r\n");
        return CY_RSLT_WCM_INTERFACE_NOT_SUPPORTED;
    }

    if(mac_addr == NULL)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Bad argument \r\n");
        return CY_RSLT_WCM_BAD_ARG;
    }

    if((interface_type == CY_WCM_INTERFACE_TYPE_AP_STA) && (addr_count < MAX_WHD_INTERFACE))
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Error in size of mac address array \r\n");
        return CY_RSLT_WCM_BAD_ARG;
    }

    if(cy_rtos_get_mutex(&wcm_mutex, CY_WCM_MAX_MUTEX_WAIT_TIME_MS) != CY_RSLT_SUCCESS)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Unable to acquire WCM mutex \n");
        return CY_RSLT_WCM_WAIT_TIMEOUT;
    }

    if(((current_interface == CY_WCM_INTERFACE_TYPE_AP) && (interface_type == CY_WCM_INTERFACE_TYPE_STA)))
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Error : Interface is not up \r\n");
        res = CY_RSLT_WCM_INTERFACE_NOT_UP;
        memset(mac_addr, 0, sizeof(mac));
        goto exit;
    }

    if((current_interface == CY_WCM_INTERFACE_TYPE_STA) && (interface_type == CY_WCM_INTERFACE_TYPE_AP))
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Error : Interface is not up \r\n");
        res = CY_RSLT_WCM_INTERFACE_NOT_UP;
        memset(mac_addr, 0, sizeof(mac));
        goto exit;
    }

    memset(&mac, 0, sizeof(whd_mac_t));
    if((interface_type == CY_WCM_INTERFACE_TYPE_STA) || (interface_type == CY_WCM_INTERFACE_TYPE_AP))
    {
        if((res = whd_wifi_get_mac_address(whd_ifs[interface_type], &mac) != CY_RSLT_SUCCESS))
        {
            cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Error in getting the mac address of the interface \r\n");
            memset(mac_addr, 0, sizeof(mac));
            goto exit;
        }
        memcpy(mac_addr, &mac, sizeof(mac));
    }
    else
    {
        memset(mac_addr, 0, (addr_count * sizeof(mac)));
        if((res = whd_wifi_get_mac_address(whd_ifs[CY_WCM_INTERFACE_TYPE_STA], &mac)) != CY_RSLT_SUCCESS)
        {
            cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Error in getting the mac address of STA interface \r\n");
            goto exit;
        }
        else
        {
            memcpy(mac_addr, &mac, sizeof(mac));
        }

        mac_addr++;
        memset(&mac, 0, sizeof(whd_mac_t));

        if((res = whd_wifi_get_mac_address(whd_ifs[CY_WCM_INTERFACE_TYPE_AP], &mac)) != CY_RSLT_SUCCESS)
        {
            cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Error in getting the mac address of AP interface \r\n");
        }
        else
        {
            memcpy(mac_addr, &mac, sizeof(mac));
        }
    }
 exit:
    if(cy_rtos_set_mutex(&wcm_mutex) != CY_RSLT_SUCCESS)
    {
        res = ((res != CY_RSLT_SUCCESS) ? res : CY_RSLT_WCM_MUTEX_ERROR);
    }
    return res;
}

#if LWIP_IPV4
cy_rslt_t cy_wcm_get_gateway_ip_address(cy_wcm_interface_t interface_type, cy_wcm_ip_address_t *gateway_addr, uint8_t addr_count)
{
    struct netif *net_interface  = NULL;
    uint32_t ipv4_addr;
    UNUSED_PARAMETER(addr_count);

#if LWIP_IPV4
    ip4_addr_t* addr = NULL;
    UNUSED_VARIABLE(addr);
#endif

    if(!is_wcm_initalized)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "WCM is not initialized \n");
        return CY_RSLT_WCM_NOT_INITIALIZED;
    }

    if((interface_type != CY_WCM_INTERFACE_TYPE_AP_STA) && (interface_type  != CY_WCM_INTERFACE_TYPE_STA) && (interface_type != CY_WCM_INTERFACE_TYPE_AP))
    {
        return CY_RSLT_WCM_INTERFACE_NOT_SUPPORTED;
    }

    if(gateway_addr == NULL)
    {
        return CY_RSLT_WCM_BAD_ARG;
    }

    if((interface_type == CY_WCM_INTERFACE_TYPE_AP_STA) && (addr_count < MAX_WHD_INTERFACE))
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Error in size of IP address array \r\n");
        return CY_RSLT_WCM_BAD_ARG;
    }

    if(interface_type == CY_WCM_INTERFACE_TYPE_STA)
    {
        if(is_sta_network_up == false)
        {
            cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Network is not up, call cy_wcm_connect_ap API to bring network up \r\n");
            /** Network is not up and hence mem setting the ip address to zero**/
            memset(gateway_addr, 0, sizeof(cy_wcm_ip_address_t));
            return CY_RSLT_WCM_STA_NETWORK_DOWN;
        }

        net_interface  = cy_lwip_get_interface(CY_LWIP_STA_NW_INTERFACE);
#if LWIP_IPV6
        ipv4_addr = net_interface->gw.u_addr.ip4.addr;
        addr = &net_interface->gw.u_addr.ip4;
#else
        ipv4_addr = net_interface->gw.addr;
        addr = &net_interface->gw;
#endif

        gateway_addr->version = CY_WCM_IP_VER_V4;
        gateway_addr->ip.v4 = ipv4_addr;
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "Gateway IP Address %s assigned \n", ip4addr_ntoa(addr));
        return CY_RSLT_SUCCESS;
    }
    else if(interface_type == CY_WCM_INTERFACE_TYPE_AP)
    {
        if(is_soft_ap_up == false)
        {
            cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Network is not up, call cy_wcm_connect_ap API to bring network up \r\n");
            /** Network is not up and hence mem setting the ip address to zero**/
            memset(gateway_addr, 0, sizeof(cy_wcm_ip_address_t));
            return CY_RSLT_WCM_STA_NETWORK_DOWN;
        }

        net_interface  = cy_lwip_get_interface(CY_LWIP_AP_NW_INTERFACE);
#if LWIP_IPV6
        ipv4_addr = net_interface->gw.u_addr.ip4.addr;
        addr = &net_interface->gw.u_addr.ip4;
#else
        ipv4_addr = net_interface->gw.addr;
        addr = &net_interface->gw;
#endif

        gateway_addr->version = CY_WCM_IP_VER_V4;
        gateway_addr->ip.v4 = ipv4_addr;
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "Gateway IP Address %s assigned \n", ip4addr_ntoa(addr));
        return CY_RSLT_SUCCESS;
    }
    else
    {
        /* fill the 0th index with STA IP address and 1st index with AP IP address */
        if(is_sta_network_up == false)
        {
            cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Network is not up, call cy_wcm_connect_ap API to bring network up \r\n");
            /** Network is not up and hence mem setting the ip address to zero**/
            memset(gateway_addr, 0, sizeof(cy_wcm_ip_address_t));
        }

        net_interface  = cy_lwip_get_interface(CY_LWIP_STA_NW_INTERFACE);
#if LWIP_IPV6
        ipv4_addr = net_interface->gw.u_addr.ip4.addr;
        addr = &net_interface->gw.u_addr.ip4;
#else
        ipv4_addr = net_interface->gw.addr;
        addr = &net_interface->gw;
#endif
        gateway_addr->version = CY_WCM_IP_VER_V4;
        gateway_addr->ip.v4 = ipv4_addr;
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "STA Gateway IP Address %s assigned \n", ip4addr_ntoa(addr));

        gateway_addr++;
        if(is_soft_ap_up == false)
        {
            cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Network is not up, call cy_wcm_connect_ap API to bring network up \r\n");
            /** Network is not up and hence mem setting the ip address to zero**/
            memset(gateway_addr, 0, sizeof(cy_wcm_ip_address_t));
        }
        net_interface  = cy_lwip_get_interface(CY_LWIP_AP_NW_INTERFACE);
#if LWIP_IPV6
        ipv4_addr = net_interface->gw.u_addr.ip4.addr;
        addr = &net_interface->gw.u_addr.ip4;
#else
        ipv4_addr = net_interface->gw.addr;
        addr = &net_interface->gw;
#endif

        gateway_addr->version = CY_WCM_IP_VER_V4;
        gateway_addr->ip.v4 = ipv4_addr;
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "Soft AP IP Address %s assigned \n", ip4addr_ntoa(addr));
    }
    return CY_RSLT_SUCCESS;
}

cy_rslt_t cy_wcm_get_ip_netmask(cy_wcm_interface_t interface_type, cy_wcm_ip_address_t *net_mask_addr, uint8_t addr_count)
{
    struct netif *net_interface  = NULL;
    uint32_t ipv4_addr;
    UNUSED_PARAMETER(addr_count);

#if LWIP_IPV4
    ip4_addr_t* addr = NULL;
    UNUSED_VARIABLE(addr);
#endif

    if(!is_wcm_initalized)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "WCM is not initialized \n");
        return CY_RSLT_WCM_NOT_INITIALIZED;
    }

    if((interface_type != CY_WCM_INTERFACE_TYPE_AP_STA) && (interface_type  != CY_WCM_INTERFACE_TYPE_STA) && (interface_type != CY_WCM_INTERFACE_TYPE_AP))
    {
        return CY_RSLT_WCM_INTERFACE_NOT_SUPPORTED;
    }

    if(net_mask_addr == NULL)
    {
        return CY_RSLT_WCM_BAD_ARG;
    }

    if((interface_type == CY_WCM_INTERFACE_TYPE_AP_STA) && (addr_count < MAX_WHD_INTERFACE))
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Error in size of IP address array \r\n");
        return CY_RSLT_WCM_BAD_ARG;
    }

    if(interface_type == CY_WCM_INTERFACE_TYPE_STA)
    {
        if(is_sta_network_up == false)
        {
            cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Network is not up, call cy_wcm_connect_ap API to bring network up \r\n");
            /** Network is not up and hence mem setting the ip address to zero**/
            memset(net_mask_addr, 0, sizeof(cy_wcm_ip_address_t));
            return CY_RSLT_WCM_STA_NETWORK_DOWN;
        }

        net_interface  = cy_lwip_get_interface(CY_LWIP_STA_NW_INTERFACE);
#if LWIP_IPV6
        ipv4_addr = net_interface->netmask.u_addr.ip4.addr;
        addr = &net_interface->netmask.u_addr.ip4;
#else
        ipv4_addr = net_interface->netmask.addr;
        addr = &net_interface->netmask;
#endif

        net_mask_addr->version = CY_WCM_IP_VER_V4;
        net_mask_addr->ip.v4 = ipv4_addr;
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "net mask IP Address %s assigned \n", ip4addr_ntoa(addr));
        return CY_RSLT_SUCCESS;
    }
    else if(interface_type == CY_WCM_INTERFACE_TYPE_AP)
    {
        if(is_soft_ap_up == false)
        {
            cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Network is not up, call cy_wcm_connect_ap API to bring network up \r\n");
            /** Network is not up and hence mem setting the ip address to zero**/
            memset(net_mask_addr, 0, sizeof(cy_wcm_ip_address_t));
            return CY_RSLT_WCM_STA_NETWORK_DOWN;
        }

        net_interface  = cy_lwip_get_interface(CY_LWIP_AP_NW_INTERFACE);
#if LWIP_IPV6
        ipv4_addr = net_interface->netmask.u_addr.ip4.addr;
        addr = &net_interface->netmask.u_addr.ip4;
#else
        ipv4_addr = net_interface->netmask.addr;
        addr = &net_interface->netmask;
#endif

        net_mask_addr->version = CY_WCM_IP_VER_V4;
        net_mask_addr->ip.v4 = ipv4_addr;
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "net mask Address %s assigned \n", ip4addr_ntoa(addr));
        return CY_RSLT_SUCCESS;
    }
    else
    {
        /* fill the 0th index with STA IP address and 1st index with AP IP address */
        if(is_sta_network_up == false)
        {
            cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Network is not up, call cy_wcm_connect_ap API to bring network up \r\n");
            /** Network is not up and hence mem setting the ip address to zero**/
            memset(net_mask_addr, 0, sizeof(cy_wcm_ip_address_t));
        }

        net_interface  = cy_lwip_get_interface(CY_LWIP_STA_NW_INTERFACE);
#if LWIP_IPV6
        ipv4_addr = net_interface->netmask.u_addr.ip4.addr;
        addr = &net_interface->netmask.u_addr.ip4;
#else
        ipv4_addr = net_interface->netmask.addr;
        addr = &net_interface->netmask;
#endif
        net_mask_addr->version = CY_WCM_IP_VER_V4;
        net_mask_addr->ip.v4 = ipv4_addr;
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "STA Gateway IP Address %s assigned \n", ip4addr_ntoa(addr));

        net_mask_addr++;
        if(is_soft_ap_up == false)
        {
            cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Network is not up, call cy_wcm_connect_ap API to bring network up \r\n");
            /** Network is not up and hence mem setting the ip address to zero**/
            memset(net_mask_addr, 0, sizeof(cy_wcm_ip_address_t));
        }
        net_interface  = cy_lwip_get_interface(CY_LWIP_AP_NW_INTERFACE);
#if LWIP_IPV6
        ipv4_addr = net_interface->netmask.u_addr.ip4.addr;
        addr = &net_interface->netmask.u_addr.ip4;
#else
        ipv4_addr = net_interface->netmask.addr;
        addr = &net_interface->netmask;
#endif

        net_mask_addr->version = CY_WCM_IP_VER_V4;
        net_mask_addr->ip.v4 = ipv4_addr;
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "Soft AP net mask Address %s assigned \n", ip4addr_ntoa(addr));
    }
    return CY_RSLT_SUCCESS;
}
#endif

uint8_t cy_wcm_is_connected_to_ap(void)
{
    return (wcm_sta_link_up ? 1 : 0);
}

uint8_t cy_wcm_is_ap_up(void)
{
    return (is_soft_ap_up ? 1 : 0);
}

cy_rslt_t cy_wcm_get_associated_ap_info(cy_wcm_associated_ap_info_t *ap_info)
{
    cy_rslt_t res = CY_RSLT_SUCCESS;
    wl_bss_info_t  bss_info;
    whd_security_t security;

    if(!is_wcm_initalized)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "WCM is not initialized \n");
        return CY_RSLT_WCM_NOT_INITIALIZED;
    }

    if(ap_info == NULL)
    {
        return CY_RSLT_WCM_BAD_ARG;
    }

    if(!wcm_sta_link_up)
    {
        return CY_RSLT_WCM_STA_NETWORK_DOWN;
    }

    if(whd_wifi_get_ap_info(whd_ifs[CY_WCM_INTERFACE_TYPE_STA], &bss_info, &security) == CY_RSLT_SUCCESS)
    {
        memcpy(ap_info->BSSID, bss_info.BSSID.octet, CY_WCM_MAC_ADDR_LEN);
        memcpy(ap_info->SSID, bss_info.SSID, bss_info.SSID_len);
        ap_info->SSID[bss_info.SSID_len] = 0;
        ap_info->security = whd_to_wcm_security(security);
        ap_info->channel = bss_info.ctl_ch;
        ap_info->signal_strength = bss_info.RSSI;
        ap_info->channel_width = channel_to_bandwidth(bss_info.chanspec);
        if(ap_info->channel_width == UNKNOWN_BAND_WIDTH)
        {
            res = CY_RSLT_WCM_CHANNEL_ERROR;
        }
    }

    return res;
}

cy_rslt_t cy_wcm_get_wlan_statistics(cy_wcm_interface_t interface, cy_wcm_wlan_statistics_t *stat)
{
    whd_buffer_t buffer;
    whd_buffer_t response;
    uint32_t data_rate;
    wl_cnt_ver_ten_t* received_counters;

    if(!is_wcm_initalized)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "WCM is not initialized, to initialize call cy_wcm_init()\r\n");
        return CY_RSLT_WCM_NOT_INITIALIZED;
    }

    if(interface != CY_WCM_INTERFACE_TYPE_STA)
    {
        return CY_RSLT_WCM_INTERFACE_NOT_SUPPORTED;
    }

    if(stat == NULL)
    {
        return CY_RSLT_WCM_BAD_ARG;
    }

    CHECK_IOCTL_BUFFER(whd_cdc_get_iovar_buffer(whd_ifs[CY_WCM_INTERFACE_TYPE_STA]->whd_driver, &buffer, sizeof(wl_cnt_ver_ten_t), IOVAR_STR_COUNTERS));

    CHECK_RETURN(whd_cdc_send_iovar(whd_ifs[CY_WCM_INTERFACE_TYPE_STA], CDC_GET, buffer, &response));

    received_counters = (wl_cnt_ver_ten_t*) whd_buffer_get_current_piece_data_pointer(whd_ifs[CY_WCM_INTERFACE_TYPE_STA]->whd_driver, response);

    /* Copy the required statistics */
    stat->rx_bytes   = received_counters->rxbyte;
    stat->tx_bytes   = received_counters->txbyte;
    stat->rx_packets = received_counters->rxfrag;
    stat->tx_packets = received_counters->txfrag;
    stat->tx_failed  = received_counters->txfail;
    stat->tx_retries = received_counters->txretry;

    whd_buffer_release(whd_ifs[CY_WCM_INTERFACE_TYPE_STA]->whd_driver, response, WHD_NETWORK_RX);

    /* get data rate */
    CHECK_RETURN (whd_wifi_get_ioctl_value(whd_ifs[CY_WCM_INTERFACE_TYPE_STA], WLC_GET_RATE, &data_rate));
    /* The data rate received is in units of 500Kbits per sec, convert to Kbits per sec*/
    stat->tx_bitrate = (data_rate * TX_BIT_RATE_CONVERTER);
    return CY_RSLT_SUCCESS;
}

#if LWIP_IPV4
cy_rslt_t cy_wcm_get_gateway_mac_address(cy_wcm_mac_t *mac_addr)
{
    err_t err;
    cy_wcm_ip_address_t gateway_ip_addr;
    struct eth_addr *eth_ret = NULL;
    const ip4_addr_t *ip_ret = NULL;
    int32_t arp_waittime = ARP_WAIT_TIME_IN_MSEC;
    ssize_t arp_index = -1;
    ip4_addr_t ipv4addr;
    struct netif *net_interface  = NULL;

    if(!is_wcm_initalized)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "WCM is not initialized, to initialize call cy_wcm_init()\r\n");
        return CY_RSLT_WCM_NOT_INITIALIZED;
    }

    if(mac_addr == NULL)
    {
        return CY_RSLT_WCM_BAD_ARG;
    }

    if(current_interface == CY_WCM_INTERFACE_TYPE_AP)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "feature not supported for AP interface \r\n");
        return CY_RSLT_WCM_INTERFACE_NOT_SUPPORTED;
    }

    net_interface = cy_lwip_get_interface(CY_LWIP_STA_NW_INTERFACE);
    if(net_interface == NULL)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "network interface is NULL \r\n");
        return CY_RSLT_WCM_NETIF_DOES_NOT_EXIST;
    }

    cy_wcm_get_gateway_ip_address(CY_WCM_INTERFACE_TYPE_STA, &gateway_ip_addr, 1);

    ipv4addr.addr = gateway_ip_addr.ip.v4;

    /* Check if entry of the gateway address is already present in the ARP cache. */
    arp_index = etharp_find_addr(net_interface, (const ip4_addr_t *) &ipv4addr, &eth_ret, (const ip4_addr_t **) &ip_ret);

    if(arp_index == -1)
    {
        /* Entry for the address is not present in the ARP cache. Sent ARP request.*/
        err = etharp_request(net_interface, (const ip4_addr_t *) &ipv4addr);
        if(err != ERR_OK)
        {
            cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "etharp_request failed with error %d\n", err);
            return  CY_RSLT_WCM_ARP_REQUEST_FAILURE;
        }

        do
        {
            arp_index = etharp_find_addr(net_interface, (const ip4_addr_t *) &ipv4addr, &eth_ret, (const ip4_addr_t **) &ip_ret);
            if(arp_index != -1)
            {
                cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "arp entry found \r\n");
                break;
            }
            cy_rtos_delay_milliseconds(ARP_CACHE_CHECK_INTERVAL_IN_MSEC);
            arp_waittime -= ARP_CACHE_CHECK_INTERVAL_IN_MSEC;
            if(arp_waittime <= 0)
            {
                cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "Could not resolve MAC address for the given destination address \r\n");
                return CY_RSLT_WCM_WAIT_TIMEOUT;
            }
        } while(1);
    }
    memcpy(mac_addr, eth_ret->addr, CY_WCM_MAC_ADDR_LEN);
    return CY_RSLT_SUCCESS;
}

cy_rslt_t cy_wcm_ping(cy_wcm_interface_t interface, cy_wcm_ip_address_t* address, uint32_t timeout_ms, uint32_t* elapsed_time_ms)
{
    cy_time_t send_time;
    cy_time_t recvd_time;
    err_t err;
    struct timeval timeout_val;
    struct icmp_packet ping_packet;
    uint16_t ping_seq_num = 0;
    int socket_for_ping = -1;
    struct netif *net_interface;
    char if_name[PING_IF_NAME_LEN];
    struct ifreq iface;
    cy_lwip_nw_interface_role_t role;
    cy_rslt_t res = CY_RSLT_SUCCESS;

    if(!is_wcm_initalized)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "WCM is not initialized \n");
        return CY_RSLT_WCM_NOT_INITIALIZED;
    }

    if((interface != CY_WCM_INTERFACE_TYPE_STA) && (interface != CY_WCM_INTERFACE_TYPE_AP))
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "interface not supported \r\n");
        return CY_RSLT_WCM_INTERFACE_NOT_SUPPORTED;
    }

    if((address == NULL) || (elapsed_time_ms == NULL))
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "BAD arguments \r\n");
        return CY_RSLT_WCM_BAD_ARG;
    }

    if((interface == CY_WCM_INTERFACE_TYPE_STA) && (wcm_sta_link_up == false))
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "STA network down \r\n");
        return CY_RSLT_WCM_STA_NETWORK_DOWN;
    }

    if((interface == CY_WCM_INTERFACE_TYPE_AP) && (is_soft_ap_up == false))
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "AP not up \r\n");
        return CY_RSLT_WCM_AP_NOT_UP;
    }

    if(cy_rtos_get_mutex(&wcm_mutex, CY_WCM_MAX_MUTEX_WAIT_TIME_MS) != CY_RSLT_SUCCESS)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Unable to acquire WCM mutex \n");
        return CY_RSLT_WCM_WAIT_TIMEOUT;
    }

    /* Open a local socket for pinging */
    socket_for_ping = lwip_socket(AF_INET, SOCK_RAW, IP_PROTO_ICMP);
    if (socket_for_ping < 0)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Unable to create a socket for Ping \n");
        res = CY_RSLT_WCM_PING_FAILURE;
        goto exit;
    }

    /* convert the timeout into struct timeval */
    timeout_val.tv_sec  = (long)(timeout_ms / 1000);
    timeout_val.tv_usec = (long)((timeout_ms % 1000) * 1000);
    /* Set the receive timeout on local socket so ping will time out. */
    if(lwip_setsockopt(socket_for_ping, SOL_SOCKET, SO_RCVTIMEO, &timeout_val, sizeof(struct timeval)) != ERR_OK)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Error while setting socket for Ping \n");
        res = CY_RSLT_WCM_PING_FAILURE;
        goto exit;
    }

    role = ((interface == CY_WCM_INTERFACE_TYPE_STA) ? CY_LWIP_STA_NW_INTERFACE : CY_LWIP_AP_NW_INTERFACE);
    /* Bind interface to device. */
    net_interface = cy_lwip_get_interface(role);
    memset(&iface, 0, sizeof(iface));
    memcpy(if_name, net_interface->name, sizeof(net_interface->name));
    snprintf(&if_name[2], (PING_IF_NAME_LEN - 2), "%u", (uint8_t)(net_interface->num));
    memcpy(iface.ifr_name, if_name, PING_IF_NAME_LEN);
    if(lwip_setsockopt(socket_for_ping, SOL_SOCKET, SO_BINDTODEVICE, &iface, sizeof(iface)) != ERR_OK)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Error while binding socket to interface \n");
        res = CY_RSLT_WCM_PING_FAILURE;
        goto exit;
    }

    /* Send a ping */
    err = ping_send(socket_for_ping, address, &ping_packet, &ping_seq_num);
    if (err != ERR_OK)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Error while sending Ping request \n");
        res = CY_RSLT_WCM_PING_FAILURE;
        goto exit;
    }
    /* Record time ping was sent */
    cy_rtos_get_time(&send_time);

    /* Wait for ping reply */
    err = ping_recv(socket_for_ping, address, &ping_seq_num);
    if (err != ERR_OK)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Ping response timed out \n");
        res = CY_RSLT_WCM_PING_REQUEST_TIMEOUT;
        goto exit;
    }

    /* compute the elapsed time since a ping request was initiated */
    cy_rtos_get_time(&recvd_time);
    *elapsed_time_ms = (uint32_t)(recvd_time - send_time);

exit :
    /* close the socket */
    if(socket_for_ping >= 0)
    {
        lwip_close(socket_for_ping);
    }

    if(cy_rtos_set_mutex(&wcm_mutex) != CY_RSLT_SUCCESS)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Failed to release the mutex \n");
        res = ((res != CY_RSLT_SUCCESS) ? res : CY_RSLT_WCM_MUTEX_ERROR);
    }

    return res;
}

cy_rslt_t cy_wcm_start_ap(const cy_wcm_ap_config_t *ap_config)
{
    cy_rslt_t res = CY_RSLT_SUCCESS;
    whd_ssid_t ssid;
    uint8_t *key;
    uint8_t keylen;
    whd_security_t security;
    ip_static_addr_t static_ip;

    if(!is_wcm_initalized)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "WCM not initialized, call cy_wcm_init() \n");
        return CY_RSLT_WCM_NOT_INITIALIZED;
    }

    if((res = check_soft_ap_config(ap_config)) != CY_RSLT_SUCCESS)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Bad soft ap config \n");
        return res;
    }

    if(cy_rtos_get_mutex(&wcm_mutex, CY_WCM_MAX_MUTEX_WAIT_TIME_MS) != CY_RSLT_SUCCESS)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Unable to acquire WCM mutex \n");
        return CY_RSLT_WCM_WAIT_TIMEOUT;
    }

    if(is_soft_ap_up)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "AP is already Up !!\n");
        goto exit;
    }

    read_ap_config(ap_config, &ssid, &key, &keylen, &security, &static_ip);
    /* set up the AP info */
    res = whd_wifi_init_ap(whd_ifs[CY_WCM_INTERFACE_TYPE_AP], &ssid, security, (const uint8_t *)key,
                           keylen, ap_config->channel);
    if (res != CY_RSLT_SUCCESS)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "ERROR initializing AP = %d \n", res);
        goto exit;
    }

    if (ap_config->ie_info)
    {
        res = whd_wifi_manage_custom_ie(whd_ifs[CY_WCM_INTERFACE_TYPE_AP], WHD_ADD_CUSTOM_IE, (const uint8_t *)ap_config->ie_info->oui,
                                        ap_config->ie_info->subtype, (const void *)ap_config->ie_info->data, ap_config->ie_info->length, ap_config->ie_info->ie_packet_mask);
        if (res != CY_RSLT_SUCCESS)
        {
            cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "ERROR while adding Custom IE = %d\n", res);
            goto exit;
        }
    }

    res = whd_wifi_start_ap(whd_ifs[CY_WCM_INTERFACE_TYPE_AP]);
    if (res != CY_RSLT_SUCCESS)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "unable to start AP \n");
        goto exit;
    }

    if((res = network_up(whd_ifs[CY_WCM_INTERFACE_TYPE_AP], CY_LWIP_AP_NW_INTERFACE, &static_ip)) != CY_RSLT_SUCCESS)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Failed to bring up the network stack\n");
        res = CY_RSLT_WCM_STA_NETWORK_DOWN;
        /* stop the AP, stopping of AP will remove custom IE */
        whd_wifi_stop_ap(whd_ifs[CY_WCM_INTERFACE_TYPE_AP]);
        goto exit;
    }

    /* Register for AP Link events*/
    if((res = whd_management_set_event_handler(whd_ifs[CY_WCM_INTERFACE_TYPE_AP], ap_link_events, ap_link_events_handler, NULL, &ap_event_handler_index)) != CY_RSLT_SUCCESS)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Failed to set the Event handler \n");
        network_down(whd_ifs[CY_WCM_INTERFACE_TYPE_AP], CY_LWIP_AP_NW_INTERFACE);
        whd_wifi_stop_ap(whd_ifs[CY_WCM_INTERFACE_TYPE_AP]);
        goto exit;
    }
    is_soft_ap_up = true;

exit:
    if (cy_rtos_set_mutex(&wcm_mutex) != CY_RSLT_SUCCESS)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Failed to release the mutex \n");
        res = ((res != CY_RSLT_SUCCESS) ? res : CY_RSLT_WCM_MUTEX_ERROR);
    }
    return res;
}

cy_rslt_t cy_wcm_stop_ap(void)
{
    cy_rslt_t res = CY_RSLT_SUCCESS;

    if(!is_wcm_initalized)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "WCM not initialized, call cy_wcm_init() \n");
        return CY_RSLT_WCM_NOT_INITIALIZED;
    }

    if(cy_rtos_get_mutex(&wcm_mutex, CY_WCM_MAX_MUTEX_WAIT_TIME_MS) != CY_RSLT_SUCCESS)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Unable to acquire WCM mutex \n");
        return CY_RSLT_WCM_WAIT_TIMEOUT;
    }

    if(is_soft_ap_up)
    {
        network_down(whd_ifs[CY_WCM_INTERFACE_TYPE_AP], CY_LWIP_AP_NW_INTERFACE);
        /* De-register of AP link event handler */
        whd_wifi_deregister_event_handler(whd_ifs[CY_WCM_INTERFACE_TYPE_AP], ap_event_handler_index);
        res = whd_wifi_stop_ap(whd_ifs[CY_WCM_INTERFACE_TYPE_AP]);
        if (res != CY_RSLT_SUCCESS)
        {
            cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "WHD stop ap failed \n");
            goto exit;
        }
        is_soft_ap_up = false;
    }
exit:
    if (cy_rtos_set_mutex(&wcm_mutex) != CY_RSLT_SUCCESS)
    {
        res = ((res != CY_RSLT_SUCCESS) ? res : CY_RSLT_WCM_MUTEX_ERROR);
    }
    return res;
}

static void ping_prepare_echo(struct icmp_packet *iecho, uint16_t len, uint16_t *ping_seq_num)
{
    int i;
    ICMPH_TYPE_SET(&iecho->hdr, ICMP_ECHO);
    ICMPH_CODE_SET(&iecho->hdr, 0);
    iecho->hdr.chksum = 0;
    iecho->hdr.id = PING_ID;
    iecho->hdr.seqno = htons(++(*ping_seq_num));

    /* fill the additional data buffer with some data */
    for ( i = 0; i < (int)sizeof(iecho->data); i++ )
    {
        iecho->data[i] = (uint8_t)i;
    }

    iecho->hdr.chksum = inet_chksum(iecho, len);
}

static err_t ping_send(int socket_hnd, const cy_wcm_ip_address_t* address, struct icmp_packet *iecho, uint16_t *sequence_number)
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

static err_t ping_recv(int socket_hnd, cy_wcm_ip_address_t* address, uint16_t *ping_seq_num)
{
    char                  buf[PING_RESPONSE_LEN];
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

            if ((iecho->id == PING_ID) &&
                 (iecho->seqno == htons(*ping_seq_num)) &&
                 (ICMPH_TYPE(iecho) == ICMP_ER))
            {
                return ERR_OK; /* Echo reply received - return success */
            }
        }
    } while (len > 0);

    return ERR_TIMEOUT; /* No valid echo reply received before timeout */
}
#endif

cy_rslt_t cy_wcm_get_associated_client_list(cy_wcm_mac_t *client_list, uint8_t num_clients)
{
    cy_rslt_t res = CY_RSLT_SUCCESS;

    whd_maclist_t *mac_list_ptr;
    uint32_t      buf_length;

    if(!is_wcm_initalized)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "WCM not initialized, call cy_wcm_init() \n");
        return CY_RSLT_WCM_NOT_INITIALIZED;
    }

    if(client_list == NULL)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Bad arguments \n");
        return CY_RSLT_WCM_BAD_ARG;
    }

    if(cy_rtos_get_mutex(&wcm_mutex, CY_WCM_MAX_MUTEX_WAIT_TIME_MS) != CY_RSLT_SUCCESS)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Unable to acquire WCM mutex \n");
        return CY_RSLT_WCM_WAIT_TIMEOUT;
    }

    if(current_interface == CY_WCM_INTERFACE_TYPE_STA)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "operation not allowed for STA Interface \n");
        res = CY_RSLT_WCM_INTERFACE_NOT_SUPPORTED;
        goto exit;
    }

    if(!is_ap_network_up)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "AP is not Up !!\n");
        res = CY_RSLT_WCM_AP_NOT_UP;
        goto exit;
    }

    buf_length   = (sizeof(whd_mac_t) * num_clients) + sizeof(mac_list_ptr->count);
    mac_list_ptr = (whd_maclist_t*)malloc(buf_length);
    if(mac_list_ptr == NULL)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Heap space error \n");
        res = CY_RSLT_WCM_OUT_OF_MEMORY;
        goto exit;
    }
    memset(mac_list_ptr, 0, buf_length);
    mac_list_ptr->count = num_clients;

    if((res = whd_wifi_get_associated_client_list(whd_ifs[CY_WCM_INTERFACE_TYPE_AP], mac_list_ptr, buf_length)) != CY_RSLT_SUCCESS)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Error while getting associated client list, error = %d \n", res);
        free(mac_list_ptr);
        goto exit;
    }

    memcpy(client_list, mac_list_ptr->mac_list, (sizeof(whd_mac_t) * num_clients));
    free(mac_list_ptr);

exit:
    if(cy_rtos_set_mutex(&wcm_mutex) != CY_RSLT_SUCCESS)
    {
        res = ((res != CY_RSLT_SUCCESS) ? res : CY_RSLT_WCM_MUTEX_ERROR);
    }
    return res;
}

static uint16_t channel_to_bandwidth(wl_chanspec_t chanspec)
 {
    uint16_t band_width;
    switch(chanspec & WL_CHANSPEC_BW_MASK)
    {
        case WL_CHANSPEC_BW_5 :
            band_width = 5;
            break;

        case WL_CHANSPEC_BW_10 :
            band_width = 10;
            break;

        case WL_CHANSPEC_BW_20 :
            band_width = 20;
            break;

        case WL_CHANSPEC_BW_40 :
            band_width = 40;
            break;

        case WL_CHANSPEC_BW_80 :
            band_width = 80;
            break;

        case WL_CHANSPEC_BW_160 :
            band_width = 160;
            break;

        case WL_CHANSPEC_BW_8080 :
            band_width = 8080;
            break;

        default :
            band_width = UNKNOWN_BAND_WIDTH;
            break;
    }
    return band_width;
 }

static cy_rslt_t check_ap_credentials(const cy_wcm_connect_params_t *connect_params)
{
    uint8_t ssid_len;
    uint8_t pwd_len;

    if(connect_params == NULL)
    {
        return CY_RSLT_WCM_BAD_ARG;
    }

    ssid_len = (uint8_t)strlen((char*)connect_params->ap_credentials.SSID);
    pwd_len  = (uint8_t)strlen((char*)connect_params->ap_credentials.password);

    if(ssid_len == 0 || ssid_len > CY_WCM_MAX_SSID_LEN )
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "SSID length error\n");
        return CY_RSLT_WCM_BAD_SSID_LEN;
    }
    if(!check_wcm_security(connect_params->ap_credentials.security))
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "AP credentials security error\n");
        return CY_RSLT_WCM_SECURITY_NOT_SUPPORTED;
    }
    if((connect_params->ap_credentials.security != CY_WCM_SECURITY_OPEN) &&
       (pwd_len == 0 || pwd_len > CY_WCM_MAX_PASSPHRASE_LEN))
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "AP credentials passphrase length error\n");
        return CY_RSLT_WCM_BAD_PASSPHRASE_LEN;
    }
    return CY_RSLT_SUCCESS;
}

static cy_rslt_t convert_connect_params(const cy_wcm_connect_params_t *connect_params, whd_ssid_t *ssid, whd_mac_t *bssid,
                                          uint8_t **key, uint8_t *keylen, whd_security_t *security, ip_static_addr_t *static_ip_addr)
{
    ssid->length = (uint8_t)strlen((char*)connect_params->ap_credentials.SSID);
    memcpy(&ssid->value, connect_params->ap_credentials.SSID, ssid->length + 1);

    *keylen = strlen((char*)connect_params->ap_credentials.password);
    *key = (uint8_t*)connect_params->ap_credentials.password;
    if(!NULL_MAC(connect_params->BSSID))
    {
        memcpy(bssid, connect_params->BSSID, CY_WCM_MAC_ADDR_LEN);
    }
    else
    {
        memset(bssid, 0, sizeof(whd_mac_t));
    }
    *security = wcm_to_whd_security(connect_params->ap_credentials.security);
    if(connect_params->static_ip_settings == NULL)
    {
        memset(static_ip_addr, 0, sizeof(ip_static_addr_t));
    }
    else
    {
        memset(static_ip_addr, 0, sizeof(ip_static_addr_t));
        if(connect_params->static_ip_settings->gateway.version == CY_WCM_IP_VER_V4)
        {
#if LWIP_IPV4 && LWIP_IPV6
             static_ip_addr->gateway.u_addr.ip4.addr = connect_params->static_ip_settings->gateway.ip.v4;
             static_ip_addr->addr.u_addr.ip4.addr    = connect_params->static_ip_settings->ip_address.ip.v4;
             static_ip_addr->netmask.u_addr.ip4.addr = connect_params->static_ip_settings->netmask.ip.v4;
#elif LWIP_IPV4
             static_ip_addr->gateway.addr = connect_params->static_ip_settings->gateway.ip.v4;
             static_ip_addr->addr.addr    = connect_params->static_ip_settings->ip_address.ip.v4;
             static_ip_addr->netmask.addr = connect_params->static_ip_settings->netmask.ip.v4;
#elif LWIP_IPV6
             return CY_RSLT_WCM_BAD_ARG;
#endif
        }
        else
        {
            // TO DO : Copy the IPV6 addr
            return CY_RSLT_WCM_STATIC_IP_NOT_SUPPORTED;
        }
    }
    return CY_RSLT_SUCCESS;
}

static void donothing(void *arg)
{
}

static bool is_connected_to_same_ap(const cy_wcm_connect_params_t *connect_params)
{
    wl_bss_info_t  bss_info;

    if(whd_wifi_get_bss_info(whd_ifs[CY_WCM_INTERFACE_TYPE_STA], &bss_info) != CY_RSLT_SUCCESS)
    {
        return false;
    }
    if((strcmp((char*)connect_params->ap_credentials.SSID, (char*)bss_info.SSID) == 0) && wcm_sta_link_up)
    {
        return true;
    }
    return false;
}



static bool check_wcm_security(cy_wcm_security_t sec)
{
    switch (sec) {
        case CY_WCM_SECURITY_OPEN:
        case CY_WCM_SECURITY_WPA_TKIP_PSK:
        case CY_WCM_SECURITY_WPA_AES_PSK:
        case CY_WCM_SECURITY_WPA_MIXED_PSK:
        case CY_WCM_SECURITY_WPA2_AES_PSK:
        case CY_WCM_SECURITY_WPA2_TKIP_PSK:
        case CY_WCM_SECURITY_WPA2_MIXED_PSK:
        case CY_WCM_SECURITY_WPA2_FBT_PSK:
        case CY_WCM_SECURITY_WPA3_SAE:
        case CY_WCM_SECURITY_WPA2_WPA_AES_PSK:
        case CY_WCM_SECURITY_WPA2_WPA_MIXED_PSK:
        case CY_WCM_SECURITY_WPA3_WPA2_PSK:
        case CY_WCM_SECURITY_WPA_TKIP_ENT:
        case CY_WCM_SECURITY_WPA_AES_ENT:
        case CY_WCM_SECURITY_WPA_MIXED_ENT:
        case CY_WCM_SECURITY_WPA2_TKIP_ENT:
        case CY_WCM_SECURITY_WPA2_AES_ENT:
        case CY_WCM_SECURITY_WPA2_MIXED_ENT:
        case CY_WCM_SECURITY_WPA2_FBT_ENT:
        case CY_WCM_SECURITY_IBSS_OPEN:
        case CY_WCM_SECURITY_WPS_SECURE:
            return true;
        default:
            return false;
    }
}

void internal_scan_callback(whd_scan_result_t **result_ptr,
                             void *user_data, whd_scan_status_t status)
{
    /* Check if we don't have a scan result to send to the user */
    if (( result_ptr == NULL ) || ( *result_ptr == NULL ))
    {
        /* Check for scan complete */
        if (status == WHD_SCAN_COMPLETED_SUCCESSFULLY)
        {
            /* Notify scan complete */
            scan_handler.scan_status = status;
            memset(&scan_handler.scan_res, 0x0, sizeof(whd_scan_result_t));
 
            if(cy_worker_thread_enqueue(&cy_wcm_worker_thread, notify_scan_event, &scan_handler) != CY_RSLT_SUCCESS)
            {
                cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Error in calling the worker thread func \n");
            }
        }
        return;
    }

    scan_handler.scan_status = status;
    memcpy(&scan_handler.scan_res, *result_ptr, sizeof(whd_scan_result_t));

    if(scan_handler.scan_filter.mode == CY_WCM_SCAN_FILTER_TYPE_RSSI)
    {
         int16_t requested_range = scan_handler.scan_filter.param.rssi_range;
         int16_t signal_strength = scan_handler.scan_res.signal_strength;
         if(signal_strength < requested_range)
         {
             /* Notify the scan completion, even though the signal strength is not as requested. */
             if (status == WHD_SCAN_COMPLETED_SUCCESSFULLY)
             {
                 if(cy_worker_thread_enqueue(&cy_wcm_worker_thread, notify_scan_event, &scan_handler) != CY_RSLT_SUCCESS)
                 {
                     cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Error in calling the worker thread func \n");
                 }
             }
             memset(*result_ptr, 0, sizeof(whd_scan_result_t));
             return;
         }
    }

    if(cy_worker_thread_enqueue(&cy_wcm_worker_thread, notify_scan_event, &scan_handler) != CY_RSLT_SUCCESS)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Error in calling the worker thread func \n");
    }
    memset(*result_ptr, 0, sizeof(whd_scan_result_t));
}


static void notify_scan_event(void *arg)
{
    wcm_internal_scan_t *scan_record = (wcm_internal_scan_t*)arg;
    cy_wcm_scan_result_t scan_res;
    bool invoke_application_callback = false;
    memset(&scan_res, 0, sizeof(scan_res));

    if(cy_rtos_get_mutex(&wcm_mutex, CY_WCM_MAX_MUTEX_WAIT_TIME_MS) != CY_RSLT_SUCCESS)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Unable to acquire WCM mutex \n");
        return;
    }

    if(scan_record->scan_status == WHD_SCAN_COMPLETED_SUCCESSFULLY)
    {
        /* Reset the flag */
        scan_handler.is_scanning = false;
        invoke_application_callback = true;
    }
    else if ((scan_record->scan_status == WHD_SCAN_INCOMPLETE) && (scan_handler.is_scanning == true))
    {
        /** copy all the parameters to scan_res **/
        memcpy(scan_res.SSID, scan_record->scan_res.SSID.value, scan_record->scan_res.SSID.length + 1);
        memcpy(scan_res.BSSID, scan_record->scan_res.BSSID.octet, sizeof(scan_res.BSSID));
        scan_res.security = whd_to_wcm_security(scan_record->scan_res.security);
        scan_res.band = whd_to_wcm_band(scan_record->scan_res.band);
        scan_res.bss_type = whd_to_wcm_bss_type(scan_record->scan_res.bss_type);
        memcpy(scan_res.ccode, scan_record->scan_res.ccode, 2);
        scan_res.channel = scan_record->scan_res.channel;
        scan_res.signal_strength = scan_record->scan_res.signal_strength;
        scan_res.flags = scan_record->scan_res.flags;
        scan_res.ie_len = scan_record->scan_res.ie_len;
        scan_res.ie_ptr = (uint8_t*)malloc(scan_res.ie_len);
        if(scan_res.ie_ptr == NULL)
        {
            cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Heap space error \n");
            if (cy_rtos_set_mutex(&wcm_mutex) != CY_RSLT_SUCCESS)
            {
                cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Unable to release the WCM mutex \n");
                return;
            }
        }
        else
        {
            memcpy(scan_res.ie_ptr, scan_record->scan_res.ie_ptr, scan_res.ie_len);
            invoke_application_callback = true;
        }

    }
    else
    {
        /* Scan is aborted, reset the flag */
        scan_handler.is_scanning = false;
    }

    if (cy_rtos_set_mutex(&wcm_mutex) != CY_RSLT_SUCCESS)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Unable to release the WCM mutex \n");
        if(scan_res.ie_ptr != NULL)
        {
            free(scan_res.ie_ptr);
        }
        return;
    }

    /* Notify application appropriately based on scan status */
    if(invoke_application_callback == true)
    {
        if(scan_record->scan_status == WHD_SCAN_INCOMPLETE)
        {
            scan_handler.p_scan_calback(&scan_res, scan_handler.user_data, CY_WCM_SCAN_INCOMPLETE);
            free(scan_res.ie_ptr);
        }
        else
        {
            scan_handler.p_scan_calback(NULL, scan_handler.user_data, CY_WCM_SCAN_COMPLETE);
        }
    }
}

static void invoke_app_callbacks(cy_wcm_event_t event_type, cy_wcm_event_data_t* arg)
{
    int i = 0;

    for ( i = 0; i < CY_WCM_MAXIMUM_CALLBACKS_COUNT; i++ )
    {
        if ( wcm_event_handler[i] != NULL )
        {
            wcm_event_handler[i](event_type, arg);
        }
    }
}


static void ap_callback_handler(void* arg)
{
    cy_wcm_event_data_t link_event_data;
    wcm_ap_link_event *event_data = (wcm_ap_link_event*)arg;

    cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "Notify application about AP events \n");
    memset(&link_event_data, 0, sizeof(cy_wcm_event_data_t));
    memcpy(link_event_data.sta_mac, event_data->mac_addr, CY_WCM_MAC_ADDR_LEN);
    invoke_app_callbacks(event_data->event, &link_event_data);
    free(arg);
}

static void* ap_link_events_handler(whd_interface_t ifp, const whd_event_header_t *event_header, const uint8_t *event_data, void *handler_user_data)
{
    cy_rslt_t res = CY_RSLT_SUCCESS;
    UNUSED_PARAMETER(res);
    wcm_ap_link_event *ap_event_data;

    ap_event_data = (wcm_ap_link_event*)malloc(sizeof(wcm_ap_link_event));
    if(ap_event_data == NULL)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "malloc failed %s %d\r\n", __FILE__, __LINE__);
        return handler_user_data;
    }

    if (event_header->event_type == WLC_E_DISASSOC_IND)
    {
        ap_event_data->event = CY_WCM_EVENT_STA_LEFT_SOFTAP;
    }
    else if (event_header->event_type == WLC_E_ASSOC_IND || event_header->event_type == WLC_E_REASSOC_IND)
    {
        ap_event_data->event = CY_WCM_EVENT_STA_JOINED_SOFTAP;
    }
    else
    {
        /* Ignore other events */
        free(ap_event_data);
        return handler_user_data;
    }

    memcpy(ap_event_data->mac_addr, event_header->addr.octet, CY_WCM_MAC_ADDR_LEN);
    if((res = cy_worker_thread_enqueue(&cy_wcm_worker_thread, ap_callback_handler, (void*)ap_event_data)) != CY_RSLT_SUCCESS)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "L%d : %s() : ERROR : Failed to handle link up event. Err = [%lu]\r\n", __LINE__, __FUNCTION__, res);
        free(ap_event_data);
    }

    return handler_user_data;
}

static void* link_events_handler(whd_interface_t ifp, const whd_event_header_t *event_header, const uint8_t *event_data, void *handler_user_data)
{
    uint8_t connection_status;
    cy_rslt_t res = CY_RSLT_SUCCESS;
    UNUSED_PARAMETER(res);
    cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Link event (type, status, reason, flags) %u %u %u %u\n", (unsigned int)event_header->event_type, (unsigned int)event_header->status,
        (unsigned int)event_header->reason, (unsigned int)event_header->flags);
    switch (event_header->event_type)
    {
        case WLC_E_LINK:
            if ((event_header->flags & WLC_EVENT_MSG_LINK) != 0)
            {
                switch (sta_security_type)
                {
                    case WHD_SECURITY_OPEN:
                    case WHD_SECURITY_IBSS_OPEN:
                    case WHD_SECURITY_WPS_SECURE:
                    case WHD_SECURITY_WEP_PSK:
                    case WHD_SECURITY_WEP_SHARED:
                    {
                        /* Advertise link-up immediately as no EAPOL is required */
                        link_up();
                        break;
                    }
                    case WHD_SECURITY_WPA_TKIP_PSK:
                    case WHD_SECURITY_WPA_AES_PSK:
                    case WHD_SECURITY_WPA_MIXED_PSK:
                    case WHD_SECURITY_WPA2_AES_PSK:
                    case WHD_SECURITY_WPA2_TKIP_PSK:
                    case WHD_SECURITY_WPA2_MIXED_PSK:
                    case WHD_SECURITY_WPA2_WPA_AES_PSK:
                    case WHD_SECURITY_WPA2_WPA_MIXED_PSK:
                    case WHD_SECURITY_WPA_TKIP_ENT:
                    case WHD_SECURITY_WPA_AES_ENT:
                    case WHD_SECURITY_WPA_MIXED_ENT:
                    case WHD_SECURITY_WPA2_TKIP_ENT:
                    case WHD_SECURITY_WPA2_AES_ENT:
                    case WHD_SECURITY_WPA2_MIXED_ENT:
                    case WHD_SECURITY_WPA2_FBT_PSK:
                    case WHD_SECURITY_WPA2_FBT_ENT:
                    case WHD_SECURITY_WPA3_SAE:
                    case WHD_SECURITY_WPA3_WPA2_PSK:
                    {
                        link_up_event_received = WHD_TRUE;
                        /* Start a timer and wait for WLC_E_PSK_SUP event */
                        cy_rtos_start_timer(&sta_handshake_timer, WCM_HANDSHAKE_TIMEOUT_MS);
                        break;
                    }
                    case WHD_SECURITY_UNKNOWN:
                    case WHD_SECURITY_FORCE_32_BIT:
                    default:
                    {
                        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Bad Security type\r\n");
                        break;
                    }
                }
                /* Notify application connecting */
                connection_status = CY_WCM_EVENT_CONNECTING;
                if((res = cy_worker_thread_enqueue(&cy_wcm_worker_thread, notify_connection_status, &connection_status)) != CY_RSLT_SUCCESS)
                {
                    cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "L%d : %s() : ERROR : Failed to send connection status. Err = [%lu]\r\n", __LINE__, __FUNCTION__, res);
                }
            }
            else
            {
                /* Check if the link down event is followed by too many RSN IE Error. If yes, try join again */
                if (too_many_ie_error)
                {
                    /* Try to join the AP again */
                    handshake_timeout_handler(0);

                    /* Clear the error flag */
                    too_many_ie_error = WHD_FALSE;
                }
                else
                {
                    /* WPA handshake is aborted because of link down. Stop handshake timer. */

                    /* Stop the handshake timer */
                    cy_rtos_stop_timer(&sta_handshake_timer);

                    /* Stop the retry timer */
                    cy_rtos_stop_timer(&sta_retry_timer);
                    retry_backoff_timeout = DEFAULT_RETRY_BACKOFF_TIMEOUT_IN_MS;

                    link_down();
                }
            }
            break;

        case WLC_E_DEAUTH_IND:
        case WLC_E_DISASSOC_IND:
            link_down();
            break;

        case WLC_E_PSK_SUP:
            {
                cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "recieved WLC_E_PSK_SUP \n");
                /* WLC_E_PSK_SUP is received. Check for status and reason. */
                if ( event_header->status == WLC_SUP_KEYED && event_header->reason == WLC_E_SUP_OTHER && link_up_event_received == WHD_TRUE )
                {
                    /* Successful WPA key exchange. Stop timer and announce link up event to application */

                    /* Stop handshake Timer */
                    cy_rtos_stop_timer(&sta_handshake_timer);

                    /* Stop retry Timer */
                    cy_rtos_stop_timer(&sta_retry_timer);

                    retry_backoff_timeout = DEFAULT_RETRY_BACKOFF_TIMEOUT_IN_MS;

                    link_up();
                    link_up_event_received = WHD_FALSE;
                }
                else if ( event_header->reason == WLC_E_SUP_MSG3_TOO_MANY_IE )
                {
                    /* Wifi firmware will do disassoc internally and will not retry to join the AP.
                     * Set a flag to indicate the join should be retried (from the host side).
                     */
                    too_many_ie_error = WHD_TRUE;
                }
                break;
            }

        /* Note - These are listed to keep gcc pedantic checking happy */
        case WLC_E_NONE:
        case WLC_E_ROAM:
        case WLC_E_SET_SSID:
        case WLC_E_DEAUTH:
        case WLC_E_DISASSOC:
        case WLC_E_JOIN:
        case WLC_E_START:
        case WLC_E_AUTH:
        case WLC_E_AUTH_IND:
        case WLC_E_ASSOC:
        case WLC_E_ASSOC_IND:
        case WLC_E_REASSOC:
        case WLC_E_REASSOC_IND:
        case WLC_E_QUIET_START:
        case WLC_E_QUIET_END:
        case WLC_E_BEACON_RX:
        case WLC_E_MIC_ERROR:
        case WLC_E_NDIS_LINK:
        case WLC_E_TXFAIL:
        case WLC_E_PMKID_CACHE:
        case WLC_E_RETROGRADE_TSF:
        case WLC_E_PRUNE:
        case WLC_E_AUTOAUTH:
        case WLC_E_EAPOL_MSG:
        case WLC_E_SCAN_COMPLETE:
        case WLC_E_ADDTS_IND:
        case WLC_E_DELTS_IND:
        case WLC_E_BCNSENT_IND:
        case WLC_E_BCNRX_MSG:
        case WLC_E_BCNLOST_MSG:
        case WLC_E_ROAM_PREP:
        case WLC_E_PFN_NET_FOUND:
        case WLC_E_PFN_NET_LOST:
        case WLC_E_RESET_COMPLETE:
        case WLC_E_JOIN_START:
        case WLC_E_ROAM_START:
        case WLC_E_ASSOC_START:
        case WLC_E_IBSS_ASSOC:
        case WLC_E_RADIO:
        case WLC_E_PSM_WATCHDOG:
        case WLC_E_CCX_ASSOC_START:
        case WLC_E_CCX_ASSOC_ABORT:
        case WLC_E_PROBREQ_MSG:
        case WLC_E_SCAN_CONFIRM_IND:
        case WLC_E_COUNTRY_CODE_CHANGED:
        case WLC_E_EXCEEDED_MEDIUM_TIME:
        case WLC_E_ICV_ERROR:
        case WLC_E_UNICAST_DECODE_ERROR:
        case WLC_E_MULTICAST_DECODE_ERROR:
        case WLC_E_TRACE:
        case WLC_E_BTA_HCI_EVENT:
        case WLC_E_IF:
        case WLC_E_P2P_DISC_LISTEN_COMPLETE:
        case WLC_E_RSSI:
        case WLC_E_PFN_SCAN_COMPLETE:
        case WLC_E_EXTLOG_MSG:
        case WLC_E_ACTION_FRAME:
        case WLC_E_ACTION_FRAME_COMPLETE:
        case WLC_E_PRE_ASSOC_IND:
        case WLC_E_PRE_REASSOC_IND:
        case WLC_E_CHANNEL_ADOPTED:
        case WLC_E_AP_STARTED:
        case WLC_E_DFS_AP_STOP:
        case WLC_E_DFS_AP_RESUME:
        case WLC_E_WAI_STA_EVENT:
        case WLC_E_WAI_MSG:
        case WLC_E_ESCAN_RESULT:
        case WLC_E_ACTION_FRAME_OFF_CHAN_COMPLETE:
        case WLC_E_PROBRESP_MSG:
        case WLC_E_P2P_PROBREQ_MSG:
        case WLC_E_DCS_REQUEST:
        case WLC_E_FIFO_CREDIT_MAP:
        case WLC_E_ACTION_FRAME_RX:
        case WLC_E_WAKE_EVENT:
        case WLC_E_RM_COMPLETE:
        case WLC_E_HTSFSYNC:
        case WLC_E_OVERLAY_REQ:
        case WLC_E_CSA_COMPLETE_IND:
        case WLC_E_EXCESS_PM_WAKE_EVENT:
        case WLC_E_PFN_SCAN_NONE:
        case WLC_E_PFN_SCAN_ALLGONE:
        case WLC_E_GTK_PLUMBED:
        case WLC_E_ASSOC_IND_NDIS:
        case WLC_E_REASSOC_IND_NDIS:
        case WLC_E_ASSOC_REQ_IE:
        case WLC_E_ASSOC_RESP_IE:
        case WLC_E_ASSOC_RECREATED:
        case WLC_E_ACTION_FRAME_RX_NDIS:
        case WLC_E_AUTH_REQ:
        case WLC_E_TDLS_PEER_EVENT:
        case WLC_E_SPEEDY_RECREATE_FAIL:
        case WLC_E_NATIVE:
        case WLC_E_PKTDELAY_IND:
        case WLC_E_AWDL_AW:
        case WLC_E_AWDL_ROLE:
        case WLC_E_AWDL_EVENT:
        case WLC_E_NIC_AF_TXS:
        case WLC_E_NAN:
        case WLC_E_BEACON_FRAME_RX:
        case WLC_E_SERVICE_FOUND:
        case WLC_E_GAS_FRAGMENT_RX:
        case WLC_E_GAS_COMPLETE:
        case WLC_E_P2PO_ADD_DEVICE:
        case WLC_E_P2PO_DEL_DEVICE:
        case WLC_E_WNM_STA_SLEEP:
        case WLC_E_TXFAIL_THRESH:
        case WLC_E_PROXD:
        case WLC_E_IBSS_COALESCE:
        case WLC_E_AWDL_RX_PRB_RESP:
        case WLC_E_AWDL_RX_ACT_FRAME:
        case WLC_E_AWDL_WOWL_NULLPKT:
        case WLC_E_AWDL_PHYCAL_STATUS:
        case WLC_E_AWDL_OOB_AF_STATUS:
        case WLC_E_AWDL_SCAN_STATUS:
        case WLC_E_AWDL_AW_START:
        case WLC_E_AWDL_AW_END:
        case WLC_E_AWDL_AW_EXT:
        case WLC_E_AWDL_PEER_CACHE_CONTROL:
        case WLC_E_CSA_START_IND:
        case WLC_E_CSA_DONE_IND:
        case WLC_E_CSA_FAILURE_IND:
        case WLC_E_CCA_CHAN_QUAL:
        case WLC_E_BSSID:
        case WLC_E_TX_STAT_ERROR:
        case WLC_E_BCMC_CREDIT_SUPPORT:
        case WLC_E_PSTA_PRIMARY_INTF_IND:
        case WLC_E_RRM:
        case WLC_E_ULP:
        case WLC_E_LAST:
        case WLC_E_BT_WIFI_HANDOVER_REQ:
        case WLC_E_PFN_BEST_BATCHING:
        case WLC_E_SPW_TXINHIBIT:
        case WLC_E_FBT_AUTH_REQ_IND:
        case WLC_E_RSSI_LQM:
        case WLC_E_PFN_GSCAN_FULL_RESULT:
        case WLC_E_PFN_SWC:
        case WLC_E_AUTHORIZED:
        case WLC_E_PROBREQ_MSG_RX:
        case WLC_E_RMC_EVENT:
        case WLC_E_DPSTA_INTF_IND:
        default:
            cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Received event which was not registered\n");
            break;
    }
    return handler_user_data;
}

static void link_up( void )
{
    cy_rslt_t res = CY_RSLT_SUCCESS;
    UNUSED_PARAMETER(res);
    if(!wcm_sta_link_up)
    {
        if((res = cy_worker_thread_enqueue(&cy_wcm_worker_thread, sta_link_up_handler, NULL)) != CY_RSLT_SUCCESS)
        {
            cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "L%d : %s() : ERROR : Failed to handle link up event. Err = [%lu]\r\n", __LINE__, __FUNCTION__, res);
            return;
        }
        wcm_sta_link_up = true;
    }
    else
    {
        /* Do not renew DHCP if the link was connected through static IP */
#if LWIP_IPV4 && LWIP_IPV6
        if(connected_ap_details.static_ip.addr.u_addr.ip4.addr != 0)
#elif LWIP_IPV4
        if(connected_ap_details.static_ip.addr.addr != 0)
#endif
        {
            return;
        }
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Executing DHCP renewal \n");
        /* This condition will be hit when handshake fails and wcm reconnection is successful through retry timer*/
        if((res = cy_worker_thread_enqueue(&cy_wcm_worker_thread, sta_link_up_renew_handler, NULL)) != CY_RSLT_SUCCESS)
        {
            cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "L%d : %s() : ERROR : Failed to handle link up event. Err = [%lu]\r\n", __LINE__, __FUNCTION__, res);
            return;
        }
    }
}

static void sta_link_up_handler(void* arg)
{
    UNUSED_PARAMETER(arg);
    cy_lwip_nw_interface_t nw_interface;
    nw_interface.whd_iface = whd_ifs[CY_WCM_INTERFACE_TYPE_STA];
    nw_interface.role = CY_LWIP_STA_NW_INTERFACE;
    cy_lwip_network_up(&nw_interface);
    cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "Notify application that network is connected again!\n");
    invoke_app_callbacks(CY_WCM_EVENT_RECONNECTED, NULL);
}

static void notify_connection_status(void* arg)
{
    uint8_t *val = (uint8_t*)arg;
    cy_wcm_event_t connect_status = (cy_wcm_event_t) *val;
    cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "Notify connection status = %d!\n", connect_status);
    invoke_app_callbacks(connect_status, NULL);
}

static void sta_link_up_renew_handler(void* arg)
{
    UNUSED_PARAMETER(arg);
#if LWIP_IPV4
    cy_lwip_nw_interface_t nw_interface;
    nw_interface.role = CY_LWIP_STA_NW_INTERFACE;
    nw_interface.whd_iface = whd_ifs[CY_WCM_INTERFACE_TYPE_STA];
    cy_lwip_dhcp_renew(&nw_interface);
#endif
}

static void link_down(void)
{
    cy_rslt_t res = CY_RSLT_SUCCESS;
    UNUSED_PARAMETER(res);
    if (wcm_sta_link_up)
    {
        if((res = cy_worker_thread_enqueue(&cy_wcm_worker_thread, sta_link_down_handler, NULL)) != CY_RSLT_SUCCESS)
        {
            cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "L%d : %s() : ERROR : Failed to handle link down event. Err = [%lu]\r\n", __LINE__, __FUNCTION__, res);
        }
        wcm_sta_link_up = false;
    }
}

static void sta_link_down_handler(void* arg)
{
    cy_lwip_nw_interface_t nw_interface;
    UNUSED_PARAMETER(arg);

    nw_interface.whd_iface = whd_ifs[CY_WCM_INTERFACE_TYPE_STA];
    nw_interface.role = CY_LWIP_STA_NW_INTERFACE;
    cy_lwip_network_down(&nw_interface);
    cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "Notify application that network is down!\n");
    invoke_app_callbacks(CY_WCM_EVENT_DISCONNECTED, NULL);
}
static void hanshake_retry_timer(cy_timer_callback_arg_t arg)
{
    UNUSED_PARAMETER(arg);
    handshake_error_callback(0);
}

static void handshake_timeout_handler(cy_timer_callback_arg_t arg)
{
    cy_rslt_t result;

    UNUSED_PARAMETER( arg );

    cy_rtos_stop_timer(&sta_handshake_timer);
    cy_rtos_stop_timer(&sta_retry_timer);
    retry_backoff_timeout = DEFAULT_RETRY_BACKOFF_TIMEOUT_IN_MS;

    result = cy_worker_thread_enqueue(&cy_wcm_worker_thread, handshake_error_callback, NULL);
    if (result != CY_RSLT_SUCCESS)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "L%d : %s() : Failed to send async event to n/w worker thread. Err = [%lu]\r\n", __LINE__, __FUNCTION__, result);
    }
}


static void handshake_error_callback(void *arg)
{
    cy_rslt_t res;
    uint8_t   retries;

    UNUSED_PARAMETER(arg);

    /* stop the retry timer */
    cy_rtos_stop_timer(&sta_retry_timer);
    retry_backoff_timeout = DEFAULT_RETRY_BACKOFF_TIMEOUT_IN_MS;

    /* De-register the link event handler */
    whd_wifi_deregister_event_handler(whd_ifs[CY_WCM_INTERFACE_TYPE_STA], sta_event_handler_index);

    /* Explicitly leave AP and then rejoin */
    whd_wifi_leave(whd_ifs[CY_WCM_INTERFACE_TYPE_STA]);
    cy_rtos_delay_milliseconds(100);

    for(retries = 0; retries < JOIN_RETRY_ATTEMPTS; retries++)
    {
        cy_rslt_t join_result;
        if(!NULL_MAC(connected_ap_details.sta_mac.octet))
        {
            whd_scan_result_t ap;
            ap.security = connected_ap_details.security;
            ap.SSID.length = connected_ap_details.SSID.length;
            memcpy(ap.SSID.value, connected_ap_details.SSID.value, ap.SSID.length);
            memcpy(ap.BSSID.octet, connected_ap_details.sta_mac.octet, CY_WCM_MAC_ADDR_LEN);
            /*
             * If MAC address of a AP is know there is no need to populate channel or band
             * instead we can set the band to auto and invoke whd join
             */
            whd_wifi_set_ioctl_value(whd_ifs[CY_WCM_INTERFACE_TYPE_STA], WLC_SET_BAND, WLC_BAND_AUTO);
            join_result = whd_wifi_join_specific(whd_ifs[CY_WCM_INTERFACE_TYPE_STA], &ap, connected_ap_details.key, connected_ap_details.keylen);
        }
        else
        {
            if(connected_ap_details.band == CY_WCM_WIFI_BAND_5GHZ)
            {
                whd_wifi_set_ioctl_value(whd_ifs[CY_WCM_INTERFACE_TYPE_STA], WLC_SET_BAND, WLC_BAND_5G);
            }
            else
            {
                /*
                 * It could be possible on a dual band supported device,
                 * the current band set is 5G and the requested band from user is 2.4Ghz
                 */
                whd_wifi_set_ioctl_value(whd_ifs[CY_WCM_INTERFACE_TYPE_STA], WLC_SET_BAND, WLC_BAND_2G);

            }
            /** Join to Wi-Fi AP **/
            join_result = whd_wifi_join(whd_ifs[CY_WCM_INTERFACE_TYPE_STA], &connected_ap_details.SSID, connected_ap_details.security, connected_ap_details.key, connected_ap_details.keylen);
        }

        if(join_result == CY_RSLT_SUCCESS)
        {
            link_up();
            sta_security_type = connected_ap_details.security;
            /* Register for Link events*/
            if(whd_management_set_event_handler(whd_ifs[CY_WCM_INTERFACE_TYPE_STA], sta_link_events, link_events_handler, NULL, &sta_event_handler_index) != CY_RSLT_SUCCESS)
            {
                cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "sta link event registration failed \n");
            }
            /* Reset retry-backoff-timeout */
            retry_backoff_timeout = DEFAULT_RETRY_BACKOFF_TIMEOUT_IN_MS;
            return;
        }

    }

    /* Register retry with network worker thread */
    cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "L%d : %s() : Retrying to join with back-off [%ld secs]\r\n", __LINE__, __FUNCTION__, retry_backoff_timeout);
    res = cy_rtos_start_timer(&sta_retry_timer, retry_backoff_timeout);
    if (res != CY_RSLT_SUCCESS)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "L%d : %s() : ERROR : Failed in join retry with back-off. Err = [%ld]\r\n", __LINE__, __FUNCTION__, res);
    }
    /* Update backoff timeout */
    retry_backoff_timeout = (retry_backoff_timeout < MAX_RETRY_BACKOFF_TIMEOUT_IN_MS)? (uint32_t)(retry_backoff_timeout * 2) : MAX_RETRY_BACKOFF_TIMEOUT_IN_MS;
}

static void lwip_ip_change_callback(void *arg)
{
    UNUSED_PARAMETER(arg);
    cy_rslt_t result;
    if(!wcm_sta_link_up)
    {
        return;
    }
    result = cy_worker_thread_enqueue(&cy_wcm_worker_thread, notify_ip_change, NULL);
    if (result != CY_RSLT_SUCCESS)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "L%d : %s() : Failed to send async event to n/w worker thread. Err = [%lu]\r\n", __LINE__, __FUNCTION__, result);
    }
}
void notify_ip_change(void *arg)
{
#if LWIP_IPV4
    cy_wcm_event_data_t link_event_data;
    ip4_addr_t* addr = NULL;
    struct netif *net_interface = NULL;
    UNUSED_VARIABLE(addr);

    uint32_t ipv4_addr;
    UNUSED_PARAMETER(arg);

    cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "Notify application that ip has changed!\n");
    memset(&link_event_data, 0, sizeof(cy_wcm_event_data_t));
    net_interface = cy_lwip_get_interface(CY_LWIP_STA_NW_INTERFACE);
#if LWIP_IPV6
    ipv4_addr = net_interface->ip_addr.u_addr.ip4.addr;
    addr = &net_interface->ip_addr.u_addr.ip4;
#else
    ipv4_addr = net_interface->ip_addr.addr;
    addr = &net_interface->ip_addr;
#endif

    if (ipv4_addr != 0)
    {
        link_event_data.ip_addr.version = CY_WCM_IP_VER_V4;
        link_event_data.ip_addr.ip.v4   = ipv4_addr;
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "IP Address %s assigned \n", ip4addr_ntoa(addr));
        invoke_app_callbacks(CY_WCM_EVENT_IP_CHANGED, &link_event_data);
    }
#endif
}


static bool check_if_platform_supports_band(cy_wcm_wifi_band_t requested_band)
{
    whd_band_list_t band_list;

    CHECK_RETURN(whd_wifi_get_ioctl_buffer(whd_ifs[CY_WCM_INTERFACE_TYPE_STA], WLC_GET_BANDLIST, (uint8_t*)&band_list, sizeof(whd_band_list_t)));

    cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "band_list.current_band = %ld, band_list.number_of_bands = %ld , requested band = %d \n ", band_list.current_band, band_list.number_of_bands, requested_band);

     /*
      * All CY chips which supports 5Ghz will also support 2.4Ghz and
      * for such platforms number of bands will be 2
      */
    if(band_list.number_of_bands == 1)
    {
        if(requested_band == CY_WCM_WIFI_BAND_5GHZ)
        {
            return false;
        }
        else
        {
            return true;
        }
    }
    return true;
}

static cy_rslt_t network_up(whd_interface_t interface, cy_lwip_nw_interface_role_t role, ip_static_addr_t *static_ip_ptr)
{
    cy_rslt_t res = CY_RSLT_SUCCESS;
    cy_lwip_nw_interface_t nw_interface;

    nw_interface.role = role;
    nw_interface.whd_iface = interface;
    if ((res = cy_lwip_add_interface(&nw_interface, static_ip_ptr)) != CY_RSLT_SUCCESS)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "failed to add the network interface \n");
        return res;
    }

    if((res = cy_lwip_network_up(&nw_interface)) != CY_RSLT_SUCCESS)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "failed to bring up the network stack \n");
        return res;
    }
    (role == CY_LWIP_STA_NW_INTERFACE) ? (is_sta_network_up = true) : (is_ap_network_up = true);
    return res;
}

static void network_down(whd_interface_t interface, cy_lwip_nw_interface_role_t role)
{
    cy_lwip_nw_interface_t nw_interface;

    nw_interface.role = role;
    nw_interface.whd_iface = interface;
    cy_lwip_network_down(&nw_interface);
    cy_lwip_remove_interface(&nw_interface);
    (role == CY_LWIP_STA_NW_INTERFACE) ? (is_sta_network_up = false) : (is_ap_network_up = false);
}

static whd_security_t wcm_to_whd_security(cy_wcm_security_t sec)
{
    switch(sec)
    {
        case CY_WCM_SECURITY_OPEN :
            return WHD_SECURITY_OPEN;

        case CY_WCM_SECURITY_WEP_PSK:
            return WHD_SECURITY_WEP_PSK;

        case CY_WCM_SECURITY_WEP_SHARED:
            return WHD_SECURITY_WEP_SHARED;

        case CY_WCM_SECURITY_WPA_TKIP_PSK:
            return WHD_SECURITY_WPA_TKIP_PSK;

        case CY_WCM_SECURITY_WPA_AES_PSK:
            return WHD_SECURITY_WPA_AES_PSK;

        case CY_WCM_SECURITY_WPA_MIXED_PSK:
            return WHD_SECURITY_WPA_MIXED_PSK;

        case CY_WCM_SECURITY_WPA2_AES_PSK:
            return WHD_SECURITY_WPA2_AES_PSK;

        case CY_WCM_SECURITY_WPA2_TKIP_PSK:
            return WHD_SECURITY_WPA2_TKIP_PSK;

        case CY_WCM_SECURITY_WPA2_MIXED_PSK:
            return WHD_SECURITY_WPA2_MIXED_PSK;

        case CY_WCM_SECURITY_WPA2_FBT_PSK:
            return WHD_SECURITY_WPA2_FBT_PSK;

        case CY_WCM_SECURITY_WPA3_SAE:
            return WHD_SECURITY_WPA3_SAE;

        case CY_WCM_SECURITY_WPA2_WPA_AES_PSK:
            return WHD_SECURITY_WPA2_WPA_AES_PSK;

        case CY_WCM_SECURITY_WPA2_WPA_MIXED_PSK:
            return WHD_SECURITY_WPA2_WPA_MIXED_PSK;

        case CY_WCM_SECURITY_WPA3_WPA2_PSK:
            return WHD_SECURITY_WPA3_WPA2_PSK;

        case CY_WCM_SECURITY_WPA_TKIP_ENT:
            return WHD_SECURITY_WPA_TKIP_ENT;

        case CY_WCM_SECURITY_WPA_AES_ENT:
            return WHD_SECURITY_WPA_AES_ENT;

        case CY_WCM_SECURITY_WPA_MIXED_ENT:
            return WHD_SECURITY_WPA_MIXED_ENT;

        case CY_WCM_SECURITY_WPA2_TKIP_ENT:
            return WHD_SECURITY_WPA2_TKIP_ENT;

        case CY_WCM_SECURITY_WPA2_AES_ENT:
            return WHD_SECURITY_WPA2_AES_ENT;

        case CY_WCM_SECURITY_WPA2_MIXED_ENT:
            return WHD_SECURITY_WPA2_MIXED_ENT;

        case CY_WCM_SECURITY_WPA2_FBT_ENT:
            return WHD_SECURITY_WPA2_FBT_ENT;

        case CY_WCM_SECURITY_IBSS_OPEN:
            return WHD_SECURITY_IBSS_OPEN;

        case CY_WCM_SECURITY_WPS_SECURE:
            return WHD_SECURITY_WPS_SECURE;

        default:
            return WHD_SECURITY_UNKNOWN;
    }
}

static cy_wcm_security_t whd_to_wcm_security(whd_security_t sec)
{
    switch(sec)
    {
        case WHD_SECURITY_OPEN:
            return CY_WCM_SECURITY_OPEN;

        case WHD_SECURITY_WEP_PSK:
            return CY_WCM_SECURITY_WEP_PSK;

        case WHD_SECURITY_WEP_SHARED:
            return CY_WCM_SECURITY_WEP_SHARED;

        case WHD_SECURITY_WPA_TKIP_PSK:
            return CY_WCM_SECURITY_WPA_TKIP_PSK;

        case WHD_SECURITY_WPA_AES_PSK:
            return CY_WCM_SECURITY_WPA_AES_PSK;

        case WHD_SECURITY_WPA_MIXED_PSK:
            return CY_WCM_SECURITY_WPA_MIXED_PSK;

        case WHD_SECURITY_WPA2_AES_PSK:
            return CY_WCM_SECURITY_WPA2_AES_PSK;

        case WHD_SECURITY_WPA2_TKIP_PSK:
            return CY_WCM_SECURITY_WPA2_TKIP_PSK;

        case WHD_SECURITY_WPA2_MIXED_PSK:
            return CY_WCM_SECURITY_WPA2_MIXED_PSK;

        case WHD_SECURITY_WPA2_FBT_PSK:
            return CY_WCM_SECURITY_WPA2_FBT_PSK;

        case WHD_SECURITY_WPA3_SAE:
            return CY_WCM_SECURITY_WPA3_SAE;

        case WHD_SECURITY_WPA2_WPA_AES_PSK:
            return CY_WCM_SECURITY_WPA2_WPA_AES_PSK;

        case WHD_SECURITY_WPA2_WPA_MIXED_PSK:
            return CY_WCM_SECURITY_WPA2_WPA_MIXED_PSK;

        case WHD_SECURITY_WPA3_WPA2_PSK:
            return CY_WCM_SECURITY_WPA3_WPA2_PSK;

        case WHD_SECURITY_WPA_TKIP_ENT:
            return CY_WCM_SECURITY_WPA_TKIP_ENT;

        case WHD_SECURITY_WPA_AES_ENT:
            return CY_WCM_SECURITY_WPA_AES_ENT;

        case WHD_SECURITY_WPA_MIXED_ENT:
            return CY_WCM_SECURITY_WPA_MIXED_ENT;

        case WHD_SECURITY_WPA2_TKIP_ENT:
            return CY_WCM_SECURITY_WPA2_TKIP_ENT;

        case WHD_SECURITY_WPA2_AES_ENT:
            return CY_WCM_SECURITY_WPA2_AES_ENT;

        case WHD_SECURITY_WPA2_MIXED_ENT:
            return CY_WCM_SECURITY_WPA2_MIXED_ENT;

        case WHD_SECURITY_WPA2_FBT_ENT:
            return CY_WCM_SECURITY_WPA2_FBT_ENT;

        case WHD_SECURITY_IBSS_OPEN:
            return CY_WCM_SECURITY_IBSS_OPEN;

        case WHD_SECURITY_WPS_SECURE:
            return CY_WCM_SECURITY_WPS_SECURE;

        default:
            return CY_WCM_SECURITY_UNKNOWN;
    }
}

static cy_wcm_wifi_band_t whd_to_wcm_band(whd_802_11_band_t band)
{
    return ((band == WHD_802_11_BAND_5GHZ) ? CY_WCM_WIFI_BAND_5GHZ : CY_WCM_WIFI_BAND_2_4GHZ);
}

static cy_wcm_bss_type_t whd_to_wcm_bss_type(whd_bss_type_t bss_type)
{
    switch(bss_type)
    {
        case WHD_BSS_TYPE_INFRASTRUCTURE:
            return CY_WCM_BSS_TYPE_INFRASTRUCTURE;

        case WHD_BSS_TYPE_ADHOC:
            return CY_WCM_BSS_TYPE_ADHOC;

        case WHD_BSS_TYPE_ANY:
            return CY_WCM_BSS_TYPE_ANY;

        case WHD_BSS_TYPE_MESH:
            return CY_WCM_BSS_TYPE_MESH;

        default :
            return CY_WCM_BSS_TYPE_UNKNOWN;
    }
}

static void read_ap_config(const cy_wcm_ap_config_t *ap_config, whd_ssid_t *ssid,
                              uint8_t **key, uint8_t *keylen, whd_security_t *security, ip_static_addr_t *static_ip_addr)
{
    ssid->length = (uint8_t)strlen((char*)ap_config->ap_credentials.SSID);
    memcpy(&ssid->value, ap_config->ap_credentials.SSID, ssid->length + 1);

    *keylen = strlen((char*)ap_config->ap_credentials.password);
    *key = (uint8_t*)ap_config->ap_credentials.password;
    *security = wcm_to_whd_security(ap_config->ap_credentials.security);

    memset(static_ip_addr, 0, sizeof(ip_static_addr_t));
    if(ap_config->ip_settings.gateway.version == CY_WCM_IP_VER_V4)
    {
#if LWIP_IPV4 && LWIP_IPV6
        static_ip_addr->gateway.u_addr.ip4.addr = ap_config->ip_settings.gateway.ip.v4;
        static_ip_addr->addr.u_addr.ip4.addr    = ap_config->ip_settings.ip_address.ip.v4;
        static_ip_addr->netmask.u_addr.ip4.addr = ap_config->ip_settings.netmask.ip.v4;
#elif LWIP_IPV4
        static_ip_addr->gateway.addr = ap_config->ip_settings.gateway.ip.v4;
        static_ip_addr->addr.addr    = ap_config->ip_settings.ip_address.ip.v4;
        static_ip_addr->netmask.addr = ap_config->ip_settings.netmask.ip.v4;
#endif
    }
}

static cy_rslt_t check_soft_ap_config(const cy_wcm_ap_config_t *ap_config_params)
{
    uint8_t ssid_len;
    uint8_t pwd_len;

    if(ap_config_params == NULL)
    {
        return CY_RSLT_WCM_BAD_ARG;
    }

    ssid_len = (uint8_t)strlen((char*)ap_config_params->ap_credentials.SSID);
    pwd_len  = (uint8_t)strlen((char*)ap_config_params->ap_credentials.password);

    if(ap_config_params->ip_settings.ip_address.ip.v4 == 0)
    {
        return CY_RSLT_WCM_BAD_STATIC_IP;
    }

    if(ssid_len == 0 || ssid_len > CY_WCM_MAX_SSID_LEN )
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "SSID length error\n");
        return CY_RSLT_WCM_BAD_SSID_LEN;
    }

    if(!check_wcm_security(ap_config_params->ap_credentials.security))
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "AP credentials security error\n");
        return CY_RSLT_WCM_SECURITY_NOT_SUPPORTED;
    }

    if((ap_config_params->ap_credentials.security != CY_WCM_SECURITY_OPEN) &&
       (pwd_len == 0 || pwd_len > CY_WCM_MAX_PASSPHRASE_LEN))
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "AP credentials passphrase length error\n");
        return CY_RSLT_WCM_BAD_PASSPHRASE_LEN;
    }

    if(ap_config_params->ie_info != NULL)
    {
        if(ap_config_params->ie_info->data == NULL)
        {
            cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Invalid IE \n");
            return CY_RSLT_WCM_INVALID_IE;
        }
    }

    /*
     * Validate the channel and return appropriate error
     * channel number 1 to 11 is 2G
     * channel number 36 to 100 is 5G
     */
    if((ap_config_params->channel > 0) && (ap_config_params->channel < 12))
    {
        /* All platforms by default supports 2.4Ghz so return success*/
        return CY_RSLT_SUCCESS;
    }
    else if((ap_config_params->channel > 35) && (ap_config_params->channel < 101))
    {
        if(!check_if_platform_supports_band(CY_WCM_WIFI_BAND_5GHZ))
        {
            return CY_RSLT_WCM_BAND_NOT_SUPPORTED;
        }
    }
    else
    {
        return CY_RSLT_WCM_AP_BAD_CHANNEL;
    }

    return CY_RSLT_SUCCESS;
}

static cy_rslt_t init_whd_wifi_interface(cy_wcm_interface_t iface_type)
{

    if(iface_type != CY_WCM_INTERFACE_TYPE_AP_STA)
    {
        if (cybsp_wifi_init_primary(&whd_ifs[iface_type]) != CY_RSLT_SUCCESS)
        {
            cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Error in initializing primary interface !!\n");
            return CY_RSLT_WCM_BSP_INIT_ERROR;
        }

    }
    else
    {
        /* In concurrent mode primary interface would be STA and secondary AP*/
        if (cybsp_wifi_init_primary(&whd_ifs[CY_WCM_INTERFACE_TYPE_STA]) != CY_RSLT_SUCCESS)
        {
            cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Error in getting primary interface mac address !!\n");
            return CY_RSLT_WCM_BSP_INIT_ERROR;
        }

        /* SoftAP is the secondary interface in Concurrent mode*/
        if(cybsp_wifi_init_secondary(&whd_ifs[CY_WCM_INTERFACE_TYPE_AP], NULL) != CY_RSLT_SUCCESS)
        {
            cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Error Unable to bring up the secondary interface !!\n");
            return CY_RSLT_WCM_SECONDARY_INTERFACE_ERROR;
        }
    }
    return CY_RSLT_SUCCESS;
}

cy_rslt_t cy_wcm_get_whd_interface(cy_wcm_interface_t interface_type, whd_interface_t *whd_iface)
{
    if(whd_iface == NULL)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Error : bad arguments \n");
        return CY_RSLT_WCM_BAD_ARG;
    }

    if(!is_wcm_initalized)
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "WCM is not initialized. Call cy_wcm_init() to initialize \n");
        return CY_RSLT_WCM_NOT_INITIALIZED;
    }

    if((interface_type != CY_WCM_INTERFACE_TYPE_STA) || (current_interface == CY_WCM_INTERFACE_TYPE_AP))
    {
        cy_wcm_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Supported only for STA interface !!\n");
        return CY_RSLT_WCM_INTERFACE_NOT_SUPPORTED;
    }

    *whd_iface = whd_ifs[interface_type];
    return CY_RSLT_SUCCESS;
}
