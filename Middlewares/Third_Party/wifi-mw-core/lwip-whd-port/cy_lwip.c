/*
 * Copyright 2020 Cypress Semiconductor Corporation
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <string.h>
#include <stdint.h>
#include "lwipopts.h"
#include "lwip/netif.h"
#include "lwip/netifapi.h"
#include "lwip/init.h"
#include "lwip/dhcp.h"
#include "lwip/etharp.h"
#include "lwip/tcpip.h"
#include "lwip/ethip6.h"
#include "lwip/igmp.h"
#include "lwip/nd6.h"
#include "netif/ethernet.h"
#include "lwip/prot/autoip.h"
#include "lwip/prot/dhcp.h"
#include "lwip/dns.h"

#include "cybsp_wifi.h"
#include "cy_network_buffer.h"
#include "whd.h"
#include "cy_lwip.h"
#include "cy_lwip_error.h"
#include "cy_lwip_dhcp_server.h"
#include "cy_result.h"
#include "whd.h"
#include "whd_wifi_api.h"
#include "whd_network_types.h"
#include "whd_buffer_api.h"
#include "cy_lwip_log.h"

#define EAPOL_PACKET_TYPE                        0x888E

#define MULTICAST_IP_TO_MAC(ip)                  {   (uint8_t) 0x01,             \
                                                     (uint8_t) 0x00,             \
                                                     (uint8_t) 0x5e,             \
                                                     (uint8_t) ((ip)[1] & 0x7F), \
                                                     (uint8_t) (ip)[2],          \
                                                     (uint8_t) (ip)[3]           \
                                                 }

#define IPV6_MULTICAST_TO_MAC_PREFIX             (0x33)

#define DHCP_IP_ADDRESS_RESOLUTION_TIMEOUT_IN_MS (15000)
#ifndef AUTO_IP_ADDRESS_RESOLUTION_TIMEOUT_IN_MS
#define AUTO_IP_ADDRESS_RESOLUTION_TIMEOUT_IN_MS (60000 * 10)
#endif
#define DCHP_RENEWAL_DELAY_IN_MS                 (100)
#define DHCP_STOP_DELAY_IN_MS                    (400)

#define MAX_NW_INTERFACE                         (2)

struct  netif                                    *cy_lwip_ip_handle[MAX_NW_INTERFACE];
#define IP_HANDLE(interface)                     (cy_lwip_ip_handle[(interface) & 3])

#define MAX_AUTO_IP_RETRIES                      (5)

/******************************************************
 *               Variable Definitions
 ******************************************************/
static cy_network_activity_event_callback_t activity_callback = NULL;
static bool is_dhcp_client_required = false;
static cy_eapol_packet_handler_t internal_eapol_packet_handler = NULL;
static cy_lwip_ip_change_callback_t ip_change_callback = NULL;

#if LWIP_IPV4
static cy_lwip_dhcp_server_t internal_dhcp_server;
#endif

static struct netif       sta_ip_handle;
static struct netif       ap_ip_handle;

struct netif* cy_lwip_ip_handle[MAX_NW_INTERFACE] =
{
    [CY_LWIP_STA_NW_INTERFACE] =  &sta_ip_handle,
    [CY_LWIP_AP_NW_INTERFACE]  =  &ap_ip_handle
};

/* Interface init status */
bool ip_networking_inited[MAX_NW_INTERFACE];
#define SET_IP_NETWORK_INITED(interface, status)   (ip_networking_inited[(interface)&3] = status)

/* IP UP status */
bool ip_up[MAX_NW_INTERFACE];
#define SET_IP_UP(interface, status)               (ip_up[(interface)&3] = status)

/******************************************************
 *               Static Function Declarations
 ******************************************************/
static void internal_ip_change_callback (struct netif *netif);
#if LWIP_IPV4
static void invalidate_all_arp_entries(struct netif *netif);
#endif
static bool is_interface_added(cy_lwip_nw_interface_role_t role);
static cy_rslt_t is_interface_valid(cy_lwip_nw_interface_t *iface);
static bool is_network_up(cy_lwip_nw_interface_role_t role);
/******************************************************
 *               Function Definitions
 ******************************************************/
/*
 * This function takes packets from the radio driver and passes them into the
 * LwIP stack.  If the stack is not initialized, or if the LwIP stack does not
 * accept the packet, the packet is freed (dropped). If packet is of type EAPOL
 * and if EAPOL handler is registered, packet will be redirected to registered
 * handler and should be freed by EAPOL handler.
 */
void cy_network_process_ethernet_data(whd_interface_t iface, whd_buffer_t buf)
{
    uint8_t *data = whd_buffer_get_current_piece_data_pointer(iface->whd_driver, buf);
    uint16_t ethertype;
    struct netif *net_interface = NULL;

    if(iface->role == WHD_STA_ROLE)
    {
        net_interface = &sta_ip_handle;
    }
    else if(iface->role == WHD_AP_ROLE)
    {
        net_interface = &ap_ip_handle;
    }
    else
    {
        cy_buffer_release(buf, WHD_NETWORK_RX) ;
        return;
    }

    ethertype = (uint16_t)(data[12] << 8 | data[13]);
    if (ethertype == EAPOL_PACKET_TYPE)
    {
        if( internal_eapol_packet_handler != NULL )
        {
            internal_eapol_packet_handler(buf, iface);
        }
        else
        {
            cy_buffer_release(buf, WHD_NETWORK_RX) ;
        }
    }
    else
    {
        /* Call activity handler which is registered with argument as false
         * indicating there is RX packet
         */
        if (activity_callback)
        {
            activity_callback(false);
        }

        if (net_interface->input(buf, net_interface) != ERR_OK)
        {
            cy_buffer_release(buf, WHD_NETWORK_RX) ;
        }
    }
}

/* This function creates duplicate pbuf of input pbuf */
static struct pbuf *pbuf_dup(const struct pbuf *orig)
{
    struct pbuf *p = pbuf_alloc(PBUF_LINK, orig->tot_len, PBUF_RAM);
    if (p != NULL)
    {
        pbuf_copy(p, orig);
        p->flags = orig->flags;
    }
    return p;
}

/*
 * This function takes packets from the LwIP stack and sends them down to the radio.
 * If the radio is not ready, we return and error, otherwise we add a reference to
 * the packet for the radio driver and send the packet to the radio driver.  The radio
 * driver puts the packet into a send queue and will send based on another thread.  This
 * other thread will release the packet reference once the packet is actually sent.
 */
static err_t wifioutput(struct netif *iface, struct pbuf *p)
{
    if (whd_wifi_is_ready_to_transceive((whd_interface_t)iface->state) != WHD_SUCCESS)
    {
        wm_cy_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "wifi is not ready, packet not sent\n");
        return ERR_INPROGRESS ;
    }

    struct pbuf *whd_buf = pbuf_dup(p);
    if (whd_buf == NULL)
    {
        wm_cy_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "failed to allocate buffer for outgoing packet\n");
        return ERR_MEM;
    }
    /* Call activity handler which is registered with argument as true
     * indicating there is TX packet
     */
    if (activity_callback)
    {
        activity_callback(true);
    }
    whd_network_send_ethernet_data((whd_interface_t)iface->state, whd_buf) ;
    return ERR_OK ;
}
#if LWIP_IPV4 && LWIP_IGMP
/*
 * This function is used to respond to IGMP (group management) requests.
 */
static err_t igmp_filter(struct netif *iface, const ip4_addr_t *group, enum netif_mac_filter_action action)
{
    whd_mac_t mac = { MULTICAST_IP_TO_MAC((uint8_t*)group) };

    switch ( action )
    {
        case NETIF_ADD_MAC_FILTER:
            if ( whd_wifi_register_multicast_address( (whd_interface_t)iface->state, &mac ) != CY_RSLT_SUCCESS )
            {
                return ERR_VAL;
            }
            break;

        case NETIF_DEL_MAC_FILTER:
            if ( whd_wifi_unregister_multicast_address( (whd_interface_t)iface->state, &mac ) != CY_RSLT_SUCCESS )
            {
                return ERR_VAL;
            }
            break;

        default:
            return ERR_VAL;
    }

    return ERR_OK;
}
#endif

#if LWIP_IPV6 && LWIP_IPV6_MLD
/*
 * This function is called by lwIP to add or delete an entry in the IPv6 multicast filter table of the ethernet MAC.
 */
static err_t mld_mac_filter(struct netif *iface, const ip6_addr_t *group, enum netif_mac_filter_action action)
{
    whd_mac_t macaddr;
    cy_rslt_t res;
    const uint8_t *ptr = (const uint8_t *)group->addr;

    /* Convert IPv6 multicast address to MAC address.
     * The first two octets of the converted MAC address are the fixed values 0x33 and 0x33.
     * And last four octets are the last four octets of the ipv6 multicast address.
     */
    macaddr.octet[0] = IPV6_MULTICAST_TO_MAC_PREFIX ;
    macaddr.octet[1] = IPV6_MULTICAST_TO_MAC_PREFIX ;
    macaddr.octet[2] = ptr[12] ;
    macaddr.octet[3] = ptr[13] ;
    macaddr.octet[4] = ptr[14] ;
    macaddr.octet[5] = ptr[15] ;

    switch ( action )
    {
        case NETIF_ADD_MAC_FILTER:
            res = whd_wifi_register_multicast_address( (whd_interface_t)iface->state, &macaddr );
            if (  res != CY_RSLT_SUCCESS )
            {
                wm_cy_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "whd_wifi_register_multicast_address call failed, err = %lx\n", res);
                return ERR_VAL;
            }
            break;

        case NETIF_DEL_MAC_FILTER:
            res = whd_wifi_unregister_multicast_address( (whd_interface_t)iface->state, &macaddr );
            if ( res != CY_RSLT_SUCCESS )
            {
                wm_cy_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "whd_wifi_unregister_multicast_address call failed, err = %lx\n", res);
                return ERR_VAL;
            }
            break;

        default:
            wm_cy_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Invalid MAC Filter Action: %d\n", action);
            return ERR_VAL;
    }

    return ERR_OK;
}
#endif

/*
 * This function is called when adding the wifi network interface to LwIP,
 * it actually performs the initialization for the netif interface.
 */
static err_t wifiinit(struct netif *iface)
{
    cy_rslt_t res;
    whd_mac_t macaddr;
    whd_interface_t whd_iface = (whd_interface_t)iface->state;

    /*
     * Set the MAC address of the interface
     */
    res = whd_wifi_get_mac_address(whd_iface, &macaddr);

    if (res != CY_RSLT_SUCCESS)
    {
        wm_cy_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "whd_wifi_get_mac_address call failed, err = %lx\n", res);
        return res ;
    }  
    memcpy(&iface->hwaddr, &macaddr, sizeof(macaddr));
    iface->hwaddr_len = sizeof(macaddr);

    /*
     * Setup the information associated with sending packets
     */
#if LWIP_IPV4
    iface->output = etharp_output;
#endif
    iface->linkoutput = wifioutput;
    iface->mtu = WHD_LINK_MTU;
    iface->flags |= (NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_IGMP) ;
#ifdef LWIP_IPV6_MLD
    iface->flags |= NETIF_FLAG_MLD6;
#endif
    /*
     * Set the interface name for the interface
     */
    iface->name[0] = 'w' ;
    iface->name[1] = 'l' ;

#if LWIP_IPV4 && LWIP_IGMP
    netif_set_igmp_mac_filter(iface, igmp_filter) ;
#endif


#if LWIP_IPV6 == 1
    /*
     * Filter output packets for IPV6 through the ethernet output
     * function for IPV6.
     */
    iface->output_ip6 = ethip6_output ;

    /*
     * Automatically generate a unicast IP address based on
     * neighbor discovery.
     */
    iface->ip6_autoconfig_enabled = 1 ;

    /*
     * Create a link local IPV6 address
     */
    netif_create_ip6_linklocal_address(iface, 1);

    /*
     * Tell the radio that we want to listen to solicited-node multicast
     * packets.  These packets are part of the IPV6 neighbor discovery
     * process.
     */
    macaddr.octet[0] = 0x33 ;
    macaddr.octet[1] = 0x33 ;
    macaddr.octet[2] = 0xff ;
    whd_wifi_register_multicast_address(iface->state, &macaddr) ;

    /*
     * Tell the radio that we want to listen to the multicast address
     * that targets all IPV6 devices.  These packets are part of the IPV6
     * neighbor discovery process.
     */
    memset(&macaddr, 0, sizeof(macaddr)) ;
    macaddr.octet[0] = 0x33 ;
    macaddr.octet[1] = 0x33 ;
    macaddr.octet[5] = 0x01 ;
    whd_wifi_register_multicast_address(iface->state, &macaddr) ;

#if LWIP_IPV6_MLD
    /*
     * Register mld mac filter callback function that will be called by lwIP to add or delete an
     * entry in the IPv6 multicast filter table of the ethernet MAC.
     */
    netif_set_mld_mac_filter(iface, mld_mac_filter);
#endif
#endif
    return 0 ;
}

/*
 * This is the main entry point in this file.  This function takes a WHD radio driver
 * handle, interface type and static IP address, and adds interface to LwIP.
 * note : static IP address is optional for STA interface.
 */
cy_rslt_t cy_lwip_add_interface(cy_lwip_nw_interface_t *iface, ip_static_addr_t *static_ipaddr)
{
#if LWIP_IPV4
    ip4_addr_t ipaddr, netmask, gateway ;
#endif

    if(is_interface_valid(iface) != CY_RSLT_SUCCESS)
    {
        return CY_RSLT_LWIP_BAD_ARG;
    }

    /* static IP addr is mandatory for AP */
    if((iface->role == CY_LWIP_AP_NW_INTERFACE) && (static_ipaddr == NULL))
    {
        wm_cy_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Error static IP addr cannot be NULL for AP interface \n");
        return CY_RSLT_LWIP_BAD_ARG;
    }

    if (is_interface_added(iface->role))
    {
        return CY_RSLT_SUCCESS;
    }

#if LWIP_IPV4
    /* Assign the IP address if static, otherwise, zero the IP address */
    if (static_ipaddr != NULL)
    {
        memcpy(&gateway, &static_ipaddr->gateway, sizeof(gateway));
        memcpy(&ipaddr, &static_ipaddr->addr, sizeof(ipaddr));
        memcpy(&netmask, &static_ipaddr->netmask, sizeof(netmask));
    }
    else
    {
        memset(&gateway, 0, sizeof(gateway));
        memset(&ipaddr, 0, sizeof(ipaddr));
        memset(&netmask, 0, sizeof(netmask));
    }

    /* Add the interface to LwIP and make it the default */
    if(netifapi_netif_add((IP_HANDLE(iface->role)), &ipaddr, &netmask, &gateway, iface->whd_iface, wifiinit, tcpip_input) != CY_RSLT_SUCCESS)
    {
        wm_cy_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Error adding interface \n");
        return CY_RSLT_LWIP_ERROR_ADDING_INTERFACE;
    }
#else
    if(netifapi_netif_add((IP_HANDLE(iface->role)), iface->whd_iface, wifiinit, tcpip_input) != CY_RSLT_SUCCESS)
    {
        wm_cy_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Error adding interface \n");
        return CY_RSLT_LWIP_ERROR_ADDING_INTERFACE;
    }
#endif

    if(iface->role == CY_LWIP_STA_NW_INTERFACE)
    {
        if(static_ipaddr == NULL)
        {
            is_dhcp_client_required = true;
        }
        netifapi_netif_set_default(IP_HANDLE(iface->role)) ;
    }

    /*
     * Register a handler for any address changes
     * Note : The "status" callback will also be called when the interface
     * goes up or down
     */
    netif_set_status_callback((IP_HANDLE(iface->role)), internal_ip_change_callback);
    SET_IP_NETWORK_INITED(iface->role, true);

    return CY_RSLT_SUCCESS ;
}

struct netif* cy_lwip_get_interface(cy_lwip_nw_interface_role_t role)
{
    if((role != CY_LWIP_AP_NW_INTERFACE) && (role != CY_LWIP_STA_NW_INTERFACE))
    {
        return NULL;
    }
    return (IP_HANDLE(role));
}

cy_rslt_t cy_lwip_remove_interface(cy_lwip_nw_interface_t *iface)
{
    if(is_interface_valid(iface) != CY_RSLT_SUCCESS)
    {
        return CY_RSLT_LWIP_BAD_ARG;
    }

    /* Interface can be removed only if the interface was previously added and network is down */
    if(is_interface_added(iface->role))
    {
        wm_cy_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Error Interface doesn't exist \n");
        return CY_RSLT_LWIP_INTERFACE_DOES_NOT_EXIST;
    }

    if(is_network_up(iface->role))
    {
        wm_cy_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Error removing interface, bring down the network before removing the interface \n");
        return CY_RSLT_LWIP_ERROR_REMOVING_INTERFACE;
    }
    /* remove status callback */
    netif_set_remove_callback(IP_HANDLE(iface->role), internal_ip_change_callback);
    /* remove the interface */
    netifapi_netif_remove(IP_HANDLE(iface->role));
    if(iface->role == CY_LWIP_STA_NW_INTERFACE)
    {
        is_dhcp_client_required = false;
    }

    SET_IP_NETWORK_INITED(iface->role, false);
    return CY_RSLT_SUCCESS;
}

cy_rslt_t cy_lwip_network_up(cy_lwip_nw_interface_t *iface)
{
    cy_rslt_t result                     = CY_RSLT_SUCCESS;
    uint32_t  address_resolution_timeout = 0;
    bool      timeout_occurred           = false;

#if LWIP_IPV4
    ip4_addr_t ip_addr;
#endif

    if(is_interface_valid(iface) != CY_RSLT_SUCCESS)
    {
        return CY_RSLT_LWIP_BAD_ARG;
    }

    if(is_network_up(iface->role))
    {
        wm_cy_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "Network is already up \n");
        return CY_RSLT_SUCCESS;
    }

    /*
    * Bring up the network interface
    */
    netifapi_netif_set_up((IP_HANDLE(iface->role)));

    /*
    * Bring up the network link layer
    */
    netifapi_netif_set_link_up((IP_HANDLE(iface->role)));

#if LWIP_IPV6
    /* Wait for IPV6 address to change from tentative to valid or invalid */
    while(ip6_addr_istentative(netif_ip6_addr_state(IP_HANDLE(iface->role), 0)))
    {
        /* Give LwIP time to change the state */
        cy_rtos_delay_milliseconds(ND6_TMR_INTERVAL);
    }

    /* LWIP changes state to either INVALID or VALID. Check if the state is VALID */
    if(ip6_addr_isvalid(netif_ip6_addr_state(IP_HANDLE(iface->role), 0)))
    {
        wm_cy_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "IPv6 Network ready IP: %s \r\n", ip6addr_ntoa(netif_ip6_addr(IP_HANDLE(iface->role), 0)));
    }
    else
    {
        wm_cy_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "IPv6 network not ready \r\n");
    }
#endif

#if LWIP_IPV4
    if(iface->role == CY_LWIP_STA_NW_INTERFACE)
    {
        if(is_dhcp_client_required)
        {
            /* TO DO :  Save the current power save state */
            /* TO DO :  Disable power save for the DHCP exchange */

            /*
             * For DHCP only, we should reset netif IP address
             * We don't want to re-use previous netif IP address
             * given from previous DHCP session
             */
            ip4_addr_set_zero(&ip_addr);
            netif_set_ipaddr(IP_HANDLE(iface->role), &ip_addr);

            /* TO DO : DHCPV6 need to be handled when we support IPV6 addresses other than the link local address */
            /* Start DHCP */
            if(netifapi_dhcp_start(IP_HANDLE(iface->role)) != CY_RSLT_SUCCESS)
            {
                return CY_RSLT_LWIP_ERROR_STARTING_DHCP;
            }
            /* Wait a little to allow DHCP a chance to complete */

            while((netif_dhcp_data(IP_HANDLE(iface->role))->state != DHCP_STATE_BOUND) && (timeout_occurred == false))
            {
                cy_rtos_delay_milliseconds(10);
                address_resolution_timeout += 10;
                if(address_resolution_timeout >= DHCP_IP_ADDRESS_RESOLUTION_TIMEOUT_IN_MS)
                {
                    /* timeout has occurred */
                    timeout_occurred = true;
                }
            }

            if (timeout_occurred)
            {
                netifapi_dhcp_release_and_stop(IP_HANDLE(iface->role));
#if LWIP_AUTOIP
                int   tries = 0;
                wm_cy_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "Unable to obtain IP address via DHCP. Perform Auto IP\n");
                address_resolution_timeout = 0;
                timeout_occurred            = false;

                if (autoip_start(IP_HANDLE(iface->role) ) != ERR_OK )
                {
                    /* trick: skip the while-loop, do the cleaning up stuff */
                    timeout_occurred = true;
                }

                while ((timeout_occurred == false) && ((netif_autoip_data(IP_HANDLE(iface->role))->state != AUTOIP_STATE_BOUND)))
                {
                    cy_rtos_delay_milliseconds(10);
                    address_resolution_timeout += 10;
                    if(address_resolution_timeout >= AUTO_IP_ADDRESS_RESOLUTION_TIMEOUT_IN_MS)
                    {
                        if(tries++ < MAX_AUTO_IP_RETRIES)
                        {
                            address_resolution_timeout = 0;
                        }
                        else
                        {
                            timeout_occurred = true;
                        }
                     }
                }

                if (timeout_occurred)
                {
                    wm_cy_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Unable to obtain IP address via DCHP and AutoIP\n");
                    autoip_stop(IP_HANDLE(iface->role));
                    return CY_RSLT_LWIP_DHCP_WAIT_TIMEOUT;
                }
                else
                {
                    wm_cy_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG, "IP address obtained through AutoIP \n");
                }
#else
                wm_cy_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "Unable to obtain IP address via DHCP\n");
                return CY_RSLT_LWIP_DHCP_WAIT_TIMEOUT;
#endif
            }
        }
    }
    else
    {
        memset(&internal_dhcp_server, 0, sizeof(internal_dhcp_server));
        igmp_start(IP_HANDLE(iface->role));
        /* Start internal DHCP server */
        if((result = cy_lwip_dhcp_server_start(&internal_dhcp_server, iface->role))!= CY_RSLT_SUCCESS)
        {
            return CY_RSLT_LWIP_ERROR_STARTING_DHCP;
        }
    }
#endif

    SET_IP_UP(iface->role, true);
    return result;
}

cy_rslt_t cy_lwip_network_down(cy_lwip_nw_interface_t *iface)
{
    if(is_interface_valid(iface) != CY_RSLT_SUCCESS)
    {
        return CY_RSLT_LWIP_BAD_ARG;
    }

    if(!is_network_up(iface->role))
    {
        wm_cy_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "Network is not UP \r\n");
        return CY_RSLT_LWIP_INTERFACE_NETWORK_NOT_UP;
    }

#if LWIP_IPV4
    if(is_dhcp_client_required)
    {
#if LWIP_AUTOIP
        if(netif_autoip_data(IP_HANDLE(iface->role))->state == AUTOIP_STATE_BOUND)
        {
            autoip_stop(IP_HANDLE(iface->role));
        }
        else
#endif
        {
            netifapi_dhcp_release_and_stop(IP_HANDLE(iface->role));
            cy_rtos_delay_milliseconds(DHCP_STOP_DELAY_IN_MS);
            dhcp_cleanup(IP_HANDLE(iface->role));
        }
    }

    if(iface->role == CY_LWIP_AP_NW_INTERFACE)
    {
        /* Stop internal dhcp server for SoftAP interface */
        cy_lwip_dhcp_server_stop(&internal_dhcp_server);
    }
#endif
    /*
    * Bring down the network link layer
    */
    netifapi_netif_set_link_down(IP_HANDLE(iface->role));

    /*
    * Bring down the network interface
    */
    netifapi_netif_set_down(IP_HANDLE(iface->role));

    /* TO DO : clear all ARP cache */

    /** TO DO:
     *  Kick the radio chip if it's in power save mode in case the link down event is due to missing beacons.
     *  Setting the chip to the same power save mode is sufficient.
     */
    SET_IP_UP(iface->role, false);
    return CY_RSLT_SUCCESS;
}

void cy_lwip_register_ip_change_cb(cy_lwip_ip_change_callback_t cb)
{
    ip_change_callback = cb;
}

/*
 * This functions helps to register/unregister callback for network activity
 */
void cy_network_activity_register_cb(cy_network_activity_event_callback_t cb)
{
    /* update the activity callback with the argument passed */
    activity_callback = cb;
}
#if LWIP_IPV4
cy_rslt_t cy_lwip_dhcp_renew(cy_lwip_nw_interface_t *iface)
{
    if(is_interface_valid(iface) != CY_RSLT_SUCCESS)
    {
        return CY_RSLT_LWIP_BAD_ARG;
    }
    /* Invalidate ARP entries */
    netifapi_netif_common(IP_HANDLE(iface->role), (netifapi_void_fn) invalidate_all_arp_entries, NULL );

    /* DHCP renewal*/
    netifapi_netif_common(IP_HANDLE(iface->role), (netifapi_void_fn)dhcp_renew, NULL);

    cy_rtos_delay_milliseconds(DCHP_RENEWAL_DELAY_IN_MS);
    return CY_RSLT_SUCCESS;
}

/**
 * Remove all ARP table entries of the specified netif.
 * @param netif points to a network interface
 */
static void invalidate_all_arp_entries(struct netif *netif)
{
     /*free all the entries in arp list */
    etharp_cleanup_netif(netif);
}
#endif

/* Used to register callback for EAPOL packets */
whd_result_t cy_eapol_register_receive_handler( cy_eapol_packet_handler_t eapol_packet_handler )
{
    internal_eapol_packet_handler = eapol_packet_handler;
    return WHD_SUCCESS;
}

static void internal_ip_change_callback (struct netif *netif)
{
    /* notify wcm about ip change */
    if(ip_change_callback != NULL)
    {
        ip_change_callback(NULL);
    }
}

static bool is_interface_added(cy_lwip_nw_interface_role_t role)
{
    return (ip_networking_inited[((uint8_t)role)&3]);
}

static bool is_network_up(cy_lwip_nw_interface_role_t role)
{
    return (ip_up[((uint8_t)role)&3]);
}

static cy_rslt_t is_interface_valid(cy_lwip_nw_interface_t *iface)
{
    if((iface == NULL) || ((iface->role != CY_LWIP_AP_NW_INTERFACE) && (iface->role != CY_LWIP_STA_NW_INTERFACE)))
    {
        return CY_RSLT_LWIP_BAD_ARG;
    }

    return CY_RSLT_SUCCESS;
}
