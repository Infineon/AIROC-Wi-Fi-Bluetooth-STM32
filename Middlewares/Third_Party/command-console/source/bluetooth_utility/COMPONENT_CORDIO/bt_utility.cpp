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

/** @file
 *
 */
#ifndef DISABLE_COMMAND_CONSOLE_BT
#include <events/mbed_events.h>
#include <mbed.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "ble/BLE.h"
#include "ble/Gap.h"
#include "command_console.h"
#include "l2c_api.h"
#include "l2c_handler.h"
#include "gap/AdvertisingDataParser.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/
#define BT_LE_INFO(X)             printf X
#define BT_LE_DEBUG(X)            //printf X
#define BT_LE_ERROR(X)            printf X

#ifndef BLE_COC_MTU_SIZE
#define BLE_COC_MTU_SIZE               (100)
#endif
#ifndef BLE_CONNECTION_INTERVAL
#define BLE_CONNECTION_INTERVAL        (40)
#endif
#define LE_COC_NAME_LEN                (6)
#define L2CAP_COC_PSM_USED             (19)
#define LE_DEFAULT_SUPERVISION_TIMEOUT (100)

#if(BLE_COC_MTU_SIZE > 512)
#error Maximum value of BLE_COC_MTU_SIZE that can be configured is 512
#endif

#if defined(__ICCARM__)
#define CORDIO_BT_WEAK_FUNC            __WEAK
#elif defined(__GNUC__) || defined(__clang__) || defined(__CC_ARM)
#define CORDIO_BT_WEAK_FUNC            __attribute__((weak))
#else
#define CORDIO_BT_WEAK_FUNC           __attribute__((weak))
#endif

/******************************************************
 *                    Constants
 ******************************************************/
// LE COC connection pending or not
static bool le_coc_connect_pending = false;

// LE COC connection ID
static uint16_t le_coc_conn_id = DM_CONN_ID_NONE;

// LE COC MTU
static int le_coc_mtu = BLE_COC_MTU_SIZE;

// LE COC PSM
static int le_coc_psm = L2CAP_COC_PSM_USED;

// LE COC Register ID
static l2cCocRegId_t l2c_coc_regId = L2C_COC_REG_ID_NONE;

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/
static EventQueue event_queue(/* event count */ 10 * EVENTS_EVENT_SIZE);
/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/
int handle_ble_on( int argc, char *argv[], tlv_buffer_t** data );
int handle_ble_off( int argc, char *argv[], tlv_buffer_t** data );
int handle_start_scan( int argc, char *argv[], tlv_buffer_t** data );
int handle_stop_scan( int argc, char *argv[], tlv_buffer_t** data );
int handle_start_adv( int argc, char *argv[], tlv_buffer_t** data );
int handle_stop_adv( int argc, char *argv[], tlv_buffer_t** data );
int handle_connect( int argc, char *argv[], tlv_buffer_t** data );
int handle_disconnect( int argc, char *argv[], tlv_buffer_t** data );
int handle_coc_init( int argc, char *argv[], tlv_buffer_t** data );
int handle_coc_adv( int argc, char *argv[], tlv_buffer_t** data );
int handle_coc_scan_connect( int argc, char *argv[], tlv_buffer_t** data );
int handle_coc_disconnect( int argc, char *argv[], tlv_buffer_t** data );
int handle_coc_send_data( int argc, char *argv[], tlv_buffer_t** data );
int handle_coc_stop_data( int argc, char *argv[], tlv_buffer_t** data );
int handle_coc_throughput( int argc, char *argv[], tlv_buffer_t** data );
int handle_bt_get_device_address( int argc, char *argv[], tlv_buffer_t** data );
/******************************************************
 *               Variables Definitions
 ******************************************************/
#define BT_COEX_COMMANDS \
    { "bt_on"  ,               handle_ble_on,           0, NULL, NULL, (char*) "",   (char*) "Turn on BLE \n\t [NOTE]  Command Not Supported, BLE is turned On as part of during application init"}, \
    { "bt_off"  ,              handle_ble_off,          0, NULL, NULL, (char*) "",   (char*) "Turn off BLE \n\t [NOTE] Command Not Supported"}, \
    { "bt_get_device_address", handle_bt_get_device_address,  0, NULL, NULL, (char*) "", (char*) "Get Bluetooth Device Address" }, \
    { "ble_start_scan",        handle_start_scan,       0, NULL, NULL, (char*) "",   (char*) "Start BLE Scan"}, \
    { "ble_stop_scan",         handle_stop_scan,        0, NULL, NULL, (char*) "",   (char*) "Stop BLE Scan"}, \
    { "ble_start_adv",         handle_start_adv,        0, NULL, NULL, (char*) "",   (char*) "Start Advertisement"}, \
    { "ble_stop_adv",          handle_stop_adv,         0, NULL, NULL, (char*) "",   (char*) "Stop  Advertisement"}, \
    { "ble_coc_init",          handle_coc_init,         0, NULL, NULL, (char*) "",   (char*) "Initializes LE COC with PSM 19 and MTU 100"}, \
    { "ble_coc_adv",           handle_coc_adv,          0, NULL, NULL, (char*) "",   (char*) "Start LE COC advertisements"}, \
    { "ble_coc_scan_connect",  handle_coc_scan_connect, 0, NULL, NULL, (char*) "",   (char*) "Scan and Connect to a LE COC server"}, \
    { "ble_coc_disconnect",    handle_coc_disconnect,   0, NULL, NULL, (char*) "",   (char*) "Disconnect LE COC"}, \
    { "ble_coc_send_start",    handle_coc_send_data,    0, NULL, NULL, (char*) "",   (char*) "Start Sending LE COC data"}, \
    { "ble_coc_send_stop",     handle_coc_stop_data,    0, NULL, NULL, (char*) "",   (char*) "Stop Sending LE COC data"}, \
    { "ble_get_throughput",    handle_coc_throughput,   0, NULL, NULL, (char*) "",   (char*) "Get LE COC Throughput"}, \

const cy_command_console_cmd_t bt_coex_command_table[] =
{
    BT_COEX_COMMANDS
    CMD_TABLE_END
};

// Default LE COC Registration param structure.
l2cCocReg_t l2c_coc_reg = {
    .psm = L2CAP_COC_PSM_USED,
    .mps = BLE_COC_MTU_SIZE,
    .mtu = BLE_COC_MTU_SIZE,
    .credits = 4,
    .authoriz = 0,
    .secLevel = 0,
    .role = L2C_COC_ROLE_ACCEPTOR
};

// Data Tx state.
static bool le_coc_send_data = false;

// Total No of bytes sent
static uint64_t total_sent_data = 0;
// Timer for Tx
mbed::Timer mbed_sent_data_timer;

// Total No of bytes received
static uint64_t total_received_data = 0;
// Timer for Rx
mbed::Timer mbed_received_data_timer;

// BLE instant class.
class BLECoexTest : ble::Gap::EventHandler {
public:
    BLECoexTest(BLE &ble, events::EventQueue &event_queue) :
        _ble(ble),
        _event_queue(event_queue)
         {  }

    // Initialize BLE.
    void start() {
        _ble.gap().setEventHandler(this);
        _ble.init(this, &BLECoexTest::on_init_complete);
        _event_queue.dispatch_forever();
    }

    // Shut down BLE.
    bool stop() {
        if (_ble.hasInitialized())
        {
            if (_ble.shutdown() == BLE_ERROR_NONE)
                return true;
        }
        return false;
    }

    // Start/stop scan.
    bool start_scan(int start) {
        if (!_ble.hasInitialized())
            return false;
        if (start)
        {
            // Start scanning.
            if ( _ble.gap().startScan(ble::scan_duration_t(0),ble::duplicates_filter_t::ENABLE, ble::scan_period_t(0)) == BLE_ERROR_NONE )
                 return true;
        }
        else
        {
            //Stop scanning.
            if ( _ble.gap().stopScan() == BLE_ERROR_NONE )
                return true;
        }
        return false;
    }

    // Start/stop adv.
    bool start_adv(int start) {
        if (!_ble.hasInitialized())
            return false;
        if (start)
        {

            ble_error_t error;

            ble::AdvertisingParameters adv_parameters(
                ble::advertising_type_t::CONNECTABLE_UNDIRECTED,
                true
            );

            adv_parameters.setOwnAddressType(ble::own_address_type_t :: PUBLIC);

            error = _ble.gap().setAdvertisingParameters(
                ble::LEGACY_ADVERTISING_HANDLE,
                adv_parameters
            );

            // Set advertising data.
            ble::AdvertisingDataBuilder adv_data_builder(_adv_buffer);
            adv_data_builder.setFlags();
            adv_data_builder.setName("LE CoC", false);
            error = _ble.gap().setAdvertisingPayload(
                    ble::LEGACY_ADVERTISING_HANDLE,
                    adv_data_builder.getAdvertisingData()
                    );
            if (error) {
                BT_LE_ERROR(("_ble.gap().setAdvertisingPayload() failed"));
                return false;
            }

            // Start advertisement.
            if ( _ble.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE) == BLE_ERROR_NONE )
                 return true;
        }
        else
        {
            //Stop advertisement.
            if ( _ble.gap().stopAdvertising(ble::LEGACY_ADVERTISING_HANDLE) == BLE_ERROR_NONE )
                return true;
        }
        return false;
    }

    bool connect()
    {
        BT_LE_INFO(("scanning and connecting to LE CoC server \n"));
        start_scan(1);
        return true;
    }

    // Disconnect
    bool disconnect(int handle)
    {
        if ( _ble.gap().disconnect(handle, ble::local_disconnection_reason_t::USER_TERMINATION) == BLE_ERROR_NONE )
            return true;
        return false;
    }

    /** Callback triggered when the BLE initialization process has finished. */
    void on_init_complete(BLE::InitializationCompleteCallbackContext *params) {
        if (params->error != BLE_ERROR_NONE) {
            BT_LE_ERROR(("Ble initialization failed."));
            return;
        }
        BT_LE_INFO(("Bluetooth Initialization Completed \n"));
    }
private:
    virtual void onAdvertisingReport(const ble::AdvertisingReportEvent &event)
    {
        ble::address_t addr(event.getPeerAddress());
        ble::AdvertisingDataParser adv_parser(event.getPayload());
        bool le_coc_device_found = false;
        char adv_name[LE_COC_NAME_LEN] = "";

        BT_LE_INFO(("Found Device: %02X:%02X:%02X:%02X:%02X:%02X \t RSSI: %d\n",
                addr[5], addr[4], addr[3], addr[2], addr[1], addr[0], event.getRssi()));

        if(!le_coc_connect_pending)
        {
            /* For scan request, no need to parse and connect. */
            return;
        }

        /* Parse the advertising payload, looking for a discoverable device. */
        while (adv_parser.hasNext())
        {
            ble::AdvertisingDataParser::element_t field = adv_parser.next();

            /* skip non discoverable device */
            if (field.type == ble::adv_data_type_t::SHORTENED_LOCAL_NAME)
            {
                if(field.value.size() == LE_COC_NAME_LEN)
                {
                    for(int i=0; i<LE_COC_NAME_LEN; i++)
                    {
                        adv_name[i] = field.value[i];
                    }
                }

                if (strncmp (adv_name,"LE CoC",LE_COC_NAME_LEN) == 0)
                {
                    BT_LE_INFO(("Found LE COC Server \n"));
                    le_coc_device_found = true;
                    break;
                }
            }
        }

        if(le_coc_device_found)
         {
             _ble.gap().stopScan();
             ble::phy_t phy = ble::phy_t::LE_1M;
             ble::conn_interval_t minConnectionInterval = ble::conn_interval_t(BLE_CONNECTION_INTERVAL);
             ble::conn_interval_t maxConnectionInterval = ble::conn_interval_t(BLE_CONNECTION_INTERVAL);
             ble::slave_latency_t slaveLatency = ble::slave_latency_t::min();
             ble::supervision_timeout_t connectionSupervisionTimeout = ble::supervision_timeout_t(LE_DEFAULT_SUPERVISION_TIMEOUT);
             ble::conn_event_length_t minEventLength = ble::conn_event_length_t::min();
             ble::conn_event_length_t maxEventLength = ble::conn_event_length_t::max();
             if (_ble.gap().connect(event.getPeerAddressType(), event.getPeerAddress(),
                  ble::ConnectionParameters().setConnectionParameters(phy, minConnectionInterval, maxConnectionInterval, slaveLatency, connectionSupervisionTimeout, minEventLength, maxEventLength)) == BLE_ERROR_NONE)
             {
                     BT_LE_INFO(("gap().connect is successfull \n"));
             }
         }
    }
    virtual void onConnectionComplete(const ble::ConnectionCompleteEvent &event)
    {
        if (event.getStatus() == BLE_ERROR_NONE) {
            BT_LE_INFO(("Connected  \n"));
            BT_LE_INFO(("Handle 0x%x Connection interval 0x%x connection latency 0x%x Supervision timeout 0x%x \n", event.getConnectionHandle(),
                            (unsigned int)event.getConnectionInterval().value(),
                            (unsigned int)event.getConnectionLatency().value(),
                            (unsigned int)event.getSupervisionTimeout().value()));

            if(l2c_coc_reg.role == L2C_COC_ROLE_ACCEPTOR)
            {
                BT_LE_INFO(("le coc role is acceptor \n"));
                return;
            }

            // Check if LE COC connection is pending.
            if (le_coc_connect_pending && (le_coc_conn_id==L2C_COC_CID_NONE) )
            {
                // Assume we are getting dm_id 1.
                int dm_id = 1;
                le_coc_conn_id = L2cCocConnectReq(dm_id, l2c_coc_regId, le_coc_psm);
                if (le_coc_conn_id == L2C_COC_CID_NONE)
                {
                    BT_LE_ERROR(("Failed to connect LE COC \n"));
                }
                le_coc_connect_pending = false;
            }
        }
        else
        {
            BT_LE_INFO(("Failed to connect \n"));
        }
    }
    virtual void onDisconnectionComplete(const ble::DisconnectionCompleteEvent &event)
    {
        BT_LE_INFO(("Disconnected Handle 0x%x Reason 0x%x \n",event.getConnectionHandle(), event.getReason().value()));
    }
private:
    BLE &_ble;
    events::EventQueue &_event_queue;
    uint8_t _adv_buffer[ble::LEGACY_ADVERTISING_MAX_SIZE];
};

BLECoexTest *test_coex;

/******************************************************
 *               Function Definitions
 ******************************************************/
// Convert ASCII decimal string to integer.
int ascii_dec_to_int(char *str)
{
    int len = strlen(str);
    int total = 0;
    for (int i=0;i<len;i++)
    {
        if ( str[i] >= '0' && str[i] <= '9')
        {
            total = ((total*10) + (str[i]-'0'));
        }
        else
        {
            return -1;
        }
    }
    return total;
}

/** Schedule processing of events from the BLE middleware in the event queue. */
void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context) {
    event_queue.call(Callback<void()>(&context->ble, &BLE::processEvents));
}
int handle_ble_on( int argc, char *argv[], tlv_buffer_t** data )
{
    BT_LE_INFO(("command not supported - bluetooth is turned on during application init \n"));
    return 0;
}

int handle_ble_off( int argc, char *argv[], tlv_buffer_t** data )
{
    test_coex->stop();
    return 0;
}

int handle_stop_scan( int argc, char *argv[], tlv_buffer_t** data )
{
    bool status;
    BT_LE_INFO(("Stop scan operation \n"));
    status = test_coex->start_scan(0);
    BT_LE_INFO(("[handle_stop_scan] status %d \n", status));
    return 0;
}

int handle_start_scan( int argc, char *argv[], tlv_buffer_t** data )
{
    bool status;

    BT_LE_INFO(("Starting scan operation \n"));
    status = test_coex->start_scan(1);
    BT_LE_INFO(("Start Scan, result = %d\n", status));

    return 0;
}

int handle_start_adv( int argc, char *argv[], tlv_buffer_t** data )
{
    bool status;

    BT_LE_INFO(("Starting advertisement \n"));
    status = test_coex->start_adv(1);
    BT_LE_INFO(("Start advertisement,  result =  %d\n", status));

    return 0;
}

int handle_stop_adv( int argc, char *argv[], tlv_buffer_t** data )
{
    bool status;

    BT_LE_INFO(("Stopping advertisement \n"));
    status = test_coex->start_adv(0);
    BT_LE_INFO(("Stop advertisement, result = %d\n", status));

    return 0;
}

int handle_connect( int argc, char *argv[], tlv_buffer_t** data )
{
    bool status;

    status = test_coex->connect();
    BT_LE_INFO(("[handle_connect] status %d \n", status));

    return 0;
}

int handle_disconnect( int argc, char *argv[], tlv_buffer_t** data )
{
    bool status;
    int handle_len = strlen(argv[1]);
    int handle_value = 0;
    if ( handle_len <= 0 || handle_len > 4 )
    {
        BT_LE_ERROR(("[handle_disconnect] Invalid Handle value %s \n",argv[1]));
        return -1;
    }

    for (int i=0; i<handle_len; i++)
    {
        char input = argv[1][i];
        uint8_t curr_val = 0;
        if ( (input >= '0') && (input <= '9'))
        {
            curr_val = input-'0';
        }
        else if ( (input >= 'a') && (input <= 'f') )
        {
            curr_val = input-'a';
        }
        else if ( (input >= 'A') && (input <= 'F') )
        {
            curr_val = input-'A';
        }
        handle_value += ( curr_val<< ((handle_len-i-1)*4) );
    }
    status = test_coex->disconnect(handle_value);
    BT_LE_INFO(("[handle_disconnect] status %d \n", status));
    return 0;
}

void generate_send_coc_data(void)
{
    uint8_t data[1024];
    int     index = 0;
    char    ch = 'a';
    for (index = 0; index < l2c_coc_reg.mtu; index++)
    {
        data[index] = ch;
        if (ch == 'z')
        {
            ch = 'a';
        }
        else
        {
            ch++;
        }
    }
    //BTLE_INFO(("Sending data of length %d to %d cid \n",l2c_coc_reg.mtu, le_coc_conn_id));
    total_sent_data += l2c_coc_reg.mtu;
    L2cCocDataReq(le_coc_conn_id, l2c_coc_reg.mtu, data);
}

// LE COC event callback.
void l2c_coc_cb (l2cCocEvt_t *msg)
{
    //BT_LE_INFO(("[l2c_coc_cb] param %d status %d event %d \n",msg->hdr.param, msg->hdr.status, msg->hdr.event));
    switch(msg->hdr.event)
    {
        case L2C_COC_CONNECT_IND:
        {
            BT_LE_INFO(("[L2C_COC_CONNECT_IND] Connection Id : %d \n", msg->connectInd.cid));
            le_coc_conn_id = msg->connectInd.cid;
        }
        break;
        case L2C_COC_DISCONNECT_IND:
        {
            BT_LE_INFO(("[L2C_COC_DISCONNECT_IND] Disconnection Id : %d Reason %d \n", msg->disconnectInd.cid, msg->disconnectInd.result));
            le_coc_conn_id = DM_CONN_ID_NONE;
        }
        break;
        case L2C_COC_DATA_IND:
        {
            BT_LE_INFO(("."));
            // Check if its first received packet, reset and start timer.
            if (total_received_data == 0)
            {
                mbed_received_data_timer.reset();
                mbed_received_data_timer.start();
            }
            total_received_data+= msg->dataInd.dataLen;
            BT_LE_INFO(("total_received_data = %llu \n", total_received_data));
        }
        break;
        case L2C_COC_DATA_CNF:
        {
            BT_LE_INFO(("+"));
            if((msg->hdr.status == L2C_COC_DATA_SUCCESS) && le_coc_send_data == true)
            {
                // Send again if data send is success.
                generate_send_coc_data();
            }
        }
        break;
        default:
            BT_LE_INFO(("[l2c_coc_cb] Unhandled event %d \n", msg->hdr.event));
        break;
    }
}

int handle_coc_init( int argc, char *argv[], tlv_buffer_t** data )
{
    L2cCocInit();

    l2c_coc_reg.mtu = le_coc_mtu;
    l2c_coc_reg.psm = le_coc_psm;

    HciSetMaxRxAclLen (l2c_coc_reg.mtu + 5);
    {
        wsfHandlerId_t handlerId = WsfOsSetNextHandler(L2cCocHandler);
        BT_LE_DEBUG(("Handler Id %d \n", handlerId));
        L2cCocHandlerInit(handlerId);
    }

    BT_LE_INFO(("LE COC initalized local device MTU = %d and PSM = %d \n", l2c_coc_reg.mtu, l2c_coc_reg.psm));
    return 0;
}

int handle_coc_adv( int argc, char *argv[], tlv_buffer_t** data )
{
    bool status;
    if (l2c_coc_regId != L2C_COC_REG_ID_NONE)
    {
        BT_LE_DEBUG(("Deregistering ...\n"));
        L2cCocDeregister(l2c_coc_regId);
        l2c_coc_regId = L2C_COC_REG_ID_NONE;
    }

    l2c_coc_reg.role = L2C_COC_ROLE_ACCEPTOR;
    l2c_coc_regId = L2cCocRegister(&l2c_coc_cb, &l2c_coc_reg);
    if ( l2c_coc_regId == L2C_COC_REG_ID_NONE)
    {
        BT_LE_ERROR((" FAILED to register L2CAP COC \n"));
        return -1;
    }
    BT_LE_INFO(("Register L2CAP COC as ACCEPTOR \n"));

    status = test_coex->start_adv(1);
    BT_LE_INFO(("[handle_coc_adv] status %d %d \n", status, HciGetMaxRxAclLen()));

    return 0;
}

int handle_coc_scan_connect( int argc, char *argv[], tlv_buffer_t** data )
{
    if (l2c_coc_regId != L2C_COC_REG_ID_NONE )
    {
        BT_LE_DEBUG(("Deregistering ...\n"));
        L2cCocDeregister(l2c_coc_regId);
        l2c_coc_regId = L2C_COC_REG_ID_NONE;
    }

    l2c_coc_reg.role = L2C_COC_ROLE_INITIATOR;
    l2c_coc_regId = L2cCocRegister(l2c_coc_cb, &l2c_coc_reg);
    le_coc_connect_pending = true;

    return handle_connect(argc,argv,data);
}

int handle_coc_disconnect( int argc, char *argv[], tlv_buffer_t** data )
{
    L2cCocDisconnectReq(le_coc_conn_id);
    return 0;
}

int handle_coc_send_data( int argc, char *argv[], tlv_buffer_t** data )
{
    BT_LE_INFO(("handle_coc_send_data \n"));
    if (le_coc_send_data != true)
    {
        total_sent_data = 0;
        le_coc_send_data = true;
        generate_send_coc_data();
        mbed_sent_data_timer.reset();
        mbed_sent_data_timer.start();
    }
    else
    {
        BT_LE_ERROR(("Already data send is ON \n"));
        return -1;
    }
    return 0;
}

int handle_coc_stop_data( int argc, char *argv[], tlv_buffer_t** data )
{
    BT_LE_INFO(("handle_coc_stop_data \n"));
    if (le_coc_send_data == true)
    {
        le_coc_send_data = false;
        mbed_sent_data_timer.stop();
    }
    else
    {
        BT_LE_ERROR(("Already data send is OFF \n"));
        return -1;
    }
    return 0;
}

int handle_coc_throughput( int argc, char *argv[], tlv_buffer_t** data )
{
    auto elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(mbed_sent_data_timer.elapsed_time());
    int time_in_us = elapsed_time.count();
    float result = 0;

    BT_LE_INFO(("get_throughput \n"));

    BT_LE_INFO(("Throughput details for DATA TX ..\n"));
    if (time_in_us != 0)
    {
        result = ((float)total_sent_data/time_in_us) * (8 * 1000 * 1000);
        BT_LE_INFO(("elapsed time in seconds = %f \n", (float)(time_in_us/1000000)));
        BT_LE_INFO(("total le bytes transferred  = %llu \n", total_sent_data));
        BT_LE_INFO(("TX throughput =  %f bps\n", result));
        mbed_sent_data_timer.reset();
        total_sent_data = 0;
    }
    else
    {
        BT_LE_INFO(("Total Sent time is Zero \n"));
    }

    BT_LE_INFO(("Throughput details for DATA RX ..\n"));
    elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(mbed_received_data_timer.elapsed_time());
    time_in_us = elapsed_time.count();
    if (time_in_us != 0)
    {
        result =  ((float)total_received_data/time_in_us) * (8 * 1000 * 1000);
        BT_LE_INFO(("elapsed time in seconds = %f \n", (float)(time_in_us/1000000)));
        BT_LE_INFO(("total le bytes recieved  = %llu \n", total_received_data));
        BT_LE_INFO(("RX throughput =  %f bps\n", result));
        mbed_received_data_timer.reset();
        total_received_data = 0;
    }
    else
    {
        BT_LE_INFO(("Total Receive time is Zero \n"));
    }
    return 0;
}

int handle_bt_get_device_address( int argc, char *argv[], tlv_buffer_t** data )
{
    BT_LE_INFO(("This command is currently not supported.\n"));
    return 0;
}

// BT Coex Utility initialization.
CORDIO_BT_WEAK_FUNC void bt_utility_init(void)
{
    cy_command_console_add_table ( bt_coex_command_table );

    BLE &ble = BLE::Instance();
    ble.onEventsToProcess(schedule_ble_events);
    static BLECoexTest temp_coex(ble, event_queue);
    test_coex = &temp_coex;
    // TODO : Start should be called when ble_on command is received.
    test_coex->start();
}
#ifdef __cplusplus
}
#endif
#endif