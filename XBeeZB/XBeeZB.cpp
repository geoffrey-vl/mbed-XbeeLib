/**
 * Copyright (c) 2015 Digi International Inc.,
 * All rights not expressly granted are reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Digi International Inc. 11001 Bren Road East, Minnetonka, MN 55343
 * =======================================================================
 */

#include "XBeeZB.h"
#include "IO/IOSampleZB.h"
#include "Frames/ZigbeeFrames.h"

using namespace XBeeLib;

/* Class constructor */
XBeeZB::XBeeZB(PinName tx, PinName rx, PinName reset, PinName rts, PinName cts, int baud) :
         XBee(tx, rx, reset, rts, cts, baud), _nw_role(UnknownRole), _joined_network(false),
        _vcc_exceeded_cnt(0), _broadcast_radious(0), _nd_handler(NULL), 
        _rx_pkt_handler(NULL), _io_data_handler(NULL)
{
    _reset_timeout = RESET_TIMEOUT_MS;
}

RadioStatus XBeeZB::init()
{
    RadioStatus retval = XBee::init();
    /* Determine the role of this device in the network */
    switch(_fw_version & 0xFF00) {
        case 0x2100:    _nw_role = Coordinator;     break;
        case 0x2300:    _nw_role = Router;          break;
        case 0x2900:    _nw_role = EndDevice;       break;
        default:        _nw_role = UnknownRole;     break;
    }

    const RadioProtocol radioProtocol = get_radio_protocol();
    if (radioProtocol != ZigBee) {
        digi_log(LogLevelError, "Radio protocol does not match, needed a %d got a %d\r\n", ZigBee, radioProtocol);
        retval = Failure;
    }
    assert(radioProtocol == ZigBee);

    return retval;
}

/* Class destructor */
XBeeZB::~XBeeZB()
{
    unregister_node_discovery_cb();
    unregister_receive_cb();
    unregister_io_sample_cb();
}

RadioStatus XBeeZB::set_channel_mask(uint16_t  chmask)
{
    AtCmdFrame::AtCmdResp cmdresp;

    cmdresp = set_param("SC", chmask);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }
    return Success;
}

RadioStatus XBeeZB::get_channel_mask(uint16_t * const  chmask)
{
    if (chmask == NULL) {
        return Failure;
    }
    AtCmdFrame::AtCmdResp cmdresp;

    uint32_t var32;
    cmdresp = get_param("SC", &var32);
    if (cmdresp != AtCmdFrame::AtCmdRespOk)
        return Failure;
    *chmask = var32;
    return Success;
}

RadioStatus XBeeZB::set_panid(uint64_t  panid)
{
    uint8_t panid_u8[8];
    AtCmdFrame::AtCmdResp cmdresp;

    rmemcpy(panid_u8, (const uint8_t *) &panid, sizeof panid_u8);

    cmdresp = set_param("ID", panid_u8, sizeof panid_u8);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }
    return Success;
}

RadioStatus XBeeZB::get_operating_panid(uint64_t * const  opanid)
{
    if (opanid == NULL) {
        return Failure;
    }
    uint8_t opanid_u8[8];
    uint16_t len = sizeof opanid_u8;
    AtCmdFrame::AtCmdResp cmdresp;

    cmdresp = get_param("OP", opanid_u8, &len);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }
    if (len != sizeof opanid_u8) {
        digi_log(LogLevelError, "XBeeZB::get_operating_panid: Read %d bytes instead of %d for OP", len, sizeof opanid_u8);
        return Failure;
    }
    rmemcpy((uint8_t *)opanid, opanid_u8, len);
    return Success;
}

RadioStatus XBeeZB::get_configured_panid(uint64_t * const  panid)
{
    if (panid == NULL) {
        return Failure;
    }
    uint8_t panid_u8[8];
    uint16_t len = sizeof panid_u8;
    AtCmdFrame::AtCmdResp cmdresp;

    cmdresp = get_param("ID", panid_u8, &len);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }
    if (len != sizeof panid_u8) {
        digi_log(LogLevelError, "XBeeZB::get_configured_panid: Read %d bytes instead of %d for ID", len, sizeof panid_u8);
        return Failure;
    }
    rmemcpy((uint8_t *)panid, panid_u8, len);
    return Success;
}

RadioStatus XBeeZB::set_panid(const RemoteXBee& remote, uint64_t  panid)
{
    uint8_t panid_u8[8];
    AtCmdFrame::AtCmdResp cmdresp;

    rmemcpy(panid_u8, (const uint8_t *) &panid, sizeof panid_u8);

    cmdresp = set_param(remote, "ID", panid_u8, sizeof panid_u8);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }
    return Success;
}

RadioStatus XBeeZB::get_operating_panid(const RemoteXBee& remote, uint64_t * const  opanid)
{
    if (opanid == NULL) {
        return Failure;
    }
    uint8_t opanid_u8[8];
    uint16_t len = sizeof opanid_u8;
    AtCmdFrame::AtCmdResp cmdresp;

    cmdresp = get_param(remote, "OP", opanid_u8, &len);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }
    if (len != sizeof opanid_u8) {
        digi_log(LogLevelError, "XBeeZB::get_operating_panid: Read %d bytes instead of %d for OP", len, sizeof opanid_u8);
        return Failure;
    }
    rmemcpy((uint8_t *)opanid, opanid_u8, len);
    return Success;
}

RadioStatus XBeeZB::get_configured_panid(const RemoteXBee& remote, uint64_t * const  panid)
{
    if (panid == NULL) {
        return Failure;
    }
    uint8_t panid_u8[8];
    uint16_t len = sizeof panid_u8;
    AtCmdFrame::AtCmdResp cmdresp;

    cmdresp = get_param(remote, "ID", panid_u8, &len);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }
    if (len != sizeof panid_u8) {
        digi_log(LogLevelError, "XBeeZB::get_configured_panid: Read %d bytes instead of %d for ID", len, sizeof panid_u8);
        return Failure;
    }
    rmemcpy((uint8_t *)panid, panid_u8, len);
    return Success;
}

RadioStatus XBeeZB::check_for_coordinator_at_start(bool enable)
{
    AtCmdFrame::AtCmdResp cmdresp;

    cmdresp = set_param("JV", (uint8_t)enable);
    if (cmdresp != AtCmdFrame::AtCmdRespOk)
        return Failure;
    return Success;
}

RadioStatus XBeeZB::enable_network_encryption(bool enable)
{
    AtCmdFrame::AtCmdResp cmdresp;

    cmdresp = set_param("EE", enable);
    if (cmdresp != AtCmdFrame::AtCmdRespOk)
        return Failure;
    return Success;
}

RadioStatus XBeeZB::set_encryption_key(const uint8_t * const key, const uint16_t length)
{
    if (key == NULL || length == 0) {
        return Failure;
    }
    AtCmdFrame::AtCmdResp cmdresp;

    cmdresp = set_param("NK", key, length);
    if (cmdresp != AtCmdFrame::AtCmdRespOk)
        return Failure;
    return Success;
}

RadioStatus XBeeZB::set_tc_link_key(const uint8_t * const key, const uint16_t length)
{
    if (key == NULL || length == 0) {
        return Failure;
    }
    AtCmdFrame::AtCmdResp cmdresp;

    cmdresp = set_param("KY", key, length);
    if (cmdresp != AtCmdFrame::AtCmdRespOk)
        return Failure;
    return Success;
}

RadioStatus XBeeZB::set_encryption_options(const uint8_t options)
{
    AtCmdFrame::AtCmdResp cmdresp;

    cmdresp = set_param("EO", options);
    if (cmdresp != AtCmdFrame::AtCmdRespOk)
        return Failure;
    return Success;
}

void XBeeZB::radio_status_update(AtCmdFrame::ModemStatus modem_status)
{
    /* Update the radio status variables */
    if (modem_status == AtCmdFrame::HwReset) {
        _hw_reset_cnt++;
        _joined_network = false;
    }
    else if (modem_status == AtCmdFrame::WdReset) {
        _wd_reset_cnt++;
        _joined_network = false;
    }
    else if (modem_status == AtCmdFrame::JoinedNW)
        _joined_network = true;
    else if (modem_status == AtCmdFrame::Disassociated)
        _joined_network = false;
    else if (modem_status == AtCmdFrame::VccExceeded)
        _vcc_exceeded_cnt++;

    _modem_status = modem_status;

    digi_log(LogLevelDebug, "\r\nUpdating radio status: %02x\r\n", modem_status);
}

TxStatus XBeeZB::send_data(const RemoteXBee& remote, const uint8_t *const data, uint16_t len)
{
    if (!remote.is_valid_addr64b())
        return TxStatusInvalidAddr;

    const uint64_t remote64 = remote.get_addr64();
    const uint16_t remote16 = remote.get_addr16();

    TxFrameZB frame = TxFrameZB(remote64, remote16, _broadcast_radious,
                                _tx_options, data, len);
    return send_data(&frame);
}

TxStatus XBeeZB::send_data(uint64_t remote64, const uint8_t *const data, uint16_t len)
{
    TxFrameZB frame = TxFrameZB(remote64, ADDR16_UNKNOWN, _broadcast_radious,
                                _tx_options, data, len);
    return send_data(&frame);
}

TxStatus XBeeZB::send_data(uint64_t remote64, uint16_t remote16,
                                const uint8_t *const data, uint16_t len)
{
    TxFrameZB frame = TxFrameZB(remote64, remote16, _broadcast_radious,
                                _tx_options, data, len);
    return send_data(&frame);
}

TxStatus XBeeZB::send_data(const RemoteXBee& remote, uint8_t source_ep, 
                                uint8_t dest_ep, uint16_t cluster_id, uint16_t profile_id,
                                const uint8_t *const data, uint16_t len)
{
    if (!remote.is_valid_addr64b())
        return TxStatusInvalidAddr;

    const uint64_t remote64 = remote.get_addr64();
    const uint16_t remote16 = remote.get_addr16();

    TxFrameZB frame = TxFrameZB(remote64, remote16, source_ep, dest_ep,
                                cluster_id, profile_id, _broadcast_radious,
                                _tx_options, data, len);
    return send_data(&frame);
}

TxStatus XBeeZB::send_data(uint64_t remote64, uint16_t remote16, uint8_t source_ep,
                                uint8_t dest_ep, uint16_t cluster_id, uint16_t profile_id,
                                const uint8_t *const data, uint16_t len)
{
    TxFrameZB frame = TxFrameZB(remote64, remote16, source_ep, dest_ep,
                                cluster_id, profile_id, _broadcast_radious,
                                _tx_options, data, len);
    return send_data(&frame);
}
                                
TxStatus XBeeZB::send_data_to_coordinator(const uint8_t *const data, uint16_t len)
{
    const uint64_t remaddr = ADDR64_COORDINATOR;
    
    TxFrameZB frame = TxFrameZB(remaddr, ADDR16_UNKNOWN, _broadcast_radious,
                                _tx_options, data, len);
    return send_data(&frame);
}

NetworkRole XBeeZB::get_network_role()
{
    return _nw_role;
}

bool XBeeZB::is_joined()
{
    return _joined_network;
}

void XBeeZB::register_node_discovery_cb(node_discovery_zb_cb_t function)
{
    if (_nd_handler == NULL) {
        _nd_handler = new FH_NodeDiscoveryZB();
        register_frame_handler(_nd_handler);
    }
    _nd_handler->register_node_discovery_cb(function);
}

void XBeeZB::unregister_node_discovery_cb()
{
    if (_nd_handler != NULL) {
        _nd_handler->unregister_node_discovery_cb();
        unregister_frame_handler(_nd_handler);
        delete _nd_handler;
        _nd_handler = NULL; /* as delete does not set to NULL */
    }
}

void XBeeZB::register_receive_cb(receive_zb_cb_t function)
{
    if (_rx_pkt_handler == NULL) {
        _rx_pkt_handler = new FH_RxPacketZB();
        register_frame_handler(_rx_pkt_handler);
    }
    _rx_pkt_handler->register_receive_cb(function);
}

void XBeeZB::unregister_receive_cb()
{
    if (_rx_pkt_handler != NULL) {
        _rx_pkt_handler->unregister_receive_cb();
        unregister_frame_handler(_rx_pkt_handler);
        delete _rx_pkt_handler;
        _rx_pkt_handler = NULL; /* as delete does not set to NULL */
    }
}

void XBeeZB::register_io_sample_cb(io_data_cb_zb_t function)
{
    if (_io_data_handler == NULL) {
        _io_data_handler = new FH_IoDataSampeZB();
        register_frame_handler(_io_data_handler);
    }
    _io_data_handler->register_io_data_cb(function);
}

void XBeeZB::unregister_io_sample_cb()
{
    if (_io_data_handler != NULL) {
        _io_data_handler->unregister_io_data_cb();
        unregister_frame_handler(_io_data_handler);
        delete _io_data_handler;
        _io_data_handler = NULL; /* as delete does not set to NULL */
    }
}

XBeeZB * XBeeZB::get_device_by_id(const char * const node_id)
{
    return NULL;
}

RadioStatus XBeeZB::get_device_by_id(const char * const node_id, uint64_t * const dev_addr)
{
    AtCmdFrame::AtCmdResp cmdresp;
    uint32_t dh, dl;
    
    if (strlen(node_id) > MAX_NI_PARAM_LEN)
        return Failure;

    cmdresp = set_param("DN", (const uint8_t *)node_id, strlen(node_id));
    if (cmdresp != AtCmdFrame::AtCmdRespOk)
        return Failure;
    
    /* Read the address of the remote device from the DH, DL parameters */
    cmdresp = get_param("DH", &dh);
    if (cmdresp != AtCmdFrame::AtCmdRespOk)
        return Failure;
    
    cmdresp = get_param("DL", &dl);
    if (cmdresp != AtCmdFrame::AtCmdRespOk)
        return Failure;
    
    *dev_addr = UINT64(dh, dl);
    
    return Success;
}

AtCmdFrame::AtCmdResp XBeeZB::get_param(const RemoteXBee& remote, const char * const param, uint32_t * const data)
{
    if (!remote.is_valid_addr64b())
        return AtCmdFrame::AtCmdRespInvalidAddr;

    const uint64_t remote64 = remote.get_addr64();
    const uint16_t remote16 = remote.get_addr16();
    uint16_t len = sizeof *data;
    AtCmdFrame::AtCmdResp atCmdResponse;

    AtCmdFrame cmd_frame = AtCmdFrame(remote64, remote16, param);
    atCmdResponse = send_at_cmd(&cmd_frame, (uint8_t *)data, &len, RadioRemote);

    if (atCmdResponse == AtCmdFrame::AtCmdRespOk && len > sizeof *data)
        atCmdResponse = AtCmdFrame::AtCmdRespLenMismatch;

    return atCmdResponse;
}

AtCmdFrame::AtCmdResp XBeeZB::set_param(const RemoteXBee& remote, const char * const param, uint32_t data)
{
    if (!remote.is_valid_addr64b())
        return AtCmdFrame::AtCmdRespInvalidAddr;

    const uint64_t remote64 = remote.get_addr64();
    const uint16_t remote16 = remote.get_addr16();

    AtCmdFrame cmd_frame = AtCmdFrame(remote64, remote16, param, data);
    return send_at_cmd(&cmd_frame, NULL, NULL, RadioRemote);
}

AtCmdFrame::AtCmdResp XBeeZB::set_param(const RemoteXBee& remote, const char * const param, const uint8_t * data, uint16_t len)
{
    if (!remote.is_valid_addr64b())
        return AtCmdFrame::AtCmdRespInvalidAddr;

    const uint64_t remote64 = remote.get_addr64();
    const uint16_t remote16 = remote.get_addr16();

    AtCmdFrame cmd_frame = AtCmdFrame(remote64, remote16, param, data, len);
    return send_at_cmd(&cmd_frame, NULL, NULL, RadioRemote);
}

AtCmdFrame::AtCmdResp XBeeZB::get_param(const RemoteXBee& remote, const char * const param, uint8_t * const data, uint16_t * const len)
{

    if (!remote.is_valid_addr64b())
        return AtCmdFrame::AtCmdRespInvalidAddr;

    const uint64_t remote64 = remote.get_addr64();
    const uint16_t remote16 = remote.get_addr16();

    AtCmdFrame cmd_frame = AtCmdFrame(remote64, remote16, param);
    return send_at_cmd(&cmd_frame, data, len, RadioRemote, false);
}

static void get_dio_cmd(XBeeZB::IoLine line, char * const iocmd)
{
    if (line >= XBeeZB::DIO10) {
        iocmd[0] = 'P';
        iocmd[1] = '0' + line - XBeeZB::DIO10;
    } else {
        iocmd[0] = 'D';
        iocmd[1] = '0' + line;
    }
    iocmd[2] = '\0';
}

RadioStatus XBeeZB::set_pin_config(const RemoteXBee& remote, IoLine line, IoMode mode)
{
    AtCmdFrame::AtCmdResp cmdresp;
    char iocmd[3];

    get_dio_cmd(line, iocmd);

    cmdresp = set_param(remote, iocmd, (uint8_t)mode);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        digi_log(LogLevelError, "set_pin_config: set_param returned %d\r\n", cmdresp);
        return Failure;
    }

    return Success;
}

RadioStatus XBeeZB::get_pin_config(const RemoteXBee& remote, IoLine line, IoMode * const mode)
{
    AtCmdFrame::AtCmdResp cmdresp;
    char iocmd[3];

    get_dio_cmd(line, iocmd);

    uint32_t var32;
    cmdresp = get_param(remote, iocmd, &var32);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }
    *mode = (IoMode)var32;

    return Success;
}

RadioStatus XBeeZB::set_dio(const RemoteXBee& remote, IoLine line, DioVal val)
{
    if (val == Low)
        return set_pin_config(remote, line, DigitalOutLow);
    else
        return set_pin_config(remote, line, DigitalOutHigh);
}

RadioStatus XBeeZB::get_dio(const RemoteXBee& remote, IoLine line, DioVal * const val)
{
    uint8_t io_sample[MAX_IO_SAMPLE_BUF_LEN];
    uint16_t len;

    RadioStatus resp = get_iosample(remote, io_sample, &len);
    if (resp != Success)
        return resp;

    IOSampleZB ioSample = IOSampleZB(io_sample, len);
    return ioSample.get_dio(line, val);
}

RadioStatus XBeeZB::get_adc(const RemoteXBee& remote, IoLine line, uint16_t * const val)
{
    uint8_t io_sample[MAX_IO_SAMPLE_BUF_LEN];
    uint16_t len;

    switch (line) {
        case SUPPLY_VOLTAGE:
        case DIO3_AD3:
        case DIO2_AD2:
        case DIO1_AD1:
        case DIO0_AD0:
            break;
        default:
            digi_log(LogLevelError, "get_adc: Pin %d not supported as ADC\r\n", line);
            return Failure;
    }

    RadioStatus resp = get_iosample(remote, io_sample, &len);
    if (resp != Success)
        return resp;

    IOSampleZB ioSample = IOSampleZB(io_sample, len);
    return ioSample.get_adc(line, val);
}

static uint16_t get_dio_pr_mask(XBeeZB::IoLine line)
{
    switch (line) {
        case XBeeZB::DIO4:
            return (1 << 0);
        case XBeeZB::DIO3_AD3:
            return (1 << 1);
        case XBeeZB::DIO2_AD2:
            return (1 << 2);
        case XBeeZB::DIO1_AD1:
            return (1 << 3);
        case XBeeZB::DIO0_AD0:
            return (1 << 4);
        case XBeeZB::DIO6:
            return (1 << 5);
        case XBeeZB::DIO5:
            return (1 << 8);
        case XBeeZB::DIO12:
            return (1 << 10);
        case XBeeZB::DIO10:
            return (1 << 11);
        case XBeeZB::DIO11:
            return (1 << 12);
        case XBeeZB::DIO7:
            return (1 << 13);
        default:
            return 0;
    }
}

RadioStatus XBeeZB::set_pin_pull_up(const RemoteXBee& remote, IoLine line, bool enable)
{
    AtCmdFrame::AtCmdResp cmdresp;
    uint32_t var32;
    uint16_t pr;

    cmdresp = get_param(remote, "PR", &var32);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }
    pr = var32;

    const uint16_t dio_mask = get_dio_pr_mask(line);
    if (dio_mask == 0) {
        digi_log(LogLevelError, "XBeeZB::set_pin_pull_up: invalid pin %d\r\n", line);
        return Failure;
    }

    if (enable) {
        pr |= dio_mask;
    } else {
        pr &= ~dio_mask;
    }

    cmdresp = set_param(remote, "PR", pr);
    if (cmdresp != AtCmdFrame::AtCmdRespOk)
        return Failure;

    return Success;
}

static uint16_t get_dio_ic_mask(XBeeZB::IoLine line)
{
    if (line < XBeeZB::DIO12) {
        return (1 << line);
    }
    return 0;
}

RadioStatus XBeeZB::enable_dio_change_detection(const RemoteXBee& remote, IoLine line, bool enable)
{
    if (line > DIO11) {
        digi_log(LogLevelError, "XBeeZB::enable_dio_change_detection: pin not supported (%d)\r\n", line);
        return Failure;
    }

    AtCmdFrame::AtCmdResp cmdresp;
    uint32_t var32;
    uint16_t ic;

    cmdresp = get_param(remote, "IC", &var32);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }
    ic = var32;

    const uint16_t dio_mask = get_dio_ic_mask(line);
    if (dio_mask == 0) {
        digi_log(LogLevelError, "XBeeZB::enable_dio_change_detection: invalid pin %d\r\n", line);
        return Failure;
    }

    if (enable) {
        ic |= dio_mask;
    } else {
        ic &= ~dio_mask;
    }

    cmdresp = set_param(remote, "IC", ic);
    if (cmdresp != AtCmdFrame::AtCmdRespOk)
        return Failure;

    return Success;
}
