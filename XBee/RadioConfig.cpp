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

#include "XBeeLib.h"
#include "Frames/ApiFrame.h"

using namespace XBeeLib;

RadioStatus XBee::write_config(void)
{
    AtCmdFrame::AtCmdResp cmdresp;

    cmdresp = set_param("WR");
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }
    return Success;
}

RadioStatus XBee::sleep_now(void)
{
    AtCmdFrame::AtCmdResp cmdresp;

    cmdresp = set_param("SI");
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }
    return Success;
}

RadioStatus XBee::set_power_level(uint8_t  level)
{
    AtCmdFrame::AtCmdResp cmdresp;

    if (level > 4) {
        return Failure;
    }

    cmdresp = set_param("PL", level);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }

    return Success;
}

RadioStatus XBee::get_power_level(uint8_t * const  level)
{
    if (level == NULL) {
        return Failure;
    }
    AtCmdFrame::AtCmdResp cmdresp;

    uint32_t var32;
    cmdresp = get_param("PL", &var32);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }
    *level = var32;
    return Success;
}

RadioStatus XBee::get_network_address(uint16_t * const  addr16)
{
    if (addr16 == NULL) {
        return Failure;
    }
    AtCmdFrame::AtCmdResp cmdresp;

    uint32_t var32;
    cmdresp = get_param("MY", &var32);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }
    *addr16 = var32;
    return Success;
}

RadioStatus XBee::software_reset(void)
{
    volatile uint16_t * const rst_cnt_p = &_wd_reset_cnt;
    const uint16_t init_rst_cnt = *rst_cnt_p;

    AtCmdFrame::AtCmdResp cmdresp;

    cmdresp = set_param("FR");
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }

    return wait_for_module_to_reset(rst_cnt_p, init_rst_cnt);
}

RadioStatus XBee::set_node_identifier(const char * const node_id)
{
    if (node_id == NULL) {
        return Failure;
    }

    AtCmdFrame::AtCmdResp cmdresp;
    const size_t str_len = strlen(node_id);

    if(str_len > 20 || str_len < 1) {
        return Failure;
    }

    cmdresp = set_param("NI", (const uint8_t *)node_id, str_len);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }
    return Success;
}

RadioStatus XBee::get_node_identifier(char * const node_id)
{
    if (node_id == NULL) {
        return Failure;
    }

    uint16_t max_ni_length = 20;
    AtCmdFrame::AtCmdResp cmdresp;

    cmdresp = get_param("NI", (uint8_t *)node_id, &max_ni_length);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }
    node_id[max_ni_length] = '\0';
    return Success;
}

uint16_t XBee::get_hw_version() const
{
    return _hw_version;
}

uint16_t XBee::get_fw_version() const
{
    return _fw_version;
}

void XBee::set_tx_options(uint8_t options)
{
    _tx_options = options;
}

uint8_t XBee::get_tx_options() const
{
    return _tx_options;
}

void XBee::set_broadcast_radius(uint8_t bc_radius)
{
    _bc_radius = bc_radius;
}

uint8_t XBee::get_bc_radius() const
{
    return _bc_radius;
}

RadioStatus XBee::start_node_discovery()
{
    AtCmdFrame cmd_frame = AtCmdFrame("ND");
    send_api_frame(&cmd_frame);

    return Success;
}

void XBee::_get_remote_node_by_id(const char * const node_id, uint64_t * const addr64, uint16_t * const addr16)
{
    *addr64 = ADDR64_UNASSIGNED;
    *addr16 = ADDR16_UNKNOWN;
    if (node_id == NULL) {
        return;
    }
    const size_t node_id_len = strlen(node_id);
    if (node_id_len == 0 || node_id_len > MAX_NI_PARAM_LEN) {
        return;
    }

    const uint16_t old_timeout = _timeout_ms;

    uint32_t nd_timeout_100msec;
    const AtCmdFrame::AtCmdResp nt_resp = get_param("NT", &nd_timeout_100msec);
    if (nt_resp != AtCmdFrame::AtCmdRespOk) {
        _timeout_ms = 10000;
    } else {
        _timeout_ms = (uint16_t)nd_timeout_100msec * 100 + 1000;
    }

    const AtCmdFrame::AtCmdResp cmdresp = set_param("ND", (const uint8_t *)node_id, strlen(node_id));
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        _timeout_ms = old_timeout;
        return;
    }

    const int nd_start_time = _timer.read_ms();
    const int nd_end_time = nd_start_time + _timeout_ms;

    AtCmdFrame atnd_frame = AtCmdFrame("ND", (const uint8_t *)node_id, strlen(node_id));
    send_api_frame(&atnd_frame);

    ApiFrame * const resp_frame = get_this_api_frame(atnd_frame.get_frame_id(), ApiFrame::AtCmdResp);
    _timeout_ms = old_timeout;

    while (_timer.read_ms() < nd_end_time) {
        wait_ms(10);
    }

    if (resp_frame == NULL) {
        digi_log(LogLevelWarning, "XBeeZB::get_remote_node_by_id timeout when waiting for ATND response");
        return;
    }

    const AtCmdFrame::AtCmdResp resp = (AtCmdFrame::AtCmdResp)resp_frame->get_data_at(ATCMD_RESP_STATUS_OFFSET);
    if (resp != AtCmdFrame::AtCmdRespOk) {
        digi_log(LogLevelWarning, "send_at_cmd bad response: 0x%x\r\n", resp);
        _framebuf.free_frame(resp_frame);
        return;
    }

    rmemcpy((uint8_t *)addr16, resp_frame->get_data() + ATCMD_RESP_DATA_OFFSET, sizeof *addr16);
    rmemcpy((uint8_t *)addr64, resp_frame->get_data() + ATCMD_RESP_DATA_OFFSET + sizeof *addr16, sizeof *addr64);
    _framebuf.free_frame(resp_frame);
    return;
}

RadioStatus XBee::config_node_discovery(uint16_t timeout_ms, uint8_t options)
{
    AtCmdFrame::AtCmdResp cmdresp;

    cmdresp = set_param("NT", (uint8_t)(timeout_ms / 100));
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }

    cmdresp = set_param("NO", (uint8_t)options);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }
    cmdresp = set_param("AC");
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }

    return Success;
}

RadioStatus XBee::get_config_node_discovery(uint16_t * const timeout_ms, uint8_t * const options)
{
    AtCmdFrame::AtCmdResp cmdresp;
    uint32_t var32;

    cmdresp = get_param("NT", &var32);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }
    *timeout_ms = var32;

    cmdresp = get_param("NO", &var32);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        return Failure;
    }
    *options = var32;
    return Success;
}

RadioStatus XBee::get_iosample(const RemoteXBee& remote, uint8_t * const io_sample, uint16_t * const len)
{
    AtCmdFrame::AtCmdResp cmdresp;
    *len = MAX_IO_SAMPLE_BUF_LEN;

    /* Force a sample read */
    cmdresp = get_param(remote, "IS", io_sample, len);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        digi_log(LogLevelError, "get_iosample error %d:\r\n", cmdresp);
        return Failure;
    }

    return Success;
}

RadioStatus XBee::config_io_sample_destination(const RemoteXBee& remote, const RemoteXBee& destination)
{
    uint32_t dh;
    uint32_t dl;

    if (destination.is_valid_addr64b()) {
        const uint64_t dest64 = destination.get_addr64();
        dh = (uint32_t)((dest64 >> 32) & 0xFFFFFFFF);
        dl = (uint32_t)((dest64 & 0xFFFFFFFF));
    } else if (destination.is_valid_addr16b()) {
        const uint16_t destAddr16 = destination.get_addr16();
        dh = 0;
        dl = destAddr16;
    } else {
        digi_log(LogLevelError, "send_io_sample_to: Invalid destination");
        return Failure;
    }

    AtCmdFrame::AtCmdResp cmdresp;

    cmdresp = set_param(remote, "DH", dh);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        digi_log(LogLevelError, "send_io_sample_to error %d:\r\n", cmdresp);
        return Failure;
    }

    cmdresp = set_param(remote, "DL", dl);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        digi_log(LogLevelError, "send_io_sample_to error %d:\r\n", cmdresp);
        return Failure;
    }

    return Success;
}

RadioStatus XBee::set_io_sample_rate(const RemoteXBee& remote, const float seconds)
{
    const float max_seconds = 65.535;

    if (seconds > max_seconds) {
        digi_log(LogLevelError, "XBee::set_io_sample_rate error seconds rate exceeds maximum %d:\r\n", max_seconds);
        return Failure;
    }

    AtCmdFrame::AtCmdResp cmdresp;
    const uint16_t milliseconds = seconds * 1000;

    cmdresp = set_param(remote, "IR", milliseconds);
    if (cmdresp != AtCmdFrame::AtCmdRespOk) {
        digi_log(LogLevelError, "XBee::set_io_sample_rate error %d:\r\n", cmdresp);
        return Failure;
    }

    return Success;
}
