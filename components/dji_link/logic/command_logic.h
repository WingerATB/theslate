/* SPDX-License-Identifier: MIT */
/*
 * Copyright (C) 2025 SZ DJI Technology Co., Ltd.
 *  
 * All information contained herein is, and remains, the property of DJI.
 * The intellectual and technical concepts contained herein are proprietary
 * to DJI and may be covered by U.S. and foreign patents, patents in process,
 * and protected by trade secret or copyright law.  Dissemination of this
 * information, including but not limited to data and other proprietary
 * material(s) incorporated within the information, in any form, is strictly
 * prohibited without the express written consent of DJI.
 *
 * If you receive this source code without DJI’s authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 */

#ifndef __COMMAND_LOGIC_H__
#define __COMMAND_LOGIC_H__

#include "enums_logic.h"

#include "dji_protocol_data_structures.h"

#include <string.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"

uint16_t generate_seq(void);

typedef struct {
    void *structure;
    size_t length;  // This is not the length of structure, but the length of DATA segment excluding CmdSet and CmdID
                    // 这里的长度并不是 structure 长度，而是 DATA 段除去 CmdSet 和 CmdID 的长度
} CommandResult;

esp_err_t command_logic_send_raw_bytes(const char *raw_data_string, int timeout_ms);

CommandResult send_command(uint8_t cmd_set, uint8_t cmd_id, uint8_t cmd_type, const void *structure, uint16_t seq, int timeout_ms);

camera_mode_switch_response_frame_t* command_logic_switch_camera_mode(camera_mode_t mode);

version_query_response_frame_t* command_logic_get_version(void);

record_control_response_frame_t* command_logic_start_record(void);

record_control_response_frame_t* command_logic_stop_record(void);


key_report_response_frame_t* command_logic_key_report_qs(void);

key_report_response_frame_t* command_logic_key_report_snapshot(void);

#endif
