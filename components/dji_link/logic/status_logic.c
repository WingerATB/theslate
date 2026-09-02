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

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include "esp_log.h"

#include "enums_logic.h"
#include "connect_logic.h"
#include "command_logic.h"
#include "dji_protocol_data_structures.h"
#include "dji_protocol_data_descriptors.h"
#include "status_logic.h"

static const char *TAG = "LOGIC_STATUS";

/* SLATE ADDITION */
static camera_status_cb_t s_status_cb = NULL;

void status_logic_set_callback(camera_status_cb_t cb) {
    s_status_cb = cb;
}

// Global variables to store various camera status information
// 全局变量，保存相机的各种状态信息
uint8_t current_camera_mode = 0;
uint8_t current_camera_status = 0;
uint8_t current_video_resolution = 0;
uint8_t current_fps_idx = 0;
uint8_t current_eis_mode = 0;
uint8_t current_user_mode = 0;
uint8_t current_camera_mode_next_flag = 0;
uint16_t current_record_time = 0;
uint16_t current_timelapse_interval = 0;
bool camera_status_initialized = false;

// Global variables for new camera status push command frame
// 新相机状态推送命令帧的全局变量
uint8_t current_type_mode_name = 0;
uint8_t current_mode_name_length = 0;
uint8_t current_mode_name[20] = {0};
uint8_t current_type_mode_param = 0;
uint8_t current_mode_param_length = 0;
uint8_t current_mode_param[20] = {0};


/**
 * @brief Check if camera is recording
 *        检查相机是否正在录制
 * 
 * Check if camera is in recording or pre-recording state, and status is initialized.
 * 判断相机是否处于录制状态或预录制状态，并且状态已初始化。
 * 
 * @return bool Returns true if camera is recording, false otherwise
 *              如果相机正在录制，则返回 true，否则返回 false
 */
bool is_camera_recording() {
    if ((current_camera_status == CAMERA_STATUS_PHOTO_OR_RECORDING || current_camera_status == CAMERA_STATUS_PRE_RECORDING) && camera_status_initialized) {
        return true;
    }
    return false;
}

/**
 * @brief Print current camera status (partial status, other status can be printed as needed)
 *        打印当前相机状态（部分状态，后续可自行打印其它状态）
 * 
 * Print camera mode, status, resolution, frame rate and electronic image stabilization mode.
 * 打印相机的模式、状态、分辨率、帧率和电子防抖模式等信息。
 */
void print_camera_status() {
    if (!camera_status_initialized) {
        ESP_LOGW(TAG, "Camera status has not been initialized.");
        return;
    }

    const char *mode_str = camera_mode_to_string((camera_mode_t)current_camera_mode);
    const char *status_str = camera_status_to_string((camera_status_t)current_camera_status);
    const char *resolution_str = video_resolution_to_string((video_resolution_t)current_video_resolution);
    const char *fps_str = fps_idx_to_string((fps_idx_t)current_fps_idx);
    const char *eis_str = eis_mode_to_string((eis_mode_t)current_eis_mode);

    ESP_LOGI(TAG, "[1D02] =========== Camera Status Push ===========");
    ESP_LOGI(TAG, "  Mode: %s", mode_str);
    ESP_LOGI(TAG, "  Status: %s", status_str);
    ESP_LOGI(TAG, "  Resolution: %s (value: %d)", resolution_str, current_video_resolution);
    ESP_LOGI(TAG, "  FPS: %s", fps_str);
    ESP_LOGI(TAG, "  EIS: %s", eis_str);
    ESP_LOGI(TAG, "  User mode: %d", current_user_mode);
    ESP_LOGI(TAG, "  Camera mode next flag: %d", current_camera_mode_next_flag);
    ESP_LOGI(TAG, "  Record time: %d", current_record_time);
    ESP_LOGI(TAG, "  Timelapse interval: %d", current_timelapse_interval);
    ESP_LOGI(TAG, "=================================================");
}

/**
 * @brief Subscribe to camera status
 *        订阅相机状态
 * 
 * @param push_mode Subscription mode
 *                  订阅模式
 * @param push_freq Subscription frequency
 *                  订阅频率
 * @return int Returns 0 on success, -1 on failure
 *             返回 0 表示成功，-1 表示失败
 */
int subscript_camera_status(uint8_t push_mode, uint8_t push_freq) {
    ESP_LOGI(TAG, "Subscribing to Camera Status with push_mode: %d, push_freq: %d", push_mode, push_freq);

    if (connect_logic_get_state() != PROTOCOL_CONNECTED) {
        ESP_LOGE(TAG, "Protocol connection to the camera failed. Current connection state: %d", connect_logic_get_state());
        return -1;
    }

    uint16_t seq = generate_seq();

    camera_status_subscription_command_frame command_frame = {
        .push_mode = push_mode,
        .push_freq = push_freq,
        .reserved = {0, 0, 0, 0}
    };

    send_command(0x1D, 0x05, CMD_NO_RESPONSE, &command_frame, seq, 5000);

    return 0;
}

/**
 * @brief Update camera state machine (callback function)
 *        更新相机状态机（回调函数）
 * 
 * Process and update various camera states, check for state changes and print updated information.
 * 处理并更新相机的各项状态，检查状态是否发生变化并打印更新后的信息。
 * 
 * @param data Input camera status data
 *             传入的相机状态数据
 */
void update_camera_state_handler(void *data) {
    if (!data) {
        ESP_LOGE(TAG, "logic_update_camera_state: Received NULL data.");
        return;
    }

    const camera_status_push_command_frame *parsed_data = (const camera_status_push_command_frame *)data;

    bool state_changed = false;

    // Check and update camera mode
    // 检查并更新相机模式
    if (current_camera_mode != parsed_data->camera_mode) {
        current_camera_mode = parsed_data->camera_mode;
        ESP_LOGI(TAG, "Camera mode updated to: %d", current_camera_mode);
        state_changed = true;
    }

    // Check and update camera status
    // 检查并更新相机状态
    if (current_camera_status != parsed_data->camera_status) {
        current_camera_status = parsed_data->camera_status;
        ESP_LOGI(TAG, "Camera status updated to: %d", current_camera_status);
        state_changed = true;
    }

    // Check and update video resolution
    // 检查并更新视频分辨率
    if (current_video_resolution != parsed_data->video_resolution) {
        current_video_resolution = parsed_data->video_resolution;
        ESP_LOGI(TAG, "Video resolution updated to: %d", current_video_resolution);
        state_changed = true;
    }

    // Check and update frame rate
    // 检查并更新帧率
    if (current_fps_idx != parsed_data->fps_idx) {
        current_fps_idx = parsed_data->fps_idx;
        ESP_LOGI(TAG, "FPS index updated to: %d", current_fps_idx);
        state_changed = true;
    }

    // Check and update electronic image stabilization mode
    // 检查并更新电子防抖模式
    if (current_eis_mode != parsed_data->eis_mode) {
        current_eis_mode = parsed_data->eis_mode;
        ESP_LOGI(TAG, "EIS mode updated to: %d", current_eis_mode);
        state_changed = true;
    }

    // Check and update user mode
    // 检查并更新用户模式
    if (current_user_mode != parsed_data->user_mode) {
        current_user_mode = parsed_data->user_mode;
        ESP_LOGI(TAG, "User mode updated to: %d", current_user_mode);
        state_changed = true;
    }

    // Check and update camera mode next flag
    // 检查并更新相机模式下一个标志
    if (current_camera_mode_next_flag != parsed_data->camera_mode_next_flag) {
        current_camera_mode_next_flag = parsed_data->camera_mode_next_flag;
        ESP_LOGI(TAG, "Camera mode next flag updated to: %d", current_camera_mode_next_flag);
        state_changed = true;
    }

    // Check and update record time
    // 检查并更新录制时间
    if (current_record_time != parsed_data->record_time) {
        current_record_time = parsed_data->record_time;
        ESP_LOGI(TAG, "Record time updated to: %d", current_record_time);
        state_changed = true;
    }

    // Check and update timelapse interval
    // 检查并更新延时摄影间隔
    if (current_timelapse_interval != parsed_data->timelapse_interval) {
        current_timelapse_interval = parsed_data->timelapse_interval;
        ESP_LOGI(TAG, "Timelapse interval updated to: %d", current_timelapse_interval);
        state_changed = true;
    }

    // If status not initialized, mark as initialized
    // 如果状态尚未初始化，标记为已初始化
    if (!camera_status_initialized) {
        camera_status_initialized = true;
        ESP_LOGI(TAG, "Camera state fully updated and marked as initialized.");
        state_changed = true;  // Force status print as this is initialization
                               // 强制打印状态，因为这是初始化
    }

    // If state changed or first initialization, print current camera status
    // 如果状态变更或第一次初始化，打印当前相机状态
    if (state_changed) {
        print_camera_status();
    }

    /* SLATE ADDITION: hand the frame to the observer before it is freed,
     * along with how many bytes the camera really sent. */
    if (s_status_cb != NULL) {
        s_status_cb(parsed_data, g_dji_camera_status_payload_len);
    }

    free(data);
}

void update_new_camera_state_handler(void *data) {
    if (!data) {
        ESP_LOGE(TAG, "update_new_camera_state_handler: Received NULL data.");
        return;
    }

    const new_camera_status_push_command_frame *parsed_data = (const new_camera_status_push_command_frame *)data;

    ESP_LOGI(TAG, "[1D06] ========== New Camera Status Push =========");

    // Update type_mode_name
    // 更新相机模式名字类型
    current_type_mode_name = parsed_data->type_mode_name;
    ESP_LOGI(TAG, "[1D06] Camera mode name type: 0x%02X", current_type_mode_name);
    
    // Update mode_name_length
    // 更新模式名字长度
    current_mode_name_length = parsed_data->mode_name_length;
    ESP_LOGI(TAG, "[1D06] Mode name length: %d", current_mode_name_length);
    
    // Update mode_name array
    // 更新模式名字数组
    memcpy(current_mode_name, parsed_data->mode_name, 20);

    // Ensure null termination for safe string printing
    // 确保字符串 null 终止以便安全打印
    char mode_name_str[21] = {0};
    memcpy(mode_name_str, parsed_data->mode_name, 20);
    ESP_LOGI(TAG, "[1D06] Mode name: %s", mode_name_str);
    
    // Update type_mode_param
    // 更新相机模式参数类型
    current_type_mode_param = parsed_data->type_mode_param;
    ESP_LOGI(TAG, "[1D06] Camera mode parameter type: 0x%02X", current_type_mode_param);
    
    // Update mode_param_length
    // 更新模式参数长度
    current_mode_param_length = parsed_data->mode_param_length;
    ESP_LOGI(TAG, "[1D06] Mode parameter length: %d", current_mode_param_length);
    
    // Update mode_param array
    // 更新模式参数数组
    memcpy(current_mode_param, parsed_data->mode_param, 20);

    // Ensure null termination for safe string printing
    // 确保字符串 null 终止以便安全打印
    char mode_param_str[21] = {0};
    memcpy(mode_param_str, parsed_data->mode_param, 20);
    ESP_LOGI(TAG, "[1D06] Mode parameters: %s", mode_param_str);

    ESP_LOGI(TAG, "[1D06] ==========================================");

    free(data);
}
