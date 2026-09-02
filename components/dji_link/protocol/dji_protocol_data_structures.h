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

#ifndef DJI_PROTOCOL_DATA_STRUCTURES_H
#define DJI_PROTOCOL_DATA_STRUCTURES_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// Define command and response frame structures
// 定义命令帧、应答帧结构体
typedef struct __attribute__((packed)) {
    uint32_t device_id;            // Device ID
                                   // 设备ID
    uint8_t mode;                  // Mode, refer to camera status in camera status push
                                   // 模式，参考相机状态推送中的相机状态
    uint8_t reserved[4];           // Reserved field
                                   // 保留字段
} camera_mode_switch_command_frame_t;

typedef struct __attribute__((packed)) {
    uint8_t ret_code;              // Return code: 0 for success, non-zero for failure
                                   // 返回码：0表示切换成功，非0表示切换失败
    uint8_t reserved[4];           // Reserved field
                                   // 保留字段
} camera_mode_switch_response_frame_t;

typedef struct __attribute__((packed)) {
    uint16_t ack_result;           // Acknowledgment result
                                   // 应答结果
    uint8_t product_id[16];        // Product ID, e.g., DJI-RS3
                                   // 产品ID，如 DJI-RS3
    uint8_t sdk_version[];         // SDK version data (flexible array)
                                   // sdk version 的数据（柔性数组）
} version_query_response_frame_t;

typedef struct __attribute__((packed)) {
    uint32_t device_id;            // Device ID
                                   // 设备ID
    uint8_t record_ctrl;           // Recording control: 0 - Start, 1 - Stop
                                   // 拍录控制：0 - 开始拍录，1 - 停止拍录
    uint8_t reserved[4];           // Reserved field
                                   // 预留字段
} record_control_command_frame_t;

typedef struct __attribute__((packed)) {
    uint8_t ret_code;              // Return code (refer to common return codes)
                                   // 返回码（参考普通返回码）
} record_control_response_frame_t;

typedef struct __attribute__((packed)) {
    int32_t year_month_day;        // Date (year*10000 + month*100 + day)
                                   // 年月日 (year*10000 + month*100 + day)
    int32_t hour_minute_second;    // Time ((hour+8)*10000 + minute*100 + second)
                                   // 时分秒 ((hour+8)*10000 + minute*100 + second)
    int32_t gps_longitude;         // Longitude (value = actual value * 10^7)
                                   // 经度 (value = 实际值 * 10^7)
    int32_t gps_latitude;          // Latitude (value = actual value * 10^7)
                                   // 纬度 (value = 实际值 * 10^7)
    int32_t height;                // Height in mm
                                   // 高度 单位：mm
    float speed_to_north;          // Speed to north in cm/s
                                   // 向北速度 单位：cm/s
    float speed_to_east;           // Speed to east in cm/s
                                   // 向东速度 单位：cm/s
    float speed_to_wnward;         // Speed downward in cm/s
                                   // 向下降速度 单位：cm/s
    uint32_t vertical_accuracy;    // Vertical accuracy estimate in mm
                                   // 垂直精度估计 单位：mm
    uint32_t horizontal_accuracy;  // Horizontal accuracy estimate in mm
                                   // 水平精度估计 单位：mm
    uint32_t speed_accuracy;       // Speed accuracy estimate in cm/s
                                   // 速度精度估计 单位：cm/s
    uint32_t satellite_number;     // Number of satellites
                                   // 卫星数量
} gps_data_push_command_frame;

typedef struct __attribute__((packed)) {
    uint8_t ret_code;              // Return code
                                   // 返回码
} gps_data_push_response_frame;

typedef struct __attribute__((packed)) {
    uint32_t device_id;            // Device ID
                                   // 设备ID
    uint8_t mac_addr_len;          // MAC address length
                                   // MAC地址长度
    int8_t mac_addr[16];           // MAC address
                                   // MAC地址
    uint32_t fw_version;           // Firmware version
                                   // 固件版本
    uint8_t conidx;                // Reserved field
                                   // 保留字段
    uint8_t verify_mode;           // Verification mode
                                   // 验证模式
    uint16_t verify_data;          // Verification data or result
                                   // 验证数据或结果
    uint8_t reserved[4];           // Reserved field
                                   // 预留字段
} connection_request_command_frame;

typedef struct __attribute__((packed)) {
    uint32_t device_id;            // Device ID
                                   // 设备ID
    uint8_t ret_code;              // Return code
                                   // 返回码
    uint8_t reserved[4];           // Reserved field
                                   // 预留字段
} connection_request_response_frame;

typedef struct __attribute__((packed)) {
    uint8_t push_mode;             // Push mode: 0-Off, 1-Single, 2-Periodic, 3-Periodic+Status change
                                   // 推送模式：0-关闭，1-单次，2-周期，3-周期+状态变化后推送一次
    uint8_t push_freq;             // Push frequency in 0.1Hz, only 20 is allowed
                                   // 推送频率，单位：0.1Hz，这里只能填 20，固定频率为 2Hz，不可调整
    uint8_t reserved[4];           // Reserved field
                                   // 预留字段
} camera_status_subscription_command_frame;

typedef struct __attribute__((packed)) {
    uint8_t camera_mode;           // Camera mode: 0x00-Slow Motion, 0x01-Video, 0x02-Timelapse, 0x05-Photo, 0x0A-Hyperlapse, 0x1A-Live stream, 0x23-UVC live stream, 0x28-Low light video (Super night scene in Osmo Action 5 Pro), 0x34-Subject follow, others-Use new protocol (refer to New camera status push 1D06)
                                   // 相机模式：0x00 - 慢动作，0x01 - 视频，0x02 - 静止延时，0x05 - 拍照，0x0A - 运动延时，0x1A - 直播，0x23 - UVC直播，0x28 - 低光视频（超级夜景），0x34 - 人物跟随，其它值 - 使用新协议（参考新相机状态推送1D06）
    uint8_t camera_status;         // Camera status: 0x00-Screen off, 0x01-Live view (including screen on but not recording), 0x02-Playback, 0x03-Photo/video in progress, 0x05-Pre-recording
                                   // 相机状态：0x00 - 屏幕关闭，0x01 - 直播（包括亮屏未录制），0x02 - 回放，0x03 - 拍照或录像中，0x05 - 预录制中
    uint8_t video_resolution;      // Video resolution: 10-1080P, 16-4K 16:9, 45-2.7K 16:9, 66-1080P 9:16, 67-2.7K 9:16, 95-2.7K 4:3, 103-4K 4:3, 109-4K 9:16; Photo format (Osmo Action): 4-L, 3-M; Photo format (Osmo 360): 4-Ultra Wide 30MP, 3-Wide 20MP, 2-Standard 12MP
                                   // 视频分辨率：10 - 1080P，16 - 4K 16:9，45 - 2.7K 16:9，66 - 1080P 9:16，67 - 2.7K 9:16，95 - 2.7K 4:3，103 - 4K 4:3，109 - 4K 9:16；拍照画幅（Osmo Action）：4 - L，3 - M；拍照画幅（Osmo 360）：4 - Ultra Wide 30MP，3 - Wide 20MP，2 - Standard 12MP
    uint8_t fps_idx;               // Frame rate: 1-24fps, 2-25fps, 3-30fps, 4-48fps, 5-50fps, 6-60fps, 10-100fps, 7-120fps, 19-200fps, 8-240fps; In Slow Motion mode: multiplier = fps/30; In photo mode: burst count (1-single photo, >1-burst count)
                                   // 帧率：1 - 24fps，2 - 25fps，3 - 30fps，4 - 48fps，5 - 50fps，6 - 60fps，10 - 100fps，7 - 120fps，19 - 200fps，8 - 240fps；慢动作模式时：倍率 = 帧率/30；拍照模式时：连拍数（1 - 普通拍照只拍一张，>1 - 连拍张数）
    uint8_t eis_mode;              // Electronic image stabilization mode: 0-Off, 1-RS, 2-HS, 3-RS+, 4-HB
                                   // 电子防抖模式：0 - 关闭，1 - RS，2 - HS，3 - RS+，4 - HB
    uint16_t record_time;          // Current recording time (including pre-recording duration) in seconds; In burst mode: burst time limit in milliseconds
                                   // 当前录像时间（包括预录制时长），单位：秒；连拍状态下：连拍时限，单位：毫秒
    uint8_t fov_type;              // FOV type, reserved field
                                   // FOV类型，保留字段
    uint8_t photo_ratio;           // Photo aspect ratio: 0-4:3, 1-16:9
                                   // 照片比例：0 - 4:3，1 - 16:9
    uint16_t real_time_countdown;  // Real-time countdown in seconds
                                   // 实时倒计时，单位：秒
    uint16_t timelapse_interval;   // In Timelapse mode: shooting interval in 0.1s (e.g., for 0.5s interval, value is 5); In Hyperlapse mode: shooting rate (0 for Auto option)
                                   // 静止延时摄影模式下：拍摄时间间隔，单位：0.1秒（例如间隔0.5秒时值为5）；运动延时摄影模式下：拍摄速率（Auto选项下值为0）
    uint16_t timelapse_duration;   // Time-lapse recording duration in seconds
                                   // 延时录像时长，单位：秒
    uint32_t remain_capacity;      // Remaining SD card capacity in MB
                                   // SD卡剩余容量，单位：MB
    uint32_t remain_photo_num;     // Remaining number of photos
                                   // 剩余拍照张数
    uint32_t remain_time;          // Remaining recording time in seconds
                                   // 剩余录像时间，单位：秒
    uint8_t user_mode;             // User mode (invalid values treated as 0): 0-General mode, 1-Custom mode 1, 2-Custom mode 2, 3-Custom mode 3, 4-Custom mode 4, 5-Custom mode 5
                                   // 用户模式（非法值按0处理）：0 - 通用模式，1 - 自定义模式1，2 - 自定义模式2，3 - 自定义模式3，4 - 自定义模式4，5 - 自定义模式5
    uint8_t power_mode;            // Power mode: 0-Normal working mode, 3-Sleep mode
                                   // 电源模式：0 - 正常工作模式，3 - 休眠模式
    uint8_t camera_mode_next_flag; // Pre-switch flag: In pre-switch mode (e.g., QS), only shows mode name and icon without specific parameters. Indicates target mode to switch to; if not in pre-switch mode, equals camera_mode
                                   // 预切换标志：在预切换（例如QS）模式下，仅显示模式名称和图标，不展示具体参数。表示即将切换的目标模式；若当前不处于预切换模式，则该字段表示当前模式，与camera_mode保持一致
    uint8_t temp_over;             // Camera error status: 0-Normal temperature, 1-Temperature warning (can record but high temperature), 2-High temperature (cannot record), 3-Overheating (will shut down)
                                   // 相机发生了错误：0 - 温度正常，1 - 温度警告（可以录制但温度比较高），2 - 温度高（不可以录制），3 - 温度过高（要关机了）
    uint32_t photo_countdown_ms;   // Photo countdown parameter in milliseconds (remote controller converts to 0.5s, 1s, 2s, 3s, 5s, 10s options)
                                   // 拍照倒计时参数，单位：毫秒（遥控器转换为0.5s，1s，2s，3s，5s，10s几个档显示）
    uint16_t loop_record_sends;    // Loop recording duration in seconds (remote controller shows as off, max, 5m, 20m, 1h options, where off=0, max=65535)
                                   // 循环录像时长，单位：秒（遥控器转为off，max，5m，20m，1h几档显示，其中off=0，max=65535）
    uint8_t camera_bat_percentage; // Camera battery percentage: 0-100%
                                   // 相机电池电量：0~100%
} camera_status_push_command_frame;

typedef struct __attribute__((packed)) {
    uint8_t type_mode_name;        // Fixed to 0x01
                                   // 固定为 0x01
    uint8_t mode_name_length;      // Mode name length
                                   // 模式名字长度
    uint8_t mode_name[21];         // Mode name, ASCII code, max 20 bytes
                                   // 模式名字，ASCII码，最长不超过20字节
    uint8_t type_mode_param;       // Fixed to 0x02
                                   // 固定为 0x02
    uint8_t mode_param_length;     // Mode parameter length, ASCII code, max 20 bytes
                                   // 模式参数长度，ASCII码，最长不超过20字节
    uint8_t mode_param[21];        // Mode parameters
                                   // 模式参数
} new_camera_status_push_command_frame;

typedef struct __attribute__((packed)) {
    uint8_t key_code;              // Key code
                                   // 按键代码
    uint8_t mode;                  // Report mode: 0x00-Report key press/release status, 0x01-Report key events
                                   // 上报模式选择：0x00 上报按键按下/松开状态，0x01 上报按键事件
    uint16_t key_value;            // Key event value: For mode 0: 0x00-Key pressed, 0x01-Key released; For mode 1: 0x00-Short press, 0x01-Long press, 0x02-Double click, 0x03-Triple click, 0x04-Quadruple click
                                   // 按键事件值: 当 mode 为 0 时，0x00 按键按下，0x01 按键松开；当 mode 为 1 时，按键事件类型: 0x00 短按事件，0x01 长按事件，0x02 双击事件，0x03 三击事件，0x04 四击事件
} key_report_command_frame_t;

typedef struct __attribute__((packed)) {
    uint8_t ret_code;             // Return code (refer to common return codes)
                                  // 返回码（参考普通返回码）
} key_report_response_frame_t;

#endif