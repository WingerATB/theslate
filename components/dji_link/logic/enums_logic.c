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

#include "enums_logic.h"

const char* camera_mode_to_string(camera_mode_t mode) {
    switch (mode) {
        case CAMERA_MODE_SLOW_MOTION:
            return "慢动作 / Slow Motion";
        case CAMERA_MODE_NORMAL:
            return "视频 / Video";
        case CAMERA_MODE_TIMELAPSE:
            return "静止延时 / Timelapse";
        case CAMERA_MODE_PHOTO:
            return "拍照 / Photo";
        case CAMERA_MODE_HYPERLAPSE:
            return "运动延时 / Hyperlapse";
        case CAMERA_MODE_LIVE_STREAMING:
            return "直播 / Live Streaming";
        case CAMERA_MODE_UVC_STREAMING:
            return "UVC 直播 / UVC Live Streaming";
        case CAMERA_MODE_SUPERNIGHT:
            return "低光视频（超级夜景）/ SuperNight";
        case CAMERA_MODE_SUBJECT_TRACKING:
            return "人物跟随 / Subject Tracking";

        case CAMERA_MODE_PANORAMIC_VIDEO_360:
            return "全景视频 / Panoramic Video (Osmo360)";
        case CAMERA_MODE_HYPERLAPSE_360:
            return "运动延时 / Hyperlapse (Osmo360)";
        case CAMERA_MODE_SELFIE_360:
            return "自拍模式 / Selfie Mode (Osmo360)";
        case CAMERA_MODE_PANORAMIC_PHOTO_360:
            return "全景拍照 / Panoramic Photo (Osmo360)";
        case CAMERA_MODE_BOOST_VIDEO_360:
            return "极广角视频 / Boost Video (Osmo360)";
        case CAMERA_MODE_VORTEX_360:
            return "时空凝固 / Vortex (Osmo360)";
        case CAMERA_MODE_PANORAMIC_SUPERNIGHT_360:
            return "全景超级夜景 / 360° SuperNight (Osmo360)";
        case CAMERA_MODE_SINGLE_LENS_SUPERNIGHT_360:
            return "单镜头超级夜景 / Single Lens SuperNight (Osmo360)";

        default:
            return "未知模式 / Unknown mode";
    }
}

const char* camera_status_to_string(camera_status_t status) {
    switch (status) {
        case CAMERA_STATUS_SCREEN_OFF:
            return "屏幕关闭 / Screen off";
        case CAMERA_STATUS_LIVE_STREAMING:
            return "直播 / Live streaming (including screen-on without recording)";
        case CAMERA_STATUS_PLAYBACK:
            return "回放 / Playback";
        case CAMERA_STATUS_PHOTO_OR_RECORDING:
            return "拍照或录像中 / Photo or recording";
        case CAMERA_STATUS_PRE_RECORDING:
            return "预录制中 / Pre-recording";
        default:
            return "未知状态 / Unknown status";
    }
}

const char* video_resolution_to_string(video_resolution_t res) {
    switch (res) {
        case VIDEO_RESOLUTION_1080P: return "1920x1080P";
        case VIDEO_RESOLUTION_4K_16_9: return "4096x2160P 4K 16:9";
        case VIDEO_RESOLUTION_2K_16_9: return "2720x1530P 2.7K 16:9";
        case VIDEO_RESOLUTION_1080P_9_16: return "1920x1080P 9:16";
        case VIDEO_RESOLUTION_2K_9_16: return "2720x1530P 9:16";
        case VIDEO_RESOLUTION_2K_4_3: return "2720x2040P 2.7K 4:3";
        case VIDEO_RESOLUTION_4K_4_3: return "4096x3072P 4K 4:3";
        case VIDEO_RESOLUTION_4K_9_16: return "4096x2160P 4K 9:16";
        case VIDEO_RESOLUTION_L: return "拍照画幅 L / Ultra Wide 30MP (Osmo360)";
        case VIDEO_RESOLUTION_M: return "拍照画幅 M / Wide 20MP (Osmo360)";
        case VIDEO_RESOLUTION_S: return "Standard 12MP (Osmo360)";
        default: return "未知分辨率";
    }
}

const char* fps_idx_to_string(fps_idx_t fps) {
    switch (fps) {
        case FPS_24: return "24fps";
        case FPS_25: return "25fps";
        case FPS_30: return "30fps";
        case FPS_48: return "48fps";
        case FPS_50: return "50fps";
        case FPS_60: return "60fps";
        case FPS_100: return "100fps";
        case FPS_120: return "120fps";
        case FPS_200: return "200fps";
        case FPS_240: return "240fps";
        default: return "未知帧率 / Unknown FPS";
    }
}

const char* eis_mode_to_string(eis_mode_t mode) {
    switch (mode) {
        case EIS_MODE_OFF: return "关闭 / Off";
        case EIS_MODE_RS: return "RS";
        case EIS_MODE_RS_PLUS: return "RS+";
        case EIS_MODE_HB: return "HB";
        case EIS_MODE_HS: return "HS";
        default: return "未知防抖模式 / Unknown EIS mode";
    }
}
