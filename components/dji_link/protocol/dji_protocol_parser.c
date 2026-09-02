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

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "custom_crc16.h"
#include "custom_crc32.h"

#include "dji_protocol_data_processor.h"
#include "dji_protocol_parser.h"

#define TAG "DJI_PROTOCOL_PARSER"

/* Protocol frame field length definitions */
/* 协议帧部分长度定义 */

// SOF start byte
// SOF 起始字节
#define PROTOCOL_SOF_LENGTH 1
// Ver/Length field
// Ver/Length 字段
#define PROTOCOL_VER_LEN_LENGTH 2
// CmdType
#define PROTOCOL_CMD_TYPE_LENGTH 1
// ENC encryption field
// ENC 加密字段
#define PROTOCOL_ENC_LENGTH 1
// RES reserved bytes
// RES 保留字节
#define PROTOCOL_RES_LENGTH 3
// SEQ sequence number
// SEQ 序列号
#define PROTOCOL_SEQ_LENGTH 2
// CRC-16 checksum
// CRC-16 校验
#define PROTOCOL_CRC16_LENGTH 2
// CmdSet field
// CmdSet 字段
#define PROTOCOL_CMD_SET_LENGTH 1
// CmdID field
// CmdID 字段
#define PROTOCOL_CMD_ID_LENGTH 1
// CRC-32 checksum
// CRC-32 校验
#define PROTOCOL_CRC32_LENGTH 4

/**
 * Define header length (excluding CmdSet, CmdID and payload)
 * 定义帧头长度（不包含 CmdSet、CmdID 和有效载荷）
 */
#define PROTOCOL_HEADER_LENGTH ( \
    PROTOCOL_SOF_LENGTH +        \
    PROTOCOL_VER_LEN_LENGTH +    \
    PROTOCOL_CMD_TYPE_LENGTH +   \
    PROTOCOL_ENC_LENGTH +        \
    PROTOCOL_RES_LENGTH +        \
    PROTOCOL_SEQ_LENGTH +        \
    PROTOCOL_CRC16_LENGTH +      \
    PROTOCOL_CMD_SET_LENGTH +    \
    PROTOCOL_CMD_ID_LENGTH)

/**
 * Define tail length (only includes CRC-32)
 * 定义帧尾长度（仅包含 CRC-32）
 */
#define PROTOCOL_TAIL_LENGTH PROTOCOL_CRC32_LENGTH

/**
 * Define total frame length macro (dynamic calculation, including DATA segment)
 * 定义帧总长度宏（动态计算，包含 DATA 段）
 */
#define PROTOCOL_FULL_FRAME_LENGTH(data_length) ( \
    PROTOCOL_HEADER_LENGTH +                      \
    (data_length) +                               \
    PROTOCOL_TAIL_LENGTH)



/**
 * Parse notification frame
 * 解析通知帧
 *
 * Takes raw frame data and length, returns parsed result in frame_out structure
 * 传入帧原始数据、帧长度，frame_out 结构体作为载体返回
 *
 * @param frame_data Raw frame data
 *                   帧原始数据
 * @param frame_length Frame length
 *                     帧长度
 * @param frame_out Output structure for parsed result
 *                  解析结果输出结构体
 *
 * @return 0 on success, negative value on failure
 *         成功返回 0，失败返回负值
 */
int protocol_parse_notification(const uint8_t *frame_data, size_t frame_length, protocol_frame_t *frame_out)
{
    // Check minimum frame length
    // 检查最小帧长度
    if (frame_length < 16)
    { // SOF(1) + Ver/Length(2) + CmdType(1) + ENC(1) + RES(3) + SEQ(2) + CRC-16(2) + CRC-32(4)
        ESP_LOGE(TAG, "Frame too short to be valid");
        return -1;
    }

    // Check frame header (SOF)
    // 检查帧头 (SOF)
    if (frame_data[0] != 0xAA)
    {
        ESP_LOGE(TAG, "Invalid SOF: 0x%02X", frame_data[0]);
        return -2;
    }

    // Parse Ver/Length
    // 解析 Ver/Length
    uint16_t ver_length = (frame_data[2] << 8) | frame_data[1];
    uint16_t version = ver_length >> 10;            // High 6 bits for version
                                                    // 高 6 位为版本号
    uint16_t expected_length = ver_length & 0x03FF; // Low 10 bits for frame length
                                                    // 低 10 位为帧长度

    if (expected_length != frame_length)
    {
        ESP_LOGE(TAG, "Frame length mismatch: expected %u, got %zu", expected_length, frame_length);
        return -3;
    }

    // Verify CRC-16
    // 验证 CRC-16
    uint16_t crc16_received = (frame_data[11] << 8) | frame_data[10];
    uint16_t crc16_calculated = calculate_crc16(frame_data, 10); // From SOF to SEQ
                                                                 // 从 SOF 到 SEQ
    if (crc16_received != crc16_calculated)
    {
        ESP_LOGE(TAG, "CRC-16 mismatch: received 0x%04X, calculated 0x%04X", crc16_received, crc16_calculated);
        return -4;
    }

    // Verify CRC-32
    // 验证 CRC-32
    uint32_t crc32_received = (frame_data[frame_length - 1] << 24) | (frame_data[frame_length - 2] << 16) |
                              (frame_data[frame_length - 3] << 8) | frame_data[frame_length - 4];
    uint32_t crc32_calculated = calculate_crc32(frame_data, frame_length - 4); // From SOF to DATA
                                                                               // 从 SOF 到 DATA
    if (crc32_received != crc32_calculated)
    {
        ESP_LOGE(TAG, "CRC-32 mismatch: received 0x%08X, calculated 0x%08X", (unsigned int)crc32_received, (unsigned int)crc32_calculated);
        return -5;
    }

    // Fill parsing results into structure
    // 填充解析结果到结构体
    frame_out->sof = frame_data[0];
    frame_out->version = version;
    frame_out->frame_length = expected_length;
    frame_out->cmd_type = frame_data[3];
    frame_out->enc = frame_data[4];
    memcpy(frame_out->res, &frame_data[5], 3);
    frame_out->seq = (frame_data[9] << 8) | frame_data[8];
    frame_out->crc16 = crc16_received;

    // Process data segment (DATA)
    // 处理数据段 (DATA)
    if (frame_length > 16)
    { // DATA segment exists
      // DATA 段存在
        frame_out->data = &frame_data[12];
        frame_out->data_length = frame_length - 16; // DATA length
                                                    // DATA 长度
    }
    else
    { // DATA segment is empty
      // DATA 段为空
        frame_out->data = NULL;
        frame_out->data_length = 0;
        ESP_LOGW(TAG, "DATA segment is empty");
    }

    frame_out->crc32 = crc32_received;

    ESP_LOGI(TAG, "Frame parsed successfully");
    return 0;
}

/**
 * @brief Parse data segment from protocol frame
 *        解析协议帧中的数据段
 *
 * Takes DATA segment, length and command type, returns parsed result length through data_length_without_cmd_out
 * 传入 DATA 数据段、长度和命令类型，data_length_without_cmd_out 返回上层
 *
 * @param data Raw data segment
 *             原始数据段
 * @param data_length Length of data segment
 *                    数据段长度
 * @param cmd_type Command type
 *                 命令类型
 * @param data_length_without_cmd_out Output parameter for data length without cmdSet&CmdID
 *                                    不包含 cmdSet&CmdID 的数据长度输出参数
 *
 * @return void* Pointer to parsed result structure, NULL on failure
 *               指向解析结果结构体的指针，失败时返回 NULL
 */
void *protocol_parse_data(const uint8_t *data, size_t data_length, uint8_t cmd_type, size_t *data_length_without_cmd_out)
{
    if (data == NULL || data_length < 2)
    {
        ESP_LOGE(TAG, "Invalid data segment: data is NULL or too short");
        return NULL;
    }

    uint8_t cmd_set = data[0];
    uint8_t cmd_id = data[1];

    // 查找对应的命令描述符
    // Find corresponding command descriptor
    const data_descriptor_t *descriptor = find_data_descriptor(cmd_set, cmd_id);

    if (descriptor == NULL)
    {
        ESP_LOGW(TAG, "No descriptor found for CmdSet 0x%02X and CmdID 0x%02X by trying structure descriptor", cmd_set, cmd_id);
        return NULL;
    }

    // 取出应答帧数据
    // Extract response frame data
    const uint8_t *response_data = &data[2];
    size_t response_length = data_length - 2;

    ESP_LOGI(TAG, "CmdSet: 0x%02X, CmdID: 0x%02X", cmd_set, cmd_id);

    void *response_struct = malloc(response_length);
    if (response_struct == NULL)
    {
        ESP_LOGE(TAG, "Memory allocation failed for parsed data");
        return NULL;
    }

    int result = -1;
    if (descriptor != NULL)
    {
        result = data_parser_by_structure(cmd_set, cmd_id, cmd_type, response_data, response_length, response_struct);
    }

    if (result == 0)
    {
        ESP_LOGI(TAG, "Data parsed successfully for CmdSet 0x%02X and CmdID 0x%02X", cmd_set, cmd_id);
        if (data_length_without_cmd_out != NULL)
        {
            *data_length_without_cmd_out = response_length;
        }
    }
    else if (result == -2)
    {
        ESP_LOGW(TAG, "Parser function is NULL for CmdSet 0x%02X and CmdID 0x%02X by trying structure descriptor", cmd_set, cmd_id);
        free(response_struct);
        return NULL;
    }
    else
    {
        ESP_LOGE(TAG, "Failed to parse data for CmdSet 0x%02X and CmdID 0x%02X", cmd_set, cmd_id);
        free(response_struct);
        return NULL;
    }

    return response_struct;
}

/**
 * @brief Create protocol frame
 *        创建协议帧
 *
 * Creates a complete protocol frame with given parameters and data structure
 * 根据给定的参数和数据结构创建完整的协议帧
 *
 * @param cmd_set Command set
 *                命令集
 * @param cmd_id Command ID
 *               命令 ID
 * @param cmd_type Command type
 *                 命令类型
 * @param structure Pointer to data structure
 *                 数据结构指针
 * @param seq Sequence number
 *            序列号
 * @param frame_length_out Output parameter for total frame length
 *                        总帧长度输出参数
 *
 * @return uint8_t* Pointer to created frame buffer, NULL on failure
 *                  指向创建的帧缓冲区的指针，失败时返回 NULL
 */
uint8_t *protocol_create_frame(uint8_t cmd_set, uint8_t cmd_id, uint8_t cmd_type, const void *structure, uint16_t seq, size_t *frame_length_out)
{
    size_t data_length = 0;
    uint8_t *payload_data = NULL;

    // Create payload data from structure
    // 从结构体创建有效载荷数据
    payload_data = data_creator_by_structure(cmd_set, cmd_id, cmd_type, structure, &data_length);

    // Handle empty payload case
    // 处理空数据段的情况
    if (payload_data == NULL && data_length > 0)
    {
        ESP_LOGE(TAG, "Failed to create payload data with non-zero length");
        return NULL;
    }

    // Calculate total frame length
    // 计算总帧长度
    *frame_length_out = PROTOCOL_HEADER_LENGTH + data_length + PROTOCOL_TAIL_LENGTH;

    // Allocate memory for complete frame
    // 为完整帧分配内存
    uint8_t *frame = (uint8_t *)malloc(*frame_length_out);
    if (frame == NULL)
    {
        ESP_LOGE(TAG, "Memory allocation failed for protocol frame");
        free(payload_data);
        return NULL;
    }

    // Initialize frame content
    // 初始化帧内容
    memset(frame, 0, *frame_length_out);

    // Fill protocol header
    // 填充协议头部
    size_t offset = 0;
    frame[offset++] = 0xAA; // SOF start byte
                            // SOF 起始字节

    // Ver/Length field
    // Ver/Length 字段
    uint16_t version = 0; // Fixed version number
                          // 固定版本号
    uint16_t ver_length = (version << 10) | (*frame_length_out & 0x03FF);
    frame[offset++] = ver_length & 0xFF;        // Ver/Length low byte
                                                // Ver/Length 低字节
    frame[offset++] = (ver_length >> 8) & 0xFF; // Ver/Length high byte
                                                // Ver/Length 高字节

    // Fill command type
    // 填充命令类型
    frame[offset++] = cmd_type;

    // ENC (no encryption, fixed 0)
    // ENC（不加密，固定 0）
    frame[offset++] = 0x00;

    // RES (reserved bytes, fixed 0)
    // RES（保留字节，固定 0）
    frame[offset++] = 0x00;
    frame[offset++] = 0x00;
    frame[offset++] = 0x00;

    // Sequence number
    // 序列号
    frame[offset++] = seq & 0xFF;        // Low byte of sequence number
                                         // 序列号低字节
    frame[offset++] = (seq >> 8) & 0xFF; // High byte of sequence number
                                         // 序列号高字节

    // Calculate and fill CRC-16 (covers from SOF to SEQ)
    // 计算并填充 CRC-16（覆盖从 SOF 到 SEQ）
    uint16_t crc16 = calculate_crc16(frame, offset);
    frame[offset++] = crc16 & 0xFF;        // CRC-16 low byte
                                           // CRC-16 低字节
    frame[offset++] = (crc16 >> 8) & 0xFF; // CRC-16 high byte
                                           // CRC-16 高字节

    // Fill command set and ID
    // 填充命令集和命令 ID
    frame[offset++] = cmd_set;
    frame[offset++] = cmd_id;

    // Fill payload data
    // 填充有效载荷数据
    memcpy(&frame[offset], payload_data, data_length);
    offset += data_length;

    // Calculate and fill CRC-32 (covers from SOF to DATA)
    // 计算并填充 CRC-32（覆盖从 SOF 到 DATA）
    uint32_t crc32 = calculate_crc32(frame, offset);
    frame[offset++] = crc32 & 0xFF;         // CRC-32 byte 1
                                            // CRC-32 第 1 字节
    frame[offset++] = (crc32 >> 8) & 0xFF;  // CRC-32 byte 2
                                            // CRC-32 第 2 字节
    frame[offset++] = (crc32 >> 16) & 0xFF; // CRC-32 byte 3
                                            // CRC-32 第 3 字节
    frame[offset++] = (crc32 >> 24) & 0xFF; // CRC-32 byte 4
                                            // CRC-32 第 4 字节

    // Free payload data
    // 释放有效载荷数据
    free(payload_data);

    return frame;
}
