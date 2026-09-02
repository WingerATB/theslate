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

#include <string.h>
#include <stdio.h>
#include "esp_log.h"

#include "dji_protocol_data_processor.h"

#define TAG "DJI_PROTOCOL_DATA_PROCESSOR"

/**
 * @brief Find data descriptor by command set and command ID
 *        根据命令集和命令ID查找对应的数据描述符
 * 
 * @param cmd_set Command set
 *                命令集
 * @param cmd_id Command ID
 *               命令ID
 * @return Return pointer to found data descriptor, NULL if not found
 *         返回找到的数据描述符指针，如果未找到则返回NULL
 */
const data_descriptor_t *find_data_descriptor(uint8_t cmd_set, uint8_t cmd_id) {
    for (size_t i = 0; i < DATA_DESCRIPTORS_COUNT; ++i) {
        if (data_descriptors[i].cmd_set == cmd_set && data_descriptors[i].cmd_id == cmd_id) {
            return &data_descriptors[i];
        }
    }
    return NULL;
}

/**
 * @brief Parse data according to structure
 *        根据结构体解析数据
 * 
 * @param cmd_set Command set
 *                命令集
 * @param cmd_id Command ID
 *               命令ID
 * @param cmd_type Command type
 *                 命令类型
 * @param data Data to be parsed
 *             待解析的数据
 * @param data_length Data length
 *                    数据长度
 * @param structure_out Output structure pointer
 *                      输出结构体指针
 * @return Return 0 on success, -1 or -2 on failure
 *         成功返回0，失败返回-1或-2
 */
int data_parser_by_structure(uint8_t cmd_set, uint8_t cmd_id, uint8_t cmd_type, const uint8_t *data, size_t data_length, void *structure_out) {
    ESP_LOGI(TAG, "Parsing CmdSet: 0x%02X, CmdID: 0x%02X, CmdType: 0x%02X", cmd_set, cmd_id, cmd_type);

    // Find corresponding descriptor
    // 查找对应的命令描述符
    const data_descriptor_t *descriptor = find_data_descriptor(cmd_set, cmd_id);

    // Check if parser function exists
    // 检查解析函数是否存在
    if (descriptor->parser == NULL) {
        ESP_LOGW(TAG, "Parser function is NULL for CmdSet: 0x%02X, CmdID: 0x%02X", cmd_set, cmd_id);
        return -2;
    }

    return descriptor->parser(data, data_length, structure_out, cmd_type);
}

/**
 * @brief Create data according to structure
 *        根据结构体创建数据
 * 
 * @param cmd_set Command set
 *                命令集
 * @param cmd_id Command ID
 *               命令ID
 * @param cmd_type Command type
 *                 命令类型
 * @param structure Input structure pointer
 *                  输入结构体指针
 * @param data_length Output data length
 *                    输出数据长度
 * @return Return pointer to created data buffer, NULL on failure
 *         返回创建的数据缓冲区指针，失败返回NULL
 */
uint8_t* data_creator_by_structure(uint8_t cmd_set, uint8_t cmd_id, uint8_t cmd_type, const void *structure, size_t *data_length) {
    // Find corresponding descriptor
    // 查找对应的命令描述符
    const data_descriptor_t *descriptor = find_data_descriptor(cmd_set, cmd_id);
    if (descriptor == NULL) {
        ESP_LOGW(TAG, "Descriptor not found for CmdSet: 0x%02X, CmdID: 0x%02X", cmd_set, cmd_id);
        return NULL;
    }

    // Check if creator function exists
    // 检查创建函数是否存在
    if (descriptor->creator == NULL) {
        ESP_LOGW(TAG, "Creator function is NULL for CmdSet: 0x%02X, CmdID: 0x%02X", cmd_set, cmd_id);
        return NULL;
    }

    return descriptor->creator(structure, data_length, cmd_type);
}
