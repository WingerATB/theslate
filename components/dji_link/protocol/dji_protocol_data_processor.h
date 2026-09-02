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

#ifndef DJI_PROTOCOL_DATA_PROCESSOR_H
#define DJI_PROTOCOL_DATA_PROCESSOR_H

#include <stdint.h>
#include <stddef.h>

#include "dji_protocol_data_descriptors.h"

const data_descriptor_t *find_data_descriptor(uint8_t cmd_set, uint8_t cmd_id);

int data_parser_by_structure(uint8_t cmd_set, uint8_t cmd_id, uint8_t cmd_type, const uint8_t *data, size_t data_length, void *output);

uint8_t* data_creator_by_structure(uint8_t cmd_set, uint8_t cmd_id, uint8_t cmd_type, const void *structure, size_t *data_length);

#endif