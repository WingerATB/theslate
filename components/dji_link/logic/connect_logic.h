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

#ifndef __CONNECT_LOGIC_H__
#define __CONNECT_LOGIC_H__

/* SLATE ADDITION: this header has always used bool and uint32_t while relying
 * on every includer to have pulled them in first. Say so here instead. */
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    BLE_NOT_INIT = -1,
    BLE_INIT_COMPLETE = 0,
    BLE_SEARCHING = 1,
    BLE_CONNECTED = 2,
    PROTOCOL_CONNECTED = 3,
    BLE_DISCONNECTING = 4,   // Actively disconnecting state
                             // 主动断开连接中状态
} connect_state_t;

connect_state_t connect_logic_get_state(void);

/* SLATE ADDITION: is there a camera link right now?
 *
 * The enum above is NOT ordered by how connected the module is -- BLE_CONNECTED
 * is 2 while BLE_DISCONNECTING is 4 -- so a comparison like
 * "state >= BLE_CONNECTED" answers a different question from the one it looks
 * like it is asking, and reads a link that is on its way down as up. Ask this
 * instead; it names the two states that actually mean a camera is reachable,
 * and it keeps meaning that if the enum grows again. */
bool connect_logic_link_up(void);

int connect_logic_ble_init();

int connect_logic_ble_connect(bool is_reconnecting);

int connect_logic_ble_disconnect(void);

int connect_logic_protocol_connect(uint32_t device_id, uint8_t mac_addr_len, const int8_t *mac_addr,
                                    uint32_t fw_version, uint8_t verify_mode, uint16_t verify_data,
                                    uint8_t camera_reserved);

int connect_logic_ble_wakeup(void);

#endif