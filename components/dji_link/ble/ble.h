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

#ifndef __BLE_H__
#define __BLE_H__

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "esp_err.h"
#include "esp_gatt_defs.h"
#include "esp_gattc_api.h"
#include "camvendor.h"
#include "esp_gap_ble_api.h"

/* Connection status structure */
/* 连接状态结构体 */
typedef struct {
    bool is_connected; // Connection status
} connection_status_t;

/* Handle discovery status structure */
/* 特征句柄查找状态结构体 */
typedef struct {
    bool notify_char_handle_found; // Notify characteristic handle found
    bool write_char_handle_found;  // Write characteristic handle found
} handle_discovery_t;

/* Global profile structure to manage connection and characteristic information */
/* 为了简化，做一个全局的 profile 结构体来管理连接与特征信息 */
typedef struct {
    uint16_t conn_id;              // Connection ID
    esp_gatt_if_t gattc_if;        // GATT client interface

    /* Handles for characteristics we need to operate on */
    /* 根据需要记录我们要操作的特征 handle */
    uint16_t notify_char_handle;   // Notify characteristic handle
    uint16_t write_char_handle;    // Write characteristic handle
    uint8_t  write_char_props;     // CAMLINK: properties of the write char, so
                                   // the correct write type can be chosen
    uint16_t read_char_handle;     // Read characteristic handle

    /* Start and end handles of the service */
    /* service 的起始和结束 handle */
    uint16_t service_start_handle; // Service start handle
    uint16_t service_end_handle;   // Service end handle

    /* Remote device address */
    /* 远程设备地址 */
    esp_bd_addr_t remote_bda;      // Remote Bluetooth device address

    connection_status_t connection_status;     // Connection status
    handle_discovery_t handle_discovery;       // Handle discovery status
} ble_profile_t;

extern ble_profile_t s_ble_profile;

/**
 * @brief Notify callback function type for receiving data from remote
 * Notify 回调函数类型，用于接收从远端发来的数据
 *
 * @param data   Pointer to the notification data
 *               通知的数据指针
 * @param length Length of the notification data
 *               通知的数据长度
 */
typedef void (*ble_notify_callback_t)(const uint8_t *data, size_t length);

typedef void (*connect_logic_state_callback_t)(void);

esp_err_t ble_init();

esp_err_t ble_start_scanning_and_connect(void);

void ble_set_reconnecting(bool flag);

bool ble_get_reconnecting(void);

esp_err_t ble_reconnect(void);

esp_err_t ble_disconnect(void);


esp_err_t ble_read(uint16_t conn_id, uint16_t handle);

esp_err_t ble_write_without_response(uint16_t conn_id, uint16_t handle, const uint8_t *data, size_t length);

esp_err_t ble_write_with_response(uint16_t conn_id, uint16_t handle, const uint8_t *data, size_t length);

esp_err_t ble_register_notify(uint16_t conn_id, uint16_t char_handle);

esp_err_t ble_unregister_notify(uint16_t conn_id, uint16_t char_handle);

void ble_set_notify_callback(ble_notify_callback_t cb);

void ble_set_state_callback(connect_logic_state_callback_t cb);

esp_err_t ble_start_advertising(void);

/* SLATE ADDITION: radio tuning for a 20 cm link on a quad.
 * _static  -> DEFAULT/ADV/SCAN power, call after esp_bt_controller_enable().
 * _conn    -> connection power, only valid once a connection exists.
 * _slow_conn_params -> request a 30-50 ms connection interval. */
/* CAMLINK: which cameras this module is allowed to connect to.
 *
 * ONE SCAN DECIDES. The scan window already sees every camera that is
 * advertising, so the choice between them is made from that single window
 * rather than by trying each in turn: whichever bound camera is present with
 * the LOWEST INDEX wins, and RSSI only breaks a tie between adverts from the
 * same camera.
 *
 * Trying them one at a time was the obvious implementation and it is subtly
 * wrong as well as slow. A camera that is switched on but slow to advertise
 * would time out and hand the link to a lower-priority one that answered
 * quicker -- so "highest priority that is switched on" would quietly become
 * "whichever replied first". Seeing them all in one window cannot do that.
 *
 * Pass n = 0 to accept nothing at all, which is what a module with no camera
 * configured must do. */
#define BLE_BOUND_MAX  8
void ble_set_bound_list(const uint8_t (*addrs)[6], int n);

/* Equivalent to a one-camera list. Pass NULL to clear. */
void ble_set_bound_addr(const uint8_t *addr);

/* Which of the bound cameras the last scan actually chose. False when none was
 * seen. The caller looks it up in its own list to find out what protocol to
 * speak to it -- the BLE layer holds addresses and nothing else. */
bool ble_get_selected_addr(uint8_t out[6]);

/* True if this advertisement is a DJI camera. Exported so the config-mode
 * scanner identifies cameras by exactly the same rule the connect path uses --
 * a camera that shows up in the picker but is then rejected on connect, or the
 * reverse, would be worse than no picker at all. */
uint8_t bsp_link_is_dji_camera_adv(esp_ble_gap_cb_param_t *scan_result);

/* Override the compiled-in BLE transmit power. Takes an esp_power_level_t.
 * Call before ble_init(). Out-of-range values are ignored. */
void camlink_ble_set_tx_level(int lvl);

/* Apply a transmit level to the LIVE connection as well as to future ones.
 * set_tx_level only records the value for next time; this takes effect now. */
void camlink_ble_apply_tx_level(int lvl);

/* Remove all stored BLE bonds, for a clean re-pair. */
void camlink_ble_clear_bonds(void);

/* Why the camera link is not up. reason is an HCI disconnect code (0 if none),
 * count is how many failures since the last successful connection, and ever
 * says whether a link has been established at least once this boot. */
void camlink_ble_link_fault(uint8_t *reason, uint32_t *count, bool *ever);
const char *camlink_ble_fault_text(uint8_t reason);


/* Acknowledge unsolicited R SDK command frames from the camera. Without this an
 * Osmo 360 polls 0x00/0x12 unanswered and shows a firmware-update screen. */
void camlink_rsdk_autoreply(const uint8_t *frame, size_t len);
void camlink_rsdk_set_autoreply(bool on);
bool ble_has_bound_addr(void);

/* ===================== SLATE ADDITION ===================================
 * Vendor-aware transport.
 *
 * Scanning, connecting and bonding are the same job whoever made the camera,
 * so they stay here and stay shared -- there is one BLE controller, and the
 * priority scan has to see a DJI camera and a GoPro in the same window. What
 * differs per vendor is only WHICH service and characteristics to look for,
 * and that is described by a profile the session layer installs before the
 * connect.
 *
 * DJI keeps its existing behaviour exactly: one channel, 16-bit UUIDs, and the
 * same scalar handles the vendored code has always used.
 * ========================================================================= */
#define BLE_CHAN_MAX  4

/* One request/response pair. DJI has a single pair; GoPro has three (command,
 * settings and query), which is the reason this is a table rather than the two
 * scalars it replaces. */
typedef struct {
    esp_bt_uuid_t write;
    esp_bt_uuid_t notify;
} ble_chan_uuid_t;

typedef struct {
    const char     *name;
    esp_bt_uuid_t   service;
    ble_chan_uuid_t chan[BLE_CHAN_MAX];
    uint8_t         n_chan;
} ble_gatt_profile_t;

/* Which vendor made this advertisement, or CAM_VENDOR_NONE. The DJI arm is the
 * original bsp_link_is_dji_camera_adv() unchanged. */
cam_vendor_t camlink_cam_adv_vendor(esp_ble_gap_cb_param_t *scan_result);

/* The vendor of the camera the last scan chose. Known BEFORE the connect is
 * issued, which is what lets the session layer install the right profile and
 * the right notify handler rather than discovering the vendor afterwards. */
cam_vendor_t ble_selected_vendor(void);

/* Declare which GATT layout a vendor's cameras use.
 *
 * REGISTERED RATHER THAN SET, because of an ordering problem: discovery runs
 * as part of connecting, but which camera won the scan is only decided inside
 * that same call. There is no moment between "we know it is a GoPro" and "we
 * must know which characteristics to look for" in which a caller could set it.
 * So each vendor declares its layout once at start-up and the transport picks
 * the right one the instant the scan chooses an address.
 *
 * It also keeps the dependency pointing one way: the GoPro component registers
 * its profile with the transport, and the transport knows nothing about GoPro. */
void ble_register_vendor_profile(cam_vendor_t vendor, const ble_gatt_profile_t *profile);

/* Per-channel handles found during discovery. 0 when not found. */
uint16_t ble_chan_write_handle(int chan);
uint16_t ble_chan_notify_handle(int chan);

/* Which channel a notification arrived on, or -1 for a handle we did not
 * register. A protocol with three notify characteristics cannot tell its
 * command replies from its status pushes without this. */
int ble_chan_of_notify_handle(uint16_t handle);

/* Have all of the profile's notify channels had their CCCD write acknowledged?
 *
 * Not cosmetic: a GoPro silently drops a command written before its command
 * response channel is subscribed -- the reply simply never arrives and the
 * session hangs waiting for it. */
bool ble_channels_ready(void);

/* One channel's own answer to that question. Exists so a session that gives up
 * can say WHICH channel never came up: "not all channels subscribed" sends
 * somebody looking at the whole camera, where "channel 2 has no handle" and
 * "channel 2 was found but never acknowledged its CCCD" are different faults
 * with different causes. */
bool ble_chan_subscribed(int chan);

/* Write to one channel, choosing write-with-response or without from THAT
 * characteristic's own properties rather than from a single global byte. */
esp_err_t ble_chan_write(int chan, const uint8_t *data, size_t len);

/* A notify callback that is told where the data came from. Takes precedence
 * over ble_set_notify_callback() when set; pass NULL to go back to the
 * handle-less one. */
typedef void (*ble_notify_h_callback_t)(uint16_t handle, const uint8_t *data, size_t len);
void ble_set_notify_handle_callback(ble_notify_h_callback_t cb);

esp_err_t ble_write_raw(uint16_t handle, const uint8_t *data, size_t length, bool with_response);

void camlink_ble_set_min_tx_power_static(void);
void camlink_ble_set_min_tx_power_conn(void);
void camlink_ble_request_slow_conn_params(const esp_bd_addr_t peer);

#endif