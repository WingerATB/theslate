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
#include "ble.h"
#include "custom_crc16.h"
#include "custom_crc32.h"
#include <stdlib.h>
#include "sdkconfig.h"
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_timer.h"

#define TAG "BLE"

/* Target device name */
/* 目标设备名称 */
static char s_remote_device_name[ESP_BLE_ADV_NAME_LEN_MAX] = {0};

/* Flags indicating whether a connection has been initiated and whether the target service has been found, for demonstration only */
/* 是否已发起连接、是否已找到目标服务等标记，仅作演示 */
static bool s_connecting = false;

/* Globally saved Notify callback */
/* 全局保存的 Notify 回调 */
static ble_notify_callback_t s_notify_cb = NULL;

/* Why the camera link is not up, kept for the settings page.
 *
 * A module that cannot connect currently says nothing about why, so every
 * cause -- too far away, flat camera battery, a bond one side has forgotten,
 * a camera already paired to a phone -- presents identically as "it just does
 * not work", and the only remedy anyone can find is to re-pair and hope. The
 * reason is already known at the moment of failure; it was simply never kept. */
static uint8_t  s_last_fail_reason;
static uint32_t s_fail_count;
static bool     s_ever_connected;

void camlink_ble_link_fault(uint8_t *reason, uint32_t *count, bool *ever)
{
    if (reason) *reason = s_last_fail_reason;
    if (count)  *count  = s_fail_count;
    if (ever)   *ever   = s_ever_connected;
}

const char *camlink_ble_fault_text(uint8_t reason)
{
    /* Written for someone holding a quad, not someone holding the Core spec.
     * Each one names the most likely physical cause, because that is the thing
     * they can actually go and change. */
    switch (reason) {
    case 0x00: return "";
    case 0x08: return "camera stopped answering — out of range, or switched off";
    case 0x13: return "the camera hung up — often a flat camera battery";
    case 0x14: return "the camera is out of resources";
    case 0x15: return "the camera powered off";
    case 0x16: return "the module closed the link";
    case 0x22: return "the camera stopped responding mid-negotiation";
    case 0x3b: return "the camera refused the connection settings";
    case 0x3d: return "pairing keys rejected — re-pair the camera";
    case 0x3e: return "could not establish a connection — check the camera is "
                      "actually switched on, then that it is close enough and "
                      "not already paired to a phone";
    default:   return "the camera refused the connection";
    }
}



/* Off by default. Answering the camera's 0x00/0x12 poll with ret_code 0 made
 * matters worse, not better: the poll rate went from once per 13 s to once per
 * second and the update screen stayed. On that command a zero is evidently
 * consent, not acknowledgement. Kept for experiments, not enabled. */

/* Set logic layer disconnection state callback */
/* 设置逻辑层断开连接状态回调 */
static connect_logic_state_callback_t s_state_cb = NULL;

/* Attempt to connect when the target device is scanned */
/* 扫描到目标设备，尝试连接 */
#define MIN_RSSI_THRESHOLD -80          // Set minimum signal strength threshold, adjust as needed
static esp_bd_addr_t best_addr = {0};   // Store the address of the device with the strongest signal
/* The address TYPE the camera advertised with, remembered alongside the address
 * itself. esp_ble_gattc_open() needs both, and a connection request aimed at
 * the right bytes with the wrong type reaches nobody: the controller gets no
 * response and the link fails with reason 0x3e, which reads exactly like a
 * camera that is out of range. Defaults to public so an unset value behaves
 * the way the hard-coded constant used to. */
static uint8_t s_best_addr_type = BLE_ADDR_TYPE_PUBLIC;
static int8_t best_rssi = -128;         // Store the RSSI value of the device with the strongest signal, initialized to the weakest signal strength
static bool s_is_reconnecting = false;  // Whether in reconnection mode
static bool s_found_previous_device = false;  // Whether the original device was found in reconnection mode

/* Only one profile is stored */
/* 仅存一个 profile */
ble_profile_t s_ble_profile = {
    .conn_id = 0,
    .gattc_if = ESP_GATT_IF_NONE,
    .remote_bda = {0x60, 0x60, 0x1F, 0x60, 0x11, 0xE7},  // Temporarily store the MAC address of the last connected device; you can initialize it with a test value for debugging purposes.
                                                         // 此处暂存上次连接设备的 MAC 地址，可以初始化一个值进行测试
    .notify_char_handle = 0,
    .write_char_handle = 0,
    .read_char_handle = 0,
    .service_start_handle = 0,
    .service_end_handle = 0,
    .connection_status = {
        .is_connected = false,
    },
    .handle_discovery = {
        .notify_char_handle_found = false,
        .write_char_handle_found = false,
    },
};

/* Define the Service/Characteristic UUIDs to filter, for search use */
/* 这里定义想要过滤的 Service/Characteristic UUID，供搜索使用 */
#define REMOTE_TARGET_SERVICE_UUID   0xFFF0
#define REMOTE_NOTIFY_CHAR_UUID      0xFFF4
#define REMOTE_WRITE_CHAR_UUID       0xFFF5

static esp_bt_uuid_t s_filter_notify_char_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid.uuid16 = REMOTE_NOTIFY_CHAR_UUID,
};

static esp_bt_uuid_t s_filter_write_char_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid.uuid16 = REMOTE_WRITE_CHAR_UUID,
};

static esp_bt_uuid_t s_notify_descr_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,
};

/* Scan parameters, adjustable as needed */
/* 扫描参数，可根据需求调整 */
static esp_ble_scan_params_t s_ble_scan_params = {
    .scan_type          = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval      = 0x50,
    .scan_window        = 0x30,
    .scan_duplicate     = BLE_SCAN_DUPLICATE_DISABLE
};

/* Callback function declarations */
/* 回调函数声明 */
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void gattc_event_handler(esp_gattc_cb_event_t event,
                                esp_gatt_if_t gattc_if,
                                esp_ble_gattc_cb_param_t *param);

static TimerHandle_t scan_timer;

/* How long one connect attempt keeps looking for its camera, in ms since boot.
 * Matches the 30 s the caller waits, so the two cannot disagree about when to
 * give up. */
static uint32_t s_scan_until_ms;
#define SCAN_ATTEMPT_MS  30000

/* Whether a scan is currently running. Two things drive scanning now -- the
 * rescan-on-nothing-found loop below, and cam_task starting a fresh attempt --
 * and they overlap when an attempt times out while a rescan is mid-flight. The
 * controller answers a redundant start with "scan already active" and a
 * redundant stop with "scan not active", neither of which breaks anything but
 * both of which put errors in the log that mean nothing. Errors that mean
 * nothing are how errors that mean something get missed. */
static bool s_scan_active;

static uint32_t ble_now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

void ble_scan_begin_attempt(void)
{
    s_scan_until_ms = ble_now_ms() + SCAN_ATTEMPT_MS;
}

void scan_stop_timer_callback(TimerHandle_t xTimer) {
    if (!s_scan_active) return;
    esp_ble_gap_stop_scanning();
    ESP_LOGI(TAG, "Scan stopped after timeout");
}

static void trigger_scan_task(void) {
    /* A scan already running serves the new request just as well; restarting
     * the window is all that is wanted. */
    if (!s_scan_active) {
        ESP_LOGI(TAG, "esp_ble_gap_start_scanning...");
        esp_err_t ret = esp_ble_gap_start_scanning(4);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start scanning: %s", esp_err_to_name(ret));
        } else {
            s_scan_active = true;
        }
    }
    // Start a timer to stop scanning after 4 seconds
    // 启动定时器，在4秒后停止扫描
    /* One timer, reused. This used to xTimerCreate() on every scan and never
     * delete any of them, which leaked a FreeRTOS timer per attempt -- and a
     * reconnect loop makes a lot of attempts. xTimerStart on a running timer
     * restarts it, which is exactly the wanted behaviour. */
    if (scan_timer == NULL) {
        scan_timer = xTimerCreate("scan_timer", pdMS_TO_TICKS(4000), pdFALSE,
                                  (void *)0, scan_stop_timer_callback);
    }
    if (scan_timer != NULL) {
        xTimerStart(scan_timer, 0);
    }
}

/* -------------------------
 *  Initialization/Scan/Connection related interfaces
 *  初始化/扫描/连接相关接口
 * ------------------------- */

/**
 * @brief BLE client initialization
 * BLE 客户端初始化
 *
 * @return esp_err_t
 *         - ESP_OK on success
 *         - Others on failure
 */

/* ===================== SLATE ADDITION ===================================
 * Minimum-TX-power helpers. Kept together so the deviation from the upstream
 * DJI demo is easy to audit. ESP_PWR_LVL_N24 == -24 dBm is the lowest level
 * the ESP32-C3 supports (esp_bt.h: ESP_PWR_LVL_N24 = 0).
 * ========================================================================= */
#if   defined(CONFIG_CAMLINK_TX_N12)
#define CAMLINK_TX_LVL  ESP_PWR_LVL_N12
#define CAMLINK_TX_NAME "-12 dBm"
#elif defined(CONFIG_CAMLINK_TX_N0)
#define CAMLINK_TX_LVL  ESP_PWR_LVL_N0
#define CAMLINK_TX_NAME "0 dBm"
#elif defined(CONFIG_CAMLINK_TX_P3)
#define CAMLINK_TX_LVL  ESP_PWR_LVL_P3
#define CAMLINK_TX_NAME "+3 dBm"
#else
#define CAMLINK_TX_LVL  ESP_PWR_LVL_N24
#define CAMLINK_TX_NAME "-24 dBm"
#endif

/* Runtime override. The Kconfig choice stays the first-boot default, but the
 * level a user actually needs differs between the bench (loud enough to make a
 * connection across a desk) and the air (quiet enough not to sit on top of the
 * ELRS receiver), and forcing a reflash to move between them is exactly the
 * kind of extra work the settings page exists to remove. Set before ble_init();
 * the controller must be enabled before it takes effect on the hardware. */
static esp_power_level_t s_tx_lvl = CAMLINK_TX_LVL;

/* The level actually in force, not the one compiled in. CAMLINK_TX_NAME is a
 * Kconfig string, so logging it after a runtime override reported the build's
 * default while the radio ran at something else -- a status line that is
 * confidently wrong, which is the one thing this project does not do. */
static const char *tx_lvl_name(esp_power_level_t l)
{
    switch (l) {
    case ESP_PWR_LVL_N24: return "-24 dBm";
    case ESP_PWR_LVL_N12: return "-12 dBm";
    case ESP_PWR_LVL_N0:  return "0 dBm";
    case ESP_PWR_LVL_P3:  return "+3 dBm";
    default:              return "(other)";
    }
}

/* Drop every stored BLE bond.
 *
 * A camera that has decided our module is an accessory needing a firmware
 * update remembers that against our address. Re-pairing from scratch means
 * clearing our half of the bond too, or the camera is talking to a peer it
 * already has opinions about. */
void camlink_ble_clear_bonds(void)
{
    int n = esp_ble_get_bond_device_num();
    if (n <= 0) { ESP_LOGW(TAG, "no bonds to clear"); return; }
    esp_ble_bond_dev_t *list = calloc((size_t)n, sizeof(esp_ble_bond_dev_t));
    if (list == NULL) return;
    if (esp_ble_get_bond_device_list(&n, list) == ESP_OK) {
        for (int i = 0; i < n; i++) esp_ble_remove_bond_device(list[i].bd_addr);
        ESP_LOGW(TAG, "cleared %d BLE bond(s)", n);
    }
    free(list);
}

void camlink_ble_set_tx_level(int lvl)
{
    if (lvl < ESP_PWR_LVL_N24 || lvl > ESP_PWR_LVL_P9) return;
    s_tx_lvl = (esp_power_level_t)lvl;
}

/* Change transmit power on a live link.
 *
 * camlink_ble_set_tx_level() only records the value for the next connection;
 * this applies it now, to the connection and to the defaults behind it, so the
 * radio can be turned down for the duration of a flight and back up again
 * without dropping the camera. Silent on success and on a no-op: it is called
 * on every arm and disarm, and a log line per transition would bury everything
 * else in a flight log. */
void camlink_ble_apply_tx_level(int lvl)
{
    if (lvl < ESP_PWR_LVL_N24 || lvl > ESP_PWR_LVL_P9) return;
    s_tx_lvl = (esp_power_level_t)lvl;

    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, s_tx_lvl);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV,     s_tx_lvl);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN,    s_tx_lvl);
    if (s_ble_profile.connection_status.is_connected) {
        esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL0, s_tx_lvl);
    }
}

void camlink_ble_set_min_tx_power_static(void)
{
    const esp_power_level_t lvl = s_tx_lvl;
    struct { esp_ble_power_type_t t; const char *n; } types[] = {
        { ESP_BLE_PWR_TYPE_DEFAULT, "DEFAULT" },
        { ESP_BLE_PWR_TYPE_ADV,     "ADV"     },
        { ESP_BLE_PWR_TYPE_SCAN,    "SCAN"    },
    };
    for (size_t i = 0; i < sizeof(types) / sizeof(types[0]); i++) {
        esp_err_t e = esp_ble_tx_power_set(types[i].t, lvl);
        if (e != ESP_OK) {
            ESP_LOGW(TAG, "TX power %s -> %s failed: %s", types[i].n, tx_lvl_name(lvl), esp_err_to_name(e));
        } else {
            ESP_LOGI(TAG, "TX power %s set to %s", types[i].n, tx_lvl_name(lvl));
        }
    }
}

void camlink_ble_set_min_tx_power_conn(void)
{
    esp_err_t e = esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL0, s_tx_lvl);
    if (e != ESP_OK) {
        ESP_LOGW(TAG, "TX power CONN_HDL0 -> %s failed: %s", tx_lvl_name(s_tx_lvl), esp_err_to_name(e));
    } else {
        ESP_LOGI(TAG, "TX power CONN_HDL0 set to %s", tx_lvl_name(s_tx_lvl));
    }
}

/* Ask the camera for a slower connection interval. Fewer on-air events per
 * second is worth more to the quad's RC link than the TX power reduction is.
 *
 * 30-50 ms is a deliberate middle ground: the camera only pushes status at 2 Hz
 * so nothing is lost, and the worst-case extra latency added to a record
 * start/stop command (~50 ms) is small next to the camera's own 0.5-1 s
 * shutter delay. Do not push this much higher without re-checking that. */
void camlink_ble_request_slow_conn_params(const esp_bd_addr_t peer)
{
    esp_ble_conn_update_params_t p = { 0 };
    memcpy(p.bda, peer, sizeof(esp_bd_addr_t));
    p.min_int = 24;    /* 24 * 1.25 ms = 30 ms */
    p.max_int = 40;    /* 40 * 1.25 ms = 50 ms */
    p.latency = 0;     /* no slave latency: keep command response prompt */
    /* 600 * 10 ms = 6 s supervision timeout.
     *
     * This was 2 s, chosen to make the OSD stop showing a stale readout
     * quickly. It was the wrong knob for that job, and it cost a real fault:
     * an Osmo Nano stops servicing BLE for over two seconds while it finalises
     * a clip on the card, so stopping a recording could kill the link outright
     * (testlogs/nano-stop.log):
     *
     *     W DUMLCAM: Do Record STOP
     *     #ST link=1 valid=0                      <- camera went quiet
     *     W BLE: Disconnected, reason=0x08 (supervision timeout)
     *
     * 0x08 means WE gave up on the camera. It is intermittent because two
     * seconds is right at the edge of how long the close takes -- which is the
     * worst kind of margin to ship.
     *
     * OSD responsiveness does not depend on this value at all: CAM_STATUS_
     * STALE_MS in camlink.c ages the readings out after 3 s of no pushes,
     * whether or not BLE still believes it has a link, and that is what blanks
     * the slots. All this timeout decides is when to tear the link down and
     * start reconnecting. Six seconds clears the observed stall with real
     * margin and still notices a camera that has actually gone. */
    p.timeout = 600;

    esp_err_t e = esp_ble_gap_update_conn_params(&p);
    if (e != ESP_OK) {
        ESP_LOGW(TAG, "conn param update request failed: %s", esp_err_to_name(e));
    } else {
        ESP_LOGI(TAG, "requested conn interval 30-50 ms");
    }
}

esp_err_t ble_init() {
    /* Initialize NVS */
    /* 初始化 NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Release classic Bluetooth memory */
    /* 释放经典蓝牙内存 */
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    /* Configure and initialize the Bluetooth controller */
    /* 配置并初始化蓝牙控制器 */
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "initialize controller failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Start the BLE controller */
    /* 启动 BLE 控制器 */
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "enable controller failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* SLATE PATCH: drop TX power to the minimum (-24 dBm) before any radio
     * activity. The camera sits ~20 cm away, so this is ~45 dB more link budget
     * than needed and keeps us quiet next to the ELRS receiver.
     *
     * NOTE: esp_ble_tx_power_set() requires the controller to be ENABLED, so this
     * cannot be done before esp_bt_controller_enable(). ADV/SCAN/DEFAULT are set
     * here; the CONN power type can only be set once a connection exists and is
     * therefore set in ESP_GATTC_CONNECT_EVT below (see esp_bt.h note 1). */
    camlink_ble_set_min_tx_power_static();

    /* Initialize the Bluedroid stack */
    /* 初始化 Bluedroid 堆栈 */
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "init bluedroid failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Enable Bluedroid */
    /* 启用 Bluedroid */
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "enable bluedroid failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Register GAP callback */
    /* 注册 GAP 回调 */
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "gap register error, err code = %x", ret);
        return ret;
    }

    /* Register GATTC callback */
    /* 注册 GATTC 回调 */
    /* Say plainly what we are, at the BLE level.
     *
     * This module runs a GATT server it never uses (the stack enables one by
     * default) and had never set a device name, so a camera reading our GAP
     * name got whatever the stack happened to default to. An Osmo 360 raises a
     * firmware-update prompt for accessories it believes are DJI hardware, and
     * an unnamed device is a poor way to argue otherwise.
     *
     * This is a different identification channel from the R SDK handshake's
     * device_id -- that one is inside the protocol, this one is underneath it,
     * and sweeping the former can never affect the latter. */
    esp_ble_gap_set_device_name("SlateFPV");

    ret = esp_ble_gattc_register_callback(gattc_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "gattc register error, err code = %x", ret);
        return ret;
    }

    /* Register GATTC application (only one profile here, app_id = 0) */
    /* 注册 GATTC 应用（此处只有一个 profile，app_id = 0） */
    ret = esp_ble_gattc_app_register(0);
    if (ret) {
        ESP_LOGE(TAG, "gattc app register error, err code = %x", ret);
        return ret;
    }

    /* Set local MTU (optional) */
    /* 设置本地 MTU（可选） */
    esp_ble_gatt_set_local_mtu(500);

#if CONFIG_CAMLINK_BLE_BONDING
    /* SLATE ADDITION: BLE bonding / link encryption.
     *
     * Upstream performs everything with ESP_GATT_AUTH_REQ_NONE and never calls
     * esp_ble_gap_set_security_param(), i.e. the link is never encrypted. The
     * Osmo Nano advertises as HID with a Battery Service, and HID-over-GATT
     * mandates an encrypted link, so an unencrypted peer can be accepted at
     * GATT level and then dropped. Observed exactly that: the camera ACKs our
     * write and then terminates the connection itself with reason 0x13
     * (remote user terminated) about 16 s later.
     *
     * Just Works pairing (NoInputNoOutput) is used: we have no display or
     * keypad. If the camera insists on MITM protection this will fail visibly
     * in ESP_GAP_BLE_AUTH_CMPL_EVT rather than silently. */
    {
        esp_ble_auth_req_t  auth_req = ESP_LE_AUTH_REQ_SC_BOND;
        esp_ble_io_cap_t    iocap    = ESP_IO_CAP_NONE;
        uint8_t             key_size = 16;
        uint8_t             init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
        uint8_t             rsp_key  = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;

        esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(auth_req));
        esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE,      &iocap,    sizeof(iocap));
        esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE,    &key_size, sizeof(key_size));
        esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY,    &init_key, sizeof(init_key));
        esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY,     &rsp_key,  sizeof(rsp_key));
        ESP_LOGI(TAG, "BLE bonding enabled (SC_BOND, Just Works)");
    }

    /* SLATE ADDITION: drop any stale bonds at boot.
     *
     * A bond kept on our side that the camera has forgotten (or vice versa)
     * makes the next connection fail at link level with reason 0x3e before any
     * GATT traffic happens -- observed exactly that after a successful bond.
     * Re-pairing from clean on every boot is the right trade here: pairing is
     * cheap, and a stale bond is otherwise invisible and hard to diagnose. */
#if CONFIG_CAMLINK_BLE_CLEAR_BONDS
    {
        int n = esp_ble_get_bond_device_num();
        if (n > 0) {
            esp_ble_bond_dev_t *list = calloc((size_t)n, sizeof(esp_ble_bond_dev_t));
            if (list != NULL) {
                if (esp_ble_get_bond_device_list(&n, list) == ESP_OK) {
                    for (int i = 0; i < n; i++) {
                        esp_ble_remove_bond_device(list[i].bd_addr);
                    }
                }
                free(list);
            }
            ESP_LOGI(TAG, "cleared %d stale BLE bond(s)", n);
        }
    }
#else
    ESP_LOGI(TAG, "keeping %d stored BLE bond(s)", esp_ble_get_bond_device_num());
#endif
#endif

    ESP_LOGI(TAG, "ble_init success!");
    return ESP_OK;
}

/**
 * @brief Connect to a device with a specified name (if already scanning, it will automatically connect when the device is found)
 * 连接到指定名称的设备（若已在扫描中，会自动在扫描到该设备时连接）
 *
 * @note  This interface is for demonstration only. If you want to actively specify an address to connect, you can extend the interface yourself.
 *        本接口仅作为演示，如果想主动指定地址连接，可自行扩展接口
 * @return esp_err_t
 */
esp_err_t ble_start_scanning_and_connect(void) {
    // TODO: Add reconnection logic; current implementation has issues and needs to be fixed.
    // 补充重连逻辑，当前实现存在问题，待修复
    if(ble_get_reconnecting()) {
        return ble_reconnect();
    }

    // Reset scan-related variables
    // 重置扫描相关变量
    memset(best_addr, 0, sizeof(esp_bd_addr_t));
    best_rssi = -128;
    memset(s_remote_device_name, 0, ESP_BLE_ADV_NAME_LEN_MAX);
    s_is_reconnecting = false;
    s_found_previous_device = false;

    // Set scan parameters
    // 设置扫描参数
    esp_err_t ret = esp_ble_gap_set_scan_params(&s_ble_scan_params);
    if (ret) {
        ESP_LOGE(TAG, "Set scan params error: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Set scan params ok!");
    ble_scan_begin_attempt();
    return ESP_OK;
}

static void try_to_connect(esp_bd_addr_t addr) {
    // Check if already connecting
    // 检查是否正在连接中
    if (s_connecting) {
        ESP_LOGW(TAG, "Already in connecting state, please wait...");
        return;
    }

    // Check if the address is the initial value (all zeros)
    // 检查地址是否为初始值（全0）
    bool is_valid = false;
    for (int i = 0; i < ESP_BD_ADDR_LEN; i++) {
        if (addr[i] != 0) {
            is_valid = true;
            break;
        }
    }

    if (!is_valid) {
        ESP_LOGE(TAG, "Invalid device address (all zeros)");
        return;
    }

    s_connecting = true;
    ESP_LOGI(TAG, "Try to connect target device name = %s, MAC: %02X:%02X:%02X:%02X:%02X:%02X, addr_type=%u (%s)",
             s_remote_device_name,
             addr[0], addr[1], addr[2],
             addr[3], addr[4], addr[5],
             (unsigned)s_best_addr_type,
             s_best_addr_type == BLE_ADDR_TYPE_PUBLIC     ? "public"
           : s_best_addr_type == BLE_ADDR_TYPE_RANDOM     ? "random"
           : s_best_addr_type == BLE_ADDR_TYPE_RPA_PUBLIC ? "RPA-public"
           : s_best_addr_type == BLE_ADDR_TYPE_RPA_RANDOM ? "RPA-random" : "?");

    // Do not call lightly, if you connect to a non-existent device address, you will have to wait a while before you can connect again
    // 不要轻易调用，如果连接不存在的设备地址会等待一段时间后才能再次连接
    /* The type the scanner saw, not a hard-coded BLE_ADDR_TYPE_PUBLIC. */
    esp_ble_gattc_open(s_ble_profile.gattc_if,
                       addr,
                       s_best_addr_type,
                       true);
}

void ble_set_reconnecting(bool flag) {
    s_is_reconnecting = flag;
}

bool ble_get_reconnecting(void) {
    return s_is_reconnecting;
}

/**
 * @brief Reconnect to the last connected device
 * 重新连接到上一次连接的设备
 * 
 * @note Only applicable to non-active disconnection situations, as device information is not cleared
 *       仅适用于非主动断开连接的情况，因为设备信息未被清除
 * @return esp_err_t
 */
esp_err_t ble_reconnect(void) {
    // Check if there is a valid last connection address
    // 检查是否有有效的上一次连接地址
    bool is_valid = false;
    for (int i = 0; i < ESP_BD_ADDR_LEN; i++) {
        if (best_addr[i] != 0) {
            is_valid = true;
            break;
        }
    }

    if (!is_valid) {
        ESP_LOGE(TAG, "No valid previous device address found");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Attempting to reconnect to previous device: %s, MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             s_remote_device_name,
             best_addr[0], best_addr[1], best_addr[2],
             best_addr[3], best_addr[4], best_addr[5]);

    // Set reconnection mode flag
    // 设置重连模式标记
    s_is_reconnecting = true;
    s_found_previous_device = false;  // Reset discovery flag

    // Start scan task
    // 开始扫描任务
    ble_scan_begin_attempt();
    trigger_scan_task();
    
    return ESP_OK;
}

/**
 * @brief Disconnect (if connected)
 * 断开连接（如果已经连接）
 *
 * @return esp_err_t
 */
esp_err_t ble_disconnect(void) {
    if (s_ble_profile.connection_status.is_connected) {
        esp_ble_gattc_close(s_ble_profile.gattc_if, s_ble_profile.conn_id);
    }
    return ESP_OK;
}

/* -------------------------
 *  Read/Write and Notify related interfaces
 *  读写与 Notify 相关接口
 * ------------------------- */
/**
 * @brief Read a specified characteristic
 * 读取指定特征
 *
 * @param conn_id  Connection ID (obtained from callback events or internal management)
 *                 连接 ID（由回调事件或内部管理获得）
 * @param handle   Handle of the characteristic
 *                 特征的 handle
 * @return esp_err_t
 */
esp_err_t ble_read(uint16_t conn_id, uint16_t handle) {
    if (!s_ble_profile.connection_status.is_connected) {
        ESP_LOGW(TAG, "Not connected, skip read");
        return ESP_FAIL;
    }
    /* Initiate GATTC read request */
    /* 发起 GATTC 读请求 */
    esp_err_t ret = esp_ble_gattc_read_char(s_ble_profile.gattc_if,
                                            conn_id,
                                            handle,
                                            ESP_GATT_AUTH_REQ_NONE);
    if (ret) {
        ESP_LOGE(TAG, "read_char failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

/**
 * @brief Write characteristic (Write Without Response)
 * 写特征（Write Without Response）
 *
 * @param conn_id   Connection ID
 *                  连接 ID
 * @param handle    Handle of the characteristic
 *                  特征 handle
 * @param data      Data to be written
 *                  要写入的数据
 * @param length    Length of the data
 *                  数据长度
 * @return esp_err_t
 */
esp_err_t ble_write_without_response(uint16_t conn_id, uint16_t handle, const uint8_t *data, size_t length) {
    if (!s_ble_profile.connection_status.is_connected) {
        ESP_LOGW(TAG, "Not connected, skip write_without_response");
        return ESP_FAIL;
    }
    esp_err_t ret = esp_ble_gattc_write_char(s_ble_profile.gattc_if,
                                             conn_id,
                                             handle,
                                             length,
                                             (uint8_t *)data,
                                             ESP_GATT_WRITE_TYPE_NO_RSP,
                                             ESP_GATT_AUTH_REQ_NONE);
    if (ret) {
        ESP_LOGE(TAG, "write_char NO_RSP failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

/**
 * @brief Write characteristic (Write With Response)
 * 写特征（Write With Response）
 *
 * @param conn_id   Connection ID
 *                  连接 ID
 * @param handle    Handle of the characteristic
 *                  特征 handle
 * @param data      Data to be written
 *                  要写入的数据
 * @param length    Length of the data
 *                  数据长度
 * @return esp_err_t
 */
/* SLATE ADDITION: write to an arbitrary characteristic with an explicit
 * write type. Needed for the DUML path, which must write the pairing trigger
 * to 0xFFF4 (write-with-response) and DUML frames to 0xFFF5 (write-without-
 * response) -- two different handles and two different write types. */
esp_err_t ble_write_raw(uint16_t handle, const uint8_t *data, size_t length, bool with_response)
{
    if (!s_ble_profile.connection_status.is_connected) {
        return ESP_FAIL;
    }
    return esp_ble_gattc_write_char(s_ble_profile.gattc_if,
                                    s_ble_profile.conn_id,
                                    handle,
                                    length,
                                    (uint8_t *)data,
                                    with_response ? ESP_GATT_WRITE_TYPE_RSP
                                                  : ESP_GATT_WRITE_TYPE_NO_RSP,
                                    ESP_GATT_AUTH_REQ_NONE);
}

esp_err_t ble_write_with_response(uint16_t conn_id, uint16_t handle, const uint8_t *data, size_t length) {
    if (!s_ble_profile.connection_status.is_connected) {
        ESP_LOGW(TAG, "Not connected, skip write_with_response");
        return ESP_FAIL;
    }

    /* SLATE PATCH: pick the write type the characteristic actually supports.
     *
     * Upstream always used ESP_GATT_WRITE_TYPE_RSP here. On the Osmo Nano the
     * 0xFFF5 write characteristic reports props=0x36 -- READ | WRITE_NR |
     * NOTIFY | INDICATE -- with the WRITE (0x08) bit CLEAR, i.e. it accepts
     * write-WITHOUT-response only. Writing with response to it is an
     * unsupported request, which is why every command was silently ignored and
     * the camera never answered the 0x00 0x19 handshake.
     *
     * (Action-series cameras report 0x3a, which includes WRITE, so upstream's
     * hardcoded choice happened to work there.) */
    const esp_gatt_write_type_t write_type =
        (s_ble_profile.write_char_props & ESP_GATT_CHAR_PROP_BIT_WRITE)
            ? ESP_GATT_WRITE_TYPE_RSP
            : ESP_GATT_WRITE_TYPE_NO_RSP;

    esp_err_t ret = esp_ble_gattc_write_char(s_ble_profile.gattc_if,
                                             conn_id,
                                             handle,
                                             length,
                                             (uint8_t *)data,
                                             write_type,
                                             ESP_GATT_AUTH_REQ_NONE);
    if (ret) {
        ESP_LOGE(TAG, "write_char RSP failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

/**
 * @brief Register (enable) Notify
 * 注册（开启）Notify
 *
 * @param conn_id   Connection ID
 *                  连接 ID
 * @param char_handle Handle of the characteristic to enable notification
 *                    需要开启通知的特征 handle
 * @return esp_err_t
 */
esp_err_t ble_register_notify(uint16_t conn_id, uint16_t char_handle) {
    if (!s_ble_profile.connection_status.is_connected) {
        ESP_LOGW(TAG, "Not connected, skip register_notify");
        return ESP_FAIL;
    }
    /* Request to subscribe to notifications from the protocol stack */
    /* 向协议栈请求订阅通知 */
    esp_err_t ret = esp_ble_gattc_register_for_notify(s_ble_profile.gattc_if,
                                                      s_ble_profile.remote_bda,
                                                      char_handle);
    if (ret) {
        ESP_LOGE(TAG, "register_notify failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

/**
 * @brief Unregister (disable) Notify
 * 反注册（关闭）Notify
 *
 * @note  This is just a demonstration logic. You need the Client Config descriptor handle of the characteristic to operate.
 *        If needed in actual development, you can directly save the descr handle previously, and then close it by writing 0x0000 here.
 *        此处仅示例逻辑，需要特征的 Client Config 描述符 handle 来进行操作
 *        若实际开发需要，也可直接先前保存 descr handle，然后在此进行关闭写 0x0000
 *
 * @param conn_id   Connection ID
 *                  连接 ID
 * @param char_handle Handle of the characteristic to disable notification
 *                    需要关闭通知的特征 handle
 * @return esp_err_t
 */
esp_err_t ble_unregister_notify(uint16_t conn_id, uint16_t char_handle) {
    /* In fact, you need to get the corresponding descriptor handle and then write 0x0000 to disable it */
    /* 实际上需要获取到对应的描述符 handle，然后写 0x0000 进行关闭 */
    /* This is just a demonstration of the process. If needed, you can save the descr handle during register_notify */
    /* 这里只是演示一下流程，需要时可在 register_notify 时保存 descr handle */
    ESP_LOGI(TAG, "ble_unregister_notify called (demo), not fully implemented");
    return ESP_OK;
}

/**
 * @brief Set global Notify callback (for receiving data)
 * 设置全局的 Notify 回调（用于接收数据）
 *
 * @param cb Callback function pointer
 *           回调函数指针
 */
void ble_set_notify_callback(ble_notify_callback_t cb) {
    s_notify_cb = cb;
}

/**
 * @brief Set global logic layer disconnection state callback
 * 设置全局的逻辑层断连状态回调
 *
 * @param cb Callback function pointer
 *           回调函数指针
 */
void ble_set_state_callback(connect_logic_state_callback_t cb) {
    s_state_cb = cb;
}

/* ----------------------------------------------------------------
 *   GAP & GATTC callback function implementation (simplified version)
 *   GAP & GATTC 回调函数实现（精简版）
 * ---------------------------------------------------------------- */

/* 判断是否为 DJI 相机的广播 */
/* Determine whether it is a DJI camera advertisement */
/* SLATE PATCH -------------------------------------------------------------
 * Upstream identified a DJI camera by manufacturer data matching
 * data[0]==0xAA && data[1]==0x08 && data[4]==0xFA.
 *
 * The 0xFA test is wrong: byte 4 is not a DJI signature, it varies by model.
 * Observed on real hardware:
 *
 *   Osmo Action 5 Pro : AA 08 15 00 FA 60 60 1F C7 50 96 00
 *   Osmo Nano         : AA 08 19 00 C0 4C 43 F6 66 67 3F 02
 *                       ^^^^^ ^^    ^^ ^^^^^^^^^^^^^^^^^
 *                       DJI   model |  advertising MAC
 *                       co.id       per-model byte
 *
 * Byte 2 is a model code (0x15 Action 5 Pro, 0x19 Osmo Nano) and byte 4
 * differs with it, so an Osmo Nano was silently dropped by the upstream check
 * even though it advertises correctly.
 *
 * What IS reliable: 0xAA08 is DJI's Bluetooth SIG company identifier (it
 * renders as "SZ DJI TECHNOLOGY CO.,LTD <08AA>" in a scanner), and this advert
 * layout carries the device's own MAC at bytes 5..10. Requiring the company ID
 * plus a MAC that matches the advertising address identifies the camera advert
 * without pinning us to any single model. The original 0xFA test is kept as a
 * fallback so any camera that does not embed its MAC still works.
 * -------------------------------------------------------------------------- */
uint8_t bsp_link_is_dji_camera_adv(esp_ble_gap_cb_param_t *scan_result) {
    const uint8_t *ble_adv = scan_result->scan_rst.ble_adv;
    const uint8_t adv_len = scan_result->scan_rst.adv_data_len + scan_result->scan_rst.scan_rsp_len;

    for (int i = 0; i < adv_len; ) {
        const uint8_t len = ble_adv[i];

        if (len == 0 || (i + len + 1) > adv_len) break;

        const uint8_t type = ble_adv[i+1];
        const uint8_t *data = &ble_adv[i+2];
        const uint8_t data_len = len - 1;

        if (type == ESP_BLE_AD_MANUFACTURER_SPECIFIC_TYPE) {
            /* DJI company identifier, little endian. */
            if (data_len >= 5 && data[0] == 0xAA && data[1] == 0x08) {

                /* Advert embeds its own MAC at bytes 5..10. */
                if (data_len >= 11 &&
                    memcmp(&data[5], scan_result->scan_rst.bda, ESP_BD_ADDR_LEN) == 0) {
                    return 1;
                }

                /* Upstream's original Action-series signature. */
                if (data[4] == 0xFA) {
                    return 1;
                }
            }
        }
        i += (len + 1);
    }

    /* CAMLINK: fall back to the advertised name.
     *
     * Manufacturer data is not always present. The Osmo Nano alternates
     * between an advert carrying AA 08 ... and one with none at all (and after
     * a "Reset Connection" it emitted only the latter for a while); lib-osmo-ble
     * reports the Osmo Pocket 3 never includes manufacturer data. Requiring it
     * makes the camera undiscoverable exactly when it is sitting right there
     * advertising its name. */
    for (int i = 0; i < adv_len; ) {
        const uint8_t len = ble_adv[i];
        if (len == 0 || (i + len + 1) > adv_len) break;
        const uint8_t type = ble_adv[i+1];
        const uint8_t *data = &ble_adv[i+2];
        const uint8_t data_len = len - 1;
        if ((type == ESP_BLE_AD_TYPE_NAME_CMPL || type == ESP_BLE_AD_TYPE_NAME_SHORT)
            && data_len >= 4) {
            if (memcmp(data, "Osmo", 4) == 0 || memcmp(data, "OA", 2) == 0) {
                return 1;
            }
        }
        i += (len + 1);
    }
    return 0;
}

/* ===================== SLATE ADDITION ===================================
 * Read every readable characteristic once after connecting.
 *
 * FFF3/FFF4/FFF5 all carry the READ property, and the camera also exposes the
 * standard 0x2A00/0x2A01/0x2A04/0x2AA6 set. None of these have ever been read.
 * If the camera is refusing the remote protocol for a stateful reason, its
 * current state may well be visible here -- and it costs one round trip.
 *
 * BLE reads must be serialised, so the handles are queued and the next read is
 * issued from ESP_GATTC_READ_CHAR_EVT.
 * ========================================================================= */
#define CAMLINK_READ_QUEUE_MAX 16
static uint16_t s_read_q[CAMLINK_READ_QUEUE_MAX];
static uint16_t s_read_q_uuid[CAMLINK_READ_QUEUE_MAX];
static uint8_t  s_read_q_n;
static uint8_t  s_read_q_i;

/* SLATE ADDITION: cycle the write target across connection attempts.
 *
 * DJI's FAQ says commands go to 0xFFF5. On the Osmo Nano 0xFFF5 is WRITE_NR
 * only, while 0xFFF3 and 0xFFF4 both advertise WRITE (with response). Since the
 * camera silently ignores everything sent to 0xFFF5, try the other two as well
 * rather than assuming the FAQ holds for a camera DJI never documented.
 *
 * One candidate per connection attempt; the chosen UUID is logged so the
 * working combination is obvious in the trace. */
static const uint16_t s_write_candidates[] = { 0xFFF5 };
static uint8_t s_write_candidate_idx;

/* SLATE ADDITION: bound-camera filter.
 *
 * Without this the scanner connects to whichever DJI camera is loudest. At a
 * meetup that means binding to a stranger's camera -- and then starting a
 * recording on it when the pilot arms. Once a camera is bound we accept only
 * that address; the "loudest wins" behaviour is reachable only in bind mode. */
static esp_bd_addr_t s_bound_addr;
static bool          s_have_bound;

void ble_set_bound_addr(const uint8_t *addr)
{
    if (addr == NULL) {
        s_have_bound = false;
        memset(s_bound_addr, 0, sizeof(s_bound_addr));
        ESP_LOGI(TAG, "bind cleared -- will accept the nearest camera");
    } else {
        memcpy(s_bound_addr, addr, sizeof(s_bound_addr));
        s_have_bound = true;
        ESP_LOGI(TAG, "bound to %02X:%02X:%02X:%02X:%02X:%02X",
                 addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
    }
}

bool ble_has_bound_addr(void) { return s_have_bound; }


static void camlink_read_next(esp_gatt_if_t gattc_if)
{
    if (s_read_q_i >= s_read_q_n) {
        if (s_read_q_n > 0) {
            ESP_LOGI(TAG, "[READ] characteristic dump complete (%u read)",
                     (unsigned)s_read_q_n);
            s_read_q_n = 0;
        }
        return;
    }
    uint16_t h = s_read_q[s_read_q_i];
    esp_err_t e = esp_ble_gattc_read_char(gattc_if, s_ble_profile.conn_id, h,
                                          ESP_GATT_AUTH_REQ_NONE);
    if (e != ESP_OK) {
        ESP_LOGW(TAG, "[READ] handle 0x%02x request failed: %s", h, esp_err_to_name(e));
        s_read_q_i++;
        camlink_read_next(gattc_if);
    }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        ESP_LOGI(TAG, "ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT");
        trigger_scan_task();
        break;

#if CONFIG_CAMLINK_BLE_BONDING
    case ESP_GAP_BLE_SEC_REQ_EVT:
        /* Camera asked us to start security -- always accept. */
        ESP_LOGI(TAG, "security request from camera, accepting");
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;

    case ESP_GAP_BLE_AUTH_CMPL_EVT: {
        const esp_ble_auth_cmpl_t *a = &param->ble_security.auth_cmpl;
        if (a->success) {
            ESP_LOGI(TAG, "BONDING OK (addr_type=%d, auth_mode=0x%x)",
                     a->addr_type, a->auth_mode);
        } else {
            ESP_LOGE(TAG, "BONDING FAILED, reason=0x%x", a->fail_reason);
        }
        break;
    }

    case ESP_GAP_BLE_PASSKEY_NOTIF_EVT:
        /* The camera wants a passkey shown to the user. We have no display, so
         * surface it on the console instead. */
        ESP_LOGW(TAG, "PASSKEY from camera: %06u",
                 (unsigned)param->ble_security.key_notif.passkey);
        break;

    case ESP_GAP_BLE_NC_REQ_EVT:
        ESP_LOGW(TAG, "numeric comparison: %06u -- auto-confirming",
                 (unsigned)param->ble_security.key_notif.passkey);
        esp_ble_confirm_reply(param->ble_security.ble_req.bd_addr, true);
        break;

    case ESP_GAP_BLE_KEY_EVT:
        ESP_LOGI(TAG, "bond key exchanged, type=%d", param->ble_security.ble_key.key_type);
        break;
#endif

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        ESP_LOGI(TAG, "scan stopped");
        s_scan_active = false;
        // After scanning ends, decide whether to connect based on reconnection mode and device discovery status
        // 扫描结束后，根据重连模式和设备发现状态决定是否连接
        if (best_rssi > -128) {
            if (!ble_get_reconnecting() || (ble_get_reconnecting() && s_found_previous_device)) {
                try_to_connect(best_addr);
                ESP_LOGI(TAG, "Connected to device: %02x:%02x:%02x:%02x:%02x:%02x",
                         best_addr[0], best_addr[1], best_addr[2], best_addr[3], best_addr[4], best_addr[5]);
            } else {
                ESP_LOGW(TAG, "In reconnection mode but target device not found");
            }
        } else if (ble_now_ms() < s_scan_until_ms) {
            /* The camera was not on the air during those four seconds. Look
             * again immediately rather than idling until the caller's timeout.
             *
             * This mattered more than it looks. A camera that loses its central
             * without a clean disconnect -- which is what a module reset or a
             * power cycle is -- can take tens of seconds to start advertising
             * again; an Osmo 360 was measured silent for the whole of the first
             * scan window and back on the air well before the second. Scanning
             * for 4 s and then waiting out a 30 s timeout meant listening for
             * four seconds in every thirty-six, so a camera that came back at
             * six seconds was not found until thirty-six
             * (testlogs/360-connect-time.log: 36.11 s, three times running,
             * against 4.06 s when the camera happened to be advertising).
             * Looking continuously turns that into "as soon as it appears". */
            ESP_LOGI(TAG, "target not seen this pass -- scanning again");
            trigger_scan_task();
        } else {
            ESP_LOGW(TAG, "No suitable device found with sufficient signal strength");
            s_is_reconnecting = false;
        }
        break;

    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *r = param;
        if (r->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
#if CONFIG_CAMLINK_BLE_SCAN_DEBUG
            /* SLATE ADDITION: log EVERY advertisement before the DJI filter.
             *
             * The filter below requires manufacturer data matching
             * AA 08 ?? ?? FA. If the Osmo Nano advertises a different
             * signature it is dropped silently, which on the console is
             * indistinguishable from the camera being switched off. This
             * dump tells those two cases apart: if the Nano appears here but
             * never connects, the signature is the problem, not the camera. */
            {
                uint8_t nl = 0;
                uint8_t *nm = esp_ble_resolve_adv_data_by_type(
                    r->scan_rst.ble_adv,
                    r->scan_rst.adv_data_len + r->scan_rst.scan_rsp_len,
                    ESP_BLE_AD_TYPE_NAME_CMPL, &nl);
                char nbuf[32] = "<no name>";
                if (nm && nl > 0) {
                    size_t cl = nl < sizeof(nbuf) - 1 ? nl : sizeof(nbuf) - 1;
                    memcpy(nbuf, nm, cl);
                    nbuf[cl] = '\0';
                }

                uint8_t ml = 0;
                uint8_t *mf = esp_ble_resolve_adv_data_by_type(
                    r->scan_rst.ble_adv,
                    r->scan_rst.adv_data_len + r->scan_rst.scan_rsp_len,
                    ESP_BLE_AD_MANUFACTURER_SPECIFIC_TYPE, &ml);

                char hex[64] = "none";
                if (mf && ml > 0) {
                    uint8_t n = ml > 16 ? 16 : ml;
                    for (uint8_t k = 0; k < n; k++) {
                        snprintf(hex + k * 3, 4, "%02X ", mf[k]);
                    }
                }
                ESP_LOGI(TAG, "[SCAN] %-20s rssi=%4d dji=%d mfg=[%s] %02X:%02X:%02X:%02X:%02X:%02X",
                         nbuf, r->scan_rst.rssi, bsp_link_is_dji_camera_adv(r), hex,
                         r->scan_rst.bda[0], r->scan_rst.bda[1], r->scan_rst.bda[2],
                         r->scan_rst.bda[3], r->scan_rst.bda[4], r->scan_rst.bda[5]);
            }
#endif
            // Check if it is a DJI camera advertisement
            // 检查是否为 DJI 相机广播
            if (!bsp_link_is_dji_camera_adv(r)) {
                break;
            }
            // Get the complete name from the advertisement data
            // 获取广播数据里的完整名称
            uint8_t *adv_name = NULL;
            uint8_t adv_name_len = 0;
            adv_name = esp_ble_resolve_adv_data_by_type(r->scan_rst.ble_adv,
                                r->scan_rst.adv_data_len + r->scan_rst.scan_rsp_len,
                                ESP_BLE_AD_TYPE_NAME_CMPL,
                                &adv_name_len);

            // Prepare a safe string pointer for logging
            // 为打印日志准备安全的字符串指针
            const char *adv_name_str = NULL;
            if (adv_name && adv_name_len > 0) {
                static char name_buf[64];
                size_t copy_len = adv_name_len < sizeof(name_buf) - 1 ? adv_name_len : sizeof(name_buf) - 1;
                memcpy(name_buf, adv_name, copy_len);
                name_buf[copy_len] = '\0';
                adv_name_str = name_buf;
            } else {
                adv_name_str = "NULL";
            }

            /* The advert TYPE, not just its presence.
             *
             * A camera that is busy -- paired to a phone, or otherwise not
             * taking connections -- can still advertise, and then every connect
             * attempt fails with 0x3e for a reason no amount of signal strength
             * explains. ADV_IND means it will accept a connection;
             * ADV_NONCONN_IND and ADV_SCAN_IND mean it is talking but not
             * listening, which is a completely different problem and until now
             * looked identical. */
            const char *advk =
                r->scan_rst.ble_evt_type == ESP_BLE_EVT_CONN_ADV      ? "connectable"
              : r->scan_rst.ble_evt_type == ESP_BLE_EVT_CONN_DIR_ADV  ? "directed"
              : r->scan_rst.ble_evt_type == ESP_BLE_EVT_DISC_ADV      ? "scannable, NOT connectable"
              : r->scan_rst.ble_evt_type == ESP_BLE_EVT_NON_CONN_ADV  ? "NOT connectable"
              : r->scan_rst.ble_evt_type == ESP_BLE_EVT_SCAN_RSP      ? "scan response"
                                                                      : "?";
            ESP_LOGI(TAG, "Found device: %s with RSSI: %d [%s], MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                adv_name_str,
                r->scan_rst.rssi, advk,
                r->scan_rst.bda[0], r->scan_rst.bda[1], r->scan_rst.bda[2],
                r->scan_rst.bda[3], r->scan_rst.bda[4], r->scan_rst.bda[5]);

            // Compare names and record signal strength
            // 对比名称并记录信号强度
            if (ble_get_reconnecting()) {
                // In reconnection mode, compare device addresses
                // 在重连模式下，比对设备地址
                if (memcmp(best_addr, r->scan_rst.bda, sizeof(esp_bd_addr_t)) == 0) {
                    s_found_previous_device = true;
                    s_best_addr_type = r->scan_rst.ble_addr_type;
                    ESP_LOGI(TAG, "Found previous device: %s, RSSI: %d", adv_name_str, r->scan_rst.rssi);
                }
            } else if (s_have_bound) {
                /* CAMLINK: bound to a specific camera -- ignore every other
                 * one, however loud. */
                if (memcmp(s_bound_addr, r->scan_rst.bda, sizeof(esp_bd_addr_t)) == 0 &&
                    r->scan_rst.rssi > best_rssi) {
                    best_rssi = r->scan_rst.rssi;
                    memcpy(best_addr, r->scan_rst.bda, sizeof(esp_bd_addr_t));
                    s_best_addr_type = r->scan_rst.ble_addr_type;
                    strncpy(s_remote_device_name, adv_name_str, sizeof(s_remote_device_name) - 1);
                    s_remote_device_name[sizeof(s_remote_device_name) - 1] = '\0';
                }
            }
            /* SLATE PATCH: upstream fell back here to "connect to whichever
             * camera is loudest". That is deleted, not disabled.
             *
             * With no binding stored, the correct behaviour is to connect to
             * NOTHING and wait. Adopting the nearest camera made "Forget"
             * impossible to observe -- the next boot silently re-bound to
             * whatever was in the room, so releasing a camera looked like it
             * had not saved -- and at a field with two pilots it would bind to
             * the wrong quad's camera outright.
             *
             * Cameras are now chosen by the user from the config-mode scan
             * list, which is the only place a binding is created. */
        }
        break;
    }

    default:
        break;
    }
}

/* HCI error codes, Core spec Vol 1 Part F. Only the ones a camera link
 * actually produces -- anything else prints as its number.
 *
 * The distinction that matters: 0x08 means WE stopped hearing the camera and
 * gave up, which is our supervision timeout being too tight for whatever the
 * camera was busy doing. 0x13 means the camera deliberately hung up. They look
 * identical from the outside and need opposite fixes. */
static const char *ble_disconnect_reason_name(uint8_t r)
{
    switch (r) {
    case 0x08: return "supervision timeout";      /* we stopped hearing it   */
    case 0x13: return "remote ended it";          /* the camera hung up      */
    case 0x14: return "remote out of resources";
    case 0x15: return "remote powered off";
    case 0x16: return "we ended it";              /* our own disconnect call */
    case 0x22: return "LMP response timeout";
    case 0x28: return "instant passed";
    case 0x3b: return "unacceptable conn params";
    case 0x3d: return "MIC failure";              /* bond mismatch           */
    case 0x3e: return "connection not established";
    default:   return "?";
    }
}

static void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param) {
    switch (event) {
    case ESP_GATTC_REG_EVT: {
        // Handle GATT client registration event
        // 处理 GATT 客户端注册事件
        if (param->reg.status == ESP_GATT_OK) {
            s_ble_profile.gattc_if = gattc_if;
            ESP_LOGI(TAG, "GATTC register OK, app_id=%d, gattc_if=%d",
                     param->reg.app_id, gattc_if);
        } else {
            ESP_LOGE(TAG, "GATTC register failed, status=%d", param->reg.status);
        }
        break;
    }
    case ESP_GATTC_CONNECT_EVT: {
        // Handle connection event
        // 处理连接事件
        s_ble_profile.conn_id = param->connect.conn_id;
        s_ble_profile.connection_status.is_connected = true;
        memcpy(s_ble_profile.remote_bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(TAG, "Connected, conn_id=%d", s_ble_profile.conn_id);

        /* SLATE PATCH: connection TX power can only be set after the link
         * exists, so it is applied here rather than in ble_init(). */
        camlink_ble_set_min_tx_power_conn();
        camlink_ble_request_slow_conn_params(s_ble_profile.remote_bda);

#if CONFIG_CAMLINK_BLE_BONDING
        /* Ask for an encrypted link before we speak the DJI protocol. */
        esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT);
#endif

        ESP_LOGI(TAG, "Connect to camera MAC: %02X:%02X:%02X:%02X:%02X:%02X", 
            param->connect.remote_bda[0],
            param->connect.remote_bda[1],
            param->connect.remote_bda[2],
            param->connect.remote_bda[3],
            param->connect.remote_bda[4],
            param->connect.remote_bda[5]);

        // Initiate MTU request
        // 发起 MTU 请求
        esp_ble_gattc_send_mtu_req(gattc_if, param->connect.conn_id);
        break;
    }
    case ESP_GATTC_OPEN_EVT: {
        // Handle connection open event
        // 处理连接打开事件
        s_connecting = false;
        if (param->open.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "Open failed, status=%d", param->open.status);
            break;
        }
        /* Reaching here means a link was genuinely established, which separates
         * "never worked" from "worked and then stopped" -- different problems
         * with different answers. */
        s_ever_connected  = true;
        s_fail_count      = 0;
        s_last_fail_reason = 0;
        ESP_LOGI(TAG, "Open success, MTU=%u", param->open.mtu);
        break;
    }
    case ESP_GATTC_CFG_MTU_EVT: {
        // Handle MTU configuration event
        // 处理 MTU 配置事件
        if (param->cfg_mtu.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "Config MTU Error, status=%d", param->cfg_mtu.status);
        }
        ESP_LOGI(TAG, "MTU=%d", param->cfg_mtu.mtu);

        // Start service discovery after MTU configuration
        // MTU 配置完后开始发现服务
        esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id, NULL);
        break;
    }
    case ESP_GATTC_SEARCH_RES_EVT: {
#if CONFIG_CAMLINK_BLE_SCAN_DEBUG
        /* SLATE ADDITION: log EVERY service, not just 0xFFF0. If a camera
         * exposes its control protocol somewhere else, upstream's UUID filter
         * hides that completely. */
        {
            const esp_gatt_id_t *sid = &param->search_res.srvc_id;
            if (sid->uuid.len == ESP_UUID_LEN_16) {
                ESP_LOGI(TAG, "[GATT] service 0x%04X  handles %d..%d",
                         sid->uuid.uuid.uuid16,
                         param->search_res.start_handle, param->search_res.end_handle);
            } else if (sid->uuid.len == ESP_UUID_LEN_128) {
                char b[48]; int n = 0;
                for (int k = 15; k >= 0; k--) {
                    n += snprintf(b + n, sizeof(b) - n, "%02X", sid->uuid.uuid.uuid128[k]);
                }
                ESP_LOGI(TAG, "[GATT] service %s  handles %d..%d", b,
                         param->search_res.start_handle, param->search_res.end_handle);
            }
        }
#endif

        // Handle service search result event
        // 处理服务搜索结果事件
        if ((param->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16) &&
            (param->search_res.srvc_id.uuid.uuid.uuid16 == REMOTE_TARGET_SERVICE_UUID)) {
            s_ble_profile.service_start_handle = param->search_res.start_handle;
            s_ble_profile.service_end_handle   = param->search_res.end_handle;
            ESP_LOGI(TAG, "Service found: start=%d, end=%d",
                     s_ble_profile.service_start_handle,
                     s_ble_profile.service_end_handle);
        }
        break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT: {
#if CONFIG_CAMLINK_BLE_SCAN_DEBUG
        /* Dump every characteristic in the whole database with its properties,
         * so a wrong-characteristic theory can be settled by looking rather
         * than guessing. */
        {
            uint16_t off = 0;
            for (;;) {
                esp_gattc_char_elem_t ch[8];
                uint16_t cnt = 8;
                esp_gatt_status_t st = esp_ble_gattc_get_all_char(
                    gattc_if, s_ble_profile.conn_id, 1, 0xFFFF, ch, &cnt, off);
                if (st != ESP_GATT_OK || cnt == 0) break;
                for (uint16_t i = 0; i < cnt; i++) {
                    if (ch[i].uuid.len == ESP_UUID_LEN_16) {
                        ESP_LOGI(TAG, "[GATT]   char 0x%04X handle=0x%02x props=0x%02x%s%s%s%s",
                                 ch[i].uuid.uuid.uuid16, ch[i].char_handle, ch[i].properties,
                                 (ch[i].properties & ESP_GATT_CHAR_PROP_BIT_READ)      ? " READ"   : "",
                                 (ch[i].properties & ESP_GATT_CHAR_PROP_BIT_WRITE)     ? " WRITE"  : "",
                                 (ch[i].properties & ESP_GATT_CHAR_PROP_BIT_WRITE_NR)  ? " WRITE_NR": "",
                                 (ch[i].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY)    ? " NOTIFY" : "");
                    } else {
                        char b[48]; int n = 0;
                        for (int k = 15; k >= 0; k--) {
                            n += snprintf(b + n, sizeof(b) - n, "%02X", ch[i].uuid.uuid.uuid128[k]);
                        }
                        ESP_LOGI(TAG, "[GATT]   char %s handle=0x%02x props=0x%02x",
                                 b, ch[i].char_handle, ch[i].properties);
                    }
                }
                off += cnt;
                if (cnt < 8) break;
            }
        }
#endif

        // Handle service search complete event
        // 处理服务搜索完成事件
        if (param->search_cmpl.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "Service search failed, status=%d", param->search_cmpl.status);
            break;
        }
        ESP_LOGI(TAG, "Service search complete, next get char by UUID");

        // Get notify characteristic handle
        // 获取通知特征句柄
        uint16_t count = 1;
        esp_gattc_char_elem_t char_elem_result;
        esp_ble_gattc_get_char_by_uuid(gattc_if,
                                       s_ble_profile.conn_id,
                                       s_ble_profile.service_start_handle,
                                       s_ble_profile.service_end_handle,
                                       s_filter_notify_char_uuid,
                                       &char_elem_result,
                                       &count);
        if (count > 0) {
            s_ble_profile.notify_char_handle = char_elem_result.char_handle;
            s_ble_profile.handle_discovery.notify_char_handle_found = true;
            ESP_LOGI(TAG, "Notify Char found, handle=0x%x",
                     s_ble_profile.notify_char_handle);
        }

        // Get write characteristic handle
        // 获取写特征句柄
        count = 1;
        esp_gattc_char_elem_t write_char_elem_result;
        esp_ble_gattc_get_char_by_uuid(gattc_if,
                                       s_ble_profile.conn_id,
                                       s_ble_profile.service_start_handle,
                                       s_ble_profile.service_end_handle,
                                       s_filter_write_char_uuid,
                                       &write_char_elem_result,
                                       &count);
        if (count > 0) {
            s_ble_profile.write_char_handle = write_char_elem_result.char_handle;
            s_ble_profile.write_char_props  = write_char_elem_result.properties;

            /* CAMLINK: override with this attempt's candidate, if it is not
             * the default 0xFFF5 that upstream assumes. */
            {
                uint16_t want = s_write_candidates[s_write_candidate_idx %
                    (sizeof(s_write_candidates)/sizeof(s_write_candidates[0]))];
                s_write_candidate_idx++;
                if (want != REMOTE_WRITE_CHAR_UUID) {
                    esp_bt_uuid_t u = { .len = ESP_UUID_LEN_16, .uuid.uuid16 = want };
                    esp_gattc_char_elem_t alt;
                    uint16_t c1 = 1;
                    esp_ble_gattc_get_char_by_uuid(gattc_if, s_ble_profile.conn_id,
                                                   s_ble_profile.service_start_handle,
                                                   s_ble_profile.service_end_handle,
                                                   u, &alt, &c1);
                    if (c1 > 0) {
                        s_ble_profile.write_char_handle = alt.char_handle;
                        s_ble_profile.write_char_props  = alt.properties;
                    }
                }
                ESP_LOGD(TAG, "writing commands to 0x%04X (handle=0x%02x props=0x%02x)",
                         want, s_ble_profile.write_char_handle, s_ble_profile.write_char_props);
            }
            s_ble_profile.handle_discovery.write_char_handle_found = true;
            ESP_LOGI(TAG, "Write Char props=0x%02x (%s)",
                     write_char_elem_result.properties,
                     (write_char_elem_result.properties & ESP_GATT_CHAR_PROP_BIT_WRITE)
                         ? "supports write-with-response"
                         : "WRITE_NR only -- will use write-without-response");
            ESP_LOGI(TAG, "Write Char found, handle=0x%x",
                     s_ble_profile.write_char_handle);
        }

        /* SLATE ADDITION: subscribe to EVERY notifying characteristic in the
         * DJI service, not just 0xFFF4.
         *
         * DJI's FAQ says the camera talks on 0xFFF4, but on the Osmo Nano
         * 0xFFF3 and 0xFFF5 also carry the NOTIFY property. If this camera
         * answers on one of those instead, subscribing only to 0xFFF4 makes the
         * reply invisible and looks exactly like a camera that never answered. */
        {
            uint16_t off = 0;
            for (;;) {
                esp_gattc_char_elem_t ch[8];
                uint16_t cnt = 8;
                esp_gatt_status_t st = esp_ble_gattc_get_all_char(
                    gattc_if, s_ble_profile.conn_id,
                    s_ble_profile.service_start_handle,
                    s_ble_profile.service_end_handle, ch, &cnt, off);
                if (st != ESP_GATT_OK || cnt == 0) break;
                for (uint16_t k = 0; k < cnt; k++) {
                    if ((ch[k].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY) &&
                        ch[k].char_handle != s_ble_profile.notify_char_handle) {
                        esp_err_t e = esp_ble_gattc_register_for_notify(
                            gattc_if, s_ble_profile.remote_bda, ch[k].char_handle);
                        ESP_LOGI(TAG, "also subscribing to char handle 0x%02x: %s",
                                 ch[k].char_handle, esp_err_to_name(e));
                    }
                }
                off += cnt;
                if (cnt < 8) break;
            }
        }

#if CONFIG_CAMLINK_BLE_SCAN_DEBUG
        /* SLATE ADDITION: queue a one-shot read of every readable
         * characteristic in the database (see camlink_read_next).
         * Diagnostic only -- it adds a second or so to every connection. */
        {
            s_read_q_n = 0;
            s_read_q_i = 0;
            uint16_t off = 0;
            for (;;) {
                esp_gattc_char_elem_t ch[8];
                uint16_t cnt = 8;
                esp_gatt_status_t st = esp_ble_gattc_get_all_char(
                    gattc_if, s_ble_profile.conn_id, 1, 0xFFFF, ch, &cnt, off);
                if (st != ESP_GATT_OK || cnt == 0) break;
                for (uint16_t k = 0; k < cnt && s_read_q_n < CAMLINK_READ_QUEUE_MAX; k++) {
                    if ((ch[k].properties & ESP_GATT_CHAR_PROP_BIT_READ) &&
                        ch[k].uuid.len == ESP_UUID_LEN_16) {
                        s_read_q_uuid[s_read_q_n] = ch[k].uuid.uuid.uuid16;
                        s_read_q[s_read_q_n++]    = ch[k].char_handle;
                    }
                }
                off += cnt;
                if (cnt < 8) break;
            }
            ESP_LOGI(TAG, "[READ] reading %u characteristics...", (unsigned)s_read_q_n);
            camlink_read_next(gattc_if);
        }
#endif

        break;
    }
    case ESP_GATTC_READ_CHAR_EVT: {
        /* SLATE ADDITION: log each value as hex plus printable ASCII. */
        uint16_t uuid = (s_read_q_i < s_read_q_n) ? s_read_q_uuid[s_read_q_i] : 0;
        if (param->read.status == ESP_GATT_OK) {
            char hex[3 * 24 + 1] = {0};
            char asc[24 + 1] = {0};
            uint16_t n = param->read.value_len > 24 ? 24 : param->read.value_len;
            for (uint16_t i = 0; i < n; i++) {
                snprintf(hex + i * 3, 4, "%02X ", param->read.value[i]);
                asc[i] = (param->read.value[i] >= 0x20 && param->read.value[i] < 0x7F)
                             ? (char)param->read.value[i] : '.';
            }
            ESP_LOGI(TAG, "[READ] 0x%04X handle=0x%02x len=%u [%s] \"%s\"",
                     uuid, param->read.handle, param->read.value_len, hex, asc);
        } else {
            ESP_LOGW(TAG, "[READ] 0x%04X handle=0x%02x FAILED status=0x%x",
                     uuid, param->read.handle, param->read.status);
        }
        s_read_q_i++;
        camlink_read_next(gattc_if);
        break;
    }
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        // Handle notification registration event
        // 处理通知注册事件
        if (param->reg_for_notify.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "Notify register failed, status=%d", param->reg_for_notify.status);
            break;
        }
        ESP_LOGI(TAG, "Notify register success, handle=0x%x", param->reg_for_notify.handle);

        // Find descriptor and write 0x01 to enable notification
        // 找到对应描述符并写入 0x01 使能通知
        uint16_t count = 1;
        esp_gattc_descr_elem_t descr_elem;
        esp_ble_gattc_get_descr_by_char_handle(gattc_if,
                                               s_ble_profile.conn_id,
                                               param->reg_for_notify.handle,
                                               s_notify_descr_uuid,
                                               &descr_elem,
                                               &count);
        /* SLATE ADDITION: upstream skipped the CCCD write silently when the
         * descriptor was not found. Without that write the camera never sends
         * a single notification, which looks exactly like a camera ignoring
         * our commands. Log what actually happened. */
        if (count > 0 && descr_elem.handle) {
            uint16_t notify_en = 1;
            esp_err_t we = esp_ble_gattc_write_char_descr(gattc_if,
                                           s_ble_profile.conn_id,
                                           descr_elem.handle,
                                           sizeof(notify_en),
                                           (uint8_t *)&notify_en,
                                           ESP_GATT_WRITE_TYPE_RSP,
                                           ESP_GATT_AUTH_REQ_NONE);
            ESP_LOGI(TAG, "CCCD write issued: descr handle=0x%x, count=%u, err=%s",
                     descr_elem.handle, (unsigned)count, esp_err_to_name(we));
        } else {
            ESP_LOGE(TAG, "CCCD (0x2902) NOT FOUND for char handle 0x%x (count=%u) -- "
                          "notifications will never arrive",
                     param->reg_for_notify.handle, (unsigned)count);
        }
        break;
    }
    case ESP_GATTC_WRITE_DESCR_EVT: {
        if (param->write.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "CCCD write FAILED, status=0x%x handle=0x%x",
                     param->write.status, param->write.handle);
        } else {
            ESP_LOGI(TAG, "CCCD write OK, notifications enabled (handle=0x%x)",
                     param->write.handle);
        }
        break;
    }
    case ESP_GATTC_WRITE_CHAR_EVT: {
        if (param->write.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "characteristic write FAILED, status=0x%x handle=0x%x",
                     param->write.status, param->write.handle);
        }
        break;
    }
    case ESP_GATTC_NOTIFY_EVT: {
        // Handle notification data event
        // 处理通知数据事件

        if (s_notify_cb) {
            s_notify_cb(param->notify.value, param->notify.value_len);
        }
        break;
    }
    case ESP_GATTC_DISCONNECT_EVT: {
        // Handle disconnection event
        // 处理断开连接事件
        s_ble_profile.connection_status.is_connected = false;
        s_ble_profile.handle_discovery.write_char_handle_found = false;
        s_ble_profile.handle_discovery.notify_char_handle_found = false;
        s_connecting = false;
        /* WARN with a name, not INFO with a number. A dropped camera link is
         * the single most reported fault, and "reason=0x8" versus
         * "reason=0x13" is the difference between "our supervision timeout is
         * too tight" and "the camera hung up on us" -- two opposite fixes. It
         * should never need a spec lookup, or a debug build, to tell them
         * apart. */
        ESP_LOGW(TAG, "Disconnected, reason=0x%02x (%s)", param->disconnect.reason,
                 ble_disconnect_reason_name(param->disconnect.reason));
        s_last_fail_reason = param->disconnect.reason;
        s_fail_count++;

        if (s_state_cb) {
            s_state_cb();
        }
        break;
    }
    default:
        break;
    }
}

// BLE Advertising Data Format
// 广播数据
static uint8_t adv_data[] = {
    10, 0xff, 'W','K','P','1','2','3','4','5','6'
};

static esp_timer_handle_t adv_timer;

static void stop_adv_after_2s(void* arg) {
    esp_ble_gap_stop_advertising();
    ESP_LOGI(TAG, "Advertising stopped after 2 seconds");
}

esp_err_t ble_start_advertising() {
    // Check if remote_bda is initialized
    // 检查remote_bda是否已初始化
    if (memcmp(s_ble_profile.remote_bda, "\x00\x00\x00\x00\x00\x00", 6) == 0) {
        ESP_LOGE(TAG, "错误：remote_bda未初始化！");
        ESP_LOGE(TAG, "Error: remote_bda not initialized!");
        return ESP_ERR_INVALID_STATE;
    }

    for (int i = 0; i < 6; i++) {
        // adv_data[8 + i] = s_ble_profile.remote_bda[5 - i];
        adv_data[5 + i] = s_ble_profile.remote_bda[5 - i];
    }

    ESP_LOGI(TAG, "Modified Advertising Data (with MAC):");
    ESP_LOG_BUFFER_HEX(TAG, adv_data, sizeof(adv_data));

    esp_ble_adv_params_t adv_params = {
        .adv_int_min = 0x20,
        .adv_int_max = 0x40,
        .adv_type = ADV_TYPE_IND,
        .channel_map = ADV_CHNL_ALL,
    };

    esp_err_t ret = esp_ble_gap_config_adv_data_raw(adv_data, sizeof(adv_data));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set adv data.");
        return ret;
    }

    ret = esp_ble_gap_start_advertising(&adv_params);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start advertising: %s", esp_err_to_name(ret));
        return ret;
    }

    if (adv_timer == NULL) {
        const esp_timer_create_args_t timer_args = {
            .callback = &stop_adv_after_2s,
            .name = "adv_timer"
        };
        esp_timer_create(&timer_args, &adv_timer);
    }
    
    esp_timer_start_once(adv_timer, 2000000);  // 2000ms = 2,000,000us

    ESP_LOGI(TAG, "Advertising started (will auto-stop after 2s)");
    return ESP_OK;
}
