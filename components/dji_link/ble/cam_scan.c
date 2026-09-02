/* SPDX-License-Identifier: MIT
 *
 * Config-mode camera scanner.
 *
 * Discovery only: this brings up the BLE controller and GAP, watches for DJI
 * camera advertisements, and keeps a short list of what it can see. It never
 * connects. Connecting is what the normal boot does, and keeping the two apart
 * means the settings page cannot accidentally take over a camera while the
 * user is only looking at a list.
 *
 * It runs alongside the Wi-Fi access point. One radio serves both, arbitrated
 * by the coexistence layer, which is why the scan duty cycle below is
 * deliberately modest: a scanner that hogs the radio makes the page it is
 * feeding unresponsive, and the user is looking at that page.
 *
 * Why a picker at all: binding used to adopt whichever camera advertised
 * loudest. That is fine with one camera on a desk and wrong the moment a
 * second one is in range -- and it is unverifiable, because the user cannot
 * see what they got until it is already wrong.
 */
#include "cam_scan.h"
#include "ble.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_timer.h"

static const char *TAG = "CAMSCAN";

/* An entry not seen for this long is dropped from the list. A camera that has
 * been switched off must stop being offered promptly: presenting a stale entry
 * invites the user to bind to something that is not there, and then wait for a
 * connection that can never happen. */
/* Twelve seconds was long enough that a camera switched off in front of the
 * user stayed on the list well past the point it read as the page being stale.
 * DJI cameras advertise several times a second and the scanner runs at a ~60%
 * duty cycle, so five seconds is still many missed adverts before an entry is
 * dropped -- comfortably more than a fading signal needs, and quick enough that
 * switching a camera off looks like switching it off. */
#define FORGET_MS   5000

/* How long the scanner may hear nothing at all before it assumes it has stopped
 * listening rather than that the room is empty. Long enough not to fight a busy
 * radio, short enough that switching a camera on and waiting feels like it
 * worked. */
#define SCAN_QUIET_MS  6000

static cam_scan_entry_t  s_tbl[CAM_SCAN_MAX];
static int               s_count;
static SemaphoreHandle_t s_lock;
static bool              s_running;

static esp_ble_scan_params_t s_scan_params = {
    .scan_type          = BLE_SCAN_TYPE_ACTIVE,   /* ask for scan responses:
                                                     the name often lives there */
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    /* 30 ms of every 100 ms. Enough to find a camera 20 cm away within a
     * second or two, while leaving the radio to the access point the rest of
     * the time. */
    .scan_interval      = 0x00A0,
    .scan_window        = 0x0030,
    .scan_duplicate     = BLE_SCAN_DUPLICATE_DISABLE,  /* we want RSSI updates */
};

/* Manufacturer-data byte 2 is a model code. Codes marked (m) are measured on
 * hardware here; the rest come from datagutt/node-osmo, which identifies DJI
 * cameras from the same advert field.
 *
 * Only codes with a source are named. An unrecognised camera still appears in
 * the picker with its advertised name and MAC -- a mislabelled camera is worse
 * than an unlabelled one, and guessing at a code would produce exactly that. */
static uint8_t model_code(const uint8_t *adv, uint8_t adv_len)
{
    for (int i = 0; i < adv_len; ) {
        const uint8_t len = adv[i];
        if (len == 0 || (i + len + 1) > adv_len) break;
        if (adv[i + 1] == ESP_BLE_AD_MANUFACTURER_SPECIFIC_TYPE) {
            const uint8_t *d = &adv[i + 2];
            if ((len - 1) >= 3 && d[0] == 0xAA && d[1] == 0x08) return d[2];
        }
        i += len + 1;
    }
    return 0;
}

const char *cam_scan_model_name(uint8_t model)
{
    switch (model) {
    case 0x12: return "Osmo Action 3";
    case 0x14: return "Osmo Action 4";
    case 0x15: return "Osmo Action 5 Pro";
    case 0x17: return "Osmo 360";          /* (m) measured */
    case 0x19: return "Osmo Nano";         /* (m) measured */
    default:   return "";                  /* incl. Action 6, whose code is
                                            * undocumented -- it still appears
                                            * by name, and binding resolves the
                                            * protocol from that. */
    }
}

static void upsert(const uint8_t *bda, const char *name, int8_t rssi, uint8_t model)
{
    if (!s_lock || xSemaphoreTake(s_lock, pdMS_TO_TICKS(20)) != pdTRUE) return;

    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    int slot = -1;

    for (int i = 0; i < s_count; i++) {
        if (memcmp(s_tbl[i].bda, bda, 6) == 0) { slot = i; break; }
    }
    if (slot < 0) {
        if (s_count < CAM_SCAN_MAX) {
            slot = s_count++;
        } else {
            /* Full: replace the weakest. The user's own camera is centimetres
             * away, so it is never the one evicted. */
            int worst = 0;
            for (int i = 1; i < s_count; i++) {
                if (s_tbl[i].rssi < s_tbl[worst].rssi) worst = i;
            }
            if (rssi <= s_tbl[worst].rssi) { xSemaphoreGive(s_lock); return; }
            slot = worst;
        }
        memset(&s_tbl[slot], 0, sizeof(s_tbl[slot]));
        memcpy(s_tbl[slot].bda, bda, 6);
        /* Log first sightings only. A camera appearing (or never appearing) is
         * the one thing worth seeing on the serial console from config mode --
         * every subsequent advert would just be noise. */
        /* Raw model byte included deliberately: it is the first thing needed
         * when bringing up a camera this firmware has never seen, and without
         * it an unrecognised model logs an empty string and tells you nothing. */
        ESP_LOGI(TAG, "found %02X:%02X:%02X:%02X:%02X:%02X  %-18s model=0x%02X %-18s %d dBm",
                 bda[0], bda[1], bda[2], bda[3], bda[4], bda[5],
                 (name && name[0]) ? name : "(no name)",
                 model, cam_scan_model_name(model), rssi);
    }

    s_tbl[slot].rssi    = rssi;
    s_tbl[slot].last_ms = now;
    if (model) s_tbl[slot].model = model;
    if (name && name[0] && s_tbl[slot].name[0] == '\0') {
        strlcpy(s_tbl[slot].name, name, sizeof(s_tbl[slot].name));
    }
    xSemaphoreGive(s_lock);
}

/* When the scan last produced ANY result, filtered or not.
 *
 * A camera switched on after config mode was entered never appeared, and the
 * scanner had no way to say whether it was still running: a start that fails is
 * a return code nobody reads, and a scan the controller drops produces no event
 * at all. Both look exactly like an empty room. This is the evidence that
 * distinguishes them, and the watchdog below acts on it. */
static uint32_t s_last_result_ms;
static esp_timer_handle_t s_watchdog;

static uint32_t scan_now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static void scan_kick(void)
{
    esp_err_t e = esp_ble_gap_start_scanning(0);   /* 0 = until we stop it */
    if (e != ESP_OK) {
        ESP_LOGW(TAG, "start scanning failed: %s", esp_err_to_name(e));
    }
}

/* Restart a scan that has gone quiet.
 *
 * Deliberately not conditional on having ever seen a camera: in a room with no
 * BLE at all this restarts every few seconds for nothing, which costs nothing.
 * The alternative -- deciding the silence is legitimate -- is how the original
 * fault stayed invisible. A scanner that cannot tell "nothing is there" from
 * "I stopped looking" should assume the second, because only one of them is
 * worth doing something about. */
static void scan_watchdog(void *arg)
{
    (void)arg;
    if (!s_running) return;
    uint32_t quiet = scan_now_ms() - s_last_result_ms;
    if (quiet < SCAN_QUIET_MS) return;

    ESP_LOGI(TAG, "no scan results for %u ms -- restarting the scan", (unsigned)quiet);
    s_last_result_ms = scan_now_ms();
    esp_ble_gap_stop_scanning();   /* STOP_COMPLETE restarts it */
    scan_kick();                   /* ...and if no event arrives, this does */
}

static void gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        scan_kick();
        break;

    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        /* Upstream never handled this, so a refused start was silent. Starting
         * the access point immediately afterwards is exactly the sort of thing
         * that refuses one. */
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGW(TAG, "scan did not start (status %d) -- retrying",
                     param->scan_start_cmpl.status);
        } else {
            ESP_LOGI(TAG, "scanning");
        }
        s_last_result_ms = scan_now_ms();
        break;

    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        if (param->scan_rst.search_evt != ESP_GAP_SEARCH_INQ_RES_EVT) break;
        /* Counted BEFORE the camera filter: any advert at all proves the radio
         * is still listening, which is the only thing the watchdog needs. */
        s_last_result_ms = scan_now_ms();
        if (!bsp_link_is_dji_camera_adv(param)) break;

        uint8_t adv_len = param->scan_rst.adv_data_len + param->scan_rst.scan_rsp_len;
        uint8_t nlen = 0;
        uint8_t *n = esp_ble_resolve_adv_data_by_type(param->scan_rst.ble_adv, adv_len,
                                                      ESP_BLE_AD_TYPE_NAME_CMPL, &nlen);
        if (n == NULL || nlen == 0) {
            n = esp_ble_resolve_adv_data_by_type(param->scan_rst.ble_adv, adv_len,
                                                 ESP_BLE_AD_TYPE_NAME_SHORT, &nlen);
        }
        char name[CAM_SCAN_NAME_LEN] = {0};
        if (n && nlen) {
            size_t c = nlen < sizeof(name) - 1 ? nlen : sizeof(name) - 1;
            memcpy(name, n, c);
        }
        upsert(param->scan_rst.bda, name, param->scan_rst.rssi,
               model_code(param->scan_rst.ble_adv, adv_len));
        break;
    }

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        /* Keep looking. The list has to stay live while the user reads it --
         * a camera switched on while they are looking at the page has to turn
         * up without them having to leave and come back. */
        if (s_running) scan_kick();
        break;

    default:
        break;
    }
}

esp_err_t cam_scan_start(void)
{
    esp_err_t e;

    s_lock = xSemaphoreCreateMutex();

    esp_bt_controller_config_t cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((e = esp_bt_controller_init(&cfg)) != ESP_OK)                 return e;
    if ((e = esp_bt_controller_enable(ESP_BT_MODE_BLE)) != ESP_OK)    return e;
    if ((e = esp_bluedroid_init()) != ESP_OK)                         return e;
    if ((e = esp_bluedroid_enable()) != ESP_OK)                       return e;
    if ((e = esp_ble_gap_register_callback(gap_cb)) != ESP_OK)        return e;

    /* Scanning transmits on an active scan, so the same reasoning as the
     * connect path applies: stay quiet. */
    camlink_ble_set_min_tx_power_static();

    s_running = true;
    s_last_result_ms = scan_now_ms();
    if ((e = esp_ble_gap_set_scan_params(&s_scan_params)) != ESP_OK)  return e;

    /* The watchdog runs for as long as config mode does. It exists because the
     * access point is started immediately after this returns, and a scan the
     * radio drops while that happens announces itself in no way at all. */
    const esp_timer_create_args_t wd = {
        .callback = scan_watchdog,
        .name     = "camscan_wd",
    };
    if (esp_timer_create(&wd, &s_watchdog) == ESP_OK) {
        esp_timer_start_periodic(s_watchdog, (uint64_t)SCAN_QUIET_MS * 1000);
    } else {
        ESP_LOGW(TAG, "no scan watchdog -- a dropped scan will stay dropped");
    }

    ESP_LOGI(TAG, "scanning for cameras");
    return ESP_OK;
}

int cam_scan_get(cam_scan_entry_t *out, int max)
{
    if (out == NULL || max <= 0 || s_lock == NULL) return 0;
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(50)) != pdTRUE) return 0;

    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);

    /* Drop anything gone quiet, compacting in place. */
    int keep = 0;
    for (int i = 0; i < s_count; i++) {
        if ((now - s_tbl[i].last_ms) <= FORGET_MS) {
            if (keep != i) s_tbl[keep] = s_tbl[i];
            keep++;
        }
    }
    s_count = keep;

    int n = s_count < max ? s_count : max;
    memcpy(out, s_tbl, (size_t)n * sizeof(cam_scan_entry_t));
    xSemaphoreGive(s_lock);

    /* First-detected order, which is the order the table is already in --
     * entries are appended and the compaction above preserves it.
     *
     * This used to sort by signal strength, on the reasoning that the user's own
     * camera is centimetres away and everyone else's is metres away, so the one
     * they want would sit at the top. What that missed is that RSSI moves by ten
     * dB between adverts on a stationary device: two cameras of similar strength
     * swapped places continuously, and the list reordered itself under the
     * user's finger while they were reaching for an entry. A list that is
     * roughly sorted but never still is worse to use than one in an arbitrary
     * fixed order. The signal bars are still drawn per entry, so "which one is
     * mine" is still answered -- by looking, not by position. */
    return n;
}
