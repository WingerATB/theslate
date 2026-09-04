/* SPDX-License-Identifier: MIT
 *
 * Config-mode camera scanner.
 *
 * Discovery only: this brings up the BLE controller and GAP, watches for DJI
 * and GoPro camera advertisements, and keeps a short list of what it can see. It never
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

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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
    /* 60 ms of every 100 ms: aggressive, because the scanner no longer runs
     * alongside Wi-Fi. On this chip an enabled Bluetooth controller starves
     * the softAP's beacon no matter how little it scans, so config mode now
     * takes a short camera census with Wi-Fi still down, THEN shuts Bluetooth
     * off entirely and brings the access point up alone (see cam_scan_stop and
     * config_mode). With the air to itself, the scan may as well be quick: a
     * 60% window finds a camera on the desk in well under a second. */
    .scan_interval      = 0x00A0,
    .scan_window        = 0x0060,
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

const char *cam_scan_entry_model(const cam_scan_entry_t *e)
{
    if (e->vendor == CAM_VENDOR_GOPRO) return "GoPro";
    return cam_scan_model_name(e->model);
}

/* --------------------------------------------------------------------------
 * GoPro adverts.
 *
 * Per the Open GoPro BLE spec a camera advertises the 16-bit service UUID
 * 0xFEA6 and manufacturer data under GoPro's company ID 0x02F2, and its scan
 * response carries the name, "GoPro 1234". Any one of the three is accepted:
 * the adverts and the scan response arrive as separate events, the name lives
 * only in the latter, and a filter that demanded all three at once would
 * reject the very advert that proves the camera is there.
 *
 * The HERO (2024) this is being brought up against is not on Open GoPro's
 * supported list, so what it actually advertises is the first thing worth
 * knowing -- which is why a GoPro's first sighting logs its raw manufacturer
 * bytes below, where the DJI line logs a model code.
 * -------------------------------------------------------------------------- */
#define GOPRO_SVC_UUID16   0xFEA6
#define GOPRO_COMPANY_ID   0x02F2

static bool is_gopro_adv(const uint8_t *adv, uint8_t adv_len)
{
    for (int i = 0; i < adv_len; ) {
        const uint8_t len = adv[i];
        if (len == 0 || (i + len + 1) > adv_len) break;
        const uint8_t  type = adv[i + 1];
        const uint8_t *d    = &adv[i + 2];
        const uint8_t  dlen = len - 1;

        if (type == ESP_BLE_AD_TYPE_16SRV_PART || type == ESP_BLE_AD_TYPE_16SRV_CMPL) {
            for (int k = 0; k + 1 < dlen; k += 2) {
                if (d[k] == (GOPRO_SVC_UUID16 & 0xFF) && d[k + 1] == (GOPRO_SVC_UUID16 >> 8)) {
                    return true;
                }
            }
        } else if (type == ESP_BLE_AD_MANUFACTURER_SPECIFIC_TYPE) {
            /* Company ID is little-endian on the air. The other order is
             * accepted too: the spec writes it as "0xF202", and if a firmware
             * takes that literally, rejecting it would hide the camera. */
            if (dlen >= 2 &&
                ((d[0] == (GOPRO_COMPANY_ID & 0xFF) && d[1] == (GOPRO_COMPANY_ID >> 8)) ||
                 (d[1] == (GOPRO_COMPANY_ID & 0xFF) && d[0] == (GOPRO_COMPANY_ID >> 8)))) {
                return true;
            }
        } else if (type == ESP_BLE_AD_TYPE_NAME_CMPL || type == ESP_BLE_AD_TYPE_NAME_SHORT) {
            if (dlen >= 5 && memcmp(d, "GoPro", 5) == 0) return true;
        }
        i += len + 1;
    }
    return false;
}

/* The manufacturer payload as hex, for the first-sighting log. */
static void mfg_hex(const uint8_t *adv, uint8_t adv_len, char *out, size_t out_len)
{
    out[0] = '\0';
    for (int i = 0; i < adv_len; ) {
        const uint8_t len = adv[i];
        if (len == 0 || (i + len + 1) > adv_len) break;
        if (adv[i + 1] == ESP_BLE_AD_MANUFACTURER_SPECIFIC_TYPE) {
            size_t w = 0;
            for (uint8_t k = 0; k < len - 1 && w + 3 < out_len; k++) {
                w += (size_t)snprintf(out + w, out_len - w, "%02X ", adv[i + 2 + k]);
            }
            return;
        }
        i += len + 1;
    }
}

static void upsert(const uint8_t *bda, const char *name, int8_t rssi, uint8_t model,
                   cam_vendor_t vendor, const char *mfg, uint8_t addr_type)
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
        if (vendor == CAM_VENDOR_GOPRO) {
            ESP_LOGI(TAG, "found %02X:%02X:%02X:%02X:%02X:%02X  %-18s GoPro mfg=[%s] %d dBm",
                     bda[0], bda[1], bda[2], bda[3], bda[4], bda[5],
                     (name && name[0]) ? name : "(no name)", mfg, rssi);
        } else {
            ESP_LOGI(TAG, "found %02X:%02X:%02X:%02X:%02X:%02X  %-18s model=0x%02X %-18s %d dBm",
                     bda[0], bda[1], bda[2], bda[3], bda[4], bda[5],
                     (name && name[0]) ? name : "(no name)",
                     model, cam_scan_model_name(model), rssi);
        }
        s_tbl[slot].vendor    = (uint8_t)vendor;
        s_tbl[slot].addr_type = addr_type;
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

/* --------------------------------------------------------------------------
 * Raw advert sniffer (diagnostic).
 *
 * Logs every BLE device heard, once each, with its name, address, signal, the
 * full advertising payload and whether it is connectable. This exists to bring
 * up a camera the vendor filters do not yet recognise -- an older GoPro, say --
 * by showing exactly what it broadcasts, so the filter can be taught to match
 * it. Bounded to one line per address so a busy room does not flood the log.
 * -------------------------------------------------------------------------- */
static uint8_t s_sniff[24][6];
static int     s_sniff_n;

static bool sniff_seen(const uint8_t *bda)
{
    for (int i = 0; i < s_sniff_n; i++) if (memcmp(s_sniff[i], bda, 6) == 0) return true;
    if (s_sniff_n < (int)(sizeof(s_sniff) / 6)) { memcpy(s_sniff[s_sniff_n++], bda, 6); }
    return false;
}

static void sniff_advert(esp_ble_gap_cb_param_t *param)
{
    const uint8_t *bda = param->scan_rst.bda;
    if (sniff_seen(bda)) return;

    uint8_t adv_len = param->scan_rst.adv_data_len + param->scan_rst.scan_rsp_len;
    const uint8_t *adv = param->scan_rst.ble_adv;

    uint8_t nlen = 0;
    uint8_t *nm = esp_ble_resolve_adv_data_by_type(adv, adv_len, ESP_BLE_AD_TYPE_NAME_CMPL, &nlen);
    if (!nm || !nlen) nm = esp_ble_resolve_adv_data_by_type(adv, adv_len, ESP_BLE_AD_TYPE_NAME_SHORT, &nlen);
    char name[32] = "(no name)";
    if (nm && nlen) { size_t c = nlen < sizeof(name) - 1 ? nlen : sizeof(name) - 1; memcpy(name, nm, c); name[c] = 0; }

    char hex[3 * 62 + 4];
    size_t w = 0;
    for (int i = 0; i < adv_len && i < 62 && w + 3 < sizeof(hex); i++) {
        w += (size_t)snprintf(hex + w, sizeof(hex) - w, "%02X ", adv[i]);
    }

    bool conn = param->scan_rst.ble_evt_type == ESP_BLE_EVT_CONN_ADV ||
                param->scan_rst.ble_evt_type == ESP_BLE_EVT_CONN_DIR_ADV;

    ESP_LOGW(TAG, "SNIFF %02X:%02X:%02X:%02X:%02X:%02X %-16s %4d dBm %s atype=%d %s",
             bda[0], bda[1], bda[2], bda[3], bda[4], bda[5], name,
             param->scan_rst.rssi, conn ? "connectable" : "not-conn",
             param->scan_rst.ble_addr_type, hex);
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
        uint8_t adv_len = param->scan_rst.adv_data_len + param->scan_rst.scan_rsp_len;

#if CONFIG_CAMLINK_SNIFF
        sniff_advert(param);
#endif

        cam_vendor_t vendor;
        if (bsp_link_is_dji_camera_adv(param)) {
            vendor = CAM_VENDOR_DJI;
        } else if (is_gopro_adv(param->scan_rst.ble_adv, adv_len)) {
            vendor = CAM_VENDOR_GOPRO;
        } else {
            break;
        }

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
        char mfg[3 * 31 + 1] = "";
        if (vendor == CAM_VENDOR_GOPRO) mfg_hex(param->scan_rst.ble_adv, adv_len, mfg, sizeof(mfg));
        upsert(param->scan_rst.bda, name, param->scan_rst.rssi,
               model_code(param->scan_rst.ble_adv, adv_len), vendor, mfg,
               (uint8_t)param->scan_rst.ble_addr_type);
        break;
    }

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        /* Keep looking while config mode wants a census. cam_scan_stop() sets
         * s_running false before tearing Bluetooth down, so a stop that is
         * really the shutdown is not restarted here. */
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

/* Stop scanning and free the Bluetooth controller entirely.
 *
 * This exists because, on this chip, an ENABLED Bluetooth controller starves
 * the Wi-Fi access point's beacon even when it is not scanning -- so config
 * mode cannot leave it running behind the settings page. The captured camera
 * list (s_tbl) is deliberately kept: the picker still shows what the census
 * found, and binding validates against it. A reboot back into config mode
 * takes a fresh census. */
void cam_scan_stop(void)
{
    if (!s_running) return;
    s_running = false;

    if (s_watchdog) {
        esp_timer_stop(s_watchdog);
        esp_timer_delete(s_watchdog);
        s_watchdog = NULL;
    }
    esp_ble_gap_stop_scanning();
    vTaskDelay(pdMS_TO_TICKS(60));   /* let STOP_COMPLETE land before teardown */

    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
    ESP_LOGI(TAG, "camera census done, Bluetooth off -- Wi-Fi has the radio");
}

int cam_scan_get(cam_scan_entry_t *out, int max)
{
    if (out == NULL || max <= 0 || s_lock == NULL) return 0;
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(50)) != pdTRUE) return 0;

    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);

    /* Drop anything gone quiet, compacting in place -- but only while actually
     * scanning. Once cam_scan_stop() has powered Bluetooth down, no advert can
     * refresh an entry, so aging here would empty the picker a few seconds
     * after setup opened. A stopped scan is a frozen census: show all of it. */
    if (s_running) {
        int keep = 0;
        for (int i = 0; i < s_count; i++) {
            if ((now - s_tbl[i].last_ms) <= FORGET_MS) {
                if (keep != i) s_tbl[keep] = s_tbl[i];
                keep++;
            }
        }
        s_count = keep;
    }

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
