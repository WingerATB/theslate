/* SPDX-License-Identifier: PolyForm-Strict-1.0.0
 *
 * GoPro BLE session. See gopro_cam.h.
 *
 * Shape of the protocol (Open GoPro, BLE):
 *
 *   service 0xFEA6, six characteristics under the base UUID
 *   b5f9XXXX-aa8d-11e3-9046-0002a5d5c51b:
 *     0072 command       (write)     0073 command response (notify)
 *     0074 setting       (write)     0075 setting response (notify)
 *     0076 query         (write)     0077 query response   (notify)
 *
 *   Every message is packetised into <= 20-byte BLE writes. The first byte is
 *   a header: bit 7 set is a continuation packet; otherwise bits 6..5 say how
 *   the length is encoded (00: five bits in this byte; 01: thirteen bits over
 *   two; 10: sixteen bits over the next two). The bytes in the Open GoPro
 *   tables include that header, so "03 01 01 01" is sent exactly as written.
 *
 *   The camera must be BONDED. The first time, it has to be in pairing mode
 *   (Connections > Connect Device > Quik App); after that the bond is stored
 *   on both sides. It goes to sleep on its own unless a keep-alive arrives
 *   every few seconds, and while asleep it keeps advertising for hours, so a
 *   connection request is also how it is woken.
 */
#include "gopro_cam.h"

#include <string.h>
#include <stdio.h>
#include <ctype.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_defs.h"

#include "ble.h"   /* camlink_ble_apply_tx_level / camlink_ble_get_tx_level */

static const char *TAG = "GOPRO";

#define GP_APP_ID        0x47
#define GP_SVC_UUID16    0xFEA6

/* Characteristic slots. Order matters: the three responses are subscribed in
 * sequence, and the three writes are addressed by gopro_ch_t. */
enum { CH_CMD = 0, CH_SET, CH_QRY, CH_CMD_RSP, CH_SET_RSP, CH_QRY_RSP, CH_COUNT };
static const uint16_t s_ch_short[CH_COUNT] = { 0x0072, 0x0074, 0x0076, 0x0073, 0x0075, 0x0077 };
static const char    *s_ch_name[CH_COUNT]  = { "cmd", "set", "qry", "cmd_rsp", "set_rsp", "qry_rsp" };

/* Command / setting / query IDs used here. */
#define CMD_SET_SHUTTER   0x01
#define CMD_SLEEP         0x05
#define CMD_HW_INFO       0x3C
#define SET_KEEP_ALIVE    0x5B
#define QRY_REG_STATUS    0x53
#define QRY_STATUS_PUSH   0x93
#define QRY_REG_SETTING   0x52
#define QRY_SETTING_PUSH  0x92
#define SET_RES           2
#define SET_FPS           3

/* Status IDs we read. */
#define ST_HOT        6
#define ST_BUSY       8
#define ST_ENCODING   10
#define ST_VIDEO_S    13
#define ST_SD_STATUS  33
#define ST_LEFT_S     35
#define ST_BATT_PCT   70

#define KEEPALIVE_MS      3000
#define RECONNECT_MS      3000
#define PAIR_FAIL_WAIT_MS 10000
#define OPEN_TIMEOUT_MS   15000
#define MSG_MAX           600

/* ---- state -------------------------------------------------------------- */

static SemaphoreHandle_t s_lock;
static gopro_status_t    s_st;
static uint32_t          s_last_status_ms;

static uint8_t   s_bda[6];
static uint8_t   s_addr_type;
static esp_gatt_if_t s_gattc_if = ESP_GATT_IF_NONE;
static uint16_t  s_conn_id;
static uint16_t  s_svc_start, s_svc_end;
static uint16_t  s_handle[CH_COUNT];

static volatile bool s_connected;     /* GATT open succeeded                   */
static volatile bool s_encrypted;     /* bonding / encryption complete         */
static volatile bool s_ready;         /* all three notifications subscribed    */
static volatile bool s_opening;
static uint32_t      s_open_started_ms;
static uint32_t      s_next_connect_ms;
static int           s_subscribe_idx;  /* which response char is being enabled */
static bool          s_want_connect = true;

static bool s_rec_pending;
static bool s_rec_value;
static bool s_sleep_pending;
static bool s_hilight_pending;
static bool s_reg_all_fallback;

static uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000); }

/* ---- UUIDs -------------------------------------------------------------- */

static esp_bt_uuid_t gp_uuid(uint16_t x)
{
    esp_bt_uuid_t u = { .len = ESP_UUID_LEN_128 };
    const uint8_t base[16] = { 0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x46, 0x90,
                               0xe3, 0x11, 0x8d, 0xaa, 0x00, 0x00, 0xf9, 0xb5 };
    memcpy(u.uuid.uuid128, base, 16);
    u.uuid.uuid128[12] = (uint8_t)(x & 0xFF);
    u.uuid.uuid128[13] = (uint8_t)(x >> 8);
    return u;
}

/* ---- packet reassembly -------------------------------------------------- */

typedef struct {
    uint8_t  buf[MSG_MAX];
    size_t   expect;
    size_t   got;
    bool     active;
} reasm_t;

static reasm_t s_rx[3];   /* indexed by CH_*_RSP - CH_CMD_RSP */

/* Returns true when a whole message is in r->buf (length r->expect). */
static bool reasm_feed(reasm_t *r, const uint8_t *p, size_t n)
{
    if (n == 0) return false;
    uint8_t h = p[0];
    if (h & 0x80) {
        if (!r->active) return false;          /* continuation with no start */
        p++; n--;
    } else {
        size_t len, skip;
        switch ((h >> 5) & 0x03) {
        case 0:  len = h & 0x1F; skip = 1; break;
        case 1:  if (n < 2) return false; len = ((size_t)(h & 0x1F) << 8) | p[1]; skip = 2; break;
        case 2:  if (n < 3) return false; len = ((size_t)p[1] << 8) | p[2]; skip = 3; break;
        default: return false;
        }
        r->active = true; r->expect = len; r->got = 0;
        p += skip; n -= skip;
    }
    size_t room = (r->expect > r->got) ? r->expect - r->got : 0;
    if (n > room) n = room;
    if (r->got + n > sizeof(r->buf)) n = sizeof(r->buf) - r->got;
    memcpy(r->buf + r->got, p, n);
    r->got += n;
    if (r->got >= r->expect) { r->active = false; return true; }
    return false;
}

static void hexdump(const char *what, const uint8_t *p, size_t n)
{
    char hex[3 * 48 + 4];
    size_t w = 0;
    for (size_t i = 0; i < n && i < 48; i++) w += (size_t)snprintf(hex + w, sizeof(hex) - w, "%02X ", p[i]);
    if (n > 48) snprintf(hex + w, sizeof(hex) - w, "..");
    ESP_LOGI(TAG, "%s (%u) [%s]", what, (unsigned)n, hex);
}

/* ---- writes ------------------------------------------------------------- */

static bool write_ch(int ch, const uint8_t *bytes, size_t n)
{
    if (!s_connected || s_handle[ch] == 0) return false;
    esp_err_t e = esp_ble_gattc_write_char(s_gattc_if, s_conn_id, s_handle[ch],
                                           (uint16_t)n, (uint8_t *)bytes,
                                           ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
    if (e != ESP_OK) ESP_LOGW(TAG, "write %s failed: %s", s_ch_name[ch], esp_err_to_name(e));
    return e == ESP_OK;
}

static void send_keepalive(void)
{
    static const uint8_t ka[] = { 0x03, SET_KEEP_ALIVE, 0x01, 0x42 };
    write_ch(CH_SET, ka, sizeof(ka));
}

static void send_shutter(bool on)
{
    const uint8_t sh[] = { 0x03, CMD_SET_SHUTTER, 0x01, on ? 0x01 : 0x00 };
    ESP_LOGI(TAG, "shutter %s", on ? "ON" : "OFF");
    write_ch(CH_CMD, sh, sizeof(sh));
}

static void send_hw_info(void)
{
    static const uint8_t q[] = { 0x01, CMD_HW_INFO };
    write_ch(CH_CMD, q, sizeof(q));
}

static void send_register_status(bool all)
{
    if (all) {
        static const uint8_t q[] = { 0x01, QRY_REG_STATUS };
        ESP_LOGW(TAG, "registering for ALL status pushes");
        write_ch(CH_QRY, q, sizeof(q));
    } else {
        static const uint8_t q[] = { 0x08, QRY_REG_STATUS,
                                     ST_ENCODING, ST_VIDEO_S, ST_LEFT_S, ST_BATT_PCT,
                                     ST_HOT, ST_BUSY, ST_SD_STATUS };
        write_ch(CH_QRY, q, sizeof(q));
    }
}

/* ---- parsing ------------------------------------------------------------ */

static uint32_t be_val(const uint8_t *p, uint8_t len)
{
    uint32_t v = 0;
    for (uint8_t i = 0; i < len && i < 4; i++) v = (v << 8) | p[i];
    return v;
}

static void label_from_model(const char *model, char out[6])
{
    /* "HERO13 Black" -> "H13", "HERO" -> "HERO", "MAX 2" -> "MAX2". Four
     * characters is what the OSD field has; keep the digits, they are the
     * part that tells two GoPros apart. */
    char letters[8] = {0}, digits[8] = {0};
    size_t l = 0, d = 0;
    for (const char *p = model; *p && (l < 7 || d < 7); p++) {
        if (isdigit((unsigned char)*p)) { if (d < 7) digits[d++] = *p; }
        else if (isalpha((unsigned char)*p) && l < 7 && d == 0) letters[l++] = (char)toupper((unsigned char)*p);
        else if (*p == ' ' && d) break;
    }
    if (d == 0) { strlcpy(out, letters[0] ? letters : "GPRO", 5); return; }
    size_t keep = 4 - (d > 3 ? 3 : d);
    if (keep > l) keep = l;
    memcpy(out, letters, keep);
    memcpy(out + keep, digits, d > 3 ? 3 : d);
    out[keep + (d > 3 ? 3 : d)] = '\0';
}

/* Enum -> label for the video resolution and framerate settings. These enum
 * numbers are the long-standing Open GoPro values, stable across HERO5 and up
 * for the common modes; an unrecognised value is shown as its raw number rather
 * than guessed at, since the far ends of the table are model-specific. */
static const char *res_label(uint32_t v)
{
    switch (v) {
    case 1: case 2: case 18: case 28: case 103: case 108: return "4K";
    case 4: case 5: case 6: case 111:                     return "2.7K";
    case 7:                                               return "1440";
    case 8: case 9:                                       return "1080";
    case 10:                                              return "960";
    case 11: case 12:                                     return "720";
    case 13:                                              return "480";
    case 24: case 25:                                     return "5K";
    case 26: case 27: case 100: case 102: case 107:       return "5.3K";
    default:                                              return NULL;
    }
}
static const char *fps_label(uint32_t v)
{
    switch (v) {
    case 0:  return "240"; case 1: return "120"; case 2: return "100";
    case 3:  return "90";  case 4: return "80";  case 5: return "60";
    case 6:  return "50";  case 7: return "48";  case 8: return "30";
    case 9:  return "25";  case 10: return "24"; case 13: return "200";
    default: return NULL;
    }
}

static void send_register_settings(void)
{
    /* Register for ALL settings (empty id list), not just resolution and
     * framerate. Simpler cameras -- the HERO (2024) among them -- do not answer
     * a request for specific setting ids, but do answer the register-all form;
     * its response carries every current value and later pushes only the ones
     * that change. We keep only resolution (2) and framerate (3). */
    static const uint8_t q[] = { 0x01, QRY_REG_SETTING };
    write_ch(CH_QRY, q, sizeof(q));
}

static void on_setting_values(const uint8_t *p, size_t n)
{
    size_t i = 0;
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(20)) != pdTRUE) return;
    while (i + 2 <= n) {
        uint8_t id = p[i], len = p[i + 1];
        if (i + 2 + len > n) break;
        uint32_t val = be_val(p + i + 2, len);
#if CONFIG_CAMLINK_GPSETTINGS_DUMP
        ESP_LOGW(TAG, "SETTING %u len %u = %u", id, len, (unsigned)val);
#endif
        if (id == SET_RES) {
            const char *l = res_label(val);
            if (l) strlcpy(s_st.res, l, sizeof(s_st.res));
            else   snprintf(s_st.res, sizeof(s_st.res), "%u", (unsigned)val);
        } else if (id == SET_FPS) {
            const char *l = fps_label(val);
            if (l) strlcpy(s_st.fps, l, sizeof(s_st.fps));
            else   snprintf(s_st.fps, sizeof(s_st.fps), "%u", (unsigned)val);
        }
        i += 2 + len;
    }
    xSemaphoreGive(s_lock);
}

static void on_status_values(const uint8_t *p, size_t n, bool push)
{
    /* [id][len][value]... */
    size_t i = 0;
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(20)) != pdTRUE) return;
    while (i + 2 <= n) {
        uint8_t id = p[i], len = p[i + 1];
        if (i + 2 + len > n) break;
        const uint8_t *v = p + i + 2;
        uint32_t val = be_val(v, len);
        switch (id) {
        case ST_ENCODING:  s_st.encoding = val != 0; break;
        case ST_VIDEO_S:   s_st.clip_s = val; break;
        case ST_LEFT_S:    s_st.left_s = val; break;
        case ST_BATT_PCT:  s_st.battery_pct = (uint8_t)(val > 100 ? 100 : val); s_st.battery_valid = true; break;
        case ST_HOT:       s_st.hot = (uint8_t)val; break;
        case ST_BUSY:      s_st.busy = val != 0; break;
        case ST_SD_STATUS: s_st.sd_ok = (val == 0); break;
        default:
            ESP_LOGD(TAG, "status %u len %u = %u", id, len, (unsigned)val);
            break;
        }
        if (push && id != ST_VIDEO_S) {
            ESP_LOGI(TAG, "status %u = %u", id, (unsigned)val);
        }
        i += 2 + len;
    }
    s_last_status_ms = now_ms();
    s_st.status_valid = true;
    xSemaphoreGive(s_lock);
}

static void on_message(int rsp_ch, const uint8_t *m, size_t n)
{
    if (n < 2) { hexdump("short message", m, n); return; }
    uint8_t id = m[0], status = m[1];

    if (rsp_ch == CH_CMD_RSP) {
        if (id == CMD_HW_INFO && status == 0) {
            /* [len][model number][len][model name][len][board type][len][fw]... */
            size_t i = 2;
            if (i < n) { uint8_t l = m[i]; i += 1 + l; }          /* model number */
            if (i < n) {
                uint8_t l = m[i++];
                if (i + l <= n) {
                    size_t c = l < sizeof(s_st.model) - 1 ? l : sizeof(s_st.model) - 1;
                    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(20)) == pdTRUE) {
                        memcpy(s_st.model, m + i, c); s_st.model[c] = '\0';
                        label_from_model(s_st.model, s_st.label);
                        xSemaphoreGive(s_lock);
                    }
                    ESP_LOGI(TAG, "camera model \"%s\" -> label %s", s_st.model, s_st.label);
                }
                i += l;
            }
            /* Firmware version is worth one log line for a camera nobody has
             * seen this firmware talk to before. */
            if (i < n) { uint8_t l = m[i]; i += 1 + l; }          /* board type */
            if (i < n) {
                uint8_t l = m[i++];
                if (i + l <= n) ESP_LOGI(TAG, "camera firmware %.*s", (int)l, (const char *)m + i);
            }
            return;
        }
        ESP_LOGI(TAG, "command 0x%02X -> status %u", id, status);
        if (status != 0) hexdump("command response", m, n);
        return;
    }
    if (rsp_ch == CH_SET_RSP) {
        if (id == SET_KEEP_ALIVE && status == 0) return;     /* every 3 s; silence */
        ESP_LOGI(TAG, "setting 0x%02X -> status %u", id, status);
        return;
    }
    /* query response */
    if (id == QRY_REG_STATUS) {
        if (status != 0) {
            ESP_LOGW(TAG, "status registration refused (status %u)", status);
            if (!s_reg_all_fallback) { s_reg_all_fallback = true; send_register_status(true); }
            return;
        }
        ESP_LOGI(TAG, "status registration accepted, %u bytes of current values", (unsigned)(n - 2));
        on_status_values(m + 2, n - 2, false);
        return;
    }
    if (id == QRY_STATUS_PUSH) { on_status_values(m + 2, n - 2, true); return; }
    if (id == QRY_REG_SETTING || id == QRY_SETTING_PUSH) { on_setting_values(m + 2, n - 2); return; }
    ESP_LOGI(TAG, "query 0x%02X -> status %u", id, status);
    hexdump("query response", m, n);
}

/* ---- link state --------------------------------------------------------- */

static void link_down(const char *why)
{
    s_connected = false; s_encrypted = false; s_ready = false; s_opening = false;
    memset(s_rx, 0, sizeof(s_rx));
    memset(s_handle, 0, sizeof(s_handle));
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(20)) == pdTRUE) {
        s_st.link_up = false;
        s_st.status_valid = false;
        s_st.encoding = false;
        s_st.battery_valid = false;
        xSemaphoreGive(s_lock);
    }
    ESP_LOGW(TAG, "link down: %s", why);
}

static void subscribe_next(void)
{
    /* Enable notifications on the three response characteristics one at a
     * time: register with the stack, then write the CCCD. Bluedroid has to be
     * driven a step per event. */
    if (s_subscribe_idx >= 3) {
        s_ready = true;
        if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(20)) == pdTRUE) {
            s_st.link_up = true;
            xSemaphoreGive(s_lock);
        }
        ESP_LOGI(TAG, "session up");
        send_keepalive();
        send_hw_info();
        send_register_status(false);
        send_register_settings();
        return;
    }
    int ch = CH_CMD_RSP + s_subscribe_idx;
    esp_err_t e = esp_ble_gattc_register_for_notify(s_gattc_if, s_bda, s_handle[ch]);
    if (e != ESP_OK) ESP_LOGW(TAG, "register_for_notify %s: %s", s_ch_name[ch], esp_err_to_name(e));
}

static void on_search_complete(void)
{
    if (s_svc_start == 0) {
        ESP_LOGE(TAG, "service 0xFEA6 not found -- is this a GoPro?");
        esp_ble_gap_disconnect(s_bda);
        return;
    }
    for (int i = 0; i < CH_COUNT; i++) {
        esp_gattc_char_elem_t el;
        uint16_t count = 1;
        esp_bt_uuid_t u = gp_uuid(s_ch_short[i]);
        esp_gatt_status_t st = esp_ble_gattc_get_char_by_uuid(s_gattc_if, s_conn_id,
                                                              s_svc_start, s_svc_end, u, &el, &count);
        if (st != ESP_GATT_OK || count == 0) {
            ESP_LOGE(TAG, "characteristic %s (GP-%04X) missing", s_ch_name[i], s_ch_short[i]);
            esp_ble_gap_disconnect(s_bda);
            return;
        }
        s_handle[i] = el.char_handle;
        ESP_LOGD(TAG, "%s handle 0x%04X props 0x%02X", s_ch_name[i], el.char_handle, el.properties);
    }
    s_subscribe_idx = 0;
    subscribe_next();
}

static void gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *p)
{
    switch (event) {
    case ESP_GATTC_REG_EVT:
        if (p->reg.status == ESP_GATT_OK) s_gattc_if = gattc_if;
        else ESP_LOGE(TAG, "gattc app register failed %d", p->reg.status);
        break;

    case ESP_GATTC_CONNECT_EVT:
        s_conn_id = p->connect.conn_id;
        ESP_LOGI(TAG, "connected (conn_id %u), requesting encryption", s_conn_id);
        /* Bonded already: this just turns encryption on. Not bonded: this
         * starts pairing, which the camera only accepts in pairing mode. */
        esp_ble_set_encryption(p->connect.remote_bda, ESP_BLE_SEC_ENCRYPT_NO_MITM);
        break;

    case ESP_GATTC_OPEN_EVT:
        s_opening = false;
        if (p->open.status != ESP_GATT_OK) {
            ESP_LOGW(TAG, "open failed: 0x%02X", p->open.status);
            s_next_connect_ms = now_ms() + RECONNECT_MS;
            break;
        }
        s_connected = true;
        s_conn_id = p->open.conn_id;
        break;

    case ESP_GATTC_SEARCH_RES_EVT:
        if (p->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16 &&
            p->search_res.srvc_id.uuid.uuid.uuid16 == GP_SVC_UUID16) {
            s_svc_start = p->search_res.start_handle;
            s_svc_end   = p->search_res.end_handle;
        }
        break;

    case ESP_GATTC_SEARCH_CMPL_EVT:
        on_search_complete();
        break;

    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        if (p->reg_for_notify.status != ESP_GATT_OK) {
            ESP_LOGW(TAG, "notify registration failed 0x%02X", p->reg_for_notify.status);
            esp_ble_gap_disconnect(s_bda);
            break;
        }
        esp_gattc_descr_elem_t d;
        uint16_t count = 1;
        esp_bt_uuid_t cccd = { .len = ESP_UUID_LEN_16, .uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG };
        esp_gatt_status_t st = esp_ble_gattc_get_descr_by_char_handle(s_gattc_if, s_conn_id,
                                                                      p->reg_for_notify.handle,
                                                                      cccd, &d, &count);
        if (st != ESP_GATT_OK || count == 0) {
            ESP_LOGW(TAG, "no CCCD for handle 0x%04X", p->reg_for_notify.handle);
            esp_ble_gap_disconnect(s_bda);
            break;
        }
        uint8_t en[2] = { 0x01, 0x00 };
        esp_ble_gattc_write_char_descr(s_gattc_if, s_conn_id, d.handle, sizeof(en), en,
                                       ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
        break;
    }

    case ESP_GATTC_WRITE_DESCR_EVT:
        if (p->write.status != ESP_GATT_OK) {
            ESP_LOGW(TAG, "CCCD write failed 0x%02X (encrypted=%d)", p->write.status, s_encrypted);
            esp_ble_gap_disconnect(s_bda);
            break;
        }
        s_subscribe_idx++;
        subscribe_next();
        break;

    case ESP_GATTC_WRITE_CHAR_EVT:
        if (p->write.status != ESP_GATT_OK) {
            ESP_LOGW(TAG, "write to handle 0x%04X failed 0x%02X", p->write.handle, p->write.status);
        }
        break;

    case ESP_GATTC_NOTIFY_EVT: {
        int ch = -1;
        for (int i = CH_CMD_RSP; i < CH_COUNT; i++) if (s_handle[i] == p->notify.handle) { ch = i; break; }
        if (ch < 0) { hexdump("notify from unknown handle", p->notify.value, p->notify.value_len); break; }
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, p->notify.value, p->notify.value_len, ESP_LOG_DEBUG);
        reasm_t *r = &s_rx[ch - CH_CMD_RSP];
        if (reasm_feed(r, p->notify.value, p->notify.value_len)) {
            on_message(ch, r->buf, r->expect);
        }
        break;
    }

    case ESP_GATTC_DISCONNECT_EVT: {
        char why[40];
        snprintf(why, sizeof(why), "disconnected, reason 0x%02X", p->disconnect.reason);
        link_down(why);
        s_next_connect_ms = now_ms() + RECONNECT_MS;
        break;
    }

    default:
        break;
    }
}

static void gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *p)
{
    switch (event) {
    case ESP_GAP_BLE_SEC_REQ_EVT:
        esp_ble_gap_security_rsp(p->ble_security.ble_req.bd_addr, true);
        break;

    case ESP_GAP_BLE_NC_REQ_EVT:
        /* Numeric comparison with nothing to compare on: accept. */
        esp_ble_confirm_reply(p->ble_security.ble_req.bd_addr, true);
        break;

    case ESP_GAP_BLE_AUTH_CMPL_EVT:
        if (p->ble_security.auth_cmpl.success) {
            s_encrypted = true;
            ESP_LOGI(TAG, "%s, link encrypted", p->ble_security.auth_cmpl.auth_mode & ESP_LE_AUTH_BOND
                     ? "bonded" : "paired (no bond)");
            s_svc_start = s_svc_end = 0;
            esp_bt_uuid_t svc = { .len = ESP_UUID_LEN_16, .uuid.uuid16 = GP_SVC_UUID16 };
            esp_ble_gattc_search_service(s_gattc_if, s_conn_id, &svc);
        } else {
            ESP_LOGE(TAG, "pairing failed, reason 0x%02X -- put the camera in pairing mode "
                          "(Connections > Connect Device > Quik App) and it will retry",
                     p->ble_security.auth_cmpl.fail_reason);
            esp_ble_gap_disconnect(p->ble_security.auth_cmpl.bd_addr);
            s_next_connect_ms = now_ms() + PAIR_FAIL_WAIT_MS;
        }
        break;

    default:
        break;
    }
}

/* ---- session task ------------------------------------------------------- */

static void connect_now(void)
{
    ESP_LOGI(TAG, "connecting to %02X:%02X:%02X:%02X:%02X:%02X (%s address)",
             s_bda[0], s_bda[1], s_bda[2], s_bda[3], s_bda[4], s_bda[5],
             s_addr_type == BLE_ADDR_TYPE_RANDOM ? "random" : "public");
    s_opening = true;
    s_open_started_ms = now_ms();
    esp_err_t e = esp_ble_gattc_open(s_gattc_if, s_bda, (esp_ble_addr_type_t)s_addr_type, true);
    if (e != ESP_OK) {
        ESP_LOGW(TAG, "open: %s", esp_err_to_name(e));
        s_opening = false;
        s_next_connect_ms = now_ms() + RECONNECT_MS;
    }
}

static bool ble_bringup(void)
{
    esp_err_t e;
    esp_bt_controller_config_t cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((e = esp_bt_controller_init(&cfg)) != ESP_OK)              { ESP_LOGE(TAG, "ctrl init %s", esp_err_to_name(e)); return false; }
    if ((e = esp_bt_controller_enable(ESP_BT_MODE_BLE)) != ESP_OK) { ESP_LOGE(TAG, "ctrl enable %s", esp_err_to_name(e)); return false; }
    if ((e = esp_bluedroid_init()) != ESP_OK)                      { ESP_LOGE(TAG, "bluedroid init %s", esp_err_to_name(e)); return false; }
    if ((e = esp_bluedroid_enable()) != ESP_OK)                    { ESP_LOGE(TAG, "bluedroid enable %s", esp_err_to_name(e)); return false; }
    if ((e = esp_ble_gap_register_callback(gap_cb)) != ESP_OK)     return false;
    if ((e = esp_ble_gattc_register_callback(gattc_cb)) != ESP_OK) return false;
    if ((e = esp_ble_gattc_app_register(GP_APP_ID)) != ESP_OK)     return false;

    /* Same posture as the DJI path: Just Works, bonded, encrypted. */
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_BOND;
    esp_ble_io_cap_t   iocap    = ESP_IO_CAP_NONE;
    uint8_t key_size = 16;
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key  = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(auth_req));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE,      &iocap,    sizeof(iocap));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE,    &key_size, sizeof(key_size));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY,    &init_key, sizeof(init_key));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY,     &rsp_key,  sizeof(rsp_key));

    camlink_ble_apply_tx_level(camlink_ble_get_tx_level());

    ESP_LOGI(TAG, "BLE up, %d stored bond(s)", esp_ble_get_bond_device_num());
    return true;
}

static void cam_task(void *arg)
{
    (void)arg;
    if (!ble_bringup()) { vTaskDelete(NULL); return; }
    for (int i = 0; i < 50 && s_gattc_if == ESP_GATT_IF_NONE; i++) vTaskDelay(pdMS_TO_TICKS(20));

    uint32_t last_ka = 0;
    for (;;) {
        uint32_t t = now_ms();

        if (!s_connected) {
            if (s_opening && (t - s_open_started_ms) > OPEN_TIMEOUT_MS) {
                ESP_LOGW(TAG, "no answer to connection request -- camera off, out of range, or wrong address type");
                esp_ble_gap_disconnect(s_bda);      /* cancels the pending open */
                s_opening = false;
                s_next_connect_ms = t + RECONNECT_MS;
            } else if (!s_opening && s_want_connect && (int32_t)(t - s_next_connect_ms) >= 0) {
                connect_now();
            }
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (s_ready) {
            if ((t - last_ka) >= KEEPALIVE_MS) { last_ka = t; send_keepalive(); }
            if (s_rec_pending)   { s_rec_pending = false; send_shutter(s_rec_value); }
            if (s_hilight_pending) {
                s_hilight_pending = false;
                static const uint8_t hl[] = { 0x01, 0x18 };   /* Tag HiLight */
                ESP_LOGI(TAG, "HiLight tag");
                write_ch(CH_CMD, hl, sizeof(hl));
            }
            if (s_sleep_pending) {
                s_sleep_pending = false;
                static const uint8_t z[] = { 0x01, CMD_SLEEP };
                ESP_LOGW(TAG, "sending sleep");
                write_ch(CH_CMD, z, sizeof(z));
                s_want_connect = false;   /* stay off it until asked to wake */
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* ---- API ---------------------------------------------------------------- */

void gopro_cam_start(const uint8_t addr[6], uint8_t addr_type)
{
    s_lock = xSemaphoreCreateMutex();
    memset(&s_st, 0, sizeof(s_st));
    strlcpy(s_st.label, "GPRO", sizeof(s_st.label));
    memcpy(s_bda, addr, 6);
    s_addr_type = addr_type;
    xTaskCreate(cam_task, "goprocam", 6144, NULL, 4, NULL);
}

void gopro_cam_get(gopro_status_t *out)
{
    if (out == NULL) return;
    if (s_lock == NULL || xSemaphoreTake(s_lock, pdMS_TO_TICKS(20)) != pdTRUE) {
        memset(out, 0, sizeof(*out));
        return;
    }
    *out = s_st;
    out->status_age_ms = s_last_status_ms ? (now_ms() - s_last_status_ms) : UINT32_MAX;
    xSemaphoreGive(s_lock);
}

void gopro_cam_request_record(bool on) { s_rec_value = on; s_rec_pending = true; }
void gopro_cam_sleep(void)             { s_sleep_pending = true; }
void gopro_cam_hilight(void)           { s_hilight_pending = true; }
void gopro_cam_wake(void)              { s_want_connect = true; s_next_connect_ms = 0; }

void gopro_cam_disconnect(void)
{
    s_want_connect = false;
    if (s_connected) {
        esp_ble_gap_disconnect(s_bda);
        for (int i = 0; i < 20 && s_connected; i++) vTaskDelay(pdMS_TO_TICKS(50));
    }
}

bool gopro_cam_write(gopro_ch_t ch, const uint8_t *bytes, size_t n)
{
    int c = ch == GOPRO_CH_CMD ? CH_CMD : ch == GOPRO_CH_SET ? CH_SET : CH_QRY;
    return write_ch(c, bytes, n);
}
