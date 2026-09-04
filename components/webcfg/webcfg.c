/* SPDX-License-Identifier: PolyForm-Strict-1.0.0 */
#include "webcfg.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_coexist.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "esp_app_desc.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_image_format.h"
#include "esp_http_server.h"
#include "esp_task_wdt.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "camlink_cfg.h"
#include "osd_fields.h"
#include "url_decode.h"
#include "duml_cam.h"
#include "cam_scan.h"
#include "ble.h"
#include "msp.h"

static const char *TAG = "WEBCFG";

/* The NVS namespace stays "camlink" for good -- it happens to match the
 * project name again, but that is beside the point. It is the key under which
 * every module stores its bound camera and settings, and renaming it would
 * silently discard both on the update that did it. */
#define NVS_NS       "camlink"
#define NVS_KEY_FLAG "cfgmode"
#define NVS_KEY_BIND "bindmode"
#define NVS_KEY_USB  "usbcfgmode"
#define NVS_KEY_NORMAL "bootnormal"

#define AP_CHANNEL       1
#define AP_MAX_CONN      2
#define AP_IP            "192.168.4.1"

/* The page, gzipped at build time and linked in. Serving it from flash means
 * one binary and one flash step -- no data partition to write separately, and
 * no way for the firmware and the page it serves to end up out of sync. */
extern const uint8_t page_start[] asm("_binary_index_html_gz_start");
extern const uint8_t page_end[]   asm("_binary_index_html_gz_end");

static char        s_ssid[20];
static httpd_handle_t s_httpd;

/* ==========================================================================
 * One-shot boot flag
 * ========================================================================== */
bool webcfg_boot_flag_take(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return false;

    uint8_t v = 0;
    bool set = (nvs_get_u8(h, NVS_KEY_FLAG, &v) == ESP_OK && v != 0);
    if (set) {
        /* Consume it before doing anything else. If config mode itself is what
         * crashes the module, the next boot must come up normally rather than
         * re-entering the thing that just failed. */
        nvs_erase_key(h, NVS_KEY_FLAG);
        nvs_commit(h);
    }
    nvs_close(h);
    return set;
}

void webcfg_reboot_into_config(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, NVS_KEY_FLAG, 1);
        nvs_commit(h);
        nvs_close(h);
    }
    ESP_LOGW(TAG, "rebooting into config mode");
    duml_cam_disconnect();            /* leave the camera advertising, not waiting */
    vTaskDelay(pdMS_TO_TICKS(120));   /* let the log line drain */
    esp_restart();
    for (;;) { }                      /* esp_restart() does not return */
}

bool webcfg_bind_flag_take(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return false;
    uint8_t v = 0;
    bool set = (nvs_get_u8(h, NVS_KEY_BIND, &v) == ESP_OK && v != 0);
    if (set) {
        nvs_erase_key(h, NVS_KEY_BIND);
        nvs_commit(h);
    }
    nvs_close(h);
    return set;
}

bool webcfg_bootnormal_flag_take(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return false;
    uint8_t v = 0;
    bool set = (nvs_get_u8(h, NVS_KEY_NORMAL, &v) == ESP_OK && v != 0);
    if (set) { nvs_erase_key(h, NVS_KEY_NORMAL); nvs_commit(h); }
    nvs_close(h);
    return set;
}

void webcfg_reboot_into_normal(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, NVS_KEY_NORMAL, 1);
        nvs_commit(h);
        nvs_close(h);
    }
    ESP_LOGW(TAG, "rebooting into NORMAL mode for one boot (test)");
    duml_cam_disconnect();
    vTaskDelay(pdMS_TO_TICKS(120));
    esp_restart();
    for (;;) { }
}

bool webcfg_usbcfg_flag_take(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return false;
    uint8_t v = 0;
    bool set = (nvs_get_u8(h, NVS_KEY_USB, &v) == ESP_OK && v != 0);
    if (set) { nvs_erase_key(h, NVS_KEY_USB); nvs_commit(h); }
    nvs_close(h);
    return set;
}

void webcfg_reboot_into_usbcfg(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, NVS_KEY_USB, 1);
        nvs_commit(h);
        nvs_close(h);
    }
    ESP_LOGW(TAG, "rebooting into USB-config mode");
    duml_cam_disconnect();
    vTaskDelay(pdMS_TO_TICKS(120));
    esp_restart();
    for (;;) { }
}

void webcfg_reboot_into_bind(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, NVS_KEY_BIND, 1);
        nvs_commit(h);
        nvs_close(h);
    }
    ESP_LOGW(TAG, "rebooting into bind mode");
    duml_cam_disconnect();
    vTaskDelay(pdMS_TO_TICKS(120));
    esp_restart();
    for (;;) { }
}

const char *webcfg_ssid(void) { return s_ssid; }

/* ==========================================================================
 * Captive-portal DNS
 *
 * Answers every A query with our own address, which is what makes a phone pop
 * the settings page by itself on joining. Without it the user has to know to
 * type an IP address, and "type 192.168.4.1" is exactly the kind of step this
 * whole feature exists to delete.
 * ========================================================================== */
static void dns_task(void *arg)
{
    (void)arg;
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "DNS socket failed");
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in bind_addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(53),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
        ESP_LOGE(TAG, "DNS bind failed");
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    uint8_t buf[512];
    for (;;) {
        struct sockaddr_in from;
        socklen_t from_len = sizeof(from);
        int n = recvfrom(sock, buf, sizeof(buf), 0, (struct sockaddr *)&from, &from_len);

        /* 12-byte header, and at least one question. Anything smaller is not a
         * query we can answer; drop it rather than index into a short buffer. */
        if (n < 12 + 5) continue;

        uint16_t qdcount = (uint16_t)((buf[4] << 8) | buf[5]);
        if (qdcount != 1) continue;

        /* Walk the QNAME label chain to find where the question ends. Bounded
         * by n at every step: a malformed length byte must not run off the end. */
        int p = 12;
        while (p < n && buf[p] != 0) {
            if ((buf[p] & 0xC0) != 0) { p = n; break; }   /* no compression in a query */
            p += buf[p] + 1;
        }
        if (p >= n) continue;
        p += 1 + 4;                       /* terminator + QTYPE + QCLASS */
        if (p > n || p + 16 > (int)sizeof(buf)) continue;

        buf[2] = 0x81; buf[3] = 0x80;     /* response, recursion available     */
        buf[6] = 0x00; buf[7] = 0x01;     /* ANCOUNT = 1                       */
        buf[8] = 0x00; buf[9] = 0x00;     /* NSCOUNT = 0                       */
        buf[10] = 0x00; buf[11] = 0x00;   /* ARCOUNT = 0                       */

        uint8_t *a = &buf[p];
        *a++ = 0xC0; *a++ = 0x0C;                       /* name -> offset 12   */
        *a++ = 0x00; *a++ = 0x01;                       /* type A              */
        *a++ = 0x00; *a++ = 0x01;                       /* class IN            */
        *a++ = 0; *a++ = 0; *a++ = 0; *a++ = 60;        /* TTL 60 s            */
        *a++ = 0x00; *a++ = 0x04;                       /* RDLENGTH 4          */
        *a++ = 192; *a++ = 168; *a++ = 4; *a++ = 1;     /* AP_IP               */

        sendto(sock, buf, (size_t)(a - buf), 0, (struct sockaddr *)&from, from_len);
    }
}

/* ==========================================================================
 * HTTP
 * ========================================================================== */
static esp_err_t page_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, (const char *)page_start, page_end - page_start);
}

int webcfg_state_json(char *body, size_t cap)
{
    camlink_cfg_t cfg;
    camlink_cfg_get(&cfg);

    msp_state_t fc;
    msp_get_state(&fc);

    uint8_t mac[6];
    bool bound = duml_cam_bound_addr(mac);

    uint8_t  fault_reason = 0;
    uint32_t fault_count  = 0;
    bool     fault_ever   = false;
    camlink_ble_link_fault(&fault_reason, &fault_count, &fault_ever);

    const esp_app_desc_t *app = esp_app_get_description();

    int  n = snprintf(body, cap,
        "{\"ssid\":\"%s\",\"fw\":\"%s\","
        "\"cfg\":{\"mode\":%u,\"aux\":%d,\"min\":%u,\"max\":%u,\"tx\":%u,\"kind\":%u,\"txauto\":%u,\"cfgaux\":%d,\"cfgkind\":%u,\"cfghold\":%u},"
        "\"cam\":{\"bound\":%s,\"mac\":\"%02X:%02X:%02X:%02X:%02X:%02X\","
        "\"fault\":\"%s\",\"fails\":%u,\"everup\":%s},"
        "\"fc\":{\"link\":%s,\"armed\":%s,\"n\":%u,\"rc\":[",
        s_ssid, app ? app->version : "?",
        cfg.mode, cfg_index_to_aux(cfg.switch_channel),
        cfg.range_min, cfg.range_max, cfg.tx_power, cfg.switch_kind, cfg.tx_auto,
        cfg.cfg_channel ? cfg_index_to_aux(cfg.cfg_channel) : 0,
        cfg.cfg_kind, cfg.cfg_hold_ds,
        bound ? "true" : "false",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
        camlink_ble_fault_text(fault_reason), (unsigned)fault_count,
        fault_ever ? "true" : "false",
        fc.link_up ? "true" : "false",
        fc.armed   ? "true" : "false",
        fc.rc_count);

    /* Every channel the FC reports, not a convenient prefix of them. Truncating
     * this list once hid the user's own arm and record switches, which sat on
     * channels past the cut, and cost an evening of "the button isn't detected". */
    for (unsigned i = 0; i < fc.rc_count && i < MSP_MAX_RC_CHANNELS; i++) {
        n += snprintf(body + n, cap - n, "%s%u", i ? "," : "", fc.rc[i]);
        if (n >= (int)sizeof(body) - 8) break;
    }
    n += snprintf(body + n, cap - n, "]},");

    /* The OSD field catalogue, straight from the firmware.
     *
     * The page needs each field's worst-case width to tell the user whether a
     * row fits, and that number belongs to the renderer. Sending it rather than
     * hard-coding it in the page means the two cannot drift: add a field to
     * osd_fields[] and the picker grows a tile for it, with the right budget,
     * without the JavaScript being touched. */
    n += snprintf(body + n, cap - n, "\"osdmax\":%d,\"fields\":[", OSD_ROW_MAX);
    for (int f = 1; f < OSD_F__COUNT; f++) {
        n += snprintf(body + n, cap - n,
                      "%s{\"id\":%d,\"name\":\"%s\",\"ex\":\"%s\",\"w\":%u}",
                      f > 1 ? "," : "", f, osd_fields[f].name,
                      osd_fields[f].example, (unsigned)osd_fields[f].width);
    }
    n += snprintf(body + n, cap - n, "],\"osd\":[");
    for (int r = 0; r < OSD_ROWS; r++) {
        n += snprintf(body + n, cap - n, "%s[", r ? "," : "");
        for (int i = 0; i < OSD_ROW_FIELDS; i++) {
            n += snprintf(body + n, cap - n, "%s%u",
                          i ? "," : "", (unsigned)cfg.osd[r][i]);
        }
        n += snprintf(body + n, cap - n, "]");
    }
    n += snprintf(body + n, cap - n, "]}");

    return n;
}

static esp_err_t state_get(httpd_req_t *req)
{
    char body[2048];
    int n = webcfg_state_json(body, sizeof(body));
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, body, n);
}



/* Read a form-urlencoded body into buf. Returns false on overlong input.
 *
 * Form encoding rather than JSON on purpose: the values here are four small
 * integers, httpd_query_key_value() already parses this format, and it keeps a
 * JSON parser out of a binary that has to fit beside the BLE stack. */
static bool read_body(httpd_req_t *req, char *buf, size_t cap)
{
    size_t len = req->content_len;
    if (len == 0 || len >= cap) return false;
    size_t got = 0;
    while (got < len) {
        int r = httpd_req_recv(req, buf + got, len - got);
        if (r <= 0) return false;
        got += (size_t)r;
    }
    buf[got] = '\0';
    return true;
}

static bool field_u32(const char *body, const char *key, uint32_t *out)
{
    char v[16];
    if (httpd_query_key_value(body, key, v, sizeof(v)) != ESP_OK) return false;
    url_decode(v);
    char *end = NULL;
    unsigned long parsed = strtoul(v, &end, 10);
    if (end == v || *end != '\0') return false;
    *out = (uint32_t)parsed;
    return true;
}

const char *webcfg_apply_config(const char *body)
{

    /* Validate EVERYTHING before writing ANYTHING. A half-applied form -- new
     * mode saved, out-of-range channel rejected -- leaves the module in a state
     * the user never asked for and cannot see. Either the whole form lands or
     * none of it does. */
    uint32_t mode, aux, lo, hi, tx, kind, txauto, cfgaux, cfgkind, cfghold;
    if (!field_u32(body, "mode", &mode) || !field_u32(body, "aux", &aux) ||
        !field_u32(body, "min", &lo)    || !field_u32(body, "max", &hi)  ||
        !field_u32(body, "tx", &tx)     || !field_u32(body, "kind", &kind) ||
        !field_u32(body, "txauto", &txauto) || !field_u32(body, "cfgaux", &cfgaux) ||
        !field_u32(body, "cfgkind", &cfgkind) || !field_u32(body, "cfghold", &cfghold)) {
        return "missing field";
    }
    /* The same limits the config store enforces, by name rather than by two
     * copies of the digits. They disagreed once -- this handler accepted
     * 900-2100 while the store still rejected anything outside 1000-2000, so a
     * legitimate form came back 500 with the mode already written and the
     * window not. Sharing the constants is what stops that recurring. */
    /* Named checks: a vague "out of range" cost an afternoon of guessing which
     * of ten fields was the problem. Each says which one. */
    if (mode >= CFG_MODE__COUNT)                 return "mode out of range";
    if (tx >= CFG_TX__COUNT)                     return "radio power out of range";
    if (kind > CFG_SW_BUTTON)                    return "control type out of range";
    if (txauto > 1)                              return "turn-down-while-flying out of range";
    if (aux < 1 || aux > 14)                     return "AUX channel out of range";
    if (lo < CAMLINK_RC_US_MIN || hi > CAMLINK_RC_US_MAX)
                                                 return "record window out of the 900-2100 range";
    if (lo >= hi)                                return "record window: lower edge is not below the upper";
    if (cfgaux > 14)                             return "setup channel out of range";
    if (cfgkind > CFG_SW_BUTTON)                 return "setup control type out of range";
    if (cfghold > CAMLINK_CFG_HOLD_DS_MAX)       return "setup hold time out of range";
    /* Optional: older pages do not send it, so it stays 0 (off). */

    bool ok = camlink_cfg_set_mode((uint8_t)mode)
           && camlink_cfg_set_channel(cfg_aux_to_index((uint8_t)aux))
           && camlink_cfg_set_range((uint16_t)lo, (uint16_t)hi)
           && camlink_cfg_set_tx_power((uint8_t)tx)
           && camlink_cfg_set_switch_kind((uint8_t)kind)
           && camlink_cfg_set_tx_auto((uint8_t)txauto)
           /* 0 means no setup switch; anything else is an AUX number. */
           && camlink_cfg_set_config_channel(cfgaux ? cfg_aux_to_index((uint8_t)cfgaux) : 0)
           && camlink_cfg_set_config_kind((uint8_t)cfgkind)
           && camlink_cfg_set_config_hold((uint8_t)cfghold);

    if (!ok) {
        return "save failed";
    }
    return NULL;
}

static esp_err_t config_post(httpd_req_t *req)
{
    char body[192];
    if (!read_body(req, body, sizeof(body))) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad body");
        return ESP_FAIL;
    }
    const char *err = webcfg_apply_config(body);
    if (err) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, err); return ESP_FAIL; }
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

/* The layout arrives as one field: sixteen ids, comma separated, row-major.
 *
 * Kept separate from /api/config because the two are edited independently and
 * a failed layout must not be able to reject a radio-power change the user made
 * in the same visit. */
const char *webcfg_apply_osd(const char *body)
{
    char v[128];
    if (httpd_query_key_value(body, "osd", v, sizeof(v)) != ESP_OK) {
        return "missing osd";
    }
    url_decode(v);   /* the separators arrive as %2C */

    uint8_t osd[OSD_ROWS][OSD_ROW_FIELDS];
    const char *p = v;
    for (int i = 0; i < OSD_ROWS * OSD_ROW_FIELDS; i++) {
        char *end = NULL;
        unsigned long id = strtoul(p, &end, 10);
        if (end == p || id >= OSD_F__COUNT) {
            return "bad field id";
        }
        osd[i / OSD_ROW_FIELDS][i % OSD_ROW_FIELDS] = (uint8_t)id;
        p = (*end == ',') ? end + 1 : end;
    }

    /* camlink_cfg_set_osd() re-checks the budget. The page checks it too, but
     * the page is not the only thing that can post here. */
    if (!camlink_cfg_set_osd((const uint8_t (*)[OSD_ROW_FIELDS])osd)) {
        return "row too long";
    }
    return NULL;
}

static esp_err_t osd_post(httpd_req_t *req)
{
    char body[192];
    if (!read_body(req, body, sizeof(body))) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad body");
        return ESP_FAIL;
    }
    const char *err = webcfg_apply_osd(body);
    if (err) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, err); return ESP_FAIL; }
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

/* Forget the pairing keys, keep the chosen camera.
 *
 * A bond that one side has kept and the other has dropped cannot be detected
 * from here -- it presents as a connection that fails before any GATT traffic,
 * which looks exactly like being out of range. This is the manual remedy, and
 * it is deliberately manual: clearing bonds automatically on a run of failures
 * would re-pair the module every time a camera went briefly out of range, and
 * a PIN prompt the user did not ask for is worse than a link that is honestly
 * reported as down. */
static esp_err_t repair_post(httpd_req_t *req)
{
    camlink_ble_clear_bonds();
    ESP_LOGW(TAG, "pairing keys cleared by the user; the camera will ask to pair again");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

/* The live camera list. Polled by the page while the user is looking at it, so
 * a camera switched on after the page loaded still turns up. */
int webcfg_scan_json(char *body, size_t cap)
{
    cam_scan_entry_t cams[CAM_SCAN_MAX];
    int n = cam_scan_get(cams, CAM_SCAN_MAX);

    uint8_t bound_mac[6];
    bool bound = duml_cam_bound_addr(bound_mac);

    int  w = snprintf(body, cap, "{\"cams\":[");

    for (int i = 0; i < n; i++) {
        const cam_scan_entry_t *c = &cams[i];
        /* Stop cleanly on the last entry that fits rather than emitting a
         * truncated object: a half-written JSON array fails to parse and the
         * page shows nothing at all, which is a worse answer than a short
         * list. CAM_SCAN_MAX is 12 and each entry is ~80 bytes, so this is a
         * backstop, not the normal path. */
        if (w > (int)cap - 160) break;
        w += snprintf(body + w, cap - w,
            "%s{\"mac\":\"%02X:%02X:%02X:%02X:%02X:%02X\","
            "\"id\":\"%02X%02X%02X%02X%02X%02X\",\"name\":\"%s\","
            "\"model\":\"%s\",\"vendor\":\"%s\",\"rssi\":%d,\"bound\":%s}",
            i ? "," : "",
            c->bda[0], c->bda[1], c->bda[2], c->bda[3], c->bda[4], c->bda[5],
            c->bda[0], c->bda[1], c->bda[2], c->bda[3], c->bda[4], c->bda[5],
            c->name, cam_scan_entry_model(c),
            c->vendor == CAM_VENDOR_GOPRO ? "gopro" : "dji", c->rssi,
            (bound && memcmp(bound_mac, c->bda, 6) == 0) ? "true" : "false");
    }
    w += snprintf(body + w, cap - w, "]}");

    return w;
}

static esp_err_t scan_get(httpd_req_t *req)
{
    char body[1400];
    int w = webcfg_scan_json(body, sizeof(body));
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, body, w);
}

/* Twelve hex digits, no separators.
 *
 * The page used to send "4C:43:..." and this rejected every one of them:
 * URLSearchParams percent-encodes a colon to %3A, and httpd_query_key_value()
 * hands back the raw form without decoding it. Rather than add a URL decoder
 * for one field, the MAC now travels in a form that has nothing to encode.
 * Colons are a display concern and stay in the page. */
static bool parse_mac(const char *s, uint8_t out[6])
{
    if (strlen(s) != 12) return false;
    for (int i = 0; i < 6; i++) {
        uint8_t b = 0;
        for (int n = 0; n < 2; n++) {
            char c = s[i * 2 + n];
            uint8_t v;
            if      (c >= '0' && c <= '9') v = (uint8_t)(c - '0');
            else if (c >= 'a' && c <= 'f') v = (uint8_t)(c - 'a' + 10);
            else if (c >= 'A' && c <= 'F') v = (uint8_t)(c - 'A' + 10);
            else return false;
            b = (uint8_t)((b << 4) | v);
        }
        out[i] = b;
    }
    return true;
}

const char *webcfg_apply_bind(const char *body)
{
    char macstr[16];
    uint8_t mac[6];
    if (httpd_query_key_value(body, "mac", macstr, sizeof(macstr)) != ESP_OK ||
        !parse_mac(macstr, mac)) {
        return "bad mac";
    }

    /* Only bind to something the scanner can actually see right now. This is
     * the whole point of the picker: a MAC typed, stale or replayed from an old
     * page would put us straight back to binding blind. */
    cam_scan_entry_t cams[CAM_SCAN_MAX];
    int n = cam_scan_get(cams, CAM_SCAN_MAX);
    bool visible = false;
    uint8_t model = 0;
    uint8_t vendor = CAM_VENDOR_DJI;
    uint8_t addr_type = 0;
    const char *name = NULL;
    for (int i = 0; i < n; i++) {
        if (memcmp(cams[i].bda, mac, 6) == 0) {
            visible = true;
            vendor = cams[i].vendor;
            addr_type = cams[i].addr_type;
            /* The model code decides which protocol the module will speak to
             * this camera, so it is stored with the binding rather than being
             * rediscovered at connect time -- by then we are already choosing. */
            model = cams[i].model;
            name  = cams[i].name;
            break;
        }
    }
    if (!visible) {
        return "camera not in range";
    }

    duml_cam_set_binding(mac, model, name, vendor, addr_type);
    return NULL;
}

static esp_err_t bind_post(httpd_req_t *req)
{
    char body[96];
    if (!read_body(req, body, sizeof(body))) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad body");
        return ESP_FAIL;
    }
    const char *err = webcfg_apply_bind(body);
    if (err) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, err); return ESP_FAIL; }
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

static void reboot_task(void *arg);   /* defined with the exit handler below */

/* ==========================================================================
 * Firmware update
 *
 * The new image is streamed into the app slot we are NOT running from, so
 * nothing is overwritten while it is in use and a failed upload simply leaves
 * the old firmware to boot again.
 *
 * The safety net is the bootloader's, not ours: with rollback enabled the new
 * image boots as PENDING_VERIFY and must call
 * esp_ota_mark_app_valid_cancel_rollback() to keep itself. If it crashes or
 * hangs first, the bootloader reverts on its own. That matters more here than
 * in most projects -- the user this feature exists for is precisely the one who
 * cannot recover a bricked module over USB.
 * ========================================================================== */

/* Enough for the image header, the first segment header, and the app
 * descriptor that carries the project name. */
#define OTA_HDR_BYTES  (sizeof(esp_image_header_t) + \
                        sizeof(esp_image_segment_header_t) + \
                        sizeof(esp_app_desc_t))

/* Refuse anything that is not a SlateFPV image for this chip, BEFORE a byte of
 * it reaches the slot. Without this, picking the wrong file in a phone's file
 * browser -- a photo, the merged full-flash image, firmware for another board
 * -- would half-write the spare slot and then be booted into. */
static bool image_is_ours(const uint8_t *hdr, char *ver, size_t ver_len)
{
    const esp_image_header_t *ih = (const esp_image_header_t *)hdr;
    if (ih->magic != ESP_IMAGE_HEADER_MAGIC) {
        ESP_LOGW(TAG, "rejected: not an ESP application image");
        return false;
    }
    if (ih->chip_id != CONFIG_IDF_FIRMWARE_CHIP_ID) {
        ESP_LOGW(TAG, "rejected: built for chip id %d, this is %d",
                 ih->chip_id, CONFIG_IDF_FIRMWARE_CHIP_ID);
        return false;
    }

    const esp_app_desc_t *ad = (const esp_app_desc_t *)
        (hdr + sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t));
    if (ad->magic_word != ESP_APP_DESC_MAGIC_WORD) {
        ESP_LOGW(TAG, "rejected: no application descriptor");
        return false;
    }

    const esp_app_desc_t *self = esp_app_get_description();
    if (self && strncmp(ad->project_name, self->project_name,
                        sizeof(ad->project_name)) != 0) {
        ESP_LOGW(TAG, "rejected: image is \"%.*s\", not \"%s\"",
                 (int)sizeof(ad->project_name), ad->project_name,
                 self->project_name);
        return false;
    }

    if (ver && ver_len) {
        snprintf(ver, ver_len, "%.*s", (int)sizeof(ad->version), ad->version);
    }
    return true;
}

static esp_err_t update_post(httpd_req_t *req)
{
    const esp_partition_t *dst = esp_ota_get_next_update_partition(NULL);
    if (dst == NULL) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no spare slot");
        return ESP_FAIL;
    }
    if (req->content_len == 0 || req->content_len > dst->size) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad size");
        return ESP_FAIL;
    }

    ESP_LOGW(TAG, "update: %d bytes -> %s", req->content_len, dst->label);

    esp_ota_handle_t h = 0;
    esp_err_t e = esp_ota_begin(dst, req->content_len, &h);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "ota_begin: %s", esp_err_to_name(e));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "cannot start");
        return ESP_FAIL;
    }

    /* Streamed in chunks. There are 107 KB of heap and the image is 1.4 MB, so
     * buffering the upload was never an option. */
    const size_t CHUNK = 2048;
    uint8_t *buf = malloc(CHUNK);
    uint8_t  hdr[OTA_HDR_BYTES];
    size_t   hdr_n = 0, got = 0;
    bool     checked = false;
    char     ver[32] = {0};

    if (buf == NULL) {
        esp_ota_abort(h);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "out of memory");
        return ESP_FAIL;
    }

    while (got < (size_t)req->content_len) {
        size_t want = (size_t)req->content_len - got;
        if (want > CHUNK) want = CHUNK;

        int r = httpd_req_recv(req, (char *)buf, want);
        if (r == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (r <= 0) {
            ESP_LOGE(TAG, "upload died after %u of %d bytes",
                     (unsigned)got, req->content_len);
            goto fail;
        }

        size_t off = 0;
        if (!checked) {
            /* The header can arrive split across TCP segments, so accumulate
             * it before judging the image -- and write nothing until it has
             * passed. */
            size_t take = OTA_HDR_BYTES - hdr_n;
            if (take > (size_t)r) take = (size_t)r;
            memcpy(hdr + hdr_n, buf, take);
            hdr_n += take;
            off    = take;
            got   += (size_t)r;
            if (hdr_n < OTA_HDR_BYTES) continue;

            if (!image_is_ours(hdr, ver, sizeof(ver))) {
                free(buf);
                esp_ota_abort(h);
                httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                    "not a SlateFPV firmware image");
                return ESP_FAIL;
            }
            checked = true;
            ESP_LOGW(TAG, "image accepted, version \"%s\"", ver);
            if (esp_ota_write(h, hdr, OTA_HDR_BYTES) != ESP_OK) goto fail;
            if ((size_t)r > off) {
                if (esp_ota_write(h, buf + off, (size_t)r - off) != ESP_OK) goto fail;
            }
            continue;
        }

        if (esp_ota_write(h, buf, (size_t)r) != ESP_OK) goto fail;
        got += (size_t)r;
    }

    free(buf);
    buf = NULL;

    e = esp_ota_end(h);
    if (e != ESP_OK) {
        /* ESP_ERR_OTA_VALIDATE_FAILED here means the image is corrupt: it
         * arrived, but its own checksum disagrees. Nothing has been booted
         * into, so this is a clean refusal. */
        ESP_LOGE(TAG, "ota_end: %s", esp_err_to_name(e));
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "image failed validation");
        return ESP_FAIL;
    }
    if ((e = esp_ota_set_boot_partition(dst)) != ESP_OK) {
        ESP_LOGE(TAG, "set_boot: %s", esp_err_to_name(e));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "cannot switch slot");
        return ESP_FAIL;
    }

    ESP_LOGW(TAG, "update staged, booting %s", dst->label);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    xTaskCreate(reboot_task, "reboot", 4096, NULL, 5, NULL);
    return ESP_OK;

fail:
    free(buf);
    esp_ota_abort(h);
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "write failed");
    return ESP_FAIL;
}

static esp_err_t forget_post(httpd_req_t *req)
{
    ESP_LOGW(TAG, "forget requested by client");
    duml_cam_forget_binding();
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

/* Leaving config mode has to be unconditional.
 *
 * A bare esp_restart() is enough from an idle access point, but not from the
 * request path with a phone associated and a socket open: esp_restart() runs
 * the registered shutdown handlers first, the Wi-Fi one takes the radio down,
 * and the sequence can then block before it ever reaches the reset. What the
 * user sees is the network vanish and the module sit there dark forever --
 * which is precisely the state config mode was designed never to reach.
 *
 * So: tear down in dependency order ourselves, sockets before the AP carrying
 * them, and arm a watchdog first that resets the chip regardless. */
void webcfg_restart_now(void)
{
    /* CONFIG_ESP_TASK_WDT_PANIC is off in this project, so a hung task is
     * otherwise never rescued -- nothing in the system would reset a module
     * stuck here. This is the backstop that makes the exit path total: if any
     * step below blocks, the chip resets anyway. */
    esp_task_wdt_config_t twdt = {
        .timeout_ms     = 4000,
        .idle_core_mask = 0,
        .trigger_panic  = true,
    };
    if (esp_task_wdt_reconfigure(&twdt) == ESP_OK) {
        esp_task_wdt_add(NULL);
    }

    if (s_httpd) {
        httpd_stop(s_httpd);          /* closes listener and live connections */
        s_httpd = NULL;
    }
    esp_wifi_stop();
    esp_wifi_deinit();

    esp_restart();
    for (;;) { }
}

static void reboot_task(void *arg)
{
    (void)arg;
    /* Long enough for the reply to reach the browser, short enough that the
     * user does not wonder whether the button worked. Stack sized for the
     * Wi-Fi teardown, which runs on whichever task calls it. */
    vTaskDelay(pdMS_TO_TICKS(400));
    webcfg_restart_now();
}

static esp_err_t exit_post(httpd_req_t *req)
{
    ESP_LOGW(TAG, "exit requested by client");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Connection", "close");
    esp_err_t e = httpd_resp_sendstr(req, "{\"ok\":true}");
    xTaskCreate(reboot_task, "reboot", 4096, NULL, 5, NULL);
    return e;
}

/* Anything we do not serve becomes a redirect to the settings page. This is
 * what the OS connectivity probes hit, and answering them with a redirect
 * rather than a 404 is what triggers the "sign in to network" sheet. */
static esp_err_t not_found(httpd_req_t *req, httpd_err_code_t err)
{
    (void)err;
    ESP_LOGI(TAG, "unhandled %d %s -> redirect", (int)req->method, req->uri);
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "http://" AP_IP "/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

static void http_start(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.max_uri_handlers = 12;
    cfg.lru_purge_enable = true;
    cfg.stack_size       = 5120;
    /* A 1.4 MB upload from a phone is slow and bursty; the stock 5 s would
     * abandon it during an ordinary stall. */
    cfg.recv_wait_timeout = 20;
    cfg.send_wait_timeout = 20;

    if (httpd_start(&s_httpd, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "httpd start failed");
        return;
    }

    static const httpd_uri_t uris[] = {
        { .uri = "/",            .method = HTTP_GET,  .handler = page_get   },
        { .uri = "/api/state",   .method = HTTP_GET,  .handler = state_get  },
        { .uri = "/api/config",  .method = HTTP_POST, .handler = config_post },
        { .uri = "/api/osd",     .method = HTTP_POST, .handler = osd_post   },
        { .uri = "/api/scan",    .method = HTTP_GET,  .handler = scan_get   },
        { .uri = "/api/bind",    .method = HTTP_POST, .handler = bind_post  },
        { .uri = "/api/forget",  .method = HTTP_POST, .handler = forget_post },
        { .uri = "/api/repair",  .method = HTTP_POST, .handler = repair_post },
        { .uri = "/api/exit",    .method = HTTP_POST, .handler = exit_post  },
        { .uri = "/api/update",  .method = HTTP_POST, .handler = update_post },
    };
    for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
        httpd_register_uri_handler(s_httpd, &uris[i]);
    }
    httpd_register_err_handler(s_httpd, HTTPD_404_NOT_FOUND, not_found);
}

/* ==========================================================================
 * Access point
 * ========================================================================== */
static void wifi_start(void)
{
    ESP_ERROR_CHECK(esp_netif_init());

    esp_err_t e = esp_event_loop_create_default();
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) ESP_ERROR_CHECK(e);

    esp_netif_create_default_wifi_ap();

    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));

    /* Name the AP after this board's own MAC. Two modules within range of each
     * other must not present the same SSID -- the user would have no way to
     * tell which quad they are about to reconfigure. */
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    snprintf(s_ssid, sizeof(s_ssid), "SLATE-%02X%02X", mac[4], mac[5]);

    wifi_config_t ap = { 0 };
    strncpy((char *)ap.ap.ssid, s_ssid, sizeof(ap.ap.ssid));
    ap.ap.ssid_len       = strlen(s_ssid);
    ap.ap.channel        = AP_CHANNEL;
    ap.ap.max_connection = AP_MAX_CONN;
    /* Open, deliberately. A password would have to be either printed on the
     * board or identical on every unit, and neither is real security; what
     * actually protects the module is that the AP only exists for the minutes
     * someone deliberately held the button, on the ground, and vanishes on the
     * next reboot. An open network is also what lets the captive portal open
     * the page by itself. */
    ap.ap.authmode = WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));

    /* Give the access point the radio.
     *
     * Config mode runs the BLE camera scanner and this access point on one
     * antenna, arbitrated by the coexistence layer. Left in balance, an active
     * BLE scan transmits often enough to starve the softAP's beacons -- the
     * camera list stays fine while the network the user is trying to join
     * never appears, which is exactly the wrong way round: the page is what
     * they came for, and the scan can miss a few adverts without anyone
     * noticing. So Wi-Fi is preferred here, and power save is off so the
     * beacon goes out on time rather than being deferred. */
    esp_coex_preference_set(ESP_COEX_PREFER_WIFI);
    esp_wifi_set_ps(WIFI_PS_NONE);

    ESP_ERROR_CHECK(esp_wifi_start());
    /* softAP TX at full power: the phone is across a room, not centimetres
     * away like the camera, and this radio is only up on the ground. */
    esp_wifi_set_max_tx_power(78);

    ESP_LOGI(TAG, "AP \"%s\" up on ch %d, settings at http://%s/", s_ssid, AP_CHANNEL, AP_IP);
}

void webcfg_start(void)
{
    wifi_start();
    http_start();
    xTaskCreate(dns_task, "dns", 3072, NULL, 4, NULL);
}
