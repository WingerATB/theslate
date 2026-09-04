/* SPDX-License-Identifier: PolyForm-Strict-1.0.0
 *
 * USB console. See usbcli.h for what it is for.
 *
 * How it reads: the console is on the C3's native USB Serial/JTAG peripheral,
 * and the log already writes to it through the VFS in polling mode -- which
 * copes with an unplugged cable by dropping output after a short timeout, so a
 * module in a frame with nothing on USB never blocks on a log line. Installing
 * the interrupt driver would replace that path for the whole console, and its
 * behaviour with no host attached is exactly the thing this firmware must not
 * find out about in the air. So this reads the receive FIFO directly instead,
 * polled from a low-priority task, and leaves output alone. The log and the
 * command replies interleave on one port, which is what you want at a bench:
 * the reply to `rec on` is followed by whatever the camera said about it.
 */
#include "usbcli.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_app_desc.h"
#include "hal/usb_serial_jtag_ll.h"

#include "camlink.h"
#include "duml_cam.h"
#include "gopro_cam.h"
#include "cam_scan.h"
#include "ble.h"
#include "msp.h"
#include "webcfg.h"
#include "esp_system.h"

static const char *TAG = "CLI";

static bool s_config_mode;

#define CLI_LINE_MAX 256

/* ---- output ------------------------------------------------------------- */

static void out(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void out(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
}

static void mac_str(const uint8_t *m, char out6[18])
{
    snprintf(out6, 18, "%02X:%02X:%02X:%02X:%02X:%02X", m[0], m[1], m[2], m[3], m[4], m[5]);
}

/* ---- commands ----------------------------------------------------------- */

static int parse_hex(const char *s, uint8_t *out, int max);

static void cmd_help(void)
{
    out("commands:\n"
        "  st            camera, flight controller and OSD state\n"
        "  rec on|off|t  record on, off, or toggle (as the module's button would)\n"
        "  scan          cameras in range (setup mode only)\n"
        "  bind <mac>    bind a camera from that list, e.g. bind CB58A04DA4A2 (setup mode)\n"
        "  setup         reboot into setup (Wi-Fi + camera list)\n"
        "  bindnear      reboot into bind mode: bind the nearest camera (same as the 3 s button hold)\n"
        "  usbcfg        reboot into USB-config mode (settings UI over this cable, no Wi-Fi)\n"
        "  api ...       machine transport for the browser tool; not meant to be typed\n"
        "  reboot        reboot normally\n"
        "  forget        forget the bound camera\n"
        "  bonds         clear every stored BLE bond\n"
        "  tx <dBm>      radio power now: -24 -12 0 3 (not saved)\n"
        "  gp rec 1|0    GoPro: shutter on / off\n"
        "  gp sleep|wake GoPro: put to sleep / reconnect (wakes it)\n"
        "  gp hw         GoPro: ask for hardware info\n"
        "  gp cmd|set|qry <hex bytes>   GoPro: raw packet, e.g. gp cmd 03 01 01 01\n"
        "  log e|w|i|d|v log level: error, warn, info, debug, verbose\n"
        "  ver           firmware version\n"
        "  help          this\n");
}

static void cmd_status(void)
{
    const uint32_t up = (uint32_t)(esp_timer_get_time() / 1000000);
    out("-- module: %s mode, up %us, heap %u B free (min %u)\n",
        s_config_mode ? "SETUP" : "normal", (unsigned)up,
        (unsigned)esp_get_free_heap_size(), (unsigned)esp_get_minimum_free_heap_size());

    uint8_t mac[6];
    char ms[18];
    if (duml_cam_bound_addr(mac)) {
        mac_str(mac, ms);
        out("-- bound camera: %s model=0x%02X\n", ms, duml_cam_bound_model());
    } else {
        out("-- bound camera: none\n");
    }

    if (s_config_mode) {
        out("-- camera link: not started in setup mode\n");
    } else {
        duml_cam_status_t d;
        duml_cam_get(&d);
        cam_status_t c;
        camlink_get_cam_status(&c);
        uint8_t reason; uint32_t fails; bool ever;
        camlink_ble_link_fault(&reason, &fails, &ever);
        out("-- camera: link=%s label=%s status=%s (age %ums)\n",
            d.link_up ? "UP" : "down", d.label,
            d.status_valid ? "valid" : "STALE", (unsigned)d.status_age_ms);
        out("   rec=%s clip=%us left=%us free=%uMB batt=%u%%%s temp=%u%s mode=%u\n",
            d.rec_state == DUML_REC_RECORDING ? "REC" :
            d.rec_state == DUML_REC_STARTING  ? "starting" : "idle",
            (unsigned)d.clip_s, (unsigned)d.left_s, (unsigned)d.free_mb,
            d.battery_pct, d.battery_valid ? "" : "?",
            c.temp_over, c.temp_over_valid ? "" : "?", d.work_mode);
        out("   manual record latch=%s  last BLE fault=0x%02X %s (%u fails%s)\n",
            camlink_get_manual_record() ? "on" : "off",
            reason, camlink_ble_fault_text(reason), (unsigned)fails,
            ever ? "" : ", never connected");
        if (duml_cam_is_gopro()) {
            gopro_status_t g;
            gopro_cam_get(&g);
            out("   gopro: model=\"%s\" encoding=%d busy=%d hot=%u sd_ok=%d clip=%us left=%us batt=%u%%\n",
                g.model, g.encoding, g.busy, g.hot, g.sd_ok,
                (unsigned)g.clip_s, (unsigned)g.left_s, g.battery_pct);
        }
    }

    msp_state_t m;
    msp_get_state(&m);
    out("-- flight controller: link=%s armed=%s boxarm=%s replies=%u crc_err=%u osd_writes=%u\n",
        m.link_up ? "UP" : "down", m.armed ? "YES" : "no",
        m.boxarm_known ? "known" : "unknown",
        (unsigned)m.good_replies, (unsigned)m.crc_errors, (unsigned)m.osd_writes);
    if (m.rc_valid) {
        out("   rc:");
        for (int i = 0; i < m.rc_count && i < MSP_MAX_RC_CHANNELS; i++) out(" %u", m.rc[i]);
        out("\n");
    }

    if (!s_config_mode) {
        char rows[4][17];
        camlink_get_osd(rows);
        for (int i = 0; i < 4; i++) out("-- osd%d: |%-16s|\n", i + 1, rows[i]);
    }
}

static void cmd_rec(const char *arg)
{
    if (s_config_mode) { out("no camera link in setup mode\n"); return; }
    if (arg && (!strcmp(arg, "on") || !strcmp(arg, "1"))) {
        camlink_set_manual_record(true);
        out("record: on\n");
    } else if (arg && (!strcmp(arg, "off") || !strcmp(arg, "0"))) {
        camlink_set_manual_record(false);
        out("record: off\n");
    } else {
        camlink_press_record();
        out("record: toggled\n");
    }
}

static void cmd_scan(void)
{
    if (!s_config_mode) { out("the camera list only exists in setup mode (try: setup)\n"); return; }
    cam_scan_entry_t cams[CAM_SCAN_MAX];
    int n = cam_scan_get(cams, CAM_SCAN_MAX);
    if (n == 0) { out("no cameras in range\n"); return; }
    for (int i = 0; i < n; i++) {
        char ms[18];
        mac_str(cams[i].bda, ms);
        out("  %s  %-20s %-6s %-18s %4d dBm\n", ms,
            cams[i].name[0] ? cams[i].name : "(no name)",
            cams[i].vendor == CAM_VENDOR_GOPRO ? "GoPro" : "DJI",
            cam_scan_entry_model(&cams[i]), cams[i].rssi);
    }
}

static void cmd_bind(const char *arg)
{
    if (!s_config_mode) { out("bind only works in setup mode (try: setup)\n"); return; }
    if (!arg) { out("bind <12 hex digits of the MAC, from scan>\n"); return; }
    uint8_t mac[6];
    char clean[13]; int n = 0;
    for (const char *p = arg; *p && n < 12; p++) if (isxdigit((unsigned char)*p)) clean[n++] = *p;
    clean[n] = '\0';
    if (n != 12 || parse_hex(clean, mac, 6) != 6) { out("bad mac\n"); return; }

    cam_scan_entry_t cams[CAM_SCAN_MAX];
    int c = cam_scan_get(cams, CAM_SCAN_MAX);
    for (int i = 0; i < c; i++) {
        if (memcmp(cams[i].bda, mac, 6) == 0) {
            duml_cam_set_binding(mac, cams[i].model, cams[i].name, cams[i].vendor, cams[i].addr_type);
            out("bound %s (%s). reboot to connect.\n", cams[i].name[0] ? cams[i].name : "camera",
                cams[i].vendor == CAM_VENDOR_GOPRO ? "GoPro" : "DJI");
            return;
        }
    }
    out("not in range -- see scan\n");
}

static void cmd_log(const char *arg)
{
    esp_log_level_t lvl;
    switch (arg ? arg[0] : 0) {
    case 'e': lvl = ESP_LOG_ERROR;   break;
    case 'w': lvl = ESP_LOG_WARN;    break;
    case 'i': lvl = ESP_LOG_INFO;    break;
    case 'd': lvl = ESP_LOG_DEBUG;   break;
    case 'v': lvl = ESP_LOG_VERBOSE; break;
    default:  out("log e|w|i|d|v\n"); return;
    }
    esp_log_level_set("*", lvl);
    out("log level set\n");
}

static int parse_hex(const char *s, uint8_t *out, int max)
{
    int n = 0;
    while (s && *s && n < max) {
        while (*s == ' ') s++;
        if (!isxdigit((unsigned char)s[0]) || !isxdigit((unsigned char)s[1])) break;
        char b[3] = { s[0], s[1], 0 };
        out[n++] = (uint8_t)strtoul(b, NULL, 16);
        s += 2;
    }
    return n;
}

static void cmd_gp(char *arg)
{
    if (!duml_cam_is_gopro()) { out("bound camera is not a GoPro\n"); return; }
    if (!arg) { out("gp rec 1|0 | sleep | wake | hw | cmd|set|qry <hex>\n"); return; }
    char *sub = arg;
    char *rest = arg;
    while (*rest && *rest != ' ') rest++;
    if (*rest) { *rest++ = '\0'; while (*rest == ' ') rest++; }
    if (!*rest) rest = NULL;

    if (!strcmp(sub, "rec"))        { gopro_cam_request_record(rest && rest[0] == '1'); out("ok\n"); }
    else if (!strcmp(sub, "sleep")) { gopro_cam_sleep(); out("ok\n"); }
    else if (!strcmp(sub, "wake"))  { gopro_cam_wake(); out("ok\n"); }
    else if (!strcmp(sub, "hw"))    { uint8_t q[] = { 0x01, 0x3C }; out(gopro_cam_write(GOPRO_CH_CMD, q, 2) ? "sent\n" : "not connected\n"); }
    else if (!strcmp(sub, "cmd") || !strcmp(sub, "set") || !strcmp(sub, "qry")) {
        uint8_t b[40];
        int n = parse_hex(rest, b, sizeof(b));
        if (n == 0) { out("no bytes\n"); return; }
        gopro_ch_t ch = sub[0] == 'c' ? GOPRO_CH_CMD : sub[0] == 's' ? GOPRO_CH_SET : GOPRO_CH_QRY;
        out(gopro_cam_write(ch, b, (size_t)n) ? "sent %d bytes\n" : "not connected\n", n);
    }
    else out("gp: unknown\n");
}

/* The browser tool's transport. Every reply is one line beginning "@api <id>"
 * so the host can match it and ignore the log lines interleaved on the same
 * wire. The JSON is exactly what the Wi-Fi server serves -- same builders. */
static void cmd_api(char *arg)
{
    if (!arg) { out("@api - err noargs\n"); return; }
    char *id = arg, *p = arg;
    while (*p && *p != ' ') p++;
    if (*p) { *p++ = '\0'; while (*p == ' ') p++; }
    char *verb = p;
    while (*p && *p != ' ') p++;
    if (*p) { *p++ = '\0'; while (*p == ' ') p++; }
    char *rest = (*p) ? p : NULL;

    static char buf[2048];
    if (!strcmp(verb, "state")) {
        int n = webcfg_state_json(buf, sizeof(buf));
        out("@api %s ok %.*s\n", id, n, buf);
    } else if (!strcmp(verb, "scan")) {
        int n = webcfg_scan_json(buf, sizeof(buf));
        out("@api %s ok %.*s\n", id, n, buf);
    } else if (!strcmp(verb, "config")) {
        const char *e = webcfg_apply_config(rest ? rest : "");
        if (e) out("@api %s err %s\n", id, e); else out("@api %s ok {}\n", id);
    } else if (!strcmp(verb, "osd")) {
        const char *e = webcfg_apply_osd(rest ? rest : "");
        if (e) out("@api %s err %s\n", id, e); else out("@api %s ok {}\n", id);
    } else if (!strcmp(verb, "bind")) {
        const char *e = webcfg_apply_bind(rest ? rest : "");
        if (e) out("@api %s err %s\n", id, e); else out("@api %s ok {}\n", id);
    } else if (!strcmp(verb, "forget")) {
        duml_cam_forget_binding(); out("@api %s ok {}\n", id);
    } else if (!strcmp(verb, "repair")) {
        camlink_ble_clear_bonds(); out("@api %s ok {}\n", id);
    } else if (!strcmp(verb, "mode")) {
        out("@api %s ok {\"usbcfg\":%s}\n", id, s_config_mode ? "true" : "false");
    } else if (!strcmp(verb, "usbcfg")) {
        out("@api %s ok {}\n", id); fflush(stdout);
        vTaskDelay(pdMS_TO_TICKS(150)); webcfg_reboot_into_usbcfg();
    } else if (!strcmp(verb, "normal")) {
        out("@api %s ok {}\n", id); fflush(stdout);
        vTaskDelay(pdMS_TO_TICKS(150)); webcfg_reboot_into_normal();
    } else if (!strcmp(verb, "exit")) {
        out("@api %s ok {}\n", id); fflush(stdout);
        vTaskDelay(pdMS_TO_TICKS(150)); esp_restart();
    } else {
        out("@api %s err unknown-verb\n", id);
    }
}

static void cmd_tx(const char *arg)
{
    if (!arg) { out("tx -24|-12|0|3\n"); return; }
    int dbm = atoi(arg);
    if (dbm < -24 || dbm > 9 || ((dbm + 24) % 3) != 0) { out("tx: use -24, -12, 0 or 3\n"); return; }
    camlink_ble_apply_tx_level((dbm + 24) / 3);
    out("radio %d dBm until reboot\n", dbm);
}

static void run_line(char *line)
{
    /* Trim, split off the first word. */
    while (*line && isspace((unsigned char)*line)) line++;
    char *end = line + strlen(line);
    while (end > line && isspace((unsigned char)end[-1])) *--end = '\0';
    if (*line == '\0') return;

    char *arg = line;
    while (*arg && !isspace((unsigned char)*arg)) arg++;
    if (*arg) { *arg++ = '\0'; while (*arg && isspace((unsigned char)*arg)) arg++; }
    if (*arg == '\0') arg = NULL;

    if      (!strcmp(line, "help") || !strcmp(line, "?")) cmd_help();
    else if (!strcmp(line, "st") || !strcmp(line, "status")) cmd_status();
    else if (!strcmp(line, "rec"))    cmd_rec(arg);
    else if (!strcmp(line, "scan"))   cmd_scan();
    else if (!strcmp(line, "bind"))   cmd_bind(arg);
    else if (!strcmp(line, "setup"))  { out("rebooting into setup\n"); webcfg_reboot_into_config(); }
    else if (!strcmp(line, "bindnear")) { out("rebooting into bind mode\n"); webcfg_reboot_into_bind(); }
    else if (!strcmp(line, "reboot")) { out("rebooting\n"); fflush(stdout); vTaskDelay(pdMS_TO_TICKS(100)); esp_restart(); }
    else if (!strcmp(line, "forget")) { duml_cam_forget_binding(); out("camera binding cleared (takes effect on reboot)\n"); }
    else if (!strcmp(line, "bonds"))  { camlink_ble_clear_bonds(); out("BLE bonds cleared\n"); }
    else if (!strcmp(line, "log"))    cmd_log(arg);
    else if (!strcmp(line, "gp"))     cmd_gp(arg);
    else if (!strcmp(line, "api"))    cmd_api(arg);
    else if (!strcmp(line, "usbcfg")) { out("rebooting into USB-config mode\n"); webcfg_reboot_into_usbcfg(); }
    else if (!strcmp(line, "normal")) { out("rebooting into normal mode for one boot (test)\n"); webcfg_reboot_into_normal(); }
    else if (!strcmp(line, "tx"))     cmd_tx(arg);
    else if (!strcmp(line, "ver"))    {
        const esp_app_desc_t *a = esp_app_get_description();
        out("%s %s (%s %s, IDF %s)\n", a->project_name, a->version, a->date, a->time, a->idf_ver);
    }
    else out("unknown: %s (try help)\n", line);
}

/* ---- input -------------------------------------------------------------- */

static void cli_task(void *arg)
{
    (void)arg;
    static char line[CLI_LINE_MAX];
    size_t n = 0;
    uint8_t buf[64];

    for (;;) {
        int got = 0;
        if (usb_serial_jtag_ll_rxfifo_data_available()) {
            got = usb_serial_jtag_ll_read_rxfifo(buf, sizeof(buf));
        }
        if (got <= 0) { vTaskDelay(pdMS_TO_TICKS(20)); continue; }

        for (int i = 0; i < got; i++) {
            char c = (char)buf[i];
            if (c == '\r' || c == '\n') {
                if (n == 0) continue;          /* CR LF, or an empty line */
                out("\n");
                line[n] = '\0';
                run_line(line);
                n = 0;
            } else if (c == 0x7F || c == 0x08) {
                if (n) { n--; out("\b \b"); }
            } else if (c >= 0x20 && c < 0x7F) {
                if (n < CLI_LINE_MAX - 1) { line[n++] = c; out("%c", c); }
            }
        }
        fflush(stdout);
    }
}

void usbcli_start(bool config_mode)
{
#if CONFIG_CAMLINK_USB_CONSOLE
    s_config_mode = config_mode;
    /* Low priority and a modest stack: it polls, and it must never win the CPU
     * from the camera or the MSP link. */
    xTaskCreate(cli_task, "usbcli", 3072, NULL, 2, NULL);
    ESP_LOGI(TAG, "USB console ready -- type help");
#else
    (void)config_mode;
#endif
}
