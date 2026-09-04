/* SPDX-License-Identifier: PolyForm-Strict-1.0.0 */
#include <string.h>
#include <stdio.h>

#include "duml.h"
#include "duml_cam.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <stdlib.h>
#include "esp_gap_ble_api.h"
#include "gopro_cam.h"
#include "cam_scan.h"

#include "ble.h"
#include "connect_logic.h"
#include "data.h"
#include "status_logic.h"
#include "command_logic.h"
#include "nvs_flash.h"
#include "esp_mac.h"
#include "nvs.h"

static const char *TAG = "DUMLCAM";

#define CAM_STATUS_CMDSET  0x02
#define CAM_STATUS_CMDID   0x80
#define CAM_DO_RECORD      0x02
#define BATT_CMDSET        0x0D
#define BATT_CMDID         0x02
#define BATT_PCT_OFFSET      20    /* candidate, unconfirmed */

/* Pairing identity. Replayed from a known-good captured frame so pairing is
 * reproducible; the camera displays this PIN for the user to confirm once. */
#define PAIR_IDENTIFIER "001749319286102"
#define PAIR_PIN        "5160"

static duml_cam_status_t  s_st;
static SemaphoreHandle_t  s_lock;
static volatile bool      s_req_pending;
static volatile bool      s_req_value;
static uint32_t           s_last_status_ms;
static volatile uint8_t   s_status_snd, s_status_rcv;
static volatile uint8_t   s_last_rec_reply;
static volatile bool      s_have_rec_reply;
static volatile uint8_t   s_last_other_set, s_last_other_id;
static volatile uint8_t   s_last_other_snd, s_last_other_rcv;
static volatile uint32_t  s_other_count;
static uint8_t            s_last_other_pl[8];
static volatile uint8_t   s_last_other_pl_len;

/* Arbitrary command probe, for bringing up a camera whose command set differs.
 * The Osmo 360 pairs, streams status we decode correctly, and then ignores our
 * record command entirely -- so the next step is trying candidates, and doing
 * that by reflashing once per guess is far too slow. */
/* Command sweep.
 *
 * Sends one command id per second across a command set and watches whether the
 * camera starts recording, so the module identifies the command itself rather
 * than relying on someone watching a timer.
 *
 * This is a blunt instrument pointed at someone's camera. Unknown ids in a
 * camera command set are not all harmless -- DJI devices carry commands for
 * formatting storage, resetting settings and firmware operations, and this has
 * no way to know which id is which. Bring-up only, on a camera whose card you
 * are willing to lose. */
static uint32_t           s_last_batt_ms;

#define NVS_NS   "camlink"
#define NVS_KEY  "cam_addr"
#define NVS_KEY_MODEL "cam_model"
#define NVS_KEY_PROTO "cam_proto"
#define NVS_KEY_LABEL "cam_label"
#define NVS_KEY_ATYPE "cam_atype"

/* Which protocol a camera speaks, decided by its advertised model code.
 *
 * Established on hardware: the Nano ignores the R SDK entirely and the Osmo 360
 * ignores every DUML camera command, answering only DUML pairing. They are not
 * dialects of one protocol, they are two protocols that share a transport.
 *
 * Unknown models default to DUML because that is the path this project proved
 * first and understands best -- and because a wrong guess there fails visibly
 * at connect rather than silently at record time. */
typedef enum { CAM_PROTO_DUML = 0, CAM_PROTO_RSDK = 1, CAM_PROTO_GOPRO = 2 } cam_proto_t;

/* Model code first, advertised name second.
 *
 * The codes below are measured or taken from datagutt/node-osmo. The Osmo
 * Action 6 is not among them: its code is documented nowhere public, and
 * guessing a byte would be worse than not having one -- a wrong value routes
 * some OTHER camera to the wrong protocol, which fails at record time rather
 * than at connect.
 *
 * The name catches it instead. Every camera in this family advertises its
 * model: "Osmo360-8ED9", "OsmoNano-673E", so an Action advertises
 * "OsmoAction...". That covers Action 6 and whatever follows it without
 * needing a code at all. */
static const char *proto_name(cam_proto_t p)
{
    return p == CAM_PROTO_RSDK ? "R SDK" : p == CAM_PROTO_GOPRO ? "GoPro" : "DUML";
}

static cam_proto_t proto_for_camera(uint8_t model, const char *name, uint8_t vendor)
{
    /* A GoPro is a GoPro by its advert, never by a model code: it does not
     * send one. The picker knew which it was; that decision travels here. */
    if (vendor == CAM_VENDOR_GOPRO) return CAM_PROTO_GOPRO;
    switch (model) {
    case 0x12:   /* Osmo Action 3      */
    case 0x14:   /* Osmo Action 4      */
    case 0x15:   /* Osmo Action 5 Pro  */
    case 0x17:   /* Osmo 360           */
        return CAM_PROTO_RSDK;
    case 0x19:   /* Osmo Nano          */
        return CAM_PROTO_DUML;
    default:
        break;
    }
    if (name && strstr(name, "Action") != NULL) {
        return CAM_PROTO_RSDK;
    }
    return CAM_PROTO_DUML;
}

static uint8_t     s_bound_model;
static uint8_t     s_bound_addr[6];
static uint8_t     s_addr_type = BLE_ADDR_TYPE_PUBLIC;
static cam_proto_t s_proto = CAM_PROTO_DUML;
static char        s_label[6] = "CAM";

/* Four characters at most: that is what a row can spare for a camera name
 * beside anything else worth showing. Long enough for O360 and NANO, which is
 * the distinction that actually matters on a bench with two cameras on it. */
static void label_for_camera(uint8_t model, const char *name, char out[6])
{
    const char *by_model = NULL;
    switch (model) {
    case 0x12: by_model = "A3";   break;
    case 0x14: by_model = "A4";   break;
    case 0x15: by_model = "A5";   break;
    case 0x17: by_model = "O360"; break;
    case 0x19: by_model = "NANO"; break;
    default:   break;
    }
    if (by_model) { strlcpy(out, by_model, 6); return; }

    /* No published model code. The advertised name is all there is, and for the
     * Action series it carries the generation as a digit -- "OsmoAction6-1A2B".
     * Reading it out beats printing a generic label for a camera the user can
     * see the name of on the settings page. */
    if (name) {
        const char *a = strstr(name, "Action");
        if (a) {
            char d = a[6];
            if (d >= '0' && d <= '9') { out[0] = 'A'; out[1] = d; out[2] = '\0'; return; }
            strlcpy(out, "ACT", 6);
            return;
        }
        if (strstr(name, "360"))  { strlcpy(out, "O360", 6); return; }
        if (strstr(name, "Nano")) { strlcpy(out, "NANO", 6); return; }
    }
    strlcpy(out, "CAM", 6);
}

/* Persist the bound camera so the module comes back to the SAME camera after a
 * power cycle, instead of grabbing whichever one is nearest. */
static void bind_load(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGW(TAG, "no camera selected -- hold the button 10 s and pick one in setup");
        return;
    }
    uint8_t addr[6];
    size_t len = sizeof(addr);
    if (nvs_get_blob(h, NVS_KEY, addr, &len) == ESP_OK && len == sizeof(addr)) {
        ble_set_bound_addr(addr);
        memcpy(s_bound_addr, addr, 6);
        uint8_t m = 0, pr = 0, at = BLE_ADDR_TYPE_PUBLIC;
        nvs_get_u8(h, NVS_KEY_MODEL, &m);
        nvs_get_u8(h, NVS_KEY_ATYPE, &at);
        s_bound_model = m;
        s_addr_type   = at;
        /* The protocol was resolved when the camera was chosen, using its name
         * as well as its model code. The name is gone by now, so the decision
         * is stored rather than recomputed. */
        s_proto = (nvs_get_u8(h, NVS_KEY_PROTO, &pr) == ESP_OK)
                      ? (cam_proto_t)pr : proto_for_camera(m, NULL, CAM_VENDOR_DJI);
        size_t ll = sizeof(s_label);
        if (nvs_get_str(h, NVS_KEY_LABEL, s_label, &ll) != ESP_OK) {
            /* Bound by a firmware that predates the label. The model code is
             * enough for every camera whose code is known, which is all of them
             * except the Action 6 -- and that one only loses its digit until
             * the next time it is picked in setup. */
            label_for_camera(m, NULL, s_label);
        }
        ESP_LOGI(TAG, "bound camera model 0x%02X -> %s", m, proto_name(s_proto));
    } else {
        ESP_LOGW(TAG, "no camera selected -- hold the button 10 s and pick one in setup");
    }
    nvs_close(h);
}

static void bind_save(const uint8_t *addr, uint8_t model, const char *name,
                      uint8_t vendor, uint8_t addr_type)
{
    s_proto = proto_for_camera(model, name, vendor);
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_blob(h, NVS_KEY, addr, 6);
    nvs_set_u8(h, NVS_KEY_MODEL, model);
    nvs_set_u8(h, NVS_KEY_PROTO, (uint8_t)s_proto);
    nvs_set_u8(h, NVS_KEY_ATYPE, addr_type);
    if (s_proto == CAM_PROTO_GOPRO) {
        /* Placeholder until the camera says what it is: the session asks for
         * hardware info on connect and derives the real label from the model
         * name, which the advert does not carry. */
        strlcpy(s_label, "GPRO", sizeof(s_label));
    } else {
        label_for_camera(model, name, s_label);
    }
    nvs_set_str(h, NVS_KEY_LABEL, s_label);
    nvs_commit(h);
    nvs_close(h);
    s_bound_model = model;
    ESP_LOGI(TAG, "camera bound, model 0x%02X \"%s\" -> %s", model, s_label,
             proto_name(s_proto));
}

bool duml_cam_is_gopro(void)
{
    return s_proto == CAM_PROTO_GOPRO;
}

void duml_cam_disconnect(void)
{
    if (s_proto == CAM_PROTO_GOPRO) { gopro_cam_disconnect(); return; }
    if (connect_logic_get_state() >= BLE_CONNECTED) {
        connect_logic_ble_disconnect();
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

bool duml_cam_bound_addr(uint8_t out[6])
{
    nvs_handle_t h;
    if (out == NULL) return false;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return false;
    size_t len = 6;
    bool ok = (nvs_get_blob(h, NVS_KEY, out, &len) == ESP_OK && len == 6);
    nvs_close(h);
    return ok;
}

/* Declared in the header since the picker landed, never defined: nothing
 * called it until the USB console did. */
uint8_t duml_cam_bound_model(void)
{
    return s_bound_model;
}

void duml_cam_set_binding(const uint8_t addr[6], uint8_t model, const char *name,
                          uint8_t vendor, uint8_t addr_type)
{
    if (addr == NULL) return;
    bind_save(addr, model, name, vendor, addr_type);
    ESP_LOGW(TAG, "bound by user to %02X:%02X:%02X:%02X:%02X:%02X",
             addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
}

void duml_cam_forget_binding(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_erase_key(h, NVS_KEY);
    nvs_commit(h);
    nvs_close(h);
    ESP_LOGW(TAG, "binding erased");
}

static uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000); }

static void on_notify(const uint8_t *data, size_t len)
{
    duml_frame_t f;
    if (!duml_parse(data, len, &f) || !f.crc_ok) {
        return;
    }

    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(20)) != pdTRUE) {
        return;
    }

    if (f.cmd_set == CAM_STATUS_CMDSET && f.cmd_id == CAM_STATUS_CMDID &&
        f.payload_len >= 31) {
        const uint8_t *p = f.payload;
        s_st.rec_state = (duml_rec_state_t)((p[0] >> 6) & 0x03);
        /* Payload byte 4 is the camera's work mode, per the DUML camera
         * dissector in o-gs/dji-firmware-tools: 0 TAKEPHOTO, 1 RECORD,
         * 2 PLAYBACK, 3 TRANSCODE, 4 TUNING, 5 SAVEPOWER, 6 DOWNLOAD.
         *
         * Never read until now, and it is the obvious explanation for a camera
         * that accepts Do Record and does nothing: the command is only
         * meaningful in RECORD mode. The same dissector confirms the offsets
         * this project reverse-engineered by hand -- free space at [9..12],
         * remaining time at [17..20], record state as mask 0x00c0 of the first
         * word -- so byte 4 can be trusted without re-deriving it. */
        s_st.work_mode = p[4];
        s_st.free_mb   = (uint32_t)p[9]  | ((uint32_t)p[10] << 8)
                       | ((uint32_t)p[11] << 16) | ((uint32_t)p[12] << 24);
        s_st.left_s    = (uint32_t)p[17] | ((uint32_t)p[18] << 8)
                       | ((uint32_t)p[19] << 16) | ((uint32_t)p[20] << 24);
        s_st.clip_s    = (uint16_t)(p[29] | (p[30] << 8));
        s_last_status_ms = now_ms();
        s_status_snd = f.sender;
        s_status_rcv = f.receiver;
    } else if (f.cmd_set == BATT_CMDSET && f.cmd_id == BATT_CMDID &&
               f.payload_len > BATT_PCT_OFFSET) {
        uint8_t b = f.payload[BATT_PCT_OFFSET];
        if (b <= 100) {
            s_st.battery_pct = b;
            s_last_batt_ms = now_ms();
        }
    } else if (f.cmd_set == 0x02 && f.cmd_id == CAM_DO_RECORD) {
        /* The camera's answer to a record command.
         *
         * This was silently dropped until an Osmo 360 accepted pairing, streamed
         * status we decode correctly, and then ignored every DoRecord -- leaving
         * no way to tell a rejected command from an unheard one. In DUML the
         * first payload byte of a reply is a return code, 0x00 being success.
         * Logged unconditionally: it is the only direct evidence of WHY a
         * camera will not record. */
        s_last_rec_reply = f.payload_len ? f.payload[0] : 0xFF;
        s_have_rec_reply = true;
    } else {
        /* Anything else this firmware does not model. Kept for bring-up on a
         * camera whose command set differs; recorded rather than logged here
         * because this runs in the BLE callback. */
        s_last_other_set = f.cmd_set;
        s_last_other_id  = f.cmd_id;
        /* Who is actually talking. Our commands are addressed to a fixed
         * DUML device id; if this camera's modules sit at different addresses
         * then every command we send is delivered to nobody, which looks
         * exactly like a command the camera does not implement. */
        s_last_other_snd = f.sender;
        s_last_other_rcv = f.receiver;
        size_t n = f.payload_len < sizeof(s_last_other_pl)
                 ? f.payload_len : sizeof(s_last_other_pl);
        memcpy(s_last_other_pl, f.payload, n);
        s_last_other_pl_len = (uint8_t)n;
        s_other_count++;
    }

    xSemaphoreGive(s_lock);
}

void duml_cam_get(duml_cam_status_t *out)
{
    if (out == NULL) return;
    if (s_proto == CAM_PROTO_GOPRO) {
        /* Facade over the GoPro session: same struct, so the record state
         * machine, the OSD and the console cannot tell which camera it is. */
        gopro_status_t g;
        gopro_cam_get(&g);
        memset(out, 0, sizeof(*out));
        out->link_up       = g.link_up;
        out->status_valid  = g.link_up && g.status_valid;
        out->status_age_ms = g.status_age_ms;
        out->rec_state     = (out->status_valid && g.encoding) ? DUML_REC_RECORDING : DUML_REC_IDLE;
        out->clip_s        = (uint16_t)(g.clip_s > 0xFFFF ? 0xFFFF : g.clip_s);
        out->left_s        = out->status_valid ? g.left_s : 0;
        out->free_mb       = 0;
        out->work_mode     = 1;          /* RECORD: a GoPro is always ready to */
        out->battery_pct   = g.battery_pct;
        out->battery_valid = out->status_valid && g.battery_valid;
        out->hot           = g.hot;
        out->hot_valid     = out->status_valid;
        strlcpy(out->label, g.label[0] ? g.label : s_label, sizeof(out->label));
        strlcpy(out->res, g.res, sizeof(out->res));
        strlcpy(out->fps, g.fps, sizeof(out->fps));
        return;
    }
    if (s_lock == NULL || xSemaphoreTake(s_lock, pdMS_TO_TICKS(20)) != pdTRUE) {
        memset(out, 0, sizeof(*out));
        return;
    }
    *out = s_st;
    strlcpy(out->label, s_label, sizeof(out->label));

    uint32_t t = now_ms();
    out->status_age_ms = (s_last_status_ms == 0) ? UINT32_MAX : (t - s_last_status_ms);
    out->status_valid  = (s_last_status_ms != 0) &&
                         (out->status_age_ms < DUML_CAM_STALE_MS) &&
                         out->link_up;
    out->work_mode = s_st.work_mode;
    out->battery_valid = (s_last_batt_ms != 0) &&
                         ((t - s_last_batt_ms) < 5000) && out->link_up;
    if (!out->status_valid) {
        /* Do not hand back frozen values dressed up as current ones. */
        out->rec_state = DUML_REC_IDLE;
        out->clip_s = 0;
        out->free_mb = 0;
        out->left_s = 0;
    }
    xSemaphoreGive(s_lock);
}

/* What we tell the camera we are, in the R SDK handshake.
 *
 * Both zero, and the zero version is the load-bearing half. Every non-zero
 * value tried -- including a well-formed, current v01.01.06.30 -- told the
 * camera we are a device that HAS firmware and can therefore be serviced, and
 * it responded by putting a firmware-update screen in front of its owner. Zero
 * reads as "no firmware here" rather than "ancient firmware here".
 *
 * Verified on an Osmo 360: no update prompt, and a clip on the card. */
static const uint32_t s_rsdk_device_id  = 0x00000000u;
static const uint32_t s_rsdk_fw_version = 0x00000000u;

/* Map DJI's status push onto the same snapshot the DUML path fills, so the OSD,
 * the record state machine and the CLI cannot tell which protocol produced it. */
static void rsdk_status_cb(void *data)
{
    const camera_status_push_command_frame *f = data;
    if (f == NULL) return;
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(20)) != pdTRUE) return;

    /* 0x03 is photo/video in progress; 0x05 is pre-recording, which is rolling
     * as far as anything downstream is concerned. */
    s_st.rec_state = (f->camera_status == 0x03) ? DUML_REC_RECORDING
                   : (f->camera_status == 0x05) ? DUML_REC_STARTING
                                                : DUML_REC_IDLE;
    s_st.clip_s    = f->record_time;
    s_st.free_mb   = f->remain_capacity;
    s_st.left_s    = f->remain_time;
    s_st.work_mode = f->camera_mode;
    if (f->camera_bat_percentage <= 100) {
        s_st.battery_pct = f->camera_bat_percentage;
        s_last_batt_ms   = now_ms();
    }
    s_last_status_ms = now_ms();
    xSemaphoreGive(s_lock);
}

static void rsdk_new_status_cb(void *data) { (void)data; }

static void rsdk_record_now(bool on)
{
    if (connect_logic_get_state() != PROTOCOL_CONNECTED) {
        ESP_LOGE(TAG, "R SDK: not connected; cannot record");
        return;
    }
    record_control_response_frame_t *r = on ? command_logic_start_record()
                                            : command_logic_stop_record();
    if (r == NULL) {
        ESP_LOGE(TAG, "R SDK: no response to record command");
        return;
    }
    ESP_LOGI(TAG, "R SDK: record %s -> ret_code=%d", on ? "START" : "STOP", r->ret_code);
    free(r);
}

static bool rsdk_session_start(void)
{
    ESP_LOGI(TAG, "R SDK: establishing session");
    ble_set_notify_callback(receive_camera_notify_handler);

    int8_t mac[6] = {0};
    esp_read_mac((uint8_t *)mac, ESP_MAC_BT);

    /* Report a firmware version above anything the camera ships with.
     *
     * The handshake tells the camera what we are, and the first version used
     * here was a placeholder 0x00000001. The camera believed it: a genuine DJI
     * remote reporting 0.0.0.1 is years out of date, so an Osmo 360 offered to
     * firmware-update the module and sat on an update screen waiting for a
     * transfer this firmware does not implement. Identifying ourselves as
     * already-current is what stops that.
     *
     * device_id is deliberately left alone. It is plausibly what makes the
     * handshake acceptable in the first place, and changing both at once would
     * leave us unable to say which mattered. */
    int rc = connect_logic_protocol_connect(s_rsdk_device_id, 6, mac, s_rsdk_fw_version,
                                            0 /* verify_mode */,
                                            0 /* verify_data */,
                                            0 /* camera_reserved */);
    ESP_LOGE(TAG, "RSDK: handshake returned %d (state now %d)",
             rc, (int)connect_logic_get_state());

    /* The version exchange, which this test path had been skipping.
     *
     * DJI's required order, established during the Nano bring-up: handshake,
     * THEN a version query as the first real round trip, THEN the
     * status subscription. Going straight to subscribe leaves that exchange
     * undone -- and an unanswered version negotiation is exactly the shape of
     * a camera that decides its accessory needs updating.
     *
     * The response frame is malloc'd and owned by the caller. */
    version_query_response_frame_t *v = command_logic_get_version();
    if (v) free(v);

    /* Register the status callbacks. These are step 1 of the required
     * sequence: without them the 1D02/1D06 pushes are parsed and then
     * dropped, so the camera is talking to something that never listens. */
    data_register_status_update_callback(rsdk_status_cb);
    data_register_new_status_update_callback(rsdk_new_status_cb);

    subscript_camera_status(2 /* periodic */, 1 /* 2 Hz */);
    ESP_LOGI(TAG, "R SDK: session up");
    return true;

    /* Behave like the accessory we claim to be.
     *
     * DJI's demo is a GPS controller, and its steady state is pushing GPS data.
     * This project strips that, so after the
     * handshake we complete the sequence and then go silent -- a subset of the
     * demo, not the demo. A camera expecting an accessory to act like one and
     * receiving nothing has a reason to decide it is faulty, which is a better
     * explanation for the update prompt than anything tried in the eight
     * previous attempts. */

}


void duml_cam_request_hilight(void)
{
    /* GoPro only. DJI's protocols have no equivalent this project speaks, so a
     * HiLight request on a DJI camera is quietly a no-op rather than an error --
     * the setting simply does nothing on a camera that cannot do it. */
    if (s_proto == CAM_PROTO_GOPRO) gopro_cam_hilight();
}

void duml_cam_request_record(bool on)
{
    if (s_proto == CAM_PROTO_GOPRO) { gopro_cam_request_record(on); return; }
    s_req_value   = on;
    s_req_pending = true;
}

static void send_duml(uint16_t target, uint8_t flags, uint8_t cs, uint8_t ci,
                      const uint8_t *pl, size_t pn)
{
    uint8_t f[96];
    size_t n = duml_build(f, sizeof(f), target, flags, cs, ci, pl, pn);
    if (n) {
        ble_write_raw(s_ble_profile.write_char_handle, f, n, false);
    }
}

static void do_pairing(void)
{
    const uint8_t trigger[2] = { 0x01, 0x00 };
    ble_write_raw(s_ble_profile.notify_char_handle, trigger, sizeof(trigger), true);
    vTaskDelay(pdMS_TO_TICKS(300));

    uint8_t pl[64];
    size_t n = 0;
    n += duml_pack_string(pl + n, sizeof(pl) - n, PAIR_IDENTIFIER);
    n += duml_pack_string(pl + n, sizeof(pl) - n, PAIR_PIN);
    send_duml(DUML_TARGET(DUML_DEV_APP, DUML_DEV_WIFI), DUML_FLAG_REQUEST,
              DUML_CMDSET_WIFI, DUML_CMD_SET_PAIRING_PIN, pl, n);
}

static void cam_task(void *arg)
{
    (void)arg;

    /* The binding decides which BLE stack owner this boot gets, so it is read
     * before either is started. A GoPro hands the radio to gopro_cam and
     * nothing DJI ever runs; the two cannot share the controller. */
    bind_load();
    if (s_proto == CAM_PROTO_GOPRO) {
        gopro_cam_start(s_bound_addr, s_addr_type);
        vTaskDelete(NULL);
        return;
    }

    if (!is_data_layer_initialized()) {
        data_init();
    }
    if (connect_logic_ble_init() != 0) {
        ESP_LOGE(TAG, "BLE init failed");
        vTaskDelete(NULL);
        return;
    }

    for (;;) {
        /* No camera chosen: connect to nothing at all.
         *
         * There is deliberately no fallback here. A module that has never been
         * set up should sit quietly with a dark LED until someone picks a
         * camera in config mode -- not reach out and claim the nearest one. */
        if (!ble_has_bound_addr()) {
            if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(20)) == pdTRUE) {
                s_st.link_up = false;
                xSemaphoreGive(s_lock);
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (connect_logic_get_state() < BLE_CONNECTED) {
            if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(20)) == pdTRUE) {
                s_st.link_up = false;
                xSemaphoreGive(s_lock);
            }
            if (connect_logic_ble_connect(false) != 0) {
                vTaskDelay(pdMS_TO_TICKS(2000));
            }
            continue;
        }

        /* Pairing is DUML on every camera: it is what puts the PIN prompt on
         * the screen, and the Osmo 360 answers it even though it refuses every
         * other DUML command. So pair first, always, then hand over. */
        ble_set_notify_callback(on_notify);
        do_pairing();

        if (s_proto == CAM_PROTO_RSDK) {
            vTaskDelay(pdMS_TO_TICKS(1500));   /* let pairing settle */
            if (!rsdk_session_start()) {
                ESP_LOGE(TAG, "R SDK session failed; dropping the link to retry");
                connect_logic_ble_disconnect();
                continue;
            }
        }

        if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(20)) == pdTRUE) {
            s_st.link_up = true;
            xSemaphoreGive(s_lock);
        }

        while (connect_logic_get_state() >= BLE_CONNECTED) {
            if (s_req_pending) {
                bool on = s_req_value;
                s_req_pending = false;

                /* One entry point, two protocols. Everything upstream -- the
                 * button, the arm switch, the record state machine -- asks for
                 * a recording without knowing which camera is attached. */
                if (s_proto == CAM_PROTO_RSDK) {
                    rsdk_record_now(on);
                    vTaskDelay(pdMS_TO_TICKS(50));
                    continue;
                }

                const uint8_t p[1] = { on ? 0x01 : 0x00 };
                /* WARN: this command changes what the camera is physically
                 * doing. It should never be invisible. */
                ESP_LOGW(TAG, "Do Record %s", on ? "START" : "STOP");
                s_have_rec_reply = false;
                send_duml(DUML_TARGET(DUML_DEV_APP, DUML_DEV_CAMERA),
                          DUML_FLAG_REQUEST, 0x02, CAM_DO_RECORD, p, 1);
            }
            if (s_have_rec_reply) {
                s_have_rec_reply = false;
                ESP_LOGD(TAG, "camera replied to DoRecord: 0x%02X (%s)",
                         s_last_rec_reply,
                         s_last_rec_reply == 0 ? "accepted" : "REJECTED");
            }
            static uint32_t addr_shown;
            if (s_last_status_ms && addr_shown != s_last_status_ms &&
                (s_last_status_ms - addr_shown) > 5000) {
                addr_shown = s_last_status_ms;
                static const char *mode_name[] = {
                    "TAKEPHOTO", "RECORD", "PLAYBACK", "TRANSCODE",
                    "TUNING", "SAVEPOWER", "DOWNLOAD", "NEW_PLAYBACK",
                };
                uint8_t m = s_st.work_mode;
                ESP_LOGD(TAG, "camera work mode = 0x%02X (%s)", m,
                         m < (sizeof(mode_name)/sizeof(mode_name[0]))
                             ? mode_name[m] : "?");
            }
            static uint32_t seen;
            if (s_other_count != seen) {
                seen = s_other_count;
                char hex[3 * sizeof(s_last_other_pl) + 1] = {0};
                for (uint8_t i = 0; i < s_last_other_pl_len; i++) {
                    snprintf(hex + i * 3, 4, "%02X ", s_last_other_pl[i]);
                }
                ESP_LOGD(TAG, "unmodelled snd=0x%02X rcv=0x%02X cmdset=0x%02X cmd=0x%02X [%s] (%u total)",
                         s_last_other_snd, s_last_other_rcv,
                         s_last_other_set, s_last_other_id, hex, (unsigned)seen);
            }
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

void duml_cam_start(void)
{
    s_lock = xSemaphoreCreateMutex();
    memset(&s_st, 0, sizeof(s_st));
    xTaskCreate(cam_task, "dumlcam", 6144, NULL, 4, NULL);
}
