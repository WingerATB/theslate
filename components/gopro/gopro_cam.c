/* SPDX-License-Identifier: PolyForm-Strict-1.0.0
 *
 * Open GoPro BLE session: connect, keep alive, report status, run the shutter.
 */
#include "gopro_cam.h"
#include "gopro_proto.h"
#include "camvendor.h"
#include "ble.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"

#include <string.h>

static const char *TAG = "GOPRO";

/* --------------------------------------------------------------------------
 * The GATT layout
 *
 * Three request/response pairs, all 128-bit under GoPro's own base UUID, all
 * inside the 16-bit service 0xFEA6.
 *
 * THE SERVICE UUID IS NOT IN GOPRO'S BASE. It is a SIG-assigned 16-bit value,
 * so its 128-bit form expands with the standard Bluetooth base
 * (0000fea6-0000-1000-8000-00805f9b34fb) and not with b5f9. GoPro's own
 * documentation lists it in the same table as the GP-XXXX shorthand, which
 * invites exactly the wrong expansion; it is reportedly the single most common
 * mistake people make implementing this. It is declared here as the 16-bit
 * value it actually is, which sidesteps the question.
 * -------------------------------------------------------------------------- */

/* b5f9XXXX-aa8d-11e3-9046-0002a5d5c51b, little-endian as ESP-IDF wants it. */
#define GP_UUID128(xx_hi, xx_lo) { \
    0x1b, 0xc5, 0xd5, 0xa5, 0x02, 0x00, 0x46, 0x90, \
    0xe3, 0x11, 0x8d, 0xaa, (xx_lo), (xx_hi), 0xf9, 0xb5 }

enum { GP_CHAN_CMD = 0, GP_CHAN_SET = 1, GP_CHAN_QRY = 2, GP_CHAN_N = 3 };

static const ble_gatt_profile_t s_gopro_profile = {
    .name    = "GoPro",
    .service = { .len = ESP_UUID_LEN_16, .uuid.uuid16 = GOPRO_SERVICE_UUID16 },
    .chan = {
        [GP_CHAN_CMD] = { .write  = { .len = ESP_UUID_LEN_128,
                                      .uuid.uuid128 = GP_UUID128(0x00, 0x72) },
                          .notify = { .len = ESP_UUID_LEN_128,
                                      .uuid.uuid128 = GP_UUID128(0x00, 0x73) } },
        [GP_CHAN_SET] = { .write  = { .len = ESP_UUID_LEN_128,
                                      .uuid.uuid128 = GP_UUID128(0x00, 0x74) },
                          .notify = { .len = ESP_UUID_LEN_128,
                                      .uuid.uuid128 = GP_UUID128(0x00, 0x75) } },
        [GP_CHAN_QRY] = { .write  = { .len = ESP_UUID_LEN_128,
                                      .uuid.uuid128 = GP_UUID128(0x00, 0x76) },
                          .notify = { .len = ESP_UUID_LEN_128,
                                      .uuid.uuid128 = GP_UUID128(0x00, 0x77) } },
    },
    .n_chan = GP_CHAN_N,
};

/* --------------------------------------------------------------------------
 * State
 *
 * ONE REASSEMBLER PER CHANNEL. The three response characteristics are
 * independent streams and their packets interleave -- a status push can land
 * between the two halves of a command reply. A single reassembler would splice
 * them together and produce a message that never existed.
 * -------------------------------------------------------------------------- */
static gopro_reasm_t     s_reasm[GP_CHAN_N];
static gopro_status_t    s_st;
static SemaphoreHandle_t s_lock;

static volatile bool     s_hw_ok;        /* Get Hardware Info answered       */
static volatile bool     s_cmd_ok;       /* last command answered Success    */
static volatile uint8_t  s_last_cmd;
static uint32_t          s_last_push_ms;
static uint32_t          s_last_ka_ms;
static bool              s_registered;

/* Which of GoPro's two status-id families this camera speaks.
 *
 * Chosen by model, then CONFIRMED by the registration reply: the camera
 * answers in the family it accepted, or refuses, and a refusal sends the
 * other family. s_reg_result is written by the notify callback and polled by
 * the session -- 0 nothing yet, 1 accepted, 2 refused. */
static bool              s_wide;
static volatile uint8_t  s_reg_result;

/* The keep-alive form follows the same logic, but a setting write is only
 * ever answered with a status, so it is judged on its reply rather than
 * chosen up front. Flipped at most twice so two refusals cannot oscillate. */
static bool              s_ka_wide;
static uint8_t           s_ka_sent;
static uint8_t           s_ka_flips;
static volatile uint8_t  s_set_replies;
static volatile bool     s_set_refused;

/* Whether the camera is in a mode where the shutter records video.
 *
 * GoPro's shutter command fires whatever the current preset does. In a photo
 * or timelapse preset that is a still, and the record state machine upstream
 * re-issues an unmet record request every few seconds -- so arming would take
 * a photograph every three seconds for the whole flight, filling the card,
 * while the pilot believes they are filming. The session therefore refuses to
 * run the shutter unless it has established the camera is in video, and
 * reports that refusal so the OSD can say NOT REC on the ground. */
static uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000); }

/* --------------------------------------------------------------------------
 * Sending
 * -------------------------------------------------------------------------- */
static bool gp_send(int chan, const uint8_t *msg, uint16_t n)
{
    uint8_t pkt[24];
    uint16_t len = gopro_pack(pkt, sizeof(pkt), msg, n);
    if (len == 0) {
        ESP_LOGE(TAG, "message too long for one packet (%u B)", (unsigned)n);
        return false;
    }
    esp_err_t e = ble_chan_write(chan, pkt, len);
    if (e != ESP_OK) {
        ESP_LOGW(TAG, "write to channel %d failed: %s", chan, esp_err_to_name(e));
        return false;
    }
    return true;
}

/* Wait for a flag the notify callback sets. Polled rather than blocked on a
 * semaphore because this runs in the camera task, which must stay responsive
 * to a disconnect. */
static bool gp_wait(volatile bool *flag, uint32_t ms)
{
    /* Signed difference, so this stays correct across the 32-bit millisecond
     * wrap rather than returning instantly for 49 days. */
    uint32_t start = now_ms();
    while ((int32_t)(now_ms() - start) < (int32_t)ms) {
        if (*flag) return true;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return false;
}

/* --------------------------------------------------------------------------
 * Receiving
 * -------------------------------------------------------------------------- */
void gopro_cam_on_notify(uint16_t handle, const uint8_t *data, size_t len)
{
    int chan = ble_chan_of_notify_handle(handle);
    if (chan < 0 || chan >= GP_CHAN_N) {
        /* The blanket subscribe sweep in the transport picks up whatever else
         * the camera exposes. Not ours: ignore it rather than feeding it to a
         * reassembler that would then be out of step. */
        return;
    }

    uint16_t mlen = 0;
    const uint8_t *msg = gopro_reasm_feed(&s_reasm[chan], data, (uint16_t)len, &mlen);
    if (msg == NULL) return;

    if (chan == GP_CHAN_QRY) {
        /* The registration reply, in whichever family: the camera's verdict on
         * the family we asked in. Read before the lock, because the session
         * is polling for it and needs no status to have been applied. */
        if (mlen >= 2 && (msg[0] == GOPRO_Q_REG_STATUS || msg[0] == GOPRO_Q_REG_STATUS16)) {
            s_reg_result = (msg[1] == GOPRO_ST_SUCCESS) ? 1 : 2;
            if (msg[1] != GOPRO_ST_SUCCESS) {
                ESP_LOGW(TAG, "status registration 0x%02X refused, status %u", msg[0], msg[1]);
            }
        }

        /* This runs on the Bluedroid callback task, which must not be held up.
         * The wait is therefore zero, not a timeout -- but a status push that
         * was simply dropped would be lost for good on a camera that only
         * sends changes, so on contention it retries briefly rather than
         * discarding. Contention is between this and gopro_cam_get(), which
         * holds the lock for a handful of assignments. */
        for (int try = 0; try < 3; try++) {
            if (s_lock && xSemaphoreTake(s_lock, 0) == pdTRUE) {
                int applied = gopro_status_apply(&s_st, msg, mlen);
                if (applied > 0) s_last_push_ms = now_ms();
                xSemaphoreGive(s_lock);
                return;
            }
            vTaskDelay(1);
        }
        ESP_LOGW(TAG, "dropped a status push: could not take the lock");
        return;
    }

    if (chan == GP_CHAN_SET) {
        /* A settings reply is the only thing that says whether the keep-alive
         * form was right. Counted as well as judged: a camera that never
         * answers at all is also a camera in the wrong form. */
        uint16_t id = 0; uint8_t status = 0;
        if (gopro_setting_reply(msg, mlen, &id, &status)) {
            s_set_replies++;
            if (status != GOPRO_ST_SUCCESS) {
                s_set_refused = true;
                ESP_LOGW(TAG, "setting 0x%04X refused, status %u", id, status);
            }
        }
        return;
    }

    /* Command replies are [id][status]. */
    if (mlen >= 2) {
        uint8_t id = msg[0], status = msg[1];
        if (chan == GP_CHAN_CMD) {
            s_last_cmd = id;
            s_cmd_ok = (status == GOPRO_ST_SUCCESS);
            if (id == GOPRO_CMD_HW_INFO && status == GOPRO_ST_SUCCESS) {
                s_hw_ok = true;
            }
            if (status != GOPRO_ST_SUCCESS) {
                ESP_LOGW(TAG, "command 0x%02X refused, status %u", id, status);
            }
        }
    }
}

/* --------------------------------------------------------------------------
 * Session
 * -------------------------------------------------------------------------- */
void gopro_cam_select(void)
{
    ble_register_vendor_profile(CAM_VENDOR_GOPRO, &s_gopro_profile);
}

/* Register for the statuses in one family and wait for the camera's answer.
 * True only on an explicit Success; a refusal or silence is false. */
static bool gp_register(bool wide)
{
    uint8_t msg[24];
    uint16_t n = gopro_status_reg_build(msg, sizeof(msg), wide);
    if (n == 0) return false;
    s_reg_result = 0;
    if (!gp_send(GP_CHAN_QRY, msg, n)) return false;
    uint32_t start = now_ms();
    while ((int32_t)(now_ms() - start) < 1500) {
        if (s_reg_result != 0) break;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    ESP_LOGI(TAG, "status registration, %s ids: %s", wide ? "2-byte" : "1-byte",
             s_reg_result == 1 ? "accepted" : s_reg_result == 2 ? "REFUSED" : "no answer");
    return s_reg_result == 1;
}

bool gopro_cam_session_start(uint8_t model)
{
    if (s_lock == NULL) s_lock = xSemaphoreCreateMutex();
    if (s_lock == NULL) {
        /* Every status read takes this lock and returns nothing without it, so
         * a camera would connect and report absolutely nothing, for ever, with
         * no explanation. Refuse the session instead. */
        ESP_LOGE(TAG, "could not create the session lock");
        return false;
    }

    for (int i = 0; i < GP_CHAN_N; i++) gopro_reasm_reset(&s_reasm[i]);
    memset(&s_st, 0, sizeof(s_st));
    s_hw_ok = false;
    s_registered = false;
    s_last_push_ms = 0;
    s_last_ka_ms = 0;
    s_wide = cam_gopro_wide_ids(model);
    s_reg_result = 0;
    s_ka_sent = 0;
    s_ka_flips = 0;
    s_set_replies = 0;
    s_set_refused = false;

    ble_set_notify_handle_callback(gopro_cam_on_notify);

    /* Every response characteristic must be subscribed before a single command
     * goes out. A GoPro answers a command written before its command-response
     * channel is live by simply not answering: the reply is dropped, nothing
     * reports an error, and the session waits for something that will never
     * arrive. */
    /* Generous, because this also covers a camera that was ASLEEP when the
     * scan chose it: connecting wakes it, and a GoPro takes several seconds to
     * boot before it will subscribe anything. */
    uint32_t start = now_ms();
    while (!ble_channels_ready() && (int32_t)(now_ms() - start) < 15000) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (!ble_channels_ready()) {
        /* Named channel by channel, because this is the failure the bench
         * cannot reproduce and the log is all anybody will have. A channel
         * with no handle means the camera's GATT does not carry that
         * characteristic -- an older HERO, or one that is not really speaking
         * Open GoPro. A channel with a handle but no acknowledgement means it
         * is there and did not answer, which is a pairing or a timing fault. */
        static const char *k_chan_name[GP_CHAN_N] = { "command", "settings", "query" };
        for (int i = 0; i < GP_CHAN_N; i++) {
            if (ble_chan_subscribed(i)) continue;
            ESP_LOGE(TAG, "channel %d (%s) not usable: notify=0x%04x write=0x%04x -- %s",
                     i, k_chan_name[i], ble_chan_notify_handle(i),
                     ble_chan_write_handle(i),
                     ble_chan_notify_handle(i) == 0
                         ? "the camera does not expose this characteristic"
                         : "found, but it never acknowledged its CCCD");
        }
        ESP_LOGE(TAG, "not all channels subscribed -- cannot talk to this camera");
        return false;
    }

    /* Get Hardware Info doubles as GoPro's documented readiness probe: retry
     * it until it answers Success, and only then is the camera listening. */
    const uint8_t hw[] = { GOPRO_CMD_HW_INFO };
    for (int attempt = 0; attempt < 10 && !s_hw_ok; attempt++) {
        gp_send(GP_CHAN_CMD, hw, sizeof(hw));
        gp_wait(&s_hw_ok, 500);
    }
    if (!s_hw_ok) {
        ESP_LOGE(TAG, "camera never became ready");
        return false;
    }
    ESP_LOGI(TAG, "camera ready");

    /* Subscribe to the statuses the OSD needs, in one write -- in the family
     * the model suggests, then the other if the camera says no. The reply
     * carries the current value of every status, so when this returns the
     * first OSD frame already has real numbers in it. */
    if (!gp_register(s_wide)) {
        ESP_LOGW(TAG, "trying the other status-id family");
        if (gp_register(!s_wide)) {
            s_wide = !s_wide;
        } else {
            /* Keep the session: the shutter still works, and the OSD will say
             * plainly that nothing is confirmed rather than inventing it. */
            ESP_LOGE(TAG, "the camera accepted neither status family -- "
                          "recording will run, the OSD cannot confirm it");
        }
    }
    s_registered = true;
    s_ka_wide = s_wide;

    s_last_ka_ms = now_ms();
    return true;
}

void gopro_cam_session_end(void)
{
    ble_set_notify_handle_callback(NULL);
    s_registered = false;
    s_hw_ok = false;
    if (s_lock && xSemaphoreTake(s_lock, pdMS_TO_TICKS(20)) == pdTRUE) {
        memset(&s_st, 0, sizeof(s_st));
        s_last_push_ms = 0;
        if (s_lock) xSemaphoreGive(s_lock);
    }
}

void gopro_cam_tick(void)
{
    if (!s_registered) return;

    /* The keep-alive, every three seconds.
     *
     * Not optional and not a nicety: without it the camera's own timer sleeps
     * the link. It is a SETTING write -- setting 0x5B with the magic value
     * 0x42 -- rather than a command, which is how both of GoPro's own SDKs
     * encode it. */
    uint32_t t = now_ms();
    if ((t - s_last_ka_ms) >= GOPRO_KEEPALIVE_MS) {
        /* Judge the form on what the camera said to the last ones. A refusal
         * is explicit; two sends with no reply at all is the same verdict said
         * quietly. Either way, switch -- but at most twice, so a camera that
         * refuses both forms is not asked alternately for ever. */
        bool flip = false;
        if (s_set_refused) { flip = true; s_set_refused = false; }
        else if (s_ka_sent >= 2 && s_set_replies == 0) flip = true;
        if (flip && s_ka_flips < 2) {
            s_ka_wide = !s_ka_wide;
            s_ka_flips++;
            s_ka_sent = 0;
            ESP_LOGW(TAG, "keep-alive not acknowledged; switching to the %s form",
                     s_ka_wide ? "2-byte" : "1-byte");
        }

        uint8_t ka[8];
        uint16_t n = gopro_keepalive_build(ka, sizeof(ka), s_ka_wide);
        if (n && gp_send(GP_CHAN_SET, ka, n)) s_ka_sent++;
        s_last_ka_ms = t;
    }
}


void gopro_cam_request_record(bool on)
{
    /* Refuse to start unless the camera is in video.
     *
     * See the note on s_video_mode: the shutter fires whatever the preset
     * does, and upstream retries an unmet request, so starting in a photo
     * preset means a still every few seconds for the whole flight. Stopping is
     * always allowed -- the asymmetry is the same one the manual override
     * uses, and for the same reason: refusing to stop can only ever leave a
     * camera running. */
    if (on) {
        bool blocked = false;
        if (s_lock && xSemaphoreTake(s_lock, pdMS_TO_TICKS(20)) == pdTRUE) {
            /* Blocked only by a group we KNOW takes stills. An unrecognised
             * group is allowed through: a whitelist of one would turn every
             * future preset group GoPro invents into a camera that refuses to
             * record for the whole flight, which is a far worse failure than
             * the one this guard exists to prevent. */
            blocked = s_st.preset_valid &&
                      (s_st.preset_group == GOPRO_PRESET_PHOTO ||
                       s_st.preset_group == GOPRO_PRESET_TIMELAPSE);
            xSemaphoreGive(s_lock);
        }
        if (blocked) {
            ESP_LOGW(TAG, "not starting: the camera is in a photo or timelapse "
                          "preset, so the shutter would take stills");
            return;
        }
    }

    const uint8_t msg[] = { GOPRO_CMD_SHUTTER, 0x01, (uint8_t)(on ? 1 : 0) };
    s_cmd_ok = false;
    if (gp_send(GP_CHAN_CMD, msg, sizeof(msg))) {
        ESP_LOGW(TAG, "shutter %s", on ? "START" : "STOP");
    }
}

void gopro_cam_get(gopro_cam_status_t *out)
{
    if (out == NULL) return;
    memset(out, 0, sizeof(*out));

    if (s_lock == NULL) return;
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(20)) != pdTRUE) return;

    uint32_t age = s_last_push_ms ? (now_ms() - s_last_push_ms) : 0xFFFFFFFFu;

    out->link_up      = s_registered;
    out->status_age_ms = age;
    /* Valid once the camera has told us anything at all, and for as long as
     * the link holds.
     *
     * NOT aged out on a timer, unlike the DUML side. A GoPro pushes only when
     * a value changes, so a camera that is switched on, idle and perfectly
     * healthy sends nothing for minutes -- and any timeout short enough to
     * catch a wedged camera would blank a working one first. What actually
     * fails here is the link, and BLE reports that within seconds; the session
     * ends and link_up goes false, which is the honest signal. */
    out->status_valid = (s_last_push_ms != 0) && s_registered;

    out->recording   = s_st.encoding_valid && s_st.encoding;
    out->clip_s      = (uint16_t)(s_st.clip_valid ? s_st.clip_s : 0);
    out->clip_valid  = s_st.clip_valid;
    out->left_s      = s_st.left_valid ? s_st.left_s : 0;
    out->left_valid  = s_st.left_valid;
    out->battery_pct = s_st.battery;
    out->battery_valid = s_st.battery_valid;

    /* Overheating is a BOOLEAN on a GoPro and it can be true while the camera
     * is still happily recording. It is reported as-is here and mapped
     * upstream, rather than being promoted to "cannot record" -- doing that
     * would put a blinking CAM HOT over a working recording and invite the
     * pilot to abort a good pack. */
    out->hot = s_st.overheat_valid && s_st.overheat;

    /* Anything other than OK means no recording can start. Reported as a fault
     * in its own right rather than by suppressing the time-remaining reading:
     * blanking the reading would ALSO silence the warning, which is exactly
     * backwards for the case the warning exists for. */
    out->card_fault = s_st.storage_valid && (s_st.storage != GOPRO_SD_OK);

    /* Whether the shutter would produce video.
     *
     * Unknown means ALLOWED, deliberately: if the camera never reports its
     * preset group, refusing would turn a camera that works today into one
     * that never records. The guard exists to catch a camera we can SEE is in
     * the wrong mode, not to demand proof it is in the right one. */
    out->can_record = !s_st.preset_valid ||
                      (s_st.preset_group != GOPRO_PRESET_PHOTO &&
                       s_st.preset_group != GOPRO_PRESET_TIMELAPSE);

    xSemaphoreGive(s_lock);
}
