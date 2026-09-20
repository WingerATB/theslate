/* SPDX-License-Identifier: PolyForm-Strict-1.0.0
 *
 * Record state machine, camera status aggregation, and OSD reporting.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "camlink.h"
#include "camlink_warn.h"
#include "msp.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "duml_cam.h"
#include "camlink_cfg.h"
#include "ble.h"
#include "esp_bt.h"
#include "board.h"

static const char *TAG = "SLATE";

/* Byte offsets (end-of-field) within the packed 1D02 frame. A field is only
 * real if the camera sent at least this many bytes. Verified by compiling the
 * struct and printing offsetof(); see README. */
#define OFF_END_CAMERA_STATUS   2
#define OFF_END_RECORD_TIME     7
#define OFF_END_REMAIN_TIME    27
#define OFF_END_TEMP_OVER      31
#define OFF_END_BATTERY        38

/* How stale a camera push may get before we stop believing it. The camera
 * pushes at 2 Hz, so 3 s is ~6 missed pushes. */
#define CAM_STATUS_STALE_MS  3000

/* Do not re-issue a record command more often than this.
 *
 * It must exceed the time the camera takes to REFLECT a command in its status,
 * or every command is followed by a redundant one. Measured on an Osmo 360
 * (testlogs/rec-stop-5cycles.log): a start shows up in ~0.2 s, but a stop takes
 * ~1.65 s -- the camera is finalising the clip on the card before it will admit
 * it has stopped. At 1500 ms that produced a second STOP on all five of five
 * stops, always rejected with ret_code 223, and always aimed at a camera in the
 * middle of closing a file. 3 s clears the measured worst case with margin; the
 * cost is that a genuinely lost command takes 3 s rather than 1.5 s to retry,
 * which is nothing next to the camera's own shutter delay. */
#define RECORD_RETRY_MS      3000

/* How long a cut waits for the camera to finish closing the old clip before
 * starting the new one anyway.
 *
 * The camera saying "not recording" is the signal this waits for; the deadline
 * is for the camera that goes quiet instead of answering. An Osmo Nano stops
 * servicing BLE for over two seconds while it finalises, so it is the one that
 * needs the fallback, and 2.5 s clears its measured stall. Longer would be a
 * gap in the footage; shorter would fire into the file close, which is the
 * mistake this replaced. */
#define CUT_RESTART_MAX_MS   2500

/* How often every OSD slot is rewritten whether or not it changed. See the note
 * at the write itself: the FC can forget, and nothing tells us when it has. */
#define OSD_REFRESH_MS       1000

static cam_status_t      s_cam;
static SemaphoreHandle_t s_cam_lock;
static camlink_config_t  s_cfg;

/* Record request handoff: the logic task decides, the camera task executes.
 * The DJI send_command() path blocks for up to 5 s, which must never happen
 * inside the task that also refreshes the OSD. */
static volatile bool s_req_pending;
static volatile bool s_req_value;
static volatile bool s_cut_pending;

/* Manual record intent.
 *
 * s_press is a one-shot event posted by the front-panel button and consumed by
 * the logic task. File scope rather than a task local so a press that lands
 * before the task starts is not lost, and volatile because two tasks touch it.
 *
 * s_manual_record is what the override currently wants, kept only so
 * camlink_get_manual_record() can answer. */
/* What the setup control is doing. Written by the task that watches the setup
 * channel, read by the one that draws the OSD. */
static volatile camlink_setup_hint_t s_setup_hint = CAMLINK_SETUP_NONE;

void camlink_set_setup_hint(camlink_setup_hint_t hint)
{
    s_setup_hint = hint;
}

static volatile bool s_press;
static volatile bool s_manual_record;

/* Last OSD frame, for the bench readout. Written only by the logic task. */
static char s_osd[4][17];

/* The warning in force. Written by the logic task, read by the LED task. */
static volatile camlink_warn_t s_warn = CAM_WARN_NONE;

camlink_warn_t camlink_get_warn(void) { return s_warn; }

void camlink_get_osd(char out[4][17])
{
    if (out == NULL) return;
    memcpy(out, s_osd, sizeof(s_osd));
}

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

/* ------------------------------------------------------------------------- */
/* Camera status                                                              */
/* ------------------------------------------------------------------------- */

/* Pull the latest camera state from the DUML session and republish it as
 * cam_status_t for the OSD and the record state machine.
 *
 * The Osmo Nano speaks DUML, not the DJI R SDK protocol.
 * Validity comes straight from duml_cam, which
 * already applies the staleness rule: the camera can stop pushing its status
 * channel while BLE stays up, and a frozen reading must never be presented as
 * current. */
void camlink_poll_camera(void)
{
    duml_cam_status_t d;
    duml_cam_get(&d);

    if (s_cam_lock == NULL) return;
    if (xSemaphoreTake(s_cam_lock, pdMS_TO_TICKS(20)) != pdTRUE) return;

    s_cam.connected = d.link_up;

    s_cam.recording       = (d.rec_state == DUML_REC_RECORDING);
    s_cam.recording_valid = d.status_valid;

    /* Per field, not per frame. A camera that is talking is not necessarily
     * talking about the card -- and a card time of zero that the camera never
     * sent would read as NO SD on a perfectly good card. */
    s_cam.record_time_s     = d.clip_s;
    s_cam.record_time_valid = d.status_valid && d.clip_valid;

    s_cam.remain_time_s     = d.left_s;
    s_cam.remain_time_valid = d.status_valid && d.left_valid;

    s_cam.battery_pct   = d.battery_pct;
    s_cam.battery_valid = d.battery_valid;

    s_cam.card_fault       = d.card_fault;
    s_cam.card_fault_valid = d.status_valid;

    /* Which camera is bound is known whether or not it is answering, so this
     * is copied unconditionally -- unlike every reading around it. */
    memcpy(s_cam.label, d.label, sizeof(s_cam.label));

    /* DUML exposes no overheat field, so on a DJI camera this stays
     * unconfirmed rather than being faked as "temperature normal". A GoPro
     * does report it, as a plain boolean.
     *
     * Mapped to level 1 while recording and level 2 while not. The ladder
     * treats 2 and above as "cannot record", and a GoPro's flag can be true
     * while it is filming perfectly happily -- so promoting it unconditionally
     * would blink CAM HOT over a working recording and invite the pilot to
     * abort a good pack. */
    if (d.hot) {
        s_cam.temp_over       = (d.rec_state == DUML_REC_RECORDING) ? 1 : 2;
        s_cam.temp_over_valid = d.status_valid;
    } else {
        s_cam.temp_over       = 0;
        s_cam.temp_over_valid = false;
    }

    if (d.status_valid) {
        s_cam.last_update_ms = now_ms();
    }

    xSemaphoreGive(s_cam_lock);
}

void camlink_set_connected(bool connected)
{
    if (s_cam_lock == NULL) return;
    if (xSemaphoreTake(s_cam_lock, pdMS_TO_TICKS(20)) != pdTRUE) return;

    s_cam.connected = connected;
    if (!connected) {
        /* Drop every field: nothing we knew is trustworthy any more. */
        s_cam.recording_valid   = false;
        s_cam.record_time_valid = false;
        s_cam.remain_time_valid = false;
        s_cam.battery_valid     = false;
        s_cam.temp_over_valid   = false;
        s_cam.card_fault_valid  = false;
    }
    xSemaphoreGive(s_cam_lock);
}

void camlink_get_cam_status(cam_status_t *out)
{
    if (out == NULL) return;
    if (s_cam_lock && xSemaphoreTake(s_cam_lock, pdMS_TO_TICKS(20)) == pdTRUE) {
        *out = s_cam;

        /* Age out stale data even if we still think we are connected. */
        if (out->last_update_ms == 0 ||
            (now_ms() - out->last_update_ms) > CAM_STATUS_STALE_MS) {
            out->recording_valid   = false;
            out->record_time_valid = false;
            out->remain_time_valid = false;
            out->battery_valid     = false;
            out->temp_over_valid   = false;
        }
        xSemaphoreGive(s_cam_lock);
    } else {
        memset(out, 0, sizeof(*out));
    }
}

/* ------------------------------------------------------------------------- */
/* Record state machine                                                       */
/* ------------------------------------------------------------------------- */

void camlink_logic_init(const camlink_config_t *cfg)
{
    s_cam_lock = xSemaphoreCreateMutex();
    memset(&s_cam, 0, sizeof(s_cam));
    s_cfg = *cfg;
    camlink_cfg_init();
}

/* Raw microseconds on the configured channel, or 0 if there is nothing to
 * read. Zero is safe as "no reading" -- a receiver never produces it. */
static uint16_t switch_us(const msp_state_t *m, const camlink_cfg_t *cfg)
{
    if (!m->rc_valid || cfg->switch_channel >= m->rc_count) {
        return 0;
    }
    return m->rc[cfg->switch_channel];
}

static bool switch_is_high(const msp_state_t *m, const camlink_cfg_t *cfg)
{
    if (!m->rc_valid || cfg->switch_channel >= m->rc_count) {
        return false;
    }
    /* rcData[] straight from the receiver, in microseconds and NOT confined to
     * 1000-2000 -- see the scale note in camlink_cfg.h. The window test lives
     * there too, so the firmware and the settings page cannot disagree about
     * where a switch has to sit. */
    return camlink_cfg_in_window(m->rc[cfg->switch_channel], cfg);
}

void camlink_set_manual_record(bool on)
{
    /* Kept for the bench CLI, which asks for a state rather than a toggle.
     * Expressed as a press so there is exactly one path into the override:
     * two ways to set the same intent is how they drift apart. */
    if (on != s_manual_record) s_press = true;
}

void camlink_press_record(void)
{
    s_press = true;
}

bool camlink_get_manual_record(void)
{
    return s_manual_record;
}

void camlink_logic_task(void *arg)
{
    (void)arg;

    /* Has a flight controller EVER answered? This distinguishes two situations
     * that otherwise look identical (armed == false) but need opposite
     * behaviour:
     *
     *   never seen an FC   -> bench. The state machine must stand down and let
     *                         the button own recording, otherwise it stops a
     *                         manually started clip within one retry period.
     *   seen one, now gone -> in flight with a dead UART. The brief's rule
     *                         applies: force disarmed so the clip is closed
     *                         rather than left running forever.
     */
    bool ever_had_fc = false;

    bool     prev_switch = false;
    bool     switch_seen = false;   /* have we ever had a valid RC reading?   */
    camlink_press_t press_state = {0};   /* BUTTON mode; see camlink_cfg.h    */
    camlink_ovr_t   ovr         = {0};   /* the manual override               */
    bool     auto_prev   = false;        /* last tick's automatic intent      */
    bool     auto_seen   = false;
    bool     armed_ref   = false;        /* armed, sampled only with a live link */
    bool     rc_latch    = false;        /* SWITCH+BUTTON's own latch         */
    bool     unsent      = false;        /* an intent not yet delivered       */
    uint8_t  tx_now     = 0;             /* level currently applied           */
    bool     tx_applied = false;
    bool     want        = false;
    bool     prev_want   = false;        /* last tick's effective intent      */
    camlink_stopdelay_t sd = {0};        /* post-roll                        */
    uint32_t unmet_since = 0;            /* when the want first went unmet   */
    bool     unmet_seen  = false;
    char     prev[4][17] = {{0}};
    bool     first_push  = true;
    uint32_t last_full_push = 0;
    bool     prev_link_up   = false;

    for (;;) {
        camlink_poll_camera();

        msp_state_t m;
        msp_get_state(&m);

        cam_status_t cam;
        camlink_get_cam_status(&cam);

        /* Read config every pass: changing mode or channel takes effect
         * immediately, with no reboot. */
        camlink_cfg_t cfg;
        camlink_cfg_get(&cfg);

        /* Automatic transmit power.
         *
         * The two situations the radio has to serve want opposite things, which
         * is why one fixed level has always been a compromise. Setting up on the
         * bench, the camera may be a room away and the module wants to be loud.
         * In flight it sits centimetres from the camera and millimetres from the
         * RC receiver, where being loud buys nothing and costs the link that
         * actually matters.
         *
         * Armed is the cleanest signal available for "in flight now", so this
         * follows it: the chosen level while disarmed, the quietest level while
         * armed. Applied only on a transition -- it is called on every pass and
         * the radio does not need telling four times a second.
         *
         * Not applied when no FC has ever answered: on the bench, arm state is
         * meaningless and turning the radio down would fight the setup this is
         * meant to help. */
        if (cfg.tx_auto && ever_had_fc) {
            uint8_t want_tx = m.armed ? CFG_TX_QUIETEST : cfg.tx_power;
            if (!tx_applied || want_tx != tx_now) {
                camlink_ble_apply_tx_level(board_tx_level(want_tx));
                ESP_LOGI(TAG, "BLE tx -> %s (%s)", board_tx_name(want_tx),
                         m.armed ? "armed" : "disarmed");
                tx_now = want_tx;
                tx_applied = true;
            }
        }

        bool sw = switch_is_high(&m, &cfg);
        if (!m.rc_valid) {
            /* No RC data: hold the last known switch position rather than
             * inventing a low edge that would cut a clip. */
            sw = prev_switch;
        } else if (!switch_seen) {
            /* First valid reading: adopt it without treating it as an edge. */
            prev_switch = sw;
            switch_seen = true;
        }

        /* One "press", however the channel is wired.
         *
         * LEVEL treats a rising edge into the window as the press, which is
         * what a toggle switch produces. BUTTON treats any movement of the
         * channel as the press, because a momentary button leaves the value
         * wherever it put it: it never returns, so a second press produces
         * another change but never another edge, and level logic sees nothing.
         * That is the "you have to press it twice" fault, and it is not a
         * tuning problem -- the two are different questions and need different
         * code. */
        bool rising = sw && !prev_switch;
        bool press  = rising;

        if (cfg.switch_kind == CFG_SW_BUTTON) {
            press = camlink_press_update(&press_state, switch_us(&m, &cfg));
        }

        if (m.link_up) {
            ever_had_fc = true;
        }

        /* m.armed is already forced false by the MSP layer when the link is
         * down, so a dead UART closes the clip here without extra handling --
         * but only once an FC has actually been seen. */
        /* An automatic source is "live" when the thing that drives it can be
         * heard from. On a bench where no FC has ever answered, nothing is
         * missing and the button is the only source -- so authority is true
         * and the override simply owns recording, which is what this module
         * has always done there. */
        bool authority = !ever_had_fc || m.link_up;

        /* Arm state, sampled only while the link is up.
         *
         * msp.c forces armed=false the moment the link goes quiet, so reading
         * it raw would turn every brief UART gap into an arm edge and back --
         * two fabricated movements of the automatic source that would release
         * an override the user had just set. Holding the last value seen while
         * the link was actually up makes a dropout a non-event, and the
         * authority test above is what handles a dropout instead. */
        if (m.link_up && m.boxarm_known) armed_ref = m.armed;

        /* What the automatic source is asking for, read from the HELD arm
         * state -- the raw intent, before the question of whether that source
         * can still be heard from. */
        bool auto_want_raw = ever_had_fc
                           ? camlink_auto_want(cfg.mode, cfg.switch_kind,
                                               armed_ref, sw, rc_latch)
                           : false;

        /* ...and a source that cannot be heard from is not asking for anything.
         *
         * Holding armed_ref through a dropout is right -- it is what stops a
         * brief UART gap from fabricating two arm edges. But it also meant the
         * automatic intent stayed RECORD on a flight controller that had gone
         * silent, so a cut UART left the clip running: authority only ever
         * demoted an ACTIVE override, and with no override the arbiter falls
         * straight back to auto_want. Five places say a dead UART closes an
         * open clip, including the README and the timeout constant's own
         * justification; this is the line that makes that true.
         *
         * Gating the INTENT while leaving the MOVEMENT (below) computed from
         * the raw value is the whole trick: the clip stops, no edge is
         * fabricated, no override is released, and a link that comes back
         * still armed resumes recording. */
        bool auto_want = auto_want_raw && authority;

        /* In SWITCH+BUTTON the RC press drives the mode's own latch. In BOTH it
         * is a manual press like the front-panel button. Everywhere else it is
         * either a cut (CUT) or nothing. */
        bool manual_press = s_press;
        s_press = false;
        if (ever_had_fc && press) {
            if (cfg.mode == CFG_MODE_SWITCH && cfg.switch_kind == CFG_SW_BUTTON) {
                rc_latch = !rc_latch;
            } else if (cfg.mode == CFG_MODE_BOTH) {
                manual_press = true;
            }
        }

        camlink_ovr_in_t oin = {
            .press         = manual_press,
            .auto_want     = auto_want,
            /* Movement is measured on the RAW intent, so a UART dropout -- which
             * gates auto_want without the switch or the arm state having moved
             * -- is not mistaken for the pilot moving something. */
            .auto_moved    = auto_seen && (auto_want_raw != auto_prev),
            .authority     = authority,
            .cam_connected = cam.connected,
            .cam_valid     = cam.recording_valid,
            .cam_recording = cam.recording,
            .unsent        = unsent,
        };
        /* An automatic STOP specifically, not merely a move -- captured here
         * because auto_prev is about to be overwritten and the answer needs
         * both values.
         *
         * AND the module has to have been recording for a stop to be worth
         * padding. Without that second half, a pilot who stops the clip with
         * the button while still armed and then disarms gets a post-roll hold
         * latched onto an idle camera: the disarm releases the override, raises
         * the stop edge, and the hold starts a clip nobody asked for and then
         * records it on the ground. Post-roll extends a recording; it must not
         * be able to begin one. */
        bool auto_stop = oin.auto_moved && !oin.auto_want && prev_want;

        auto_prev = auto_want_raw;
        auto_seen = true;

        want = camlink_ovr_update(&ovr, &oin);
        s_manual_record = ovr.active && ovr.want;

        /* Post-roll, layered on top of the intent rather than inside it.
         *
         * It sits AFTER the override deliberately. The override is a person
         * pressing a button; this is a guess about what they would have wanted
         * had the aircraft not just hit the ground. A guess must not be able to
         * overrule the person, which is what `overridden` says. */
        uint8_t  sd_remain = 0;
        uint32_t tnow = now_ms();
        camlink_stopdelay_in_t sin = {
            .auto_stop     = auto_stop,
            .want          = want,
            .overridden    = ovr.active,
            .authority     = authority,
            .cam_connected = cam.connected,
            .delay_s       = cfg.stop_delay_s,
            .now_ms        = tnow,
        };
        want = camlink_stopdelay_update(&sd, &sin, &sd_remain);

        /* The effective intent this tick, for the next tick to consult. `want`
         * is final here -- everything below only reads it. */
        prev_want = want;
        if (sd_remain) {
            ESP_LOGD(TAG, "stop delayed, %u s left", (unsigned)sd_remain);
        }

        /* How long the camera has disagreed with us.
         *
         * Timed here rather than inside camlink_warn_pick(), which stays pure
         * and testable at a desk. Reset the moment the disagreement ends, so a
         * camera that starts late clears the warning rather than latching it. */
        if (want && cam.recording_valid && !cam.recording) {
            if (!unmet_seen) { unmet_since = tnow; unmet_seen = true; }
        } else {
            unmet_seen = false;
        }
        bool unmet_long = unmet_seen &&
                          (tnow - unmet_since) >= CAMLINK_NOT_REC_GRACE_MS;

        camlink_warn_in_t win = {
            .cam_gone        = !cam.connected || !cam.recording_valid,
            .want_record     = want,
            .rec_valid       = cam.recording_valid,
            .recording       = cam.recording,
            .want_unmet_long = unmet_long,
            .temp_valid      = cam.temp_over_valid,
            .temp_over       = cam.temp_over,
            .card_valid      = cam.remain_time_valid,
            .card_s          = cam.remain_time_s,
            .card_fault      = cam.card_fault_valid && cam.card_fault,
            .batt_valid      = cam.battery_valid,
            .batt_pct        = cam.battery_pct,
            .warn_batt_pct   = cfg.warn_batt_pct,
            .warn_card_s     = (uint16_t)(cfg.warn_card_min * 60u),
        };
        camlink_warn_t warn = camlink_warn_pick(&win);
        s_warn = warn;

        /* CUT: a press while armed ends the take and starts a fresh clip. The
         * queued cut is dropped whenever the intent is not to record, so an
         * override that says stop cannot be overtaken by a cut that says
         * start. */
        if (ever_had_fc && cfg.mode == CFG_MODE_CUT && press && armed_ref && want) {
            s_cut_pending = true;
            ESP_LOGI(TAG, "cut requested: restarting clip");
        }
        if (!want) s_cut_pending = false;

        prev_switch = sw;

        /* Publish the desired state for the camera task.
         *
         * When the camera confirms its state we retry until reality matches.
         * When it does NOT confirm (connected but not reporting, or a status
         * frame too short to include camera_status) we send once per change of
         * intent instead of retrying forever -- otherwise an unconfirmed
         * camera would draw a command every RECORD_RETRY_MS for the whole
         * flight, for nothing. */
        static uint32_t last_req = 0;
        static bool have_sent = false;
        static bool last_sent_want = false;
        /* Second half of a cut: the STOP is out and the fresh clip is owed. */
        static bool     cut_restart    = false;
        static uint32_t cut_restart_by = 0;
        uint32_t t = now_ms();

        bool need = cam.recording_valid ? (cam.recording != want)
                                        : (!have_sent || last_sent_want != want);

        /* What the override needs to know next tick: is a change of intent
         * still waiting to go out? While it is, a press must be absorbed
         * rather than inverting an intent that has not been acted on yet. */
        unsent = need && cam.connected && (t - last_req) < RECORD_RETRY_MS;

        if (!cam.connected) {
            /* Nothing can be delivered, and a cut queued now would fire as a
             * surprise on reconnect. Drop it and re-arm on the next edge. */
            s_cut_pending = false;
            cut_restart   = false;
            have_sent = false;
        } else if (cut_restart) {
            /* Second half of a cut: the STOP has gone, and the fresh clip waits
             * for the camera to finish closing the old one.
             *
             * It used to be one blind pair -- STOP, sleep 200 ms, START -- and
             * 200 ms lands squarely inside the finalise window this file
             * measured for the retry gate above: ~1.65 s on an Osmo 360 before
             * a stop is even reflected, and over two seconds on a Nano during
             * which it does not service BLE at all. So the START was issued
             * into a camera mid-file-close, was rejected with ret_code 223, and
             * the retry gate then held the fresh clip off for a further 3 s
             * because last_req had been stamped by the cut. "It never leaves
             * you not recording" was exactly what failed.
             *
             * Waiting for the camera to say it has stopped is the honest
             * signal. A Nano goes quiet rather than answering, so the deadline
             * is what covers it -- and this branch is deliberately exempt from
             * the retry gate, because the whole point is to be prompt. The 200
             * ms vTaskDelay is gone with it, which also stops the cut from
             * stalling the OSD for most of a tick. */
            if (!want) {
                /* The standing intent stopped being RECORD while the old clip
                 * was still closing -- the pilot disarmed, or pressed stop.
                 * Splitting the cut into two ticks opened this window; the old
                 * blind pair could not be interrupted because it never gave the
                 * loop back. Abandon the restart rather than starting a clip
                 * nobody wants any more. The next pass reconciles normally. */
                cut_restart = false;
                ESP_LOGI(TAG, "cut abandoned: no longer recording");
            } else {
                bool cam_idle = cam.recording_valid && !cam.recording;
                if (cam_idle || (int32_t)(t - cut_restart_by) >= 0) {
                    cut_restart    = false;
                    duml_cam_request_record(true);
                    last_req       = t;
                    have_sent      = true;
                    last_sent_want = true;
                    ESP_LOGI(TAG, "cut: fresh clip started (%s)",
                             cam_idle ? "camera confirmed idle" : "deadline");
                }
            }
        } else if ((need || s_cut_pending) && (t - last_req) >= RECORD_RETRY_MS) {
            if (s_cut_pending) {
                /* Cut a bad take: stop now, start as soon as the camera is
                 * ready -- see the branch above. */
                s_cut_pending  = false;
                duml_cam_request_record(false);
                cut_restart    = true;
                cut_restart_by = t + CUT_RESTART_MAX_MS;
                last_req       = t;
                have_sent      = true;
                /* A STOP was what went out, whatever the standing intent is. */
                last_sent_want = false;
            } else {
                duml_cam_request_record(want);
                last_req       = t;
                have_sent      = true;
                last_sent_want = want;
            }
        }

        
        /* ---- OSD ---- */
        char lines[4][17];
        /* One second on, one second off at a 250 ms tick. Slow on purpose: a
         * fast-moving character sits in peripheral vision and distracts,
         * whereas a 0.5 Hz dot is only noticed when you go looking for it --
         * which is exactly when it is needed. */
        static uint32_t osd_tick;
        uint32_t tk = osd_tick++;
        bool heartbeat = ((tk / 4) & 1) == 0;
        /* Twice the dot's rate. Two things flashing at the same speed in the
         * same corner of the screen read as one thing, and the whole point of a
         * warning is that it is not the thing you were already ignoring. */
        bool warn_on   = ((tk / 2) & 1) == 0;

        camlink_osd_extra_t extra = {
            .warn         = warn,
            .warn_on      = warn_on,
            .stop_delay_s = sd_remain,
        };
        camlink_format_osd(&cam, m.link_up, want, 0, heartbeat,
                           (const uint8_t (*)[OSD_ROW_FIELDS])cfg.osd,
                           s_setup_hint, &extra, lines);

        memcpy(s_osd, lines, sizeof(s_osd));

        /* Write a slot when its text changed -- and rewrite all four once a
         * second regardless.
         *
         * Change detection alone keeps the FC's UART almost idle, and that was
         * the whole reasoning until it met the thing it cannot handle: the
         * receiver forgetting. Betaflight has no way to expire a custom
         * message, so whatever it holds it holds for ever, and this task has no
         * way to know what that is. Three ordinary events leave the two out of
         * step, and none of them produce a change for the detector to notice:
         *
         *   - the module reboots while the FC does not, so the FC still shows
         *     text from before the reset until something happens to differ
         *   - the FC reboots while the module does not, so its slots are back
         *     to Betaflight's "CUSTOM_MSG1" placeholders and nothing here ever
         *     rewrites them
         *   - a single write is lost to a busy UART, and is never repeated
         *
         * All three end the same way: a stale readout beside a live aircraft,
         * which is the one thing the whole component exists to prevent. A
         * periodic rewrite is four frames a second against the twenty the
         * status poll already sends -- far too cheap to be worth being clever
         * about. */
        uint32_t now = now_ms();
        bool force = first_push
                  || (now - last_full_push) >= OSD_REFRESH_MS
                  || (m.link_up && !prev_link_up);   /* the FC just came back */
        if (force) last_full_push = now;
        prev_link_up = m.link_up;

        for (int i = 0; i < 4; i++) {
            if (force || strcmp(lines[i], prev[i]) != 0) {
                msp_set_osd_text((uint8_t)i, lines[i]);
                strcpy(prev[i], lines[i]);
            }
        }
        first_push = false;

        vTaskDelay(pdMS_TO_TICKS(s_cfg.osd_period_ms));
    }
}
