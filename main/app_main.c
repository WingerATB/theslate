/* SPDX-License-Identifier: PolyForm-Strict-1.0.0
 *
 * SlateFPV -- action-camera link for Betaflight (DJI Osmo + GoPro).
 *
 * Three FreeRTOS tasks, none of which may block the others:
 *
 *   camera task  -- owns BLE: connects to the camera, subscribes to status,
 *                   and executes record commands (which block for seconds).
 *   msp task     -- owns UART1: polls the FC at 10 Hz, never blocks on it.
 *   logic task   -- owns the record state machine and the OSD, at 4 Hz.
 *
 * Shared state moves only through the mutex-protected snapshot accessors in
 * msp.h and camlink.h.
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "msp.h"
#include "camlink.h"
#include "connect_logic.h"
#include "status_logic.h"
#include "data.h"
#include "duml.h"
#include "duml_cam.h"
#include "camlink_cfg.h"
#include "ui.h"
#include "driver/usb_serial_jtag.h"
#include <stdarg.h>
#include <string.h>
#include "ble.h"
#include "command_logic.h"
#include "enums_logic.h"
#include "esp_random.h"
#include "esp_bt_device.h"
#include "esp_gap_ble_api.h"
#include "esp_ota_ops.h"
#include "esp_bt.h"
#include "nvs_flash.h"
#include "webcfg.h"
#include "cam_scan.h"
#include "usbcli.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"

/* Bench console. Lives in dev/, which the published tree does not carry, so
 * this is the only place the firmware refers to it -- and only when the option
 * is on, which it is not by default. Bench mode ADDS a console to the normal
 * startup rather than replacing it: a bench build that boots differently from a
 * flight build is a bench build that tests something else. */
#if CONFIG_CAMLINK_BENCH
#include "bench.h"
#endif
#include <stdlib.h>

static const char *TAG = "APP";

/* ---- Wiring ------------------------------------------------------------- *
 * GPIO4 = UART1 TX -> FC UART RX
 * GPIO5 = UART1 RX <- FC UART TX
 * Do NOT move these to GPIO2/8/9: those are strapping pins on the C3 and a
 * pull on them at boot stops the chip from starting. GPIO20/21 stay free for
 * the console. */
#define MSP_UART_NUM     UART_NUM_1
#define MSP_TX_GPIO      CONFIG_CAMLINK_MSP_TX_GPIO
#define MSP_RX_GPIO      CONFIG_CAMLINK_MSP_RX_GPIO
#define MSP_BAUD         CONFIG_CAMLINK_MSP_BAUD

/* Report the FC link dead after this much silence. Short enough that a cut
 * UART closes an open clip promptly. */
#define MSP_LINK_TIMEOUT_MS   500

/* Default record behaviour. CUT: arm starts the clip, and the switch cuts a
 * bad take and immediately starts a fresh one. */
#if   defined(CONFIG_CAMLINK_MODE_ARM)
#define REC_MODE_DEFAULT      REC_MODE_ARM
#elif defined(CONFIG_CAMLINK_MODE_SWITCH)
#define REC_MODE_DEFAULT      REC_MODE_SWITCH
#else
#define REC_MODE_DEFAULT      REC_MODE_CUT
#endif
#define REC_SWITCH_CHANNEL    CONFIG_CAMLINK_SWITCH_CHANNEL
#define REC_SWITCH_THRESHOLD  CONFIG_CAMLINK_SWITCH_THRESHOLD
#define OSD_PERIOD_MS         250    /* 4 Hz */

/* Camera status subscription: periodic + on-change, at 2 Hz (the only
 * frequency the protocol accepts). */
#define CAM_PUSH_MODE   PUSH_MODE_PERIODIC_WITH_STATE_CHANGE
#define CAM_PUSH_FREQ   PUSH_FREQ_2HZ

/* --------------------------------------------------------------------------
 * Camera / BLE task
 * -------------------------------------------------------------------------- */




/* --------------------------------------------------------------------------
 * Front panel
 *
 * Long press  -> bind mode: forget this camera, adopt the next one seen.
 * Short press -> manual record toggle, but ONLY while the FC link is down.
 *
 * That restriction is deliberate. With an FC attached, arm state owns
 * recording; a button that also owned it would fight the state machine and
 * lose on the next poll, which would look like a broken button. On the bench
 * with no FC, the same press is the only way to test recording, so that is
 * exactly where it is allowed.
 * -------------------------------------------------------------------------- */
static void on_button(ui_button_event_t evt)
{
    /* Always visible. Whether a press was even seen is the first thing worth
     * knowing when the button "does not work", and it separates a dead button
     * from a command the camera ignored. */
    ESP_LOGW(TAG, "button: %s",
             evt == UI_BTN_SHORT        ? "short" :
             evt == UI_BTN_LONG         ? "long"  :
             evt == UI_BTN_LONG_RELEASE ? "long, released" : "very long");

    if (evt == UI_BTN_VERY_LONG) {
        /* Held straight through the bind threshold. Bind mode only cleared the
         * in-RAM address -- the stored one is still in NVS -- so the reboot
         * lands in config mode with the camera binding intact. */
        msp_state_t ms;
        msp_get_state(&ms);
        if (!camlink_may_enter_setup(ms.link_up, ms.boxarm_known, ms.armed)) {
            ESP_LOGW(TAG, "setup refused: the aircraft is armed");
            return;
        }
        webcfg_reboot_into_config();
    }

    if (evt == UI_BTN_LONG) {
        /* Three seconds in. Let go here to bind the nearest camera; keep
         * holding to reach setup. Nothing happens at this mark itself. */
        ESP_LOGI(TAG, "release now to bind the nearest camera, or keep holding to 10 s for setup");
        return;
    }

    if (evt == UI_BTN_LONG_RELEASE) {
        /* Same guard as setup: bind mode is a reboot with no camera link and
         * no MSP, so it must never start on an armed aircraft. */
        msp_state_t ms;
        msp_get_state(&ms);
        if (!camlink_may_enter_setup(ms.link_up, ms.boxarm_known, ms.armed)) {
            ESP_LOGW(TAG, "bind refused: the aircraft is armed");
            return;
        }
        webcfg_reboot_into_bind();
    }

    /* A test button, so it answers to the person holding it.
     *
     * This used to return early whenever the FC link was up, on the reasoning
     * that arm state owns recording and a button that fought the state machine
     * would lose on the next poll. The reasoning was sound and the conclusion
     * was wrong for what this control actually is: it sits on the module, it is
     * reached with a fingertip on a bench, and refusing it silently made it
     * look broken. The state machine now takes the press as an override rather
     * than being fought by it.
     *
     * What the press cannot do is hold the camera recording through a dead FC
     * link or a missing camera -- see camlink_ovr_update(). In those states it
     * can only ever stop a clip, never start one. */
    ESP_LOGW(TAG, "manual record: toggle");
    camlink_press_record();
}

/* The setup channel's current value, or 0 when the question cannot be answered
 * -- no switch configured, no RC data, or a channel past what the receiver
 * sends. Zero is safe as "unknown": a receiver never produces it, and for a
 * control that reboots the module "I cannot tell" must never read as "yes". */
static uint16_t setup_channel_us(const msp_state_t *m, const camlink_cfg_t *c)
{
    if (c->cfg_channel == 0)          return 0;
    if (!m->link_up || !m->rc_valid)  return 0;
    if (c->cfg_channel >= m->rc_count) return 0;
    return m->rc[c->cfg_channel];
}

/* Enter setup from the transmitter.
 *
 * The button is the documented way in, and on a finished aircraft the module is
 * buried inside the frame where no finger reaches it. A switch is the only
 * control the pilot still has.
 *
 * It is guarded, because this is not a display toggle: entering setup reboots
 * the module, stops the camera link entirely and raises a Wi-Fi access point.
 * Doing that in flight would end the recording and put an AP on the aircraft.
 * So it is refused while ARMED -- checked every pass rather than latched, so
 * arming at any point during a hold abandons it.
 *
 * A SWITCH holds a position, so it is asked to hold it: nothing happens until
 * the channel has been high for the configured time. A BUTTON has no position
 * to hold and acts the moment it moves, which is the same distinction, and the
 * same detector, as the record control. */
/* Watch the setup control, and narrate it on the OSD.
 *
 * A SWITCH is a two-part gesture: hold it up until the module says READY, then
 * drop it to commit. Three things fall out of that, and all three are the
 * reason for it rather than side effects.
 *
 *   The pilot can change their mind. Releasing before READY abandons the whole
 *   thing and the OSD goes back to what it was saying.
 *
 *   Committing on the RELEASE means the switch is already down at the moment
 *   setup starts -- so leaving setup cannot walk straight back into it, which
 *   is a trap a hold-to-enter design has to work around with an edge rule.
 *
 *   The wait is visible. A hold with no feedback is indistinguishable from a
 *   control that does not work, which is how the record switch came to be
 *   reported broken.
 *
 * A BUTTON has no position to hold and no release to wait for, so it commits
 * the moment it moves. The settings page says so.
 */
static void setup_switch_poll(void)
{
    static uint32_t held_since;
    static camlink_press_t press;
    /* The switch must be seen LOW before it can start a gesture, so a module
     * that boots with the switch already up does nothing until it is cycled. */
    static bool seen_low;
    static camlink_setup_hint_t hint = CAMLINK_SETUP_NONE;

    camlink_cfg_t c;
    camlink_cfg_get(&c);
    if (c.cfg_channel == 0) {
        if (hint != CAMLINK_SETUP_NONE) {
            hint = CAMLINK_SETUP_NONE;
            camlink_set_setup_hint(hint);
        }
        return;
    }

    msp_state_t m;
    msp_get_state(&m);
    uint16_t us = setup_channel_us(&m, &c);

    /* Anything that makes setup impossible abandons a gesture in progress, and
     * says so by putting the OSD back. */
    if (!camlink_may_enter_setup(m.link_up, m.boxarm_known, m.armed) || us == 0) {
        held_since = 0;
        camlink_press_update(&press, us);
        if (hint != CAMLINK_SETUP_NONE) {
            hint = CAMLINK_SETUP_NONE;
            camlink_set_setup_hint(hint);
        }
        return;
    }

    if (us < CAMLINK_AUX_HIGH_US) seen_low = true;

    if (c.cfg_kind == CFG_SW_BUTTON) {
        if (!seen_low || !camlink_press_update(&press, us)) return;
        ESP_LOGW(TAG, "setup button pressed on AUX%d -- entering setup",
                 cfg_index_to_aux(c.cfg_channel));
        camlink_set_setup_hint(CAMLINK_SETUP_ENTERING);
        /* Long enough for the logic task to draw CONFIG and for the frame to
         * clear the UART. Without it the reboot beats the write and the pilot
         * sees nothing at all between the press and the module vanishing. */
        vTaskDelay(pdMS_TO_TICKS(400));
        webcfg_reboot_into_config();
        return;
    }

    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    bool high = (us >= CAMLINK_AUX_HIGH_US);

    switch (hint) {
    case CAMLINK_SETUP_NONE:
        if (high && seen_low) {
            held_since = now;
            hint = CAMLINK_SETUP_ARMING;
            camlink_set_setup_hint(hint);
        }
        break;

    case CAMLINK_SETUP_ARMING:
        if (!high) {                       /* released early: changed their mind */
            hint = CAMLINK_SETUP_NONE;
            camlink_set_setup_hint(hint);
        } else if ((now - held_since) >= (uint32_t)c.cfg_hold_ds * 100u) {
            hint = CAMLINK_SETUP_READY;
            camlink_set_setup_hint(hint);
        }
        break;

    case CAMLINK_SETUP_READY:
        if (!high) {                       /* the release is the commit */
            ESP_LOGW(TAG, "setup switch released on AUX%d -- entering setup",
                     cfg_index_to_aux(c.cfg_channel));
            camlink_set_setup_hint(CAMLINK_SETUP_ENTERING);
            vTaskDelay(pdMS_TO_TICKS(400));
            webcfg_reboot_into_config();
        }
        break;

    default:
        break;
    }
}

static void ui_task_update(void *arg)
{
    (void)arg;
    for (;;) {
        setup_switch_poll();

        cam_status_t cam;
        camlink_get_cam_status(&cam);

        /* Highest-priority true condition wins the single LED. */
        if (cam.connected && cam.recording_valid && cam.recording) {
            ui_set_led(UI_LED_RECORDING);
        } else if (cam.connected) {
            ui_set_led(UI_LED_CONNECTED);
        } else if (ble_has_bound_addr()) {
            ui_set_led(UI_LED_SEARCHING);
        } else {
            /* No camera chosen: stay dark. A module that has not been set up
             * has nothing to report, and a dark LED says that unambiguously. */
            ui_set_led(UI_LED_UNPAIRED);
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

/* Map the stored setting onto the controller's power enum. The enum's numeric
 * values are an IDF detail; the stored value is ours. Keep the translation in
 * one place so a future IDF renumbering cannot quietly make a saved -24 dBm
 * mean something louder. */
static int tx_level_for(uint8_t cfg_tx)
{
    switch (cfg_tx) {
    case CFG_TX_N12: return ESP_PWR_LVL_N12;
    case CFG_TX_N0:  return ESP_PWR_LVL_N0;
    case CFG_TX_P3:  return ESP_PWR_LVL_P3;
    default:         return ESP_PWR_LVL_N24;
    }
}

/* --------------------------------------------------------------------------
 * Config mode
 *
 * A separate boot with the radio switched to Wi-Fi: BLE is never started, so
 * there is no coexistence to arbitrate and no way for an access point to be up
 * while the quad is in the air.
 *
 * MSP still runs. The flight controller is on a wire, not the radio, and
 * having it live is what lets the settings page show the user's switch moving
 * in real time while they pick a channel -- the difference between "AUX 3,
 * probably" and watching the marker jump when they flick it.
 *
 * BLE runs too, but ONLY as a scanner -- it never connects. That is what makes
 * the camera picker live: the user sees what is actually in range, sorted by
 * signal, and binds to the one they meant instead of whichever advertised
 * loudest. Wi-Fi and BLE share the single radio through the coexistence layer.
 * That layer is compiled in for every build, but normal boots never start
 * Wi-Fi, so in flight it has nothing to arbitrate.
 * -------------------------------------------------------------------------- */
static void on_button_config(ui_button_event_t evt)
{
    if (evt == UI_BTN_SHORT) return;
    /* Any long press leaves. The way out must not depend on a phone having
     * successfully joined the access point. Same hardened path as the page's
     * own Done button -- a bare esp_restart() here can hang with a station
     * associated. */
    ESP_LOGW(TAG, "leaving config mode");
    webcfg_restart_now();
}

/* Say SETUP on the OSD, and leave again when the switch is released.
 *
 * Config mode never wrote the OSD at all: it starts msp_task but not the logic
 * task, so whatever text was on screen when setup was entered stayed there,
 * frozen, for as long as setup lasted. On a module mounted inside the airframe
 * that is the only screen the pilot has, and it was showing a stale readout of
 * a camera the module had by then disconnected from.
 *
 * Rows are padded to the full sixteen characters rather than to the width of
 * what is being written, because Betaflight never clears and the row underneath
 * may be longer than this one. */
static void config_osd_task(void *arg)
{
    (void)arg;

    for (;;) {
        msp_state_t m;
        msp_get_state(&m);

        char row[MSP_TEXT_MAX_LEN + 1];
        memset(row, ' ', MSP_TEXT_MAX_LEN);
        row[MSP_TEXT_MAX_LEN] = '\0';
        memcpy(row, "CONFIG", 6);
        msp_set_osd_text(0, row);
        memset(row, ' ', MSP_TEXT_MAX_LEN);
        for (int i = 1; i < 4; i++) msp_set_osd_text((uint8_t)i, row);

        /* Arming leaves setup, and it is the only thing on the aircraft that
         * does.
         *
         * The setup switch deliberately does NOT bring you back out. It brought
         * you in, and then you put the phone down and start moving switches --
         * so an exit on the same control would drop you out of the page in the
         * middle of using it. Leaving is either a deliberate act on the page
         * (Done & restart), or the unambiguous statement that the aircraft is
         * about to fly.
         *
         * Gated on boxarm_known so a reply that has not yet told us where
         * BOXARM lives cannot read as armed. */
        if (m.link_up && m.boxarm_known && m.armed) {
            ESP_LOGW(TAG, "armed while in setup -- leaving setup");
            webcfg_restart_now();
        }

        /* Fast enough that arming is acted on promptly; the OSD text does not
         * need this rate but the arm check rides on it. */
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

/* --------------------------------------------------------------------------
 * Bind mode: choose the nearest camera from the button alone.
 *
 * A separate boot, like setup, because the scanner and the camera session
 * cannot share the BLE controller, and because a bind that could happen
 * mid-flight is a bind that eventually will.
 *
 * "Nearest" is guarded, not assumed. This firmware once adopted whichever
 * camera advertised loudest, and with a second camera in range it picked the
 * wrong one with no way to tell. So the strongest camera is bound only when
 * it is genuinely close AND clearly ahead of anything else, and has stayed
 * that way for a couple of seconds. Anything less and the module says why on
 * the console and goes back to normal with its binding untouched -- the
 * settings page is always there for the ambiguous case.
 *
 * For a GoPro this is also the moment to have the camera in pairing mode: the
 * reboot that follows makes the first connection, and that is where the bond
 * is formed.
 * -------------------------------------------------------------------------- */
#define BIND_WINDOW_MS     30000
#define BIND_MIN_RSSI      (-75)   /* on the desk, tens of centimetres away   */
#define BIND_MIN_GAP_DB    12      /* the runner-up must be clearly further    */
#define BIND_STABLE_POLLS  4       /* same winner for 2 s of 500 ms polls      */

static void on_button_bind(ui_button_event_t evt)
{
    (void)evt;
    ESP_LOGW(TAG, "bind cancelled by the button");
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_restart();
}

static void bind_mode(void)
{
    ESP_LOGW(TAG, "BIND MODE: looking for the nearest camera for %d s", BIND_WINDOW_MS / 1000);
    ui_init(on_button_bind);
    ui_set_led(UI_LED_BINDING);
    usbcli_start(true);

    esp_err_t e = cam_scan_start();
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "camera scan failed to start: %s", esp_err_to_name(e));
        vTaskDelay(pdMS_TO_TICKS(200));
        esp_restart();
    }

    uint8_t     stable_mac[6] = {0};
    int         stable        = 0;
    const char *verdict       = "no camera in range";
    const char *said          = NULL;
    const uint32_t t0 = (uint32_t)(esp_timer_get_time() / 1000);

    while ((uint32_t)(esp_timer_get_time() / 1000) - t0 < BIND_WINDOW_MS) {
        vTaskDelay(pdMS_TO_TICKS(500));
        cam_scan_entry_t cams[CAM_SCAN_MAX];
        int n = cam_scan_get(cams, CAM_SCAN_MAX);

        int best = -1, second = -1;
        for (int i = 0; i < n; i++) {
            if (best < 0 || cams[i].rssi > cams[best].rssi) { second = best; best = i; }
            else if (second < 0 || cams[i].rssi > cams[second].rssi) { second = i; }
        }

        if (best < 0) {
            verdict = "no camera in range"; stable = 0;
        } else if (cams[best].rssi < BIND_MIN_RSSI) {
            verdict = "nearest camera is too far -- hold it against the module"; stable = 0;
        } else if (second >= 0 && (cams[best].rssi - cams[second].rssi) < BIND_MIN_GAP_DB) {
            verdict = "two cameras equally close -- move one away, or pick in setup"; stable = 0;
        } else if (memcmp(stable_mac, cams[best].bda, 6) != 0) {
            memcpy(stable_mac, cams[best].bda, 6);
            stable = 1;
            verdict = "candidate found, confirming";
        } else if (++stable >= BIND_STABLE_POLLS) {
            const cam_scan_entry_t *c = &cams[best];
            duml_cam_set_binding(c->bda, c->model, c->name, c->vendor, c->addr_type);
            ESP_LOGW(TAG, "bound %s %02X:%02X:%02X:%02X:%02X:%02X at %d dBm -- rebooting to connect",
                     c->name[0] ? c->name : (c->vendor == CAM_VENDOR_GOPRO ? "GoPro" : "DJI camera"),
                     c->bda[0], c->bda[1], c->bda[2], c->bda[3], c->bda[4], c->bda[5], c->rssi);
            ui_set_led(UI_LED_CONNECTED);
            vTaskDelay(pdMS_TO_TICKS(1500));
            esp_restart();
        }
        if (verdict != said) { said = verdict; ESP_LOGI(TAG, "bind: %s", verdict); }
    }

    ESP_LOGW(TAG, "bind gave up: %s. Binding unchanged.", verdict);
    ui_set_led(UI_LED_UNPAIRED);
    vTaskDelay(pdMS_TO_TICKS(300));
    esp_restart();
}

/* --------------------------------------------------------------------------
 * USB-config mode: the settings UI, driven over the USB cable.
 *
 * Same job as config_mode, but the transport is the USB serial API (see the
 * `api` command in usbcli) instead of a Wi-Fi access point, and there is no
 * Wi-Fi at all. That is the whole point: on this chip Bluetooth and Wi-Fi
 * share one antenna and an enabled Bluetooth controller starves the softAP,
 * which is why the Wi-Fi picker had to freeze the camera list. With Wi-Fi out
 * of the way the BLE scanner runs continuously, so a browser on the far end of
 * the cable gets a genuinely live camera list while it edits settings.
 * -------------------------------------------------------------------------- */
static void usbcfg_mode(void)
{
    ESP_LOGW(TAG, "USB-CONFIG MODE: settings over the USB serial API, no Wi-Fi");
    ui_init(on_button_config);       /* a long press still leaves */
    ui_set_led(UI_LED_CONFIG);

    msp_config_t mcfg = {
        .uart_num        = MSP_UART_NUM,
        .tx_gpio         = MSP_TX_GPIO,
        .rx_gpio         = MSP_RX_GPIO,
        .baud            = MSP_BAUD,
        .link_timeout_ms = MSP_LINK_TIMEOUT_MS,
    };
    if (msp_init(&mcfg) == ESP_OK) {
        xTaskCreate(msp_task, "msp", 4096, NULL, 6, NULL);
    }

    camlink_cfg_init();

    camlink_cfg_t c;
    camlink_cfg_get(&c);
    camlink_ble_set_tx_level(tx_level_for(c.tx_power));

    esp_err_t e = cam_scan_start();
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "camera scan failed to start: %s", esp_err_to_name(e));
    }

    usbcli_start(true);
    ESP_LOGW(TAG, "connect the browser tool over USB serial; hold the button to leave");
}

static void config_mode(void)
{
    ESP_LOGW(TAG, "CONFIG MODE: Wi-Fi access point + BLE camera scan");

    ui_init(on_button_config);
    ui_set_led(UI_LED_CONFIG);

    msp_config_t mcfg = {
        .uart_num        = MSP_UART_NUM,
        .tx_gpio         = MSP_TX_GPIO,
        .rx_gpio         = MSP_RX_GPIO,
        .baud            = MSP_BAUD,
        .link_timeout_ms = MSP_LINK_TIMEOUT_MS,
    };
    if (msp_init(&mcfg) == ESP_OK) {
        xTaskCreate(msp_task, "msp", 4096, NULL, 6, NULL);
    }

    /* Settings are read straight out of NVS by the web handlers, so the config
     * store has to be live even though the record state machine is not. */
    camlink_cfg_init();

    /* A short camera census, THEN Wi-Fi alone.
     *
     * The two radios cannot both be live on this board: an enabled Bluetooth
     * controller starves the Wi-Fi access point's beacon, so a settings page
     * served with Bluetooth still up is invisible to the phone trying to reach
     * it. So config mode scans for a couple of seconds to populate the picker,
     * powers Bluetooth all the way down, and only then raises the access point
     * -- which from that point has the antenna to itself. The camera list the
     * page shows is the census taken at entry; a reboot back into setup takes
     * a fresh one. Wi-Fi here is only ever used to edit settings and reboot. */
    {
        camlink_cfg_t c;
        camlink_cfg_get(&c);
        camlink_ble_set_tx_level(tx_level_for(c.tx_power));

        esp_err_t e = cam_scan_start();
        if (e != ESP_OK) {
            ESP_LOGE(TAG, "camera scan failed to start: %s", esp_err_to_name(e));
        } else {
            /* Long enough to hear a camera on the desk several times over;
             * short enough that the page is up within a few seconds of the
             * button. */
            vTaskDelay(pdMS_TO_TICKS(2500));
            cam_scan_stop();
        }
    }

    webcfg_start();

    /* Both radios up. This is the peak allocation the design has to survive,
     * and it is worth stating out loud on every config boot rather than
     * discovering it on someone else's board. */
    ESP_LOGW(TAG, "heap free %u B (min %u B) with BLE scan + Wi-Fi AP up",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT),
             (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT));

    /* Last, so the OSD only says SETUP once setup is genuinely up. */
    xTaskCreate(config_osd_task, "cfgosd", 3072, NULL, 3, NULL);

    ESP_LOGW(TAG, "join Wi-Fi \"%s\", then open http://192.168.4.1/", webcfg_ssid());

    usbcli_start(true);
}

/* --------------------------------------------------------------------------
 * Post-update health check
 *
 * After a firmware update the new image boots on probation: the bootloader
 * will put the old one back unless this image declares itself healthy. This is
 * where that declaration happens, and what it means is deliberately narrow --
 * "reached the end of app_main and then ran for a while without resetting".
 *
 * That catches the class of failure this net exists for: an image that panics
 * during init, hangs before its tasks start, or reboot-loops. It cannot catch
 * a subtle bug, and it is not meant to; a user who cannot reflash over USB
 * needs to be rescued from a brick, not from a regression.
 *
 * The delay is a real trade. Too short and a crash at 40 s gets confirmed as
 * good; too long and unplugging the module early triggers a rollback nobody
 * asked for. Thirty seconds sits where boot-time failures have all already
 * happened, and the settings page says so before it restarts.
 * -------------------------------------------------------------------------- */
#define OTA_PROBATION_MS  30000

static void ota_confirm_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(OTA_PROBATION_MS));
    esp_err_t e = esp_ota_mark_app_valid_cancel_rollback();
    if (e == ESP_OK) {
        ESP_LOGW(TAG, "update confirmed healthy; rollback cancelled");
    } else {
        ESP_LOGE(TAG, "could not confirm update: %s", esp_err_to_name(e));
    }
    vTaskDelete(NULL);
}

static void ota_probation_start(void)
{
    const esp_partition_t *run = esp_ota_get_running_partition();
    esp_ota_img_states_t st;
    if (run == NULL || esp_ota_get_state_partition(run, &st) != ESP_OK) return;
    if (st != ESP_OTA_IMG_PENDING_VERIFY) return;

    ESP_LOGW(TAG, "running a NEW image on probation -- confirming in %d s",
             OTA_PROBATION_MS / 1000);
    xTaskCreate(ota_confirm_task, "otaok", 3072, NULL, 2, NULL);
}

/* -------------------------------------------------------------------------- */

void app_main(void)
{
    ESP_LOGI(TAG, "SlateFPV starting");

    /* NVS before anything reads it. ble_init() also initialises NVS, but the
     * config-mode boot never gets that far, and the boot flag is read before
     * either. A second nvs_flash_init() is a no-op. */
    esp_err_t nvs = nvs_flash_init();
    if (nvs == ESP_ERR_NVS_NO_FREE_PAGES || nvs == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    /* Consumed on read: config mode lasts exactly one boot, so a module can
     * never be left stuck in it. */
    if (webcfg_boot_flag_take()) {
        config_mode();
        return;
    }
    if (webcfg_bind_flag_take()) {
        bind_mode();
        return;
    }
    if (webcfg_usbcfg_flag_take()) {
        usbcfg_mode();
        return;
    }

    /* Plugged into a computer? Then this is a setup session, not a flight.
     *
     * The one thing that tells a computer apart from the flight controller's
     * 5 V pad or a dumb charger is that a computer's USB host sends a
     * start-of-frame packet every millisecond; nothing else does. The driver's
     * connection monitor watches for those, so once it has settled we can ask.
     * If a host is there, boot straight into the browser-UI mode -- no console
     * command, no button, no Wi-Fi. Unplug and wire to the aircraft and the
     * same firmware boots normally.
     *
     * Skipped for an image still on OTA probation: a fresh update has to reach
     * normal operation to be kept, and must not be diverted into setup by a
     * cable that happens to be attached while it is being tested. */
    if (webcfg_bootnormal_flag_take()) {
        ESP_LOGW(TAG, "one-shot NORMAL override -- skipping USB auto-detect this boot");
    } else {
        const esp_partition_t *run = esp_ota_get_running_partition();
        esp_ota_img_states_t ost = ESP_OTA_IMG_UNDEFINED;
        bool on_probation = run && esp_ota_get_state_partition(run, &ost) == ESP_OK
                            && ost == ESP_OTA_IMG_PENDING_VERIFY;
        if (!on_probation) {
            /* Let the SOF monitor settle before trusting it: it starts by
             * assuming a host is present and only confirms otherwise after a
             * few missed frames. */
            vTaskDelay(pdMS_TO_TICKS(700));
            if (usb_serial_jtag_is_connected()) {
                ESP_LOGW(TAG, "USB host detected -- entering browser-UI mode");
                usbcfg_mode();
                return;
            }
        }
    }




    /* MSP first: the OSD and arm state are the half that must work even if the
     * camera never shows up. */
    msp_config_t mcfg = {
        .uart_num         = MSP_UART_NUM,
        .tx_gpio          = MSP_TX_GPIO,
        .rx_gpio          = MSP_RX_GPIO,
        .baud             = MSP_BAUD,
        .link_timeout_ms  = MSP_LINK_TIMEOUT_MS,
    };
    if (msp_init(&mcfg) != ESP_OK) {
        ESP_LOGE(TAG, "MSP init failed");
        return;
    }

    camlink_config_t ccfg = {
        .mode             = REC_MODE_DEFAULT,
        .switch_channel   = REC_SWITCH_CHANNEL,
        .switch_threshold = REC_SWITCH_THRESHOLD,
        .osd_period_ms    = OSD_PERIOD_MS,
    };
    camlink_logic_init(&ccfg);

    /* camlink_logic_init() loaded the stored settings; apply the transmit power
     * from them BEFORE the BLE stack starts, so a user who turned it up for a
     * bench session and back down for flight never has to reflash. */
    {
        camlink_cfg_t c;
        camlink_cfg_get(&c);
        camlink_ble_set_tx_level(tx_level_for(c.tx_power));
        ESP_LOGI(TAG, "BLE TX power %s", camlink_cfg_tx_name(c.tx_power));
    }

    /* The camera half speaks DUML: the Osmo Nano ignores the DJI R SDK
     * protocol entirely. duml_cam owns BLE, pairing and record commands. */
    duml_cam_start();
    ui_init(on_button);

    xTaskCreate(msp_task,           "msp",   4096, NULL, 6, NULL);
    xTaskCreate(camlink_logic_task, "logic", 4096, NULL, 5, NULL);
    xTaskCreate(ui_task_update,     "uiled", 2560, NULL, 2, NULL);

    ESP_LOGI(TAG, "tasks started");

    /* Deliberately last, and deliberately NOT in config mode: a new image has
     * to reach full normal operation before it is allowed to keep itself. */
    ota_probation_start();

    usbcli_start(false);

#if CONFIG_CAMLINK_BENCH
    bench_start();
#endif
}
