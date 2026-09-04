/* SPDX-License-Identifier: PolyForm-Strict-1.0.0 */
#include "ui.h"
#include "board.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

/* Nothing is logged from here any more: board_report() states the pins once at
 * boot, which is the only thing this file ever had to say. */

#define TICK_MS       20
#define DEBOUNCE_MS   40

static ui_button_cb_t          s_cb;
static volatile ui_led_state_t s_led = UI_LED_UNPAIRED;

/* Resolved once in ui_init(). Held rather than fetched per tick because the
 * button is read every 20 ms and the profile cannot change while running. */
static const board_profile_t *s_bd;

/* Blink patterns as on/off millisecond pairs, looped.
 *
 * An off_ms of 0 means "stay on"; an on_ms of 0 means "stay dark". A single
 * blue LED carries every state, so these are chosen to be distinguishable at a
 * glance rather than to look pretty. */
typedef struct { uint16_t on_ms, off_ms; } pattern_t;

static const pattern_t k_patterns[] = {
    [UI_LED_CONFIG]    = {   0,    1 },   /* dark                           */
    [UI_LED_WARNING]   = { 120,  120 },   /* fast, ~4 Hz -- clearly urgent  */
    [UI_LED_RECORDING] = { 900,  100 },   /* solid with a dip once a second */
    [UI_LED_CONNECTED] = {   1,    0 },   /* solid on                       */
    [UI_LED_SEARCHING] = { 120,  880 },   /* slow, ~1 Hz                    */
    [UI_LED_UNPAIRED]  = {   0,    1 },   /* dark                           */
};

static inline void led_write(bool on)
{
    /* A board with no LED runs the whole pattern machine and drives nothing.
     * Cheaper than branching at every call site, and it keeps the LED states
     * meaningful to anything else that reads them. */
    if (s_bd == NULL || s_bd->led_kind == BOARD_LED_NONE) return;
    int level = s_bd->led_active_low ? !on : on;
    gpio_set_level((gpio_num_t)s_bd->led_gpio, level);
}

/* True when the button is down. Returns false on a board that has none, so a
 * press is simply never reported rather than being read off a floating pin. */
static inline bool btn_is_pressed(void)
{
    if (s_bd == NULL || s_bd->btn_gpio < 0) return false;
    int raw = gpio_get_level((gpio_num_t)s_bd->btn_gpio);
    return s_bd->btn_active_low ? (raw == 0) : (raw != 0);
}

void ui_set_led(ui_led_state_t state) { s_led = state; }

static void ui_task(void *arg)
{
    (void)arg;

    bool     pressed      = false;
    uint32_t press_ms     = 0;
    uint32_t stable_ms    = 0;
    bool     long_fired   = false;
    bool     vlong_fired  = false;
    bool     last_raw     = false;   /* last button level, before debouncing */

    ui_led_state_t cur = s_led;
    int      phase   = 0;
    uint32_t phase_ms = 0;

    for (;;) {
        /* ---- button ---- */
        /* Polarity is resolved inside btn_is_pressed(), so everything from
         * here down is in terms of pressed / not pressed rather than pin
         * levels -- there is one place that knows which way up the button is. */
        bool raw = btn_is_pressed();
        if (raw != last_raw) {
            last_raw = raw;
            stable_ms = 0;
        } else if (stable_ms < DEBOUNCE_MS) {
            stable_ms += TICK_MS;
            if (stable_ms >= DEBOUNCE_MS) {
                bool now_pressed = raw;
                if (now_pressed && !pressed) {
                    pressed = true;
                    press_ms = 0;
                    long_fired = false;
                    vlong_fired = false;
                } else if (!now_pressed && pressed) {
                    pressed = false;
                    if (!long_fired && s_cb) {
                        s_cb(UI_BTN_SHORT);
                    }
                }
            }
        }

        if (pressed) {
            press_ms += TICK_MS;
            if (!long_fired && press_ms >= UI_LONG_PRESS_MS) {
                long_fired = true;      /* fire once, while still held */
                if (s_cb) s_cb(UI_BTN_LONG);
            }
            if (!vlong_fired && press_ms >= UI_CONFIG_PRESS_MS) {
                vlong_fired = true;
                if (s_cb) s_cb(UI_BTN_VERY_LONG);
            }
        }

        /* ---- LED ---- */
        if (cur != s_led) {
            cur = s_led;
            phase = 0;
            phase_ms = 0;
        }
        const pattern_t *p = &k_patterns[cur];
        if (p->off_ms == 0) {
            led_write(true);          /* solid on  */
        } else if (p->on_ms == 0) {
            led_write(false);         /* fully dark */
        } else {
            led_write(phase == 0);
            phase_ms += TICK_MS;
            uint16_t dur = (phase == 0) ? p->on_ms : p->off_ms;
            if (phase_ms >= dur) {
                phase_ms = 0;
                phase ^= 1;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(TICK_MS));
    }
}

void ui_init(ui_button_cb_t cb)
{
    s_cb = cb;
    s_bd = board_profile();

    /* Only touch a pin the board actually has. A board that declares no LED
     * must be left entirely alone there -- on somebody else's hardware that
     * pin is carrying something, and driving it is worse than a dark LED. */
    if (s_bd->led_kind == BOARD_LED_GPIO) {
        gpio_config_t led = {
            .pin_bit_mask = 1ULL << (unsigned)s_bd->led_gpio,
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        gpio_config(&led);
        led_write(false);
    }

    if (s_bd->btn_gpio >= 0) {
        /* Held at the not-pressed level either way round. An input left
         * floating reads as a press held forever, and a press held forever is
         * the ten second hold -- the module would reboot into setup on its
         * own, repeatedly, with nobody touching it. */
        gpio_config_t btn = {
            .pin_bit_mask = 1ULL << (unsigned)s_bd->btn_gpio,
            .mode         = GPIO_MODE_INPUT,
            .pull_up_en   = s_bd->btn_active_low ? GPIO_PULLUP_ENABLE
                                                 : GPIO_PULLUP_DISABLE,
            .pull_down_en = s_bd->btn_active_low ? GPIO_PULLDOWN_DISABLE
                                                 : GPIO_PULLDOWN_ENABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        gpio_config(&btn);
    }

    xTaskCreate(ui_task, "ui", 2560, NULL, 3, NULL);
}
