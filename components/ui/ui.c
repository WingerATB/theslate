/* SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0 */
#include "ui.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "UI";

#define TICK_MS       20
#define DEBOUNCE_MS   40

static ui_button_cb_t         s_cb;
static volatile ui_led_state_t s_led = UI_LED_UNPAIRED;

/* Blink patterns as on/off millisecond pairs, looped.
 *
 * An off_ms of 0 means "stay on"; an on_ms of 0 means "stay dark". A single
 * blue LED carries every state, so these are chosen to be distinguishable at a
 * glance rather than to look pretty. */
typedef struct { uint16_t on_ms, off_ms; } pattern_t;

static const pattern_t k_patterns[] = {
    [UI_LED_CONFIG]    = {   0,    1 },   /* dark                           */
    [UI_LED_RECORDING] = { 900,  100 },   /* solid with a dip once a second */
    [UI_LED_CONNECTED] = {   1,    0 },   /* solid on                       */
    [UI_LED_SEARCHING] = { 120,  880 },   /* slow, ~1 Hz                    */
    [UI_LED_UNPAIRED]  = {   0,    1 },   /* dark                           */
};

static inline void led_write(bool on)
{
    /* Inverted: the LED sits between 3V3 and the pin. */
    gpio_set_level(UI_LED_GPIO, on ? 0 : 1);
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
    int      last_raw     = 1;

    ui_led_state_t cur = s_led;
    int      phase   = 0;
    uint32_t phase_ms = 0;

    for (;;) {
        /* ---- button ---- */
        int raw = gpio_get_level(UI_BUTTON_GPIO);   /* 0 = pressed */
        if (raw != last_raw) {
            last_raw = raw;
            stable_ms = 0;
        } else if (stable_ms < DEBOUNCE_MS) {
            stable_ms += TICK_MS;
            if (stable_ms >= DEBOUNCE_MS) {
                bool now_pressed = (raw == 0);
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

    gpio_config_t led = {
        .pin_bit_mask = 1ULL << UI_LED_GPIO,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&led);
    led_write(false);

    gpio_config_t btn = {
        .pin_bit_mask = 1ULL << UI_BUTTON_GPIO,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&btn);

    xTaskCreate(ui_task, "ui", 2560, NULL, 3, NULL);
    ESP_LOGI(TAG, "button GPIO%d, LED GPIO%d", UI_BUTTON_GPIO, UI_LED_GPIO);
}
