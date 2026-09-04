/* SPDX-License-Identifier: PolyForm-Strict-1.0.0
 *
 * Per-target and per-board tables. See board.h for why they are here.
 */
#include "board.h"

#include "esp_bt.h"
#include "esp_log.h"
#include "sdkconfig.h"

static const char *TAG = "BOARD";

/* --------------------------------------------------------------------------
 * The transmit ladder
 *
 * Four rungs where the part has them, three on the original ESP32. The entries
 * are written out per family rather than computed, because the enum values are
 * not comparable across parts and a clever expression here would be a clever
 * expression nobody could check against a datasheet.
 * -------------------------------------------------------------------------- */
typedef struct { int level; const char *name; } tx_rung_t;

#if CONFIG_IDF_TARGET_ESP32
/* Floor is -12 dBm. ESP_PWR_LVL_N14 exists but is an ALIAS for N12 -- listing
 * both would be two rungs that do the same thing, and a switch on the level
 * would not compile. Three rungs is the honest count. */
static const tx_rung_t k_tx[] = {
    { ESP_PWR_LVL_N12, "-12 dBm" },
    { ESP_PWR_LVL_N0,  "0 dBm"   },
    { ESP_PWR_LVL_P3,  "+3 dBm"  },
};

#elif CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
/* ESP_PWR_LVL_N24 does not exist on this family; the radio stops at -15 dBm.
 * Rungs 0 and 1 are only 3 dB apart as a result, which is closer than the
 * ladder is elsewhere -- kept anyway so the rung count and the settings page
 * stay the same shape on every part. */
static const tx_rung_t k_tx[] = {
    { ESP_PWR_LVL_N15, "-15 dBm" },
    { ESP_PWR_LVL_N12, "-12 dBm" },
    { ESP_PWR_LVL_N0,  "0 dBm"   },
    { ESP_PWR_LVL_P3,  "+3 dBm"  },
};

#else
/* C3, S3 (which shares the C3's controller outright), C5, C2. The reference
 * ladder, and the one that has flown. */
static const tx_rung_t k_tx[] = {
    { ESP_PWR_LVL_N24, "-24 dBm" },
    { ESP_PWR_LVL_N12, "-12 dBm" },
    { ESP_PWR_LVL_N0,  "0 dBm"   },
    { ESP_PWR_LVL_P3,  "+3 dBm"  },
};
#endif

#define TX_COUNT ((int)(sizeof(k_tx) / sizeof(k_tx[0])))

int board_tx_count(void) { return TX_COUNT; }

static const tx_rung_t *rung_at(uint8_t rung)
{
    /* Clamp up, not down. A stored rung past the end of a shorter ladder came
     * from a user asking for MORE power, and answering that with the quietest
     * level would be the opposite of what they chose. Nothing here can select
     * a level the part does not have. */
    if ((int)rung >= TX_COUNT) return &k_tx[TX_COUNT - 1];
    return &k_tx[rung];
}

int         board_tx_level(uint8_t rung) { return rung_at(rung)->level; }
const char *board_tx_name (uint8_t rung) { return rung_at(rung)->name;  }

const char *board_tx_level_name(int level)
{
    for (int i = 0; i < TX_COUNT; i++) {
        if (k_tx[i].level == level) return k_tx[i].name;
    }
    return "(other)";
}

/* --------------------------------------------------------------------------
 * The board
 * -------------------------------------------------------------------------- */
#if CONFIG_SLATE_BOARD_C3_SUPERMINI
/* The board this ships and flies on.
 *
 * GPIO8 (LED, inverted) and GPIO9 (BOOT button, pulled up) are both strapping
 * pins. The warning about them concerns EXTERNAL circuits that can hold them at
 * reset; reading the onboard button and driving the onboard LED once the chip
 * is running is what the board is designed for and cannot affect the next boot.
 * Nothing else may be wired to them. */
static const board_profile_t k_board = {
    .name           = "ESP32-C3 Supermini",
    .led_kind       = BOARD_LED_GPIO,
    .led_gpio       = 8,
    .led_active_low = true,
    .btn_gpio       = 9,
    .btn_active_low = true,
};

#else
/* Anything else. The pins are the user's to state, and the LED is absent
 * unless they do -- see the note in board.h about why a guess is worse than
 * nothing here. */
static const board_profile_t k_board = {
    .name           = "generic " CONFIG_IDF_TARGET,
    .led_kind       = (CONFIG_SLATE_LED_GPIO >= 0) ? BOARD_LED_GPIO : BOARD_LED_NONE,
    .led_gpio       = CONFIG_SLATE_LED_GPIO,
#if CONFIG_SLATE_LED_ACTIVE_LOW
    .led_active_low = true,
#else
    .led_active_low = false,
#endif
    .btn_gpio       = CONFIG_SLATE_BUTTON_GPIO,
#if CONFIG_SLATE_BUTTON_ACTIVE_LOW
    .btn_active_low = true,
#else
    .btn_active_low = false,
#endif
};
#endif

const board_profile_t *board_profile(void) { return &k_board; }

void board_report(void)
{
    ESP_LOGI(TAG, "%s (%s)", k_board.name, CONFIG_IDF_TARGET);

    if (k_board.led_kind == BOARD_LED_NONE) {
        /* Said out loud, because every LED state this firmware has is now
         * invisible, and a silent module that is working looks exactly like a
         * dead one. */
        ESP_LOGW(TAG, "no LED configured -- the status LED will not light");
    } else {
        ESP_LOGI(TAG, "LED on GPIO%d (%s)", k_board.led_gpio,
                 k_board.led_active_low ? "active low" : "active high");
    }

    if (k_board.btn_gpio < 0) {
        /* The button is the recovery gesture: it is the only way into setup
         * that needs no flight controller, no radio and no working camera
         * link. Without it, a module whose setup switch is unset can only be
         * reconfigured over USB. */
        ESP_LOGW(TAG, "no button configured -- setup is reachable only from the "
                      "setup switch or a cable");
    } else {
        ESP_LOGI(TAG, "button on GPIO%d (%s)", k_board.btn_gpio,
                 k_board.btn_active_low ? "active low" : "active high");
    }

    ESP_LOGI(TAG, "BLE transmit ladder: %d rungs, quietest %s",
             TX_COUNT, k_tx[0].name);
}
