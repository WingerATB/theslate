/* SPDX-License-Identifier: PolyForm-Strict-1.0.0
 *
 * Everything that differs between one chip -- or one board -- and the next.
 *
 * WHY THIS COMPONENT EXISTS. The firmware was written for an ESP32-C3
 * Supermini and said so in three separate ways, none of them in one place: a
 * transmit-power table that named a level only the C3 family has, an LED and a
 * button nailed to GPIO8 and GPIO9 as #defines, and a console that assumed a
 * USB Serial/JTAG peripheral. Each of those is a different kind of fact -- one
 * about the radio, one about the board, one about the peripheral set -- and
 * scattering them meant a second target could not be added without editing
 * files that have nothing to do with either.
 *
 * They live here now. This component depends on NOTHING else in the tree, so
 * everything else may depend on it, and adding a target is a table entry rather
 * than a search through the sources.
 *
 * ONE BINARY PER CHIP IS NOT A CHOICE WE GET TO MAKE. An ESP-IDF image carries
 * its chip id in the image header, the second-stage bootloader refuses a
 * mismatch, and our own updater refuses one too (webcfg.c, image_is_ours()).
 * Underneath that the C3/C5/C6/C61 are RISC-V while the S3 and the original
 * ESP32 are Xtensa, and each links a different prebuilt BLE controller. There
 * is no fat binary for this platform. What this component buys is one SOURCE
 * TREE that builds for all of them -- the flasher is what makes the user never
 * have to know which image is theirs.
 */
#ifndef SLATE_BOARD_H
#define SLATE_BOARD_H

#include <stdbool.h>
#include <stdint.h>

/* --------------------------------------------------------------------------
 * Front panel
 *
 * The C3 Supermini is the only board this has ever run on, and it is the only
 * one with a profile here. Everything else takes its pins from Kconfig, and the
 * LED DEFAULTS TO ABSENT rather than to a guess.
 *
 * That default is deliberate. Plenty of dev boards put an addressable WS2812
 * where the C3 Supermini has a plain LED -- the S3 DevKitC and the C6 DevKitC
 * both do -- and several put nothing there at all. Driving GPIO8 on a board
 * whose GPIO8 is wired to something else is a real way to break hardware, and
 * an LED that silently does nothing is a far better failure than one that
 * silently does something. A board with no LED is stated at boot rather than
 * discovered.
 * -------------------------------------------------------------------------- */
typedef enum {
    BOARD_LED_NONE = 0,   /* nothing we can drive; the LED calls are no-ops */
    BOARD_LED_GPIO,       /* a plain LED on one GPIO                       */
} board_led_kind_t;

typedef struct {
    const char      *name;             /* shown once at boot                 */
    board_led_kind_t led_kind;
    int16_t          led_gpio;         /* valid only when led_kind != NONE   */
    bool             led_active_low;   /* LED between 3V3 and the pin        */
    int16_t          btn_gpio;         /* < 0 when the board has no button   */
    bool             btn_active_low;
} board_profile_t;

const board_profile_t *board_profile(void);

/* Log the board and the transmit ladder. Called once, early, because "which
 * board does this build think it is" is the first question when a port does
 * nothing at all. */
void board_report(void);

/* --------------------------------------------------------------------------
 * BLE transmit power
 *
 * The user picks a RUNG, not a number of dBm, and the stored setting is that
 * rung's index -- which is why camlink_cfg.h stores an index of its own rather
 * than an esp_power_level_t. The reasoning there was that IDF's enum values
 * drift between releases. They also differ between PARTS, which is the same
 * argument with more teeth:
 *
 *     level        ESP32-C3   ESP32
 *     -12 dBm         4         0
 *       0 dBm         8         4
 *      +3 dBm         9         5
 *
 * The same stored byte means different power on different silicon, so the
 * translation has to happen here and nowhere else.
 *
 * THE INVARIANT THAT MATTERS: rung 0 is always the quietest level the part
 * has. "Low", and the drop that happens on arming when "turn it down while
 * flying" is on, therefore always mean "as quiet as this chip goes" -- which is
 * the property the ELRS receiver beside it cares about. What that is worth in
 * dBm is not the same everywhere:
 *
 *     C3 / S3 / C5      -24 dBm     the reference, and what has flown
 *     C6 / C61          -15 dBm     9 dB louder; its radio goes no quieter
 *     ESP32             -12 dBm    12 dB louder, and only three rungs
 *
 * A port to a C6 or an original ESP32 is therefore louder next to the receiver
 * than the board this flies on today, and that has to be re-checked with the
 * receiver in the frame rather than assumed to carry over.
 *
 * The names are served to the settings page over /api/state instead of being
 * written into it, for the same reason the OSD field catalogue is: a page that
 * says "-24 dBm" on a part whose floor is -15 dBm is confidently wrong, and
 * this firmware does not do that.
 * -------------------------------------------------------------------------- */

/* How many rungs this part actually has. Never more than CFG_TX__COUNT. */
int board_tx_count(void);

/* Rung -> esp_power_level_t, as an int so this header stays free of esp_bt.h.
 * A rung at or past board_tx_count() is clamped to the loudest one available,
 * so a config blob written by a build with more rungs cannot select a level
 * that does not exist. */
int board_tx_level(uint8_t rung);

/* "-24 dBm". Never NULL: an out-of-range rung gives the clamped one's name. */
const char *board_tx_name(uint8_t rung);

/* The name of a level actually in force, for logging what the radio is doing
 * rather than what was asked for. "(other)" when it is not one of ours. */
const char *board_tx_level_name(int level);

#endif /* SLATE_BOARD_H */
