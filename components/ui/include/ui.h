/* SPDX-License-Identifier: PolyForm-Strict-1.0.0
 *
 * Front panel: one status LED and one button.
 *
 * WHICH pins those are is not decided here. It used to be -- GPIO8 and GPIO9
 * were #defines in this header, which is correct for the ESP32-C3 Supermini
 * and correct for nothing else. Boards carrying the same chip put the LED
 * somewhere else, drive it the other way up, replace it with an addressable
 * WS2812 this code cannot drive, or have none at all. The pins, their polarity
 * and whether they exist come from board_profile() now.
 *
 * This file is left with what is genuinely about the front panel rather than
 * about the board: the gestures, the timings and the LED vocabulary.
 *
 * A board with no LED runs every state below as a no-op, and one with no
 * button simply never reports a press. Both are announced at boot by
 * board_report() rather than being left to be discovered.
 */
#ifndef CAMLINK_UI_H
#define CAMLINK_UI_H

#include <stdbool.h>
#include <stdint.h>

/* Long-press threshold for entering bind mode. Deliberately long: binding
 * drops the current camera, so it must not be reachable by a fumbled press. */
#define UI_LONG_PRESS_MS 3000

/* Keep holding past bind and the module reboots into Wi-Fi config mode.
 *
 * Ten seconds is not an accident anyone has. It also has to be reachable from
 * any state, including a module that is misconfigured badly enough to be
 * useless -- this is the recovery gesture, so it must not depend on the camera
 * link, the FC, or a laptop. Passing through the bind threshold on the way is
 * harmless: bind mode only clears the in-RAM address, and the reboot restores
 * the stored one. */
#define UI_CONFIG_PRESS_MS 10000

typedef enum {
    UI_BTN_SHORT,      /* released before UI_LONG_PRESS_MS   */
    UI_BTN_LONG,       /* held past UI_LONG_PRESS_MS         */
    UI_BTN_VERY_LONG,  /* held past UI_CONFIG_PRESS_MS       */
} ui_button_event_t;

/* Ordered by priority: the highest-priority true condition wins the LED.
 *
 * The scheme reads as "how settled is the link?" -- dark means nothing to say,
 * faster blinking means less settled, solid means done:
 *
 *   config     dark            Wi-Fi config mode
 *   unpaired   dark            no camera chosen yet
 *   searching  slow blink      paired, looking for that camera
 *   connected  solid           camera present
 *   recording  solid + dip     camera present AND rolling
 *   warning    fast blink      asked to record, and it is not happening
 *
 * The warning breaks the "how settled is the link" reading, and outranks all
 * of it, because it is the only state that means something is WRONG rather
 * than merely early. It is raised for the warnings that mean there is no
 * recording right now -- not for a battery or a card that will run out later,
 * which would leave the LED flashing at somebody for the whole of a session
 * they can do nothing about until they land.
 *
 * Recording is deliberately a variant of solid rather than a separate blink:
 * it is a sub-state of connected, so it should still read as "connected" at a
 * glance, with the dip marking that it is also rolling.
 *
 * Config mode is dark. It keeps its own state rather than reusing UI_LED_UNPAIRED
 * so the intent stays explicit and the appearance is one line to change, but
 * nothing about a module sitting on the bench with its settings page open needs
 * announcing -- the access point already says it, and the LED going quiet is a
 * clear enough signal that this boot is not flying anywhere. */
typedef enum {
    UI_LED_CONFIG,       /* off: Wi-Fi config mode                          */
    UI_LED_WARNING,      /* fast blink: it was asked to record and is not   */
    UI_LED_RECORDING,    /* solid with a brief dip once a second            */
    UI_LED_CONNECTED,    /* solid                                           */
    UI_LED_SEARCHING,    /* slow blink                                      */
    UI_LED_UNPAIRED,     /* off                                             */
} ui_led_state_t;

typedef void (*ui_button_cb_t)(ui_button_event_t evt);

void ui_init(ui_button_cb_t cb);
void ui_set_led(ui_led_state_t state);

#endif
