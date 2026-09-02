/* SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 *
 * Front panel: the Supermini's onboard button and LED.
 *
 * GPIO8  = blue LED, INVERTED (drive LOW to light it)
 * GPIO9  = BOOT button, internally pulled up (reads LOW when pressed)
 *
 * Both are ESP32-C3 strapping pins, and the project brief says to avoid them.
 * That warning is about EXTERNAL circuits that can hold them at boot: GPIO9 low
 * at reset enters the serial bootloader, and GPIO8 participates in boot
 * configuration. Reading the onboard button and driving the onboard LED once
 * the chip is already running is exactly what the board is designed for, adds
 * no external load, and cannot affect the next boot. Nothing else may be wired
 * to these pins.
 */
#ifndef CAMLINK_UI_H
#define CAMLINK_UI_H

#include <stdbool.h>
#include <stdint.h>

#define UI_LED_GPIO     8
#define UI_BUTTON_GPIO  9

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
    UI_LED_RECORDING,    /* solid with a brief dip once a second            */
    UI_LED_CONNECTED,    /* solid                                           */
    UI_LED_SEARCHING,    /* slow blink                                      */
    UI_LED_UNPAIRED,     /* off                                             */
} ui_led_state_t;

typedef void (*ui_button_cb_t)(ui_button_event_t evt);

void ui_init(ui_button_cb_t cb);
void ui_set_led(ui_led_state_t state);

#endif
