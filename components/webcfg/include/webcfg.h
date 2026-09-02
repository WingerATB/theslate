/* SPDX-License-Identifier: PolyForm-Strict-1.0.0
 *
 * Config mode: a Wi-Fi access point serving a settings page, so the module can
 * be set up from a phone at the field with no laptop, no toolchain and no
 * serial console. A shipped board needs flashing once and nothing after that.
 *
 * Config mode is a SEPARATE BOOT, not a mode switch:
 *
 *   - The ESP32-C3 has one radio. BLE and Wi-Fi can share it, but only through
 *     a coexistence arbiter that has to time-slice the camera link against an
 *     HTTP server. The camera link is the product; nothing is worth degrading
 *     it for a settings page.
 *   - Rebooting guarantees Wi-Fi cannot be running in flight. Entering config
 *     mode is visible (the module resets), and leaving it is unconditional.
 *   - It also means the settings task starts from a known-clean radio state
 *     rather than one left behind by a live camera session.
 *
 * The gesture is a ten-second hold on the module button (UI_CONFIG_PRESS_MS),
 * which sets a one-shot NVS flag and resets. The flag is consumed on the next
 * boot, so a power cycle always returns to normal operation -- a module can
 * never be left stuck in config mode.
 */
#ifndef CAMLINK_WEBCFG_H
#define CAMLINK_WEBCFG_H

#include <stdbool.h>

/* Read and clear the one-shot boot flag. Call once, early in app_main().
 * Clearing on read is what makes config mode self-limiting: even a crash loop
 * inside the web server comes back as a normal boot. */
bool webcfg_boot_flag_take(void);

/* Set the flag and reset into config mode. Does not return. */
void webcfg_reboot_into_config(void) __attribute__((noreturn));

/* Bring up the AP, the captive-portal DNS responder and the HTTP server.
 * Returns once they are running; the servers live in their own tasks. */
void webcfg_start(void);

/* SSID of the access point, e.g. "SLATE-3A90". Valid after webcfg_start(). */
const char *webcfg_ssid(void);

/* Leave config mode: shut the server and the AP down in order, then reset.
 * Does not return. Use this rather than esp_restart() -- from the request path,
 * with a station associated, a bare restart can stall in the Wi-Fi shutdown
 * handler and leave the module dark with its radio down. */
void webcfg_restart_now(void) __attribute__((noreturn));

#endif
