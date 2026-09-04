/* SPDX-License-Identifier: PolyForm-Strict-1.0.0
 *
 * USB console: a line-at-a-time command prompt on the Supermini's USB-C port.
 *
 * Exists so the module can be driven from a desk with nothing but a cable:
 * read the camera and flight-controller state, start and stop recording, drop
 * into setup, forget a camera. It is the bring-up tool for a camera the
 * firmware has never spoken to, and the fastest way to answer "what does the
 * module think is going on" without goggles or a phone.
 *
 * Type `help` for the commands. See usbcli.c for why this shares the port with
 * the log rather than replacing it.
 */
#ifndef CAMLINK_USBCLI_H
#define CAMLINK_USBCLI_H

#include <stdbool.h>

/* Start the console task. config_mode says which boot this is, because some
 * commands only make sense in one of them: the camera list exists only while
 * the config-mode scanner runs, and recording only while the camera link does.
 * Safe to call in either boot; does nothing if the option is compiled out. */
void usbcli_start(bool config_mode);

#endif
