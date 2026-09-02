/* SPDX-License-Identifier: MIT
 *
 * Config-mode camera scanner: discovery only, never connects. See cam_scan.c.
 */
#ifndef CAMLINK_CAM_SCAN_H
#define CAMLINK_CAM_SCAN_H

#include <stdint.h>
#include "esp_err.h"

#define CAM_SCAN_MAX       12
#define CAM_SCAN_NAME_LEN  24

typedef struct {
    uint8_t  bda[6];
    char     name[CAM_SCAN_NAME_LEN];
    int8_t   rssi;
    uint8_t  model;      /* manufacturer model code; 0 = unidentified */
    uint32_t last_ms;
} cam_scan_entry_t;

/* Bring up BLE and start scanning. Call instead of ble_init(): this path never
 * connects, and the two must not both own the controller. */
esp_err_t cam_scan_start(void);

/* Snapshot, strongest signal first, stale entries already dropped.
 * Returns how many were written. */
int cam_scan_get(cam_scan_entry_t *out, int max);

/* "Osmo Nano", "Osmo Action 5 Pro", or "" when the model code is not one we
 * have actually seen on hardware. */
const char *cam_scan_model_name(uint8_t model);

#endif
