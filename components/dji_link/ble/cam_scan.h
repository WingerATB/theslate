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

/* Who made the camera. Decided from the advert, and it decides everything
 * downstream: which protocol the binding will speak, and what the picker calls
 * it. DJI is the value a zeroed entry gets, which is also what every entry was
 * before GoPro existed here. */
typedef enum {
    CAM_VENDOR_DJI   = 0,
    CAM_VENDOR_GOPRO = 1,
} cam_vendor_t;

typedef struct {
    uint8_t  bda[6];
    char     name[CAM_SCAN_NAME_LEN];
    int8_t   rssi;
    uint8_t  model;      /* DJI manufacturer model code; 0 = unidentified.
                          * Meaningless for a GoPro, which does not advertise
                          * its model at all -- the name is all there is.   */
    uint8_t  vendor;     /* cam_vendor_t                                   */
    uint8_t  addr_type;  /* esp_ble_addr_type_t as advertised. GoPros use a
                          * static random address, and a connection request
                          * with the wrong type is never answered.        */
    uint32_t last_ms;
} cam_scan_entry_t;

/* Bring up BLE and start scanning. Call instead of ble_init(): this path never
 * connects, and the two must not both own the controller. */
esp_err_t cam_scan_start(void);

/* Stop scanning and power the Bluetooth controller down, freeing the radio for
 * Wi-Fi. The captured list survives for the picker. See cam_scan.c. */
void cam_scan_stop(void);

/* Snapshot, strongest signal first, stale entries already dropped.
 * Returns how many were written. */
int cam_scan_get(cam_scan_entry_t *out, int max);

/* "Osmo Nano", "Osmo Action 5 Pro", or "" when the model code is not one we
 * have actually seen on hardware. */
const char *cam_scan_model_name(uint8_t model);

/* What the picker shows under the name: the DJI model name, or "GoPro" for a
 * camera that does not say which GoPro it is. */
const char *cam_scan_entry_model(const cam_scan_entry_t *e);

#endif
