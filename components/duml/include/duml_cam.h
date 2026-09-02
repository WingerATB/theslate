/* SPDX-License-Identifier: PolyForm-Strict-1.0.0
 *
 * DUML camera session: connect, pair, track status, control recording.
 *
 * Staleness rule (learned the hard way): the camera can stop pushing the
 * 0x02/0x80 status channel while the BLE link stays perfectly healthy and every
 * other channel keeps streaming. A status value older than DUML_CAM_STALE_MS is
 * therefore reported as NOT VALID rather than as the last thing we saw. Never
 * act on, or display, a frozen reading as if it were current.
 */
#ifndef CAMLINK_DUML_CAM_H
#define CAMLINK_DUML_CAM_H

#include <stdint.h>
#include <stdbool.h>

#define DUML_CAM_STALE_MS 1500

typedef enum {
    DUML_REC_IDLE      = 0,
    DUML_REC_STARTING  = 1,
    DUML_REC_RECORDING = 2,
} duml_rec_state_t;

typedef struct {
    bool     link_up;        /* BLE connected and paired                     */
    bool     status_valid;   /* a 0x02/0x80 push arrived recently            */
    uint32_t status_age_ms;

    duml_rec_state_t rec_state;
    uint16_t clip_s;         /* elapsed clip time                            */
    uint32_t free_mb;        /* SD free space                                */
    uint32_t left_s;         /* recording time remaining                     */

    uint8_t  work_mode;      /* 0 TAKEPHOTO, 1 RECORD, 2 PLAYBACK, ...       */
    uint8_t  battery_pct;    /* CANDIDATE -- not confirmed by measurement    */
    bool     battery_valid;

    /* Short name for the OSD: NANO, O360, A5... Resolved when the camera was
     * chosen and stored with the binding, because the advertised name -- the
     * only thing that identifies an Osmo Action 6, whose model code is not
     * published -- is long gone by the time we connect to it. */
    char     label[6];
} duml_cam_status_t;

void duml_cam_start(void);

/* There is deliberately no "adopt the next camera found" call. Bindings are
 * created in exactly one place -- the user picking from the config-mode scan
 * list -- because anything that adopts a camera on its own eventually adopts
 * the wrong one, and cannot be observed doing it. */

/* The stored binding, straight out of NVS. Both are pure storage calls that do
 * not touch the BLE stack, so config mode -- which runs with the radio switched
 * to Wi-Fi and BLE never started -- can read and clear the binding safely.
 * duml_cam_bound_addr() returns false when nothing is bound. */
bool duml_cam_bound_addr(uint8_t out[6]);
void duml_cam_forget_binding(void);

/* Bind to a specific camera, chosen by the user from the config-mode scan list
 * rather than adopted because it happened to be loudest. Storage only -- it
 * takes effect on the next normal boot, which is also when the pairing PIN
 * appears on the camera. */
void duml_cam_set_binding(const uint8_t addr[6], uint8_t model,
                          const char *name);

/* Advertised model code of the bound camera, 0 if unknown. The protocol the
 * module speaks is derived from this, so it is stored with the binding. */
uint8_t duml_cam_bound_model(void);
void duml_cam_get(duml_cam_status_t *out);

/* Ask the camera to start/stop. Executed by the camera task. */
void duml_cam_request_record(bool on);











#endif
