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

#include "cam_bind.h"

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
 * the wrong one, and cannot be observed doing it.
 *
 * ---- The bound-camera list ------------------------------------------------
 *
 * A module holds up to CAM_BIND_MAX cameras in priority order and connects to
 * the highest-priority one that is actually switched on. See cam_bind.h for
 * the list operations and why the choice happens at connect time only.
 *
 * All of these are storage calls that do not touch the BLE stack, so config
 * mode -- which runs with the radio switched to Wi-Fi and BLE never started --
 * can read and edit the list safely. */
int  duml_cam_bind_count(void);
bool duml_cam_bind_get(int i, cam_bind_t *out);   /* i = 0 is top priority */

/* Add a camera the user picked, at the end of the list. Re-adding one already
 * held refreshes it and keeps its position. False when the list is full. */
bool duml_cam_bind_add(const uint8_t addr[6], uint8_t model, const char *name);

bool duml_cam_bind_forget(const uint8_t addr[6]);

/* Reorder by address, top priority first. Addresses rather than indices,
 * because the page's copy of the list can be stale -- see cam_bind.h. */
int  duml_cam_bind_reorder(const uint8_t (*order)[6], int n);

/* Is this camera one of ours? Used by the scan list to mark the entries the
 * module already holds. */
bool duml_cam_is_bound(const uint8_t addr[6]);

/* The top-priority camera's address, false when nothing is bound at all. */
bool duml_cam_bound_addr(uint8_t out[6]);

/* Forget every camera. */
void duml_cam_forget_binding(void);

/* Advertised model code of the camera currently CONNECTED (or last tried), 0
 * if unknown. Not the top of the list: with several cameras held, which one
 * answered is the one whose model matters. */
uint8_t duml_cam_bound_model(void);
void duml_cam_get(duml_cam_status_t *out);

/* Ask the camera to start/stop. Executed by the camera task. */
void duml_cam_request_record(bool on);











#endif
