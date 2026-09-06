/* SPDX-License-Identifier: PolyForm-Strict-1.0.0
 *
 * The GoPro camera session.
 *
 * A peer of the DUML / R SDK session, not a layer over it: everything below
 * "the scan picked this address and bonded to it" is GoPro's own, and no DJI
 * code is on this path. What the two share is the transport in
 * components/dji_link/ble -- one BLE controller, one scan window -- and the
 * shape of the status they publish, so the OSD and the record state machine
 * above cannot tell which kind of camera is attached.
 */
#ifndef GOPRO_CAM_H
#define GOPRO_CAM_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* The same fields duml_cam_status_t carries, deliberately.
 *
 * A separate type rather than a shared one because sharing would mean this
 * component and the DUML component including each other's headers, which is a
 * build cycle. Ten lines of copying at the boundary is the cheaper price. */
typedef struct {
    bool     link_up;        /* session established and usable               */
    bool     status_valid;   /* a status push arrived recently               */
    uint32_t status_age_ms;

    bool     recording;
    uint16_t clip_s;   bool clip_valid;
    uint32_t left_s;   bool left_valid;   /* seconds of video left on the card */
    uint8_t  battery_pct;
    bool     battery_valid;

    /* GoPro reports these; DJI's DUML path does not. */
    bool     hot;            /* overheating, and NOT necessarily unable to
                              * record -- see the note in gopro_cam.c        */
    bool     card_fault;     /* no card, unformatted, or errored             */
    bool     can_record;     /* the camera is in a mode where the shutter
                              * produces video rather than stills            */
} gopro_cam_status_t;

/* Install the GoPro GATT profile. Called before the connect is issued, because
 * the transport must know which characteristics to look for during discovery
 * and discovery happens as part of connecting. */
void gopro_cam_select(void);

/* Bring the session up on an already-connected, already-bonded link. Returns
 * false if the camera never became ready, in which case the caller should drop
 * the link and try again.
 *
 * The model chooses which of GoPro's two status-id families to try first (the
 * MISSION 1 series uses 2-byte ids); the camera's reply has the final say. */
bool gopro_cam_session_start(uint8_t model);

/* Housekeeping: sends the keep-alive on its own schedule. Call every logic
 * tick while connected.
 *
 * It does NOT re-register the status subscription. The camera keeps that for
 * the life of the connection, and re-registering would answer with a full set
 * of current values -- which is harmless but would mask a camera that had
 * genuinely stopped pushing, and that is a fault worth being able to see. */
void gopro_cam_tick(void);

/* The session is over. */
void gopro_cam_session_end(void);

/* Snapshot. */
void gopro_cam_get(gopro_cam_status_t *out);

/* Ask the camera to start or stop recording. */
void gopro_cam_request_record(bool on);

/* Notification hook, installed by the session on the transport. Exposed so the
 * connect path can route notifications here for a GoPro link. */
void gopro_cam_on_notify(uint16_t handle, const uint8_t *data, size_t len);

#endif /* GOPRO_CAM_H */
