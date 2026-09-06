/* SPDX-License-Identifier: PolyForm-Strict-1.0.0
 *
 * CAMLINK: record state machine + OSD reporting.
 *
 * Design rule that governs this whole component: the module is a PASSENGER.
 * It sits outside the motor, PID and RC control paths. It only ever reads FC
 * state and writes text into OSD message slots. It must never be able to stall
 * the FC's UART or command the aircraft.
 */
#ifndef CAMLINK_H
#define CAMLINK_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "osd_fields.h"
#include "camlink_warn.h"

/* --------------------------------------------------------------------------
 * Camera status, as far as we are willing to trust it.
 *
 * Every field carries its own _valid flag. A field is valid only if the camera
 * actually transmitted the bytes it lives in AND we have a live connection. A
 * blank OSD slot is correct; a wrong readout is worse than no readout.
 * -------------------------------------------------------------------------- */
typedef struct {
    bool     connected;          /* BLE + DJI protocol session is up          */

    bool     recording;          /* camera reports recording/pre-recording    */
    bool     recording_valid;

    uint16_t record_time_s;      /* elapsed clip time, camera's own counter   */
    bool     record_time_valid;

    uint32_t remain_time_s;      /* remaining card time in seconds            */
    bool     remain_time_valid;

    uint8_t  battery_pct;        /* 0..100                                    */
    bool     battery_valid;

    uint8_t  temp_over;          /* 0 ok, 1 warn, 2 too hot, 3 shutting down  */
    bool     temp_over_valid;

    bool     card_fault;         /* no card, unformatted, full or errored     */
    bool     card_fault_valid;

    char     label[6];           /* NANO / O360 / A5 ... resolved at bind
                                  * time; never blank, so the field renders
                                  * even for a camera we do not recognise    */

    uint32_t last_update_ms;
    size_t   last_payload_len;   /* bytes the camera sent in the last push    */
} cam_status_t;

/* --------------------------------------------------------------------------
 * Record trigger modes
 * -------------------------------------------------------------------------- */
typedef enum {
    REC_MODE_ARM = 0,   /* arm starts, disarm stops                           */
    REC_MODE_SWITCH,    /* an RC channel toggles recording                    */
    REC_MODE_CUT,       /* arm starts the clip; a press cuts a bad take
                         * and immediately starts a fresh one                 */
    REC_MODE_BOTH,      /* arm starts the clip; a press toggles recording
                         * independently of arm state                         */
} record_mode_t;

typedef struct {
    record_mode_t mode;
    uint8_t  switch_channel;   /* 0-based RC index; 7 == AUX4                 */
    uint16_t switch_threshold; /* us; above this counts as "high"             */
    uint32_t osd_period_ms;    /* OSD refresh period                          */
} camlink_config_t;

/* Snapshot the camera status (mutex protected). */
void camlink_get_cam_status(cam_status_t *out);

/* Pull the latest state from the DUML camera session into cam_status_t.
 * Called by the logic task; not needed by application code. */
void camlink_poll_camera(void);

/* Mark the BLE/protocol session up or down. */
void camlink_set_connected(bool connected);

/* Manual record intent from the front-panel button. Honoured only while no
 * flight controller has ever been seen; once an FC answers, arm state owns
 * recording and this is ignored. */
void camlink_set_manual_record(bool on);

/* One press of the front-panel button. Toggles recording whatever else is going
 * on -- it is a test button, so it answers to the person holding it rather than
 * to the flight controller. What it may NOT do is hold the camera recording
 * through a dead FC link or a missing camera; see camlink_ovr_update(). */
void camlink_press_record(void);

/* What the setup control is doing, shown in the OSD's state field.
 *
 * The switch is a two-part gesture: hold it up until the module says READY,
 * then drop it to commit. That is deliberate. It gives the pilot somewhere to
 * change their mind -- releasing early abandons it and the OSD goes back to
 * normal -- and it means the switch is already DOWN at the moment setup
 * starts, so leaving setup cannot walk straight back into it.
 *
 * A button has no position to hold, so it goes straight to ENTERING. */
typedef enum {
    CAMLINK_SETUP_NONE = 0,
    CAMLINK_SETUP_ARMING,    /* held up, waiting out the hold time  -> ENTER  */
    CAMLINK_SETUP_READY,     /* hold satisfied, release to commit   -> READY  */
    CAMLINK_SETUP_ENTERING,  /* committed, rebooting into setup     -> CONFIG */
} camlink_setup_hint_t;

void camlink_set_setup_hint(camlink_setup_hint_t hint);

/* Whether a manual record intent is currently latched. Exposed so it can be
 * displayed: an invisible sticky toggle looks exactly like a camera starting
 * to record on its own. */
bool camlink_get_manual_record(void);

void camlink_logic_init(const camlink_config_t *cfg);

/* The four slots exactly as they were last pushed to the FC.
 *
 * Exposed because "what does the OSD say" was, until now, only answerable by
 * plugging the module into a drone and looking through goggles -- which meant
 * OSD faults could not be reproduced at the bench at all. */
void camlink_get_osd(char out[4][17]);

/* Logic task body: runs the record state machine and pushes OSD text. */
void camlink_logic_task(void *arg);

/* The warning currently in force, for the front-panel LED.
 *
 * The OSD is the real report and says far more than one LED can. This exists
 * for the bench, where there are no goggles: a module that is powered, paired
 * and quietly not recording looks exactly like one that is working. */
camlink_warn_t camlink_get_warn(void);

/* What the state slot has to say beyond REC / IDLE.
 *
 * Passed in rather than derived, because both fields need a clock the
 * formatter deliberately does not have: the warning needs a grace period and
 * the countdown needs a deadline. Keeping time out of the renderer is what
 * lets the host tests drive it. NULL is legal and means neither. */
typedef struct {
    camlink_warn_t warn;
    /* Warnings blink, and blink FASTER than the liveness dot -- two things
     * flashing at the same rate in the same corner read as one thing. On the
     * off phase the slot shows the normal state, so the pilot still sees
     * whether it is recording rather than losing that to the alarm. */
    bool           warn_on;
    /* Seconds still being held after an automatic stop. Rendered as "REC 5",
     * because a camera that carries on recording after landing with nothing
     * on screen explaining it looks exactly like a stop that failed. */
    uint8_t        stop_delay_s;
    /* Set by the renderer, not by the caller: true when the layout has no
     * Warning field, so the state slot has to carry warnings itself. */
    bool           warn_in_state;
} camlink_osd_extra_t;

/* Exposed for testing the formatter without an FC attached. */
/* heartbeat: draw the liveness dot this frame, or not. Toggled by the caller
 * once a second. See the note on the dot in osd_format.c -- it must be driven
 * from THIS side to mean anything. */
void camlink_format_osd(const cam_status_t *cam, bool msp_link_up,
                        bool want_recording, uint32_t clip_elapsed_s,
                        bool heartbeat,
                        const uint8_t layout[OSD_ROWS][OSD_ROW_FIELDS],
                        camlink_setup_hint_t setup,
                        const camlink_osd_extra_t *extra,
                        char out[4][17]);

#endif /* CAMLINK_H */
