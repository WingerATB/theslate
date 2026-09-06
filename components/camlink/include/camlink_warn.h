/* SPDX-License-Identifier: PolyForm-Strict-1.0.0
 *
 * The warning ladder: what the OSD says instead of REC or IDLE when something
 * is wrong.
 *
 * WHY THIS EXISTS. Until now the state field said REC or IDLE and nothing
 * else, which means the one failure this whole product was built to prevent --
 * flying a pack and landing to find the camera never rolled -- was reported by
 * a field reading IDLE, in a corner of the screen, next to three other numbers.
 * "IDLE" is not a warning. It is the same word the module shows while it sits
 * on the bench doing exactly what it should.
 *
 * Pure, and separate from both the clock and the renderer, so the host tests
 * exercise the shipped decision rather than a copy of it. That rule has now
 * earned itself five times: the switch window, the press detector, the manual
 * override, the setup guard, and this.
 *
 * WHAT IS DELIBERATELY NOT HERE: the grace period. Deciding whether a want has
 * gone unmet for long enough needs a clock, and a clock is what makes a
 * function untestable at a desk. The caller does the timing and passes the
 * answer in as a bool.
 */
#ifndef CAMLINK_WARN_H
#define CAMLINK_WARN_H

#include <stdint.h>
#include <stdbool.h>

/* Ordered by severity, ascending, and that IS load-bearing: everything from
 * CAM_WARN_NOT_REC upwards means "there is no recording happening right now",
 * which is what the LED escalates on, while the two below it mean "there will
 * not be one much longer". A single >= test separates them.
 *
 * The full priority order is still written out as a list in
 * camlink_warn_pick(), because a reader should not have to infer it from
 * integers. Stored nowhere, so the values are free to change. */
typedef enum {
    CAM_WARN_NONE = 0,
    CAM_WARN_SD_LOW,     /* card time is nearly up                          */
    CAM_WARN_BATT_LOW,   /* camera battery is nearly flat                   */
    CAM_WARN_NOT_REC,    /* we asked it to record and it is not             */
    CAM_WARN_HOT,        /* too hot to record                               */
    CAM_WARN_NO_SD,      /* no card, or the card is full                    */
} camlink_warn_t;

/* Everything the decision needs, all of it already known to the logic task.
 *
 * The _valid flags are not optional politeness. The governing rule of this
 * firmware is that a field is believed only if the camera confirmed it, and a
 * warning invented from a value the camera never sent would be the worst
 * possible version of breaking it: an alarm with nothing behind it, on the one
 * screen the pilot is relying on. Every threshold below is skipped when its
 * reading is unconfirmed. */
typedef struct {
    bool     cam_gone;        /* no camera at all -- NO CAM already says so   */

    bool     want_record;     /* the effective intent, whatever produced it   */
    bool     rec_valid;       /* the camera's own recording state is current  */
    bool     recording;
    bool     want_unmet_long; /* want_record has gone unmet past the grace    */

    bool     temp_valid;
    uint8_t  temp_over;       /* 0 ok, 1 warn, 2 too hot, 3 shutting down     */

    bool     card_valid;
    uint32_t card_s;          /* recording seconds left on the card           */
    /* The card cannot be recorded to at all. A separate signal from card_s,
     * because a camera reporting "no card" has no time remaining to report --
     * so a warning that depended on card_s being present would go quiet in
     * exactly the case it exists for. */
    bool     card_fault;

    bool     batt_valid;
    uint8_t  batt_pct;

    /* Thresholds, from the user's settings. Zero means "do not warn about
     * this" -- a threshold is one person's opinion about someone else's
     * aircraft, and it has to be possible to disagree with it. */
    uint8_t  warn_batt_pct;
    uint16_t warn_card_s;
} camlink_warn_in_t;

/* The highest-priority thing that is true, or CAM_WARN_NONE.
 *
 * The order is the argument. Top of the list is what stops a recording from
 * being possible at all; bottom is what will stop one later. A pilot glancing
 * at one field gets the most actionable thing, and the rest is still on the
 * battery and card rows where it always was.
 */
static inline camlink_warn_t camlink_warn_pick(const camlink_warn_in_t *in)
{
    /* No camera is not a warning, it is a state, and the state field already
     * reads NO CAM. Layering a warning on top would replace the one piece of
     * information that explains every blank field beside it. */
    if (in->cam_gone) return CAM_WARN_NONE;

    /* Cannot record at all. First because it is the only one the pilot can
     * still do something about while standing next to the aircraft.
     *
     * Two ways to reach it: the camera says the card is faulty or absent, or
     * it says there is no time left on it. The first does not imply the
     * second -- a camera with no card has nothing to say about time. */
    if (in->card_fault) return CAM_WARN_NO_SD;
    if (in->card_valid && in->card_s == 0) return CAM_WARN_NO_SD;

    /* 2 = too hot to record, 3 = about to shut down. 1 is the camera's own
     * early warning and it still records, so it is not ours to raise. */
    if (in->temp_valid && in->temp_over >= 2) return CAM_WARN_HOT;

    /* THE ONE THIS WAS BUILT FOR. We asked for a recording, the camera says
     * there is not one, and enough time has passed that this is not simply the
     * shutter delay.
     *
     * Gated on rec_valid: a camera that has gone quiet is not a camera that is
     * refusing, and reporting a refusal we cannot see would be inventing it. */
    if (in->want_record && in->rec_valid && !in->recording && in->want_unmet_long) {
        return CAM_WARN_NOT_REC;
    }

    if (in->batt_valid && in->warn_batt_pct &&
        in->batt_pct <= in->warn_batt_pct) {
        return CAM_WARN_BATT_LOW;
    }

    if (in->card_valid && in->warn_card_s &&
        in->card_s <= in->warn_card_s) {
        return CAM_WARN_SD_LOW;
    }

    return CAM_WARN_NONE;
}

/* The word for the state slot. Never wider than OSD_F_STATE's 7-character
 * budget, which is what stops a warning pushing the field beside it sideways.
 * NULL for CAM_WARN_NONE, so the caller falls through to REC / IDLE. */
static inline const char *camlink_warn_word(camlink_warn_t w)
{
    switch (w) {
    case CAM_WARN_NO_SD:    return "NO SD";
    case CAM_WARN_HOT:      return "CAM HOT";
    case CAM_WARN_NOT_REC:  return "NOT REC";
    case CAM_WARN_BATT_LOW: return "BAT LOW";
    case CAM_WARN_SD_LOW:   return "SD LOW";
    default:                return NULL;
    }
}

/* How long a want may go unmet before it counts as a fault rather than as the
 * camera still getting started.
 *
 * Arm-to-recording is a BLE round trip plus the camera's own shutter delay --
 * measured at 0.5 to 1 second, and documented as unavoidable. A warning that
 * fired inside that window would appear on every single arm, and a warning
 * that appears every time is one people stop reading. Three seconds clears the
 * measured worst case with room, and is still short enough to see before the
 * quad leaves the ground. */
#define CAMLINK_NOT_REC_GRACE_MS  3000

#endif /* CAMLINK_WARN_H */
