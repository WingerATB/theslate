/* SPDX-License-Identifier: PolyForm-Strict-1.0.0
 *
 * Runtime configuration, persisted in NVS.
 *
 * These were compile-time Kconfig values, which meant changing a record mode or
 * an AUX channel required a rebuild and a reflash. They are now settings, with
 * the Kconfig values acting only as first-boot defaults. This is also the data
 * model the eventual web UI edits -- getting it right here means the UI is just
 * an editor for a format that already exists.
 */
#ifndef CAMLINK_CFG_H
#define CAMLINK_CFG_H

#include <stdint.h>
#include <stdbool.h>

#include "osd_fields.h"

/* Stored values are permanent: a module in the field has one of these in NVS,
 * and reusing a number for a different meaning would silently change what
 * somebody's quad does on the update that did it. Append only. */
typedef enum {
    CFG_MODE_ARM    = 0,   /* arm starts recording, disarm stops            */
    CFG_MODE_SWITCH = 1,   /* an RC channel controls recording              */
    CFG_MODE_CUT    = 2,   /* arm records; a press cuts and restarts        */
    CFG_MODE_BOTH   = 3,   /* arm records; a press independently toggles    */
    CFG_MODE__COUNT
} cfg_mode_t;

/* How the RC channel is read.
 *
 * A toggle switch HOLDS a position, so asking "is it inside the window" is the
 * whole question. A momentary button does not: each press moves the channel to
 * a new value and leaves it there, so the same question answers the same way
 * twice running and the second press appears to do nothing. Watching for a
 * CHANGE instead is what makes a button work -- and it costs the window its
 * meaning, which is why this is a mode rather than a cleverness applied to
 * both. */
typedef enum {
    CFG_SW_LEVEL  = 0,   /* records while the channel sits inside the window */
    CFG_SW_BUTTON = 1,   /* every change of the channel counts as one press   */
} cfg_switch_kind_t;

/* How far the channel must move to count as a press.
 *
 * Well above the few microseconds of jitter a receiver produces at rest, and
 * well below the ~500 us step between the detents of a three-position switch,
 * so neither a still channel nor a deliberate flick can be mistaken for the
 * other. */
#define CAMLINK_PRESS_DELTA_US  50

/* Where the setup switch counts as thrown. Open-ended above, for the reason the
 * whole scale note above exists: a receiver at +100% lands near 2012 us and an
 * extended endpoint goes further, so an upper bound here would only ever
 * exclude the position the user was pointing at. */
#define CAMLINK_AUX_HIGH_US   1700

/* The hold time is a setting, 0..10 s in tenths. Two seconds is the default:
 * long enough that a knock or a stick sweeping past a detent cannot reboot the
 * module, short enough to feel like a control rather than a wait. Zero is
 * allowed and means the instant the switch crosses -- the user's aircraft, the
 * user's call. */
/* May the module enter setup right now?
 *
 * One predicate, used by every route in: the front-panel button, the setup
 * switch and the bench console. It started as an inline check on the switch
 * path only, which left the ten-second button hold able to do the same thing
 * with no guard at all -- the same decision written in two places and only one
 * of them correct, which is the argument for it being neither inline nor
 * duplicated.
 *
 * Entering setup reboots the module, drops the camera link and raises a Wi-Fi
 * access point. None of that belongs in the air.
 *
 * `boxarm_known` matters as much as `armed`: until the FC has said which bit of
 * flightModeFlags is BOXARM, `armed` is a default rather than a reading, and a
 * default must not be allowed to read as "disarmed, go ahead". So a live link
 * that has not resolved BOXARM yet refuses too. No link at all is a different
 * situation -- a bench, or a module on USB -- and is allowed, because otherwise
 * a module with no flight controller could never be set up.
 */
static inline bool camlink_may_enter_setup(bool link_up, bool boxarm_known, bool armed)
{
    if (!link_up)      return true;    /* no FC to be armed by */
    if (!boxarm_known) return false;   /* cannot tell yet, so do not */
    return !armed;
}

#define CAMLINK_CFG_HOLD_DS_MAX     100
#define CAMLINK_CFG_HOLD_DS_DEFAULT  20

/* Press detector for BUTTON mode, kept here rather than inside the logic task
 * so the host tests can exercise the shipped code instead of a copy of it. The
 * last two faults in this project reached hardware because the path that had
 * them could not be run at a desk. */
typedef struct {
    uint16_t last_us;
    bool     seen;
} camlink_press_t;

/* Feed one channel reading. Returns true exactly once per press.
 *
 * `us` of 0 means "nothing to read" -- a receiver never produces it -- and is
 * held rather than treated as movement, so an RC dropout cannot manufacture a
 * press. The first real reading is adopted as a position, not a press, for the
 * same reason a switch's first reading is not an edge: the channel was already
 * wherever it is before anyone was watching. */
static inline bool camlink_press_update(camlink_press_t *st, uint16_t us)
{
    if (us == 0) return false;
    if (!st->seen) { st->last_us = us; st->seen = true; return false; }

    uint16_t d = (us > st->last_us) ? (uint16_t)(us - st->last_us)
                                    : (uint16_t)(st->last_us - us);
    if (d < CAMLINK_PRESS_DELTA_US) return false;

    st->last_us = us;
    return true;
}

/* Transmit power, as an index into a fixed table rather than an
 * esp_power_level_t. The enum's numeric values have shifted between IDF
 * releases, and this blob outlives any single build -- storing the raw enum
 * would mean a firmware update could silently reinterpret a stored -24 dBm as
 * something loud. An index into a list this project owns cannot drift. */
typedef enum {
    CFG_TX_N24 = 0,   /* flight default */
    CFG_TX_N12 = 1,
    CFG_TX_N0  = 2,
    CFG_TX_P3  = 3,
    CFG_TX__COUNT
} cfg_tx_t;

/* Rung 0, by what it MEANS rather than by what it measures on one part.
 *
 * The stored names date from a firmware that only ran on a C3, where rung 0 is
 * -24 dBm. It is -15 dBm on a C6 and -12 dBm on the original ESP32 -- the
 * names cannot change, because the numbers behind them are in every shipped
 * module's NVS, but code that means "as quiet as this radio goes" should not
 * have to say "N24" to ask for it. board.h owns what each rung is worth. */
#define CFG_TX_QUIETEST  CFG_TX_N24

/* Persisted in NVS. Fields may only ever be APPENDED: the loader copies a
 * short stored blob over the defaults and leaves the rest at their default
 * values, which is what makes a firmware update that adds a setting keep the
 * user's existing ones. Reordering or resizing an existing field breaks that. */
typedef struct {
    uint8_t  mode;              /* cfg_mode_t                               */
    uint8_t  switch_channel;    /* RAW index. AUX n is index n+3 -- MSP_RC
                                 * returns rcData[] with the four sticks
                                 * first, so AUX1 is index 4.              */
    uint16_t range_min;         /* inclusive, us                            */
    uint16_t range_max;         /* inclusive, us                            */
    uint8_t  tx_power;          /* cfg_tx_t                                 */
    uint8_t  cfg_ver;           /* see the migration note below             */
    /* Which fields go on which OSD row. Appended, so a blob written before
     * the rows became configurable simply keeps the default layout -- which
     * is the layout the module shipped with, so an update changes nothing on
     * screen until the user asks it to. */
    uint8_t  osd[OSD_ROWS][OSD_ROW_FIELDS];
    uint8_t  switch_kind;       /* cfg_switch_kind_t                        */
    uint8_t  tx_auto;           /* 1 = drop to the quiet level while armed  */
    uint8_t  cfg_channel;       /* RAW index of the setup switch, 0 = off.
                                 * Deliberately not an AUX number: 0 would be
                                 * a valid channel and there would be no way
                                 * to say "no switch".                       */
    uint8_t  cfg_kind;          /* cfg_switch_kind_t, for the setup control  */
    uint8_t  cfg_hold_ds;       /* how long a SWITCH must be held, in tenths
                                 * of a second, 0..100. Ignored by a BUTTON,
                                 * which has no position to hold.            */
    /* Appended together. A blob written before they existed keeps the
     * defaults below, which is what makes the update that adds them change
     * nothing about how somebody's quad already behaves. */
    uint8_t  stop_delay_s;      /* keep recording this long after an
                                 * automatic stop. 0 = off, and off is the
                                 * default -- see the note above.           */
    uint8_t  warn_batt_pct;     /* warn at or below this camera battery %.
                                 * 0 = never warn about the battery.        */
    uint8_t  warn_card_min;     /* warn at or below this many minutes of
                                 * card time. 0 = never warn about the card.*/
} camlink_cfg_t;

/* --------------------------------------------------------------------------
 * The pulse-width scale.
 *
 * MSP_RC hands back Betaflight's rcData[], which is the receiver's own output
 * converted to microseconds -- NOT clamped to the tidy 1000-2000 band. Both
 * protocols that matter here put a channel at +/-100% well outside it:
 *
 *   CRSF / ELRS   raw 172 .. 1811  ->  (raw * 1024 / 1639) + 881  ->  988 .. 2012
 *   SBUS          raw 172 .. 1811  ->  (raw * 5 / 8) + 880        ->  988 .. 2012
 *
 * So a two-position switch flicked high reads 2012 us, and a radio with
 * extended endpoints reads higher still. A window that stopped at 2000 could
 * not contain the one position the user was actually pointing at, which is
 * exactly the bug this scale exists to prevent. 900-2100 is also the range
 * Betaflight's own Modes tab offers, so the numbers match what the user sees
 * in the configurator.
 * -------------------------------------------------------------------------- */
#define CAMLINK_RC_US_MIN   900
#define CAMLINK_RC_US_MAX  2100

/* Config blob schema. Bumped when a stored value needs reinterpreting rather
 * than merely extending:
 *   0  written when the window topped out at 2000 us
 *   1  written by a build that can express the full 900-2100 range
 * Blobs from before this field existed carry a zero here -- the struct was
 * padded to eight bytes and s_cfg lives in BSS, so the byte was written as 0.
 */
#define CAMLINK_CFG_VER  1

/* Does this pulse width sit inside the recording window?
 *
 * A limit parked at the end of its travel means "and beyond". A receiver at
 * +100% is not obliged to stop on the number the slider stops on, so a window
 * that ended exactly at the top of the scale would still exclude an endpoint
 * that overshot it -- the failure this whole comment block is about, moved a
 * hundred microseconds up. Open ends remove it for good.
 *
 * Shared with the host tests, which is why it lives in the header: a predicate
 * this load-bearing should be exercised by the tests themselves, not by a copy
 * of it that can drift.
 */
static inline bool camlink_cfg_in_window(uint16_t us, const camlink_cfg_t *c)
{
    bool above = (c->range_min <= CAMLINK_RC_US_MIN) || (us >= c->range_min);
    bool below = (c->range_max >= CAMLINK_RC_US_MAX) || (us <= c->range_max);
    return above && below;
}

/* --------------------------------------------------------------------------
 * What the automatic source wants, and the manual override on top of it.
 *
 * Both are pure and live here rather than inside camlink_logic_task(), so the
 * host tests exercise the shipped code. This is the third time that rule has
 * earned itself: the switch window, the button press detector and now this.
 * -------------------------------------------------------------------------- */

/* The mode's own opinion, with no manual input considered. */
static inline bool camlink_auto_want(uint8_t mode, uint8_t kind,
                                     bool armed, bool sw, bool rc_latch)
{
    switch (mode) {
    case CFG_MODE_SWITCH:
        return (kind == CFG_SW_BUTTON) ? rc_latch : sw;
    case CFG_MODE_ARM:
    case CFG_MODE_CUT:
    case CFG_MODE_BOTH:
    default:
        /* A default that records with the aircraft is the safe answer for a
         * mode byte we do not recognise: it cannot leave a clip running after
         * the pilot lands. */
        return armed;
    }
}

typedef struct {
    bool active;      /* an override is in force                              */
    bool want;        /* what it wants                                        */
    bool confirmed;   /* the camera has been seen agreeing with it at least
                       * once, so a later disagreement means somebody else
                       * changed it                                           */
} camlink_ovr_t;

typedef struct {
    bool press;          /* a manual press happened this tick                 */
    bool auto_want;      /* camlink_auto_want() for this tick                 */
    bool auto_moved;     /* the automatic source changed its mind this tick   */
    bool authority;      /* an automatic source is live, or none is expected  */
    bool cam_connected;
    bool cam_valid;      /* the camera's own state is current, not stale      */
    bool cam_recording;
    bool unsent;         /* the last change of intent has not been delivered  */
} camlink_ovr_in_t;

/* Fold one tick into the override and return the effective intent.
 *
 * THE RULE THAT MAKES THIS SAFE is the asymmetry: an override may hold the
 * camera IDLE against anything at all, but it may only hold it RECORDING while
 * there is a live automatic authority AND a camera connected to hear it.
 *
 * Every dangerous sequence found while reviewing this is an override holding
 * RECORD past something -- a severed UART, a camera that was not there when the
 * button was pressed, a reboot into config mode. Holding IDLE past those same
 * events is harmless in every one of them, so one rule closes the lot instead
 * of six special cases. It also means the button can only ever close a clip in
 * a degraded state, never open one, which is strictly stronger than the
 * existing "a dead UART must close an open clip" rule rather than a weakening
 * of it.
 */
static inline bool camlink_ovr_update(camlink_ovr_t *o, const camlink_ovr_in_t *in)
{
    /* The automatic source moved, so it takes control back. This is what makes
     * the override temporary without a timer: arm, disarm or move the switch
     * and the mode owns recording again. */
    if (in->auto_moved) {
        o->active = false;
        o->confirmed = false;
    }

    /* Somebody used the camera itself. Stopping a clip on the camera body is an
     * instruction too, and without this the reconcile loop would see the
     * mismatch and restart it every retry period -- fighting the person holding
     * it. Only counts once the camera has been seen agreeing with the override,
     * so a command still in flight is not mistaken for a disagreement. */
    if (o->active && o->confirmed && in->cam_valid &&
        in->cam_recording != o->want) {
        o->active = false;
        o->confirmed = false;
    }

    if (in->press) {
        /* Absorb a press while a change of intent is still undelivered.
         *
         * Commands are rate limited to one per RECORD_RETRY_MS, and inverting
         * on every press inside that window makes an even number of presses
         * cancel out: press, press again because nothing happened, and the
         * module sends nothing at all. Absorbing means N impatient presses do
         * one thing rather than nothing. */
        if (!in->unsent) {
            bool now  = o->active ? o->want : in->auto_want;
            /* Invert what the CAMERA is doing when that is known, rather than
             * what we last asked for -- they differ while a command is landing,
             * and the camera is what the user is looking at. */
            o->want      = in->cam_valid ? !in->cam_recording : !now;
            o->active    = true;
            o->confirmed = false;
        }
    }

    /* The asymmetry. Demoted to IDLE rather than released, because releasing
     * would fall back to auto_want -- which in SWITCH mode deliberately holds
     * its last value through an RC dropout, and would therefore restart the
     * very clip the user just stopped. */
    if (o->active && o->want && (!in->authority || !in->cam_connected)) {
        o->want      = false;
        o->confirmed = false;
    }

    if (o->active && in->cam_valid && in->cam_recording == o->want) {
        o->confirmed = true;
    }

    return o->active ? o->want : in->auto_want;
}

/* --------------------------------------------------------------------------
 * Post-roll
 *
 * Keep recording for a few seconds after the AUTOMATIC stop, so the footage
 * does not end before it shows where the aircraft went. A crash disarms, and a
 * disarm is what closes the clip; the seconds you actually want are the ones
 * after that.
 *
 * Named the way broadcast names it. Post-roll is the padding at the tail of a
 * recording, and its opposite -- pre-roll, the seconds captured BEFORE the
 * trigger -- is the other half of the same idea, and one the camera itself can
 * do (DJI calls it pre-recording, GoPro calls it HindSight). Naming this one
 * properly leaves that one an obvious name when it arrives.
 *
 * Pure and here rather than inside the logic task, for the same reason as
 * camlink_ovr_update() below it: the host tests exercise the shipped code.
 *
 * FOUR RULES MAKE IT SAFE, and every one of them is a way of NOT weakening a
 * guarantee that already exists:
 *
 *   1. Only the automatic source's stop is delayed. A press of the button or
 *      the record switch is a deliberate act and takes effect at once -- the
 *      person pressing stop is looking at the aircraft.
 *
 *   2. The automatic source wanting to record again cancels it. Re-arming does
 *      not restart a clip, it simply continues the one that never stopped.
 *
 *   3. Losing authority or the camera cancels it immediately. The existing
 *      rule is that a dead UART closes an open clip. A delay that outlived a
 *      dead UART would quietly repeal that rule, which is exactly the class of
 *      thing camlink_ovr_update()'s asymmetry exists to prevent: a hold on
 *      RECORD needs a live authority and a camera to hear it.
 *
 *   4. Zero seconds means off, and off is the default. A module updating to a
 *      firmware that gained this setting must not start behaving differently
 *      from the one its owner flew last week.
 *
 * CUT mode is untouched: a cut is a stop followed immediately by a start,
 * issued directly rather than through the intent, so there is no automatic
 * stop edge here to delay. Delaying one would turn a cut into a pause.
 * -------------------------------------------------------------------------- */
#define CAMLINK_STOP_DELAY_MAX_S  30

typedef struct {
    bool     active;
    uint32_t until_ms;
} camlink_stopdelay_t;

typedef struct {
    bool     auto_stop;       /* the automatic source went record -> stop now */
    bool     want;            /* the effective intent for this tick           */
    bool     overridden;      /* a manual override is in force -- hands off   */
    bool     authority;       /* an automatic source is live, or none expected*/
    bool     cam_connected;
    uint8_t  delay_s;         /* the setting; 0 disables the whole thing      */
    uint32_t now_ms;
} camlink_stopdelay_in_t;

/* Fold one tick in and return the effective intent.
 *
 * `remain_s`, when not NULL, is the countdown for the OSD -- 0 when nothing is
 * being held. Showing it is not decoration: a camera that keeps recording for
 * five seconds after landing, with nothing on screen saying why, is
 * indistinguishable from a stop command that failed.
 */
static inline bool camlink_stopdelay_update(camlink_stopdelay_t *st,
                                            const camlink_stopdelay_in_t *in,
                                            uint8_t *remain_s)
{
    if (remain_s) *remain_s = 0;

    /* Off, or hands off. Any latched hold is dropped rather than left to
     * expire, so turning the setting to zero takes effect at once. */
    if (in->delay_s == 0 || in->overridden) {
        st->active = false;
        return in->want;
    }

    /* Rule 2, and it comes first: a source that wants to record again ends the
     * hold whether or not one was running. Checked before the start edge so a
     * tick that is both cannot leave a hold armed behind a live recording. */
    if (in->want) {
        st->active = false;
        return true;
    }

    /* Rule 3. Demoted the moment the thing that would have to hear a stop is
     * gone -- the same asymmetry as the manual override: holding IDLE past a
     * failure is harmless, holding RECORD past one is not. */
    if (!in->authority || !in->cam_connected) {
        st->active = false;
        return false;
    }

    if (in->auto_stop && !st->active) {
        st->active   = true;
        st->until_ms = in->now_ms + (uint32_t)in->delay_s * 1000u;
    }

    if (!st->active) return false;

    /* Unsigned subtraction, so this stays correct across the 32-bit
     * millisecond wrap rather than holding the clip open for 49 days. */
    if ((int32_t)(in->now_ms - st->until_ms) >= 0) {
        st->active = false;
        return false;
    }

    if (remain_s) {
        uint32_t left = st->until_ms - in->now_ms;
        /* Round up: a countdown that shows 0 while still recording is the
         * confusion this display exists to remove. */
        *remain_s = (uint8_t)((left + 999u) / 1000u);
    }
    return true;
}

/* Betaflight numbers AUX channels from 1, after the four sticks. Users think
 * in AUX numbers; MSP_RC speaks raw indices. Convert at the edges only. */
static inline uint8_t cfg_aux_to_index(uint8_t aux) { return (uint8_t)(aux + 3); }
static inline int     cfg_index_to_aux(uint8_t idx) { return (int)idx - 3; }

/* Load from NVS, falling back to the Kconfig defaults on first boot. */
void camlink_cfg_init(void);

/* Snapshot. Never returns invalid values -- anything out of range is clamped
 * to a default, so a corrupt NVS blob cannot produce a state machine that
 * watches channel 200. */
void camlink_cfg_get(camlink_cfg_t *out);

/* Apply and persist. Returns false if a value was out of range. */
bool camlink_cfg_set_mode(uint8_t mode);
bool camlink_cfg_set_channel(uint8_t ch);
bool camlink_cfg_set_range(uint16_t lo, uint16_t hi);
bool camlink_cfg_set_tx_power(uint8_t tx);
bool camlink_cfg_set_switch_kind(uint8_t kind);
bool camlink_cfg_set_tx_auto(uint8_t on);
bool camlink_cfg_set_config_channel(uint8_t ch);
bool camlink_cfg_set_config_kind(uint8_t kind);
bool camlink_cfg_set_config_hold(uint8_t ds);
bool camlink_cfg_set_stop_delay(uint8_t s);
bool camlink_cfg_set_warn(uint8_t batt_pct, uint8_t card_min);

/* Replace the whole OSD layout. Rejected as a unit if any row would not fit in
 * OSD_ROW_MAX characters -- a row that overflows loses its last field silently
 * on screen, which is exactly the failure the length budget exists to prevent.
 * Unknown field ids are rejected rather than quietly dropped. */
bool camlink_cfg_set_osd(const uint8_t osd[OSD_ROWS][OSD_ROW_FIELDS]);

/* The layout the module ships with, for "reset to default" and for the loader
 * when nothing is stored. */
void camlink_cfg_default_osd(uint8_t out[OSD_ROWS][OSD_ROW_FIELDS]);

const char *camlink_cfg_mode_name(uint8_t mode);
const char *camlink_cfg_switch_kind_name(uint8_t kind);
/* There is deliberately no camlink_cfg_tx_name(). It used to live here with the
 * C3's four figures written into it, which made it a second source of truth
 * that was wrong on any part with a different ladder -- a C6 logging "-24 dBm"
 * for a radio at -15 dBm. board_tx_name() in board.h is the only one. */

#endif
