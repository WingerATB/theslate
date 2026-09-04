/* SPDX-License-Identifier: PolyForm-Strict-1.0.0
 *
 * OSD line formatting, deliberately split out with no FreeRTOS or ESP-IDF
 * dependencies so the host tests in test/host exercise this exact code rather
 * than a copy of it.
 *
 * The governing rule: a field is rendered only if the camera confirmed it.
 * A blank slot is correct. A wrong readout is worse than no readout.
 *
 * TWO Betaflight rendering constraints shape the output, both verified in
 * src/main/osd/osd_elements.c:
 *
 * 1. An EMPTY string does not hide the element -- osdElementCustomMsg() falls
 *    back to printing the literal placeholder "CUSTOM_MSG1". So "blank" must be
 *    sent as spaces, never as "".
 *
 * 2. Elements are drawn as plain strings at a fixed position, with no clearing
 *    of what was there before. A shorter string leaves the tail of the previous
 *    one on screen ("IDLE" after "CAM LOST" would read "IDLEOST"). Every slot is
 *    therefore padded to a fixed width.
 */

#include <stdio.h>
#include <string.h>
#include "camlink.h"
#include "osd_fields.h"

/* The catalogue. Widths are worst cases -- see the note in osd_fields.h. */
const osd_field_info_t osd_fields[OSD_F__COUNT] = {
    [OSD_F_NONE]       = { "",           "",          0 },
    [OSD_F_STATE]      = { "State",      "NO CAM",    7 },  /* "CAM HOT" */
    [OSD_F_CLIP]       = { "Clip time",  "12:34",     7 },  /* "1092:15" */
    [OSD_F_BATTERY]    = { "Battery",    "87%",       5 },  /* icon + "100%" on screen */
    [OSD_F_BATT_PCT]   = { "Battery %",  "87%",       4 },  /* "100%" */
    [OSD_F_CARD]       = { "Card left",  "SD 1H49",   8 },  /* "SD 99H59" */
    [OSD_F_CARD_SHORT] = { "Card",       "1H49",      5 },  /* "99H59" */
    [OSD_F_DOT]        = { "Alive dot",  ".",         1 },
    [OSD_F_CAMERA]     = { "Camera",     "O360",      4 },
    [OSD_F_WARN]       = { "Warning",    "CAM HOT",   7 },  /* blinks when set */
    [OSD_F_RES]        = { "Resolution", "5.3K",      4 },  /* GoPro           */
    [OSD_F_FPS]        = { "FPS",        "240",       3 },  /* GoPro           */
};

int osd_row_width(const uint8_t row[OSD_ROW_FIELDS])
{
    int w = 0, n = 0;
    for (int i = 0; i < OSD_ROW_FIELDS; i++) {
        uint8_t f = row[i];
        if (f == OSD_F_NONE || f >= OSD_F__COUNT) continue;
        if (n++) w += 1;                    /* one space between fields */
        w += osd_fields[f].width;
    }
    return w;
}

int osd_row_fits(const uint8_t row[OSD_ROW_FIELDS])
{
    return osd_row_width(row) <= OSD_ROW_MAX;
}

/* The layout used when none is stored: exactly what the module shipped with
 * before the rows became configurable, so an update changes nothing on screen
 * until the user asks it to. */
static const uint8_t default_layout[OSD_ROWS][OSD_ROW_FIELDS] = {
    { OSD_F_STATE,   OSD_F_DOT,  OSD_F_NONE, OSD_F_NONE },
    { OSD_F_CLIP,    OSD_F_NONE, OSD_F_NONE, OSD_F_NONE },
    { OSD_F_BATTERY, OSD_F_NONE, OSD_F_NONE, OSD_F_NONE },
    { OSD_F_CARD,    OSD_F_NONE, OSD_F_NONE, OSD_F_NONE },
};

/* Render one field, or return 0 if the camera has not confirmed it.
 *
 * The governing rule is unchanged by the layout becoming configurable: a field
 * appears only if the camera actually reported it. Moving a field to a
 * different row does not make it any more known than it was. */
static const char *setup_word(camlink_setup_hint_t h)
{
    switch (h) {
    case CAMLINK_SETUP_ARMING:   return "ENTER";
    case CAMLINK_SETUP_READY:    return "READY";
    case CAMLINK_SETUP_ENTERING: return "CONFIG";
    default:                     return NULL;
    }
}

/* The single most important warning right now, or NULL.
 *
 * Priority is "what needs doing before the next second". Nothing here is shown
 * unless the camera confirmed the reading it rests on -- a warning built on a
 * value the camera never sent would be worse than no warning. Thresholds are
 * fixed for now; making them configurable is a follow-up.
 *
 * REC? is first and it is the whole point of the product: the module wants a
 * recording (armed, or the switch is on) and the camera is not rolling. That is
 * the failure you can still do something about while the props are spinning. */
static const char *osd_warning(const cam_status_t *cam, bool want_recording, bool cam_gone)
{
    if (cam_gone) return NULL;                    /* the state field says NO CAM */
    if (want_recording && !(cam->recording && cam->recording_valid)) return "REC?";
    if (cam->temp_over_valid && cam->temp_over >= 2)                  return "CAM HOT";
    if (cam->remain_time_valid && cam->remain_time_s == 0)           return "NO SD";
    if (cam->remain_time_valid && cam->remain_time_s < 120)          return "SD LOW";
    if (cam->battery_valid && cam->battery_pct <= 15)                return "BAT LOW";
    return NULL;
}

/* Pick the battery glyph for a charge level: full (0x90) down to empty (0x96),
 * rounded across the seven steps -- 100% is full, 0% is empty. This is the same
 * idea Betaflight uses for its own battery gauge, so the icon reads the way a
 * pilot already expects. */
static char osd_batt_glyph(uint8_t pct)
{
    if (pct > 100) pct = 100;
    int idx = ((100 - (int)pct) * (OSD_SYM_BATT_EMPTY - OSD_SYM_BATT_FULL) + 50) / 100;
    if (idx < 0) idx = 0;
    if (idx > (OSD_SYM_BATT_EMPTY - OSD_SYM_BATT_FULL)) idx = OSD_SYM_BATT_EMPTY - OSD_SYM_BATT_FULL;
    return (char)(OSD_SYM_BATT_FULL + idx);
}

static int render_field(uint8_t f, const cam_status_t *cam, bool heartbeat,
                        bool cam_gone, bool want_recording,
                        camlink_setup_hint_t setup,
                        char *buf, size_t cap)
{
    buf[0] = '\0';

    /* The setup gesture takes the state slot while it is running. It is the
     * only thing on screen that is about to change what the module IS, so it
     * outranks what the camera happens to be doing -- and the pilot needs to
     * see the gesture land before committing to it. Every other field carries
     * on as normal: the battery and card readings are still true, and still
     * worth having while deciding. */
    if (f == OSD_F_STATE) {
        const char *w = setup_word(setup);
        if (w) { snprintf(buf, cap, "%s", w); return 1; }
    }

    if (f == OSD_F_DOT) {
        buf[0] = heartbeat ? '.' : ' ';
        buf[1] = '\0';
        return 1;
    }

    /* Nothing is shown for a camera that is not talking, except that it is not
     * talking -- which is what the state field is for.
     *
     * The camera NAME goes too, and it is worth saying why, because the
     * opposite is defensible and was tried: the binding is known whether or not
     * the camera answers, so the name could survive and distinguish "the 360
     * dropped out" from "no camera is configured". It reads as clutter in
     * practice. Every other field blanks, the state slot already says NO CAM,
     * and a name left glowing beside it suggests a camera that is still there.
     * Which camera is bound is a setup question, and the settings page answers
     * it without costing a row in flight. */
    if (cam_gone) {
        if (f != OSD_F_STATE) return 0;
        snprintf(buf, cap, "NO CAM");
        return 1;
    }


    if (f == OSD_F_CAMERA) {
        snprintf(buf, cap, "%s", cam->label[0] ? cam->label : "CAM");
        return 1;
    }

    if (f == OSD_F_RES) {
        if (!cam->res_valid) return 0;
        snprintf(buf, cap, "%s", cam->res);
        return 1;
    }
    if (f == OSD_F_FPS) {
        if (!cam->fps_valid) return 0;
        snprintf(buf, cap, "%s", cam->fps);
        return 1;
    }

    if (f == OSD_F_WARN) {
        const char *w = osd_warning(cam, want_recording, cam_gone);
        if (!w) return 0;                 /* nothing wrong: blank, holds place */
        /* Blink so it catches the eye rather than sitting there like a label. */
        snprintf(buf, cap, "%s", heartbeat ? w : "");
        return 1;
    }

    switch (f) {
    case OSD_F_STATE:
        if (cam->temp_over_valid && cam->temp_over >= 2) {
            /* 2 = too hot to record, 3 = about to shut down. Worth saying: it
             * explains why recording silently is not happening. */
            snprintf(buf, cap, "CAM HOT");
        } else if (want_recording && !(cam->recording && cam->recording_valid)) {
            /* The module wants a recording and the camera is not rolling. This
             * is the alarm the whole product exists for, so the state slot --
             * the one field a pilot who picks only one will pick -- carries it,
             * blinking. */
            snprintf(buf, cap, "%s", heartbeat ? "REC?" : "");
        } else {
            snprintf(buf, cap, "%s", cam->recording ? "REC" : "IDLE");
        }
        return 1;

    case OSD_F_CLIP:
        if (!(cam->recording && cam->record_time_valid)) return 0;
        snprintf(buf, cap, "%02u:%02u", (unsigned)(cam->record_time_s / 60),
                 (unsigned)(cam->record_time_s % 60));
        return 1;

    case OSD_F_BATTERY:
        if (!cam->battery_valid) return 0;
        /* Battery glyph, then the percentage. Written as a byte rather than in
         * a string literal so the hex escape cannot swallow the digit after
         * it. */
        buf[0] = osd_batt_glyph(cam->battery_pct);
        snprintf(buf + 1, cap - 1, "%u%%", (unsigned)cam->battery_pct);
        return 1;

    case OSD_F_BATT_PCT:
        if (!cam->battery_valid) return 0;
        snprintf(buf, cap, "%u%%", (unsigned)cam->battery_pct);
        return 1;

    case OSD_F_CARD:
    case OSD_F_CARD_SHORT: {
        if (!cam->remain_time_valid) return 0;
        const char *label = (f == OSD_F_CARD) ? "SD " : "";
        uint32_t r = cam->remain_time_s;
        if (r >= 3600) {
            snprintf(buf, cap, "%s%uH%02u", label, (unsigned)(r / 3600),
                     (unsigned)((r % 3600) / 60));
        } else {
            snprintf(buf, cap, "%s%02u:%02u", label, (unsigned)(r / 60),
                     (unsigned)(r % 60));
        }
        return 1;
    }

    default:
        return 0;
    }
}

void camlink_format_osd(const cam_status_t *cam, bool msp_link_up,
                        bool want_recording, uint32_t clip_elapsed_s,
                        bool heartbeat, const uint8_t layout[OSD_ROWS][OSD_ROW_FIELDS],
                        camlink_setup_hint_t setup,
                        char out[4][17])
{
    (void)msp_link_up;
    (void)clip_elapsed_s;

    if (layout == NULL) layout = default_layout;

    /* A camera that has gone silent counts as gone, not merely unconfirmed.
     * The status goes stale on its own clock, well before BLE admits the link
     * is down; asking BLE instead left a window where the module held a link
     * nothing was arriving through and the state read "CAM ?". Two fixes for that
     * shipped wrong before this one, both because a camera cannot be made to
     * vanish at a desk. */
    bool cam_gone = !cam->connected || !cam->recording_valid;

    for (int r = 0; r < OSD_ROWS; r++) {
        char row[OSD_ROW_MAX + 1];
        int  used = 0;
        int  n    = 0;

        for (int i = 0; i < OSD_ROW_FIELDS; i++) {
            uint8_t f = layout[r][i];
            if (f == OSD_F_NONE || f >= OSD_F__COUNT) continue;

            char part[OSD_ROW_MAX + 1];
            if (!render_field(f, cam, heartbeat, cam_gone, want_recording, setup, part, sizeof(part))) {
                /* Unconfirmed: the field contributes nothing but still holds
                 * its place in the row's width, so the fields after it do not
                 * shuffle sideways every time a reading comes and goes. */
                part[0] = '\0';
            }

            int w = (int)osd_fields[f].width;
            if (n++ && used < OSD_ROW_MAX) row[used++] = ' ';

            int len = (int)strlen(part);
            for (int k = 0; k < w && used < OSD_ROW_MAX; k++) {
                row[used++] = (k < len) ? part[k] : ' ';
            }
        }

        /* Never empty: Betaflight prints the literal placeholder "CUSTOM_MSG1"
         * for an empty string rather than hiding the element. A row with
         * nothing in it is therefore one space, not "". */
        if (used == 0) row[used++] = ' ';
        row[used] = '\0';
        memcpy(out[r], row, (size_t)used + 1);
    }
}
