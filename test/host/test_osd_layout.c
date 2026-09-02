/* SPDX-License-Identifier: PolyForm-Strict-1.0.0 */
/* Host-side tests for the configurable OSD layout.
 *
 * The rule that makes this worth testing: Betaflight never clears a row, so
 * every row is padded to the WORST case its fields can produce. A row that
 * renders correctly with today's values can still be broken -- it just has not
 * met a three-digit battery or an hour-long clip yet. Every case below is built
 * from the worst case, not the typical one.
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "camlink.h"
#include "osd_fields.h"

static int fails = 0;
#define CHECK(cond, ...) do { if (!(cond)) { \
    printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

#define EXPECT(row, want) do { \
    if (strcmp(out[row], want) != 0) { \
        printf("  FAIL: row %d = [%s], expected [%s]\n", row, out[row], want); \
        fails++; \
    } } while (0)

static cam_status_t live(void)
{
    cam_status_t c = {0};
    c.connected = true;
    c.recording = true;       c.recording_valid = true;
    c.record_time_s = 754;    c.record_time_valid = true;   /* 12:34 */
    c.battery_pct = 87;       c.battery_valid = true;
    c.remain_time_s = 6540;   c.remain_time_valid = true;   /* 1H49 */
    return c;
}

int main(void)
{
    char out[4][17];
    printf("=== OSD layout ===\n\n");

    printf("1. row width is the sum of worst cases plus separators\n");
    {
        uint8_t r1[OSD_ROW_FIELDS] = { OSD_F_STATE, OSD_F_DOT, 0, 0 };
        CHECK(osd_row_width(r1) == 9, "state+dot = %d, expected 9", osd_row_width(r1));
        uint8_t r2[OSD_ROW_FIELDS] = { OSD_F_BATTERY, OSD_F_CARD, 0, 0 };
        CHECK(osd_row_width(r2) == 17, "bat+card = %d, expected 17", osd_row_width(r2));
        CHECK(!osd_row_fits(r2), "bat+card must NOT fit in %d", OSD_ROW_MAX);
        uint8_t r3[OSD_ROW_FIELDS] = { OSD_F_BATTERY, OSD_F_CARD_SHORT, 0, 0 };
        CHECK(osd_row_width(r3) == 14, "bat+card short = %d, expected 14", osd_row_width(r3));
        CHECK(osd_row_fits(r3), "bat+short card must fit");
        uint8_t r0[OSD_ROW_FIELDS] = { 0, 0, 0, 0 };
        CHECK(osd_row_width(r0) == 0, "empty row width must be 0");
    }

    printf("2. the default layout renders as it always did\n");
    {
        cam_status_t c = live();
        camlink_format_osd(&c, true, true, 0, true, NULL, CAMLINK_SETUP_NONE, out);
        EXPECT(0, "REC     ."); EXPECT(1, "12:34  ");
        EXPECT(2, "BAT 87% ");   EXPECT(3, "SD 1H49 ");
    }

    printf("3. two fields on one row, each padded to its own worst case\n");
    {
        const uint8_t l[OSD_ROWS][OSD_ROW_FIELDS] = {
            { OSD_F_STATE, OSD_F_CLIP, 0, 0 },
            { OSD_F_BATT_PCT, OSD_F_CARD_SHORT, OSD_F_DOT, 0 },
            { 0, 0, 0, 0 }, { 0, 0, 0, 0 },
        };
        cam_status_t c = live();
        camlink_format_osd(&c, true, true, 0, true, l, CAMLINK_SETUP_NONE, out);
        EXPECT(0, "REC     12:34  ");   /* 7 + sep + 7 */
        EXPECT(1, "87%  1H49  .");      /* 4 + sep + 5 + sep + 1 */
        EXPECT(2, " ");
        EXPECT(3, " ");
    }

    printf("4. an unconfirmed field holds its place rather than shuffling the row\n");
    {
        const uint8_t l[OSD_ROWS][OSD_ROW_FIELDS] = {
            { OSD_F_CLIP, OSD_F_BATTERY, 0, 0 },
            { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 },
        };
        cam_status_t c = live();
        c.recording = false;            /* clip time is not a thing right now */
        camlink_format_osd(&c, true, true, 0, false, l, CAMLINK_SETUP_NONE, out);
        /* Battery must stay in the same columns it occupies while recording,
         * otherwise it walks left and right as clips start and stop. */
        EXPECT(0, "        BAT 87% ");
    }

    printf("5. a silent camera says NO CAM wherever the state field was put\n");
    {
        const uint8_t l[OSD_ROWS][OSD_ROW_FIELDS] = {
            { OSD_F_BATTERY, 0, 0, 0 },
            { OSD_F_CARD, OSD_F_STATE, 0, 0 },
            { 0, 0, 0, 0 }, { 0, 0, 0, 0 },
        };
        cam_status_t c = {0};
        c.connected = true;             /* BLE has not given up yet... */
        c.recording_valid = false;      /* ...but nothing is arriving */
        c.battery_pct = 87; c.battery_valid = true;   /* stale, must not show */
        camlink_format_osd(&c, true, true, 0, false, l, CAMLINK_SETUP_NONE, out);
        EXPECT(0, "        ");
        EXPECT(1, "         NO CAM ");
    }

    printf("6. no row is ever empty (Betaflight placeholder trap)\n");
    {
        const uint8_t l[OSD_ROWS][OSD_ROW_FIELDS] = {
            { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 },
        };
        cam_status_t c = live();
        camlink_format_osd(&c, true, true, 0, true, l, CAMLINK_SETUP_NONE, out);
        for (int r = 0; r < 4; r++) {
            CHECK(out[r][0] != '\0', "row %d is empty -- Betaflight would print "
                  "CUSTOM_MSG%d instead of hiding it", r, r + 1);
        }
    }

    printf("7. no row can exceed the MSP text limit, whatever is asked for\n");
    {
        /* Four of the widest fields: far over budget. The config layer rejects
         * this, but the renderer must not run off the end of the buffer if one
         * ever reaches it. */
        const uint8_t l[OSD_ROWS][OSD_ROW_FIELDS] = {
            { OSD_F_BATTERY, OSD_F_CARD, OSD_F_STATE, OSD_F_CLIP },
            { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 },
        };
        cam_status_t c = live();
        camlink_format_osd(&c, true, true, 0, true, l, CAMLINK_SETUP_NONE, out);
        for (int r = 0; r < 4; r++) {
            CHECK((int)strlen(out[r]) <= OSD_ROW_MAX, "row %d is %d chars, limit %d",
                  r, (int)strlen(out[r]), OSD_ROW_MAX);
        }
    }

    printf("8. worst-case values, not typical ones\n");
    {
        const uint8_t l[OSD_ROWS][OSD_ROW_FIELDS] = {
            { OSD_F_STATE, OSD_F_CLIP, 0, 0 },
            { OSD_F_BATTERY, 0, 0, 0 },
            { OSD_F_CARD, 0, 0, 0 },
            { 0, 0, 0, 0 },
        };
        cam_status_t c = live();
        c.temp_over = 3;          c.temp_over_valid = true;   /* "CAM HOT" */
        c.record_time_s = 65535;  /* 1092:15 */
        c.battery_pct = 100;      /* "BAT 100%" */
        c.remain_time_s = 359999; /* "SD 99H59" */
        camlink_format_osd(&c, true, true, 0, false, l, CAMLINK_SETUP_NONE, out);
        EXPECT(0, "CAM HOT 1092:15");
        EXPECT(1, "BAT 100%");
        EXPECT(2, "SD 99H59");
        CHECK((int)strlen(out[0]) == osd_row_width(l[0]),
              "row 0 rendered %d chars but budgets %d -- the picker would be lying",
              (int)strlen(out[0]), osd_row_width(l[0]));
    }

    printf("9. the camera name shows when connected, and goes with the camera\n");
    {
        const uint8_t l[OSD_ROWS][OSD_ROW_FIELDS] = {
            { OSD_F_CAMERA, OSD_F_STATE, 0, 0 },
            { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 },
        };
        cam_status_t c = live();
        strcpy(c.label, "O360");
        camlink_format_osd(&c, true, true, 0, false, l, CAMLINK_SETUP_NONE, out);
        EXPECT(0, "O360 REC    ");

        /* A camera that has gone quiet takes its name with it. Every other
         * field blanks and the state slot already says NO CAM; a name left
         * glowing beside it suggests a camera that is still there. */
        cam_status_t g = {0};
        g.connected = true; g.recording_valid = false;
        strcpy(g.label, "O360");
        camlink_format_osd(&g, true, true, 0, false, l, CAMLINK_SETUP_NONE, out);
        EXPECT(0, "     NO CAM ");

        /* An unrecognised camera still renders something. */
        cam_status_t u = live();
        u.label[0] = '\0';
        camlink_format_osd(&u, true, true, 0, false, l, CAMLINK_SETUP_NONE, out);
        EXPECT(0, "CAM  REC    ");
    }

    printf("10. the setup gesture takes the state slot, and only that slot\n");
    {
        const uint8_t l[OSD_ROWS][OSD_ROW_FIELDS] = {
            { OSD_F_STATE, OSD_F_DOT, 0, 0 },
            { OSD_F_BATTERY, 0, 0, 0 },
            { 0, 0, 0, 0 }, { 0, 0, 0, 0 },
        };
        cam_status_t c = live();

        camlink_format_osd(&c, true, true, 0, false, l, CAMLINK_SETUP_ARMING, out);
        EXPECT(0, "ENTER    ");
        /* Everything else carries on: those readings are still true, and still
         * worth having while deciding whether to commit. */
        EXPECT(1, "BAT 87% ");

        camlink_format_osd(&c, true, true, 0, false, l, CAMLINK_SETUP_READY, out);
        EXPECT(0, "READY    ");

        camlink_format_osd(&c, true, true, 0, false, l, CAMLINK_SETUP_ENTERING, out);
        EXPECT(0, "CONFIG   ");

        /* Abandoning the gesture puts the state slot straight back. */
        camlink_format_osd(&c, true, true, 0, false, l, CAMLINK_SETUP_NONE, out);
        EXPECT(0, "REC      ");
    }

    printf("11. the gesture is visible even with no camera at all\n");
    {
        /* This is the ordinary case: setup is reached on a bench or a quad
         * whose camera is off, where the state slot would otherwise read
         * NO CAM and the pilot would have no feedback for the hold. */
        const uint8_t l[OSD_ROWS][OSD_ROW_FIELDS] = {
            { OSD_F_STATE, 0, 0, 0 }, { 0,0,0,0 }, { 0,0,0,0 }, { 0,0,0,0 },
        };
        cam_status_t c = {0};
        camlink_format_osd(&c, true, true, 0, false, l, CAMLINK_SETUP_NONE, out);
        EXPECT(0, "NO CAM ");
        camlink_format_osd(&c, true, true, 0, false, l, CAMLINK_SETUP_ARMING, out);
        EXPECT(0, "ENTER  ");
        camlink_format_osd(&c, true, true, 0, false, l, CAMLINK_SETUP_READY, out);
        EXPECT(0, "READY  ");
        camlink_format_osd(&c, true, true, 0, false, l, CAMLINK_SETUP_ENTERING, out);
        EXPECT(0, "CONFIG ");
    }

    printf("12. every word fits the state field's budget\n");
    {
        const char *w[] = { "ENTER", "READY", "CONFIG" };
        for (unsigned i = 0; i < 3; i++)
            CHECK(strlen(w[i]) <= osd_fields[OSD_F_STATE].width,
                  "\"%s\" is %zu chars, budget is %u", w[i], strlen(w[i]),
                  osd_fields[OSD_F_STATE].width);
    }

    printf(fails ? "\n%d FAILURE(S)\n" : "\nALL PASS (%d failures)\n", fails);
    return fails ? 1 : 0;
}
