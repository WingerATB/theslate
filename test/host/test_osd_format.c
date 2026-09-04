/* SPDX-License-Identifier: PolyForm-Strict-1.0.0 */
/* Host tests for the real camlink_format_osd() implementation.
 *   cc -I../../components/camlink/include -o test_osd_format \
 *      test_osd_format.c ../../components/camlink/osd_format.c && ./test_osd_format
 */
#include <stdio.h>
#include <string.h>
#include "camlink.h"
#include "osd_fields.h"

static int fails = 0;
/* Slots are space-padded to a fixed width: an empty string would make
 * Betaflight render the literal "CUSTOM_MSG1" placeholder, and a short string
 * would leave the tail of the previous one on screen. Compare trimmed. */
static char *trim(char *s)
{
    size_t n = strlen(s);
    while (n > 0 && s[n-1] == ' ') s[--n] = '\0';
    return s;
}
#define EXPECT(slot, want) do { \
    if (strcmp(trim(out[slot]), want) != 0) { \
        printf("  FAIL: slot %d = \"%s\", expected \"%s\"\n", slot, out[slot], want); \
        fails++; } } while (0)

int main(void)
{
    char out[4][17];
    printf("OSD formatter tests\n\n");

    /* 1. Disconnected camera: state says so, everything else blank. */
    printf("1. camera disconnected\n");
    {
        cam_status_t c = {0};
        c.connected = false;
        camlink_format_osd(&c, true, false, 0, false, NULL, CAMLINK_SETUP_NONE, NULL, out);
        EXPECT(0, "NO CAM");
        EXPECT(1, ""); EXPECT(2, ""); EXPECT(3, "");
    }

    /* 1b. Disconnected while every field still claims to be valid.
     *
     * The flags cannot currently reach this combination -- they are all gated
     * on the BLE link -- which is exactly why it is worth pinning. It is the
     * state a future refactor would reintroduce by accident, and it is the
     * worst one: a confident "BAT 100%  SD 44:59" sitting beside "NO CAM",
     * describing a camera that is not there. */
    printf("1b. disconnected outranks every stale field\n");
    {
        cam_status_t c = {0};
        c.connected         = false;
        c.recording_valid   = true;  c.recording       = true;
        c.record_time_valid = true;  c.record_time_s   = 95;
        c.battery_valid     = true;  c.battery_pct     = 100;
        c.remain_time_valid = true;  c.remain_time_s   = 2699;
        c.temp_over_valid   = true;  c.temp_over       = 3;
        camlink_format_osd(&c, true, true, 95, false, NULL, CAMLINK_SETUP_NONE, NULL, out);
        EXPECT(0, "NO CAM");
        EXPECT(1, ""); EXPECT(2, ""); EXPECT(3, "");
    }

    /* 2. A link that is up but silent is NO CAM, not CAM ?.
     *
     * This is the regression that raising the BLE supervision timeout to 6 s
     * introduced: the status goes stale at 3 s, so a camera switched off left
     * three seconds where the link still existed and nothing was arriving. The
     * display must not depend on which clock expires first. */
    printf("2. connected but silent -- still NO CAM\n");
    {
        cam_status_t c = {0};
        c.connected = true;              /* BLE has not given up yet */
        c.recording_valid = false;       /* ...but nothing has arrived */
        camlink_format_osd(&c, true, true, 0, false, NULL, CAMLINK_SETUP_NONE, NULL, out);
        EXPECT(0, "NO CAM");
        EXPECT(1, ""); EXPECT(2, ""); EXPECT(3, "");
    }

    /* 2b. A stale battery or card reading must not survive that rule either --
     * the same single early return has to blank every slot. */
    printf("2b. silent link blanks the other slots too\n");
    {
        cam_status_t c = {0};
        c.connected = true;
        c.recording_valid = false;
        c.battery_pct = 87;   c.battery_valid = true;
        c.remain_time_s = 900; c.remain_time_valid = true;
        camlink_format_osd(&c, true, true, 0, false, NULL, CAMLINK_SETUP_NONE, NULL, out);
        EXPECT(0, "NO CAM");
        EXPECT(1, ""); EXPECT(2, ""); EXPECT(3, "");
    }

    /* 3. Recording with a full 38-byte frame: all four slots populated. */
    printf("3. recording, all fields valid\n");
    {
        cam_status_t c = {0};
        c.connected = true;
        c.recording = true;        c.recording_valid = true;
        c.record_time_s = 95;      c.record_time_valid = true;   /* 01:35 */
        c.battery_pct = 87;        c.battery_valid = true;
        c.remain_time_s = 754;     c.remain_time_valid = true;   /* 12:34 */
        c.temp_over = 0;           c.temp_over_valid = true;
        camlink_format_osd(&c, true, true, 0, false, NULL, CAMLINK_SETUP_NONE, NULL, out);
        EXPECT(0, "REC");
        EXPECT(1, "01:35");
        EXPECT(2, "BAT 87%");
        EXPECT(3, "SD 12:34");
    }

    /* 4. Short frame (37 bytes): battery must be blank, not zero. This is the
     *    case that matters most -- "BAT 0%" would be a lie. */
    printf("4. truncated frame, battery unconfirmed\n");
    {
        cam_status_t c = {0};
        c.connected = true;
        c.recording = true;    c.recording_valid = true;
        c.record_time_s = 5;   c.record_time_valid = true;
        c.battery_valid = false;                 /* 37 bytes received */
        c.remain_time_s = 3600; c.remain_time_valid = true;
        camlink_format_osd(&c, true, true, 0, false, NULL, CAMLINK_SETUP_NONE, NULL, out);
        EXPECT(0, "REC");
        EXPECT(1, "00:05");
        EXPECT(2, "");                            /* blank, never "BAT 0%" */
        EXPECT(3, "SD 1H00");
    }

    /* 5. Idle: timer blank because there is no clip running. */
    printf("5. idle\n");
    {
        cam_status_t c = {0};
        c.connected = true;
        c.recording = false;   c.recording_valid = true;
        c.record_time_s = 42;  c.record_time_valid = true;
        c.battery_pct = 100;   c.battery_valid = true;
        camlink_format_osd(&c, true, false, 0, false, NULL, CAMLINK_SETUP_NONE, NULL, out);
        EXPECT(0, "IDLE");
        EXPECT(1, "");
        EXPECT(2, "BAT 100%");
    }

    /* 6. A warning takes the state slot on its ON phase and gives it back on
     *    the OFF phase, so the pilot still sees what the camera is doing.
     *
     *    Overheating used to be tested inside the formatter. It is one entry in
     *    the ladder now (test_warn.c owns the ladder itself); what is checked
     *    here is only that a warning renders, and that it blinks rather than
     *    parking on screen. */
    printf("6. a warning blinks in the state slot\n");
    {
        cam_status_t c = {0};
        c.connected = true;
        c.recording = false; c.recording_valid = true;

        camlink_osd_extra_t on  = { .warn = CAM_WARN_HOT, .warn_on = true  };
        camlink_osd_extra_t off = { .warn = CAM_WARN_HOT, .warn_on = false };

        camlink_format_osd(&c, true, false, 0, false, NULL, CAMLINK_SETUP_NONE, &on, out);
        EXPECT(0, "CAM HOT");
        camlink_format_osd(&c, true, false, 0, false, NULL, CAMLINK_SETUP_NONE, &off, out);
        EXPECT(0, "IDLE");

        /* Every warning must actually reach the screen, and must not disturb
         * the row's width -- the default layout puts the liveness dot beside
         * the state, so the row is padded to STATE(7) + space + DOT(1) = 9
         * whatever the word is. A word over budget would push the dot right.
         * The words' own lengths are checked in test_warn.c. */
        for (int w = CAM_WARN_NONE + 1; w <= CAM_WARN_NO_SD; w++) {
            camlink_osd_extra_t e = { .warn = (camlink_warn_t)w, .warn_on = true };
            camlink_format_osd(&c, true, false, 0, false, NULL, CAMLINK_SETUP_NONE, &e, out);
            const char *word = camlink_warn_word((camlink_warn_t)w);
            if (strncmp(out[0], word, strlen(word)) != 0) {
                printf("   FAIL: warning %d rendered \"%s\", wanted \"%s\"\n",
                       w, out[0], word);
                fails++;
            }
            if (strlen(out[0]) != 9) {
                printf("   FAIL: warning %d moved the row width to %zu: \"%s\"\n",
                       w, strlen(out[0]), out[0]);
                fails++;
            }
        }
    }

    /* 6b. The setup gesture still outranks a warning. It is about to change
     *     what the module IS, and the pilot has to see it land. */
    printf("6b. setup outranks a warning\n");
    {
        cam_status_t c = {0};
        c.connected = true;
        c.recording = false; c.recording_valid = true;
        camlink_osd_extra_t e = { .warn = CAM_WARN_NO_SD, .warn_on = true };
        camlink_format_osd(&c, true, false, 0, false, NULL, CAMLINK_SETUP_READY, &e, out);
        EXPECT(0, "READY");
    }

    /* 6c. The stop-delay countdown. Still says REC, because it still is --
     *     a camera recording after landing with nothing explaining it looks
     *     exactly like a stop command that failed. */
    printf("6c. stop-delay countdown\n");
    {
        cam_status_t c = {0};
        c.connected = true;
        c.recording = true;  c.recording_valid = true;
        camlink_osd_extra_t e = { .stop_delay_s = 5 };
        camlink_format_osd(&c, true, false, 0, false, NULL, CAMLINK_SETUP_NONE, &e, out);
        EXPECT(0, "REC 5");

        /* A warning still outranks it: not recording at all matters more than
         * how long a hold has left to run. */
        e.warn = CAM_WARN_NO_SD; e.warn_on = true;
        camlink_format_osd(&c, true, false, 0, false, NULL, CAMLINK_SETUP_NONE, &e, out);
        EXPECT(0, "NO SD");
    }

    /* 7. Every slot must fit Betaflight's 16-char MAX_NAME_LENGTH. */
    printf("7. length bounds\n");
    {
        cam_status_t c = {0};
        c.connected = true;
        c.recording = true;      c.recording_valid = true;
        c.record_time_s = 65535; c.record_time_valid = true;
        c.battery_pct = 100;     c.battery_valid = true;
        c.remain_time_s = 359999; c.remain_time_valid = true;
        camlink_format_osd(&c, true, true, 0, false, NULL, CAMLINK_SETUP_NONE, NULL, out);
        for (int i = 0; i < 4; i++) {
            if (strlen(out[i]) > 16) {
                printf("  FAIL: slot %d is %zu chars (\"%s\"), max 16\n",
                       i, strlen(out[i]), out[i]);
                fails++;
            }
        }
        printf("     worst case: [%s][%s][%s][%s]\n", out[0], out[1], out[2], out[3]);
    }

    /* 8. No slot may ever be empty -- Betaflight would print "CUSTOM_MSGn". */
    /* 9. The liveness dot: present when asked, gone when not, and never at the
     *    cost of the state word beside it. A dot that ate a character of
     *    "NO CAM" would trade one clear signal for another.
     *
     *    Its column is now decided by the layout rather than hard-coded: in the
     *    default layout it follows the state field, which is padded to its own
     *    worst case ("CAM HOT"), so the dot sits at index 8. Asserting the
     *    index the layout implies -- rather than a literal 7 -- is what keeps
     *    this test meaningful if the default layout is ever rearranged. */
    printf("9. liveness dot\n");
    {
        const int dot = (int)osd_fields[OSD_F_STATE].width + 1;   /* + separator */

        cam_status_t c = {0};
        c.connected = true; c.recording_valid = true; c.recording = true;

        camlink_format_osd(&c, true, true, 0, true, NULL, CAMLINK_SETUP_NONE, NULL, out);
        if (out[0][dot] != '.') { printf("   FAIL: dot missing\n"); fails++; }
        if (strncmp(out[0], "REC", 3) != 0) { printf("   FAIL: state clobbered\n"); fails++; }

        camlink_format_osd(&c, true, true, 0, false, NULL, CAMLINK_SETUP_NONE, NULL, out);
        if (out[0][dot] != ' ') { printf("   FAIL: dot stuck on\n"); fails++; }
        if (strncmp(out[0], "REC", 3) != 0) { printf("   FAIL: state clobbered\n"); fails++; }

        /* Longest state string must still leave the dot its column. */
        cam_status_t d = {0};
        d.connected = false;
        camlink_format_osd(&d, true, false, 0, true, NULL, CAMLINK_SETUP_NONE, NULL, out);
        if (out[0][dot] != '.') { printf("   FAIL: no dot on NO CAM\n"); fails++; }
        if (strncmp(out[0], "NO CAM", 6) != 0) { printf("   FAIL: NO CAM clobbered\n"); fails++; }
        if ((int)strlen(out[0]) != dot + 1) { printf("   FAIL: slot 0 width\n"); fails++; }
    }

    printf("8. no empty slots (Betaflight placeholder trap)\n");
    {
        cam_status_t c = {0};
        c.connected = false;               /* worst case: nothing known */
        camlink_format_osd(&c, false, false, 0, false, NULL, CAMLINK_SETUP_NONE, NULL, out);
        for (int i = 0; i < 4; i++) {
            if (out[i][0] == '\0') {
                printf("  FAIL: slot %d is empty -- FC would render CUSTOM_MSG%d\n",
                       i, i + 1);
                fails++;
            }
        }
    }

    printf("\n%s (%d failure%s)\n", fails ? "FAILED" : "ALL PASS",
           fails, fails == 1 ? "" : "s");
    return fails ? 1 : 0;
}
