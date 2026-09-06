/* Host tests for the warning ladder and the stop-delay, both of which are the
 * shipped implementations rather than copies:
 *   cc -I../../components/camlink/include -o test_warn test_warn.c && ./test_warn
 */
#include <stdio.h>
#include <string.h>
#include "camlink_warn.h"
#include "camlink_cfg.h"

static int fails = 0;

#define CHECK(cond, msg) do { \
    if (!(cond)) { printf("   FAIL: %s\n", (msg)); fails++; } \
} while (0)

#define WANT_WARN(in, expect, msg) do {                                    \
    camlink_warn_t got = camlink_warn_pick(&(in));                         \
    if (got != (expect)) {                                                 \
        printf("   FAIL: %s -- wanted %d, got %d\n", (msg), (expect), got); \
        fails++;                                                           \
    }                                                                      \
} while (0)

/* A camera that is present, recording what we asked for, with plenty of
 * everything. Nothing here should ever warn. */
static camlink_warn_in_t healthy(void)
{
    camlink_warn_in_t in = {0};
    in.want_record   = true;
    in.rec_valid     = true;
    in.recording     = true;
    in.temp_valid    = true;  in.temp_over = 0;
    in.card_valid    = true;  in.card_s    = 3600;
    in.batt_valid    = true;  in.batt_pct  = 90;
    in.warn_batt_pct = 20;
    in.warn_card_s   = 300;
    return in;
}

int main(void)
{
    printf("=== warning ladder ===\n\n");

    printf("1. a healthy camera warns about nothing\n");
    {
        camlink_warn_in_t in = healthy();
        WANT_WARN(in, CAM_WARN_NONE, "healthy");
    }

    /* THE ONE THIS WAS BUILT FOR. */
    printf("2. asked to record, and it is not -- past the grace period\n");
    {
        camlink_warn_in_t in = healthy();
        in.recording = false;

        in.want_unmet_long = false;
        WANT_WARN(in, CAM_WARN_NONE,
                  "inside the grace period this must stay silent");

        in.want_unmet_long = true;
        WANT_WARN(in, CAM_WARN_NOT_REC, "past the grace period it must warn");
    }

    printf("3. BLOCKER: never warn from a reading the camera did not confirm\n");
    {
        /* A camera that has gone quiet is not a camera that is refusing.
         * Reporting a refusal we cannot actually see would be inventing an
         * alarm, on the one screen the pilot is relying on. */
        camlink_warn_in_t in = healthy();
        in.recording = false; in.want_unmet_long = true;
        in.rec_valid = false;
        WANT_WARN(in, CAM_WARN_NONE, "unconfirmed record state must not warn");

        in = healthy();
        in.batt_valid = false; in.batt_pct = 1;
        WANT_WARN(in, CAM_WARN_NONE, "unconfirmed battery must not warn");

        in = healthy();
        in.card_valid = false; in.card_s = 0;
        WANT_WARN(in, CAM_WARN_NONE, "unconfirmed card must not warn");

        in = healthy();
        in.temp_valid = false; in.temp_over = 3;
        WANT_WARN(in, CAM_WARN_NONE, "unconfirmed temperature must not warn");
    }

    printf("4. no camera is a state, not a warning\n");
    {
        /* The state slot already reads NO CAM, and that is the one piece of
         * information explaining every blank field beside it. */
        camlink_warn_in_t in = healthy();
        in.cam_gone = true;
        in.recording = false; in.want_unmet_long = true;
        in.card_s = 0; in.batt_pct = 1; in.temp_over = 3;
        WANT_WARN(in, CAM_WARN_NONE, "cam_gone must suppress every warning");
    }

    printf("5. a threshold of zero switches that warning off\n");
    {
        camlink_warn_in_t in = healthy();
        in.batt_pct = 1;
        WANT_WARN(in, CAM_WARN_BATT_LOW, "a flat battery warns by default");
        in.warn_batt_pct = 0;
        WANT_WARN(in, CAM_WARN_NONE, "zero threshold means do not warn");

        in = healthy();
        in.card_s = 10;
        WANT_WARN(in, CAM_WARN_SD_LOW, "a nearly full card warns by default");
        in.warn_card_s = 0;
        WANT_WARN(in, CAM_WARN_NONE, "zero threshold means do not warn");
    }

    printf("6. thresholds are inclusive at the boundary\n");
    {
        camlink_warn_in_t in = healthy();
        in.batt_pct = 20;              /* exactly the threshold */
        WANT_WARN(in, CAM_WARN_BATT_LOW, "at the threshold must warn");
        in.batt_pct = 21;
        WANT_WARN(in, CAM_WARN_NONE, "one above must not");
    }

    printf("7. the priority order\n");
    {
        /* Every one true at once: the most actionable must win, and then the
         * next, as each is cleared in turn. */
        camlink_warn_in_t in = healthy();
        in.recording = false; in.want_unmet_long = true;
        in.card_s    = 0;
        in.temp_over = 3;
        in.batt_pct  = 1;

        WANT_WARN(in, CAM_WARN_NO_SD,   "cannot record at all comes first");
        in.card_s = 3600;
        WANT_WARN(in, CAM_WARN_HOT,     "then too hot");
        in.temp_over = 0;
        WANT_WARN(in, CAM_WARN_NOT_REC, "then asked-and-not-recording");
        in.recording = true;
        WANT_WARN(in, CAM_WARN_BATT_LOW, "then the battery");
        in.batt_pct = 90; in.card_s = 60;
        WANT_WARN(in, CAM_WARN_SD_LOW,  "then the card");
    }

    printf("8. temp_over 1 is the camera's own early warning, not ours\n");
    {
        camlink_warn_in_t in = healthy();
        in.temp_over = 1;
        WANT_WARN(in, CAM_WARN_NONE, "1 still records, so it is not a warning");
        in.temp_over = 2;
        WANT_WARN(in, CAM_WARN_HOT, "2 is too hot to record");
    }

    printf("9. every warning has a word, and it fits the state slot\n");
    {
        CHECK(camlink_warn_word(CAM_WARN_NONE) == NULL,
              "NONE must have no word, so the caller falls through to REC/IDLE");
        for (int w = CAM_WARN_NONE + 1; w <= CAM_WARN_NO_SD; w++) {
            const char *s = camlink_warn_word((camlink_warn_t)w);
            if (s == NULL)      { printf("   FAIL: warning %d has no word\n", w); fails++; }
            else if (strlen(s) > 7) {
                printf("   FAIL: \"%s\" is %zu chars, over the 7-char budget\n",
                       s, strlen(s));
                fails++;
            }
        }
    }

    printf("10. the LED escalation line\n");
    {
        /* Everything from NOT_REC up means "no recording is happening"; the two
         * below mean "there will not be one much longer". app_main.c separates
         * them with a single >=, so the order has to hold. */
        CHECK(CAM_WARN_NOT_REC > CAM_WARN_BATT_LOW, "NOT_REC outranks BATT_LOW");
        CHECK(CAM_WARN_NOT_REC > CAM_WARN_SD_LOW,   "NOT_REC outranks SD_LOW");
        CHECK(CAM_WARN_HOT     > CAM_WARN_NOT_REC,  "HOT outranks NOT_REC");
        CHECK(CAM_WARN_NO_SD   > CAM_WARN_HOT,      "NO_SD outranks HOT");
    }

    printf("11b. a faulty card warns even with no time reading to go on\n");
    {
        /* THE CASE THE WARNING EXISTS FOR. A camera with no card has no time
         * remaining to report, so a warning that only triggered on
         * "card_s == 0" would go silent exactly when the card is missing. */
        camlink_warn_in_t in = healthy();
        in.card_fault = true;
        in.card_valid = false;          /* nothing to report, because no card */
        in.card_s = 0;
        WANT_WARN(in, CAM_WARN_NO_SD, "a card fault warns on its own");

        in = healthy();
        in.card_valid = true; in.card_s = 0;
        WANT_WARN(in, CAM_WARN_NO_SD, "and so does a card with no time left");

        in = healthy();
        in.card_fault = false;
        WANT_WARN(in, CAM_WARN_NONE, "a healthy card does not");
    }

    printf("11c. a card fault outranks everything below it\n");
    {
        camlink_warn_in_t in = healthy();
        in.card_fault = true;
        in.batt_pct = 1;                       /* also flat */
        in.recording = false; in.want_unmet_long = true;   /* also not recording */
        WANT_WARN(in, CAM_WARN_NO_SD, "the card comes first");
    }

    printf("\n=== post-roll ===\n\n");

    printf("11. off by default: zero seconds changes nothing\n");
    {
        camlink_stopdelay_t st = {0};
        camlink_stopdelay_in_t in = {
            .auto_stop = true, .want = false, .authority = true,
            .cam_connected = true, .delay_s = 0, .now_ms = 1000,
        };
        CHECK(camlink_stopdelay_update(&st, &in, NULL) == false,
              "with the setting off, an automatic stop stops");
        CHECK(!st.active, "and nothing is latched");
    }

    printf("12. it holds the clip open, then lets go\n");
    {
        camlink_stopdelay_t st = {0};
        uint8_t left = 0;
        camlink_stopdelay_in_t in = {
            .auto_stop = true, .want = false, .authority = true,
            .cam_connected = true, .delay_s = 5, .now_ms = 1000,
        };
        CHECK(camlink_stopdelay_update(&st, &in, &left), "the stop is held");
        CHECK(left == 5, "and the countdown reads 5");

        in.auto_stop = false; in.now_ms = 3000;
        CHECK(camlink_stopdelay_update(&st, &in, &left), "still held at 3 s");
        CHECK(left == 3, "counting down");

        in.now_ms = 5999;
        CHECK(camlink_stopdelay_update(&st, &in, &left), "still held at 4.999 s");
        CHECK(left == 1, "rounded up, never showing 0 while still recording");

        in.now_ms = 6000;
        CHECK(camlink_stopdelay_update(&st, &in, &left) == false, "expired");
        CHECK(left == 0, "and the countdown is gone");
        CHECK(!st.active, "and nothing is latched");
    }

    printf("13. re-arming during the countdown continues the clip\n");
    {
        camlink_stopdelay_t st = {0};
        camlink_stopdelay_in_t in = {
            .auto_stop = true, .want = false, .authority = true,
            .cam_connected = true, .delay_s = 5, .now_ms = 1000,
        };
        camlink_stopdelay_update(&st, &in, NULL);
        CHECK(st.active, "held");

        in.auto_stop = false; in.want = true; in.now_ms = 2000;
        CHECK(camlink_stopdelay_update(&st, &in, NULL), "recording continues");
        CHECK(!st.active, "and the hold is released rather than left to expire");

        /* Which matters: if the hold were still latched, the NEXT disarm would
         * find it active and refuse to start a fresh countdown. */
        in.want = false; in.auto_stop = true; in.now_ms = 3000;
        uint8_t left = 0;
        CHECK(camlink_stopdelay_update(&st, &in, &left), "a later stop is held");
        CHECK(left == 5, "for the full delay, not the remains of the last one");
    }

    printf("14. BLOCKER: a dead FC link must not be outlived by the hold\n");
    {
        /* The standing rule is that a dead UART closes an open clip. A delay
         * that survived one would quietly repeal it. */
        camlink_stopdelay_t st = {0};
        camlink_stopdelay_in_t in = {
            .auto_stop = true, .want = false, .authority = true,
            .cam_connected = true, .delay_s = 30, .now_ms = 1000,
        };
        camlink_stopdelay_update(&st, &in, NULL);
        CHECK(st.active, "held");

        in.auto_stop = false; in.authority = false; in.now_ms = 2000;
        CHECK(camlink_stopdelay_update(&st, &in, NULL) == false,
              "losing authority stops immediately");
        CHECK(!st.active, "and clears the hold");
    }

    printf("15. BLOCKER: a camera that goes away must not be held recording\n");
    {
        camlink_stopdelay_t st = {0};
        camlink_stopdelay_in_t in = {
            .auto_stop = true, .want = false, .authority = true,
            .cam_connected = true, .delay_s = 30, .now_ms = 1000,
        };
        camlink_stopdelay_update(&st, &in, NULL);
        in.auto_stop = false; in.cam_connected = false; in.now_ms = 2000;
        CHECK(camlink_stopdelay_update(&st, &in, NULL) == false,
              "no camera, no hold");
        CHECK(!st.active, "and the hold is cleared");
    }

    printf("16. a deliberate stop is not delayed\n");
    {
        /* Someone pressing the button is looking at the aircraft. A guess
         * about what they would have wanted must not overrule them. */
        camlink_stopdelay_t st = {0};
        camlink_stopdelay_in_t in = {
            .auto_stop = true, .want = false, .overridden = true,
            .authority = true, .cam_connected = true,
            .delay_s = 10, .now_ms = 1000,
        };
        CHECK(camlink_stopdelay_update(&st, &in, NULL) == false,
              "an override stops at once");
        CHECK(!st.active, "and arms nothing");
    }

    printf("17. an override arriving mid-countdown ends it\n");
    {
        camlink_stopdelay_t st = {0};
        camlink_stopdelay_in_t in = {
            .auto_stop = true, .want = false, .authority = true,
            .cam_connected = true, .delay_s = 10, .now_ms = 1000,
        };
        camlink_stopdelay_update(&st, &in, NULL);
        CHECK(st.active, "held");

        in.auto_stop = false; in.overridden = true; in.now_ms = 2000;
        CHECK(camlink_stopdelay_update(&st, &in, NULL) == false, "stops now");
        CHECK(!st.active, "and the hold is gone");
    }

    printf("18. the countdown survives the millisecond counter wrapping\n");
    {
        /* esp_timer milliseconds are a uint32: about 49 days. A module left
         * powered would otherwise hold a clip open for the next seven weeks. */
        camlink_stopdelay_t st = {0};
        uint8_t left = 0;
        camlink_stopdelay_in_t in = {
            .auto_stop = true, .want = false, .authority = true,
            .cam_connected = true, .delay_s = 5,
            .now_ms = 0xFFFFF000u,       /* about 4 s before the wrap */
        };
        CHECK(camlink_stopdelay_update(&st, &in, &left), "held across the wrap");

        in.auto_stop = false;
        in.now_ms = 0xFFFFF000u + 2000u;   /* wrapped */
        CHECK(camlink_stopdelay_update(&st, &in, &left), "still held");

        in.now_ms = 0xFFFFF000u + 5000u;   /* wrapped, and expired */
        CHECK(camlink_stopdelay_update(&st, &in, &left) == false,
              "and expires on time rather than in 49 days");
    }

    printf("\n%s (%d failure%s)\n", fails ? "FAILURES" : "ALL PASS",
           fails, fails == 1 ? "" : "s");
    return fails ? 1 : 0;
}
