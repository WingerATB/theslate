/* SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 *
 * Host tests for the manual record override.
 *
 * Every case below is a sequence an adversarial review of this design found and
 * named. They are written as the scenario, not as the implementation, so that a
 * future rewrite of the arbiter still has to survive the same events.
 */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "camlink_cfg.h"

static int fails = 0;
#define CHECK(cond, ...) do { if (!(cond)) { \
    printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

/* A healthy bench: camera connected and reporting, no FC expected. */
static camlink_ovr_in_t base(void)
{
    camlink_ovr_in_t in = {0};
    in.authority = true;          /* no FC expected, so nothing is missing */
    in.cam_connected = true;
    in.cam_valid = true;
    return in;
}

/* Run one tick and let the camera catch up with whatever was asked, the way a
 * real camera does a fraction of a second later. */
static bool tick(camlink_ovr_t *o, camlink_ovr_in_t *in, bool press)
{
    in->press = press;
    bool eff = camlink_ovr_update(o, in);
    if (in->cam_connected && in->cam_valid) in->cam_recording = eff;
    return eff;
}

int main(void)
{
    printf("=== manual record override ===\n\n");

    printf("1. a press toggles, and keeps toggling\n");
    {
        camlink_ovr_t o = {0};
        camlink_ovr_in_t in = base();
        CHECK(tick(&o, &in, true)  == true,  "first press must start");
        CHECK(tick(&o, &in, false) == true,  "must stay started");
        CHECK(tick(&o, &in, true)  == false, "second press must stop");
        CHECK(tick(&o, &in, true)  == true,  "third press must start again");
    }

    printf("2. the automatic source takes control back when it moves\n");
    {
        /* This is the whole of "arming or disarming takes control back", and
         * it is what makes the override temporary without a timer. */
        camlink_ovr_t o = {0};
        camlink_ovr_in_t in = base();
        in.auto_want = true;                  /* armed, recording */
        in.cam_recording = true;
        CHECK(tick(&o, &in, true) == false, "a press must stop it mid-flight");
        CHECK(tick(&o, &in, false) == false, "and it must stay stopped");
        in.auto_want = false; in.auto_moved = true;      /* disarm */
        CHECK(tick(&o, &in, false) == false, "disarm: auto says stop");
        in.auto_moved = false; in.auto_want = true; in.auto_moved = true;  /* arm */
        CHECK(tick(&o, &in, false) == true, "arming again must record: auto has control back");
    }

    printf("3. BLOCKER: a press with no camera must not fire a clip later\n");
    {
        /* Bench, camera switched off. The press appears to do nothing, and ten
         * minutes later somebody switches the camera on. Nothing must start. */
        camlink_ovr_t o = {0};
        camlink_ovr_in_t in = base();
        in.cam_connected = false; in.cam_valid = false;
        CHECK(tick(&o, &in, true) == false, "pressing with no camera must not latch RECORD");
        in.cam_connected = true; in.cam_valid = true; in.cam_recording = false;
        for (int i = 0; i < 20; i++) {
            CHECK(tick(&o, &in, false) == false, "camera arriving later must not start a clip");
        }
    }

    printf("4. BLOCKER: a dead FC link must not leave an override recording\n");
    {
        camlink_ovr_t o = {0};
        camlink_ovr_in_t in = base();
        in.auto_want = false;                 /* disarmed */
        CHECK(tick(&o, &in, true) == true, "press records on a healthy bench");
        in.authority = false;                 /* the UART dies */
        CHECK(tick(&o, &in, false) == false, "a dead link must close the clip");
        for (int i = 0; i < 10; i++)
            CHECK(tick(&o, &in, false) == false, "and it must stay closed");
    }

    printf("5. the override may still STOP with no authority -- only never START\n");
    {
        /* The asymmetry, stated as a test: degraded states may always close a
         * clip and may never open one. */
        camlink_ovr_t o = {0};
        camlink_ovr_in_t in = base();
        in.auto_want = true; in.cam_recording = true;   /* recording */
        in.authority = false;                           /* link already dead */
        CHECK(tick(&o, &in, true) == false, "a press must still be able to stop");
        CHECK(tick(&o, &in, true) == false, "and must NOT be able to start again");
    }

    printf("6. BLOCKER: stopping on the camera must not be undone\n");
    {
        camlink_ovr_t o = {0};
        camlink_ovr_in_t in = base();
        CHECK(tick(&o, &in, true) == true, "press starts");
        /* One quiet tick, which is when the module first sees the camera
         * agreeing. Until it has, a disagreement means "the command has not
         * landed yet", not "somebody changed it" -- so the override must NOT
         * release here, and this tick is what earns it the right to later. */
        CHECK(tick(&o, &in, false) == true, "the camera agreeing must not release the override");

        in.press = false;
        in.cam_recording = false;             /* user pressed stop on the camera */
        CHECK(camlink_ovr_update(&o, &in) == false, "the camera's own stop must be adopted");
        for (int i = 0; i < 10; i++) {
            in.press = false;
            CHECK(camlink_ovr_update(&o, &in) == false, "and must not be restarted");
        }
    }

    printf("7. BLOCKER: impatient presses must do one thing, not nothing\n");
    {
        /* Commands are rate limited; inverting on every press made an even
         * number of presses cancel out and send nothing at all. */
        camlink_ovr_t o = {0};
        camlink_ovr_in_t in = base();
        CHECK(tick(&o, &in, true) == true, "press 1 starts");
        in.unsent = true;                     /* the stop has not gone out yet */
        in.press = true; camlink_ovr_update(&o, &in);
        in.press = true; camlink_ovr_update(&o, &in);
        in.press = true; bool eff = camlink_ovr_update(&o, &in);
        CHECK(eff == true, "presses during an undelivered change must be absorbed");
        in.unsent = false;
        CHECK(tick(&o, &in, true) == false, "and the next real press still works");
    }

    printf("8. a transient dropout must not be read as the source moving\n");
    {
        /* auto_moved is the caller's job to debounce; this pins the contract:
         * a tick with auto_moved false must never release the override. */
        camlink_ovr_t o = {0};
        camlink_ovr_in_t in = base();
        in.auto_want = true; in.cam_recording = true;
        CHECK(tick(&o, &in, true) == false, "press stops a running clip");
        for (int i = 0; i < 8; i++)
            CHECK(tick(&o, &in, false) == false, "no auto movement, so the press holds");
    }

    printf("9. an override is never left recording after the camera goes\n");
    {
        camlink_ovr_t o = {0};
        camlink_ovr_in_t in = base();
        CHECK(tick(&o, &in, true) == true, "recording under an override");
        in.cam_connected = false;             /* BLE drops mid-clip */
        CHECK(tick(&o, &in, false) == false, "the override must not hold RECORD with no camera");
        in.cam_connected = true; in.cam_valid = true; in.cam_recording = false;
        CHECK(tick(&o, &in, false) == false, "and reconnecting must not restart it");
    }

    printf("10. auto_want is honoured whenever no override is in force\n");
    {
        camlink_ovr_t o = {0};
        camlink_ovr_in_t in = base();
        in.auto_want = true;
        CHECK(camlink_ovr_update(&o, &in) == true,  "no override: follow the mode");
        in.auto_want = false;
        CHECK(camlink_ovr_update(&o, &in) == false, "no override: follow the mode");
    }

    printf("11. the mode mapping, including an unknown mode\n");
    {
        CHECK(camlink_auto_want(CFG_MODE_ARM,  CFG_SW_LEVEL, true,  false, false) == true,  "ARM follows armed");
        CHECK(camlink_auto_want(CFG_MODE_CUT,  CFG_SW_LEVEL, true,  false, false) == true,  "CUT follows armed");
        CHECK(camlink_auto_want(CFG_MODE_BOTH, CFG_SW_LEVEL, true,  false, false) == true,  "BOTH follows armed");
        CHECK(camlink_auto_want(CFG_MODE_BOTH, CFG_SW_LEVEL, false, true,  true)  == false, "BOTH ignores the switch");
        CHECK(camlink_auto_want(CFG_MODE_SWITCH, CFG_SW_LEVEL,  false, true,  false) == true, "SWITCH/LEVEL follows the window");
        CHECK(camlink_auto_want(CFG_MODE_SWITCH, CFG_SW_BUTTON, false, true,  false) == false, "SWITCH/BUTTON follows its latch");
        CHECK(camlink_auto_want(CFG_MODE_SWITCH, CFG_SW_BUTTON, false, false, true)  == true,  "SWITCH/BUTTON follows its latch");
        /* An unrecognised mode byte must land somewhere that cannot leave a
         * clip running after the pilot has landed. */
        CHECK(camlink_auto_want(99, CFG_SW_LEVEL, true,  false, false) == true,  "unknown mode follows armed");
        CHECK(camlink_auto_want(99, CFG_SW_LEVEL, false, true,  true)  == false, "unknown mode follows armed");
    }

    printf(fails ? "\n%d FAILURE(S)\n" : "\nALL PASS (%d failures)\n", fails);
    return fails ? 1 : 0;
}
