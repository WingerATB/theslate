/* SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 *
 * Host-side tests for BUTTON-mode press detection.
 *
 * The fault this exists for: a momentary button leaves the channel wherever the
 * press put it. It never returns, so a second press produces another change but
 * never another rising edge -- and level logic, which is all there was, saw
 * nothing. The user's description was "you have to press it twice".
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "camlink_cfg.h"

static int fails = 0;
#define CHECK(cond, ...) do { if (!(cond)) { \
    printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

/* Feed a run of readings, return how many presses came out. */
static int feed(camlink_press_t *st, const uint16_t *v, int n)
{
    int p = 0;
    for (int i = 0; i < n; i++) if (camlink_press_update(st, v[i])) p++;
    return p;
}

int main(void)
{
    printf("=== BUTTON press detection ===\n\n");

    printf("1. the first reading is a position, not a press\n");
    {
        camlink_press_t st = {0};
        CHECK(!camlink_press_update(&st, 1000), "adopting the first value fired a press");
        CHECK(!camlink_press_update(&st, 1000), "a still channel fired a press");
    }

    printf("2. a momentary button: each press is one press\n");
    {
        /* What the fault looked like. The value steps and STAYS -- there is
         * never a return, so there is never a second edge. */
        camlink_press_t st = {0};
        uint16_t run[] = { 1000,1000, 2000,2000,2000, 1000,1000, 2000,2000 };
        CHECK(feed(&st, run, 9) == 3, "expected 3 presses, got %d", feed(&st, run, 9));
    }

    printf("3. a channel that steps up and never comes back still presses again\n");
    {
        camlink_press_t st = {0};
        uint16_t run[] = { 1000, 1200, 1400, 1600, 1800 };   /* one way only */
        CHECK(feed(&st, run, 5) == 4, "a one-way stepper must still register each step");
    }

    printf("4. receiver jitter is not a press\n");
    {
        camlink_press_t st = {0};
        /* CRSF at rest wanders a few microseconds. None of this is a press. */
        uint16_t run[] = { 1500,1502,1499,1503,1498,1501,1500,1497,1504,1500 };
        CHECK(feed(&st, run, 10) == 0, "jitter registered as a press");
    }

    printf("5. jitter does not accumulate into a press\n");
    {
        /* Each step is under the threshold but they all go one way. The
         * detector compares against the last SETTLED value, not the previous
         * sample, so a slow drift eventually crosses -- and must, or a slow
         * potentiometer could never trigger. What must not happen is firing
         * repeatedly while still inside the deadband. */
        camlink_press_t st = {0};
        uint16_t run[] = { 1500, 1510, 1520, 1530, 1540 };   /* +40 total */
        CHECK(feed(&st, run, 5) == 0, "drift inside the deadband fired");
        CHECK(camlink_press_update(&st, 1551), "crossing the threshold must fire");
        CHECK(!camlink_press_update(&st, 1551), "the same value fired twice");
    }

    printf("6. an RC dropout cannot manufacture a press\n");
    {
        camlink_press_t st = {0};
        camlink_press_update(&st, 2000);
        CHECK(!camlink_press_update(&st, 0), "a dropout registered as a press");
        CHECK(!camlink_press_update(&st, 0), "a dropout registered as a press");
        CHECK(!camlink_press_update(&st, 2000), "recovering to the SAME value fired");
        CHECK(camlink_press_update(&st, 1000), "a real move after a dropout must fire");
    }

    printf("7. the threshold sits between jitter and a real detent\n");
    {
        CHECK(CAMLINK_PRESS_DELTA_US > 10, "threshold is inside receiver jitter");
        CHECK(CAMLINK_PRESS_DELTA_US < 400, "threshold is wider than a 3-position detent");
        camlink_press_t a = {0};
        camlink_press_update(&a, 1500);
        CHECK(!camlink_press_update(&a, 1500 + CAMLINK_PRESS_DELTA_US - 1), "fired below threshold");
        camlink_press_t b = {0};
        camlink_press_update(&b, 1500);
        CHECK(camlink_press_update(&b, 1500 + CAMLINK_PRESS_DELTA_US), "did not fire at threshold");
    }

    printf("8. direction does not matter -- a button pressed is a button pressed\n");
    {
        camlink_press_t up = {0}, dn = {0};
        camlink_press_update(&up, 1000);
        camlink_press_update(&dn, 2000);
        CHECK(camlink_press_update(&up, 2000), "upward move missed");
        CHECK(camlink_press_update(&dn, 1000), "downward move missed");
    }

    printf(fails ? "\n%d FAILURE(S)\n" : "\nALL PASS (%d failures)\n", fails);
    return fails ? 1 : 0;
}
