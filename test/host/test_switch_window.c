/* SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0 */
/* Host-side tests for the record-switch window.
 *
 * These exist because of a shipped bug: the window could not be set above
 * 2000 us, and every mainstream receiver puts a switch at +100% on 2012 us.
 * The setting looked correct, the marker sat at the end of the track, and the
 * camera never rolled. The numbers below are the receiver's, not ours -- if a
 * change to the scale breaks them, it has broken real hardware.
 *
 *   cc -I../../components/camlink/include -o test_switch_window \
 *      test_switch_window.c && ./test_switch_window
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "camlink_cfg.h"

static int fails = 0;
#define CHECK(cond, ...) do { if (!(cond)) { \
    printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

/* Betaflight's own conversions, from rx/crsf.c and rx/sbus.c. Both protocols
 * carry the same 172..1811 endpoints for -100%..+100%. */
static uint16_t crsf_us(uint16_t raw) { return (uint16_t)((raw * 1024 / 1639) + 881); }
static uint16_t sbus_us(uint16_t raw) { return (uint16_t)((raw * 5 / 8) + 880); }

int main(void)
{
    printf("=== switch window ===\n");

    /* The receiver's actual output, which is what MSP_RC hands back. */
    printf("  CRSF  low=%u mid=%u high=%u\n", crsf_us(172), crsf_us(992), crsf_us(1811));
    printf("  SBUS  low=%u mid=%u high=%u\n", sbus_us(172), sbus_us(992), sbus_us(1811));

    CHECK(crsf_us(1811) > 2000, "CRSF high is %u, expected above 2000", crsf_us(1811));
    CHECK(sbus_us(1811) > 2000, "SBUS high is %u, expected above 2000", sbus_us(1811));
    CHECK(crsf_us(1811) <= CAMLINK_RC_US_MAX,
          "CRSF high %u is off the top of the scale", crsf_us(1811));
    CHECK(crsf_us(172) >= CAMLINK_RC_US_MIN,
          "CRSF low %u is off the bottom of the scale", crsf_us(172));

    /* The default "records above 1750" window, with the top of the slider as
     * its upper limit. This is the case that was broken. */
    camlink_cfg_t up = { .range_min = 1750, .range_max = CAMLINK_RC_US_MAX };
    CHECK(camlink_cfg_in_window(crsf_us(1811), &up), "CRSF high must trigger");
    CHECK(camlink_cfg_in_window(sbus_us(1811), &up), "SBUS high must trigger");
    CHECK(camlink_cfg_in_window(2000, &up), "a tidy 2000 must still trigger");
    CHECK(!camlink_cfg_in_window(crsf_us(992), &up), "centre must not trigger");
    CHECK(!camlink_cfg_in_window(crsf_us(172), &up), "low must not trigger");

    /* An end-stop limit means "and beyond": a radio with extended endpoints
     * overshoots the scale, and the switch is still where the user put it. */
    CHECK(camlink_cfg_in_window(2159, &up), "an overshooting endpoint must trigger");
    camlink_cfg_t down = { .range_min = CAMLINK_RC_US_MIN, .range_max = 1250 };
    CHECK(camlink_cfg_in_window(880, &down), "an undershooting endpoint must trigger");
    CHECK(!camlink_cfg_in_window(1500, &down), "centre must not trigger a low window");

    /* A deliberately bounded window still excludes both ends -- the open-ended
     * rule must apply only to limits parked at the end of their travel. */
    camlink_cfg_t mid = { .range_min = 1300, .range_max = 1700 };
    CHECK(camlink_cfg_in_window(1500, &mid), "centre must trigger a mid window");
    CHECK(!camlink_cfg_in_window(crsf_us(1811), &mid), "high must not trigger a mid window");
    CHECK(!camlink_cfg_in_window(crsf_us(172), &mid), "low must not trigger a mid window");

    /* Three-position switch, middle detent, matched to the mid window. */
    CHECK(camlink_cfg_in_window(crsf_us(992), &mid), "3-pos centre must trigger");

    printf(fails ? "\n%d FAILURE(S)\n" : "\nALL PASS (%d failures)\n", fails);
    return fails ? 1 : 0;
}
