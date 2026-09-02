/* SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 *
 * Host tests for the "may I enter setup?" guard.
 *
 * Entering setup reboots the module, drops the camera link and raises a Wi-Fi
 * access point. The interesting cases are not "armed" and "disarmed" but the
 * ones in between, where the answer is not yet knowable.
 */
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include "camlink_cfg.h"

static int fails = 0;
#define CHECK(cond, ...) do { if (!(cond)) { \
    printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

int main(void)
{
    printf("=== setup guard ===\n\n");

    printf("1. armed refuses, disarmed allows\n");
    CHECK(!camlink_may_enter_setup(true,  true,  true),  "armed must refuse");
    CHECK( camlink_may_enter_setup(true,  true,  false), "disarmed must allow");

    printf("2. a bench with no flight controller must still be settable up\n");
    /* Otherwise a module that has never met an FC could never be configured,
     * which is the state every module ships in. */
    CHECK(camlink_may_enter_setup(false, false, false), "no link must allow");
    CHECK(camlink_may_enter_setup(false, false, true),
          "no link must allow even if a stale armed flag says otherwise");

    printf("3. a live link that has not resolved BOXARM must refuse\n");
    /* Until the FC has said which bit of flightModeFlags is BOXARM, `armed` is
     * a default rather than a reading -- and a default must never be allowed to
     * read as "disarmed, go ahead". This is the window just after boot with an
     * FC attached, which is exactly when someone is fiddling with the module. */
    CHECK(!camlink_may_enter_setup(true, false, false),
          "link up but BOXARM unresolved must refuse");
    CHECK(!camlink_may_enter_setup(true, false, true),
          "link up but BOXARM unresolved must refuse");

    printf("4. the full truth table\n");
    {
        const struct { bool link, known, armed, expect; const char *why; } t[] = {
            { false, false, false, true,  "bench, nothing known"          },
            { false, true,  true,  true,  "no link: armed is meaningless" },
            { true,  false, false, false, "link up, BOXARM not resolved"  },
            { true,  true,  false, true,  "link up, disarmed"             },
            { true,  true,  true,  false, "link up, armed"                },
        };
        for (unsigned i = 0; i < sizeof(t)/sizeof(t[0]); i++) {
            bool got = camlink_may_enter_setup(t[i].link, t[i].known, t[i].armed);
            CHECK(got == t[i].expect, "%s: got %d expected %d",
                  t[i].why, got, t[i].expect);
        }
    }

    printf("5. the only way to get a yes with a live link is a resolved disarm\n");
    {
        int yes = 0;
        for (int k = 0; k < 2; k++)
            for (int a = 0; a < 2; a++)
                if (camlink_may_enter_setup(true, k, a)) yes++;
        CHECK(yes == 1, "expected exactly one permitted combination, got %d", yes);
    }

    printf(fails ? "\n%d FAILURE(S)\n" : "\nALL PASS (%d failures)\n", fails);
    return fails ? 1 : 0;
}
