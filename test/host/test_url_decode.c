/* SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0 */
/* Host-side tests for form-value percent-decoding.
 *
 * This function exists because the same trap caught two fields in a row: a MAC
 * address whose colons arrived as %3A, and an OSD layout whose commas arrived
 * as %2C and answered "bad field id" to a layout that was perfectly valid. The
 * cases below are the actual strings the settings page puts on the wire.
 */
#include <stdio.h>
#include <string.h>

#include "url_decode.h"

static int fails = 0;
#define CHECK(in, want) do { \
    char buf[128]; strcpy(buf, in); url_decode(buf); \
    if (strcmp(buf, want) != 0) { \
        printf("  FAIL: [%s] -> [%s], expected [%s]\n", in, buf, want); fails++; } \
    } while (0)

int main(void)
{
    printf("=== url_decode ===\n\n");

    printf("1. what the page actually posts\n");
    /* URLSearchParams({osd: "1,7,0,0,..."}) -- the bug, verbatim. */
    CHECK("1%2C7%2C0%2C0%2C2%2C0%2C0%2C0", "1,7,0,0,2,0,0,0");
    /* The earlier one, kept so the fix covers both. */
    CHECK("4C%3A43%3AF6%3A66%3A67%3A3F", "4C:43:F6:66:67:3F");

    printf("2. plain values pass through untouched\n");
    CHECK("1750", "1750");
    CHECK("", "");
    CHECK("SLATE-8ED9", "SLATE-8ED9");

    printf("3. lower-case hex, and '+' as a space\n");
    CHECK("%2c%3a", ",:");
    CHECK("a+b", "a b");
    CHECK("%41%42", "AB");

    printf("4. a malformed escape is passed through, not guessed at\n");
    CHECK("100%", "100%");
    CHECK("%zz", "%zz");
    CHECK("%2", "%2");
    CHECK("50%%20", "50% ");

    printf("5. decoding never lengthens the string (in-place is safe)\n");
    {
        const char *cases[] = { "%2C%2C%2C", "a%20b%20c", "+++", "%41", NULL };
        for (int i = 0; cases[i]; i++) {
            char buf[64]; strcpy(buf, cases[i]);
            size_t before = strlen(buf);
            url_decode(buf);
            if (strlen(buf) > before) {
                printf("  FAIL: [%s] grew to %zu chars\n", cases[i], strlen(buf));
                fails++;
            }
        }
    }

    printf(fails ? "\n%d FAILURE(S)\n" : "\nALL PASS (%d failures)\n", fails);
    return fails ? 1 : 0;
}
