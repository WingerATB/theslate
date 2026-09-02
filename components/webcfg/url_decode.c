/* SPDX-License-Identifier: PolyForm-Strict-1.0.0
 *
 * Percent-decoding for form bodies, kept free of ESP-IDF so the host tests
 * exercise this exact code rather than a copy of it.
 *
 * esp_http_server's httpd_query_key_value() does not decode, and the settings
 * page posts with URLSearchParams, which encodes everything outside
 * [A-Za-z0-9*-._]. That has now cost two bugs: a MAC address whose colons
 * arrived as %3A and rejected every camera selection, and an OSD layout whose
 * commas arrived as %2C and answered "bad field id" to a perfectly good layout.
 * The first was worked around by picking a wire format with nothing to encode,
 * which fixed that field and left the hole open for the next one.
 */
#include <stddef.h>

#include "url_decode.h"

void url_decode(char *s)
{
    static const char hex[] = "0123456789ABCDEF";
    if (s == NULL) return;

    char *w = s;
    for (char *r = s; *r; r++) {
        if (*r == '+') {
            /* application/x-www-form-urlencoded spells a space this way. */
            *w++ = ' ';
        } else if (*r == '%' && r[1] && r[2]) {
            const char *hi = NULL, *lo = NULL;
            for (const char *h = hex; *h; h++) {
                char c1 = r[1] >= 'a' && r[1] <= 'z' ? (char)(r[1] - 32) : r[1];
                char c2 = r[2] >= 'a' && r[2] <= 'z' ? (char)(r[2] - 32) : r[2];
                if (*h == c1) hi = h;
                if (*h == c2) lo = h;
            }
            if (hi && lo) {
                *w++ = (char)(((hi - hex) << 4) | (lo - hex));
                r += 2;
            } else {
                /* Malformed escape: pass the characters through rather than
                 * guessing. A field that contains a stray '%' is the caller's
                 * problem to reject, not this function's to invent a value for. */
                *w++ = *r;
            }
        } else {
            *w++ = *r;
        }
    }
    *w = '\0';   /* never lengthens, so decoding in place is always safe */
}
