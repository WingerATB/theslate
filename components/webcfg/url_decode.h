/* SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0 */
#ifndef URL_DECODE_H
#define URL_DECODE_H

/* Percent-decode a form value in place. Handles %XX and '+'; leaves a
 * malformed escape as its literal characters. Never lengthens the string. */
void url_decode(char *s);

#endif
