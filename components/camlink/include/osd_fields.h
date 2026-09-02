/* SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 *
 * The OSD field catalogue: what can be put on screen, and how much room each
 * one needs in the worst case.
 *
 * WIDTH IS THE WHOLE POINT of this table. Betaflight draws a custom message as
 * a plain string at a fixed position and never clears what was there before, so
 * every row has to be padded to the longest thing it can ever produce -- see
 * the rendering notes in osd_format.c. That makes a row's capacity a hard
 * budget rather than a guideline, and it is why the settings page can tell the
 * user whether a layout fits before they fly with it.
 *
 * The widths below are worst cases, not typical ones. "12:34" is what a clip
 * counter usually looks like; "1092:15" is what it looks like when a uint16 of
 * seconds runs out, and that is the number the row has to be built for.
 */
#ifndef OSD_FIELDS_H
#define OSD_FIELDS_H

#include <stdint.h>

typedef enum {
    OSD_F_NONE = 0,     /* empty slot in a row                               */
    OSD_F_STATE,        /* REC / IDLE / NO CAM / CAM HOT                     */
    OSD_F_CLIP,         /* 12:34   -- elapsed clip time                      */
    OSD_F_BATTERY,      /* BAT 87%                                           */
    OSD_F_BATT_PCT,     /* 87%     -- the same, without the label            */
    OSD_F_CARD,         /* SD 1H49 -- recording time left on the card        */
    OSD_F_CARD_SHORT,   /* 1H49    -- the same, without the label            */
    OSD_F_DOT,          /* the liveness dot                                  */
    OSD_F_CAMERA,       /* NANO / O360 / A5 -- which camera is bound         */
    OSD_F__COUNT
} osd_field_t;

typedef struct {
    const char *name;      /* what the settings page calls it                */
    const char *example;   /* a representative rendering, for the picker     */
    uint8_t     width;     /* worst-case characters, excluding separators    */
} osd_field_info_t;

extern const osd_field_info_t osd_fields[OSD_F__COUNT];

/* A row holds up to this many fields, and the module has this many rows --
 * Betaflight offers exactly four custom messages. */
#define OSD_ROWS         4
#define OSD_ROW_FIELDS   4

/* The hard ceiling on a rendered row, from MSP_TEXT_MAX_LEN. */
#define OSD_ROW_MAX     16

/* Worst-case rendered width of one row: the sum of its fields' widths plus one
 * space between each pair. Returns 0 for an empty row. This is the number the
 * settings page budgets against, and the number the row is padded to. */
int osd_row_width(const uint8_t row[OSD_ROW_FIELDS]);

/* Is this row within OSD_ROW_MAX? A row that is not would be truncated on
 * screen, silently losing whichever field ended up last. */
int osd_row_fits(const uint8_t row[OSD_ROW_FIELDS]);

#endif /* OSD_FIELDS_H */
