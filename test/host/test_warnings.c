/* SPDX-License-Identifier: PolyForm-Strict-1.0.0
 *
 * Warning ladder: which warning wins, when it blinks, and when it stays quiet.
 * Compiled against the real osd_format.c, so it exercises the shipping logic. */
#include <stdio.h>
#include <string.h>
#include "camlink.h"
#include "osd_fields.h"

static int fails = 0;
static char *trim(char *s){ size_t n=strlen(s); while(n&&s[n-1]==' ')s[--n]=0; return s; }
#define EXPECT(slot, want) do { \
    if (strcmp(trim(out[slot]), want) != 0) { \
        printf("  FAIL: slot %d = \"%s\", expected \"%s\"\n", slot, out[slot], want); \
        fails++; } } while (0)

/* WARN in row 0, STATE in row 1. */
static const uint8_t L[OSD_ROWS][OSD_ROW_FIELDS] = {
    { OSD_F_WARN, 0, 0, 0 },
    { OSD_F_STATE, 0, 0, 0 },
    { 0, 0, 0, 0 }, { 0, 0, 0, 0 },
};

static cam_status_t rolling(void){
    cam_status_t c = {0};
    c.connected=true; c.recording=true; c.recording_valid=true;
    c.battery_pct=80; c.battery_valid=true;
    c.remain_time_s=3600; c.remain_time_valid=true;
    c.temp_over=0; c.temp_over_valid=true;
    return c;
}

int main(void){
    char out[4][17];
    printf("Warning tests\n\n");

    /* 1. Wants to record, camera connected but not rolling -> REC? on both. */
    printf("1. armed but not recording -> REC?\n");
    { cam_status_t c=rolling(); c.recording=false;
      camlink_format_osd(&c,true,true,0,true,L,CAMLINK_SETUP_NONE,out);
      EXPECT(0,"REC?"); EXPECT(1,"REC?"); }

    /* 2. Actually rolling -> no warning, state REC. */
    printf("2. recording -> quiet\n");
    { cam_status_t c=rolling();
      camlink_format_osd(&c,true,true,0,true,L,CAMLINK_SETUP_NONE,out);
      EXPECT(0,""); EXPECT(1,"REC"); }

    /* 3. Too hot outranks a low battery. */
    printf("3. too hot wins\n");
    { cam_status_t c=rolling(); c.temp_over=2; c.battery_pct=5;
      camlink_format_osd(&c,true,true,0,true,L,CAMLINK_SETUP_NONE,out);
      EXPECT(0,"CAM HOT"); EXPECT(1,"CAM HOT"); }

    /* 4. No card (remaining == 0). */
    printf("4. no card\n");
    { cam_status_t c=rolling(); c.remain_time_s=0;
      camlink_format_osd(&c,true,true,0,true,L,CAMLINK_SETUP_NONE,out);
      EXPECT(0,"NO SD"); }

    /* 5. Card nearly full. */
    printf("5. card low\n");
    { cam_status_t c=rolling(); c.remain_time_s=60;
      camlink_format_osd(&c,true,true,0,true,L,CAMLINK_SETUP_NONE,out);
      EXPECT(0,"SD LOW"); }

    /* 6. Battery low. */
    printf("6. battery low\n");
    { cam_status_t c=rolling(); c.battery_pct=12;
      camlink_format_osd(&c,true,true,0,true,L,CAMLINK_SETUP_NONE,out);
      EXPECT(0,"BAT LOW"); }

    /* 7. A warning blinks: off-beat frame is blank even while it is active. */
    printf("7. blink: off-beat is blank\n");
    { cam_status_t c=rolling(); c.battery_pct=12;
      camlink_format_osd(&c,true,true,0,false,L,CAMLINK_SETUP_NONE,out);
      EXPECT(0,""); }

    /* 8. No camera: the warning stays quiet (state already says NO CAM). */
    printf("8. no camera -> quiet warning\n");
    { cam_status_t c={0}; c.connected=true; c.recording_valid=false;
      camlink_format_osd(&c,true,true,0,true,L,CAMLINK_SETUP_NONE,out);
      EXPECT(0,""); EXPECT(1,"NO CAM"); }

    /* 9. Unconfirmed battery does not raise BAT LOW. */
    printf("9. unconfirmed battery is not a warning\n");
    { cam_status_t c=rolling(); c.battery_valid=false; c.battery_pct=0;
      camlink_format_osd(&c,true,true,0,true,L,CAMLINK_SETUP_NONE,out);
      EXPECT(0,""); }

    printf(fails?"\n%d FAILURE(S)\n":"\nALL PASS (0 failures)\n", fails);
    return fails?1:0;
}
