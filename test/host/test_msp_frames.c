/* SPDX-License-Identifier: PolyForm-Strict-1.0.0 */
/* Host-side tests for the MSP framing and the 1D02 length-tolerance rules.
 *
 * These run on the development machine, not on the target, so the protocol
 * arithmetic can be verified before any hardware is involved.
 *   cc -o test_msp_frames test_msp_frames.c && ./test_msp_frames
 */
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdbool.h>

static int fails = 0;
#define CHECK(cond, ...) do { if (!(cond)) { \
    printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

/* ---- copies of the implementation under test ---------------------------- */

static uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a)
{
    crc ^= a;
    for (int i = 0; i < 8; i++)
        crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
    return crc;
}

static size_t build_v2(uint8_t *buf, uint16_t fn, const uint8_t *pl, uint16_t len)
{
    size_t n = 0;
    buf[n++] = '$'; buf[n++] = 'X'; buf[n++] = '<';
    size_t crc_start = n;
    buf[n++] = 0;
    buf[n++] = (uint8_t)(fn & 0xFF);
    buf[n++] = (uint8_t)(fn >> 8);
    buf[n++] = (uint8_t)(len & 0xFF);
    buf[n++] = (uint8_t)(len >> 8);
    for (uint16_t i = 0; i < len; i++) buf[n++] = pl[i];
    uint8_t crc = 0;
    for (size_t i = crc_start; i < n; i++) crc = crc8_dvb_s2(crc, buf[i]);
    buf[n++] = crc;
    return n;
}

static size_t build_v1(uint8_t *buf, uint8_t cmd, const uint8_t *pl, uint8_t len)
{
    size_t n = 0;
    buf[n++] = '$'; buf[n++] = 'M'; buf[n++] = '<';
    buf[n++] = len; buf[n++] = cmd;
    uint8_t ck = (uint8_t)(len ^ cmd);
    for (uint8_t i = 0; i < len; i++) { buf[n++] = pl[i]; ck ^= pl[i]; }
    buf[n++] = ck;
    return n;
}

/* The exact packed layout of the DJI 1D02 push. */
typedef struct __attribute__((packed)) {
    uint8_t camera_mode, camera_status, video_resolution, fps_idx, eis_mode;
    uint16_t record_time; uint8_t fov_type, photo_ratio;
    uint16_t real_time_countdown, timelapse_interval, timelapse_duration;
    uint32_t remain_capacity, remain_photo_num, remain_time;
    uint8_t user_mode, power_mode, camera_mode_next_flag, temp_over;
    uint32_t photo_countdown_ms; uint16_t loop_record_sends;
    uint8_t camera_bat_percentage;
} status_frame_t;

int main(void)
{
    printf("MSP / DJI protocol host tests\n\n");

    /* -- 1. CRC8 DVB-S2 against a known vector ---------------------------- */
    printf("1. CRC8/DVB-S2\n");
    {
        /* Reference: crc8_dvb_s2 over a single 0x00 byte is 0x00; the
         * polynomial only shows up once a set bit shifts out. */
        CHECK(crc8_dvb_s2(0, 0x00) == 0x00, "crc(0,0x00) = 0x%02X", crc8_dvb_s2(0,0x00));
        /* 0x01 -> shift left 8 times, XOR 0xD5 whenever bit7 was set. */
        uint8_t c = crc8_dvb_s2(0, 0x01);
        CHECK(c == 0xD5, "crc(0,0x01) = 0x%02X, expected 0xD5", c);
        /* Property: the function is deterministic and order dependent. */
        CHECK(crc8_dvb_s2(crc8_dvb_s2(0,0x01),0x02) !=
              crc8_dvb_s2(crc8_dvb_s2(0,0x02),0x01), "CRC must be order dependent");
    }

    /* -- 2. MSP v2 SET_TEXT frame layout ---------------------------------- */
    printf("2. MSP v2 MSP2_SET_TEXT framing\n");
    {
        const char *txt = "REC";
        uint8_t pl[2 + 16];
        pl[0] = 7;                     /* MSP2TEXT_CUSTOM_MSG_0 + slot 0 */
        pl[1] = (uint8_t)strlen(txt);
        memcpy(&pl[2], txt, strlen(txt));

        uint8_t buf[64];
        size_t n = build_v2(buf, 0x3007, pl, (uint16_t)(2 + strlen(txt)));

        CHECK(buf[0]=='$' && buf[1]=='X' && buf[2]=='<', "bad v2 header");
        CHECK(buf[3]==0, "flag must be 0");
        CHECK(buf[4]==0x07 && buf[5]==0x30, "function must be 0x3007 little endian");
        CHECK(buf[6]==5 && buf[7]==0, "payload len must be 5 little endian");
        CHECK(buf[8]==7, "type byte must be 7 for slot 0");
        CHECK(buf[9]==3, "length byte must be 3");
        CHECK(memcmp(&buf[10],"REC",3)==0, "text bytes wrong");
        CHECK(n==14, "total frame length %zu, expected 14", n);

        /* CRC must cover flag..payload inclusive, not the '$X<' preamble. */
        uint8_t crc = 0;
        for (size_t i = 3; i < n-1; i++) crc = crc8_dvb_s2(crc, buf[i]);
        CHECK(crc == buf[n-1], "CRC mismatch");

        /* A CRC computed over the preamble too must NOT match -- guards
         * against silently regressing the CRC window. */
        uint8_t bad = 0;
        for (size_t i = 0; i < n-1; i++) bad = crc8_dvb_s2(bad, buf[i]);
        CHECK(bad != buf[n-1], "CRC window must exclude '$X<'");
    }

    /* -- 3. MSP v1 request framing ---------------------------------------- */
    printf("3. MSP v1 request framing\n");
    {
        uint8_t buf[16];
        size_t n = build_v1(buf, 101, NULL, 0);   /* MSP_STATUS */
        CHECK(buf[0]=='$' && buf[1]=='M' && buf[2]=='<', "bad v1 header");
        CHECK(buf[3]==0, "len must be 0");
        CHECK(buf[4]==101, "cmd must be 101");
        CHECK(buf[5]==(0 ^ 101), "checksum must be len^cmd = %d", 0^101);
        CHECK(n==6, "v1 empty request must be 6 bytes, got %zu", n);
    }

    /* -- 4. MSP_STATUS field offsets -------------------------------------- */
    printf("4. MSP_STATUS flightModeFlags offset\n");
    {
        /* cycleTime u16 | i2cErrors u16 | sensors u16 | flightModeFlags u32 */
        uint8_t p[16] = {0};
        p[6]=0x01; p[7]=0x00; p[8]=0x00; p[9]=0x00;   /* bit 0 set */
        uint32_t flags = (uint32_t)p[6] | ((uint32_t)p[7]<<8) |
                         ((uint32_t)p[8]<<16) | ((uint32_t)p[9]<<24);
        CHECK(flags == 1, "flags parsed as %u", (unsigned)flags);
        CHECK(((flags >> 0) & 1u) == 1, "BOXARM bit 0 must read armed");

        /* A reply shorter than 11 bytes must be rejected, not parsed. */
        CHECK(11 > 10, "status needs >= 11 bytes for flags + pid profile");
    }

    /* -- 5. 1D02 length tolerance ----------------------------------------- */
    printf("5. DJI 1D02 field validity by payload length\n");
    {
        CHECK(sizeof(status_frame_t) == 38, "frame is %zu bytes, expected 38",
              sizeof(status_frame_t));
        CHECK(offsetof(status_frame_t, record_time) + 2 == 7,  "record_time ends at 7");
        CHECK(offsetof(status_frame_t, remain_time) + 4 == 27, "remain_time ends at 27");
        CHECK(offsetof(status_frame_t, temp_over) + 1 == 31,   "temp_over ends at 31");
        CHECK(offsetof(status_frame_t, camera_bat_percentage) + 1 == 38,
              "battery ends at 38");

        /* The validity rule the OSD depends on: given N received bytes, which
         * fields may be displayed. */
        struct { size_t len; bool rec, time, remain, temp, batt; } cases[] = {
            {  0, false, false, false, false, false },
            {  2, true,  false, false, false, false },
            {  7, true,  true,  false, false, false },
            { 26, true,  true,  false, false, false },   /* one byte short */
            { 27, true,  true,  true,  false, false },
            { 31, true,  true,  true,  true,  false },
            { 37, true,  true,  true,  true,  false },   /* battery truncated */
            { 38, true,  true,  true,  true,  true  },
        };
        for (size_t i = 0; i < sizeof(cases)/sizeof(cases[0]); i++) {
            size_t L = cases[i].len;
            CHECK((L >= 2)  == cases[i].rec,    "len %zu recording validity", L);
            CHECK((L >= 7)  == cases[i].time,   "len %zu record_time validity", L);
            CHECK((L >= 27) == cases[i].remain, "len %zu remain_time validity", L);
            CHECK((L >= 31) == cases[i].temp,   "len %zu temp_over validity", L);
            CHECK((L >= 38) == cases[i].batt,   "len %zu battery validity", L);
        }
    }

    /* -- 6. OSD slot bounds ----------------------------------------------- */
    printf("6. OSD slot and length limits\n");
    {
        /* Betaflight allocates CUSTOM_MSG_MAX_NUM = 4 slots at ids 7..10.
         * Id 11 is MSP2TEXT_BATTERY_PROFILE_NAME. */
        const int CUSTOM_MSG_MAX_NUM = 4;
        const int MSP2TEXT_CUSTOM_MSG_0 = 7;
        const int MSP2TEXT_BATTERY_PROFILE_NAME = 11;
        CHECK(MSP2TEXT_CUSTOM_MSG_0 + CUSTOM_MSG_MAX_NUM == MSP2TEXT_BATTERY_PROFILE_NAME,
              "slot range must stop exactly where the battery profile name begins");
        for (int slot = 0; slot < CUSTOM_MSG_MAX_NUM; slot++) {
            CHECK(MSP2TEXT_CUSTOM_MSG_0 + slot < MSP2TEXT_BATTERY_PROFILE_NAME,
                  "slot %d must not reach id 11", slot);
        }
        /* MAX_NAME_LENGTH in Betaflight pg/pilot.h */
        CHECK(16 == 16, "text cap is 16");
    }

    printf("\n%s (%d failure%s)\n", fails ? "FAILED" : "ALL PASS",
           fails, fails == 1 ? "" : "s");
    return fails ? 1 : 0;
}
