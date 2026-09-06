/* Host tests for the Open GoPro wire format -- the shipped implementation.
 *   cc -I../../components/gopro/include -o test_gopro test_gopro.c && ./test_gopro
 */
#include <stdio.h>
#include <string.h>
#include "gopro_proto.h"

static int fails = 0;
#define CHECK(c, m) do { if (!(c)) { printf("   FAIL: %s\n", (m)); fails++; } } while (0)

/* Feed a packet, expect no complete message yet. */
#define FEED_PARTIAL(st, arr) do {                                        \
    uint16_t _l; const uint8_t _p[] = arr;                                \
    const uint8_t *_m = gopro_reasm_feed(&(st), _p, sizeof(_p), &_l);     \
    if (_m != NULL) { printf("   FAIL: completed too early\n"); fails++; }\
} while (0)

int main(void)
{
    printf("=== GoPro packet reassembly ===\n\n");

    printf("1. a General (5-bit) header, whole message in one packet\n");
    {
        /* 02 13 00 -- length 2, then the message [0x13][0x00]. */
        gopro_reasm_t st = {0};
        const uint8_t p[] = { 0x02, 0x13, 0x00 };
        uint16_t len = 0;
        const uint8_t *m = gopro_reasm_feed(&st, p, sizeof(p), &len);
        CHECK(m != NULL, "should complete in one packet");
        CHECK(len == 2, "length 2");
        if (m) CHECK(m[0] == 0x13 && m[1] == 0x00, "payload is the message, header stripped");
    }

    printf("2. an Extended-13 header\n");
    {
        /* 20 03 01 01 01 -- ext-13, length 3, Set Shutter On. */
        gopro_reasm_t st = {0};
        const uint8_t p[] = { 0x20, 0x03, 0x01, 0x01, 0x01 };
        uint16_t len = 0;
        const uint8_t *m = gopro_reasm_feed(&st, p, sizeof(p), &len);
        CHECK(m != NULL && len == 3, "length 3 from the two-byte header");
        if (m) CHECK(m[0] == 0x01 && m[1] == 0x01 && m[2] == 0x01, "payload correct");
    }

    printf("3. the real HERO9 capture: ext-13 start plus a continuation\n");
    {
        /* From a genuine register-for-all-statuses reply. Header 0x21 0x73 =
         * ext-13, length 0x173 is far too big for us -- so use the documented
         * shape at a length we do hold. */
        gopro_reasm_t st = {0};
        uint16_t len = 0;
        /* length 0x18 = 24 payload bytes, split 18 + 6 */
        const uint8_t p1[] = { 0x20, 0x18,
            1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18 };
        const uint8_t *m = gopro_reasm_feed(&st, p1, sizeof(p1), &len);
        CHECK(m == NULL, "not complete after the start packet");

        const uint8_t p2[] = { 0x80, 19,20,21,22,23,24 };
        m = gopro_reasm_feed(&st, p2, sizeof(p2), &len);
        CHECK(m != NULL && len == 24, "complete after the continuation");
        if (m) {
            int ok = 1;
            for (int i = 0; i < 24; i++) if (m[i] != i + 1) ok = 0;
            CHECK(ok, "continuation header stripped, payload contiguous");
        }
    }

    printf("4. the continuation counter is ignored, not validated\n");
    {
        /* GoPro's own Kotlin SDK sends a constant 0x80 for every continuation
         * and never increments it, and neither SDK checks it on receive. A
         * parser that enforced the counter would reject valid traffic. */
        gopro_reasm_t st = {0};
        uint16_t len = 0;
        const uint8_t p1[] = { 0x20, 0x06, 1, 2 };
        gopro_reasm_feed(&st, p1, sizeof(p1), &len);
        const uint8_t p2[] = { 0x8F, 3, 4 };     /* counter 0xF, out of order */
        gopro_reasm_feed(&st, p2, sizeof(p2), &len);
        const uint8_t p3[] = { 0x80, 5, 6 };     /* counter back to 0        */
        const uint8_t *m = gopro_reasm_feed(&st, p3, sizeof(p3), &len);
        CHECK(m != NULL && len == 6, "accepted regardless of the counter");
    }

    printf("5. BLOCKER: an oversized message is dropped whole, not truncated\n");
    {
        /* A half-parsed status is exactly the confidently-wrong reading this
         * firmware refuses to display. */
        gopro_reasm_t st = {0};
        uint16_t len = 0;
        uint8_t big[3];
        big[0] = 0x20 | ((GOPRO_MSG_MAX + 50) >> 8);
        big[1] = (GOPRO_MSG_MAX + 50) & 0xFF;
        big[2] = 0xAA;
        const uint8_t *m = gopro_reasm_feed(&st, big, sizeof(big), &len);
        CHECK(m == NULL, "oversized start returns nothing");
        CHECK(st.dropping, "and marks itself dropping");

        const uint8_t cont[] = { 0x80, 1, 2, 3 };
        m = gopro_reasm_feed(&st, cont, sizeof(cont), &len);
        CHECK(m == NULL, "its continuations are swallowed");

        /* And the NEXT message must parse correctly. */
        const uint8_t good[] = { 0x02, 0x13, 0x00 };
        m = gopro_reasm_feed(&st, good, sizeof(good), &len);
        CHECK(m != NULL && len == 2, "the message after it is unaffected");
    }

    printf("6. a start packet abandons an incomplete message\n");
    {
        gopro_reasm_t st = {0};
        uint16_t len = 0;
        const uint8_t p1[] = { 0x20, 0x10, 1, 2, 3 };   /* wants 16, got 3 */
        gopro_reasm_feed(&st, p1, sizeof(p1), &len);
        const uint8_t p2[] = { 0x02, 0x13, 0x00 };      /* new start */
        const uint8_t *m = gopro_reasm_feed(&st, p2, sizeof(p2), &len);
        CHECK(m != NULL && len == 2, "the new message parses");
        if (m) CHECK(m[0] == 0x13, "and is not spliced onto the abandoned one");
    }

    printf("7. a continuation with no start is ignored\n");
    {
        gopro_reasm_t st = {0};
        uint16_t len = 0;
        const uint8_t p[] = { 0x80, 1, 2, 3 };
        CHECK(gopro_reasm_feed(&st, p, sizeof(p), &len) == NULL, "ignored");
    }

    printf("8. the reserved header type is refused rather than guessed at\n");
    {
        gopro_reasm_t st = {0};
        uint16_t len = 0;
        const uint8_t p[] = { 0x60, 0x04, 1, 2, 3, 4 };
        CHECK(gopro_reasm_feed(&st, p, sizeof(p), &len) == NULL, "type 11 refused");
    }

    printf("9. building a message uses the Extended-13 header, as GoPro's SDK does\n");
    {
        /* Pinned against the official Python SDK's fragmenter, which emits
         * exactly this for a three-byte message: 20 03 followed by the
         * payload. The specification's own instruction is "Always use
         * Extended (13-bit) packet headers when sending messages". If this
         * test ever needs "fixing", check the SDK first, not the encoder. */
        uint8_t out[8];
        const uint8_t shutter_on[] = { 0x01, 0x01, 0x01 };
        uint16_t n = gopro_pack(out, sizeof(out), shutter_on, 3);
        CHECK(n == 5, "3-byte message becomes 5 bytes on the wire");
        CHECK(out[0] == 0x20 && out[1] == 0x03, "ext-13 header, length 3");
        CHECK(out[2] == 0x01 && out[3] == 0x01 && out[4] == 0x01, "payload");

        /* And it must round-trip through our own parser. */
        gopro_reasm_t st = {0};
        uint16_t len = 0;
        const uint8_t *m = gopro_reasm_feed(&st, out, n, &len);
        CHECK(m != NULL && len == 3, "round-trips");

        uint8_t small[4];
        CHECK(gopro_pack(small, sizeof(small), shutter_on, 3) == 0,
              "refuses to overflow the caller's buffer");
    }

    printf("9b. the exact bytes this firmware sends, against GoPro's documented forms\n");
    {
        /* Each one is the documented TLV wrapped the way the SDK wraps it.
         * The keep-alive is a SETTING write: setting 91 (LED) = 66, which the
         * SDK names LED_SPECIAL.BLE_KEEP_ALIVE and the tutorial serialises as
         * [id][len][value] -- the same shape as its 03 02 01 xx resolution
         * example. */
        uint8_t out[16]; uint16_t n;

        const uint8_t hw[] = { 0x3C };
        n = gopro_pack(out, sizeof(out), hw, sizeof(hw));
        CHECK(n == 3 && out[0] == 0x20 && out[1] == 0x01 && out[2] == 0x3C,
              "Get Hardware Info: 20 01 3C");

        const uint8_t ka[] = { 0x5B, 0x01, 0x42 };
        n = gopro_pack(out, sizeof(out), ka, sizeof(ka));
        CHECK(n == 5 && out[0] == 0x20 && out[1] == 0x03 &&
              out[2] == 0x5B && out[3] == 0x01 && out[4] == 0x42,
              "keep-alive: 20 03 5B 01 42");

        const uint8_t reg[] = { 0x53, 10, 13 };
        n = gopro_pack(out, sizeof(out), reg, sizeof(reg));
        CHECK(n == 5 && out[0] == 0x20 && out[1] == 0x03 &&
              out[2] == 0x53 && out[3] == 10 && out[4] == 13,
              "register statuses: 20 03 53 0A 0D -- bare ids, no inner length");
    }

    printf("9c. the camera's own short form still parses: real SDK test vectors\n");
    {
        /* From the Kotlin SDK's bleByteData.kt, byte for byte: a General
         * header on a status push and on a registration reply. */
        gopro_reasm_t st = {0}; uint16_t len = 0; gopro_status_t s = {0};

        const uint8_t push[] = { 0x05, 0x93, 0x00, 0x0A, 0x01, 0x01 };
        const uint8_t *m = gopro_reasm_feed(&st, push, sizeof(push), &len);
        CHECK(m != NULL && len == 5, "isEncodingNotificationMessage reassembles");
        CHECK(gopro_status_apply(&s, m, len) == 1 && s.encoding_valid && s.encoding,
              "and reads as encoding");

        const uint8_t reg[] = { 0x05, 0x53, 0x00, 0x08, 0x01, 0x01 };
        m = gopro_reasm_feed(&st, reg, sizeof(reg), &len);
        CHECK(m != NULL && gopro_status_apply(&s, m, len) == 1 && s.busy_valid && s.busy,
              "registerBusyResponseMessage reads as busy");
    }

    printf("9d. MISSION 1: the 2-byte status-id family, both directions\n");
    {
        /* GoPro's Data Protocol page: 0x13/0x53/0x93 become 0x16/0x56/0x96,
         * element ids are two bytes big-endian in requests AND results, and a
         * setting change uses a 0xFF marker then a 2-byte id. */
        uint8_t out[32]; uint16_t n;

        n = gopro_status_reg_build(out, sizeof(out), false);
        CHECK(n == 9 && out[0] == 0x53 && out[1] == 10 && out[8] == 96,
              "1-byte registration: 53 then eight ids");

        n = gopro_status_reg_build(out, sizeof(out), true);
        CHECK(n == 17 && out[0] == 0x56, "2-byte registration: 56 then eight 2-byte ids");
        CHECK(out[1] == 0x00 && out[2] == 10 && out[15] == 0x00 && out[16] == 96,
              "each id is 00 xx, big-endian");
        CHECK(gopro_status_reg_build(out, 16, true) == 0, "refuses to overflow");

        /* A 2-byte push: 96 00 | 00 0A 01 01 -- encoding = true. */
        gopro_reasm_t st = {0}; uint16_t len = 0; gopro_status_t g = {0};
        const uint8_t push[] = { 0x06, 0x96, 0x00, 0x00, 0x0A, 0x01, 0x01 };
        const uint8_t *m = gopro_reasm_feed(&st, push, sizeof(push), &len);
        CHECK(m != NULL && gopro_status_apply(&g, m, len) == 1 && g.encoding_valid && g.encoding,
              "a 0x96 push with a 2-byte id reads as encoding");

        /* A 2-byte registration reply carrying a 4-byte clip time. */
        const uint8_t reply[] = { 0x09, 0x56, 0x00, 0x00, 0x0D, 0x04, 0x00, 0x00, 0x01, 0x2C };
        m = gopro_reasm_feed(&st, reply, sizeof(reply), &len);
        CHECK(m != NULL && gopro_status_apply(&g, m, len) == 1 && g.clip_valid && g.clip_s == 300,
              "a 0x56 reply: id 000D, len 4, value 300");

        /* The SAME bytes under a 1-byte query id would misparse -- which is why
         * the width comes from the query id and never from the data. */
        gopro_status_t h = {0};
        const uint8_t bad[] = { 0x93, 0x00, 0x00, 0x0A, 0x01, 0x01 };
        int r = gopro_status_apply(&h, bad, sizeof(bad));
        CHECK(!h.encoding_valid, "1-byte parse of 2-byte data does not invent 'encoding'");
        (void)r;

        /* An id above 255 -- which is what the width exists for -- is skipped,
         * not mistaken for one of ours, and parsing continues past it. */
        gopro_status_t k = {0};
        const uint8_t big[] = { 0x96, 0x00, 0x01, 0x2C, 0x01, 0x07, 0x00, 0x0A, 0x01, 0x01 };
        CHECK(gopro_status_apply(&k, big, sizeof(big)) == 1 && k.encoding && !k.busy_valid,
              "id 0x012C skipped, the encoding triple after it still lands");

        /* Truncated in the middle of a 2-byte id: stop cleanly. */
        gopro_status_t q = {0};
        const uint8_t cut[] = { 0x96, 0x00, 0x00 };
        CHECK(gopro_status_apply(&q, cut, sizeof(cut)) == 0, "half an id is nothing");

        /* Keep-alive in both forms, and the reply reader for each. */
        n = gopro_keepalive_build(out, sizeof(out), false);
        CHECK(n == 3 && out[0] == 0x5B && out[1] == 0x01 && out[2] == 0x42, "keep-alive: 5B 01 42");
        n = gopro_keepalive_build(out, sizeof(out), true);
        CHECK(n == 5 && out[0] == 0xFF && out[1] == 0x00 && out[2] == 0x5B &&
              out[3] == 0x01 && out[4] == 0x42, "wide keep-alive: FF 00 5B 01 42");
        uint16_t id = 0; uint8_t stt = 9;
        const uint8_t r1[] = { 0x5B, 0x00 };
        CHECK(gopro_setting_reply(r1, 2, &id, &stt) && id == 0x5B && stt == 0, "plain reply");
        const uint8_t r2[] = { 0xFF, 0x00, 0x5B, 0x02 };
        CHECK(gopro_setting_reply(r2, 4, &id, &stt) && id == 0x5B && stt == 2, "wide reply, refused");
        CHECK(!gopro_setting_reply(r2, 3, &id, &stt), "a short wide reply is not read");
    }

    printf("\n=== GoPro status parsing ===\n\n");

    printf("10. the statuses the OSD needs, from one push\n");
    {
        gopro_status_t s = {0};
        /* [0x93][status 0] then triples, exactly as the camera sends them. */
        const uint8_t msg[] = {
            0x93, 0x00,
            10, 1, 0x01,                      /* encoding = true            */
            13, 4, 0x00,0x00,0x00,0x2A,       /* clip = 42 s                */
            70, 1, 87,                        /* battery = 87 %             */
            35, 4, 0x00,0x00,0x1A,0xAA,       /* left = 6826 s              */
            33, 1, 0x00,                      /* storage OK                 */
             6, 1, 0x00,                      /* not overheating            */
             8, 1, 0x00,                      /* not busy                   */
        };
        int applied = gopro_status_apply(&s, msg, sizeof(msg));
        CHECK(applied == 7, "all seven understood");
        CHECK(s.encoding_valid && s.encoding, "encoding");
        CHECK(s.clip_valid && s.clip_s == 42, "clip seconds, big endian");
        CHECK(s.battery_valid && s.battery == 87, "battery percent");
        CHECK(s.left_valid && s.left_s == 6826, "remaining seconds");
        CHECK(s.storage_valid && s.storage == GOPRO_SD_OK, "storage");
        CHECK(s.overheat_valid && !s.overheat, "not overheating");
        CHECK(s.busy_valid && !s.busy, "not busy");
    }

    printf("11. BLOCKER: an undocumented status id must not break the parse\n");
    {
        /* Real HERO9 cameras return ids 3, 4, 14, 36, 37, 40, 57, 61, 62, 63,
         * 90, 91 and 109 that the specification does not document. A parser
         * that choked would fail on the very first reply. */
        gopro_status_t s = {0};
        const uint8_t msg[] = {
            0x53, 0x00,
            57, 2, 0xDE, 0xAD,            /* undocumented, before          */
            70, 1, 42,                    /* the one we want               */
            109, 4, 1,2,3,4,              /* undocumented, after           */
        };
        int applied = gopro_status_apply(&s, msg, sizeof(msg));
        CHECK(applied == 1, "only the known one counted");
        CHECK(s.battery_valid && s.battery == 42, "and it was read correctly");
    }

    printf("12. a zero-length value means 'no value yet', not zero\n");
    {
        /* Registering for a status the camera has no value for yet returns
         * length 0. Recording that as 0 would be a confident lie -- a battery
         * reading of 0%% on a full camera. */
        gopro_status_t s = {0};
        const uint8_t msg[] = { 0x53, 0x00, 70, 0 };
        gopro_status_apply(&s, msg, sizeof(msg));
        CHECK(!s.battery_valid, "stays unconfirmed rather than becoming 0");
    }

    printf("13. widths come from the length byte, not from a table\n");
    {
        /* The spec never states widths and the SDKs' assumed widths are not
         * promised stable. Reading the declared length is the only version-
         * proof approach -- so the same id must parse at any width. */
        gopro_status_t s = {0};
        const uint8_t one[]  = { 0x93, 0x00, 13, 1, 0x2A };
        gopro_status_apply(&s, one, sizeof(one));
        CHECK(s.clip_s == 42, "1-byte clip time");

        gopro_status_t t = {0};
        const uint8_t two[]  = { 0x93, 0x00, 13, 2, 0x01, 0x00 };
        gopro_status_apply(&t, two, sizeof(two));
        CHECK(t.clip_s == 256, "2-byte clip time, big endian");

        gopro_status_t u = {0};
        const uint8_t eight[] = { 0x93, 0x00, 13, 8, 0,0,0,0, 0,0,0x01,0x00 };
        gopro_status_apply(&u, eight, sizeof(eight));
        CHECK(u.clip_s == 256, "8-byte value keeps its low 32 bits");
    }

    printf("14. storage 'unknown' is signed, not 255\n");
    {
        /* The enum includes -1 and the camera delivers it as 0xFF in a
         * one-byte field. Read unsigned it would look like a valid state. */
        gopro_status_t s = {0};
        const uint8_t msg[] = { 0x93, 0x00, 33, 1, 0xFF };
        gopro_status_apply(&s, msg, sizeof(msg));
        CHECK(s.storage_valid && s.storage == GOPRO_SD_UNKNOWN, "-1, not 255");
    }

    printf("15. a truncated triple stops the parse cleanly\n");
    {
        gopro_status_t s = {0};
        const uint8_t msg[] = { 0x93, 0x00, 70, 1, 55, 13, 4, 0x00, 0x00 };
        int applied = gopro_status_apply(&s, msg, sizeof(msg));
        CHECK(applied == 1, "the complete triple before it still counts");
        CHECK(s.battery_valid && s.battery == 55, "and is correct");
        CHECK(!s.clip_valid, "the truncated one is not invented");
    }

    printf("16. a failed or foreign response is refused\n");
    {
        gopro_status_t s = {0};
        const uint8_t err[] = { 0x93, 0x01, 70, 1, 42 };
        CHECK(gopro_status_apply(&s, err, sizeof(err)) == -1, "error status refused");
        CHECK(!s.battery_valid, "and nothing was taken from it");

        const uint8_t setting[] = { 0x92, 0x00, 2, 1, 9 };
        CHECK(gopro_status_apply(&s, setting, sizeof(setting)) == -1,
              "a SETTING push is not a status push");

        const uint8_t tiny[] = { 0x93 };
        CHECK(gopro_status_apply(&s, tiny, sizeof(tiny)) == -1, "too short");
    }

    printf("17. pushes accumulate -- the camera only sends what changed\n");
    {
        gopro_status_t s = {0};
        const uint8_t a[] = { 0x53, 0x00, 70, 1, 90, 10, 1, 0 };
        gopro_status_apply(&s, a, sizeof(a));
        CHECK(s.battery == 90 && !s.encoding, "initial registration reply");

        const uint8_t b[] = { 0x93, 0x00, 10, 1, 1 };   /* recording started */
        gopro_status_apply(&s, b, sizeof(b));
        CHECK(s.encoding, "encoding updated");
        CHECK(s.battery_valid && s.battery == 90, "battery survives the push");
    }

    printf("\n%s (%d failure%s)\n", fails ? "FAILURES" : "ALL PASS",
           fails, fails == 1 ? "" : "s");
    return fails ? 1 : 0;
}
