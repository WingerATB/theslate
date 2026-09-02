/* SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0 */
/* Host tests for DUML framing (DJI Osmo BLE protocol).
 *
 * Golden vector from lib-osmo-ble's tools/verify-crc.mjs -- a real
 * SetPairingPIN message captured from a DJI Osmo:
 *   552204ea020780924007450f30303137343933313932383631303204353136302e42
 *
 *   cc -Wall -o test_duml test_duml.c && ./test_duml
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>

static int fails = 0;
#define CHECK(c, ...) do { if(!(c)){ printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); fails++; } } while(0)

/* ---- Rocksoft-model CRC with explicit reflection ------------------------ */
static uint8_t reflect8(uint8_t v)
{
    uint8_t r = 0;
    for (int i = 0; i < 8; i++) if (v & (1u << i)) r |= (uint8_t)(1u << (7 - i));
    return r;
}
static uint16_t reflect16(uint16_t v)
{
    uint16_t r = 0;
    for (int i = 0; i < 16; i++) if (v & (1u << i)) r |= (uint16_t)(1u << (15 - i));
    return r;
}

/* CRC8: poly=0x31 init=0xEE xorOut=0x00 refIn=true refOut=true */
uint8_t duml_crc8(const uint8_t *d, size_t n)
{
    uint8_t crc = 0xEE;
    for (size_t i = 0; i < n; i++) {
        crc ^= reflect8(d[i]);
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
    }
    return reflect8(crc);
}

/* CRC16: poly=0x1021 init=0x496C xorOut=0x0000 refIn=true refOut=true */
uint16_t duml_crc16(const uint8_t *d, size_t n)
{
    uint16_t crc = 0x496C;
    for (size_t i = 0; i < n; i++) {
        crc ^= (uint16_t)reflect8(d[i]) << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return reflect16(crc);
}

static int hexb(const char *h, uint8_t *out)
{
    int n = 0;
    for (const char *p = h; p[0] && p[1]; p += 2) {
        unsigned v; sscanf(p, "%2x", &v); out[n++] = (uint8_t)v;
    }
    return n;
}

int main(void)
{
    printf("DUML framing tests\n\n");

    const char *hex =
      "552204ea020780924007450f30303137343933313932383631303204353136302e42";
    uint8_t m[64];
    int n = hexb(hex, m);

    printf("1. golden frame structure\n");
    CHECK(n == 34, "length %d, expected 34", n);
    CHECK(m[0] == 0x55, "magic 0x%02X, expected 0x55", m[0]);
    int len = m[1] | ((m[2] & 0x03) << 8);
    int ver = m[2] >> 2;
    CHECK(len == 34, "declared length %d", len);
    CHECK(ver == 1, "version %d, expected 1", ver);
    CHECK(m[4] == 0x02, "sender 0x%02X, expected 0x02 (App)", m[4]);
    CHECK(m[5] == 0x07, "receiver 0x%02X, expected 0x07 (WiFi)", m[5]);
    CHECK(((m[6] << 8) | m[7]) == 0x8092, "msgId BE wrong");
    CHECK(m[8] == 0x40, "flags 0x%02X, expected 0x40 (request)", m[8]);
    CHECK(m[9] == 0x07, "cmdSet 0x%02X, expected 0x07", m[9]);
    CHECK(m[10] == 0x45, "cmdId 0x%02X, expected 0x45 (SetPairingPIN)", m[10]);

    printf("2. CRC8 over header bytes [0..2]\n");
    uint8_t c8 = duml_crc8(m, 3);
    CHECK(c8 == m[3], "CRC8 computed 0x%02X, frame has 0x%02X", c8, m[3]);

    printf("3. CRC16 over everything before the trailer\n");
    uint16_t c16 = duml_crc16(m, (size_t)n - 2);
    uint16_t stored = (uint16_t)(m[n-2] | (m[n-1] << 8));
    CHECK(c16 == stored, "CRC16 computed 0x%04X, frame has 0x%04X", c16, stored);

    printf("4. packString payload decoding\n");
    CHECK(m[11] == 0x0F, "identifier length 0x%02X, expected 0x0F", m[11]);
    CHECK(memcmp(&m[12], "001749319286102", 15) == 0, "identifier mismatch");
    CHECK(m[27] == 0x04, "PIN length 0x%02X, expected 0x04", m[27]);
    CHECK(memcmp(&m[28], "5160", 4) == 0, "PIN mismatch");

    printf("\n%s (%d failure%s)\n", fails ? "FAILED" : "ALL PASS",
           fails, fails == 1 ? "" : "s");
    return fails ? 1 : 0;
}
