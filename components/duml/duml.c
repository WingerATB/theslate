/* SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0 */
#include <string.h>
#include "duml.h"

static uint16_t s_msg_seq = 0x0100;

void duml_reset_sequence(uint16_t v) { s_msg_seq = v; }

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

uint8_t duml_crc8(const uint8_t *d, size_t n)
{
    uint8_t crc = 0xEE;                       /* init */
    for (size_t i = 0; i < n; i++) {
        crc ^= reflect8(d[i]);                /* refIn */
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
    }
    return reflect8(crc);                     /* refOut */
}

uint16_t duml_crc16(const uint8_t *d, size_t n)
{
    uint16_t crc = 0x496C;                    /* init */
    for (size_t i = 0; i < n; i++) {
        crc ^= (uint16_t)reflect8(d[i]) << 8; /* refIn */
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return reflect16(crc);                    /* refOut */
}

size_t duml_build(uint8_t *out, size_t out_cap,
                  uint16_t target, uint8_t flags,
                  uint8_t cmd_set, uint8_t cmd_id,
                  const uint8_t *payload, size_t payload_len)
{
    const size_t total = DUML_OVERHEAD + payload_len;
    if (out == NULL || out_cap < total || total > 1023) {
        return 0;
    }

    size_t off = 0;
    out[off++] = 0x55;
    out[off++] = (uint8_t)(total & 0xFF);
    out[off++] = (uint8_t)(((total >> 8) & 0x03) | (1u << 2));  /* version 1 */
    out[off++] = duml_crc8(out, 3);

    out[off++] = (uint8_t)(target & 0xFF);          /* target, little endian */
    out[off++] = (uint8_t)(target >> 8);

    out[off++] = (uint8_t)(s_msg_seq >> 8);         /* msgId, BIG endian */
    out[off++] = (uint8_t)(s_msg_seq & 0xFF);
    s_msg_seq++;

    out[off++] = flags;
    out[off++] = cmd_set;
    out[off++] = cmd_id;

    if (payload != NULL && payload_len > 0) {
        memcpy(&out[off], payload, payload_len);
        off += payload_len;
    }

    uint16_t crc = duml_crc16(out, off);
    out[off++] = (uint8_t)(crc & 0xFF);
    out[off++] = (uint8_t)(crc >> 8);

    return off;
}

size_t duml_pack_string(uint8_t *out, size_t out_cap, const char *s)
{
    size_t n = (s != NULL) ? strlen(s) : 0;
    if (n > 255 || out_cap < n + 1) return 0;
    out[0] = (uint8_t)n;
    if (n) memcpy(&out[1], s, n);
    return n + 1;
}

bool duml_parse(const uint8_t *data, size_t len, duml_frame_t *out)
{
    if (data == NULL || out == NULL || len < DUML_OVERHEAD || data[0] != 0x55) {
        return false;
    }
    uint16_t flen = (uint16_t)(data[1] | ((data[2] & 0x03) << 8));
    if (flen < DUML_OVERHEAD || flen > len) return false;

    out->length      = flen;
    out->sender      = data[4];
    out->receiver    = data[5];
    out->msg_id      = (uint16_t)((data[6] << 8) | data[7]);   /* BE */
    out->flags       = data[8];
    out->cmd_set     = data[9];
    out->cmd_id      = data[10];
    out->payload     = &data[DUML_HEADER_LEN];
    out->payload_len = (size_t)(flen - DUML_OVERHEAD);

    uint16_t stored = (uint16_t)(data[flen - 2] | (data[flen - 1] << 8));
    out->crc_ok = (duml_crc16(data, (size_t)flen - 2) == stored)
                  && (duml_crc8(data, 3) == data[3]);
    return true;
}
