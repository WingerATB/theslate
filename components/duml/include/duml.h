/* SPDX-License-Identifier: PolyForm-Strict-1.0.0
 *
 * DUML — the protocol DJI's "Osmo" class cameras actually speak over BLE.
 *
 * This is NOT the DJI R SDK protocol in components/dji_link (magic 0xAA) that
 * the Osmo Action series uses. Osmo Pocket 3 and Osmo Nano use DUML, magic
 * 0x55, and silently ignore R SDK frames — which is exactly what we measured.
 *
 * Wire format:
 *   [0x55][len_lo][ver<<2|len_hi][crc8][target:2 LE][msgId:2 BE]
 *   [flags][cmdSet][cmdId][payload...][crc16:2 LE]
 *
 *   len   : 10-bit, total frame length including the CRC16 (= 13 + payload)
 *   crc8  : over bytes [0..2]      poly 0x31,   init 0xEE,   reflected
 *   crc16 : over everything before poly 0x1021, init 0x496C, reflected
 *   target: sender | (receiver << 8)
 *   msgId : BIG endian (everything else is little endian)
 *
 * Verified against a real captured frame in test/host/test_duml.c.
 */
#ifndef CAMLINK_DUML_H
#define CAMLINK_DUML_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* Subsystem addresses */
#define DUML_DEV_CAMERA   0x01
#define DUML_DEV_APP      0x02
#define DUML_DEV_GIMBAL   0x04
#define DUML_DEV_RC       0x06
#define DUML_DEV_WIFI     0x07

#define DUML_TARGET(sender, receiver)  ((uint16_t)((sender) | ((receiver) << 8)))

/* Flags */
#define DUML_FLAG_REQUEST   0x40
#define DUML_FLAG_RESPONSE  0xC0
#define DUML_FLAG_NOTIFY    0x00

/* CmdSet 0x07 = Wi-Fi / pairing */
#define DUML_CMDSET_WIFI        0x07
#define DUML_CMD_SET_PAIRING_PIN 0x45
#define DUML_CMD_PAIRING_OK      0x46
#define DUML_CMD_WIFI_CONNECT    0x47

#define DUML_HEADER_LEN   11
#define DUML_OVERHEAD     13   /* header + crc16 */

uint8_t  duml_crc8(const uint8_t *data, size_t len);
uint16_t duml_crc16(const uint8_t *data, size_t len);

/* Build a DUML frame into out (must hold DUML_OVERHEAD + payload_len).
 * Returns the total frame length, or 0 on error. */
size_t duml_build(uint8_t *out, size_t out_cap,
                  uint16_t target, uint8_t flags,
                  uint8_t cmd_set, uint8_t cmd_id,
                  const uint8_t *payload, size_t payload_len);

/* Append a length-prefixed string: [len_u8][bytes]. Returns bytes written. */
size_t duml_pack_string(uint8_t *out, size_t out_cap, const char *s);

typedef struct {
    uint16_t length;
    uint8_t  sender;
    uint8_t  receiver;
    uint16_t msg_id;
    uint8_t  flags;
    uint8_t  cmd_set;
    uint8_t  cmd_id;
    const uint8_t *payload;
    size_t   payload_len;
    bool     crc_ok;
} duml_frame_t;

/* Parse one frame. Returns true if a structurally valid frame was found. */
bool duml_parse(const uint8_t *data, size_t len, duml_frame_t *out);

void duml_reset_sequence(uint16_t v);

#endif /* CAMLINK_DUML_H */
