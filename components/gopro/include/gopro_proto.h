/* SPDX-License-Identifier: PolyForm-Strict-1.0.0
 *
 * Open GoPro BLE wire format: packet reassembly and query-response parsing.
 *
 * Pure, and free of ESP-IDF, so the host tests exercise the shipped code rather
 * than a copy of it. That rule has earned itself six times in this project now;
 * this is the seventh, and the strongest case yet -- a fragmentation bug shows
 * up as a camera that connects, answers nothing, and looks like a dead link.
 *
 * WHY THERE IS A PACKET LAYER AT ALL. GoPro specifies its BLE protocol against
 * a fixed 20-byte packet, which is the default ATT MTU (23) minus the 3-byte
 * ATT header. Anything longer is split across notifications with a header on
 * each. Negotiating a larger MTU does NOT stop it: the fragmentation is an
 * application-layer scheme of GoPro's own, and the camera fragments regardless
 * of what the link will carry. So the reassembler is not an optimisation for
 * slow links, it is mandatory.
 */
#ifndef GOPRO_PROTO_H
#define GOPRO_PROTO_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* ==========================================================================
 * Packet layer
 * ========================================================================== */

/* Byte 0 of a packet. Bit 7 says start-or-continuation; on a start packet bits
 * 6-5 choose which of three length encodings follows. */
#define GOPRO_HDR_CONT_BIT   0x80
#define GOPRO_HDR_TYPE_MASK  0x60
#define GOPRO_HDR_TYPE_GEN   0x00   /* 1 header byte,  5-bit length          */
#define GOPRO_HDR_TYPE_E13   0x20   /* 2 header bytes, 13-bit length         */
#define GOPRO_HDR_TYPE_E16   0x40   /* 3 header bytes, 16-bit length, RX only */
#define GOPRO_HDR_GEN_LEN    0x1F

/* The largest reassembled message we will hold.
 *
 * Sized for what this firmware actually asks for: a status registration reply
 * carrying six values is under 50 bytes, and an async push is smaller still.
 * Get Hardware Info is the one long message we send, and it is read only to
 * identify the model. A message larger than this is DROPPED CLEANLY rather
 * than truncated -- a half-parsed status is exactly the kind of confidently
 * wrong reading this firmware refuses to display. */
#define GOPRO_MSG_MAX  192

typedef struct {
    uint8_t  buf[GOPRO_MSG_MAX];
    uint16_t want;      /* declared payload length; 0 when idle             */
    uint16_t got;
    bool     dropping;  /* oversized: swallow continuations until the next
                         * start packet, so one long message does not
                         * corrupt the one after it                          */
} gopro_reasm_t;

static inline void gopro_reasm_reset(gopro_reasm_t *st)
{
    st->want = 0;
    st->got = 0;
    st->dropping = false;
}

/* Feed one BLE notification.
 *
 * Returns a pointer to the complete message and writes its length to *out_len,
 * or NULL when more packets are needed (or the packet was malformed).
 *
 * The returned pointer is into the reassembler, valid until the next feed.
 */
static inline const uint8_t *gopro_reasm_feed(gopro_reasm_t *st,
                                              const uint8_t *p, uint16_t n,
                                              uint16_t *out_len)
{
    if (out_len) *out_len = 0;
    if (p == NULL || n == 0) return NULL;

    uint16_t off;

    if (p[0] & GOPRO_HDR_CONT_BIT) {
        /* Continuation. The 4-bit counter in bits 3-0 is deliberately ignored:
         * GoPro's own SDKs neither validate it on receive nor increment it on
         * send (the Kotlin SDK emits a constant 0x80), so a parser that
         * enforced it would reject traffic the camera considers valid. */
        if (st->dropping) return NULL;
        if (st->want == 0) return NULL;   /* continuation with no start */
        off = 1;
    } else {
        /* A start packet always abandons whatever was in progress. A truncated
         * message is not recoverable and holding on to it would splice two
         * replies together. */
        st->got = 0;
        st->dropping = false;

        switch (p[0] & GOPRO_HDR_TYPE_MASK) {
        case GOPRO_HDR_TYPE_GEN:
            st->want = (uint16_t)(p[0] & GOPRO_HDR_GEN_LEN);
            off = 1;
            break;
        case GOPRO_HDR_TYPE_E13:
            if (n < 2) { st->want = 0; return NULL; }
            st->want = (uint16_t)(((uint16_t)(p[0] & GOPRO_HDR_GEN_LEN) << 8) | p[1]);
            off = 2;
            break;
        case GOPRO_HDR_TYPE_E16:
            /* Receive-only: the camera sends this for messages of 8192 bytes
             * or more. Bits 4-0 of byte 0 are reserved and ignored. */
            if (n < 3) { st->want = 0; return NULL; }
            st->want = (uint16_t)(((uint16_t)p[1] << 8) | p[2]);
            off = 3;
            break;
        default:
            /* Type 11 is reserved. Treat as malformed rather than guessing. */
            st->want = 0;
            return NULL;
        }

        if (st->want == 0) return NULL;          /* zero-length: nothing to do */
        if (st->want > GOPRO_MSG_MAX) {
            st->dropping = true;                 /* too big -- skip it whole */
            st->want = 0;
            return NULL;
        }
    }

    if (off >= n) return NULL;                   /* header only, no payload */

    uint16_t avail = (uint16_t)(n - off);
    uint16_t need  = (uint16_t)(st->want - st->got);
    uint16_t take  = avail < need ? avail : need;

    memcpy(st->buf + st->got, p + off, take);
    st->got = (uint16_t)(st->got + take);

    if (st->got < st->want) return NULL;

    uint16_t len = st->want;
    st->want = 0;
    st->got  = 0;
    if (out_len) *out_len = len;
    return st->buf;
}

/* Wrap a message for sending, using the Extended-13 header for everything.
 *
 * Checked against both of GoPro's own sources rather than reasoned about:
 * the specification's Data Protocol page says, in those words, "Always use
 * Extended (13-bit) packet headers when sending messages", and the official
 * Python SDK's fragmenter (communicator_interface.py, _fragment) has exactly
 * one start-header branch for anything under 8191 bytes -- Extended-13 -- so
 * every camera in GoPro's compatibility table is exercised with this form on
 * every message the SDK sends. The General (5-bit) form is also accepted, and
 * GoPro's tutorial uses it for its four-byte shutter example, but there is no
 * reason to carry a second encoder and a boundary at 32 for a form the
 * reference implementation never emits.
 *
 * Returns the number of bytes written, or 0 if the message does not fit --
 * which for this firmware means a caller bug, since every message it sends
 * is a handful of bytes. */
static inline uint16_t gopro_pack(uint8_t *out, uint16_t cap,
                                  const uint8_t *msg, uint16_t n)
{
    if (out == NULL || msg == NULL) return 0;
    if (n == 0 || n > 0x1FFF) return 0;
    if ((uint16_t)(n + 2) > cap) return 0;
    out[0] = (uint8_t)(GOPRO_HDR_TYPE_E13 | ((n >> 8) & GOPRO_HDR_GEN_LEN));
    out[1] = (uint8_t)(n & 0xFF);
    memcpy(out + 2, msg, n);
    return (uint16_t)(n + 2);
}

/* ==========================================================================
 * Query responses
 *
 * A reassembled query message is [query id][command status][triples...], where
 * each triple is [element id][length][value]. Async pushes use the same shape
 * with query id 0x93 (status) or 0x92 (setting).
 * ========================================================================== */

#define GOPRO_Q_GET_STATUS      0x13
#define GOPRO_Q_REG_STATUS      0x53
#define GOPRO_Q_UNREG_STATUS    0x73
#define GOPRO_Q_PUSH_STATUS     0x93
#define GOPRO_Q_GET_SETTING     0x12
#define GOPRO_Q_REG_SETTING     0x52
#define GOPRO_Q_PUSH_SETTING    0x92

/* THE 2-BYTE FAMILY. The MISSION 1 and MISSION 1 Pro (and, one assumes, what
 * follows them) carry status ids as two bytes, big-endian, and do it under
 * their own query ids -- GoPro's Data Protocol page lists the pairs:
 *
 *     Get Status Value              0x13  ->  0x16
 *     Register for Status Updates   0x53  ->  0x56
 *     Notify Status Update          0x93  ->  0x96
 *
 * and says the results array is [id hi][id lo][len][value] in both the reply
 * and the push. The 1-byte ids are listed for the HERO9 through HERO13, MAX 2
 * and LIT HERO, and NOT for the MISSION 1 -- so a firmware that only spoke
 * the old family would connect to a MISSION 1, drive its shutter perfectly,
 * and never learn whether it was recording. Both families parse here; the
 * session chooses which to send by model and confirms it from the camera's
 * own reply, so a camera that refuses one is asked in the other. */
#define GOPRO_Q_GET_STATUS16    0x16
#define GOPRO_Q_REG_STATUS16    0x56
#define GOPRO_Q_UNREG_STATUS16  0x76
#define GOPRO_Q_PUSH_STATUS16   0x96

/* Is this query id a status reply or push we understand, and if so, are its
 * element ids two bytes wide? */
static inline bool gopro_qid_is_status(uint8_t qid, bool *wide)
{
    switch (qid) {
    case GOPRO_Q_GET_STATUS: case GOPRO_Q_REG_STATUS: case GOPRO_Q_PUSH_STATUS:
        if (wide) { *wide = false; }
        return true;
    case GOPRO_Q_GET_STATUS16: case GOPRO_Q_REG_STATUS16: case GOPRO_Q_PUSH_STATUS16:
        if (wide) { *wide = true; }
        return true;
    default:
        return false;
    }
}

/* Command status byte, shared by command, setting and query responses. */
#define GOPRO_ST_SUCCESS        0
#define GOPRO_ST_ERROR          1
#define GOPRO_ST_INVALID_PARAM  2

/* The statuses this firmware asks for. Numbering is one global namespace and
 * has not changed across HERO 9 through HERO 13, so these are safe to hold as
 * constants -- unlike the SETTING option ids for resolution, which are
 * model-dependent and deliberately not used here. */
#define GOPRO_ST_ID_OVERHEAT    6    /* bool                                  */
#define GOPRO_ST_ID_BUSY        8    /* bool                                  */
#define GOPRO_ST_ID_ENCODING   10    /* bool -- IS RECORDING                  */
#define GOPRO_ST_ID_CLIP_S     13    /* uint32 seconds                        */
#define GOPRO_ST_ID_STORAGE    33    /* enum, see below                       */
#define GOPRO_ST_ID_LEFT_S     35    /* uint32 seconds of video left          */
#define GOPRO_ST_ID_BATTERY    70    /* uint8 percent                         */
#define GOPRO_ST_ID_PRESET_GRP 96    /* uint32, see the group ids below       */

/* Preset groups. The shutter fires whatever the current preset does, so this
 * is what separates "recording" from "taking a photograph every few seconds".
 * Confirmed against GoPro's own SDK, which names status 96 PRESET_GROUP. */
#define GOPRO_PRESET_VIDEO      1000
#define GOPRO_PRESET_PHOTO      1001
#define GOPRO_PRESET_TIMELAPSE  1002

/* Status 33, "Primary Storage". Only OK means a recording can start. */
#define GOPRO_SD_UNKNOWN      (-1)
#define GOPRO_SD_OK             0
#define GOPRO_SD_FULL           1
#define GOPRO_SD_REMOVED        2
#define GOPRO_SD_FORMAT_ERROR   3
#define GOPRO_SD_BUSY           4
#define GOPRO_SD_SWAPPED        8

/* What we have learned from the camera so far.
 *
 * Every field carries its own _valid, exactly as cam_status_t does upstream,
 * and for the same reason: a value the camera has not sent is not a value. The
 * camera pushes only what changed, so these accumulate across notifications
 * rather than being rebuilt from each one. */
typedef struct {
    bool     encoding,  encoding_valid;
    uint32_t clip_s;    bool clip_valid;
    uint32_t left_s;    bool left_valid;
    uint8_t  battery;   bool battery_valid;
    int8_t   storage;   bool storage_valid;
    bool     overheat,  overheat_valid;
    bool     busy,      busy_valid;
    uint32_t preset_group; bool preset_valid;
} gopro_status_t;

/* Big-endian unsigned of 1, 2, 4 or 8 bytes, clamped into 32 bits.
 *
 * Widths come from the LENGTH BYTE of each triple, never from a table: the
 * specification gives only "integer" or "boolean" and says nothing about
 * width, and the widths the SDKs assume are not promised to be stable across
 * firmware. Reading the declared length is the only version-proof approach. */
static inline uint32_t gopro_be(const uint8_t *v, uint8_t len)
{
    uint32_t r = 0;
    if (len > 4) { v += (len - 4); len = 4; }   /* keep the low 32 bits */
    for (uint8_t i = 0; i < len; i++) r = (r << 8) | v[i];
    return r;
}

/* Fold one reassembled query message into the accumulated status.
 *
 * Returns the number of triples understood, or -1 if the message is not a
 * usable status response. Unknown element ids are SKIPPED, not treated as an
 * error: registering for statuses returns undocumented ids on real cameras,
 * and a parser that choked on them would fail on the very first reply. */
static inline int gopro_status_apply(gopro_status_t *st,
                                     const uint8_t *msg, uint16_t n)
{
    if (st == NULL || msg == NULL || n < 2) return -1;

    bool wide = false;
    if (!gopro_qid_is_status(msg[0], &wide)) return -1;
    if (msg[1] != GOPRO_ST_SUCCESS) return -1;

    /* One parser for both families: only the width of the id differs, and it
     * is fixed by the query id, never guessed from the bytes. */
    const uint16_t idw = wide ? 2 : 1;
    int applied = 0;
    uint16_t i = 2;
    while ((uint16_t)(i + idw) < n) {            /* id and length both present */
        uint16_t id  = wide ? (uint16_t)(((uint16_t)msg[i] << 8) | msg[i + 1])
                            : msg[i];
        uint8_t  len = msg[i + idw];
        i = (uint16_t)(i + idw + 1);
        if ((uint16_t)(i + len) > n) break;      /* truncated -- stop cleanly */

        /* A length of zero is legal and means "registered, no value yet".
         * Skipping it rather than recording a zero is the difference between
         * a blank OSD field and a confident lie. */
        if (len == 0) { continue; }

        const uint8_t *v = msg + i;
        switch (id) {
        /* Every read goes through gopro_be(), which honours the declared
         * length. Reading v[0] directly would be right only while the camera
         * chose to send one byte -- the specification promises no width, and
         * this file's own rule is to use the length byte. */
        case GOPRO_ST_ID_ENCODING:
            st->encoding = (gopro_be(v, len) != 0); st->encoding_valid = true; applied++; break;
        case GOPRO_ST_ID_CLIP_S:
            st->clip_s = gopro_be(v, len);  st->clip_valid = true;    applied++; break;
        case GOPRO_ST_ID_LEFT_S:
            st->left_s = gopro_be(v, len);  st->left_valid = true;    applied++; break;
        case GOPRO_ST_ID_BATTERY:
            st->battery = (uint8_t)gopro_be(v, len); st->battery_valid = true; applied++; break;
        case GOPRO_ST_ID_STORAGE:
            /* Read as SIGNED: the enum includes -1 for Unknown and the camera
             * delivers that as 0xFF in a one-byte field. */
            /* Signed: the enum includes -1 for Unknown, delivered as 0xFF in
             * a one-byte field. Read unsigned it would look like a valid
             * state. Taken from the LAST byte so a wider field still lands on
             * the value rather than on its padding. */
            st->storage = (int8_t)(gopro_be(v, len) & 0xFF);
            st->storage_valid = true; applied++; break;
        case GOPRO_ST_ID_OVERHEAT:
            st->overheat = (gopro_be(v, len) != 0); st->overheat_valid = true; applied++; break;
        case GOPRO_ST_ID_BUSY:
            st->busy = (gopro_be(v, len) != 0);     st->busy_valid = true;    applied++; break;
        case GOPRO_ST_ID_PRESET_GRP:
            st->preset_group = gopro_be(v, len);
            st->preset_valid = true;        applied++; break;
        default:
            break;                            /* undocumented id -- skip it */
        }
        i = (uint16_t)(i + len);
    }
    return applied;
}

/* ==========================================================================
 * Messages this firmware sends. TLV: [id][len][value]...
 * ========================================================================== */

/* Shutter, on the Command characteristic. */
#define GOPRO_CMD_SHUTTER      0x01
#define GOPRO_CMD_SLEEP        0x05
#define GOPRO_CMD_HILIGHT      0x18
#define GOPRO_CMD_HW_INFO      0x3C

/* Keep-alive is a SETTING write, not a command: setting 0x5B (the LED setting)
 * with the magic value 0x42, on the Settings characteristic. Both of GoPro's
 * own SDKs encode it exactly that way. Without it the camera sleeps. */
#define GOPRO_SET_KEEPALIVE    0x5B
#define GOPRO_KEEPALIVE_VALUE  0x42
#define GOPRO_KEEPALIVE_MS     3000

/* A setting change on a 2-byte-id camera: GoPro's Data Protocol page says
 * "instead of using the Setting ID as the type byte, a 0xFF marker byte is
 * used as the type, followed by [id hi][id lo][len][value]". */
#define GOPRO_SET_WIDE_MARKER  0xFF

/* The statuses this firmware subscribes to. One list, so the two families
 * cannot drift apart. The camera answers a registration with the current value
 * of each, which doubles as the initial read. */
static const uint8_t k_gopro_status_ids[] = {
    GOPRO_ST_ID_ENCODING, GOPRO_ST_ID_CLIP_S, GOPRO_ST_ID_LEFT_S,
    GOPRO_ST_ID_BATTERY,  GOPRO_ST_ID_STORAGE,
    GOPRO_ST_ID_OVERHEAT, GOPRO_ST_ID_BUSY, GOPRO_ST_ID_PRESET_GRP,
};
#define GOPRO_STATUS_ID_COUNT  ((uint16_t)(sizeof(k_gopro_status_ids)))

/* Build the registration message in either family. Returns the length, or 0
 * if it does not fit. The wide form is 17 bytes -- still one packet. */
static inline uint16_t gopro_status_reg_build(uint8_t *out, uint16_t cap, bool wide)
{
    uint16_t need = (uint16_t)(1 + GOPRO_STATUS_ID_COUNT * (wide ? 2 : 1));
    if (out == NULL || cap < need) return 0;
    uint16_t o = 0;
    out[o++] = wide ? GOPRO_Q_REG_STATUS16 : GOPRO_Q_REG_STATUS;
    for (uint16_t k = 0; k < GOPRO_STATUS_ID_COUNT; k++) {
        if (wide) out[o++] = 0x00;               /* every id we use is < 256 */
        out[o++] = k_gopro_status_ids[k];
    }
    return o;
}

/* Build the keep-alive in either form. */
static inline uint16_t gopro_keepalive_build(uint8_t *out, uint16_t cap, bool wide)
{
    if (out == NULL) return 0;
    if (wide) {
        if (cap < 5) return 0;
        out[0] = GOPRO_SET_WIDE_MARKER; out[1] = 0x00; out[2] = GOPRO_SET_KEEPALIVE;
        out[3] = 0x01; out[4] = GOPRO_KEEPALIVE_VALUE;
        return 5;
    }
    if (cap < 3) return 0;
    out[0] = GOPRO_SET_KEEPALIVE; out[1] = 0x01; out[2] = GOPRO_KEEPALIVE_VALUE;
    return 3;
}

/* Read a settings reply: [id][status], or [0xFF][id hi][id lo][status] for a
 * change sent in the wide form. False if too short to be either. */
static inline bool gopro_setting_reply(const uint8_t *msg, uint16_t n,
                                       uint16_t *id, uint8_t *status)
{
    if (msg == NULL || n < 2) return false;
    if (msg[0] == GOPRO_SET_WIDE_MARKER) {
        if (n < 4) return false;
        if (id)     *id = (uint16_t)(((uint16_t)msg[1] << 8) | msg[2]);
        if (status) *status = msg[3];
        return true;
    }
    if (id)     *id = msg[0];
    if (status) *status = msg[1];
    return true;
}

#endif /* GOPRO_PROTO_H */
