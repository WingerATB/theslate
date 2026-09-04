/* SPDX-License-Identifier: PolyForm-Strict-1.0.0
 *
 * GoPro camera session over BLE, per the Open GoPro specification:
 * connect, bond, subscribe, keep alive, track status, control the shutter.
 *
 * Bring-up target is a HERO (2024), which Open GoPro does not list as
 * supported. Everything here is therefore written to SAY what the camera
 * answered rather than assume it: every command's response status is logged,
 * and a status registration the camera refuses falls back to asking for
 * everything and reading what arrives.
 */
#ifndef CAMLINK_GOPRO_CAM_H
#define CAMLINK_GOPRO_CAM_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef struct {
    bool     link_up;        /* connected, bonded, subscribed, keep-alive running */
    bool     status_valid;   /* at least one status value has arrived this link  */
    uint32_t status_age_ms;  /* since the last status push, UINT32_MAX if never   */

    bool     encoding;       /* status 10                                         */
    uint32_t clip_s;         /* status 13, video encoding duration                */
    uint32_t left_s;         /* status 35, remaining video time                   */
    uint8_t  battery_pct;    /* status 70                                         */
    bool     battery_valid;
    uint8_t  hot;            /* status 6, overheating                             */
    bool     busy;           /* status 8                                          */
    bool     sd_ok;          /* status 33 == 0 (OK)                               */

    char     res[8];         /* video resolution label ("4K", "1080"), "" unknown */
    char     fps[8];         /* framerate label ("60", "30"), "" unknown          */
    char     model[24];      /* from Get Hardware Info, "" until it answers       */
    char     label[6];       /* four-character OSD label derived from model       */
} gopro_status_t;

/* Which characteristic a raw write goes to. */
typedef enum { GOPRO_CH_CMD = 0, GOPRO_CH_SET = 1, GOPRO_CH_QRY = 2 } gopro_ch_t;

/* Bring up BLE and run the session against this camera, forever. Must be the
 * only thing that owns the BLE controller in this boot. addr_type is what the
 * scanner saw: GoPros use static random addresses, and a connection request
 * with the wrong type is simply never answered. */
void gopro_cam_start(const uint8_t addr[6], uint8_t addr_type);

void gopro_cam_get(gopro_status_t *out);

/* Set Shutter. Executed by the session task on its next pass. */
void gopro_cam_request_record(bool on);

/* Put the camera to sleep. It stays connectable; reconnecting wakes it. */
void gopro_cam_sleep(void);

/* Tag a HiLight moment (command 0x18). Meaningful while recording. */
void gopro_cam_hilight(void);

/* Ask the camera to reconnect now (e.g. after sleep, to wake it). */
void gopro_cam_wake(void);

/* Drop the link politely and stop reconnecting. Called before a reboot: a
 * link that just vanishes leaves the camera waiting out its supervision
 * timeout, during which it neither advertises nor accepts a connection --
 * which is exactly the window the next boot wants to find it in. */
void gopro_cam_disconnect(void);

/* Raw packet to one of the three write characteristics, bytes as they appear
 * in the Open GoPro tables, header included. Bench tool. Returns false if not
 * connected. */
bool gopro_cam_write(gopro_ch_t ch, const uint8_t *bytes, size_t n);

#endif
