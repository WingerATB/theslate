/* SPDX-License-Identifier: PolyForm-Strict-1.0.0
 *
 * The bound-camera list, and the order it is tried in.
 *
 * A module used to hold exactly one camera. Anyone flying more than one -- a
 * Nano on the small quad and a 360 on the big one, or a spare in the bag --
 * had to open setup and re-pick every time they swapped, which is a laptop-era
 * answer to a question that comes up at a field.
 *
 * So the module holds a LIST, in priority order, and connects to the
 * highest-priority one that is actually switched on.
 *
 * WHEN THE CHOICE HAPPENS: at connect time, and only then. A camera further up
 * the list appearing later does NOT take the link away from one that is already
 * connected -- dropping a live camera mid-flight to switch to a better one
 * would end a recording to satisfy a preference. Priority decides who gets
 * picked up, never who gets put down.
 *
 * Pure, and separate from NVS, so the host tests exercise the shipped list
 * operations rather than a copy. Same rule as the switch window, the press
 * detector, the override, the setup guard and the warning ladder.
 */
#ifndef CAM_BIND_H
#define CAM_BIND_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "camvendor.h"

/* How many cameras a module may hold.
 *
 * Bounded by the time a sweep costs, not by storage: every camera that is
 * switched OFF has to be waited out before the next is tried, so the list
 * length is a delay budget for the worst case where the one you want is last.
 * Four is two aircraft plus a spare, and a sweep that still finishes in
 * seconds. */
#define CAM_BIND_MAX  4

/* Stored in NVS as a blob of these, in order. The struct is written to flash,
 * so its fields may only ever be APPENDED -- the same discipline camlink_cfg_t
 * documents, for the same reason: reordering it silently repoints every
 * shipped module at a different camera. */
/* Which protocol to speak to a bound camera.
 *
 * Stored in NVS as part of cam_bind_t, so this is APPEND ONLY -- renumbering
 * it would point every shipped module's bindings at the wrong protocol, which
 * presents as a camera that connects perfectly and then ignores every command.
 * The values are declared here rather than in duml_cam.h because the binding
 * carries them and the binding is what outlives any one session. */
typedef enum {
    CAM_PROTO_DUML  = 0,   /* Osmo Nano, Osmo Pocket                        */
    CAM_PROTO_RSDK  = 1,   /* Osmo Action series, Osmo 360                  */
    CAM_PROTO_GOPRO = 2,   /* Open GoPro, HERO 9 and newer                  */
} cam_proto_t;

/* Which make speaks a protocol. The binding stores the protocol rather than
 * the vendor, because the protocol is the thing that actually has to be got
 * right on the wire -- but the settings page wants to name the make, so the
 * one is derived from the other rather than stored twice and allowed to
 * disagree. */
static inline cam_vendor_t cam_vendor_of_proto(uint8_t proto)
{
    return (proto == CAM_PROTO_GOPRO) ? CAM_VENDOR_GOPRO : CAM_VENDOR_DJI;
}

typedef struct {
    uint8_t addr[6];
    uint8_t model;      /* advertised model code; 0 when unknown            */
    uint8_t proto;      /* cam_proto_t, resolved when the camera was picked */
    char    label[6];   /* NANO / O360 / A5 / GP12 -- for the OSD           */
} cam_bind_t;

typedef struct {
    uint8_t    n;
    cam_bind_t e[CAM_BIND_MAX];
} cam_bind_list_t;

static inline bool cam_addr_eq(const uint8_t a[6], const uint8_t b[6])
{
    return memcmp(a, b, 6) == 0;
}

/* Index of this address, or -1. */
static inline int cam_bind_find(const cam_bind_list_t *l, const uint8_t addr[6])
{
    for (int i = 0; i < (int)l->n; i++) {
        if (cam_addr_eq(l->e[i].addr, addr)) return i;
    }
    return -1;
}

/* Add, or refresh a camera already held.
 *
 * Re-binding one that is already in the list UPDATES IT IN PLACE and keeps its
 * position. Picking your Nano again to correct its label must not silently
 * demote it to last, and must not add a second entry for the same camera --
 * two entries for one MAC would make the second unreachable and the list one
 * shorter than it looks.
 *
 * Returns false only when the list is full and this is a new camera. */
static inline bool cam_bind_add(cam_bind_list_t *l, const cam_bind_t *c)
{
    int at = cam_bind_find(l, c->addr);
    if (at >= 0) { l->e[at] = *c; return true; }
    if (l->n >= CAM_BIND_MAX) return false;
    l->e[l->n++] = *c;
    return true;
}

/* Remove one. Everything below it moves up, so priorities stay 1..n with no
 * gap -- a list numbered 1, 2, 4 would be asking the user what happened to 3. */
static inline bool cam_bind_remove(cam_bind_list_t *l, const uint8_t addr[6])
{
    int at = cam_bind_find(l, addr);
    if (at < 0) return false;
    for (int i = at; i + 1 < (int)l->n; i++) l->e[i] = l->e[i + 1];
    l->n--;
    return true;
}

/* Reorder to match a list of addresses, top priority first.
 *
 * Addressed BY MAC rather than by index on purpose. The settings page sends
 * back an order it built from a list it fetched, and between those two moments
 * the list can change -- another tab forgets a camera, or the page was left
 * open. Indices would then silently reorder the wrong entries; addresses
 * either match something or they do not.
 *
 * Unknown addresses are ignored, and anything the caller did not mention keeps
 * its relative order at the end rather than being dropped. A partial or stale
 * order is therefore incomplete, never destructive.
 *
 * Returns how many were placed. */
static inline int cam_bind_reorder(cam_bind_list_t *l,
                                   const uint8_t (*order)[6], int n)
{
    cam_bind_list_t out = {0};
    bool taken[CAM_BIND_MAX] = {false};

    for (int i = 0; i < n; i++) {
        int at = cam_bind_find(l, order[i]);
        if (at < 0 || taken[at]) continue;   /* unknown, or named twice */
        taken[at] = true;
        out.e[out.n++] = l->e[at];
    }
    int placed = (int)out.n;

    for (int i = 0; i < (int)l->n; i++) {
        if (!taken[i]) out.e[out.n++] = l->e[i];
    }

    *l = out;
    return placed;
}

#endif /* CAM_BIND_H */
