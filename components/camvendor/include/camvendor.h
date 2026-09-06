/* SPDX-License-Identifier: PolyForm-Strict-1.0.0
 *
 * Which vendor a camera is, and what its GATT layout looks like.
 *
 * WHY THIS IS ITS OWN COMPONENT. The BLE transport in components/dji_link/ has
 * to recognise a GoPro advertisement and discover GoPro characteristics, and
 * the GoPro session has to speak the protocol. If the transport depended on the
 * GoPro component and the GoPro component depended on the transport, that is a
 * build cycle. So the vendor-facing FACTS -- how an advert identifies itself,
 * which UUIDs a vendor uses -- live here, in a leaf that depends on nothing in
 * this tree, and both sides depend on it.
 *
 * It also keeps GoPro constants out of components/dji_link/, which is vendored
 * MIT code tracked by NOTICE and PATCHES.md.
 */
#ifndef CAMVENDOR_H
#define CAMVENDOR_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

typedef enum {
    CAM_VENDOR_NONE = 0,   /* not a camera we know                          */
    CAM_VENDOR_DJI,
    CAM_VENDOR_GOPRO,
} cam_vendor_t;

/* ==========================================================================
 * GoPro advertisements
 *
 * A GoPro advertises the SIG-assigned 16-bit service 0xFEA6 and, separately,
 * manufacturer data under company id 0xF202 that carries a status bitfield and
 * the model id. Both are parsed here, from a raw AD blob, so the host tests can
 * exercise the matcher without a radio.
 * ========================================================================== */

#define GOPRO_SERVICE_UUID16   0xFEA6
/* GoPro's SIG company identifier, recorded for reference. Deliberately NOT
 * used as a filter: the service UUID above is the identifier this project can
 * verify, and a second unverified one could only ever reject a camera the
 * first had already accepted. */
#define GOPRO_COMPANY_ID       0xF202

/* Model ids, from Get Hardware Info and from advertisement byte 13. Checked
 * against GoPro's own firmware table (HERO9 55 ... MISSION 1 71).
 *
 * The floor matters: HERO 8 and older advertise the same service and cannot
 * speak this protocol at all, so binding one produces a camera that sits in
 * the user's list and never connects. HERO 9 is where Open GoPro begins.
 *
 * Two models inside the range are refused by name. The HERO (2024), model 66,
 * is not in GoPro's Open GoPro compatibility table -- GoPro's own answer on
 * their tracker was "currently unsupported ... support will be added in a
 * later firmware", and the table still omits it. The LIT HERO, model 70, is
 * not supported by this project. Both are named in the picker so the user
 * knows what they are looking at, and refused at bind with the reason. */
#define GOPRO_MODEL_HERO9        55
#define GOPRO_MODEL_HERO10       57
#define GOPRO_MODEL_HERO11       58
#define GOPRO_MODEL_HERO11_MINI  60
#define GOPRO_MODEL_HERO12       62
#define GOPRO_MODEL_MAX2         64
#define GOPRO_MODEL_HERO13       65
#define GOPRO_MODEL_HERO2024     66   /* "HERO" (2024), firmware H24.03    */
#define GOPRO_MODEL_MISSION1PRO  69
#define GOPRO_MODEL_LITHERO      70
#define GOPRO_MODEL_MISSION1     71

/* The oldest model that can work. Anything below this is refused at bind time
 * rather than allowed to fail silently at connect. Unknown ids ABOVE it are
 * allowed, so a HERO that does not exist yet still binds. */
#define GOPRO_MODEL_MIN          GOPRO_MODEL_HERO9

/* Walk AD structures. Returns the data pointer for `want_type` and writes its
 * length, or NULL. Shared by every parser below. */
static inline const uint8_t *cam_adv_field(const uint8_t *adv, uint8_t adv_len,
                                           uint8_t want_type, uint8_t *out_len)
{
    if (adv == NULL) return NULL;
    for (uint8_t i = 0; i + 1 < adv_len; ) {
        uint8_t len = adv[i];
        if (len == 0 || (uint16_t)(i + len + 1) > adv_len) break;
        uint8_t type = adv[i + 1];
        if (type == want_type) {
            if (out_len) *out_len = (uint8_t)(len - 1);
            return &adv[i + 2];
        }
        i = (uint8_t)(i + len + 1);
    }
    return NULL;
}

/* Does this advertisement carry the GoPro service UUID?
 *
 * Checked in both the complete (0x03) and incomplete (0x02) 16-bit service
 * lists, little-endian, because which one a camera uses is not something to
 * bet a scan filter on. */
static inline bool cam_adv_is_gopro(const uint8_t *adv, uint8_t adv_len)
{
    const uint8_t types[2] = { 0x02, 0x03 };
    for (int t = 0; t < 2; t++) {
        uint8_t n = 0;
        const uint8_t *d = cam_adv_field(adv, adv_len, types[t], &n);
        if (d == NULL) continue;
        for (uint8_t i = 0; i + 1 < n; i += 2) {
            uint16_t u = (uint16_t)(d[i] | ((uint16_t)d[i + 1] << 8));
            if (u == GOPRO_SERVICE_UUID16) return true;
        }
    }
    return false;
}

/* GoPro's manufacturer-specific data, counted from the start of the AD
 * structure's payload -- which is where cam_adv_field() points, and which
 * INCLUDES the two company-id bytes:
 *
 *   0-1  company id
 *   2    schema version
 *   3    camera status   (bit 0 = awake)
 *   4    model id
 *   5    capabilities
 *
 * Confirmed against GoPro's own Kotlin SDK, whose parser reads schema at [0],
 * status at [1] and cameraId at [2] -- it is handed the payload with the
 * company id already stripped by the phone's BLE API, which ESP-IDF does not
 * do. Getting that difference wrong shifts everything by two, and reading the
 * table as absolute offsets into the advert shifts it by seven; both land on
 * bytes that carry a serial number and look like plausible rubbish.
 *
 * The company id is deliberately NOT checked. cam_adv_is_gopro() has already
 * established the vendor from the service UUID, which is SIG-assigned and
 * unambiguous, so demanding a second identifier here could only ever reject a
 * camera we have already identified. */
#define GOPRO_ADV_SCHEMA_OFFSET  2
#define GOPRO_ADV_STATUS_OFFSET  3
#define GOPRO_ADV_MODEL_OFFSET   4

/* The model id, or 0 when it cannot be read. Zero means "unknown", which the
 * caller must treat as "not a model we can vouch for" rather than as a model
 * number. */
static inline uint8_t cam_adv_gopro_model(const uint8_t *adv, uint8_t adv_len)
{
    /* Guarded on the service UUID rather than the company id.
     *
     * The manufacturer field of a DJI advert is perfectly well-formed and its
     * fifth byte is a number -- it just is not a GoPro model. Something has to
     * establish the vendor before these offsets mean anything, and the SIG
     * service UUID is the identifier this project can actually verify. */
    if (!cam_adv_is_gopro(adv, adv_len)) return 0;

    uint8_t n = 0;
    const uint8_t *d = cam_adv_field(adv, adv_len, 0xFF, &n);   /* mfg data */
    if (d == NULL || n <= GOPRO_ADV_MODEL_OFFSET) return 0;
    return d[GOPRO_ADV_MODEL_OFFSET];
}

/* Is the camera awake, as far as its advertisement admits?
 *
 * The low bit of the status byte is the processor state. `*known` says whether
 * the question could be answered at all -- an advert without manufacturer data
 * tells us nothing, and "I cannot tell" must never read as "asleep", because
 * that would demote a camera sitting there switched on. */
#define GOPRO_ADV_STATUS_AWAKE   0x01

static inline bool cam_adv_gopro_awake(const uint8_t *adv, uint8_t adv_len,
                                       bool *known)
{
    if (known) *known = false;
    if (!cam_adv_is_gopro(adv, adv_len)) return false;   /* see the note above */
    uint8_t n = 0;
    const uint8_t *d = cam_adv_field(adv, adv_len, 0xFF, &n);
    if (d == NULL || n <= GOPRO_ADV_STATUS_OFFSET) return false;
    if (known) *known = true;
    return (d[GOPRO_ADV_STATUS_OFFSET] & GOPRO_ADV_STATUS_AWAKE) != 0;
}

/* May this GoPro be bound?
 *
 * Refused below the HERO 9 floor, refused for the two models this project does
 * not support, and refused when the model could not be read at all -- an advert
 * we cannot identify is not one to promise the user works. The reason string is
 * written for somebody holding a camera, because the settings page shows it to
 * them verbatim; it says what is wrong without turning into a list. */
static inline bool cam_gopro_model_supported(uint8_t model, const char **why)
{
    if (model == 0) {
        if (why) *why = "this GoPro did not say which model it is, so it cannot be added";
        return false;
    }
    if (model < GOPRO_MODEL_MIN) {
        if (why) *why = "this GoPro is older than a HERO 9 and cannot be controlled over Bluetooth";
        return false;
    }
    if (model == GOPRO_MODEL_HERO2024 || model == GOPRO_MODEL_LITHERO) {
        if (why) *why = "this GoPro model is not supported";
        return false;
    }
    return true;
}

/* Does this model carry setting and status ids as two bytes?
 *
 * GoPro's Data Protocol page: cameras that report supports_2byte_ids use a
 * separate family of query ids with 2-byte element ids, and their operation
 * table lists that family for the MISSION 1 and MISSION 1 Pro only -- and
 * lists the 1-byte family for everything older and NOT for them. A model newer
 * than the newest we know is assumed to follow the MISSION 1, which is GoPro's
 * direction of travel; either way the session confirms the choice from the
 * camera's own reply and switches if refused, so this is a first guess rather
 * than a verdict. */
static inline bool cam_gopro_wide_ids(uint8_t model)
{
    return model == GOPRO_MODEL_MISSION1 || model == GOPRO_MODEL_MISSION1PRO ||
           model > GOPRO_MODEL_MISSION1;
}

/* Four characters for the OSD, matching what the DJI side does. */
static inline void cam_gopro_label(uint8_t model, char out[6])
{
    const char *s;
    switch (model) {
    case GOPRO_MODEL_HERO9:       s = "GP9";  break;
    case GOPRO_MODEL_HERO10:      s = "GP10"; break;
    case GOPRO_MODEL_HERO11:      s = "GP11"; break;
    case GOPRO_MODEL_HERO11_MINI: s = "GP11"; break;
    case GOPRO_MODEL_HERO12:      s = "GP12"; break;
    case GOPRO_MODEL_HERO13:      s = "GP13"; break;
    case GOPRO_MODEL_MAX2:        s = "MAX2"; break;
    case GOPRO_MODEL_MISSION1:
    case GOPRO_MODEL_MISSION1PRO: s = "MSN";  break;
    default:                      s = "GPRO"; break;
    }
    size_t i = 0;
    for (; s[i] && i < 5; i++) out[i] = s[i];
    out[i] = '\0';
}

/* Human-readable, for the settings page's camera list. */
static inline const char *cam_gopro_model_name(uint8_t model)
{
    switch (model) {
    case GOPRO_MODEL_HERO9:       return "GoPro HERO9 Black";
    case GOPRO_MODEL_HERO10:      return "GoPro HERO10 Black";
    case GOPRO_MODEL_HERO11:      return "GoPro HERO11 Black";
    case GOPRO_MODEL_HERO11_MINI: return "GoPro HERO11 Black Mini";
    case GOPRO_MODEL_HERO12:      return "GoPro HERO12 Black";
    case GOPRO_MODEL_MAX2:        return "GoPro MAX 2";
    case GOPRO_MODEL_HERO13:      return "GoPro HERO13 Black";
    case GOPRO_MODEL_HERO2024:    return "GoPro HERO (2024)";
    case GOPRO_MODEL_MISSION1PRO: return "GoPro Mission 1 Pro";
    case GOPRO_MODEL_LITHERO:     return "GoPro LIT HERO";
    case GOPRO_MODEL_MISSION1:    return "GoPro Mission 1";
    default:                      return "GoPro";
    }
}

#endif /* CAMVENDOR_H */
