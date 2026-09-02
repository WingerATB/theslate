/* SPDX-License-Identifier: PolyForm-Strict-1.0.0 */
#include <string.h>

#include "camlink_cfg.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "sdkconfig.h"

static const char *TAG = "CFG";

#define NVS_NS  "camlink"
#define NVS_KEY "cfg"

#define MAX_RC_CHANNEL   17
#define MIN_THRESHOLD  CAMLINK_RC_US_MIN
#define MAX_THRESHOLD  CAMLINK_RC_US_MAX

static camlink_cfg_t     s_cfg;
static SemaphoreHandle_t s_lock;

/* The shipped layout: state and the liveness dot on the first row, then clip
 * time, battery and card, one per row. */
static const uint8_t k_default_osd[OSD_ROWS][OSD_ROW_FIELDS] = {
    { OSD_F_STATE,   OSD_F_DOT,  OSD_F_NONE, OSD_F_NONE },
    { OSD_F_CLIP,    OSD_F_NONE, OSD_F_NONE, OSD_F_NONE },
    { OSD_F_BATTERY, OSD_F_NONE, OSD_F_NONE, OSD_F_NONE },
    { OSD_F_CARD,    OSD_F_NONE, OSD_F_NONE, OSD_F_NONE },
};

void camlink_cfg_default_osd(uint8_t out[OSD_ROWS][OSD_ROW_FIELDS])
{
    if (out) memcpy(out, k_default_osd, sizeof(k_default_osd));
}

/* Is this layout renderable? Every id known, every row inside the budget.
 *
 * An all-empty layout is legal: someone who wants no OSD at all is entitled to
 * it, and the renderer emits a single space per row so Betaflight still does
 * not fall back to printing "CUSTOM_MSG1". */
static bool osd_valid(const uint8_t osd[OSD_ROWS][OSD_ROW_FIELDS])
{
    for (int r = 0; r < OSD_ROWS; r++) {
        for (int i = 0; i < OSD_ROW_FIELDS; i++) {
            if (osd[r][i] >= OSD_F__COUNT) return false;
        }
        if (!osd_row_fits(osd[r])) return false;
    }
    return true;
}

static void clamp(camlink_cfg_t *c)
{
    if (c->mode >= CFG_MODE__COUNT) {
        c->mode = CFG_MODE_CUT;
    }
    if (c->switch_channel > MAX_RC_CHANNEL) {
        c->switch_channel = CONFIG_CAMLINK_SWITCH_CHANNEL;
    }
    if (c->switch_kind > CFG_SW_BUTTON) {
        c->switch_kind = CFG_SW_LEVEL;
    }
    if (c->tx_auto > 1) {
        c->tx_auto = 0;
    }
    if (c->cfg_channel > MAX_RC_CHANNEL) {
        c->cfg_channel = 0;      /* unusable index -> no setup switch at all */
    }
    if (c->cfg_kind > CFG_SW_BUTTON) {
        c->cfg_kind = CFG_SW_LEVEL;
    }
    if (c->cfg_hold_ds > CAMLINK_CFG_HOLD_DS_MAX) {
        c->cfg_hold_ds = CAMLINK_CFG_HOLD_DS_DEFAULT;
    }
    if (c->tx_power >= CFG_TX__COUNT) {
        c->tx_power = CFG_TX_N24;   /* unknown value -> the quiet, flight-safe one */
    }
    /* A layout that cannot be rendered is replaced whole rather than patched.
     * Patching would leave the user with a layout they never chose and cannot
     * tell apart from the one they did. */
    if (!osd_valid((const uint8_t (*)[OSD_ROW_FIELDS])c->osd)) {
        memcpy(c->osd, k_default_osd, sizeof(k_default_osd));
    }
    if (c->range_min < MIN_THRESHOLD || c->range_min > MAX_THRESHOLD ||
        c->range_max < MIN_THRESHOLD || c->range_max > MAX_THRESHOLD ||
        c->range_min >= c->range_max) {
        /* An inverted or out-of-range window would never match, silently
         * disabling the trigger. Fall back to "upper half". */
        c->range_min = CONFIG_CAMLINK_SWITCH_THRESHOLD;
        c->range_max = MAX_THRESHOLD;
    }
}

static void save(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_blob(h, NVS_KEY, &s_cfg, sizeof(s_cfg));
    nvs_commit(h);
    nvs_close(h);
}

void camlink_cfg_init(void)
{
    s_lock = xSemaphoreCreateMutex();

    /* Kconfig values are first-boot defaults only. */
#if   defined(CONFIG_CAMLINK_MODE_ARM)
    s_cfg.mode = CFG_MODE_ARM;
#elif defined(CONFIG_CAMLINK_MODE_SWITCH)
    s_cfg.mode = CFG_MODE_SWITCH;
#else
    s_cfg.mode = CFG_MODE_CUT;
#endif
    s_cfg.switch_channel = CONFIG_CAMLINK_SWITCH_CHANNEL;
    s_cfg.range_min      = CONFIG_CAMLINK_SWITCH_THRESHOLD;
    s_cfg.range_max      = MAX_THRESHOLD;
    s_cfg.cfg_ver        = CAMLINK_CFG_VER;
    s_cfg.cfg_hold_ds    = CAMLINK_CFG_HOLD_DS_DEFAULT;
    memcpy(s_cfg.osd, k_default_osd, sizeof(k_default_osd));
#if   defined(CONFIG_CAMLINK_TX_N12)
    s_cfg.tx_power       = CFG_TX_N12;
#elif defined(CONFIG_CAMLINK_TX_N0)
    s_cfg.tx_power       = CFG_TX_N0;
#elif defined(CONFIG_CAMLINK_TX_P3)
    s_cfg.tx_power       = CFG_TX_P3;
#else
    s_cfg.tx_power       = CFG_TX_N24;
#endif

    bool loaded = false;
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        /* Copy however much was stored over the defaults, rather than demanding
         * an exact size match. A blob written by an older firmware is SHORTER,
         * not corrupt: the fields it does carry are still the user's settings,
         * and the ones it predates keep their defaults. Requiring equality here
         * would silently reset every setting on the update that adds one. */
        uint8_t stored[sizeof(camlink_cfg_t)];
        size_t  len = sizeof(stored);
        if (nvs_get_blob(h, NVS_KEY, stored, &len) == ESP_OK && len > 0 &&
            len <= sizeof(camlink_cfg_t)) {
            memcpy(&s_cfg, stored, len);
            if (len < sizeof(camlink_cfg_t)) {
                ESP_LOGI(TAG, "migrated %u-byte config blob to %u bytes",
                         (unsigned)len, (unsigned)sizeof(camlink_cfg_t));
            }
            loaded = true;
        }
        nvs_close(h);
    }

    /* A blob written before the scale widened stored the user's intent against
     * a window that could not go above 2000 us. "As high as it goes" was the
     * only thing 2000 could mean there, and leaving it at the literal number
     * would leave every existing unit unable to see a CRSF switch at 2012 --
     * the setting would look right and do nothing. Carry the intent across
     * rather than the digits, once, and write it back so this runs one time. */
    if (loaded && s_cfg.cfg_ver < CAMLINK_CFG_VER) {
        uint16_t was_min = s_cfg.range_min, was_max = s_cfg.range_max;
        if (s_cfg.range_max >= 2000) s_cfg.range_max = MAX_THRESHOLD;
        if (s_cfg.range_min <= 1000) s_cfg.range_min = MIN_THRESHOLD;
        s_cfg.cfg_ver = CAMLINK_CFG_VER;
        ESP_LOGW(TAG, "widened stored window %u-%u -> %u-%u for the real RC scale",
                 was_min, was_max, s_cfg.range_min, s_cfg.range_max);
        clamp(&s_cfg);
        save();
    }
    clamp(&s_cfg);

    ESP_LOGI(TAG, "mode=%s AUX%d (idx %u) range %u-%u tx=%s",
             camlink_cfg_mode_name(s_cfg.mode),
             cfg_index_to_aux(s_cfg.switch_channel), s_cfg.switch_channel,
             s_cfg.range_min, s_cfg.range_max, camlink_cfg_tx_name(s_cfg.tx_power));
}

void camlink_cfg_get(camlink_cfg_t *out)
{
    if (out == NULL) return;
    if (s_lock && xSemaphoreTake(s_lock, pdMS_TO_TICKS(20)) == pdTRUE) {
        *out = s_cfg;
        xSemaphoreGive(s_lock);
    } else {
        *out = s_cfg;
    }
    clamp(out);
}

static bool commit(void)
{
    clamp(&s_cfg);
    /* Anything we write out is current-schema by definition. Stamped here
     * rather than in clamp(), so the marker can only ever be set by an actual
     * write -- a clamp that silently claimed the new schema would let a stored
     * blob skip the migration above. */
    s_cfg.cfg_ver = CAMLINK_CFG_VER;
    save();
    ESP_LOGI(TAG, "mode=%s AUX%d (idx %u) range %u-%u tx=%s",
             camlink_cfg_mode_name(s_cfg.mode),
             cfg_index_to_aux(s_cfg.switch_channel), s_cfg.switch_channel,
             s_cfg.range_min, s_cfg.range_max, camlink_cfg_tx_name(s_cfg.tx_power));
    return true;
}

bool camlink_cfg_set_mode(uint8_t mode)
{
    if (mode >= CFG_MODE__COUNT) return false;
    if (s_lock) xSemaphoreTake(s_lock, portMAX_DELAY);
    s_cfg.mode = mode;
    bool ok = commit();
    if (s_lock) xSemaphoreGive(s_lock);
    return ok;
}

bool camlink_cfg_set_channel(uint8_t ch)
{
    if (ch > MAX_RC_CHANNEL) return false;
    if (s_lock) xSemaphoreTake(s_lock, portMAX_DELAY);
    s_cfg.switch_channel = ch;
    bool ok = commit();
    if (s_lock) xSemaphoreGive(s_lock);
    return ok;
}

bool camlink_cfg_set_range(uint16_t lo, uint16_t hi)
{
    if (lo < MIN_THRESHOLD || hi > MAX_THRESHOLD || lo >= hi) return false;
    if (s_lock) xSemaphoreTake(s_lock, portMAX_DELAY);
    s_cfg.range_min = lo;
    s_cfg.range_max = hi;
    bool ok = commit();
    if (s_lock) xSemaphoreGive(s_lock);
    return ok;
}

bool camlink_cfg_set_osd(const uint8_t osd[OSD_ROWS][OSD_ROW_FIELDS])
{
    if (osd == NULL || !osd_valid(osd)) return false;
    if (s_lock) xSemaphoreTake(s_lock, portMAX_DELAY);
    memcpy(s_cfg.osd, osd, sizeof(s_cfg.osd));
    bool ok = commit();
    if (s_lock) xSemaphoreGive(s_lock);
    return ok;
}

bool camlink_cfg_set_switch_kind(uint8_t kind)
{
    if (kind > CFG_SW_BUTTON) return false;
    if (s_lock) xSemaphoreTake(s_lock, portMAX_DELAY);
    s_cfg.switch_kind = kind;
    bool ok = commit();
    if (s_lock) xSemaphoreGive(s_lock);
    return ok;
}

bool camlink_cfg_set_config_channel(uint8_t ch)
{
    if (ch > MAX_RC_CHANNEL) return false;
    if (s_lock) xSemaphoreTake(s_lock, portMAX_DELAY);
    s_cfg.cfg_channel = ch;
    bool ok = commit();
    if (s_lock) xSemaphoreGive(s_lock);
    return ok;
}

bool camlink_cfg_set_config_kind(uint8_t kind)
{
    if (kind > CFG_SW_BUTTON) return false;
    if (s_lock) xSemaphoreTake(s_lock, portMAX_DELAY);
    s_cfg.cfg_kind = kind;
    bool ok = commit();
    if (s_lock) xSemaphoreGive(s_lock);
    return ok;
}

bool camlink_cfg_set_config_hold(uint8_t ds)
{
    if (ds > CAMLINK_CFG_HOLD_DS_MAX) return false;
    if (s_lock) xSemaphoreTake(s_lock, portMAX_DELAY);
    s_cfg.cfg_hold_ds = ds;
    bool ok = commit();
    if (s_lock) xSemaphoreGive(s_lock);
    return ok;
}

bool camlink_cfg_set_tx_auto(uint8_t on)
{
    if (on > 1) return false;
    if (s_lock) xSemaphoreTake(s_lock, portMAX_DELAY);
    s_cfg.tx_auto = on;
    bool ok = commit();
    if (s_lock) xSemaphoreGive(s_lock);
    return ok;
}

bool camlink_cfg_set_tx_power(uint8_t tx)
{
    if (tx >= CFG_TX__COUNT) return false;
    if (s_lock) xSemaphoreTake(s_lock, portMAX_DELAY);
    s_cfg.tx_power = tx;
    bool ok = commit();
    if (s_lock) xSemaphoreGive(s_lock);
    return ok;
}

const char *camlink_cfg_tx_name(uint8_t tx)
{
    switch (tx) {
    case CFG_TX_N24: return "-24 dBm";
    case CFG_TX_N12: return "-12 dBm";
    case CFG_TX_N0:  return "0 dBm";
    case CFG_TX_P3:  return "+3 dBm";
    default:         return "?";
    }
}

const char *camlink_cfg_switch_kind_name(uint8_t kind)
{
    return kind == CFG_SW_BUTTON ? "BUTTON" : "SWITCH";
}

const char *camlink_cfg_mode_name(uint8_t mode)
{
    switch (mode) {
    case CFG_MODE_ARM:    return "ARM";
    case CFG_MODE_SWITCH: return "SWITCH";
    case CFG_MODE_CUT:    return "CUT";
    case CFG_MODE_BOTH:   return "BOTH";
    default:              return "?";
    }
}
