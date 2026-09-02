/* SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 *
 * MSP client implementation. See msp.h for the threading contract.
 */
#include <string.h>
#include "msp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "MSP";

#define MSP_RX_BUF_SIZE     512
#define MSP_TX_BUF_SIZE     512
#define MSP_MAX_PAYLOAD     255

#define POLL_PERIOD_MS      100   /* 10 Hz status/RC poll                     */
#define BOXIDS_PERIOD_MS  10000   /* re-query so an FC reboot recovers        */

static msp_config_t     s_cfg;
static msp_state_t      s_state;
static SemaphoreHandle_t s_lock;
/* Separate TX mutex. Two tasks transmit on this UART: msp_task sends the
 * STATUS/RC/BOXIDS polls, and the logic task sends OSD text. uart_write_bytes()
 * is not safe against concurrent callers on the same port -- without this the
 * two frames interleave mid-byte-stream and both are corrupted. Kept distinct
 * from s_lock so a transmit never waits on the parser. */
static SemaphoreHandle_t s_tx_lock;
static bool             s_inited;

/* ------------------------------------------------------------------------- */
/* Checksums                                                                  */
/* ------------------------------------------------------------------------- */

/* DVB-S2, polynomial 0xD5. Used by MSP v2 over the whole header-after-'<'
 * through end-of-payload. */
uint8_t msp_crc8_dvb_s2(uint8_t crc, uint8_t a)
{
    crc ^= a;
    for (int i = 0; i < 8; i++) {
        crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
    }
    return crc;
}

/* ------------------------------------------------------------------------- */
/* Transmit                                                                   */
/* ------------------------------------------------------------------------- */

/* Single serialisation point for every byte we put on the wire. */
static void msp_uart_write(const uint8_t *buf, size_t n)
{
    if (s_tx_lock == NULL) return;
    if (xSemaphoreTake(s_tx_lock, pdMS_TO_TICKS(50)) != pdTRUE) {
        /* Never block the caller indefinitely on a busy UART -- dropping a
         * poll or an OSD refresh is harmless, stalling a task is not. */
        return;
    }
    uart_write_bytes(s_cfg.uart_num, (const char *)buf, n);
    xSemaphoreGive(s_tx_lock);
}

static void msp_send_v1(uint8_t cmd, const uint8_t *payload, uint8_t len)
{
    uint8_t buf[6 + MSP_MAX_PAYLOAD];
    size_t n = 0;
    buf[n++] = '$';
    buf[n++] = 'M';
    buf[n++] = '<';
    buf[n++] = len;
    buf[n++] = cmd;
    uint8_t ck = (uint8_t)(len ^ cmd);
    for (uint8_t i = 0; i < len; i++) {
        buf[n++] = payload[i];
        ck ^= payload[i];
    }
    buf[n++] = ck;
    msp_uart_write(buf, n);
}

static void msp_send_v2(uint16_t function, const uint8_t *payload, uint16_t len)
{
    uint8_t buf[9 + MSP_MAX_PAYLOAD];
    size_t n = 0;
    buf[n++] = '$';
    buf[n++] = 'X';
    buf[n++] = '<';

    size_t crc_start = n;
    buf[n++] = 0;                              /* flag */
    buf[n++] = (uint8_t)(function & 0xFF);     /* function, little endian */
    buf[n++] = (uint8_t)(function >> 8);
    buf[n++] = (uint8_t)(len & 0xFF);          /* payload len, little endian */
    buf[n++] = (uint8_t)(len >> 8);
    for (uint16_t i = 0; i < len; i++) {
        buf[n++] = payload[i];
    }

    /* CRC8/DVB-S2 over flag .. end of payload */
    uint8_t crc = 0;
    for (size_t i = crc_start; i < n; i++) {
        crc = msp_crc8_dvb_s2(crc, buf[i]);
    }
    buf[n++] = crc;

    msp_uart_write(buf, n);
}

esp_err_t msp_set_osd_text(uint8_t slot, const char *text)
{
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    /* Guard the slot: id 11 is MSP2TEXT_BATTERY_PROFILE_NAME upstream, so an
     * out-of-range slot would silently overwrite unrelated FC config. */
    if (slot >= MSP_CUSTOM_MSG_COUNT || text == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t len = strlen(text);
    if (len > MSP_TEXT_MAX_LEN) {
        len = MSP_TEXT_MAX_LEN;
    }

    /* Payload is [type][len][chars] -- confirmed against Betaflight msp.c,
     * which does sbufReadU8(type), then MIN(space, sbufReadU8(len)). */
    uint8_t payload[2 + MSP_TEXT_MAX_LEN];
    payload[0] = (uint8_t)(MSP2TEXT_CUSTOM_MSG_0 + slot);
    payload[1] = (uint8_t)len;
    memcpy(&payload[2], text, len);

    msp_send_v2(MSP2_SET_TEXT, payload, (uint16_t)(2 + len));
    s_state.osd_writes++;
    return ESP_OK;
}

/* ------------------------------------------------------------------------- */
/* Reply handling                                                             */
/* ------------------------------------------------------------------------- */

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static void handle_status(const uint8_t *p, uint16_t len)
{
    /* cycleTime u16 | i2cErrors u16 | sensors u16 | flightModeFlags u32
     * | currentPidProfile u8 | ... (more follows, ignored) */
    if (len < 11) {
        return;
    }
    uint32_t flags = (uint32_t)p[6] | ((uint32_t)p[7] << 8) |
                     ((uint32_t)p[8] << 16) | ((uint32_t)p[9] << 24);

    s_state.flight_mode_flags = flags;
    if (s_state.boxarm_known) {
        s_state.armed = (flags >> s_state.boxarm_bit) & 1u;
    }
}

static void handle_rc(const uint8_t *p, uint16_t len)
{
    uint8_t n = (uint8_t)(len / 2);
    if (n > MSP_MAX_RC_CHANNELS) {
        n = MSP_MAX_RC_CHANNELS;
    }
    for (uint8_t i = 0; i < n; i++) {
        s_state.rc[i] = (uint16_t)(p[i * 2] | (p[i * 2 + 1] << 8));
    }
    s_state.rc_count = n;
    s_state.rc_valid = (n > 0);
}

static void handle_boxids(const uint8_t *p, uint16_t len)
{
    /* One byte per active box, in flightModeFlags bit order. Betaflight builds
     * both the reply and the flag bitmap by walking activeBoxIds in the same
     * order, so index == bit position. */
    for (uint16_t i = 0; i < len && i < 32; i++) {
        if (p[i] == MSP_BOX_PERM_ID_ARM) {
            if (!s_state.boxarm_known || s_state.boxarm_bit != i) {
                ESP_LOGI(TAG, "BOXARM located at flightModeFlags bit %u", (unsigned)i);
            }
            s_state.boxarm_known = true;
            s_state.boxarm_bit = (uint8_t)i;
            return;
        }
    }
    /* BOXARM is unconditionally active in Betaflight and is boxId 0, so it is
     * in practice always bit 0. If a reply somehow lacks it, keep whatever we
     * had rather than silently switching to a wrong bit. */
    if (!s_state.boxarm_known && len > 0) {
        ESP_LOGW(TAG, "BOXARM not in BOXIDS reply (%u ids); defaulting to bit 0", (unsigned)len);
        s_state.boxarm_known = true;
        s_state.boxarm_bit = 0;
    }
}

static void dispatch(uint16_t function, const uint8_t *payload, uint16_t len)
{
    s_state.last_reply_ms = now_ms();
    s_state.good_replies++;

    switch (function) {
    case MSP_STATUS: handle_status(payload, len); break;
    case MSP_RC:     handle_rc(payload, len);     break;
    case MSP_BOXIDS: handle_boxids(payload, len); break;
    default: break;   /* acks (e.g. MSP2_SET_TEXT) still count as link traffic */
    }
}

/* ------------------------------------------------------------------------- *
 * Incremental frame parser.
 *
 * Handles BOTH v1 ('$M') and v2 ('$X') replies. The v2 case matters: every
 * MSP2_SET_TEXT we send gets acked, and if those acks were not consumed the
 * parser would desync and we would lose arm state.
 * ------------------------------------------------------------------------- */
typedef enum {
    ST_IDLE, ST_HDR_M, ST_DIR,
    /* v1 */ ST_V1_LEN, ST_V1_CMD, ST_V1_DATA, ST_V1_CRC,
    /* v2 */ ST_V2_FLAG, ST_V2_FN_LO, ST_V2_FN_HI, ST_V2_LEN_LO, ST_V2_LEN_HI,
             ST_V2_DATA, ST_V2_CRC
} parse_state_t;

static parse_state_t s_ps = ST_IDLE;
static bool     s_is_v2;
static uint16_t s_fn;
static uint16_t s_len;
static uint16_t s_idx;
static uint8_t  s_ck;
static uint8_t  s_buf[MSP_MAX_PAYLOAD];

static void parse_byte(uint8_t c)
{
    switch (s_ps) {
    case ST_IDLE:
        if (c == '$') s_ps = ST_HDR_M;
        break;

    case ST_HDR_M:
        if (c == 'M')      { s_is_v2 = false; s_ps = ST_DIR; }
        else if (c == 'X') { s_is_v2 = true;  s_ps = ST_DIR; }
        else if (c == '$') { /* stay */ }
        else               { s_ps = ST_IDLE; }
        break;

    case ST_DIR:
        /* '>' is a reply. '!' is the FC rejecting our request -- drop it and
         * resync; it is not a link error worth counting. */
        if (c == '>')      { s_ps = s_is_v2 ? ST_V2_FLAG : ST_V1_LEN; }
        else               { s_ps = ST_IDLE; }
        break;

    /* ---- v1 ---- */
    case ST_V1_LEN:
        s_len = c;
        if (s_len > MSP_MAX_PAYLOAD) { s_ps = ST_IDLE; break; }
        s_ck = c;
        s_ps = ST_V1_CMD;
        break;

    case ST_V1_CMD:
        s_fn = c;
        s_ck ^= c;
        s_idx = 0;
        s_ps = (s_len == 0) ? ST_V1_CRC : ST_V1_DATA;
        break;

    case ST_V1_DATA:
        s_buf[s_idx++] = c;
        s_ck ^= c;
        if (s_idx >= s_len) s_ps = ST_V1_CRC;
        break;

    case ST_V1_CRC:
        if (s_ck == c) dispatch(s_fn, s_buf, s_len);
        else           s_state.crc_errors++;
        s_ps = ST_IDLE;
        break;

    /* ---- v2 ---- */
    case ST_V2_FLAG:
        s_ck = msp_crc8_dvb_s2(0, c);
        s_ps = ST_V2_FN_LO;
        break;

    case ST_V2_FN_LO:
        s_fn = c;
        s_ck = msp_crc8_dvb_s2(s_ck, c);
        s_ps = ST_V2_FN_HI;
        break;

    case ST_V2_FN_HI:
        s_fn |= (uint16_t)c << 8;
        s_ck = msp_crc8_dvb_s2(s_ck, c);
        s_ps = ST_V2_LEN_LO;
        break;

    case ST_V2_LEN_LO:
        s_len = c;
        s_ck = msp_crc8_dvb_s2(s_ck, c);
        s_ps = ST_V2_LEN_HI;
        break;

    case ST_V2_LEN_HI:
        s_len |= (uint16_t)c << 8;
        s_ck = msp_crc8_dvb_s2(s_ck, c);
        if (s_len > MSP_MAX_PAYLOAD) { s_ps = ST_IDLE; break; }
        s_idx = 0;
        s_ps = (s_len == 0) ? ST_V2_CRC : ST_V2_DATA;
        break;

    case ST_V2_DATA:
        s_buf[s_idx++] = c;
        s_ck = msp_crc8_dvb_s2(s_ck, c);
        if (s_idx >= s_len) s_ps = ST_V2_CRC;
        break;

    case ST_V2_CRC:
        if (s_ck == c) dispatch(s_fn, s_buf, s_len);
        else           s_state.crc_errors++;
        s_ps = ST_IDLE;
        break;
    }
}

/* ------------------------------------------------------------------------- */
/* Public API                                                                 */
/* ------------------------------------------------------------------------- */

esp_err_t msp_init(const msp_config_t *cfg)
{
    if (cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    s_cfg = *cfg;

    s_lock    = xSemaphoreCreateMutex();
    s_tx_lock = xSemaphoreCreateMutex();
    if (s_lock == NULL || s_tx_lock == NULL) {
        return ESP_ERR_NO_MEM;
    }

    memset(&s_state, 0, sizeof(s_state));

    uart_config_t uc = {
        .baud_rate  = s_cfg.baud,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t e = uart_driver_install(s_cfg.uart_num, MSP_RX_BUF_SIZE, MSP_TX_BUF_SIZE, 0, NULL, 0);
    if (e != ESP_OK) return e;
    e = uart_param_config(s_cfg.uart_num, &uc);
    if (e != ESP_OK) return e;
    e = uart_set_pin(s_cfg.uart_num, s_cfg.tx_gpio, s_cfg.rx_gpio,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (e != ESP_OK) return e;

    s_inited = true;
    ESP_LOGI(TAG, "UART%d @ %d baud, TX=GPIO%d RX=GPIO%d",
             (int)s_cfg.uart_num, s_cfg.baud, s_cfg.tx_gpio, s_cfg.rx_gpio);
    return ESP_OK;
}

void msp_get_state(msp_state_t *out)
{
    if (out == NULL) return;
    if (s_lock && xSemaphoreTake(s_lock, pdMS_TO_TICKS(20)) == pdTRUE) {
        *out = s_state;
        xSemaphoreGive(s_lock);
    } else {
        /* Never block a caller on a contended lock; a stale-but-safe answer is
         * better than stalling the logic task. */
        memset(out, 0, sizeof(*out));
    }
}

void msp_task(void *arg)
{
    (void)arg;
    uint8_t rx[128];
    uint32_t last_boxids = 0;

    /* Ask for the box list immediately; arm state is meaningless without it. */
    msp_send_v1(MSP_BOXIDS, NULL, 0);
    last_boxids = now_ms();

    for (;;) {
        /* Drain whatever the FC has sent. Zero timeout: this never blocks. */
        int n;
        while ((n = uart_read_bytes(s_cfg.uart_num, rx, sizeof(rx), 0)) > 0) {
            if (s_lock && xSemaphoreTake(s_lock, pdMS_TO_TICKS(20)) == pdTRUE) {
                for (int i = 0; i < n; i++) {
                    parse_byte(rx[i]);
                }
                xSemaphoreGive(s_lock);
            }
        }

        uint32_t t = now_ms();

        if (s_lock && xSemaphoreTake(s_lock, pdMS_TO_TICKS(20)) == pdTRUE) {
            bool up = s_state.last_reply_ms != 0 &&
                      (t - s_state.last_reply_ms) < s_cfg.link_timeout_ms;
            if (s_state.link_up && !up) {
                ESP_LOGW(TAG, "MSP link lost -- forcing armed=false");
            }
            s_state.link_up = up;

            /* Hard rule: no link means not armed. A dead UART must close an
             * open clip rather than leave the camera rolling. */
            if (!up) {
                s_state.armed = false;
                s_state.rc_valid = false;
            }
            xSemaphoreGive(s_lock);
        }

        if ((t - last_boxids) >= BOXIDS_PERIOD_MS) {
            msp_send_v1(MSP_BOXIDS, NULL, 0);
            last_boxids = t;
        }

        msp_send_v1(MSP_STATUS, NULL, 0);
        msp_send_v1(MSP_RC, NULL, 0);

        vTaskDelay(pdMS_TO_TICKS(POLL_PERIOD_MS));
    }
}
