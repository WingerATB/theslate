/* SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
 *
 * MSP client for Betaflight (2025.12+).
 *
 * Deliberately standalone: this component knows nothing about DJI, BLE or the
 * record state machine, so it can be built and bench-tested with the camera
 * disconnected (see build order step 4 in the project brief).
 *
 * Threading contract:
 *   - msp_task() owns the UART and is the only writer of the internal state.
 *   - msp_get_state() takes a snapshot under a mutex; callers get a private
 *     copy and never hold a lock while doing anything slow.
 *   - Nothing here ever blocks waiting on the flight controller. Every read is
 *     a zero-timeout poll. A dead FC costs us nothing but a stale timestamp.
 */
#ifndef CAMLINK_MSP_H
#define CAMLINK_MSP_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/uart.h"
#include "esp_err.h"

#define MSP_MAX_RC_CHANNELS   18

/* ---- MSP v1 commands (verified against Betaflight master src/main/msp) ---- */
#define MSP_STATUS            101
#define MSP_RC                105
#define MSP_BOXIDS            119

/* ---- MSP v2 (msp_protocol_v2_betaflight.h) ------------------------------- *
 * NOTE: Betaflight allocates CUSTOM_MSG_MAX_NUM == 4 slots, ids 7..10, which
 * surface as OSD elements OSD_CUSTOM_MSG0..3 ("Custom Message 1..4" in the
 * Configurator OSD tab). Id 11 is MSP2TEXT_BATTERY_PROFILE_NAME -- writing
 * past slot 3 would clobber the battery profile name, so the slot argument is
 * range-checked. Text is capped at MAX_NAME_LENGTH (16) by the FC. */
#define MSP2_SET_TEXT             0x3007
#define MSP2TEXT_CUSTOM_MSG_0     7
#define MSP_CUSTOM_MSG_COUNT      4
#define MSP_TEXT_MAX_LEN          16

/* BOXARM's permanent id. Its *bit position* in flightModeFlags follows the FC's
 * configured box list and is discovered via MSP_BOXIDS, not assumed. */
#define MSP_BOX_PERM_ID_ARM   0

typedef struct {
    /* Link health */
    bool     link_up;             /* a valid reply arrived within the timeout  */
    uint32_t last_reply_ms;       /* esp_timer ms of last good reply           */
    uint32_t osd_writes;     /* OSD slot writes sent, for "is it still
                              * refreshing?" -- a stale readout looks the same
                              * whether we stopped writing or the FC stopped
                              * listening, and this separates them            */
    uint32_t good_replies;        /* cumulative, for diagnostics               */
    uint32_t crc_errors;

    /* Arm state. armed is forced false whenever link_up is false, so a dead
     * UART closes an open clip instead of leaving it running forever. */
    bool     armed;
    bool     boxarm_known;        /* BOXARM bit position resolved from BOXIDS  */
    uint8_t  boxarm_bit;          /* bit index within flightModeFlags          */
    uint32_t flight_mode_flags;   /* first 32 bits                             */

    /* RC channels */
    uint16_t rc[MSP_MAX_RC_CHANNELS];
    uint8_t  rc_count;
    bool     rc_valid;
} msp_state_t;

typedef struct {
    uart_port_t uart_num;
    int         tx_gpio;
    int         rx_gpio;
    int         baud;
    uint32_t    link_timeout_ms;  /* declare link dead after this quiet period */
} msp_config_t;

esp_err_t msp_init(const msp_config_t *cfg);

/* Snapshot current state. Safe from any task. */
void msp_get_state(msp_state_t *out);

/* The polling task body. Create with xTaskCreate(). Never returns. */
void msp_task(void *arg);

/* Write one OSD custom message slot (0..MSP_CUSTOM_MSG_COUNT-1).
 * Text longer than MSP_TEXT_MAX_LEN is truncated. Returns ESP_ERR_INVALID_ARG
 * on a bad slot. Safe to call from another task. */
esp_err_t msp_set_osd_text(uint8_t slot, const char *text);

/* Exposed for unit testing the framing without a UART. */
uint8_t msp_crc8_dvb_s2(uint8_t crc, uint8_t a);

#endif /* CAMLINK_MSP_H */
