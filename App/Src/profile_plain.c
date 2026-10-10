/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "profile.h"

#include "main.h"

#include "relay.h"

/*
 * Plain variant: generic 4-channel relay board.
 * Holding 0x0010: relay mask, 0x0020..0x0023: pulse timers (100 ms units).
 */

#define HREG_RELAY_MASK  0x0010
#define HREG_PULSE_FIRST 0x0020
#define HREG_PULSE_LAST  (HREG_PULSE_FIRST + RELAY_COUNT - 1)

#define PULSE_UNIT_MS 100

static bool pulse_active[RELAY_COUNT];
static uint32_t pulse_end_ms[RELAY_COUNT];

static void pulse_cancel(uint8_t ch)
{
    pulse_active[ch] = false;
}

static void plain_init(void)
{
    for (uint8_t ch = 0; ch < RELAY_COUNT; ch++) {
        pulse_cancel(ch);
    }
}

static void plain_poll(uint32_t now_ms)
{
    for (uint8_t ch = 0; ch < RELAY_COUNT; ch++) {
        if (pulse_active[ch] && (int32_t)(now_ms - pulse_end_ms[ch]) >= 0) {
            pulse_active[ch] = false;
            relay_set(ch, false);
        }
    }
}

static void plain_safe_state(void)
{
    plain_init();
    relay_set_mask(0);
}

static nmbs_error plain_write_coils(uint16_t addr, uint16_t qty, const nmbs_bitfield coils)
{
    for (uint16_t i = 0; i < qty; i++) {
        uint8_t ch = (uint8_t)(addr + i);
        pulse_cancel(ch);
        relay_set(ch, nmbs_bitfield_read(coils, i));
    }
    return NMBS_ERROR_NONE;
}

static nmbs_error plain_read_hreg(uint16_t addr, uint16_t *val)
{
    if (addr == HREG_RELAY_MASK) {
        *val = relay_get_mask();
        return NMBS_ERROR_NONE;
    }
    if (addr >= HREG_PULSE_FIRST && addr <= HREG_PULSE_LAST) {
        uint8_t ch = (uint8_t)(addr - HREG_PULSE_FIRST);
        *val = 0;
        if (pulse_active[ch]) {
            int32_t left = (int32_t)(pulse_end_ms[ch] - HAL_GetTick());
            if (left > 0) {
                *val = (uint16_t)((left + PULSE_UNIT_MS - 1) / PULSE_UNIT_MS);
            }
        }
        return NMBS_ERROR_NONE;
    }
    return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
}

static nmbs_error plain_write_hreg(uint16_t addr, uint16_t val, bool apply, bool *control)
{
    if (addr == HREG_RELAY_MASK) {
        if (val > RELAY_MASK_ALL) {
            return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE;
        }
        if (apply) {
            plain_init();
            relay_set_mask((uint8_t)val);
            *control = true;
        }
        return NMBS_ERROR_NONE;
    }
    if (addr >= HREG_PULSE_FIRST && addr <= HREG_PULSE_LAST) {
        if (apply) {
            uint8_t ch = (uint8_t)(addr - HREG_PULSE_FIRST);
            if (val == 0) {
                pulse_cancel(ch);
            } else {
                pulse_end_ms[ch] = HAL_GetTick() + (uint32_t)val * PULSE_UNIT_MS;
                pulse_active[ch] = true;
                relay_set(ch, true);
            }
            *control = true;
        }
        return NMBS_ERROR_NONE;
    }
    return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
}

static nmbs_error plain_read_ireg(uint16_t addr, uint16_t *val)
{
    (void)addr;
    (void)val;
    return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
}

static uint16_t plain_status_flags(void)
{
    return 0;
}

const profile_t profile = {
    .id = PROFILE_ID_PLAIN,
    .name = "plain",
    .init = plain_init,
    .poll = plain_poll,
    .safe_state = plain_safe_state,
    .write_coils = plain_write_coils,
    .read_hreg = plain_read_hreg,
    .write_hreg = plain_write_hreg,
    .read_ireg = plain_read_ireg,
    .status_flags = plain_status_flags,
};
