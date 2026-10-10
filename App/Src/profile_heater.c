/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "profile.h"

#include "main.h"

#include "config.h"
#include "heater_ctrl.h"
#include "relay.h"

/*
 * Heater variant: 3-phase heating rod, 7 power steps.
 * Holding 0x0010: setpoint (0.1 %), 0x0011: hold time (s), 0x0012: hysteresis (0.1 %).
 * Input 0x0010: actual power (0.1 %), 0x0011: actual step, 0x0012: target step.
 */

#define HREG_SETPOINT   0x0010
#define HREG_HOLD_TIME  0x0011
#define HREG_HYSTERESIS 0x0012

#define IREG_ACTUAL_POWER 0x0010
#define IREG_ACTUAL_STEP  0x0011
#define IREG_TARGET_STEP  0x0012

#define STATUS_HOLD_ACTIVE 0x0100
#define STATUS_SEQUENCING  0x0200

static heater_ctrl_t heater;

static void heater_init(void)
{
    heater_ctrl_init(&heater, HAL_GetTick());
    relay_set_mask(heater.mask);
}

static void heater_poll(uint32_t now_ms)
{
    uint8_t mask = heater_ctrl_poll(&heater, now_ms, config.hold_time_s, config.hysteresis);
    if (mask != relay_get_mask()) {
        relay_set_mask(mask);
    }
}

/* Step down to 0 through the regular switching sequence */
static void heater_safe_state(void)
{
    heater_ctrl_set_setpoint(&heater, 0);
}

static nmbs_error heater_read_hreg(uint16_t addr, uint16_t *val)
{
    switch (addr) {
    case HREG_SETPOINT:
        *val = heater.setpoint;
        return NMBS_ERROR_NONE;
    case HREG_HOLD_TIME:
        *val = config.hold_time_s;
        return NMBS_ERROR_NONE;
    case HREG_HYSTERESIS:
        *val = config.hysteresis;
        return NMBS_ERROR_NONE;
    default:
        return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
    }
}

static nmbs_error heater_write_hreg(uint16_t addr, uint16_t val, bool apply, bool *control)
{
    switch (addr) {
    case HREG_SETPOINT:
        if (val > HEATER_SETPOINT_MAX) {
            return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE;
        }
        if (apply) {
            heater_ctrl_set_setpoint(&heater, val);
            *control = true;
        }
        return NMBS_ERROR_NONE;
    case HREG_HOLD_TIME:
        if (apply) {
            config.hold_time_s = val;
        }
        return NMBS_ERROR_NONE;
    case HREG_HYSTERESIS:
        if (val > HEATER_POWER_FULL) {
            return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE;
        }
        if (apply) {
            config.hysteresis = val;
        }
        return NMBS_ERROR_NONE;
    default:
        return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
    }
}

static nmbs_error heater_read_ireg(uint16_t addr, uint16_t *val)
{
    switch (addr) {
    case IREG_ACTUAL_POWER:
        *val = heater_steps[heater.actual_step].power;
        return NMBS_ERROR_NONE;
    case IREG_ACTUAL_STEP:
        *val = heater.actual_step;
        return NMBS_ERROR_NONE;
    case IREG_TARGET_STEP:
        *val = heater.target_step;
        return NMBS_ERROR_NONE;
    default:
        return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
    }
}

static uint16_t heater_status_flags(void)
{
    uint16_t flags = 0;
    if (heater.hold_active) {
        flags |= STATUS_HOLD_ACTIVE;
    }
    if (heater_ctrl_sequencing(&heater)) {
        flags |= STATUS_SEQUENCING;
    }
    return flags;
}

const profile_t profile = {
    .id = PROFILE_ID_HEATER,
    .name = "heater",
    .init = heater_init,
    .poll = heater_poll,
    .safe_state = heater_safe_state,
    .write_coils = NULL,
    .read_hreg = heater_read_hreg,
    .write_hreg = heater_write_hreg,
    .read_ireg = heater_read_ireg,
    .status_flags = heater_status_flags,
};
