#include "profile.h"

#include "relay.h"

/* Heater variant, implemented in step 5 */

static void heater_init(void)
{
}

static void heater_poll(uint32_t now_ms)
{
    (void)now_ms;
}

static void heater_safe_state(void)
{
    relay_set_mask(0);
}

static nmbs_error heater_read_hreg(uint16_t addr, uint16_t *val)
{
    (void)addr;
    (void)val;
    return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
}

static nmbs_error heater_write_hreg(uint16_t addr, uint16_t val, bool apply, bool *control)
{
    (void)addr;
    (void)val;
    (void)apply;
    (void)control;
    return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
}

static nmbs_error heater_read_ireg(uint16_t addr, uint16_t *val)
{
    (void)addr;
    (void)val;
    return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
}

static uint16_t heater_status_flags(void)
{
    return 0;
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
