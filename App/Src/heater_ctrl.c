#include "heater_ctrl.h"

/* Relay masks per step, wiring per docs/heater_rod_4_changeover_schematic.svg.
   Invariant: R3 only together with R2. */
const uint8_t heater_step_mask[HEATER_STEPS] = {
    0x0, /* 0/6  off */
    0x6, /* 1/6  R2+R3: L2-E2-E3-N */
    0x1, /* 2/6  R1: E1 */
    0xE, /* 3/6  R2+R3+R4: L2-E2-E3-L3 */
    0x3, /* 4/6  R1+R2: E1+E2 */
    0xF, /* 5/6  all: E1 + E2/E3 on L2-L3 */
    0xB, /* 6/6  R1+R2+R4: all L-N */
};

/* floor(k * 1000 / 6): lowest setpoint selecting step k */
const uint16_t heater_step_threshold[HEATER_STEPS] = { 0, 166, 333, 500, 666, 833, 1000 };

/* round(k * 1000 / 6): reported power of step k */
const uint16_t heater_step_power[HEATER_STEPS] = { 0, 167, 333, 500, 667, 833, 1000 };

void heater_ctrl_init(heater_ctrl_t *h, uint32_t now_ms)
{
    h->setpoint = 0;
    h->actual_step = 0;
    h->target_step = 0;
    h->mask = 0;
    h->hold_active = false;
    h->seq_phase = 0;
    h->seq_step = 0;
    h->seq_next_ms = now_ms;
    h->last_change_ms = now_ms;
}

void heater_ctrl_set_setpoint(heater_ctrl_t *h, uint16_t setpoint)
{
    h->setpoint = setpoint > HEATER_SETPOINT_MAX ? HEATER_SETPOINT_MAX : setpoint;
}

uint8_t heater_ctrl_step_for(uint16_t setpoint)
{
    uint8_t step = 0;
    for (uint8_t k = 1; k < HEATER_STEPS; k++) {
        if (setpoint >= heater_step_threshold[k]) {
            step = k;
        }
    }
    return step;
}

static uint8_t target_with_hysteresis(uint16_t setpoint, uint8_t current, uint16_t hysteresis)
{
    uint8_t down = heater_ctrl_step_for(setpoint);
    if (down < current) {
        return down;
    }

    /* Stepping up to k needs the threshold plus hysteresis (capped at 100 %) */
    uint8_t up = current;
    for (uint8_t k = current + 1; k < HEATER_STEPS; k++) {
        uint32_t thr = (uint32_t)heater_step_threshold[k] + hysteresis;
        if (thr > HEATER_SETPOINT_MAX) {
            thr = HEATER_SETPOINT_MAX;
        }
        if (setpoint >= thr) {
            up = k;
        }
    }
    return up;
}

static bool time_reached(uint32_t now_ms, uint32_t t)
{
    return (int32_t)(now_ms - t) >= 0;
}

/*
 * Switching sequence from mask A to B, HEATER_SEQ_PAUSE_MS between phases:
 *   1. release R3 if B does not have it
 *   2. set R1, R2, R4 to B
 *   3. energize R3 if B has it
 * A pause is only inserted if a later phase still has to act.
 */
static void sequence_run(heater_ctrl_t *h, uint32_t now_ms)
{
    const uint8_t b = heater_step_mask[h->seq_step];

    while (h->seq_phase != 0 && time_reached(now_ms, h->seq_next_ms)) {
        switch (h->seq_phase) {
        case 1:
            h->seq_phase = 2;
            if ((h->mask & HEATER_R3) && !(b & HEATER_R3)) {
                h->mask &= (uint8_t)~HEATER_R3;
                uint8_t phase2 = (uint8_t)((h->mask & HEATER_R3) | (b & (uint8_t)~HEATER_R3));
                if (phase2 != h->mask) {
                    h->seq_next_ms = now_ms + HEATER_SEQ_PAUSE_MS;
                }
            }
            break;
        case 2: {
            h->seq_phase = 3;
            uint8_t next = (uint8_t)((h->mask & HEATER_R3) | (b & (uint8_t)~HEATER_R3));
            if (next != h->mask) {
                h->mask = next;
                if ((b & HEATER_R3) && !(h->mask & HEATER_R3)) {
                    h->seq_next_ms = now_ms + HEATER_SEQ_PAUSE_MS;
                }
            }
            break;
        }
        default:
            if (b & HEATER_R3) {
                h->mask |= HEATER_R3;
            }
            h->seq_phase = 0;
            h->actual_step = h->seq_step;
            h->last_change_ms = now_ms;
            break;
        }
    }
}

uint8_t heater_ctrl_poll(heater_ctrl_t *h, uint32_t now_ms, uint16_t hold_time_s, uint16_t hysteresis)
{
    h->target_step = target_with_hysteresis(h->setpoint, h->actual_step, hysteresis);
    h->hold_active = false;

    if (h->seq_phase == 0 && h->target_step != h->actual_step) {
        bool up = h->target_step > h->actual_step;
        if (up && !time_reached(now_ms, h->last_change_ms + (uint32_t)hold_time_s * 1000)) {
            h->hold_active = true;
        } else {
            h->seq_step = h->target_step;
            h->seq_phase = 1;
            h->seq_next_ms = now_ms;
        }
    }

    sequence_run(h, now_ms);
    return h->mask;
}

bool heater_ctrl_sequencing(const heater_ctrl_t *h)
{
    return h->seq_phase != 0;
}
