#ifndef HEATER_CTRL_H
#define HEATER_CTRL_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Heater step logic, hardware independent (see docs/IMPLEMENTATION.md, 4).
 * Power values are in 0.1 % (0..1000).
 */

#define HEATER_STEPS        7
#define HEATER_POWER_FULL   1000 /* 100 % in 0.1 % */
#define HEATER_SETPOINT_MAX HEATER_POWER_FULL
#define HEATER_SEQ_PAUSE_MS 50

/* Relay bits, bit0 = relay 1 */
#define HEATER_R1 0x1
#define HEATER_R2 0x2
#define HEATER_R3 0x4
#define HEATER_R4 0x8

typedef struct {
    uint8_t mask;   /* relay outputs */
    /* Power share in 0.1 %, rounded up. This is also the lowest setpoint
       selecting the step, so writing back a reported value gives exactly
       that step. Must be strictly ascending. */
    uint16_t power;
} heater_step_t;

extern const heater_step_t heater_steps[HEATER_STEPS];

typedef struct {
    uint16_t setpoint;
    uint8_t actual_step;     /* last completed step */
    uint8_t target_step;     /* requested by the setpoint (with hysteresis) */
    uint8_t mask;            /* relay outputs */
    bool hold_active;        /* a higher step waits for the hold time */
    /* sequencer */
    uint8_t seq_phase;       /* 0 = idle */
    uint8_t seq_step;
    uint32_t seq_next_ms;
    uint32_t last_change_ms; /* hold time reference */
} heater_ctrl_t;

/* Step 0, all relays off. The hold time starts now. */
void heater_ctrl_init(heater_ctrl_t *h, uint32_t now_ms);

/* setpoint must be <= HEATER_SETPOINT_MAX */
void heater_ctrl_set_setpoint(heater_ctrl_t *h, uint16_t setpoint);

/* Run step selection and the switching sequence, returns the relay mask */
uint8_t heater_ctrl_poll(heater_ctrl_t *h, uint32_t now_ms, uint16_t hold_time_s, uint16_t hysteresis);

/* Highest step whose power is <= setpoint (rounded down), without hysteresis */
uint8_t heater_ctrl_step_for(uint16_t setpoint);

bool heater_ctrl_sequencing(const heater_ctrl_t *h);

#endif
