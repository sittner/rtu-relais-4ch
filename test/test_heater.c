/* SPDX-License-Identifier: GPL-3.0-or-later */
/* Host tests: heater step logic and the heater variant register map */
#include "test_common.h"

#include "config.h"
#include "heater_ctrl.h"
#include "main.h"
#include "profile.h"
#include "relay.h"

/* ---- heater_ctrl ---------------------------------------------------- */

/*
 * Power of a relay mask per the schematic, in units of one element at
 * 230 V. R3 off ties junction J to N, R4 off ties the bottom of E3 to N.
 */
static double circuit_power(uint8_t m)
{
    double p = (m & HEATER_R1) ? 1.0 : 0.0; /* E1: L1-N */
    if (!(m & HEATER_R3)) {
        p += (m & HEATER_R2) ? 1.0 : 0.0; /* E2: L2-J(N) */
        p += (m & HEATER_R4) ? 1.0 : 0.0; /* E3: J(N)-L3 */
    } else if (m & HEATER_R2) {
        /* E2+E3 in series: L2-N (230 V) or L2-L3 (400 V) over 2R */
        p += (m & HEATER_R4) ? 1.5 : 0.5;
    }
    return p;
}

static void test_step_table(void)
{
    for (int k = 0; k < HEATER_STEPS; k++) {
        uint8_t m = heater_steps[k].mask;
        CHECK(!(m & HEATER_R3) || (m & HEATER_R2));
        /* power == share of the 3 elements, in 0.1 % rounded up */
        double share = circuit_power(m) / 3.0 * HEATER_POWER_FULL;
        CHECK(heater_steps[k].power >= share - 1e-6 && heater_steps[k].power < share + 1.0 - 1e-6);
        if (k > 0) {
            CHECK(heater_steps[k].power > heater_steps[k - 1].power);
        }
    }
    CHECK_EQ(heater_steps[0].power, 0);
    CHECK_EQ(heater_steps[HEATER_STEPS - 1].power, HEATER_POWER_FULL);
}

static void test_quantization(void)
{
    for (int k = 0; k < HEATER_STEPS; k++) {
        CHECK_EQ(heater_ctrl_step_for(heater_steps[k].power), k);
        if (k > 0) {
            CHECK_EQ(heater_ctrl_step_for(heater_steps[k].power - 1), k - 1);
        }
    }
    CHECK_EQ(heater_ctrl_step_for(333), 1); /* 33.3 % < 2/6: rounded down */
    CHECK_EQ(heater_ctrl_step_for(HEATER_POWER_FULL), HEATER_STEPS - 1);
}

/* Run the controller in 1 ms steps, check every relay transition */
static uint32_t sim_now;
static int sim_transitions;

static void sim_run(heater_ctrl_t *h, uint32_t ms, uint16_t hold, uint16_t hyst)
{
    static uint32_t last_change;
    for (uint32_t i = 0; i < ms; i++) {
        uint8_t before = h->mask;
        uint8_t after = heater_ctrl_poll(h, ++sim_now, hold, hyst);
        if (after == before) {
            continue;
        }
        sim_transitions++;
        /* R3 only with R2, in every intermediate state */
        CHECK(!(after & HEATER_R3) || (after & HEATER_R2));
        /* R3 never switches together with another relay */
        if ((before ^ after) & HEATER_R3) {
            CHECK_EQ((before ^ after) & ~HEATER_R3, 0);
        }
        /* pause between consecutive transitions of one sequence */
        if (sim_transitions > 1 && heater_ctrl_sequencing(h)) {
            CHECK(sim_now - last_change >= HEATER_SEQ_PAUSE_MS);
        }
        last_change = sim_now;
    }
}

static void test_all_transitions(void)
{
    for (int a = 0; a < HEATER_STEPS; a++) {
        for (int b = 0; b < HEATER_STEPS; b++) {
            heater_ctrl_t h;
            sim_now = 0;
            heater_ctrl_init(&h, sim_now);
            heater_ctrl_set_setpoint(&h, heater_steps[a].power);
            sim_run(&h, 500, 0, 0);
            CHECK_EQ(h.actual_step, a);
            CHECK_EQ(h.mask, heater_steps[a].mask);

            sim_transitions = 0;
            heater_ctrl_set_setpoint(&h, heater_steps[b].power);
            sim_run(&h, 500, 0, 0);
            CHECK_EQ(h.actual_step, b);
            CHECK_EQ(h.mask, heater_steps[b].mask);
            CHECK(!heater_ctrl_sequencing(&h));
        }
    }
}

static void test_hold_time(void)
{
    heater_ctrl_t h;
    sim_now = 0;
    heater_ctrl_init(&h, sim_now);

    /* stepping up waits for the hold time, counted from init */
    heater_ctrl_set_setpoint(&h, 1000);
    sim_run(&h, 59999, 60, 0);
    CHECK_EQ(h.mask, 0);
    CHECK(h.hold_active);
    sim_run(&h, 300, 60, 0);
    CHECK_EQ(h.actual_step, 6);
    CHECK(!h.hold_active);

    /* stepping down is immediate */
    heater_ctrl_set_setpoint(&h, 500);
    sim_run(&h, 200, 60, 0);
    CHECK_EQ(h.actual_step, 3);

    /* up again: hold time since the last change */
    heater_ctrl_set_setpoint(&h, 1000);
    sim_run(&h, 59000, 60, 0);
    CHECK_EQ(h.actual_step, 3);
    sim_run(&h, 1300, 60, 0);
    CHECK_EQ(h.actual_step, 6);

    /* setpoint 0 always goes down at once */
    heater_ctrl_set_setpoint(&h, 0);
    sim_run(&h, 200, 60, 0);
    CHECK_EQ(h.actual_step, 0);
    CHECK_EQ(h.mask, 0);
}

static void test_hysteresis(void)
{
    heater_ctrl_t h;
    sim_now = 0;
    heater_ctrl_init(&h, sim_now);

    heater_ctrl_set_setpoint(&h, 180); /* above 167, below 167 + 20 */
    sim_run(&h, 500, 0, 20);
    CHECK_EQ(h.actual_step, 0);
    heater_ctrl_set_setpoint(&h, 187);
    sim_run(&h, 500, 0, 20);
    CHECK_EQ(h.actual_step, 1);

    /* down uses the plain threshold */
    heater_ctrl_set_setpoint(&h, 170);
    sim_run(&h, 500, 0, 20);
    CHECK_EQ(h.actual_step, 1);
    heater_ctrl_set_setpoint(&h, 165);
    sim_run(&h, 500, 0, 20);
    CHECK_EQ(h.actual_step, 0);

    /* 100 % stays reachable */
    heater_ctrl_set_setpoint(&h, 1000);
    sim_run(&h, 500, 0, 20);
    CHECK_EQ(h.actual_step, 6);
}

/* ---- heater variant over Modbus ------------------------------------- */

static void boot(void)
{
    fake_tick = 1000;
    relay_init();
    config_init();
    profile.init();
    mb_server_init();
}

static void test_server(void)
{
    fake_flash_erase_all();
    boot();
    uint16_t r[3] = { 0 };
    uint8_t bits = 0;

    read_regs(1, 0x04, 0x0001, 1, r);
    CHECK_EQ(r[0], PROFILE_ID_HEATER);
    read_regs(1, 0x03, 0x0000, 1, r);
    CHECK_EQ(r[0], 60); /* heater default comm timeout */
    read_regs(1, 0x03, 0x0010, 3, r);
    CHECK_EQ(r[0], 0);
    CHECK_EQ(r[1], 60);
    CHECK_EQ(r[2], 0);

    /* coils are read only */
    write_coil(1, 0, 0xFF00);
    CHECK_EQ(last_exception(), 1);
    write_coils(1, 0, 4, 0x0F);
    CHECK_EQ(last_exception(), 1);
    read_coils(1, 0, 4, &bits);
    CHECK_EQ(rsp_len, 3);

    write_reg(1, 0x0010, 1001);
    CHECK_EQ(last_exception(), 3);

    /* 50 %: waits for the hold time after boot */
    write_reg(1, 0x0010, 500);
    CHECK_EQ(rsp_len, 5);
    run_for(1000);
    read_regs(1, 0x04, 0x0003, 1, r);
    CHECK_EQ(r[0] & 0x0100, 0x0100); /* hold active */
    read_regs(1, 0x04, 0x0010, 3, r);
    CHECK_EQ(r[0], 0);
    CHECK_EQ(r[1], 0);
    CHECK_EQ(r[2], 3);

    write_reg(1, 0x0010, 500); /* keep the comm timeout alive */
    run_for(59500);
    read_regs(1, 0x04, 0x0010, 2, r);
    CHECK_EQ(r[0], 500);
    CHECK_EQ(r[1], 3);
    CHECK_EQ(relay_get_mask(), 0xE);
    read_coils(1, 0, 4, &bits);
    CHECK_EQ(bits, 0xE);

    /* comm timeout: steps down to 0 through the sequence */
    run_for(60000);
    CHECK(mb_server_comm_timeout());
    run_for(200);
    CHECK_EQ(relay_get_mask(), 0);
    read_regs(1, 0x04, 0x0003, 1, r);
    CHECK_EQ(r[0] & 0x0001, 1);
    read_regs(1, 0x03, 0x0010, 1, r);
    CHECK_EQ(r[0], 0);

    /* hold time and hysteresis are persistent */
    uint16_t cfg[2] = { 30, 15 };
    write_regs(1, 0x0011, 2, cfg);
    CHECK_EQ(rsp_len, 5);
    boot();
    CHECK_EQ(config.hold_time_s, 30);
    CHECK_EQ(config.hysteresis, 15);
    write_reg(1, 0x0012, 1001);
    CHECK_EQ(last_exception(), 3);
}

int main(void)
{
    RUN(test_step_table);
    RUN(test_quantization);
    RUN(test_all_transitions);
    RUN(test_hold_time);
    RUN(test_hysteresis);
    RUN(test_server);

    printf(test_failures ? "%d FAILURE(S)\n" : "all passed\n", test_failures);
    return test_failures ? 1 : 0;
}
