/* SPDX-License-Identifier: GPL-3.0-or-later */
/* Host tests: Modbus register map of the plain variant */
#include "test_common.h"

#include "config.h"
#include "main.h"
#include "nanomodbus.h"
#include "profile.h"
#include "relay.h"

static void boot(void)
{
    fake_tick = 1000;
    relay_init();
    config_init();
    profile.init();
    mb_server_init();
}

static void fresh_boot(void)
{
    fake_flash_erase_all();
    boot();
}

static void test_ident(void)
{
    fresh_boot();
    uint16_t r[6] = { 0 };
    read_regs(1, 0x04, 0x0000, 6, r);
    CHECK_EQ(rsp_len, 2 + 12);
    CHECK_EQ(r[0], (FW_VERSION_MAJOR << 8) | FW_VERSION_MINOR);
    CHECK_EQ(r[1], PROFILE_ID_PLAIN);
    CHECK_EQ(r[2], 0);
    CHECK_EQ(r[3], 0x0002); /* defaults in use */

    /* FC 2B/0E basic device identification */
    uint8_t pdu[] = { 0x2B, 0x0E, 0x01, 0x00 };
    transact(1, pdu, sizeof(pdu));
    CHECK(rsp_len > 10 && rsp[0] == 0x2B);
    CHECK(memmem(rsp, rsp_len, "plain", 5) != NULL);
}

static void test_coils(void)
{
    fresh_boot();
    uint8_t bits = 0xFF;

    write_coil(1, 0, 0xFF00);
    CHECK_EQ(rsp_len, 5);
    CHECK_EQ(relay_get_mask(), 0x01);
    /* active low: relay 1 pin low, others high */
    CHECK_EQ(fake_gpiob.odr & 0x0F, 0x0E);

    read_coils(1, 0, 4, &bits);
    CHECK_EQ(bits, 0x01);

    write_coils(1, 0, 4, 0x0A);
    CHECK_EQ(rsp_len, 5);
    CHECK_EQ(relay_get_mask(), 0x0A);

    write_coils(1, 1, 2, 0x01); /* relay 2 on, relay 3 off */
    CHECK_EQ(relay_get_mask(), 0x0A & ~0x04);

    /* stock "toggle" value is not accepted */
    write_coil(1, 0, 0x5500);
    CHECK_EQ(last_exception(), 3);

    read_coils(1, 2, 3, &bits);
    CHECK_EQ(last_exception(), 2);
    write_coil(1, 4, 0xFF00);
    CHECK_EQ(last_exception(), 2);
}

static void test_mask_and_pulse(void)
{
    fresh_boot();
    uint16_t v = 0;

    write_reg(1, 0x0010, 0x05);
    CHECK_EQ(relay_get_mask(), 0x05);
    read_regs(1, 0x03, 0x0010, 1, &v);
    CHECK_EQ(v, 0x05);
    write_reg(1, 0x0010, 0x10);
    CHECK_EQ(last_exception(), 3);

    write_reg(1, 0x0010, 0);
    write_reg(1, 0x0021, 5); /* relay 2 on for 500 ms */
    CHECK_EQ(relay_get_mask(), 0x02);
    read_regs(1, 0x03, 0x0021, 1, &v);
    CHECK_EQ(v, 5);
    run_for(499);
    CHECK_EQ(relay_get_mask(), 0x02);
    read_regs(1, 0x03, 0x0021, 1, &v);
    CHECK_EQ(v, 1);
    run_for(1);
    CHECK_EQ(relay_get_mask(), 0x00);
    read_regs(1, 0x03, 0x0021, 1, &v);
    CHECK_EQ(v, 0);

    /* a coil write cancels a running pulse */
    write_reg(1, 0x0020, 10);
    write_coil(1, 0, 0xFF00);
    run_for(2000);
    CHECK_EQ(relay_get_mask(), 0x01);

    /* writing 0 cancels the timer, the relay stays on */
    write_reg(1, 0x0022, 10);
    write_reg(1, 0x0022, 0);
    run_for(2000);
    CHECK_EQ(relay_get_mask(), 0x05);
}

static void test_addressing(void)
{
    fresh_boot();

    /* broadcast: executed, no reply */
    write_coil(0, 3, 0xFF00);
    CHECK_EQ(rsp_len, -1);
    CHECK_EQ(relay_get_mask(), 0x08);

    /* other slave: ignored */
    write_coil(2, 0, 0xFF00);
    CHECK_EQ(rsp_len, -1);
    CHECK_EQ(relay_get_mask(), 0x08);

    /* unknown registers and function codes */
    uint16_t v;
    read_regs(1, 0x03, 0x0001, 1, &v);
    CHECK_EQ(last_exception(), 2);
    read_regs(1, 0x04, 0x0008, 1, &v);
    CHECK_EQ(last_exception(), 2);
    read_regs(1, 0x03, 0x00FE, 4, &v); /* crosses into unmapped range */
    CHECK_EQ(last_exception(), 2);
    uint8_t pdu[] = { 0x02, 0, 0, 0, 1 }; /* discrete inputs: not supported */
    transact(1, pdu, sizeof(pdu));
    CHECK_EQ(last_exception(), 1);
}

static void test_serial_settings(void)
{
    fresh_boot();
    uint16_t r[3] = { 0 };

    /* invalid value inside FC 10: nothing is written */
    uint16_t bad[3] = { 5, 9, 1 };
    write_regs(1, 0x0100, 3, bad);
    CHECK_EQ(last_exception(), 3);
    read_regs(1, 0x03, 0x0100, 3, r);
    CHECK_EQ(r[0], 1);
    CHECK_EQ(r[1], 1);
    CHECK_EQ(r[2], 0);

    /* staged values are read back, but not active yet */
    uint16_t good[3] = { 5, 3, 1 }; /* addr 5, 38400, even */
    write_regs(1, 0x0100, 3, good);
    CHECK_EQ(rsp_len, 5);
    read_regs(1, 0x03, 0x0100, 3, r);
    CHECK_EQ(r[0], 5);
    CHECK_EQ(r[1], 3);
    CHECK_EQ(r[2], 1);
    CHECK_EQ(fake_uart_baud, 9600);

    /* apply: reply still comes from the old address */
    int inits = fake_uart_inits;
    write_reg(1, 0x010F, 1);
    CHECK_EQ(rsp_len, 5);
    CHECK_EQ(fake_tx[0], 1);
    CHECK_EQ(fake_uart_inits, inits + 1);
    CHECK_EQ(fake_uart_baud, 38400);
    CHECK_EQ(fake_uart_parity, CONFIG_PARITY_EVEN);

    read_regs(1, 0x04, 0, 1, r);
    CHECK_EQ(rsp_len, -1);
    read_regs(5, 0x04, 0x0003, 1, r);
    CHECK_EQ(r[0], 0); /* no longer defaults */

    /* settings survive a reboot */
    boot();
    CHECK_EQ(config.slave_addr, 5);
    CHECK_EQ(fake_uart_baud, 38400);

    /* invalid command */
    write_reg(5, 0x010F, 7);
    CHECK_EQ(last_exception(), 3);

    /* restore defaults */
    write_reg(5, 0x010F, 2);
    CHECK_EQ(rsp_len, 5);
    CHECK_EQ(fake_uart_baud, 9600);
    read_regs(1, 0x04, 0x0003, 1, r);
    CHECK_EQ(r[0], 0x0002);
    boot();
    CHECK_EQ(config.slave_addr, 1);
}

static void test_comm_timeout(void)
{
    fresh_boot();
    uint16_t v;

    write_reg(1, 0x0000, 2);
    CHECK_EQ(rsp_len, 5);
    write_coils(1, 0, 4, 0x0F);
    run_for(1999);
    CHECK_EQ(relay_get_mask(), 0x0F);

    /* reads do not retrigger */
    read_regs(1, 0x04, 0x0002, 1, &v);
    run_for(1);
    CHECK_EQ(relay_get_mask(), 0x00);
    read_regs(1, 0x04, 0x0003, 1, &v);
    CHECK_EQ(v & 0x0001, 1);
    CHECK(mb_server_comm_timeout());

    /* a control write clears the flag */
    write_coil(1, 1, 0xFF00);
    read_regs(1, 0x04, 0x0003, 1, &v);
    CHECK_EQ(v & 0x0001, 0);
    CHECK_EQ(relay_get_mask(), 0x02);

    /* timeout value is persistent */
    boot();
    CHECK_EQ(config.comm_timeout_s, 2);
}

static void test_flash_log(void)
{
    fresh_boot();
    int erases = fake_flash_erases;

    /* first boot with empty flash: defaults are not written */
    run_for(100);
    CHECK_EQ(fake_flash_erases, erases);
    uint16_t v = 0;
    read_regs(1, 0x04, 0x0003, 1, &v);
    CHECK_EQ(v & 0x0008, 0);

    /*
     * Blank flash: the first page is not erased. Then exactly one erase per
     * page switch (an erase stalls the UART, see config.c). Time advances
     * between the writes, so the rate limit does not defer them.
     */
    const uint16_t per_page = FLASH_PAGE_SIZE / CFG_RECORD_SIZE;
    for (uint16_t i = 1; i <= per_page; i++) {
        fake_tick += CFG_WRITE_REFILL_MS;
        write_reg(1, 0x0000, i);
    }
    CHECK_EQ(fake_flash_erases, erases);
    fake_tick += CFG_WRITE_REFILL_MS;
    write_reg(1, 0x0000, per_page + 1); /* to page 1, blank */
    CHECK_EQ(fake_flash_erases, erases);
    for (uint16_t i = per_page + 2; i <= 2 * per_page + 1; i++) {
        fake_tick += CFG_WRITE_REFILL_MS;
        write_reg(1, 0x0000, i);
    }
    CHECK_EQ(fake_flash_erases, erases + 1); /* back to page 0 */
    boot();
    CHECK_EQ(config.comm_timeout_s, 2 * per_page + 1);

    /* many changes: wraps across both pages several times */
    for (uint16_t i = 1; i <= 300; i++) {
        fake_tick += CFG_WRITE_REFILL_MS;
        write_reg(1, 0x0000, i);
        CHECK_EQ(rsp_len, 5);
    }
    CHECK(fake_flash_erases > erases);
    CHECK(!config_flash_error());
    boot();
    CHECK_EQ(config.comm_timeout_s, 300);

    /* unchanged value: no flash write */
    erases = fake_flash_erases;
    for (int i = 0; i < 100; i++) {
        write_reg(1, 0x0000, 300);
    }
    CHECK_EQ(fake_flash_erases, erases);
}

/* Settings from the heater variant: serial settings kept, rest defaults */
static void test_variant_switch(void)
{
    fake_flash_erase_all();
    put_config_record(0, 3, PROFILE_ID_HEATER, 9, CONFIG_BAUD_19200, CONFIG_PARITY_ODD, 60, 30, 5);
    boot();
    CHECK_EQ(config.slave_addr, 9);
    CHECK_EQ(config.baud_idx, CONFIG_BAUD_19200);
    CHECK_EQ(config.parity, CONFIG_PARITY_ODD);
    CHECK_EQ(config.comm_timeout_s, 0);
    CHECK_EQ(config.hold_time_s, 60);
    CHECK_EQ(config.hysteresis, 0);
    run_for(1);
    CHECK_EQ(__config_start__[CFG_RECORD_SIZE + CFG_RECORD_VARIANT], PROFILE_ID_PLAIN);

    /* own variant: taken as is */
    fake_flash_erase_all();
    put_config_record(0, 3, PROFILE_ID_PLAIN, 9, CONFIG_BAUD_19200, CONFIG_PARITY_ODD, 60, 30, 5);
    boot();
    CHECK_EQ(config.comm_timeout_s, 60);
    CHECK_EQ(config.hold_time_s, 30);
    CHECK_EQ(config.hysteresis, 5);
    run_for(1);
    CHECK_EQ(__config_start__[CFG_RECORD_SIZE], 0xFF); /* nothing rewritten */
}

/*
 * Flash write rate limit: a burst of writes lands at once, further changes
 * are deferred (status bit 4) and written once a minute, latest value wins.
 */
static void test_write_rate_limit(void)
{
    fresh_boot();
    uint16_t v = 0;

    for (uint16_t i = 1; i <= CFG_WRITE_BURST; i++) {
        write_reg(1, 0x0000, i);
    }
    read_regs(1, 0x04, 0x0003, 1, &v);
    CHECK_EQ(v & 0x0010, 0);
    CHECK_EQ(__config_start__[(CFG_WRITE_BURST - 1) * CFG_RECORD_SIZE], 0xF2);
    CHECK_EQ(__config_start__[CFG_WRITE_BURST * CFG_RECORD_SIZE], 0xFF);

    write_reg(1, 0x0000, 100);
    CHECK_EQ(rsp_len, 5);
    read_regs(1, 0x04, 0x0003, 1, &v);
    CHECK_EQ(v & 0x0010, 0x0010); /* deferred */
    CHECK_EQ(__config_start__[CFG_WRITE_BURST * CFG_RECORD_SIZE], 0xFF);
    write_reg(1, 0x0000, 101);
    run_for(CFG_WRITE_REFILL_MS - 100);
    CHECK(config_save_pending());
    run_for(100);
    CHECK(!config_save_pending());
    read_regs(1, 0x04, 0x0003, 1, &v);
    CHECK_EQ(v & 0x0010, 0);
    CHECK_EQ(__config_start__[CFG_WRITE_BURST * CFG_RECORD_SIZE], 0xF2);
    CHECK_EQ(__config_start__[(CFG_WRITE_BURST + 1) * CFG_RECORD_SIZE], 0xFF); /* one record for both */
    boot();
    CHECK_EQ(config.comm_timeout_s, 101);

    /* a reboot resets the bucket */
    write_reg(1, 0x0000, 102);
    CHECK(!config_save_pending());
}

/* Records of the 16-byte pre-release format (magic 0xC0F1) are not read */
static void test_old_record_format(void)
{
    fake_flash_erase_all();
    uint8_t r[16] = { 0xF1, 0xC0, 1, 0, 9, CONFIG_BAUD_19200, CONFIG_PARITY_ODD, PROFILE_ID_PLAIN, 60, 0, 30, 0, 5, 0 };
    uint16_t crc = nmbs_crc_calc(r, 14, NULL);
    r[14] = crc & 0xFF;
    r[15] = crc >> 8;
    memcpy(__config_start__, r, sizeof(r));
    boot();
    CHECK(config_defaults_used());
    CHECK_EQ(config.slave_addr, 1);
    /* the next write starts a fresh log: both pages erased */
    int erases = fake_flash_erases;
    write_reg(1, 0x0000, 7);
    CHECK_EQ(fake_flash_erases, erases + 1); /* page 0; page 1 is blank */
    boot();
    CHECK_EQ(config.comm_timeout_s, 7);
}

/* CRC errors are counted for frames to this device and broadcasts only */
static void test_crc_counter(void)
{
    fresh_boot();
    uint16_t v = 0;
    const uint8_t pdu[] = { 0x04, 0, 0, 0, 1 };

    transact_bad_crc(1, pdu, sizeof(pdu));
    CHECK_EQ(rsp_len, -1);
    read_regs(1, 0x04, 0x0006, 1, &v);
    CHECK_EQ(v, 1);
    transact_bad_crc(0, pdu, sizeof(pdu));
    read_regs(1, 0x04, 0x0006, 1, &v);
    CHECK_EQ(v, 2);
    transact_bad_crc(2, pdu, sizeof(pdu)); /* another slave's frame */
    read_regs(1, 0x04, 0x0006, 1, &v);
    CHECK_EQ(v, 2);
}

int main(void)
{
    RUN(test_ident);
    RUN(test_coils);
    RUN(test_mask_and_pulse);
    RUN(test_addressing);
    RUN(test_serial_settings);
    RUN(test_comm_timeout);
    RUN(test_flash_log);
    RUN(test_variant_switch);
    RUN(test_write_rate_limit);
    RUN(test_old_record_format);
    RUN(test_crc_counter);

    printf(test_failures ? "%d FAILURE(S)\n" : "all passed\n", test_failures);
    return test_failures ? 1 : 0;
}
