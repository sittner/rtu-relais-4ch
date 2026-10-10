/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "test_common.h"

#include "nanomodbus.h"
#include "profile.h"

int test_failures;
uint8_t rsp[256];
int rsp_len;

static uint16_t crc16(const uint8_t *d, size_t n)
{
    uint16_t crc = 0xFFFF;
    while (n--) {
        crc ^= *d++;
        for (int i = 0; i < 8; i++) {
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
        }
    }
    return crc;
}

static void transact_crc(uint8_t unit, const uint8_t *pdu, int len, bool good_crc)
{
    fake_rx[0] = unit;
    memcpy(fake_rx + 1, pdu, len);
    uint16_t crc = crc16(fake_rx, len + 1);
    if (!good_crc) {
        crc ^= 0x5A5A;
    }
    fake_rx[len + 1] = crc & 0xFF;
    fake_rx[len + 2] = crc >> 8;
    fake_rx_len = len + 3;
    fake_rx_pos = 0;
    fake_tx_len = 0;

    mb_server_poll(fake_tick);

    rsp_len = -1;
    if (fake_tx_len >= 4) {
        uint16_t c = crc16(fake_tx, fake_tx_len - 2);
        if ((c & 0xFF) == fake_tx[fake_tx_len - 2] && (c >> 8) == fake_tx[fake_tx_len - 1]) {
            rsp_len = (int)fake_tx_len - 3;
            memcpy(rsp, fake_tx + 1, rsp_len);
        } else {
            printf("  bad response CRC\n");
            test_failures++;
        }
    } else if (fake_tx_len > 0) {
        printf("  short response\n");
        test_failures++;
    }
}

void transact(uint8_t unit, const uint8_t *pdu, int len)
{
    transact_crc(unit, pdu, len, true);
}

void transact_bad_crc(uint8_t unit, const uint8_t *pdu, int len)
{
    transact_crc(unit, pdu, len, false);
}

int last_exception(void)
{
    return (rsp_len == 2 && (rsp[0] & 0x80)) ? rsp[1] : 0;
}

int read_regs(uint8_t unit, uint8_t fc, uint16_t addr, uint16_t qty, uint16_t *out)
{
    uint8_t pdu[] = { fc, addr >> 8, addr & 0xFF, qty >> 8, qty & 0xFF };
    transact(unit, pdu, sizeof(pdu));
    if (rsp_len == 2 + 2 * qty && rsp[0] == fc) {
        for (int i = 0; i < qty; i++) {
            out[i] = (uint16_t)(rsp[2 + 2 * i] << 8 | rsp[3 + 2 * i]);
        }
    }
    return rsp_len;
}

int write_reg(uint8_t unit, uint16_t addr, uint16_t val)
{
    uint8_t pdu[] = { 0x06, addr >> 8, addr & 0xFF, val >> 8, val & 0xFF };
    transact(unit, pdu, sizeof(pdu));
    return rsp_len;
}

int write_regs(uint8_t unit, uint16_t addr, uint16_t qty, const uint16_t *vals)
{
    uint8_t pdu[6 + 2 * 16] = { 0x10, addr >> 8, addr & 0xFF, qty >> 8, qty & 0xFF, (uint8_t)(2 * qty) };
    for (int i = 0; i < qty; i++) {
        pdu[6 + 2 * i] = vals[i] >> 8;
        pdu[7 + 2 * i] = vals[i] & 0xFF;
    }
    transact(unit, pdu, 6 + 2 * qty);
    return rsp_len;
}

int write_coil(uint8_t unit, uint16_t addr, uint16_t val)
{
    uint8_t pdu[] = { 0x05, addr >> 8, addr & 0xFF, val >> 8, val & 0xFF };
    transact(unit, pdu, sizeof(pdu));
    return rsp_len;
}

int write_coils(uint8_t unit, uint16_t addr, uint16_t qty, uint8_t bits)
{
    uint8_t pdu[] = { 0x0F, addr >> 8, addr & 0xFF, qty >> 8, qty & 0xFF, 1, bits };
    transact(unit, pdu, sizeof(pdu));
    return rsp_len;
}

int read_coils(uint8_t unit, uint16_t addr, uint16_t qty, uint8_t *bits)
{
    uint8_t pdu[] = { 0x01, addr >> 8, addr & 0xFF, qty >> 8, qty & 0xFF };
    transact(unit, pdu, sizeof(pdu));
    if (rsp_len == 3 && rsp[0] == 0x01) {
        *bits = rsp[2];
    }
    return rsp_len;
}

void run_for(uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++) {
        fake_tick++;
        fake_rx_len = fake_rx_pos = 0;
        mb_server_poll(fake_tick);
        profile.poll(fake_tick);
    }
}

void put_config_record(uint16_t slot, uint16_t seq, uint8_t variant, uint8_t addr, uint8_t baud, uint8_t parity,
                       uint16_t timeout, uint16_t hold, uint16_t hyst)
{
    /* Layout of record_t in config.c (little endian), reserved bytes erased */
    uint8_t r[CFG_RECORD_SIZE];
    memset(r, 0xFF, sizeof(r));
    const uint8_t head[] = {
        0xF2, 0xC0, 1, variant, seq & 0xFF, seq >> 8, addr, baud, parity, 0xFF,
        timeout & 0xFF, timeout >> 8, hold & 0xFF, hold >> 8, hyst & 0xFF, hyst >> 8,
    };
    memcpy(r, head, sizeof(head));
    uint16_t crc = nmbs_crc_calc(r, sizeof(r) - 2, NULL);
    r[sizeof(r) - 2] = crc & 0xFF;
    r[sizeof(r) - 1] = crc >> 8;
    memcpy(__config_start__ + slot * sizeof(r), r, sizeof(r));
}
