/* SPDX-License-Identifier: GPL-3.0-or-later */
/* Minimal test helpers: assertions and Modbus RTU transactions */
#ifndef TEST_COMMON_H
#define TEST_COMMON_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "fake_hal.h"
#include "mb_server.h"

extern int test_failures;

#define CHECK(cond)                                                           \
    do {                                                                      \
        if (!(cond)) {                                                        \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);          \
            test_failures++;                                                  \
        }                                                                     \
    } while (0)

#define CHECK_EQ(a, b)                                                        \
    do {                                                                      \
        long _a = (long)(a), _b = (long)(b);                                  \
        if (_a != _b) {                                                       \
            printf("  FAIL %s:%d: %s == %ld, expected %ld\n", __FILE__,       \
                   __LINE__, #a, _a, _b);                                     \
            test_failures++;                                                  \
        }                                                                     \
    } while (0)

#define RUN(fn)                                                               \
    do {                                                                      \
        printf("%s\n", #fn);                                                  \
        fn();                                                                 \
    } while (0)

/* Response PDU of the last transaction (without address and CRC) */
extern uint8_t rsp[256];
extern int rsp_len; /* -1: no response */

/* Send a request PDU to unit, run one server poll, decode the reply */
void transact(uint8_t unit, const uint8_t *pdu, int len);
/* Same, with a corrupted CRC */
void transact_bad_crc(uint8_t unit, const uint8_t *pdu, int len);

/* Helpers building common requests, all return the response length */
int read_regs(uint8_t unit, uint8_t fc, uint16_t addr, uint16_t qty, uint16_t *out);
int write_reg(uint8_t unit, uint16_t addr, uint16_t val);
int write_regs(uint8_t unit, uint16_t addr, uint16_t qty, const uint16_t *vals);
int write_coil(uint8_t unit, uint16_t addr, uint16_t val);
int write_coils(uint8_t unit, uint16_t addr, uint16_t qty, uint8_t bits);
int read_coils(uint8_t unit, uint16_t addr, uint16_t qty, uint8_t *bits);

/* Exception code of the last response, 0 if none */
int last_exception(void);

/* Settings record layout (config.c), slot = offset / CFG_RECORD_SIZE */
#define CFG_RECORD_SIZE    32
#define CFG_RECORD_VARIANT 3 /* offset of the variant byte */

/* Write a settings record into page 0 of the fake flash, as another
   firmware (variant) would have */
void put_config_record(uint16_t slot, uint16_t seq, uint8_t variant, uint8_t addr, uint8_t baud, uint8_t parity,
                       uint16_t timeout, uint16_t hold, uint16_t hyst);

/* Flash write rate limit of config.c */
#define CFG_WRITE_BURST     32
#define CFG_WRITE_REFILL_MS 60000

/* Advance fake time and run the main loop pieces */
void run_for(uint32_t ms);

#endif
