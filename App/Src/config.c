/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "config.h"

#include <stddef.h>
#include <string.h>

#include "main.h"
#include "nanomodbus.h"
#include "profile.h"
#include "util.h"

/*
 * Settings log in the last two flash pages (see App/STM32F030C8Tx_app.ld).
 * Records are appended to the active page. When it is full, the record
 * goes to the start of the other page, which is erased first. The old page
 * is left as it is: at boot the valid record with the highest sequence
 * number wins, and it is erased at the next page switch.
 *
 * An erase stalls the CPU for 20 to 40 ms and UART bytes are lost meanwhile,
 * so erase only when needed (one erase per RECORDS_PER_PAGE writes).
 *
 * Each record carries a format version and the variant ID. Reserved fields
 * are left erased (0xFFFF), so a later version can add a field that reads
 * as "unset" in older records without a new layout. After flashing the
 * other variant, the serial settings are kept (the device stays reachable),
 * all others are reset to the variant defaults: the heater must not inherit
 * the plain comm timeout of 0 (off).
 *
 * Write rate limit (token bucket): WRITE_BURST writes in quick succession,
 * then one per WRITE_REFILL_MS. A master that toggles a persistent register
 * cyclically would otherwise wear the pages out within days. A deferred
 * write is retried on every config_save() call (every main loop pass) and
 * reported by config_save_pending(). The latest settings win.
 */

#define PAGE_SIZE        FLASH_PAGE_SIZE
#define RECORD_MAGIC     0xC0F2u /* 0xC0F1 was the 16-byte pre-release format */
#define RECORD_VERSION   1
#define RECORD_HALFWORDS 16
#define RECORD_SIZE      (RECORD_HALFWORDS * 2)
#define RECORDS_PER_PAGE (PAGE_SIZE / RECORD_SIZE)
#define FLASH_ERASED     0xFFFFu

#define WRITE_BURST     32
#define WRITE_REFILL_MS (60u * MS_PER_S)

#ifdef FW_PROFILE_HEATER
#define CONFIG_VARIANT PROFILE_ID_HEATER
#else
#define CONFIG_VARIANT PROFILE_ID_PLAIN
#endif

/* Defaults, the comm timeout differs per variant */
#define DEFAULT_SLAVE_ADDR 1
#define DEFAULT_BAUD_IDX   CONFIG_BAUD_9600
#define DEFAULT_PARITY     CONFIG_PARITY_NONE
#ifdef FW_PROFILE_HEATER
#define DEFAULT_COMM_TIMEOUT_S 60
#else
#define DEFAULT_COMM_TIMEOUT_S 0 /* off */
#endif
#define DEFAULT_HOLD_TIME_S 60
#define DEFAULT_HYSTERESIS  0

typedef struct {
    uint16_t magic;
    uint8_t version; /* RECORD_VERSION: layout of the fields below */
    uint8_t variant; /* PROFILE_ID_* of the firmware that wrote the record */
    uint16_t seq;
    uint8_t slave_addr;
    uint8_t baud_idx;
    uint8_t parity;
    uint8_t reserved8;     /* erased */
    uint16_t comm_timeout_s;
    uint16_t hold_time_s;
    uint16_t hysteresis;
    uint16_t reserved[7];  /* erased */
    uint16_t crc;          /* Modbus CRC16 over everything before */
} record_t;

_Static_assert(sizeof(record_t) == RECORD_SIZE, "record size");

extern const uint8_t __config_start__[];

const uint32_t config_baud_rates[CONFIG_BAUD_COUNT] = {
    [CONFIG_BAUD_4800] = 4800,
    [CONFIG_BAUD_9600] = 9600,
    [CONFIG_BAUD_19200] = 19200,
    [CONFIG_BAUD_38400] = 38400,
    [CONFIG_BAUD_57600] = 57600,
    [CONFIG_BAUD_115200] = 115200,
    [CONFIG_BAUD_128000] = 128000,
    [CONFIG_BAUD_256000] = 256000,
};

config_t config;

static config_t stored;        /* what is in flash (defaults if nothing) */
static bool log_valid;         /* a valid record exists */
static bool stored_foreign;    /* newest record is from another variant */
static uint8_t active_page;    /* page holding the newest record */
static uint16_t next_slot;     /* next free slot in active_page */
static uint16_t seq;           /* sequence number of the newest record */
static bool flash_error;
static uint8_t write_tokens;   /* rate limit, see above */
static uint32_t refill_ms;     /* time of the last token refill */

static const record_t *slot_ptr(uint8_t page, uint16_t slot)
{
    return (const record_t *)(__config_start__ + page * PAGE_SIZE + slot * RECORD_SIZE);
}

/* Modbus CRC16 from nanoMODBUS */
static uint16_t record_crc(const record_t *r)
{
    return nmbs_crc_calc((const uint8_t *)r, offsetof(record_t, crc), NULL);
}

static bool record_valid(const record_t *r)
{
    return r->magic == RECORD_MAGIC && r->version == RECORD_VERSION && r->crc == record_crc(r);
}

static bool slot_erased(const record_t *r)
{
    const uint16_t *hw = (const uint16_t *)r;
    for (uint8_t i = 0; i < RECORD_HALFWORDS; i++) {
        if (hw[i] != FLASH_ERASED) {
            return false;
        }
    }
    return true;
}

/* Field-wise compare, config_t has padding */
static bool config_equal(const config_t *a, const config_t *b)
{
    return a->slave_addr == b->slave_addr && a->baud_idx == b->baud_idx && a->parity == b->parity &&
           a->comm_timeout_s == b->comm_timeout_s && a->hold_time_s == b->hold_time_s &&
           a->hysteresis == b->hysteresis;
}

static bool settings_valid(const config_t *c)
{
    return c->slave_addr >= CONFIG_ADDR_MIN && c->slave_addr <= CONFIG_ADDR_MAX &&
           c->baud_idx < CONFIG_BAUD_COUNT && c->parity <= CONFIG_PARITY_ODD;
}

void config_get_defaults(config_t *cfg)
{
    cfg->slave_addr = DEFAULT_SLAVE_ADDR;
    cfg->baud_idx = DEFAULT_BAUD_IDX;
    cfg->parity = DEFAULT_PARITY;
    cfg->comm_timeout_s = DEFAULT_COMM_TIMEOUT_S;
    cfg->hold_time_s = DEFAULT_HOLD_TIME_S;
    cfg->hysteresis = DEFAULT_HYSTERESIS;
}

void config_init(void)
{
    const record_t *newest = NULL;

    write_tokens = WRITE_BURST;
    refill_ms = HAL_GetTick();

    for (uint8_t page = 0; page < 2; page++) {
        for (uint16_t slot = 0; slot < RECORDS_PER_PAGE; slot++) {
            const record_t *r = slot_ptr(page, slot);
            if (!record_valid(r)) {
                continue;
            }
            if (newest == NULL || (int16_t)(r->seq - newest->seq) > 0) {
                newest = r;
                active_page = page;
                next_slot = slot + 1;
            }
        }
    }

    if (newest != NULL) {
        config_t c = {
            .slave_addr = newest->slave_addr,
            .baud_idx = newest->baud_idx,
            .parity = newest->parity,
            .comm_timeout_s = newest->comm_timeout_s,
            .hold_time_s = newest->hold_time_s,
            .hysteresis = newest->hysteresis,
        };
        if (settings_valid(&c)) {
            seq = newest->seq;
            stored = c;
            log_valid = true;
            config = c;
            stored_foreign = newest->variant != CONFIG_VARIANT;
            if (stored_foreign) {
                /* Keep the serial settings, defaults for the rest */
                config_t def;
                config_get_defaults(&def);
                config.comm_timeout_s = def.comm_timeout_s;
                config.hold_time_s = def.hold_time_s;
                config.hysteresis = def.hysteresis;
            }
            return;
        }
    }

    /* Nothing usable stored: start a fresh log on the next write */
    active_page = 0;
    next_slot = RECORDS_PER_PAGE;
    seq = 0;
    log_valid = false;
    stored_foreign = false;
    config_get_defaults(&config);
    stored = config;
}

static bool page_blank(uint8_t page)
{
    for (uint16_t slot = 0; slot < RECORDS_PER_PAGE; slot++) {
        if (!slot_erased(slot_ptr(page, slot))) {
            return false;
        }
    }
    return true;
}

static bool erase_page(uint8_t page)
{
    if (page_blank(page)) {
        return true;
    }

    FLASH_EraseInitTypeDef erase = {
        .TypeErase = FLASH_TYPEERASE_PAGES,
        .PageAddress = (uintptr_t)slot_ptr(page, 0),
        .NbPages = 1,
    };
    uint32_t page_error;
    return HAL_FLASHEx_Erase(&erase, &page_error) == HAL_OK;
}

static bool program_record(const record_t *slot, const record_t *rec)
{
    const uint16_t *hw = (const uint16_t *)rec;
    for (uint8_t i = 0; i < RECORD_HALFWORDS; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (uintptr_t)slot + 2 * i, hw[i]) != HAL_OK) {
            return false;
        }
    }
    return memcmp(slot, rec, RECORD_SIZE) == 0;
}

static void refill_tokens(uint32_t now_ms)
{
    if (write_tokens >= WRITE_BURST) {
        refill_ms = now_ms;
        return;
    }
    while (write_tokens < WRITE_BURST && now_ms - refill_ms >= WRITE_REFILL_MS) {
        write_tokens++;
        refill_ms += WRITE_REFILL_MS;
    }
}

bool config_save_pending(void)
{
    return !flash_error && !(config_equal(&stored, &config) && !stored_foreign);
}

bool config_save(void)
{
    if (config_equal(&stored, &config) && !stored_foreign) {
        return true;
    }
    /* Do not retry (and block in erase) on every call after a failure */
    if (flash_error) {
        return false;
    }
    refill_tokens(HAL_GetTick());
    if (write_tokens == 0) {
        return true; /* deferred, see config_save_pending() */
    }
    write_tokens--;

    record_t rec;
    memset(&rec, 0xFF, sizeof(rec)); /* reserved fields stay erased */
    rec.magic = RECORD_MAGIC;
    rec.version = RECORD_VERSION;
    rec.variant = CONFIG_VARIANT;
    rec.seq = (uint16_t)(seq + 1);
    rec.slave_addr = config.slave_addr;
    rec.baud_idx = config.baud_idx;
    rec.parity = config.parity;
    rec.comm_timeout_s = config.comm_timeout_s;
    rec.hold_time_s = config.hold_time_s;
    rec.hysteresis = config.hysteresis;
    rec.crc = record_crc(&rec);

    bool ok;
    HAL_FLASH_Unlock();
    if (next_slot < RECORDS_PER_PAGE && slot_erased(slot_ptr(active_page, next_slot))) {
        ok = program_record(slot_ptr(active_page, next_slot), &rec);
        next_slot++;
    } else {
        /* Page full (or no log yet): continue on the other page */
        uint8_t new_page = log_valid ? (uint8_t)(active_page ^ 1) : 0;
        ok = erase_page(new_page) && program_record(slot_ptr(new_page, 0), &rec);
        /*
         * Without a valid log, page 1 may hold stale records (e.g. a valid
         * CRC with invalid settings) that would win at boot: erase it too
         */
        if (ok && !log_valid) {
            ok = erase_page(1);
        }
        active_page = new_page;
        next_slot = 1;
    }
    HAL_FLASH_Lock();

    if (!ok) {
        flash_error = true;
        return false;
    }
    seq = rec.seq;
    stored = config;
    log_valid = true;
    stored_foreign = false;
    return true;
}

bool config_defaults_used(void)
{
    config_t def;
    config_get_defaults(&def);
    return config_equal(&def, &config);
}

void config_set_defaults(void)
{
    config_get_defaults(&config);
}

bool config_flash_error(void)
{
    return flash_error;
}
