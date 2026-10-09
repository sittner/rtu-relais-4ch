#include "config.h"

#include <stddef.h>
#include <string.h>

#include "main.h"

/*
 * Settings log in the last two flash pages (see App/STM32F030C8Tx_app.ld).
 * Records are appended to the active page. When it is full, the other page
 * is erased, the record is written there and the old page is erased.
 * At boot the valid record with the highest sequence number wins.
 */

#define PAGE_SIZE        FLASH_PAGE_SIZE
#define RECORD_MAGIC     0xC0F1u
#define RECORD_HALFWORDS 8
#define RECORD_SIZE      (RECORD_HALFWORDS * 2)
#define RECORDS_PER_PAGE (PAGE_SIZE / RECORD_SIZE)

typedef struct {
    uint16_t magic;
    uint16_t seq;
    uint8_t slave_addr;
    uint8_t baud_idx;
    uint8_t parity;
    uint8_t reserved;
    uint16_t comm_timeout_s;
    uint16_t hold_time_s;
    uint16_t hysteresis;
    uint16_t crc;
} record_t;

_Static_assert(sizeof(record_t) == RECORD_SIZE, "record size");

extern const uint8_t __config_start__[];

const uint32_t config_baud_rates[CONFIG_BAUD_COUNT] = {
    4800, 9600, 19200, 38400, 57600, 115200, 128000, 256000,
};

config_t config;

static config_t stored;        /* what is in flash (defaults if nothing) */
static bool log_valid;         /* a valid record exists */
static uint8_t active_page;    /* page holding the newest record */
static uint16_t next_slot;     /* next free slot in active_page */
static uint16_t seq;           /* sequence number of the newest record */
static bool flash_error;

static const record_t *slot_ptr(uint8_t page, uint16_t slot)
{
    return (const record_t *)(__config_start__ + page * PAGE_SIZE + slot * RECORD_SIZE);
}

static uint16_t crc16(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFF;
    while (len--) {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; i++) {
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
        }
    }
    return crc;
}

static bool record_valid(const record_t *r)
{
    return r->magic == RECORD_MAGIC && r->crc == crc16((const uint8_t *)r, offsetof(record_t, crc));
}

static bool slot_erased(const record_t *r)
{
    const uint16_t *hw = (const uint16_t *)r;
    for (uint8_t i = 0; i < RECORD_HALFWORDS; i++) {
        if (hw[i] != 0xFFFF) {
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
    cfg->slave_addr = 1;
    cfg->baud_idx = 1; /* 9600 */
    cfg->parity = CONFIG_PARITY_NONE;
#ifdef FW_PROFILE_HEATER
    cfg->comm_timeout_s = 60;
#else
    cfg->comm_timeout_s = 0;
#endif
    cfg->hold_time_s = 60;
    cfg->hysteresis = 0;
}

void config_init(void)
{
    const record_t *newest = NULL;

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
            return;
        }
    }

    /* Nothing usable stored: start a fresh log on the next write */
    active_page = 0;
    next_slot = RECORDS_PER_PAGE;
    seq = 0;
    log_valid = false;
    config_get_defaults(&config);
    stored = config;
}

static bool erase_page(uint8_t page)
{
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

bool config_save(void)
{
    if (config_equal(&stored, &config)) {
        return true;
    }
    /* Do not retry (and block in erase) on every call after a failure */
    if (flash_error) {
        return false;
    }

    record_t rec = {
        .magic = RECORD_MAGIC,
        .seq = (uint16_t)(seq + 1),
        .slave_addr = config.slave_addr,
        .baud_idx = config.baud_idx,
        .parity = config.parity,
        .reserved = 0,
        .comm_timeout_s = config.comm_timeout_s,
        .hold_time_s = config.hold_time_s,
        .hysteresis = config.hysteresis,
    };
    rec.crc = crc16((const uint8_t *)&rec, offsetof(record_t, crc));

    bool ok;
    HAL_FLASH_Unlock();
    if (next_slot < RECORDS_PER_PAGE && slot_erased(slot_ptr(active_page, next_slot))) {
        ok = program_record(slot_ptr(active_page, next_slot), &rec);
        next_slot++;
    } else {
        /* Page full (or no log yet): continue on the other page */
        uint8_t old_page = active_page;
        uint8_t new_page = log_valid ? (uint8_t)(active_page ^ 1) : 0;
        ok = erase_page(new_page) && program_record(slot_ptr(new_page, 0), &rec);
        /* Without a valid log, page 1 may hold stale records: erase it too */
        if (ok) {
            ok = erase_page(log_valid ? old_page : 1);
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
