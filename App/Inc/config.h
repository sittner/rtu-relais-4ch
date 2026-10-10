/* SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef CONFIG_H
#define CONFIG_H

#include <stdbool.h>
#include <stdint.h>

/* Persistent settings, shared by all variants */
typedef struct {
    uint8_t slave_addr;      /* 1..247 */
    uint8_t baud_idx;        /* index into config_baud_rates[] */
    uint8_t parity;          /* CONFIG_PARITY_* */
    uint16_t comm_timeout_s; /* 0 = off */
    uint16_t hold_time_s;    /* heater only */
    uint16_t hysteresis;     /* heater only, 0.1 % */
} config_t;

#define CONFIG_ADDR_MIN 1
#define CONFIG_ADDR_MAX 247

/* Baud rate index, see config_baud_rates[] */
enum {
    CONFIG_BAUD_4800,
    CONFIG_BAUD_9600,
    CONFIG_BAUD_19200,
    CONFIG_BAUD_38400,
    CONFIG_BAUD_57600,
    CONFIG_BAUD_115200,
    CONFIG_BAUD_128000,
    CONFIG_BAUD_256000,
    CONFIG_BAUD_COUNT
};

#define CONFIG_PARITY_NONE 0
#define CONFIG_PARITY_EVEN 1
#define CONFIG_PARITY_ODD  2

extern const uint32_t config_baud_rates[CONFIG_BAUD_COUNT];

/* Active settings */
extern config_t config;

/* Load settings from flash, or the variant defaults if none are stored */
void config_init(void);

/* Variant defaults */
void config_get_defaults(config_t *cfg);

/* Write the active settings to flash if they differ from the stored ones
   (cheap no-op otherwise). The write rate is limited (see config.c): a
   deferred write is retried on the next calls. After a flash error no
   further writes are tried. Returns false on a flash error. */
bool config_save(void);

/* The active settings differ from the stored ones: a write is deferred by
   the rate limit (status bit 4) */
bool config_save_pending(void);

/* The active settings equal the variant defaults */
bool config_defaults_used(void);
void config_set_defaults(void);

bool config_flash_error(void);

#endif
