#ifndef PROFILE_H
#define PROFILE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "nanomodbus.h"

/* Variant IDs, reported in input register 0x0001 */
#define PROFILE_ID_PLAIN  1
#define PROFILE_ID_HEATER 2

/* Register range handled by the variant */
#define PROFILE_REG_FIRST 0x0010
#define PROFILE_REG_LAST  0x00FF

/*
 * Variant interface. The register handlers are only called for addresses
 * in PROFILE_REG_FIRST..PROFILE_REG_LAST. They return NMBS_ERROR_NONE or a
 * Modbus exception.
 */
typedef struct {
    uint16_t id;
    const char *name;

    void (*init)(void);
    void (*poll)(uint32_t now_ms);
    /* Communication timeout tripped */
    void (*safe_state)(void);

    /* Coil writes (addresses 0..RELAY_COUNT-1, range already checked).
       NULL: coils are read only, FC 05/0F give exception 01. */
    nmbs_error (*write_coils)(uint16_t addr, uint16_t qty, const nmbs_bitfield coils);

    nmbs_error (*read_hreg)(uint16_t addr, uint16_t *val);
    /* apply = false: only validate. *control is set if the write retriggers
       the communication timeout. */
    nmbs_error (*write_hreg)(uint16_t addr, uint16_t val, bool apply, bool *control);
    nmbs_error (*read_ireg)(uint16_t addr, uint16_t *val);

    /* Variant status flags, bits 8..15 of input register 0x0003 */
    uint16_t (*status_flags)(void);
} profile_t;

/* Provided by profile_plain.c or profile_heater.c */
extern const profile_t profile;

#endif
