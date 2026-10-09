#ifndef PROFILE_H
#define PROFILE_H

#include <stdint.h>

/* Variant IDs, reported in input register 0x0001 */
#define PROFILE_ID_PLAIN  1
#define PROFILE_ID_HEATER 2

/* Variant interface, extended with the Modbus handlers later */
typedef struct {
    uint16_t id;
    const char *name;
} profile_t;

/* Provided by profile_plain.c or profile_heater.c */
extern const profile_t profile;

#endif
