#ifndef RELAY_H
#define RELAY_H

#include <stdbool.h>
#include <stdint.h>

#define RELAY_COUNT    4
#define RELAY_MASK_ALL 0x0F

/* Switch all relays off */
void relay_init(void);

/* Set all relays at once, bit0 = relay 1, 1 = energized */
void relay_set_mask(uint8_t mask);
uint8_t relay_get_mask(void);

void relay_set(uint8_t ch, bool on);
bool relay_get(uint8_t ch);

#endif
