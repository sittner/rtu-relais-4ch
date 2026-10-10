/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "relay.h"

#include "main.h"

/* Relay outputs are active low: GPIO_PIN_RESET = energized */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} relay_pin_t;

static const relay_pin_t relay_pins[RELAY_COUNT] = {
    { REL_1_GPIO_Port, REL_1_Pin },
    { REL_2_GPIO_Port, REL_2_Pin },
    { REL_3_GPIO_Port, REL_3_Pin },
    { REL_4_GPIO_Port, REL_4_Pin },
};

static uint8_t relay_mask;

void relay_init(void)
{
    relay_set_mask(0);
}

void relay_set_mask(uint8_t mask)
{
    mask &= RELAY_MASK_ALL;
    for (uint8_t ch = 0; ch < RELAY_COUNT; ch++) {
        HAL_GPIO_WritePin(relay_pins[ch].port, relay_pins[ch].pin,
                          (mask & (1u << ch)) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
    relay_mask = mask;
}

uint8_t relay_get_mask(void)
{
    return relay_mask;
}

void relay_set(uint8_t ch, bool on)
{
    if (ch >= RELAY_COUNT) {
        return;
    }
    if (on) {
        relay_set_mask(relay_mask | (1u << ch));
    } else {
        relay_set_mask(relay_mask & ~(1u << ch));
    }
}

bool relay_get(uint8_t ch)
{
    return ch < RELAY_COUNT && (relay_mask & (1u << ch));
}
