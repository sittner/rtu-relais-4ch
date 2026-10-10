/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "led.h"

#include <stdbool.h>

#include "main.h"
#include "util.h"

/* Blink patterns: alternating on/off durations in ms, starting with on */
typedef struct {
    const uint16_t *steps;
    uint8_t count;
} led_pattern_t;

static const uint16_t pat_normal[]  = { 500, 500 };
static const uint16_t pat_timeout[] = { 100, 150, 100, 650 };
static const uint16_t pat_ack[]     = { 50, 50 };

#define PAT(p) { p, ARRAY_SIZE(p) }
static const led_pattern_t patterns[] = {
    [LED_MODE_NORMAL]  = PAT(pat_normal),
    [LED_MODE_TIMEOUT] = PAT(pat_timeout),
};
static const led_pattern_t pattern_ack = PAT(pat_ack);

#define ACK_BLINKS 10

static led_mode_t mode = LED_MODE_NORMAL;
static const led_pattern_t *pattern = &patterns[LED_MODE_NORMAL];
static uint8_t step;
static uint8_t ack_steps_left;
static uint32_t step_start;
static bool restart = true;

static void led_write(bool on)
{
    /* Active low */
    HAL_GPIO_WritePin(LED_STATE_GPIO_Port, LED_STATE_Pin, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void led_set_mode(led_mode_t new_mode)
{
    if (new_mode == mode) {
        return;
    }
    mode = new_mode;
    if (ack_steps_left == 0) {
        pattern = &patterns[mode];
        restart = true;
    }
}

void led_ack(void)
{
    pattern = &pattern_ack;
    ack_steps_left = 2 * ACK_BLINKS;
    restart = true;
}

void led_poll(uint32_t now_ms)
{
    if (restart) {
        restart = false;
        step = 0;
        step_start = now_ms;
        led_write(true);
        return;
    }

    if (now_ms - step_start < pattern->steps[step]) {
        return;
    }
    step_start = now_ms;

    if (ack_steps_left > 0 && --ack_steps_left == 0) {
        pattern = &patterns[mode];
        step = 0;
    } else {
        step = (step + 1) % pattern->count;
    }
    /* Even steps are "on" phases */
    led_write((step & 1) == 0);
}
