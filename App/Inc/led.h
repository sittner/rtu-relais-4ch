#ifndef LED_H
#define LED_H

#include <stdint.h>

typedef enum {
    LED_MODE_NORMAL,   /* 1 Hz blink */
    LED_MODE_TIMEOUT,  /* double blink, communication timeout tripped */
} led_mode_t;

void led_set_mode(led_mode_t mode);

/* 10 fast blinks (factory reset done), then back to the current mode */
void led_ack(void);

void led_poll(uint32_t now_ms);

#endif
