/* SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef APP_H
#define APP_H

#include <stdbool.h>
#include <stdint.h>

void app_init(void);
void app_loop(void);

/* Last reset was caused by the watchdog */
bool app_wdt_reset(void);

uint32_t app_uptime_s(void);

#endif
