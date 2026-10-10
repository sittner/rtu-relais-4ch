/* SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef FAKE_HAL_H
#define FAKE_HAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

extern uint32_t fake_tick;
extern bool fake_wdt_reset;
extern uint32_t fake_uptime;
extern int fake_flash_erases;

extern uint8_t fake_rx[512];
extern size_t fake_rx_len, fake_rx_pos;
extern uint8_t fake_tx[512];
extern size_t fake_tx_len;
extern uint32_t fake_uart_baud;
extern uint8_t fake_uart_parity;
extern int fake_uart_inits;

void fake_flash_erase_all(void);

#endif
