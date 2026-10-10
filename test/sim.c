/* SPDX-License-Identifier: GPL-3.0-or-later */
/*
 * Device simulator: firmware modules (Modbus server, settings, variant)
 * running on the host, serving Modbus RTU on a pseudo-terminal.
 * Used to run tools/test_device.py without hardware:
 *
 *   test/build/sim_plain &        # prints the pty path
 *   tools/test_device.py --yes --port /dev/pts/N
 *
 * Baud rate and parity are not simulated (a pty ignores them).
 */
#define _GNU_SOURCE
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "config.h"
#include "main.h"
#include "mb_server.h"
#include "profile.h"
#include "relay.h"
#include "rs485.h"

GPIO_TypeDef fake_gpiob = { 0xFFFF };
GPIO_TypeDef fake_gpioc = { 0xFFFF };
uint8_t __config_start__[2 * FLASH_PAGE_SIZE];

static int pty_fd;
static struct timespec t_start;

uint32_t HAL_GetTick(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (uint32_t)((t.tv_sec - t_start.tv_sec) * 1000 + (t.tv_nsec - t_start.tv_nsec) / 1000000);
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    uint16_t before = port->odr;
    port->odr = state == GPIO_PIN_SET ? (port->odr | pin) : (port->odr & (uint16_t)~pin);
    if (port == &fake_gpiob && before != port->odr) {
        printf("[%7u ms] relays 0x%X\n", HAL_GetTick(), (~port->odr) & 0xF);
        fflush(stdout);
    }
}

void Error_Handler(void)
{
    fprintf(stderr, "Error_Handler\n");
    exit(1);
}

HAL_StatusTypeDef HAL_FLASH_Unlock(void) { return HAL_OK; }
HAL_StatusTypeDef HAL_FLASH_Lock(void) { return HAL_OK; }

HAL_StatusTypeDef HAL_FLASH_Program(uint32_t type, uintptr_t address, uint64_t data)
{
    (void)type;
    uint16_t v = (uint16_t)data;
    memcpy((void *)address, &v, 2);
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *erase, uint32_t *page_error)
{
    *page_error = 0xFFFFFFFF;
    memset((void *)erase->PageAddress, 0xFF, erase->NbPages * FLASH_PAGE_SIZE);
    printf("[%7u ms] flash page erased\n", HAL_GetTick());
    return HAL_OK;
}

void rs485_init(uint32_t baud, uint8_t parity)
{
    printf("[%7u ms] serial %u baud, parity %u, address %u\n", HAL_GetTick(), baud, parity, config.slave_addr);
    fflush(stdout);
}

int32_t rs485_read(uint8_t *buf, uint16_t count, int32_t timeout_ms, void *arg)
{
    (void)arg;
    uint16_t n = 0;
    while (n < count) {
        struct pollfd p = { pty_fd, POLLIN, 0 };
        if (poll(&p, 1, timeout_ms < 0 ? -1 : timeout_ms) <= 0) {
            break;
        }
        ssize_t r = read(pty_fd, buf + n, count - n);
        if (r <= 0) {
            break;
        }
        n += (uint16_t)r;
    }
    return n;
}

int32_t rs485_write(const uint8_t *buf, uint16_t count, int32_t timeout_ms, void *arg)
{
    (void)timeout_ms;
    (void)arg;
    usleep(2000); /* t3.5 */
    return (int32_t)write(pty_fd, buf, count);
}

void rs485_flush(void)
{
    uint8_t b[64];
    while (rs485_read(b, sizeof(b), 5, NULL) > 0) {
    }
}

int32_t rs485_byte_timeout_ms(void) { return 20; }
uint16_t rs485_rx_error_count(void) { return 0; }
uint16_t rs485_rx_overflow_count(void) { return 0; }

bool app_wdt_reset(void) { return false; }
uint32_t app_uptime_s(void) { return HAL_GetTick() / 1000; }

int main(void)
{
    clock_gettime(CLOCK_MONOTONIC, &t_start);
    memset(__config_start__, 0xFF, sizeof(__config_start__));

    pty_fd = posix_openpt(O_RDWR | O_NOCTTY);
    if (pty_fd < 0 || grantpt(pty_fd) || unlockpt(pty_fd)) {
        perror("pty");
        return 1;
    }
    struct termios t;
    tcgetattr(pty_fd, &t);
    cfmakeraw(&t);
    tcsetattr(pty_fd, TCSANOW, &t);
    printf("simulator (%s) on %s\n", profile.name, ptsname(pty_fd));
    fflush(stdout);

    relay_init();
    config_init();
    profile.init();
    mb_server_init();

    for (;;) {
        mb_server_poll(HAL_GetTick());
        profile.poll(HAL_GetTick());
        usleep(200);
    }
}
