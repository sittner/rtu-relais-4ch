/* Fake HAL, flash, UART transport and app hooks for host tests */
#include "fake_hal.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "main.h"
#include "rs485.h"

GPIO_TypeDef fake_gpiob = { 0xFFFF };
GPIO_TypeDef fake_gpioc = { 0xFFFF };

uint32_t fake_tick;
bool fake_wdt_reset;
uint32_t fake_uptime;

/* Settings flash, address = offset into this array */
uint8_t __config_start__[2 * FLASH_PAGE_SIZE];
int fake_flash_erases;

uint8_t fake_rx[512];
size_t fake_rx_len, fake_rx_pos;
uint8_t fake_tx[512];
size_t fake_tx_len;
uint32_t fake_uart_baud;
uint8_t fake_uart_parity;
int fake_uart_inits;

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    if (state == GPIO_PIN_SET) {
        port->odr |= pin;
    } else {
        port->odr &= (uint16_t)~pin;
    }
}

uint32_t HAL_GetTick(void)
{
    return fake_tick;
}

void Error_Handler(void)
{
    fprintf(stderr, "Error_Handler called\n");
    abort();
}

HAL_StatusTypeDef HAL_FLASH_Unlock(void)
{
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FLASH_Lock(void)
{
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FLASH_Program(uint32_t type, uintptr_t address, uint64_t data)
{
    if (type != FLASH_TYPEPROGRAM_HALFWORD || address % 2) {
        return HAL_ERROR;
    }
    uint8_t *p = (uint8_t *)(uintptr_t)address;
    if (p < __config_start__ || p + 2 > __config_start__ + sizeof(__config_start__)) {
        return HAL_ERROR;
    }
    uint16_t cur;
    memcpy(&cur, p, 2);
    if (cur != 0xFFFF) {
        return HAL_ERROR; /* not erased */
    }
    uint16_t v = (uint16_t)data;
    memcpy(p, &v, 2);
    return HAL_OK;
}

HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *erase, uint32_t *page_error)
{
    uint8_t *p = (uint8_t *)(uintptr_t)erase->PageAddress;
    *page_error = 0xFFFFFFFF;
    if (p < __config_start__ || p + erase->NbPages * FLASH_PAGE_SIZE > __config_start__ + sizeof(__config_start__)) {
        return HAL_ERROR;
    }
    memset(p, 0xFF, erase->NbPages * FLASH_PAGE_SIZE);
    fake_flash_erases++;
    return HAL_OK;
}

void fake_flash_erase_all(void)
{
    memset(__config_start__, 0xFF, sizeof(__config_start__));
}

/* rs485 replacement */
void rs485_init(uint32_t baud, uint8_t parity)
{
    fake_uart_baud = baud;
    fake_uart_parity = parity;
    fake_uart_inits++;
}

int32_t rs485_read(uint8_t *buf, uint16_t count, int32_t timeout_ms, void *arg)
{
    (void)timeout_ms;
    (void)arg;
    uint16_t n = 0;
    while (n < count && fake_rx_pos < fake_rx_len) {
        buf[n++] = fake_rx[fake_rx_pos++];
    }
    return n;
}

int32_t rs485_write(const uint8_t *buf, uint16_t count, int32_t timeout_ms, void *arg)
{
    (void)timeout_ms;
    (void)arg;
    memcpy(fake_tx + fake_tx_len, buf, count);
    fake_tx_len += count;
    return count;
}

void rs485_flush(void)
{
    fake_rx_pos = fake_rx_len;
}

int32_t rs485_byte_timeout_ms(void)
{
    return 20;
}

uint16_t rs485_rx_error_count(void)
{
    return 0;
}

uint16_t rs485_rx_overflow_count(void)
{
    return 0;
}

/* app hooks */
bool app_wdt_reset(void)
{
    return fake_wdt_reset;
}

uint32_t app_uptime_s(void)
{
    return fake_uptime;
}
