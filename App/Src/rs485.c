#include "rs485.h"

#include "main.h"
#include "usart.h"

#include "config.h"

/*
 * RX: the RXNE interrupt fills a ring buffer, nanoMODBUS pulls from it.
 * TX: polled. The receiver is disabled while sending, so a possible echo
 * from the auto-direction RS485 transceiver is discarded.
 */

#define RX_BUF_SIZE 256 /* index wraps as uint8_t */

#define USART_ERR_FLAGS (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE)
#define USART_ERR_CLEAR (USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NCF | USART_ICR_PECF)

/* Minimum byte timeout, tolerates gaps from USB-RS485 adapters */
#define BYTE_TIMEOUT_MIN_MS 20
/* Upper bound for rs485_flush() */
#define FLUSH_MAX_MS 100

static uint8_t rx_buf[RX_BUF_SIZE];
static volatile uint8_t rx_head; /* written by ISR */
static volatile uint8_t rx_tail; /* written by main loop */
static volatile uint16_t rx_errors;
static volatile uint16_t rx_overflows;

static uint32_t t35_us;

void USART1_IRQHandler(void)
{
    uint32_t isr = USART1->ISR;

    if (isr & USART_ERR_FLAGS) {
        USART1->ICR = USART_ERR_CLEAR;
        rx_errors++;
    }

    if (isr & USART_ISR_RXNE) {
        /* With parity the MSB is the parity bit, dropped by the cast */
        uint8_t b = (uint8_t)USART1->RDR;
        uint8_t next = (uint8_t)(rx_head + 1);
        if (next != rx_tail) {
            rx_buf[rx_head] = b;
            rx_head = next;
        } else {
            rx_overflows++;
        }
    }
}

/* Busy wait based on SysTick (1 ms reload), thread mode only */
static void delay_us(uint32_t us)
{
    uint32_t load = SysTick->LOAD + 1;
    uint32_t remaining = us * (SystemCoreClock / 1000000);
    uint32_t last = SysTick->VAL;

    while (remaining > 0) {
        uint32_t now = SysTick->VAL;
        uint32_t elapsed = (last >= now) ? last - now : last + load - now;
        if (elapsed >= remaining) {
            break;
        }
        remaining -= elapsed;
        last = now;
    }
}

void rs485_init(uint32_t baud, uint8_t parity)
{
    HAL_NVIC_DisableIRQ(USART1_IRQn);

    huart1.Init.BaudRate = baud;
    switch (parity) {
    case CONFIG_PARITY_EVEN:
        huart1.Init.Parity = UART_PARITY_EVEN;
        break;
    case CONFIG_PARITY_ODD:
        huart1.Init.Parity = UART_PARITY_ODD;
        break;
    default:
        huart1.Init.Parity = UART_PARITY_NONE;
        break;
    }
    /* The word length includes the parity bit */
    huart1.Init.WordLength = (huart1.Init.Parity == UART_PARITY_NONE) ? UART_WORDLENGTH_8B : UART_WORDLENGTH_9B;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }

    /* Modbus t3.5: 3.5 characters of 11 bits, fixed 1750 us above 19200 baud */
    t35_us = (baud <= 19200) ? (38500000 + baud - 1) / baud : 1750;

    rx_tail = rx_head;
    USART1->ICR = USART_ERR_CLEAR;
    SET_BIT(USART1->CR3, USART_CR3_EIE);
    SET_BIT(USART1->CR1, USART_CR1_RXNEIE | USART_CR1_PEIE);

    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

int32_t rs485_read(uint8_t *buf, uint16_t count, int32_t timeout_ms, void *arg)
{
    (void)arg;
    uint16_t n = 0;
    uint32_t start = HAL_GetTick();

    while (n < count) {
        if (rx_tail != rx_head) {
            buf[n++] = rx_buf[rx_tail];
            rx_tail = (uint8_t)(rx_tail + 1);
            start = HAL_GetTick();
            continue;
        }
        if (timeout_ms >= 0 && HAL_GetTick() - start >= (uint32_t)timeout_ms) {
            break;
        }
    }
    return n;
}

int32_t rs485_write(const uint8_t *buf, uint16_t count, int32_t timeout_ms, void *arg)
{
    (void)timeout_ms;
    (void)arg;

    /* Keep t3.5 of silence after the request before answering */
    delay_us(t35_us);

    CLEAR_BIT(USART1->CR1, USART_CR1_RE);
    for (uint16_t i = 0; i < count; i++) {
        while (!(USART1->ISR & USART_ISR_TXE)) {
        }
        USART1->TDR = buf[i];
    }
    while (!(USART1->ISR & USART_ISR_TC)) {
    }
    rx_tail = rx_head;
    SET_BIT(USART1->CR1, USART_CR1_RE);

    return count;
}

void rs485_flush(void)
{
    uint32_t idle_ms = (t35_us + 999) / 1000 + 1;
    uint32_t start = HAL_GetTick();
    uint32_t last_rx = start;

    while (HAL_GetTick() - start < FLUSH_MAX_MS) {
        if (rx_tail != rx_head) {
            rx_tail = rx_head;
            last_rx = HAL_GetTick();
        } else if (HAL_GetTick() - last_rx >= idle_ms) {
            break;
        }
    }
}

int32_t rs485_byte_timeout_ms(void)
{
    uint32_t t35_ms = (t35_us + 999) / 1000;
    return (int32_t)(t35_ms > BYTE_TIMEOUT_MIN_MS ? t35_ms : BYTE_TIMEOUT_MIN_MS);
}

uint16_t rs485_rx_error_count(void)
{
    return rx_errors;
}

uint16_t rs485_rx_overflow_count(void)
{
    return rx_overflows;
}
