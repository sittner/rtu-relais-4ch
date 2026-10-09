#ifndef RS485_H
#define RS485_H

#include <stdint.h>

/* (Re)configure USART1 and start interrupt driven reception.
   parity: CONFIG_PARITY_* */
void rs485_init(uint32_t baud, uint8_t parity);

/* nanoMODBUS platform callbacks: read up to count bytes, waiting at most
   timeout_ms for each byte (0 = return immediately, < 0 = forever) */
int32_t rs485_read(uint8_t *buf, uint16_t count, int32_t timeout_ms, void *arg);
int32_t rs485_write(const uint8_t *buf, uint16_t count, int32_t timeout_ms, void *arg);

/* Discard received bytes until the line has been idle for t3.5 */
void rs485_flush(void);

/* Byte timeout for nanoMODBUS at the current baud rate */
int32_t rs485_byte_timeout_ms(void);

/* Diagnostic counters */
uint16_t rs485_rx_error_count(void);    /* framing, noise, parity, overrun */
uint16_t rs485_rx_overflow_count(void); /* ring buffer full */

#endif
