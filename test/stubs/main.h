/* SPDX-License-Identifier: GPL-3.0-or-later */
/* Host test replacement for the CubeMX main.h: just enough HAL */
#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>

typedef enum { HAL_OK = 0, HAL_ERROR = 1 } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET = 1 } GPIO_PinState;
typedef struct { uint16_t odr; } GPIO_TypeDef;

extern GPIO_TypeDef fake_gpiob;
extern GPIO_TypeDef fake_gpioc;

#define REL_1_Pin 0x0001
#define REL_2_Pin 0x0002
#define REL_3_Pin 0x0004
#define REL_4_Pin 0x0008
#define REL_1_GPIO_Port (&fake_gpiob)
#define REL_2_GPIO_Port (&fake_gpiob)
#define REL_3_GPIO_Port (&fake_gpiob)
#define REL_4_GPIO_Port (&fake_gpiob)
#define LED_STATE_Pin 0x2000
#define LED_STATE_GPIO_Port (&fake_gpioc)

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
uint32_t HAL_GetTick(void);
void Error_Handler(void);

/* Flash */
#define FLASH_PAGE_SIZE 1024
#define FLASH_TYPEERASE_PAGES 0
#define FLASH_TYPEPROGRAM_HALFWORD 1
typedef struct {
    uint32_t TypeErase;
    uintptr_t PageAddress; /* uint32_t on target */
    uint32_t NbPages;
} FLASH_EraseInitTypeDef;

HAL_StatusTypeDef HAL_FLASH_Unlock(void);
HAL_StatusTypeDef HAL_FLASH_Lock(void);
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t type, uintptr_t address, uint64_t data);
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *erase, uint32_t *page_error);

#endif
