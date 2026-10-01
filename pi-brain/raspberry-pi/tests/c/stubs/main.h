#ifndef TEST_STM32_MAIN_H
#define TEST_STM32_MAIN_H

#include <stdint.h>

typedef struct
{
    uint8_t unused;
} UART_HandleTypeDef;

typedef int HAL_StatusTypeDef;

#define HAL_OK 0
#define HAL_ERROR 1
#define HAL_BUSY 2

#define __weak __attribute__((weak))

static inline uint32_t __get_PRIMASK(void)
{
    return 0U;
}

static inline void __disable_irq(void)
{
}

static inline void __enable_irq(void)
{
}

uint32_t HAL_GetTick(void);

HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart,
                                     uint8_t *data,
                                     uint16_t length);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart,
                                   const uint8_t *data,
                                   uint16_t length,
                                   uint32_t timeout);

#endif /* TEST_STM32_MAIN_H */
