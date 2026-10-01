#ifndef TEST_USBD_CDC_H
#define TEST_USBD_CDC_H

#include <stdint.h>

typedef struct
{
    volatile uint32_t TxState;
} USBD_CDC_HandleTypeDef;

#endif /* TEST_USBD_CDC_H */
