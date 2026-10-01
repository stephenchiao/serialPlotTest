#ifndef TEST_USBD_DEF_H
#define TEST_USBD_DEF_H

#include <stdint.h>

#define USBD_OK 0U
#define USBD_BUSY 1U
#define USBD_FAIL 2U

typedef struct
{
    void *pClassData;
} USBD_HandleTypeDef;

#endif /* TEST_USBD_DEF_H */
