#ifndef __ZDTCAN_H__
#define __ZDTCAN_H__

#include <stdint.h>
#include "main.h"

typedef void (*ZDT_CAN_RxCallback_t)(uint32_t ExtId, uint8_t *Data, uint8_t Len);

void ZDT_CAN_ConfigFilter(void);
void ZDT_CAN_RegisterCallback(ZDT_CAN_RxCallback_t callback);
uint8_t ZDT_CAN_Send_ExtId(uint32_t ExtId, uint8_t *Data, uint8_t Len);
void ZDT_CAN_RxFIFO0_Handler(CAN_HandleTypeDef *hcan);
#endif
