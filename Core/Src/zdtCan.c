/*
 * zdtCan.c
 *
 *  Created on: Feb 7, 2026
 *      Author: steph
 */
#include "zdtCan.h"

extern CAN_HandleTypeDef hcan1;
static ZDT_CAN_RxCallback_t pRxCallback = NULL;

// 配置过滤器：允许所有扩展帧通过
void ZDT_CAN_ConfigFilter(void) {
    CAN_FilterTypeDef sFilterConfig;

    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;

    // ID和Mask都设为0，表示接收总线上所有数据
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;

    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        Error_Handler();
    }
}

void ZDT_CAN_RegisterCallback(ZDT_CAN_RxCallback_t callback) {
    pRxCallback = callback;
}

uint8_t ZDT_CAN_Send_ExtId(uint32_t ExtId, uint8_t *Data, uint8_t Len) {
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;

    TxHeader.StdId = 0;
    TxHeader.ExtId = ExtId;
    TxHeader.IDE = CAN_ID_EXT;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = Len;
    TxHeader.TransmitGlobalTime = DISABLE;

    // 等待邮箱空闲（简易超时机制）
    uint32_t tick = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
        if (HAL_GetTick() - tick > 10) return 1; // 超时
    }

    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, Data, &TxMailbox) != HAL_OK) {
        return 2; // 发送失败
    }
    return 0; // 成功
}

// 供 main.c 的 HAL 回调调用
void ZDT_CAN_RxFIFO0_Handler(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
        if (pRxCallback != NULL) {
            pRxCallback(RxHeader.ExtId, RxData, RxHeader.DLC);
        }
    }
}
