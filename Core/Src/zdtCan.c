#include "zdtCan.h"

extern CAN_HandleTypeDef hcan1;
static ZDT_CAN_RxCallback_t pRxCallback = NULL;

void ZDT_CAN_ConfigFilter(void) {
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0004;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0004;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
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
    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, Data, &TxMailbox) != HAL_OK) {
        return 2;
    }
    return 0;
}
