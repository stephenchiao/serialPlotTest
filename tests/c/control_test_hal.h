#ifndef CONTROL_TEST_HAL_H
#define CONTROL_TEST_HAL_H
#include <stdint.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY } HAL_StatusTypeDef;
typedef struct { uint32_t ESR, TSR; } CAN_TestRegisters;
typedef struct { CAN_TestRegisters *Instance; } CAN_HandleTypeDef;
typedef struct { void *Instance; uint32_t gState; } UART_HandleTypeDef;
typedef struct {
    uint32_t FilterBank, FilterMode, FilterScale, FilterIdHigh, FilterIdLow;
    uint32_t FilterMaskIdHigh, FilterMaskIdLow, FilterFIFOAssignment;
    uint32_t FilterActivation, SlaveStartFilterBank;
} CAN_FilterTypeDef;
typedef struct { uint32_t ExtId, StdId, IDE, RTR, DLC; } CAN_TxHeaderTypeDef;
typedef CAN_TxHeaderTypeDef CAN_RxHeaderTypeDef;
#define CAN_FILTERMODE_IDMASK 0U
#define CAN_FILTERSCALE_32BIT 0U
#define CAN_RX_FIFO0 0U
#define ENABLE 1U
#define CAN_IT_RX_FIFO0_MSG_PENDING 1U
#define CAN_IT_TX_MAILBOX_EMPTY 2U
#define CAN_IT_ERROR 4U
#define CAN_IT_LAST_ERROR_CODE 8U
#define CAN_IT_BUSOFF 16U
#define CAN_IT_ERROR_PASSIVE 32U
#define CAN_IT_ERROR_WARNING 64U
#define HAL_CAN_STATE_LISTENING 2U
#define HAL_CAN_ERROR_NONE 0U
#define CAN_ESR_EWGF 1U
#define CAN_ESR_EPVF 2U
#define CAN_ESR_BOFF 4U
uint32_t HAL_CAN_GetState(const CAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_CAN_ResetError(CAN_HandleTypeDef *);
#define HAL_CAN_ERROR_ACK 0x20U
#define HAL_CAN_ERROR_BOF 0x4U
#define HAL_CAN_ERROR_EPV 0x2U
#define HAL_CAN_ERROR_RX_FOV0 0x200U
#define HAL_CAN_ERROR_RX_FOV1 0x400U
#define HAL_CAN_ERROR_TX_ALST0 0x800U
#define HAL_CAN_ERROR_TIMEOUT 0x20000U
#define HAL_CAN_ERROR_NOT_INITIALIZED 0x40000U
#define HAL_CAN_ERROR_NOT_READY 0x80000U
#define HAL_CAN_ERROR_NOT_STARTED 0x100000U
#define HAL_CAN_ERROR_PARAM 0x200000U
#define HAL_CAN_ERROR_INVALID_CALLBACK 0x400000U
#define HAL_CAN_ERROR_INTERNAL 0x800000U
uint32_t HAL_CAN_GetError(CAN_HandleTypeDef *);
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *);
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *);
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *);
void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *);
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *);
#define CAN_TX_MAILBOX0 1U
#define CAN_TX_MAILBOX1 2U
#define CAN_TX_MAILBOX2 4U
#define CAN_ID_EXT 4U
#define CAN_RTR_DATA 0U
#define HAL_UART_STATE_READY 0U
#define USART2 ((void *)(uintptr_t)2U)
extern CAN_HandleTypeDef hcan1;
extern UART_HandleTypeDef huart1, huart2;
uint32_t HAL_GetTick(void);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __enable_irq(void);
void __set_PRIMASK(uint32_t value);
void Error_Handler(void);
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *, CAN_FilterTypeDef *);
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_CAN_Init(CAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *, uint32_t);
uint32_t HAL_CAN_IsTxMessagePending(CAN_HandleTypeDef *, uint32_t);
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *, CAN_TxHeaderTypeDef *, uint8_t *, uint32_t *);
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *, uint32_t, CAN_RxHeaderTypeDef *, uint8_t *);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
#endif
