/* Hardware-independent fault injection; never a real motor/bus validation. */
#include "can_test_hal.h"
#include "zdtEmm.h"
#include "zdtCan.h"
#include <assert.h>
#include <string.h>
static CAN_TypeDef instance;
CAN_HandleTypeDef hcan1 = {&instance, HAL_CAN_STATE_READY, 0U};
TestCanFrame test_can_frames[4096];
unsigned test_can_count, test_can_start_calls, test_can_abort_calls;
uint8_t test_can_init_fail = 1U, test_can_send_fail, test_can_abort_fail, test_can_pending;
static uint8_t rx_pending, rx_data[8];
static CAN_RxHeaderTypeDef rx_header;
void TestCan_Reset(void)
{
    instance.MCR = 0x40U; instance.BTR = 0x0003000DU; instance.ESR = 0U;
    hcan1.State = HAL_CAN_STATE_READY; hcan1.ErrorCode = 0U;
    test_can_count = test_can_start_calls = test_can_abort_calls = 0U;
    test_can_init_fail = test_can_send_fail = test_can_abort_fail = test_can_pending = rx_pending = 0U;
    ZDT_CAN_TestReset();
}
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *h, CAN_FilterTypeDef *f)
{
    assert(h == &hcan1 && f->FilterBank == 0U && f->SlaveStartFilterBank == 14U);
    assert(f->FilterIdLow == 4U && f->FilterMaskIdLow == 6U);
    assert(f->FilterIdHigh == 0U && f->FilterMaskIdHigh == 0U);
    return test_can_init_fail == 1U ? HAL_ERROR : HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *h)
{
    test_can_start_calls++;
    if (test_can_init_fail == 2U) return HAL_ERROR;
    h->State = HAL_CAN_STATE_LISTENING; return HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *h, uint32_t mask)
{
    (void)h;
    assert((mask & (CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_BUSOFF)) ==
           (CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_BUSOFF));
    return test_can_init_fail == 3U ? HAL_ERROR : HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *h, CAN_TxHeaderTypeDef *header,
                                      uint8_t *data, uint32_t *mailbox)
{
    assert(h == &hcan1 && !test_can_pending && __get_PRIMASK() == 1U);
    if (test_can_send_fail) return HAL_ERROR;
    assert(test_can_count < 4096U && header->DLC <= 8U);
    assert(data[0] != 0xF3U); /* No motor enable/disable command in any scenario. */
    TestCanFrame *f = &test_can_frames[test_can_count++];
    f->header = *header; memset(f->data, 0, 8U);
    /* Match the real STM32 HAL, which reads eight bytes regardless of DLC. */
    memcpy(f->data, data, 8U); f->tick = HAL_GetTick();
    for (unsigned i = header->DLC; i < 8U; ++i) assert(f->data[i] == 0U);
    *mailbox = CAN_TX_MAILBOX0; test_can_pending = 1U; return HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *h, uint32_t mailbox)
{
    (void)h; assert(mailbox == CAN_TX_MAILBOX0); test_can_abort_calls++;
    return test_can_abort_fail ? HAL_ERROR : HAL_OK; /* Async until AbortDone. */
}
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *h)
{ (void)h; return test_can_pending ? 2U : 3U; }
uint32_t HAL_CAN_IsTxMessagePending(CAN_HandleTypeDef *h, uint32_t mailbox)
{ (void)h; (void)mailbox; return test_can_pending; }
uint32_t HAL_CAN_GetState(CAN_HandleTypeDef *h) { return h->State; }
uint32_t HAL_CAN_GetError(const CAN_HandleTypeDef *h) { return h->ErrorCode; }
HAL_StatusTypeDef HAL_CAN_ResetError(CAN_HandleTypeDef *h) { h->ErrorCode = 0U; return HAL_OK; }
void TestCan_Complete(uint8_t success)
{
    assert(test_can_pending); test_can_pending = 0U;
    if (success) ZDT_CAN_TxComplete(CAN_TX_MAILBOX0);
    else { hcan1.ErrorCode = HAL_CAN_ERROR_TX_TERR0; ZDT_CAN_CanError(); }
}
void TestCan_AbortDone(void)
{ assert(test_can_pending); test_can_pending = 0U; ZDT_CAN_TxAbort(CAN_TX_MAILBOX0); }
void TestCan_Receive(uint32_t id, uint32_t ide, uint32_t rtr, uint8_t len, const uint8_t *data)
{
    assert(!rx_pending && len <= 8U);
    rx_header.ExtId = id; rx_header.IDE = ide; rx_header.RTR = rtr; rx_header.DLC = len;
    memset(rx_data, 0, 8U); memcpy(rx_data, data, len); rx_pending = 1U;
    ZDT_CAN_RxFIFO0();
}
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *h, uint32_t fifo)
{ (void)h; (void)fifo; return rx_pending; }
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *h, uint32_t fifo,
                                     CAN_RxHeaderTypeDef *header, uint8_t *data)
{
    (void)h; (void)fifo; assert(rx_pending); rx_pending = 0U;
    *header = rx_header; memcpy(data, rx_data, 8U); return HAL_OK;
}
