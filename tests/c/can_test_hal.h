#ifndef CAN_TEST_HAL_H
#define CAN_TEST_HAL_H
#include "control_test_hal.h"
typedef struct { CAN_TxHeaderTypeDef header; uint8_t data[8]; uint32_t tick; } TestCanFrame;
extern TestCanFrame test_can_frames[4096];
extern unsigned test_can_count, test_can_start_calls, test_can_abort_calls;
extern uint8_t test_can_init_fail, test_can_send_fail, test_can_abort_fail, test_can_pending;
void TestCan_Reset(void);
void TestCan_Complete(uint8_t success);
void TestCan_AbortDone(void);
void TestCan_Receive(uint32_t id, uint32_t ide, uint32_t rtr, uint8_t len, const uint8_t *data);
#endif
