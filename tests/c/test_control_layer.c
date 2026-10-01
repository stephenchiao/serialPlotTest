#include "control_test_hal.h"
#include "control_runtime.h"
#include "host_uart_tx.h"
#include "mecanum_chassis.h"
#include "ops9.h"
#include "pid.h"
#include "zdtCan.h"
#include "zdtEmm.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static CAN_TestRegisters can_regs;
CAN_HandleTypeDef hcan1 = {&can_regs};
static uint32_t can_error;
static uint32_t can_state = HAL_CAN_STATE_LISTENING;
uint32_t HAL_CAN_GetState(const CAN_HandleTypeDef *h) { (void)h; return can_state; }
HAL_StatusTypeDef HAL_CAN_ResetError(CAN_HandleTypeDef *h) { (void)h; can_error = 0; return HAL_OK; }
uint32_t HAL_CAN_GetError(CAN_HandleTypeDef *h) { (void)h; return can_error; }
UART_HandleTypeDef huart1, huart2 = {USART2, HAL_UART_STATE_READY};
static uint32_t tick, primask, pending, sent_count, abort_count, rx_remaining;
static uint8_t auto_complete, fail_can, fail_uart, fail_abort;
static uint32_t last_notify_mask, can_stop_count;
static uint8_t fail_can_stop, fail_can_start, fail_notify;
static struct { uint32_t id; uint8_t bytes[8]; } sent[256];
static const uint8_t *dma_bytes;
static uint16_t dma_length;
static uint32_t dma_count;
uint32_t HAL_GetTick(void) { return tick; }
uint32_t __get_PRIMASK(void) { return primask; }
void __disable_irq(void) { primask = 1U; }
void __enable_irq(void) { primask = 0U; }
void __set_PRIMASK(uint32_t value) { primask = value; }
void Error_Handler(void) { assert(0); }
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *h, CAN_FilterTypeDef *f)
{ (void)h; (void)f; return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_Init(CAN_HandleTypeDef *h)
{ (void)h; can_state = 1U; can_error = 0U; return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *h)
{
    (void)h;
    if (fail_can_start) { can_state = 3U; can_error |= HAL_CAN_ERROR_TIMEOUT; return HAL_ERROR; }
    can_state = HAL_CAN_STATE_LISTENING; can_error = 0U; return HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef *h)
{
    (void)h; can_stop_count++;
    if (fail_can_stop) { can_error |= HAL_CAN_ERROR_TIMEOUT; return HAL_ERROR; }
    pending = 0U; can_state = 1U; return HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *h, uint32_t n)
{ (void)h; last_notify_mask = n; return fail_notify ? HAL_ERROR : HAL_OK; }
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *h, uint32_t mask)
{
    (void)h; abort_count++;
    /* fail_abort 模拟 Abort 返回成功但硬件 TME 清不掉。 */
    if (!fail_abort) pending &= ~mask;
    return HAL_OK;
}
uint32_t HAL_CAN_IsTxMessagePending(CAN_HandleTypeDef *h, uint32_t mask)
{ (void)h; return pending & mask; }
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *h)
{ (void)h; return 3U - !!(pending & 1U) - !!(pending & 2U) - !!(pending & 4U); }
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *h, CAN_TxHeaderTypeDef *hdr,
                                     uint8_t *bytes, uint32_t *mailbox)
{
    uint32_t mask;
    (void)h;
    if (fail_can) return HAL_ERROR;
    for (mask = 1U; mask <= 4U; mask <<= 1U) if (!(pending & mask)) break;
    assert(mask <= 4U && sent_count < 256U);
    *mailbox = mask;
    sent[sent_count].id = hdr->ExtId;
    memcpy(sent[sent_count++].bytes, bytes, hdr->DLC);
    if (!auto_complete) pending |= mask;
    return HAL_OK;
}
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *h, uint32_t fifo)
{ (void)h; (void)fifo; return rx_remaining; }
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *h, uint32_t fifo,
                                     CAN_RxHeaderTypeDef *hdr, uint8_t *bytes)
{
    (void)h; (void)fifo;
    memset(hdr, 0, sizeof(*hdr)); memset(bytes, 0, 8U);
    if (rx_remaining) rx_remaining--;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *h, uint8_t *bytes, uint16_t length)
{
    assert(primask == 1U); /* Completion must not race the owner's busy flag. */
    if (fail_uart) return HAL_ERROR;
    assert(h->gState == HAL_UART_STATE_READY);
    h->gState = 1U; dma_bytes = bytes; dma_length = length; dma_count++;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *h, uint8_t *bytes, uint16_t length)
{ (void)h; assert(length == 4U && memcmp(bytes, "ACT0", 4U) == 0); return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *h, uint8_t *bytes, uint16_t length)
{ (void)h; (void)bytes; (void)length; return HAL_OK; }
static void near(float a, float b) { assert(fabsf(a - b) < 0.0001f); }
static void uart_done(void) { huart1.gState = HAL_UART_STATE_READY; HostUartTx_Complete(); }
static void uart_reset(void)
{ huart1.gState = HAL_UART_STATE_READY; fail_uart = 0U; HostUartTx_Reset(); }
static void write_line(const char *line) { assert(HostUartTx_Write(line, (int)strlen(line)) > 0); }
static void expect_line(const char *line)
{ assert(dma_length == strlen(line) && memcmp(dma_bytes, line, dma_length) == 0); }
static void can_reset(uint32_t now)
{
    tick = now;
    pending = 0U;
    ZDT_CAN_TestResetFault();
    sent_count = 0U; fail_can = auto_complete = fail_abort = 0U;
    fail_can_stop = fail_can_start = fail_notify = 0U;
    can_regs.ESR = 0U; can_error = 0U;
    can_state = HAL_CAN_STATE_LISTENING;
}
static void record_all(MotorFeedback samples[4], float rpm, uint32_t now)
{ unsigned i; for (i = 0U; i < 4U; ++i) MotorFeedback_Record(&samples[i], rpm, now); }

static void test_pid_dt_and_limits(void)
{
    PID_Controller a, b;
    unsigned i;
    PID_Init(&a, 0, 1, 0, 1000, 1000); PID_Init(&b, 0, 1, 0, 1000, 1000);
    for (i = 0; i < 5; ++i) PID_CalcErrorDt(&a, 2, .020f);
    for (i = 0; i < 10; ++i) PID_CalcErrorDt(&b, 2, .010f);
    near(a.integral, b.integral); near(a.integral, 10);
    PID_CalcErrorDt(&a, 2, .035f); near(a.integral, 13.5f);
    PID_ApplyOutput(&a, 1); near(a.integral, 10);
    PID_CalcErrorDt(&a, -2, .020f); PID_ApplyOutput(&a, 0); near(a.integral, 8);
    PID_Reset(&a); a.max_out = 1; PID_CalcErrorDt(&a, 2, .020f); near(a.integral, 0);
    PID_Init(&a, 0, 0, 1, 1000, 1000);
    near(PID_CalcErrorDt(&a, 100, .020f), 0); /* No derivative kick at start. */
    near(PID_CalcErrorDt(&a, 102, .020f), 1); /* tau = 20 ms => alpha = 1/2. */
    a.derivative_tau_s = 0; near(PID_CalcErrorDt(&a, 103, .010f), 2);
    near(PID_CalcErrorDt(&a, 105, .020f), 2);
    near(PID_CalcErrorDt(&a, NAN, .020f), 0); assert(!a.has_last_error);
    near(PID_CalcErrorDt(&a, 2, 0), 0); near(PID_CalcErrorDt(&a, 2, .101f), 0);
    PID_Init(&a, 1, 0, 0, 100, 100); PID_SetTarget(&a, 5); near(PID_Calc(&a, 2), 3);
}

static void test_stop_confirmation(void)
{
    MotorFeedback samples[4] = {0};
    MotorStopMonitor stop = {0};
    record_all(samples, 0, 90); record_all(samples, 0, 95);
    MotorStop_Request(&stop, 100);
    MotorStop_Update(&stop, samples, 1, 100); assert(stop.state == MOTOR_STOP_REQUESTED);
    record_all(samples, 0, 110); record_all(samples, 0, 120);
    MotorStop_Update(&stop, samples, 0, 120); assert(stop.state == MOTOR_STOP_WAIT_FEEDBACK);
    record_all(samples, 0, 130);
    MotorStop_Update(&stop, samples, 0, 130); assert(stop.state == MOTOR_STOP_WAIT_FEEDBACK);
    record_all(samples, 0, 140);
    MotorStop_Update(&stop, samples, 0, 140); assert(stop.state == MOTOR_STOP_CONFIRMED);
    MotorFeedback_Record(&samples[2], 10, 150);
    MotorStop_Update(&stop, samples, 0, 150); assert(stop.state == MOTOR_STOP_WAIT_FEEDBACK);
    MotorStop_Request(&stop, 600); assert(stop.requested_tick == 100);
    MotorStop_Update(&stop, samples, 0, 701); assert(stop.state == MOTOR_STOP_UNCONFIRMED);
    record_all(samples, 0, 710); record_all(samples, 0, 750);
    MotorStop_Update(&stop, samples, 0, 750); assert(stop.state == MOTOR_STOP_CONFIRMED);
    MotorStop_Update(&stop, samples, 0, 1051); assert(stop.state == MOTOR_STOP_UNCONFIRMED);
    assert(MotorFeedback_FreshMask(samples, 1051) == 0);
    memset(&stop, 0, sizeof(stop));
    MotorStop_Request(&stop, 2000); record_all(samples, 0, 2601);
    MotorStop_Update(&stop, samples, 1, 2601); assert(stop.state == MOTOR_STOP_UNCONFIRMED);
    MotorStop_Update(&stop, samples, 0, 2602); assert(stop.state == MOTOR_STOP_UNCONFIRMED);
    record_all(samples, 0, 2610); record_all(samples, 0, 2620);
    MotorStop_Update(&stop, samples, 0, 2620); assert(stop.state == MOTOR_STOP_CONFIRMED);
    memset(&stop, 0, sizeof(stop));
    MotorStop_Request(&stop, UINT32_MAX - 20U);
    MotorStop_Update(&stop, samples, 0, UINT32_MAX - 20U);
    record_all(samples, 0, UINT32_MAX - 5U); record_all(samples, 0, 10U);
    MotorStop_Update(&stop, samples, 0, 10U); assert(stop.state == MOTOR_STOP_CONFIRMED);
    assert(MotorFeedback_FreshMask(samples, 20U) == 15U);

    /* A selected motor can confirm a stop without feedback from absent motors. */
    memset(samples, 0, sizeof(samples));
    memset(&stop, 0, sizeof(stop));
    MotorStop_Request(&stop, 3000U);
    MotorFeedback_Record(&samples[0], 0.0f, 3010U);
    MotorStop_UpdateMasked(&stop, samples, 0x01U, 0U, 3010U);
    assert(stop.state == MOTOR_STOP_WAIT_FEEDBACK);
    MotorFeedback_Record(&samples[0], 0.0f, 3020U);
    MotorFeedback_Record(&samples[0], 0.0f, 3030U);
    MotorStop_UpdateMasked(&stop, samples, 0x01U, 0U, 3030U);
    assert(stop.state == MOTOR_STOP_CONFIRMED);
    assert((MotorFeedback_FreshMask(samples, 3030U) & 0x0EU) == 0U);
}


static void test_uart_dma_ownership_and_priority(void)
{
    uint8_t saved[512];
    uint16_t saved_length;
    uart_reset(); tick = 10;
    write_line("# FRAG"); HostUartTx_Process(tick); assert(!HostUartTx_Busy());
    write_line("MENT\r\n"); HostUartTx_Process(tick); expect_line("# FRAGMENT\r\n");
    saved_length = dma_length; memcpy(saved, dma_bytes, saved_length);
    write_line("@W,old\n"); write_line("@P,pose\n"); write_line("@W,new\n");
    write_line("1,tune\n"); write_line("# NORMAL\n");
    write_line("# ROUND START 1\n");
    write_line("# CAN FEEDBACK LOST MASK=0x0F FRESH=0x0B\n");
    write_line("# ROUND STOP MOTOR FEEDBACK LOST\n");
    write_line("# POSE STOP\n");
    HostUartTx_Process(11); assert(!memcmp(saved, dma_bytes, saved_length));
    uart_done(); HostUartTx_Process(12); expect_line("# ROUND START 1\n");
    write_line("# STOP MODE=WORK\n");
    uart_done(); HostUartTx_Process(13); expect_line("# CAN FEEDBACK LOST MASK=0x0F FRESH=0x0B\n");
    uart_done(); HostUartTx_Process(14); expect_line("# ROUND STOP MOTOR FEEDBACK LOST\n");
    uart_done(); HostUartTx_Process(15); expect_line("# POSE STOP\n");
    uart_done(); HostUartTx_Process(16); expect_line("# STOP MODE=WORK\n");
    uart_done(); HostUartTx_Process(17); expect_line("# NORMAL\n");
    uart_done(); HostUartTx_Process(18); expect_line("@W,new\n");
    uart_done(); HostUartTx_Process(19); expect_line("@P,pose\n");
    uart_done(); HostUartTx_Process(20); expect_line("1,tune\n");
    /* Binary transition may discard queued text, never the active DMA buffer. */
    write_line("# STALE\n"); HostUartTx_DiscardPending(); expect_line("1,tune\n");
    HostUartTx_Process(121); assert(HostUartTx_ConsumeFault());
    uart_done(); HostUartTx_Process(122); assert(!HostUartTx_Busy());
    assert(HostUartTx_GetStats().replaced > 0);
}

static void test_uart_burst_and_errors(void)
{
    unsigned i;
    uint32_t before;
    char oversized[600];
    uart_reset();
    for (i = 0; i < 25; ++i) write_line("# HELP OR BOOT LINE\n");
    assert(!HostUartTx_ConsumeFault());
    before = dma_count;
    for (i = 0; i < 25; ++i) { HostUartTx_Process(i); uart_done(); }
    assert(dma_count - before == 25);
    for (i = 0; i < 33; ++i) write_line("# NORMAL\n");
    assert(HostUartTx_ConsumeFault()); uart_reset();
    memset(oversized, 'X', sizeof(oversized)); oversized[599] = '\n';
    HostUartTx_Write(oversized, sizeof(oversized)); assert(HostUartTx_ConsumeFault());
    write_line("# AFTER INVALID\n"); HostUartTx_Process(10); expect_line("# AFTER INVALID\n");
    uart_done(); fail_uart = 1; write_line("# FAIL\n"); HostUartTx_Process(11);
    assert(!HostUartTx_Busy() && HostUartTx_ConsumeFault()); uart_reset();
    for (i = 0; i < 5; ++i) write_line("# STOP MODE=WORK\n");
    assert(HostUartTx_ConsumeFault()); uart_reset();
    write_line("# WRAP\n"); HostUartTx_Process(UINT32_MAX - 10U);
    HostUartTx_Process(20); assert(!HostUartTx_ConsumeFault());
    HostUartTx_Process(100); assert(HostUartTx_ConsumeFault()); uart_reset();
}


extern uint8_t ops9_rx_byte;
static void ops_byte(uint8_t ch) { ops9_rx_byte = ch; OPS9_UART_RxCpltCallback(&huart2); }
static void ops_frame(float x, float y, float yaw)
{
    float values[6] = {yaw, 0, 0, x, y, 0};
    uint8_t bytes[24]; unsigned i;
    memcpy(bytes, values, sizeof(bytes));
    ops_byte(13); ops_byte(10);
    for (i = 0; i < sizeof(bytes); ++i) ops_byte(bytes[i]);
    ops_byte(10); ops_byte(13);
}
static void test_ops_snapshot_and_invalid_frame(void)
{
    OPS9_Snapshot snapshot;
    tick = 6000; ops_frame(123, -456, 90);
    primask = 1; snapshot = OPS9_GetSnapshot(); assert(primask == 1);
    primask = 0; snapshot = OPS9_GetSnapshot(); assert(primask == 0);
    near(snapshot.x_mm, 123); near(snapshot.y_mm, -456); near(snapshot.yaw_deg, 90);
    assert(snapshot.frame_count == 1 && snapshot.last_update_tick == tick);
    tick++; ops_frame(NAN, 888, 10); snapshot = OPS9_GetSnapshot();
    near(snapshot.x_mm, 123); near(snapshot.y_mm, -456);
    assert(snapshot.frame_count == 1 && snapshot.last_update_tick == 6000);
    assert(ops9_invalid_frame_count == 1);
    tick++; ops_frame(789, 222, -90); snapshot = OPS9_GetSnapshot();
    near(snapshot.x_mm, 789); near(snapshot.y_mm, 222); assert(snapshot.frame_count == 2);
    OPS9_Reset_Zero();
}

static void test_runtime_deadline(void)
{
    tick = UINT32_MAX - 10U; ControlRuntime_Init();
    ControlRuntime_Begin(20); ControlRuntime_End(21);
    assert(!ControlRuntime_GetStats().fault);
    ControlRuntime_RecordControl(20); ControlRuntime_RecordControl(35);
    assert(ControlRuntime_GetStats().control_max_ms == 35);
    assert(ControlRuntime_GetStats().control_late_count == 1);
    ControlRuntime_Begin(121); assert(ControlRuntime_GetStats().fault);
    assert(ControlRuntime_GetStats().loop_overruns == 1);
    ControlRuntime_ClearFault(); ControlRuntime_End(222);
    assert(ControlRuntime_GetStats().fault);
}

static void tx_complete(void)
{
    assert(pending != 0U);
    if (pending & 1U) HAL_CAN_TxMailbox0CompleteCallback(&hcan1);
    if (pending & 2U) HAL_CAN_TxMailbox1CompleteCallback(&hcan1);
    if (pending & 4U) HAL_CAN_TxMailbox2CompleteCallback(&hcan1);
    pending = 0U;
}
static void pump_stops(void)
{
    unsigned i;
    for (i = 0U; i < 4U; ++i) {
        if (!pending) ZDT_CAN_Process(tick);
        assert(pending);
        assert(sent[sent_count - 1U].bytes[0] == 0xFEU);
        tx_complete(); tick++; ZDT_CAN_Process(tick);
    }
}
static void test_command_only_transport(void)
{
    MotorFeedback samples[4];
    unsigned i;
    uint8_t query[2] = {0x35U, 0x6BU};
    ZDT_CAN_Stats_t stats;
    can_reset(1000U); ZDT_Emm_InitAll();
    ZDT_Emm_SetProtocol(ZDT_PROTOCOL_EMM);
    assert(!ZDT_Emm_SetSpeedByID(1U, 10.0f));
    assert(!ZDT_Emm_SetSpeedByID(1U, -40.0f));
    ZDT_CAN_Process(tick);
    assert(sent_count == 1U && sent[0].bytes[1] == 1U && sent[0].bytes[3] == 40U);
    assert(pending == 1U); /* Exactly one in flight. */
    ZDT_CAN_Process(tick); assert(sent_count == 1U);
    tx_complete(); ZDT_CAN_Process(++tick);
    ZDT_Emm_GetFeedback(samples); assert(!MotorFeedback_FreshMask(samples, tick));
    assert(!StopAllMotors()); pump_stops(); Mecanum_ProcessFeedback(tick);
    assert(Mecanum_GetStopStatus().state == MOTOR_STOP_SENT);
    assert(ZDT_CAN_StopSent(15U));
    assert(ZDT_CAN_StopSentMask() == 15U);
    /* No fabricated speed feedback or physical stop confirmation. */
    assert(!samples[0].valid);

    can_reset(2000U); ZDT_Emm_InitAll();
    assert(!ZDT_Emm_SetSpeedByID(1U, 80.0f)); ZDT_CAN_Process(tick);
    fail_abort = 1U; /* Abort request accepted; hardware is still busy. */
    assert(!StopAllMotors()); ZDT_CAN_Process(++tick);
    assert(sent_count == 1U); /* STOP waits for old mailbox cancellation. */
    pending = 0U; fail_abort = 0U; pump_stops();
    for (i = 1U; i < sent_count; ++i) assert(sent[i].bytes[0] == 0xFEU);

    can_reset(3000U); ZDT_Emm_InitAll();
    assert(!ZDT_Emm_SetSpeedByID(1U, 70.0f)); ZDT_CAN_Process(tick);
    tick += 50U; ZDT_CAN_Process(tick); assert(ZDT_CAN_HasFault());
    assert(!ZDT_CAN_RecoverWhenIdle(1U));
    assert(ZDT_Emm_SetSpeedByID(1U, 90.0f) == 4U);
    assert(!StopAllMotors()); pump_stops();
    assert(!ZDT_CAN_RecoverWhenIdle(0U));
    assert(ZDT_CAN_RecoverWhenIdle(1U)); /* No motor reply required. */
    assert(ZDT_CAN_IsReady());
    ZDT_CAN_Process(++tick);
    for (i = 1U; i < sent_count; ++i) assert(sent[i].bytes[0] == 0xFEU);
    assert(!ZDT_Emm_SetSpeedByID(1U, 33.0f)); ZDT_CAN_Process(tick);
    assert(sent[sent_count - 1U].bytes[3] == 33U);
    tx_complete(); ZDT_CAN_Process(++tick);

    /* Late motor power: repeated failed STOP attempts are retained, rotate,
     * and eventually complete without RX; no old motion is replayed. */
    can_reset(4000U); ZDT_Emm_InitAll(); assert(!StopAllMotors());
    tick += 50U; ZDT_CAN_Process(tick);
    tick++; ZDT_CAN_Process(tick); assert(pending);
    assert(sent[1].id != sent[0].id);
    tx_complete(); ZDT_CAN_Process(++tick);
    tx_complete(); ZDT_CAN_Process(++tick);
    tx_complete(); ZDT_CAN_Process(++tick);
    assert(!ZDT_CAN_StopSent(15U));
    tick += 100U; ZDT_CAN_Process(tick); assert(pending);
    tx_complete(); ZDT_CAN_Process(++tick);
    assert(ZDT_CAN_StopSent(15U) && ZDT_CAN_RecoverWhenIdle(1U));

    /* Latest enable/disable survives retry, 5ms settling precedes speed. */
    can_reset(5000U); ZDT_Emm_InitAll();
    assert(!ZDT_Emm_EnableByID(1U, 1U));
    assert(!ZDT_Emm_SetSpeedByID(1U, 22.0f)); ZDT_CAN_Process(tick);
    assert(sent[0].bytes[0] == 0xF3U);
    tx_complete(); ZDT_CAN_Process(++tick); assert(sent_count == 1U);
    tick += 5U; ZDT_CAN_Process(tick); assert(sent_count == 2U);
    tx_complete(); ZDT_CAN_Process(++tick);
    assert(!ZDT_Emm_EnableByID(1U, 0U)); ZDT_CAN_Process(tick);
    tx_complete(); ZDT_CAN_Process(++tick); ZDT_Emm_RefreshEnables();
    ZDT_CAN_Process(++tick); assert(sent_count == 3U);

    can_reset(6000U); ZDT_Emm_InitAll();
    for (i = 0U; i < 8U; ++i) assert(!ZDT_CAN_Send_ExtId(0x100U, query, 2U));
    assert(ZDT_CAN_Send_ExtId(0x100U, query, 2U) == 1U);
    assert(!ZDT_CAN_HasFault()); /* Diagnostic congestion is not a motion fault. */
    assert(ZDT_CAN_Send_ExtId(0x101U, query, 2U) == 3U);
    ZDT_CAN_RxFIFO0_Handler(&hcan1);
    ZDT_CAN_GetStats(&stats); assert(stats.rx_count == 0U);

    can_reset(0xFFFFFFE0U); ZDT_Emm_InitAll();
    assert(!ZDT_Emm_SetSpeedByID(1U, 20.0f)); ZDT_CAN_Process(tick);
    tick = 17U; ZDT_CAN_Process(tick); assert(pending);
    tick = 18U; ZDT_CAN_Process(tick); assert(!pending && ZDT_CAN_HasFault());
}


static void test_transport_repair_and_formats(void)
{
    unsigned i;
    ZDT_CAN_Stats_t before, after;
    uint8_t bad[5] = {0x35, 0, 0, 10, 0};
    MotorFeedback samples[4];
    can_reset(10000U); ZDT_Emm_InitAll();
    ZDT_Emm_SetProtocol(ZDT_PROTOCOL_X);
    assert(!ZDT_Emm_SetSpeedByID(2U, -123.4f)); ZDT_CAN_Process(tick);
    assert(sent[0].id == 0x200U && sent[0].bytes[1] == 1U);
    assert(sent[0].bytes[2] == 1U && sent[0].bytes[3] == 244U);
    assert(sent[0].bytes[4] == 4U && sent[0].bytes[5] == 210U && sent[0].bytes[7] == 0x6BU);
    tx_complete(); ZDT_CAN_Process(++tick);
    ZDT_Emm_RxHandler(0x200U, bad, 5U);
    ZDT_Emm_GetFeedback(samples); assert(!samples[1].valid);
    bad[4] = 0x6B; ZDT_Emm_RxHandler(0x201U, bad, 5U);
    ZDT_Emm_GetFeedback(samples); assert(!samples[1].valid);
    ZDT_Emm_RxHandler(0x200U, bad, 5U);
    ZDT_Emm_GetFeedback(samples); assert(samples[1].valid); near(samples[1].rpm, 1.0f);

    can_reset(11000U); ZDT_Emm_InitAll(); ZDT_Emm_SetProtocol(ZDT_PROTOCOL_EMM);
    assert(!SetAllMotorsSpeed(0.1f, 0.0f, 0.2f, 0.0f));
    for (i = 0U; i < 4U; ++i) {
        ZDT_CAN_Process(tick); assert(pending);
        tx_complete(); tick++;
    }
    ZDT_CAN_Process(tick); assert(sent_count == 4U);
    assert(!Mecanum_FeedbackReady(15U));
    assert(!SetAllMotorsSpeed(2.0f, 1.0f, 0.0f, -2.0f)); near(Mecanum_GetAppliedScale(), 0.4f);
    assert(SetAllMotorsSpeed(NAN, 0.0f, 0.0f, 0.0f) == 4U);
    pump_stops();

    /* Abort accepted but stuck: restart steps must preserve STOP and reject motion. */
    can_reset(12000U); ZDT_Emm_InitAll();
    assert(!ZDT_Emm_SetSpeedByID(1U, 60.0f)); ZDT_CAN_Process(tick);
    fail_abort = 1U; tick += 50U; ZDT_CAN_Process(tick);
    assert(ZDT_CAN_HasFault()); assert(!StopAllMotors());
    ZDT_CAN_GetStats(&before);
    tick += 450U; ZDT_CAN_Process(tick); /* Stop phase */
    assert(!ZDT_CAN_IsReady());
    ZDT_CAN_Process(++tick); /* Start/notifications phase */
    ZDT_CAN_GetStats(&after); assert(after.stall_recoveries == before.stall_recoveries + 1U);
    fail_abort = 0U; pump_stops(); assert(ZDT_CAN_RecoverWhenIdle(1U));
    for (i = 1U; i < sent_count; ++i) assert(sent[i].bytes[0] == 0xFEU);

    /* Start failure must stay faulted, then retry from HAL Init, not Stop(READY). */
    can_reset(14000U); ZDT_Emm_InitAll();
    assert(!ZDT_Emm_SetSpeedByID(1U, 60.0f)); ZDT_CAN_Process(tick);
    fail_abort = 1U; tick += 50U; ZDT_CAN_Process(tick);
    assert(!StopAllMotors()); tick += 450U; ZDT_CAN_Process(tick);
    fail_can_start = 1U; ZDT_CAN_Process(++tick); assert(!ZDT_CAN_IsReady());
    fail_can_start = fail_abort = 0U; tick += 500U; ZDT_CAN_Process(tick);
    ZDT_CAN_Process(++tick); pump_stops(); assert(ZDT_CAN_RecoverWhenIdle(1U));

    /* Notification failure is not a successful restart; retry must restore it. */
    can_reset(15000U); ZDT_Emm_InitAll();
    assert(!ZDT_Emm_SetSpeedByID(1U, 60.0f)); ZDT_CAN_Process(tick);
    fail_abort = 1U; tick += 50U; ZDT_CAN_Process(tick);
    assert(!StopAllMotors()); tick += 450U; ZDT_CAN_Process(tick);
    fail_notify = 1U; ZDT_CAN_Process(++tick); assert(!ZDT_CAN_IsReady());
    fail_notify = fail_abort = 0U; tick += 500U; ZDT_CAN_Process(tick);
    ZDT_CAN_Process(++tick); pump_stops(); assert(ZDT_CAN_RecoverWhenIdle(1U));

    can_reset(16000U); ZDT_Emm_InitAll();
    assert(!ZDT_Emm_SetSpeedByID(1U, 88.0f)); ZDT_CAN_Process(tick);
    can_regs.ESR = CAN_ESR_BOFF; ZDT_CAN_GetStats(&before);
    ZDT_CAN_Process(++tick); assert(!pending); /* cancel before ABOM recovery */
    assert(!StopAllMotors()); tick += 1000U; ZDT_CAN_Process(tick);
    assert(!ZDT_CAN_IsReady()); ZDT_CAN_GetStats(&after);
    assert(after.stall_recoveries == before.stall_recoveries);
    can_regs.ESR = CAN_ESR_EPVF; /* Passive can still transmit; don't restart repeatedly. */
    pump_stops(); assert(ZDT_CAN_RecoverWhenIdle(1U));
    ZDT_CAN_ConfigFilter();
    assert(last_notify_mask == (CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY));
}

static void test_transport_no_tx_progress(void)
{
    ZDT_CAN_Stats_t stats;
    unsigned i;
    uint8_t query[2] = {0x35U, 0x6BU};
    /* 现场模式：每帧超时后撤销成功、FREE=2/3，但持续没有 TXOK。
     * 跨帧监督必须触发，不能每次换邮箱/重试都重新计时。 */
    can_reset(20000U); ZDT_Emm_InitAll();
    assert(!ZDT_Emm_SetSpeedByID(1U, 60.0f)); ZDT_CAN_Process(tick);
    tick += 50U; ZDT_CAN_Process(tick);
    assert(!StopAllMotors());
    for (i = 0U; i < 600U; ++i) ZDT_CAN_Process(++tick);
    ZDT_CAN_GetStats(&stats);
    assert(stats.tx_timeout >= 4U && stats.tx_ok == 0U);
    assert(stats.stall_recoveries >= 1U);
    assert(!ZDT_CAN_IsReady());
    for (i = 0U; i < 200U; ++i) {
        if (pending) tx_complete();
        ZDT_CAN_Process(++tick);
    }
    assert(ZDT_CAN_StopSent(15U) && ZDT_CAN_RecoverWhenIdle(1U));
    for (i = 1U; i < sent_count; ++i) assert(sent[i].bytes[0] == 0xFEU);

    /* 单次失败查询后空闲，不应因为历史 TXOK 为零而周期重启。 */
    can_reset(30000U); ZDT_Emm_InitAll();
    assert(!ZDT_CAN_Send_ExtId(0x100U, query, 2U));
    for (i = 0U; i < 1200U; ++i) ZDT_CAN_Process(++tick);
    ZDT_CAN_GetStats(&stats); assert(stats.stall_recoveries == 0U);

    /* 有新 TXOK 的持续流量与长时间空闲后首次发帧不应误触发。 */
    can_reset(40000U); ZDT_Emm_InitAll();
    for (i = 0U; i < 200U; ++i) {
        if (pending) tx_complete();
        assert(!ZDT_CAN_Send_ExtId(0x100U, query, 2U));
        tick += 10U; ZDT_CAN_Process(tick);
    }
    tx_complete(); ZDT_CAN_Process(++tick);
    tick += 5000U; ZDT_CAN_Process(tick);
    assert(!ZDT_CAN_Send_ExtId(0x100U, query, 2U)); ZDT_CAN_Process(++tick);
    ZDT_CAN_GetStats(&stats); assert(stats.stall_recoveries == 0U);

    can_reset(UINT32_MAX - 200U); ZDT_Emm_InitAll(); assert(!StopAllMotors());
    for (i = 0U; i < 650U; ++i) ZDT_CAN_Process(++tick);
    ZDT_CAN_GetStats(&stats); assert(stats.stall_recoveries >= 1U);
}

int main(void)
{
    test_pid_dt_and_limits();
    test_command_only_transport();
    test_transport_repair_and_formats();
    test_transport_no_tx_progress();
    test_stop_confirmation();
    test_uart_dma_ownership_and_priority();
    test_uart_burst_and_errors();
    test_ops_snapshot_and_invalid_frame();
    test_runtime_deadline();
    puts("control layer: 3 CAN groups and 6 control groups passed");
    return 0;
}
