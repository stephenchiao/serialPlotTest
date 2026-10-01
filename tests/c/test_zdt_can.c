#include "can_test_hal.h"
#include "zdtEmm.h"
#include "zdtCan.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static uint32_t tick, primask;
static uint8_t remote_options, remote_status, remote_connected, reply_enabled, tx_success;
static float remote_rpm;
uint32_t HAL_GetTick(void) { return tick; }
uint32_t __get_PRIMASK(void) { return primask; }
void __disable_irq(void) { primask = 1U; }
void __enable_irq(void) { primask = 0U; }
void __set_PRIMASK(uint32_t p) { primask = p; }
static ZDT_CanStats snapshot(void) { ZDT_CanStats s; ZDT_CAN_GetStats(&s); return s; }
static MotorFeedback sample(void) { MotorFeedback f[4]; ZDT_Emm_GetFeedback(f); return f[0]; }
static void receive(const uint8_t *data, uint8_t len)
{ TestCan_Receive(0x100U, CAN_ID_EXT, CAN_RTR_DATA, len, data); }
static unsigned count_code(uint8_t code, uint8_t nonzero)
{
    unsigned n = 0U;
    for (unsigned i = 0U; i < test_can_count; ++i)
        if (test_can_frames[i].data[0] == code && (!nonzero ||
            (test_can_frames[i].data[2] || test_can_frames[i].data[3]))) n++;
    return n;
}
static void reset(uint32_t now)
{
    tick = now; primask = 0U; TestCan_Reset();
    remote_options = 6U; remote_status = 0x81U; remote_rpm = 0.0f;
    remote_connected = reply_enabled = tx_success = 1U;
    ZDT_Emm_InitAll();
}
/* Deliberately ideal simulated motor. The 1 ms delay is synthetic, not measured. */
static void step(void)
{
    tick++;
    if (test_can_pending) {
        TestCanFrame f = test_can_frames[test_can_count - 1U];
        TestCan_Complete(tx_success);
        if (tx_success && remote_connected) {
            uint8_t data[8] = {f.data[0], 2U, 0x6BU};
            uint8_t len = 3U;
            if (f.data[0] == 0xF6U) {
                remote_rpm = (float)(((uint16_t)f.data[2] << 8) | f.data[3]);
                if (remote_options & 0x80U) remote_rpm /= 10.0f;
                if (f.data[1]) remote_rpm = -remote_rpm;
            } else if (f.data[0] == 0xFEU) remote_rpm = 0.0f;
            else if (f.data[0] == 0x1AU) data[1] = remote_options;
            else if (f.data[0] == 0x3AU) data[1] = remote_status;
            else if (f.data[0] == 0x35U) {
                unsigned rpm = (unsigned)lroundf(fabsf(remote_rpm));
                data[1] = remote_rpm < 0.0f; data[2] = rpm >> 8; data[3] = rpm;
                data[4] = 0x6BU; len = 5U;
            }
            if (reply_enabled) receive(data, len);
        }
    }
    ZDT_CAN_Process(tick);
}
static void run(unsigned ms) { while (ms--) step(); }
static void idle(void) { for (unsigned n = 0; snapshot().in_flight && n < 10U; ++n) step(); }
static void warm(void)
{
    run(150U); idle();
    assert(ZDT_CAN_ReadyByID(1U) && snapshot().stop_confirmed);
    assert(!ZDT_Emm_IsAvailable() && !ZDT_CAN_ReadyByID(2U));
}
static void speed(float rpm)
{ idle(); assert(ZDT_Emm_SetSpeedByID(1U, rpm) == ZDT_MOTOR_SUBMITTED); }
static void test_init_and_failure(void)
{
    for (uint8_t fail = 1U; fail <= 3U; ++fail) {
        reset(0U); TestCan_Reset(); test_can_init_fail = fail; ZDT_Emm_InitAll();
        assert(!ZDT_CAN_Started() && snapshot().init_error == fail);
        assert(ZDT_Emm_SetSpeedByID(1U, 0.0f) == ZDT_MOTOR_UNAVAILABLE);
        assert(ZDT_Emm_StopMask(1U) == ZDT_MOTOR_UNAVAILABLE);
        run(1000U); assert(!sample().valid && !snapshot().stop_confirmed);
    }
    reset(10U); run(100U); ZDT_Emm_InitAll(); assert(test_can_start_calls == 1U);
    assert(!count_code(0xF3U, 0U) && !count_code(0xF6U, 1U));
    assert(ZDT_Emm_SetSpeedByID(0U, 1.0f) == ZDT_MOTOR_INVALID);
    assert(ZDT_Emm_SetSpeedByID(1U, NAN) == ZDT_MOTOR_INVALID);
    assert(ZDT_Emm_StopMask(0U) == ZDT_MOTOR_INVALID);
    assert(ZDT_Emm_StopMask(0xF0U) == ZDT_MOTOR_INVALID);
    assert(ZDT_Emm_StopMask(0x0FU) == ZDT_MOTOR_UNAVAILABLE);
    assert(ZDT_Emm_ReadSpeedByID(2U) == ZDT_MOTOR_UNAVAILABLE);
}
static void test_format_and_scale(void)
{
    reset(0U); warm(); speed(-60.0f);
    TestCanFrame *f = &test_can_frames[test_can_count - 1U];
    const uint8_t expected[] = {0xF6U, 1U, 0U, 60U, 0U, 0U, 0x6BU};
    assert(f->header.ExtId == 0x100U && f->header.IDE == CAN_ID_EXT &&
           f->header.RTR == CAN_RTR_DATA && f->header.DLC == 7U);
    assert(memcmp(f->data, expected, sizeof(expected)) == 0);
    run(30U); assert(sample().rpm == -60.0f);
    assert(ZDT_Emm_StopMask(1U) == 0U); run(100U); idle();
    uint8_t options[] = {0x1AU, 0x86U, 0x6BU};
    remote_options = 0x86U; receive(options, 3U); ZDT_CAN_Process(tick); idle();
    speed(60.0f); f = &test_can_frames[test_can_count - 1U];
    assert(f->data[2] == 2U && f->data[3] == 0x58U); /* 600 means 60 RPM command. */
    run(30U); assert(sample().rpm == 60.0f); /* Reply remains 60 RPM, never /10. */
    for (unsigned i = 0; i < test_can_count; ++i) {
        f = &test_can_frames[i];
        if (f->data[0] == 0x35U || f->data[0] == 0x3AU || f->data[0] == 0x1AU)
            assert(f->header.DLC == 2U && f->data[1] == 0x6BU);
        if (f->data[0] == 0xFEU) assert(f->header.DLC == 4U && f->data[1] == 0x98U && f->data[2] == 0U);
    }
}
static void test_real_feedback_only(void)
{
    reset(0U); warm(); MotorFeedback before = sample();
    uint8_t ack[] = {0xF6U, 2U, 0x6BU}, status[] = {0x3AU, 0x81U, 0x6BU};
    receive(ack, 3U); receive(status, 3U); ZDT_CAN_Process(tick);
    assert(sample().sequence == before.sequence && sample().tick == before.tick);
    uint8_t speed_data[] = {0x35U, 1U, 3U, 0x84U, 0x6BU};
    TestCan_Receive(0x100U, CAN_ID_STD, 0U, 5U, speed_data);
    TestCan_Receive(0x100U, CAN_ID_EXT, CAN_RTR_REMOTE, 5U, speed_data);
    TestCan_Receive(0x101U, CAN_ID_EXT, 0U, 5U, speed_data);
    TestCan_Receive(0x200U, CAN_ID_EXT, 0U, 5U, speed_data);
    TestCan_Receive(0x100U, CAN_ID_EXT, 0U, 4U, speed_data);
    speed_data[1] = 2U; receive(speed_data, 5U); speed_data[1] = 1U;
    speed_data[4] = 0U; receive(speed_data, 5U); speed_data[4] = 0x6BU;
    ZDT_CAN_Process(tick); assert(sample().sequence == before.sequence);
    assert(snapshot().rx_rejected == 7U);
    tick += 3U; receive(speed_data, 5U); ZDT_CAN_Process(tick);
    assert(sample().rpm == -900.0f && sample().tick == tick && sample().sequence == before.sequence + 1U);
}
static void test_stop_cancel_and_confirmation(void)
{
    reset(0U); warm(); speed(60.0f);
    assert(test_can_pending); MotorFeedback before = sample();
    assert(ZDT_Emm_StopMask(1U) == 0U && motors[0].target_speed == 0.0f);
    assert(test_can_abort_calls == 1U && !snapshot().stop_confirmed);
    reply_enabled = 0U;
    /* An already-on-wire old speed may win the abort race; never restore its target. */
    TestCan_Complete(1U); ZDT_CAN_Process(tick); run(20U);
    assert(snapshot().stop_sent && !snapshot().stop_confirmed && motors[0].target_speed == 0.0f);
    assert(sample().sequence == before.sequence && sample().tick == before.tick);
    uint8_t ack[] = {0xFEU, 2U, 0x6BU}, zero[] = {0x35U, 0U, 0U, 0U, 0x6BU};
    receive(ack, 3U); ZDT_CAN_Process(tick); assert(!snapshot().stop_confirmed);
    zero[3] = 1U; receive(zero, 5U); ZDT_CAN_Process(tick);
    assert(!snapshot().stop_confirmed); zero[3] = 0U;
    tick++; receive(zero, 5U); ZDT_CAN_Process(tick); assert(!snapshot().stop_confirmed);
    tick++; receive(zero, 5U); ZDT_CAN_Process(tick); assert(snapshot().stop_confirmed);
    unsigned old_speeds = count_code(0xF6U, 1U);
    reply_enabled = 1U; run(200U);
    assert(count_code(0xF6U, 1U) == old_speeds && motors[0].target_speed == 0.0f);
}
static void test_outage_and_busoff(void)
{
    reset(0U); warm(); speed(60.0f); run(10U);
    reply_enabled = 0U; run(100U);
    assert(motors[0].target_speed == 0.0f && snapshot().fault_cancels == 1U);
    unsigned old_speeds = count_code(0xF6U, 1U);
    reply_enabled = 1U; run(250U); assert(snapshot().stop_confirmed);
    assert(count_code(0xF6U, 1U) == old_speeds);
    speed(30.0f); run(5U);
    hcan1.Instance->ESR = 0x00FF0047U; ZDT_CAN_CanError(); ZDT_CAN_Process(tick);
    assert(motors[0].target_speed == 0.0f && !ZDT_CAN_ReadyByID(1U));
    old_speeds = count_code(0xF6U, 1U); run(80U);
    hcan1.Instance->ESR = 0U; run(250U);
    assert(snapshot().bus_off_entries == 1U && snapshot().auto_bus_off_exits == 1U);
    assert(snapshot().software_restarts == 0U && test_can_start_calls == 1U);
    assert(count_code(0xF6U, 1U) == old_speeds);
    assert(snapshot().tec_max == 255U && snapshot().stop_confirmed);
}
static void test_stuck_mailbox_and_fair_retry(void)
{
    reset(0U); ZDT_CAN_Process(tick); assert(test_can_pending);
    for (unsigned n = 0U; n < 1000U; ++n) { tick++; ZDT_CAN_Process(tick); }
    assert(test_can_abort_calls == 1U && test_can_start_calls == 1U);
    assert(snapshot().abort_stuck == 1U && !sample().valid);
    TestCan_AbortDone(); run(300U); assert(sample().valid && snapshot().stop_confirmed);
    reset(0U); tx_success = 0U; run(350U);
    assert(!snapshot().tx_done && count_code(0x35U, 0U) && count_code(0xFEU, 0U));
    assert(count_code(0xF6U, 0U) && test_can_count < 40U);
    assert(!sample().valid && !snapshot().stop_confirmed && test_can_start_calls == 1U);
}
static void test_late_motor_and_reset(void)
{
    reset(0U); remote_connected = 0U; run(350U);
    assert(snapshot().tx_done > 0U && !sample().valid && !snapshot().stop_confirmed);
    assert(count_code(0xFEU, 0U) > 1U && count_code(0xF6U, 0U) > 1U);
    remote_connected = 1U; run(1200U); assert(ZDT_CAN_ReadyByID(1U) && snapshot().stop_confirmed);
    speed(45.0f); run(30U); assert(sample().rpm == 45.0f);
    /* Reset MCU while the simulated motor retains a nonzero speed. */
    reset(tick); remote_rpm = 45.0f; remote_status = 0x81U; run(250U);
    assert(remote_rpm == 0.0f && !count_code(0xF6U, 1U) && !count_code(0xF3U, 0U));
    assert(snapshot().stop_confirmed && !ZDT_Emm_IsAvailable());
}
static void test_overwrite_nack_and_wrap(void)
{
    reset(UINT32_MAX - 80U); warm(); speed(15.0f); run(30U);
    uint8_t nack[] = {0xF6U, 0xE2U, 0x6BU};
    receive(nack, 3U); ZDT_CAN_Process(tick);
    assert(motors[0].target_speed == 0.0f && snapshot().nack == 1U);
    run(80U); idle(); speed(15.0f); run(5U);
    uint8_t ack[] = {0xFEU, 2U, 0x6BU};
    receive(nack, 3U); receive(ack, 3U); ZDT_CAN_Process(tick);
    assert(motors[0].target_speed == 0.0f && snapshot().nack == 2U);
    run(80U); idle(); assert(ZDT_Emm_StopMask(1U) == 0U); reply_enabled = 0U; run(10U);
    uint8_t zero[] = {0x35U, 0U, 0U, 0U, 0x6BU};
    receive(zero, 5U); receive(zero, 5U); ZDT_CAN_Process(tick);
    assert(snapshot().rx_overwritten > 0U && !snapshot().stop_confirmed);
    tick++; receive(zero, 5U); ZDT_CAN_Process(tick); assert(snapshot().stop_confirmed);
}
static void test_submit_failure_and_delayed_consumer(void)
{
    reset(0U); warm(); speed(25.0f); run(30U); idle();
    test_can_send_fail = 1U;
    assert(ZDT_Emm_SetSpeedByID(1U, 40.0f) == ZDT_MOTOR_CAN_ERROR);
    assert(motors[0].target_speed == 0.0f);
    test_can_send_fail = 0U; run(100U); idle(); speed(25.0f); run(5U);
    /* A fresh reply after a main-loop pause must not hide the preceding gap. */
    uint8_t data[] = {0x35U, 0U, 0U, 25U, 0x6BU};
    tick += 100U; receive(data, 5U); ZDT_CAN_Process(tick);
    assert(motors[0].target_speed == 0.0f && snapshot().wheel[0].gap_max >= 100U);
    run(120U); idle(); speed(25.0f);
    /* Late TXOK is counted as transmitted, but its motion target is cancelled. */
    tick += 50U; TestCan_Complete(1U); ZDT_CAN_Process(tick);
    assert(motors[0].target_speed == 0.0f && snapshot().tx_timeout > 0U);
    reset(0U); warm(); speed(25.0f);
    for (unsigned n = 0U; n < 200U; ++n) { tick++; ZDT_CAN_Process(tick); }
    assert(snapshot().fault_cancels == 1U && test_can_abort_calls == 1U);
}
static void test_sustained_simulation(void)
{
    reset(0U); warm(); run(30000U);
    ZDT_CanStats s = snapshot();
    assert(s.speed_replies > 1400U && sample().sequence == s.speed_replies);
    assert(s.wheel[0].interval_max <= 20U && s.wheel[0].rtt_max == 1U);
    assert(s.wheel[0].age_max <= 20U && !s.wheel[0].gap_max);
    assert(s.tx_done > 1500U && !s.software_restarts && test_can_start_calls == 1U);
    printf("Synthetic 30s: speed replies=%lu interval_max=%lu age_avg=%.1f age_max=%lu rtt_max=%lu ms; NOT hardware evidence.\n",
           (unsigned long)s.speed_replies, (unsigned long)s.wheel[0].interval_max,
           (double)s.wheel[0].age_sum / s.wheel[0].age_samples,
           (unsigned long)s.wheel[0].age_max, (unsigned long)s.wheel[0].rtt_max);
}
static void test_slow_tx_fairness(void)
{
    reset(0U);
    /* A valid frame can complete after the 20 ms poll period, before timeout.
     * Background polling must leave slots for stop and capability/status reads. */
    for (unsigned n = 0U; n < 900U; ++n) {
        if (test_can_pending &&
            (uint32_t)(tick - test_can_frames[test_can_count - 1U].tick) >= 29U) step();
        else { tick++; ZDT_CAN_Process(tick); }
    }
    assert(count_code(0x35U, 0U) > 2U);
    assert(count_code(0xFEU, 0U) > 0U && count_code(0xF6U, 0U) > 0U);
    assert(count_code(0x1AU, 0U) > 0U && count_code(0x3AU, 0U) > 0U);
    assert(snapshot().stop_confirmed && remote_rpm == 0.0f);
}

static void test_power_on_without_enable_request(void)
{
    reset(0U); warm(); speed(30.0f); run(30U);
    assert(sample().rpm == 30.0f && motors[0].target_speed == 30.0f);
    /* The reported enable bit is diagnostic, not an admission/cancel gate. */
    remote_status = 0x80U;
    uint8_t state[] = {0x3AU, 0x80U, 0x6BU};
    receive(state, 3U); ZDT_CAN_Process(tick); idle();
    assert(ZDT_CAN_ReadyByID(1U));
    speed(15.0f); run(30U);
    assert(motors[0].target_speed == 15.0f && sample().rpm == 15.0f);
    assert(!count_code(0xF3U, 0U));
    /* Actual protection status still cancels motion. */
    state[1] = 0x88U; receive(state, 3U); ZDT_CAN_Process(tick);
    assert(!ZDT_CAN_ReadyByID(1U) && motors[0].target_speed == 0.0f);
}

int main(void)
{
    test_init_and_failure(); test_format_and_scale(); test_real_feedback_only();
    test_stop_cancel_and_confirmation(); test_outage_and_busoff();
    test_stuck_mailbox_and_fair_retry(); test_late_motor_and_reset();
    test_overwrite_nack_and_wrap(); test_submit_failure_and_delayed_consumer(); test_sustained_simulation();
    test_slow_tx_fairness();
    test_power_on_without_enable_request();
    puts("ZDT batch 1: 12 portable safety/transport groups passed.");
    return 0;
}
