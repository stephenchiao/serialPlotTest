#include "motor_monitor.h"
#include <math.h>
void MotorFeedback_Record(MotorFeedback *sample, float rpm, uint32_t now)
{
    if (!isfinite(rpm)) return;
    sample->rpm = rpm; sample->tick = now; sample->sequence++; sample->valid = 1U;
    if (fabsf(rpm) <= MOTOR_ZERO_RPM) {
        if (sample->zero_streak < 255U) sample->zero_streak++;
    } else sample->zero_streak = 0U;
}
uint8_t MotorFeedback_FreshMask(const MotorFeedback samples[4], uint32_t now)
{
    uint8_t i, mask = 0U;
    for (i = 0U; i < 4U; ++i)
        if (samples[i].valid && (uint32_t)(now - samples[i].tick) <= MOTOR_FEEDBACK_TIMEOUT_MS)
            mask |= (uint8_t)(1U << i);
    return mask;
}
void MotorStop_Request(MotorStopMonitor *monitor, uint32_t now)
{
    /* 同一未完成请求的重发不能刷新超时起点。 */
    if (monitor->state == MOTOR_STOP_REQUESTED || monitor->state == MOTOR_STOP_WAIT_FEEDBACK ||
        monitor->state == MOTOR_STOP_UNCONFIRMED) return;
    monitor->state = MOTOR_STOP_REQUESTED;
    monitor->requested_tick = now;
    monitor->baseline_valid = 0U;
}
void MotorStop_UpdateMasked(MotorStopMonitor *monitor,
                            const MotorFeedback samples[4],
                            uint8_t required_mask,
                            uint8_t tx_pending, uint32_t now)
{
    uint8_t i;
    uint8_t stopped;

    required_mask &= 0x0FU;
    stopped = required_mask != 0U &&
              (MotorFeedback_FreshMask(samples, now) & required_mask) == required_mask;
    if (monitor->state == MOTOR_STOP_IDLE) return;
    if (!monitor->baseline_valid && !tx_pending) {
        for (i = 0U; i < 4U; ++i) monitor->baseline[i] = samples[i].sequence;
        monitor->baseline_valid = 1U;
        monitor->state = MOTOR_STOP_WAIT_FEEDBACK;
    }
    if (!monitor->baseline_valid || tx_pending) stopped = 0U;
    for (i = 0U; i < 4U; ++i) {
        uint32_t count = samples[i].sequence - monitor->baseline[i];
        if (!(required_mask & (uint8_t)(1U << i))) continue;
        if (count < 2U || count >= 0x80000000UL || samples[i].zero_streak < 2U)
            stopped = 0U;
    }
    if (stopped) {
        if (monitor->state != MOTOR_STOP_CONFIRMED) monitor->confirmed_tick = now;
        monitor->state = MOTOR_STOP_CONFIRMED;
    } else if ((uint32_t)(now - monitor->requested_tick) > MOTOR_STOP_TIMEOUT_MS) {
        monitor->state = MOTOR_STOP_UNCONFIRMED;
    } else if (monitor->state == MOTOR_STOP_CONFIRMED) {
        monitor->state = MOTOR_STOP_WAIT_FEEDBACK;
    }
}

void MotorStop_Update(MotorStopMonitor *monitor, const MotorFeedback samples[4],
                      uint8_t tx_pending, uint32_t now)
{
    MotorStop_UpdateMasked(monitor, samples, 0x0FU, tx_pending, now);
}
const char *MotorStop_Name(MotorStopState state)
{
    static const char *names[] = {"IDLE", "REQUESTED", "WAIT_FEEDBACK", "CONFIRMED", "UNCONFIRMED", "SENT"};
    return names[(unsigned int)state <= MOTOR_STOP_SENT ? state : MOTOR_STOP_UNCONFIRMED];
}
