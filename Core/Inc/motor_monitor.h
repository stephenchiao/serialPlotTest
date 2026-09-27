#ifndef MOTOR_MONITOR_H
#define MOTOR_MONITOR_H
#include <stdint.h>
#define MOTOR_FEEDBACK_TIMEOUT_MS 300U
#define MOTOR_STOP_TIMEOUT_MS 600U
#define MOTOR_ZERO_RPM 1.0f
typedef struct {
    float rpm;
    uint32_t tick, sequence;
    uint8_t valid, zero_streak;
} MotorFeedback;
typedef enum { MOTOR_STOP_IDLE, MOTOR_STOP_REQUESTED, MOTOR_STOP_WAIT_FEEDBACK,
               MOTOR_STOP_CONFIRMED, MOTOR_STOP_UNCONFIRMED, MOTOR_STOP_SENT } MotorStopState;
typedef struct {
    MotorStopState state;
    uint32_t requested_tick, confirmed_tick, baseline[4];
    uint8_t baseline_valid;
} MotorStopMonitor;
void MotorFeedback_Record(MotorFeedback *sample, float rpm, uint32_t now);
uint8_t MotorFeedback_FreshMask(const MotorFeedback samples[4], uint32_t now);
void MotorStop_Request(MotorStopMonitor *monitor, uint32_t now);
void MotorStop_UpdateMasked(MotorStopMonitor *monitor,
                            const MotorFeedback samples[4],
                            uint8_t required_mask,
                            uint8_t tx_pending, uint32_t now);
void MotorStop_Update(MotorStopMonitor *monitor, const MotorFeedback samples[4],
                      uint8_t tx_pending, uint32_t now);
const char *MotorStop_Name(MotorStopState state);
#endif
