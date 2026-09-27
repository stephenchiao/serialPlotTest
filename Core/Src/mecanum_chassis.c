/*
 * mecanum_chassis.c
 *
 *  Created on: Mar 7, 2026
 *      Author: steph
 */
/*
 * @brief  麦克纳姆轮车体速度逆解算
 * @param  Vx: 车体向右速度，单位 m/s
 * @param  Vy: 车体向前速度，单位 m/s
 * @param  Vz: 沿 OPS 航向角正方向的角速度，单位 rad/s
 * @retval 算出各轮目标线速度，存入指针
 */
#include "mecanum_chassis.h"
#include "zdtEmm.h"
#include "zdtCan.h"
#include <math.h>

static MotorStopMonitor stop_monitor;
static uint32_t stop_motion_generation, stop_retry_tick;
static float applied_scale = 1.0f;
static uint8_t required_motor_mask = 0x0FU;
static uint32_t poll_failures;

float Mecanum_GetAppliedScale(void) { return applied_scale; }
MotorStopMonitor Mecanum_GetStopStatus(void) { return stop_monitor; }
uint8_t Mecanum_GetRequiredMotorMask(void) { return required_motor_mask; }
uint8_t Mecanum_SetRequiredMotorMask(uint8_t mask)
{
    if (mask == 0U || (mask & 0xF0U) != 0U) return 0U;
    required_motor_mask = mask;
    stop_monitor.state = MOTOR_STOP_IDLE;
    stop_monitor.baseline_valid = 0U;
    return 1U;
}
uint8_t Mecanum_FeedbackReady(uint8_t mask)
{
    MotorFeedback samples[4];
    ZDT_Emm_GetFeedback(samples);
    return (MotorFeedback_FreshMask(samples, HAL_GetTick()) & mask) == mask;
}
void Mecanum_ProcessFeedback(uint32_t now)
{
    if (stop_motion_generation != ZDT_Emm_MotionGeneration()) {
        stop_monitor.state = MOTOR_STOP_IDLE;
        return;
    }
    if (stop_monitor.state == MOTOR_STOP_IDLE) return;
    if (ZDT_CAN_StopSent(required_motor_mask)) {
        if (stop_monitor.state != MOTOR_STOP_SENT) stop_monitor.confirmed_tick = now;
        stop_monitor.state = MOTOR_STOP_SENT; /* no physical zero-speed assertion */
    } else if ((uint32_t)(now - stop_monitor.requested_tick) > MOTOR_STOP_TIMEOUT_MS) {
        stop_monitor.state = MOTOR_STOP_UNCONFIRMED;
    }
    if (!ZDT_CAN_StopSent(required_motor_mask) && (uint32_t)(now - stop_retry_tick) >= 100U) {
        stop_retry_tick = now;
        Mecanum_ReportCanTxResult(ZDT_Emm_StopMask(required_motor_mask));
    }
}

void Mecanum_Kinematics(float Vx, float Vy, float Vz, float *V_bl, float *V_fl, float *V_fr, float *V_br) {
    float L = (ROBOT_H / 2.0f) + (ROBOT_W / 2.0f);

    *V_bl = -Vx + Vy - Vz * L;  // ID 1 左后
    *V_fl =  Vx + Vy - Vz * L;  // ID 2 左前
    *V_fr =  Vx - Vy - Vz * L;  // ID 3 右前
    *V_br = -Vx - Vy - Vz * L;  // ID 4 右后
}

/*
 * @brief  线速度 (m/s) 转 电机转速 (RPM)
 */
float MsToRpm(float v_ms) {
    if (WHEEL_DIAMETER <= 0.0f) return 0.0f;
    // V = RPM * π * D / 60  =>  RPM = V * 60 / (π * D)
    return v_ms * 60.0f / (3.1415926f * WHEEL_DIAMETER);
}

/*
 * @brief  设置 4 个轮子速度并下发至 CAN 节点
 */
uint8_t SetAllMotorsSpeed(float V_bl, float V_fl, float V_fr, float V_br) {
    float max_abs = fabsf(V_bl);
    float scale;
    uint8_t result = 0U;
    uint8_t mask = required_motor_mask;

    applied_scale = 1.0f;
    /* Feedback is optional; finite targets and speed limits remain mandatory. */
    if (!isfinite(V_bl) || !isfinite(V_fl) || !isfinite(V_fr) || !isfinite(V_br)) {
        applied_scale = 0.0f;
        ZDT_CAN_RaiseFault();
        (void)StopAllMotors();
        return 4U;
    }

    if (fabsf(V_fl) > max_abs) max_abs = fabsf(V_fl);
    if (fabsf(V_fr) > max_abs) max_abs = fabsf(V_fr);
    if (fabsf(V_br) > max_abs) max_abs = fabsf(V_br);
    if (max_abs > MECANUM_MAX_WHEEL_SPEED_MPS) {
        /* 四轮同时按比例缩放，保留期望的平移与旋转方向比例。 */
        scale = MECANUM_MAX_WHEEL_SPEED_MPS / max_abs;
        applied_scale = scale;
        V_bl *= scale;
        V_fl *= scale;
        V_fr *= scale;
        V_br *= scale;
    }

    if (mask & 0x01U) result |= ZDT_Emm_SetSpeedByID(1, MsToRpm(V_bl));  // ID 1: 左后
    if (mask & 0x02U) result |= ZDT_Emm_SetSpeedByID(2, MsToRpm(V_fl));  // ID 2: 左前
    if (mask & 0x04U) result |= ZDT_Emm_SetSpeedByID(3, MsToRpm(V_fr));  // ID 3: 右前
    if (mask & 0x08U) result |= ZDT_Emm_SetSpeedByID(4, MsToRpm(V_br));  // ID 4: 右后
    if (result != 0U) {
        ZDT_CAN_RaiseFault();
        applied_scale = 0.0f;
        /* 任一轮入队失败时撤销其他轮已入队的速度，避免部分下发。 */
        (void)StopAllMotors();
    }
    return result;
}

/*
 * @brief  向 4 个电机发送读取速度的指令
 */
void ReadAllMotorsSpeed(void) {
    ZDT_Emm_ReadSpeedByID(1);
    ZDT_Emm_ReadSpeedByID(2);
    ZDT_Emm_ReadSpeedByID(3);
    ZDT_Emm_ReadSpeedByID(4);
}

/*
 * @brief  紧急停止所有电机
 */
uint8_t StopAllMotors(void) {
    uint8_t result;
    if (stop_motion_generation != ZDT_Emm_MotionGeneration()) stop_monitor.state = MOTOR_STOP_IDLE;
    MotorStop_Request(&stop_monitor, HAL_GetTick());
    stop_motion_generation = ZDT_Emm_MotionGeneration();
    stop_retry_tick = HAL_GetTick();
    result = ZDT_Emm_StopMask(required_motor_mask);
    if (result) ZDT_CAN_RaiseFault();
    return result;
}

uint8_t Mecanum_ConsumeCanTxFault(void)
{
    return ZDT_CAN_ConsumeFault();
}

void Mecanum_ReportCanTxResult(uint8_t result)
{
    if (result != 0U) ZDT_CAN_RaiseFault();
}

void Mecanum_ReportPollResult(uint8_t result)
{
    if (result != 0U) poll_failures++;
}

uint32_t Mecanum_GetPollFailures(void)
{
    return poll_failures;
}


