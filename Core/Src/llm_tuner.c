#include "llm_tuner.h"
#include "motion_math.h"

#include "mecanum_chassis.h"
#include "ops9.h"
#include "can.h"
#include "zdtCan.h"
#include "zdtEmm.h"
#include "control_runtime.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define LLM_TUNE_OVERTRAVEL_MM       50.0f
#define LLM_TUNE_WRONG_DIR_MM        25.0f
#define LLM_TUNE_MAX_YAW_ERROR_DEG   15.0f
#define LLM_TUNE_SETTLE_CYCLES       10U
#define LLM_TUNE_HOLD_LINEAR_MPS     0.10f
#define LLM_TUNE_HOLD_YAW_RADPS      0.15f
#define LLM_TUNE_CROSS_TRACK_MM      50.0f
#define LLM_TUNE_YAW_TRANSLATION_MM  50.0f
#define LLM_TUNE_TELEMETRY_PERIOD_MS 50U
#define LLM_TUNE_ARM_TIMEOUT_MS      2000U

typedef enum {
    LLM_TUNE_STATE_WAIT = 0,
    LLM_TUNE_STATE_ARMING,
    LLM_TUNE_STATE_RUN
} LLM_TuneState_t;

static PID_Controller *tuner_pid_x;
static PID_Controller *tuner_pid_y;
static PID_Controller *tuner_pid_yaw;
static LLM_TuneState_t tuner_state = LLM_TUNE_STATE_WAIT;
static LLM_TuneAxis_t tuner_axis = LLM_TUNE_AXIS_Y;
static uint32_t tune_start_time;
static uint32_t last_control_time;
static uint32_t last_telemetry_time;
static float start_x_pos;
static float start_y_pos;
static float start_yaw_deg;
static float tune_direction = 1.0f;
static float tune_output;
static uint32_t tune_round_count;
static uint16_t tune_settle_cycles;









static float BrakeLimitLinear(float error_mm)
{
    float remaining_m = (fabsf(error_mm) - LLM_TUNE_POSITION_TOL_MM) / 1000.0f;
    if (remaining_m <= 0.0f) return 0.0f;
    return sqrtf(2.0f * LLM_TUNE_MAX_DECEL_MPS2 * remaining_m);
}

static float BrakeLimitYaw(float error_deg)
{
    float remaining_rad = (fabsf(error_deg) - LLM_TUNE_YAW_TOL_DEG) *
                          (3.1415926f / 180.0f);
    if (remaining_rad <= 0.0f) return 0.0f;
    return sqrtf(2.0f * LLM_TUNE_YAW_DECEL_RADPS2 * remaining_rad);
}

void LLM_TunerInit(PID_Controller *pid_x,
                   PID_Controller *pid_y,
                   PID_Controller *pid_yaw)
{
    tuner_pid_x = pid_x;
    tuner_pid_y = pid_y;
    tuner_pid_yaw = pid_yaw;
    LLM_TunerResetSession();
}

void LLM_TunerSetAxis(LLM_TuneAxis_t axis)
{
    tuner_axis = axis;
}

LLM_TuneAxis_t LLM_TunerGetAxis(void)
{
    return tuner_axis;
}

const char *LLM_TunerAxisName(LLM_TuneAxis_t axis)
{
    if (axis == LLM_TUNE_AXIS_X) return "X";
    if (axis == LLM_TUNE_AXIS_YAW) return "YAW";
    return "Y";
}

PID_Controller *LLM_TunerGetPidForAxis(LLM_TuneAxis_t axis)
{
    if (axis == LLM_TUNE_AXIS_X) return tuner_pid_x;
    if (axis == LLM_TUNE_AXIS_YAW) return tuner_pid_yaw;
    return tuner_pid_y;
}

PID_Controller *LLM_TunerGetPid(void)
{
    return LLM_TunerGetPidForAxis(tuner_axis);
}

uint8_t LLM_TunerIsRunning(void)
{
    return tuner_state == LLM_TUNE_STATE_RUN ? 1U : 0U;
}

static void LLM_TunerBeginRun(uint32_t now)
{
    const OPS9_Snapshot ops = OPS9_GetSnapshot();
    PID_Controller *pid = LLM_TunerGetPid();
    float center_x;
    float center_y;

    if (tune_round_count > 0U) tune_direction = -tune_direction;
    tune_round_count++;
    PID_Reset(tuner_pid_x);
    PID_Reset(tuner_pid_y);
    PID_Reset(tuner_pid_yaw);
    Motion_OpsToCenter(ops.x_mm, ops.y_mm, ops.yaw_deg, &center_x, &center_y);
    start_x_pos = center_x;
    start_y_pos = center_y;
    start_yaw_deg = ops.yaw_deg;
    PID_SetTarget(pid, tuner_axis == LLM_TUNE_AXIS_YAW ?
                       LLM_TUNE_TARGET_YAW_DEG : LLM_TUNE_TARGET_MM);
    tune_output = 0.0f;
    tune_settle_cycles = 0U;
    tune_start_time = now;
    last_control_time = now;
    last_telemetry_time = now;
    tuner_state = LLM_TUNE_STATE_RUN;
    printf("# ROUND START %lu AXIS=%s DIR %.0f X=%.2f Y=%.2f YAW=%.2f "
           "CENTER_X=%.2f CENTER_Y=%.2f\r\n",
           (unsigned long)tune_round_count, LLM_TunerAxisName(tuner_axis),
           tune_direction, ops.x_mm, ops.y_mm, ops.yaw_deg, center_x, center_y);
}

uint8_t LLM_TunerGetState(void)
{
    return (uint8_t)tuner_state;
}

void LLM_TunerAbort(void)
{
    tune_output = 0.0f;
    tune_settle_cycles = 0U;
    tuner_state = LLM_TUNE_STATE_WAIT;
}

void LLM_TunerResetSession(void)
{
    LLM_TunerAbort();
    tune_round_count = 0U;
    tune_direction = 1.0f;
}

void LLM_TunerStartRound(void)
{
    uint32_t now = HAL_GetTick();
    PID_Controller *pid = LLM_TunerGetPid();

    if (pid == NULL || tuner_pid_x == NULL ||
        tuner_pid_y == NULL || tuner_pid_yaw == NULL) {
        StopAllMotors();
        LLM_TunerAbort();
        printf("# ERROR TUNER NOT INITIALIZED\r\n");
        return;
    }
    if (tune_round_count >= LLM_TUNE_MAX_SESSION_ROUNDS) {
        StopAllMotors();
        LLM_TunerAbort();
        printf("# ERROR TUNE ROUND LIMIT MAX=%lu\r\n",
               (unsigned long)LLM_TUNE_MAX_SESSION_ROUNDS);
        return;
    }

    /* Stopping cancels old speed mailboxes. Wait for stop-frame transmission
     * and CAN recovery before announcing a runnable round. */
    StopAllMotors();
    /* ARMING waits for stop-frame transmission and the idle recovery gate. */
    tune_start_time = now;
    tuner_state = LLM_TUNE_STATE_ARMING;
    printf("# ROUND ARMING AXIS=%s\r\n", LLM_TunerAxisName(tuner_axis));
}

void LLM_TunerStopRound(const char *reason)
{
    const OPS9_Snapshot ops = OPS9_GetSnapshot();
    ZDT_CAN_Stats_t can_stats;
    uint32_t can_error = HAL_CAN_GetError(&hcan1);
    float center_x;
    float center_y;

    ZDT_CAN_GetStats(&can_stats);
    StopAllMotors();
    LLM_TunerAbort();
    Motion_OpsToCenter(ops.x_mm, ops.y_mm, ops.yaw_deg, &center_x, &center_y);
    if (strcmp(reason, "CAN FAULT") == 0) {
        printf("# ROUND STOP CAN FAULT ERROR=0x%08lX ESR=0x%08lX "
               "FATAL_CB=%lu TX_TIMEOUT=%lu AXIS=%s X=%.2f Y=%.2f YAW=%.2f "
               "CENTER_X=%.2f CENTER_Y=%.2f\r\n",
               (unsigned long)can_error, (unsigned long)can_stats.esr,
               (unsigned long)can_stats.fatal_error_callbacks,
               (unsigned long)can_stats.tx_timeout,
               LLM_TunerAxisName(tuner_axis), ops.x_mm, ops.y_mm, ops.yaw_deg,
               center_x, center_y);
    } else {
        printf("# ROUND STOP %s AXIS=%s X=%.2f Y=%.2f YAW=%.2f "
               "CENTER_X=%.2f CENTER_Y=%.2f\r\n",
               reason, LLM_TunerAxisName(tuner_axis), ops.x_mm, ops.y_mm, ops.yaw_deg,
               center_x, center_y);
    }
}

void LLM_TunerProcess(uint32_t now)
{
    const OPS9_Snapshot ops = OPS9_GetSnapshot();
    float current_ops_x;
    float current_ops_y;
    float current_x;
    float current_y;
    float current_yaw;
    float dx;
    float dy;
    float heading_rad;
    float body_right_mm;
    float body_forward_mm;
    float yaw_delta_deg;
    float normalized_input;
    float normalized_error;
    float cross_track_mm;
    float target_value;
    float tolerance;
    float brake_limit;
    float desired_output;
    float hold_cross_output = 0.0f;
    float hold_yaw_output = 0.0f;
    float max_output_step;
    float command_vx = 0.0f;
    float command_vy = 0.0f;
    float command_vz = 0.0f;
    float v1, v2, v3, v4;
    PID_Controller *pid;
    uint32_t last_ops_tick;
    uint32_t elapsed_ms;
    float dt_s;

    if (tuner_state == LLM_TUNE_STATE_ARMING) {
        uint8_t ops_ready;
        uint8_t stop_confirmed;
        uint8_t can_ready;
        now = HAL_GetTick();
        ops_ready = isfinite(ops.x_mm) && isfinite(ops.y_mm) &&
                    isfinite(ops.yaw_deg) && ops.frame_count > 0U &&
                    (uint32_t)(now - ops.last_update_tick) <= LLM_TUNE_OPS_TIMEOUT_MS;
        stop_confirmed = Mecanum_GetStopStatus().state == MOTOR_STOP_SENT;
        can_ready = ZDT_CAN_IsReady();
        if (ops_ready && stop_confirmed && can_ready) {
            LLM_TunerBeginRun(now);
        } else if ((uint32_t)(now - tune_start_time) > LLM_TUNE_ARM_TIMEOUT_MS) {
            if (!ops_ready) LLM_TunerStopRound("OPS NOT READY");
            else if (!stop_confirmed) LLM_TunerStopRound("STOP NOT SENT");
            else LLM_TunerStopRound("CAN NOT READY");
        }
        return;
    }
    if (!LLM_TunerIsRunning()) {
        return;
    }

    now = HAL_GetTick();
    elapsed_ms = (uint32_t)(now - last_control_time);
    if (elapsed_ms < LLM_TUNE_CONTROL_PERIOD_MS) return;
    ControlRuntime_RecordControl(elapsed_ms);
    if (elapsed_ms > LLM_TUNE_DT_MAX_MS) elapsed_ms = LLM_TUNE_DT_MAX_MS;
    dt_s = (float)elapsed_ms / 1000.0f;

    current_ops_x = ops.x_mm;
    current_ops_y = ops.y_mm;
    current_yaw = ops.yaw_deg;
    last_ops_tick = ops.last_update_tick;
    now = HAL_GetTick();
    last_control_time = now;
    if (!isfinite(current_ops_x) || !isfinite(current_ops_y) ||
        !isfinite(current_yaw) || ops.frame_count == 0U ||
        (uint32_t)(now - last_ops_tick) > LLM_TUNE_OPS_TIMEOUT_MS) {
        LLM_TunerStopRound("OPS LOST");
        return;
    }
    /* CAN runtime safety is evaluated once by ChassisSafety_Process. */
    /*
     * TUNE 自动轮次固定最多运行 5 秒，并保留 OPS、方向、漂移和越界保护。
     * 为便于使用普通串口助手观察完整输出，调参轮次不依赖主机 PING。
     */
    if ((uint32_t)(now - tune_start_time) >= LLM_TUNE_DURATION_MS) {
        LLM_TunerStopRound("TIMEOUT");
        return;
    }

    Motion_OpsToCenter(current_ops_x, current_ops_y, current_yaw,
                       &current_x, &current_y);
    dx = current_x - start_x_pos;
    dy = current_y - start_y_pos;
    heading_rad = start_yaw_deg * (3.1415926f / 180.0f);
    body_right_mm = cosf(heading_rad) * dx + sinf(heading_rad) * dy;
    body_forward_mm = -sinf(heading_rad) * dx + cosf(heading_rad) * dy;
    yaw_delta_deg = Motion_AngleDeltaDeg(current_yaw, start_yaw_deg);

    if (tuner_axis == LLM_TUNE_AXIS_X) {
        normalized_input = tune_direction * body_right_mm;
        cross_track_mm = body_forward_mm;
        target_value = LLM_TUNE_TARGET_MM;
        tolerance = LLM_TUNE_POSITION_TOL_MM;
    } else if (tuner_axis == LLM_TUNE_AXIS_YAW) {
        normalized_input = tune_direction * yaw_delta_deg;
        cross_track_mm = sqrtf(dx * dx + dy * dy);
        target_value = LLM_TUNE_TARGET_YAW_DEG;
        tolerance = LLM_TUNE_YAW_TOL_DEG;
    } else {
        normalized_input = tune_direction * body_forward_mm;
        cross_track_mm = body_right_mm;
        target_value = LLM_TUNE_TARGET_MM;
        tolerance = LLM_TUNE_POSITION_TOL_MM;
    }
    normalized_error = target_value - normalized_input;

    if (normalized_input < -(tuner_axis == LLM_TUNE_AXIS_YAW ?
                             3.0f : LLM_TUNE_WRONG_DIR_MM)) {
        LLM_TunerStopRound("WRONG DIR");
        return;
    }
    if (tuner_axis != LLM_TUNE_AXIS_YAW &&
        fabsf(yaw_delta_deg) > LLM_TUNE_MAX_YAW_ERROR_DEG) {
        LLM_TunerStopRound("YAW LIMIT");
        return;
    }
    if (tuner_axis == LLM_TUNE_AXIS_YAW) {
        if (cross_track_mm > LLM_TUNE_YAW_TRANSLATION_MM) {
            LLM_TunerStopRound("TRANSLATION LIMIT");
            return;
        }
    } else if (fabsf(cross_track_mm) > LLM_TUNE_CROSS_TRACK_MM) {
        LLM_TunerStopRound("CROSS TRACK");
        return;
    }
    if (normalized_input > target_value +
        (tuner_axis == LLM_TUNE_AXIS_YAW ? 10.0f : LLM_TUNE_OVERTRAVEL_MM)) {
        LLM_TunerStopRound("OVERTRAVEL");
        return;
    }

    if (fabsf(normalized_error) <= tolerance) {
        tune_settle_cycles++;
        if (tune_settle_cycles >= LLM_TUNE_SETTLE_CYCLES) {
            LLM_TunerStopRound("TARGET");
            return;
        }
    } else {
        tune_settle_cycles = 0U;
    }

    pid = LLM_TunerGetPid();
    desired_output = PID_CalcDt(pid, normalized_input, dt_s);
    brake_limit = tuner_axis == LLM_TUNE_AXIS_YAW ?
                  BrakeLimitYaw(normalized_error) : BrakeLimitLinear(normalized_error);
    if (brake_limit < pid->max_out) {
        desired_output = Motion_Clamp(desired_output, -brake_limit, brake_limit);
    }
    desired_output *= tune_direction;
    max_output_step = (tuner_axis == LLM_TUNE_AXIS_YAW ?
                       (fabsf(desired_output) < fabsf(tune_output) ?
                        LLM_TUNE_YAW_DECEL_RADPS2 : LLM_TUNE_YAW_ACCEL_RADPS2) :
                       (fabsf(desired_output) < fabsf(tune_output) ?
                        LLM_TUNE_MAX_DECEL_MPS2 : LLM_TUNE_MAX_ACCEL_MPS2)) *
                       dt_s;
    tune_output = Motion_Slew(tune_output, desired_output, max_output_step);

    if (tuner_axis == LLM_TUNE_AXIS_X) {
        command_vx = tune_output;
        hold_cross_output = PID_CalcErrorDt(tuner_pid_y, -body_forward_mm, dt_s);
        hold_yaw_output = PID_CalcErrorDt(tuner_pid_yaw, -yaw_delta_deg, dt_s);
        command_vy = Motion_Clamp(hold_cross_output,
                                -LLM_TUNE_HOLD_LINEAR_MPS,
                                LLM_TUNE_HOLD_LINEAR_MPS);
        command_vz = Motion_Clamp(hold_yaw_output,
                                -LLM_TUNE_HOLD_YAW_RADPS,
                                LLM_TUNE_HOLD_YAW_RADPS);
    } else if (tuner_axis == LLM_TUNE_AXIS_Y) {
        command_vy = tune_output;
        hold_cross_output = PID_CalcErrorDt(tuner_pid_x, -body_right_mm, dt_s);
        hold_yaw_output = PID_CalcErrorDt(tuner_pid_yaw, -yaw_delta_deg, dt_s);
        command_vx = Motion_Clamp(hold_cross_output,
                                -LLM_TUNE_HOLD_LINEAR_MPS,
                                LLM_TUNE_HOLD_LINEAR_MPS);
        command_vz = Motion_Clamp(hold_yaw_output,
                                -LLM_TUNE_HOLD_YAW_RADPS,
                                LLM_TUNE_HOLD_YAW_RADPS);
    } else {
        command_vz = tune_output;
    }

    Mecanum_Kinematics(command_vx, command_vy, command_vz, &v1, &v2, &v3, &v4);
    if (SetAllMotorsSpeed(v1, v2, v3, v4) != 0U) {
        LLM_TunerStopRound("CAN FAULT");
        return;
    }
    PID_ApplyOutput(pid, Mecanum_GetAppliedScale() * tune_direction * tune_output);
    if (tuner_axis == LLM_TUNE_AXIS_X) PID_ApplyOutput(tuner_pid_y, Mecanum_GetAppliedScale() * command_vy);
    if (tuner_axis == LLM_TUNE_AXIS_Y) PID_ApplyOutput(tuner_pid_x, Mecanum_GetAppliedScale() * command_vx);
    if (tuner_axis != LLM_TUNE_AXIS_YAW) PID_ApplyOutput(tuner_pid_yaw, Mecanum_GetAppliedScale() * command_vz);
    if ((uint32_t)(now - last_telemetry_time) >= LLM_TUNE_TELEMETRY_PERIOD_MS) {
        last_telemetry_time = now;
        printf("%lu,%.2f,%.2f,%.4f,%.2f,%.7f,%.8f,%.7f,"
               "%.2f,%.2f,%.2f,%.2f,%.2f,%.4f,%.4f,%.2f,%.2f\r\n",
               (unsigned long)(now - tune_start_time), target_value,
               normalized_input, tune_direction * tune_output, normalized_error,
               pid->Kp, pid->Ki, pid->Kd, current_ops_x, current_ops_y, current_yaw,
               cross_track_mm, yaw_delta_deg,
               tuner_axis == LLM_TUNE_AXIS_Y ? command_vx : command_vy,
               command_vz, current_x, current_y);
    }
}
