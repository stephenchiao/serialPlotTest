#ifndef RPI_APP_COMMANDS_H
#define RPI_APP_COMMANDS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "rpi_link.h"

typedef enum
{
    RPI_POSE_IDLE = 0x00,
    RPI_POSE_ACCEPTED = 0x01,
    RPI_POSE_MOVING = 0x02,
    RPI_POSE_REACHED = 0x03,
    RPI_POSE_CANCELLED = 0x04,
    RPI_POSE_FAULT = 0x05
} RpiPoseGoalState;

typedef enum
{
    RPI_MOTION_FAULT_UNSPECIFIED = 0x0000,
    RPI_MOTION_FAULT_OPS9_LOST = 0x0001,
    RPI_MOTION_FAULT_HOST_LOST = 0x0002,
    RPI_MOTION_FAULT_CAN = 0x0003,
    RPI_MOTION_FAULT_OUT_OF_BOUNDS = 0x0004,
    RPI_MOTION_FAULT_TIMEOUT = 0x0005,
    RPI_MOTION_FAULT_CANCEL_TIMEOUT = 0x0006,
    RPI_MOTION_FAULT_INTERNAL = 0x00FF
} RpiMotionFaultReason;

typedef struct
{
    uint32_t goal_id;
    int32_t x_mm;
    int32_t y_mm;
    int32_t yaw_mrad;
    uint32_t timeout_ms;
} RpiPoseGoal;

typedef struct
{
    uint32_t goal_id;
    RpiPoseGoalState state;
    int32_t x_mm;
    int32_t y_mm;
    int32_t yaw_mrad;
    uint16_t fault_reason;
} RpiPoseGoalStatus;

/*
 * 以下钩子由底盘项目实现。默认 weak 实现返回 INTERNAL_ERROR，避免在尚未
 * 接入电机/舵机时向树莓派误报执行成功。
 */
RpiResponseStatus RpiApp_StopAll(void);
RpiResponseStatus RpiApp_SetChassisVelocity(int16_t vx_mm_s,
                                            int16_t vy_mm_s,
                                            int16_t wz_mrad_s);
RpiResponseStatus RpiApp_SetServoAngle(uint8_t servo_id,
                                      int16_t angle_tenths_degree);
RpiResponseStatus RpiApp_QueryStatus(uint8_t *output,
                                    uint16_t output_capacity,
                                    uint16_t *output_length);
RpiResponseStatus RpiApp_SetTaskCode(const uint8_t *ascii_code,
                                    uint16_t code_length);
RpiResponseStatus RpiApp_SetPoseGoal(const RpiPoseGoal *goal);
RpiResponseStatus RpiApp_CancelPoseGoal(uint32_t goal_id);
RpiResponseStatus RpiApp_GetPoseGoalStatus(RpiPoseGoalStatus *status);

#ifdef __cplusplus
}
#endif

#endif /* RPI_APP_COMMANDS_H */
