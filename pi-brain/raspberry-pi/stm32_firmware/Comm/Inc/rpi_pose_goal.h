#ifndef RPI_POSE_GOAL_H
#define RPI_POSE_GOAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "rpi_app_commands.h"
#include "rpi_transport.h"

typedef enum
{
    RPI_EVENT_POSE_STARTED = 0x10,
    RPI_EVENT_POSE_REACHED = 0x11,
    RPI_EVENT_POSE_CANCELLED = 0x12,
    RPI_EVENT_MOTION_FAULT = 0x13
} RpiPoseEventCode;

HAL_StatusTypeDef RpiPoseGoal_SendStarted(uint32_t goal_id);

HAL_StatusTypeDef RpiPoseGoal_SendReached(uint32_t goal_id,
                                         int32_t x_mm,
                                         int32_t y_mm,
                                         int32_t yaw_mrad,
                                         int32_t position_error_mm,
                                         int32_t yaw_error_mrad);

HAL_StatusTypeDef RpiPoseGoal_SendCancelled(uint32_t goal_id);

HAL_StatusTypeDef RpiPoseGoal_SendFault(uint32_t goal_id,
                                       RpiMotionFaultReason reason);

#ifdef __cplusplus
}
#endif

#endif /* RPI_POSE_GOAL_H */
