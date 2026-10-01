#include "rpi_pose_goal.h"

#include <stddef.h>

static void write_u16_le(uint8_t *output, uint16_t value)
{
    output[0] = (uint8_t)(value & 0xFFU);
    output[1] = (uint8_t)(value >> 8);
}

static void write_u32_le(uint8_t *output, uint32_t value)
{
    output[0] = (uint8_t)(value & 0xFFU);
    output[1] = (uint8_t)((value >> 8) & 0xFFU);
    output[2] = (uint8_t)((value >> 16) & 0xFFU);
    output[3] = (uint8_t)(value >> 24);
}

static void write_i32_le(uint8_t *output, int32_t value)
{
    write_u32_le(output, (uint32_t)value);
}

static HAL_StatusTypeDef send_goal_only(uint8_t event_code, uint32_t goal_id)
{
    uint8_t payload[5];

    if (goal_id == 0U)
    {
        return HAL_ERROR;
    }
    payload[0] = event_code;
    write_u32_le(&payload[1], goal_id);
    return RpiTransport_Send(RPI_MSG_EVENT, payload, sizeof(payload));
}

HAL_StatusTypeDef RpiPoseGoal_SendStarted(uint32_t goal_id)
{
    return send_goal_only(RPI_EVENT_POSE_STARTED, goal_id);
}

HAL_StatusTypeDef RpiPoseGoal_SendReached(uint32_t goal_id,
                                         int32_t x_mm,
                                         int32_t y_mm,
                                         int32_t yaw_mrad,
                                         int32_t position_error_mm,
                                         int32_t yaw_error_mrad)
{
    uint8_t payload[25];

    if (goal_id == 0U)
    {
        return HAL_ERROR;
    }
    payload[0] = RPI_EVENT_POSE_REACHED;
    write_u32_le(&payload[1], goal_id);
    write_i32_le(&payload[5], x_mm);
    write_i32_le(&payload[9], y_mm);
    write_i32_le(&payload[13], yaw_mrad);
    write_i32_le(&payload[17], position_error_mm);
    write_i32_le(&payload[21], yaw_error_mrad);
    return RpiTransport_Send(RPI_MSG_EVENT, payload, sizeof(payload));
}

HAL_StatusTypeDef RpiPoseGoal_SendCancelled(uint32_t goal_id)
{
    return send_goal_only(RPI_EVENT_POSE_CANCELLED, goal_id);
}

HAL_StatusTypeDef RpiPoseGoal_SendFault(uint32_t goal_id,
                                       RpiMotionFaultReason reason)
{
    uint8_t payload[7];

    if (goal_id == 0U)
    {
        return HAL_ERROR;
    }
    payload[0] = RPI_EVENT_MOTION_FAULT;
    write_u32_le(&payload[1], goal_id);
    write_u16_le(&payload[5], (uint16_t)reason);
    return RpiTransport_Send(RPI_MSG_EVENT, payload, sizeof(payload));
}
