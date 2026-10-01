#include "rpi_app_commands.h"

#include <stddef.h>

static int16_t read_i16_le(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static uint32_t read_u32_le(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

static int32_t read_i32_le(const uint8_t *data)
{
    return (int32_t)read_u32_le(data);
}

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

RpiResponseStatus RpiLink_HandleCommand(uint8_t command,
                                       const uint8_t *data,
                                       uint16_t data_length,
                                       uint8_t *response_data,
                                       uint16_t response_capacity,
                                       uint16_t *response_length)
{
    if (response_length == NULL)
    {
        return RPI_STATUS_INTERNAL_ERROR;
    }
    *response_length = 0U;
    if ((data == NULL) && (data_length != 0U))
    {
        return RPI_STATUS_INVALID_LENGTH;
    }
    if ((response_data == NULL) && (response_capacity != 0U))
    {
        return RPI_STATUS_INTERNAL_ERROR;
    }

    switch (command)
    {
        case RPI_CMD_PING:
            return (data_length == 0U) ?
                   RPI_STATUS_OK : RPI_STATUS_INVALID_LENGTH;

        case RPI_CMD_STOP_ALL:
            if (data_length != 0U)
            {
                return RPI_STATUS_INVALID_LENGTH;
            }
            return RpiApp_StopAll();

        case RPI_CMD_SET_CHASSIS_VELOCITY:
            if (data_length != 6U)
            {
                return RPI_STATUS_INVALID_LENGTH;
            }
            return RpiApp_SetChassisVelocity(read_i16_le(&data[0]),
                                             read_i16_le(&data[2]),
                                             read_i16_le(&data[4]));

        case RPI_CMD_SET_SERVO_ANGLE:
            if (data_length != 3U)
            {
                return RPI_STATUS_INVALID_LENGTH;
            }
            return RpiApp_SetServoAngle(data[0], read_i16_le(&data[1]));

        case RPI_CMD_QUERY_STATUS:
            if (data_length != 0U)
            {
                return RPI_STATUS_INVALID_LENGTH;
            }
            return RpiApp_QueryStatus(response_data,
                                      response_capacity,
                                      response_length);

        case RPI_CMD_SET_TASK_CODE:
            if ((data_length == 0U) || (data_length > 31U))
            {
                return RPI_STATUS_INVALID_LENGTH;
            }
            return RpiApp_SetTaskCode(data, data_length);

        case RPI_CMD_SET_POSE_GOAL:
        {
            RpiPoseGoal goal;

            if (data_length != 20U)
            {
                return RPI_STATUS_INVALID_LENGTH;
            }
            goal.goal_id = read_u32_le(&data[0]);
            goal.x_mm = read_i32_le(&data[4]);
            goal.y_mm = read_i32_le(&data[8]);
            goal.yaw_mrad = read_i32_le(&data[12]);
            goal.timeout_ms = read_u32_le(&data[16]);
            if ((goal.goal_id == 0U) || (goal.timeout_ms == 0U))
            {
                return RPI_STATUS_INVALID_ARGUMENT;
            }
            return RpiApp_SetPoseGoal(&goal);
        }

        case RPI_CMD_CANCEL_POSE_GOAL:
        {
            uint32_t goal_id;

            if (data_length != 4U)
            {
                return RPI_STATUS_INVALID_LENGTH;
            }
            goal_id = read_u32_le(data);
            if (goal_id == 0U)
            {
                return RPI_STATUS_INVALID_ARGUMENT;
            }
            return RpiApp_CancelPoseGoal(goal_id);
        }

        case RPI_CMD_QUERY_POSE_GOAL:
        {
            RpiPoseGoalStatus pose_status;
            RpiResponseStatus result;

            if (data_length != 0U)
            {
                return RPI_STATUS_INVALID_LENGTH;
            }
            if (response_capacity < 19U)
            {
                return RPI_STATUS_INTERNAL_ERROR;
            }
            result = RpiApp_GetPoseGoalStatus(&pose_status);
            if (result != RPI_STATUS_OK)
            {
                return result;
            }
            if (pose_status.state > RPI_POSE_FAULT)
            {
                return RPI_STATUS_INTERNAL_ERROR;
            }
            write_u32_le(&response_data[0], pose_status.goal_id);
            response_data[4] = (uint8_t)pose_status.state;
            write_i32_le(&response_data[5], pose_status.x_mm);
            write_i32_le(&response_data[9], pose_status.y_mm);
            write_i32_le(&response_data[13], pose_status.yaw_mrad);
            write_u16_le(&response_data[17], pose_status.fault_reason);
            *response_length = 19U;
            return RPI_STATUS_OK;
        }

        default:
            return RPI_STATUS_UNKNOWN_COMMAND;
    }
}

__weak RpiResponseStatus RpiApp_StopAll(void)
{
    return RPI_STATUS_INTERNAL_ERROR;
}

__weak RpiResponseStatus RpiApp_SetChassisVelocity(int16_t vx_mm_s,
                                                   int16_t vy_mm_s,
                                                   int16_t wz_mrad_s)
{
    (void)vx_mm_s;
    (void)vy_mm_s;
    (void)wz_mrad_s;
    return RPI_STATUS_INTERNAL_ERROR;
}

__weak RpiResponseStatus RpiApp_SetServoAngle(uint8_t servo_id,
                                             int16_t angle_tenths_degree)
{
    (void)servo_id;
    (void)angle_tenths_degree;
    return RPI_STATUS_INTERNAL_ERROR;
}

__weak RpiResponseStatus RpiApp_QueryStatus(uint8_t *output,
                                           uint16_t output_capacity,
                                           uint16_t *output_length)
{
    (void)output;
    (void)output_capacity;
    *output_length = 0U;
    return RPI_STATUS_INTERNAL_ERROR;
}

__weak RpiResponseStatus RpiApp_SetTaskCode(const uint8_t *ascii_code,
                                           uint16_t code_length)
{
    (void)ascii_code;
    (void)code_length;
    return RPI_STATUS_INTERNAL_ERROR;
}

__weak RpiResponseStatus RpiApp_SetPoseGoal(const RpiPoseGoal *goal)
{
    (void)goal;
    return RPI_STATUS_INTERNAL_ERROR;
}

__weak RpiResponseStatus RpiApp_CancelPoseGoal(uint32_t goal_id)
{
    (void)goal_id;
    return RPI_STATUS_INTERNAL_ERROR;
}

__weak RpiResponseStatus RpiApp_GetPoseGoalStatus(RpiPoseGoalStatus *status)
{
    (void)status;
    return RPI_STATUS_INTERNAL_ERROR;
}
