#include "rpi_app_commands.h"
#include "rpi_pose_goal.h"

#include <assert.h>
#include <stdint.h>
#include <string.h>

static RpiPoseGoal captured_goal;
static uint32_t cancelled_goal_id;
static uint8_t captured_type;
static uint8_t captured_payload[32];
static uint16_t captured_length;

RpiResponseStatus RpiApp_SetPoseGoal(const RpiPoseGoal *goal)
{
    captured_goal = *goal;
    return RPI_STATUS_OK;
}

RpiResponseStatus RpiApp_CancelPoseGoal(uint32_t goal_id)
{
    cancelled_goal_id = goal_id;
    return RPI_STATUS_OK;
}

RpiResponseStatus RpiApp_GetPoseGoalStatus(RpiPoseGoalStatus *status)
{
    status->goal_id = 0x12345678U;
    status->state = RPI_POSE_MOVING;
    status->x_mm = -100;
    status->y_mm = 200;
    status->yaw_mrad = -1571;
    status->fault_reason = RPI_MOTION_FAULT_UNSPECIFIED;
    return RPI_STATUS_OK;
}

HAL_StatusTypeDef RpiUsbCdcLink_Send(uint8_t message_type,
                                    const uint8_t *payload,
                                    uint16_t payload_length)
{
    captured_type = message_type;
    captured_length = payload_length;
    memcpy(captured_payload, payload, payload_length);
    return HAL_OK;
}

int main(void)
{
    static const uint8_t goal_data[] = {
        0x78U, 0x56U, 0x34U, 0x12U,
        0x85U, 0xFFU, 0xFFU, 0xFFU,
        0xC8U, 0x01U, 0x00U, 0x00U,
        0xDDU, 0xF9U, 0xFFU, 0xFFU,
        0xB8U, 0x88U, 0x00U, 0x00U
    };
    uint8_t response[32];
    uint16_t response_length;

    assert(RpiLink_HandleCommand(RPI_CMD_SET_POSE_GOAL,
                                 goal_data,
                                 sizeof(goal_data),
                                 response,
                                 sizeof(response),
                                 &response_length) == RPI_STATUS_OK);
    assert(captured_goal.goal_id == 0x12345678U);
    assert(captured_goal.x_mm == -123);
    assert(captured_goal.y_mm == 456);
    assert(captured_goal.yaw_mrad == -1571);
    assert(captured_goal.timeout_ms == 35000U);

    assert(RpiLink_HandleCommand(RPI_CMD_CANCEL_POSE_GOAL,
                                 goal_data,
                                 4U,
                                 response,
                                 sizeof(response),
                                 &response_length) == RPI_STATUS_OK);
    assert(cancelled_goal_id == 0x12345678U);

    assert(RpiLink_HandleCommand(RPI_CMD_QUERY_POSE_GOAL,
                                 NULL,
                                 0U,
                                 response,
                                 sizeof(response),
                                 &response_length) == RPI_STATUS_OK);
    assert(response_length == 19U);
    assert(response[4] == RPI_POSE_MOVING);

    assert(RpiPoseGoal_SendStarted(0x12345678U) == HAL_OK);
    assert(captured_type == RPI_MSG_EVENT);
    assert(captured_length == 5U);
    assert(captured_payload[0] == RPI_EVENT_POSE_STARTED);

    assert(RpiPoseGoal_SendReached(0x12345678U,
                                   -123,
                                   456,
                                   -1571,
                                   2,
                                   -3) == HAL_OK);
    assert(captured_length == 25U);
    assert(captured_payload[0] == RPI_EVENT_POSE_REACHED);

    assert(RpiPoseGoal_SendFault(0x12345678U,
                                 RPI_MOTION_FAULT_CAN) == HAL_OK);
    assert(captured_length == 7U);
    assert(captured_payload[0] == RPI_EVENT_MOTION_FAULT);
    assert(captured_payload[5] == RPI_MOTION_FAULT_CAN);
    return 0;
}
