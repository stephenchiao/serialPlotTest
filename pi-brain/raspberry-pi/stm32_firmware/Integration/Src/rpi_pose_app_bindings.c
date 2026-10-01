#include "rpi_pose_platform.h"

#include <stddef.h>

/* 覆盖 Comm/Src/rpi_app_commands.c 中的 weak 位姿钩子。 */
RpiResponseStatus RpiApp_SetPoseGoal(const RpiPoseGoal *goal)
{
    if (goal == NULL)
    {
        return RPI_STATUS_INVALID_ARGUMENT;
    }
    return RpiPosePlatform_Start(goal);
}

RpiResponseStatus RpiApp_CancelPoseGoal(uint32_t goal_id)
{
    if (goal_id == 0U)
    {
        return RPI_STATUS_INVALID_ARGUMENT;
    }
    return RpiPosePlatform_Cancel(goal_id);
}

RpiResponseStatus RpiApp_GetPoseGoalStatus(RpiPoseGoalStatus *status)
{
    if (status == NULL)
    {
        return RPI_STATUS_INVALID_ARGUMENT;
    }
    return RpiPosePlatform_GetStatus(status);
}
