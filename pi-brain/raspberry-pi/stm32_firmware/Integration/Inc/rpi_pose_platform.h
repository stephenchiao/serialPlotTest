#ifndef RPI_POSE_PLATFORM_H
#define RPI_POSE_PLATFORM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "rpi_app_commands.h"

/*
 * 由具体底盘工程实现的最小端口。实现必须非阻塞：Start 只装载目标并启动
 * 状态机，Cancel 必须立即请求零速，控制循环仍由底盘主循环定时推进。
 */
RpiResponseStatus RpiPosePlatform_Start(const RpiPoseGoal *goal);
RpiResponseStatus RpiPosePlatform_Cancel(uint32_t goal_id);
RpiResponseStatus RpiPosePlatform_GetStatus(RpiPoseGoalStatus *status);

#ifdef __cplusplus
}
#endif

#endif /* RPI_POSE_PLATFORM_H */
