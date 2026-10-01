# 队友底盘位姿闭环接入二进制协议

本目录不复制队友的 `main.c`，只定义稳定边界。把 `Comm` 和 `Integration` 加入
CubeIDE 后，由底盘工程实现 `rpi_pose_platform.h` 的三个函数。

## 需要从队友 main.c 暴露的状态

队友工程当前把 `pose_control_active`、目标位姿、规划器复位和安全停车函数定义为
`static` 或文件全局变量。不要让通信层直接 `extern` 这些变量；在底盘控制模块中
新增以下端口，并由该模块独占这些状态：

```c
RpiResponseStatus RpiPosePlatform_Start(const RpiPoseGoal *goal);
RpiResponseStatus RpiPosePlatform_Cancel(uint32_t goal_id);
RpiResponseStatus RpiPosePlatform_GetStatus(RpiPoseGoalStatus *status);
```

`Start` 的顺序应为：

1. 检查 WORK 模式、CAN、OPS9、主机心跳和当前没有活动目标；
2. 检查目标相对当前位置的硬行程上限；
3. 将协议的 mm/mrad 转换成底盘内部的 mm/degree；
4. 装载 X/Y/YAW PID、平移阶段初始航向和 `timeout_ms`；
5. 保存 `goal_id`，启动非阻塞控制状态机；
6. 返回 `RPI_STATUS_OK`；在下一次主循环发送
   `RpiPoseGoal_SendStarted(goal_id)`，确保命令响应先于启动事件。

正在执行其他目标时必须返回 `RPI_STATUS_BUSY`，不能覆盖旧目标。树莓派不会自动
重发运动命令。

`Cancel` 只接受当前活动 `goal_id`。函数必须立即让四轮目标为零、清除规划器和
PID 历史，然后返回 OK；在主循环发送 `RpiPoseGoal_SendCancelled(goal_id)`。

到位稳定计数完成并确认四轮零速后调用：

```c
RpiPoseGoal_SendReached(goal_id,
                        current_ops_x_mm,
                        current_ops_y_mm,
                        current_yaw_mrad,
                        position_error_mm,
                        yaw_error_mrad);
```

CAN、OPS9、主机心跳、越界和动作超时导致本地停车时，先停电机并清除旧目标，再
发送 `RpiPoseGoal_SendFault(goal_id, reason)`。通信恢复后不得继续故障前的目标。

## 主循环和 USB CDC

在原 20 ms 位姿控制循环之外，每轮调用 `RpiUsbCdcLink_Process()`。
`CDC_Receive_FS()` 只调用 `RpiUsbCdcLink_OnReceive(Buf, *Len)` 并重新挂载 USB
接收包，不得在 USB 回调里启动 PID 或发送电机命令。

正式 RPI 固件的 Type-C CDC 只运行本项目二进制协议。PC 调参如仍需文本协议，
使用独立 UART，或构建单独的 TUNE 固件；不能让两个解析器同时消费 CDC 字节流。

## 坐标约定

- 二进制协议：OPS9 原始绝对 `x/y`，单位 mm；yaw 单位 mrad；时间单位 ms。
- 队友控制器：内部 yaw 为 degree，在平台边界转换。
- OPS9 到车体中心的安装偏移只在 STM32 位姿控制器中补偿一次。
- 树莓派地图通过 `Ops9MapTransform.invert()` 转换后才提交目标。

在实车启用前必须依次验证 PING、STOP、OPS9 遥测、零位移目标、100 mm 单轴目标、
取消、断心跳、断 OPS9 和 CAN 故障；任一失败都不能接入完整比赛状态机。
