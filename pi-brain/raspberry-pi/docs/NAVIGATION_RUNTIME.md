# 导航运行链路与接入说明

## 已接通的数据流

```text
OPS9
  -> STM32解析私有协议
  -> OPS9_POSE遥测
  -> Stm32Ops9Receiver
  -> 地图位姿/物理运动判断
                        \
前视摄像头单帧 ----------> 灰色可行域与黄白禁入检测
                        -> 黑色圆柱候选与三帧确认
                        -> 动态道路封闭
                        -> A*改道
                        -> Stm32PoseMapNavigator
                        -> SET_POSE_GOAL(goal_id, OPS9绝对位姿)
                        -> STM32 20 ms位姿闭环/PID/麦轮解算

NavigationSafetyMonitor
  -> 短暂视觉异常：在容错窗口内继续当前运动
  -> 近障碍、OPS9失效、持续视觉异常：SAFETY_PAUSED -> 复核后恢复原任务
  -> STM32断联：SAFETY_PAUSED -> 重连、确认静止、复核后继续原任务
  -> 地图越界、底层硬故障：SAFE_STOP
```

导航阶段使用 `DualCameraVisionController.observe_navigation()`，一帧只采集一次，
不会运行旧的黑线巡线算法。二维码阶段仍可用同一个前摄像头调用
`scan_task_code()`，夹爪相机逻辑不受影响。

## 真实组件注入

完成标定后：

```python
from robot_hardware.navigation import build_real_navigation

navigation = build_real_navigation()

components = ComponentBundle(
    navigator=navigation,
    motion=navigation.stack.chassis,
    safety=navigation.safety,
    # 以下组件继续填写你们真实的按钮、显示、机械臂等适配器
    start_button=...,
    task_code_reader=...,
    display=...,
    material_perception=...,
    manipulator=...,
    statistics=...,
    telemetry=...,
    lighting=...,
    recovery=...,
)
```

`NavigationRuntime` 负责统一打开前/夹爪摄像头和 STM32 USB CDC 设备、订阅 OPS9、位姿
事件、执行自检及关闭资源。`Stm32PoseGoalController.is_active()` 仍使用 OPS9
实际位姿变化，而不是“存在活动 goal”，因此堵转不会被误认为仍在运动。

`config/navigation.json` 的 `planner.control_mode` 默认是 `stm32_pose_goal`。只有在
台架回退调试时才改为 `legacy_velocity`；两种模式不能同时运行。

## 比赛优先的容错策略

- 行驶中道路颜色误判、观测过期、相机采集异常或健康度短暂失效，允许在最后
  有效视觉帧后的 `perception_grace_seconds` 内继续，默认 0.6 秒。没有历史有效
  帧、尚未下发运动或正在停车复核时不适用；重复旧帧和交替报错不延长这个窗口。
  在 300 mm/s 下，0.6 秒相当于最多约 180 mm 的视觉缺失行驶，需结合比赛场地
  调整。此窗口不代表障碍或边界在这段时间内仍可被及时检测。
- OPS9 位姿丢失和已检测到的制动包络内近障碍仍立即暂停；非有限
  位姿、地图包络越界、底层硬故障和算法异常仍终止停车。
- 运行中 USB 断开或心跳失效进入通信等待，不因等待超过 10 秒或无动作而结束
  比赛，也不消耗动作重试次数。断线期间由 STM32 原有看门狗停车；后台重新握手
  后查询旧航点是否停止，再等待新会话的有效定位和视觉复核。恢复后从当前位置
  重新规划，使用新的 `goal_id`，不重放旧命令；CAN 等非通信故障不会被重连解除。
  通信等待期间暂停二维码读取提醒计时；链路确认恢复后重新进行感知复核。
- 导航只检查前摄像头，夹爪相机在导航期间不采集不会误触发停车。非导航且无
  运动命令时跳过前视检查；真正开始导航前仍需有效前视观测。
- `SAFETY_PAUSED` 超过默认 10 秒只提醒一次，保持停车等待有效感知，不再因等待
  时间过长结束比赛。恢复要求为 0.2 秒/2 个不同时间戳的有效观测；仍需确认旧
  航点已经停止，并保留原制动距离和恢复间隙。暂停不叠加无动作终止计时。
- 当前动作连续 14 秒无物理动作时先尝试恢复，默认最多 2 次。恢复和重新执行
  的动作独立计时，不伪造物理活动。导航和物料定位耗尽重试、恢复超时或返回
  可重试错误后进入 `ACTION_WAITING`，停车默认 1 秒，再重新规划/识别，重置该轮
  重试预算。任务码、批次、物料索引和搬运统计不变。恢复组件显式致命错误仍终止。
- 扫码默认每 8 秒提醒并继续扫描，始终等待合法任务码，不使用猜测结果启动任务。
- 提交航点 ACK 超时或 BUSY 时进入 `RECONCILING`，不重新发送运动指令。查询确认
  相同 `goal_id` 仍 ACCEPTED/MOVING 时接续等待该目标；确认到位只推进一次。
  取消 ACK/事件超时也先查询，查询失败继续等待，仍运动则再次发 `STOP_ALL`；确认
  旧航点停止后才能重新规划和提交新 `goal_id`。明确拒绝或驱动异常仍按内部故障处理。
- 固件 `OPS9_LOST`、`TIMEOUT` 先锁定位姿。查询确认旧航点停止，收到至少两帧故障
  后的新鲜、不同时间戳、无跳变定位，再解除定位锁并进行视觉复核。重复、逆序、
  无效或过期帧不能放行。没有匹配事务的迟到软故障通过新会话隔离，不盲目解锁。
- CAN、地图越界、内部及未知故障维持 `SAFE_STOP`，不会被后续定位故障或重连覆盖。
  抓取、加工放置、暂存和堆叠仍保留有限重试与失败终止，必须由真实机械臂适配器
  确认动作结果，不能用无限重试绕过物料状态不确定。
- 只有目标改变或剩余路径被封闭时才取消/重规划航点；无关道路变化、已走过
  路段封闭，以及捷径重新开放都不打断当前路线。两种导航模式共用此判断。

视觉容错参数位于 `config/navigation_safety.json`，将 `perception_grace_seconds`
设为 `0` 可恢复视觉异常立即暂停；恢复门限和重试次数位于 `config/robot.json`。
`action_retry_wait_seconds` 默认 1 秒；兼容旧键 `task_code_timeout_seconds` 和
`safety_pause_timeout_seconds`，现在分别表示扫码提醒间隔与复核提醒阈值。
实体 Start、通信握手、急停、底层看门狗及启动标定检查维持原有要求。

## 启动锁

以下配置任意一项仍是占位值，`build_real_navigation()` 都会拒绝启动：

- `config/stm32.json` 的真实 STM32 串口 `/dev/serial/by-id/...` 与对应 `transport`；
- `config/navigation.json` 的实测点位、道路和车体包络；
- `config/ops9.json` 的起始坐标变换；
- `config/obstacle.json` 的前视相机地面单应矩阵；
- `config/road.json` 的现场灰、黄、白 HSV 阈值。

完成对应标定后才能将各文件的 `calibration_required`，以及地图中的
`nominal_map_requires_field_calibration` 改为 `false`。

## 制动公式

独立安全监控使用：

```text
d_stop = d_blind + v * t_reaction + v^2 / (2 * a_brake) + margin
```

参数在 `config/navigation_safety.json`。其中盲区、总延迟和满载最差制动减速度
必须实测，不能直接使用当前初值参加比赛。
