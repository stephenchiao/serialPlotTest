# 树莓派软件分层

## 依赖方向

```text
robot_runtime -> robot_mission + 组件接口
真实组件      -> robot_control + robot_perception + robot_hardware
模拟组件      -> robot_simulation
监督与记录    -> robot_services
```

## 目录职责

- `robot_runtime`：入口、状态机、接口和公共运行模型。
- `robot_mission`：任务码、任务计划和比赛规则。
- `robot_hardware`：摄像头、GPIO、显示器和STM32通信。
- `robot_perception`：二维码、颜色、巡线、物料和色环识别。
- `robot_control`：导航、视觉对准、机械臂协调和恢复。
- `robot_services`：安全监督、日志、遥测和统计。
- `robot_simulation`：无硬件模拟组件和故障场景。
- `tools`：只能人工运行的调试、标定和评测入口。
- `references`：往届资料，仅供阅读，正式程序不得导入。

摄像头只能由 `robot_hardware.camera` 管理，视觉模块接收图像帧；STM32链路也
只能由一个通信组件持有。状态机不得直接导入 OpenCV、Picamera2、串口或GPIO。

正式导航采用两层闭环：树莓派负责比赛任务、视觉、地图和航点顺序；STM32负责
OPS9绝对位姿闭环、PID、加减速、麦轮解算和本地安全停车。两层通过带 CRC、请求
序号和 `goal_id` 的唯一二进制链路通信，不再使用 ASCII `POSE SET`。

树莓派5使用两路固定角色摄像头：`front`负责前向导航和起始二维码，`gripper`
负责夹爪附近的物料颜色与位置。真实组件共享一个 `DualCameraManager`，不得让
导航、二维码或物料识别模块重复创建 Picamera2 实例。配置和标定步骤见
`docs/DUAL_CAMERA.md`。
