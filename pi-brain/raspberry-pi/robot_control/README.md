# 动作控制层

本层把视觉和任务目标转换为导航、对准、机械臂及恢复动作。它可以调用硬件与
感知接口，但不应打开摄像头、串口或GPIO。当前 `line_navigation.py` 只输出
`LEFT`、`RIGHT`、`STRAIGHT`、`STOP` 决策，后续由真实 `Navigator` 转换成
STM32速度命令。

当前已增加基于 OPS9 和预录路网的导航实现：

- `navigation_map.py`：从 `config/navigation.json` 读取节点、道路和车体包络，
  将视觉障碍膨胀后封闭相交道路，并用 A* 从其他路口改道；
- `navigator.py`：保留地图变换和旧速度闭环，并提供地图到 OPS9 的逆变换；
- `pose_navigation.py`：把 A* 路线逐点转换为带 `goal_id` 的 STM32 位姿事务；
- `navigation_factory.py`：默认装配 STM32 位姿闭环；`legacy_velocity` 只用于回退。

最小装配方式：

```python
from robot_control import build_stm32_navigation
from robot_hardware.stm32 import SerialLink

link = SerialLink("/dev/serial/by-id/你的设备")
link.open()
stack = build_stm32_navigation(link)
stack.start()
navigator = stack.navigator
```

`config/navigation.json` 内的点位目前是 2400 mm 方形场地上的初始拓扑值，不是
官方精确测量结果。上车前必须回填真实车体半径、道路中心、目标停靠点和起始位姿。
`target_headings_mrad` 同样是待标定值；协议边界统一使用 mm/mrad/ms。
`config/obstacle.json` 的 `calibration_required` 默认为 `true`，占位单应矩阵不能
用于驱动车辆；完成前视相机地面标定后再改为 `false`。
