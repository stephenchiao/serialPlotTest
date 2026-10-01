# 智能搬运机器人底层状态机

该目录提供可由 systemd 开机启动的任务编排层。它不会直接假定 GPIO、
电机板、机械臂或显示屏型号；真实硬件通过 `ComponentBundle` 注入。

## 安全边界

- 开机只初始化、自检并进入 `WAITING_FOR_START`，不会自动驱动车辆。
- 必须先观察到实体按钮释放，再检测到一次按下，防止开机时按钮卡住误启动。
- 比赛开始后持续检查急停、边界、堵转/电源等安全汇总。
- 当前动作连续 14 秒没有检测到底盘或机械臂物理动作时，先进入有限恢复重试，
  默认最多重试 2 次。导航、物料定位耗尽重试或恢复超时后进入 `ACTION_WAITING`，
  默认停车 1 秒再重新规划/识别；不跳过任务、不计为抓取成功。
- 可恢复的导航异常进入 `SAFETY_PAUSED`，默认 10 秒后提醒并继续等；新画面连续安全
  0.2 秒且至少有 2 个不同时间戳的观测后恢复原任务。暂停不叠加无动作超时，
  恢复后为原动作重新计时，但不会重置重试预算或伪造物理活动。
- 通信断开/重新握手期间保持暂停，不消耗动作重试；
  链路恢复并确认旧航点停止后，重新开始感知复核，再继续原任务。
- 扫码等待默认每 8 秒提醒后继续扫描，识别合法任务码前不进入搬运流程。
- 抓取、放置、堆叠等不可盲目重复的动作仍有有限重试；耗尽重试或恢复失败
  进入 `SAFE_STOP`。恢复组件显式返回致命错误时也终止任务。
- 任意未处理异常都会先取消导航、停止底盘并停止机械臂。
- 遥测接口只用于日志和观测，不能绕过唯一实体 Start 按钮控制比赛流程。

## 主状态流程

```text
BOOTING -> SELF_CHECK -> WAITING_FOR_START -> READING_TASK_CODE
  -> 第一批：转盘导航 -> 依序定位/抓取三件 -> 加工区依序放置
            -> 暂存区依序放置
  -> 第二批：转盘导航 -> 依序定位/抓取三件 -> 加工区依序放置
            -> 暂存区按物料编号对应堆叠
  -> REPORTING -> COMPLETED
```

硬急停、地图越界、CAN/内部/未知故障、非通信停车操作失败及未处理异常进入
`SAFE_STOP`。感知等待、可重复动作重试耗尽不再直接结束比赛。

兼容原配置键：`task_code_timeout_seconds` 现在为扫码提醒间隔，
`safety_pause_timeout_seconds` 为复核提醒阈值；新增 `action_retry_wait_seconds`
控制重复尝试前的停车间隔。恢复等待不会伪造物理活动或增加物料统计。

## 在开发机运行模拟任务

从项目根目录运行：

```bash
python3 -m robot_runtime \
  --simulate \
  --auto-start \
  --exit-on-terminal \
  --task-code "452+321+254+312"
```

这里的 `--auto-start` 只用于无硬件模拟测试。树莓派正式配置不得使用它，
应当由 `StartButton` 的 GPIO 适配器提供实体按钮状态。

运行测试：

```bash
python3 -m unittest \
  discover -s tests -v
```

## 接入真实硬件

所有接口位于 `interfaces.py`：

- `StartButton`：GPIO 实体按钮与消抖。
- `TaskCodeReader`：摄像头二维码识别，返回完整任务码文本。
- `Display`：持续显示任务码、当前状态和最终统计。
- `MotionController`：电机/PWM/编码器底层停车与活动状态。
- `Navigator`：巡线、定位、路径规划、避障和逻辑区域导航。
- `MaterialPerception`：颜色、形状、物料位置和转盘目标定位。
- `Manipulator`：抓取到车载槽、加工放置、暂存和对应堆叠。
- `SafetyMonitor`：急停、边界、堵转、姿态和电池状态汇总。
- `StatisticsRecorder`：抓取、放置、堆叠和最终正确数。
- `Telemetry`：本地日志或只读遥测。
- `LightingController`：垂直向下照明控制。
- `RecoveryController`：丢线、丢目标和动作超时后的有界恢复。

创建自己的模块，例如：

```text
robot_hardware/
├── __init__.py
├── factory.py
├── gpio_button.py
├── motor_driver.py
├── navigation.py
├── perception.py
├── manipulator.py
└── display.py
```

在 `robot_hardware.factory` 中提供：

```python
def build_components(config):
    return ComponentBundle(
        start_button=...,
        task_code_reader=...,
        display=...,
        motion=...,
        navigator=...,
        material_perception=...,
        manipulator=...,
        safety=...,
        statistics=...,
        telemetry=...,
        lighting=...,
        recovery=...,
    )
```

然后修改 `config/robot.json`：

```json
{
  "component_factory": "robot_hardware.factory:build_components"
}
```

实际文件中应保留其他配置项。组件动作必须快速返回：动作未完成时返回
`ActionResult.running()`，完成时返回 `ActionResult.done()`，可恢复错误返回
`ActionResult.retryable()`，不可恢复错误返回 `ActionResult.fatal()`。同一方法会在
状态机循环中重复调用，硬件适配器必须保证幂等。

颜色识别算法位于 `robot_perception/color`，巡线识别位于
`robot_perception/line`，方向决策位于 `robot_control/line_navigation.py`。
真实硬件组件应在 `robot_hardware.factory` 中组装，状态机不直接导入调试脚本。

## 树莓派开机启动

确认项目位于树莓派本地目录并已完成真实硬件工厂配置，然后执行：

```bash
cd ~/python
sudo bash deployment/systemd/install_service.sh
```

安装脚本会根据当前项目路径、普通用户名和 `python3` 路径生成 systemd 服务，
并立即启用。服务默认只在异常退出时重启；正常完成后进程保持最终统计显示，
直到关机或手动停止。

常用命令：

```bash
systemctl status robot-runtime.service
journalctl -u robot-runtime.service -f
sudo systemctl restart robot-runtime.service
sudo systemctl stop robot-runtime.service
```

当前默认 `component_factory` 是 `null`，因此启动的是安全模拟组件且不会自动
按下 Start。完成真实硬件适配并配置工厂之前，它不会控制实际电机。
