# 智能搬运小车树莓派程序

项目按运行调度、比赛任务、硬件驱动、视觉算法、动作控制、公共服务和模拟测试
分层。详细边界见 `docs/ARCHITECTURE.md`。

## 目录

```text
robot_runtime/       主入口、状态机、接口和运行模型
robot_mission/       二维码任务文本解析与比赛规则
robot_hardware/      摄像头、GPIO、显示器和STM32通信
robot_perception/    二维码、颜色、巡线、物料和色环识别
robot_control/       导航、视觉对准、机械臂协调和恢复
robot_services/      安全、日志、遥测和统计
robot_simulation/    无硬件模拟组件
config/              运行、颜色和巡线配置
tests/               无硬件自动测试
tools/               人工调试、标定和离线评测入口
deployment/          systemd等部署脚本
docs/                架构与标定文档
references/          往届参考代码，不参与正式运行
```

## 无硬件模拟

```bash
cd ~/python
python3 -m robot_runtime \
  --simulate \
  --auto-start \
  --exit-on-terminal \
  --task-code "452+321+254+312"
```

`--auto-start` 只能用于模拟；正式比赛必须由唯一实体 Start 按钮触发。

## 自动测试

```bash
cd ~/python
python3 -m unittest discover -s tests -v
```

## 颜色识别调试

```bash
cd ~/python
python3 -m tools.debug_color --preview
```

颜色参数位于 `config/color.json`，离线标定见 `docs/COLOR_TUNING.md`。

## 树莓派5双摄像头

双摄像头统一配置位于 `config/cameras.json`：CAM/DISP0 上的夹爪相机
（`camera_num: 0`）用于物料模型识别与中心定位；CAM/DISP1 上的前置相机
（`camera_num: 1`）用于巡线、导航和二维码。摄像头型号未确定时保持 `model: null`。

cam0 默认选择模型后端，配置位于 `config/material.json`。目前没有训练模型
和物料编号映射，权重保持为空，返回 `MODEL_NOT_CONFIGURED` 且禁止抓取。
后续接入流程见 [物料模型说明](docs/MATERIAL_MODEL.md)；旧颜色后端需显式添加
`--material-backend color`。

抓取相机框架与对准调试：

```bash
python3 -m tools.debug_gripper --no-preview
```

物料数据交给 STM32：树莓派只发送类别、编号、置信度和像素位置，
STM32 自己负责纠偏与抓取。新增实机上报入口为 `tools.report_material`；
这次队友压缩包尚未实现该物料接口；
需先接入 STM32 的 0x85 接收模块并声明 0x40 扩展能力。旧固件会被拒绝，
不能把串口 ACK 当作抓取完成。接入和启动步骤见
[物料数据下发说明](docs/MATERIAL_STM32.md)。

两路相机联合调试（默认无窗口，适合SSH）：

```bash
python3 -m tools.debug_dual_camera --mode all
```

这里的“摄像头1”是夹爪相机，“摄像头2”是车头导航相机；它们和 Picamera2
打印的 `camera_num` 不是同一个概念。联合程序让摄像头2的巡线与二维码算法
复用同一帧，避免重复打开或重复采集设备。

安装、编号核对和标定流程见 `docs/DUAL_CAMERA.md`。

## 巡线调试

```bash
cd ~/python
python3 -m tools.debug_line
```

无桌面环境时使用 `--no-preview`。巡线参数位于 `config/line.json`。

## OPS9 地图导航

OPS9 接在 STM32 上，由 STM32 转发统一位姿遥测；树莓派不再占用第二个 OPS9
串口。初始田字路网和点位在 `config/navigation.json`，OPS9 健康门限在
`config/ops9.json`，前视黑色障碍参数在 `config/obstacle.json`。

正式导航由树莓派规划航点、STM32 执行 20 ms 位姿闭环。每个航点使用带 CRC、
请求序号和 `goal_id` 的 v2 二进制事务。当前队友固件不支持旧连续速度命令，
必须使用 `stm32_pose_goal` 模式。

这次队友压缩包实际使用 USART1（PA9/PA10，115200），原生 USB CDC 仍需底层适配。
直接连接这份固件时使用 USB 转 3.3V TTL 串口，并运行：

```bash
python3 -m tools.stm32_link_test --transport uart --port /dev/ttyUSB0 --count 10
```

正式运行时将 `config/stm32.json` 的 `transport` 设置为 `uart` 并填写实际端口，
模板见 `config/stm32_uart.json`。新握手会检查 WORK、OPS9、CAN 和 PID 参数后才启用
二进制会话。接口差异及未实现的物料/抓取功能见 [队友接口适配说明](docs/TEAMMATE_INTERFACE.md)。

队友完成 CDC 适配后，先测试 USB CDC 双向通信，不使能新的运动会话
（遗留活动会话会先停车）：

```bash
python3 -m tools.stm32_link_test --port /dev/ttyACM0 --count 10
```

完整握手和位姿监视会进入 RPI 模式，可能使能底盘，必须架空车轮、准备急停。
设置 `config/stm32.json` 的实际设备路径后可运行 `python3 -m tools.stm32_ops9_monitor`。
USB CDC 配置、队友固件接口要求和测试步骤见
[`robot_hardware/stm32/README.md`](robot_hardware/stm32/README.md)。

地图点位、车体包络和相机单应矩阵都是待现场测量的初值；未完成标定前禁止启用
自动行驶。导航阶段已改为灰色可行域、黄白禁入区域和黑色圆柱检测，不再依赖
旧黑线巡线。实现与协议说明见 `docs/NAVIGATION_OPS9_DESIGN.md`，真实组件接线见
`docs/NAVIGATION_RUNTIME.md`。

当前导航采用比赛优先的容错配置：短暂视觉异常有 0.6 秒容错窗口，无动作超时
先恢复重试，无关道路变化不取消当前航点。停车复核默认在 0.2 秒、2 个新观测
确认后恢复。扫码和感知复核超时会继续等待；导航、物料定位耗尽重试后停车
1 秒再尝试，OPS9 丢失和航点超时需核对旧航点及新鲜定位后恢复。详细参数和
仍保留的保护见
[`导航运行策略`](docs/NAVIGATION_RUNTIME.md#比赛优先的容错策略)。

## 树莓派开机启动

真实硬件组件接入完成后执行：

```bash
cd ~/python
sudo bash deployment/systemd/install_service.sh
```

硬件组件接口和安全约束见 `robot_runtime/README.md`。
