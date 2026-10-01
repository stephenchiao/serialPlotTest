# cam0 物料视觉数据交给 STM32

职责已按要求分开：树莓派识别类别/颜色、选择目标、计算图像中的位置；
STM32 接收观测，负责坐标标定、纠偏、机械臂/夹爪动作和机械安全检查。
树莓派不根据 safe_to_pick 发抓取命令。原 aligned / safe_to_pick 字段只保留给调试，
不会发送到 STM32，也不会成为位置上报的门槛。

本修改对应本地项目，尚未上传到树莓派、尚未烧录 STM32，也未完成实机联调。
默认仍无模型权重和物料编号，因此会上报 MODEL_NOT_CONFIGURED，而不是虚构识别结果。

## 使用入口

1. 按 MATERIAL_MODEL.md 配置模型权重和 class_to_material，编号不得猜测。
2. 把下文的接收模块接入队友实际使用的 **v2 固件**，编译并烧录。
3. 在 config/stm32.json 填写真实串口及对应 transport（usb_cdc 或 uart）。
4. 架空车轮、准备急停，再从本地项目的树莓派副本启动：

```bash
python3 -m tools.report_material --port /dev/ttyACM0 --enable-stm32-session
# 明确指定已决定的物料编号时，再追加 --target-code 编号
```

该入口只启动 cam0，不占用 cam1；默认无预览窗口，适合 SSH。
完整握手可能使能底盘，所以要求显式的 --enable-stm32-session。
打开链路时，第一次 SESSION_PROBE 就要求物料能力位 0x40；旧固件不进入新使能会话。
握手成功不代表运动安全已验证，不得在夹爪控制逻辑尚未检查时让机械臂带载运行。

原 debug_gripper / debug_dual_camera 仍是离线视觉调试入口，默认不打开串口。
正式运行已有相机管理器和 SerialLink 时不要同时启动 report_material，以免重复占用设备。
可复用已有链路，通过协调层的参数接入：

```python
from robot_hardware.stm32 import Stm32MaterialVisionPublisher
from robot_control.dual_camera_vision import build_dual_camera_vision

# link 是已由运行程序管理的唯一 SerialLink，固件需声明物料能力位。
reporter = Stm32MaterialVisionPublisher(link)
vision = build_dual_camera_vision(material_reporter=reporter)
# 按现有运行流程启动 vision；每次 observe_gripper(target_code) 自动上报新观测。
```

任务状态机的真实 MaterialPerception / Manipulator 适配仍需按实际 STM32 动作接口接入。
不能把 0x85 的 OK 应答作为“定位完成”或“抓取完成”，也不能调用模拟抓取冒充实机成功。
本次只建立视觉数据下发和接收接口，不猜测尚未提供的机械臂动作协议。

## 下发协议

沿用现有 v2 帧头、请求序号、CRC16 和应答机制，不更改导航 0x80~0x84：

```text
COMMAND 0x10，opcode = UPDATE_MATERIAL_VISION / 0x85
payload = opcode:u8 + data:47 bytes，共 48 字节
RESPONSE 0x11：request_sequence:u8 + opcode:u8 + status:u8；无返回数据
```

多字节字段全部小端序。Python struct 为 `<BBBBBIII9H2h4H`。
STM32 必须在现有帧解析器校验版本、长度、CRC 后调用接收模块，不能直接按 USB 包解析。

| data 偏移 | 字段 | 类型/含义 |
|---|---|---|
| 0 | schema_version | u8，固定 1，不是外层协议版本 |
| 1 | camera_num | u8，固定 0（夹爪相机） |
| 2 | backend | u8：1 颜色后端、2 模型后端 |
| 3 | status | u8，见状态列表 |
| 4 | flags | u8：bit0 目标可见、bit1 连续确认完成 |
| 5 | session_id | u32，视觉会话随机非零编号，USB 会话更换后重新生成 |
| 9 | frame_id | u32，非零递增帧号，允许自然回绕 |
| 13 | capture_tick_ms | u32，树莓派 monotonic 采图开始时间，仅供日志 |
| 17 | valid_for_ms | u16，接收后剩余有效期，1~1000 ms |
| 19 | target_material_code | u16：请求的目标编号，0 表示自动选择 |
| 21 | material_code | u16：实际识别编号，0 表示没有有效目标 |
| 23 | class_id | u16：模型原始类别，0 合法，65535 表示无类别/颜色后端 |
| 25 | confidence_permille | u16：置信度 ×1000，例如 920 = 0.92 |
| 27 / 29 | frame_width / height | u16，原始图像尺寸 |
| 31 / 33 | center_x / y | u16，原图像素坐标，原点在左上角 |
| 35 / 37 | offset_x / y_tenths | i16，相对配置夹爪中心的偏差，单位 0.1 像素 |
| 39 / 41 | box_x / y | u16，原图检测框左上角 |
| 43 / 45 | box_width / height | u16，原图检测框尺寸 |

material_code 在颜色后端对应其颜色/物料编号，在模型后端对应显式的类别映射。
不发送任意颜色名称字符串，也不把模型类别自动当成物料编号。
模型类别不一定是颜色：需要训练/映射定义它的业务含义。
串口编号范围为 1~65535，模型 class_id 范围为 0~65534，超范围拒绝发送，不截断。

偏差正 x 表示目标在图像右侧，正 y 表示在图像下方。
例如夹爪中心 (320,240)、目标 (350,225)，发送偏差 (300,-150)，即 (+30,-15) 像素。
这不是毫米，更不能直接当成舵机角度；实际转换/闭环由 STM32 根据相机与机械臂标定实现。

状态编号：

```text
0 SEARCHING           1 CONFIRMING              2 TRACKING
3 MODEL_NOT_CONFIGURED 4 MAPPING_NOT_CONFIGURED  5 UNMAPPED_CLASS
6 TARGET_NOT_MAPPED   7 TARGET_NOT_FOUND          8 AMBIGUOUS
9 INFERENCE_ERROR    10 CAMERA_ERROR            11 STOPPED
12 HOLD              13 STALE
```

ALIGNING 和 READY 统一上报 TRACKING，均携带位置，不下发“允许抓取”。
确认次数不足的观测仍携带位置，但只有 VISIBLE，没有 CONFIRMED。
其他状态的 flags、实际物料编号、置信度、中心、偏差及检测框全部为零，class_id=65535。
STM32 不得在无效状态中继续沿用之前保存的目标坐标。

## STM32 接入

新增模块不依赖 HAL 或旧 v1 通信层：

```text
stm32_firmware/Comm/Inc/rpi_material_vision.h
stm32_firmware/Comm/Src/rpi_material_vision.c
```

把这两个文件加入队友 v2 工程，增加 include path。仓库里的 rpi_protocol.c 等旧 v1
示例不能整体复制替换队友 v2 协议；本模块不负责 v2 帧解析、握手或心跳。

在唯一通信任务/主循环中持有接收器：

```c
#include "rpi_material_vision.h"

static RpiMaterialReceiver material_receiver;

/* 初始化，以及新二进制会话开始/结束、STOP_ALL、USB/主机故障时调用。 */
RpiMaterialReceiver_Reset(&material_receiver);

/* 嵌入现有 v2 command 分发器，data 已剔除 opcode。 */
if (opcode == RPI_CMD_UPDATE_MATERIAL_VISION)
{
    uint8_t status = (uint8_t)RpiMaterialReceiver_Receive(
        &material_receiver, data, data_length, HAL_GetTick());
    /* 使用现有 RESPONSE 发送函数回复原 request_sequence、opcode、status。
       返回 data 长度为 0。status=0 只表示已经接收，并不执行机械动作。 */
}
```

实际加入分发逻辑后，在 SESSION_PROBE 和 HOST BINARY READY 中将能力位加上
RPI_CAP_MATERIAL_VISION (0x40)：原 0x3F 变为 0x7F。不要只改能力常量而不接收数据。

接收器验证字段、拒绝重复/倒序帧；切换视觉 session_id 必须先 Reset。
无效状态替换旧位置。GetLatest 提供新鲜诊断数据；GetTarget 只返回新鲜且连续确认的目标。
两者在失败时都会清空 output，避免调用者误用上一次结果。
读写/Reset 应在同一主循环完成；FreeRTOS 多任务共享时须用互斥锁保护整个接收器，
不要在 USB 接收中断内驱动机械臂或并发读写该结构。

STM32 控制循环应在每个纠偏周期调用 GetTarget，而不是只在收到包时复制一次坐标：

```c
RpiMaterialObservation target;
if (!RpiMaterialReceiver_GetTarget(&material_receiver, HAL_GetTick(), &target))
{
    /* 停止/暂停视觉纠偏；不得启动新抓取。替换为本机实际安全状态转换。 */
}
else
{
    /* 检查本机任务编号、标定、限位、碰撞/急停，再用 center/offset 做闭环。
       只有本机判断已对准且动作安全，才启动抓取；抓取完成需独立反馈。 */
}
```

此模块没有电机/舵机/PID 实现。视觉过期不等于应打断任何机械阶段：
具体安全动作应由 STM32 当前动作状态决定，但不得用过期坐标继续新的视觉运动。
Pi 和 STM32 的时钟不是同一时基，禁止用 HAL_GetTick 减 capture_tick_ms 判断新鲜度。

## 发送频率和失效保护

config/material_serial.json 当前为最多 10 Hz、视觉有效期 250 ms、应答超时 200 ms。
有效期只是待实测的初值；模型耗时超过有效期会发送 STALE。树莓派扣除采集和推理耗时，
STM32 从本机接收时间计剩余有效期；运输/队列延迟应通过实机测试纳入额外安全裕量。
目标丢失、状态变化或切换目标会绕过限频立即上报，不等待下一个周期。
不自动重试 ACK 超时，不重放旧位置；USB 重连后只上报重新采集的结果。
正常退出发送 STOPPED；采图异常尝试发送 CAMERA_ERROR。
如果下发失败或相机阻塞，即使串口 PING 仍正常，STM32 也必须独立让视觉数据过期。

## 测试

```bash
python3 -m unittest tests.test_material_stm32 -v
python3 -m unittest discover -s tests -t .
```

有 gcc 时会用 -Wall -Wextra -Werror 编译 C 接收模块，并用 Python 编码的真实 47 字节
输入进行交叉测试。测试覆盖 ACK 匹配、旧固件能力拒绝、无对准门控、丢失清零、
有效期、重复/倒序保护、计数回绕和新会话重置；不等价于真实相机/USB/机械臂联调。
