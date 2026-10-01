# STM32F407 Type-C USB CDC 集成说明

`Comm/Inc` 和 `Comm/Src` 是可复制进 STM32CubeIDE 工程的通信层。协议核心
`rpi_protocol.c` 不依赖 USB；正式方案使用 `rpi_usb_cdc_link.c`。CDC 接收回调只把
完整帧放入队列，命令在 `RpiUsbCdcLink_Process()` 中处理，不在 USB 中断中驱动
底盘。原 `rpi_uart_link.c` 作为台架备用后端保留。

注意：本目录上述传输示例为旧 v1，与树莓派当前使用的队友 v2 协议不兼容。
新增 `rpi_material_vision.h/.c` 不依赖旧传输，可单独加入实际 v2 工程，
接收 cam0 的类别和像素位置，再由 STM32 做纠偏与抓取。
加入 0x85 分发、能力位和视觉过期检查的方法见
[物料接入说明](../docs/MATERIAL_STM32.md)；不要整套覆盖 v2 固件。

## 1. CubeMX 配置

在 CubeMX 中配置：

- `USB_OTG_FS`：`Device_Only`；
- Middleware → `USB_DEVICE`：Class 选择 `Communication Device Class (CDC)`；
- USB 时钟必须为准确的 48 MHz；
- STM32F407 USB FS 通常使用 `PA11/USB_DM`、`PA12/USB_DP`；
- VBUS sensing 是否启用必须匹配实际板卡原理图；没有连接 VBUS sensing 引脚时
  在 CubeMX 中关闭它。

Type-C 口必须有完整 D+/D- 数据连接和设备端 CC 配置。成品开发板通常已经完成；
自制 PCB 必须按 USB Type-C Device 规范检查 CC1/CC2 下拉、ESD 和走线。

## 2. 加入文件

把以下文件加入工程，并将 `Comm/Inc` 加入编译器 include path：

```text
Comm/Inc/rpi_protocol.h
Comm/Inc/rpi_link.h
Comm/Inc/rpi_transport.h
Comm/Inc/rpi_usb_cdc_link.h
Comm/Inc/rpi_app_commands.h
Comm/Inc/rpi_ops9.h
Comm/Inc/rpi_pose_goal.h
Comm/Src/rpi_protocol.c
Comm/Src/rpi_usb_cdc_link.c
Comm/Src/rpi_app_commands.c
Comm/Src/rpi_ops9.c
Comm/Src/rpi_pose_goal.c
```

若接入队友的位姿闭环底盘，再加入：

```text
Integration/Inc/rpi_pose_platform.h
Integration/Src/rpi_pose_app_bindings.c
```

底盘工程只需实现 `RpiPosePlatform_Start/Cancel/GetStatus` 三个非阻塞端口。开始、
到位、取消或本地安全停车时，分别调用 `RpiPoseGoal_SendStarted/Reached/Cancelled/Fault`。
不要保留原 ASCII `POSE SET` 解析器与二进制命令同时驱动同一组静态控制变量。

正式 USB CDC 固件不需要加入 `rpi_uart_link.h/.c`。`rpi_transport.h` 默认选择 USB
CDC；台架若要切回 UART，再加入这两个文件并在编译器宏中设置：

```text
RPI_TRANSPORT_BACKEND=RPI_TRANSPORT_UART
```

在 CubeMX 生成的 USB 初始化完成后启动链路：

```c
#include "rpi_usb_cdc_link.h"

MX_USB_DEVICE_Init();
if (RpiUsbCdcLink_Start() != HAL_OK)
{
    Error_Handler();
}
```

在 CubeMX 生成的 `USB_DEVICE/App/usbd_cdc_if.c` 中找到 `CDC_Receive_FS()`，在重新
挂载接收缓冲区之前加入一行：

```c
static int8_t CDC_Receive_FS(uint8_t *Buf, uint32_t *Len)
{
    RpiUsbCdcLink_OnReceive(Buf, *Len);

    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    return (USBD_OK);
}
```

不要删除 CubeMX 生成的 `SetRxBuffer/ReceivePacket`，否则 USB 只会收到第一包。

裸机主循环持续处理队列：

```c
while (1)
{
    RpiUsbCdcLink_Process();
    /* 其他非阻塞任务 */
}
```

使用 FreeRTOS 时，在唯一通信任务中每 1~5 ms 调一次
`RpiUsbCdcLink_Process()`。OPS9、位姿事件等上行消息应从该任务或受控的消息队列
触发，不要从多个任务并发调用 CDC 发送函数。

OPS9 驱动完成一帧解析后，在主循环或传感器任务中转发位姿：

```c
#include "rpi_ops9.h"

RpiOps9_SendPose(ops9_x_mm,
                 ops9_y_mm,
                 ops9_yaw_mrad,
                 HAL_GetTick(),
                 ops9_quality,
                 RPI_OPS9_STATUS_VALID |
                 RPI_OPS9_STATUS_CALIBRATED |
                 RPI_OPS9_STATUS_CONTACT_OK);
```

推荐 20 Hz 上报。OPS9 原始帧格式、串口号和清零方式仍由实际 OPS9 型号的驱动
负责；`rpi_ops9.c` 只统一 STM32 到树莓派的数据格式。

## 3. 绑定底盘函数

`rpi_app_commands.c` 已完成参数长度检查和小端解析，USB CDC 与备用 UART 共用。
请在自己的应用 `.c` 文件中
提供以下非 weak 函数，覆盖默认安全实现：

```c
#include "rpi_app_commands.h"

RpiResponseStatus RpiApp_StopAll(void)
{
    Chassis_Stop();
    Manipulator_Stop();
    return RPI_STATUS_OK;
}

RpiResponseStatus RpiApp_SetChassisVelocity(int16_t vx,
                                            int16_t vy,
                                            int16_t wz)
{
    if ((vx < -1000) || (vx > 1000) ||
        (vy < -1000) || (vy > 1000) ||
        (wz < -3000) || (wz > 3000))
    {
        return RPI_STATUS_INVALID_ARGUMENT;
    }
    Chassis_SetVelocity(vx, vy, wz); /* 替换为你们的真实函数 */
    return RPI_STATUS_OK;
}
```

未覆盖钩子时会返回 `INTERNAL_ERROR`，不会假装动作已经成功。`PING` 不依赖底盘
钩子，可先用于接线测试。

位姿事务命令的数据布局为小端序：

```text
SET_POSE_GOAL: goal_id:u32, x_mm:i32, y_mm:i32, yaw_mrad:i32, timeout_ms:u32
CANCEL_POSE_GOAL: goal_id:u32
QUERY_POSE_GOAL响应: goal_id:u32, state:u8, x/y/yaw:i32, fault_reason:u16
```

`goal_id=0` 和 `timeout_ms=0` 会被通信层拒绝。底盘端收到重复或冲突目标时应返回
`BUSY`，不得覆盖正在运动的旧目标。

## 4. 通信失联保护

树莓派端默认每 100 ms 发送 HEARTBEAT。底盘使能后，应在安全任务里检查最近有效
帧时间；建议连续 300 ms 未收到帧就立即停止，具体阈值再按现场测试调整：

```c
if (motors_enabled &&
    ((uint32_t)(HAL_GetTick() - RpiUsbCdcLink_GetLastReceiveTick()) > 300U))
{
    Chassis_Stop();
    Manipulator_Stop();
    motors_enabled = 0U;
}
```

该保护属于 STM32 本地安全逻辑，不能依赖树莓派发 STOP。协议说明和树莓派用法见
[`robot_hardware/stm32/README.md`](../robot_hardware/stm32/README.md)。
