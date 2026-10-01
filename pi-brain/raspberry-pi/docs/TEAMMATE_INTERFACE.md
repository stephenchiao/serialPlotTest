# 队友固件接口适配（2026-10-01）

依据：`serialPlotTest-main (1).zip`，SHA256：
`F222EB1B293EF52997E7E6812A2ED50D24CA9E5541F236BDA0491619F9849EBA`。
接口以压缩包内 `protocol/rpi_binary_protocol.json`、`Core/Src/robot_app.c`、
`Core/Src/usart.c` 为准。压缩包中的交接提示和历史计划作为参考资料。

## 树莓派改动

| 接口 | 树莓派实现 |
| --- | --- |
| ASCII v4 / 二进制 v2，64 字节 payload，CRC16 | 保留现有 v2 实现，新增原始 schema 及逐项兼容性测试 |
| USART1 115200 8-N-1，PA9 TX / PA10 RX | SerialLink 支持 uart，新增 stm32_uart.json 模板及工具 --transport 参数 |
| ASCII 启动自检 | 在现有握手中加入 STATUS、OPS STATUS、CAN STATUS、PID STATUS ALL |
| CAN STATUS 五行应答 | 等待最终 ESR 行，汇总 STATE/READY/ESR；兼容任意分片和多行合并 |
| 启动失败 | 不进入二进制会话，尝试 STOP_ALL 后关闭设备 |
| 原子位姿和限速 0x84 | 继续使用 goal_id + mm/mrad + ms + µm/s/µrad/s，配置角速度上限改为 800 mrad/s |
| 断线恢复 | 每次重连重新自检，旧航点不重发，启动快照清除 |
| 故障 6 | 补充 UART_FAULT 名称，保留 USB_LINK_FAULT 别名 |

查询 PID 只读取 STM32 已有参数。每个航点的速度限制通过 0x84 原子提交，遵循固件
20~300 mm/s、20~800 mrad/s 范围。ASCII OPS 启动坐标为 mm/mm/degree，
二进制遥测和航点的角度为 mrad，两者不能直接混用。

## 连接和验证

压缩包没有 USB CDC 中间件，原生 Type-C 数据线连接仍需队友完成 CDC 适配。
直接使用当前固件：USB 转 3.3V TTL 串口 TX 接 PA10/RX，RX 接 PA9/TX，GND 共地。
GPIO UART 连接也可使用 `transport: uart`，端口以树莓派实际配置为准。

```bash
python3 -m tools.stm32_link_test --list-ports
python3 -m tools.stm32_link_test --transport uart --port /dev/ttyUSB0 --count 10
```

完整自检会进入 RPI/WORK 并可能使能底盘，架空车轮并准备急停后运行：

```bash
python3 -m tools.stm32_link_test --transport uart --port /dev/ttyUSB0 --handshake
python3 -m tools.stm32_ops9_monitor --transport uart --port /dev/ttyUSB0
```

正式导航统一读取 `config/stm32.json`。已有 USB 配置保留；实际走 UART 时将
`transport` 改为 `uart`、`port` 改为实际设备，可参考 `config/stm32_uart.json`。
地图、相机和 OPS 起点标定要求仍需完成。

## 队友尚未实现的接口

- 原生 USB CDC：当前固件只接 USART1，不能仅靠树莓派端改配置获得 CDC 设备。
- 物料视觉 0x85 / 能力 0x40：当前固件只有基础能力 0x3F；现有上报器会拒绝发送。
  需要队友接入仓库里的独立物料接收模块，再声明能力位，见 [物料上报说明](MATERIAL_STM32.md)。
- 任务码、舵机、机械臂抓取完成事件：当前二进制命令表没有这些接口；
  视觉识别可单独运行，完整实机搬运仍需双方约定动作接口。

不要将导航 REACHED 或普通 ACK 解释为机械臂抓取完成。

## 无硬件验证

```bash
python3 -m unittest discover -s tests -v
```

覆盖 schema 常量、UART/CDC 会话、自检应答拆包、OPS/CAN 故障拒绝、PID 无效应答、
未实现的物料接口拒绝，以及已有航点/遥测/断线恢复测试。真实串口、CAN、电机和
OPS9 联调需要在板上执行。
