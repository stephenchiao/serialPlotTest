# 串口助手命令参考

本文档依据当前 `Core/Src/robot_app.c`、`Core/Src/llm_tuner.c` 和
`Core/Inc/llm_tuner.h` 整理，适用于通过普通串口助手向 STM32F407 发送文本命令。

2026-09-28 起，CAN1 使用无轮速反馈控制：取消 PLOT 和后台查询；轮速缓存仅供
按需诊断，运动和恢复不再要求新鲜轮速。详见 `can-command-only-2026-09-28.md`。

## 1. 串口设置

| 项目 | 设置 |
|---|---|
| 主机串口 | USART1 |
| 波特率 | 115200 |
| 数据位 | 8 |
| 停止位 | 1 |
| 校验位 | 无 |
| 流控 | 无 |
| 行结束 | `LF` 或 `CRLF`，每条命令单独一行 |

命令区分大小写，本文示例均使用固件接受的大写形式。普通状态输出以 `#` 开头，
位姿遥测以 `@P` 开头，PID 调参数据为不带前缀的 CSV。

上电后固件等待主机声明所有权。未声明主机时仅接受 `PING`、`PROTO VERSION`、
`HELP` 和 `STOP`。

普通串口助手连接后建议先发送：

```text
STOP
HOST LINK COM
PROTO VERSION
STATUS
CAN STATUS
OPS STATUS
MOTOR FEEDBACK
```

`HOST LINK COM` 会执行以下安全动作：

- 停止当前运动；
- 将模式恢复为 `WORK`；
- 将电机掩码恢复为 `0x0F`；
- 关闭连续遥测；
- 重置旧调参会话；
- 必要时重新使能四台 ZDT 电机；
- COM 模式不要求周期性发送 `PING`。

> `STOP` 是全局安全停车命令。发现运动异常、OPS 丢失或 CAN 异常时应优先发送
> `STOP`，必要时直接断开电机电源。

## 2. 主机连接与基础命令

| 命令 | 作用 | 说明 |
|---|---|---|
| `HOST LINK COM` | 声明普通串口助手为主机 | 停车并恢复 `WORK`、`MASK=0x0F` |
| `HOST LINK RPI` | 声明树莓派为主机 | RPI 模式要求心跳，不用于普通串口助手 |
| `HOST STATUS` | 查看主机所有权、底盘使能和心跳状态 | 可在声明主机前使用 |
| `PING` | 链路探测 | 回复 `# PONG`；COM 模式无需定期发送 |
| `PROTO VERSION` | 查看文本协议版本与能力 | 不会产生运动 |
| `HELP` | 让 MCU 打印内置命令摘要 | 不会产生运动 |
| `STATUS` | 查看模式、当前调参轴、PID、输出限幅、OPS 帧数等 | 需要先声明主机 |
| `STOP` | 停止所有运动 | TUNE 轮次中会结束当前轮次 |
| `RESET` | 停车并重置当前 PID 内部状态及调参轮次计数 | 不会把 PID 系数恢复为编译默认值 |

### RPI 二进制入口

`HOST BINARY START` 用于树莓派二进制协议切换，只能在 `HOST LINK RPI`、`WORK`
模式、底盘空闲且 CAN/OPS 正常时使用。普通串口助手不要发送该命令，否则后续通信
将不再是普通文本交互。

## 3. 工作模式

| 命令 | 模式用途 | 主要允许的运动 |
|---|---|---|
| `MODE WORK` | 正常工作模式 | `POSE SET`；允许使能 G6220 |
| `MODE TUNE` | PID 调参和台架测试 | PID 轮次、`MOTOR RUN`、`MOVE`、`TURN` |
| `MODE PLOT` | 已停用 | 返回 PLOT RETIRED；调试动作使用 TUNE |
| `MODE STATUS` | 查询当前模式 | 不产生运动 |

所有模式切换都会先停车并清除旧运动状态。切换到 `WORK` 时，电机掩码
恢复为 `0x0F`；`MODE TUNE` 保留当前掩码。

## 4. 诊断命令

| 命令 | 输出内容 |
|---|---|
| `CAN STATUS` | CAN1 状态、HAL 错误、邮箱、TX/RX、超时、恢复次数、ESR/TSR、TEC/REC |
| `MOTOR FEEDBACK` | ID1～4 的反馈有效性、实际 RPM、年龄和序号 |
| `MOTOR STOP STATUS` | 停车命令状态、掩码、诊断新鲜度和耗时；SENT 仅代表 CAN 发送完成 |
| `CONTROL STATUS` | 主循环最大周期、控制超时、UART 队列和 CAN 队列统计 |
| `HOST RX STATUS` | 文本接收丢弃数和无效命令统计 |
| `OPS STATUS` | OPS 链路、原始坐标、车体中心坐标、帧计数和数据年龄 |
| `POSE STATUS` | 位姿目标、当前坐标、规划速度与实际命令速度 |
| `PID STATUS ALL` | 一次打印 X、Y、YAW 三轴 PID 系数 |
| `MOTOR MASK STATUS` | 当前参与控制的电机掩码 |
| `TELEM STATUS` | 连续遥测选择、序号和发送统计 |
| `PLOT STATUS` | 返回 ENABLED=0 RETIRED=1 |
| `G6220 STATUS` | CAN2 上 G6220 的状态和反馈 |

`OPS STATUS` 中的 `LINK` 含义：

- `OK`：最近收到有效坐标帧；
- `STALE`：曾经收到有效帧，但当前已停止更新；
- `BYTES_NO_FRAME`：串口有字节，但未解析出有效 OPS 帧；
- `NO_DATA`：没有收到任何 OPS 串口字节。

## 5. ZDT 电机命令

轮位与 ID：

| ID | 轮位 | 掩码位 |
|---:|---|---:|
| 1 | 左后 BL | `0x01` |
| 2 | 左前 FL | `0x02` |
| 3 | 右前 FR | `0x04` |
| 4 | 右后 BR | `0x08` |

### 协议选择

| 命令 | 作用 |
|---|---|
| `PROTO EMM` | 使用 Emm 固件的数据格式 |
| `PROTO X` | 使用 X 固件的数据格式 |

切换协议会先停车。该设置必须与电机实际固件一致，不能用它尝试修正电机方向。

### 使能、停止和查询

| 命令 | 作用 |
|---|---|
| `MOTOR EN <id>` | 使能指定电机 |
| `MOTOR DIS <id>` | 先立即停止，再失能指定电机 |
| `MOTOR STOP <id>` | 指定电机立即停止但不失能 |
| `MOTOR STOP ALL` | 停止当前掩码内全部 ZDT 电机，并取消其他底盘运动 |
| `MOTOR GET <id>` | 主动读取指定电机的转速和状态 |

`id` 必须为 1～4。

### 单电机定时运行

```text
MOTOR RUN <id> <signed_rpm> [duration_ms]
```

参数限制：

- 仅允许在 `TUNE` 模式运行；
- `id`：1～4；
- `signed_rpm`：非零，范围 `-300～+300 RPM`；
- `duration_ms`：可省略，默认 2000 ms，范围 100～10000 ms；
- 指定电机必须包含在当前 `MOTOR MASK` 中；
- 到期后自动停车；
- 成功入队后会输出一次该电机的 `0xF6` ACK，正常结果为 `CODE=0x02 OK`。

示例：

```text
MODE TUNE
MOTOR MASK 0x01
MOTOR RUN 1 30 1000
MOTOR STOP 1
```

## 6. 电机掩码

```text
MOTOR MASK 0x01..0x0F
MOTOR MASK STATUS
```

只有 `TUNE` 模式可以修改掩码。修改掩码会立即停车。掩码同时决定：

- 单电机命令可以操作哪些电机；
- 底盘速度下发到哪些电机；
- TUNE ARMING 与故障恢复要求哪些电机的停车帧发送完成。

常用值：

| 掩码 | 电机 |
|---:|---|
| `0x01` | 仅 ID1 |
| `0x02` | 仅 ID2 |
| `0x04` | 仅 ID3 |
| `0x08` | 仅 ID4 |
| `0x03` | ID1 + ID2 |
| `0x0C` | ID3 + ID4 |
| `0x0F` | 四轮整车 |

`MOVE`、`TURN` 等整车调试命令要求 `MASK=0x0F`。

## 7. 底盘定时调试动作

这些命令只允许在 `TUNE` 模式使用，要求 `MASK=0x0F`、CAN 正常且
OPS 数据新鲜。动作到期后自动停车。

### 平移

```text
MOVE FWD|BACK|LEFT|RIGHT [speed_mps] [duration_ms]
```

- `speed_mps`：可省略，默认 0.04 m/s，范围大于 0 且不超过 0.08 m/s；
- `duration_ms`：可省略，默认 1000 ms，范围 100～3000 ms；
- 方向是车体坐标：`FWD=+Y`、`BACK=-Y`、`LEFT=-X`、`RIGHT=+X`；
- 车体方向仅在车头对齐 OPS `+Y` 时与 OPS 全局方向重合。

示例：

```text
MODE TUNE
MOTOR MASK 0x0F
MOVE FWD 0.04 1000
MOVE LEFT 0.03 600
MOVE STOP
```

### 旋转

```text
TURN CW|CCW [speed_radps] [duration_ms]
```

- `speed_radps`：可省略，默认 0.15 rad/s，范围大于 0 且不超过 0.30 rad/s；
- `duration_ms`：可省略，默认 1000 ms，范围 100～3000 ms；
- `CW` 为顺时针，`CCW` 为逆时针。

停止底盘调试动作：

```text
MOVE STOP
```

## 8. OPS-9 与位姿控制

| 命令 | 作用 |
|---|---|
| `OPS STATUS` | 打印一次 OPS 状态 |
| `OPS MONITOR ON` | 每 1000 ms 自动打印一次 OPS 状态 |
| `OPS MONITOR OFF` | 关闭 OPS 自动打印 |
| `OPS ZERO` | 向 OPS-9 发送置零命令 `ACT0` |

### 绝对位姿目标

```text
POSE SET <x_mm> <y_mm> <yaw_deg>
POSE STATUS
POSE STOP
```

- `x_mm`、`y_mm` 是 OPS 全局绝对坐标，单位 mm；
- `yaw_deg` 是 OPS 航向角，单位 degree；
- 相对当前车体中心的单次目标距离不能超过 10000 mm；
- 启动前要求 CAN 正常且 OPS 数据新鲜；
- 控制过程先平移并保持起始航向，到位停车后再原地转到目标航向；
- 位置容差 2 mm，航向容差 0.5°；
- 正常完成时输出 `# POSE TARGET ...`；
- `POSE STOP` 立即取消位姿控制并停车。

普通串口助手建议先执行 `HOST LINK COM`。COM 主机不需要发送心跳；RPI 主机模式下
位姿控制依赖主机心跳。

## 9. PID 参数与自动测试轮次

### 当前固件基线参数

| 轴 | P | I | D |
|---|---:|---:|---:|
| X | 0.00495 | 0 | 0 |
| Y | 0.0018 | 0 | 0 |
| YAW | 0.02 | 0.000015 | 0 |

### 查看、装载参数

```text
PID STATUS ALL
PID SET X <p> <i> <d>
PID SET Y <p> <i> <d>
PID SET YAW <p> <i> <d>
```

`PID SET` 只装载参数并清除对应 PID 的历史状态，不会启动车辆。

参数硬限制：

| 轴 | P | I | D |
|---|---:|---:|---:|
| X/Y | 0～0.005 | 0～0.00005 | 0～0.002 |
| YAW | 0～0.05 | 0～0.00010 | 0～0.02 |

### 设置输出上限

```text
PID LIMIT X <mps>
PID LIMIT Y <mps>
PID LIMIT YAW <radps>
```

- X/Y：0.02～0.30 m/s；
- YAW：0.02～0.80 rad/s；
- `PID LIMIT` 可以按轴设置，不会启动运动。

### 启动 TUNE 轮次

先选择轴：

```text
TUNE AXIS X
TUNE AXIS Y
TUNE AXIS YAW
```

`TUNE AXIS` 会自动进入 `TUNE` 模式、停车并重置轮次计数。

为当前调参轴设置本轮输出上限：

```text
TUNE LIMIT <mps_or_radps>
```

- X/Y：0.02～0.30 m/s；
- YAW：0.02～0.80 rad/s。

以下四种格式等价，都会更新当前轴 PID 并立即启动一轮测试：

```text
PID <p> <i> <d>
SET P:<p> I:<i> D:<d>
SET KP:<p> KI:<i> KD:<d>
P:<p>,I:<i>,D:<d>
```

TUNE 轮次特性：

- X/Y 相对移动目标为 200 mm；
- YAW 相对旋转目标为 30°；
- 每轮最长 5 秒；
- 每次新轮次自动切换正、反方向；
- 每个会话最多 20 轮；
- COM 模式下轮次不依赖 `PING`；
- 正常结束原因是 `TARGET` 或 `TIMEOUT`；
- `CAN FAULT`、`OPS LOST`、`WRONG DIR`、`YAW LIMIT`、`CROSS TRACK`、
  `TRANSLATION LIMIT`、`OVERTRAVEL` 等属于安全停止，出现后不要直接继续下一轮。

推荐的手工测试顺序：

```text
STOP
HOST LINK COM
PID SET X 0.00495 0 0
PID SET Y 0.0018 0 0
PID SET YAW 0.02 0.000015 0
TUNE AXIS X
MOTOR MASK 0x0F
TUNE LIMIT 0.10
STATUS
PID 0.00495 0 0
```

测试结束：

```text
STOP
PID STATUS ALL
MODE WORK
```

### TUNE CSV 格式

轮次进入 `# ROUND START` 后，MCU 每约 50 ms 输出一行 17 列 CSV：

```text
elapsed_ms,target,input,output,error,p,i,d,
ops_x,ops_y,ops_yaw,cross_track,yaw_delta,
hold_cross_output,hold_yaw_output,center_x,center_y
```

字段说明：

| 列 | 含义 |
|---:|---|
| 1 | 本轮已运行时间，ms |
| 2 | 目标值，X/Y 为 mm，YAW 为 degree |
| 3 | 按当前轮次方向归一化后的输入 |
| 4 | 主轴输出，X/Y 为 m/s，YAW 为 rad/s |
| 5 | `target - input` |
| 6～8 | 当前 P/I/D |
| 9～11 | OPS 原始 X/Y/YAW |
| 12 | 交叉方向偏差 |
| 13 | 相对本轮起点的航向变化 |
| 14～15 | 交叉轴与航向保持输出 |
| 16～17 | 经过 OPS 安装偏置修正后的车体中心坐标 |

调参期间建议发送 `TELEM OFF`，避免 `@P` 连续遥测与 TUNE CSV 混杂。

## 10. 连续遥测与 PLOT

| 命令 | 作用 |
|---|---|
| `TELEM OFF` | 关闭连续遥测 |
| `TELEM WHEEL` | 已停用，返回 PLOT RETIRED |
| `TELEM POSE` | 只输出位姿遥测 |
| `TELEM BOTH` | 已停用，返回 PLOT RETIRED |
| `TELEM STATUS` | 查看遥测状态 |
| `PLOT ON` | 已停用，返回 PLOT RETIRED |
| `PLOT OFF` | 关闭遥测并回到 `WORK` |
| `PLOT STATUS` | 查看停用状态 |

POSE 周期约 50 ms。没有后台电机查询，轮速需要 `MOTOR GET <id>` 按需读取。
`MOTOR FEEDBACK` 的 AGE 增长、VALID=0 不再阻止运动或故障恢复。

位姿格式：

```text
@P,1,tick_ms,sequence,ops_x,ops_y,ops_yaw,center_x,center_y,
command_vx,command_vy,command_vz
```

坐标单位为 mm/degree，速度单位为 m/s、m/s、rad/s。

## 11. CAN2 / G6220 命令

| 命令 | 作用 | 限制 |
|---|---|---|
| `G6220 STATUS` | 查询 CAN2 和 G6220 反馈 | 不产生运动 |
| `G6220 ENABLE` | 使能 G6220 | 仅允许 `WORK` 模式 |
| `G6220 DISABLE` | 失能 G6220 | 任意已连接模式 |

这些命令操作 CAN2，与 ZDT 电机所在的 CAN1 独立。

## 12. 常用操作流程

### 只检查系统，不运动

```text
STOP
HOST LINK COM
STATUS
CAN STATUS
OPS STATUS
MOTOR FEEDBACK
MOTOR STOP STATUS
PID STATUS ALL
```

### 测试一台 ZDT 电机

```text
STOP
HOST LINK COM
TUNE AXIS X
MOTOR MASK 0x01
MOTOR GET 1
MOTOR RUN 1 20 500
MOTOR GET 1
MOTOR STOP 1
MOTOR STOP STATUS
MODE WORK
```

测试其他电机时，将掩码和 ID 同步替换为 `0x02/2`、`0x04/3` 或 `0x08/4`。

### 低速验证整车方向

```text
STOP
HOST LINK COM
MODE TUNE
MOTOR MASK 0x0F
MOVE FWD 0.03 500
MOVE BACK 0.03 500
MOVE LEFT 0.03 500
MOVE RIGHT 0.03 500
TURN CCW 0.10 500
TURN CW 0.10 500
STOP
MODE WORK
```

每条动作之间应等待自动停车并观察实际方向，不要一次性粘贴整段运动命令。

## 13. 常见错误

| 返回 | 含义与处理 |
|---|---|
| `# ERROR HOST NOT LINKED` | 先发送 `HOST LINK COM` |
| `# ERROR MODE REQUIRED=TUNE CURRENT=WORK` | 调试运动前切换到 `MODE TUNE` |
| `# ERROR MODE REQUIRED=TUNE CURRENT=...` | PID 轮次前执行 `TUNE AXIS ...` |
| `# ERROR CHASSIS MOVE REQUIRES MOTOR MASK=0x0F` | 整车动作前恢复 `MOTOR MASK 0x0F` |
| `# ERROR DEBUG CHASSIS SAFETY CAN=... OPS=...` | 检查 `CAN STATUS` 和 `OPS STATUS` |
| `# ERROR MOTOR RUN CAN NOT READY ...` | CAN1 当前不允许运动，检查 ESR、TEC、REC 和电机供电 |
| `# ROUND STOP STOP NOT SENT` | ARMING 超时前停车命令未发送完成，检查 CAN 与电机供电 |
| `# ROUND STOP YAW LIMIT` | 平移过程中相对航向变化超过 15° |
| `# ROUND STOP CROSS TRACK` | X/Y 测试的交叉方向偏移超过 50 mm |
| `# ROUND STOP CAN FAULT` | 立即停止测试并保存 `CAN STATUS` 输出 |
| `# ERROR UNKNOWN COMMAND` | 检查大小写、空格和参数格式 |
