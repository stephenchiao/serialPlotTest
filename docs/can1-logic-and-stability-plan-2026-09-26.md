# CAN1 代码逻辑与稳定性改造方案

本文记录 2026-09-26 **重构前**的代码审查、软件复现和初版方案，其中源码行号及候选代码属于当时版本。后续已按两批实施生产固件重构，最终逻辑、各批测试结果与复测方法见 [CAN1 两批重构与验证记录](can1-refactor-two-waves.md)。未烧录或操作电机。按软件问题梳理，不将排查方向转回接线。

## 1. 核心结论

当前 `CAN READY` 是“控制器状态合格且没有运动故障锁存”，并不是“CAN 是否还能收发”。存在可纯软件复现的 READY=0 路径。代码审查不能给出现场发生概率，也不能确定六条路径中哪条是本次首次触发源。

```c
/* Core/Src/zdtCan.c:483 */
uint8_t ZDT_CAN_IsReady(void)
{
    return CanHardwareHealthy() && !tx_fault;
}
```

`CanHardwareHealthy()` 要求：

- HAL 状态为 LISTENING。
- HAL 历史错误不含 TIMEOUT / NOT_INITIALIZED / NOT_READY / NOT_STARTED / INTERNAL。
- 实时 ESR 无 EWGF / EPVF / BOFF。

它不要求 TEC/REC=0，也不因历史 PARAM、ACK、普通协议错误直接返回 0。但是恢复入口另有更严格条件，导致“运行健康判定通过、故障却不能解除”的矛盾。

旧文档中 ConsumeFault 清锁存、MOTOR RUN 开始前无条件 ClearFault、多处自动解锁等问题已被当前代码修复，不能再作为当前缺陷引用。当前 `ConsumeFault()` 只消费事件，`AUTO_REC` 恒为 0，100 ms 的链路观察也不解锁。

## 2. 当前通信路径

| 层 | 文件 / 入口 | 职责 |
| --- | --- | --- |
| CAN1 初始化 | `Core/Src/can.c:31` | 正常模式、500 kbit/s、自动重发、自动 Bus-Off 恢复；三个 TX 邮箱 |
| 应用初始化 | `Core/Src/robot_app.c:2358` | 配置并启动 CAN、注册接收回调、初始化四个电机、停车、使能 |
| 应用调度 | `Core/Src/robot_app.c:2441` | 主机命令、安全检查、运动计算、反馈轮询、CAN 泵、空闲恢复 |
| 协议层 | `Core/Src/zdtEmm.c` | 地址、功能码、EMM/X 编解码，反馈更新 |
| 传输层 | `Core/Src/zdtCan.c` | 软件待发项、三个硬件邮箱、超时、撤销、故障锁存、恢复 |
| 中断入口 | `Core/Src/board_events.c` / `stm32f4xx_it.c` | CAN1 RX 交给 ZDT，CAN2 RX 交给 G6220；TX 完成更新计数 |
| 运动安全 | `Core/Src/mecanum_chassis.c` / `motor_monitor.c` | 反馈新鲜度、停车确认、停止重试 |

### 初始化

`MX_CAN1_Init()` → `RobotApp_Init()` → `ZDT_CAN_ConfigFilter()` → HAL ConfigFilter/Start/ActivateNotification。

CAN1 的过滤器 bank=0，CAN2 从 bank=14 开始；CAN1 当前过滤器接收所有帧，ISR 再筛选扩展数据帧。CAN1 位时序为 42 MHz / [14 × (1+4+1)] = 500 kbit/s。此次方案保留这些参数。

当前仅启用 RX FIFO0 pending 与 TX mailbox 通知；错误状态主要由主循环轮询 ESR。错误类通知关闭不等于错误回调绝不执行：HAL TX 中断处理中的发送错误也可走 ErrorCallback。

启动顺序目前是先 Start/开中断，再注册协议回调，再初始化电机对象。正常启动阶段未发送查询前通常没有回复，但已有总线帧可能落在这个窗口；建议先初始化对象、注册回调，再启动 CAN，消除窗口。

### 发送

```text
串口命令 / PID / 调试动作
    → SetAllMotorsSpeed 或 ZDT_Emm_SetSpeedByID
    → zdtEmm 编码
    → ZDT_CAN_Send_ExtId 入队
    → ZDT_CAN_Process 选帧
    → HAL_CAN_AddTxMessage 装入硬件邮箱
    → CAN1 TX IRQ / TXOK 回调
    → 电机应用回复 / 速度反馈
```

协议使用扩展 ID `(电机地址 << 8) | 包序号`，当前单帧命令包序号=0，地址为 1–4：

| 功能 | 内容 |
| --- | --- |
| 使能 / 失能 | `F3 AB enable 00 6B` |
| EMM 速度 | `F6 dir velH velL acc sync 6B`，7 字节 |
| X 速度 | `F6 dir accH accL velH velL sync 6B`，8 字节，速度为 RPM×10 |
| 查询速度 | `35 6B` |
| 查询状态 | `3A 6B` |
| 停车 | 当前使用 F6 零速帧，走独立 STOP 槽 |

发送返回 0 只代表入队成功；TX_OK 代表 CAN 帧发送完成，不能证明指定电机执行了命令。应用 ACK 和新的有效速度反馈才提供下一层证据。

当前有三类软件待发项：普通命令 FIFO 16 项、每电机一个最新速度槽、每电机一个 STOP 槽。选择顺序为 STOP → 普通命令 → 速度；STOP/速度均从 motor 1 开始扫描，每次 Process 最多提交 3 帧。

使能 F3 提交邮箱后开始 5 ms 延迟，该电机的后续非 STOP 帧暂时不发。这个时间从“提交”开始，并非从 TXOK 或使能确认开始；如果当前选中项被延迟，整个发送循环返回，后面其他电机也被挡住。

### 接收与轮询

`CAN1_RX0_IRQHandler` → HAL IRQ → `board_events.c` → `ZDT_CAN_RxFIFO0_Handler` → `ZDT_Emm_RxHandler`。

ISR 每次最多读取 3 帧；协议层验证完整地址 1–4、包序号=0、末字节 6B。有效 `0x35` 回复为 5 字节，更新 RPM、tick、sequence、zero_streak。反馈更新在 ISR 中完成，主循环使用关中断拷贝取得一致快照。

应用每 10 ms 轮询一个地址，1→2→3→4，所以四轮模式下每轮约 40 ms 查询一次。未包含在 MOTOR MASK 的地址会跳过，但仍占用轮询时间位置。当前没有独立查询槽、没有相同查询去重，也没有“指定电机是否已回复”的事务管理。

### 停车与恢复

停车调用 BeginStop：丢弃普通队列和速度槽，申请撤销三个邮箱，然后填充所需电机的 STOP 槽。停车帧优先，仍待发时继续尝试。

停车确认需要：STOP 无待发邮箱/槽，所需电机反馈均在 300 ms 内，基线之后每电机至少两次新反馈，且至少连续两次速度绝对值 ≤1 RPM。600 ms 未确认则进入 UNCONFIRMED，之后每 100 ms 再申请停车。

运动中反馈过期立即停止；当前控制器不健康立即停止；软件锁存故障持续 100 ms 后停止。由于软件锁存只在空闲状态清除，这个 100 ms 已经是延迟停车，而不是“通信一恢复就可以继续”的容错窗。

唯一解除入口是 `ZDT_CAN_RecoverWhenIdle()`，要求：没有活动运动、所需电机反馈新鲜、停车确认、HAL LISTENING、TEC/REC 均为零、无当前 ESR 告警、HAL 高位错误为零，且观察 500 ms 内无新增故障、TX_OK 与 RX 增长。原队列不会恢复，原运动不会重启。

## 3. 当前问题与可复现条件

### A. 查询拥塞被升级为运动故障

`Motor_ProcessFeedbackPolling()` 注释和 `Mecanum_ReportPollResult()` 都说查询入队失败只记录统计。但是 `ZDT_CAN_Send_ExtId()` 在普通队列满时先执行 `CanLatchFault()`。上层拿到失败后只计数也无济于事，READY 已变为 0。

同样，查询在普通队列等待超过 50 ms 时，Process 会锁存故障并清掉包括速度在内的普通待发项。甚至帧尚未进入邮箱、FREE=3、控制器没有错误，也会触发。

这是已经证实的层间职责冲突。修复应在传输层识别查询类型，不能只修改 `ReportPollResult`。

### B. 恢复判据与运行判据不一致

当前 `RecoverWhenIdle()` 的 ESR 掩码含 `0xFFFF0000`，只要 TEC 或 REC 任意非零就不开始观察。REC=18 且 EWGF=EPVF=BOFF=0，能通过 HardwareReady，却不能解除旧故障。只要计数持续非零，就可一直保持 READY=0，即使收发持续增长。

另外恢复拒绝 `HAL_CAN_GetError() & ~0x0001FFFF`；PARAM=0x00200000，仍在拒绝范围内。于是当前代码虽然已经把 PARAM 从 READY 的硬阻断中移除，但只要同时发生过 tx_fault，历史 PARAM 仍可阻止解除。

HAL 库本地源码表明：邮箱满时 AddTxMessage 会记录 PARAM，GetRxMessage 读空 FIFO 也会记录 PARAM。因此必须按具体调用来源、调用参数和当前状态解释 PARAM，不能把累计位统一解释为配置不可恢复。真实非法参数仍必须报错，不应简单忽略。

### C. 最新速度继承旧等待时间，固定扫描可能导致饥饿

速度替换保留旧 `queued_at`。旧速度排队 40 ms 后被新目标覆盖，再过 11 ms，即使最新目标才 11 ms、邮箱已空闲，当前实现仍按 51 ms 判超时并锁存故障。

保留等待时长本来有检测长期无法下发的安全意图，不应简单把它全改为最新时间，否则持续覆盖会掩盖发送停滞。应拆成“目标更新时间”与“首次未服务时间”，分别判断数据新鲜度与控制服务截止时间。

速度槽固定从 1 扫描，预算为 3。在每 20 ms 只调用一次 Process、每次又更新前面三轮的条件下，第 4 轮可以一直得不到服务；等终于被选到时又因旧时间超时。默认主循环约 1 ms 服务时不一定出现，但主循环变慢、长命令处理、拥塞都可能放大这个缺陷。需要轮转游标和服务时限，而非只增加预算。

### D. Abort 被当作完成，重启路径太急

HAL Abort 只是设置 ABRQ 请求位，返回 HAL_OK 不代表 TME 已恢复。当前超时路径和 BeginStop 在申请后就清跟踪；ForceUnstick 在请求 Abort 后立即检查 FREE=0，并可能马上 Stop/Start。

应保留邮箱身份直到 TXOK / ABORT 完成或确认 TME 空闲，用分阶段非阻塞状态机等待。否则无法区分“撤销尚未完成”和“外设确实卡住”。Stop/Start 只改变初始化状态，也不应被当作完整外设硬复位。

本地 HAL Start/Stop 内部各有最长约 10 ms 的等待，可拖延主循环。若它们失败，HAL 状态变为 ERROR，而现有 RecoverWhenIdle 只接受 LISTENING，没有明确的失败后修复状态，容易留下新的锁死路径。

### E. 故障来源混在一个位中

队列满、查询过期、速度过期、AddTxMessage 失败、实时 EPVF/BOFF、反馈不新鲜甚至无效运动参数，都可能进入同一个 `tx_fault`。例如 SetAllMotorsSpeed 在反馈过期或输入非有限数时也 RaiseFault，之后 CAN READY 会为 0，但传输层本身未必坏。

CanPollEsrState 在 EPVF/BOFF 持续时每一轮都递增 generation，而不是只记录状态进入或新的独立故障；旧 PARAM/RX_FOV 等累计位又可能影响后续错误回调。需要记录故障来源和实时状态边沿，历史错误只作诊断。

## 4. 推荐简化后的流程

```text
初始化电机对象和协议回调 → 配置/启动 CAN1
    ↓
主循环：处理 RX/完成事件 → 检查当前链路和反馈 → 生成控制目标
    ↓
单一发送泵：STOP → 有界配置命令 → 公平轮转速度/查询
    ↓
正常：最新速度覆盖；每轮查询最多一个待发和一个在途事务
    ↓
故障：记录来源 → 禁止新的非零输出 → 清旧运动 → 发 STOP、保留反馈查询
    ↓
等待撤销完成 / 硬件 Bus-Off 自动恢复；确有停滞才修复外设
    ↓
当前控制器健康 + 所需轮停车确认 + 500 ms 连续有效通信
    ↓
解除运动故障锁存，等待新的显式运动命令
```

保留一个故障锁存和一个恢复观察窗口。100 ms “只显示恢复迹象”的独立计时可删去，其功能并入统一状态机。故障解除不能自动恢复旧目标。

建议接口分工：

```c
/* API 方案示意，尚未实现。 */
CanResult Can_SetSpeed(uint8_t motor, const CanFrame *frame); /* 每轮一个最新槽 */
CanResult Can_RequestSpeed(uint8_t motor);                   /* 可合并的查询槽 */
CanResult Can_QueueConfig(const CanFrame *frame);             /* 有界 FIFO */
void Can_RequestStop(uint8_t mask);                           /* 幂等停车事务 */
void Can_Process(uint32_t now);                              /* 唯一 HAL 发送入口 */
bool Can_LinkReady(void);                                    /* 当前控制器状态 */
bool Motion_CanStart(uint8_t mask);                           /* 锁存、反馈、停车、使能 */
```

保留现有 CAN STATUS 的 READY 字段含义，另加 HW_READY、FAULT_REASON、FAULT_AT_MS、RECOVERY_WAIT、QUERY_TIMEOUT、SPEED_WAIT_MAX_MS、每轮 TXOK 和有效反馈计数。上位机继续检查 READY 与 MOTOR FEEDBACK；新增字段用于解释拒绝原因。

## 5. 可以实施的代码改动

### 第一批：小范围修复，不改位时序、不改 300 ms 反馈和停车门槛

1. 给 `0x35/0x3A` 查询独立待发槽或去重标记。满/过期仅记录查询失败并丢弃该查询，不能清速度槽或设置运动锁存。实际反馈连续超过 300 ms 仍由安全层停车。
2. 用统一当前健康判据替代恢复中的 TEC/REC=0 和历史 PARAM 全阻断；恢复还需有效反馈与 TXOK 进展。HAL API 实际失败必须记录操作来源；真实配置错误走显式修复分支。
3. 速度使用轮转扫描；某电机处于使能等待时跳过该电机，不阻塞所有电机。配置命令限制服务配额，反馈查询也要有保证，不能长期饿死速度或查询。
4. 只允许一个发送泵直接调用 HAL；StopMask 不再自行递归调用 Process，而只设置停车请求，主循环及时服务。

查询槽示意：

```c
/* 方案示意：slot[n] 为每电机最新的一次查询请求。 */
static void RequestSpeedQuery(unsigned n, uint32_t now)
{
    if (query_inflight[n]) return; /* 等有效回复或回复超时 */
    if (query_slot[n].pending) return; /* 不刷新首次等待时间 */
    query_slot[n] = MakeSpeedQuery(n + 1U, now);
}

/* Process 中查询等待过期；不走 CanLatchFault / CanDropPendingQueue。 */
if (frame->kind == CAN_FRAME_QUERY && age > QUERY_QUEUE_MAX_MS) {
    frame->pending = 0U;
    query_expired++;
    continue;
}
```

`query_inflight` 应从查询 TXOK 开始跟踪，靠对应 motor 的有效 0x35 回复解除；回复截止时间从 TXOK 开始，不从入队开始。诊断查询 0x3A 需独立标记，不能把 0x35 的有效回复当作 0x3A 完成。回复丢失只重试查询；实际安全以反馈新鲜度为准。协议没有可回显事务号时，需约束同一 motor/function 只有一笔在途，不能声称能严格匹配跨代迟到回复。

速度槽示意：

```c
typedef struct {
    CanFrame frame;
    uint32_t updated_at;   /* 当前目标年龄 */
    uint32_t waiting_since;/* 连续未获得发送服务的时长 */
    bool pending;
} LatestSpeed;

/* 覆盖只更新目标年龄；等待截止时间由独立服务监督处理。 */
slot->frame = newest;
slot->updated_at = now;
if (!slot->pending) slot->waiting_since = now;
slot->pending = true;
```

即使总 TX_OK 在增长，也必须监测每轮速度是否得到服务；只有查询 TXOK 增长不能掩盖某轮控制输出停滞。已有 50 ms 可暂时保留为控制服务截止时间，待实测后再定，不能靠持续覆盖无限延长。

### 空闲恢复函数替换候选

下面是面向当前代码结构的完整候选函数，放在 `zdtCan.c` 中替换同名函数，并增加 `#include "zdtEmm.h"` / `#include "mecanum_chassis.h"`。已在临时副本通过主机 GCC 编译和专项模拟检查，尚未集成生产固件或实机验证；它解决 REC/历史 PARAM 阻断并把 RX 进展改为所需轮有效反馈进展。随后可将反馈快照作为函数参数，避免传输层依赖应用层。

它沿用 CanHardwareHealthy，所以 TIMEOUT/NOT_INITIALIZED/NOT_READY/NOT_STARTED/INTERNAL 仍需显式修复，不会通过 ResetError 强行放行。仅有 PARAM 历史记录不再永久阻止恢复，但非法调用必须先由输入验证和操作故障来源拦住。

```c
uint8_t ZDT_CAN_RecoverWhenIdle(uint8_t eligible)
{
    static uint8_t watching, observed_mask;
    static uint32_t since, generation, completed, submits, timeouts;
    static uint32_t baseline[4];
    MotorFeedback samples[4];
    uint8_t mask = Mecanum_GetRequiredMotorMask();
    uint8_t recovered = 0U;
    uint32_t saved, now;
    unsigned i;

    ZDT_Emm_GetFeedback(samples);
    saved = __get_PRIMASK();
    __disable_irq();
    now = HAL_GetTick();

    if (!tx_fault || !eligible || !CanHardwareHealthy() ||
        ZDT_CAN_StopPending() || mask == 0U ||
        (MotorFeedback_FreshMask(samples, now) & mask) != mask) {
        watching = 0U;
        goto done;
    }
    for (i = 0U; i < 4U; ++i) {
        if (!(mask & (1U << i))) continue;
        if (samples[i].rpm > MOTOR_ZERO_RPM ||
            samples[i].rpm < -MOTOR_ZERO_RPM ||
            samples[i].zero_streak < 2U) {
            watching = 0U;
            goto done;
        }
    }
    if (!watching || generation != fault_generation ||
        observed_mask != mask || submits != tx_error_count ||
        timeouts != tx_timeout) {
        watching = 1U;
        since = now;
        generation = fault_generation;
        observed_mask = mask;
        completed = tx_ok_count;
        submits = tx_error_count;
        timeouts = tx_timeout;
        for (i = 0U; i < 4U; ++i) baseline[i] = samples[i].sequence;
        goto done;
    }
    if ((uint32_t)(now - since) < 500U || completed == tx_ok_count)
        goto done;
    for (i = 0U; i < 4U; ++i) {
        uint32_t advance = samples[i].sequence - baseline[i];
        if ((mask & (1U << i)) && (advance < 2U || advance >= 0x80000000UL))
            goto done;
    }
    error_latched |= HAL_CAN_GetError(&hcan1);
    if (HAL_CAN_ResetError(&hcan1) == HAL_OK) {
        tx_fault = 0U;
        recovery_phase = auto_clear_armed = 0U;
        recoveries++;
        recovered = 1U;
    }
    watching = 0U;
done:
    __set_PRIMASK(saved);
    return recovered;
}
```

不要单独应用这个候选后就宣称通信根因解决；还必须修复查询错误分类和邮箱生命周期，并验证应用层 HOST LINK/MODE/STOP 的组合时序。

### 第二批：统一邮箱与停车状态机

每个邮箱保存 kind、motor、发送 tick、运动 generation 和状态：FREE → PENDING → TXOK，或 PENDING → ABORT_REQUESTED → ABORT_DONE。回调只记录结果，主循环处理业务。

```c
/* 方案示意：发出撤销后保留原邮箱跟踪。 */
if (mb.state == MB_PENDING && TxDeadlineExpired(&mb, now)) {
    RecordFaultForFrame(&mb); /* 查询和控制帧按不同策略分类 */
    if (HAL_CAN_AbortTxRequest(&hcan1, mb.mask) == HAL_OK) {
        mb.state = MB_ABORT_REQUESTED;
        mb.abort_at = now;
    } else {
        RecordApiFailure(CAN_OP_ABORT);
    }
}
if (mb.state == MB_ABORT_REQUESTED && AbortCompleted(&mb)) {
    ReleaseMailbox(&mb);
} else if (mb.state == MB_ABORT_REQUESTED && AbortDeadlineExpired(&mb, now)) {
    RequestControllerRepair(); /* 主循环分阶段执行，禁止直接反复 Stop/Start */
}
```

撤销等待时间需要结合 bxCAN 行为和现场测量设定；5–10 ms 可作为测试起点，不能把这个估值当成设备保证。外设修复只在无运动输出且停止已请求时进行，处理 Stop/Start/通知恢复失败，并保留原错误。CAN1/CAN2 共享时钟和过滤器资源，不能不加验证地复位整个 CAN1 资源或重新分配 filter banks。

停车请求应幂等：首次停止旧运动才撤销普通在途帧；每 100 ms 重发只补充尚未确认的电机 STOP，不清已成功提交的 STOP，也不反复清空反馈查询。开始新的停车事务前必须区分旧运动 generation。

使能也应成为小事务：F3 已入队 → F3 TXOK → ACK/状态确认 → 该电机允许速度；等待某电机时继续服务其他电机。当前 `motors[].enabled` 和 `chassis_motors_enabled` 在入队成功时就置位，应区分 requested 与 confirmed，不能把它们当真实使能确认。

## 6. 验证结果与实施顺序

本次执行 `python tools/run_c_tests.py`，5 个套件通过，其中 control layer 为 14 个测试组。额外复现位于 `tmp/can-ready-review/repro.c`，复用原主机 HAL fixture，对未修改生产代码验证了六个场景：查询队列满、过期查询、速度替换继承旧时间、REC=18 阻止恢复、PARAM 阻止恢复、20 ms 服务条件下 motor 4 饥饿。六项均按当前缺陷行为通过。

编译与运行：

```powershell
gcc -std=c11 -Wall -Wextra -Werror -DCONTROL_HOST_TEST -ICore/Inc -Itests/c tmp/can-ready-review/repro.c Core/Src/pid.c Core/Src/motor_monitor.c Core/Src/zdtCan.c Core/Src/zdtEmm.c Core/Src/mecanum_chassis.c Core/Src/host_uart_tx.c Core/Src/ops9.c Core/Src/control_runtime.c -lm -o tmp/can-ready-review/repro.exe
.\tmp\can-ready-review\repro.exe
```

这是当前问题的复现，不是新方案验收。REC 在 fixture 中被保持为 18，证明判据本身会拒绝；不能据此断言真实硬件计数永远不下降。原 HAL fixture 的 Abort 通常立即清 pending，也不模拟真实异步撤销，改造时必须增加异步完成测试。

另执行 `python tmp/can-ready-review/check_candidate.py`，脚本从本文提取恢复候选到临时源码，以 `-Wall -Wextra -Werror` 编译，专项检查全部通过：REC=18 和历史 PARAM 可以恢复；新故障重启 500 ms 窗口；当前 Bus-Off 和 HAL TIMEOUT 仍阻止恢复；缺失所需电机反馈仍阻止恢复。测试复用现有 HAL fixture，eligible 由 fixture 提供，没有覆盖完整 robot_app 的停车、模式切换或硬件邮箱时序。

实施时先做查询分流、恢复判据和故障原因诊断；再做公平调度、邮箱异步生命周期和幂等停车。最后清理独立观察窗和重复发送入口。保持 CAN1 位时序、自动重发、CAN2、PID、反馈门槛和停车确认不变，便于比较前后行为。

验收需覆盖：查询拥塞不锁存运动故障；REC/历史 PARAM 不阻止有证据的空闲恢复；当前 EPVF/BOFF、API 修复失败和新故障仍阻止解锁；所需每轮反馈持续更新；控制帧失去服务仍停车；STOP 优先且不会被重试撤销；故障期间旧非零目标不会补发；HOST LINK/MODE/连续 STOP 不切断查询链。现场以同一时间段的 CAN STATUS、MOTOR FEEDBACK、CONTROL STATUS 与主机命令日志比较。

已有寄存器附件 ESR=`0x7CE30043` 表示 TEC=227、REC=124、EWGF=1、EPVF=1、BOFF=0，这类快照包含真实控制器错误状态，不能仅用软件锁存解释。它不提供错误起因证据，也不能从旧快照判断本次首先发生了哪条路径。方案的目标是修复已证实的软件误升级、恢复阻断和调度问题，而不是保证 READY 永远为 1。

HAL 行为依据以仓库内 `Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c` 为准，并核对了 [ST 官方 HAL 源码](https://raw.githubusercontent.com/STMicroelectronics/stm32f4xx-hal-driver/master/Src/stm32f4xx_hal_can.c)：Abort 设置撤销请求位、Start/Stop 含等待、HAL ErrorCode 累积。

## 7. 补充：AGE_MS 持续增长说明反馈通道停更

用户补充 AGE_MS 一直增长。这个症状应提高“恢复有效速度反馈”的优先级，不能只修 READY 解锁。

`robot_app.c` 打印 `AGE_MS = HAL_GetTick() - sample.tick`；sample.tick 只由协议层接受有效 0x35 回复后更新。发送查询、查询入队、TXOK、收到 F3/F6 ACK、收到 3A 状态帧，都不会刷新速度反馈时间。因此持续增长且 SEQ 不变表示有效速度回复没有更新，不是普通的反馈频率略低。

另外，VALID=0、SEQ=0 时 tick 从未被更新，AGE_MS 实际近似开机时长，不能解释为“曾收到一帧但已老化这么久”。VALID=1 也仅表示历史上收到过有效反馈，是否新鲜仍要看 AGE。

### 现有日志支持的判断

`llm-pid-tuner-main/logs/can_sessions/20260926_194546_341407.jsonl` 中两次连续快照：

| 字段 | 第一份 | 第二份 |
| --- | --- | --- |
| TX_OK | 17445 | 17445 |
| RX | 17399 | 17399 |
| TX_QUEUED | 57966 | 57978 |
| TX_ABORT | 25795 | 25803 |
| TX_TIMEOUT | 28631 | 28640 |
| motor 1 SEQ | 4322 | 4322 |
| motor 1 AGE_MS | 601752 | 602018 |
| ESR | 0x7ABA0043 | 0x7ABA0043 |

四轮 SEQ 均未变，AGE 约为十分钟，控制器当前 TEC=186、REC=122、EPVF=EWGF=1。这证明该时段发送完成和接收进展停止，软件仍在装入帧、撤销、超时；不只是历史锁存误报，也不只是协议层丢弃新回复。快照不包含停更瞬间，所以尚不能确定首先触发的是哪个软件操作。

另一份 `20260926_200659_092999_probe.jsonl` 中，四轮 VALID=0、SEQ=0，表示该次启动后从未收到有效速度反馈。CONTROL LOOP_MAX_MS=2，且 CAN_ENQUEUE=34611、CAN_DROP=24652、CAN_WAIT_MAX_MS=54；这一快照未显示长主循环阻塞，却显示大量入队与丢弃。CAN STATUS 被 HOST NOT LINKED 拒绝，不能用它补齐当时的 TXOK/RX/ESR。应把只读 CAN STATUS 放在主机接管门禁之前，避免为读取诊断而触发 HOST LINK 的停车和使能副作用。

### 反馈稳定性的实施要求

1. 查询调度独立于运动许可：HOST=NONE、READY=0、反馈过期、停车未确认时仍能查询速度。当前轮询并未直接被 READY 门禁关闭，问题在于下层查询与其他帧共用 FIFO/撤销机制，不能把它描述为“READY=0 时根本不轮询”。
2. 停车发出后，查询必须获得明确的服务机会；停车重试只补发 STOP，不能再次撤销已经在途的查询。没有反馈就无法确认停车，因此只增加停车重试不能完成恢复闭环。
3. 每 10 ms 选择一个待查询的有效地址，四轮目标约 40 ms 一轮。优先服务反馈最旧的轮或用轮转游标；不要积累历史查询。单轮测试跳过未选地址时应继续寻找有效地址，而不是浪费一次查询周期。
4. 分开记录每轮 QUERY_ENQUEUE、QUERY_TXOK、QUERY_REPLY_VALID、REPLY_TIMEOUT、RX_REJECT_REASON、last_valid_tick。查询入队增长不能作为通信在工作的证据；只有 TXOK 增长也不能证明指定电机反馈有效。
5. 为有效回复设置截止时间并有限重试，重试使用退避；持续没有 TXOK/RX 时进入明确的链路恢复状态，而不是维持 50 ms 超时撤销、100 ms 全量停车的循环。不要无条件反复 Stop/Start。
6. 链路修复不能以“已有新鲜反馈”作为启动前提，否则反馈停更时无法修复链路；运动重新放行仍必须要求恢复后的新鲜反馈和停车确认。修复期间禁止非零目标，先请求停车，保留停止请求。链路修复不是证明外部电机已经停下。

将方案的验收目标具体化为：正常情况下每个所需电机 SEQ 持续递增、AGE 周期性回落，四轮完整查询周期约 40 ms；先以 AGE 常态 <100 ms 作为调度目标，再依据实测延迟分布定参数。300 ms 保留为现有失联停车边界，不扩大以掩盖停更。目标延迟不是当前固件或电机的实测保证。

诊断决策应按三段证据分流：

```text
QUERY_TXOK 不增 → 查询调度 / 邮箱生命周期 / 当前控制器状态
QUERY_TXOK 增、RX 不增 → 请求已发出但没有接收回复，检查帧与驱动器响应状态
RX 增、有效 0x35 SEQ 不增 → 地址、包序号、长度、校验、功能码解析拒绝
```

此次补充仅更新分析与方案，未修改生产固件。前文恢复函数候选要求有效反馈，它不会也不应通过清锁存或人为刷新 tick 来解决 AGE 持续增长。
