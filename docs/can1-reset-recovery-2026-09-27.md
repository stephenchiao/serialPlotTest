# CAN1 复位后通信修正与验证（2026-09-27）

用户在原有两批重构后提交新的实机日志，要求继续放宽通信条件并简化流程。已实施本轮修改并完成软件验证和固件构建，尚未由本任务烧录或验证真实电机。

## 新日志说明了什么

附件包含 56 组 CAN 状态采样。第一组四轮 VALID=0、SEQ=0、AGE=19101 ms，TX_OK=0、RX=0。这是启动后尚未收到有效回复，AGE 接近开机时间。

后来 TX_OK/RX 增至 29，再到 30；四轮速度回复计数最终保持 5、3、2、3。最后一组四轮 AGE 为 22164、28297、29584、22168 ms，表明收发曾短暂成功，随后速度回复停止。VALID=1 表示曾有合法数据，不保证目前新鲜；SEQ 固定和 AGE 增长才是这里的关键。

当前附件里，四轮有效数据的最小采样 AGE 分别为 5216、11349、12636、5220 ms，未采到用户描述的约 20 ms 正常阶段；不能将另一时段的通信成功写成这份日志的结果。

| 最后一组字段 | 解读 |
| --- | --- |
| TX_QUEUED=1169，TX_ERR=0 | HAL 接受了请求；不是已经成功发上总线 |
| TX_OK=30，TX_TIMEOUT=1136 | 大量请求没有按时完成；约 97% 的提交数对应超时次数，但两类统计不是严格逐帧配对 |
| RX=30，RX_REJECT=0 | 不是大量回包被长度/校验检查丢弃；更应先恢复持续发送与回包 |
| ERROR/ERR_LATCH=0x800 | HAL 定义为邮箱 0 发送仲裁丢失，不能误写成 ACK 错误或 Bus-Off |
| ESR=0x00210040 | TEC=33、REC=0，LEC=4；本地 HAL 将 LEC=4 解释为 Bit recessive error。LEC 是最后错误记录，不等于每次采样都新发生同一错误 |
| HW_READY=1，READY=0，REASON=0x0C | HAL 处于监听且未 Bus-Off，但提交/发送结果失败与超时造成运动故障锁存 |
| REPAIR=0，STALL_REC=0 | 原修复未覆盖这种“邮箱仍有空位、abort 正常，但一直没有 TXOK”的失联形式 |
| Q_MERGED=6101，QUERY TX_OK=5,3,2,3 | 查询不断合并等待，实际发送完成极少，不能用 POLL_FAIL=0 判断查询通信正常 |

定义依据：`Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_can.h` 中的 `HAL_CAN_ERROR_TX_ALST0`，及对应 HAL 源文件的 LEC 分支。当前只开启 RX 和 TX 通知，ACK_SEEN=0 也不能证明历史上从未出现过其他位错误。

## 修复前的软件复现

给四轮排入 STOP，同时提交 F3 使能和轮询查询；每次模拟真实的邮箱仲裁失败，邮箱释放但不生成 TXOK。

原代码在 240 ms 内得到：`STOP=0x1 QUERY=0x0 F3_ATTEMPTS=0`。失败 STOP 立即回到最高优先队列，固定从 motor 1 扫描，因此查询和使能请求完全没有机会。独立复现输出保存在 `tmp/can-reset-baseline/reproduction.log`。

这能证明软件中的队列饥饿缺陷，但不能证明现场每次失败的唯一原因都是它；日志中的位错误仍需新固件实测。

## 本轮实际改动

1. **STOP 按轮轮换，并允许通信继续。** 每发送一个 STOP，允许配置或查询获得一次服务；未完成 STOP 时不发送速度槽中的运动目标。失败 STOP 最少间隔 100 ms 重试，不立即无限重新占用发送泵。
2. **F3 失败保留重试。** 使能请求失败后保留，并有独立的 100 ms 重试间隔。若已排入更新的同轮使能/失能指令，旧失败请求不再追加，避免新失能后重发旧使能。
3. **增加独立的发送进展监督。** 有持续请求而 500 ms 没有任何 TXOK，就分步修复 CAN1；FREE=2、正常 abort 都不能阻止它。该监督不依赖电机反馈。一次失败的诊断查询之后进入空闲，不会被误判为持续失联。
4. **查询不因队列等待 50 ms 而丢弃。** 每轮每功能仍最多一个待处理查询，并合并重复请求。`Q_EXPIRED` 保留兼容，当前实现保持 0。
5. **放宽非运动传输等待。** 查询、STOP、F3 和配置在途期限由 50 ms 改为 100 ms；查询回复等待由 80 ms 改为 150 ms，退避由 40 ms 改为 20 ms。非零速度的等待与发送期限保留 50 ms，过时运动目标不延后执行。
6. **STOP/F3 的单次发送失败只记诊断、等待重试。** 不再因单次仲裁失败/超时立即丢弃初始化命令并重开全局运动故障。持续失联、Bus-Off、非零速度失败仍进入相应处理。
7. **取消额外恢复观察窗口。** 静止、停车确认、所需轮反馈新鲜、无待发 STOP、控制器可用时，每个所需轮在故障后有一条新的有效速度回复即可解除旧锁存。不再额外等待 500 ms，也不再要求窗口内两条新回复和额外 TXOK 增量。
8. **允许 warning/error-passive 下继续通信。** EWGF/EPVF 不再单独阻断 HardwareReady 或锁存运动故障；当前 BOFF 和阻断类 HAL 状态仍需处理。TEC/REC 及历史 ALST/PARAM 本来就不要求清零。
9. **速度查询加快。** 每 5 ms 查询一个选中轮，四轮名义轮询周期 20 ms；实际 AGE 还取决于发送调度和真实电机回复时间。

有效速度回复检查和反馈时间戳规则保持：只有被协议接受的速度回复更新 SEQ/AGE。状态回复、TXOK、普通 ACK 不刷新速度 AGE。停车仍需要真实零速反馈，反馈新鲜度门槛保持 300 ms。链路恢复成功不会重发旧非零目标，运动需新指令。

## 简化后的通信流程

```text
请求入队 → STOP 轮换/配置/查询有机会发送 → 单帧等待 TX 结果
    ├─ TXOK：记录进展；查询开始等待回复
    ├─ STOP/F3 失败：保留请求，延迟重试
    ├─ 查询失败：退避后接受下一次周期查询
    └─ 持续 500 ms 无 TXOK 或 abort 卡死：分步修复 CAN1

有效速度回复 → 更新 SEQ/AGE
静止 + 停车确认 + 新鲜反馈 + 故障后的新回复 → 直接解除旧锁存
```

修复仍走 Stop/Init、检查并清旧邮箱、Start/通知；HAL 等待阶段保留中断。BOFF 下保留硬件 ABOM，不循环重启。没有更改 CAN1 的 500 kbit/s、EMM/X 帧格式、CAN2 或共享过滤器。

## 新增诊断

- `NO_TX_REPAIR`：因为持续无 TXOK 而进入修复的累计次数。
- `TX_PROGRESS_AT_MS`：最近一次 TXOK 的 MCU 时间；从未 TXOK 时为 0。
- `CAN QUERY ... SUBMITTED=a,b,c,d`：四轮查询被 HAL 接受的次数。配合 QUERY TX_OK、SPEED_REPLY 可区分提交、总线发送完成和有效回复。
- ESR 行新增 `LEC`，仍保持这行是 CAN STATUS 响应最后一行，兼容现有上位机读取边界。

复测时不要只看 READY：SUBMITTED、TX_OK 和 SPEED_REPLY 应连续增长，SEQ 应连续增长，AGE 应反复回落。若一直 SUBMITTED 增长但 TX_OK=0，先看 LEC、NO_TX_REPAIR、REPAIR；有 TX_OK 而无回复，再看 RX、协议与电机端响应。

## 验证结果与固件

- 六套 portable C 回归通过，包含 14 组控制层、14 组 CAN 传输场景。
- 修改前复现 STOP=0x1、QUERY=0；修改后同一故障注入得到 STOP=0xF、QUERY=0xF，F3 有三次尝试。
- 连续两次模拟复位，持续仲裁失败、正常邮箱释放、没有初始反馈：均能触发独立修复并恢复一次 READY，最终四轮 AGE 为 **16、11、6、1 ms**，各轮 SEQ 均持续增长。
- 1 ms 发送泵、5 ms 轮询、20 ms 控制更新、模拟 2 ms 回复，以及反复停车的 30 s 模拟：**最大 AGE=20 ms**，每轮超过 1400 次反馈。
- 600 ms 发送冻结、异步 abort、Stop/Start/通知开启失败、无效回复、时钟回绕等回归通过；修复不会假装产生回复，也不会恢复旧运动。
- 三项 CAN STATUS 路由/响应边界源代码检查通过。
- 相关上位机测试 73 项通过：CAN 诊断 7、串口桥 9、协议 42、状态机 15。没有修改上位机或子模块。
- ARM GCC 13.3 全部 46 单元以 `-Wall -Werror` 编译并链接通过。ELF text=112396、data=516、bss=29716 字节。

本轮最终固件：`tmp/can-reset-fix/firmware/serialPlotTest.elf`。
测试输出：`tmp/can-reset-fix/tests.log`。
修改前源码快照和复现：`tmp/can-reset-baseline/`。
本轮源码副本：`tmp/can-reset-fix/`。

```powershell
python tools/run_c_tests.py --cc E:/mingw64/bin/gcc.exe
python tools/build_firmware.py --toolchain-bin E:/STMCubeIDE/STM32CubeIDE_1.19.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.win32_1.0.0.202411081344/tools/bin --output tmp/can-reset-fix/firmware
```

以上 AGE 与复位恢复结果均来自 HAL 模拟，不能视为真实总线已恢复。烧录本轮固件后，应在电机保持供电时重复复位开发板，连续采集 CAN STATUS、MOTOR FEEDBACK 和 MOTOR STOP STATUS；若失败保留故障前后完整日志。原来的 `tmp/can-wave2/` ELF 是上轮版本，不包含本次修正。
