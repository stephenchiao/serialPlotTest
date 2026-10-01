# CAN1 持续无发送进展：离线修复记录

## 结论与边界

当前源码的 MOTOR RUN 准入经 ChassisSafety_CanReady 调用 ZDT_CAN_IsReady，检查控制器和发送故障锁存，不检查 RX 数量、轮速新鲜度或电机应用层应答。ZDT_CAN_RecoverWhenIdle 要求空闲、控制器可用、无在途帧、请求范围的停车帧均有 TXOK；同样不要求 RX。因此关闭持续状态反馈本身不会造成本次 CAN NOT READY。

CAN 底层 ACK/TXOK 与电机返回的速度、状态、命令应答帧不同。关闭应用层回复不等于关闭 CAN ACK；反过来，TXOK 也不能证明目标电机执行了命令或已经物理停止。

用户日志：TX_OK=2、TX_QUEUED=223、TX_ABORT=219、TX_TIMEOUT=220，后来超时增到 443；TX_FAULT=1、STALL_REC=0。HAL 的 0x800 是邮箱 0 仲裁丢失历史位。ESR=0x00080040 表示 TEC=8、REC=0、LEC=4（最后记录的 Bit recessive error），当前无 BOFF/EPVF/EWGF。现有通知只启用 TX/RX，ACK_SEEN=0 不能作为从未发生 ACK 错误的证据。

这些证据支持发送持续失败，而不是等待电机状态回复。最初的总线失败原因仍未实机确认。寄存器定义可参照项目内 STM32 HAL 和 ST RM0090：
https://www.st.com/resource/en/reference_manual/dm00031020-stm32f405-407-415-417-437-455-469-application-note-stmicroelectronics.pdf

## 修复内容

原代码只有单个邮箱长期撤销不掉，或 HAL 不可用时才重启。每 50 ms 超时、成功撤销后换下一帧，会不断重置在途计时，永远到不了单帧 500 ms 重启门槛。

- 新增跨帧发送进展监督：持续存在待发或在途任务且 500 ms 没有新 TXOK，申请 CAN1 分步恢复；正常重试、换轮、入队不刷新监督时间。
- 无任务或有新 TXOK 时重新计时；单次查询失败后的空闲、长期空闲后的第一帧不触发无进展恢复。
- 恢复前锁存故障、丢弃旧速度、申请撤销旧帧；恢复后仍须停车发送成功且上层空闲才解锁，不重放旧运动。
- 保留 Bus-Off 的 ABOM 路径、已有单在途传输和 50 ms 运动期限。没有修改 CAN2、共享过滤器、位时序或电机参数。
- 首次重启不受初值为 0 的上次重启时间限制，避免计时回绕处推迟首次恢复。
- CAN STATUS 增加 NO_TX_REPAIR（无发送进展触发次数）与 LEC；ESR 行仍是最后一行。

这里修复的是软件恢复漏判，不保证 Stop/Start 能消除一切真实总线故障。原有撤销卡死恢复路径并未在硬件验证。

## 验证与交付

在修复前加入故障注入测试，正常撤销、持续无 TXOK 的场景在 `stats.stall_recoveries >= 1U` 断言失败；修复后通过。新增场景覆盖持续超时、无 RX 恢复、不重放旧速度、单次查询失败后空闲、正常持续发送、长时间空闲以及计时回绕。

5 套 portable C 测试通过，其中控制层为 3 组 CAN 与 6 组其他控制测试。ARM GCC 13.3 编译全部 46 单元，以 -Wall -Werror 编译并链接成功。

```powershell
python tools/run_c_tests.py --cc E:/mingw64/bin/gcc.exe
python tools/build_firmware.py --toolchain-bin E:/STMCubeIDE/STM32CubeIDE_1.19.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.win32_1.0.0.202411081344/tools/bin --output tmp/can-no-progress/firmware
```

固件：`tmp/can-no-progress/firmware/serialPlotTest.elf`；编译记录：同目录 `build.log`。

本次尝试打开 COM3 返回拒绝访问，没有成功连接或取得新增板端数据。随后用户明确要求暂不调试，已停止实机操作。本次没有烧录、没有发送运动命令，电机恢复转动尚未验证。

## 后续授权实机测试（21:39–21:43）

用户随后明确允许烧录并测试前进。使用 STM32CubeProgrammer 2.20.0、ST-LINK `37FF71064E57343676CD1A43` 烧录上述 ELF，下载校验成功并复位。烧录记录为 `tmp/can-no-progress/flash.log`。用户关闭占用 COM3 的串口助手后，成功以 115200 连接。

完整串口日志：`logs/codex_serial/20260928_213932/raw.log`。命令按回复逐条下发；开始时连续写入多条命令导致 HOST LINK 未被处理，已重新单独发送并收到 OK。

| 时间 | TX_OK | RX | TX_TIMEOUT | STALL_REC / NO_TX_REPAIR | READY | TEC / LEC |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| 21:40:07 | 3 | 2 | 292 | 80 | 0 | 40 / 4 |
| 21:40:37 | 16 | 12 | 760 | 136 | 0 | 91 / 4 |
| 21:42:03 | 16 | 12 | 2300 | 307 | 0 | 91 / 4 |
| 21:43:26 | 16 | 12 | 3777 | 471 | 0 | 99 / 4 |

期间曾出现两次 CAN RECOVERED，随后再次失去发送进展。最后 EWGF=1、EPVF=0、BOFF=0。软件恢复漏判已覆盖，但实机故障没有解决；Stop/Start 成功次数不代表总线恢复成功。

OPS LINK=OK、FRAME_AGE=1 ms。21:41:10 发送 `MOVE FWD 0.03 500`，开发板返回 `# ERROR DEBUG CHASSIS SAFETY CAN=0 OPS=1`，未进入运动分支。随后及结束前均发送 STOP，并收到 `# STOP MODE=TUNE`。最后 MOTOR STOP 为 UNCONFIRMED、SENT_MASK=0x00，不能把收到主控 STOP 回复写成电机已执行停车。21:43:28 关闭串口，退出时再发送一次 STOP。

读取的 CAN1 BTR=0x001A0005，与当前 42 MHz / [6 × (1+11+2)] = 500 kbit/s 配置一致；PB8/PB9 为 AF9。非同时的引脚快照有高低变化，不足以判断线路开路、接触不良或波形质量。现场用户确认供电正常、未改接线，未在本轮取得电机菜单设置的独立验证。

下一步需要检查物理链路或采集波形：断电检查接头、CAN_RX/CAN_TX 跳线、CANH/CANL 与信号地；按实际拓扑核查终端电阻。保持正确终端的前提下，用短线与单台电机隔离定位，再逐台接回。现有证据尚不能把根因确定为某一根线或某个驱动器；不应通过取消 TXOK 判据来绕过故障。
