# 树莓派程序目录

现有 `app/` 是队友维护的程序。本 PR 将本地已修改的树莓派程序及运行所需依赖代码、
配置、测试和文档作为独立项目保存在 `raspberry-pi/`，不覆盖现有 app/config/tests/tools，
也不修改 STM32 的 Core、Drivers、CubeMX 或固件接口。

## 使用新提交的 USB CDC 程序

从仓库根目录运行：

```bash
cd pi-brain/raspberry-pi
sudo apt install python3-serial
python3 -m tools.stm32_link_test --list-ports
python3 -m tools.stm32_link_test --port /dev/ttyACM0 --count 10
```

默认探测不会使能新的运动会话；若发现遗留活动会话，会先发送 STOP_ALL 并等待看门狗恢复。
完整握手可能使能电机，先架空车轮并准备急停：

```bash
python3 -m tools.stm32_link_test --port /dev/ttyACM0 --handshake --count 50
```

新程序使用队友现有 v2 二进制协议和 v4 ASCII 握手；队友仍需将固件底层从 UART
改为原生 USB Device CDC。USB CDC 识别为 ttyACM 并不代表应用协议已经就绪。
正式运行前填写 `raspberry-pi/config/stm32.json` 中的实际设备路径，
并完成地图、OPS9、相机和道路现场标定。

详细接口约定：[USB CDC v2 通信说明](raspberry-pi/robot_hardware/stm32/README.md)。
本地原工程旧版 `stm32_firmware/` 不在本 PR 中，不要拿旧 v1 C 示例与 v2 Python 接口混用。

## 测试和提交范围

```bash
cd pi-brain/raspberry-pi
python3 -m unittest discover -s tests -v
```

本次本地 81 项无硬件测试通过；请求帧、原子位姿目标、状态和遥测布局与队友压缩包
编码器逐字节一致。尚未进行真实树莓派—STM32 USB 硬件联调，不表示整车已可运行。
这是一套独立入口，原 `app/main.py` 不会自动调用它；不要同时启动两套程序占用同一 CDC。

包括本地已修改的 USB 通信、导航、双摄像头接线配置及相应文档/测试，
以及使它们可独立运行的源代码和部署脚本。没有提交模型权重、二维码图片、
第三方下载缓存、训练输出和本机编辑器配置。
