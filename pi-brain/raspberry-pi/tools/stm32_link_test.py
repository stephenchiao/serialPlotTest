"""测试树莓派到 STM32 的 USB CDC / USART1 v2 协议。"""

import argparse
import json
from pathlib import Path
import time

from robot_hardware.stm32 import Command, SerialLink, SerialLinkError
from robot_hardware.stm32.messages import SessionInfo


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="config/stm32.json")
    parser.add_argument("--port", help="覆盖配置端口，例如 /dev/ttyACM0 或 /dev/ttyUSB0")
    parser.add_argument("--transport", choices=("usb_cdc", "uart"), help="覆盖通信方式")
    parser.add_argument("--baudrate", type=int, help="串口波特率，默认 115200；CDC 为逻辑参数")
    parser.add_argument("--list-ports", action="store_true", help="列出串口设备后退出")
    parser.add_argument("--count", type=int, default=10, help="连续探测次数，默认 10")
    actions = parser.add_mutually_exclusive_group()
    actions.add_argument("--handshake", action="store_true", help="进入 RPI 二进制会话；可能使能底盘，请架空车轮")
    actions.add_argument("--stop", action="store_true", help="探测并发送 STOP_ALL，不使能新的底盘会话")
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if args.count <= 0:
        raise SystemExit("--count 必须大于 0")
    try:
        if args.list_ports:
            from serial.tools import list_ports
            ports = list(list_ports.comports())
            for port in ports:
                print(f"{port.device}: {port.description} [{port.hwid}]")
            if not ports:
                print("未发现串口：检查数据线及 CDC 固件或 USB 转串口设备")
            return 0
        config = json.loads(Path(args.config).read_text(encoding="utf-8"))
        if args.port:
            config["port"] = args.port
        if args.transport:
            config["transport"] = args.transport
        if args.baudrate:
            config["baudrate"] = args.baudrate
        if "替换" in config["port"]:
            raise SerialLinkError("请先设置配置中的 port，或指定 --port /dev/ttyACM0（CDC）或 /dev/ttyUSB0（UART）")
        with SerialLink.from_config(config, negotiate=args.handshake) as link:
            print(f"STM32 {link.transport} 已打开：{link.port}")
            print(f"VERSION={link.session_info.version} CAPS=0x{link.session_info.capabilities:08X}")
            if link.startup_info is not None:
                startup = link.startup_info
                print(f"启动自检：WORK 空闲，OPS9={startup.ops_frames} 帧，CAN STATE={startup.can_state} ESR=0x{startup.can_esr:08X}")
                print(f"PID：X={startup.pid_x} Y={startup.pid_y} YAW={startup.pid_yaw}")
            if args.stop:
                link.request(Command.STOP_ALL)
                print("STOP_ALL 已确认")
            else:
                for index in range(args.count):
                    if args.handshake:
                        latency = link.ping(timeout=float(config.get("command_timeout_seconds", 0.5)))
                        print(f"PING {index + 1}/{args.count}: OK，RTT={latency * 1000:.2f} ms")
                    else:
                        response = link.request(Command.SESSION_PROBE)
                        info = SessionInfo.decode(response.data)
                        link.validate_session(info)
                        if info.active or info.armed:
                            raise SerialLinkError("探测期间会话被其他程序占用；请关闭其他串口程序")
                        print(f"PROBE {index + 1}/{args.count}: OK，active={info.active} armed={info.armed}")
                    time.sleep(0.1)
                if args.handshake:
                    print("PASS：STM32 双向通信、启动自检、v2 握手和二进制 PING 均通过；未发送运动目标")
                else:
                    print("PASS：STM32 双向通信及 v2 协议探测通过；未使能新的运动会话")
            print(f"统计：{link.statistics()}")
            if args.handshake:
                link.request(Command.STOP_ALL)
        return 0
    except (SerialLinkError, OSError, ValueError, ImportError) as error:
        print(f"FAIL：{error}")
        return 1
    except KeyboardInterrupt:
        print("\n已退出；STM32 必须通过独立看门狗停车")
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
