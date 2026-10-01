"""在树莓派上查看 STM32 转发的 OPS9 位姿和健康状态。"""

import argparse
import json
from pathlib import Path
import time

from robot_hardware.stm32 import SerialLink, Stm32Ops9Receiver


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stm32-config", default="config/stm32.json")
    parser.add_argument("--port", help="覆盖 STM32 端口")
    parser.add_argument("--transport", choices=("usb_cdc", "uart"), help="覆盖通信方式")
    parser.add_argument("--ops9-config", default="config/ops9.json")
    arguments = parser.parse_args()
    stm32 = json.loads(Path(arguments.stm32_config).read_text(encoding="utf-8"))
    if arguments.port:
        stm32["port"] = arguments.port
    if arguments.transport:
        stm32["transport"] = arguments.transport
    ops9 = json.loads(Path(arguments.ops9_config).read_text(encoding="utf-8"))
    link = SerialLink.from_config(stm32)
    receiver = Stm32Ops9Receiver(
        link,
        stale_after_seconds=float(ops9["stale_after_seconds"]),
        minimum_quality=int(ops9["minimum_quality"]),
    )
    try:
        with link:
            receiver.attach()
            print("正在等待 STM32 OPS9 遥测，按 Ctrl+C 退出")
            while True:
                sample = receiver.latest_sample()
                usable = receiver.latest()
                if sample is None:
                    print("尚未收到 OPS9 帧", end="\r", flush=True)
                else:
                    pose = sample.pose
                    age_ms = (time.monotonic() - sample.received_at) * 1000.0
                    print(
                        f"x={pose.x_mm:6d} mm  y={pose.y_mm:6d} mm  "
                        f"yaw={pose.yaw_mrad / 1000.0:7.3f} rad  "
                        f"quality={pose.quality if pose.quality is not None else '未上报'}  "
                        f"status=0x{int(pose.status):02X}  "
                        f"age={age_ms:6.1f} ms  usable={usable is not None}",
                        end="\r",
                        flush=True,
                    )
                time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n已停止")
    finally:
        receiver.detach()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
