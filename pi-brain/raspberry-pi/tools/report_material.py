"""cam0 物料类别/位置 → STM32；STM32 负责纠偏、抓取及动作安全。"""

import argparse
import json
import logging
from pathlib import Path
import signal
from threading import Event

from robot_control.dual_camera_vision import DualCameraVisionController
from robot_hardware.camera import DualCameraManager, load_camera_config
from robot_hardware.stm32 import SerialLink, Stm32MaterialVisionPublisher
from robot_hardware.stm32.messages import MATERIAL_VISION_CAPABILITY
from robot_perception.material import build_material_pipeline


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stm32-config", default=str(PROJECT_ROOT / "config/stm32.json"))
    parser.add_argument("--report-config", default=str(PROJECT_ROOT / "config/material_serial.json"))
    parser.add_argument("--camera-config")
    parser.add_argument("--material-config")
    parser.add_argument("--color-config")
    parser.add_argument("--material-backend", choices=("model", "color"))
    parser.add_argument("--port", help="覆盖 STM32 USB CDC 端口")
    parser.add_argument("--target-code", type=int, help="1~65535；不填则自动选择已映射目标")
    parser.add_argument(
        "--enable-stm32-session", action="store_true",
        help="确认允许完整握手（可能使能底盘）；先架空车轮并准备急停",
    )
    args = parser.parse_args(argv)
    if not args.enable_stm32_session:
        parser.error("实机上报需要 --enable-stm32-session；完整握手可能使能底盘")
    if args.target_code is not None and not 1 <= args.target_code <= 0xFFFF:
        parser.error("--target-code 必须为 1~65535")
    return args


def main(argv=None) -> int:
    args = parse_arguments(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    link = None
    vision = None
    stop_event = Event()
    old_handlers = {}
    try:
        stm32_config = json.loads(Path(args.stm32_config).read_text(encoding="utf-8"))
        report_config = json.loads(Path(args.report_config).read_text(encoding="utf-8"))
        if args.port:
            stm32_config["port"] = args.port
        camera_config = load_camera_config(args.camera_config)
        if camera_config["gripper"]["camera_num"] != 0:
            raise ValueError("本上报协议固定使用夹爪 cam0，请检查 cameras.json")
        pipeline = build_material_pipeline(
            camera_config["gripper"], material_config_path=args.material_config,
            color_config_path=args.color_config, backend=args.material_backend,
        )
        # 首次只读 SESSION_PROBE 即检查扩展能力；旧固件不进入使能握手。
        link = SerialLink.from_config(stm32_config, additional_capabilities=MATERIAL_VISION_CAPABILITY)
        reporter = Stm32MaterialVisionPublisher.from_config(link, report_config)
        # 独立调试入口只启动 cam0；正式运行应复用已存在的相机管理器与 SerialLink。
        vision = DualCameraVisionController(
            DualCameraManager(camera_config), None, None, None, pipeline.detector,
            gripper_calibrator=pipeline.calibrator, material_reporter=reporter,
            settle_seconds=float(camera_config["gripper"].get("settle_seconds", 0)),
        )
        for signum in (signal.SIGINT, signal.SIGTERM):
            old_handlers[signum] = signal.signal(signum, lambda *_: stop_event.set())
        # 先验证相机可启动，再开启可能使能底盘的 STM32 会话。
        vision.start(("gripper",))
        link.open()
        if not link.session_info.capabilities & MATERIAL_VISION_CAPABILITY:
            raise ValueError("固件没有 0x40 物料接收能力；请先按 docs/MATERIAL_STM32.md 接入")
        print("cam0 上报已启动：STM32 接收位置后自行纠偏/抓取，按 Ctrl+C 退出。")
        last_summary = None
        while not stop_event.is_set():
            result = vision.observe_gripper(args.target_code).material_detection
            packet = reporter.latest_packet
            summary = (result.status, packet.material_code if packet else 0)
            if summary != last_summary:
                print(f"视觉状态={result.status} 上报编号={summary[1]} 提示={result.message or '-'}")
                last_summary = summary
            stop_event.wait(0.01)
        return 0
    except KeyboardInterrupt:
        return 0
    except Exception as error:
        logging.error("物料上报停止：%s: %s", type(error).__name__, error)
        return 1
    finally:
        try:
            if vision is not None:
                vision.close()  # 先上报 STOPPED，再释放相机
        finally:
            if link is not None:
                link.close()
            for signum, handler in old_handlers.items():
                signal.signal(signum, handler)


if __name__ == "__main__":
    raise SystemExit(main())
