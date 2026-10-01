"""巡线调试入口；正式状态机不得直接导入本文件。"""

from argparse import ArgumentParser
import json
from pathlib import Path
import time

import cv2

from robot_control.line_navigation import LineFollower
from robot_hardware.camera import (
    DEFAULT_CAMERA_CONFIG_PATH,
    CameraConfigError,
    PiCamera,
    load_camera_config,
)
from robot_perception.line import LineDetector


DEFAULT_CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "line.json"


def load_config(path):
    with Path(path).open("r", encoding="utf-8") as file:
        config = json.load(file)
    if not isinstance(config, dict):
        raise ValueError("巡线配置根节点必须是JSON对象")
    return config


def parse_arguments():
    parser = ArgumentParser(description="树莓派巡线调试")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG_PATH))
    parser.add_argument(
        "--camera-config",
        default=str(DEFAULT_CAMERA_CONFIG_PATH),
        help="树莓派5双摄像头配置",
    )
    parser.add_argument("--no-preview", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()
    try:
        config = load_config(args.config)
        camera_config = load_camera_config(args.camera_config)["front"]
        camera = PiCamera(camera_config)
        detector = LineDetector(config["detection"])
        follower = LineFollower(
            turn_threshold=config["control"]["turn_threshold"],
            smoothing=config["control"]["error_smoothing"],
        )
    except (
        CameraConfigError,
        KeyError,
        OSError,
        ValueError,
        json.JSONDecodeError,
    ) as error:
        print(f"巡线配置错误：{error}")
        return 2

    cv2.setNumThreads(1)
    last_command = None
    try:
        camera.start()
        time.sleep(float(camera_config.get("settle_seconds", 1.0)))
        while True:
            frame = camera.capture_array("main")
            detection = detector.detect(frame)
            decision = follower.update(detection.error)

            if decision.command != last_command:
                print(f"{decision.command:8s} error={decision.error:+.2f}")
                last_command = decision.command

            if not args.no_preview:
                cv2.rectangle(
                    frame,
                    (0, detection.roi_top),
                    (frame.shape[1] - 1, frame.shape[0] - 1),
                    (0, 255, 255),
                    1,
                )
                if detection.center is not None:
                    cv2.circle(frame, detection.center, 6, (0, 0, 255), -1)
                cv2.putText(
                    frame,
                    f"{decision.command} error={decision.error:+.2f}",
                    (8, 24),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (255, 255, 255),
                    2,
                )
                cv2.imshow("Line Follow", frame)
                cv2.imshow("Line Mask", detection.mask)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    except KeyboardInterrupt:
        print("\n程序已停止")
    except Exception as error:
        print(f"巡线程序错误：{type(error).__name__}: {error}")
        return 1
    finally:
        camera.close()
        if not args.no_preview:
            cv2.destroyAllWindows()
        print("STOP     error=+0.00")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
