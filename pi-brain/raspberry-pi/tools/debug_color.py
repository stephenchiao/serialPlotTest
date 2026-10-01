"""树莓派比赛颜色识别主程序。

算法参数位于 color_config.json，识别算法位于 color_detector.py。
默认无界面运行，并把结果写入配置指定的JSON状态文件。
"""

from pathlib import Path
import argparse
import json
import os
import time

import cv2

from robot_hardware.camera import (
    DEFAULT_CAMERA_CONFIG_PATH,
    CameraConfigError,
    PiCamera,
    load_camera_config,
)
from robot_perception.color.config import ConfigError, DEFAULT_CONFIG_PATH, load_config
from robot_perception.color.detector import (
    CompetitionColorDetector,
    apply_white_balance,
    build_white_balance_luts,
    calibrate_white_balance,
    draw_preview,
    serializable_state,
)


cv2.setNumThreads(1)


def display_is_available():
    """SSH无图形桌面时返回False，防止Qt xcb错误。"""
    return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))


def write_state_file(path, state):
    """先写临时文件再原子替换，避免其他程序读取到半个JSON。"""
    state_path = Path(path)
    state_path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = state_path.with_name(state_path.name + ".tmp")

    with temporary_path.open("w", encoding="utf-8") as file:
        json.dump(
            serializable_state(state),
            file,
            ensure_ascii=False,
            separators=(",", ":"),
        )
    os.replace(temporary_path, state_path)


def make_safe_state(status, config, error=None):
    """生成CALIBRATING、STOPPED或ERROR安全状态。"""
    detection = config["detection"]
    state = {
        "timestamp": round(time.time(), 3),
        "valid_for_seconds": detection["state_valid_seconds"],
        "status": status,
        "safe_to_pick": False,
        "expected_color_count": detection["expected_color_count"],
        "confirmed_color_codes": [],
        "detections": [],
    }
    if error is not None:
        state["error"] = error
    return state


def try_lock_camera_white_balance(picam2, config):
    """锁定硬件自动白平衡；不支持时返回None并继续运行。"""
    white_balance = config["white_balance"]
    if not white_balance["enabled"] or not white_balance["lock_camera_awb"]:
        return None

    try:
        metadata = picam2.capture_metadata()
        camera_gains = metadata.get("ColourGains")
        if camera_gains and len(camera_gains) == 2:
            picam2.set_controls(
                {
                    "AwbEnable": False,
                    "ColourGains": tuple(camera_gains),
                }
            )
            return tuple(camera_gains)
    except (KeyError, RuntimeError, TypeError, ValueError):
        pass
    return None


def parse_arguments():
    parser = argparse.ArgumentParser(description="模块化比赛颜色识别")
    parser.add_argument(
        "--config",
        default=str(DEFAULT_CONFIG_PATH),
        help="JSON参数文件路径",
    )
    parser.add_argument(
        "--camera-config",
        default=str(DEFAULT_CAMERA_CONFIG_PATH),
        help="树莓派5双摄像头配置",
    )
    parser.add_argument(
        "--preview",
        action="store_true",
        help="在树莓派桌面打开调试窗口",
    )
    parser.add_argument(
        "--state-file",
        help="临时覆盖配置中的状态文件路径",
    )
    parser.add_argument(
        "--expected-colors",
        type=int,
        choices=range(1, 7),
        help="临时覆盖预期颜色数量",
    )
    parser.add_argument(
        "--capture-dir",
        help="预览时按S保存无标注校色帧，用于建立评测集",
    )
    return parser.parse_args()


def main():
    args = parse_arguments()

    try:
        config = load_config(args.config)
        camera_config = load_camera_config(args.camera_config)["gripper"]
    except (CameraConfigError, ConfigError) as error:
        print(f"配置错误：{error}")
        return 2

    if args.expected_colors is not None:
        config["detection"]["expected_color_count"] = args.expected_colors

    runtime = config["runtime"]
    state_file = args.state_file or runtime["state_file"]
    preview_enabled = args.preview and display_is_available()
    capture_directory = Path(args.capture_dir) if args.capture_dir else None

    if args.preview and not preview_enabled:
        print("当前没有图形桌面，已自动关闭预览；识别仍会继续运行。")
    if capture_directory is not None and not preview_enabled:
        print("--capture-dir 需要同时使用可用的 --preview，已关闭图片采集。")
        capture_directory = None
    if capture_directory is not None:
        capture_directory.mkdir(parents=True, exist_ok=True)
        print("预览窗口中按S保存无标注画面。")

    picam2 = None
    camera_started = False
    final_state = None
    last_print_time = 0.0
    last_state_write_time = 0.0
    last_summary = None

    try:
        write_state_file(state_file, make_safe_state("CALIBRATING", config))

        picam2 = PiCamera(camera_config)
        picam2.start()
        camera_started = True

        time.sleep(float(camera_config["settle_seconds"]))
        camera_gains = try_lock_camera_white_balance(picam2, config)
        software_gains = calibrate_white_balance(
            picam2,
            config["white_balance"],
        )
        white_balance_luts = build_white_balance_luts(software_gains)
        detector = CompetitionColorDetector(config)

        print(f'已加载配置：{config["_config_path"]}')
        if camera_gains is not None:
            print(
                "硬件白平衡已锁定："
                f"R={camera_gains[0]:.2f}, B={camera_gains[1]:.2f}"
            )
        print(
            "软件白平衡BGR增益："
            f"{software_gains[0]:.2f}, "
            f"{software_gains[1]:.2f}, "
            f"{software_gains[2]:.2f}"
        )
        print("比赛模式已启动；只有READY状态允许后续程序抓取。")

        while True:
            raw_frame = picam2.capture_array("main")
            frame = apply_white_balance(raw_frame, white_balance_luts)
            state, masks = detector.detect(
                frame,
                collect_masks=preview_enabled,
            )
            now = time.monotonic()

            summary = (
                state["status"],
                tuple(state["confirmed_color_codes"]),
            )
            if (
                summary != last_summary
                or now - last_print_time >= runtime["print_interval"]
            ):
                codes = "".join(
                    str(code)
                    for code in state["confirmed_color_codes"]
                )
                print(
                    f'状态={state["status"]} '
                    f'已确认颜色={codes or "无"} '
                    f'可抓取={state["safe_to_pick"]}'
                )
                last_summary = summary
                last_print_time = now

            if (
                now - last_state_write_time
                >= runtime["state_write_interval"]
            ):
                write_state_file(state_file, state)
                last_state_write_time = now

            if preview_enabled:
                clean_frame = frame.copy() if capture_directory is not None else None
                mask_preview = draw_preview(frame, state, masks)
                cv2.imshow("Competition Color Detection", frame)
                cv2.imshow("Color Classification Mask", mask_preview)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                if key in (ord("s"), ord("S")) and clean_frame is not None:
                    timestamp = time.strftime("%Y%m%d_%H%M%S")
                    milliseconds = int((time.time() % 1) * 1000)
                    image_path = capture_directory / (
                        f"frame_{timestamp}_{milliseconds:03d}.jpg"
                    )
                    if cv2.imwrite(str(image_path), clean_frame):
                        print(f"已保存评测图片：{image_path}")
                    else:
                        print(f"保存图片失败：{image_path}")

    except KeyboardInterrupt:
        final_state = make_safe_state("STOPPED", config)
        print("\n程序已停止")
        return 0
    except Exception as error:
        final_state = make_safe_state(
            "ERROR",
            config,
            error=f"{type(error).__name__}: {error}",
        )
        try:
            write_state_file(state_file, final_state)
        except OSError:
            pass
        print(f"识别程序错误：{type(error).__name__}: {error}")
        return 1
    finally:
        if final_state is None:
            final_state = make_safe_state("STOPPED", config)
        try:
            write_state_file(state_file, final_state)
        except OSError:
            pass
        if camera_started and picam2 is not None:
            picam2.stop()
        if picam2 is not None:
            picam2.close()
        if preview_enabled:
            cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
