"""双摄像头联合调试：cam0夹爪，cam1前置导航和二维码。"""

from argparse import ArgumentParser
import os
import time

from robot_control import build_dual_camera_vision
from robot_perception.material.preview import draw_model_detections, model_detection_summary


def parse_arguments():
    parser = ArgumentParser(description="树莓派双摄像头联合调试")
    parser.add_argument(
        "--mode",
        choices=("all", "navigation", "qr", "gripper"),
        default="all",
        help="all=两路同时；navigation/qr=仅启动cam1前置；gripper=仅启动cam0夹爪",
    )
    parser.add_argument(
        "--target-code",
        type=int,
        help="cam0要寻找的已映射正整数物料编号；不填时自动选择",
    )
    parser.add_argument(
        "--qr-every",
        type=int,
        default=5,
        help="all模式下每多少个循环扫描一次二维码，默认5",
    )
    parser.add_argument(
        "--loop-interval",
        type=float,
        default=0.05,
        help="循环最短间隔（秒），默认0.05",
    )
    parser.add_argument(
        "--preview",
        action="store_true",
        help="显示图形窗口；SSH无桌面时不要使用",
    )
    parser.add_argument("--camera-config")
    parser.add_argument("--color-config")
    parser.add_argument("--material-config")
    parser.add_argument("--material-backend", choices=("model", "color"))
    parser.add_argument("--line-config")
    return parser.parse_args()


def _validate_arguments(args) -> None:
    if args.target_code is not None and args.target_code <= 0:
        raise ValueError("--target-code 必须是正整数")
    if args.qr_every < 1:
        raise ValueError("--qr-every 必须大于等于1")
    if args.loop_interval <= 0:
        raise ValueError("--loop-interval 必须大于0")
    if args.preview and not (
        os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
    ):
        raise ValueError("当前没有图形桌面，请去掉 --preview")


def _draw_front(cv2, result):
    frame = result.frame.copy()
    detection = result.line_detection
    decision = result.line_decision
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
        f"cam1 FRONT {decision.command} {decision.error:+.2f}",
        (8, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        2,
    )
    return frame


def _draw_gripper(cv2, result, grip_center):
    frame = result.frame.copy()
    detection = result.material_detection
    draw_model_detections(cv2, frame, detection)
    cv2.drawMarker(
        frame,
        grip_center,
        (255, 255, 255),
        cv2.MARKER_CROSS,
        24,
        2,
    )
    observation = detection.observation
    if observation is not None:
        x, y, width, height = observation.box
        color = (0, 255, 0) if detection.safe_to_pick else (0, 165, 255)
        cv2.rectangle(frame, (x, y), (x + width, y + height), color, 2)
        cv2.line(frame, grip_center, observation.center, color, 2)
    cv2.putText(
        frame,
        f"cam0 GRIPPER {detection.status} PICK={detection.safe_to_pick}",
        (8, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        2,
    )
    return frame


def main() -> int:
    args = parse_arguments()
    try:
        _validate_arguments(args)
        vision = build_dual_camera_vision(
            camera_config_path=args.camera_config,
            color_config_path=args.color_config,
            line_config_path=args.line_config,
            material_config_path=args.material_config,
            material_backend=args.material_backend,
        )
    except Exception as error:
        print(f"双摄像头配置错误：{type(error).__name__}: {error}")
        return 2

    cv2 = None
    if args.preview:
        import cv2 as cv2_module

        cv2 = cv2_module

    camera_1_enabled = args.mode in {"all", "gripper"}
    camera_2_enabled = args.mode in {"all", "navigation", "qr"}
    qr_enabled = args.mode in {"all", "qr"}
    qr_code = None
    frame_index = 0
    last_front_summary = None
    last_gripper_summary = None
    last_print_time = 0.0

    try:
        roles = []
        if camera_2_enabled:
            roles.append("front")
        if camera_1_enabled:
            roles.append("gripper")
        vision.start(roles)
        print("双摄像头已启动：cam0=夹爪，cam1=前置导航/二维码")
        while True:
            loop_started = time.monotonic()
            front_result = None
            gripper_result = None

            if camera_2_enabled:
                if args.mode == "qr":
                    candidate = vision.scan_task_code()
                    if candidate:
                        qr_code = candidate
                else:
                    scan_now = (
                        qr_enabled
                        and qr_code is None
                        and frame_index % args.qr_every == 0
                    )
                    front_result = vision.observe_front(scan_qr=scan_now)
                    if front_result.task_code:
                        qr_code = front_result.task_code

            if camera_1_enabled:
                gripper_result = vision.observe_gripper(args.target_code)

            now = time.monotonic()
            front_summary = None
            if front_result is not None:
                front_summary = (
                    front_result.line_decision.command,
                    round(front_result.line_decision.error, 2),
                    qr_code,
                )
            gripper_summary = None
            if gripper_result is not None:
                detection = gripper_result.material_detection
                observation = detection.observation
                gripper_summary = (
                    detection.status,
                    observation.material_code if observation else None,
                    tuple(round(value, 1) for value in observation.offset_pixels)
                    if observation
                    else None,
                    detection.safe_to_pick,
                    model_detection_summary(detection),
                    detection.message,
                )

            changed = (
                front_summary != last_front_summary
                or gripper_summary != last_gripper_summary
            )
            if changed or now - last_print_time >= 1.0:
                if args.mode == "qr":
                    print(f"cam1 前置二维码={qr_code or '搜索中'}")
                elif front_summary is not None:
                    print(
                        f"cam1 前置巡线={front_summary[0]} "
                        f"误差={front_summary[1]:+.2f} "
                        f"二维码={qr_code or '未确认'}"
                    )
                if gripper_summary is not None:
                    print(
                        f"cam0 夹爪状态={gripper_summary[0]} "
                        f"物料={gripper_summary[1] or '未找到'} "
                        f"偏差={gripper_summary[2]} "
                        f"可抓取={gripper_summary[3]} "
                        f"模型候选={gripper_summary[4]} "
                        f"提示={gripper_summary[5]}"
                    )
                last_front_summary = front_summary
                last_gripper_summary = gripper_summary
                last_print_time = now

            if cv2 is not None:
                if front_result is not None:
                    cv2.imshow("cam1 - Front Navigation", _draw_front(cv2, front_result))
                if gripper_result is not None:
                    center = tuple(
                        int(value)
                        for value in vision.camera_manager.config["gripper"]["grip_center"]
                    )
                    cv2.imshow(
                        "cam0 - Gripper",
                        _draw_gripper(cv2, gripper_result, center),
                    )
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            frame_index += 1
            remaining = args.loop_interval - (time.monotonic() - loop_started)
            if remaining > 0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        print("\n双摄像头调试已停止")
    except Exception as error:
        print(f"双摄像头运行错误：{type(error).__name__}: {error}")
        return 1
    finally:
        vision.close()
        if cv2 is not None:
            cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
