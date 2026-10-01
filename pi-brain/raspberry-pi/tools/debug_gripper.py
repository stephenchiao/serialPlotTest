"""cam0 模型/颜色识别、中心偏差与对准调试；不控制夹爪。"""

from argparse import ArgumentParser
import os
import time

from robot_hardware.camera import (
    DEFAULT_CAMERA_CONFIG_PATH,
    CameraConfigError,
    PiCamera,
    load_camera_config,
)
from robot_perception.material import build_material_pipeline
from robot_perception.material.preview import draw_model_detections, model_detection_summary


def parse_arguments():
    parser = ArgumentParser(description="cam0 模型/颜色物料识别和对准调试")
    parser.add_argument("--camera-config", default=str(DEFAULT_CAMERA_CONFIG_PATH))
    parser.add_argument("--color-config")
    parser.add_argument("--material-config", help="模型路径、类别映射与确认配置")
    parser.add_argument("--material-backend", choices=("model", "color"), help="覆盖配置后端")
    parser.add_argument(
        "--target-code",
        type=int,
        help="只跟踪已映射的正整数物料编号；不填时自动选择",
    )
    parser.add_argument("--no-preview", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()
    try:
        import cv2

        if args.target_code is not None and args.target_code <= 0:
            raise ValueError("--target-code 必须是正整数")
        camera_config = load_camera_config(args.camera_config)["gripper"]
        pipeline = build_material_pipeline(
            camera_config, material_config_path=args.material_config,
            color_config_path=args.color_config, backend=args.material_backend,
        )
        camera = PiCamera(camera_config)
        detector = pipeline.detector
    except (CameraConfigError, ValueError, ImportError) as error:
        print(f"抓取视觉配置错误：{error}")
        return 2

    preview_enabled = not args.no_preview and bool(
        os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
    )
    if not args.no_preview and not preview_enabled:
        print("当前没有图形桌面，已关闭预览窗口。")

    cv2.setNumThreads(1)
    last_summary = None
    last_print_time = 0.0
    try:
        camera.start()
        time.sleep(float(camera_config.get("settle_seconds", 1.0)))
        transform = (
            pipeline.calibrator(camera)
            if pipeline.calibrator is not None else (lambda frame: frame)
        )
        grip_x, grip_y = (int(value) for value in camera_config["grip_center"])

        while True:
            raw_frame = camera.capture_array("main")
            frame = transform(raw_frame)
            result = detector.detect(
                frame,
                target_material_code=args.target_code,
            )
            observation = result.observation
            summary = (
                result.status,
                observation.material_code if observation else None,
                tuple(round(value, 1) for value in observation.offset_pixels)
                if observation
                else None,
                result.message,
                model_detection_summary(result),
            )
            now = time.monotonic()
            if summary != last_summary or now - last_print_time >= 1.0:
                if observation is None:
                    print(
                        f"状态={result.status} 目标={args.target_code or '自动'} "
                        f"提示={result.message or '未找到物料'} "
                        f"模型候选={summary[-1]} 可抓取=False"
                    )
                else:
                    print(
                        f"状态={result.status} 编号={observation.material_code} "
                        f"名称={observation.material_name} "
                        f"模型类别={observation.class_id} "
                        f"置信度={observation.confidence:.2f} "
                        f"偏差=({observation.offset_pixels[0]:+.1f},"
                        f"{observation.offset_pixels[1]:+.1f}) "
                        f"已对准={result.aligned} 可抓取={result.safe_to_pick}"
                    )
                last_summary = summary
                last_print_time = now

            if preview_enabled:
                draw_model_detections(cv2, frame, result)
                cv2.drawMarker(
                    frame,
                    (grip_x, grip_y),
                    (255, 255, 255),
                    cv2.MARKER_CROSS,
                    24,
                    2,
                )
                if observation is not None:
                    x, y, width, height = observation.box
                    color = (0, 255, 0) if result.safe_to_pick else (0, 165, 255)
                    cv2.rectangle(frame, (x, y), (x + width, y + height), color, 2)
                    cv2.line(frame, (grip_x, grip_y), observation.center, color, 2)
                    cv2.putText(
                        frame,
                        f"material={observation.material_code} conf={observation.confidence:.2f}",
                        (x, max(18, y - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        color,
                        2,
                    )
                cv2.putText(
                    frame,
                    f"{result.status} PICK={result.safe_to_pick}",
                    (8, 24),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (255, 255, 255),
                    2,
                )
                cv2.imshow("Gripper Material Alignment", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    except KeyboardInterrupt:
        print("\n抓取视觉调试已停止")
    except Exception as error:
        print(f"抓取视觉错误：{type(error).__name__}: {error}")
        return 1
    finally:
        camera.close()
        if preview_enabled:
            cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
