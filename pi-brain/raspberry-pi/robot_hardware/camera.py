"""统一管理树莓派5双摄像头，避免功能模块重复占用设备。"""

import json
from pathlib import Path
import threading
import time
from typing import Any, Mapping, Optional


DEFAULT_CAMERA_CONFIG_PATH = (
    Path(__file__).resolve().parents[1] / "config" / "cameras.json"
)


class CameraError(RuntimeError):
    """摄像头初始化或采集失败。"""


class CameraConfigError(ValueError):
    """双摄像头配置不完整或不合法。"""


def _validate_camera_section(role: str, config: Mapping[str, Any]) -> dict:
    if not isinstance(config, Mapping):
        raise CameraConfigError(f"{role} 摄像头配置必须是JSON对象")
    result = dict(config)

    camera_num = result.get("camera_num")
    if not isinstance(camera_num, int) or isinstance(camera_num, bool) or camera_num < 0:
        raise CameraConfigError(f"{role}.camera_num 必须是大于等于0的整数")

    frame_size = result.get("frame_size")
    if (
        not isinstance(frame_size, list)
        or len(frame_size) != 2
        or any(
            not isinstance(value, int)
            or isinstance(value, bool)
            or value <= 0
            for value in frame_size
        )
    ):
        raise CameraConfigError(f"{role}.frame_size 必须是两个正整数")

    for key in ("fps", "buffer_count"):
        value = result.get(key)
        if not isinstance(value, (int, float)) or isinstance(value, bool) or value <= 0:
            raise CameraConfigError(f"{role}.{key} 必须是大于0的数字")
    if not isinstance(result["buffer_count"], int):
        raise CameraConfigError(f"{role}.buffer_count 必须是整数")

    settle_seconds = result.get("settle_seconds", 0)
    if (
        not isinstance(settle_seconds, (int, float))
        or isinstance(settle_seconds, bool)
        or settle_seconds < 0
    ):
        raise CameraConfigError(f"{role}.settle_seconds 不能小于0")

    model = result.get("model")
    if model is not None and (not isinstance(model, str) or not model.strip()):
        raise CameraConfigError(f"{role}.model 必须是非空字符串或null")

    connector = result.get("connector")
    if not isinstance(connector, str) or not connector:
        raise CameraConfigError(f"{role}.connector 必须是非空字符串")

    if role == "gripper":
        for key in ("grip_center", "alignment_tolerance_pixels"):
            vector = result.get(key)
            if (
                not isinstance(vector, list)
                or len(vector) != 2
                or any(
                    not isinstance(value, (int, float))
                    or isinstance(value, bool)
                    or value < 0
                    for value in vector
                )
            ):
                raise CameraConfigError(f"gripper.{key} 必须是两个非负数字")
        if not isinstance(result.get("require_global_ready", False), bool):
            raise CameraConfigError("gripper.require_global_ready 必须是布尔值")

    logical_name = result.get("logical_name")
    expected_name = "camera_2" if role == "front" else "camera_1"
    if logical_name != expected_name:
        raise CameraConfigError(f"{role}.logical_name 必须是 {expected_name}")

    if role == "front":
        confirmations = result.get("qr_confirmations", 2)
        if (
            not isinstance(confirmations, int)
            or isinstance(confirmations, bool)
            or confirmations < 1
        ):
            raise CameraConfigError("front.qr_confirmations 必须是大于等于1的整数")

    return result


def load_camera_config(path: Optional[str] = None) -> dict:
    """读取树莓派5双摄像头配置，并检查两个设备编号不重复。"""
    config_path = Path(path) if path else DEFAULT_CAMERA_CONFIG_PATH
    try:
        with config_path.open("r", encoding="utf-8") as file:
            config = json.load(file)
    except FileNotFoundError as error:
        raise CameraConfigError(f"找不到摄像头配置：{config_path}") from error
    except json.JSONDecodeError as error:
        raise CameraConfigError(
            f"摄像头配置JSON错误：第{error.lineno}行，第{error.colno}列"
        ) from error

    if not isinstance(config, dict):
        raise CameraConfigError("摄像头配置根节点必须是JSON对象")
    if config.get("platform") != "raspberry_pi_5":
        raise CameraConfigError("platform 必须是 raspberry_pi_5")

    front = _validate_camera_section("front", config.get("front"))
    gripper = _validate_camera_section("gripper", config.get("gripper"))
    if front["camera_num"] == gripper["camera_num"]:
        raise CameraConfigError("前向摄像头和抓取摄像头不能使用同一camera_num")

    return {
        "platform": "raspberry_pi_5",
        "front": front,
        "gripper": gripper,
        "_config_path": str(config_path.resolve()),
    }


class PiCamera:
    """Picamera2 的轻量生命周期封装。

    ``picamera2`` 在 ``start`` 时才导入，因此开发机可以导入其他项目模块，
    而不必安装树莓派专用依赖。
    """

    def __init__(
        self,
        config: Mapping[str, Any],
        device_factory=None,
    ):
        self.config = dict(config)
        self.camera_num = int(self.config.get("camera_num", 0))
        self.role = str(self.config.get("role", "camera"))
        self.model = self.config.get("model")
        self._device_factory = device_factory
        self._device: Optional[Any] = None
        self._started = False
        self._last_frame_time: Optional[float] = None
        self._lock = threading.RLock()

    def start(self) -> None:
        if self._started:
            return
        device_factory = self._device_factory
        if device_factory is None:
            try:
                from picamera2 import Picamera2
            except ImportError as error:
                raise CameraError("当前环境没有安装 picamera2") from error
            device_factory = Picamera2

        frame_size = tuple(self.config.get("frame_size", (320, 240)))
        fps = float(self.config.get("fps", 15))
        buffer_count = int(self.config.get("buffer_count", 3))
        pixel_format = str(self.config.get("format", "RGB888"))
        controls = dict(self.config.get("controls", {}))
        controls.setdefault("FrameRate", fps)
        controls.setdefault("AwbEnable", True)

        device = None
        try:
            device = device_factory(self.camera_num)
            camera_config = device.create_preview_configuration(
                main={"size": frame_size, "format": pixel_format},
                raw=None,
                buffer_count=buffer_count,
                controls=controls,
                display=None,
                encode=None,
                queue=False,
            )
            device.configure(camera_config)
            device.start()
        except Exception as error:
            if device is not None:
                try:
                    device.close()
                except RuntimeError:
                    pass
            raise CameraError(
                f"{self.role}摄像头(camera_num={self.camera_num})启动失败：{error}"
            ) from error

        self._device = device
        self._started = True

    def capture_array(self, stream: str = "main"):
        with self._lock:
            if not self._started or self._device is None:
                raise CameraError(f"{self.role}摄像头尚未启动")
            try:
                frame = self._device.capture_array(stream)
            except Exception as error:
                raise CameraError(f"{self.role}摄像头采集失败：{error}") from error
            self._last_frame_time = time.monotonic()
            return frame

    def capture_metadata(self):
        with self._lock:
            if not self._started or self._device is None:
                raise CameraError(f"{self.role}摄像头尚未启动")
            return self._device.capture_metadata()

    def set_controls(self, controls) -> None:
        with self._lock:
            if not self._started or self._device is None:
                raise CameraError(f"{self.role}摄像头尚未启动")
            self._device.set_controls(controls)

    def is_healthy(self, stale_after_seconds: float = 1.0) -> bool:
        if not self._started:
            return False
        if self._last_frame_time is None:
            return True
        return time.monotonic() - self._last_frame_time <= stale_after_seconds

    def stop(self) -> None:
        with self._lock:
            if self._device is not None and self._started:
                try:
                    self._device.stop()
                finally:
                    self._started = False

    def close(self) -> None:
        with self._lock:
            self.stop()
            if self._device is not None:
                self._device.close()
                self._device = None


class DualCameraManager:
    """树莓派5前向、抓取两路摄像头的唯一生命周期持有者。"""

    ROLES = ("front", "gripper")

    def __init__(self, config: Mapping[str, Any], camera_factory=PiCamera):
        self.config = dict(config)
        self.front = camera_factory(self.config["front"])
        self.gripper = camera_factory(self.config["gripper"])
        self._started = False
        self._started_roles = set()

    def camera(self, role: str) -> PiCamera:
        if role == "front":
            return self.front
        if role == "gripper":
            return self.gripper
        raise KeyError(f"未知摄像头角色：{role}")

    def start(self, roles=None) -> None:
        requested_roles = self.ROLES if roles is None else tuple(roles)
        if not requested_roles:
            raise ValueError("至少要启动一路摄像头")
        unknown_roles = set(requested_roles) - set(self.ROLES)
        if unknown_roles:
            raise KeyError(f"未知摄像头角色：{sorted(unknown_roles)}")
        started = []
        try:
            for role in requested_roles:
                if role in self._started_roles:
                    continue
                camera = self.camera(role)
                camera.start()
                started.append((role, camera))
                self._started_roles.add(role)
        except Exception:
            for _, camera in reversed(started):
                camera.close()
            self._started_roles.difference_update(role for role, _ in started)
            raise
        self._started = bool(self._started_roles)

    def is_healthy(self, stale_after_seconds: float = 1.0, roles=None) -> bool:
        requested_roles = (
            tuple(self._started_roles) if roles is None else tuple(roles)
        )
        return bool(requested_roles) and all(
            role in self._started_roles
            and self.camera(role).is_healthy(stale_after_seconds)
            for role in requested_roles
        )

    def close(self) -> None:
        errors = []
        for role in reversed(self.ROLES):
            try:
                self.camera(role).close()
            except Exception as error:
                errors.append(error)
        self._started = False
        self._started_roles.clear()
        if errors:
            raise CameraError(f"关闭双摄像头时发生{len(errors)}个错误：{errors[0]}")
