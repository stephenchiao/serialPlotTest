"""摄像头1抓取视觉与摄像头2前向视觉的统一协调层。"""

from dataclasses import dataclass
import json
from pathlib import Path
import time
from typing import Any, Callable, Optional


DEFAULT_LINE_CONFIG_PATH = (
    Path(__file__).resolve().parents[1] / "config" / "line.json"
)


@dataclass(frozen=True)
class FrontVisionResult:
    """摄像头2的一帧导航结果。"""

    frame: Any
    line_detection: Any
    line_decision: Any
    task_code: Optional[str]


@dataclass(frozen=True)
class NavigationVisionResult:
    """前摄像头单帧产生的导航感知结果。"""

    frame: Any
    obstacle_candidates: tuple[Any, ...]
    confirmed_obstacles: tuple[Any, ...]
    road_observation: Optional[Any] = None


@dataclass(frozen=True)
class GripperVisionResult:
    """摄像头1的一帧抓取结果。"""

    frame: Any
    material_detection: Any


class DualCameraVisionController:
    """让两路相机各司其职，并保证相机只被一个管理器持有。

    逻辑名称与用户约定一致：

    - 摄像头1：``manager.gripper``，模型/颜色识别物料并计算夹爪对准偏差；
    - 摄像头2：``manager.front``，巡线，并在需要时扫描任务二维码。

    Picamera2 的 ``camera_num`` 是设备枚举编号，不等同于上述逻辑名称。
    """

    def __init__(
        self,
        camera_manager,
        line_detector,
        line_follower,
        task_code_reader,
        material_detector,
        *,
        obstacle_source=None,
        road_detector=None,
        gripper_frame_transform: Optional[Callable[[Any], Any]] = None,
        gripper_calibrator: Optional[Callable[[Any], Callable[[Any], Any]]] = None,
        material_reporter=None,
        settle_seconds: float = 0.0,
    ):
        if settle_seconds < 0:
            raise ValueError("settle_seconds 不能小于0")
        self.camera_manager = camera_manager
        self.line_detector = line_detector
        self.line_follower = line_follower
        self.task_code_reader = task_code_reader
        self.material_detector = material_detector
        self.obstacle_source = obstacle_source
        self.road_detector = road_detector
        self.latest_navigation_result: Optional[NavigationVisionResult] = None
        self.gripper_frame_transform = gripper_frame_transform or (lambda frame: frame)
        self.gripper_calibrator = gripper_calibrator
        self.material_reporter = material_reporter
        self.settle_seconds = float(settle_seconds)
        self.started = False
        self.active_roles = frozenset()

    @property
    def camera_1(self):
        """摄像头1：夹爪/机械臂相机。"""
        return self.camera_manager.gripper

    @property
    def camera_2(self):
        """摄像头2：车头固定导航相机。"""
        return self.camera_manager.front

    def start(self, roles=("front", "gripper")) -> None:
        requested_roles = frozenset(roles)
        if not requested_roles or not requested_roles <= {"front", "gripper"}:
            raise ValueError("roles 只能包含 front、gripper，且不能为空")
        if self.started and requested_roles == self.active_roles:
            return
        if self.started:
            raise RuntimeError("视觉系统已启动，不能在运行中更换摄像头角色")
        self.camera_manager.start(requested_roles)
        try:
            if self.settle_seconds:
                time.sleep(self.settle_seconds)
            if "gripper" in requested_roles and self.gripper_calibrator is not None:
                self.gripper_frame_transform = self.gripper_calibrator(self.camera_1)
        except Exception:
            self.camera_manager.close()
            raise
        self.started = True
        self.active_roles = requested_roles

    def observe_front(self, *, scan_qr: bool = False) -> FrontVisionResult:
        """摄像头2采集一次，同时生成巡线结果和可选二维码结果。"""
        self._require_role("front")
        frame = self.camera_2.capture_array("main")
        line_detection = self.line_detector.detect(frame)
        line_decision = self.line_follower.update(line_detection.error)
        task_code = (
            self.task_code_reader.update_frame(frame) if scan_qr else None
        )
        return FrontVisionResult(
            frame=frame,
            line_detection=line_detection,
            line_decision=line_decision,
            task_code=task_code,
        )

    def scan_task_code(self) -> Optional[str]:
        """只用摄像头2扫描二维码，不额外运行巡线算法。"""
        self._require_role("front")
        frame = self.camera_2.capture_array("main")
        return self.task_code_reader.update_frame(frame)

    def observe_navigation(
        self,
        map_pose,
        *,
        observed_at: Optional[float] = None,
    ) -> NavigationVisionResult:
        """只采集一帧，同时更新道路安全和障碍物，不运行旧黑线巡线。"""

        self._require_role("front")
        if self.obstacle_source is None:
            raise RuntimeError("前视障碍检测尚未配置或标定")
        timestamp = time.monotonic() if observed_at is None else observed_at
        frame = self.camera_2.capture_array("main")
        road_observation = (
            self.road_detector.detect(frame, observed_at=timestamp)
            if self.road_detector is not None
            else None
        )
        self.obstacle_source.update(frame, map_pose, observed_at=timestamp)
        result = NavigationVisionResult(
            frame=frame,
            obstacle_candidates=self.obstacle_source.candidates(),
            confirmed_obstacles=self.obstacle_source.obstacles(),
            road_observation=road_observation,
        )
        self.latest_navigation_result = result
        return result

    def observe_gripper(
        self,
        target_material_code: Optional[int] = None,
        *,
        collect_masks: bool = False,
    ) -> GripperVisionResult:
        """cam0 输出视觉结果；配置 reporter 时将类别/像素位置交给 STM32。"""
        self._require_role("gripper")
        captured_at = time.monotonic()
        try:
            raw_frame = self.camera_1.capture_array("main")
            frame = self.gripper_frame_transform(raw_frame)
            detection = self.material_detector.detect(
                frame,
                target_material_code=target_material_code,
                collect_masks=collect_masks,
            )
        except Exception:
            if self.material_reporter is not None:
                self.material_reporter.report_camera_error()
            raise
        if self.material_reporter is not None:
            height, width = frame.shape[:2]
            self.material_reporter.publish(
                detection, frame_size=(width, height), captured_at=captured_at,
            )
        return GripperVisionResult(frame=frame, material_detection=detection)

    def is_healthy(self, stale_after_seconds: float = 1.0, *, roles=None) -> bool:
        return self.started and self.camera_manager.is_healthy(
            stale_after_seconds,
            roles=self.active_roles if roles is None else roles,
        )

    def close(self) -> None:
        try:
            if self.material_reporter is not None and "gripper" in self.active_roles:
                self.material_reporter.stop()
        finally:
            self.camera_manager.close()
            self.started = False
            self.active_roles = frozenset()

    shutdown = close

    def _require_role(self, role: str) -> None:
        if not self.started:
            raise RuntimeError("双摄像头视觉系统尚未启动")
        if role not in self.active_roles:
            raise RuntimeError(f"{role}摄像头未启动")


def build_dual_camera_vision(
    camera_config_path: Optional[str] = None,
    color_config_path: Optional[str] = None,
    line_config_path: Optional[str] = None,
    obstacle_config_path: Optional[str] = None,
    road_config_path: Optional[str] = None,
    enable_navigation_perception: bool = False,
    material_config_path: Optional[str] = None,
    material_backend: Optional[str] = None,
    material_reporter=None,
) -> DualCameraVisionController:
    """读取项目配置并创建树莓派可用的双摄像头视觉系统。

    OpenCV 和 Picamera2 相关模块在这里延迟加载，使其他项目模块仍能在没有
    树莓派摄像头依赖的开发机上导入和测试。
    """
    import cv2

    from robot_hardware.camera import DualCameraManager, load_camera_config
    from robot_control.line_navigation import LineFollower
    from robot_perception.line import LineDetector
    from robot_perception.material import build_material_pipeline
    from robot_perception.obstacle import (
        CameraObstacleDetector,
        ConfirmedObstacleTracker,
        FrontCameraObstacleSource,
        ObstacleDetectorConfig,
    )
    from robot_perception.qr import CameraTaskCodeReader, QRCodeScanner
    from robot_perception.road import RoadAreaDetector, RoadDetectorConfig

    cv2.setNumThreads(1)
    camera_config = load_camera_config(camera_config_path)
    selected_line_path = (
        Path(line_config_path) if line_config_path else DEFAULT_LINE_CONFIG_PATH
    )
    with selected_line_path.open("r", encoding="utf-8") as file:
        line_config = json.load(file)
    if not isinstance(line_config, dict):
        raise ValueError("巡线配置根节点必须是JSON对象")

    manager = DualCameraManager(camera_config)
    line_detector = LineDetector(line_config["detection"])
    line_follower = LineFollower(
        turn_threshold=line_config["control"]["turn_threshold"],
        smoothing=line_config["control"]["error_smoothing"],
    )
    task_code_reader = CameraTaskCodeReader(
        manager.front,
        QRCodeScanner(),
        required_confirmations=int(
            camera_config["front"].get("qr_confirmations", 2)
        ),
    )
    material_pipeline = build_material_pipeline(
        camera_config["gripper"],
        material_config_path=material_config_path,
        color_config_path=color_config_path,
        backend=material_backend,
    )
    obstacle_source = None
    road_detector = None
    if enable_navigation_perception:
        project_root = Path(__file__).resolve().parents[1]
        obstacle_path = (
            Path(obstacle_config_path)
            if obstacle_config_path
            else project_root / "config" / "obstacle.json"
        )
        obstacle_data = json.loads(obstacle_path.read_text(encoding="utf-8"))
        obstacle_config = ObstacleDetectorConfig.load(obstacle_path)
        obstacle_source = FrontCameraObstacleSource(
            CameraObstacleDetector(obstacle_config),
            ConfirmedObstacleTracker(**obstacle_data["tracking"]),
        )
        road_path = (
            Path(road_config_path)
            if road_config_path
            else project_root / "config" / "road.json"
        )
        road_config = RoadDetectorConfig.load(road_path)
        road_detector = RoadAreaDetector(road_config)

    settle_seconds = max(
        float(camera_config["front"].get("settle_seconds", 0.0)),
        float(camera_config["gripper"].get("settle_seconds", 0.0)),
    )
    return DualCameraVisionController(
        manager,
        line_detector,
        line_follower,
        task_code_reader,
        material_pipeline.detector,
        obstacle_source=obstacle_source,
        road_detector=road_detector,
        gripper_calibrator=material_pipeline.calibrator,
        material_reporter=material_reporter,
        settle_seconds=settle_seconds,
    )
