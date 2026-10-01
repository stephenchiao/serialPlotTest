"""导航相关的独立安全监控和动态制动包络。"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import time
from typing import Callable, Iterable, Optional

from robot_control.navigation_map import NavigationMap, Pose2D
from robot_hardware.camera import CameraError
from robot_runtime.models import ActionResult, SafetyReport


@dataclass(frozen=True)
class NavigationSafetyConfig:
    camera_stale_after_seconds: float
    road_observation_stale_after_seconds: float
    camera_blind_distance_mm: float
    total_reaction_seconds: float
    minimum_braking_deceleration_mm_s2: float
    braking_margin_mm: float
    resume_clearance_margin_mm: float = 40.0
    perception_grace_seconds: float = 0.6

    @classmethod
    def load(cls, path: str | Path) -> "NavigationSafetyConfig":
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        result = cls(**{key: float(value) for key, value in data.items()})
        for key, value in vars(result).items():
            if not math.isfinite(value) or value < 0 or (value == 0 and key != "perception_grace_seconds"):
                raise ValueError(f"{key} 必须大于 0；perception_grace_seconds 可设为 0")
        return result

    def stopping_distance_mm(self, speed_mm_s: float) -> float:
        speed = max(0.0, float(speed_mm_s))
        return (
            self.camera_blind_distance_mm
            + speed * self.total_reaction_seconds
            + speed * speed / (2.0 * self.minimum_braking_deceleration_mm_s2)
            + self.braking_margin_mm
        )


class NavigationSafetyMonitor:
    """汇总串口、OPS9、相机、道路边界和近距离候选障碍。"""

    def __init__(
        self,
        navigation_map: NavigationMap,
        pose_reader: Callable[[], Optional[Pose2D]],
        chassis,
        *,
        link_health: Callable[[], bool],
        camera_health: Callable[[float], bool],
        road_observation_reader: Callable[[], object | None],
        obstacle_candidate_reader: Callable[[], Iterable[object]],
        config: NavigationSafetyConfig,
        perception_updater: Optional[Callable[[Pose2D], object]] = None,
        motion_fault_reader: Callable[[], Optional[str]] = lambda: None,
        link_recovery: Callable[[], bool] = lambda: True,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.map = navigation_map
        self.pose_reader = pose_reader
        self.chassis = chassis
        self.link_health = link_health
        self.camera_health = camera_health
        self.road_observation_reader = road_observation_reader
        self.obstacle_candidate_reader = obstacle_candidate_reader
        self.config = config
        self.clock = clock
        self.perception_updater = perception_updater
        self.motion_fault_reader = motion_fault_reader
        self.link_recovery = link_recovery
        self._navigation_required = False
        self._rechecking = False
        self._last_stopping_distance = config.stopping_distance_mm(0)
        self._paused_stopping_distance = 0.0
        self._last_good_vision_at: Optional[float] = None

    def set_navigation_required(self, required: bool) -> None:
        self._navigation_required = bool(required)

    def begin_recheck(self) -> None:
        self._rechecking = True
        # 停车后不能因为命令速度变成零，就缩小原制动包络而放行。
        self._paused_stopping_distance = self._last_stopping_distance
        self._last_good_vision_at = None

    def end_recheck(self) -> None:
        self._rechecking = False
        self._paused_stopping_distance = 0.0

    def self_check(self) -> ActionResult:
        report = self.check(require_navigation=True)
        if not report.safe:
            return (
                ActionResult.running(report.reason, activity=False)
                if report.recoverable else ActionResult.fatal(report.reason)
            )
        return ActionResult.done("导航安全传感器自检通过", activity=False)

    def check(self, *, require_navigation: bool = False) -> SafetyReport:
        link_ready = self.link_health() and self.link_recovery()
        fault = self.motion_fault_reader()
        if fault:
            return SafetyReport(False, fault, emergency_stop=True)
        if not link_ready:
            return SafetyReport(
                False, "等待 STM32 通信恢复并确认旧航点停止",
                recoverable=True, waiting_for_link=True,
            )
        moving_command = bool(self.chassis.commanded_motion_active)
        self._last_stopping_distance = self.config.stopping_distance_mm(self.chassis.commanded_speed_mm_s)
        required = require_navigation or self._navigation_required or self._rechecking or moving_command
        pose = self.pose_reader()
        if pose is None:
            if required:
                return self._pause("OPS9 位姿失效或超时，停车等待有效数据")
        elif not all(math.isfinite(value) for value in (pose.x_mm, pose.y_mm, pose.yaw_rad)):
            return SafetyReport(False, "OPS9 非有限位姿", emergency_stop=True)
        elif not self.map.contains_footprint(pose):
            return SafetyReport(
                False,
                "车体安全包络接近或越过场地边界",
                boundary_ok=False,
                emergency_stop=True,
            )
        if not required:
            self._last_good_vision_at = None
            return SafetyReport()

        capture_error = ""
        if self.perception_updater is not None:
            try:
                self.perception_updater(pose)
            except (OSError, CameraError) as error:
                capture_error = f"前视采集暂时失败：{error}"
            except Exception as error:
                return SafetyReport(False, f"前视算法异常：{error}", emergency_stop=True)

        if self._rechecking and (
            moving_command or bool(getattr(self.chassis, "is_active", lambda: False)())
        ):
            return self._pause("等待 STM32 确认旧航点停止及车体静止")

        stopping_distance = self._last_stopping_distance
        if self._rechecking:
            stopping_distance = max(stopping_distance, self._paused_stopping_distance)
            stopping_distance += self.config.resume_clearance_margin_mm
        nearest = self._nearest_forward_clearance(pose)
        if nearest is not None and nearest <= stopping_distance:
            return self._pause(f"前方候选障碍进入动态制动包络：{nearest:.0f} mm，停车复核")

        # 先检查近障碍，避免视觉容错窗口掩盖已经检测到的碰撞风险。
        if capture_error:
            return self._vision_failure(capture_error, moving_command)
        if not self.camera_health(self.config.camera_stale_after_seconds):
            return self._vision_failure("前视摄像头失效或画面冻结", moving_command)
        road = self.road_observation_reader()
        if road is None:
            return self._vision_failure("缺少前方道路安全观测", moving_command)
        age = self.clock() - road.observed_at
        if not math.isfinite(age) or not 0 <= age <= self.config.road_observation_stale_after_seconds:
            return self._vision_failure("道路安全观测已超时或时间戳无效", moving_command)
        if not road.boundary_safe:
            return self._vision_failure("视觉道路不安全，需重新确认灰色车道和黄白边界", moving_command)

        self._last_good_vision_at = road.observed_at
        return SafetyReport(observation_timestamp=road.observed_at)

    def _vision_failure(self, reason: str, moving: bool) -> SafetyReport:
        # 只允许已在行驶的小车短暂沿原航点继续；启动和暂停复核都必须有有效画面。
        # 以最后有效帧计时，交替报错或重复旧帧不会延长容错窗口。
        if moving and not self._rechecking and self._last_good_vision_at is not None:
            age = self.clock() - self._last_good_vision_at
            if 0 <= age < self.config.perception_grace_seconds:
                return SafetyReport(reason=f"视觉短时降级，沿原航点继续：{reason}")
        return self._pause(reason)

    @staticmethod
    def _pause(reason: str) -> SafetyReport:
        return SafetyReport(False, reason, recoverable=True)

    def _nearest_forward_clearance(self, pose: Pose2D) -> Optional[float]:
        nearest: Optional[float] = None
        cosine, sine = math.cos(pose.yaw_rad), math.sin(pose.yaw_rad)
        for obstacle in self.obstacle_candidate_reader():
            dx = obstacle.x_mm - pose.x_mm
            dy = obstacle.y_mm - pose.y_mm
            body_x = cosine * dx + sine * dy
            body_y = -sine * dx + cosine * dy
            required = self.map.clearance_mm + obstacle.radius_mm
            if not all(math.isfinite(value) for value in (body_x, body_y, required)) or obstacle.radius_mm <= 0:
                raise ValueError("障碍物几何数据无效")
            if body_x < 0:
                continue
            # 前向扫掠走廊外的障碍不会撞到车体，不应误触发停车。
            if abs(body_y) > required:
                continue
            clearance = max(0.0, body_x - math.sqrt(max(0.0, required * required - body_y * body_y)))
            nearest = clearance if nearest is None else min(nearest, clearance)
        return nearest
