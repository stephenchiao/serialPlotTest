"""OPS9 位姿驱动的非阻塞路点导航器。"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Callable, Iterable, Optional, Protocol

from robot_runtime.models import ActionResult, TargetArea

from .navigation_map import (
    CircularObstacle,
    NavigationMap,
    NoRouteError,
    Point2D,
    Pose2D,
    RoutePlan,
)


class VelocityController(Protocol):
    def set_velocity(self, vx_mm_s: int, vy_mm_s: int, wz_mrad_s: int) -> None: ...

    def stop(self) -> None: ...


@dataclass(frozen=True)
class NavigationLimits:
    maximum_speed_mm_s: float = 350.0
    maximum_yaw_rate_mrad_s: float = 900.0
    position_gain_per_second: float = 1.1
    heading_gain_per_second: float = 1.8
    waypoint_tolerance_mm: float = 90.0
    movement_activity_mm: float = 3.0


class Ops9MapTransform:
    """将 OPS9 的启动相对坐标刚体变换到地图坐标。"""

    def __init__(
        self,
        map_start_pose: Pose2D,
        ops9_start_pose: Pose2D = Pose2D(0.0, 0.0, 0.0),
    ) -> None:
        self.map_start_pose = map_start_pose
        self.ops9_start_pose = ops9_start_pose

    def apply(self, ops9_pose: Pose2D) -> Pose2D:
        relative_yaw = self.map_start_pose.yaw_rad - self.ops9_start_pose.yaw_rad
        dx = ops9_pose.x_mm - self.ops9_start_pose.x_mm
        dy = ops9_pose.y_mm - self.ops9_start_pose.y_mm
        cosine, sine = math.cos(relative_yaw), math.sin(relative_yaw)
        return Pose2D(
            self.map_start_pose.x_mm + cosine * dx - sine * dy,
            self.map_start_pose.y_mm + sine * dx + cosine * dy,
            _wrap_angle(ops9_pose.yaw_rad + relative_yaw),
        )

    def invert(self, map_pose: Pose2D) -> Pose2D:
        """把地图绝对位姿转换回 STM32 使用的 OPS9 原始坐标。"""

        relative_yaw = self.map_start_pose.yaw_rad - self.ops9_start_pose.yaw_rad
        dx = map_pose.x_mm - self.map_start_pose.x_mm
        dy = map_pose.y_mm - self.map_start_pose.y_mm
        cosine, sine = math.cos(relative_yaw), math.sin(relative_yaw)
        return Pose2D(
            self.ops9_start_pose.x_mm + cosine * dx + sine * dy,
            self.ops9_start_pose.y_mm - sine * dx + cosine * dy,
            _wrap_angle(map_pose.yaw_rad - relative_yaw),
        )


class MapNavigator:
    """每次 ``navigate_to`` 只做一次控制周期，适配现有状态机轮询。"""

    def __init__(
        self,
        navigation_map: NavigationMap,
        pose_reader: Callable[[], Optional[Pose2D]],
        velocity: VelocityController,
        *,
        obstacle_reader: Callable[[], Iterable[CircularObstacle]] = tuple,
        perception_updater: Optional[Callable[[Pose2D], None]] = None,
        limits: NavigationLimits = NavigationLimits(),
    ) -> None:
        self.map = navigation_map
        self.pose_reader = pose_reader
        self.velocity = velocity
        self.obstacle_reader = obstacle_reader
        self.perception_updater = perception_updater
        self.limits = limits
        self._target: Optional[TargetArea] = None
        self._plan: Optional[RoutePlan] = None
        self._waypoint_index = 0
        self._last_pose: Optional[Pose2D] = None

    @property
    def current_plan(self) -> Optional[RoutePlan]:
        return self._plan

    def navigate_to(self, target: TargetArea) -> ActionResult:
        pose = self.pose_reader()
        if pose is None:
            self.velocity.stop()
            return ActionResult.retryable("OPS9 位姿无效、质量过低或已超时")
        if not self.map.contains_footprint(pose):
            self.velocity.stop()
            return ActionResult.fatal("车体安全包络接近或越过场地边界")

        if self.perception_updater is not None:
            try:
                self.perception_updater(pose)
            except Exception as error:
                self.velocity.stop()
                return ActionResult.fatal(f"前视导航感知失败：{error}")
        blocked = self.map.blocked_edges(self.obstacle_reader())
        if (
            self._plan is None or target != self._target
            or self.map.route_is_blocked(self._plan, self._waypoint_index, blocked)
        ):
            if not self._replan(pose, target, blocked):
                self.velocity.stop()
                return ActionResult.running("当前无可用路线，保持停车并重新观察封路", activity=False)

        assert self._plan is not None
        while self._waypoint_index < len(self._plan.nodes):
            node_name = self._plan.nodes[self._waypoint_index]
            waypoint = self.map.nodes[node_name]
            if _distance(pose, waypoint) > self.limits.waypoint_tolerance_mm:
                break
            self._waypoint_index += 1

        activity = self._physical_activity(pose)
        if self._waypoint_index >= len(self._plan.nodes):
            self.velocity.stop()
            self._reset()
            return ActionResult.done(f"已到达 {target.value}", activity=activity)

        waypoint_name = self._plan.nodes[self._waypoint_index]
        waypoint = self.map.nodes[waypoint_name]
        vx, vy, wz = self._control(pose, waypoint)
        self.velocity.set_velocity(vx, vy, wz)
        return ActionResult.running(
            f"前往 {waypoint_name}；封闭道路 {sorted(blocked)}",
            activity=activity,
        )

    def cancel(self) -> None:
        self.velocity.stop()
        self._reset()

    def _replan(
        self,
        pose: Pose2D,
        target: TargetArea,
        blocked: frozenset[str],
    ) -> bool:
        start = self.map.nearest_node(pose)
        goal = self.map.target_node(target)
        try:
            plan = self.map.plan(start, goal, blocked_edges=blocked)
        except NoRouteError:
            return False
        self._target = target
        self._plan = plan
        self._waypoint_index = 0
        return True

    def _control(self, pose: Pose2D, target: Point2D) -> tuple[int, int, int]:
        dx = target.x_mm - pose.x_mm
        dy = target.y_mm - pose.y_mm
        cosine, sine = math.cos(pose.yaw_rad), math.sin(pose.yaw_rad)
        body_x = cosine * dx + sine * dy
        body_y = -sine * dx + cosine * dy
        speed = self.limits.position_gain_per_second * math.hypot(dx, dy)
        speed = min(speed, self.limits.maximum_speed_mm_s)
        direction = math.atan2(body_y, body_x)
        vx = int(round(speed * math.cos(direction)))
        vy = int(round(speed * math.sin(direction)))
        desired_heading = math.atan2(dy, dx)
        heading_error = _wrap_angle(desired_heading - pose.yaw_rad)
        wz = int(
            round(
                max(
                    -self.limits.maximum_yaw_rate_mrad_s,
                    min(
                        self.limits.maximum_yaw_rate_mrad_s,
                        heading_error * 1000.0 * self.limits.heading_gain_per_second,
                    ),
                )
            )
        )
        return vx, vy, wz

    def _physical_activity(self, pose: Pose2D) -> bool:
        previous, self._last_pose = self._last_pose, pose
        if previous is None:
            return False
        return _distance(previous, pose) >= self.limits.movement_activity_mm

    def _reset(self) -> None:
        self._target = None
        self._plan = None
        self._waypoint_index = 0
        self._last_pose = None


def _distance(first: Point2D, second: Point2D) -> float:
    return math.hypot(first.x_mm - second.x_mm, first.y_mm - second.y_mm)


def _wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi
