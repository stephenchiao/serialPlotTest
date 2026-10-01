"""把 STM32 USB/串口链路、OPS9 遥测、地图和底盘控制装配为导航组件。"""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Callable, Iterable, Optional

from robot_hardware.stm32 import SerialLink, Stm32ChassisController, Stm32Ops9Receiver
from robot_hardware.stm32.messages import RECOVERABLE_MOTION_FAULTS
from robot_hardware.stm32.pose_goal import PoseTransactionState, Stm32PoseGoalController
from robot_runtime.models import ActionResult
from robot_runtime.models import TargetArea

from .navigation_map import CircularObstacle, NavigationMap, Pose2D
from .navigator import MapNavigator, NavigationLimits, Ops9MapTransform
from .pose_navigation import Stm32PoseMapNavigator


PROJECT_ROOT = Path(__file__).resolve().parents[1]


@dataclass
class Stm32NavigationStack:
    """生命周期由真实硬件入口调用；串口本身仍由应用统一打开和关闭。"""

    navigator: MapNavigator | Stm32PoseMapNavigator
    ops9_receiver: Stm32Ops9Receiver
    chassis: Stm32ChassisController | Stm32PoseGoalController
    link: SerialLink
    pose_reader: Callable[[], Optional[Pose2D]]

    def start(self) -> None:
        self.link.open()
        self.ops9_receiver.attach()
        attach = getattr(self.chassis, "attach", None)
        if attach is not None:
            attach()

    def close(self) -> None:
        try:
            self.navigator.cancel()
        finally:
            detach = getattr(self.chassis, "detach", None)
            if detach is not None:
                detach()
            self.ops9_receiver.detach()
            self.link.close()

    initialize = start
    shutdown = close

    def self_check(self) -> ActionResult:
        if not self.link.connected:
            return ActionResult.retryable("STM32 USB/串口链路未连接")
        if self.ops9_receiver.latest() is None:
            return ActionResult.running("等待有效 OPS9 位姿", activity=False)
        return ActionResult.done("STM32、OPS9 与导航地图自检通过", activity=False)

    def recover_navigation(self) -> bool:
        if isinstance(self.navigator, Stm32PoseMapNavigator):
            return self.navigator.recovery_checker()
        return True

    def navigate_to(self, target):
        return self.navigator.navigate_to(target)

    def cancel(self) -> None:
        self.navigator.cancel()


def build_stm32_navigation(
    link: SerialLink,
    *,
    navigation_config: str | Path = PROJECT_ROOT / "config" / "navigation.json",
    ops9_config: str | Path = PROJECT_ROOT / "config" / "ops9.json",
    obstacle_reader: Callable[[], Iterable[CircularObstacle]] = tuple,
    perception_updater: Optional[Callable[[Pose2D], None]] = None,
) -> Stm32NavigationStack:
    navigation_path = Path(navigation_config)
    navigation_data = json.loads(navigation_path.read_text(encoding="utf-8"))
    ops9_data = json.loads(Path(ops9_config).read_text(encoding="utf-8"))
    navigation_map = NavigationMap.load(navigation_path)

    receiver = Stm32Ops9Receiver(
        link,
        stale_after_seconds=float(ops9_data["stale_after_seconds"]),
        minimum_quality=int(ops9_data["minimum_quality"]),
        maximum_position_jump_mm=float(ops9_data["maximum_position_jump_mm"]),
        maximum_yaw_jump_mrad=int(ops9_data["maximum_yaw_jump_mrad"]),
        movement_threshold_mm=float(ops9_data["movement_threshold_mm"]),
        movement_threshold_mrad=int(ops9_data["movement_threshold_mrad"]),
        movement_hold_seconds=float(ops9_data["movement_hold_seconds"]),
    )
    transform_data = ops9_data["map_transform"]
    ops_start = transform_data["ops9_start_pose_mm_rad"]
    map_start = transform_data["map_start_pose_mm_rad"]
    transform = Ops9MapTransform(
        Pose2D(float(map_start[0]), float(map_start[1]), float(map_start[2])),
        Pose2D(float(ops_start[0]), float(ops_start[1]), float(ops_start[2])),
    )

    def read_map_pose() -> Optional[Pose2D]:
        raw = receiver.latest()
        if raw is None:
            return None
        return transform.apply(
            Pose2D(raw.x_mm, raw.y_mm, raw.yaw_mrad / 1000.0)
        )

    planner = navigation_data["planner"]
    limits = NavigationLimits(
        maximum_speed_mm_s=float(planner["maximum_speed_mm_s"]),
        maximum_yaw_rate_mrad_s=float(planner["maximum_yaw_rate_mrad_s"]),
        waypoint_tolerance_mm=float(planner["waypoint_tolerance_mm"]),
    )
    control_mode = str(planner.get("control_mode", "stm32_pose_goal"))
    if control_mode == "legacy_velocity":
        if isinstance(link, SerialLink):
            raise ValueError("队友 v2 固件不支持 legacy_velocity，请使用 stm32_pose_goal")
        chassis = Stm32ChassisController(link, activity_reader=receiver.is_moving)
        navigator = MapNavigator(
            navigation_map,
            read_map_pose,
            chassis,
            obstacle_reader=obstacle_reader,
            perception_updater=perception_updater,
            limits=limits,
        )
    elif control_mode == "stm32_pose_goal":
        chassis = Stm32PoseGoalController(
            link,
            activity_reader=receiver.is_moving,
            maximum_speed_mm_s=limits.maximum_speed_mm_s,
            maximum_yaw_rate_mrad_s=limits.maximum_yaw_rate_mrad_s,
            request_timeout_seconds=float(
                planner.get("pose_request_timeout_seconds", 0.5)
            ),
        )
        configured_headings = navigation_data.get("target_headings_mrad", {})
        target_headings = {
            TargetArea(key): int(value)
            for key, value in configured_headings.items()
        }

        unmatched_fault_reconnect_requested = False

        def recover_navigation() -> bool:
            nonlocal unmatched_fault_reconnect_requested
            if not chassis.recover_connection():
                return False
            fault = receiver.motion_fault
            if fault is None:
                unmatched_fault_reconnect_requested = False
                return True
            if fault.reason not in RECOVERABLE_MOTION_FAULTS:
                return False
            snapshot = chassis.snapshot()
            if snapshot.state is PoseTransactionState.FAULT and snapshot.fault_reason not in RECOVERABLE_MOTION_FAULTS:
                return False
            if (
                snapshot.state is PoseTransactionState.CANCELLED and snapshot.goal is not None
                and snapshot.goal.goal_id == fault.goal_id and snapshot.fault_reason == fault.reason
            ):
                return receiver.recover_fault(snapshot.goal.goal_id, snapshot.fault_reason)
            # 启动前/迟到事件的软故障没有可匹配事务；用新会话隔离，不能永久卡住。
            if not unmatched_fault_reconnect_requested:
                link.request_reconnect()
                unmatched_fault_reconnect_requested = True
            return False

        navigator = Stm32PoseMapNavigator(
            navigation_map,
            read_map_pose,
            chassis,
            transform,
            obstacle_reader=obstacle_reader,
            perception_updater=perception_updater,
            limits=limits,
            waypoint_timeout_seconds=float(
                planner.get("waypoint_timeout_seconds", 35.0)
            ),
            target_yaw_mrad=target_headings,
            recovery_checker=recover_navigation,
        )
    else:
        raise ValueError(
            "planner.control_mode 必须是 stm32_pose_goal 或 legacy_velocity"
        )
    return Stm32NavigationStack(navigator, receiver, chassis, link, read_map_pose)
