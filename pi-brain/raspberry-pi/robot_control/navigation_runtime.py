"""真实前摄像头、STM32 OPS9、规划器和安全监控的一体化装配。"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from robot_hardware.stm32 import SerialLink
from robot_hardware.stm32.messages import RECOVERABLE_MOTION_FAULTS, MotionFaultReason
from robot_hardware.stm32.pose_goal import PoseTransactionState, Stm32PoseGoalController
from robot_runtime.models import ActionResult, ActionStatus
from robot_services.navigation_safety import (
    NavigationSafetyConfig,
    NavigationSafetyMonitor,
)

from .dual_camera_vision import DualCameraVisionController
from .navigation_factory import PROJECT_ROOT, Stm32NavigationStack, build_stm32_navigation


@dataclass
class NavigationRuntime:
    """可直接作为 ``ComponentBundle.navigator`` 使用。"""

    stack: Stm32NavigationStack
    vision: DualCameraVisionController
    safety: NavigationSafetyMonitor

    def initialize(self) -> None:
        self.vision.start()
        try:
            self.stack.start()
        except Exception:
            self.vision.close()
            raise

    def self_check(self) -> ActionResult:
        safety_result = self.safety.self_check()
        if safety_result.status is not ActionStatus.DONE:
            return safety_result
        stack_result = self.stack.self_check()
        if stack_result.status is not ActionStatus.DONE:
            return stack_result
        return safety_result

    def navigate_to(self, target):
        return self.stack.navigate_to(target)

    def cancel(self) -> None:
        self.stack.cancel()

    def shutdown(self) -> None:
        try:
            self.stack.close()
        finally:
            self.vision.close()


def build_navigation_runtime(
    link: SerialLink,
    vision: DualCameraVisionController,
    *,
    navigation_config: str | Path = PROJECT_ROOT / "config" / "navigation.json",
    ops9_config: str | Path = PROJECT_ROOT / "config" / "ops9.json",
    safety_config: str | Path = PROJECT_ROOT / "config" / "navigation_safety.json",
) -> NavigationRuntime:
    if vision.obstacle_source is None or vision.road_detector is None:
        raise ValueError(
            "视觉控制器未启用导航感知；构建时需要 "
            "enable_navigation_perception=True 且完成现场标定"
        )

    stack = build_stm32_navigation(
        link,
        navigation_config=navigation_config,
        ops9_config=ops9_config,
        obstacle_reader=vision.obstacle_source.obstacles,
        # 安全检查先采集新画面，暂停期间也采集，导航不再重复采集。
        perception_updater=None,
    )

    def latest_road_observation():
        result = vision.latest_navigation_result
        return None if result is None else result.road_observation

    def motion_fault():
        reason = None
        if isinstance(stack.chassis, Stm32PoseGoalController):
            snapshot = stack.chassis.snapshot()
            if (
                snapshot.state is PoseTransactionState.FAULT
                and snapshot.fault_reason not in RECOVERABLE_MOTION_FAULTS
            ):
                reason = snapshot.fault_reason
        fault = stack.ops9_receiver.motion_fault
        if fault is not None and fault.reason not in RECOVERABLE_MOTION_FAULTS:
            reason = fault.reason
        if reason is not None:
            try:
                name = MotionFaultReason(reason).name
            except ValueError:
                name = f"UNKNOWN_0x{reason:04X}"
            return f"STM32 不可自动恢复故障：{name}"
        return None

    safety = NavigationSafetyMonitor(
        stack.navigator.map,
        stack.pose_reader,
        stack.chassis,
        link_health=lambda: link.connected,
        camera_health=lambda age: vision.is_healthy(age, roles=("front",)),
        road_observation_reader=latest_road_observation,
        obstacle_candidate_reader=vision.obstacle_source.candidates,
        config=NavigationSafetyConfig.load(safety_config),
        perception_updater=vision.observe_navigation,
        motion_fault_reader=motion_fault,
        link_recovery=stack.recover_navigation,
    )
    return NavigationRuntime(stack, vision, safety)
