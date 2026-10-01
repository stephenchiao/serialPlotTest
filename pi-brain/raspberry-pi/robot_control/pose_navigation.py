"""地图路径规划与 STM32 单航点闭环之间的非阻塞适配器。"""

from __future__ import annotations

import math
from typing import Callable, Iterable, Mapping, Optional

from robot_hardware.stm32.messages import RECOVERABLE_MOTION_FAULTS, MotionFaultReason
from robot_hardware.stm32.pose_goal import (
    PoseGoalBusy,
    PoseTransactionState,
    Stm32PoseGoalController,
)
from robot_hardware.stm32.serial_link import SerialLinkError
from robot_runtime.models import ActionResult, TargetArea

from .navigation_map import (
    CircularObstacle,
    NavigationMap,
    NoRouteError,
    Pose2D,
    RoutePlan,
)
from .navigator import NavigationLimits, Ops9MapTransform


class Stm32PoseMapNavigator:
    """树莓派规划路网，STM32 依次闭环执行带 ``goal_id`` 的航点。"""

    def __init__(
        self,
        navigation_map: NavigationMap,
        pose_reader: Callable[[], Optional[Pose2D]],
        controller: Stm32PoseGoalController,
        transform: Ops9MapTransform,
        *,
        obstacle_reader: Callable[[], Iterable[CircularObstacle]] = tuple,
        perception_updater: Optional[Callable[[Pose2D], None]] = None,
        limits: NavigationLimits = NavigationLimits(),
        waypoint_timeout_seconds: float = 35.0,
        target_yaw_mrad: Mapping[TargetArea, int] | None = None,
        recovery_checker: Optional[Callable[[], bool]] = None,
    ) -> None:
        if waypoint_timeout_seconds <= 0:
            raise ValueError("waypoint_timeout_seconds 必须大于 0")
        self.map = navigation_map
        self.pose_reader = pose_reader
        self.controller = controller
        self.transform = transform
        self.obstacle_reader = obstacle_reader
        self.perception_updater = perception_updater
        self.limits = limits
        self.waypoint_timeout_seconds = float(waypoint_timeout_seconds)
        self.target_yaw_mrad = dict(target_yaw_mrad or {})
        self.recovery_checker = recovery_checker or controller.recover_connection
        self._target: Optional[TargetArea] = None
        self._plan: Optional[RoutePlan] = None
        self._waypoint_index = 0

    @property
    def current_plan(self) -> Optional[RoutePlan]:
        return self._plan

    def navigate_to(self, target: TargetArea) -> ActionResult:
        recovered = self.recovery_checker()
        transaction = self.controller.snapshot()
        if transaction.state is PoseTransactionState.FAULT and transaction.fault_reason not in RECOVERABLE_MOTION_FAULTS:
            return ActionResult.fatal(f"STM32 位姿闭环故障：{_fault_name(transaction.fault_reason)}")
        if not recovered:
            return ActionResult.running("等待核对旧航点及有效定位，暂不提交新目标", activity=False)
        pose = self.pose_reader()
        if pose is None:
            if self.controller.commanded_motion_active:
                self.controller.stop()
            return ActionResult.retryable("OPS9 位姿无效、质量过低或已超时")
        if not self.map.contains_footprint(pose):
            if self.controller.commanded_motion_active:
                self.controller.stop()
            return ActionResult.fatal("车体安全包络接近或越过场地边界")

        if self.perception_updater is not None:
            try:
                self.perception_updater(pose)
            except Exception as error:
                if self.controller.commanded_motion_active:
                    self.controller.stop()
                return ActionResult.fatal(f"前视导航感知失败：{error}")

        blocked = self.map.blocked_edges(self.obstacle_reader())
        if self._route_changed(target, blocked):
            if self.controller.commanded_motion_active:
                try:
                    self.controller.cancel()
                except SerialLinkError as error:
                    if self.controller.snapshot().fault_reason == MotionFaultReason.HOST_LOST:
                        return ActionResult.running("通信中断，等待新会话后重新规划", activity=False)
                    return ActionResult.fatal(f"取消旧航点失败：{error}")
                return ActionResult.running(
                    "路线或障碍变化，等待 STM32 确认取消旧航点",
                    activity=self.controller.is_active(),
                )
            self._reset_plan()

        transaction = self.controller.snapshot()
        if transaction.state in {
            PoseTransactionState.ACCEPTED,
            PoseTransactionState.MOVING,
            PoseTransactionState.CANCELLING,
            PoseTransactionState.RECONCILING,
        }:
            return ActionResult.running(
                self._active_message(transaction.state),
                activity=self.controller.is_active(),
            )
        if transaction.state == PoseTransactionState.CANCELLED:
            self.controller.clear_terminal()
            self._reset_plan()
        elif transaction.state == PoseTransactionState.FAULT:
            reason = transaction.fault_reason
            self.controller.clear_terminal()
            self._reset_plan()
            message = f"STM32 位姿闭环故障：{_fault_name(reason)}"
            return (
                ActionResult.fatal(message)
                if reason not in RECOVERABLE_MOTION_FAULTS
                else ActionResult.retryable(message)
            )
        elif transaction.state == PoseTransactionState.REACHED:
            self.controller.clear_terminal()
            self._waypoint_index += 1

        if self._plan is None and not self._replan(pose, target, blocked):
            # 不在几次快速轮询中耗尽重试；持续采集新图像后再规划。
            # 状态机负责无动作超时后的有限恢复重试。
            return ActionResult.running("当前无可用路线，保持停车并重新观察封路", activity=False)

        assert self._plan is not None
        while self._waypoint_index < len(self._plan.nodes):
            node = self.map.nodes[self._plan.nodes[self._waypoint_index]]
            if math.hypot(pose.x_mm - node.x_mm, pose.y_mm - node.y_mm) > (
                self.limits.waypoint_tolerance_mm
            ):
                break
            self._waypoint_index += 1

        if self._waypoint_index >= len(self._plan.nodes):
            self._reset_plan()
            return ActionResult.done(
                f"已到达 {target.value}",
                activity=self.controller.is_active(),
            )

        node_name = self._plan.nodes[self._waypoint_index]
        waypoint = self.map.nodes[node_name]
        yaw_rad = self._waypoint_yaw(target, pose)
        ops_goal = self.transform.invert(Pose2D(waypoint.x_mm, waypoint.y_mm, yaw_rad))
        try:
            goal_id = self.controller.submit(
                int(round(ops_goal.x_mm)),
                int(round(ops_goal.y_mm)),
                int(round(ops_goal.yaw_rad * 1000.0)),
                timeout_seconds=self.waypoint_timeout_seconds,
            )
        except (SerialLinkError, PoseGoalBusy) as error:
            transaction = self.controller.snapshot()
            if transaction.fault_reason == MotionFaultReason.HOST_LOST:
                return ActionResult.running("通信中断，等待新会话后重新规划", activity=False)
            if transaction.state is PoseTransactionState.FAULT and transaction.fault_reason not in RECOVERABLE_MOTION_FAULTS:
                return ActionResult.fatal(f"提交 STM32 航点失败：{error}")
            return ActionResult.retryable(f"提交 STM32 航点失败：{error}")
        return ActionResult.running(
            f"已提交航点 {node_name}，goal_id={goal_id}",
            activity=self.controller.is_active(),
        )

    def cancel(self) -> None:
        self.controller.cancel()
        self._reset_plan()

    def _route_changed(
        self,
        target: TargetArea,
        blocked: frozenset[str],
    ) -> bool:
        return (
            self._plan is not None
            and (
                target != self._target
                or self.map.route_is_blocked(self._plan, self._waypoint_index, blocked)
            )
        )

    def _replan(
        self,
        pose: Pose2D,
        target: TargetArea,
        blocked: frozenset[str],
    ) -> bool:
        try:
            plan = self.map.plan(
                self.map.nearest_node(pose),
                self.map.target_node(target),
                blocked_edges=blocked,
            )
        except NoRouteError:
            return False
        self._target = target
        self._plan = plan
        self._waypoint_index = 0
        return True

    def _waypoint_yaw(self, target: TargetArea, pose: Pose2D) -> float:
        assert self._plan is not None
        if self._waypoint_index == len(self._plan.nodes) - 1:
            configured = self.target_yaw_mrad.get(target)
            return pose.yaw_rad if configured is None else configured / 1000.0
        current = self.map.nodes[self._plan.nodes[self._waypoint_index]]
        following = self.map.nodes[self._plan.nodes[self._waypoint_index + 1]]
        return math.atan2(following.y_mm - current.y_mm, following.x_mm - current.x_mm)

    def _reset_plan(self) -> None:
        self._target = None
        self._plan = None
        self._waypoint_index = 0

    @staticmethod
    def _active_message(state: PoseTransactionState) -> str:
        labels = {
            PoseTransactionState.ACCEPTED: "STM32 已接受航点，等待启动事件",
            PoseTransactionState.MOVING: "STM32 正在执行航点闭环",
            PoseTransactionState.CANCELLING: "等待 STM32 确认航点取消",
            PoseTransactionState.RECONCILING: "等待查询确认 STM32 航点实际状态",
        }
        return labels[state]


def _fault_name(reason: int) -> str:
    try:
        return MotionFaultReason(reason).name
    except ValueError:
        return f"UNKNOWN_0x{reason:04X}"
