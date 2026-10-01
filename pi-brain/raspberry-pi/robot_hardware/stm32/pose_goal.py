"""带 ``goal_id`` 的 STM32 位姿事务客户端。

本模块不读取串口；它订阅唯一 ``SerialLink`` 的 EVENT 分发，并把 STM32
闭环的接受、启动、到位、取消和故障转换为线程安全快照。
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
import threading
import secrets
import time
from typing import Callable, Optional

from .messages import (
    Command,
    EventCode,
    MessageType,
    MotionFault,
    MotionFaultReason,
    RECOVERABLE_MOTION_FAULTS,
    PoseCancelled,
    PoseGoal,
    PoseGoalState,
    PoseGoalStatus,
    PoseReached,
    PoseStarted,
    ResponseStatus,
    decode_motion_event,
    encode_cancel_pose_goal,
    encode_speed_limits,
)
from .protocol import Frame
from .serial_link import CommandRejected, CommandTimeout, SerialLink, SerialLinkError


class PoseTransactionState(Enum):
    IDLE = auto()
    ACCEPTED = auto()
    MOVING = auto()
    CANCELLING = auto()
    RECONCILING = auto()
    REACHED = auto()
    CANCELLED = auto()
    FAULT = auto()


@dataclass(frozen=True)
class PoseTransactionSnapshot:
    state: PoseTransactionState
    goal: Optional[PoseGoal] = None
    reached: Optional[PoseReached] = None
    fault_reason: int = MotionFaultReason.UNSPECIFIED

    @property
    def terminal(self) -> bool:
        return self.state in {
            PoseTransactionState.REACHED,
            PoseTransactionState.CANCELLED,
            PoseTransactionState.FAULT,
        }


class PoseGoalBusy(RuntimeError):
    pass


class Stm32PoseGoalController:
    """提交一个绝对位姿，并只接受同一 ``goal_id`` 的终态事件。"""

    _ACTIVE_STATES = {
        PoseTransactionState.ACCEPTED,
        PoseTransactionState.MOVING,
        PoseTransactionState.CANCELLING,
        PoseTransactionState.RECONCILING,
    }

    def __init__(
        self,
        link: SerialLink,
        *,
        activity_reader: Optional[Callable[[], bool]] = None,
        maximum_speed_mm_s: float = 350.0,
        maximum_yaw_rate_mrad_s: float = 250.0,
        request_timeout_seconds: float = 0.5,
        cancellation_timeout_seconds: float = 1.5,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if maximum_speed_mm_s <= 0:
            raise ValueError("maximum_speed_mm_s 必须大于 0")
        if request_timeout_seconds <= 0:
            raise ValueError("request_timeout_seconds 必须大于 0")
        if cancellation_timeout_seconds <= 0:
            raise ValueError("cancellation_timeout_seconds 必须大于 0")
        self.link = link
        self._activity_reader = activity_reader
        # 服从 v2 固件限幅，不用配置中的 350 mm/s 绕过其 300 mm/s 上限。
        self._maximum_speed_mm_s = min(float(maximum_speed_mm_s), 300.0)
        self._speed_limits = encode_speed_limits(
            self._maximum_speed_mm_s, min(float(maximum_yaw_rate_mrad_s), 800.0)
        )
        self._request_timeout = float(request_timeout_seconds)
        self._cancellation_timeout = float(cancellation_timeout_seconds)
        self._clock = clock
        self._cancel_started_at: Optional[float] = None
        self._cancel_requested_goal_id: Optional[int] = None
        self._lock = threading.Lock()
        self._snapshot = PoseTransactionSnapshot(PoseTransactionState.IDLE)
        self._next_goal_id = secrets.randbelow(0xFFFFFFFF) + 1
        self._link_generation = getattr(link, "generation", 0)
        self._confirmed_generation = self._link_generation
        self._attached = False
        self.invalid_events = 0
        self.stale_events = 0

    def attach(self) -> None:
        if self._attached:
            return
        self.link.add_frame_handler(MessageType.EVENT, self._on_frame)
        self._attached = True

    def detach(self) -> None:
        if not self._attached:
            return
        self.link.remove_frame_handler(MessageType.EVENT, self._on_frame)
        self._attached = False

    def submit(
        self,
        x_mm: int,
        y_mm: int,
        yaw_mrad: int,
        *,
        timeout_seconds: float,
    ) -> int:
        timeout_ms = int(round(timeout_seconds * 1000.0))
        if timeout_ms <= 0:
            raise ValueError("timeout_seconds 必须大于 0")
        self.snapshot()  # 先将旧会话的活动事务失效，再决定能否提交新目标
        with self._lock:
            if self._snapshot.state in self._ACTIVE_STATES or self._snapshot.state is PoseTransactionState.FAULT:
                raise PoseGoalBusy("旧位姿目标活动中或故障尚未处理")
            goal_id = self._next_goal_id
            self._next_goal_id = 1 if goal_id == 0xFFFFFFFF else goal_id + 1
            goal = PoseGoal(goal_id, x_mm, y_mm, yaw_mrad, timeout_ms)
            payload = goal.encode_command_data() + self._speed_limits
            self._snapshot = PoseTransactionSnapshot(
                PoseTransactionState.ACCEPTED,
                goal=goal,
            )
            self._cancel_started_at = None
            self._cancel_requested_goal_id = None
        self._request_goal(
            Command.SET_POSE_GOAL_WITH_LIMITS, goal, payload,
        )
        return goal_id

    def cancel(self) -> bool:
        self.snapshot()  # 断线后的旧事务只在新会话确认静止后清除。
        with self._lock:
            snapshot = self._snapshot
            if snapshot.goal is None or snapshot.state not in self._ACTIVE_STATES:
                return False
            goal_id = snapshot.goal.goal_id
            if self._cancel_requested_goal_id == goal_id:
                return False
            self._cancel_requested_goal_id = goal_id
            if self._cancel_started_at is None:
                self._cancel_started_at = self._clock()
        self._request_goal(
            Command.CANCEL_POSE_GOAL, snapshot.goal, encode_cancel_pose_goal(goal_id),
        )
        with self._lock:
            if (
                self._snapshot.goal is not None
                and self._snapshot.goal.goal_id == goal_id
                and self._snapshot.state in {
                    PoseTransactionState.ACCEPTED, PoseTransactionState.MOVING,
                    PoseTransactionState.CANCELLING,
                }
            ):
                self._snapshot = PoseTransactionSnapshot(
                    PoseTransactionState.CANCELLING,
                    goal=self._snapshot.goal,
                )
        return True

    def _request_goal(
        self, command: Command, goal: PoseGoal, data: bytes,
    ) -> None:
        """丢失应答进入查询核对；明确失败保留故障，不重放动作。"""
        generation = getattr(self.link, "generation", 0)
        try:
            self.link.request(command, data, timeout=self._request_timeout)
        except Exception as error:
            uncertain = isinstance(error, CommandTimeout) or (
                isinstance(error, CommandRejected) and error.response.status is ResponseStatus.BUSY
            )
            if (
                not getattr(self.link, "connected", True)
                or generation != getattr(self.link, "generation", 0)
            ):
                failure_reason = MotionFaultReason.HOST_LOST
                uncertain = False
            elif uncertain:
                failure_reason = (
                    MotionFaultReason.REQUEST_TIMEOUT
                    if command is Command.SET_POSE_GOAL_WITH_LIMITS else MotionFaultReason.CANCEL_TIMEOUT
                )
            else:
                failure_reason = MotionFaultReason.INTERNAL_ERROR
            if not uncertain:
                try:
                    self.link.send_command(Command.STOP_ALL)
                except Exception:
                    pass
            with self._lock:
                if self._snapshot.goal == goal and not (
                    self._snapshot.state is PoseTransactionState.FAULT
                    and self._snapshot.fault_reason not in RECOVERABLE_MOTION_FAULTS
                ):
                    if not (uncertain and self._snapshot.state in {
                        PoseTransactionState.REACHED, PoseTransactionState.CANCELLED,
                    }):
                        self._snapshot = PoseTransactionSnapshot(
                            PoseTransactionState.RECONCILING if uncertain else PoseTransactionState.FAULT,
                            goal=goal, fault_reason=failure_reason,
                        )
            if uncertain:
                return  # 下一周期查询确认实际状态，不能盲目重发或覆盖已经收到的到位事件。
            raise

    def stop(self) -> None:
        """最高层安全停车；不等待响应，避免阻塞安全状态机。"""

        self.snapshot()
        if not getattr(self.link, "connected", True):
            return  # 断线由固件看门狗停车，重连后必须查询确认。
        self.link.send_command(Command.STOP_ALL)
        with self._lock:
            if self._snapshot.state in self._ACTIVE_STATES:
                if self._cancel_started_at is None:
                    self._cancel_started_at = self._clock()
                self._snapshot = PoseTransactionSnapshot(
                    PoseTransactionState.CANCELLING,
                    goal=self._snapshot.goal,
                )

    def query_status(self) -> PoseGoalStatus:
        response = self.link.request(
            Command.QUERY_POSE_GOAL,
            timeout=self._request_timeout,
        )
        return PoseGoalStatus.decode_response_data(response.data)

    def snapshot(self) -> PoseTransactionSnapshot:
        with self._lock:
            generation = getattr(self.link, "generation", 0)
            if generation != self._link_generation or not getattr(self.link, "connected", True):
                self._link_generation = generation
                if self._snapshot.state in self._ACTIVE_STATES:
                    self._snapshot = PoseTransactionSnapshot(
                        PoseTransactionState.FAULT, goal=self._snapshot.goal,
                        fault_reason=MotionFaultReason.HOST_LOST,
                    )
            if (
                self._snapshot.state == PoseTransactionState.CANCELLING
                and self._cancel_started_at is not None
                and self._clock() - self._cancel_started_at >= self._cancellation_timeout
            ):
                self._snapshot = PoseTransactionSnapshot(
                    PoseTransactionState.RECONCILING, goal=self._snapshot.goal,
                    fault_reason=MotionFaultReason.CANCEL_TIMEOUT,
                )
            return self._snapshot

    def recover_connection(self) -> bool:
        """核对重连、丢失应答及可恢复故障；确认旧目标状态后才能继续。"""
        snapshot = self.snapshot()
        if not getattr(self.link, "connected", True):
            return False
        generation = getattr(self.link, "generation", 0)
        new_session = generation != self._confirmed_generation
        if not new_session:
            if snapshot.state is PoseTransactionState.FAULT and snapshot.fault_reason in {
                MotionFaultReason.HOST_LOST, MotionFaultReason.USB_LINK_FAULT,
            }:
                self.link.request_reconnect()
                return False
            if snapshot.state is not PoseTransactionState.RECONCILING and not (
                snapshot.state is PoseTransactionState.FAULT
                and snapshot.fault_reason in RECOVERABLE_MOTION_FAULTS
            ):
                return True
        # 硬故障不能借重连或查询解除。
        if snapshot.state is PoseTransactionState.FAULT and snapshot.fault_reason not in RECOVERABLE_MOTION_FAULTS:
            return True
        try:
            status = self.query_status()
            same_goal = snapshot.goal is not None and status.goal_id == snapshot.goal.goal_id
            adopt_goal = (
                not new_session and same_goal
                and snapshot.fault_reason == MotionFaultReason.REQUEST_TIMEOUT
                and self._cancel_started_at is None
            )
            if status.state in {PoseGoalState.ACCEPTED, PoseGoalState.MOVING} and not adopt_goal:
                self.link.send_command(Command.STOP_ALL)
                return False
            if (
                not new_session and snapshot.goal is not None and not same_goal
                and status.state is not PoseGoalState.IDLE
                and not (status.state is PoseGoalState.FAULT and status.fault_reason not in RECOVERABLE_MOTION_FAULTS)
            ):
                return False
        except (SerialLinkError, ValueError):
            return False
        with self._lock:
            if not getattr(self.link, "connected", True) or generation != getattr(self.link, "generation", 0):
                return False
            if self._snapshot != snapshot:
                return False
            if status.state is PoseGoalState.FAULT and status.fault_reason not in RECOVERABLE_MOTION_FAULTS:
                self._snapshot = PoseTransactionSnapshot(
                    PoseTransactionState.FAULT, goal=snapshot.goal, fault_reason=status.fault_reason,
                )
            elif adopt_goal and status.state in {PoseGoalState.ACCEPTED, PoseGoalState.MOVING, PoseGoalState.REACHED}:
                state = {
                    PoseGoalState.ACCEPTED: PoseTransactionState.ACCEPTED,
                    PoseGoalState.MOVING: PoseTransactionState.MOVING,
                    PoseGoalState.REACHED: PoseTransactionState.REACHED,
                }[status.state]
                self._snapshot = PoseTransactionSnapshot(state, goal=snapshot.goal)
            else:
                self._snapshot = PoseTransactionSnapshot(
                    PoseTransactionState.CANCELLED, goal=snapshot.goal,
                    fault_reason=(
                        status.fault_reason if status.state is PoseGoalState.FAULT else snapshot.fault_reason
                    ),
                )
            self._cancel_started_at = None
            self._cancel_requested_goal_id = None
            self._confirmed_generation = generation
        return True

    def clear_terminal(self) -> None:
        with self._lock:
            if self._snapshot.terminal and not (
                self._snapshot.state is PoseTransactionState.FAULT
                and self._snapshot.fault_reason in RECOVERABLE_MOTION_FAULTS
            ):
                self._snapshot = PoseTransactionSnapshot(PoseTransactionState.IDLE)
                self._cancel_started_at = None
                self._cancel_requested_goal_id = None

    def is_active(self) -> bool:
        return bool(self._activity_reader and self._activity_reader())

    @property
    def commanded_motion_active(self) -> bool:
        return self.snapshot().state in self._ACTIVE_STATES

    @property
    def commanded_speed_mm_s(self) -> float:
        return self._maximum_speed_mm_s if self.commanded_motion_active else 0.0

    def _on_frame(self, frame: Frame) -> bool:
        if not frame.payload or frame.payload[0] not in {
            EventCode.POSE_STARTED,
            EventCode.POSE_REACHED,
            EventCode.POSE_CANCELLED,
            EventCode.MOTION_FAULT,
        }:
            return False
        try:
            event = decode_motion_event(frame.payload)
        except ValueError:
            self.invalid_events += 1
            return True
        with self._lock:
            goal = self._snapshot.goal
            if goal is None or event.goal_id != goal.goal_id:
                self.stale_events += 1
                return True
            if self._snapshot.state not in self._ACTIVE_STATES:
                return True
            if isinstance(event, PoseStarted):
                if self._snapshot.state == PoseTransactionState.ACCEPTED:
                    self._snapshot = PoseTransactionSnapshot(
                        PoseTransactionState.MOVING,
                        goal=goal,
                    )
            elif isinstance(event, PoseReached):
                self._snapshot = PoseTransactionSnapshot(
                    PoseTransactionState.REACHED, goal=goal, reached=event,
                )
            elif isinstance(event, PoseCancelled):
                self._snapshot = PoseTransactionSnapshot(PoseTransactionState.CANCELLED, goal=goal)
            elif isinstance(event, MotionFault):
                self._snapshot = PoseTransactionSnapshot(
                    PoseTransactionState.FAULT, goal=goal, fault_reason=event.reason,
                )
        return True
