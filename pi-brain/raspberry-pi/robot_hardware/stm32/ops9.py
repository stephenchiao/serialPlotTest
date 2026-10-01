"""从 STM32 TELEMETRY 帧取得 OPS9 位姿。"""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time
from typing import Callable, Optional

from .messages import (EventCode, MessageType, MotionFault, MotionFaultReason,
                       Ops9Pose, Ops9Status, TelemetryKind, PoseSample,
                       RECOVERABLE_MOTION_FAULTS, decode_motion_event, decode_telemetry)
from .protocol import Frame
from .serial_link import SerialLink


@dataclass(frozen=True)
class TimedOps9Pose:
    pose: Ops9Pose
    received_at: float


class Stm32Ops9Receiver:
    """线程安全的最新位姿缓冲器。

    ``attach`` 后由 ``SerialLink`` 接收线程更新，不另外读取串口，因此不会与
    RESPONSE、EVENT 消费者竞争。导航循环通过 ``latest`` 读取快照。
    """

    def __init__(
        self,
        link: SerialLink,
        *,
        stale_after_seconds: float = 0.25,
        minimum_quality: int = 30,
        maximum_position_jump_mm: float = 300.0,
        maximum_yaw_jump_mrad: int = 800,
        movement_threshold_mm: float = 3.0,
        movement_threshold_mrad: int = 10,
        movement_hold_seconds: float = 0.2,
        clock: Callable[[], float] = time.monotonic,
        legacy_telemetry: bool = False,
    ) -> None:
        if stale_after_seconds <= 0:
            raise ValueError("stale_after_seconds 必须大于 0")
        if not 0 <= minimum_quality <= 100:
            raise ValueError("minimum_quality 必须在 0~100 范围内")
        if maximum_position_jump_mm <= 0 or maximum_yaw_jump_mrad <= 0:
            raise ValueError("OPS9 跳变门限必须大于 0")
        if (
            movement_threshold_mm <= 0
            or movement_threshold_mrad <= 0
            or movement_hold_seconds <= 0
        ):
            raise ValueError("OPS9 物理活动判断门限必须大于 0")
        self._link = link
        self.stale_after_seconds = stale_after_seconds
        self.minimum_quality = minimum_quality
        self.maximum_position_jump_mm = maximum_position_jump_mm
        self.maximum_yaw_jump_mrad = maximum_yaw_jump_mrad
        self.movement_threshold_mm = movement_threshold_mm
        self.movement_threshold_mrad = movement_threshold_mrad
        self.movement_hold_seconds = movement_hold_seconds
        self._clock = clock
        self._legacy_telemetry = legacy_telemetry
        self._generation = link.generation
        self._firmware_faulted = False
        self._motion_fault: Optional[MotionFault] = None
        self._recovery_samples = 0
        self._lock = threading.Lock()
        self._latest: Optional[TimedOps9Pose] = None
        self._attached = False
        self.invalid_frames = 0
        self.jump_frames = 0
        self._last_motion_at: Optional[float] = None

    def attach(self) -> None:
        if self._attached:
            return
        self._link.add_frame_handler(MessageType.TELEMETRY, self._on_frame)
        self._link.add_frame_handler(MessageType.EVENT, self._on_event)
        if self._legacy_telemetry:
            self._link.add_frame_handler(MessageType.LEGACY_TELEMETRY, self._on_frame)
        self._attached = True

    def detach(self) -> None:
        if not self._attached:
            return
        self._link.remove_frame_handler(MessageType.TELEMETRY, self._on_frame)
        self._link.remove_frame_handler(MessageType.EVENT, self._on_event)
        if self._legacy_telemetry:
            self._link.remove_frame_handler(MessageType.LEGACY_TELEMETRY, self._on_frame)
        self._attached = False

    def clear(self) -> None:
        with self._lock:
            self._latest = None
            self._last_motion_at = None
            self._recovery_samples = 0

    @property
    def motion_fault(self) -> Optional[MotionFault]:
        with self._lock:
            return self._motion_fault

    def recover_fault(self, goal_id: int, reason: int) -> bool:
        """调用方已查询确认旧航点停止；还需两帧新鲜位姿才能解除定位故障锁。"""
        with self._lock:
            fault = self._motion_fault
            if fault is None:
                return True
            if (
                fault.goal_id != goal_id or fault.reason != reason
                or reason not in {MotionFaultReason.OPS9_LOST, MotionFaultReason.TIMEOUT}
                or not self._link.connected or self._generation != self._link.generation
                or self._latest is None or self._recovery_samples < 2
                or not 0 <= self._clock() - self._latest.received_at <= self.stale_after_seconds
            ):
                return False
            self._firmware_faulted = False
            self._motion_fault = None
            self._recovery_samples = 0
            return True

    def latest_sample(self) -> Optional[TimedOps9Pose]:
        with self._lock:
            return self._latest

    def latest(self) -> Optional[Ops9Pose]:
        with self._lock:
            sample = self._latest
            if sample is None or not 0 <= self._clock() - sample.received_at <= self.stale_after_seconds:
                return None
            if sample.pose.status & Ops9Status.FIRMWARE_MONITORED:
                if not self._link.connected or self._generation != self._link.generation or self._firmware_faulted:
                    return None
            return sample.pose  # 入缓冲前已经检查有效标记、质量和跳变。

    def _sync_generation(self) -> None:
        """在持锁时隔离旧会话数据；新会话仍保留硬故障锁。"""
        if self._generation == self._link.generation:
            return
        self._generation = self._link.generation
        self._latest = None
        self._last_motion_at = None
        if self._motion_fault is None or self._motion_fault.reason in RECOVERABLE_MOTION_FAULTS:
            self._firmware_faulted = False
            self._motion_fault = None
        self._recovery_samples = 0

    def is_moving(self) -> bool:
        with self._lock:
            last_motion_at = self._last_motion_at
        return (
            last_motion_at is not None
            and self._clock() - last_motion_at <= self.movement_hold_seconds
        )

    def _on_frame(self, frame: Frame) -> bool:
        legacy = frame.message_type == MessageType.LEGACY_TELEMETRY
        expected_kind = TelemetryKind.OPS9_POSE if legacy else TelemetryKind.POSE
        if not frame.payload or frame.payload[0] != expected_kind:
            return False
        try:
            if legacy:
                if not self._legacy_telemetry:
                    return False
                pose = Ops9Pose.decode_telemetry(frame.payload)
            else:
                sample = decode_telemetry(frame.payload)
                assert isinstance(sample, PoseSample)
                # v2 没有质量及标定状态；不能伪造 100% 质量和传感器健康位。
                pose = Ops9Pose(sample.ops_x_mm, sample.ops_y_mm, sample.ops_yaw_mrad,
                                sample.tick_ms, None, Ops9Status.FIRMWARE_MONITORED)
        except ValueError:
            self.invalid_frames += 1
            with self._lock:
                self._recovery_samples = 0
            return True
        if not pose.valid or (pose.quality is not None and pose.quality < self.minimum_quality):
            self.invalid_frames += 1
            with self._lock:
                self._recovery_samples = 0
            return True
        now = self._clock()
        with self._lock:
            self._sync_generation()
            previous = self._latest
            if previous is not None:
                timestamp_delta = (
                    pose.timestamp_ms - previous.pose.timestamp_ms
                ) & 0xFFFFFFFF
                if timestamp_delta == 0:
                    return True
                if timestamp_delta >= 0x80000000:
                    self.invalid_frames += 1
                    self._recovery_samples = 0
                    return True
                position_jump = math.hypot(
                    pose.x_mm - previous.pose.x_mm,
                    pose.y_mm - previous.pose.y_mm,
                )
                yaw_jump = abs(
                    _wrapped_mrad(pose.yaw_mrad - previous.pose.yaw_mrad)
                )
                if (
                    position_jump > self.maximum_position_jump_mm
                    or yaw_jump > self.maximum_yaw_jump_mrad
                ):
                    self.jump_frames += 1
                    self._recovery_samples = 0
                    return True
                if (
                    position_jump >= self.movement_threshold_mm
                    or yaw_jump >= self.movement_threshold_mrad
                ):
                    self._last_motion_at = now
            if self._firmware_faulted:
                self._recovery_samples = (
                    self._recovery_samples + 1
                    if previous is not None and now - previous.received_at <= self.stale_after_seconds
                    else 1
                )
            self._latest = TimedOps9Pose(pose, now)
        return True

    def _on_event(self, frame: Frame) -> bool:
        if frame.payload and frame.payload[0] == EventCode.MOTION_FAULT:
            try:
                fault = decode_motion_event(frame.payload)
            except ValueError:
                self.invalid_frames += 1
                return False
            with self._lock:
                self._sync_generation()
                if self._motion_fault is not None and self._motion_fault.reason not in RECOVERABLE_MOTION_FAULTS:
                    return False  # 后续定位/通信故障不能覆盖尚未处理的硬故障。
                self._firmware_faulted = True
                self._motion_fault = fault
                self._recovery_samples = 0
                self._latest = None
                self._last_motion_at = None
        return False  # 不抢占位姿事务控制器的事件


def _wrapped_mrad(angle: int) -> int:
    full_turn = int(round(2.0 * math.pi * 1000.0))
    half_turn = full_turn // 2
    return (angle + half_turn) % full_turn - half_turn
