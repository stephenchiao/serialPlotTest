"""树莓派与 STM32 之间的语义消息定义。

多字节整数统一使用小端序。COMMAND 的 payload 为 ``opcode + 参数``，
RESPONSE 的 payload 为 ``请求序号 + opcode + 状态码 + 返回数据``。
"""

from dataclasses import dataclass
from enum import IntEnum, IntFlag
import struct
from typing import ClassVar, Union


class MessageType(IntEnum):
    """协议帧的消息类型。"""

    COMMAND = 0x10
    RESPONSE = 0x11
    TELEMETRY = 0x23
    LEGACY_TELEMETRY = 0x20  # 仅用于旧离线数据，不是 v2 固件遥测
    EVENT = 0x22


class TelemetryKind(IntEnum):
    """TELEMETRY payload 的首字节，用于区分不同遥测数据。"""

    WHEEL = 0x01
    POSE = 0x02
    LINK_STATS = 0x03
    OPS9_POSE = 0x01  # 旧 19 字节格式，只能搭配 LEGACY_TELEMETRY


class Ops9Status(IntFlag):
    """STM32 上报的 OPS9 状态位。"""

    VALID = 0x01
    CALIBRATED = 0x02
    CONTACT_OK = 0x04
    FIRMWARE_MONITORED = 0x08  # v2 会话已验证就绪，传感器健康由 STM32 看门狗负责


class Command(IntEnum):
    """默认命令集；可以在 0x80~0xEF 范围添加项目命令。"""

    PING = 0x01
    STOP_ALL = 0x02
    SESSION_PROBE = 0x03
    SET_CHASSIS_VELOCITY = 0x10
    SET_SERVO_ANGLE = 0x20
    QUERY_STATUS = 0x30
    SET_TASK_CODE = 0x40
    SET_POSE_GOAL = 0x80
    CANCEL_POSE_GOAL = 0x81
    QUERY_POSE_GOAL = 0x82
    SET_SPEED_LIMITS = 0x83
    SET_POSE_GOAL_WITH_LIMITS = 0x84
    UPDATE_MATERIAL_VISION = 0x85


HOST_PROTOCOL_VERSION = 4


class Capability(IntFlag):
    """与队友 protocol/rpi_binary_protocol.json 一致的基础能力位。"""

    QUERY_POSE = 0x01
    SPEED_LIMITS = 0x02
    BINARY_TELEMETRY = 0x04
    ASYNC_TX = 0x08
    SESSION_RECOVERY = 0x10
    ATOMIC_POSE_LIMITS = 0x20


REQUIRED_CAPABILITIES = int(
    Capability.QUERY_POSE | Capability.SPEED_LIMITS | Capability.BINARY_TELEMETRY
    | Capability.ASYNC_TX | Capability.SESSION_RECOVERY | Capability.ATOMIC_POSE_LIMITS
)
MATERIAL_VISION_CAPABILITY = 0x40  # 可选扩展；只有接入物料接收模块的固件才声明
SUPPORTED_COMMANDS = frozenset({
    Command.PING, Command.STOP_ALL, Command.SESSION_PROBE,
    Command.SET_POSE_GOAL, Command.CANCEL_POSE_GOAL, Command.QUERY_POSE_GOAL,
    Command.SET_SPEED_LIMITS, Command.SET_POSE_GOAL_WITH_LIMITS,
    Command.UPDATE_MATERIAL_VISION,
})


class ResponseStatus(IntEnum):
    OK = 0x00
    UNKNOWN_COMMAND = 0x01
    INVALID_LENGTH = 0x02
    INVALID_ARGUMENT = 0x03
    BUSY = 0x04
    INTERNAL_ERROR = 0x05


class EventCode(IntEnum):
    START_BUTTON_PRESSED = 0x01
    ACTION_FINISHED = 0x02
    ACTION_FAILED = 0x03
    SAFETY_STOP = 0x04
    POSE_STARTED = 0x10
    POSE_REACHED = 0x11
    POSE_CANCELLED = 0x12
    MOTION_FAULT = 0x13


class PoseGoalState(IntEnum):
    """STM32 位姿事务状态；数值同时用于 QUERY_POSE_GOAL 响应。"""

    IDLE = 0x00
    ACCEPTED = 0x01
    MOVING = 0x02
    REACHED = 0x03
    CANCELLED = 0x04
    FAULT = 0x05


class MotionFaultReason(IntEnum):
    """STM32 本地停车原因；未知扩展值仍按原始整数保留。"""

    UNSPECIFIED = 0x0000
    OPS9_LOST = 0x0001
    HOST_LOST = 0x0002
    CAN_FAULT = 0x0003
    OUT_OF_BOUNDS = 0x0004
    TIMEOUT = 0x0005
    UART_FAULT = 0x0006  # 队友 v2 schema 的名称；原生 CDC 适配沿用此编号
    USB_LINK_FAULT = UART_FAULT  # 保留已有调用方名称
    CANCEL_TIMEOUT = 0x0100  # 树莓派本地故障，不与固件编号冲突
    REQUEST_TIMEOUT = 0x0101  # 应答不确定，先查询航点状态，不重发运动命令
    INTERNAL_ERROR = 0x00FF


FATAL_MOTION_FAULTS = frozenset({
    MotionFaultReason.CAN_FAULT,
    MotionFaultReason.OUT_OF_BOUNDS,
    MotionFaultReason.INTERNAL_ERROR,
    MotionFaultReason.HOST_LOST,
})

RECOVERABLE_MOTION_FAULTS = frozenset({
    MotionFaultReason.OPS9_LOST,
    MotionFaultReason.TIMEOUT,
    MotionFaultReason.HOST_LOST,
    MotionFaultReason.USB_LINK_FAULT,
    MotionFaultReason.CANCEL_TIMEOUT,
    MotionFaultReason.REQUEST_TIMEOUT,
})


@dataclass(frozen=True)
class Response:
    """STM32 对 COMMAND 的响应。"""

    request_sequence: int
    command: int
    status: ResponseStatus
    data: bytes = b""

    _PREFIX: ClassVar[struct.Struct] = struct.Struct("<BBB")

    def encode(self) -> bytes:
        return self._PREFIX.pack(
            self.request_sequence,
            self.command,
            int(self.status),
        ) + self.data

    @classmethod
    def decode(cls, payload: bytes) -> "Response":
        if len(payload) < cls._PREFIX.size:
            raise ValueError("RESPONSE payload 至少需要 3 字节")
        sequence, command, raw_status = cls._PREFIX.unpack_from(payload)
        try:
            status = ResponseStatus(raw_status)
        except ValueError as error:
            raise ValueError(f"未知响应状态码：0x{raw_status:02X}") from error
        return cls(sequence, command, status, payload[cls._PREFIX.size :])


@dataclass(frozen=True)
class Ops9Pose:
    """STM32 转发的 OPS9 平面位姿。

    坐标单位为毫米，航向单位为毫弧度。旧数据 quality 为 0~100；v2 没有
    quality，因此为 None。v2 timestamp_ms 是遥测 tick，不是 OPS9 更新计数；
    FIRMWARE_MONITORED 表示依赖已握手的固件健康看门狗，不表示标定完成。
    """

    x_mm: int
    y_mm: int
    yaw_mrad: int
    timestamp_ms: int
    quality: int | None
    status: Ops9Status

    _STRUCT: ClassVar[struct.Struct] = struct.Struct("<iiiIBB")

    @property
    def valid(self) -> bool:
        required = Ops9Status.VALID | Ops9Status.CALIBRATED | Ops9Status.CONTACT_OK
        if self.status & Ops9Status.FIRMWARE_MONITORED:
            return True
        return (self.status & required) == required and self.quality is not None and self.quality > 0

    def encode_telemetry(self) -> bytes:
        """仅编码旧格式离线夹具；v2 遥测必须使用 kind=2/<IH8i>。"""
        if not 0 <= self.timestamp_ms <= 0xFFFFFFFF:
            raise ValueError("timestamp_ms 必须在 0~4294967295 范围内")
        if self.quality is None or not 0 <= self.quality <= 100:
            raise ValueError("quality 必须在 0~100 范围内")
        try:
            body = self._STRUCT.pack(
                self.x_mm,
                self.y_mm,
                self.yaw_mrad,
                self.timestamp_ms,
                self.quality,
                int(self.status),
            )
        except struct.error as error:
            raise ValueError("OPS9 坐标、航向或状态超出协议整数范围") from error
        return bytes((TelemetryKind.OPS9_POSE,)) + body

    @classmethod
    def decode_telemetry(cls, payload: bytes) -> "Ops9Pose":
        expected = 1 + cls._STRUCT.size
        if len(payload) != expected:
            raise ValueError(f"OPS9 TELEMETRY payload 应为 {expected} 字节")
        if payload[0] != TelemetryKind.OPS9_POSE:
            raise ValueError(f"不是 OPS9 遥测：0x{payload[0]:02X}")
        x_mm, y_mm, yaw_mrad, timestamp_ms, quality, raw_status = (
            cls._STRUCT.unpack_from(payload, 1)
        )
        if quality > 100:
            raise ValueError(f"OPS9 quality 超出 0~100：{quality}")
        return cls(
            x_mm=x_mm,
            y_mm=y_mm,
            yaw_mrad=yaw_mrad,
            timestamp_ms=timestamp_ms,
            quality=quality,
            status=Ops9Status(raw_status),
        )


@dataclass(frozen=True)
class PoseGoal:
    """树莓派提交给 STM32 的绝对 OPS9 位姿目标。"""

    goal_id: int
    x_mm: int
    y_mm: int
    yaw_mrad: int
    timeout_ms: int

    _STRUCT: ClassVar[struct.Struct] = struct.Struct("<IiiiI")

    def encode_command_data(self) -> bytes:
        if not 1 <= self.goal_id <= 0xFFFFFFFF:
            raise ValueError("goal_id 必须在 1~4294967295 范围内")
        if not 1 <= self.timeout_ms <= 60000:
            raise ValueError("timeout_ms 必须在 1~60000 范围内")
        try:
            return self._STRUCT.pack(
                self.goal_id,
                self.x_mm,
                self.y_mm,
                self.yaw_mrad,
                self.timeout_ms,
            )
        except struct.error as error:
            raise ValueError("位姿目标坐标或航向超出有符号 32 位范围") from error

    @classmethod
    def decode_command_data(cls, data: bytes) -> "PoseGoal":
        if len(data) != cls._STRUCT.size:
            raise ValueError(f"SET_POSE_GOAL data 应为 {cls._STRUCT.size} 字节")
        goal = cls(*cls._STRUCT.unpack(data))
        if goal.goal_id == 0 or not 1 <= goal.timeout_ms <= 60000:
            raise ValueError("goal_id 不能为 0，timeout_ms 必须在 1~60000")
        return goal


@dataclass(frozen=True)
class PoseGoalStatus:
    """QUERY_POSE_GOAL 的固定长度响应。"""

    goal_id: int
    state: PoseGoalState
    x_mm: int
    y_mm: int
    yaw_mrad: int
    fault_reason: int = MotionFaultReason.UNSPECIFIED
    robot_mode: int = 0
    host_link: int = 0

    _STRUCT: ClassVar[struct.Struct] = struct.Struct("<IBiiiHBB")

    def encode_response_data(self) -> bytes:
        if not 0 <= self.goal_id <= 0xFFFFFFFF:
            raise ValueError("goal_id 必须为无符号 32 位整数")
        if not 0 <= int(self.fault_reason) <= 0xFFFF:
            raise ValueError("fault_reason 必须为无符号 16 位整数")
        try:
            return self._STRUCT.pack(
                self.goal_id,
                int(self.state),
                self.x_mm,
                self.y_mm,
                self.yaw_mrad,
                int(self.fault_reason),
                self.robot_mode,
                self.host_link,
            )
        except struct.error as error:
            raise ValueError("位姿状态字段超出协议整数范围") from error

    @classmethod
    def decode_response_data(cls, data: bytes) -> "PoseGoalStatus":
        if len(data) != cls._STRUCT.size:
            raise ValueError(f"QUERY_POSE_GOAL 响应应为 {cls._STRUCT.size} 字节")
        goal_id, raw_state, x_mm, y_mm, yaw_mrad, fault_reason, mode, host = cls._STRUCT.unpack(data)
        try:
            state = PoseGoalState(raw_state)
        except ValueError as error:
            raise ValueError(f"未知位姿状态：0x{raw_state:02X}") from error
        return cls(goal_id, state, x_mm, y_mm, yaw_mrad, fault_reason, mode, host)


@dataclass(frozen=True)
class SessionInfo:
    active: int
    armed: int
    host_link: int
    pose_state: int
    goal_id: int
    capabilities: int
    version: int

    @classmethod
    def decode(cls, data: bytes) -> "SessionInfo":
        if len(data) != 13:
            raise ValueError("SESSION_PROBE 响应必须为 13 字节")
        info = cls(*struct.unpack("<BBBBIIB", data))
        if info.active not in (0, 1) or info.armed not in (0, 1):
            raise ValueError("SESSION_PROBE 会话状态非法")
        return info


@dataclass(frozen=True)
class PoseSample:
    tick_ms: int
    sequence: int
    ops_x_mm: int
    ops_y_mm: int
    ops_yaw_mrad: int
    center_x_mm: int
    center_y_mm: int
    plan_vx_um_s: int
    plan_vy_um_s: int
    plan_vz_urad_s: int


@dataclass(frozen=True)
class WheelSample:
    tick_ms: int
    sequence: int
    target_rpm_tenths: tuple[int, ...]
    actual_rpm_tenths: tuple[int, ...]


@dataclass(frozen=True)
class LinkStatsSample:
    tick_ms: int
    rx_dropped: int
    tx_dropped: int
    telemetry_replaced: int
    crc_errors: int
    transport_errors: int


def decode_telemetry(payload: bytes) -> PoseSample | WheelSample | LinkStatsSample:
    if not payload:
        raise ValueError("遥测不能为空")
    if payload[0] in (TelemetryKind.WHEEL, TelemetryKind.POSE):
        if len(payload) != 39:
            raise ValueError("轮速/位姿遥测必须为 39 字节")
        values = struct.unpack_from("<IH8i", payload, 1)
        if payload[0] == TelemetryKind.POSE:
            return PoseSample(*values)
        return WheelSample(values[0], values[1], values[2:6], values[6:10])
    if payload[0] == TelemetryKind.LINK_STATS and len(payload) == 25:
        return LinkStatsSample(*struct.unpack_from("<6I", payload, 1))
    raise ValueError("未知遥测类型或长度错误")


def encode_speed_limits(linear_mm_s: float, yaw_mrad_s: float) -> bytes:
    if not 20 <= linear_mm_s <= 300 or not 20 <= yaw_mrad_s <= 800:
        raise ValueError("线速度限幅必须为 20~300 mm/s，角速度为 20~800 mrad/s")
    return struct.pack("<ii", round(linear_mm_s * 1000), round(yaw_mrad_s * 1000))


@dataclass(frozen=True)
class PoseStarted:
    goal_id: int

    def encode_event(self) -> bytes:
        return bytes((EventCode.POSE_STARTED,)) + _encode_goal_id(self.goal_id)


@dataclass(frozen=True)
class PoseReached:
    goal_id: int
    x_mm: int
    y_mm: int
    yaw_mrad: int
    position_error_mm: int
    yaw_error_mrad: int

    _STRUCT: ClassVar[struct.Struct] = struct.Struct("<Iiiiii")

    def encode_event(self) -> bytes:
        try:
            body = self._STRUCT.pack(
                self.goal_id,
                self.x_mm,
                self.y_mm,
                self.yaw_mrad,
                self.position_error_mm,
                self.yaw_error_mrad,
            )
        except struct.error as error:
            raise ValueError("POSE_REACHED 字段超出协议整数范围") from error
        if self.goal_id == 0:
            raise ValueError("goal_id 不能为 0")
        return bytes((EventCode.POSE_REACHED,)) + body


@dataclass(frozen=True)
class PoseCancelled:
    goal_id: int

    def encode_event(self) -> bytes:
        return bytes((EventCode.POSE_CANCELLED,)) + _encode_goal_id(self.goal_id)


@dataclass(frozen=True)
class MotionFault:
    goal_id: int
    reason: int

    _STRUCT: ClassVar[struct.Struct] = struct.Struct("<IH")

    def encode_event(self) -> bytes:
        if not 1 <= self.goal_id <= 0xFFFFFFFF:
            raise ValueError("goal_id 必须在 1~4294967295 范围内")
        if not 0 <= int(self.reason) <= 0xFFFF:
            raise ValueError("reason 必须为无符号 16 位整数")
        return bytes((EventCode.MOTION_FAULT,)) + self._STRUCT.pack(
            self.goal_id, int(self.reason)
        )


MotionEvent = Union[PoseStarted, PoseReached, PoseCancelled, MotionFault]


def decode_motion_event(payload: bytes) -> MotionEvent:
    """解析位姿事务 EVENT；其他事件码由调用方继续分发。"""

    if not payload:
        raise ValueError("EVENT payload 不能为空")
    try:
        code = EventCode(payload[0])
    except ValueError as error:
        raise ValueError(f"未知事件码：0x{payload[0]:02X}") from error
    body = payload[1:]
    if code == EventCode.POSE_STARTED:
        return PoseStarted(_decode_goal_id(body, code))
    if code == EventCode.POSE_CANCELLED:
        return PoseCancelled(_decode_goal_id(body, code))
    if code == EventCode.POSE_REACHED:
        if len(body) != PoseReached._STRUCT.size:
            raise ValueError(
                f"POSE_REACHED body 应为 {PoseReached._STRUCT.size} 字节"
            )
        return PoseReached(*PoseReached._STRUCT.unpack(body))
    if code == EventCode.MOTION_FAULT:
        if len(body) != MotionFault._STRUCT.size:
            raise ValueError(
                f"MOTION_FAULT body 应为 {MotionFault._STRUCT.size} 字节"
            )
        return MotionFault(*MotionFault._STRUCT.unpack(body))
    raise ValueError(f"不是位姿事务事件：{code.name}")


def encode_cancel_pose_goal(goal_id: int) -> bytes:
    return _encode_goal_id(goal_id)


def _encode_goal_id(goal_id: int) -> bytes:
    if not 1 <= goal_id <= 0xFFFFFFFF:
        raise ValueError("goal_id 必须在 1~4294967295 范围内")
    return struct.pack("<I", goal_id)


def _decode_goal_id(data: bytes, code: EventCode) -> int:
    if len(data) != 4:
        raise ValueError(f"{code.name} body 应为 4 字节")
    goal_id = struct.unpack("<I", data)[0]
    if goal_id == 0:
        raise ValueError(f"{code.name} goal_id 不能为 0")
    return goal_id


def encode_command(command: int, data: bytes = b"") -> bytes:
    if not 0 <= int(command) <= 0xFF:
        raise ValueError("command 必须在 0~255 范围内")
    return bytes((int(command),)) + bytes(data)


def decode_command(payload: bytes) -> tuple[int, bytes]:
    if not payload:
        raise ValueError("COMMAND payload 不能为空")
    return payload[0], payload[1:]


def encode_chassis_velocity(vx_mm_s: int, vy_mm_s: int, wz_mrad_s: int) -> bytes:
    """编码三轴速度，范围均为有符号 16 位整数。"""

    try:
        return struct.pack("<hhh", vx_mm_s, vy_mm_s, wz_mrad_s)
    except struct.error as error:
        raise ValueError("速度值必须在 -32768~32767 范围内") from error


def encode_servo_angle(servo_id: int, angle_tenths_degree: int) -> bytes:
    """编码舵机编号和 0.1 度单位的目标角度。"""

    if not 0 <= servo_id <= 0xFF:
        raise ValueError("servo_id 必须在 0~255 范围内")
    try:
        return struct.pack("<Bh", servo_id, angle_tenths_degree)
    except struct.error as error:
        raise ValueError("angle_tenths_degree 必须为有符号 16 位整数") from error
