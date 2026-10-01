"""cam0 视觉观测发送给 STM32；不在树莓派上执行纠偏或抓取。

0x85 命令的数据固定为 47 字节，小端序。编号未配置、目标丢失或观测过期
时仍发送状态，但清空位置。ACK 只表示接收，不表示对准/抓取已经完成。
"""

from dataclasses import dataclass, replace
from enum import IntEnum, IntFlag
import logging
from math import ceil, isfinite
import secrets
import struct
import time
from typing import ClassVar

from .messages import Command
from .serial_link import SerialLinkError


LOGGER = logging.getLogger(__name__)
NO_CLASS_ID = 0xFFFF


class MaterialVisionBackend(IntEnum):
    COLOR = 1
    MODEL = 2


class MaterialVisionStatus(IntEnum):
    SEARCHING = 0
    CONFIRMING = 1
    TRACKING = 2  # 不区分 ALIGNING / READY；STM32 自己判断对准与动作条件
    MODEL_NOT_CONFIGURED = 3
    MAPPING_NOT_CONFIGURED = 4
    UNMAPPED_CLASS = 5
    TARGET_NOT_MAPPED = 6
    TARGET_NOT_FOUND = 7
    AMBIGUOUS = 8
    INFERENCE_ERROR = 9
    CAMERA_ERROR = 10
    STOPPED = 11
    HOLD = 12
    STALE = 13


class MaterialVisionFlags(IntFlag):
    VISIBLE = 0x01
    CONFIRMED = 0x02


_STATUS_MAP = {item.name: item for item in MaterialVisionStatus}
_STATUS_MAP.update({
    "CONFIRMING_COLOR": MaterialVisionStatus.CONFIRMING,
    "CONFIRMING_MATERIAL": MaterialVisionStatus.CONFIRMING,
    "ALIGNING": MaterialVisionStatus.TRACKING,
    "READY": MaterialVisionStatus.TRACKING,
})


@dataclass(frozen=True)
class MaterialVisionPacket:
    session_id: int
    frame_id: int
    capture_tick_ms: int
    valid_for_ms: int
    backend: MaterialVisionBackend
    status: MaterialVisionStatus
    frame_width: int
    frame_height: int
    flags: MaterialVisionFlags = MaterialVisionFlags(0)
    target_material_code: int = 0  # 0 表示自动选择，不是有效物料编号
    material_code: int = 0
    class_id: int = NO_CLASS_ID
    confidence_permille: int = 0
    center_x: int = 0
    center_y: int = 0
    offset_x_tenths: int = 0  # 0.1 像素；正值向图像右侧
    offset_y_tenths: int = 0  # 0.1 像素；正值向图像下方
    box_x: int = 0
    box_y: int = 0
    box_width: int = 0
    box_height: int = 0
    camera_num: int = 0
    schema_version: int = 1

    _STRUCT: ClassVar[struct.Struct] = struct.Struct("<BBBBBIII9H2h4H")
    _FIELDS: ClassVar[tuple[str, ...]] = (
        "schema_version", "camera_num", "backend", "status", "flags",
        "session_id", "frame_id", "capture_tick_ms", "valid_for_ms",
        "target_material_code", "material_code", "class_id", "confidence_permille",
        "frame_width", "frame_height", "center_x", "center_y",
        "offset_x_tenths", "offset_y_tenths", "box_x", "box_y", "box_width", "box_height",
    )

    def validate(self) -> None:
        for name in self._FIELDS:
            if not isinstance(getattr(self, name), int) or isinstance(getattr(self, name), bool):
                raise ValueError(f"{name} 必须为协议整数")
        if self.schema_version != 1 or self.camera_num != 0:
            raise ValueError("物料数据格式必须为 1，摄像头必须为 cam0")
        MaterialVisionBackend(self.backend)
        MaterialVisionStatus(self.status)
        if int(self.flags) & ~3:
            raise ValueError("未知物料视觉标志")
        if not 1 <= self.session_id <= 0xFFFFFFFF or not 1 <= self.frame_id <= 0xFFFFFFFF:
            raise ValueError("session_id / frame_id 必须为非零 u32")
        if not 1 <= self.valid_for_ms <= 1000:
            raise ValueError("valid_for_ms 必须为 1~1000 ms")
        if not 1 <= self.frame_width <= 0xFFFF or not 1 <= self.frame_height <= 0xFFFF:
            raise ValueError("图像尺寸必须为非零 u16")
        if not 0 <= self.confidence_permille <= 1000:
            raise ValueError("置信度必须为 0~1000")
        visible = bool(self.flags & MaterialVisionFlags.VISIBLE)
        confirmed = bool(self.flags & MaterialVisionFlags.CONFIRMED)
        if visible:
            if self.status not in (MaterialVisionStatus.CONFIRMING, MaterialVisionStatus.TRACKING):
                raise ValueError("此状态不能携带有效位置")
            if confirmed != (self.status == MaterialVisionStatus.TRACKING):
                raise ValueError("确认标志与识别状态不一致")
            if not 1 <= self.material_code <= 0xFFFF:
                raise ValueError("有效物料编号必须为 1~65535")
            if self.target_material_code and self.material_code != self.target_material_code:
                raise ValueError("物料编号与请求目标不匹配")
            if not 0 <= self.center_x < self.frame_width or not 0 <= self.center_y < self.frame_height:
                raise ValueError("物料中心超出图像")
            if (self.box_x < 0 or self.box_y < 0 or self.box_width <= 0 or self.box_height <= 0
                or self.box_x + self.box_width > self.frame_width
                or self.box_y + self.box_height > self.frame_height):
                raise ValueError("检测框超出图像或为空")
            if self.backend == MaterialVisionBackend.MODEL and self.class_id == NO_CLASS_ID:
                raise ValueError("有效模型观测缺少 class_id")
            if self.backend == MaterialVisionBackend.COLOR and self.class_id != NO_CLASS_ID:
                raise ValueError("颜色后端不使用模型 class_id")
        else:
            if self.flags or self.status in (MaterialVisionStatus.CONFIRMING, MaterialVisionStatus.TRACKING):
                raise ValueError("无目标时状态或标志不一致")
            if self.class_id != NO_CLASS_ID or any((
                self.material_code, self.confidence_permille, self.center_x, self.center_y,
                self.offset_x_tenths, self.offset_y_tenths,
                self.box_x, self.box_y, self.box_width, self.box_height,
            )):
                raise ValueError("无有效目标时必须清空旧编号与位置")

    def encode_command_data(self) -> bytes:
        self.validate()
        try:
            return self._STRUCT.pack(*(getattr(self, name) for name in self._FIELDS))
        except struct.error as error:
            raise ValueError("物料字段超出协议整数范围；不截断或回绕编号/坐标") from error

    @classmethod
    def decode_command_data(cls, data: bytes) -> "MaterialVisionPacket":
        if len(data) != cls._STRUCT.size:
            raise ValueError("UPDATE_MATERIAL_VISION data 必须为 47 字节")
        packet = cls(**dict(zip(cls._FIELDS, cls._STRUCT.unpack(data))))
        packet.encode_command_data()
        return packet

    @classmethod
    def from_detection(cls, detection, *, frame_size, session_id, frame_id,
                       captured_at, valid_for_ms):
        width, height = frame_size
        backend = MaterialVisionBackend.MODEL if detection.backend == "model" else MaterialVisionBackend.COLOR
        status = _STATUS_MAP.get(detection.status, MaterialVisionStatus.HOLD)
        packet = cls(
            session_id=session_id, frame_id=frame_id,
            capture_tick_ms=int(captured_at * 1000) & 0xFFFFFFFF,
            valid_for_ms=valid_for_ms, backend=backend, status=status,
            frame_width=int(width), frame_height=int(height),
            target_material_code=detection.target_material_code or 0,
        )
        observation = detection.observation
        if observation is not None and status in (MaterialVisionStatus.CONFIRMING, MaterialVisionStatus.TRACKING):
            if not isfinite(observation.confidence) or not 0 <= observation.confidence <= 1:
                raise ValueError("观测置信度必须为有限的 0~1")
            if not all(isfinite(value) for value in observation.offset_pixels):
                raise ValueError("位置偏差必须为有限数字")
            confirmed = bool(observation.confirmed)
            packet = replace(
                packet, status=MaterialVisionStatus.TRACKING if confirmed else MaterialVisionStatus.CONFIRMING,
                flags=MaterialVisionFlags.VISIBLE | (MaterialVisionFlags.CONFIRMED if confirmed else 0),
                material_code=observation.material_code,
                class_id=observation.class_id if observation.class_id is not None else NO_CLASS_ID,
                confidence_permille=round(observation.confidence * 1000),
                center_x=observation.center[0], center_y=observation.center[1],
                offset_x_tenths=round(observation.offset_pixels[0] * 10),
                offset_y_tenths=round(observation.offset_pixels[1] * 10),
                box_x=observation.box[0], box_y=observation.box[1],
                box_width=observation.box[2], box_height=observation.box[3],
            )
        elif status in (MaterialVisionStatus.CONFIRMING, MaterialVisionStatus.TRACKING):
            packet = replace(packet, status=MaterialVisionStatus.HOLD)
        packet.encode_command_data()
        return packet


class Stm32MaterialVisionPublisher:
    """复用已有 SerialLink，限频发送新帧；不缓存或重放位置，也不发动作指令。"""

    def __init__(self, link, *, maximum_rate_hz=10.0, valid_for_ms=250,
                 command_timeout_seconds=0.2, clock=time.monotonic, session_id_factory=None):
        if not isfinite(maximum_rate_hz) or not 0 < maximum_rate_hz <= 30:
            raise ValueError("maximum_rate_hz 必须为 0~30（不含 0）")
        if not isinstance(valid_for_ms, int) or isinstance(valid_for_ms, bool) or not 1 <= valid_for_ms <= 1000:
            raise ValueError("valid_for_ms 必须为 1~1000 的整数")
        if not isfinite(command_timeout_seconds) or not 0 < command_timeout_seconds <= valid_for_ms / 1000:
            raise ValueError("应答超时必须为正数且不超过视觉有效期")
        self.link = link
        self.maximum_rate_hz = maximum_rate_hz
        self.valid_for_ms = valid_for_ms
        self.command_timeout_seconds = command_timeout_seconds
        self.clock = clock
        self._new_session_id = session_id_factory or (lambda: secrets.randbelow(0xFFFFFFFF) + 1)
        self._generation = None
        self._session_id = 0
        self._frame_id = 0
        self._last_sent_at = float("-inf")
        self._last_key = None
        self._frame_size = (640, 480)
        self._backend = MaterialVisionBackend.MODEL
        self._target_code = 0
        self.latest_packet = None

    @classmethod
    def from_config(cls, link, config):
        return cls(link, **{name: config[name] for name in (
            "maximum_rate_hz", "valid_for_ms", "command_timeout_seconds",
        ) if name in config})

    def _next_frame(self):
        if self._generation != self.link.generation:
            self._generation = self.link.generation
            self._session_id = self._new_session_id()
            self._frame_id = 0
            self._last_key = None
            self._last_sent_at = float("-inf")
        # 0 保留；接收端使用 u32 差值比较，允许自然回绕。
        return self._session_id, (self._frame_id % 0xFFFFFFFF) + 1

    def publish(self, detection, *, frame_size, captured_at):
        now = self.clock()
        if not isfinite(captured_at) or captured_at < 0 or captured_at > now:
            raise ValueError("captured_at 必须是当前 monotonic 时钟中的有效采图时间")
        session_id, frame_id = self._next_frame()
        self._frame_size = frame_size
        self._backend = MaterialVisionBackend.MODEL if detection.backend == "model" else MaterialVisionBackend.COLOR
        self._target_code = detection.target_material_code or 0
        remaining_ms = self.valid_for_ms - ceil((now - captured_at) * 1000)
        if remaining_ms <= 0:
            packet = self._invalid_packet(MaterialVisionStatus.STALE, now, session_id, frame_id)
        else:
            packet = MaterialVisionPacket.from_detection(
                detection, frame_size=frame_size, session_id=session_id, frame_id=frame_id,
                captured_at=captured_at, valid_for_ms=remaining_ms,
            )
        key = (packet.status, packet.flags, packet.target_material_code, packet.material_code)
        # 状态/目标变化（尤其丢失）不等待下一个限频周期。
        if key == self._last_key and now - self._last_sent_at < 1 / self.maximum_rate_hz:
            return None
        return self._send(packet, now, key)

    def _invalid_packet(self, status, now, session_id, frame_id):
        return MaterialVisionPacket(
            session_id=session_id, frame_id=frame_id, capture_tick_ms=int(now * 1000) & 0xFFFFFFFF,
            valid_for_ms=self.valid_for_ms, backend=self._backend, status=status,
            frame_width=self._frame_size[0], frame_height=self._frame_size[1],
            target_material_code=self._target_code,
        )

    def _send(self, packet, now, key):
        data = packet.encode_command_data()
        self._frame_id = packet.frame_id  # 失败也不复用/重发旧帧
        self.link.request(Command.UPDATE_MATERIAL_VISION, data, timeout=self.command_timeout_seconds)
        self._last_sent_at = now
        self._last_key = key
        self.latest_packet = packet
        return packet

    def invalidate(self, status=MaterialVisionStatus.STOPPED):
        now = self.clock()
        session_id, frame_id = self._next_frame()
        packet = self._invalid_packet(status, now, session_id, frame_id)
        return self._send(packet, now, None)

    def report_camera_error(self):
        try:
            self.invalidate(MaterialVisionStatus.CAMERA_ERROR)
        except (SerialLinkError, ValueError):
            LOGGER.warning("相机异常状态未能送达；STM32 必须按视觉有效期停止使用旧位置", exc_info=True)

    def stop(self):
        if self.link.connected:
            try:
                self.invalidate()
            except (SerialLinkError, ValueError):
                LOGGER.warning("视觉停止状态未能送达；STM32 必须独立检查视觉过期", exc_info=True)
