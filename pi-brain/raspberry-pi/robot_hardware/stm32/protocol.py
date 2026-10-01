"""二进制串口帧的编码、CRC16 和流式解码。"""

from dataclasses import dataclass
from binascii import crc_hqx
import struct
from typing import Iterable, List


SOF = b"\xA5\x5A"
PROTOCOL_VERSION = 2
MAX_PAYLOAD = 64
_HEADER = struct.Struct("<2sBBBH")
_CRC = struct.Struct("<H")
MIN_FRAME_SIZE = _HEADER.size + _CRC.size


class ProtocolError(ValueError):
    pass


@dataclass(frozen=True)
class Frame:
    message_type: int
    sequence: int
    payload: bytes = b""
    version: int = PROTOCOL_VERSION

    def __post_init__(self) -> None:
        if not 0 <= int(self.message_type) <= 0xFF:
            raise ValueError("message_type 必须在 0~255 范围内")
        if not 0 <= self.sequence <= 0xFF:
            raise ValueError("sequence 必须在 0~255 范围内")
        if not 0 <= self.version <= 0xFF:
            raise ValueError("version 必须在 0~255 范围内")
        if len(self.payload) > MAX_PAYLOAD:
            raise ValueError(f"payload 不能超过 {MAX_PAYLOAD} 字节")

    def encode(self) -> bytes:
        payload = bytes(self.payload)
        body = struct.pack(
            "<BBBH", self.version, int(self.message_type), self.sequence, len(payload)
        ) + payload
        return SOF + body + _CRC.pack(crc16_ccitt(body))


def crc16_ccitt(data: Iterable[int], initial: int = 0xFFFF) -> int:
    """CRC-16/CCITT-FALSE：poly=0x1021, init=0xFFFF。"""

    return crc_hqx(bytes(data), initial)


def decode_frame(raw: bytes) -> Frame:
    if len(raw) < MIN_FRAME_SIZE:
        raise ProtocolError("帧长度不足")
    sof, version, message_type, sequence, payload_length = _HEADER.unpack_from(raw)
    if sof != SOF:
        raise ProtocolError("帧头错误")
    if version != PROTOCOL_VERSION:
        raise ProtocolError(f"不支持的协议版本：{version}")
    if payload_length > MAX_PAYLOAD:
        raise ProtocolError(f"payload 长度超过上限：{payload_length}")
    expected_length = _HEADER.size + payload_length + _CRC.size
    if len(raw) != expected_length:
        raise ProtocolError(f"帧长度错误：期望 {expected_length}，实际 {len(raw)}")
    expected_crc = _CRC.unpack_from(raw, expected_length - _CRC.size)[0]
    actual_crc = crc16_ccitt(raw[2 : expected_length - _CRC.size])
    if actual_crc != expected_crc:
        raise ProtocolError(
            f"CRC 错误：期望 0x{expected_crc:04X}，计算得到 0x{actual_crc:04X}"
        )
    payload_start = _HEADER.size
    return Frame(message_type, sequence, raw[payload_start : payload_start + payload_length], version)


class FrameDecoder:
    """从任意分块的串口字节流中提取完整帧，并在噪声后自动重同步。"""

    def __init__(self, max_buffer_size: int = 4096) -> None:
        if max_buffer_size < MIN_FRAME_SIZE:
            raise ValueError("max_buffer_size 太小")
        self._buffer = bytearray()
        self._max_buffer_size = max_buffer_size
        self.discarded_bytes = 0
        self.crc_errors = 0

    def reset(self) -> None:
        self._buffer.clear()

    def feed(self, data: bytes) -> List[Frame]:
        if data:
            self._buffer.extend(data)
        if len(self._buffer) > self._max_buffer_size:
            keep_from = max(0, self._buffer.rfind(SOF))
            self.discarded_bytes += keep_from
            del self._buffer[:keep_from]
            if len(self._buffer) > self._max_buffer_size:
                self.discarded_bytes += len(self._buffer) - 1
                del self._buffer[:-1]

        frames: List[Frame] = []
        while True:
            sof_index = self._buffer.find(SOF)
            if sof_index < 0:
                keep = 1 if self._buffer.endswith(SOF[:1]) else 0
                self.discarded_bytes += len(self._buffer) - keep
                if keep:
                    del self._buffer[:-1]
                else:
                    self._buffer.clear()
                break
            if sof_index:
                self.discarded_bytes += sof_index
                del self._buffer[:sof_index]
            if len(self._buffer) < _HEADER.size:
                break
            _, version, _, _, payload_length = _HEADER.unpack_from(self._buffer)
            if version != PROTOCOL_VERSION or payload_length > MAX_PAYLOAD:
                self.discarded_bytes += 1
                del self._buffer[0]
                continue
            total_length = _HEADER.size + payload_length + _CRC.size
            if len(self._buffer) < total_length:
                break
            candidate = bytes(self._buffer[:total_length])
            try:
                frames.append(decode_frame(candidate))
            except ProtocolError:
                self.crc_errors += 1
                self.discarded_bytes += 1
                del self._buffer[0]
                continue
            del self._buffer[:total_length]
        return frames
