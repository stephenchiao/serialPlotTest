"""树莓派端单实例字节流链路。

需要树莓派安装 ``pyserial``，支持原生 USB CDC 和队友固件的 USART1。
模块采用后台接收线程，负责拆包、响应匹配和掉线重连。
"""

from __future__ import annotations

from dataclasses import dataclass
import logging
import queue
import re
import threading
import time
from typing import Callable, Dict, List, Optional, Protocol

from .messages import (
    Command,
    MessageType,
    Response,
    ResponseStatus,
    SessionInfo,
    REQUIRED_CAPABILITIES,
    MATERIAL_VISION_CAPABILITY,
    SUPPORTED_COMMANDS,
    HOST_PROTOCOL_VERSION,
    encode_command,
)
from .protocol import Frame, FrameDecoder, PROTOCOL_VERSION
from .startup import FirmwareStartupInfo


LOGGER = logging.getLogger(__name__)


class SerialPort(Protocol):
    is_open: bool

    def read(self, size: int = 1) -> bytes: ...

    def write(self, data: bytes) -> int: ...

    def close(self) -> None: ...


class SerialLinkError(RuntimeError):
    pass


class CommandTimeout(SerialLinkError):
    pass


class CommandRejected(SerialLinkError):
    def __init__(self, response: Response) -> None:
        self.response = response
        super().__init__(
            f"STM32 拒绝命令 0x{response.command:02X}：{response.status.name}"
        )


@dataclass(frozen=True)
class LinkStatistics:
    received_frames: int
    sent_frames: int
    reconnects: int
    crc_errors: int
    discarded_bytes: int


class SerialLink:
    """独占一个串口的双向通信组件。"""

    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        *,
        transport: str = "usb_cdc",
        read_timeout: float = 0.05,
        reconnect_interval: float = 1.0,
        heartbeat_interval: Optional[float] = 0.1,
        heartbeat_timeout: float = 0.5,
        handshake_timeout: float = 2.0,
        recovery_timeout: float = 3.0,
        negotiate: bool = True,
        additional_capabilities: int = 0,
        serial_factory: Optional[Callable[..., SerialPort]] = None,
    ) -> None:
        if not port:
            raise ValueError("port 不能为空")
        if baudrate <= 0:
            raise ValueError("baudrate 必须大于 0")
        if transport not in ("usb_cdc", "uart"):
            raise ValueError("transport 必须为 usb_cdc 或 uart")
        self.transport = transport
        self.port = port
        self.baudrate = baudrate
        self.read_timeout = read_timeout
        self.reconnect_interval = reconnect_interval
        if heartbeat_interval is not None and heartbeat_interval <= 0:
            raise ValueError("heartbeat_interval 必须大于 0 或为 None")
        self.heartbeat_interval = heartbeat_interval
        if min(read_timeout, reconnect_interval, heartbeat_timeout, handshake_timeout, recovery_timeout) <= 0:
            raise ValueError("链路超时参数必须大于 0")
        if heartbeat_interval is not None and read_timeout + heartbeat_interval >= heartbeat_timeout:
            raise ValueError("heartbeat_timeout 必须大于 read_timeout + heartbeat_interval")
        self.heartbeat_timeout = heartbeat_timeout
        self.handshake_timeout = handshake_timeout
        self.recovery_timeout = recovery_timeout
        self.negotiate = negotiate
        if (not isinstance(additional_capabilities, int) or isinstance(additional_capabilities, bool)
            or not 0 <= additional_capabilities <= 0xFFFFFFFF):
            raise ValueError("additional_capabilities 必须为 u32 能力位")
        self.additional_capabilities = additional_capabilities
        self.session_info: Optional[SessionInfo] = None
        self.startup_info: Optional[FirmwareStartupInfo] = None
        self.generation = 0
        self._heartbeat_sequence: Optional[int] = None
        self._heartbeat_deadline = 0.0
        self._serial_factory = serial_factory
        self._serial: Optional[SerialPort] = None
        self._decoder = FrameDecoder()
        self._ascii_buffer = bytearray()
        self._stop_event = threading.Event()
        self._connected_event = threading.Event()
        self._reader_thread: Optional[threading.Thread] = None
        self._tx_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._pending_lock = threading.Lock()
        self._pending: Dict[int, tuple[int, queue.Queue]] = {}
        self._incoming: queue.Queue[Frame] = queue.Queue(maxsize=256)
        self._handler_lock = threading.Lock()
        self._frame_handlers: Dict[int, List[Callable[[Frame], bool]]] = {}
        self._next_sequence = 0
        self._received_frames = 0
        self._sent_frames = 0
        self._reconnects = 0
        self._next_heartbeat = 0.0

    @property
    def connected(self) -> bool:
        """仅完整握手和首次二进制 PING 成功后为 True。"""
        return self._connected_event.is_set()

    @property
    def port_open(self) -> bool:
        return self._serial is not None and self._serial.is_open

    @classmethod
    def from_config(cls, config: dict, **kwargs) -> "SerialLink":
        if int(config.get("protocol_version", 2)) != PROTOCOL_VERSION:
            raise ValueError("配置协议版本必须为 2")
        return cls(
            str(config["port"]), int(config.get("baudrate", 115200)),
            transport=str(config.get("transport", "usb_cdc")),
            read_timeout=float(config.get("read_timeout_seconds", 0.05)),
            reconnect_interval=float(config.get("reconnect_interval_seconds", 1.0)),
            heartbeat_interval=float(config.get("heartbeat_interval_seconds", 0.1)),
            heartbeat_timeout=float(config.get("heartbeat_timeout_seconds", 0.5)),
            handshake_timeout=float(config.get("handshake_timeout_seconds", 2.0)),
            recovery_timeout=float(config.get("recovery_timeout_seconds", 3.0)),
            **kwargs,
        )

    def open(self) -> None:
        """首次同步打开串口，然后启动后台接收线程。"""

        if self._reader_thread and self._reader_thread.is_alive():
            return
        self._stop_event.clear()
        self._connect()
        self._schedule_next_heartbeat()
        self._reader_thread = threading.Thread(
            target=self._reader_loop,
            name="stm32-serial-reader",
            daemon=True,
        )
        self._reader_thread.start()

    def close(self) -> None:
        self._stop_event.set()
        self._disconnect()
        thread = self._reader_thread
        if thread and thread is not threading.current_thread():
            thread.join(timeout=max(1.0, self.read_timeout * 3))
        self._reader_thread = None

    def request_reconnect(self) -> None:
        """丢弃当前会话，由接收线程重新握手；不重放待处理命令。"""
        self._disconnect()

    def __enter__(self) -> "SerialLink":
        self.open()
        return self

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        self.close()

    def send(self, message_type: int, payload: bytes = b"") -> int:
        sequence = self._allocate_sequence()
        self.send_frame(Frame(message_type, sequence, payload))
        return sequence

    def send_frame(self, frame: Frame) -> None:
        generation = self.generation
        if frame.message_type != MessageType.COMMAND or not frame.payload:
            raise SerialLinkError("树莓派只能发送 v2 COMMAND 帧")
        self._check_command(frame.payload[0])
        raw = frame.encode()
        with self._tx_lock:
            self._check_command(frame.payload[0])
            if generation != self.generation:
                raise SerialLinkError("发送期间会话已更换；拒绝将旧命令发到新会话")
            serial_port = self._serial
            if serial_port is None or not serial_port.is_open:
                raise SerialLinkError(f"串口未连接：{self.port}")
            try:
                written = serial_port.write(raw)
            except Exception as error:
                self._disconnect()
                raise SerialLinkError(f"写入串口失败：{error}") from error
            if written != len(raw):
                self._disconnect()
                raise SerialLinkError(f"串口只写入 {written}/{len(raw)} 字节")
            self._sent_frames += 1

    def send_command(self, command: int, data: bytes = b"") -> int:
        return self.send(MessageType.COMMAND, encode_command(command, data))

    def request(
        self,
        command: int,
        data: bytes = b"",
        *,
        timeout: float = 0.5,
    ) -> Response:
        """发送命令并等待匹配响应。

        不自动重发，以免底盘运动或机械臂动作被重复执行。调用方只应对确认
        幂等的命令自行重试。
        """

        if timeout <= 0:
            raise ValueError("timeout 必须大于 0")
        self._check_command(command)
        response_queue: queue.Queue = queue.Queue(maxsize=1)
        with self._pending_lock:
            sequence = self._next_free_sequence_locked()
            self._pending[sequence] = (int(command), response_queue)
        try:
            self.send_frame(
                Frame(MessageType.COMMAND, sequence, encode_command(command, data))
            )
            try:
                response = response_queue.get(timeout=timeout)
                if isinstance(response, Exception):
                    raise response
            except queue.Empty as error:
                raise CommandTimeout(
                    f"等待 STM32 响应超时：command=0x{int(command):02X}, seq={sequence}"
                ) from error
        finally:
            with self._pending_lock:
                self._pending.pop(sequence, None)
        if response.command != int(command):
            raise SerialLinkError(
                f"响应命令不匹配：期望 0x{int(command):02X}，收到 0x{response.command:02X}"
            )
        if response.status != ResponseStatus.OK:
            raise CommandRejected(response)
        return response

    def ping(self, timeout: float = 0.5) -> float:
        """测量一次请求/响应往返时间，返回秒数。"""

        started = time.monotonic()
        self.request(Command.PING, timeout=timeout)
        return time.monotonic() - started

    def receive(self, timeout: Optional[float] = None) -> Frame:
        """读取未被 request 消费的遥测、事件或心跳帧。"""

        try:
            return self._incoming.get(timeout=timeout)
        except queue.Empty as error:
            raise TimeoutError("等待 STM32 消息超时") from error

    def add_frame_handler(
        self,
        message_type: int,
        handler: Callable[[Frame], bool],
    ) -> None:
        """订阅非应答帧；回调返回 True 表示已消费，不再放入公共队列。"""

        with self._handler_lock:
            handlers = self._frame_handlers.setdefault(int(message_type), [])
            if handler not in handlers:
                handlers.append(handler)

    def remove_frame_handler(
        self,
        message_type: int,
        handler: Callable[[Frame], bool],
    ) -> None:
        with self._handler_lock:
            handlers = self._frame_handlers.get(int(message_type))
            if not handlers:
                return
            if handler in handlers:
                handlers.remove(handler)
            if not handlers:
                self._frame_handlers.pop(int(message_type), None)

    def send_heartbeat(self, uptime_ms: int) -> int:
        """兼容旧调用名；v2 心跳是无参数 PING，不再发送 0x21。"""
        return self.send_command(Command.PING)

    def statistics(self) -> LinkStatistics:
        return LinkStatistics(
            received_frames=self._received_frames,
            sent_frames=self._sent_frames,
            reconnects=self._reconnects,
            crc_errors=self._decoder.crc_errors,
            discarded_bytes=self._decoder.discarded_bytes,
        )

    def _allocate_sequence(self) -> int:
        with self._pending_lock:
            return self._next_free_sequence_locked()

    def _next_free_sequence_locked(self) -> int:
        for _ in range(256):
            sequence = self._next_sequence
            self._next_sequence = (sequence + 1) & 0xFF
            if sequence not in self._pending and sequence != self._heartbeat_sequence:
                return sequence
        raise SerialLinkError("全部请求序号都在使用中")

    def _check_command(self, command: int) -> None:
        if int(command) not in SUPPORTED_COMMANDS:
            raise SerialLinkError(f"队友 v2 固件未实现命令 0x{int(command):02X}")
        if not self.connected and int(command) not in (Command.STOP_ALL, Command.SESSION_PROBE):
            raise SerialLinkError("二进制会话尚未就绪，不能发送运动命令或 PING")
        if int(command) == Command.UPDATE_MATERIAL_VISION and (
            self.session_info is None
            or not self.session_info.capabilities & MATERIAL_VISION_CAPABILITY
        ):
            raise SerialLinkError("STM32 未声明物料视觉接收能力 0x40；请先接入并烧录接收模块")

    def _default_serial_factory(self, **kwargs: object) -> SerialPort:
        try:
            import serial  # type: ignore[import-not-found]
        except ImportError as error:
            raise SerialLinkError(
                "缺少 pyserial，请执行：python3 -m pip install pyserial"
            ) from error
        return serial.Serial(**kwargs)

    def _connect(self) -> None:
        factory = self._serial_factory or self._default_serial_factory
        try:
            serial_port = factory(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=8,
                parity="N",
                stopbits=1,
                timeout=self.read_timeout,
                write_timeout=0.5,
                exclusive=True,
            )
        except SerialLinkError:
            raise
        except Exception as error:
            raise SerialLinkError(f"无法打开串口 {self.port}：{error}") from error
        with self._state_lock:
            self._serial = serial_port
            self._decoder.reset()
            self._ascii_buffer.clear()
            self.startup_info = None
        try:
            info = self._probe_session()
            if info.active or info.armed:
                self._bootstrap_request(Command.STOP_ALL)
                # 所有有效命令（包括 SESSION_PROBE）都会喂狗，恢复期间必须完全静默。
                if self._stop_event.wait(self.recovery_timeout):
                    raise SerialLinkError("会话恢复被取消")
                info = self._probe_session()
                if info.active or info.armed:
                    raise SerialLinkError("旧会话未退出；recovery_timeout 必须大于固件看门狗期限")
            if self.negotiate:
                self._ascii_command("PROTO VERSION", rf"# PROTO VERSION={HOST_PROTOCOL_VERSION}(?:\s|$)")
                self._ascii_command("HOST LINK RPI", r"# HOST LINK RPI OK")
                self._ascii_command("STOP", r"# (?:STOP |ROUND STOP |POSE STOP)")
                self._ascii_command("MODE WORK", r"# MODE WORK(?:\s|$)")
                status = self._ascii_command("STATUS", r"^# STATUS ")
                ops = self._ascii_command("OPS STATUS", r"^# OPS LINK=")
                can_lines: List[str] = []
                self._ascii_command("CAN STATUS", r"^# CAN ESR=", lines=can_lines)
                pid = self._ascii_command("PID STATUS ALL", r"^# PID ALL ")
                self.startup_info = FirmwareStartupInfo.decode(status, ops, can_lines, pid)
                ready = self._ascii_command(
                    "HOST BINARY START", r"# HOST BINARY READY VERSION=\d+ CAPS=0x[0-9A-Fa-f]+"
                )
                match = re.search(r"VERSION=(\d+) CAPS=0x([0-9A-Fa-f]+)", ready)
                if not match or int(match[1]) != PROTOCOL_VERSION or int(match[2], 16) & REQUIRED_CAPABILITIES != REQUIRED_CAPABILITIES:
                    raise SerialLinkError(f"固件版本或能力不兼容：{ready}")
                self._bootstrap_request(Command.PING)
                info = self._probe_session()
                if not info.active or not info.armed or info.host_link != 2:
                    raise SerialLinkError("STM32 未进入有效 RPI 二进制会话")
                self.generation += 1
                self._connected_event.set()
        except Exception as error:
            # 就绪失败仍尝试停车，但绝不重发目标。
            if self.negotiate:
                try:
                    self._bootstrap_request(Command.STOP_ALL, timeout=0.2)
                except Exception:
                    pass
            self._disconnect()
            if isinstance(error, SerialLinkError):
                raise
            raise SerialLinkError(f"STM32 {self.transport} 握手失败：{error}") from error

    @staticmethod
    def validate_session(info: SessionInfo) -> None:
        if info.version != PROTOCOL_VERSION or info.capabilities & REQUIRED_CAPABILITIES != REQUIRED_CAPABILITIES:
            raise SerialLinkError(f"需要 VERSION=2 CAPS=0x3F；收到 {info}")

    def _probe_session(self) -> SessionInfo:
        info = SessionInfo.decode(self._bootstrap_request(Command.SESSION_PROBE).data)
        self.validate_session(info)
        if info.capabilities & self.additional_capabilities != self.additional_capabilities:
            raise SerialLinkError(f"STM32 未声明所需扩展能力 0x{self.additional_capabilities:X}，不启动新会话")
        self.session_info = info
        return info

    def _bootstrap_write(self, data: bytes) -> None:
        if self._stop_event.is_set() or not self.port_open:
            raise SerialLinkError("握手被取消或设备已断开")
        with self._tx_lock:
            if self._serial.write(data) != len(data):
                raise SerialLinkError("握手数据未完整写出")

    def _bootstrap_request(self, command: int, *, timeout: Optional[float] = None) -> Response:
        sequence = self._allocate_sequence()
        self._bootstrap_write(Frame(MessageType.COMMAND, sequence, encode_command(command)).encode())
        self._sent_frames += 1
        deadline = time.monotonic() + (self.handshake_timeout if timeout is None else timeout)
        while time.monotonic() < deadline and not self._stop_event.is_set():
            for frame in self._decoder.feed(self._serial.read(256)):
                self._received_frames += 1
                if frame.message_type != MessageType.RESPONSE:
                    continue
                try:
                    response = Response.decode(frame.payload)
                except ValueError:
                    continue
                if response.request_sequence == sequence and response.command == command:
                    if response.status != ResponseStatus.OK:
                        raise CommandRejected(response)
                    return response
        raise CommandTimeout(f"STM32 协议探测超时：0x{int(command):02X}；检查设备、固件及 {self.transport} 接线")

    def _ascii_command(self, command: str, expected: str, *, lines: Optional[List[str]] = None) -> str:
        self._bootstrap_write((command + "\r\n").encode("ascii"))
        deadline = time.monotonic() + self.handshake_timeout
        while time.monotonic() < deadline and not self._stop_event.is_set():
            if b"\n" not in self._ascii_buffer:
                self._ascii_buffer.extend(self._serial.read(256))
            if len(self._ascii_buffer) > 8192:
                raise SerialLinkError("ASCII 握手响应过长")
            while b"\n" in self._ascii_buffer:
                raw, _, remainder = self._ascii_buffer.partition(b"\n")
                self._ascii_buffer = bytearray(remainder)
                line = raw.decode("ascii", errors="replace").strip()
                if lines is not None:
                    lines.append(line)
                if line.startswith("# ERROR"):
                    raise SerialLinkError(f"{command} 被拒绝：{line}")
                if line.startswith(("# CAN ERROR", "# ROUND STOP CAN", "# POSE STOP SAFETY")):
                    raise SerialLinkError(f"启动期间固件报告故障：{line}")
                if re.search(expected, line):
                    return line
        raise CommandTimeout(f"等待 {command} 响应超时；尚未完成 STM32 协议握手")

    def _disconnect(self) -> None:
        with self._state_lock:
            serial_port, self._serial = self._serial, None
            self._connected_event.clear()
            self.startup_info = None
        with self._pending_lock:
            self._heartbeat_sequence = None
            for _, response_queue in self._pending.values():
                try:
                    response_queue.put_nowait(SerialLinkError("STM32 链路已断开；请求不会重放"))
                except queue.Full:
                    pass
        if serial_port is not None:
            try:
                serial_port.close()
            except Exception:
                LOGGER.debug("关闭串口失败", exc_info=True)

    def _reader_loop(self) -> None:
        while not self._stop_event.is_set():
            serial_port = self._serial
            if serial_port is None:
                if self._stop_event.wait(self.reconnect_interval):
                    break
                try:
                    self._connect()
                    self._reconnects += 1
                    self._schedule_next_heartbeat()
                    LOGGER.info("STM32 USB/串口链路已重连：%s", self.port)
                except SerialLinkError:
                    LOGGER.warning("STM32 USB/串口链路重连失败：%s", self.port)
                continue
            try:
                data = serial_port.read(256)
            except Exception:
                LOGGER.exception("读取 STM32 USB/串口链路失败，将尝试重连")
                self._disconnect()
                continue
            for frame in self._decoder.feed(data):
                self._received_frames += 1
                self._dispatch(frame)
            self._send_heartbeat_if_due()

    def _schedule_next_heartbeat(self) -> None:
        if self.heartbeat_interval is not None:
            self._next_heartbeat = time.monotonic() + self.heartbeat_interval

    def _send_heartbeat_if_due(self) -> None:
        if not self.connected or self.heartbeat_interval is None:
            return
        now = time.monotonic()
        if self._heartbeat_sequence is not None:
            if now >= self._heartbeat_deadline:
                LOGGER.error("STM32 PING 心跳未应答，断开会话并等待固件停车")
                self._disconnect()
            return
        if now < self._next_heartbeat:
            return
        try:
            with self._pending_lock:
                sequence = self._next_free_sequence_locked()
                self._heartbeat_sequence = sequence
                self._heartbeat_deadline = now + self.heartbeat_timeout
            self.send_frame(Frame(MessageType.COMMAND, sequence, encode_command(Command.PING)))
        except SerialLinkError:
            LOGGER.warning("发送 STM32 心跳失败，将尝试重连")
            self._disconnect()
        finally:
            self._schedule_next_heartbeat()

    def _dispatch(self, frame: Frame) -> None:
        if frame.message_type == MessageType.RESPONSE:
            try:
                response = Response.decode(frame.payload)
            except ValueError:
                LOGGER.warning("收到格式错误的 RESPONSE", exc_info=True)
            else:
                with self._pending_lock:
                    if response.request_sequence == self._heartbeat_sequence and response.command == Command.PING:
                        self._heartbeat_sequence = None
                        if response.status != ResponseStatus.OK:
                            self._connected_event.clear()
                        else:
                            return
                    pending = self._pending.get(response.request_sequence)
                if not self.connected and self.negotiate:
                    self._disconnect()
                    return
                if pending is not None and pending[0] == response.command:
                    try:
                        pending[1].put_nowait(response)
                    except queue.Full:
                        LOGGER.warning("重复的 RESPONSE：seq=%d", response.request_sequence)
                    return
                LOGGER.debug("丢弃未等待的 RESPONSE：seq=%d", response.request_sequence)
                return
        with self._handler_lock:
            handlers = tuple(self._frame_handlers.get(int(frame.message_type), ()))
        handled = False
        for handler in handlers:
            try:
                handled = handler(frame) or handled
            except Exception:
                LOGGER.exception("STM32 帧订阅回调执行失败")
        if not handled:
            try:
                self._incoming.put_nowait(frame)
            except queue.Full:
                # 无消费者时绝不让遥测阻塞应答/心跳接收。
                LOGGER.debug("观测队列已满，丢弃未订阅帧")
