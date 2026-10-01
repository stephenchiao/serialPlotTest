"""树莓派与 STM32 USB CDC/串口字节流协议的无硬件测试。"""

import queue
import io
from contextlib import redirect_stdout
import struct
import threading
import time
import unittest
from unittest.mock import patch

from robot_hardware.stm32.messages import (
    Command,
    MessageType,
    Response,
    ResponseStatus,
    PoseCancelled,
    PoseReached,
    PoseStarted,
    MotionFault,
    MotionFaultReason,
    decode_motion_event,
    encode_chassis_velocity,
    encode_command,
    SessionInfo,
    PoseSample,
    decode_telemetry,
)
from robot_hardware.stm32.protocol import (
    Frame,
    FrameDecoder,
    ProtocolError,
    crc16_ccitt,
    decode_frame,
)
from robot_hardware.stm32.serial_link import SerialLink, SerialLinkError


class ProtocolTests(unittest.TestCase):
    def test_standard_crc_vector(self):
        self.assertEqual(crc16_ccitt(b"123456789"), 0x29B1)

    def test_crc_preserves_custom_seed_and_iterable_input(self):
        self.assertEqual(crc16_ccitt(b""), 0xFFFF)
        self.assertEqual(crc16_ccitt(b"123456789", initial=0), 0x31C3)
        prefix_crc = crc16_ccitt([49, 50, 51, 52])
        self.assertEqual(crc16_ccitt(iter(b"56789"), initial=prefix_crc), 0x29B1)

    def test_frame_round_trip(self):
        original = Frame(
            MessageType.COMMAND,
            37,
            encode_command(Command.SET_CHASSIS_VELOCITY, encode_chassis_velocity(120, -30, 250)),
        )

        self.assertEqual(decode_frame(original.encode()), original)

    def test_stream_decoder_handles_noise_fragmentation_and_multiple_frames(self):
        first = Frame(MessageType.COMMAND, 1, encode_command(Command.PING))
        second = Frame(MessageType.EVENT, 2, b"event")
        wire = b"\x00\xFFnoise" + first.encode() + second.encode()
        decoder = FrameDecoder()

        frames = []
        for start in range(0, len(wire), 3):
            frames.extend(decoder.feed(wire[start : start + 3]))

        self.assertEqual(frames, [first, second])
        self.assertGreater(decoder.discarded_bytes, 0)

    def test_stream_decoder_recovers_after_bad_crc(self):
        damaged = bytearray(Frame(MessageType.EVENT, 3, b"bad").encode())
        damaged[-1] ^= 0x80
        good = Frame(MessageType.EVENT, 4, b"good")
        decoder = FrameDecoder()

        self.assertEqual(decoder.feed(bytes(damaged) + good.encode()), [good])
        self.assertEqual(decoder.crc_errors, 1)

    def test_decode_rejects_wrong_crc(self):
        raw = bytearray(Frame(MessageType.COMMAND, 5, encode_command(Command.PING)).encode())
        raw[-1] ^= 1
        with self.assertRaisesRegex(ProtocolError, "CRC"):
            decode_frame(bytes(raw))

    def test_motion_events_round_trip(self):
        events = [
            PoseStarted(42),
            PoseReached(42, 100, -200, 1571, 3, -5),
            PoseCancelled(42),
            MotionFault(42, MotionFaultReason.CAN_FAULT),
        ]
        for event in events:
            with self.subTest(event=event):
                self.assertEqual(decode_motion_event(event.encode_event()), event)


class _LoopbackStm32Serial:
    """模拟队友 v2 ASCII/二进制会话及看门狗，USB 按 3 字节碎片返回。"""

    def __init__(self, **kwargs):
        self.is_open = True
        self._rx = queue.Queue()
        self.history = []
        self.active = self.armed = False
        self.capabilities = 0x3F
        self.reject_binary = False
        self.drop_ping = False
        self.drop_motion = False
        self.fail_read = False
        self.expire_after = None
        self.last_command = time.monotonic()
        self.status_reply = "# STATUS MODE=WORK HOST_PROTO=4 HOST=RPI STATE=0 PLOT=0"
        self.ops_reply = "# OPS LINK=OK X=12.50 Y=-40.25 YAW=90.00 FRAMES=20 FRAME_AGE=5"
        self.can_reply = (
            "# CAN STATE=2 ERROR=0x00000020 FREE=3 TX_OK=4\r\n"
            "# CAN TX_QUEUED=0 TX_ABORT=0 TX_TIMEOUT=0 ERR_LATCH=0x00000020\r\n"
            "# CAN READY=1 TX_FAULT=0 MASK=0x0F\r\n"
            "# CAN TX_WATCH NO_TX_REPAIR=0\r\n"
            "# CAN ESR=0x00000000 TSR=0 TEC=0 REC=0 BOFF=0 EPVF=0 EWGF=0"
        )
        self.pid_reply = "# PID ALL X=0.0018,0,0 Y=0.0018,0,0 YAW=0.02,0.000015,0"

    def _enqueue(self, data):
        for start in range(0, len(data), 3):
            self._rx.put(data[start:start + 3])

    def read(self, size=1):
        if self.fail_read:
            raise OSError("simulated USB unplug")
        try:
            return self._rx.get(timeout=0.005)
        except queue.Empty:
            return b""

    def write(self, data):
        if self.expire_after is not None and time.monotonic() - self.last_command > self.expire_after:
            self.active = self.armed = False
        self.last_command = time.monotonic()
        if not data.startswith(b"\xA5\x5A"):
            command = data.decode("ascii").strip()
            self.history.append(command)
            replies = {
                "PROTO VERSION": "# PROTO VERSION=4 MODES=WORK,TUNE,PLOT",
                "HOST LINK RPI": "# HOST LINK RPI OK HEARTBEAT=REQUIRED",
                "STOP": "# STOP MODE=WORK",
                "MODE WORK": "# MODE WORK PLOT=0 CHANGED=0",
                "STATUS": self.status_reply,
                "OPS STATUS": self.ops_reply,
                "CAN STATUS": self.can_reply,
                "PID STATUS ALL": self.pid_reply,
                "HOST BINARY START": "# HOST BINARY READY VERSION=2 CAPS=0x0000003F",
            }
            if command == "HOST BINARY START":
                if self.reject_binary:
                    self._enqueue(b"# ERROR HOST BINARY NOT READY\r\n")
                    return len(data)
                self.armed = True
            self._enqueue((replies[command] + "\r\n").encode())
            return len(data)
        request = decode_frame(data)
        command = request.payload[0]
        self.history.append((command, request.payload[1:], time.monotonic()))
        if self.armed:
            self.active = True
        response_data = b""
        status = ResponseStatus.OK
        if command == Command.SESSION_PROBE:
            response_data = struct.pack("<BBBBIIB", self.active, self.armed, 2 if self.armed else 0,
                                        0, 0, self.capabilities, 2)
        elif command not in (Command.STOP_ALL, Command.SESSION_PROBE) and not self.armed:
            status = ResponseStatus.BUSY
        if command == Command.PING and self.drop_ping:
            return len(data)
        if command == Command.SET_POSE_GOAL_WITH_LIMITS and self.drop_motion:
            return len(data)
        response = Response(request.sequence, command, status, response_data)
        self._enqueue(Frame(MessageType.RESPONSE, 90, response.encode()).encode())
        return len(data)

    def close(self):
        self.is_open = False


class SerialLinkTests(unittest.TestCase):
    def test_request_matches_response_without_pyserial_or_hardware(self):
        link = SerialLink("loopback", serial_factory=_LoopbackStm32Serial)
        with link:
            self.assertTrue(link.connected)
            self.assertEqual(link.generation, 1)
            response = link.request(Command.PING, timeout=0.5)

        self.assertEqual(response.command, Command.PING)
        self.assertEqual(response.status, ResponseStatus.OK)
        self.assertEqual(link.statistics().sent_frames, 4)
        self.assertEqual(link.statistics().received_frames, 4)

    def test_diagnostic_probe_never_enables_session(self):
        port = _LoopbackStm32Serial()
        with SerialLink("fake", negotiate=False, serial_factory=lambda **kw: port) as link:
            self.assertTrue(link.port_open)
            self.assertFalse(link.connected)
            self.assertFalse(link.session_info.active)
            with self.assertRaises(SerialLinkError):
                link.ping()
            info = SessionInfo.decode(link.request(Command.SESSION_PROBE).data)
            self.assertEqual(info.version, 2)
        self.assertFalse(any(isinstance(item, str) for item in port.history))

    def test_handshake_rejects_missing_capabilities(self):
        port = _LoopbackStm32Serial()
        port.capabilities = 0x1F
        link = SerialLink("fake", serial_factory=lambda **kw: port)
        with self.assertRaisesRegex(SerialLinkError, "CAPS"):
            link.open()
        self.assertFalse(link.connected)
        self.assertFalse(port.is_open)

    def test_not_ready_does_not_mark_connected(self):
        port = _LoopbackStm32Serial()
        port.reject_binary = True
        link = SerialLink("fake", serial_factory=lambda **kw: port)
        with self.assertRaisesRegex(SerialLinkError, "NOT READY"):
            link.open()
        self.assertFalse(link.connected)

    def test_stale_session_recovery_is_silent_and_never_replays_goal(self):
        port = _LoopbackStm32Serial()
        port.armed = port.active = True
        port.expire_after = 0.025
        with SerialLink("fake", recovery_timeout=0.05, heartbeat_interval=None,
                        serial_factory=lambda **kw: port) as link:
            self.assertTrue(link.connected)
        binary = [item for item in port.history if isinstance(item, tuple)]
        self.assertEqual([item[0] for item in binary[:3]],
                         [Command.SESSION_PROBE, Command.STOP_ALL, Command.SESSION_PROBE])
        self.assertGreaterEqual(binary[2][2] - binary[1][2], 0.045)
        self.assertNotIn(Command.SET_POSE_GOAL_WITH_LIMITS, [item[0] for item in binary])

    def test_heartbeat_is_ping_and_failure_disconnects(self):
        port = _LoopbackStm32Serial()
        link = SerialLink("fake", read_timeout=0.005, heartbeat_interval=0.01,
                          heartbeat_timeout=0.04, reconnect_interval=1,
                          serial_factory=lambda **kw: port)
        with link:
            port.drop_ping = True
            deadline = time.monotonic() + 0.3
            while link.connected and time.monotonic() < deadline:
                time.sleep(0.005)
            self.assertFalse(link.connected)
        binary = [item for item in port.history if isinstance(item, tuple)]
        self.assertTrue(any(item[0] == Command.PING for item in binary[3:]))

    def test_unknown_legacy_motion_command_is_blocked(self):
        with SerialLink("fake", serial_factory=_LoopbackStm32Serial) as link:
            with self.assertRaisesRegex(SerialLinkError, "未实现"):
                link.request(Command.SET_CHASSIS_VELOCITY)

    def test_close_unblocks_pending_request(self):
        port = _LoopbackStm32Serial()
        link = SerialLink("fake", heartbeat_interval=None, serial_factory=lambda **kw: port)
        link.open()
        port.drop_motion = True
        errors = []
        def request():
            try:
                link.request(Command.SET_POSE_GOAL_WITH_LIMITS, b"goal", timeout=3)
            except SerialLinkError as error:
                errors.append(error)
        worker = threading.Thread(target=request)
        worker.start()
        deadline = time.monotonic() + 0.3
        while not link._pending and time.monotonic() < deadline:
            time.sleep(0.001)
        link.close()
        worker.join(0.3)
        self.assertFalse(worker.is_alive())
        self.assertEqual(len(errors), 1)

    def test_sequence_wrap_skips_pending_and_heartbeat(self):
        link = SerialLink("fake")
        link._next_sequence = 255
        link._pending[255] = (Command.PING, queue.Queue())
        link._heartbeat_sequence = 0
        self.assertEqual(link._allocate_sequence(), 1)

    def test_wrong_opcode_cannot_consume_pending_response(self):
        link = SerialLink("fake", negotiate=False)
        waiting = queue.Queue()
        link._pending[1] = (Command.SESSION_PROBE, waiting)
        link._dispatch(Frame(MessageType.RESPONSE, 7,
                             Response(1, Command.PING, ResponseStatus.OK).encode()))
        self.assertTrue(waiting.empty())

    def test_reconnect_handshakes_new_session_without_replaying_motion(self):
        ports = []
        def factory(**kwargs):
            port = _LoopbackStm32Serial()
            ports.append(port)
            return port
        with SerialLink("fake", serial_factory=factory, reconnect_interval=0.01,
                        heartbeat_interval=None) as link:
            link.request(Command.SET_POSE_GOAL_WITH_LIMITS, b"goal")
            ports[0].fail_read = True
            deadline = time.monotonic() + 0.5
            while link.generation < 2 and time.monotonic() < deadline:
                time.sleep(0.005)
            self.assertTrue(link.connected)
            self.assertEqual(link.generation, 2)
            self.assertEqual(link.statistics().reconnects, 1)
            commands = [item[0] for item in ports[1].history if isinstance(item, tuple)]
            self.assertNotIn(Command.SET_POSE_GOAL_WITH_LIMITS, commands)

    def test_unconsumed_telemetry_queue_is_bounded(self):
        link = SerialLink("fake")
        for index in range(1000):
            link._dispatch(Frame(MessageType.TELEMETRY, index % 256, b"\x01"))
        self.assertEqual(link._incoming.qsize(), 256)


class V2MessageTests(unittest.TestCase):
    def test_pose_wheel_and_link_stats_layouts(self):
        data = struct.pack("<IH8i", 100, 2, 10, 20, 30, 40, 50, 60, 70, 80)
        pose = decode_telemetry(b"\x02" + data)
        self.assertIsInstance(pose, PoseSample)
        self.assertEqual((pose.ops_x_mm, pose.center_x_mm, pose.plan_vz_urad_s), (10, 40, 80))
        wheel = decode_telemetry(b"\x01" + data)
        self.assertEqual(wheel.target_rpm_tenths, (10, 20, 30, 40))
        stats = decode_telemetry(b"\x03" + struct.pack("<6I", 1, 2, 3, 4, 5, 6))
        self.assertEqual(stats.transport_errors, 6)

    def test_v1_frames_rejected_and_v2_payload_limit(self):
        with self.assertRaises(ProtocolError):
            decode_frame(Frame(MessageType.COMMAND, 1, b"\x01", version=1).encode())
        with self.assertRaises(ValueError):
            Frame(MessageType.COMMAND, 1, b"x" * 65)
        decoder = FrameDecoder()
        good = Frame(MessageType.COMMAND, 2, b"\x01")
        self.assertEqual(decoder.feed(Frame(MessageType.COMMAND, 1, b"\x01", version=1).encode() + good.encode()), [good])


class DiagnosticToolTests(unittest.TestCase):
    def test_cli_probe_and_handshake_pass_with_simulated_usb(self):
        from tools.stm32_link_test import main
        for flags in ([], ["--handshake"], ["--stop"]):
            ports = []
            def configured_link(config, **kwargs):
                port = _LoopbackStm32Serial()
                ports.append(port)
                return SerialLink("fake", serial_factory=lambda **kw: port, **kwargs)
            output = io.StringIO()
            with patch("tools.stm32_link_test.SerialLink.from_config", side_effect=configured_link):
                with redirect_stdout(output):
                    result = main(["--port", "fake", "--count", "1"] + flags)
            self.assertEqual(result, 0)
            self.assertIn("STOP_ALL" if flags == ["--stop"] else "PASS", output.getvalue())
            if not flags:
                self.assertFalse(any(isinstance(item, str) for item in ports[0].history))


if __name__ == "__main__":
    unittest.main()
