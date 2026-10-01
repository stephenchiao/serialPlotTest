"""以队友压缩包中的协议 schema 和真实 ASCII 布局验证树莓派适配。"""

import json
from pathlib import Path
import unittest

from robot_hardware.stm32 import Capability, SerialLink, SerialLinkError
from robot_hardware.stm32 import messages, protocol
from robot_hardware.stm32.startup import FirmwareStartupInfo
from tests.test_stm32_protocol import _LoopbackStm32Serial


class TeammateContractTests(unittest.TestCase):
    def test_constants_match_teammate_schema(self):
        schema = json.loads((Path(__file__).resolve().parents[1] / "config" /
                             "rpi_binary_protocol.json").read_text(encoding="utf-8"))
        self.assertEqual(protocol.PROTOCOL_VERSION, schema["wire_version"])
        self.assertEqual(protocol.MAX_PAYLOAD, schema["max_payload"])
        self.assertEqual(messages.HOST_PROTOCOL_VERSION, schema["host_protocol_version"])
        for table, enum in (
            ("capabilities", Capability), ("message_types", messages.MessageType),
            ("commands", messages.Command), ("response_status", messages.ResponseStatus),
            ("events", messages.EventCode), ("pose_states", messages.PoseGoalState),
            ("motion_faults", messages.MotionFaultReason),
            ("telemetry_types", messages.TelemetryKind),
        ):
            for name, value in schema[table].items():
                with self.subTest(table=table, name=name):
                    self.assertEqual(int(enum[name]), value)
        self.assertEqual(messages.REQUIRED_CAPABILITIES, sum(schema["capabilities"].values()))

    def test_uart_and_usb_configurations_use_same_v2_session(self):
        root = Path(__file__).resolve().parents[1]
        for config_file, transport in (("stm32.json", "usb_cdc"), ("stm32_uart.json", "uart")):
            config = json.loads((root / "config" / config_file).read_text(encoding="utf-8"))
            with SerialLink.from_config(config, serial_factory=_LoopbackStm32Serial) as link:
                self.assertTrue(link.connected)
                self.assertEqual(link.transport, transport)
                self.assertEqual(link.baudrate, 115200)
                self.assertIsNotNone(link.startup_info)
        with self.assertRaisesRegex(ValueError, "transport"):
            SerialLink.from_config({"port": "fake", "transport": "invalid"})


class StartupHandshakeTests(unittest.TestCase):
    def open_port(self, port):
        return SerialLink("fake", serial_factory=lambda **kwargs: port,
                          heartbeat_interval=None, handshake_timeout=0.05)

    def test_reads_full_self_check_before_binary_start_and_preserves_pid(self):
        port = _LoopbackStm32Serial()
        with self.open_port(port) as link:
            info = link.startup_info
            self.assertEqual((info.ops_x_mm, info.ops_y_mm, info.ops_yaw_deg), (12.5, -40.25, 90))
            self.assertEqual(info.pid_yaw, (0.02, 0.000015, 0))
            commands = [entry for entry in port.history if isinstance(entry, str)]
            self.assertEqual(commands, ["PROTO VERSION", "HOST LINK RPI", "STOP", "MODE WORK",
                                        "STATUS", "OPS STATUS", "CAN STATUS", "PID STATUS ALL",
                                        "HOST BINARY START"])
        self.assertIsNone(link.startup_info)

    def test_can_response_handles_merged_lines_and_fragmentation(self):
        class MergedSerial(_LoopbackStm32Serial):
            def _enqueue(self, data):
                self._rx.put(data)
        for serial_type in (_LoopbackStm32Serial, MergedSerial):
            with self.subTest(serial_type=serial_type), self.open_port(serial_type()) as link:
                self.assertEqual(link.startup_info.can_state, 2)
                self.assertEqual(link.startup_info.can_esr, 0)

    def assert_startup_rejected(self, attribute, reply):
        port = _LoopbackStm32Serial()
        setattr(port, attribute, reply)
        link = self.open_port(port)
        with self.assertRaises(SerialLinkError):
            link.open()
        self.assertFalse(link.connected)
        self.assertFalse(port.is_open)
        self.assertIsNone(link.startup_info)
        self.assertNotIn("HOST BINARY START", port.history)
        self.assertIn(messages.Command.STOP_ALL,
                      [item[0] for item in port.history if isinstance(item, tuple)])

    def test_busy_wrong_host_or_wrong_mode_does_not_start_binary(self):
        for field, value in (("MODE=WORK", "MODE=TUNE"), ("HOST=RPI", "HOST=COM"),
                             ("HOST_PROTO=4", "HOST_PROTO=3"), ("STATE=0", "STATE=2"),
                             ("PLOT=0", "PLOT=1")):
            with self.subTest(field=field):
                self.assert_startup_rejected("status_reply", _LoopbackStm32Serial().status_reply.replace(field, value))

    def test_ops_stale_no_frames_or_invalid_coordinate_rejected(self):
        for field, value in (("LINK=OK", "LINK=STALE"), ("FRAMES=20", "FRAMES=0"),
                             ("X=12.50", "X=nan")):
            with self.subTest(field=field):
                self.assert_startup_rejected("ops_reply", _LoopbackStm32Serial().ops_reply.replace(field, value))

    def test_can_requires_all_status_parts_and_current_health(self):
        for field, value in (("STATE=2", "STATE=1"), ("READY=1", "READY=0"),
                             ("ESR=0x00000000", "ESR=0x00000004"),
                             ("STATE=2", "MISSING_STATE=2"), ("READY=1", "MISSING_READY=1")):
            with self.subTest(field=field, value=value):
                self.assert_startup_rejected("can_reply", _LoopbackStm32Serial().can_reply.replace(field, value))

    def test_missing_or_invalid_pid_reply_rejected(self):
        for pid in ("# PID ALL X=nan,0,0 Y=1,0,0 YAW=1,0,0",
                    "# PID ALL X=1,0 Y=1,0,0 YAW=1,0,0", "# PID ALL X=1,0,0 Y=1,0,0"):
            with self.subTest(pid=pid):
                self.assert_startup_rejected("pid_reply", pid)

    def test_firmware_error_in_can_response_rejected(self):
        self.assert_startup_rejected("can_reply", "# CAN ERROR BUS_OFF\r\n" + _LoopbackStm32Serial().can_reply)

    def test_missing_last_can_line_times_out_without_binary_start(self):
        self.assert_startup_rejected("can_reply", "# CAN STATE=2\r\n# CAN READY=1")

    def test_material_extension_is_still_refused_by_teammate_base_firmware(self):
        port = _LoopbackStm32Serial()
        with self.open_port(port) as link:
            with self.assertRaisesRegex(SerialLinkError, "0x40"):
                link.request(messages.Command.UPDATE_MATERIAL_VISION, b"material")


if __name__ == "__main__":
    unittest.main()
