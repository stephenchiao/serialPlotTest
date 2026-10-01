"""视觉观测到 STM32 的协议、失效保护和 Python/C 交叉验证。"""

from dataclasses import replace
from pathlib import Path
import shutil
import subprocess
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from robot_hardware.stm32 import (
    MaterialVisionBackend, MaterialVisionFlags, MaterialVisionPacket,
    MaterialVisionStatus, SerialLink, SerialLinkError, Stm32MaterialVisionPublisher,
)
from robot_hardware.stm32.messages import Command, Response, ResponseStatus
from robot_hardware.stm32.protocol import Frame, decode_frame
from robot_perception.material.detector import GripperDetection, MaterialObservation
from tests import test_dual_camera_vision as camera_tests
from tests.test_stm32_protocol import _LoopbackStm32Serial


def sample_detection(*, status="ALIGNING", confirmed=True, backend="model", target=7):
    observation = MaterialObservation(
        material_code=7, color_name="material", color_cn_name="物料",
        center=(350, 225), offset_pixels=(30.0, -15.0),
        normalized_offset=(30 / 320, -15 / 240), box=(330, 205, 40, 40),
        area=1600, confirmed=confirmed, class_id=0 if backend == "model" else None,
        class_name="material", confidence=0.92, backend=backend,
    )
    return GripperDetection(
        status=status, target_material_code=target, observation=observation,
        aligned=False, safe_to_pick=False, detected_color_codes=(7,), backend=backend,
    )


def sample_packet():
    return MaterialVisionPacket.from_detection(
        sample_detection(), frame_size=(640, 480), session_id=0x12345678,
        frame_id=1, captured_at=10.0, valid_for_ms=250,
    )


class FakeLink:
    def __init__(self):
        self.generation = 1
        self.connected = True
        self.requests = []
        self.error = None

    def request(self, command, data, *, timeout):
        self.requests.append((command, data, timeout))
        if self.error is not None:
            raise self.error
        return Response(1, command, ResponseStatus.OK)


class MaterialPacketTests(unittest.TestCase):
    def test_fixed_payload_roundtrip_fits_existing_v2_frame(self):
        packet = sample_packet()
        data = packet.encode_command_data()
        self.assertEqual(len(data), 47)
        self.assertEqual(MaterialVisionPacket.decode_command_data(data), packet)
        frame = Frame(0x10, 8, bytes((Command.UPDATE_MATERIAL_VISION,)) + data)
        self.assertEqual(decode_frame(frame.encode()), frame)
        self.assertEqual(len(frame.encode()), 57)
        self.assertEqual(packet.class_id, 0)  # class 0 is not "no class"
        self.assertEqual((packet.offset_x_tenths, packet.offset_y_tenths), (300, -150))

    def test_alignment_is_only_debug_information_not_a_command_gate(self):
        for status in ("ALIGNING", "READY"):
            detection = replace(sample_detection(), status=status, safe_to_pick=(status == "READY"))
            packet = MaterialVisionPacket.from_detection(
                detection, frame_size=(640, 480), session_id=1, frame_id=1,
                captured_at=10, valid_for_ms=250,
            )
            self.assertEqual(packet.status, MaterialVisionStatus.TRACKING)
            self.assertEqual(packet.flags, MaterialVisionFlags.VISIBLE | MaterialVisionFlags.CONFIRMED)
            self.assertFalse(hasattr(packet, "safe_to_pick"))

    def test_unconfirmed_and_color_observations_are_reported(self):
        for backend in ("model", "color"):
            packet = MaterialVisionPacket.from_detection(
                sample_detection(status="CONFIRMING_MATERIAL", confirmed=False, backend=backend),
                frame_size=(640, 480), session_id=1, frame_id=1, captured_at=10, valid_for_ms=250,
            )
            self.assertEqual(packet.status, MaterialVisionStatus.CONFIRMING)
            self.assertEqual(packet.flags, MaterialVisionFlags.VISIBLE)
            self.assertEqual(packet.class_id, 0 if backend == "model" else 0xFFFF)

    def test_blocked_and_lost_states_clear_all_position_fields(self):
        for status in ("MODEL_NOT_CONFIGURED", "MAPPING_NOT_CONFIGURED", "UNMAPPED_CLASS",
                       "TARGET_NOT_MAPPED", "TARGET_NOT_FOUND", "AMBIGUOUS", "INFERENCE_ERROR", "SEARCHING"):
            packet = MaterialVisionPacket.from_detection(
                replace(sample_detection(), status=status, observation=None),
                frame_size=(640, 480), session_id=1, frame_id=1, captured_at=10, valid_for_ms=250,
            )
            self.assertEqual(packet.flags, 0)
            self.assertEqual(packet.material_code, 0)
            self.assertEqual(packet.class_id, 0xFFFF)
            self.assertEqual((packet.center_x, packet.center_y, packet.offset_x_tenths, packet.box_width), (0, 0, 0, 0))
            self.assertEqual(packet.target_material_code, 7)

    def test_invalid_values_are_rejected_not_truncated(self):
        cases = (
            {"material_code": 65536, "target_material_code": 65536},
            {"offset_x_tenths": 32768}, {"offset_y_tenths": -32769},
            {"session_id": 0}, {"frame_id": 0}, {"valid_for_ms": 0},
            {"valid_for_ms": 1001}, {"center_x": 640}, {"box_width": 700},
            {"confidence_permille": 1001}, {"flags": 7}, {"class_id": 0xFFFF},
            {"target_material_code": 8}, {"camera_num": 1},
            {"flags": MaterialVisionFlags.VISIBLE}, {"material_code": True},
        )
        for changes in cases:
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                replace(sample_packet(), **changes).encode_command_data()

    def test_invalid_state_cannot_keep_old_coordinates(self):
        with self.assertRaises(ValueError):
            replace(sample_packet(), status=MaterialVisionStatus.STOPPED, flags=MaterialVisionFlags(0)).encode_command_data()

    def test_wrong_length_and_nan_observations_are_rejected(self):
        with self.assertRaises(ValueError):
            MaterialVisionPacket.decode_command_data(b"\0" * 46)
        for changes in ({"confidence": float("nan")}, {"offset_pixels": (float("inf"), 0)}):
            detection = sample_detection()
            with self.assertRaises(ValueError):
                MaterialVisionPacket.from_detection(
                    replace(detection, observation=replace(detection.observation, **changes)),
                    frame_size=(640, 480), session_id=1, frame_id=1, captured_at=10, valid_for_ms=250,
                )


class MaterialPublisherTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.0
        self.link = FakeLink()
        self.session_ids = iter((101, 202, 303))
        self.publisher = Stm32MaterialVisionPublisher(
            self.link, clock=lambda: self.now, session_id_factory=lambda: next(self.session_ids),
        )

    def publish(self, detection=None, *, captured_at=None):
        return self.publisher.publish(
            detection or sample_detection(), frame_size=(640, 480),
            captured_at=self.now if captured_at is None else captured_at,
        )

    def test_unaligned_position_is_sent_and_ack_does_not_mean_grasp_finished(self):
        packet = self.publish()
        self.assertEqual(packet.status, MaterialVisionStatus.TRACKING)
        self.assertEqual(packet.offset_x_tenths, 300)
        self.assertEqual(len(self.link.requests), 1)
        self.assertEqual(self.link.requests[0][0], Command.UPDATE_MATERIAL_VISION)

    def test_rate_limit_never_reuses_cached_coordinates(self):
        self.publish()
        self.now += 0.02
        self.assertIsNone(self.publish())
        self.now += 0.10
        packet = self.publish()
        self.assertEqual(packet.frame_id, 2)
        self.assertEqual(len(self.link.requests), 2)

    def test_target_loss_and_target_change_bypass_rate_limit(self):
        self.publish()
        self.now += 0.01
        lost = replace(sample_detection(), status="TARGET_NOT_FOUND", observation=None)
        packet = self.publish(lost)
        self.assertEqual(packet.flags, 0)
        self.assertEqual(len(self.link.requests), 2)
        self.now += 0.01
        packet = self.publish(replace(lost, target_material_code=8))
        self.assertEqual(packet.target_material_code, 8)
        self.assertEqual(len(self.link.requests), 3)

    def test_stale_frame_is_invalid_and_processing_time_reduces_ttl(self):
        packet = self.publish(captured_at=9.9)
        self.assertTrue(149 <= packet.valid_for_ms <= 150)
        self.now += 0.01
        packet = self.publish(captured_at=9.0)
        self.assertEqual(packet.status, MaterialVisionStatus.STALE)
        self.assertEqual(packet.flags, 0)

    def test_future_and_nonfinite_capture_times_are_rejected(self):
        for value in (11.0, -1, float("nan"), float("inf")):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.publish(captured_at=value)
        self.assertEqual(self.link.requests, [])

    def test_reconnect_starts_new_visual_session_without_replay(self):
        first = self.publish()
        self.link.generation += 1
        self.now += 0.01
        packet = self.publish(replace(sample_detection(), status="MODEL_NOT_CONFIGURED", observation=None))
        self.assertEqual((first.session_id, packet.session_id), (101, 202))
        self.assertEqual(packet.frame_id, 1)
        self.assertEqual(packet.material_code, 0)

    def test_ack_failure_is_not_retried_or_counted_as_completion(self):
        self.link.error = SerialLinkError("lost ACK")
        with self.assertRaises(SerialLinkError):
            self.publish()
        self.assertIsNone(self.publisher.latest_packet)
        self.assertEqual(len(self.link.requests), 1)
        self.link.error = None
        self.now += 0.01
        self.assertEqual(self.publish().frame_id, 2)

    def test_stop_and_camera_error_clear_position(self):
        self.publish()
        self.publisher.report_camera_error()
        self.assertEqual(self.publisher.latest_packet.status, MaterialVisionStatus.CAMERA_ERROR)
        self.publisher.stop()
        self.assertEqual(self.publisher.latest_packet.status, MaterialVisionStatus.STOPPED)
        self.assertEqual(self.publisher.latest_packet.flags, 0)
        self.assertTrue(all(command == Command.UPDATE_MATERIAL_VISION for command, _, _ in self.link.requests))

    def test_disconnected_stop_does_not_attempt_to_send(self):
        self.link.connected = False
        self.publisher.stop()
        self.assertEqual(self.link.requests, [])

    def test_invalid_report_config_is_rejected(self):
        for changes in ({"maximum_rate_hz": 0}, {"maximum_rate_hz": float("nan")},
                        {"valid_for_ms": 0}, {"valid_for_ms": 1001},
                        {"command_timeout_seconds": 0.5}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                Stm32MaterialVisionPublisher(self.link, **changes)


class MaterialSerialCapabilityTests(unittest.TestCase):
    def test_reporting_session_checks_capability_before_enabling_old_firmware(self):
        port = _LoopbackStm32Serial()
        link = SerialLink(
            "fake", serial_factory=lambda **_: port, heartbeat_interval=None,
            additional_capabilities=0x40,
        )
        try:
            with self.assertRaisesRegex(SerialLinkError, "0x40"):
                link.open()
            self.assertNotIn("HOST LINK RPI", port.history)
            self.assertNotIn("HOST BINARY START", port.history)
        finally:
            link.close()

    def test_old_firmware_cannot_receive_new_command(self):
        port = _LoopbackStm32Serial()
        with SerialLink("fake", serial_factory=lambda **_: port, heartbeat_interval=None) as link:
            count = len(port.history)
            with self.assertRaisesRegex(SerialLinkError, "0x40"):
                link.request(Command.UPDATE_MATERIAL_VISION, sample_packet().encode_command_data())
            self.assertEqual(len(port.history), count)

    def test_capable_firmware_receives_full_packet_and_returns_matching_ack(self):
        port = _LoopbackStm32Serial()
        port.capabilities |= 0x40
        with SerialLink("fake", serial_factory=lambda **_: port, heartbeat_interval=None) as link:
            response = link.request(Command.UPDATE_MATERIAL_VISION, sample_packet().encode_command_data())
            self.assertEqual(response.status, ResponseStatus.OK)
            self.assertEqual(response.command, Command.UPDATE_MATERIAL_VISION)
        sent = [item for item in port.history if isinstance(item, tuple) and item[0] == Command.UPDATE_MATERIAL_VISION]
        self.assertEqual(len(sent), 1)
        self.assertEqual(sent[0][1], sample_packet().encode_command_data())


class MaterialCameraReportingTests(unittest.TestCase):
    def make_controller(self):
        helper = camera_tests.DualCameraVisionTests()
        self.reporter = SimpleNamespace(publish_calls=[], errors=0, stops=0)
        self.reporter.publish = lambda detection, **kwargs: self.reporter.publish_calls.append((detection, kwargs))
        self.reporter.report_camera_error = lambda: setattr(self.reporter, "errors", self.reporter.errors + 1)
        self.reporter.stop = lambda: setattr(self.reporter, "stops", self.reporter.stops + 1)
        controller = helper.make_controller(material_reporter=self.reporter)
        helper.manager.gripper.frame = SimpleNamespace(shape=(480, 640, 3))
        helper.material_detector.detect = lambda *_args, **_kwargs: sample_detection()
        controller.start(("gripper",))
        return controller

    def test_cam0_detection_is_reported_once_without_grasping(self):
        controller = self.make_controller()
        result = controller.observe_gripper(7)
        self.assertEqual(len(self.reporter.publish_calls), 1)
        self.assertIs(self.reporter.publish_calls[0][0], result.material_detection)
        self.assertEqual(self.reporter.publish_calls[0][1]["frame_size"], (640, 480))
        controller.close()
        self.assertEqual(self.reporter.stops, 1)

    def test_camera_failure_invalidates_output_and_close_still_releases_camera(self):
        controller = self.make_controller()
        controller.camera_1.capture_array = lambda *_: (_ for _ in ()).throw(RuntimeError("camera fault"))
        with self.assertRaisesRegex(RuntimeError, "camera fault"):
            controller.observe_gripper(7)
        self.assertEqual(self.reporter.errors, 1)
        controller.close()
        self.assertTrue(controller.camera_manager.closed)

    def test_cli_does_not_implicitly_enable_hardware(self):
        from tools.report_material import parse_arguments
        with patch("sys.stderr"), self.assertRaises(SystemExit):
            parse_arguments([])
        args = parse_arguments(["--enable-stm32-session", "--target-code", "7"])
        self.assertEqual(args.target_code, 7)

    def test_reporting_entry_owns_only_cam0_and_closes_vision_before_link(self):
        from tools import report_material
        link = SimpleNamespace(
            session_info=SimpleNamespace(capabilities=0x7F),
            open=lambda: order.append("link-open"), close=lambda: order.append("link-close"),
        )
        order = []
        vision = SimpleNamespace(
            start=lambda roles: order.append(tuple(roles)),
            observe_gripper=lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()),
            close=lambda: order.append("vision-close"),
        )
        with (
            patch.object(report_material.SerialLink, "from_config", return_value=link) as make_link,
            patch.object(report_material.Stm32MaterialVisionPublisher, "from_config", return_value="reporter"),
            patch.object(report_material, "build_material_pipeline", return_value=SimpleNamespace(detector="detector", calibrator=None)),
            patch.object(report_material, "DualCameraManager", return_value="manager"),
            patch.object(report_material, "DualCameraVisionController", return_value=vision) as make_vision,
            patch.object(report_material.logging, "basicConfig"),
            patch.object(report_material.signal, "signal"), patch("builtins.print"),
        ):
            self.assertEqual(report_material.main(["--enable-stm32-session"]), 0)
        self.assertEqual(make_link.call_args.kwargs["additional_capabilities"], 0x40)
        self.assertEqual(make_vision.call_args.kwargs["material_reporter"], "reporter")
        self.assertEqual(order, [("gripper",), "link-open", "vision-close", "link-close"])


class MaterialCInteropTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which("gcc")
        if compiler is None:
            raise unittest.SkipTest("没有 C 编译器，无法运行接收端交叉测试")
        cls.temporary = tempfile.TemporaryDirectory(prefix="material-vision-test-")
        cls.addClassCleanup(cls.temporary.cleanup)
        root = Path(__file__).resolve().parents[1]
        cls.executable = Path(cls.temporary.name) / "test_material_vision.exe"
        # Some MinGW bundles ship ld.bfd.exe without an ld.exe alias.
        linker_options = ["-fuse-ld=bfd"] if Path(compiler).with_name("ld.bfd.exe").is_file() else []
        result = subprocess.run([
            compiler, *linker_options, "-std=c99", "-Wall", "-Wextra", "-Werror",
            "-I", str(root / "stm32_firmware/Comm/Inc"),
            str(root / "stm32_firmware/Comm/Src/rpi_material_vision.c"),
            str(root / "tests/c/test_rpi_material_vision.c"), "-o", str(cls.executable),
        ], capture_output=True, text=True, timeout=30)
        if result.returncode:
            raise RuntimeError(f"C 接收端编译失败：{result.stderr}")

    def test_python_payload_matches_c_receiver_and_expiry_replay_guards(self):
        result = subprocess.run(
            [str(self.executable)], input=sample_packet().encode_command_data(),
            capture_output=True, timeout=5,
        )
        self.assertEqual(result.returncode, 0, result.stderr.decode(errors="replace"))


if __name__ == "__main__":
    unittest.main()
