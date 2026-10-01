"""OPS9 遥测、静态地图和障碍改道测试。"""

from pathlib import Path
from dataclasses import replace
from types import SimpleNamespace
import struct
import unittest

from robot_control.navigation_map import CircularObstacle, NavigationMap, Pose2D
from robot_control.navigator import MapNavigator
from robot_perception.obstacle import (
    CameraObstacleDetector, ObstacleDetectorConfig,
    ConfirmedObstacleTracker, DetectedObstacle,
)
from robot_hardware.stm32.messages import (
    MessageType,
    Ops9Pose,
    Ops9Status,
    MotionFault,
    MotionFaultReason,
)
from robot_hardware.stm32.ops9 import Stm32Ops9Receiver
from robot_hardware.stm32.protocol import Frame
from robot_hardware.stm32.serial_link import SerialLink
from robot_runtime.models import ActionStatus, TargetArea


ROOT = Path(__file__).resolve().parents[1]


class Ops9TelemetryTests(unittest.TestCase):
    def test_v2_pose_uses_ops_coordinates_not_center_and_has_no_fake_quality(self):
        link = SerialLink("offline")
        link._connected_event.set()
        link.generation = 1
        receiver = Stm32Ops9Receiver(link)
        receiver.attach()
        data = b"\x02" + struct.pack("<IH8i", 100, 2, 10, 20, 30, 40, 50, 0, 0, 0)
        link._dispatch(Frame(MessageType.TELEMETRY, 2, data))
        pose = receiver.latest()
        self.assertEqual((pose.x_mm, pose.y_mm, pose.yaw_mrad), (10, 20, 30))
        self.assertIsNone(pose.quality)
        link._connected_event.clear()
        self.assertIsNone(receiver.latest())
        receiver.detach()

    def test_v2_fault_invalidates_pose_until_new_session(self):
        link = SerialLink("offline")
        link._connected_event.set()
        link.generation = 1
        receiver = Stm32Ops9Receiver(link)
        receiver.attach()
        def emit(tick):
            link._dispatch(Frame(MessageType.TELEMETRY, 2,
                b"\x02" + struct.pack("<IH8i", tick, 2, 0, 0, 0, 0, 0, 0, 0, 0)))
        emit(100)
        link._dispatch(Frame(MessageType.EVENT, 1, MotionFault(1, MotionFaultReason.OPS9_LOST).encode_event()))
        emit(101)
        self.assertIsNone(receiver.latest())
        link.generation = 2
        emit(1)  # STM32 重启后 tick 归零，不沿用旧跳变/时间戳基线
        self.assertIsNotNone(receiver.latest())
        receiver.detach()

    def test_ops9_payload_round_trip(self):
        original = Ops9Pose(
            x_mm=-123,
            y_mm=456,
            yaw_mrad=-1571,
            timestamp_ms=0xFFFFFFFE,
            quality=87,
            status=(
                Ops9Status.VALID
                | Ops9Status.CALIBRATED
                | Ops9Status.CONTACT_OK
            ),
        )
        self.assertEqual(Ops9Pose.decode_telemetry(original.encode_telemetry()), original)
        self.assertTrue(original.valid)

    def test_receiver_rejects_stale_pose(self):
        now = [10.0]
        link = SerialLink("not-opened", heartbeat_interval=None)
        receiver = Stm32Ops9Receiver(link, clock=lambda: now[0], legacy_telemetry=True)
        receiver.attach()
        pose = Ops9Pose(
            100,
            200,
            300,
            900,
            80,
            Ops9Status.VALID | Ops9Status.CALIBRATED | Ops9Status.CONTACT_OK,
        )
        link._dispatch(Frame(MessageType.LEGACY_TELEMETRY, 1, pose.encode_telemetry()))
        self.assertEqual(receiver.latest(), pose)
        now[0] += 0.3
        self.assertIsNone(receiver.latest())
        receiver.detach()

    def test_receiver_does_not_refresh_a_frozen_stm32_timestamp(self):
        now = [20.0]
        link = SerialLink("not-opened", heartbeat_interval=None)
        receiver = Stm32Ops9Receiver(link, clock=lambda: now[0], legacy_telemetry=True)
        receiver.attach()
        pose = Ops9Pose(
            0,
            0,
            0,
            1234,
            80,
            Ops9Status.VALID | Ops9Status.CALIBRATED | Ops9Status.CONTACT_OK,
        )
        link._dispatch(Frame(MessageType.LEGACY_TELEMETRY, 1, pose.encode_telemetry()))
        now[0] += 0.2
        link._dispatch(Frame(MessageType.LEGACY_TELEMETRY, 2, pose.encode_telemetry()))
        now[0] += 0.1
        self.assertIsNone(receiver.latest())
        receiver.detach()

    def test_receiver_reports_physical_motion_from_pose_change(self):
        now = [30.0]
        link = SerialLink("not-opened", heartbeat_interval=None)
        receiver = Stm32Ops9Receiver(link, clock=lambda: now[0], legacy_telemetry=True)
        receiver.attach()
        status = Ops9Status.VALID | Ops9Status.CALIBRATED | Ops9Status.CONTACT_OK
        first = Ops9Pose(0, 0, 0, 1, 90, status)
        second = Ops9Pose(5, 0, 0, 2, 90, status)
        link._dispatch(Frame(MessageType.LEGACY_TELEMETRY, 1, first.encode_telemetry()))
        link._dispatch(Frame(MessageType.LEGACY_TELEMETRY, 2, second.encode_telemetry()))
        self.assertTrue(receiver.is_moving())
        now[0] += 0.3
        self.assertFalse(receiver.is_moving())
        receiver.detach()


class MapPlannerTests(unittest.TestCase):
    def setUp(self):
        self.navigation_map = NavigationMap.load(ROOT / "config" / "navigation.json")

    def test_direct_route_uses_centre(self):
        route = self.navigation_map.plan("SOURCE", "PROCESS")
        self.assertEqual(route.nodes, ("SOURCE", "T", "C", "B", "PROCESS"))

    def test_obstacle_on_centre_edge_forces_global_detour(self):
        obstacle = CircularObstacle(1200, 1500, 25)
        blocked = self.navigation_map.blocked_edges([obstacle])
        self.assertIn("T-C", blocked)
        route = self.navigation_map.plan("SOURCE", "PROCESS", blocked_edges=blocked)
        self.assertNotIn("C", route.nodes)
        self.assertTrue(
            {"TL", "L", "BL"}.issubset(route.nodes)
            or {"TR", "R", "BR"}.issubset(route.nodes)
        )


class _VelocityRecorder:
    def __init__(self):
        self.commands = []
        self.stop_count = 0

    def set_velocity(self, vx_mm_s, vy_mm_s, wz_mrad_s):
        self.commands.append((vx_mm_s, vy_mm_s, wz_mrad_s))

    def stop(self):
        self.stop_count += 1


class NavigatorTests(unittest.TestCase):
    def test_no_route_waits_for_reobservation_then_continues(self):
        navigation_map = NavigationMap.load(ROOT / "config" / "navigation.json")
        obstacles = [CircularObstacle(1200, 1925, 60)]
        velocity = _VelocityRecorder()
        navigator = MapNavigator(
            navigation_map, lambda: Pose2D(1200, 2050, -1.5708), velocity,
            obstacle_reader=lambda: obstacles,
        )
        for _ in range(5):
            result = navigator.navigate_to(TargetArea.PROCESSING)
            self.assertEqual(result.status, ActionStatus.RUNNING)
            self.assertFalse(result.activity)
        self.assertFalse(velocity.commands)
        obstacles.clear()
        self.assertEqual(navigator.navigate_to(TargetArea.PROCESSING).status, ActionStatus.RUNNING)
        self.assertTrue(velocity.commands)

    def test_navigator_plans_and_issues_velocity(self):
        navigation_map = NavigationMap.load(ROOT / "config" / "navigation.json")
        pose = [Pose2D(1200, 2050, -1.5708)]
        velocity = _VelocityRecorder()
        navigator = MapNavigator(navigation_map, lambda: pose[0], velocity)

        result = navigator.navigate_to(TargetArea.PROCESSING)

        self.assertEqual(result.status, ActionStatus.RUNNING)
        self.assertTrue(velocity.commands)
        self.assertEqual(navigator.current_plan.nodes[0], "SOURCE")

    def test_navigator_stops_on_missing_ops9(self):
        navigation_map = NavigationMap.load(ROOT / "config" / "navigation.json")
        velocity = _VelocityRecorder()
        navigator = MapNavigator(navigation_map, lambda: None, velocity)

        result = navigator.navigate_to(TargetArea.PROCESSING)

        self.assertEqual(result.status, ActionStatus.RETRYABLE_ERROR)
        self.assertEqual(velocity.stop_count, 1)


class ObstacleTrackerTests(unittest.TestCase):
    @staticmethod
    def detection(now, confidence=0.8):
        return DetectedObstacle(1200, 1500, 60, now, confidence)

    def confirm(self, tracker):
        for now in (1.0, 1.1, 1.2):
            tracker.update([self.detection(now)], now)

    def test_intermittent_noise_does_not_accumulate_confirmation(self):
        tracker = ConfirmedObstacleTracker()
        for now in (1.0, 1.2, 1.4):
            self.assertEqual(tracker.update([self.detection(now)], now), ())
            self.assertEqual(tracker.update([], now + 0.05), ())

    def test_repeated_timestamp_and_low_confidence_do_not_confirm(self):
        tracker = ConfirmedObstacleTracker()
        for _ in range(5):
            self.assertEqual(tracker.update([self.detection(1.0)], 1.0), ())
        for now in (1.1, 1.2, 1.3):
            self.assertEqual(tracker.update([self.detection(now, 0.2)], now), ())

    def test_three_visible_clear_frames_remove_confirmed_false_positive(self):
        tracker = ConfirmedObstacleTracker()
        self.confirm(tracker)
        self.assertTrue(tracker.update([], 1.3, visible=lambda _o: True))
        self.assertTrue(tracker.update([], 1.4, visible=lambda _o: True))
        self.assertEqual(tracker.update([], 1.5, visible=lambda _o: True), ())

    def test_confirmed_obstacle_outside_view_is_not_cleared_by_timeout(self):
        tracker = ConfirmedObstacleTracker()
        self.confirm(tracker)
        self.assertTrue(tracker.update([], 5.0, visible=lambda _o: False))
        self.assertTrue(tracker.update([], 10.0, visible=lambda _o: False))

    def test_disappearance_outside_view_resets_clear_confirmation(self):
        tracker = ConfirmedObstacleTracker()
        self.confirm(tracker)
        tracker.update([], 1.3, visible=lambda _o: True)
        tracker.update([], 1.4, visible=lambda _o: False)
        self.assertTrue(tracker.update([], 1.5, visible=lambda _o: True))
        self.assertTrue(tracker.update([], 1.6, visible=lambda _o: True))
        self.assertEqual(tracker.update([], 1.7, visible=lambda _o: True), ())

    def test_unknown_camera_visibility_does_not_count_clear_frames(self):
        config = replace(
            ObstacleDetectorConfig.load(ROOT / "config" / "obstacle.json"),
            calibration_required=False,
        )
        detector = CameraObstacleDetector(config)
        pose = Pose2D(0, 0, 0)
        frame = SimpleNamespace(shape=(480, 640, 3))
        self.assertTrue(detector.is_visible(DetectedObstacle(400, 0, 60, 1, 0.8), pose, frame))
        self.assertFalse(detector.is_visible(DetectedObstacle(-400, 0, 60, 1, 0.8), pose, frame))
        self.assertFalse(detector.is_visible(DetectedObstacle(2000, 0, 60, 1, 0.8), pose, frame))
        self.assertFalse(detector.is_visible(DetectedObstacle(400, 0, 60, 1, 0.8), pose, object()))

    def test_requires_three_consistent_frames(self):
        tracker = ConfirmedObstacleTracker(confirmations_required=3)
        first = DetectedObstacle(1200, 1500, 60, 1.0, 0.8)
        second = DetectedObstacle(1210, 1495, 60, 1.1, 0.85)
        third = DetectedObstacle(1195, 1505, 60, 1.2, 0.9)

        self.assertEqual(tracker.update([first], 1.0), ())
        self.assertEqual(tracker.update([second], 1.1), ())
        self.assertEqual(tracker.update([third], 1.2), (third,))


if __name__ == "__main__":
    unittest.main()
