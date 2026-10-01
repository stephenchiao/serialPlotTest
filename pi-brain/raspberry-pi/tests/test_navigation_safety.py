"""导航独立安全监控测试。"""

from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
import unittest

from robot_control.navigation_map import NavigationMap, Pose2D
from robot_hardware.camera import CameraError
from robot_runtime.models import SafetyReport
from robot_services.navigation_safety import (
    NavigationSafetyConfig,
    NavigationSafetyMonitor,
)


ROOT = Path(__file__).resolve().parents[1]


class _Chassis:
    commanded_motion_active = True
    commanded_speed_mm_s = 300.0


class NavigationSafetyTests(unittest.TestCase):
    def setUp(self):
        self.map = NavigationMap.load(ROOT / "config" / "navigation.json")
        self.config = NavigationSafetyConfig.load(
            ROOT / "config" / "navigation_safety.json"
        )
        self.now = 10.0
        self.pose = Pose2D(1200, 1200, 0.0)
        self.road = SimpleNamespace(observed_at=10.0, boundary_safe=True)
        self.candidates = []
        self.chassis = _Chassis()

    def monitor(self, **overrides):
        options = dict(
            link_health=lambda: True,
            camera_health=lambda _age: True,
            road_observation_reader=lambda: self.road,
            obstacle_candidate_reader=lambda: self.candidates,
            config=self.config,
            clock=lambda: self.now,
        )
        options.update(overrides)
        return NavigationSafetyMonitor(
            self.map,
            lambda: self.pose,
            self.chassis,
            **options,
        )

    def test_dynamic_braking_distance_increases_with_speed(self):
        self.assertGreater(
            self.config.stopping_distance_mm(400),
            self.config.stopping_distance_mm(100),
        )

    def test_candidate_inside_braking_envelope_requests_immediate_recheck_stop(self):
        self.candidates = [
            SimpleNamespace(x_mm=1500, y_mm=1200, radius_mm=60)
        ]

        report = self.monitor().check()

        self.assertFalse(report.safe)
        self.assertTrue(report.recoverable)
        self.assertFalse(report.emergency_stop)
        self.assertIn("制动包络", report.reason)

    def test_visual_forbidden_area_pauses_without_claiming_actual_boundary_violation(self):
        self.road = SimpleNamespace(observed_at=10.0, boundary_safe=False)

        report = self.monitor().check()

        self.assertFalse(report.safe)
        self.assertTrue(report.boundary_ok)
        self.assertTrue(report.recoverable)

    def test_actual_map_boundary_still_latches_stop(self):
        self.pose = Pose2D(50, 1200)
        report = self.monitor().check()
        self.assertFalse(report.boundary_ok)
        self.assertTrue(report.emergency_stop)
        self.assertFalse(report.recoverable)

    def test_side_obstacle_outside_swept_corridor_does_not_stop(self):
        self.candidates = [SimpleNamespace(x_mm=1300, y_mm=1500, radius_mm=60)]
        self.assertTrue(self.monitor().check().safe)

    def test_sensor_loss_requires_valid_pose_even_after_stopping(self):
        monitor = self.monitor()
        monitor.begin_recheck()
        self.chassis.commanded_motion_active = False
        self.pose = None
        self.assertTrue(monitor.check().recoverable)

    def test_pause_retains_original_braking_envelope(self):
        self.candidates = [SimpleNamespace(x_mm=1700, y_mm=1200, radius_mm=60)]
        monitor = self.monitor()
        self.assertFalse(monitor.check().safe)
        monitor.begin_recheck()
        self.chassis.commanded_motion_active = False
        self.chassis.commanded_speed_mm_s = 0
        self.assertFalse(monitor.check().safe)

    def test_pause_does_not_resume_before_old_goal_stops(self):
        monitor = self.monitor()
        monitor.begin_recheck()
        self.assertIn("旧航点", monitor.check().reason)

    def test_recheck_collects_new_frame_before_camera_health_and_road_check(self):
        self.chassis.commanded_motion_active = False
        self.road = None
        captures = []

        def observe(pose):
            captures.append(pose)
            self.road = SimpleNamespace(observed_at=self.now, boundary_safe=True)

        monitor = self.monitor(
            perception_updater=observe,
            camera_health=lambda _age: bool(captures),
        )
        monitor.begin_recheck()
        self.assertTrue(monitor.check().safe)
        self.assertEqual(len(captures), 1)

    def test_navigation_requires_fresh_road_before_first_goal(self):
        self.chassis.commanded_motion_active = False
        self.road = None
        monitor = self.monitor()
        monitor.set_navigation_required(True)
        self.assertTrue(monitor.check().recoverable)

    def test_capture_timeout_pauses_but_algorithm_bug_latches(self):
        def timeout(_pose):
            raise TimeoutError("frame timeout")

        def bug(_pose):
            raise ValueError("bad matrix")

        self.assertTrue(self.monitor(perception_updater=timeout).check().recoverable)
        self.assertTrue(self.monitor(perception_updater=bug).check().emergency_stop)

    def test_link_loss_waits_for_reconnect_but_controller_fault_remains_fatal(self):
        report = self.monitor(link_health=lambda: False).check()
        self.assertTrue(report.recoverable)
        self.assertTrue(report.waiting_for_link)
        self.assertFalse(report.emergency_stop)
        report = self.monitor(motion_fault_reader=lambda: "CAN_FAULT").check()
        self.assertTrue(report.emergency_stop)
        self.assertEqual(report.reason, "CAN_FAULT")
        report = self.monitor(
            link_health=lambda: False, motion_fault_reader=lambda: "CAN_FAULT",
        ).check()
        self.assertTrue(report.emergency_stop)

    def test_stale_or_future_road_never_releases_pause(self):
        self.chassis.commanded_motion_active = False
        monitor = self.monitor()
        monitor.begin_recheck()
        for timestamp in (8, 11, float("nan")):
            self.road.observed_at = timestamp
            self.assertFalse(monitor.check().safe)

    def test_missing_road_is_allowed_only_when_no_motion_is_commanded(self):
        self.road = None
        self.chassis.commanded_motion_active = False
        report = self.monitor().check()
        self.assertEqual(report, SafetyReport())

        self.chassis.commanded_motion_active = True
        self.assertFalse(self.monitor().check().safe)

    def test_transient_vision_failures_keep_existing_goal_but_are_time_bounded(self):
        for failure in ("road", "missing", "stale", "camera", "capture", "camera_error"):
            with self.subTest(failure=failure):
                self.now = 10.0
                self.road = SimpleNamespace(observed_at=self.now, boundary_safe=True)
                failed = False

                def capture(_pose):
                    if failed and failure == "capture":
                        raise TimeoutError("temporary frame timeout")
                    if failed and failure == "camera_error":
                        raise CameraError("temporary camera failure")

                monitor = self.monitor(
                    perception_updater=capture,
                    camera_health=lambda _age: not (failed and failure == "camera"),
                )
                self.assertTrue(monitor.check().safe)
                failed = True
                self.now += 0.1
                if failure == "road":
                    self.road = SimpleNamespace(observed_at=self.now, boundary_safe=False)
                elif failure == "missing":
                    self.road = None
                elif failure == "stale":
                    self.road.observed_at = 8.0

                report = monitor.check()
                self.assertTrue(report.safe)
                self.assertIn("短时降级", report.reason)
                self.assertIsNone(report.observation_timestamp)
                self.now = 10.0 + self.config.perception_grace_seconds + 0.01
                self.assertTrue(monitor.check().recoverable)

    def test_recovered_vision_can_continue_without_a_pause(self):
        monitor = self.monitor()
        self.assertTrue(monitor.check().safe)
        self.now += 0.1
        self.road.boundary_safe = False
        self.assertTrue(monitor.check().safe)
        self.now += 0.1
        self.road = SimpleNamespace(observed_at=self.now, boundary_safe=True)
        self.assertEqual(monitor.check().observation_timestamp, self.now)
        self.now += 0.1
        self.road.boundary_safe = False
        self.assertTrue(monitor.check().safe)

    def test_alternating_failures_and_repeated_frames_do_not_extend_grace(self):
        monitor = self.monitor()
        self.assertTrue(monitor.check().safe)
        self.now += 0.1
        self.road.boundary_safe = False
        self.assertTrue(monitor.check().safe)
        self.now += 0.1
        self.road.boundary_safe = True  # 重复旧帧，不能重置时间戳。
        self.assertTrue(monitor.check().safe)
        self.now += self.config.perception_grace_seconds
        self.road = None
        self.assertTrue(monitor.check().recoverable)

    def test_grace_never_masks_close_obstacle_pose_loss_or_map_boundary(self):
        for failure in ("obstacle", "pose", "boundary"):
            with self.subTest(failure=failure):
                self.pose = Pose2D(1200, 1200)
                self.road = SimpleNamespace(observed_at=self.now, boundary_safe=True)
                self.candidates = []
                monitor = self.monitor()
                self.assertTrue(monitor.check().safe)
                self.now += 0.1
                self.road.boundary_safe = False
                if failure == "obstacle":
                    self.candidates = [SimpleNamespace(x_mm=1500, y_mm=1200, radius_mm=60)]
                elif failure == "pose":
                    self.pose = None
                else:
                    self.pose = Pose2D(50, 1200)
                self.assertFalse(monitor.check().safe)

    def test_grace_does_not_release_recheck_or_allow_start_without_vision(self):
        monitor = self.monitor()
        self.assertTrue(monitor.check().safe)
        self.now += 0.1
        self.road.boundary_safe = False
        self.chassis.commanded_motion_active = False
        monitor.set_navigation_required(True)
        self.assertTrue(monitor.check().recoverable)
        monitor.begin_recheck()
        self.assertTrue(monitor.check().recoverable)

    def test_non_navigation_stage_ignores_front_camera_health(self):
        self.chassis.commanded_motion_active = False
        self.road = None
        monitor = self.monitor(camera_health=lambda _age: False)
        self.assertEqual(monitor.check(), SafetyReport())
        monitor.set_navigation_required(True)
        self.assertTrue(monitor.check().recoverable)

    def test_zero_grace_restores_immediate_vision_pause(self):
        monitor = self.monitor(config=replace(self.config, perception_grace_seconds=0))
        self.assertTrue(monitor.check().safe)
        self.road.boundary_safe = False
        self.assertTrue(monitor.check().recoverable)


if __name__ == "__main__":
    unittest.main()
