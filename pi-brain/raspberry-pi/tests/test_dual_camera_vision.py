"""双摄像头视觉协调层的无硬件测试。"""

import unittest
from types import SimpleNamespace

from robot_control.dual_camera_vision import DualCameraVisionController
from robot_control.navigation_runtime import build_navigation_runtime
from robot_hardware.stm32.serial_link import SerialLink


class FakeCamera:
    def __init__(self, frame):
        self.frame = frame
        self.capture_count = 0

    def capture_array(self, stream):
        if stream != "main":
            raise AssertionError("测试只允许main流")
        self.capture_count += 1
        return self.frame


class FakeManager:
    def __init__(self):
        self.front = FakeCamera("front-frame")
        self.gripper = FakeCamera("gripper-frame")
        self.started = False
        self.closed = False
        self.stale_roles = set()

    def start(self, roles=None):
        self.started = True
        self.roles = set(roles or ("front", "gripper"))

    def is_healthy(self, _stale_after_seconds=1.0, roles=None):
        requested = set(roles or self.roles)
        return (
            self.started and not self.closed and requested <= self.roles
            and not requested & self.stale_roles
        )

    def close(self):
        self.closed = True


class FakeLineDetector:
    def __init__(self):
        self.frames = []

    def detect(self, frame):
        self.frames.append(frame)
        return SimpleNamespace(error=0.25)


class FakeLineFollower:
    def update(self, error):
        return SimpleNamespace(command="RIGHT", error=error, line_found=True)


class FakeTaskCodeReader:
    def __init__(self):
        self.frames = []

    def update_frame(self, frame):
        self.frames.append(frame)
        return "452+321+254+312"


class FakeMaterialDetector:
    def __init__(self):
        self.calls = []

    def detect(self, frame, target_material_code=None, collect_masks=False):
        self.calls.append((frame, target_material_code, collect_masks))
        return SimpleNamespace(status="READY", safe_to_pick=True)


class FakeObstacleSource:
    def __init__(self):
        self.updates = []

    def update(self, frame, pose, *, observed_at):
        self.updates.append((frame, pose, observed_at))

    def candidates(self):
        return ("candidate",)

    def obstacles(self):
        return ("confirmed",)


class FakeRoadDetector:
    def __init__(self):
        self.frames = []

    def detect(self, frame, *, observed_at):
        self.frames.append((frame, observed_at))
        return SimpleNamespace(boundary_safe=True, observed_at=observed_at)


class DualCameraVisionTests(unittest.TestCase):
    def make_controller(self, **kwargs):
        self.manager = FakeManager()
        self.line_detector = FakeLineDetector()
        self.task_reader = FakeTaskCodeReader()
        self.material_detector = FakeMaterialDetector()
        return DualCameraVisionController(
            self.manager,
            self.line_detector,
            FakeLineFollower(),
            self.task_reader,
            self.material_detector,
            **kwargs,
        )

    def test_camera_aliases_match_user_roles(self):
        controller = self.make_controller()

        self.assertIs(controller.camera_1, self.manager.gripper)
        self.assertIs(controller.camera_2, self.manager.front)

    def test_front_line_and_qr_share_one_frame(self):
        controller = self.make_controller()
        controller.start()

        result = controller.observe_front(scan_qr=True)

        self.assertEqual(self.manager.front.capture_count, 1)
        self.assertEqual(self.line_detector.frames, ["front-frame"])
        self.assertEqual(self.task_reader.frames, ["front-frame"])
        self.assertEqual(result.line_decision.command, "RIGHT")
        self.assertEqual(result.task_code, "452+321+254+312")

    def test_gripper_applies_calibrated_transform(self):
        def calibrator(camera):
            self.assertIs(camera, self.manager.gripper)
            return lambda frame: f"corrected:{frame}"

        controller = self.make_controller(gripper_calibrator=calibrator)
        controller.start()

        result = controller.observe_gripper(4, collect_masks=True)

        self.assertEqual(result.frame, "corrected:gripper-frame")
        self.assertEqual(
            self.material_detector.calls,
            [("corrected:gripper-frame", 4, True)],
        )
        self.assertTrue(result.material_detection.safe_to_pick)

    def test_navigation_obstacles_and_road_share_one_front_frame(self):
        obstacle_source = FakeObstacleSource()
        road_detector = FakeRoadDetector()
        controller = self.make_controller(
            obstacle_source=obstacle_source,
            road_detector=road_detector,
        )
        controller.start(("front",))
        pose = SimpleNamespace(x_mm=10, y_mm=20, yaw_rad=0.0)

        result = controller.observe_navigation(pose, observed_at=12.5)

        self.assertEqual(self.manager.front.capture_count, 1)
        self.assertEqual(obstacle_source.updates, [("front-frame", pose, 12.5)])
        self.assertEqual(road_detector.frames, [("front-frame", 12.5)])
        self.assertEqual(result.obstacle_candidates, ("candidate",))
        self.assertEqual(result.confirmed_obstacles, ("confirmed",))
        self.assertTrue(result.road_observation.boundary_safe)

    def test_use_before_start_is_rejected_and_close_releases_manager(self):
        controller = self.make_controller()
        with self.assertRaisesRegex(RuntimeError, "尚未启动"):
            controller.observe_front()

        controller.start()
        self.assertTrue(controller.is_healthy())
        controller.close()
        self.assertTrue(self.manager.closed)
        self.assertFalse(controller.is_healthy())

    def test_single_role_start_blocks_the_other_camera(self):
        controller = self.make_controller()
        controller.start(("front",))

        controller.observe_front()
        with self.assertRaisesRegex(RuntimeError, "gripper摄像头未启动"):
            controller.observe_gripper()

    def test_navigation_health_ignores_idle_gripper_but_checks_front(self):
        controller = self.make_controller(
            obstacle_source=FakeObstacleSource(), road_detector=FakeRoadDetector(),
        )
        controller.start()
        navigation = build_navigation_runtime(
            SerialLink("not-opened", heartbeat_interval=None), controller,
        )
        self.manager.stale_roles.add("gripper")
        self.assertFalse(controller.is_healthy())
        self.assertTrue(navigation.safety.camera_health(0.5))
        self.manager.stale_roles.add("front")
        self.assertFalse(navigation.safety.camera_health(0.5))
        controller.close()
        self.assertFalse(navigation.safety.camera_health(0.5))


if __name__ == "__main__":
    unittest.main()
