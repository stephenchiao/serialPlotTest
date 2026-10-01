"""树莓派5双摄像头配置与生命周期的无硬件测试。"""

import unittest

from robot_hardware.camera import DualCameraManager, PiCamera, load_camera_config


class FakeCamera:
    instances = []

    def __init__(self, config):
        self.config = dict(config)
        self.started = False
        self.closed = False
        type(self).instances.append(self)

    def start(self):
        self.started = True

    def is_healthy(self, _stale_after_seconds=1.0):
        return self.started and not self.closed

    def close(self):
        self.closed = True


class FakePicamera2Device:
    def __init__(self, camera_num):
        self.camera_num = camera_num
        self.config = None
        self.started = False
        self.closed = False

    def create_preview_configuration(self, **kwargs):
        return kwargs

    def configure(self, config):
        self.config = config

    def start(self):
        self.started = True

    def stop(self):
        self.started = False

    def close(self):
        self.closed = True


class DualCameraTests(unittest.TestCase):
    def setUp(self):
        FakeCamera.instances = []

    def test_default_config_assigns_distinct_pi5_cameras(self):
        config = load_camera_config()

        self.assertEqual(config["platform"], "raspberry_pi_5")
        self.assertEqual(config["front"]["camera_num"], 1)
        self.assertEqual(config["front"]["logical_name"], "camera_2")
        self.assertEqual(config["front"]["connector"], "CAM/DISP1")
        self.assertEqual(config["gripper"]["camera_num"], 0)
        self.assertEqual(config["gripper"]["logical_name"], "camera_1")
        self.assertEqual(config["gripper"]["connector"], "CAM/DISP0")
        self.assertIsNone(config["front"]["model"])
        self.assertIsNone(config["gripper"]["model"])

    def test_manager_starts_and_closes_both_roles(self):
        manager = DualCameraManager(load_camera_config(), camera_factory=FakeCamera)

        manager.start()
        self.assertTrue(manager.is_healthy())
        self.assertTrue(manager.front.started)
        self.assertTrue(manager.gripper.started)

        manager.close()
        self.assertTrue(manager.front.closed)
        self.assertTrue(manager.gripper.closed)

    def test_pi_camera_opens_configured_camera_number(self):
        config = load_camera_config()["gripper"]
        device = None

        def factory(camera_num):
            nonlocal device
            device = FakePicamera2Device(camera_num)
            return device

        camera = PiCamera(config, device_factory=factory)
        camera.start()

        self.assertEqual(device.camera_num, 0)
        self.assertEqual(device.config["main"]["size"], (640, 480))
        camera.close()
        self.assertTrue(device.closed)

    def test_manager_can_start_only_one_role_for_low_load_debug(self):
        manager = DualCameraManager(load_camera_config(), camera_factory=FakeCamera)

        manager.start(("front",))

        self.assertTrue(manager.front.started)
        self.assertFalse(manager.gripper.started)
        self.assertTrue(manager.is_healthy(roles=("front",)))
        self.assertFalse(manager.is_healthy(roles=("front", "gripper")))


if __name__ == "__main__":
    unittest.main()
