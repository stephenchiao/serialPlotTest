"""迁移后不依赖真实硬件的配置与控制测试。"""

import unittest
from pathlib import Path

from robot_control.line_navigation import LineFollower
from robot_perception.color import load_config as load_color_config
from robot_runtime.config import load_runtime_config
from robot_perception.road import RoadAreaDetector, RoadDetectorConfig
from robot_services.navigation_safety import NavigationSafetyConfig
from robot_hardware.navigation import (
    NavigationHardwareConfigError,
    validate_real_navigation_configuration,
)


class ProjectStructureTests(unittest.TestCase):
    def test_default_configs_load_from_config_directory(self):
        runtime = load_runtime_config()
        color = load_color_config()

        self.assertGreater(runtime.loop_interval_seconds, 0)
        self.assertEqual(color["detection"]["expected_color_count"], 3)
        self.assertEqual(Path(color["_config_path"]).parts[-2:], ("config", "color.json"))

        road = RoadDetectorConfig.load(Path("config/road.json"))
        safety = NavigationSafetyConfig.load(Path("config/navigation_safety.json"))
        self.assertTrue(road.calibration_required)
        self.assertGreater(safety.minimum_braking_deceleration_mm_s2, 0)
        with self.assertRaisesRegex(ValueError, "现场标定"):
            RoadAreaDetector(road)

    def test_real_navigation_refuses_placeholder_calibration(self):
        with self.assertRaises(NavigationHardwareConfigError) as context:
            validate_real_navigation_configuration()
        self.assertIn("场地图", str(context.exception))
        self.assertIn("OPS9", str(context.exception))
        self.assertIn("单应矩阵", str(context.exception))

    def test_line_follower_stops_when_line_is_missing(self):
        follower = LineFollower(turn_threshold=0.15, smoothing=1.0)
        decision = follower.update(None)

        self.assertEqual(decision.command, "STOP")
        self.assertFalse(decision.line_found)

    def test_line_follower_turns_from_normalized_error(self):
        follower = LineFollower(turn_threshold=0.15, smoothing=1.0)

        self.assertEqual(follower.update(-0.5).command, "LEFT")
        self.assertEqual(follower.update(0.0).command, "STRAIGHT")
        self.assertEqual(follower.update(0.5).command, "RIGHT")


if __name__ == "__main__":
    unittest.main()
