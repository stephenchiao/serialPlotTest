"""抓取摄像头颜色与对准结果测试。"""

import unittest

from robot_perception.material import GripperMaterialDetector


class FakeFrame:
    shape = (480, 640, 3)


class FakeColorDetector:
    def __init__(self, state):
        self.state = state

    def detect(self, _frame, collect_masks=False):
        masks = ["mask"] if collect_masks else []
        return self.state, masks


def make_detection(code=4, center=(330, 232), confirmed=True):
    return {
        "code": code,
        "name": "GREEN",
        "cn_name": "绿色",
        "center": list(center),
        "box": [300, 200, 60, 64],
        "area": 2300.0,
        "confirmed": confirmed,
    }


class GripperMaterialDetectorTests(unittest.TestCase):
    def make_detector(self, state, **overrides):
        config = {
            "grip_center": [320, 240],
            "alignment_tolerance_pixels": [18, 18],
            "require_global_ready": False,
        }
        config.update(overrides)
        return GripperMaterialDetector(FakeColorDetector(state), config)

    def test_reports_color_center_offset_and_ready(self):
        state = {
            "status": "SEARCHING",
            "safe_to_pick": False,
            "detections": [make_detection()],
        }
        detector = self.make_detector(state)

        result = detector.detect(FakeFrame(), target_material_code=4)

        self.assertEqual(result.status, "READY")
        self.assertTrue(result.safe_to_pick)
        self.assertTrue(result.aligned)
        self.assertEqual(result.observation.material_code, 4)
        self.assertEqual(result.observation.color_cn_name, "绿色")
        self.assertEqual(result.observation.offset_pixels, (10.0, -8.0))

    def test_wrong_target_is_not_safe_to_pick(self):
        state = {
            "status": "READY",
            "safe_to_pick": True,
            "detections": [make_detection(code=4)],
        }
        detector = self.make_detector(state)

        result = detector.detect(FakeFrame(), target_material_code=2)

        self.assertEqual(result.status, "TARGET_NOT_FOUND")
        self.assertIsNone(result.observation)
        self.assertFalse(result.safe_to_pick)

    def test_ambiguous_color_blocks_pick(self):
        state = {
            "status": "AMBIGUOUS",
            "safe_to_pick": False,
            "detections": [make_detection()],
        }
        detector = self.make_detector(state)

        result = detector.detect(FakeFrame(), target_material_code=4)

        self.assertFalse(result.safe_to_pick)
        self.assertEqual(result.status, "AMBIGUOUS")


if __name__ == "__main__":
    unittest.main()
