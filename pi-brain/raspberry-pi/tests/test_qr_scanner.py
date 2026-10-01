"""二维码扫描器的无硬件单元测试。"""

import unittest

try:
    import numpy as np
    from robot_perception.qr import CameraTaskCodeReader, QRCodeScanner
except ModuleNotFoundError as error:
    if error.name not in {"cv2", "numpy"}:
        raise
    raise unittest.SkipTest(
        "二维码图像测试需要安装numpy和opencv"
    ) from error


class FakeDetector:
    def __init__(self, multi_values=(), single_value=""):
        self.multi_values = tuple(multi_values)
        self.single_value = single_value

    def detectAndDecodeMulti(self, _frame):
        return bool(self.multi_values), self.multi_values, None, ()

    def detectAndDecode(self, _frame):
        return self.single_value, None, None


class FakeCamera:
    def __init__(self):
        self.frame = np.zeros((32, 32, 3), dtype=np.uint8)
        self.streams = []

    def capture_array(self, stream):
        self.streams.append(stream)
        return self.frame


class SequenceScanner:
    def __init__(self, results):
        self.results = iter(results)

    def decode(self, _frame):
        return next(self.results)


class QRCodeScannerTests(unittest.TestCase):
    def test_decode_multiple_values_strips_and_deduplicates(self):
        detector = FakeDetector(
            multi_values=(" 452+321+254+312 ", "452+321+254+312", ""),
        )
        scanner = QRCodeScanner(detector=detector, enhance_fallback=False)

        result = scanner.decode(np.zeros((32, 32, 3), dtype=np.uint8))

        self.assertEqual(result, ("452+321+254+312",))

    def test_decode_falls_back_to_single_code_api(self):
        scanner = QRCodeScanner(
            detector=FakeDetector(single_value="452+321+254+312"),
            enhance_fallback=False,
        )

        result = scanner.decode(np.zeros((32, 32, 3), dtype=np.uint8))

        self.assertEqual(result, ("452+321+254+312",))

    def test_reject_invalid_frame(self):
        scanner = QRCodeScanner(detector=FakeDetector(), enhance_fallback=False)
        with self.assertRaisesRegex(ValueError, "有效图像"):
            scanner.decode(None)


class CameraTaskCodeReaderTests(unittest.TestCase):
    def test_requires_consecutive_confirmations(self):
        camera = FakeCamera()
        scanner = SequenceScanner(
            [
                ("452+321+254+312",),
                ("452+321+254+312",),
            ]
        )
        reader = CameraTaskCodeReader(camera, scanner, required_confirmations=2)

        self.assertIsNone(reader.read_task_code())
        self.assertEqual(reader.read_task_code(), "452+321+254+312")
        self.assertEqual(camera.streams, ["main", "main"])

    def test_ambiguous_frame_resets_confirmation(self):
        camera = FakeCamera()
        scanner = SequenceScanner(
            [
                ("452+321+254+312",),
                ("452+321+254+312", "123+321+321+123"),
                ("452+321+254+312",),
                ("452+321+254+312",),
            ]
        )
        reader = CameraTaskCodeReader(camera, scanner, required_confirmations=2)

        self.assertIsNone(reader.read_task_code())
        self.assertIsNone(reader.read_task_code())
        self.assertIsNone(reader.read_task_code())
        self.assertEqual(reader.read_task_code(), "452+321+254+312")

    def test_validate_confirmation_count(self):
        with self.assertRaisesRegex(ValueError, "大于等于1"):
            CameraTaskCodeReader(FakeCamera(), required_confirmations=0)

    def test_update_frame_reuses_frame_without_camera_capture(self):
        camera = FakeCamera()
        scanner = SequenceScanner([("452+321+254+312",)])
        reader = CameraTaskCodeReader(camera, scanner, required_confirmations=1)

        result = reader.update_frame(camera.frame)

        self.assertEqual(result, "452+321+254+312")
        self.assertEqual(camera.streams, [])


if __name__ == "__main__":
    unittest.main()
