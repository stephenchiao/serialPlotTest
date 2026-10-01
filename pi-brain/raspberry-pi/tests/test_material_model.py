"""无需训练权重或真实相机的 cam0 模型框架测试。"""

from dataclasses import replace
import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from robot_control.dual_camera_vision import build_dual_camera_vision
from robot_perception.material import (
    GripperMaterialDetector, ModelDetection, ModelMaterialSource,
    UltralyticsYoloBackend, build_material_pipeline, load_material_config,
)
from robot_perception.material.config import MaterialClass, MaterialConfigError
from robot_perception.material.model import ModelInferenceError

if importlib.util.find_spec("numpy"):
    # 在 patch.dict(sys.modules) 前加载，避免模块恢复时卸载/重载 NumPy。
    import numpy


GRIPPER = {
    "grip_center": [320, 240],
    "alignment_tolerance_pixels": [18, 18],
    "require_global_ready": False,
}


class Frame:
    shape = (480, 640, 3)


def prediction(class_id=0, confidence=0.9, xyxy=(300, 220, 340, 260)):
    return ModelDetection(class_id, f"class_{class_id}", confidence, xyxy)


class FakeInference:
    def __init__(self, predictions=None):
        self.predictions = [prediction()] if predictions is None else predictions
        self.calls = 0
        self.error = None

    def predict(self, _frame):
        self.calls += 1
        if self.error is not None:
            raise self.error
        return self.predictions


class MaterialConfigTests(unittest.TestCase):
    def load_data(self, data):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "material.json"
            path.write_text(json.dumps(data), encoding="utf-8")
            config = load_material_config(path)
            return config, path

    def test_default_model_is_deliberately_unconfigured(self):
        config = load_material_config()
        self.assertEqual(config.backend, "model")
        self.assertIsNone(config.weights)
        self.assertEqual(config.class_to_material, {})

    def test_relative_weights_use_config_directory_and_no_implicit_numbering(self):
        config, path = self.load_data({
            "model": {"weights": "../weights/custom.pt"},
            "class_to_material": {"0": {"material_code": 42, "name": "sample"}},
        })
        self.assertEqual(config.weights.resolve(), (path.parent / "../weights/custom.pt").resolve())
        self.assertEqual(config.class_to_material[0].material_code, 42)

    def test_invalid_and_duplicate_material_mappings_are_rejected(self):
        bad_mappings = [
            {"-1": {"material_code": 1}},
            {"01": {"material_code": 1}},
            {"0": {"material_code": 0}},
            {"0": {"material_code": True}},
            {"0": 1},
            {"0": {"material_code": 1}, "1": {"material_code": 1}},
        ]
        for mapping in bad_mappings:
            with self.subTest(mapping=mapping), self.assertRaises(MaterialConfigError):
                self.load_data({"class_to_material": mapping})

    def test_invalid_settings_are_rejected(self):
        bad_settings = [
            {"backend": "automatic"},
            {"model": {"confidence_threshold": float("nan")}},
            {"model": {"confidence_threshold": 1.1}},
            {"model": {"input_color_order": "GRAY"}},
            {"model": {"weights": ""}},
            {"model": {"image_size": True}},
            {"confirmation": {"required_frames": 0}},
            {"confirmation": {"matching_iou": 0}},
            {"confirmation": {"maximum_gap_seconds": float("inf")}},
        ]
        for data in bad_settings:
            with self.subTest(data=data), self.assertRaises(MaterialConfigError):
                self.load_data(data)


class ModelMaterialTests(unittest.TestCase):
    def setUp(self):
        self.config = replace(
            load_material_config(), weights=Path("fake-not-loaded.pt"),
            class_to_material={0: MaterialClass(42, "part_a"), 1: MaterialClass(7, "part_b")},
        )
        self.inference = FakeInference()
        self.now = 0.0
        self.source = ModelMaterialSource(
            self.config, inference_backend=self.inference, clock=lambda: self.now,
        )
        self.detector = GripperMaterialDetector(self.source, GRIPPER)

    def observe(self, code=42):
        return self.detector.detect(Frame(), target_material_code=code)

    def test_default_factory_needs_no_model_library_or_weights(self):
        pipeline = build_material_pipeline(GRIPPER)
        with patch.object(pipeline.detector.source_detector.inference, "predict") as predict:
            result = pipeline.detector.detect(Frame())
        self.assertFalse(predict.called)
        self.assertIsNone(pipeline.calibrator)
        self.assertEqual(result.status, "MODEL_NOT_CONFIGURED")
        self.assertFalse(result.safe_to_pick)
        self.assertEqual(result.backend, "model")

    def test_class_zero_can_map_to_any_explicit_positive_material_code(self):
        first = self.observe()
        second = self.observe()
        third = self.observe()
        self.assertEqual(first.status, "CONFIRMING_MATERIAL")
        self.assertFalse(second.safe_to_pick)
        self.assertTrue(third.safe_to_pick)
        self.assertEqual(third.observation.class_id, 0)
        self.assertEqual(third.observation.material_code, 42)
        self.assertEqual(third.observation.confidence, 0.9)
        self.assertEqual(third.observation.material_name, "part_a")
        self.assertEqual(third.detected_material_codes, (42,))
        self.assertEqual(third.observation.offset_pixels, (0.0, 0.0))

    def test_confirmed_but_misaligned_target_does_not_allow_pick(self):
        self.inference.predictions = [prediction(xyxy=(400, 220, 440, 260))]
        for _ in range(3):
            result = self.observe()
        self.assertEqual(result.status, "ALIGNING")
        self.assertFalse(result.safe_to_pick)
        self.assertEqual(result.observation.offset_pixels, (100.0, 0.0))

    def test_low_confidence_and_small_boxes_are_filtered(self):
        for item in [prediction(confidence=0.2), prediction(xyxy=(310, 230, 312, 232))]:
            self.inference.predictions = [item]
            for _ in range(4):
                result = self.observe()
            self.assertIsNone(result.observation)
            self.assertFalse(result.safe_to_pick)

    def test_missing_frame_resets_confirmation(self):
        self.observe()
        self.observe()
        self.inference.predictions = []
        self.assertFalse(self.observe().safe_to_pick)
        self.inference.predictions = [prediction()]
        self.assertFalse(self.observe().observation.confirmed)

    def test_same_class_at_a_different_position_restarts_confirmation(self):
        self.observe()
        self.observe()
        self.inference.predictions = [prediction(xyxy=(450, 220, 490, 260))]
        self.assertFalse(self.observe().observation.confirmed)

    def test_long_frame_gap_restarts_confirmation(self):
        self.observe()
        self.observe()
        self.now = 2.0
        self.assertFalse(self.observe().observation.confirmed)

    def test_switching_target_restarts_confirmation(self):
        self.inference.predictions = [prediction(), prediction(1, xyxy=(310, 230, 350, 270))]
        for _ in range(3):
            ready = self.observe(42)
        self.assertTrue(ready.safe_to_pick)
        self.assertFalse(self.observe(7).observation.confirmed)

    def test_unmapped_class_blocks_pick_even_with_a_known_aligned_target(self):
        self.inference.predictions = [prediction(), prediction(99)]
        for _ in range(3):
            result = self.observe()
        self.assertEqual(result.status, "UNMAPPED_CLASS")
        self.assertFalse(result.safe_to_pick)
        self.assertEqual(len(result.raw_model_detections), 2)

    def test_no_mapping_preserves_raw_classes_for_preview_but_never_guesses_code(self):
        source = ModelMaterialSource(
            replace(self.config, class_to_material={}), inference_backend=self.inference,
        )
        result = GripperMaterialDetector(source, GRIPPER).detect(Frame())
        self.assertEqual(result.status, "MAPPING_NOT_CONFIGURED")
        self.assertIsNone(result.observation)
        self.assertEqual(result.raw_model_detections[0].class_id, 0)
        self.assertFalse(result.safe_to_pick)

    def test_duplicate_instances_are_ambiguous(self):
        self.inference.predictions = [prediction(), prediction(xyxy=(400, 220, 440, 260))]
        result = self.observe()
        self.assertEqual(result.status, "AMBIGUOUS")
        self.assertFalse(result.safe_to_pick)

    def test_unconfigured_target_is_distinct_from_missing_mapped_target(self):
        self.assertEqual(self.observe(999).status, "TARGET_NOT_MAPPED")
        self.assertEqual(self.observe(7).status, "TARGET_NOT_FOUND")

    def test_inference_failure_clears_confirmation_and_blocks_pick(self):
        for _ in range(3):
            self.observe()
        self.inference.error = RuntimeError("test failure")
        result = self.observe()
        self.assertEqual(result.status, "INFERENCE_ERROR")
        self.assertFalse(result.safe_to_pick)
        self.inference.error = None
        self.assertFalse(self.observe().observation.confirmed)

    def test_invalid_model_output_is_fail_closed(self):
        for item in [
            prediction(confidence=float("nan")),
            prediction(xyxy=(340, 220, 300, 260)),
            prediction(xyxy=(300, 220, float("inf"), 260)),
            prediction(class_id=-1),
        ]:
            self.inference.predictions = [item]
            with self.subTest(item=item):
                result = self.observe()
                self.assertEqual(result.status, "INFERENCE_ERROR")
                self.assertFalse(result.safe_to_pick)

    def test_boxes_are_clipped_to_original_frame(self):
        self.inference.predictions = [prediction(xyxy=(-10, 200, 30, 280))]
        result = self.observe()
        self.assertEqual(result.observation.box, (0, 200, 30, 80))


class FakeTensor:
    def __init__(self, values):
        self.values = values

    def cpu(self):
        return self

    def tolist(self):
        return self.values


@unittest.skipUnless(importlib.util.find_spec("numpy"), "adapter input tests require numpy")
class UltralyticsAdapterTests(unittest.TestCase):
    def setUp(self):
        import numpy as np

        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.weights = Path(self.directory.name) / "fake.pt"
        self.weights.write_bytes(b"not a real model; mock loader only")
        self.config = replace(load_material_config(), weights=self.weights)
        self.frame = np.zeros((480, 640, 3), dtype=np.uint8)
        boxes = SimpleNamespace(
            xyxy=FakeTensor([[300, 220, 340, 260]]),
            cls=FakeTensor([0.0]), conf=FakeTensor([0.9]),
        )
        self.model = Mock()
        self.model.predict.return_value = [SimpleNamespace(boxes=boxes, names={0: "sample"})]
        self.loader = Mock(return_value=self.model)

    def test_local_model_is_loaded_once_and_output_keeps_class_zero(self):
        backend = UltralyticsYoloBackend(self.config, model_loader=self.loader)
        first = backend.predict(self.frame)
        backend.predict(self.frame)
        self.loader.assert_called_once_with(str(self.weights))
        self.assertEqual(first[0].class_id, 0)
        self.assertEqual(first[0].xyxy, (300.0, 220.0, 340.0, 260.0))
        kwargs = self.model.predict.call_args.kwargs
        self.assertEqual(kwargs["device"], "cpu")
        self.assertEqual(kwargs["conf"], 0.6)
        self.assertEqual(kwargs["imgsz"], 640)
        self.assertFalse(kwargs["save"])

    def test_rgb_is_explicitly_converted_to_contiguous_bgr(self):
        self.frame[0, 0] = [10, 20, 30]
        backend = UltralyticsYoloBackend(
            replace(self.config, input_color_order="RGB"), model_loader=self.loader,
        )
        backend.predict(self.frame)
        image = self.model.predict.call_args.kwargs["source"]
        self.assertEqual(image[0, 0].tolist(), [30, 20, 10])
        self.assertTrue(image.flags.c_contiguous)

    def test_missing_file_never_calls_loader_or_downloads_weights(self):
        backend = UltralyticsYoloBackend(
            replace(self.config, weights=self.weights.parent / "missing.pt"),
            model_loader=self.loader,
        )
        with self.assertRaises(ModelInferenceError):
            backend.predict(self.frame)
        self.assertFalse(self.loader.called)

    def test_wrong_model_task_is_rejected(self):
        self.model.predict.return_value = [SimpleNamespace(boxes=None)]
        with self.assertRaises(ModelInferenceError):
            UltralyticsYoloBackend(self.config, model_loader=self.loader).predict(self.frame)

    def test_missing_optional_dependency_is_reported_without_loading_weights(self):
        with patch.dict(sys.modules, {"ultralytics": None}):
            with self.assertRaisesRegex(ModelInferenceError, "ModuleNotFoundError"):
                UltralyticsYoloBackend(self.config).predict(self.frame)


@unittest.skipUnless(importlib.util.find_spec("numpy"), "factory integration requires numpy")
class MaterialEntryPointTests(unittest.TestCase):
    def fake_cv2(self):
        return SimpleNamespace(setNumThreads=Mock(), QRCodeDetector=Mock())

    def test_dual_camera_factory_uses_unconfigured_model_not_hsv(self):
        reporter = Mock()
        with patch.dict(sys.modules, {
            "cv2": self.fake_cv2(),
            "robot_perception.line": SimpleNamespace(LineDetector=Mock()),
        }):
            vision = build_dual_camera_vision(material_reporter=reporter)
        self.assertIs(vision.material_reporter, reporter)
        self.assertIsInstance(vision.material_detector.source_detector, ModelMaterialSource)
        self.assertIsNone(vision.gripper_calibrator)
        result = vision.material_detector.detect(Frame())
        self.assertEqual(result.status, "MODEL_NOT_CONFIGURED")
        self.assertFalse(result.safe_to_pick)
        self.assertEqual(vision.camera_manager.gripper.camera_num, 0)
        self.assertEqual(vision.camera_manager.front.camera_num, 1)

    def test_color_backend_remains_an_explicit_opt_in(self):
        color_module = SimpleNamespace(
            CompetitionColorDetector=Mock(),
            apply_white_balance=Mock(), build_white_balance_luts=Mock(),
            calibrate_white_balance=Mock(),
        )
        with patch.dict(sys.modules, {"robot_perception.color.detector": color_module}):
            pipeline = build_material_pipeline(GRIPPER, backend="color")
        self.assertNotIsInstance(pipeline.detector.source_detector, ModelMaterialSource)
        self.assertIsNotNone(pipeline.calibrator)

    def test_single_camera_debug_starts_without_weights_and_never_claims_ready(self):
        from tools import debug_gripper

        camera = Mock()
        camera.capture_array.side_effect = [Frame(), KeyboardInterrupt()]
        output = io.StringIO()
        with (
            patch.dict(sys.modules, {"cv2": self.fake_cv2()}),
            patch.object(sys, "argv", ["debug_gripper", "--no-preview"]),
            patch.object(debug_gripper, "PiCamera", return_value=camera),
            patch.object(debug_gripper.time, "sleep"),
            patch("sys.stdout", output),
        ):
            code = debug_gripper.main()
        self.assertEqual(code, 0)
        self.assertIn("MODEL_NOT_CONFIGURED", output.getvalue())
        self.assertIn("可抓取=False", output.getvalue())
        camera.close.assert_called_once()

    def test_debug_tools_accept_future_material_codes_without_guessing_one_to_six(self):
        from tools import debug_dual_camera, debug_gripper

        for module in (debug_dual_camera, debug_gripper):
            with patch.object(sys, "argv", ["debug", "--target-code", "42"]):
                self.assertEqual(module.parse_arguments().target_code, 42)

    def test_dual_debug_propagates_material_config_and_starts_only_cam0(self):
        from tools import debug_dual_camera

        vision = Mock()
        vision.observe_gripper.side_effect = KeyboardInterrupt()
        with (
            patch.object(sys, "argv", [
                "debug", "--mode", "gripper", "--material-config", "custom.json",
                "--material-backend", "model",
            ]),
            patch.object(debug_dual_camera, "build_dual_camera_vision", return_value=vision) as build,
            patch("sys.stdout", io.StringIO()),
        ):
            result = debug_dual_camera.main()
        self.assertEqual(result, 0)
        self.assertEqual(build.call_args.kwargs["material_config_path"], "custom.json")
        self.assertEqual(build.call_args.kwargs["material_backend"], "model")
        vision.start.assert_called_once_with(["gripper"])
        vision.observe_front.assert_not_called()
        vision.close.assert_called_once()


if __name__ == "__main__":
    unittest.main()
