"""可替换的模型推理接口，以及不允许猜测编号的安全物料适配层。"""

from dataclasses import dataclass
from math import isfinite
import time
from typing import Protocol, Sequence, Tuple

from .config import MaterialModelConfig


@dataclass(frozen=True)
class ModelDetection:
    class_id: int
    class_name: str
    confidence: float
    # 原始图像像素坐标，而非网络输入尺寸或归一化坐标。
    xyxy: Tuple[float, float, float, float]


class InferenceBackend(Protocol):
    def predict(self, frame) -> Sequence[ModelDetection]: ...


class ModelInferenceError(RuntimeError):
    pass


class UltralyticsYoloBackend:
    """仅加载用户指定的本地权重；不自动下载预训练模型。"""

    def __init__(self, config: MaterialModelConfig, *, model_loader=None):
        self.config = config
        self._model_loader = model_loader
        self._model = None

    def predict(self, frame):
        if self.config.weights is None or not self.config.weights.is_file():
            raise ModelInferenceError("指定的本地模型权重不存在")
        try:
            import numpy as np

            image = np.asarray(frame)
            if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
                raise ValueError("模型输入必须是 HWC 三通道 uint8 图像")
            if self.config.input_color_order == "RGB":
                image = image[:, :, ::-1]
            image = np.ascontiguousarray(image)
            if self._model is None:
                loader = self._model_loader
                if loader is None:
                    from ultralytics import YOLO

                    loader = YOLO
                self._model = loader(str(self.config.weights))
            results = self._model.predict(
                source=image,
                conf=self.config.confidence_threshold,
                iou=self.config.iou_threshold,
                imgsz=self.config.image_size,
                device=self.config.device,
                max_det=self.config.max_detections,
                verbose=False,
                save=False,
            )
            if len(results) != 1:
                raise ValueError("单帧推理必须返回一项结果")
            result = results[0]
            if result.boxes is None:
                raise ValueError("需要目标检测模型，不支持仅分类/姿态等输出")
            boxes = result.boxes.xyxy.cpu().tolist()
            classes = result.boxes.cls.cpu().tolist()
            scores = result.boxes.conf.cpu().tolist()
            if not len(boxes) == len(classes) == len(scores):
                raise ValueError("模型输出的框、类别、置信度数量不一致")
            detections = []
            for box, class_id, score in zip(boxes, classes, scores):
                if not isfinite(class_id) or class_id < 0 or int(class_id) != class_id:
                    raise ValueError("模型输出的类别编号无效")
                detections.append(ModelDetection(
                    int(class_id), str(result.names[int(class_id)]),
                    float(score), tuple(float(value) for value in box),
                ))
            return tuple(detections)
        except Exception as error:
            raise ModelInferenceError(
                f"模型加载/推理失败：{type(error).__name__}: {error}"
            ) from error


def _iou(first, second):
    ax1, ay1, ax2, ay2 = first.xyxy
    bx1, by1, bx2, by2 = second.xyxy
    intersection = max(0, min(ax2, bx2) - max(ax1, bx1)) * max(
        0, min(ay2, by2) - max(ay1, by1)
    )
    union = (ax2 - ax1) * (ay2 - ay1) + (bx2 - bx1) * (by2 - by1) - intersection
    return intersection / union if union > 0 else 0.0


class ModelMaterialSource:
    """把模型框转换为现有物料对准接口，连续确认同一位置的目标。"""

    backend = "model"

    def __init__(self, config: MaterialModelConfig, *, inference_backend=None, clock=None):
        self.config = config
        self.inference = (
            inference_backend if inference_backend is not None
            else UltralyticsYoloBackend(config)
        )
        self.clock = time.monotonic if clock is None else clock
        self._tracks = {}

    def reset_history(self):
        self._tracks.clear()

    def _blocked(self, status, message, raw=()):
        self.reset_history()
        return {
            "backend": "model", "status": status, "message": message,
            "blocked": True, "safe_to_pick": False, "detections": (),
            "raw_model_detections": tuple(raw),
        }, []

    def detect(self, frame, collect_masks=False):
        if self.config.weights is None:
            return self._blocked("MODEL_NOT_CONFIGURED", "尚未配置训练模型权重")
        try:
            height, width = frame.shape[:2]
            if height <= 0 or width <= 0:
                raise ValueError("空图像")
            raw = []
            for item in self.inference.predict(frame):
                if (
                    not isinstance(item, ModelDetection)
                    or not isinstance(item.class_id, int)
                    or isinstance(item.class_id, bool) or item.class_id < 0
                    or not isfinite(item.confidence) or not 0 <= item.confidence <= 1
                    or len(item.xyxy) != 4 or not all(isfinite(v) for v in item.xyxy)
                ):
                    raise ValueError("推理结果的类别、置信度或框格式无效")
                x1, y1, x2, y2 = item.xyxy
                if x2 <= x1 or y2 <= y1:
                    raise ValueError("推理结果的检测框无效")
                x1, y1 = max(0.0, min(width, x1)), max(0.0, min(height, y1))
                x2, y2 = max(0.0, min(width, x2)), max(0.0, min(height, y2))
                if (
                    item.confidence >= self.config.confidence_threshold
                    and (x2 - x1) * (y2 - y1) >= self.config.minimum_box_area_pixels
                ):
                    raw.append(ModelDetection(
                        item.class_id, item.class_name, item.confidence, (x1, y1, x2, y2)
                    ))
        except Exception as error:
            return self._blocked("INFERENCE_ERROR", str(error))
        if not self.config.class_to_material:
            return self._blocked(
                "MAPPING_NOT_CONFIGURED", "尚未配置模型类别到物料编号的映射", raw
            )
        unknown = sorted({item.class_id for item in raw} - set(self.config.class_to_material))
        if unknown:
            return self._blocked(
                "UNMAPPED_CLASS", f"出现未映射模型类别：{unknown}", raw
            )
        class_ids = [item.class_id for item in raw]
        if len(set(class_ids)) != len(class_ids):
            return self._blocked(
                "AMBIGUOUS", "同一类别出现多个目标，禁止自动抓取", raw
            )
        now = self.clock()
        next_tracks = {}
        detections = []
        for item in raw:
            previous = self._tracks.get(item.class_id)
            count = 1
            if previous is not None:
                old, old_count, old_time = previous
                if (
                    0 <= now - old_time <= self.config.maximum_gap_seconds
                    and _iou(old, item) >= self.config.matching_iou
                ):
                    count = old_count + 1
            next_tracks[item.class_id] = (item, count, now)
            material = self.config.class_to_material[item.class_id]
            x1, y1, x2, y2 = item.xyxy
            detections.append({
                "code": material.material_code,
                "name": material.name, "cn_name": material.name,
                "class_id": item.class_id, "class_name": item.class_name,
                "confidence": item.confidence,
                "center": [round((x1 + x2) / 2), round((y1 + y2) / 2)],
                "box": [int(x1), int(y1), int(x2 - x1), int(y2 - y1)],
                "area": (x2 - x1) * (y2 - y1),
                "confirmed": count >= self.config.required_frames,
            })
        self._tracks = next_tracks
        ready = bool(detections) and all(item["confirmed"] for item in detections)
        return {
            "backend": "model", "status": "READY" if ready else "SEARCHING",
            "safe_to_pick": ready, "detections": detections,
            "configured_material_codes": tuple(
                item.material_code for item in self.config.class_to_material.values()
            ),
            "raw_model_detections": tuple(raw),
        }, []
