"""抓取摄像头的模型/颜色识别与二维对准结果；不执行机械动作。"""

from dataclasses import dataclass
from math import hypot
from typing import Any, Mapping, Optional, Tuple


@dataclass(frozen=True)
class MaterialObservation:
    """一件物料相对夹爪抓取中心的位置与颜色信息。"""

    material_code: int
    color_name: str
    color_cn_name: str
    center: Tuple[int, int]
    offset_pixels: Tuple[float, float]
    normalized_offset: Tuple[float, float]
    box: Tuple[int, int, int, int]
    area: float
    confirmed: bool
    class_id: Optional[int] = None
    class_name: str = ""
    confidence: float = 1.0
    backend: str = "color"

    @property
    def material_name(self) -> str:
        """模型和颜色后端通用名称；旧 color_* 字段保留兼容。"""
        return self.color_cn_name or self.color_name


@dataclass(frozen=True)
class GripperDetection:
    """抓取相机单次识别结果。"""

    status: str
    target_material_code: Optional[int]
    observation: Optional[MaterialObservation]
    aligned: bool
    safe_to_pick: bool
    detected_color_codes: Tuple[int, ...]
    masks: Any = None
    backend: str = "color"
    message: str = ""
    raw_model_detections: Tuple[Any, ...] = ()

    @property
    def detected_material_codes(self) -> Tuple[int, ...]:
        return self.detected_color_codes


class GripperMaterialDetector:
    """在模型或颜色检测结果中选择目标，复用夹爪中心和对准容差。"""

    def __init__(self, color_detector, config: Mapping[str, Any]):
        # 保留旧构造参数/属性名称，已有调用方无需迁移。
        self.color_detector = color_detector
        self.source_detector = color_detector
        self._last_target = object()
        grip_center = config.get("grip_center")
        tolerance = config.get("alignment_tolerance_pixels", (18, 18))
        if (
            not isinstance(grip_center, (list, tuple))
            or len(grip_center) != 2
        ):
            raise ValueError("grip_center 必须是两个数字")
        if not isinstance(tolerance, (list, tuple)) or len(tolerance) != 2:
            raise ValueError("alignment_tolerance_pixels 必须是两个数字")
        if any(
            not isinstance(value, (int, float)) or isinstance(value, bool)
            for value in (*grip_center, *tolerance)
        ):
            raise ValueError("抓取中心和对准容差必须是数字")
        if any(value < 0 for value in tolerance):
            raise ValueError("对准容差不能小于0")

        self.grip_center = (float(grip_center[0]), float(grip_center[1]))
        self.tolerance = (float(tolerance[0]), float(tolerance[1]))
        self.require_global_ready = bool(config.get("require_global_ready", False))

    def detect(
        self,
        frame,
        target_material_code: Optional[int] = None,
        collect_masks: bool = False,
    ) -> GripperDetection:
        if frame is None or not hasattr(frame, "shape") or len(frame.shape) < 2:
            raise ValueError("frame 必须是有效图像")
        if target_material_code is not None and (
            not isinstance(target_material_code, int)
            or isinstance(target_material_code, bool)
            or target_material_code <= 0
        ):
            raise ValueError("target_material_code 必须是正整数或None")

        backend = getattr(self.source_detector, "backend", "color")
        if backend == "model" and target_material_code != self._last_target:
            self.source_detector.reset_history()
        self._last_target = target_material_code
        state, masks = self.source_detector.detect(
            frame,
            collect_masks=collect_masks,
        )
        backend = state.get("backend", backend)
        detections = tuple(state.get("detections", ()))
        detected_codes = tuple(
            sorted({int(item["code"]) for item in detections})
        )
        common = {
            "backend": backend,
            "message": str(state.get("message", "")),
            "raw_model_detections": tuple(state.get("raw_model_detections", ())),
        }
        if state.get("blocked", False):
            return GripperDetection(
                status=str(state.get("status", "HOLD")),
                target_material_code=target_material_code,
                observation=None, aligned=False, safe_to_pick=False,
                detected_color_codes=detected_codes,
                masks=masks if collect_masks else None,
                **common,
            )
        if (
            backend == "model" and target_material_code is not None
            and target_material_code not in state.get("configured_material_codes", ())
        ):
            common["message"] = "请求的物料编号没有模型类别映射"
            return GripperDetection(
                status="TARGET_NOT_MAPPED", target_material_code=target_material_code,
                observation=None, aligned=False, safe_to_pick=False,
                detected_color_codes=detected_codes,
                masks=masks if collect_masks else None,
                **common,
            )
        candidates = [
            item
            for item in detections
            if target_material_code is None
            or int(item["code"]) == target_material_code
        ]
        if not candidates:
            return GripperDetection(
                status="TARGET_NOT_FOUND" if target_material_code else "SEARCHING",
                target_material_code=target_material_code,
                observation=None,
                aligned=False,
                safe_to_pick=False,
                detected_color_codes=detected_codes,
                masks=masks if collect_masks else None,
                **common,
            )

        center_x, center_y = self.grip_center
        selected = min(
            candidates,
            key=lambda item: (
                not bool(item.get("confirmed", False)),
                hypot(
                    float(item["center"][0]) - center_x,
                    float(item["center"][1]) - center_y,
                ),
                -float(item.get("area", 0.0)),
            )
        )
        object_x = int(selected["center"][0])
        object_y = int(selected["center"][1])
        offset_x = object_x - center_x
        offset_y = object_y - center_y
        frame_height, frame_width = frame.shape[:2]
        normalized_x = offset_x / max(frame_width / 2.0, 1.0)
        normalized_y = offset_y / max(frame_height / 2.0, 1.0)
        confirmed = bool(selected.get("confirmed", False))
        aligned = (
            abs(offset_x) <= self.tolerance[0]
            and abs(offset_y) <= self.tolerance[1]
        )
        observation = MaterialObservation(
            material_code=int(selected["code"]),
            color_name=str(selected.get("name", "")),
            color_cn_name=str(selected.get("cn_name", "")),
            center=(object_x, object_y),
            offset_pixels=(float(offset_x), float(offset_y)),
            normalized_offset=(float(normalized_x), float(normalized_y)),
            box=tuple(int(value) for value in selected["box"]),
            area=float(selected.get("area", 0.0)),
            confirmed=confirmed,
            class_id=selected.get("class_id"),
            class_name=str(selected.get("class_name", "")),
            confidence=float(selected.get("confidence", 1.0)),
            backend=backend,
        )
        global_status = str(state.get("status", "HOLD"))
        global_ready = bool(state.get("safe_to_pick", False))
        color_state_allows_pick = (
            global_ready
            if self.require_global_ready
            else global_status != "AMBIGUOUS"
        )
        safe_to_pick = confirmed and aligned and color_state_allows_pick
        if not confirmed:
            status = "CONFIRMING_MATERIAL" if backend == "model" else "CONFIRMING_COLOR"
        elif not aligned:
            status = "ALIGNING"
        elif safe_to_pick:
            status = "READY"
        else:
            status = global_status

        return GripperDetection(
            status=status,
            target_material_code=target_material_code,
            observation=observation,
            aligned=aligned,
            safe_to_pick=safe_to_pick,
            detected_color_codes=detected_codes,
            masks=masks if collect_masks else None,
            **common,
        )
