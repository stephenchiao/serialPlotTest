"""cam0 模型与物料编号的独立配置，不依赖推理库。"""

from dataclasses import dataclass
import json
from math import isfinite
from pathlib import Path
from typing import Mapping, Optional


DEFAULT_MATERIAL_CONFIG_PATH = (
    Path(__file__).resolve().parents[2] / "config" / "material.json"
)


class MaterialConfigError(ValueError):
    """模型配置格式错误；缺少尚未训练的权重不属于格式错误。"""


@dataclass(frozen=True)
class MaterialClass:
    material_code: int
    name: str


@dataclass(frozen=True)
class MaterialModelConfig:
    backend: str
    weights: Optional[Path]
    input_color_order: str
    confidence_threshold: float
    iou_threshold: float
    image_size: int
    device: str
    max_detections: int
    class_to_material: Mapping[int, MaterialClass]
    required_frames: int
    matching_iou: float
    maximum_gap_seconds: float
    minimum_box_area_pixels: float


def _positive_integer(value, name):
    if not isinstance(value, int) or isinstance(value, bool) or value < 1:
        raise MaterialConfigError(f"{name} 必须是正整数")
    return value


def _number(value, name, *, minimum=0.0, maximum=None, allow_minimum=False):
    if (
        not isinstance(value, (int, float))
        or isinstance(value, bool)
        or not isfinite(value)
        or (value < minimum if allow_minimum else value <= minimum)
        or (maximum is not None and value > maximum)
    ):
        raise MaterialConfigError(f"{name} 超出允许范围")
    return float(value)


def load_material_config(path=None, *, backend=None) -> MaterialModelConfig:
    config_path = Path(path) if path is not None else DEFAULT_MATERIAL_CONFIG_PATH
    try:
        data = json.loads(config_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise MaterialConfigError(f"无法读取物料配置：{config_path}") from error
    if not isinstance(data, dict):
        raise MaterialConfigError("物料配置根节点必须是 JSON 对象")
    selected_backend = data.get("backend", "model") if backend is None else backend
    if selected_backend not in ("model", "color"):
        raise MaterialConfigError("backend 只能是 model 或 color")
    model = data.get("model", {})
    confirmation = data.get("confirmation", {})
    mapping = data.get("class_to_material", {})
    if not all(isinstance(item, dict) for item in (model, confirmation, mapping)):
        raise MaterialConfigError("model、confirmation、class_to_material 必须是对象")
    if model.get("engine", "ultralytics_yolo") != "ultralytics_yolo":
        raise MaterialConfigError("当前提供的模型适配器仅支持 ultralytics_yolo")
    weights = model.get("weights")
    if weights is not None:
        if not isinstance(weights, str) or not weights.strip():
            raise MaterialConfigError("weights 必须是非空本地路径或 null")
        weights = Path(weights)
        if not weights.is_absolute():
            weights = config_path.resolve().parent / weights
    order = model.get("input_color_order", "BGR")
    if order not in ("BGR", "RGB"):
        raise MaterialConfigError("input_color_order 只能是 BGR 或 RGB")
    device = model.get("device", "cpu")
    if not isinstance(device, str) or not device.strip():
        raise MaterialConfigError("device 必须是非空字符串")
    classes = {}
    used_codes = set()
    for key, item in mapping.items():
        if not isinstance(key, str) or not key.isdecimal() or str(int(key)) != key:
            raise MaterialConfigError("类别键必须是规范的非负整数文本，例如 0、1")
        if not isinstance(item, dict):
            raise MaterialConfigError("类别映射项必须包含 material_code，可选 name")
        code = _positive_integer(item.get("material_code"), "material_code")
        if code in used_codes:
            raise MaterialConfigError("多个模型类别不能映射到同一物料编号")
        name = item.get("name", f"class_{key}")
        if not isinstance(name, str) or not name.strip():
            raise MaterialConfigError("类别 name 必须是非空字符串")
        classes[int(key)] = MaterialClass(code, name)
        used_codes.add(code)
    return MaterialModelConfig(
        backend=selected_backend,
        weights=weights,
        input_color_order=order,
        confidence_threshold=_number(
            model.get("confidence_threshold", 0.6), "confidence_threshold", maximum=1
        ),
        iou_threshold=_number(
            model.get("iou_threshold", 0.45), "iou_threshold",
            maximum=1, allow_minimum=True,
        ),
        image_size=_positive_integer(model.get("image_size", 640), "image_size"),
        device=device,
        max_detections=_positive_integer(model.get("max_detections", 20), "max_detections"),
        class_to_material=classes,
        required_frames=_positive_integer(
            confirmation.get("required_frames", 3), "required_frames"
        ),
        matching_iou=_number(
            confirmation.get("matching_iou", 0.3), "matching_iou", maximum=1
        ),
        maximum_gap_seconds=_number(
            confirmation.get("maximum_gap_seconds", 1.0), "maximum_gap_seconds"
        ),
        minimum_box_area_pixels=_number(
            data.get("minimum_box_area_pixels", 100), "minimum_box_area_pixels"
        ),
    )
