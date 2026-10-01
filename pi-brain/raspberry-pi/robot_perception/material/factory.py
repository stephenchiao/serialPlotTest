"""单路与双路调试共用同一物料后端，避免某个入口仍偷偷用 HSV。"""

from dataclasses import dataclass
from typing import Callable, Optional

from .config import load_material_config
from .detector import GripperMaterialDetector
from .model import ModelMaterialSource


@dataclass(frozen=True)
class MaterialPipeline:
    detector: GripperMaterialDetector
    calibrator: Optional[Callable] = None


def build_material_pipeline(
    gripper_config, *, material_config_path=None, color_config_path=None,
    backend=None, inference_backend=None,
):
    config = load_material_config(material_config_path, backend=backend)
    if config.backend == "model":
        # 不额外做 HSV 路径的软件白平衡；训练和推理预处理应保持一致。
        source = ModelMaterialSource(config, inference_backend=inference_backend)
        return MaterialPipeline(GripperMaterialDetector(source, gripper_config))

    from robot_perception.color import load_config as load_color_config
    from robot_perception.color.detector import (
        CompetitionColorDetector, apply_white_balance,
        build_white_balance_luts, calibrate_white_balance,
    )

    color_config = load_color_config(color_config_path)
    source = CompetitionColorDetector(color_config)

    def calibrate(camera):
        gains = calibrate_white_balance(camera, color_config["white_balance"])
        lookup_tables = build_white_balance_luts(gains)
        return lambda frame: apply_white_balance(frame, lookup_tables)

    return MaterialPipeline(GripperMaterialDetector(source, gripper_config), calibrate)
