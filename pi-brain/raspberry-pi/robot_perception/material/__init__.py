"""物料模型/颜色识别、位置与二维夹爪对准。"""

from .detector import (
    GripperDetection,
    GripperMaterialDetector,
    MaterialObservation,
)
from .config import (
    DEFAULT_MATERIAL_CONFIG_PATH, MaterialConfigError, MaterialModelConfig,
    load_material_config,
)
from .factory import MaterialPipeline, build_material_pipeline
from .model import InferenceBackend, ModelDetection, ModelMaterialSource, UltralyticsYoloBackend

__all__ = [
    "GripperDetection",
    "GripperMaterialDetector",
    "MaterialObservation",
    "DEFAULT_MATERIAL_CONFIG_PATH",
    "MaterialConfigError",
    "MaterialModelConfig",
    "load_material_config",
    "MaterialPipeline",
    "build_material_pipeline",
    "InferenceBackend",
    "ModelDetection",
    "ModelMaterialSource",
    "UltralyticsYoloBackend",
]
