"""前视图像的基础道路安全检测。

该模块不再寻找黑色巡线，而是判断车体前方中央走廊是否仍由灰色可行驶区域
覆盖，并检测黄/白禁入颜色。完整 footprint 合法性仍由地图安全层负责。
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import cached_property
import json
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class RoadObservation:
    observed_at: float
    drivable_fraction: float
    forbidden_fraction: float
    central_drivable_fraction: float
    boundary_safe: bool
    confidence: float


@dataclass(frozen=True)
class RoadDetectorConfig:
    input_color_order: str
    roi_top_fraction: float
    central_corridor_fraction: float
    minimum_drivable_fraction: float
    minimum_central_drivable_fraction: float
    maximum_forbidden_fraction: float
    gray_hsv_lower: tuple[int, int, int]
    gray_hsv_upper: tuple[int, int, int]
    yellow_hsv_lower: tuple[int, int, int]
    yellow_hsv_upper: tuple[int, int, int]
    white_hsv_lower: tuple[int, int, int]
    white_hsv_upper: tuple[int, int, int]
    calibration_required: bool = True

    @classmethod
    def load(cls, path: str | Path) -> "RoadDetectorConfig":
        data = json.loads(Path(path).read_text(encoding="utf-8"))

        def hsv_range(name: str) -> tuple[tuple[int, int, int], tuple[int, int, int]]:
            section = data[name]
            lower = tuple(int(value) for value in section["lower"])
            upper = tuple(int(value) for value in section["upper"])
            if len(lower) != 3 or len(upper) != 3:
                raise ValueError(f"{name} 必须包含三通道 HSV 上下界")
            return lower, upper

        gray_lower, gray_upper = hsv_range("gray_hsv")
        yellow_lower, yellow_upper = hsv_range("yellow_hsv")
        white_lower, white_upper = hsv_range("white_hsv")
        return cls(
            input_color_order=str(data.get("input_color_order", "RGB")).upper(),
            roi_top_fraction=float(data["roi_top_fraction"]),
            central_corridor_fraction=float(data["central_corridor_fraction"]),
            minimum_drivable_fraction=float(data["minimum_drivable_fraction"]),
            minimum_central_drivable_fraction=float(
                data["minimum_central_drivable_fraction"]
            ),
            maximum_forbidden_fraction=float(data["maximum_forbidden_fraction"]),
            gray_hsv_lower=gray_lower,
            gray_hsv_upper=gray_upper,
            yellow_hsv_lower=yellow_lower,
            yellow_hsv_upper=yellow_upper,
            white_hsv_lower=white_lower,
            white_hsv_upper=white_upper,
            calibration_required=bool(data.get("calibration_required", True)),
        )


class RoadAreaDetector:
    def __init__(self, config: RoadDetectorConfig) -> None:
        if config.calibration_required:
            raise ValueError(
                "道路颜色阈值仍未现场标定；完成灰/黄/白色样标定并将 "
                "calibration_required 设为 false 后才能启用导航"
            )
        if not 0.0 < config.central_corridor_fraction <= 1.0:
            raise ValueError("central_corridor_fraction 必须在 0~1 范围内")
        self.config = config
        self._hsv_ranges = (
            (config.gray_hsv_lower, config.gray_hsv_upper),
            (config.yellow_hsv_lower, config.yellow_hsv_upper),
            (config.white_hsv_lower, config.white_hsv_upper),
        )

    @cached_property
    def _closing_kernel(self):
        import numpy as np

        return np.ones((5, 5), dtype=np.uint8)

    def detect(self, frame: Any, *, observed_at: float) -> RoadObservation:
        try:
            import cv2  # type: ignore[import-not-found]
            import numpy as np  # type: ignore[import-not-found]
        except ImportError as error:
            raise RuntimeError("道路检测需要安装 numpy 和 opencv-python") from error
        image = np.asarray(frame)
        if image.ndim != 3:
            raise ValueError("道路检测需要 RGB 或 BGR 三通道图像")
        conversion = (
            cv2.COLOR_RGB2HSV
            if self.config.input_color_order == "RGB"
            else cv2.COLOR_BGR2HSV
        )
        hsv = cv2.cvtColor(image, conversion)
        height, width = hsv.shape[:2]
        roi_top = int(height * self.config.roi_top_fraction)
        roi = hsv[roi_top:, :]
        if roi.size == 0:
            raise ValueError("道路检测 ROI 为空")

        gray, yellow, white = (
            cv2.inRange(roi, lower, upper) for lower, upper in self._hsv_ranges
        )
        gray = cv2.morphologyEx(gray, cv2.MORPH_CLOSE, self._closing_kernel)
        forbidden = cv2.bitwise_or(yellow, white)

        pixel_count = float(roi.shape[0] * roi.shape[1])
        drivable_fraction = float(cv2.countNonZero(gray)) / pixel_count
        corridor_width = max(1, int(width * self.config.central_corridor_fraction))
        corridor_left = (width - corridor_width) // 2
        corridor = gray[:, corridor_left : corridor_left + corridor_width]
        forbidden_corridor = forbidden[:, corridor_left : corridor_left + corridor_width]
        central_fraction = float(cv2.countNonZero(corridor)) / float(corridor.size)
        forbidden_fraction = float(cv2.countNonZero(forbidden_corridor)) / float(
            forbidden_corridor.size
        )
        safe = (
            drivable_fraction >= self.config.minimum_drivable_fraction
            and central_fraction >= self.config.minimum_central_drivable_fraction
            and forbidden_fraction <= self.config.maximum_forbidden_fraction
        )
        confidence = max(
            0.0,
            min(
                1.0,
                0.6 * central_fraction
                + 0.4 * drivable_fraction
                - forbidden_fraction,
            ),
        )
        return RoadObservation(
            observed_at=observed_at,
            drivable_fraction=drivable_fraction,
            forbidden_fraction=forbidden_fraction,
            central_drivable_fraction=central_fraction,
            boundary_safe=safe,
            confidence=confidence,
        )
