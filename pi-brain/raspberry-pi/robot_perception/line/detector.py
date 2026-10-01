"""只负责从单帧图像计算赛道线位置，不控制摄像头或底盘。"""

from dataclasses import dataclass
from typing import Any, Mapping, Optional, Tuple

import cv2
import numpy as np


@dataclass(frozen=True)
class LineDetection:
    found: bool
    error: Optional[float]
    center: Optional[Tuple[int, int]]
    contour_area: float
    roi_top: int
    mask: Any
    contour: Any = None


class LineDetector:
    """在画面底部ROI中寻找面积最大的有效赛道线。"""

    def __init__(self, config: Mapping[str, Any]):
        self.roi_top_ratio = float(config.get("roi_top_ratio", 0.55))
        self.line_is_dark = bool(config.get("line_is_dark", True))
        self.min_line_area = float(config.get("min_line_area", 300))
        self.max_line_area_ratio = float(config.get("max_line_area_ratio", 0.8))
        kernel_size = int(config.get("morph_kernel_size", 5))
        if not 0 < self.roi_top_ratio < 1:
            raise ValueError("roi_top_ratio 必须在0和1之间")
        if kernel_size <= 0 or kernel_size % 2 == 0:
            raise ValueError("morph_kernel_size 必须是正奇数")
        self.kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)

    def detect(self, frame) -> LineDetection:
        if frame is None or not hasattr(frame, "shape") or len(frame.shape) < 2:
            raise ValueError("frame 必须是有效图像")

        frame_height, frame_width = frame.shape[:2]
        roi_top = int(frame_height * self.roi_top_ratio)
        roi = frame[roi_top:frame_height, :]
        roi_area = frame_width * (frame_height - roi_top)

        gray = cv2.cvtColor(roi, cv2.COLOR_RGB2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        threshold_mode = (
            cv2.THRESH_BINARY_INV if self.line_is_dark else cv2.THRESH_BINARY
        )
        _, mask = cv2.threshold(
            blurred,
            0,
            255,
            threshold_mode | cv2.THRESH_OTSU,
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        valid_contours = [
            contour
            for contour in contours
            if self.min_line_area
            <= cv2.contourArea(contour)
            <= roi_area * self.max_line_area_ratio
        ]
        if not valid_contours:
            return LineDetection(False, None, None, 0.0, roi_top, mask)

        contour = max(valid_contours, key=cv2.contourArea)
        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            return LineDetection(False, None, None, 0.0, roi_top, mask)

        center_x = int(moments["m10"] / moments["m00"])
        center_y = int(moments["m01"] / moments["m00"]) + roi_top
        error = (center_x - frame_width / 2) / (frame_width / 2)
        return LineDetection(
            True,
            float(error),
            (center_x, center_y),
            float(cv2.contourArea(contour)),
            roi_top,
            mask,
            contour,
        )
