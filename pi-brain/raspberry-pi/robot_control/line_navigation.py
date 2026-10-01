"""把巡线视觉偏差转换成上层导航决策。"""

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class LineDecision:
    command: str
    error: float
    line_found: bool


class LineFollower:
    def __init__(self, turn_threshold: float = 0.15, smoothing: float = 0.35):
        if not 0 < turn_threshold < 1:
            raise ValueError("turn_threshold 必须在0和1之间")
        if not 0 < smoothing <= 1:
            raise ValueError("smoothing 必须在0和1之间")
        self.turn_threshold = float(turn_threshold)
        self.smoothing = float(smoothing)
        self.smoothed_error = 0.0

    def update(self, measured_error: Optional[float]) -> LineDecision:
        if measured_error is None:
            return LineDecision("STOP", self.smoothed_error, False)
        self.smoothed_error = (
            self.smoothing * float(measured_error)
            + (1.0 - self.smoothing) * self.smoothed_error
        )
        if self.smoothed_error < -self.turn_threshold:
            command = "LEFT"
        elif self.smoothed_error > self.turn_threshold:
            command = "RIGHT"
        else:
            command = "STRAIGHT"
        return LineDecision(command, self.smoothed_error, True)
