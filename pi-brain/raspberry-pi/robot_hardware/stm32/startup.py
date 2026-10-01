"""队友 ASCII v4 固件的启动自检结果；此处只解析查询应答。"""

from dataclasses import dataclass
import math
import re
from typing import Iterable

from .messages import HOST_PROTOCOL_VERSION


def _fields(line: str) -> dict[str, str]:
    return dict(re.findall(r"\b([A-Z_]+)=([^\s]+)", line))


@dataclass(frozen=True)
class FirmwareStartupInfo:
    """OPS 启动坐标使用文本协议的 mm/mm/degree，不是二进制的 mrad。"""

    ops_x_mm: float
    ops_y_mm: float
    ops_yaw_deg: float
    ops_frames: int
    can_state: int
    can_esr: int
    pid_x: tuple[float, float, float]
    pid_y: tuple[float, float, float]
    pid_yaw: tuple[float, float, float]

    @classmethod
    def decode(
        cls, status_line: str, ops_line: str, can_lines: Iterable[str], pid_line: str,
    ) -> "FirmwareStartupInfo":
        status, ops, pid = map(_fields, (status_line, ops_line, pid_line))
        can: dict[str, str] = {}
        for line in can_lines:
            if line.startswith("# CAN "):
                can.update(_fields(line))
        try:
            if (status["MODE"] != "WORK" or status["HOST"] != "RPI"
                or int(status["HOST_PROTO"]) != HOST_PROTOCOL_VERSION
                or int(status["STATE"]) != 0 or int(status["PLOT"]) != 0):
                raise ValueError(f"STM32 未处于 RPI/WORK 空闲状态：{status_line}")
            frames = int(ops["FRAMES"])
            pose = tuple(float(ops[key]) for key in ("X", "Y", "YAW"))
            if ops["LINK"] != "OK" or frames <= 0 or not all(map(math.isfinite, pose)):
                raise ValueError(f"OPS9 启动自检未通过：{ops_line}")
            state, ready, esr = (int(can[key], 0) for key in ("STATE", "READY", "ESR"))
            # HAL_CAN_STATE_LISTENING=2；ESR 低三位是 EWGF/EPVF/BOFF。
            # ERROR 是历史累计诊断，不能代替当前 READY/ESR 判据。
            if state != 2 or ready != 1 or esr & 7:
                raise ValueError(f"CAN 启动自检未通过：{can}")
            coefficients = tuple(
                tuple(float(value) for value in pid[axis].split(","))
                for axis in ("X", "Y", "YAW")
            )
            if any(len(values) != 3 or not all(map(math.isfinite, values))
                   for values in coefficients):
                raise ValueError(f"PID 参数应答无效：{pid_line}")
        except (KeyError, OverflowError) as error:
            raise ValueError(f"启动自检应答缺少有效字段：{error}") from error
        return cls(*pose, frames, state, esr, *coefficients)
