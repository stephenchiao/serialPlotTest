"""所有硬件和算法组件必须实现的接口。

接口方法应当快速返回，长动作通过重复调用返回 RUNNING 轮询完成。
同一状态下的方法可能被多次调用，实现时必须保证幂等。
"""

from dataclasses import dataclass
from typing import Any, Mapping, Optional, Protocol, Tuple

from robot_mission.task_code import MaterialTask

from .models import ActionResult, RobotState, SafetyReport, TargetArea


class Clock(Protocol):
    def monotonic(self) -> float: ...

    def sleep(self, seconds: float) -> None: ...


class StartButton(Protocol):
    """唯一实体 Start 按钮；建议硬件层完成消抖。"""

    def is_pressed(self) -> bool: ...


class TaskCodeReader(Protocol):
    """二维码/条码识别接口；未识别到时返回 None。"""

    def read_task_code(self) -> Optional[str]: ...


class Display(Protocol):
    """顶部静态显示屏接口；任务码应在比赛期间保持可见。"""

    def show_state(self, state: RobotState, detail: str = "") -> None: ...

    def show_task_code(self, task_code: str) -> None: ...

    def show_final(self, statistics: Mapping[str, Any]) -> None: ...


class MotionController(Protocol):
    """底盘驱动和物理运动反馈。

    ``is_active`` 必须来自编码器或 OPS9 位姿变化，不能只表示已发送非零命令，
    否则会掩盖堵转和比赛的连续静止超时。
    """

    def stop(self) -> None: ...

    def is_active(self) -> bool: ...


class Navigator(Protocol):
    """巡线、定位、路径规划和避障的统一接口。"""

    def navigate_to(self, target: TargetArea) -> ActionResult: ...

    def cancel(self) -> None: ...


class MaterialPerception(Protocol):
    """颜色、形状、位置和转盘目标识别接口。"""

    def locate_material(self, material_code: int) -> ActionResult: ...


class Manipulator(Protocol):
    """抓取、车载暂放、加工放置、暂存和堆叠接口。"""

    def pick_to_payload(
        self,
        material: MaterialTask,
        batch_number: int,
        payload_slot: int,
    ) -> ActionResult: ...

    def place_for_processing(
        self,
        material: MaterialTask,
        batch_number: int,
    ) -> ActionResult: ...

    def place_in_temporary_storage(self, material: MaterialTask) -> ActionResult: ...

    def stack_second_batch(self, material: MaterialTask) -> ActionResult: ...

    def stop(self) -> None: ...

    def is_active(self) -> bool: ...


class SafetyMonitor(Protocol):
    """急停、边界、堵转、姿态和电源状态汇总接口。"""

    def check(self) -> SafetyReport: ...


class StatisticsRecorder(Protocol):
    """记录抓取、放置和最终正确数的接口。"""

    def start_run(self) -> None: ...

    def set_task_code(self, task_code: str) -> None: ...

    def record_pickup(self, batch_number: int, material_code: int) -> None: ...

    def record_processing(
        self,
        batch_number: int,
        material_code: int,
        process_position: int,
    ) -> None: ...

    def record_storage(self, material_code: int) -> None: ...

    def record_stack(self, material_code: int) -> None: ...

    def finish_run(self, success: bool, reason: str = "") -> None: ...

    def snapshot(self) -> Mapping[str, Any]: ...


class Telemetry(Protocol):
    """只用于日志/观测，不允许绕过实体按钮远程控制比赛流程。"""

    def publish(self, topic: str, payload: Mapping[str, Any]) -> None: ...


class LightingController(Protocol):
    def set_enabled(self, enabled: bool) -> None: ...


class RecoveryController(Protocol):
    """丢线、丢目标和动作超时后的有界恢复接口。"""

    def recover(self, failed_state: RobotState, reason: str) -> ActionResult: ...


@dataclass
class ComponentBundle:
    """状态机所需组件的依赖集合。"""

    start_button: StartButton
    task_code_reader: TaskCodeReader
    display: Display
    motion: MotionController
    navigator: Navigator
    material_perception: MaterialPerception
    manipulator: Manipulator
    safety: SafetyMonitor
    statistics: StatisticsRecorder
    telemetry: Telemetry
    lighting: LightingController
    recovery: RecoveryController

    def unique_components(self) -> Tuple[Any, ...]:
        """去重后返回组件，供 initialize/self_check/shutdown 生命周期调用。"""
        values = tuple(vars(self).values())
        result = []
        seen = set()
        for component in values:
            identity = id(component)
            if identity not in seen:
                result.append(component)
                seen.add(identity)
        return tuple(result)
