"""无硬件模拟组件，用于开发机验证状态机流程。"""

import logging
import time
from typing import Any, Mapping

from robot_mission.task_code import MaterialTask

from robot_runtime.interfaces import ComponentBundle
from robot_runtime.models import ActionResult, RobotState, SafetyReport, TargetArea


LOGGER = logging.getLogger(__name__)


class HealthyComponent:
    def initialize(self) -> None:
        pass

    def self_check(self) -> ActionResult:
        return ActionResult.done("模拟组件自检通过", activity=False)

    def shutdown(self) -> None:
        pass


class SimulatedStartButton(HealthyComponent):
    def __init__(self, auto_start: bool, delay_seconds: float):
        self.auto_start = auto_start
        self.delay_seconds = delay_seconds
        self._created_at = time.monotonic()
        self._read_count = 0
        self.manually_pressed = False

    def is_pressed(self) -> bool:
        self._read_count += 1
        if self.manually_pressed:
            return True
        if self._read_count == 1:
            return False
        return (
            self.auto_start
            and time.monotonic() - self._created_at >= self.delay_seconds
        )


class StaticTaskCodeReader(HealthyComponent):
    def __init__(self, task_code: str):
        self.task_code = task_code

    def read_task_code(self):
        return self.task_code


class ConsoleDisplay(HealthyComponent):
    """日志显示器；真实显示器必须保证 show_state 不擦除任务码。"""

    def __init__(self):
        self.task_code = ""
        self.last_state = RobotState.BOOTING
        self.final_statistics = None

    def show_state(self, state: RobotState, detail: str = "") -> None:
        self.last_state = state
        LOGGER.info("显示状态：%s %s", state.name, detail)

    def show_task_code(self, task_code: str) -> None:
        self.task_code = task_code
        LOGGER.info("显示完整任务码：%s", task_code)

    def show_final(self, statistics: Mapping[str, Any]) -> None:
        self.final_statistics = dict(statistics)
        LOGGER.info("显示最终统计：%s", self.final_statistics)


class SimulatedMotionController(HealthyComponent):
    def __init__(self):
        self.active = False
        self.stop_count = 0

    def stop(self) -> None:
        self.active = False
        self.stop_count += 1

    def is_active(self) -> bool:
        return self.active


class SimulatedNavigator(HealthyComponent):
    def __init__(self):
        self.destinations = []
        self.cancel_count = 0

    def navigate_to(self, target: TargetArea) -> ActionResult:
        self.destinations.append(target.value)
        return ActionResult.done(f"模拟到达{target.value}")

    def cancel(self) -> None:
        self.cancel_count += 1


class SimulatedMaterialPerception(HealthyComponent):
    def __init__(self):
        self.located_codes = []

    def locate_material(self, material_code: int) -> ActionResult:
        self.located_codes.append(material_code)
        return ActionResult.done(f"模拟定位{material_code}号物料")


class SimulatedManipulator(HealthyComponent):
    def __init__(self):
        self.events = []
        self.active = False
        self.stop_count = 0

    def pick_to_payload(
        self,
        material: MaterialTask,
        batch_number: int,
        payload_slot: int,
    ) -> ActionResult:
        self.events.append(
            ("pickup", batch_number, material.material_code, payload_slot)
        )
        return ActionResult.done("模拟抓取完成")

    def place_for_processing(
        self,
        material: MaterialTask,
        batch_number: int,
    ) -> ActionResult:
        self.events.append(
            (
                "processing",
                batch_number,
                material.material_code,
                material.process_position,
            )
        )
        return ActionResult.done("模拟加工放置完成")

    def place_in_temporary_storage(self, material: MaterialTask) -> ActionResult:
        self.events.append(("storage", material.material_code))
        return ActionResult.done("模拟暂存完成")

    def stack_second_batch(self, material: MaterialTask) -> ActionResult:
        self.events.append(("stack", material.material_code))
        return ActionResult.done("模拟堆叠完成")

    def stop(self) -> None:
        self.active = False
        self.stop_count += 1

    def is_active(self) -> bool:
        return self.active


class SimulatedSafetyMonitor(HealthyComponent):
    def __init__(self):
        self.report = SafetyReport()

    def check(self) -> SafetyReport:
        return self.report


class InMemoryStatistics(HealthyComponent):
    def __init__(self):
        self.data = {}
        self.events = []

    def start_run(self) -> None:
        self.data = {"started": True, "success": None, "reason": ""}

    def set_task_code(self, task_code: str) -> None:
        self.data["task_code"] = task_code

    def record_pickup(self, batch_number: int, material_code: int) -> None:
        self.events.append(("pickup", batch_number, material_code))

    def record_processing(
        self,
        batch_number: int,
        material_code: int,
        process_position: int,
    ) -> None:
        self.events.append(
            ("processing", batch_number, material_code, process_position)
        )

    def record_storage(self, material_code: int) -> None:
        self.events.append(("storage", material_code))

    def record_stack(self, material_code: int) -> None:
        self.events.append(("stack", material_code))

    def finish_run(self, success: bool, reason: str = "") -> None:
        self.data["success"] = success
        self.data["reason"] = reason
        self.data["event_count"] = len(self.events)

    def snapshot(self) -> Mapping[str, Any]:
        result = dict(self.data)
        result["events"] = list(self.events)
        return result


class LoggingTelemetry(HealthyComponent):
    def __init__(self):
        self.messages = []

    def publish(self, topic: str, payload: Mapping[str, Any]) -> None:
        message = (topic, dict(payload))
        self.messages.append(message)
        LOGGER.debug("遥测 %s：%s", topic, payload)


class SimulatedLighting(HealthyComponent):
    def __init__(self):
        self.enabled = False

    def set_enabled(self, enabled: bool) -> None:
        self.enabled = enabled


class ImmediateRecovery(HealthyComponent):
    def __init__(self):
        self.events = []

    def recover(self, failed_state: RobotState, reason: str) -> ActionResult:
        self.events.append((failed_state.name, reason))
        return ActionResult.done("模拟恢复完成", activity=False)


def build_simulated_components(
    task_code: str,
    auto_start: bool = False,
    start_delay_seconds: float = 0.5,
) -> ComponentBundle:
    return ComponentBundle(
        start_button=SimulatedStartButton(auto_start, start_delay_seconds),
        task_code_reader=StaticTaskCodeReader(task_code),
        display=ConsoleDisplay(),
        motion=SimulatedMotionController(),
        navigator=SimulatedNavigator(),
        material_perception=SimulatedMaterialPerception(),
        manipulator=SimulatedManipulator(),
        safety=SimulatedSafetyMonitor(),
        statistics=InMemoryStatistics(),
        telemetry=LoggingTelemetry(),
        lighting=SimulatedLighting(),
        recovery=ImmediateRecovery(),
    )
