"""运行时 JSON 配置读取与校验。"""

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any, Mapping, Optional


DEFAULT_CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "robot.json"


class RuntimeConfigError(ValueError):
    pass


@dataclass(frozen=True)
class RuntimeConfig:
    loop_interval_seconds: float = 0.05
    task_code_timeout_seconds: float = 8.0  # 扫码等待提醒间隔
    action_timeout_seconds: float = 12.0
    inactivity_timeout_seconds: float = 14.0
    max_action_retries: int = 2
    action_retry_wait_seconds: float = 1.0
    safety_pause_timeout_seconds: float = 10.0  # 复核等待提醒阈值，不再终止任务
    safety_resume_stable_seconds: float = 0.2
    safety_resume_confirmations: int = 2
    component_factory: Optional[str] = None
    simulation_auto_start: bool = False
    simulation_start_delay_seconds: float = 0.5
    simulation_task_code: str = "452+321+254+312"
    log_level: str = "INFO"


def _positive_number(data: Mapping[str, Any], key: str, default: float) -> float:
    value = data.get(key, default)
    if (
        not isinstance(value, (int, float)) or isinstance(value, bool)
        or not math.isfinite(value) or value <= 0
    ):
        raise RuntimeConfigError(f"{key} 必须是大于0的数字")
    return float(value)


def load_runtime_config(path: Optional[str] = None) -> RuntimeConfig:
    config_path = Path(path) if path else DEFAULT_CONFIG_PATH
    try:
        with config_path.open("r", encoding="utf-8") as file:
            data = json.load(file)
    except FileNotFoundError as error:
        raise RuntimeConfigError(f"找不到配置文件：{config_path}") from error
    except json.JSONDecodeError as error:
        raise RuntimeConfigError(
            f"配置文件JSON格式错误：第{error.lineno}行，第{error.colno}列"
        ) from error

    if not isinstance(data, dict):
        raise RuntimeConfigError("配置文件根节点必须是JSON对象")

    retries = data.get("max_action_retries", 2)
    if not isinstance(retries, int) or isinstance(retries, bool) or retries < 0:
        raise RuntimeConfigError("max_action_retries 必须是大于等于0的整数")

    confirmations = data.get("safety_resume_confirmations", 2)
    if not isinstance(confirmations, int) or isinstance(confirmations, bool) or confirmations < 2:
        raise RuntimeConfigError("safety_resume_confirmations 必须是至少2的整数")
    pause_timeout = _positive_number(data, "safety_pause_timeout_seconds", 10.0)
    stable_seconds = _positive_number(data, "safety_resume_stable_seconds", 0.2)

    component_factory = data.get("component_factory")
    if component_factory is not None and (
        not isinstance(component_factory, str) or ":" not in component_factory
    ):
        raise RuntimeConfigError("component_factory 必须是 module:function 或 null")

    auto_start = data.get("simulation_auto_start", False)
    if not isinstance(auto_start, bool):
        raise RuntimeConfigError("simulation_auto_start 必须是布尔值")

    task_code = data.get("simulation_task_code", "452+321+254+312")
    if not isinstance(task_code, str) or not task_code:
        raise RuntimeConfigError("simulation_task_code 必须是非空字符串")

    log_level = data.get("log_level", "INFO")
    if not isinstance(log_level, str) or not log_level:
        raise RuntimeConfigError("log_level 必须是非空字符串")

    return RuntimeConfig(
        loop_interval_seconds=_positive_number(data, "loop_interval_seconds", 0.05),
        task_code_timeout_seconds=_positive_number(
            data, "task_code_timeout_seconds", 8.0
        ),
        action_timeout_seconds=_positive_number(data, "action_timeout_seconds", 12.0),
        inactivity_timeout_seconds=_positive_number(
            data, "inactivity_timeout_seconds", 14.0
        ),
        max_action_retries=retries,
        action_retry_wait_seconds=_positive_number(data, "action_retry_wait_seconds", 1.0),
        safety_pause_timeout_seconds=pause_timeout,
        safety_resume_stable_seconds=stable_seconds,
        safety_resume_confirmations=confirmations,
        component_factory=component_factory,
        simulation_auto_start=auto_start,
        simulation_start_delay_seconds=_positive_number(
            data, "simulation_start_delay_seconds", 0.5
        ),
        simulation_task_code=task_code,
        log_level=log_level.upper(),
    )
