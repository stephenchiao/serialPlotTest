"""机器人运行时使用的公共数据模型。"""

from dataclasses import dataclass
from enum import Enum, auto
from typing import Any, Mapping, Optional


class RobotState(Enum):
    """比赛主流程状态。"""

    BOOTING = auto()
    SELF_CHECK = auto()
    WAITING_FOR_START = auto()
    READING_TASK_CODE = auto()
    NAVIGATING_TO_SOURCE = auto()
    LOCATING_MATERIAL = auto()
    PICKING_MATERIAL = auto()
    NAVIGATING_TO_PROCESSING = auto()
    PLACING_FOR_PROCESSING = auto()
    NAVIGATING_TO_TEMPORARY_STORAGE = auto()
    PLACING_IN_TEMPORARY_STORAGE = auto()
    NAVIGATING_TO_STACKING = auto()
    STACKING_SECOND_BATCH = auto()
    RECOVERING = auto()
    ACTION_WAITING = auto()
    SAFETY_PAUSED = auto()
    REPORTING = auto()
    COMPLETED = auto()
    SAFE_STOP = auto()


class TargetArea(Enum):
    """导航层需要支持的逻辑目的地。"""

    SOURCE_TURNTABLE = "source_turntable"
    PROCESSING = "processing"
    TEMPORARY_STORAGE = "temporary_storage"


class ActionStatus(Enum):
    """非阻塞硬件动作的轮询结果。"""

    RUNNING = auto()
    DONE = auto()
    RETRYABLE_ERROR = auto()
    FATAL_ERROR = auto()


@dataclass(frozen=True)
class ActionResult:
    """组件动作结果；activity表示本次轮询检测到了物理动作。"""

    status: ActionStatus
    message: str = ""
    activity: bool = False

    @classmethod
    def running(cls, message: str = "", activity: bool = True):
        return cls(ActionStatus.RUNNING, message, activity)

    @classmethod
    def done(cls, message: str = "", activity: bool = True):
        return cls(ActionStatus.DONE, message, activity)

    @classmethod
    def retryable(cls, message: str):
        return cls(ActionStatus.RETRYABLE_ERROR, message, False)

    @classmethod
    def fatal(cls, message: str):
        return cls(ActionStatus.FATAL_ERROR, message, False)


@dataclass(frozen=True)
class SafetyReport:
    """安全监控层每个周期返回的汇总。"""

    safe: bool = True
    reason: str = ""
    battery_voltage: Optional[float] = None
    boundary_ok: bool = True
    emergency_stop: bool = False
    # 可复核的感知问题只暂停；急停/真实越界仍优先锁定停车。
    recoverable: bool = False
    observation_timestamp: Optional[float] = None
    # 断线/重新握手期间保持暂停，不使用感知复核的超时终止任务。
    waiting_for_link: bool = False


@dataclass(frozen=True)
class TransitionRecord:
    """用于日志和赛后分析的状态迁移记录。"""

    timestamp: float
    previous: RobotState
    current: RobotState
    reason: str


JsonMapping = Mapping[str, Any]
