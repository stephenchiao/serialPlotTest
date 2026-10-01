"""智能搬运机器人底层有限状态机。"""

import logging
import math
import time
from typing import Callable, Optional

from robot_hardware.stm32.serial_link import SerialLinkError

from robot_mission.task_code import (
    BatchTask,
    CompetitionTask,
    MaterialTask,
    TaskCodeError,
    parse_task_code,
)

from .config import RuntimeConfig
from .interfaces import Clock, ComponentBundle
from .models import (
    ActionResult,
    ActionStatus,
    RobotState,
    TargetArea,
    TransitionRecord,
)


LOGGER = logging.getLogger(__name__)


class SystemClock:
    def monotonic(self) -> float:
        return time.monotonic()

    def sleep(self, seconds: float) -> None:
        time.sleep(seconds)


class RobotStateMachine:
    """只负责编排流程；硬件行为全部由 ComponentBundle 注入。"""

    TERMINAL_STATES = {RobotState.COMPLETED, RobotState.SAFE_STOP}
    NAVIGATION_STATES = {
        RobotState.NAVIGATING_TO_SOURCE,
        RobotState.NAVIGATING_TO_PROCESSING,
        RobotState.NAVIGATING_TO_TEMPORARY_STORAGE,
        RobotState.NAVIGATING_TO_STACKING,
    }
    REPEATABLE_STATES = NAVIGATION_STATES | {RobotState.LOCATING_MATERIAL}

    def __init__(
        self,
        components: ComponentBundle,
        config: RuntimeConfig,
        clock: Optional[Clock] = None,
    ):
        self.components = components
        self.config = config
        self.clock = clock or SystemClock()
        self.state = RobotState.BOOTING
        self.task: Optional[CompetitionTask] = None
        self.transitions = []
        self.stop_reason = ""

        now = self.clock.monotonic()
        self._state_entered_at = now
        self._action_started_at = now
        self._last_activity_at = now
        self._task_code_deadline = 0.0
        self._self_check_index = 0
        self._start_button_armed = False
        self._mission_started = False
        self._finished_recorded = False
        self._batch_number = 1
        self._pickup_index = 0
        self._placement_index = 0
        self._retry_count = 0
        self._recovery_resume_state: Optional[RobotState] = None
        self._recovery_reason = ""
        self._action_wait_resume_state: Optional[RobotState] = None
        self._action_retry_at = 0.0
        self._safety_resume_state: Optional[RobotState] = None
        self._safety_pause_started_at = 0.0
        self._safety_clear_since: Optional[float] = None
        self._safety_clear_samples = 0
        self._safety_last_observation: Optional[float] = None
        self._safety_pause_reason = ""
        self._safety_pause_notified = False
        self._link_wait_started_at: Optional[float] = None

    @property
    def is_terminal(self) -> bool:
        return self.state in self.TERMINAL_STATES

    def tick(self) -> RobotState:
        """执行一个短周期；适合主循环反复调用。"""
        if self.is_terminal:
            return self.state

        try:
            if self.state is not RobotState.BOOTING and not self._check_safety():
                return self.state

            handler = {
                RobotState.BOOTING: self._handle_booting,
                RobotState.SELF_CHECK: self._handle_self_check,
                RobotState.WAITING_FOR_START: self._handle_waiting_for_start,
                RobotState.READING_TASK_CODE: self._handle_reading_task_code,
                RobotState.NAVIGATING_TO_SOURCE: self._handle_navigating_to_source,
                RobotState.LOCATING_MATERIAL: self._handle_locating_material,
                RobotState.PICKING_MATERIAL: self._handle_picking_material,
                RobotState.NAVIGATING_TO_PROCESSING: (
                    self._handle_navigating_to_processing
                ),
                RobotState.PLACING_FOR_PROCESSING: (
                    self._handle_placing_for_processing
                ),
                RobotState.NAVIGATING_TO_TEMPORARY_STORAGE: (
                    self._handle_navigating_to_temporary_storage
                ),
                RobotState.PLACING_IN_TEMPORARY_STORAGE: (
                    self._handle_placing_in_temporary_storage
                ),
                RobotState.NAVIGATING_TO_STACKING: (
                    self._handle_navigating_to_stacking
                ),
                RobotState.STACKING_SECOND_BATCH: self._handle_stacking_second_batch,
                RobotState.RECOVERING: self._handle_recovering,
                RobotState.ACTION_WAITING: self._handle_action_waiting,
                # 暂停和恢复都由 _check_safety 驱动，不调用任务动作。
                RobotState.SAFETY_PAUSED: lambda: None,
                RobotState.REPORTING: self._handle_reporting,
            }[self.state]
            handler()
        except Exception as error:  # 最外层保证任何组件异常都先停车
            LOGGER.exception("状态机未处理异常")
            self.safe_stop(f"{type(error).__name__}: {error}")
        return self.state

    def run(self, should_stop: Optional[Callable[[], bool]] = None) -> RobotState:
        """阻塞运行；终态后返回，服务保持显示由 app 层负责。"""
        should_stop = should_stop or (lambda: False)
        while not should_stop() and not self.is_terminal:
            self.tick()
            self.clock.sleep(self.config.loop_interval_seconds)
        return self.state

    def shutdown(self) -> None:
        """服务退出时释放硬件，始终先停止运动。"""
        self._stop_motion()
        for component in reversed(self.components.unique_components()):
            method = getattr(component, "shutdown", None)
            if method is not None:
                try:
                    method()
                except Exception:
                    LOGGER.exception("组件关闭失败：%s", type(component).__name__)
        try:
            self.components.lighting.set_enabled(False)
        except Exception:
            LOGGER.exception("关闭照明失败")

    def safe_stop(self, reason: str) -> None:
        """进入不可自动恢复的安全停车状态。"""
        if self.state is RobotState.SAFE_STOP:
            return
        self.stop_reason = reason
        self._stop_motion()
        if self._mission_started and not self._finished_recorded:
            self.components.statistics.finish_run(False, reason)
            self._finished_recorded = True
        self._transition(RobotState.SAFE_STOP, reason)
        snapshot = dict(self.components.statistics.snapshot())
        snapshot["success"] = False
        snapshot["reason"] = reason
        self.components.display.show_final(snapshot)
        self._publish("safe_stop", snapshot)

    def _handle_booting(self) -> None:
        for component in self.components.unique_components():
            initialize = getattr(component, "initialize", None)
            if initialize is not None:
                initialize()
        self.components.lighting.set_enabled(True)
        self._transition(RobotState.SELF_CHECK, "组件初始化完成")

    def _handle_self_check(self) -> None:
        components = self.components.unique_components()
        while self._self_check_index < len(components):
            component = components[self._self_check_index]
            self_check = getattr(component, "self_check", None)
            if self_check is None:
                self._self_check_index += 1
                continue

            result = self_check()

            def advance():
                self._self_check_index += 1
                self._action_started_at = self.clock.monotonic()

            self._handle_action_result(result, advance)
            return

        self._transition(RobotState.WAITING_FOR_START, "自检通过，等待实体按钮")

    def _handle_waiting_for_start(self) -> None:
        pressed = self.components.start_button.is_pressed()
        if not pressed:
            self._start_button_armed = True
            return
        if not self._start_button_armed:
            return

        now = self.clock.monotonic()
        self._mission_started = True
        self._last_activity_at = now
        self.components.statistics.start_run()
        self._task_code_deadline = now + self.config.task_code_timeout_seconds
        self._transition(RobotState.READING_TASK_CODE, "检测到实体 Start 按钮上升沿")

    def _handle_reading_task_code(self) -> None:
        raw_code = self.components.task_code_reader.read_task_code()
        if raw_code:
            try:
                task = parse_task_code(raw_code)
            except TaskCodeError as error:
                self.components.display.show_state(
                    RobotState.READING_TASK_CODE,
                    f"任务码无效：{error}",
                )
                self._publish("invalid_task_code", {"error": str(error)})
            else:
                self.task = task
                self.components.statistics.set_task_code(task.raw_code)
                self.components.display.show_task_code(task.raw_code)
                self._batch_number = 1
                self._pickup_index = 0
                self._placement_index = 0
                self._transition(
                    RobotState.NAVIGATING_TO_SOURCE,
                    f"任务码已确认：{task.raw_code}",
                )
                return

        if self.clock.monotonic() >= self._task_code_deadline:
            self._task_code_deadline = self.clock.monotonic() + self.config.task_code_timeout_seconds
            self.components.display.show_state(self.state, "暂未获得合法任务码，保持停车继续扫描")
            self._publish("task_code_waiting", {"reason": "暂未获得合法任务码"})

    def _handle_action_waiting(self) -> None:
        resume_state = self._action_wait_resume_state
        if resume_state not in self.REPEATABLE_STATES:
            self.safe_stop("动作等待缺少可重复执行的目标状态")
            return
        if self.clock.monotonic() < self._action_retry_at:
            return
        if self.components.motion.is_active() or self.components.manipulator.is_active():
            return
        self._action_wait_resume_state = None
        self._transition(resume_state, "等待结束，重新规划或识别")

    def _wait_for_action(self, resume_state: RobotState, reason: str) -> None:
        if not self._stop_motion(allow_disconnected=True):
            self.safe_stop("动作等待时无法确认停车指令已发送")
            return
        self._action_wait_resume_state = resume_state
        self._action_retry_at = self.clock.monotonic() + self.config.action_retry_wait_seconds
        self._transition(RobotState.ACTION_WAITING, reason, reset_retries=False)

    def _handle_navigating_to_source(self) -> None:
        result = self.components.navigator.navigate_to(TargetArea.SOURCE_TURNTABLE)
        self._handle_action_result(
            result,
            lambda: self._transition(RobotState.LOCATING_MATERIAL, "已到达物料转盘"),
        )

    def _handle_locating_material(self) -> None:
        material = self._pickup_material()
        result = self.components.material_perception.locate_material(
            material.material_code
        )
        self._handle_action_result(
            result,
            lambda: self._transition(
                RobotState.PICKING_MATERIAL,
                f"已定位{material.material_code}号物料",
            ),
        )

    def _handle_picking_material(self) -> None:
        material = self._pickup_material()
        result = self.components.manipulator.pick_to_payload(
            material,
            self._batch_number,
            self._pickup_index + 1,
        )

        def advance():
            self.components.statistics.record_pickup(
                self._batch_number, material.material_code
            )
            self._pickup_index += 1
            if self._pickup_index < 3:
                self._transition(RobotState.LOCATING_MATERIAL, "继续抓取本批下一件")
            else:
                self._placement_index = 0
                self._transition(
                    RobotState.NAVIGATING_TO_PROCESSING,
                    "本批三件物料已全部装载",
                )

        self._handle_action_result(result, advance)

    def _handle_navigating_to_processing(self) -> None:
        result = self.components.navigator.navigate_to(TargetArea.PROCESSING)
        self._handle_action_result(
            result,
            lambda: self._transition(
                RobotState.PLACING_FOR_PROCESSING, "已到达加工区"
            ),
        )

    def _handle_placing_for_processing(self) -> None:
        material = self._placement_material()
        result = self.components.manipulator.place_for_processing(
            material, self._batch_number
        )

        def advance():
            self.components.statistics.record_processing(
                self._batch_number,
                material.material_code,
                material.process_position,
            )
            self._placement_index += 1
            if self._placement_index < 3:
                self._restart_current_action()
            elif self._batch_number == 1:
                self._placement_index = 0
                self._transition(
                    RobotState.NAVIGATING_TO_TEMPORARY_STORAGE,
                    "第一批加工放置完成",
                )
            else:
                self._placement_index = 0
                self._transition(
                    RobotState.NAVIGATING_TO_STACKING,
                    "第二批加工放置完成",
                )

        self._handle_action_result(result, advance)

    def _handle_navigating_to_temporary_storage(self) -> None:
        result = self.components.navigator.navigate_to(TargetArea.TEMPORARY_STORAGE)
        self._handle_action_result(
            result,
            lambda: self._transition(
                RobotState.PLACING_IN_TEMPORARY_STORAGE, "已到达暂存区"
            ),
        )

    def _handle_placing_in_temporary_storage(self) -> None:
        material = self._placement_material()
        result = self.components.manipulator.place_in_temporary_storage(material)

        def advance():
            self.components.statistics.record_storage(material.material_code)
            self._placement_index += 1
            if self._placement_index < 3:
                self._restart_current_action()
            else:
                self._batch_number = 2
                self._pickup_index = 0
                self._placement_index = 0
                self._transition(
                    RobotState.NAVIGATING_TO_SOURCE,
                    "第一批暂存完成，开始第二批",
                )

        self._handle_action_result(result, advance)

    def _handle_navigating_to_stacking(self) -> None:
        result = self.components.navigator.navigate_to(TargetArea.TEMPORARY_STORAGE)
        self._handle_action_result(
            result,
            lambda: self._transition(
                RobotState.STACKING_SECOND_BATCH, "已到达对应堆叠区"
            ),
        )

    def _handle_stacking_second_batch(self) -> None:
        material = self._placement_material()
        result = self.components.manipulator.stack_second_batch(material)

        def advance():
            self.components.statistics.record_stack(material.material_code)
            self._placement_index += 1
            if self._placement_index < 3:
                self._restart_current_action()
            else:
                self._transition(RobotState.REPORTING, "全部搬运与堆叠完成")

        self._handle_action_result(result, advance)

    def _handle_recovering(self) -> None:
        resume_state = self._recovery_resume_state
        if resume_state is None:
            self.safe_stop("恢复状态缺少目标状态")
            return
        result = self.components.recovery.recover(resume_state, self._recovery_reason)
        if result.activity:
            self._last_activity_at = self.clock.monotonic()
        if result.status is ActionStatus.DONE:
            self._transition(resume_state, "恢复完成，重试动作", reset_retries=False)
        elif result.status is ActionStatus.FATAL_ERROR:
            self.safe_stop(f"恢复失败：{result.message}")
        elif (
            result.status is ActionStatus.RETRYABLE_ERROR
            or self._action_timed_out()
        ):
            reason = f"恢复超时或失败：{result.message}"
            if resume_state in self.REPEATABLE_STATES:
                self._wait_for_action(resume_state, reason)
            else:
                self.safe_stop(reason)

    def _handle_reporting(self) -> None:
        self._stop_motion()
        if not self._finished_recorded:
            self.components.statistics.finish_run(True)
            self._finished_recorded = True
        snapshot = dict(self.components.statistics.snapshot())
        snapshot["success"] = True
        self._transition(RobotState.COMPLETED, "任务正常完成")
        self.components.display.show_final(snapshot)
        self._publish("final_statistics", snapshot)

    def _handle_action_result(
        self,
        result: ActionResult,
        on_done: Callable[[], None],
    ) -> None:
        if not isinstance(result, ActionResult):
            self.safe_stop("组件没有返回 ActionResult")
            return
        if result.activity:
            self._last_activity_at = self.clock.monotonic()
        if result.status is ActionStatus.DONE:
            on_done()
        elif result.status is ActionStatus.FATAL_ERROR:
            self.safe_stop(result.message or f"{self.state.name}发生致命错误")
        elif result.status is ActionStatus.RETRYABLE_ERROR:
            self._begin_recovery(result.message or "组件要求重试")
        elif self._action_timed_out():
            self._begin_recovery(f"{self.state.name}动作超时")

    def _begin_recovery(self, reason: str) -> None:
        # 动作调用期间断线时，先转通信等待，不能耗尽动作重试预算。
        if self.components.safety.check().waiting_for_link:
            self._check_safety()
            return
        if self._retry_count >= self.config.max_action_retries:
            message = f"动作重试次数耗尽：{reason}"
            if self.state in self.REPEATABLE_STATES:
                self._wait_for_action(self.state, message)
            else:
                self.safe_stop(message)
            return
        self._retry_count += 1
        self._recovery_resume_state = self.state
        self._recovery_reason = reason
        if not self._stop_motion(allow_disconnected=True):
            self.safe_stop("动作恢复时无法确认停车指令已发送")
            return
        self._transition(
            RobotState.RECOVERING,
            f"第{self._retry_count}次有界恢复：{reason}",
            reset_retries=False,
        )

    def _check_safety(self) -> bool:
        effective_state = (
            self._safety_resume_state
            if self.state is RobotState.SAFETY_PAUSED else self.state
        )
        if effective_state is RobotState.ACTION_WAITING:
            effective_state = self._action_wait_resume_state
        elif effective_state is RobotState.RECOVERING:
            effective_state = self._recovery_resume_state
        require_navigation = getattr(self.components.safety, "set_navigation_required", None)
        if require_navigation is not None:
            require_navigation(effective_state in self.NAVIGATION_STATES)
        report = self.components.safety.check()
        # 显式急停和真实地图越界不能被 recoverable 覆盖。
        if report.emergency_stop or not report.boundary_ok or (not report.safe and not report.recoverable):
            self.safe_stop(report.reason or "安全监控触发停车")
            return False

        now = self.clock.monotonic()
        if self._mission_started and (
            self.components.motion.is_active() or self.components.manipulator.is_active()
        ):
            self._last_activity_at = now
        if not report.safe:
            if self.state is not RobotState.SAFETY_PAUSED:
                self._safety_resume_state = self.state
                self._safety_pause_started_at = now
                self._safety_pause_notified = False
                begin_recheck = getattr(self.components.safety, "begin_recheck", None)
                if begin_recheck is not None:
                    begin_recheck()
                if not self._stop_motion(allow_disconnected=True):
                    self.safe_stop("安全暂停时无法确认停车指令已发送")
                    return False
                self._transition(RobotState.SAFETY_PAUSED, report.reason, reset_retries=False)
            self._safety_pause_reason = report.reason
            self._safety_clear_since = None
            self._safety_clear_samples = 0
            self._safety_last_observation = None

        if self.state is RobotState.SAFETY_PAUSED:
            if report.waiting_for_link:
                if self._link_wait_started_at is None:
                    self._link_wait_started_at = now
                return False
            if self._link_wait_started_at is not None:
                if self._safety_resume_state is RobotState.READING_TASK_CODE:
                    self._task_code_deadline += now - self._link_wait_started_at
                self._link_wait_started_at = None
                self._safety_pause_started_at = now
            # 超过复核提醒时间仍停着等待；恢复条件不因等待时间变长而放宽。
            if (
                not self._safety_pause_notified
                and now - self._safety_pause_started_at >= self.config.safety_pause_timeout_seconds
            ):
                self._safety_pause_notified = True
                message = f"安全复核仍未完成，保持停车等待：{self._safety_pause_reason}"
                LOGGER.warning(message)
                self.components.display.show_state(self.state, message)
                self._publish("safety_pause_extended", {"reason": self._safety_pause_reason})
            if not report.safe:
                return False
            if self._safety_clear_since is None:
                self._safety_clear_since = now
            timestamp = report.observation_timestamp
            if timestamp is not None and math.isfinite(timestamp) and (
                self._safety_last_observation is None or timestamp > self._safety_last_observation
            ):
                self._safety_clear_samples += 1
                self._safety_last_observation = timestamp
            if (
                now - self._safety_clear_since < self.config.safety_resume_stable_seconds
                or self._safety_clear_samples < self.config.safety_resume_confirmations
            ):
                return False
            resume_state = self._safety_resume_state
            if resume_state is None:
                self.safe_stop("安全暂停缺少恢复目标")
                return False
            end_recheck = getattr(self.components.safety, "end_recheck", None)
            if end_recheck is not None:
                end_recheck()
            self._safety_resume_state = None
            self._transition(resume_state, "新画面持续确认安全，重新执行原任务", reset_retries=False)
            return False  # 下一周期再次检查安全后才能下发新航点

        if (
            self._mission_started
            and self.state not in {
                RobotState.RECOVERING, RobotState.ACTION_WAITING, RobotState.READING_TASK_CODE,
            }
            and now - max(self._last_activity_at, self._action_started_at)
            >= self.config.inactivity_timeout_seconds
        ):
            # 每次动作/恢复重试独立计时，不把等待恢复伪记为物理活动。
            self._begin_recovery(
                f"连续{self.config.inactivity_timeout_seconds:g}秒未检测到物理动作"
            )
            return False
        return True

    def _current_batch(self) -> BatchTask:
        if self.task is None:
            raise RuntimeError("任务码尚未解析")
        return self.task.first_batch if self._batch_number == 1 else self.task.second_batch

    def _pickup_material(self) -> MaterialTask:
        return self._current_batch().materials[self._pickup_index]

    def _placement_material(self) -> MaterialTask:
        return self._current_batch().materials[self._placement_index]

    def _action_timed_out(self) -> bool:
        return (
            self.clock.monotonic() - self._action_started_at
            >= self.config.action_timeout_seconds
        )

    def _restart_current_action(self) -> None:
        self._action_started_at = self.clock.monotonic()
        self._retry_count = 0

    def _stop_motion(self, *, allow_disconnected: bool = False) -> bool:
        succeeded = True
        for operation in (
            self.components.motion.stop,
            self.components.navigator.cancel,
            self.components.manipulator.stop,
        ):
            try:
                operation()
            except Exception as error:
                # 只容忍已进入重连等待的通信发送失败，其他组件异常仍终止。
                if not (
                    allow_disconnected and isinstance(error, SerialLinkError)
                    and self.components.safety.check().waiting_for_link
                ):
                    succeeded = False
                LOGGER.exception("安全停车操作失败")
        return succeeded

    def _transition(
        self,
        new_state: RobotState,
        reason: str,
        reset_retries: bool = True,
    ) -> None:
        now = self.clock.monotonic()
        previous = self.state
        self.state = new_state
        self._state_entered_at = now
        self._action_started_at = now
        if reset_retries:
            self._retry_count = 0
        record = TransitionRecord(now, previous, new_state, reason)
        self.transitions.append(record)
        LOGGER.info("状态迁移 %s -> %s：%s", previous.name, new_state.name, reason)
        self.components.display.show_state(new_state, reason)
        self._publish(
            "state_transition",
            {
                "timestamp": now,
                "previous": previous.name,
                "current": new_state.name,
                "reason": reason,
            },
        )

    def _publish(self, topic: str, payload) -> None:
        try:
            self.components.telemetry.publish(topic, payload)
        except Exception:
            LOGGER.exception("遥测发布失败：%s", topic)
