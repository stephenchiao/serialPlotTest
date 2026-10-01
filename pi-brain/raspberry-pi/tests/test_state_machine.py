"""状态机无硬件单元测试。"""

import unittest

from robot_runtime.config import RuntimeConfig
from robot_runtime.models import ActionResult, RobotState, SafetyReport
from robot_runtime.state_machine import RobotStateMachine
from robot_simulation.components import HealthyComponent, build_simulated_components


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds

    def advance(self, seconds):
        self.now += seconds


class HangingNavigator(HealthyComponent):
    def navigate_to(self, _target):
        return ActionResult.running("模拟底盘无物理动作", activity=False)

    def cancel(self):
        pass


class RetryOnceNavigator(HealthyComponent):
    def __init__(self):
        self.calls = 0

    def navigate_to(self, _target):
        self.calls += 1
        if self.calls == 1:
            return ActionResult.retryable("模拟丢线")
        return ActionResult.done("恢复后到达")

    def cancel(self):
        pass


class RobotStateMachineTests(unittest.TestCase):
    def make_machine(self, **config_overrides):
        values = {
            "loop_interval_seconds": 0.01,
            "task_code_timeout_seconds": 1.0,
            "action_timeout_seconds": 1.0,
            "inactivity_timeout_seconds": 5.0,
            "max_action_retries": 2,
        }
        values.update(config_overrides)
        config = RuntimeConfig(**values)
        components = build_simulated_components(
            "452+321+254+312",
            auto_start=False,
        )
        clock = FakeClock()
        return RobotStateMachine(components, config, clock), components, clock

    def advance_to_waiting(self, machine, clock):
        for _ in range(50):
            machine.tick()
            clock.advance(0.01)
            if machine.state is RobotState.WAITING_FOR_START:
                return
        self.fail("状态机未进入 WAITING_FOR_START")

    def press_start(self, machine, components, clock):
        machine.tick()  # 先观察到释放状态，完成按钮解锁
        clock.advance(0.01)
        components.start_button.manually_pressed = True
        machine.tick()
        clock.advance(0.01)
        self.assertEqual(machine.state, RobotState.READING_TASK_CODE)

    def test_complete_two_batch_mission(self):
        machine, components, clock = self.make_machine()
        self.advance_to_waiting(machine, clock)
        self.press_start(machine, components, clock)

        for _ in range(100):
            machine.tick()
            clock.advance(0.01)
            if machine.is_terminal:
                break

        self.assertEqual(machine.state, RobotState.COMPLETED)
        self.assertEqual(components.display.task_code, "452+321+254+312")
        self.assertTrue(components.display.final_statistics["success"])
        self.assertEqual(len(components.statistics.events), 18)
        self.assertEqual(
            [event for event in components.manipulator.events if event[0] == "pickup"],
            [
                ("pickup", 1, 4, 1),
                ("pickup", 1, 5, 2),
                ("pickup", 1, 2, 3),
                ("pickup", 2, 2, 1),
                ("pickup", 2, 5, 2),
                ("pickup", 2, 4, 3),
            ],
        )

    def test_does_not_start_until_button_release_then_press(self):
        machine, components, clock = self.make_machine()
        components.start_button.manually_pressed = True
        self.advance_to_waiting(machine, clock)

        for _ in range(5):
            machine.tick()
            clock.advance(0.01)
        self.assertEqual(machine.state, RobotState.WAITING_FOR_START)

        components.start_button.manually_pressed = False
        machine.tick()
        components.start_button.manually_pressed = True
        machine.tick()
        self.assertEqual(machine.state, RobotState.READING_TASK_CODE)

    def test_invalid_task_code_keeps_scanning_and_can_later_finish(self):
        machine, components, clock = self.make_machine(
            task_code_timeout_seconds=0.25
        )
        components.task_code_reader.task_code = "452"
        self.advance_to_waiting(machine, clock)
        self.press_start(machine, components, clock)

        for _ in range(10):
            machine.tick()
            clock.advance(0.05)
            if machine.is_terminal:
                break

        self.assertEqual(machine.state, RobotState.READING_TASK_CODE)
        self.assertIsNone(components.display.final_statistics)
        components.task_code_reader.task_code = "452+321+254+312"
        for _ in range(100):
            machine.tick()
            clock.advance(0.01)
            if machine.is_terminal:
                break
        self.assertEqual(machine.state, RobotState.COMPLETED)

    def test_safety_monitor_forces_stop(self):
        machine, components, clock = self.make_machine()
        self.advance_to_waiting(machine, clock)
        self.press_start(machine, components, clock)
        components.safety.report = SafetyReport(
            safe=False,
            reason="测试急停",
            emergency_stop=True,
        )

        machine.tick()

        self.assertEqual(machine.state, RobotState.SAFE_STOP)
        self.assertEqual(machine.stop_reason, "测试急停")
        self.assertGreater(components.motion.stop_count, 0)

    def test_inactivity_retry_exhaustion_waits_then_restarts_navigation(self):
        machine, components, clock = self.make_machine(
            action_timeout_seconds=2.0,
            inactivity_timeout_seconds=0.25,
        )
        components.navigator = HangingNavigator()
        working_navigator = build_simulated_components("452+321+254+312").navigator
        self.advance_to_waiting(machine, clock)
        self.press_start(machine, components, clock)

        for _ in range(20):
            machine.tick()
            clock.advance(0.05)
            if machine.state is RobotState.ACTION_WAITING:
                break

        self.assertEqual(machine.state, RobotState.ACTION_WAITING)
        self.assertEqual(len(components.recovery.events), 2)
        self.assertIn("动作重试次数耗尽", machine.transitions[-1].reason)
        last_activity = machine._last_activity_at
        task = machine.task
        components.navigator = working_navigator
        clock.advance(0.5)
        machine.tick()
        self.assertEqual(machine.state, RobotState.ACTION_WAITING)
        clock.advance(0.51)
        machine.tick()
        self.assertEqual(machine.state, RobotState.NAVIGATING_TO_SOURCE)
        self.assertEqual(machine._last_activity_at, last_activity)
        self.assertIs(machine.task, task)
        self.assertEqual(machine._retry_count, 0)
        for _ in range(100):
            machine.tick()
            clock.advance(0.01)
            if machine.is_terminal:
                break
        self.assertEqual(machine.state, RobotState.COMPLETED)

    def test_inactivity_recovery_can_finish_the_mission(self):
        machine, components, clock = self.make_machine(inactivity_timeout_seconds=0.25)
        working_navigator = components.navigator
        components.navigator = HangingNavigator()
        self.advance_to_waiting(machine, clock)
        self.press_start(machine, components, clock)
        for _ in range(10):
            machine.tick()
            clock.advance(0.05)
            if machine.state is RobotState.RECOVERING:
                break
        self.assertEqual(machine.state, RobotState.RECOVERING)
        components.navigator = working_navigator
        for _ in range(100):
            machine.tick()
            clock.advance(0.01)
            if machine.is_terminal:
                break
        self.assertEqual(machine.state, RobotState.COMPLETED)
        self.assertEqual(len(components.recovery.events), 1)

    def test_retryable_action_runs_recovery_then_continues(self):
        machine, components, clock = self.make_machine()
        retrying_navigator = RetryOnceNavigator()
        components.navigator = retrying_navigator
        self.advance_to_waiting(machine, clock)
        self.press_start(machine, components, clock)

        for _ in range(10):
            machine.tick()
            clock.advance(0.01)
            if retrying_navigator.calls >= 2:
                break

        self.assertGreaterEqual(retrying_navigator.calls, 2)
        self.assertTrue(components.recovery.events)
        self.assertNotEqual(machine.state, RobotState.SAFE_STOP)

    def paused_machine(self, **overrides):
        machine, components, clock = self.make_machine(**overrides)
        self.advance_to_waiting(machine, clock)
        self.press_start(machine, components, clock)
        machine.tick()
        components.navigator = HangingNavigator()
        components.safety.report = SafetyReport(False, "疑似障碍", recoverable=True)
        machine.tick()
        self.assertEqual(machine.state, RobotState.SAFETY_PAUSED)
        return machine, components, clock

    def test_temporary_false_positive_resumes_same_task_without_failure_record(self):
        machine, components, clock = self.paused_machine()
        destination = machine._safety_resume_state
        last_activity = machine._last_activity_at
        for _ in range(4):
            clock.advance(0.2)
            components.safety.report = SafetyReport(observation_timestamp=clock.now)
            machine.tick()
        self.assertEqual(machine.state, destination)
        self.assertIsNone(components.display.final_statistics)
        self.assertEqual(machine._last_activity_at, last_activity)
        self.assertFalse(components.recovery.events)

    def test_same_frame_cannot_count_as_multiple_clear_observations(self):
        machine, components, clock = self.paused_machine()
        timestamp = clock.now
        for _ in range(5):
            clock.advance(0.2)
            components.safety.report = SafetyReport(observation_timestamp=timestamp)
            machine.tick()
        self.assertEqual(machine.state, RobotState.SAFETY_PAUSED)
        self.assertEqual(machine._safety_clear_samples, 1)

    def test_no_fresh_observation_cannot_resume(self):
        machine, components, clock = self.paused_machine()
        components.safety.report = SafetyReport()
        clock.advance(0.8)
        machine.tick()
        self.assertEqual(machine.state, RobotState.SAFETY_PAUSED)

    def test_persistent_recheck_failure_waits_and_later_resumes(self):
        machine, components, clock = self.paused_machine(safety_pause_timeout_seconds=1.0)
        clock.advance(1.1)
        machine.tick()
        self.assertEqual(machine.state, RobotState.SAFETY_PAUSED)
        self.assertTrue(machine._safety_pause_notified)
        self.assertIsNone(components.display.final_statistics)
        clock.advance(30)
        machine.tick()
        for _ in range(2):
            clock.advance(0.21)
            components.safety.report = SafetyReport(observation_timestamp=clock.now)
            machine.tick()
        self.assertEqual(machine.state, RobotState.NAVIGATING_TO_SOURCE)

    def test_flapping_perception_does_not_restart_pause_deadline(self):
        machine, components, clock = self.paused_machine(safety_pause_timeout_seconds=1.0)
        for index in range(6):
            clock.advance(0.2)
            components.safety.report = (
                SafetyReport(observation_timestamp=clock.now) if index % 2
                else SafetyReport(False, "疑似障碍", recoverable=True)
            )
            machine.tick()
        self.assertEqual(machine.state, RobotState.SAFETY_PAUSED)
        self.assertTrue(machine._safety_pause_notified)
        self.assertEqual(machine._safety_clear_samples, 1)

    def test_pause_uses_its_own_deadline_and_resume_gets_new_action_window(self):
        machine, components, clock = self.paused_machine(inactivity_timeout_seconds=0.4)
        last_activity = machine._last_activity_at
        clock.advance(0.5)
        machine.tick()
        self.assertEqual(machine.state, RobotState.SAFETY_PAUSED)
        for _ in range(2):
            clock.advance(0.21)
            components.safety.report = SafetyReport(observation_timestamp=clock.now)
            machine.tick()
        self.assertEqual(machine.state, RobotState.NAVIGATING_TO_SOURCE)
        machine.tick()
        self.assertEqual(machine.state, RobotState.NAVIGATING_TO_SOURCE)
        self.assertEqual(machine._last_activity_at, last_activity)
        clock.advance(0.41)
        machine.tick()
        self.assertEqual(machine.state, RobotState.RECOVERING)

    def test_recovery_still_has_its_own_action_timeout(self):
        machine, components, clock = self.make_machine(
            action_timeout_seconds=0.5, inactivity_timeout_seconds=0.1,
        )
        components.navigator = HangingNavigator()
        components.recovery.recover = lambda *_args: ActionResult.running(activity=False)
        self.advance_to_waiting(machine, clock)
        self.press_start(machine, components, clock)
        for _ in range(20):
            machine.tick()
            clock.advance(0.1)
            if machine.state is RobotState.ACTION_WAITING:
                break
        self.assertEqual(machine.state, RobotState.ACTION_WAITING)
        self.assertIn("恢复超时或失败", machine.transitions[-1].reason)

    def test_hard_fault_during_recheck_latches_even_if_marked_recoverable(self):
        machine, components, clock = self.paused_machine()
        components.safety.report = SafetyReport(
            False, "硬急停", emergency_stop=True, recoverable=True,
        )
        machine.tick()
        self.assertEqual(machine.state, RobotState.SAFE_STOP)

    def test_stop_failure_cannot_be_followed_by_auto_resume(self):
        machine, components, clock = self.make_machine()
        self.advance_to_waiting(machine, clock)

        def fail_stop():
            raise OSError("lost motor link")

        components.motion.stop = fail_stop
        components.safety.report = SafetyReport(False, "疑似障碍", recoverable=True)
        with self.assertLogs("robot_runtime.state_machine", level="ERROR"):
            machine.tick()
        self.assertEqual(machine.state, RobotState.SAFE_STOP)
        self.assertIn("无法确认停车", machine.stop_reason)


if __name__ == "__main__":
    unittest.main()
