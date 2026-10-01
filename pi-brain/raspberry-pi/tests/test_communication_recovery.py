"""断线等待、重新握手确认静止，以及原任务自动续跑。"""

import struct
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from robot_control.navigation_runtime import build_navigation_runtime
from robot_hardware.stm32.messages import (
    Command, MessageType, MotionFault, MotionFaultReason,
    PoseGoalState, PoseGoalStatus, PoseReached,
)
from robot_hardware.stm32.pose_goal import PoseTransactionState, Stm32PoseGoalController
from robot_hardware.stm32.protocol import Frame
from robot_hardware.stm32.serial_link import SerialLinkError
from robot_runtime.config import RuntimeConfig
from robot_runtime.models import RobotState, SafetyReport
from robot_runtime.state_machine import RobotStateMachine
from robot_simulation.components import build_simulated_components
from tests.test_pose_goal import _FakeLink
from tests.test_state_machine import FakeClock


class RecoverableLink(_FakeLink):
    connected = True
    generation = 1

    def request(self, *args, **kwargs):
        if not self.connected:
            raise SerialLinkError("USB disconnected")
        return super().request(*args, **kwargs)

    def send_command(self, *args, **kwargs):
        if not self.connected:
            raise SerialLinkError("USB disconnected")
        return super().send_command(*args, **kwargs)

    def request_reconnect(self):
        self.connected = False


class CommunicationRecoveryTests(unittest.TestCase):
    def test_full_navigation_resumes_after_long_outage_with_a_new_goal(self):
        clock = FakeClock()
        link = RecoverableLink()
        vision = SimpleNamespace(
            obstacle_source=SimpleNamespace(obstacles=tuple, candidates=tuple),
            road_detector=object(), latest_navigation_result=None,
            is_healthy=lambda *args, **kwargs: True,
        )

        def observe(_pose):
            vision.latest_navigation_result = SimpleNamespace(
                road_observation=SimpleNamespace(observed_at=clock.now, boundary_safe=True),
            )

        vision.observe_navigation = observe
        navigation = build_navigation_runtime(link, vision)
        receiver = navigation.stack.ops9_receiver
        receiver._clock = clock.monotonic
        receiver.attach()
        controller = navigation.stack.chassis
        controller._clock = clock.monotonic
        controller.attach()
        navigation.safety.clock = clock.monotonic

        def pose_frame():
            tick = int(clock.now * 1000) + 1
            payload = b"\x02" + struct.pack("<IH8i", tick, 0, 0, 0, 0, 0, 0, 0, 0, 0)
            for handler in link.handlers[MessageType.TELEMETRY]:
                handler(Frame(MessageType.TELEMETRY, 1, payload))

        components = build_simulated_components("452+321+254+312")
        components.navigator = navigation
        components.motion = controller
        components.safety = navigation.safety
        machine = RobotStateMachine(components, RuntimeConfig(), clock)
        machine.state = RobotState.NAVIGATING_TO_SOURCE
        machine._mission_started = True
        pose_frame()
        machine.tick()
        old_goal = controller.snapshot().goal
        self.assertIsNotNone(old_goal)

        link.connected = False
        machine.tick()
        self.assertEqual(machine.state, RobotState.SAFETY_PAUSED)
        clock.advance(60)
        machine.tick()
        self.assertEqual(machine.state, RobotState.SAFETY_PAUSED)
        self.assertEqual(machine._retry_count, 0)

        link.generation += 1
        link.connected = True
        machine.tick()  # 新会话已就绪，但旧会话的定位不能放行。
        self.assertEqual(machine.state, RobotState.SAFETY_PAUSED)
        for _ in range(2):
            clock.advance(0.21)
            pose_frame()
            machine.tick()
        self.assertEqual(machine.state, RobotState.NAVIGATING_TO_SOURCE)
        goals_before_resume = [item for item in link.requests if item[0] == Command.SET_POSE_GOAL_WITH_LIMITS]
        self.assertEqual(len(goals_before_resume), 1)
        clock.advance(0.01)
        pose_frame()
        machine.tick()
        new_goal = controller.snapshot().goal
        self.assertNotEqual(old_goal.goal_id, new_goal.goal_id)
        link.dispatch_event(PoseReached(old_goal.goal_id, 0, 0, 0, 0, 0).encode_event())
        self.assertEqual(controller.snapshot().goal, new_goal)
        self.assertEqual(controller.snapshot().state, PoseTransactionState.ACCEPTED)

    def test_reconnect_waits_until_controller_confirms_old_goal_stopped(self):
        link = RecoverableLink()
        controller = Stm32PoseGoalController(link)
        goal_id = controller.submit(100, 200, 0, timeout_seconds=35)
        link.generation += 1
        link.query_status = PoseGoalStatus(goal_id, PoseGoalState.MOVING, 0, 0, 0)
        self.assertFalse(controller.recover_connection())
        self.assertEqual(link.commands[-1][0], Command.STOP_ALL)
        link.query_status = PoseGoalStatus(goal_id, PoseGoalState.CANCELLED, 0, 0, 0)
        self.assertTrue(controller.recover_connection())
        self.assertEqual(controller.snapshot().state, PoseTransactionState.CANCELLED)

    def test_reconnect_does_not_clear_can_fault(self):
        link = RecoverableLink()
        controller = Stm32PoseGoalController(link)
        controller.attach()
        goal_id = controller.submit(100, 200, 0, timeout_seconds=35)
        link.dispatch_event(MotionFault(goal_id, MotionFaultReason.CAN_FAULT).encode_event())
        link.generation += 1
        self.assertTrue(controller.recover_connection())
        self.assertEqual(controller.snapshot().fault_reason, MotionFaultReason.CAN_FAULT)

    def test_query_fault_is_preserved_and_host_fault_requests_new_handshake(self):
        link = RecoverableLink()
        controller = Stm32PoseGoalController(link)
        controller.attach()
        goal_id = controller.submit(100, 200, 0, timeout_seconds=35)
        link.dispatch_event(MotionFault(goal_id, MotionFaultReason.HOST_LOST).encode_event())
        self.assertFalse(controller.recover_connection())
        self.assertFalse(link.connected)
        link.generation += 1
        link.connected = True
        link.query_status = PoseGoalStatus(
            goal_id, PoseGoalState.FAULT, 0, 0, 0, MotionFaultReason.CAN_FAULT,
        )
        self.assertTrue(controller.recover_connection())
        self.assertEqual(controller.snapshot().fault_reason, MotionFaultReason.CAN_FAULT)

    def test_disconnect_during_goal_request_is_a_host_fault(self):
        link = RecoverableLink()
        controller = Stm32PoseGoalController(link)

        def disconnect(*_args, **_kwargs):
            link.connected = False
            raise SerialLinkError("USB disconnected during request")

        with patch.object(link, "request", side_effect=disconnect):
            with self.assertRaises(SerialLinkError):
                controller.submit(100, 200, 0, timeout_seconds=35)
        self.assertEqual(controller.snapshot().fault_reason, MotionFaultReason.HOST_LOST)

    def test_failed_stop_during_link_wait_does_not_end_mission(self):
        components = build_simulated_components("452+321+254+312")
        clock = FakeClock()
        machine = RobotStateMachine(components, RuntimeConfig(), clock)
        machine.state = RobotState.READING_TASK_CODE
        machine._mission_started = True
        machine._task_code_deadline = 8
        components.safety.report = SafetyReport(
            False, "USB disconnected", recoverable=True, waiting_for_link=True,
        )
        with patch.object(components.motion, "stop", side_effect=SerialLinkError("offline")):
            with self.assertLogs("robot_runtime.state_machine", level="ERROR"):
                machine.tick()
        clock.advance(60)
        machine.tick()
        self.assertEqual(machine.state, RobotState.SAFETY_PAUSED)
        for _ in range(2):
            clock.advance(0.21)
            components.safety.report = SafetyReport(observation_timestamp=clock.now)
            machine.tick()
        self.assertEqual(machine.state, RobotState.READING_TASK_CODE)
        self.assertGreater(machine._task_code_deadline, clock.now)

    def test_noncommunication_stop_failure_is_not_hidden_by_link_wait(self):
        components = build_simulated_components("452+321+254+312")
        machine = RobotStateMachine(components, RuntimeConfig(), FakeClock())
        machine.state = RobotState.WAITING_FOR_START
        components.safety.report = SafetyReport(
            False, "USB disconnected", recoverable=True, waiting_for_link=True,
        )
        with patch.object(components.manipulator, "stop", side_effect=ValueError("driver bug")):
            with self.assertLogs("robot_runtime.state_machine", level="ERROR"):
                machine.tick()
        self.assertEqual(machine.state, RobotState.SAFE_STOP)

    def test_disconnect_at_action_timeout_does_not_exhaust_retry_budget(self):
        components = build_simulated_components("452+321+254+312")
        machine = RobotStateMachine(components, RuntimeConfig(max_action_retries=0), FakeClock())
        machine.state = RobotState.NAVIGATING_TO_SOURCE
        components.safety.report = SafetyReport(
            False, "USB disconnected", recoverable=True, waiting_for_link=True,
        )
        machine._begin_recovery("航点调用期间动作超时并断线")
        self.assertEqual(machine.state, RobotState.SAFETY_PAUSED)
        self.assertEqual(machine._retry_count, 0)


if __name__ == "__main__":
    unittest.main()
