"""永久停车降级后的故障注入：核对旧目标、定位解锁及任务续跑。"""

import struct
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from robot_control.navigation_runtime import build_navigation_runtime
from robot_hardware.stm32.messages import (
    Command, MessageType, MotionFault, MotionFaultReason, PoseGoalState,
    PoseGoalStatus, PoseReached, Response, ResponseStatus,
)
from robot_hardware.stm32.pose_goal import PoseGoalBusy, PoseTransactionState, Stm32PoseGoalController
from robot_hardware.stm32.serial_link import CommandRejected, CommandTimeout
from robot_mission.task_code import parse_task_code
from robot_runtime.config import RuntimeConfig
from robot_runtime.models import ActionResult, ActionStatus, RobotState
from robot_runtime.state_machine import RobotStateMachine
from robot_simulation.components import build_simulated_components
from tests.test_communication_recovery import RecoverableLink
from tests.test_state_machine import FakeClock
from robot_hardware.stm32.protocol import Frame


class RequestReconciliationTests(unittest.TestCase):
    def setUp(self):
        self.link = RecoverableLink()
        self.clock = FakeClock()
        self.controller = Stm32PoseGoalController(self.link, clock=self.clock.monotonic)
        self.controller.attach()

    def uncertain_submit(self):
        with patch.object(self.link, "request", side_effect=CommandTimeout("ACK lost")):
            goal_id = self.controller.submit(100, 200, 0, timeout_seconds=35)
        self.assertEqual(self.controller.snapshot().state, PoseTransactionState.RECONCILING)
        return goal_id

    def test_lost_ack_adopts_matching_moving_goal_without_resend(self):
        goal_id = self.uncertain_submit()
        self.link.query_status = PoseGoalStatus(goal_id, PoseGoalState.MOVING, 0, 0, 0)
        self.assertTrue(self.controller.recover_connection())
        self.assertEqual(self.controller.snapshot().state, PoseTransactionState.MOVING)
        self.assertEqual([request[0] for request in self.link.requests], [Command.QUERY_POSE_GOAL])
        self.assertFalse(self.link.commands)
        with self.assertRaises(PoseGoalBusy):
            self.controller.submit(1, 1, 0, timeout_seconds=35)

    def test_lost_ack_query_can_confirm_reached(self):
        goal_id = self.uncertain_submit()
        self.link.query_status = PoseGoalStatus(goal_id, PoseGoalState.REACHED, 100, 200, 0)
        self.assertTrue(self.controller.recover_connection())
        self.assertEqual(self.controller.snapshot().state, PoseTransactionState.REACHED)
        self.controller.clear_terminal()
        self.link.dispatch_event(PoseReached(goal_id, 100, 200, 0, 0, 0).encode_event())
        self.assertEqual(self.controller.snapshot().state, PoseTransactionState.IDLE)

    def test_query_timeout_and_other_goal_cannot_release_pending_request(self):
        goal_id = self.uncertain_submit()
        with patch.object(self.link, "request", side_effect=CommandTimeout("query lost")):
            for _ in range(3):
                self.assertFalse(self.controller.recover_connection())
                self.controller.clear_terminal()
                with self.assertRaises(PoseGoalBusy):
                    self.controller.submit(1, 1, 0, timeout_seconds=35)
        self.link.query_status = PoseGoalStatus(goal_id + 1, PoseGoalState.MOVING, 0, 0, 0)
        self.assertFalse(self.controller.recover_connection())
        self.assertEqual(self.link.commands[-1][0], Command.STOP_ALL)
        self.link.query_status = PoseGoalStatus(goal_id + 1, PoseGoalState.CANCELLED, 0, 0, 0)
        self.assertFalse(self.controller.recover_connection())
        self.link.query_status = PoseGoalStatus(0, PoseGoalState.IDLE, 0, 0, 0)
        self.assertTrue(self.controller.recover_connection())
        self.assertEqual(self.controller.snapshot().state, PoseTransactionState.CANCELLED)

    def test_missing_cancel_event_queries_stopped_before_new_goal(self):
        goal_id = self.controller.submit(100, 200, 0, timeout_seconds=35)
        self.controller.cancel()
        self.clock.advance(1.6)
        self.link.query_status = PoseGoalStatus(goal_id, PoseGoalState.MOVING, 0, 0, 0)
        self.assertFalse(self.controller.recover_connection())
        with self.assertRaises(PoseGoalBusy):
            self.controller.submit(1, 1, 0, timeout_seconds=35)
        self.link.query_status = PoseGoalStatus(goal_id, PoseGoalState.CANCELLED, 0, 0, 0)
        self.assertTrue(self.controller.recover_connection())
        new_goal = self.controller.submit(1, 1, 0, timeout_seconds=35)
        self.assertNotEqual(new_goal, goal_id)
        self.link.dispatch_event(PoseReached(goal_id, 100, 200, 0, 0, 0).encode_event())
        self.assertEqual(self.controller.snapshot().goal.goal_id, new_goal)

    def test_cancel_ack_timeout_never_adopts_moving_goal(self):
        goal_id = self.controller.submit(100, 200, 0, timeout_seconds=35)
        with patch.object(self.link, "request", side_effect=CommandTimeout("cancel ACK lost")):
            self.assertTrue(self.controller.cancel())
        self.link.query_status = PoseGoalStatus(goal_id, PoseGoalState.MOVING, 0, 0, 0)
        self.assertFalse(self.controller.recover_connection())
        self.assertEqual(self.controller.snapshot().state, PoseTransactionState.RECONCILING)

    def test_busy_response_is_reconciled_but_explicit_rejection_is_hard(self):
        busy = CommandRejected(Response(1, Command.SET_POSE_GOAL_WITH_LIMITS, ResponseStatus.BUSY))
        with patch.object(self.link, "request", side_effect=busy):
            goal_id = self.controller.submit(100, 200, 0, timeout_seconds=35)
        self.link.query_status = PoseGoalStatus(goal_id, PoseGoalState.ACCEPTED, 0, 0, 0)
        self.assertTrue(self.controller.recover_connection())
        self.assertEqual(self.controller.snapshot().state, PoseTransactionState.ACCEPTED)
        self.controller.stop()
        self.clock.advance(1.6)
        self.link.query_status = PoseGoalStatus(goal_id, PoseGoalState.CANCELLED, 0, 0, 0)
        self.controller.recover_connection()
        rejected = CommandRejected(Response(1, Command.SET_POSE_GOAL_WITH_LIMITS, ResponseStatus.INVALID_ARGUMENT))
        with patch.object(self.link, "request", side_effect=rejected):
            with self.assertRaises(CommandRejected):
                self.controller.submit(1, 1, 0, timeout_seconds=35)
        self.assertEqual(self.controller.snapshot().fault_reason, MotionFaultReason.INTERNAL_ERROR)

    def test_hard_fault_received_during_ack_timeout_is_preserved(self):
        def fail_after_fault(*_args, **_kwargs):
            goal_id = self.controller.snapshot().goal.goal_id
            self.link.dispatch_event(MotionFault(goal_id, MotionFaultReason.CAN_FAULT).encode_event())
            raise CommandTimeout("ACK lost after fault")
        with patch.object(self.link, "request", side_effect=fail_after_fault):
            self.controller.submit(100, 200, 0, timeout_seconds=35)
        self.assertEqual(self.controller.snapshot().state, PoseTransactionState.FAULT)
        self.assertEqual(self.controller.snapshot().fault_reason, MotionFaultReason.CAN_FAULT)


class LocalizationRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.link = RecoverableLink()
        vision = SimpleNamespace(
            obstacle_source=SimpleNamespace(obstacles=tuple, candidates=tuple),
            road_detector=object(), latest_navigation_result=None,
            is_healthy=lambda *args, **kwargs: True,
        )
        def observe(_pose):
            vision.latest_navigation_result = SimpleNamespace(
                road_observation=SimpleNamespace(observed_at=self.clock.now, boundary_safe=True),
            )
        vision.observe_navigation = observe
        self.navigation = build_navigation_runtime(self.link, vision)
        self.receiver = self.navigation.stack.ops9_receiver
        self.receiver._clock = self.clock.monotonic
        self.receiver.attach()
        self.controller = self.navigation.stack.chassis
        self.controller._clock = self.clock.monotonic
        self.controller.attach()
        self.navigation.safety.clock = self.clock.monotonic
        components = build_simulated_components("452+321+254+312")
        components.navigator = self.navigation
        components.motion = self.controller
        components.safety = self.navigation.safety
        self.machine = RobotStateMachine(components, RuntimeConfig(), self.clock)
        self.machine.state = RobotState.NAVIGATING_TO_SOURCE
        self.machine._mission_started = True
        self.emit_pose()
        self.machine.tick()
        self.goal_id = self.controller.snapshot().goal.goal_id

    def emit_pose(self, tick=None, x=0):
        tick = int(self.clock.now * 1000) + 1 if tick is None else tick
        payload = b"\x02" + struct.pack("<IH8i", tick, 0, x, 0, 0, 0, 0, 0, 0, 0)
        for handler in self.link.handlers[MessageType.TELEMETRY]:
            handler(Frame(MessageType.TELEMETRY, 1, payload))

    def fault(self, reason):
        self.link.dispatch_event(MotionFault(self.goal_id, reason).encode_event())
        self.link.query_status = PoseGoalStatus(self.goal_id, PoseGoalState.FAULT, 0, 0, 0, reason)

    def test_ops9_loss_and_waypoint_timeout_resume_same_task_with_new_goal(self):
        for reason in (MotionFaultReason.OPS9_LOST, MotionFaultReason.TIMEOUT):
            with self.subTest(reason=reason):
                self.setUp()
                self.fault(reason)
                self.link.query_status = PoseGoalStatus(self.goal_id, PoseGoalState.MOVING, 0, 0, 0)
                self.machine.tick()
                self.assertEqual(self.machine.state, RobotState.SAFETY_PAUSED)
                self.assertIsNone(self.receiver.latest())
                self.link.query_status = PoseGoalStatus(self.goal_id, PoseGoalState.FAULT, 0, 0, 0, reason)
                self.clock.advance(30)
                self.machine.tick()
                self.assertEqual(self.machine.state, RobotState.SAFETY_PAUSED)
                for _ in range(2):
                    self.clock.advance(0.05)
                    self.emit_pose()
                    self.machine.tick()
                self.assertIsNone(self.receiver.motion_fault)
                self.assertEqual(self.machine.state, RobotState.SAFETY_PAUSED)
                self.clock.advance(0.21)
                self.emit_pose()
                self.machine.tick()
                self.assertEqual(self.machine.state, RobotState.NAVIGATING_TO_SOURCE)
                self.assertEqual(self.machine._retry_count, 0)
                self.assertEqual(sum(r[0] == Command.SET_POSE_GOAL_WITH_LIMITS for r in self.link.requests), 1)
                self.machine.tick()
                self.assertNotEqual(self.controller.snapshot().goal.goal_id, self.goal_id)
                self.link.dispatch_event(PoseReached(self.goal_id, 0, 0, 0, 0, 0).encode_event())
                self.assertEqual(self.controller.snapshot().state, PoseTransactionState.ACCEPTED)

    def test_duplicate_invalid_jumping_and_stale_samples_cannot_release_fault(self):
        self.fault(MotionFaultReason.OPS9_LOST)
        self.emit_pose(tick=10)
        self.emit_pose(tick=10, x=100)  # 同一时间戳不能改变有效位姿，也不能重复计数。
        self.assertFalse(self.navigation.stack.recover_navigation())
        self.emit_pose(tick=11, x=1000)
        self.emit_pose(tick=12)
        self.assertFalse(self.navigation.stack.recover_navigation())
        self.emit_pose(tick=11)  # 逆序帧打断连续确认。
        self.emit_pose(tick=13)
        self.assertFalse(self.navigation.stack.recover_navigation())
        self.emit_pose(tick=14)
        self.clock.advance(0.3)
        self.assertFalse(self.navigation.stack.recover_navigation())
        self.emit_pose(tick=15)
        self.assertFalse(self.navigation.stack.recover_navigation())
        self.emit_pose(tick=16)
        self.assertTrue(self.navigation.stack.recover_navigation())
        self.assertEqual(self.receiver.latest().x_mm, 0)

    def test_fresh_pose_without_query_confirmation_or_matching_goal_stays_locked(self):
        self.fault(MotionFaultReason.TIMEOUT)
        self.emit_pose(tick=10)
        self.emit_pose(tick=11)
        with patch.object(self.link, "request", side_effect=CommandTimeout("query unavailable")):
            self.assertFalse(self.navigation.stack.recover_navigation())
        self.link.query_status = PoseGoalStatus(self.goal_id + 1, PoseGoalState.CANCELLED, 0, 0, 0)
        self.assertFalse(self.navigation.stack.recover_navigation())
        self.assertFalse(self.receiver.recover_fault(self.goal_id + 1, MotionFaultReason.TIMEOUT))
        self.assertIsNone(self.receiver.latest())

    def test_hard_and_unknown_faults_remain_terminal(self):
        for reason in (MotionFaultReason.CAN_FAULT, MotionFaultReason.OUT_OF_BOUNDS,
                       MotionFaultReason.INTERNAL_ERROR, 0xABCD):
            with self.subTest(reason=reason):
                self.setUp()
                self.fault(reason)
                self.emit_pose(tick=10)
                self.emit_pose(tick=11)
                self.machine.tick()
                self.assertEqual(self.machine.state, RobotState.SAFE_STOP)
                self.assertFalse(self.receiver.recover_fault(self.goal_id, reason))

    def test_hard_receiver_fault_survives_reconnect_and_later_sensor_fault(self):
        self.fault(MotionFaultReason.CAN_FAULT)
        self.link.dispatch_event(MotionFault(self.goal_id, MotionFaultReason.OPS9_LOST).encode_event())
        self.link.generation += 1
        self.emit_pose(tick=1)
        self.assertEqual(self.receiver.motion_fault.reason, MotionFaultReason.CAN_FAULT)
        self.machine.tick()
        self.assertEqual(self.machine.state, RobotState.SAFE_STOP)

    def test_direct_navigation_stack_uses_the_same_recovery_gates(self):
        self.fault(MotionFaultReason.OPS9_LOST)
        self.emit_pose(tick=10)
        result = self.navigation.stack.navigate_to(self.navigation.stack.navigator._target)
        self.assertEqual(result.status, ActionStatus.RUNNING)
        self.assertIsNone(self.receiver.latest())
        self.emit_pose(tick=11)
        self.navigation.stack.navigate_to(self.navigation.stack.navigator._target)
        self.assertIsNone(self.receiver.motion_fault)
        self.assertNotEqual(self.controller.snapshot().goal.goal_id, self.goal_id)

    def test_unmatched_soft_fault_reconnects_once_then_resumes(self):
        self.link.dispatch_event(MotionFault(self.goal_id + 1, MotionFaultReason.OPS9_LOST).encode_event())
        with patch.object(self.link, "request_reconnect", wraps=self.link.request_reconnect) as reconnect:
            self.assertFalse(self.navigation.stack.recover_navigation())
            self.assertFalse(self.navigation.stack.recover_navigation())
            self.assertEqual(reconnect.call_count, 1)
            self.link.generation += 1
            self.link.connected = True
            self.link.query_status = PoseGoalStatus(self.goal_id, PoseGoalState.CANCELLED, 0, 0, 0)
            self.assertFalse(self.navigation.stack.recover_navigation())
            self.assertEqual(reconnect.call_count, 1)
            self.emit_pose(tick=1)
            self.assertTrue(self.navigation.stack.recover_navigation())

    def test_new_session_fault_before_first_pose_is_not_cleared_by_pose(self):
        self.link.generation += 1
        self.fault(MotionFaultReason.OPS9_LOST)
        self.emit_pose(tick=1)
        self.emit_pose(tick=2)
        self.assertIsNotNone(self.receiver.motion_fault)
        self.assertIsNone(self.receiver.latest())
        self.assertTrue(self.navigation.stack.recover_navigation())
        self.assertIsNone(self.receiver.motion_fault)

    def test_malformed_pose_breaks_consecutive_recovery_samples(self):
        self.fault(MotionFaultReason.OPS9_LOST)
        self.emit_pose(tick=10)
        for handler in self.link.handlers[MessageType.TELEMETRY]:
            handler(Frame(MessageType.TELEMETRY, 1, b"\x02broken"))
        self.emit_pose(tick=11)
        self.assertFalse(self.navigation.stack.recover_navigation())
        self.emit_pose(tick=12)
        self.assertTrue(self.navigation.stack.recover_navigation())


class ActionWaitingTests(unittest.TestCase):
    def make_machine(self, state):
        components = build_simulated_components("452+321+254+312")
        clock = FakeClock()
        machine = RobotStateMachine(components, RuntimeConfig(max_action_retries=0), clock)
        machine.state = state
        machine.task = parse_task_code("452+321+254+312")
        machine._mission_started = True
        return machine, components, clock

    def test_material_search_exhaustion_does_not_skip_or_repeat_pickup(self):
        machine, components, clock = self.make_machine(RobotState.LOCATING_MATERIAL)
        locate = components.material_perception.locate_material
        components.material_perception.locate_material = lambda _code: ActionResult.retryable("未找到目标")
        machine.tick()
        self.assertEqual(machine.state, RobotState.ACTION_WAITING)
        self.assertFalse(components.manipulator.events)
        self.assertEqual(machine._pickup_index, 0)
        components.material_perception.locate_material = locate
        clock.advance(1.01)
        machine.tick()
        machine.tick()
        self.assertEqual(machine.state, RobotState.PICKING_MATERIAL)
        machine.tick()
        self.assertEqual(machine._pickup_index, 1)
        self.assertEqual(len(components.manipulator.events), 1)

    def test_nonrepeatable_actions_still_stop_on_exhaustion_or_failed_recovery(self):
        for state in (RobotState.PICKING_MATERIAL, RobotState.PLACING_FOR_PROCESSING,
                      RobotState.PLACING_IN_TEMPORARY_STORAGE, RobotState.STACKING_SECOND_BATCH):
            with self.subTest(state=state):
                machine, components, _clock = self.make_machine(state)
                machine._begin_recovery("机械动作无法确认")
                self.assertEqual(machine.state, RobotState.SAFE_STOP)
                self.assertFalse(components.manipulator.events)
                machine, components, _clock = self.make_machine(state)
                machine.state = RobotState.RECOVERING
                machine._recovery_resume_state = state
                components.recovery.recover = lambda *_args: ActionResult.retryable("状态不确定")
                machine.tick()
                self.assertEqual(machine.state, RobotState.SAFE_STOP)

    def test_stop_failure_in_recovery_remains_terminal(self):
        machine, components, _clock = self.make_machine(RobotState.NAVIGATING_TO_SOURCE)
        machine.config = RuntimeConfig(max_action_retries=2)
        with patch.object(components.motion, "stop", side_effect=ValueError("driver bug")):
            with self.assertLogs("robot_runtime.state_machine", level="ERROR"):
                machine._begin_recovery("导航失败")
        self.assertEqual(machine.state, RobotState.SAFE_STOP)

    def test_action_waiting_does_not_resume_while_physical_motion_continues(self):
        machine, components, clock = self.make_machine(RobotState.LOCATING_MATERIAL)
        machine._begin_recovery("定位失败")
        clock.advance(2)
        with patch.object(components.motion, "is_active", return_value=True):
            machine.tick()
        self.assertEqual(machine.state, RobotState.ACTION_WAITING)
        self.assertFalse(components.manipulator.events)
        machine.tick()
        self.assertEqual(machine.state, RobotState.LOCATING_MATERIAL)

    def test_explicit_fatal_recovery_error_is_not_downgraded(self):
        machine, components, _clock = self.make_machine(RobotState.RECOVERING)
        machine._recovery_resume_state = RobotState.NAVIGATING_TO_SOURCE
        components.recovery.recover = lambda *_args: ActionResult.fatal("控制器异常")
        machine.tick()
        self.assertEqual(machine.state, RobotState.SAFE_STOP)


if __name__ == "__main__":
    unittest.main()
