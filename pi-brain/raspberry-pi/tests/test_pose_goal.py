"""STM32 位姿事务和地图航点适配的无硬件测试。"""

from pathlib import Path
import unittest
import struct
from unittest.mock import patch

from robot_control.navigation_map import CircularObstacle, NavigationMap, Pose2D
from robot_control.navigation_factory import build_stm32_navigation
from robot_control.navigator import NavigationLimits, Ops9MapTransform
from robot_control.pose_navigation import Stm32PoseMapNavigator
from robot_hardware.stm32.messages import (
    Command,
    MessageType,
    MotionFault,
    MotionFaultReason,
    PoseGoal,
    PoseGoalState,
    PoseGoalStatus,
    PoseCancelled,
    PoseReached,
    PoseStarted,
    Response,
    ResponseStatus,
)
from robot_hardware.stm32.pose_goal import (
    PoseTransactionState,
    PoseGoalBusy,
    Stm32PoseGoalController,
)
from robot_hardware.stm32.protocol import Frame
from robot_hardware.stm32.serial_link import SerialLink, SerialLinkError
from robot_runtime.models import ActionStatus, TargetArea


ROOT = Path(__file__).resolve().parents[1]


class _FakeLink:
    def __init__(self):
        self.handlers = {}
        self.requests = []
        self.commands = []
        self.query_status = PoseGoalStatus(0, PoseGoalState.IDLE, 0, 0, 0)

    def add_frame_handler(self, message_type, handler):
        self.handlers.setdefault(int(message_type), []).append(handler)

    def remove_frame_handler(self, message_type, handler):
        self.handlers[int(message_type)].remove(handler)

    def request(self, command, data=b"", timeout=0.5):
        self.requests.append((Command(command), data, timeout))
        response_data = (
            self.query_status.encode_response_data()
            if command == Command.QUERY_POSE_GOAL
            else b""
        )
        return Response(1, int(command), ResponseStatus.OK, response_data)

    def send_command(self, command, data=b""):
        self.commands.append((Command(command), data))
        return len(self.commands)

    def dispatch_event(self, payload):
        frame = Frame(MessageType.EVENT, 9, payload)
        return [handler(frame) for handler in self.handlers[MessageType.EVENT]]


class PoseMessageTests(unittest.TestCase):
    def test_goal_and_status_round_trip(self):
        goal = PoseGoal(7, -120, 450, -1571, 35000)
        status = PoseGoalStatus(
            7,
            PoseGoalState.MOVING,
            -100,
            440,
            -1500,
            MotionFaultReason.UNSPECIFIED,
        )

        self.assertEqual(PoseGoal.decode_command_data(goal.encode_command_data()), goal)
        self.assertEqual(
            PoseGoalStatus.decode_response_data(status.encode_response_data()),
            status,
        )

    def test_transform_inverse_round_trip(self):
        transform = Ops9MapTransform(
            Pose2D(2100, 1800, 3.141593),
            Pose2D(10, -20, 0.2),
        )
        original = Pose2D(123, 456, -1.1)
        restored = transform.invert(transform.apply(original))

        self.assertAlmostEqual(restored.x_mm, original.x_mm)
        self.assertAlmostEqual(restored.y_mm, original.y_mm)
        self.assertAlmostEqual(restored.yaw_rad, original.yaw_rad)


class PoseGoalControllerTests(unittest.TestCase):
    def test_failed_goal_request_stops_once_and_preserves_failure_reason(self):
        for operation in ("submit", "cancel"):
            for stop_fails in (False, True):
                with self.subTest(operation=operation, stop_fails=stop_fails):
                    link = _FakeLink()
                    controller = Stm32PoseGoalController(link)
                    controller.attach()
                    if operation == "cancel":
                        controller.submit(100, 200, 0, timeout_seconds=35)
                    request_error = SerialLinkError("request failed")
                    with patch.object(link, "request", side_effect=request_error) as request:
                        with patch.object(
                            link, "send_command",
                            side_effect=SerialLinkError("stop failed") if stop_fails else None,
                        ) as stop:
                            with self.assertRaises(SerialLinkError) as caught:
                                if operation == "submit":
                                    controller.submit(100, 200, 0, timeout_seconds=35)
                                else:
                                    controller.cancel()
                    self.assertIs(caught.exception, request_error)
                    self.assertEqual(request.call_count, 1)
                    stop.assert_called_once_with(Command.STOP_ALL)
                    snapshot = controller.snapshot()
                    self.assertEqual(snapshot.state, PoseTransactionState.FAULT)
                    self.assertEqual(snapshot.fault_reason, MotionFaultReason.INTERNAL_ERROR)
                    link.dispatch_event(PoseCancelled(snapshot.goal.goal_id).encode_event())
                    self.assertEqual(controller.snapshot(), snapshot)

    def test_reached_event_before_cancel_response_is_not_overwritten(self):
        goal_id = self.controller.submit(100, 200, 0, timeout_seconds=35)
        original_request = self.link.request

        def reply_after_event(command, data=b"", timeout=0.5):
            self.link.dispatch_event(PoseReached(goal_id, 100, 200, 0, 0, 0).encode_event())
            return original_request(command, data, timeout)

        with patch.object(self.link, "request", side_effect=reply_after_event):
            self.assertTrue(self.controller.cancel())
        self.assertEqual(self.controller.snapshot().state, PoseTransactionState.REACHED)
        self.link.dispatch_event(PoseStarted(goal_id).encode_event())
        self.link.dispatch_event(MotionFault(goal_id, MotionFaultReason.CAN_FAULT).encode_event())
        self.assertEqual(self.controller.snapshot().state, PoseTransactionState.REACHED)

    def test_stop_is_followed_by_single_cancel_and_waits_for_matching_event(self):
        goal_id = self.controller.submit(0, 0, 0, timeout_seconds=35)
        self.controller.stop()
        self.assertEqual(self.link.commands[-1][0], Command.STOP_ALL)
        self.assertTrue(self.controller.cancel())
        self.assertFalse(self.controller.cancel())
        self.assertEqual(self.link.requests[-1][0], Command.CANCEL_POSE_GOAL)
        with self.assertRaises(PoseGoalBusy):
            self.controller.submit(1, 1, 0, timeout_seconds=35)
        self.link.dispatch_event(PoseCancelled(goal_id + 1).encode_event())
        self.assertTrue(self.controller.commanded_motion_active)
        self.link.dispatch_event(PoseCancelled(goal_id).encode_event())
        self.assertFalse(self.controller.commanded_motion_active)
        self.controller.clear_terminal()
        self.assertNotEqual(self.controller.submit(1, 1, 0, timeout_seconds=35), goal_id)

    def test_missing_cancel_event_waits_for_query_or_matching_late_event(self):
        now = [1.0]
        controller = Stm32PoseGoalController(self.link, clock=lambda: now[0])
        controller.attach()
        goal_id = controller.submit(0, 0, 0, timeout_seconds=35)
        controller.stop()
        controller.cancel()
        now[0] += 1.6
        self.assertEqual(controller.snapshot().fault_reason, MotionFaultReason.CANCEL_TIMEOUT)
        self.assertEqual(controller.snapshot().state, PoseTransactionState.RECONCILING)
        with self.assertRaises(PoseGoalBusy):
            controller.submit(1, 1, 0, timeout_seconds=35)
        self.link.dispatch_event(PoseCancelled(goal_id).encode_event())
        self.assertEqual(controller.snapshot().state, PoseTransactionState.CANCELLED)

    def setUp(self):
        self.link = _FakeLink()
        self.controller = Stm32PoseGoalController(self.link)
        self.controller.attach()

    def test_accepts_only_matching_goal_events(self):
        goal_id = self.controller.submit(100, 200, 300, timeout_seconds=35)
        command, data, _ = self.link.requests[-1]
        self.assertEqual(command, Command.SET_POSE_GOAL_WITH_LIMITS)
        self.assertEqual(PoseGoal.decode_command_data(data[:20]).goal_id, goal_id)
        self.assertEqual(len(data), 28)
        self.assertEqual(struct.unpack("<ii", data[20:]), (300000, 250000))

        self.link.dispatch_event(PoseStarted(goal_id + 1).encode_event())
        self.assertEqual(self.controller.snapshot().state, PoseTransactionState.ACCEPTED)
        self.assertEqual(self.controller.stale_events, 1)

        self.link.dispatch_event(PoseStarted(goal_id).encode_event())
        self.assertEqual(self.controller.snapshot().state, PoseTransactionState.MOVING)
        reached = PoseReached(goal_id, 101, 199, 300, 2, 1)
        self.link.dispatch_event(reached.encode_event())
        snapshot = self.controller.snapshot()
        self.assertEqual(snapshot.state, PoseTransactionState.REACHED)
        self.assertEqual(snapshot.reached, reached)

    def test_fault_is_terminal_and_query_decodes(self):
        goal_id = self.controller.submit(0, 0, 0, timeout_seconds=1)
        self.link.dispatch_event(
            MotionFault(goal_id, MotionFaultReason.OPS9_LOST).encode_event()
        )
        self.assertEqual(self.controller.snapshot().state, PoseTransactionState.FAULT)
        self.assertEqual(
            self.controller.snapshot().fault_reason,
            MotionFaultReason.OPS9_LOST,
        )

        self.link.query_status = PoseGoalStatus(
            goal_id,
            PoseGoalState.FAULT,
            0,
            0,
            0,
            MotionFaultReason.OPS9_LOST,
        )
        self.assertEqual(self.controller.query_status(), self.link.query_status)

    def test_new_session_invalidates_active_goal(self):
        self.link.generation = 1
        self.controller.submit(0, 0, 0, timeout_seconds=1)
        self.link.generation = 2
        snapshot = self.controller.snapshot()
        self.assertEqual(snapshot.state, PoseTransactionState.FAULT)
        self.assertEqual(snapshot.fault_reason, MotionFaultReason.HOST_LOST)


class PoseMapNavigatorTests(unittest.TestCase):
    def make_active_route(self, obstacles):
        navigation_map = NavigationMap.load(ROOT / "config" / "navigation.json")
        pose = [Pose2D(1200, 2050, -1.5708)]
        link = _FakeLink()
        controller = Stm32PoseGoalController(link, activity_reader=lambda: True)
        controller.attach()
        navigator = Stm32PoseMapNavigator(
            navigation_map, lambda: pose[0], controller,
            Ops9MapTransform(Pose2D(0, 0, 0)), obstacle_reader=lambda: obstacles,
        )
        navigator.navigate_to(TargetArea.PROCESSING)
        return navigator, controller, link, pose

    def test_unrelated_road_changes_do_not_cancel_or_resubmit_goal(self):
        obstacles = []
        navigator, controller, link, _pose = self.make_active_route(obstacles)
        original_goal = controller.snapshot().goal
        obstacles.append(CircularObstacle(1800, 1500, 60))
        navigator.navigate_to(TargetArea.PROCESSING)
        obstacles.clear()
        navigator.navigate_to(TargetArea.PROCESSING)
        self.assertEqual(controller.snapshot().goal, original_goal)
        self.assertEqual(len(link.requests), 1)

    def test_reopened_shortcut_does_not_interrupt_detour(self):
        obstacles = [CircularObstacle(1200, 1500, 60)]
        navigator, _controller, link, _pose = self.make_active_route(obstacles)
        original_plan = navigator.current_plan
        self.assertNotIn("C", original_plan.nodes)
        obstacles.clear()
        navigator.navigate_to(TargetArea.PROCESSING)
        self.assertIs(navigator.current_plan, original_plan)
        self.assertEqual(len(link.requests), 1)

    def test_blocked_remaining_route_cancels_once_then_replans_after_confirmation(self):
        obstacles = []
        navigator, controller, link, _pose = self.make_active_route(obstacles)
        original_goal = controller.snapshot().goal
        obstacles.append(CircularObstacle(1200, 1500, 60))
        for _ in range(3):
            navigator.navigate_to(TargetArea.PROCESSING)
        self.assertEqual([request[0] for request in link.requests], [
            Command.SET_POSE_GOAL_WITH_LIMITS, Command.CANCEL_POSE_GOAL,
        ])
        link.dispatch_event(PoseCancelled(original_goal.goal_id).encode_event())
        navigator.navigate_to(TargetArea.PROCESSING)
        self.assertEqual(link.requests[-1][0], Command.SET_POSE_GOAL_WITH_LIMITS)
        self.assertNotIn("C", navigator.current_plan.nodes)
        self.assertNotEqual(controller.snapshot().goal.goal_id, original_goal.goal_id)

    def test_obstacle_on_passed_segment_does_not_cancel_next_goal(self):
        obstacles = []
        navigator, controller, link, pose = self.make_active_route(obstacles)
        goal = controller.snapshot().goal
        pose[0] = Pose2D(goal.x_mm, goal.y_mm, goal.yaw_mrad / 1000)
        link.dispatch_event(PoseReached(
            goal.goal_id, goal.x_mm, goal.y_mm, goal.yaw_mrad, 0, 0,
        ).encode_event())
        navigator.navigate_to(TargetArea.PROCESSING)
        # SOURCE-T 已走完；与当前 T-C 相隔超过车体包络。
        obstacles.append(CircularObstacle(1200, 2050, 30))
        navigator.navigate_to(TargetArea.PROCESSING)
        self.assertEqual([request[0] for request in link.requests], [
            Command.SET_POSE_GOAL_WITH_LIMITS, Command.SET_POSE_GOAL_WITH_LIMITS,
        ])

    def test_no_route_does_not_burn_retries_and_reopens_without_motion_replay(self):
        navigation_map = NavigationMap.load(ROOT / "config" / "navigation.json")
        obstacles = [CircularObstacle(1200, 1925, 60)]
        link = _FakeLink()
        controller = Stm32PoseGoalController(link)
        navigator = Stm32PoseMapNavigator(
            navigation_map, lambda: Pose2D(1200, 2050, -1.5708), controller,
            Ops9MapTransform(Pose2D(0, 0, 0)), obstacle_reader=lambda: obstacles,
        )
        for _ in range(5):
            result = navigator.navigate_to(TargetArea.PROCESSING)
            self.assertEqual(result.status, ActionStatus.RUNNING)
            self.assertFalse(result.activity)
        self.assertFalse(link.requests)
        obstacles.clear()
        self.assertEqual(navigator.navigate_to(TargetArea.PROCESSING).status, ActionStatus.RUNNING)
        self.assertEqual(link.requests[-1][0], Command.SET_POSE_GOAL_WITH_LIMITS)

    def test_factory_defaults_to_stm32_pose_goal_mode(self):
        stack = build_stm32_navigation(
            SerialLink("not-opened", heartbeat_interval=None)
        )

        self.assertIsInstance(stack.navigator, Stm32PoseMapNavigator)
        self.assertIsInstance(stack.chassis, Stm32PoseGoalController)

    def test_submits_route_nodes_one_at_a_time(self):
        navigation_map = NavigationMap.load(ROOT / "config" / "navigation.json")
        pose = [Pose2D(1200, 2050, -1.5708)]
        link = _FakeLink()
        controller = Stm32PoseGoalController(link, activity_reader=lambda: True)
        controller.attach()
        navigator = Stm32PoseMapNavigator(
            navigation_map,
            lambda: pose[0],
            controller,
            Ops9MapTransform(Pose2D(0, 0, 0), Pose2D(0, 0, 0)),
            limits=NavigationLimits(waypoint_tolerance_mm=90),
        )

        first = navigator.navigate_to(TargetArea.PROCESSING)
        first_goal = PoseGoal.decode_command_data(link.requests[-1][1][:20])
        self.assertEqual(first.status, ActionStatus.RUNNING)
        self.assertEqual((first_goal.x_mm, first_goal.y_mm), (1200, 1800))

        pose[0] = Pose2D(1200, 1800, first_goal.yaw_mrad / 1000.0)
        link.dispatch_event(
            PoseReached(first_goal.goal_id, 1200, 1800, first_goal.yaw_mrad, 0, 0).encode_event()
        )
        second = navigator.navigate_to(TargetArea.PROCESSING)
        second_goal = PoseGoal.decode_command_data(link.requests[-1][1][:20])
        self.assertEqual(second.status, ActionStatus.RUNNING)
        self.assertNotEqual(second_goal.goal_id, first_goal.goal_id)
        self.assertEqual((second_goal.x_mm, second_goal.y_mm), (1200, 1200))


if __name__ == "__main__":
    unittest.main()
