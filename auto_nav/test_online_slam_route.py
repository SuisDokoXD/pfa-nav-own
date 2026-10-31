import json
import signal
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from action_msgs.msg import GoalStatus
from online_slam_goal import (
    ROUTE_FORMAT,
    ROUTE_FORMAT_V1,
    STEP_WAIT,
    STEP_WAYPOINT,
    OnlineSlamGoal,
    parse_route_data,
)
from online_slam_goal_gui import GoalLauncher, build_route_payload


class FakeTime:
    """Minimal stand-in for rclpy.time.Time inside pure-logic tests."""

    def __init__(self, value):
        self.value = float(value)

    def __add__(self, duration):
        return FakeTime(self.value + duration.nanoseconds / 1e9)

    def __lt__(self, other):
        return self.value < other.value

    def __ge__(self, other):
        return self.value >= other.value

    def __sub__(self, other):
        return SimpleNamespace(nanoseconds=int((self.value - other.value) * 1e9))


def make_node(steps, now=100.0):
    node = OnlineSlamGoal.__new__(OnlineSlamGoal)
    node.route_steps = steps
    node.route_index = -1
    node.goal_x = None
    node.goal_y = None
    node.goal_yaw = 0.0
    node.goal_sent = False
    node.finished = False
    node.current_goal_name = None
    node.in_dwell = False
    node.dwell_step = None
    node.dwell_until = None
    node.auto_aim_pub = None
    node.auto_aim_topic = "/auto_aim_enable"
    node.auto_aim_allowed = False
    node.auto_aim_subscriber_seen = False
    node.auto_aim_no_sub_warned = False
    node.gate_enabled_at = None
    node.active_goal_uuid = None
    node.active_goal_terminal_handled = False
    node.logs = []
    node.clock_now = now
    node.get_clock = lambda: SimpleNamespace(now=lambda: FakeTime(node.clock_now))
    logger = SimpleNamespace(
        info=lambda message: node.logs.append(message),
        warn=lambda message: node.logs.append(message),
        error=lambda message: node.logs.append(message),
    )
    node.get_logger = lambda: logger
    return node


class RouteFormatTest(unittest.TestCase):
    def write_goal(self, directory, name, x, frame="odom"):
        path = Path(directory) / f"{name}.json"
        path.write_text(
            json.dumps({"frame": frame, "x": x, "y": 1.0, "yaw": 0.0}),
            encoding="utf-8",
        )
        return path

    def test_visual_order_is_preserved_and_duplicates_are_allowed(self):
        with tempfile.TemporaryDirectory() as directory:
            near = self.write_goal(directory, "near", 2.0)
            far = self.write_goal(directory, "far", 9.0)
            payload = build_route_payload(
                "patrol",
                [
                    {"type": "waypoint", "path": near},
                    {"type": "waypoint", "path": far},
                    {"type": "waypoint", "path": near},
                ],
            )

        self.assertEqual(payload["format"], ROUTE_FORMAT)
        self.assertEqual(
            [step["name"] for step in payload["steps"]], ["near", "far", "near"]
        )
        frame, steps = parse_route_data(payload)
        self.assertEqual(frame, "odom")
        self.assertEqual([step["x"] for step in steps], [2.0, 9.0, 2.0])

    def test_wait_block_keeps_position_between_waypoints(self):
        with tempfile.TemporaryDirectory() as directory:
            a1 = self.write_goal(directory, "a1", 1.0)
            a4 = self.write_goal(directory, "a4", 4.0)
            payload = build_route_payload(
                "patrol",
                [
                    {"type": "waypoint", "path": a1},
                    {"type": "waypoint", "path": a4},
                    {"type": "wait", "seconds": 15.0, "auto_aim": True},
                    {"type": "waypoint", "path": a1},
                ],
            )

        frame, steps = parse_route_data(payload)
        self.assertEqual(
            [step["type"] for step in steps],
            [STEP_WAYPOINT, STEP_WAYPOINT, STEP_WAIT, STEP_WAYPOINT],
        )
        self.assertEqual(steps[2]["seconds"], 15.0)
        self.assertTrue(steps[2]["auto_aim"])

    def test_v1_route_still_loads_as_waypoint_steps(self):
        frame, steps = parse_route_data(
            {
                "format": ROUTE_FORMAT_V1,
                "frame": "odom",
                "waypoints": [{"name": "a1", "x": 1.0, "y": 2.0}],
            }
        )
        self.assertEqual(frame, "odom")
        self.assertEqual(
            steps,
            [{"type": STEP_WAYPOINT, "name": "a1", "x": 1.0, "y": 2.0, "yaw": 0.0}],
        )

    def test_wait_values_are_validated(self):
        for seconds in (0, -1, float("nan"), float("inf"), 86401):
            with self.subTest(seconds=seconds), self.assertRaises(ValueError):
                parse_route_data(
                    {
                        "format": ROUTE_FORMAT,
                        "frame": "odom",
                        "steps": [{"type": "wait", "seconds": seconds}],
                    }
                )

    def test_route_without_waypoint_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "至少需要一个导航点"):
            parse_route_data(
                {
                    "format": ROUTE_FORMAT,
                    "frame": "odom",
                    "steps": [{"type": "wait", "seconds": 5.0}],
                }
            )

    def test_mixed_coordinate_frames_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            odom_goal = self.write_goal(directory, "odom_goal", 2.0, "odom")
            map_goal = self.write_goal(directory, "map_goal", 3.0, "map")
            with self.assertRaisesRegex(ValueError, "不能混用"):
                build_route_payload(
                    "invalid",
                    [
                        {"type": "waypoint", "path": odom_goal},
                        {"type": "waypoint", "path": map_goal},
                    ],
                )

    def test_unknown_route_format_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "format"):
            parse_route_data(
                {
                    "format": "unknown/v1",
                    "frame": "odom",
                    "waypoints": [{"x": 1.0, "y": 2.0}],
                }
            )


class RouteExecutionTest(unittest.TestCase):
    STEP_A1 = {"type": STEP_WAYPOINT, "name": "a1", "x": 1.0, "y": 1.0, "yaw": 0.0}
    STEP_A4 = {"type": STEP_WAYPOINT, "name": "a4", "x": 4.0, "y": 4.0, "yaw": 0.0}

    def steps_with_wait(self, auto_aim=True, seconds=15.0):
        return [
            dict(self.STEP_A1),
            dict(self.STEP_A4),
            {
                "type": STEP_WAIT,
                "name": f"等待 {seconds:g} 秒",
                "seconds": seconds,
                "auto_aim": auto_aim,
            },
            dict(self.STEP_A1),
        ]

    def test_wait_block_dwells_then_continues(self):
        node = make_node(self.steps_with_wait())
        node.advance_route()
        self.assertEqual(node.current_goal_name, "a1")

        node.advance_route()
        self.assertEqual(node.current_goal_name, "a4")

        node.advance_route()
        self.assertTrue(node.in_dwell)
        self.assertTrue(node.auto_aim_allowed)
        self.assertEqual(node.route_index, 2)

        node.check_dwell()
        self.assertTrue(node.in_dwell, "dwell must not end before seconds elapse")

        node.clock_now = 116.0
        node.check_dwell()
        self.assertFalse(node.in_dwell)
        self.assertFalse(node.auto_aim_allowed)
        self.assertEqual(node.current_goal_name, "a1")
        self.assertFalse(node.finished)

    def test_wait_without_auto_aim_never_opens_gate(self):
        node = make_node(self.steps_with_wait(auto_aim=False))
        node.advance_route()
        node.advance_route()
        node.advance_route()
        self.assertTrue(node.in_dwell)
        self.assertFalse(node.auto_aim_allowed)

    def test_route_finishes_after_last_step(self):
        node = make_node([dict(self.STEP_A1)])
        node.advance_route()
        self.assertFalse(node.finished)
        node.advance_route()
        self.assertTrue(node.finished)
        self.assertFalse(node.auto_aim_allowed)
        self.assertTrue(any("全部完成" in line for line in node.logs))

    def test_success_advances_to_next_step(self):
        node = make_node([dict(self.STEP_A1), dict(self.STEP_A4)])
        node.active_goal_uuid = b"goal"
        node.active_goal_terminal_handled = False
        node.advance_route()
        node.goal_sent = True

        node.complete_active_goal(GoalStatus.STATUS_SUCCEEDED, "result")

        self.assertEqual(node.current_goal_name, "a4")
        self.assertEqual(
            (node.goal_x, node.goal_y, node.goal_yaw), (4.0, 4.0, 0.0)
        )
        self.assertFalse(node.goal_sent)
        self.assertFalse(node.finished)

    def test_failed_goal_stops_route_and_closes_gate(self):
        node = make_node(self.steps_with_wait())
        node.active_goal_uuid = b"goal"
        node.active_goal_terminal_handled = False
        node.advance_route()

        node.complete_active_goal(GoalStatus.STATUS_ABORTED, "result")

        self.assertTrue(node.finished)
        self.assertFalse(node.auto_aim_allowed)
        self.assertTrue(any("不会跳过失败点" in line for line in node.logs))


class GateWarningTest(unittest.TestCase):
    def test_warns_when_gate_has_no_subscriber(self):
        node = make_node([{"type": STEP_WAYPOINT, "name": "a1", "x": 0.0, "y": 0.0, "yaw": 0.0}])
        node.auto_aim_pub = Mock()
        node.auto_aim_pub.get_subscription_count.return_value = 0
        node.gate_enabled_at = FakeTime(100.0)
        node.clock_now = 103.0

        node.publish_auto_aim()

        self.assertTrue(node.auto_aim_no_sub_warned)
        self.assertTrue(any("没有任何订阅者" in line for line in node.logs))

    def test_no_warning_before_grace_period(self):
        node = make_node([{"type": STEP_WAYPOINT, "name": "a1", "x": 0.0, "y": 0.0, "yaw": 0.0}])
        node.auto_aim_pub = Mock()
        node.auto_aim_pub.get_subscription_count.return_value = 0
        node.gate_enabled_at = FakeTime(100.0)
        node.clock_now = 101.0

        node.publish_auto_aim()

        self.assertFalse(node.auto_aim_no_sub_warned)

    def test_no_warning_when_subscriber_present(self):
        node = make_node([{"type": STEP_WAYPOINT, "name": "a1", "x": 0.0, "y": 0.0, "yaw": 0.0}])
        node.auto_aim_pub = Mock()
        node.auto_aim_pub.get_subscription_count.return_value = 1
        node.gate_enabled_at = FakeTime(100.0)
        node.clock_now = 200.0

        node.publish_auto_aim()

        self.assertTrue(node.auto_aim_subscriber_seen)
        self.assertFalse(node.auto_aim_no_sub_warned)


class ShutdownGateTest(unittest.TestCase):
    def test_shutdown_publishes_false_while_context_alive(self):
        node = make_node([])
        node.auto_aim_pub = Mock()

        with patch("online_slam_goal.rclpy.ok", return_value=True):
            node.shutdown_auto_aim()

        self.assertFalse(node.auto_aim_allowed)
        self.assertGreaterEqual(node.auto_aim_pub.publish.call_count, 1)
        for call in node.auto_aim_pub.publish.call_args_list:
            self.assertFalse(call.args[0].data)

    def test_shutdown_never_raises_when_context_already_down(self):
        node = make_node([])
        node.auto_aim_pub = Mock()

        with patch("online_slam_goal.rclpy.ok", return_value=False):
            node.shutdown_auto_aim()

        node.auto_aim_pub.publish.assert_not_called()

    def test_publish_errors_do_not_escape_shutdown(self):
        node = make_node([])
        node.auto_aim_pub = Mock()
        node.auto_aim_pub.publish.side_effect = RuntimeError("context invalid")

        with patch("online_slam_goal.rclpy.ok", return_value=True):
            node.shutdown_auto_aim()

        self.assertFalse(node.auto_aim_allowed)


class GuiStopTest(unittest.TestCase):
    def test_gui_stop_child_interrupts_and_waits_for_ros_node(self):
        child = Mock()
        # running, still running after SIGINT, then exited
        child.poll.side_effect = [None, None, 0]
        launcher = GoalLauncher.__new__(GoalLauncher)
        launcher.child = child
        launcher.score_child = None
        launcher.score_result_file = None
        launcher.score_report_file = None

        launcher.stop_child()

        child.send_signal.assert_called_once_with(signal.SIGINT)
        child.wait.assert_called_once_with(timeout=3)
        child.terminate.assert_not_called()
        self.assertIsNone(launcher.child)


if __name__ == "__main__":
    unittest.main()
