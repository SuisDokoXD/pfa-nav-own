#!/usr/bin/env python3
"""Send one fixed goal after an online SLAM map and TF become available.

This is intentionally separate from auto_nav_su.py so it can be used as a
small simulation test tool without changing the existing waypoint state
machine. The goal can be expressed in the SLAM ``map`` frame or in ``odom``.
"""

import argparse
import json
import math
import signal
import time
import uuid
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener


ROUTE_FORMAT = "pfa_online_slam_route/v2"
ROUTE_FORMAT_V1 = "pfa_online_slam_route/v1"
MAX_WAIT_SECONDS = 86400.0
STEP_WAYPOINT = "waypoint"
STEP_WAIT = "wait"


def quaternion_from_yaw(yaw):
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def _normalize_waypoint(raw, index):
    if not isinstance(raw, dict) or not all(key in raw for key in ("x", "y")):
        raise ValueError(f"路线第 {index} 项必须是包含 x 和 y 的导航点")
    try:
        x = float(raw["x"])
        y = float(raw["y"])
        yaw = float(raw.get("yaw", 0.0))
    except (TypeError, ValueError) as exc:
        raise ValueError(f"路线第 {index} 项坐标不是有效数字") from exc
    if not all(math.isfinite(value) for value in (x, y, yaw)):
        raise ValueError(f"路线第 {index} 项坐标必须是有限数字")
    return {
        "type": STEP_WAYPOINT,
        "name": str(raw.get("name") or f"waypoint_{index}"),
        "x": x,
        "y": y,
        "yaw": yaw,
    }


def _normalize_wait(raw, index):
    seconds = raw.get("seconds")
    if isinstance(seconds, bool) or not isinstance(seconds, (int, float)):
        raise ValueError(f"路线第 {index} 项等待时间必须是数字")
    seconds = float(seconds)
    if not math.isfinite(seconds) or not 0.0 < seconds <= MAX_WAIT_SECONDS:
        raise ValueError(
            f"路线第 {index} 项等待时间须大于 0 且不超过 {MAX_WAIT_SECONDS:g} 秒"
        )
    auto_aim = raw.get("auto_aim", False)
    if not isinstance(auto_aim, bool):
        raise ValueError(f"路线第 {index} 项 auto_aim 必须是布尔值")
    return {
        "type": STEP_WAIT,
        "name": str(raw.get("name") or f"等待 {seconds:g} 秒"),
        "seconds": seconds,
        "auto_aim": auto_aim,
    }


def parse_route_data(data):
    """Validate a route document and normalize it into an ordered step list.

    v1 documents (waypoints only) stay readable and are lifted into waypoint
    steps so older saved routes keep working unchanged.
    """
    if not isinstance(data, dict):
        raise ValueError("路线文件必须是 JSON 对象")
    frame = data.get("frame")
    if frame not in ("map", "odom"):
        raise ValueError("路线 frame 必须是 map 或 odom")
    fmt = data.get("format")
    if fmt == ROUTE_FORMAT_V1 or (
        fmt is None and "steps" not in data and "waypoints" in data
    ):
        raw_waypoints = data.get("waypoints")
        if not isinstance(raw_waypoints, list) or not raw_waypoints:
            raise ValueError("路线 waypoints 必须是非空数组")
        return frame, [
            _normalize_waypoint(raw, index)
            for index, raw in enumerate(raw_waypoints, start=1)
        ]
    if fmt != ROUTE_FORMAT:
        raise ValueError(f"路线 format 必须是 {ROUTE_FORMAT} 或 {ROUTE_FORMAT_V1}")
    raw_steps = data.get("steps")
    if not isinstance(raw_steps, list) or not raw_steps:
        raise ValueError("路线 steps 必须是非空数组")
    steps = []
    for index, raw in enumerate(raw_steps, start=1):
        if not isinstance(raw, dict):
            raise ValueError(f"路线第 {index} 项格式错误")
        kind = raw.get("type")
        if kind == STEP_WAYPOINT:
            steps.append(_normalize_waypoint(raw, index))
        elif kind == STEP_WAIT:
            steps.append(_normalize_wait(raw, index))
        else:
            raise ValueError(f"路线第 {index} 项 type 必须是 waypoint 或 wait")
    if not any(step["type"] == STEP_WAYPOINT for step in steps):
        raise ValueError("路线至少需要一个导航点")
    return frame, steps


class OnlineSlamGoal(Node):
    def __init__(
        self,
        x=None,
        y=None,
        yaw=0.0,
        goal_frame="map",
        map_frame="map",
        base_frame="gimbal_yaw",
        click_topic=None,
        goal_file=None,
        save_goal=True,
        start_mode="immediate",
        game_status_topic="/referee/game_status",
        route_steps=None,
        record_only=False,
        auto_aim_topic="/auto_aim_enable",
        auto_aim_gate=True,
    ):
        super().__init__("online_slam_goal")
        self.goal_x = None if x is None else float(x)
        self.goal_y = None if y is None else float(y)
        self.goal_yaw = float(yaw)
        self.goal_frame = goal_frame
        self.goal_file = Path(goal_file).expanduser() if goal_file else None
        self.save_goal = bool(save_goal)
        self.start_mode = start_mode
        self.start_allowed = start_mode == "immediate"
        self.game_status_topic = game_status_topic
        self.route_steps = list(route_steps or [])
        self.route_index = -1
        self.record_only = bool(record_only)
        self.current_goal_name = None
        self.in_dwell = False
        self.dwell_step = None
        self.dwell_until = None
        self.auto_aim_topic = auto_aim_topic
        self.auto_aim_pub = None
        self.auto_aim_allowed = False
        self.auto_aim_subscriber_seen = False
        self.auto_aim_no_sub_warned = False
        self.gate_enabled_at = None
        self.map_frame = map_frame
        self.base_frame = base_frame
        self.latest_map = None
        self.goal_sent = False
        self.finished = False
        self.active_goal_uuid = None
        self.active_goal_status_seen = False
        self.active_goal_terminal_handled = False
        self.active_goal_sent_at = None
        self.active_goal_wait_warning_sent = False
        self.start_time = self.get_clock().now()

        self.map_sub = self.create_subscription(
            OccupancyGrid, "map", self.map_callback, 10
        )
        self.game_status_sub = None
        self.game_status_type = None
        if self.start_mode == "referee":
            try:
                from pb_rm_interfaces.msg import GameStatus
            except ModuleNotFoundError as exc:
                raise RuntimeError(
                    "正式比赛模式需要 pb_rm_interfaces；请先执行 "
                    "source /opt/ros/humble/setup.bash && "
                    "source install/setup.bash"
                ) from exc
            self.game_status_type = GameStatus
            self.game_status_sub = self.create_subscription(
                GameStatus,
                self.game_status_topic,
                self.game_status_callback,
                10,
            )
            self.get_logger().warn(
                f"比赛启动门控已启用：等待 {self.game_status_topic} "
                "的 game_progress=RUNNING(4)，收到前绝不发送导航目标"
            )
        else:
            self.get_logger().warn("立即出发模式已启用，不等待比赛开始信号")
        self.click_sub = None
        if click_topic:
            self.click_sub = self.create_subscription(
                PointStamped, click_topic, self.click_callback, 10
            )
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.action_status_sub = self.create_subscription(
            GoalStatusArray,
            "navigate_to_pose/_action/status",
            self.action_status_callback,
            10,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.5, self.try_send_goal)
        self.action_watchdog = self.create_timer(1.0, self.check_action_progress)
        self.dwell_timer = self.create_timer(0.1, self.check_dwell)
        gate_steps = [
            step
            for step in self.route_steps
            if step["type"] == STEP_WAIT and step["auto_aim"]
        ]
        if gate_steps and auto_aim_gate:
            self.auto_aim_pub = self.create_publisher(Bool, self.auto_aim_topic, 10)
            self.auto_aim_timer = self.create_timer(0.1, self.publish_auto_aim)
            self.gate_enabled_at = self.get_clock().now()
            self.get_logger().warn(
                f"自瞄门控已启用：路线其余时间持续禁止开火，"
                f"仅在 {len(gate_steps)} 个标记 auto_aim 的等待块期间向 "
                f"{self.auto_aim_topic} 发送 true"
            )
        elif gate_steps:
            self.get_logger().warn(
                "路线包含自瞄等待块，但 --no-auto-aim-gate 已禁用门控发布"
            )

        if self.route_steps:
            self.get_logger().info(
                f"已加载顺序路线，共 {len(self.route_steps)} 个步骤"
            )
            self.advance_route()
        elif self.goal_x is None or self.goal_y is None:
            self.get_logger().info(
                f"等待 RViz 点击目标 ({click_topic})，点击点坐标将按 {self.goal_frame} 解释"
            )
        else:
            self.get_logger().info(
                f"已缓存目标点: ({self.goal_x:.3f}, {self.goal_y:.3f}) "
                f"frame={self.goal_frame}, yaw={self.goal_yaw:.3f} rad; "
                "等待在线 SLAM 地图和 TF"
            )

    def map_callback(self, msg):
        self.latest_map = msg

    def game_status_callback(self, msg):
        if self.start_allowed:
            return
        if msg.game_progress == self.game_status_type.RUNNING:
            self.start_allowed = True
            self.get_logger().warn("已收到比赛 RUNNING 状态，导航目标允许发送")

    def click_callback(self, msg):
        if self.goal_x is not None and self.goal_y is not None:
            return
        self.goal_x = float(msg.point.x)
        self.goal_y = float(msg.point.y)
        if msg.header.frame_id and msg.header.frame_id != self.goal_frame:
            self.get_logger().warn(
                f"点击消息 frame={msg.header.frame_id}，但命令行 goal-frame={self.goal_frame}；"
                "将按命令行指定坐标系处理"
            )
        self.get_logger().info(
            f"已记录 RViz 点击目标: ({self.goal_x:.3f}, {self.goal_y:.3f}) "
            f"frame={self.goal_frame}"
        )
        if self.save_goal and self.goal_file is not None:
            self.save_goal_file()
        if self.record_only:
            self.get_logger().info("仅录点模式完成，不发送导航目标")
            self.finished = True

    def save_goal_file(self):
        data = {
            "frame": self.goal_frame,
            "x": self.goal_x,
            "y": self.goal_y,
            "yaw": self.goal_yaw,
        }
        try:
            self.goal_file.parent.mkdir(parents=True, exist_ok=True)
            self.goal_file.write_text(
                json.dumps(data, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            self.get_logger().info(f"目标点已保存到: {self.goal_file}")
        except OSError as exc:
            self.get_logger().error(f"保存目标点失败 {self.goal_file}: {exc}")

    @staticmethod
    def yaw_from_quaternion(q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def goal_in_map(self):
        """Resolve the fixed goal into map coordinates once map->goal_frame exists."""
        if self.goal_frame == self.map_frame:
            return self.goal_x, self.goal_y, self.goal_yaw
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.goal_frame, Time()
            )
        except TransformException:
            return None
        t = transform.transform.translation
        q = transform.transform.rotation
        source_yaw = self.yaw_from_quaternion(q)
        cos_yaw = math.cos(source_yaw)
        sin_yaw = math.sin(source_yaw)
        map_x = t.x + cos_yaw * self.goal_x - sin_yaw * self.goal_y
        map_y = t.y + sin_yaw * self.goal_x + cos_yaw * self.goal_y
        return map_x, map_y, self.goal_yaw + source_yaw

    def map_contains_goal(self, goal_x, goal_y):
        if self.latest_map is None:
            return False
        info = self.latest_map.info
        if info.resolution <= 0.0 or info.width == 0 or info.height == 0:
            return False
        min_x = info.origin.position.x
        min_y = info.origin.position.y
        max_x = min_x + info.width * info.resolution
        max_y = min_y + info.height * info.resolution
        return min_x <= goal_x < max_x and min_y <= goal_y < max_y

    def tf_ready(self):
        return self.tf_buffer.can_transform(
            self.map_frame,
            self.base_frame,
            Time(),
            timeout=Duration(seconds=0.05),
        )

    def set_auto_aim(self, allowed):
        """Update the auto-aim gate; a no-op when gating is not enabled."""
        allowed = bool(allowed)
        if allowed != self.auto_aim_allowed and self.auto_aim_pub is not None:
            self.get_logger().warn(
                "自瞄门控切换为 " + ("允许开火" if allowed else "禁止开火")
            )
        self.auto_aim_allowed = allowed

    def publish_auto_aim(self):
        if self.auto_aim_pub is None:
            return
        message = Bool()
        message.data = self.auto_aim_allowed
        self.auto_aim_pub.publish(message)
        if self.auto_aim_pub.get_subscription_count() > 0:
            self.auto_aim_subscriber_seen = True
            return
        if self.auto_aim_subscriber_seen or self.auto_aim_no_sub_warned:
            return
        if self.gate_enabled_at is None:
            return
        waited = (self.get_clock().now() - self.gate_enabled_at).nanoseconds / 1e9
        if waited < 2.0:
            return
        self.auto_aim_no_sub_warned = True
        self.get_logger().error(
            f"{self.auto_aim_topic} 目前没有任何订阅者：自瞄可能未启动、"
            "运行的是不含门控的入口，或话题名不一致；本节点仍会持续发布"
        )

    def shutdown_auto_aim(self):
        """Force the gate closed before the node disappears.

        rclpy 的默认信号处理会在 SIGINT/SIGTERM 时直接关闭 context，
        那时再 publish 会抛 RCLError 并且一条都发不出去。所以这里逐次
        检查 rclpy.ok()，任何一步失败就安静退出，绝不让收尾路径崩掉。
        """
        if self.auto_aim_pub is None:
            return
        self.auto_aim_allowed = False
        message = Bool()
        message.data = False
        for _ in range(30):
            if not rclpy.ok():
                return
            try:
                self.auto_aim_pub.publish(message)
            except Exception:  # noqa: BLE001 - 收尾路径不允许抛异常
                return
            time.sleep(0.01)

    def advance_route(self):
        """Step forward; a wait block starts a dwell instead of a goal."""
        self.route_index += 1
        if self.route_index >= len(self.route_steps):
            self.finish_route()
            return
        step = self.route_steps[self.route_index]
        if step["type"] == STEP_WAIT:
            self.begin_dwell(step)
        else:
            self.load_waypoint(step)

    def load_waypoint(self, step):
        self.in_dwell = False
        self.dwell_step = None
        self.dwell_until = None
        self.goal_x = step["x"]
        self.goal_y = step["y"]
        self.goal_yaw = step["yaw"]
        self.current_goal_name = step["name"]
        self.goal_sent = False
        self.start_time = self.get_clock().now()
        self.set_auto_aim(False)
        self.get_logger().info(
            f"切换到路线步骤 {self.route_index + 1}/{len(self.route_steps)}: "
            f"{self.current_goal_name}；若尚未进入地图范围则继续等待地图扩展"
        )

    def begin_dwell(self, step):
        self.in_dwell = True
        self.dwell_step = step
        self.dwell_until = self.get_clock().now() + Duration(seconds=step["seconds"])
        self.set_auto_aim(step["auto_aim"])
        self.get_logger().warn(
            f"路线步骤 {self.route_index + 1}/{len(self.route_steps)} 停留 "
            f"{step['seconds']:g} 秒；"
            + ("该期间允许自瞄开火" if step["auto_aim"] else "该期间保持禁止开火")
        )

    def check_dwell(self):
        if not self.in_dwell or self.finished:
            return
        if self.get_clock().now() < self.dwell_until:
            return
        self.in_dwell = False
        self.dwell_step = None
        self.dwell_until = None
        self.set_auto_aim(False)
        self.get_logger().info("停留结束，继续执行后续路线步骤")
        self.advance_route()

    def finish_route(self):
        self.in_dwell = False
        self.dwell_step = None
        self.dwell_until = None
        self.set_auto_aim(False)
        if self.route_steps:
            self.get_logger().info("顺序巡航路线已全部完成")
        self.finished = True

    def try_send_goal(self):
        if self.goal_sent or self.finished or self.in_dwell:
            return
        if self.goal_x is None or self.goal_y is None:
            return
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if self.latest_map is None:
            if int(elapsed) % 5 == 0:
                self.get_logger().info("等待 /map 发布...")
            return
        goal = self.goal_in_map()
        if goal is None:
            if int(elapsed) % 5 == 0:
                self.get_logger().info(
                    f"等待 TF {self.map_frame}->{self.goal_frame}..."
                )
            return
        goal_x, goal_y, goal_yaw = goal
        if not self.map_contains_goal(goal_x, goal_y):
            if int(elapsed) % 5 == 0:
                self.get_logger().info("目标点尚未落入当前地图范围，继续等待地图扩展...")
            return
        if not self.tf_ready():
            if int(elapsed) % 5 == 0:
                self.get_logger().info(
                    f"等待 TF {self.map_frame}->{self.base_frame}..."
                )
            return
        if not self.nav_client.server_is_ready():
            if int(elapsed) % 5 == 0:
                self.get_logger().info("等待 navigate_to_pose action server...")
            return
        if not self.start_allowed:
            if int(elapsed) % 5 == 0:
                self.get_logger().info(
                    f"导航系统已就绪，仍在等待比赛开始信号: {self.game_status_topic}"
                )
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = goal_x
        goal.pose.pose.position.y = goal_y
        goal.pose.pose.position.z = 0.0
        qx, qy, qz, qw = quaternion_from_yaw(goal_yaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.goal_sent = True
        if self.route_steps:
            progress = f"[{self.route_index + 1}/{len(self.route_steps)}] "
            target_name = self.current_goal_name
        else:
            progress = ""
            target_name = "单点目标"
        self.get_logger().info(
            f"地图、TF 和 action 已就绪，发送 {progress}{target_name}"
        )
        goal_uuid = NavigateToPose.Impl.SendGoalService.Request().goal_id
        goal_uuid.uuid = list(uuid.uuid4().bytes)
        goal_uuid_key = bytes(goal_uuid.uuid)
        self.active_goal_uuid = goal_uuid_key
        self.active_goal_status_seen = False
        self.active_goal_terminal_handled = False
        self.active_goal_sent_at = self.get_clock().now()
        self.active_goal_wait_warning_sent = False
        future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback,
            goal_uuid=goal_uuid,
        )
        future.add_done_callback(
            lambda completed: self.goal_response_callback(completed, goal_uuid_key)
        )

    def feedback_callback(self, feedback_msg):
        distance = feedback_msg.feedback.distance_remaining
        prefix = ""
        if self.route_steps:
            prefix = f"路线点 {self.route_index + 1}/{len(self.route_steps)} "
        self.get_logger().info(f"{prefix}目标剩余距离: {distance:.3f} m")

    def goal_response_callback(self, future, goal_uuid_key):
        if goal_uuid_key != self.active_goal_uuid:
            return
        try:
            goal_handle = future.result()
        except Exception as exc:
            if self.active_goal_status_seen:
                self.get_logger().warn(
                    f"goal response 读取失败，但 Nav2 status 已确认目标执行中；"
                    f"继续等待终态: {exc}"
                )
            else:
                self.get_logger().error(f"发送目标失败: {exc}")
                self.finished = True
            return
        if not goal_handle.accepted:
            self.get_logger().error("Nav2 拒绝了目标")
            self.finished = True
            return
        self.get_logger().info("Nav2 已接受目标，等待结果")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda completed: self.result_callback(completed, goal_uuid_key)
        )

    def action_status_callback(self, msg):
        # The goal response and status streams are independent DDS paths. Keep
        # the route moving when status confirms execution before the response
        # future is delivered, while matching only the active goal UUID.
        goal_uuid_key = self.active_goal_uuid
        if goal_uuid_key is None:
            return
        for item in msg.status_list:
            if bytes(item.goal_info.goal_id.uuid) != goal_uuid_key:
                continue
            status = item.status
            if status in (
                GoalStatus.STATUS_ACCEPTED,
                GoalStatus.STATUS_EXECUTING,
                GoalStatus.STATUS_CANCELING,
            ):
                if not self.active_goal_status_seen:
                    self.active_goal_status_seen = True
                    self.get_logger().warn(
                        "已从 Nav2 action status 确认目标正在执行；"
                        "若 goal response 丢失，将依据状态话题推进路线"
                    )
                return
            if status in (
                GoalStatus.STATUS_SUCCEEDED,
                GoalStatus.STATUS_CANCELED,
                GoalStatus.STATUS_ABORTED,
            ):
                self.complete_active_goal(status, "action status")
                return

    def check_action_progress(self):
        if self.active_goal_uuid is None or self.active_goal_terminal_handled:
            return
        if self.active_goal_status_seen or self.active_goal_wait_warning_sent:
            return
        elapsed = (
            self.get_clock().now() - self.active_goal_sent_at
        ).nanoseconds / 1e9
        if elapsed >= 5.0:
            self.active_goal_wait_warning_sent = True
            self.get_logger().warn(
                f"已等待 {elapsed:.1f}s，尚未收到该目标的 response/status；"
                "不会重复发送目标，继续监听 Nav2 状态"
            )

    def result_callback(self, future, goal_uuid_key):
        if goal_uuid_key != self.active_goal_uuid:
            return
        try:
            wrapped = future.result()
            status = wrapped.status
        except Exception as exc:
            if self.active_goal_status_seen:
                self.get_logger().warn(
                    f"读取导航结果失败，但 Nav2 status 已确认目标执行中；"
                    f"继续等待终态: {exc}"
                )
            else:
                self.get_logger().error(f"读取导航结果失败: {exc}")
                self.finished = True
            return

        self.complete_active_goal(status, "result")

    def complete_active_goal(self, status, source):
        # A terminal status can arrive through both the result future and the
        # status topic; guard the transition so one waypoint is sent once.
        if self.active_goal_uuid is None or self.active_goal_terminal_handled:
            return
        self.active_goal_terminal_handled = True
        self.get_logger().info(f"导航结束，状态码: {status}（来源: {source}）")
        self.active_goal_uuid = None

        if status == GoalStatus.STATUS_SUCCEEDED and self.route_steps:
            self.advance_route()
            return

        if self.route_steps:
            self.get_logger().error(
                f"路线在第 {self.route_index + 1} 个步骤停止，不会跳过失败点"
            )
        self.finish_route()


def main():
    parser = argparse.ArgumentParser(
        description="在线 SLAM 地图就绪后发送一个固定 NavigateToPose 目标"
    )
    parser.add_argument("--x", type=float, help="目标坐标 x；--wait-click 时可省略")
    parser.add_argument("--y", type=float, help="目标坐标 y；--wait-click 时可省略")
    parser.add_argument("--yaw", type=float, default=0.0, help="目标 yaw，单位 rad")
    parser.add_argument(
        "--goal-frame",
        choices=("map", "odom"),
        default="map",
        help="目标点所在坐标系；odom 适合固定初始位置的相对目标",
    )
    parser.add_argument(
        "--wait-click",
        action="store_true",
        help="不指定 x/y，等待 RViz clicked_point 作为目标",
    )
    parser.add_argument(
        "--click-topic",
        default="clicked_point",
        help="RViz Publish Point 话题；命名空间由 --ros-args __ns 决定",
    )
    parser.add_argument(
        "--goal-file",
        default="online_slam_goal.json",
        help="保存/读取固定目标的 JSON 文件，默认当前目录 online_slam_goal.json",
    )
    parser.add_argument(
        "--use-saved-goal",
        action="store_true",
        help="从 --goal-file 自动加载目标，不需要 RViz 点击",
    )
    parser.add_argument(
        "--route-file",
        default="online_slam_route.json",
        help="顺序路线 JSON 文件",
    )
    parser.add_argument(
        "--use-saved-route",
        action="store_true",
        help="从 --route-file 加载路线，并按顺序逐点导航",
    )
    parser.add_argument(
        "--record-only",
        action="store_true",
        help="记录 RViz 点击并保存后退出，不发送导航目标",
    )
    parser.add_argument(
        "--no-save",
        action="store_true",
        help="--wait-click 模式下不保存本次点击目标",
    )
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--base-frame", default="gimbal_yaw")
    parser.add_argument(
        "--start-mode",
        choices=("immediate", "referee"),
        default="immediate",
        help="immediate 立即允许发目标；referee 等待比赛状态 RUNNING",
    )
    parser.add_argument(
        "--game-status-topic",
        default="/referee/game_status",
        help="比赛状态话题，消息类型为 pb_rm_interfaces/msg/GameStatus",
    )
    parser.add_argument(
        "--auto-aim-topic",
        default="/auto_aim_enable",
        help="自瞄开关话题 std_msgs/msg/Bool；true 允许开火，false 禁止",
    )
    parser.add_argument(
        "--no-auto-aim-gate",
        action="store_true",
        help="即使路线含 auto_aim 等待块也不发布自瞄门控",
    )
    args, ros_args = parser.parse_known_args()
    if args.use_saved_goal and args.use_saved_route:
        parser.error("--use-saved-goal 和 --use-saved-route 不能同时使用")

    saved_goal = None
    route_steps = None
    if args.use_saved_goal:
        try:
            saved_goal = json.loads(
                Path(args.goal_file).expanduser().read_text(encoding="utf-8")
            )
        except (OSError, json.JSONDecodeError) as exc:
            parser.error(f"读取目标文件失败 {args.goal_file}: {exc}")
        if saved_goal.get("frame") not in ("map", "odom"):
            parser.error("目标文件 frame 必须是 map 或 odom")
        if not all(key in saved_goal for key in ("x", "y")):
            parser.error("目标文件必须包含 x 和 y")
        args.x = float(saved_goal["x"])
        args.y = float(saved_goal["y"])
        args.yaw = float(saved_goal.get("yaw", 0.0))
        args.goal_frame = saved_goal["frame"]
        args.wait_click = False
    elif args.use_saved_route:
        try:
            route_data = json.loads(
                Path(args.route_file).expanduser().read_text(encoding="utf-8")
            )
            args.goal_frame, route_steps = parse_route_data(route_data)
        except (OSError, json.JSONDecodeError, ValueError) as exc:
            parser.error(f"读取路线文件失败 {args.route_file}: {exc}")
        first = next(
            step for step in route_steps if step["type"] == STEP_WAYPOINT
        )
        args.x = first["x"]
        args.y = first["y"]
        args.yaw = first["yaw"]
        args.wait_click = False
    if args.wait_click:
        if args.x is not None or args.y is not None:
            parser.error("--wait-click 不能与 --x/--y 同时使用")
        if args.goal_frame != "odom":
            parser.error("--wait-click 模式请使用 --goal-frame odom，并将 RViz Fixed Frame 设为 odom")
        if args.record_only and args.no_save:
            parser.error("--record-only 不能与 --no-save 同时使用")
    elif args.x is None or args.y is None:
        parser.error("必须同时指定 --x/--y，或使用 --wait-click")
    elif args.record_only:
        parser.error("--record-only 只能与 --wait-click 一起使用")

    rclpy.init(args=ros_args)
    node = OnlineSlamGoal(
        args.x,
        args.y,
        args.yaw,
        goal_frame=args.goal_frame,
        map_frame=args.map_frame,
        base_frame=args.base_frame,
        click_topic=args.click_topic if args.wait_click else None,
        goal_file=args.goal_file,
        save_goal=not args.no_save,
        start_mode=args.start_mode,
        game_status_topic=args.game_status_topic,
        route_steps=route_steps,
        record_only=args.record_only,
        auto_aim_topic=args.auto_aim_topic,
        auto_aim_gate=not args.no_auto_aim_gate,
    )
    # 接管 SIGINT/SIGTERM：rclpy 默认处理会先关闭 context，导致收尾的
    # 那批 false 发不出去。这里改成置标志、由主循环正常退出。
    stopping = {"requested": False}

    def request_stop(_signum, _frame):
        stopping["requested"] = True

    for _sig in (signal.SIGINT, signal.SIGTERM):
        try:
            signal.signal(_sig, request_stop)
        except (ValueError, OSError):
            pass

    try:
        while rclpy.ok() and not node.finished and not stopping["requested"]:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        node.get_logger().info("用户中止在线 SLAM 目标测试")
    except ExternalShutdownException:
        pass
    finally:
        if rclpy.ok():
            node.shutdown_auto_aim()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
