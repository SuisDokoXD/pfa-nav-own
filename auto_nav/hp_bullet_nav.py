#!/usr/bin/env python3
import argparse
import time
from collections import deque
from pathlib import Path

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

DEFAULT_BLUE_WAYPOINTS = str(Path(__file__).resolve().parent / 'config' / 'waypoints_blue.yaml')

SITUATION_IDX_SENTRY_HP = 1
SITUATION_IDX_REMAINING_PROJECTILES = 0
SITUATION_IDX_GAME_STATE = 4
SITUATION_IDX_SYSTEM_TIME = 5
ACTIVE_GAME_STATE = 4
SITUATION_FORMAT = (
    "[projectile_allowance, current_hp, current_outpost_hp, "
    "current_base_hp, game_state, stage_remain_time]"
)

# ─── 可调参数（改这里就行） ──────────────────────────────────────────
HP_FULL = 400                    # 血量上限，hp >= 此值视为回满
HP_THRESHOLD = 120               # hp <= 此值触发回补给点
BULLET_LOW_THRESHOLD = 20        # bullets <= 此值触发回补给点
BULLET_RESUPPLY_THRESHOLD = 100   # bullets >= 此值视为补充完成
RESUPPLY_POINT = 1               # 补给点航点编号 (1-based)
WORK_POINT = 2                   # 作战点航点编号 (1-based)
LATE_WORK_POINT = 3              # 剩余时间较少时使用的作战点航点编号 (1-based)
LATE_WORK_TIME_THRESHOLD = 240   # stage_remain_time < 此值时切换到 LATE_WORK_POINT
GOAL_RESPONSE_TIMEOUT = 5.0      # 等待 Nav2 goal 响应的超时时间（秒）
# ────────────────────────────────────────────────────────────────────


class HpBulletNavNode(Node):
    def __init__(self, yaml_path,
                 resupply_point, work_point, late_work_point, late_work_time_threshold,
                 hp_threshold, hp_full, bullet_low_threshold, bullet_resupply_threshold,
                 nav_action, situation_topic, goal_response_timeout,
                 nav_start_after_sec=None):
        super().__init__('hp_bullet_nav_node')

        self.waypoints = self.load_waypoints(yaml_path)
        self.get_logger().info(f"已从 {yaml_path} 加载 {len(self.waypoints)} 个航点")

        self.resupply_id = resupply_point
        self.work_id = work_point
        self.late_work_id = late_work_point
        self.late_work_time_threshold = late_work_time_threshold
        for pid, label in [
            (self.resupply_id, '补给点'),
            (self.work_id, '作战点'),
            (self.late_work_id, '后期作战点'),
        ]:
            if pid < 1 or pid > len(self.waypoints):
                raise ValueError(f"{label}编号 {pid} 超出范围 [1-{len(self.waypoints)}]")

        self.hp_threshold = hp_threshold
        self.hp_full = hp_full
        self.bullet_low_threshold = bullet_low_threshold
        self.bullet_resupply_threshold = bullet_resupply_threshold
        self.current_hp = hp_full
        self.current_bullets = max(bullet_resupply_threshold, 1)
        self.current_game_state = None
        self.current_system_time_sec = None
        self.received_first_status = False
        # 兼容旧启动参数；导航执行现在只由 game_state 控制。
        self.nav_start_after_sec = nav_start_after_sec

        self.is_resupplying = False
        self.last_arrived_id = None
        self.route_queue = deque()

        self.nav_ac = ActionClient(self, NavigateToPose, nav_action)
        self.nav_action_name = nav_action
        self.sending = False
        self.max_retry = 3
        self.retry_count = 0
        self.current_goal_handle = None
        self.goal_sequence = 0
        self.active_goal_sequence = None
        self.cancel_in_progress = False
        self.goal_response_timeout = goal_response_timeout
        self.goal_request_sent_at = None
        self.pending_goal_id = None

        self.situation_topic = situation_topic
        self.create_subscription(
            Int32MultiArray, situation_topic, self.situation_callback, 10
        )
        self.get_logger().info(f"已订阅 {situation_topic}")

        self.create_timer(1.0, self.navigation_loop)
        self.get_logger().info(
            f"hp_bullet_nav 已启动: 补给点={self.resupply_id}, 作战点={self.work_id}, "
            f"后期作战点={self.late_work_id}(<{self.late_work_time_threshold}s); "
            f"hp 阈值={self.hp_threshold}, 满血={self.hp_full}, "
            f"子弹回城阈值={self.bullet_low_threshold}, 子弹补充阈值={self.bullet_resupply_threshold}, "
            f"导航执行条件=game_state={ACTIVE_GAME_STATE}"
        )

    def load_waypoints(self, yaml_path):
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
        return [data[f'Waypoint_{i}'] for i in range(1, data['Waypoints_Num'] + 1)]

    def situation_callback(self, msg: Int32MultiArray):
        data = list(msg.data)
        if len(data) <= SITUATION_IDX_SYSTEM_TIME:
            self.get_logger().warn(
                f"{self.situation_topic} 消息长度 {len(data)} 不足，"
                f"需要 {SITUATION_FORMAT}"
            )
            return
        new_game_state = int(data[SITUATION_IDX_GAME_STATE])
        self.current_game_state = new_game_state
        if new_game_state != ACTIVE_GAME_STATE:
            self.get_logger().info(
                f"[{self.situation_topic}] game_state={new_game_state}, "
                f"当前行为: 无动作，等待 game_state={ACTIVE_GAME_STATE}"
            )
            return
        new_hp = int(data[SITUATION_IDX_SENTRY_HP])
        new_bullets = int(data[SITUATION_IDX_REMAINING_PROJECTILES])
        new_system_time_sec = int(data[SITUATION_IDX_SYSTEM_TIME])
        self.current_hp = new_hp
        self.current_bullets = new_bullets
        self.current_system_time_sec = new_system_time_sec
        if not self.received_first_status:
            self.received_first_status = True
            self.get_logger().info("已收到首条 /situation 消息")
        self.get_logger().info(
            f"[{self.situation_topic}] projectile_allowance={new_bullets}, "
            f"current_hp={new_hp}, "
            f"game_state={new_game_state}, "
            f"stage_remain_time={new_system_time_sec}s"
        )
        self.get_logger().info(f"当前行为: {self.describe_current_behavior()}")

    def describe_current_behavior(self):
        if self.current_game_state != ACTIVE_GAME_STATE:
            return f"无动作，等待 game_state={ACTIVE_GAME_STATE}"

        current_work_id = self.current_work_id()
        if self.is_resupplying:
            resupply_done = (
                self.current_hp >= self.hp_full
                and self.current_bullets >= self.bullet_resupply_threshold
            )
            if resupply_done:
                return f"补给完成，准备前往作战点 {current_work_id}"
            return f"补给中，前往补给点 {self.resupply_id}"

        if self.current_hp <= self.hp_threshold:
            return (
                f"血量不足，准备前往补给点 {self.resupply_id} "
                f"(hp={self.current_hp} <= {self.hp_threshold})"
            )
        if self.current_bullets <= self.bullet_low_threshold:
            return (
                f"弹量不足，准备前往补给点 {self.resupply_id} "
                f"(bullets={self.current_bullets} <= {self.bullet_low_threshold})"
            )
        return f"状态正常，准备前往作战点 {current_work_id}"

    def current_work_id(self) -> int:
        if (
            self.current_system_time_sec is not None
            and self.current_system_time_sec < self.late_work_time_threshold
        ):
            return self.late_work_id
        return self.work_id

    def evaluate_desired_destination(self) -> int:
        needs_supply = (
            self.current_hp <= self.hp_threshold or self.current_bullets <= self.bullet_low_threshold
        )
        if needs_supply and not self.is_resupplying:
            self.is_resupplying = True
            self.get_logger().info(
                f"hp={self.current_hp} 子弹={self.current_bullets}，进入补给状态，前往 "
                f"{self.resupply_id} 点"
            )
        elif self.is_resupplying:
            resupply_done = (
                self.current_hp >= self.hp_full
                and self.current_bullets >= self.bullet_resupply_threshold
            )
            if resupply_done:
                self.is_resupplying = False
                self.get_logger().info(
                    f"hp={self.current_hp} 子弹={self.current_bullets}，补给完成，前往 "
                    f"{self.current_work_id()} 点"
                )
        return self.resupply_id if self.is_resupplying else self.current_work_id()

    def plan_route_to(self, target_id):
        return deque([target_id])

    def replan_route_if_needed(self, desired_final_id):
        current_final = self.route_queue[-1] if self.route_queue else None
        if current_final == desired_final_id:
            return
        if (
            not self.route_queue
            and not self.sending
            and self.last_arrived_id == desired_final_id
        ):
            return
        new_route = self.plan_route_to(desired_final_id)
        self.get_logger().info(
            f"重新规划路径: 期望终点 {desired_final_id}（原终点 {current_final}）, "
            f"新 route={list(new_route)}, last_arrived={self.last_arrived_id}, "
            f"sending={self.sending}"
        )
        self.route_queue = new_route
        if self.sending:
            self.cancel_current_goal()

    def cancel_current_goal(self):
        if self.cancel_in_progress:
            return
        if self.active_goal_sequence is None and self.current_goal_handle is None:
            self.sending = False
            self.goal_request_sent_at = None
            self.pending_goal_id = None
            return
        self.cancel_in_progress = True
        old_handle = self.current_goal_handle
        self.active_goal_sequence = None
        self.current_goal_handle = None
        self.goal_request_sent_at = None
        self.pending_goal_id = None
        if old_handle is None:
            self.sending = False
            self.cancel_in_progress = False
            return
        try:
            cancel_future = old_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._on_cancel_done)
        except Exception as e:
            self.get_logger().warn(f"取消旧 Nav2 goal 抛异常: {e}")
            self.sending = False
            self.cancel_in_progress = False

    def _on_cancel_done(self, _future):
        self.sending = False
        self.cancel_in_progress = False
        self.get_logger().info("旧 Nav2 goal 已请求取消，准备发送新路径")

    def expire_unanswered_goal_if_needed(self):
        if (
            not self.sending
            or self.cancel_in_progress
            or self.current_goal_handle is not None
            or self.goal_request_sent_at is None
        ):
            return
        elapsed = time.monotonic() - self.goal_request_sent_at
        if elapsed < self.goal_response_timeout:
            return
        self.get_logger().warn(
            f"等待目标 {self.pending_goal_id} 的 Nav2 goal 响应超过 "
            f"{self.goal_response_timeout:.1f}s，清理状态并重发"
        )
        self.active_goal_sequence = None
        self.sending = False
        self.goal_request_sent_at = None
        self.pending_goal_id = None

    def navigation_loop(self):
        if self.current_game_state != ACTIVE_GAME_STATE:
            return
        self.expire_unanswered_goal_if_needed()
        desired_final = self.evaluate_desired_destination()
        self.replan_route_if_needed(desired_final)
        if self.sending or self.cancel_in_progress:
            return
        if not self.route_queue:
            return
        next_id = self.route_queue[0]
        self.send_goal(next_id)

    def send_goal(self, waypoint_id):
        if self.sending:
            return
        idx = waypoint_id - 1
        if idx < 0 or idx >= len(self.waypoints):
            self.get_logger().error(f"无效航点 {waypoint_id}")
            return
        if not self.nav_ac.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn(f"Nav2 action server {self.nav_action_name} 未就绪")
            return
        self.goal_sequence += 1
        sequence = self.goal_sequence
        self.active_goal_sequence = sequence
        wp = self.waypoints[idx]
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(wp['Pos_x'])
        goal.pose.pose.position.y = float(wp['Pos_y'])
        goal.pose.pose.position.z = float(wp['Pos_z'])
        goal.pose.pose.orientation.x = float(wp['Ori_x'])
        goal.pose.pose.orientation.y = float(wp['Ori_y'])
        goal.pose.pose.orientation.z = float(wp['Ori_z'])
        goal.pose.pose.orientation.w = float(wp['Ori_w'])
        wp_name = wp.get('Name', f'Waypoint_{waypoint_id}')
        self.get_logger().info(
            f"发送目标点 {waypoint_id} ({wp_name}), hp={self.current_hp}, "
            f"bullets={self.current_bullets}"
        )
        self.sending = True
        self.goal_request_sent_at = time.monotonic()
        self.pending_goal_id = waypoint_id
        try:
            send_future = self.nav_ac.send_goal_async(goal)
        except Exception as e:
            self.get_logger().error(f"发送 goal {waypoint_id} 失败: {e}")
            self.sending = False
            self.active_goal_sequence = None
            self.goal_request_sent_at = None
            self.pending_goal_id = None
            return
        send_future.add_done_callback(
            lambda f, s=sequence, w=waypoint_id: self._goal_response_cb(f, s, w)
        )

    def _goal_response_cb(self, future, sequence, waypoint_id):
        if sequence != self.active_goal_sequence:
            self.get_logger().info(f"忽略过期 goal {waypoint_id} 的响应")
            return
        self.goal_request_sent_at = None
        self.pending_goal_id = None
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"获取 goal {waypoint_id} 响应失败: {e}")
            self.sending = False
            return
        if not goal_handle.accepted:
            self.retry_count += 1
            self.sending = False
            if self.retry_count < self.max_retry:
                self.get_logger().warn(
                    f"目标 {waypoint_id} 被拒绝，重试第 {self.retry_count} 次"
                )
            else:
                self.get_logger().error(
                    f"目标 {waypoint_id} 连续 {self.max_retry} 次被拒绝"
                )
                self.retry_count = 0
            return
        self.current_goal_handle = goal_handle
        self.get_logger().info(f"目标 {waypoint_id} 已被接受，等待到达")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f, s=sequence, w=waypoint_id: self._result_cb(f, s, w)
        )

    def _result_cb(self, future, sequence, waypoint_id):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f"获取 goal {waypoint_id} 结果失败: {e}")
            self.sending = False
            self.current_goal_handle = None
            return
        status = result.status
        is_stale = (sequence != self.active_goal_sequence)
        if is_stale:
            self.get_logger().info(
                f"忽略过期目标 {waypoint_id} 的结果 status={status}"
            )
            return
        self.current_goal_handle = None
        self.sending = False
        self.retry_count = 0
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"✓ 到达目标点 {waypoint_id}")
            self.last_arrived_id = waypoint_id
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(f"目标 {waypoint_id} 已取消")
        else:
            self.get_logger().warn(f"目标 {waypoint_id} 失败 status={status}，将重试")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--yaml',
        type=str,
        default='waypoints_red.yaml',
        help='航点 YAML 文件路径'
    )
    parser.add_argument(
        '--red',
        action='store_true',
        help='读取 waypoints_red.yaml'
    )
    parser.add_argument(
        '--blue',
        action='store_true',
        help='读取 waypoints_blue.yaml'
    )
    parser.add_argument(
        '--resupply-point',
        type=int,
        default=RESUPPLY_POINT,
        help='补给点航点编号，1 表示 YAML 中的 Waypoint_1'
    )
    parser.add_argument(
        '--work-point',
        type=int,
        default=WORK_POINT,
        help='作战点航点编号，1 表示 YAML 中的 Waypoint_1'
    )
    parser.add_argument(
        '--late-work-point',
        type=int,
        default=LATE_WORK_POINT,
        help='剩余时间较少时使用的作战点航点编号'
    )
    parser.add_argument(
        '--late-work-time-threshold',
        type=int,
        default=LATE_WORK_TIME_THRESHOLD,
        help='stage_remain_time 小于该值时切换到后期作战点，单位秒'
    )
    parser.add_argument(
        '--hp-threshold',
        type=int,
        default=HP_THRESHOLD,
        help='血量小于等于该值时前往补给点'
    )
    parser.add_argument(
        '--hp-full',
        type=int,
        default=HP_FULL,
        help='血量大于等于该值时认为补给完成'
    )
    parser.add_argument(
        '--bullet-low-threshold',
        type=int,
        default=BULLET_LOW_THRESHOLD,
        help='弹量小于等于该值时前往补给点'
    )
    parser.add_argument(
        '--bullet-resupply-threshold',
        type=int,
        default=BULLET_RESUPPLY_THRESHOLD,
        help='弹量大于等于该值时认为补给完成'
    )
    parser.add_argument(
        '--nav-action',
        type=str,
        default='/navigate_to_pose',
        help='Nav2 NavigateToPose action 名称'
    )
    parser.add_argument(
        '--situation-topic',
        type=str,
        default='/situation',
        help='接收裁判/局势信息的 Int32MultiArray 话题'
    )
    parser.add_argument(
        '--goal-response-timeout',
        type=float,
        default=GOAL_RESPONSE_TIMEOUT,
        help='发送 Nav2 goal 后等待响应的超时时间，单位秒'
    )
    parser.add_argument(
        '--nav-start-after-sec',
        type=int,
        default=None,
        help='兼容旧参数；当前导航只在 game_state=4 时执行，不再使用时间窗口'
    )
    args = parser.parse_args()

    if args.red and args.blue:
        parser.error('--red and --blue cannot be used together')
    if args.red:
        args.yaml = 'waypoints_red.yaml'
    elif args.blue:
        args.yaml = DEFAULT_BLUE_WAYPOINTS

    rclpy.init()
    node = None
    try:
        node = HpBulletNavNode(
            yaml_path=args.yaml,
            resupply_point=args.resupply_point,
            work_point=args.work_point,
            late_work_point=args.late_work_point,
            late_work_time_threshold=args.late_work_time_threshold,
            hp_threshold=args.hp_threshold,
            hp_full=args.hp_full,
            bullet_low_threshold=args.bullet_low_threshold,
            bullet_resupply_threshold=args.bullet_resupply_threshold,
            nav_action=args.nav_action,
            situation_topic=args.situation_topic,
            goal_response_timeout=args.goal_response_timeout,
            nav_start_after_sec=args.nav_start_after_sec,
        )
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n节点已停止")
    except Exception as e:
        print(f"错误: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
