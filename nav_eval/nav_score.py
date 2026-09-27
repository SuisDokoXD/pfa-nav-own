#!/usr/bin/env python3
"""Score a Nav2 run from a rosbag2 or live ROS 2 topics.

The metrics are deliberately based on common Nav2 topics so the same scorer can
be used in simulation and on the robot.  Missing topics are reported as null,
not silently treated as good performance.
"""

from __future__ import annotations

import argparse
import json
import math
import hashlib
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Optional


ENVIRONMENT_DEFAULTS = {
    "simulation": {"namespace": "red_standard_robot1"},
    "reality": {"namespace": ""},
}


def _stamp(msg: Any, fallback: float) -> float:
    h = getattr(msg, "header", None)
    if h and getattr(h, "stamp", None):
        return float(h.stamp.sec) + float(h.stamp.nanosec) * 1e-9
    return fallback


def _norm_frame(frame: str) -> str:
    return (frame or "").strip().lstrip("/")


def _frame_without_namespace(frame: str, namespace: str) -> str:
    """Normalize frame IDs emitted by bridges that prefix the ROS namespace."""
    normalized = _norm_frame(frame)
    prefix = _norm_frame(namespace)
    if prefix and normalized.startswith(prefix + "/"):
        return normalized[len(prefix) + 1 :]
    return normalized


def _yaw(q: Any) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _transform_xy(x: float, y: float, transform: Any) -> tuple[float, float]:
    q = transform.transform.rotation
    yaw = _yaw(q)
    tx = transform.transform.translation.x
    ty = transform.transform.translation.y
    return (
        tx + math.cos(yaw) * x - math.sin(yaw) * y,
        ty + math.sin(yaw) * x + math.cos(yaw) * y,
    )


def _dist(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _mean(values: list[float]) -> Optional[float]:
    return sum(values) / len(values) if values else None


def _path_shape(points: list[tuple[float, float]], samples: int = 5) -> list[tuple[float, float]]:
    """Resample a path relative to its first point for meaningful replan checks."""
    if not points:
        return []
    if len(points) == 1:
        return [(0.0, 0.0)] * samples
    cumulative = [0.0]
    for first, second in zip(points, points[1:]):
        cumulative.append(cumulative[-1] + _dist(first, second))
    total = cumulative[-1]
    if total <= 1e-6:
        return [(0.0, 0.0)] * samples
    result = []
    for index in range(samples):
        target = total * index / (samples - 1)
        segment = next((i for i in range(1, len(cumulative)) if cumulative[i] >= target), len(cumulative) - 1)
        span = cumulative[segment] - cumulative[segment - 1]
        ratio = 0.0 if span <= 1e-9 else (target - cumulative[segment - 1]) / span
        x = points[segment - 1][0] + ratio * (points[segment][0] - points[segment - 1][0])
        y = points[segment - 1][1] + ratio * (points[segment][1] - points[segment - 1][1])
        result.append((x - points[0][0], y - points[0][1]))
    return result


def _path_changed_significantly(previous: list[tuple[float, float]], current: list[tuple[float, float]]) -> bool:
    """Do not count routine controller path publications as replans."""
    if not previous or not current:
        return False
    previous_shape = _path_shape(previous)
    current_shape = _path_shape(current)
    shape_error = _mean([_dist(a, b) for a, b in zip(previous_shape, current_shape)]) or 0.0
    previous_length = sum(_dist(a, b) for a, b in zip(previous, previous[1:]))
    current_length = sum(_dist(a, b) for a, b in zip(current, current[1:]))
    length_error = abs(current_length - previous_length)
    return shape_error > 0.35 or length_error > 0.75


def _write_json_atomic(path: Path, payload: dict[str, Any]) -> None:
    """Publish a complete snapshot so a GUI never reads a half-written file."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


@dataclass
class Evaluator:
    environment: str = "simulation"
    goal: Optional[tuple[float, float]] = None
    goal_frame: Optional[str] = None
    source_goal: Optional[tuple[float, float]] = None
    source_goal_frame: Optional[str] = None
    goal_tf_resolved: bool = False
    goal_tolerance: float = 0.25
    timeout: float = 300.0
    safety_distance: float = 0.30
    reference_time: float = 60.0
    reference_length: float = 20.0
    samples: int = 0
    first_t: Optional[float] = None
    last_t: Optional[float] = None
    last_pose: Optional[tuple[float, float, float]] = None
    path_length: float = 0.0
    min_range: Optional[float] = None
    tracking_errors: list[float] = field(default_factory=list)
    speed_errors: list[float] = field(default_factory=list)
    speeds: list[float] = field(default_factory=list)
    accelerations: list[float] = field(default_factory=list)
    last_angular_speed: float = 0.0
    cmd_last: Optional[tuple[float, float, float, float]] = None
    plan: list[tuple[float, float]] = field(default_factory=list)
    local_plan: list[tuple[float, float]] = field(default_factory=list)
    replans: int = 0
    local_replans: int = 0
    plan_lengths: list[float] = field(default_factory=list)
    local_plan_lengths: list[float] = field(default_factory=list)
    plan_update_times: list[float] = field(default_factory=list)
    local_plan_update_times: list[float] = field(default_factory=list)
    tf_errors: list[float] = field(default_factory=list)
    command_count: int = 0
    nonzero_command_count: int = 0
    command_stale_count: int = 0
    last_command_t: Optional[float] = None
    last_plan_digest: Optional[str] = None
    last_local_plan_digest: Optional[str] = None
    last_plan_t: Optional[float] = None
    last_local_plan_t: Optional[float] = None
    success: Optional[bool] = None
    action_status: Optional[int] = None
    action_status_t: Optional[float] = None
    run_start_t: Optional[float] = None
    pose_frame: Optional[str] = None
    global_plan_frame: Optional[str] = None
    local_plan_frame: Optional[str] = None
    frame_mismatch_count: int = 0
    tf_samples: int = 0
    tf_lookup_failures: int = 0
    gt_last_pose: Optional[tuple[float, float, float]] = None
    gt_last_velocity: Optional[tuple[float, float, float]] = None
    gt_last_t: Optional[float] = None
    gt_frame: Optional[str] = None
    gt_position_errors: list[float] = field(default_factory=list)
    gt_yaw_errors: list[float] = field(default_factory=list)
    gt_velocity_errors: list[float] = field(default_factory=list)
    gt_frame_mismatch_count: int = 0
    scan_last_ranges: Optional[list[float]] = None
    stationary_scan_jitter: list[float] = field(default_factory=list)
    scan_valid_ratios: list[float] = field(default_factory=list)
    scan_spike_rates: list[float] = field(default_factory=list)
    path_heading_variation: list[float] = field(default_factory=list)
    namespace: str = ""

    def __post_init__(self) -> None:
        self.source_goal = self.goal
        self.source_goal_frame = _norm_frame(self.goal_frame or "") or None
        self.namespace = _norm_frame(self.namespace)

    def _time(self, t: float) -> None:
        if self.first_t is None:
            self.first_t = t
        self.last_t = t

    def resolve_goal(self, x: float, y: float, frame: str) -> None:
        self.goal = (x, y)
        self.goal_frame = _norm_frame(frame)
        self.goal_tf_resolved = True

    def odom(self, t: float, x: float, y: float, yaw: float, vx: float, vy: float, wz: float, frame: str = "") -> None:
        self._time(t)
        self.samples += 1
        if frame:
            self.pose_frame = _frame_without_namespace(frame, self.namespace)
        pose = (x, y, yaw)
        if self.last_pose is not None:
            self.path_length += _dist((x, y), self.last_pose[:2])
        self.last_pose = pose
        speed = math.hypot(vx, vy)
        self.speeds.append(speed)
        self.last_angular_speed = abs(wz)
        if self.gt_last_pose is not None:
            if self.pose_frame and self.gt_frame and _frame_without_namespace(self.pose_frame, self.namespace) != _frame_without_namespace(self.gt_frame, self.namespace):
                self.gt_frame_mismatch_count += 1
            else:
                self.gt_position_errors.append(_dist((x, y), self.gt_last_pose[:2]))
                self.gt_yaw_errors.append(abs(math.atan2(math.sin(yaw - self.gt_last_pose[2]), math.cos(yaw - self.gt_last_pose[2]))))
                if self.gt_last_velocity is not None:
                    self.gt_velocity_errors.append(_dist((vx, vy), self.gt_last_velocity[:2]) + abs(wz - self.gt_last_velocity[2]))
        tracking_plan = self.local_plan if self.local_plan_frame in (None, self.pose_frame) else self.plan
        if tracking_plan:
            selected_frame = self.local_plan_frame if tracking_plan is self.local_plan else self.global_plan_frame
            if self.pose_frame and selected_frame and _frame_without_namespace(self.pose_frame, self.namespace) != _frame_without_namespace(selected_frame, self.namespace):
                self.frame_mismatch_count += 1
            else:
                self.tracking_errors.append(min(_dist((x, y), p) for p in tracking_plan))

    def ground_truth_odom(self, t: float, x: float, y: float, yaw: float, vx: float, vy: float, wz: float, frame: str = "") -> None:
        self.gt_last_pose = (x, y, yaw)
        self.gt_last_velocity = (vx, vy, wz)
        self.gt_last_t = t
        if frame:
            self.gt_frame = _frame_without_namespace(frame, self.namespace)

    def command(self, t: float, vx: float, vy: float, wz: float) -> None:
        self._time(t)
        self.cmd_last = (t, vx, vy, wz)
        self.command_count += 1
        if math.hypot(vx, vy) > 0.02 or abs(wz) > 0.02:
            self.nonzero_command_count += 1
            if self.run_start_t is None:
                self.run_start_t = t
        self.last_command_t = t

    def nav_status(self, t: float, status: int) -> None:
        # action_msgs/GoalStatus: ACTIVE=2, SUCCEEDED=4, CANCELED=5, ABORTED=6
        if status in (2, 4, 5, 6):
            self.action_status = status
            self.action_status_t = t
            if status == 2 and self.run_start_t is None:
                self.run_start_t = t

    def actual_command(self, t: float, vx: float, vy: float, wz: float) -> None:
        if self.cmd_last is None:
            return
        _, cx, cy, cw = self.cmd_last
        self.speed_errors.append(abs(math.hypot(vx, vy) - math.hypot(cx, cy)))
        self.accelerations.append(abs(wz - cw))
        if t - self.cmd_last[0] > 0.20:
            self.command_stale_count += 1

    def tf_error(self, error_m: float) -> None:
        if math.isfinite(error_m):
            self.tf_errors.append(abs(error_m))

    def tf_sample(self, available: bool) -> None:
        self.tf_samples += 1
        if not available:
            self.tf_lookup_failures += 1

    @staticmethod
    def _path_length(points: list[tuple[float, float]]) -> float:
        return sum(_dist(a, b) for a, b in zip(points, points[1:]))

    @staticmethod
    def _digest(points: list[tuple[float, float]]) -> str:
        return hashlib.sha1(repr([(round(x, 3), round(y, 3)) for x, y in points]).encode()).hexdigest()

    def path(self, points: Iterable[tuple[float, float]], frame: str = "") -> None:
        new_plan = list(points)
        if new_plan:
            digest = self._digest(new_plan)
            if self.plan and digest != self.last_plan_digest and _path_changed_significantly(self.plan, new_plan):
                self.replans += 1
            self.plan = new_plan
            if frame:
                self.global_plan_frame = _frame_without_namespace(frame, self.namespace)
            self.last_plan_digest = digest
            self.plan_lengths.append(self._path_length(new_plan))
            if self.last_t is not None:
                self.plan_update_times.append(self.last_t)

    def local_path(self, points: Iterable[tuple[float, float]], frame: str = "") -> None:
        new_plan = list(points)
        if new_plan:
            digest = self._digest(new_plan)
            if self.local_plan and digest != self.last_local_plan_digest and _path_changed_significantly(self.local_plan, new_plan):
                self.local_replans += 1
            self.local_plan = new_plan
            if frame:
                self.local_plan_frame = _frame_without_namespace(frame, self.namespace)
            self.last_local_plan_digest = digest
            self.local_plan_lengths.append(self._path_length(new_plan))
            headings = [math.atan2(b[1] - a[1], b[0] - a[0]) for a, b in zip(new_plan, new_plan[1:]) if _dist(a, b) > 1e-6]
            if len(headings) > 1:
                self.path_heading_variation.append(sum(abs(math.atan2(math.sin(b - a), math.cos(b - a))) for a, b in zip(headings, headings[1:])))
            if self.last_t is not None:
                self.local_plan_update_times.append(self.last_t)

    def scan(self, ranges: Iterable[float], range_min: float = 0.0, range_max: float = float("inf")) -> None:
        current_ranges = list(ranges)
        valid = [r for r in current_ranges if math.isfinite(r) and r >= range_min and r <= range_max]
        if current_ranges:
            self.scan_valid_ratios.append(len(valid) / len(current_ranges))
            interior = [
                i for i in range(1, len(current_ranges) - 1)
                if math.isfinite(current_ranges[i - 1]) and math.isfinite(current_ranges[i])
                and math.isfinite(current_ranges[i + 1])
                and current_ranges[i - 1] >= range_min and current_ranges[i] >= range_min
                and current_ranges[i + 1] >= range_min
            ]
            spikes = [
                i for i in interior
                if abs(current_ranges[i] - (current_ranges[i - 1] + current_ranges[i + 1]) / 2.0)
                > max(0.08, 0.08 * current_ranges[i])
            ]
            self.scan_spike_rates.append(len(spikes) / max(1, len(interior)))
        if valid:
            current = min(valid)
            self.min_range = current if self.min_range is None else min(self.min_range, current)
        if self.scan_last_ranges is not None and (not self.speeds or self.speeds[-1] < 0.05) and self.last_angular_speed < 0.08:
            changes = [abs(a - b) for a, b in zip(current_ranges, self.scan_last_ranges)
                       if math.isfinite(a) and math.isfinite(b) and a >= range_min and b >= range_min]
            if changes:
                self.stationary_scan_jitter.append(_mean(changes) or 0.0)
        self.scan_last_ranges = current_ranges

    def finish(self) -> dict[str, Any]:
        start_t = self.run_start_t if self.run_start_t is not None else self.first_t
        duration = (self.last_t - start_t) if start_t is not None and self.last_t is not None else None
        final_error = None
        goal_frame_mismatch = bool(
            self.goal_frame and self.pose_frame
            and _frame_without_namespace(self.goal_frame, self.namespace)
            != _frame_without_namespace(self.pose_frame, self.namespace)
        )
        if self.goal and self.last_pose and not goal_frame_mismatch:
            final_error = _dist(self.goal, self.last_pose[:2])
            self.success = final_error <= self.goal_tolerance and (duration is None or duration <= self.timeout)
        if self.action_status == 4:
            self.success = True
        elif self.action_status in (5, 6):
            self.success = False
        tracking = _mean(self.tracking_errors)
        speed_error = _mean(self.speed_errors)
        smooth = _mean(self.accelerations)
        tf_error = _mean(self.tf_errors)
        plan_length = _mean(self.plan_lengths)
        local_plan_length = _mean(self.local_plan_lengths)
        command_stale_rate = (self.command_stale_count / len(self.speed_errors)) if self.speed_errors else None
        tf_availability = (1.0 - self.tf_lookup_failures / self.tf_samples) if self.tf_samples else None
        gt_position_error = _mean(self.gt_position_errors)
        gt_yaw_error = _mean(self.gt_yaw_errors)
        gt_velocity_error = _mean(self.gt_velocity_errors)
        gt_score = None
        if gt_position_error is not None:
            gt_score = (
                0.60 * math.exp(-gt_position_error / 0.05)
                + 0.25 * math.exp(-(gt_yaw_error or 0.0) / 0.08)
                + 0.15 * math.exp(-(gt_velocity_error or 0.0) / 0.15)
            )
        scan_jitter = _mean(self.stationary_scan_jitter)
        scan_valid_ratio = _mean(self.scan_valid_ratios)
        scan_spike_rate = _mean(self.scan_spike_rates)
        jitter_score = math.exp(-(scan_jitter or 0.0) / 0.03) if scan_jitter is not None else None
        spike_score = max(0.0, 1.0 - scan_spike_rate) if scan_spike_rate is not None else None
        scan_score_parts = [part for part in (scan_valid_ratio, jitter_score, spike_score) if part is not None]
        scan_score = sum(scan_score_parts) / len(scan_score_parts) if scan_score_parts else None
        path_variation = _mean(self.path_heading_variation)
        path_smoothness_score = math.exp(-(path_variation or 0.0) / 1.5) if path_variation is not None else None
        frame_score = math.exp(-self.frame_mismatch_count / max(1, self.samples))
        planning_score = math.exp(-self.local_replans / max(1.0, (duration or 1.0) * 2.0))
        if self.success is None:
            success_score = None
        else:
            success_score = 1.0 if self.success else 0.0
        tracking_score = math.exp(-tracking / 0.20) if tracking is not None else None
        safety_score = min(1.0, self.min_range / self.safety_distance) if self.min_range is not None else None
        time_score = min(1.0, self.reference_time / duration) if duration and duration > 0 else None
        if self.path_length > 0 and plan_length and plan_length > 0:
            length_score = min(1.0, plan_length / self.path_length)
        else:
            length_score = min(1.0, self.reference_length / self.path_length) if self.path_length > 0 else None
        speed_score = math.exp(-(speed_error or 0.0) / 0.25) if speed_error is not None else None
        smooth_score = math.exp(-(smooth or 0.0) / 0.50) if smooth is not None else None
        tf_score = math.exp(-tf_error / 0.10) if tf_error is not None else tf_availability
        command_score = max(0.0, 1.0 - command_stale_rate) if command_stale_rate is not None else None
        base_weights = [(0.23, tracking_score), (0.18, safety_score), (0.12, time_score),
                        (0.10, length_score), (0.14, speed_score), (0.05, smooth_score),
                        (0.08, tf_score), (0.05, frame_score), (0.05, planning_score),
                        (0.05, scan_score), (0.05, path_smoothness_score)]
        if self.environment == "simulation":
            weighted = [(weight * 0.85, value) for weight, value in base_weights]
            weighted.append((0.15, gt_score))
        else:
            weighted = base_weights
        # A missing signal is a measurement gap, not a perfect result. Keep the
        # configured denominator so incomplete bags cannot score artificially high.
        quality = sum(w * (v if v is not None else 0.0) for w, v in weighted)
        score = None if success_score is None else 100.0 * success_score * quality
        return {
            "environment": self.environment,
            "score": round(score, 2) if score is not None else None,
            "quality_score": round(100.0 * quality, 2),
            "success": self.success,
            "nav_action_status": self.action_status,
            "duration_s": duration,
            "final_position_error_m": final_error,
            "goal_frame": self.goal_frame,
            "source_goal": self.source_goal,
            "source_goal_frame": self.source_goal_frame,
            "goal_tf_resolved": self.goal_tf_resolved,
            "goal_frame_mismatch": goal_frame_mismatch,
            "frame_transform_note": (
                "final position error omitted because goal and odometry frames differ; "
                "Nav2 action status remains authoritative"
                if goal_frame_mismatch else None
            ),
            "path_length_m": self.path_length,
            "min_obstacle_distance_m": self.min_range,
            "mean_tracking_error_m": tracking,
            "mean_speed_error_mps": speed_error,
            "mean_yaw_rate_error_rps": smooth,
            "mean_tf_consistency_error_m": tf_error,
            "mean_ground_truth_position_error_m": gt_position_error,
            "mean_ground_truth_yaw_error_rad": gt_yaw_error,
            "mean_ground_truth_velocity_error": gt_velocity_error,
            "ground_truth_frame": self.gt_frame,
            "ground_truth_frame_mismatch_count": self.gt_frame_mismatch_count,
            "mean_stationary_scan_jitter_m": scan_jitter,
            "mean_scan_valid_ratio": scan_valid_ratio,
            "mean_scan_spike_rate": scan_spike_rate,
            "mean_local_path_heading_variation_rad": path_variation,
            "tf_samples": self.tf_samples,
            "tf_lookup_failures": self.tf_lookup_failures,
            "tf_availability": tf_availability,
            "pose_frame": self.pose_frame,
            "global_plan_frame": self.global_plan_frame,
            "local_plan_frame": self.local_plan_frame,
            "frame_mismatch_count": self.frame_mismatch_count,
            "mean_global_plan_length_m": plan_length,
            "mean_local_plan_length_m": local_plan_length,
            "replans": self.replans,
            "local_replans": self.local_replans,
            "command_count": self.command_count,
            "nonzero_command_ratio": (self.nonzero_command_count / self.command_count) if self.command_count else None,
            "stale_command_rate": command_stale_rate,
            "samples": self.samples,
            "components": {
                "tracking": tracking_score, "safety": safety_score, "time": time_score,
                "length": length_score, "speed_tracking": speed_score, "smoothness": smooth_score,
                "tf_consistency": tf_score, "command_freshness": command_score,
                "frame_consistency": frame_score, "planning": planning_score,
                "scan_stability": scan_score, "path_smoothness": path_smoothness_score,
                "ground_truth": gt_score,
            },
            "missing_metric_note": "null metrics contribute zero; incomplete recordings cannot inflate the score",
        }


def _topic_name(name: str, namespace: str) -> str:
    if name.startswith("/") or not namespace:
        return name
    return namespace.rstrip("/") + "/" + name


def _apply_namespace(args: argparse.Namespace) -> None:
    namespace = (args.namespace or "").strip("/")
    if not namespace:
        return
    prefix = "/" + namespace
    for key in (
        "odom_topic", "ground_truth_odom_topic", "cmd_topic", "actual_cmd_topic", "path_topic",
        "local_path_topic", "scan_topic", "action_status_topic",
    ):
        value = getattr(args, key)
        if value.startswith(prefix + "/"):
            continue
        setattr(args, key, prefix + "/" + value.lstrip("/"))


def score_bag(args: argparse.Namespace) -> dict[str, Any]:
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise SystemExit("score_bag requires a sourced ROS 2 environment with rosbag2_py") from exc
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.bag), storage_id=args.storage), rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {x.name: x.type for x in reader.get_all_topics_and_types()}
    wanted = {
        args.odom_topic, args.ground_truth_odom_topic, args.cmd_topic, args.actual_cmd_topic, args.path_topic,
        args.local_path_topic, args.scan_topic, args.action_status_topic,
    }
    msgs = {t: (get_message(types[t]) if t in types else None) for t in wanted}
    ev = Evaluator(environment=args.environment, namespace=args.namespace or "",
                   goal=(args.goal_x, args.goal_y) if args.goal_x is not None and args.goal_y is not None else None,
                   goal_frame=args.goal_frame,
                   goal_tolerance=args.goal_tolerance, timeout=args.timeout, safety_distance=args.safety_distance)
    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        if topic not in msgs or msgs[topic] is None:
            continue
        msg = deserialize_message(data, msgs[topic])
        t = stamp_ns * 1e-9
        if topic == args.odom_topic:
            p, tw = msg.pose.pose, msg.twist.twist
            ev.odom(t, p.position.x, p.position.y, _yaw(p.orientation), tw.linear.x, tw.linear.y, tw.angular.z, msg.header.frame_id)
        elif topic == args.ground_truth_odom_topic:
            p, tw = msg.pose.pose, msg.twist.twist
            ev.ground_truth_odom(t, p.position.x, p.position.y, _yaw(p.orientation), tw.linear.x, tw.linear.y, tw.angular.z, msg.header.frame_id)
        elif topic == args.cmd_topic:
            tw = getattr(msg, "twist", msg)
            ev.command(t, tw.linear.x, tw.linear.y, tw.angular.z)
        elif topic == args.actual_cmd_topic:
            tw = getattr(msg, "twist", msg)
            ev.actual_command(t, tw.linear.x, tw.linear.y, tw.angular.z)
        elif topic == args.path_topic:
            ev.path(((p.pose.position.x, p.pose.position.y) for p in msg.poses), msg.header.frame_id)
        elif topic == args.local_path_topic:
            ev.local_path(((p.pose.position.x, p.pose.position.y) for p in msg.poses), msg.header.frame_id)
        elif topic == args.scan_topic:
            ev.scan(msg.ranges, msg.range_min, msg.range_max)
        elif topic == args.action_status_topic:
            statuses = getattr(msg, "status_list", [])
            if statuses:
                ev.nav_status(t, statuses[-1].status)
    return ev.finish()


def live_node(args: argparse.Namespace, ros_args: list[str]) -> None:
    try:
        import rclpy
        from nav_msgs.msg import Odometry, Path
        from geometry_msgs.msg import Twist, TwistStamped
        from sensor_msgs.msg import LaserScan
        from action_msgs.msg import GoalStatusArray
        from tf2_ros import Buffer, TransformListener, TransformException
        from rclpy.time import Time
        from rclpy.parameter import Parameter
        from rclpy.node import Node
    except ImportError as exc:
        raise SystemExit("score_live requires a sourced ROS 2 environment") from exc
    class ScoreNode(Node):
        def __init__(self) -> None:
            super().__init__(
                "nav_score",
                parameter_overrides=[
                    Parameter("use_sim_time", value=args.environment == "simulation")
                ],
            )
            self.ev = Evaluator(environment=args.environment, namespace=args.namespace or "",
                                goal=(args.goal_x, args.goal_y) if args.goal_x is not None and args.goal_y is not None else None,
                                goal_frame=args.goal_frame,
                                goal_tolerance=args.goal_tolerance, timeout=args.timeout, safety_distance=args.safety_distance)
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.create_subscription(Odometry, args.odom_topic, self.odom_cb, 20)
            if args.environment == "simulation":
                self.create_subscription(Odometry, args.ground_truth_odom_topic, self.ground_truth_odom_cb, 20)
            self.create_subscription(Path, args.path_topic, self.path_cb, 5)
            self.create_subscription(Path, args.local_path_topic, self.local_path_cb, 5)
            self.create_subscription(LaserScan, args.scan_topic, self.scan_cb, 10)
            self.create_subscription(Twist, args.cmd_topic, self.cmd_cb, 20)
            self.create_subscription(Twist, args.actual_cmd_topic, self.actual_cmd_cb, 20)
            self.create_subscription(TwistStamped, args.cmd_topic + "/stamped", self.cmd_stamped_cb, 20)
            self.create_subscription(GoalStatusArray, args.action_status_topic, self.action_status_cb, 10)
            if args.live_report:
                self.create_timer(1.0, self.report)
            self.create_timer(0.2, self.tf_check)
        def now(self) -> float: return self.get_clock().now().nanoseconds * 1e-9
        def odom_cb(self, m):
            p, tw = m.pose.pose, m.twist.twist
            frame = _norm_frame(m.header.frame_id)
            self.resolve_goal(frame)
            self.ev.odom(self.now(), p.position.x, p.position.y, _yaw(p.orientation), tw.linear.x, tw.linear.y, tw.angular.z, frame)
        def ground_truth_odom_cb(self, m):
            p, tw = m.pose.pose, m.twist.twist
            self.ev.ground_truth_odom(self.now(), p.position.x, p.position.y, _yaw(p.orientation), tw.linear.x, tw.linear.y, tw.angular.z, m.header.frame_id)
        def path_cb(self, m):
            self.ev.path(((p.pose.position.x, p.pose.position.y) for p in m.poses), m.header.frame_id)
        def local_path_cb(self, m):
            points = [(p.pose.position.x, p.pose.position.y) for p in m.poses]
            source_frame = _norm_frame(m.header.frame_id)
            target_frame = self.ev.pose_frame
            if points and source_frame and target_frame and source_frame != target_frame:
                try:
                    transform = self.tf_buffer.lookup_transform(target_frame, source_frame, Time())
                    points = [_transform_xy(x, y, transform) for x, y in points]
                    source_frame = target_frame
                except TransformException:
                    pass
            self.ev.local_path(points, source_frame)
        def scan_cb(self, m): self.ev.scan(m.ranges, m.range_min, m.range_max)
        def cmd_cb(self, m): self.ev.command(self.now(), m.linear.x, m.linear.y, m.angular.z)
        def actual_cmd_cb(self, m): self.ev.actual_command(self.now(), m.linear.x, m.linear.y, m.angular.z)
        def action_status_cb(self, m):
            if m.status_list:
                self.ev.nav_status(self.now(), m.status_list[-1].status)
        def resolve_goal(self, pose_frame):
            goal_frame = _norm_frame(self.ev.source_goal_frame or self.ev.goal_frame or "")
            if not self.ev.source_goal or not pose_frame or not goal_frame:
                return
            if goal_frame == pose_frame:
                self.ev.resolve_goal(*self.ev.source_goal, pose_frame)
                return
            try:
                transform = self.tf_buffer.lookup_transform(pose_frame, goal_frame, Time())
                x, y = _transform_xy(self.ev.source_goal[0], self.ev.source_goal[1], transform)
                self.ev.resolve_goal(x, y, pose_frame)
            except TransformException:
                pass
        def tf_check(self):
            try:
                self.tf_buffer.lookup_transform(args.global_frame, args.nav_base_frame, Time())
                self.ev.tf_sample(True)
            except TransformException:
                self.ev.tf_sample(False)
        def cmd_stamped_cb(self, m): self.ev.command(self.now(), m.twist.linear.x, m.twist.linear.y, m.twist.angular.z)
        def report(self):
            snapshot = self.ev.finish()
            payload = json.dumps(snapshot, ensure_ascii=False)
            print(payload, flush=True)
            if args.live_report_file:
                _write_json_atomic(args.live_report_file, snapshot)
    rclpy.init(args=ros_args); node = ScoreNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    result = node.ev.finish()
    print(json.dumps(result, ensure_ascii=False, indent=2))
    if args.result_file:
        _write_json_atomic(args.result_file, result)
    if args.live_report_file:
        _write_json_atomic(args.live_report_file, result)
    node.destroy_node(); rclpy.shutdown()


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest="mode", required=True)
    def common(q):
        q.add_argument("--environment", choices=tuple(ENVIRONMENT_DEFAULTS), default="simulation",
                       help="select topic defaults: simulation uses red_standard_robot1; reality uses an empty namespace")
        q.add_argument("--namespace", default=None, help="override the environment namespace")
        q.add_argument("--odom-topic", default="/odometry"); q.add_argument("--cmd-topic", default="/cmd_vel_nav2_result")
        q.add_argument("--ground-truth-odom-topic", default="/chassis_odometry_gt")
        q.add_argument("--actual-cmd-topic", default="/cmd_vel")
        q.add_argument("--path-topic", default="/plan"); q.add_argument("--local-path-topic", default="/local_plan")
        q.add_argument("--scan-topic", default="/obstacle_scan")
        q.add_argument("--action-status-topic", default="/navigate_to_pose/_action/status")
        q.add_argument("--global-frame", default="map")
        q.add_argument("--nav-base-frame", default="gimbal_yaw_fake")
        q.add_argument("--goal-frame", default="odom")
        q.add_argument("--goal-x", type=float); q.add_argument("--goal-y", type=float); q.add_argument("--goal-tolerance", type=float, default=0.25)
        q.add_argument("--timeout", type=float, default=300.0); q.add_argument("--safety-distance", type=float, default=0.30)
    b = sub.add_parser("score_bag", help="score a rosbag2 directory"); b.add_argument("bag", type=Path); b.add_argument("--storage", default="sqlite3"); common(b)
    l = sub.add_parser("score_live", help="score live ROS 2 topics"); common(l)
    l.add_argument("--live-report", action="store_true", help="print a JSON snapshot every second")
    l.add_argument("--result-file", type=Path, help="write final JSON result on shutdown")
    l.add_argument(
        "--live-report-file", type=Path,
        help="write the latest live JSON snapshot; suitable for a GUI dashboard",
    )
    args, ros_args = p.parse_known_args()
    if args.namespace is None:
        args.namespace = ENVIRONMENT_DEFAULTS[args.environment]["namespace"]
    _apply_namespace(args)
    if args.mode == "score_live" and args.namespace:
        ns = "/" + args.namespace.strip("/")
        ros_args = ["--ros-args", "-r", "/tf:=" + ns + "/tf", "-r", "/tf_static:=" + ns + "/tf_static"] + ros_args
    result = score_bag(args) if args.mode == "score_bag" else (live_node(args, ros_args) or None)
    if result is not None: print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__": main()
