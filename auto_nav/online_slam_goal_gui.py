#!/usr/bin/env python3
"""Small GUI launcher for recording and replaying online-SLAM goals.

The ROS implementation remains in online_slam_goal.py. This launcher only
handles the human-facing workflow and keeps saved goals in one directory.
"""

import argparse
import json
import math
import os
import re
import signal
import shutil
import subprocess
import sys
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, simpledialog, ttk


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SIMULATION_GOAL_DIR = PROJECT_ROOT / "online_slam_goals" / "simulation"
REALITY_GOAL_DIR = PROJECT_ROOT / "online_slam_goals" / "reality"
ENVIRONMENT_DEFAULTS = {
    "simulation": {"namespace": "red_standard_robot1", "use_sim_time": True, "goal_dir": SIMULATION_GOAL_DIR},
    "reality": {"namespace": "", "use_sim_time": False, "goal_dir": REALITY_GOAL_DIR},
}
CORE_SCRIPT = Path(__file__).resolve().with_name("online_slam_goal.py")
SCORER_SCRIPT = PROJECT_ROOT / "tools" / "evaluation" / "nav_eval" / "nav_score.py"
ROUTE_FORMAT = "pfa_online_slam_route/v1"


class ScoreDashboard:
    """Small live score window shared by single-goal and route tests."""

    COMPONENT_NAMES = (
        ("ground_truth", "仿真真值闭环"),
        ("tracking", "路径跟踪"),
        ("safety", "安全距离"),
        ("scan_stability", "激光稳定性/噪点"),
        ("path_smoothness", "路径平滑度"),
        ("planning", "规划稳定性"),
        ("speed_tracking", "速度跟踪"),
        ("tf_consistency", "TF 一致性"),
    )
    CURVE_METRICS = (
        ("quality_score", "质量估计分", 0.0, 100.0, "#1769aa"),
        ("mean_tracking_error_m", "跟踪误差 (m)", 0.0, None, "#c62828"),
        ("min_obstacle_distance_m", "最小障碍距离 (m)", 0.0, None, "#2e7d32"),
        ("mean_scan_spike_rate", "激光尖峰率", 0.0, 1.0, "#8e24aa"),
    )

    def __init__(self, owner):
        self.owner = owner
        self.window = tk.Toplevel(owner.root)
        self.window.title("导航评分可视化")
        self.window.geometry("680x900")
        self.window.minsize(600, 700)
        self.window.protocol("WM_DELETE_WINDOW", self.window.withdraw)
        self.score_var = tk.StringVar(value="--")
        self.state_var = tk.StringVar(value="等待评分任务")
        self.detail_vars = {}
        self.bars = {}
        self.series = []
        self.last_finished_score = None

        viewport = ttk.Frame(self.window)
        viewport.pack(fill="both", expand=True)
        view_canvas = tk.Canvas(viewport, highlightthickness=0)
        scrollbar = ttk.Scrollbar(viewport, orient="vertical", command=view_canvas.yview)
        view_canvas.configure(yscrollcommand=scrollbar.set)
        view_canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        outer = ttk.Frame(view_canvas, padding=18)
        outer_id = view_canvas.create_window((0, 0), window=outer, anchor="nw")

        def update_scroll_region(_event=None):
            view_canvas.configure(scrollregion=view_canvas.bbox("all"))

        def fit_content_width(event):
            view_canvas.itemconfigure(outer_id, width=event.width)

        outer.bind("<Configure>", update_scroll_region)
        view_canvas.bind("<Configure>", fit_content_width)
        header = ttk.Frame(outer)
        header.pack(fill="x")
        ttk.Label(header, text="导航质量评分", font=("Sans", 18, "bold")).pack(side="left")
        ttk.Label(header, textvariable=self.state_var).pack(side="right", pady=5)

        score_frame = ttk.Frame(outer)
        score_frame.pack(fill="x", pady=(16, 12))
        ttk.Label(score_frame, textvariable=self.score_var, font=("Sans", 34, "bold")).pack(side="left")
        ttk.Label(score_frame, text=" / 100", font=("Sans", 15)).pack(side="left", pady=(18, 0))
        self.summary_var = tk.StringVar(value="尚未收到评分数据")
        ttk.Label(score_frame, textvariable=self.summary_var, wraplength=330, justify="left").pack(
            side="left", padx=(24, 0), pady=4
        )

        ttk.Separator(outer).pack(fill="x", pady=(0, 10))
        ttk.Label(outer, text="综合分项（0~100%）", font=("Sans", 11, "bold")).pack(anchor="w")
        components = ttk.Frame(outer)
        components.pack(fill="x", pady=(8, 10))
        for row, (key, label) in enumerate(self.COMPONENT_NAMES):
            ttk.Label(components, text=label, width=17).grid(row=row, column=0, sticky="w", pady=3)
            bar = ttk.Progressbar(components, maximum=100, length=270)
            bar.grid(row=row, column=1, sticky="ew", padx=(6, 10), pady=3)
            value = tk.StringVar(value="--")
            ttk.Label(components, textvariable=value, width=8).grid(row=row, column=2, sticky="e", pady=3)
            self.bars[key] = bar
            self.detail_vars[key] = value
        components.columnconfigure(1, weight=1)

        details = ttk.LabelFrame(outer, text="关键观测值", padding=10)
        details.pack(fill="x", pady=(4, 0))
        detail_items = (
            ("mean_tracking_error_m", "平均跟踪误差", "m"),
            ("min_obstacle_distance_m", "最小障碍距离", "m"),
            ("mean_scan_spike_rate", "激光尖峰率", ""),
            ("mean_stationary_scan_jitter_m", "静止扫描抖动", "m"),
            ("replans", "全局重规划次数", ""),
            ("local_replans", "局部重规划次数", ""),
            ("duration_s", "导航耗时", "s"),
            ("final_position_error_m", "最终位置误差", "m"),
        )
        for index, (key, label, unit) in enumerate(detail_items):
            row, column = divmod(index, 2)
            var = tk.StringVar(value="--")
            self.detail_vars["detail_" + key] = var
            ttk.Label(details, text=label).grid(row=row, column=column * 2, sticky="w", padx=(0, 5), pady=3)
            ttk.Label(details, textvariable=var, width=13).grid(row=row, column=column * 2 + 1, sticky="w", padx=(0, 18), pady=3)
        self.unit_map = {"detail_" + key: unit for key, _, unit in detail_items}

        ttk.Label(outer, text="当前任务纵向曲线", font=("Sans", 11, "bold")).pack(anchor="w", pady=(12, 4))
        self.chart_canvas = tk.Canvas(
            outer, height=370, background="#fbfbfb", highlightthickness=1,
            highlightbackground="#c7c7c7",
        )
        self.chart_canvas.pack(fill="x", expand=False)
        ttk.Label(outer, text="历次测试总分趋势（读取 scores/，用于调参前后对比）", font=("Sans", 11, "bold")).pack(
            anchor="w", pady=(12, 4)
        )
        self.history_canvas = tk.Canvas(
            outer, height=145, background="#fbfbfb", highlightthickness=1,
            highlightbackground="#c7c7c7",
        )
        self.history_canvas.pack(fill="x", expand=False)

        ttk.Label(
            outer,
            text="仿真模式使用 chassis_odometry_gt 作为真值；实车模式会隐藏真值项，不因缺失真值扣分。",
            foreground="#555555",
            wraplength=590,
            justify="left",
        ).pack(anchor="w", pady=(12, 0))

    def show(self):
        self.window.deiconify()
        self.window.lift()
        self._draw_history_curve()

    def update(self, result, finished=False):
        self._append_series(result)
        score = result.get("score")
        self.score_var.set(f"{score:.1f}" if isinstance(score, (int, float)) else "--")
        success = result.get("success")
        if finished:
            state = "导航成功" if success else "导航失败/未完成"
        elif result.get("samples", 0):
            state = "实时采集中"
        else:
            state = "等待导航数据"
        self.state_var.set(state)
        environment = "仿真" if result.get("environment") == "simulation" else "实车"
        self.summary_var.set(f"环境：{environment}    样本：{result.get('samples', 0)}\n"
                             f"成功状态：{('成功' if success else '失败') if success is not None else '进行中'}")
        components = result.get("components") or {}
        for key, _label in self.COMPONENT_NAMES:
            value = components.get(key)
            if isinstance(value, (int, float)):
                self.bars[key]["value"] = max(0.0, min(100.0, value * 100.0))
                self.detail_vars[key].set(f"{value * 100.0:.1f}%")
            else:
                self.bars[key]["value"] = 0
                self.detail_vars[key].set("--")
        for key, unit in self.unit_map.items():
            value = result.get(key.removeprefix("detail_"))
            if isinstance(value, (int, float)):
                self.detail_vars[key].set(f"{value:.3f} {unit}".strip())
            else:
                self.detail_vars[key].set("--")
        if finished and isinstance(score, (int, float)):
            self.last_finished_score = float(score)
        self._draw_current_curves()
        self._draw_history_curve()

    def get_series(self):
        return list(self.series)

    def _append_series(self, result):
        values = {key: result.get(key) for key, *_ in self.CURVE_METRICS}
        values["samples"] = result.get("samples")
        values["duration_s"] = result.get("duration_s")
        if self.series:
            previous = self.series[-1]
            signature = tuple(values.get(key) for key in (*[m[0] for m in self.CURVE_METRICS], "samples", "duration_s"))
            previous_signature = tuple(previous.get(key) for key in (*[m[0] for m in self.CURVE_METRICS], "samples", "duration_s"))
            if signature == previous_signature:
                return
        self.series.append(values)

    @staticmethod
    def _finite(value):
        return isinstance(value, (int, float)) and math.isfinite(value)

    def _draw_current_curves(self):
        canvas = self.chart_canvas
        canvas.delete("all")
        width = max(canvas.winfo_width(), 620)
        height = 370
        row_height = height / len(self.CURVE_METRICS)
        left, right = 118, width - 18
        for row, (key, label, lower, upper, color) in enumerate(self.CURVE_METRICS):
            top = row * row_height
            bottom = (row + 1) * row_height - 8
            canvas.create_text(8, top + 12, text=label, anchor="w", fill="#333333")
            canvas.create_line(left, bottom, right, bottom, fill="#dddddd")
            canvas.create_line(left, top + 22, left, bottom, fill="#dddddd")
            values = [item.get(key) for item in self.series]
            finite = [float(value) for value in values if self._finite(value)]
            if not finite:
                canvas.create_text((left + right) / 2, (top + bottom) / 2, text="等待数据", fill="#888888")
                continue
            min_value = lower if upper is not None else min(lower, min(finite))
            max_value = upper if upper is not None else max(max(finite) * 1.15, min_value + 0.05)
            if max_value <= min_value:
                max_value = min_value + 1.0
            canvas.create_text(left - 6, top + 24, text=f"{max_value:.2f}", anchor="e", fill="#777777")
            canvas.create_text(left - 6, bottom, text=f"{min_value:.2f}", anchor="e", fill="#777777")
            previous = None
            count = max(1, len(values) - 1)
            for index, value in enumerate(values):
                if not self._finite(value):
                    previous = None
                    continue
                x = left + (right - left) * index / count
                y = bottom - (float(value) - min_value) / (max_value - min_value) * (bottom - top - 28)
                if previous is not None:
                    canvas.create_line(previous[0], previous[1], x, y, fill=color, width=2)
                canvas.create_oval(x - 2, y - 2, x + 2, y + 2, fill=color, outline=color)
                previous = (x, y)
            canvas.create_text(right, top + 12, text=f"n={len(values)}", anchor="e", fill="#777777")

    def _draw_history_curve(self):
        canvas = self.history_canvas
        canvas.delete("all")
        width = max(canvas.winfo_width(), 620)
        height = 145
        left, right, top, bottom = 52, width - 18, 18, height - 28
        scores = []
        history_dir = getattr(self.owner, "score_history_dir", None)
        if history_dir is not None:
            for path in sorted(history_dir.glob("score_*.json"), key=lambda item: item.stat().st_mtime):
                try:
                    payload = json.loads(path.read_text(encoding="utf-8"))
                    value = payload.get("score")
                    if self._finite(value):
                        scores.append((path.stem[-12:], float(value)))
                except (OSError, json.JSONDecodeError):
                    continue
        if self.last_finished_score is not None:
            scores.append(("当前", self.last_finished_score))
        if not scores:
            canvas.create_text(width / 2, height / 2, text="完成一次评分后显示历史曲线", fill="#888888")
            return
        scores = scores[-20:]
        canvas.create_line(left, bottom, right, bottom, fill="#dddddd")
        canvas.create_line(left, top, left, bottom, fill="#dddddd")
        canvas.create_text(left - 6, top, text="100", anchor="e", fill="#777777")
        canvas.create_text(left - 6, bottom, text="0", anchor="e", fill="#777777")
        count = max(1, len(scores) - 1)
        previous = None
        for index, (label, score) in enumerate(scores):
            x = left + (right - left) * index / count
            y = bottom - max(0.0, min(100.0, score)) / 100.0 * (bottom - top)
            if previous is not None:
                canvas.create_line(previous[0], previous[1], x, y, fill="#1769aa", width=2)
            canvas.create_oval(x - 3, y - 3, x + 3, y + 3, fill="#1769aa", outline="#1769aa")
            canvas.create_text(x, bottom + 10, text=label[-8:], anchor="n", fill="#555555")
            previous = (x, y)


def safe_name(value):
    value = re.sub(r"[^0-9A-Za-z_\-\u4e00-\u9fff]+", "_", value.strip())
    return value.strip("._") or f"goal_{time.strftime('%Y%m%d_%H%M%S')}"


def build_route_payload(name, point_files):
    """Build the versioned route document from ordered standalone goal files."""
    points = []
    route_frame = None
    for path in point_files:
        data = json.loads(path.read_text(encoding="utf-8"))
        frame = data.get("frame")
        if frame not in ("map", "odom") or not all(
            key in data for key in ("x", "y")
        ):
            raise ValueError(f"{path.name} 不是有效的单点文件")
        if route_frame is None:
            route_frame = frame
        elif frame != route_frame:
            raise ValueError("同一条路线不能混用 map 与 odom 坐标点")
        x = float(data["x"])
        y = float(data["y"])
        yaw = float(data.get("yaw", 0.0))
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            raise ValueError(f"{path.name} 的坐标必须是有限数字")
        points.append(
            {
                "name": path.stem,
                "x": x,
                "y": y,
                "yaw": yaw,
            }
        )
    if not points:
        raise ValueError("路线至少需要一个导航点")
    return {
        "format": ROUTE_FORMAT,
        "name": name,
        "frame": route_frame,
        "waypoints": points,
    }


class GoalLauncher:
    def __init__(
        self,
        root,
        namespace,
        environment,
        goal_dir,
        use_sim_time,
        start_mode,
        game_status_topic,
        confirm_before_start,
        enable_scoring=False,
    ):
        self.root = root
        self.namespace = namespace.strip("/")
        self.environment = environment
        self.goal_dir = Path(goal_dir).expanduser()
        self.use_sim_time = bool(use_sim_time)
        self.start_mode = start_mode
        self.game_status_topic = game_status_topic
        self.confirm_before_start = bool(confirm_before_start)
        self.goal_dir.mkdir(parents=True, exist_ok=True)
        self.route_dir = self.goal_dir / "routes"
        self.route_dir.mkdir(parents=True, exist_ok=True)
        self.score_history_dir = self.goal_dir / "scores"
        self.score_history_dir.mkdir(parents=True, exist_ok=True)
        self.child = None
        self.score_child = None
        self.score_result_file = None
        self.score_report_file = None
        self.score_started = False
        self.score_started_at = None
        self.score_dashboard = None
        self.pending_file = None
        self.name_prompt_open = False
        self.closing = False

        root.title("在线建图导航目标")
        root.geometry("620x410")
        root.resizable(False, False)
        root.protocol("WM_DELETE_WINDOW", self.close)

        frame = ttk.Frame(root, padding=24)
        frame.pack(fill="both", expand=True)
        ttk.Label(
            frame,
            text="在线建图导航",
            font=("Sans", 18, "bold"),
        ).pack(pady=(0, 8))
        ttk.Label(
            frame,
            text="可录入单点，也可将已有单点可视化排序为顺序巡航路线。",
            wraplength=550,
            justify="center",
        ).pack(pady=(0, 18))

        buttons = ttk.Frame(frame)
        buttons.pack()
        self.score_enabled = tk.BooleanVar(value=enable_scoring)
        ttk.Checkbutton(
            frame,
            text="启用本次导航评分（实时显示面板，结束后保存结果）",
            variable=self.score_enabled,
        ).pack(pady=(0, 8))
        ttk.Button(
            frame,
            text="打开评分可视化面板",
            command=self.open_score_dashboard,
        ).pack(pady=(0, 8))
        ttk.Button(
            buttons,
            text="录入并导航",
            command=lambda: self.record_goal(record_only=False),
            width=22,
        ).grid(row=0, column=0, padx=8, pady=8)
        ttk.Button(
            buttons,
            text="使用已保存目标",
            command=self.choose_saved_goal,
            width=22,
        ).grid(row=0, column=1, padx=8, pady=8)
        ttk.Button(
            buttons,
            text="仅录入目标点（不导航）",
            command=lambda: self.record_goal(record_only=True),
            width=22,
        ).grid(row=1, column=0, padx=8, pady=8)
        ttk.Button(
            buttons,
            text="编排顺序巡航路线",
            command=self.open_route_editor,
            width=22,
        ).grid(row=1, column=1, padx=8, pady=8)
        ttk.Button(
            buttons,
            text="使用已保存路线",
            command=self.choose_saved_route,
            width=22,
        ).grid(row=2, column=0, columnspan=2, padx=8, pady=8)

        self.status = ttk.Label(frame, text=f"保存目录：{self.goal_dir}", wraplength=470)
        self.status.pack(pady=(24, 0))

    def ros_prefix(self):
        ros_args = ["--ros-args"]
        if self.namespace:
            ns = f"/{self.namespace}"
            ros_args.extend(
                [
                    "-r",
                    "__ns:=" + ns,
                    "-r",
                    "/tf:=" + ns + "/tf",
                    "-r",
                    "/tf_static:=" + ns + "/tf_static",
                ]
            )
        ros_args.extend(
            ["-p", f"use_sim_time:={'true' if self.use_sim_time else 'false'}"]
        )
        return ros_args

    def start_core(self, extra_args):
        command = [
            sys.executable,
            str(CORE_SCRIPT),
            *extra_args,
            "--start-mode",
            self.start_mode,
            "--game-status-topic",
            self.game_status_topic,
            *self.ros_prefix(),
        ]
        self.status.config(text="已启动 ROS 目标节点，请查看终端输出和 RViz。")
        self.child = subprocess.Popen(command, cwd=str(PROJECT_ROOT))
        self.root.after(500, self.watch_navigation)

    def watch_navigation(self):
        if self.child is None:
            return
        if self.child.poll() is None:
            self.root.after(500, self.watch_navigation)
            return
        self.child = None
        if self.score_child is not None:
            self.stop_scorer()

    def _score_topics(self):
        prefix = f"/{self.namespace}" if self.namespace else ""
        return {
            "odom": prefix + "/odometry",
            "cmd": prefix + "/cmd_vel_nav2_result",
            "actual_cmd": prefix + "/cmd_vel",
            "path": prefix + "/plan",
            "local_path": prefix + "/local_plan",
            "scan": prefix + "/obstacle_scan",
            "action_status": prefix + "/navigate_to_pose/_action/status",
        }

    def start_scorer(self, goal_data, goal_frame=None):
        if not self.score_enabled.get() or self.score_child is not None:
            return
        try:
            goal_x = float(goal_data["x"])
            goal_y = float(goal_data["y"])
        except (KeyError, TypeError, ValueError):
            self.status.config(text="目标已启动，但目标坐标无效，未启动评分器。")
            return
        topics = self._score_topics()
        result_file = self.goal_dir / f".score_{os.getpid()}.json"
        report_file = self.goal_dir / f".score_live_{os.getpid()}.json"
        if result_file.exists():
            result_file.unlink()
        if report_file.exists():
            report_file.unlink()
        self.score_result_file = result_file
        self.score_report_file = report_file
        command = [
            sys.executable, str(SCORER_SCRIPT), "score_live",
            "--namespace", self.namespace,
            "--environment", self.environment,
            "--goal-x", str(goal_x), "--goal-y", str(goal_y),
            "--odom-topic", topics["odom"], "--cmd-topic", topics["cmd"],
            "--actual-cmd-topic", topics["actual_cmd"],
            "--path-topic", topics["path"], "--local-path-topic", topics["local_path"],
            "--scan-topic", topics["scan"],
            "--action-status-topic", topics["action_status"],
            "--global-frame", "map", "--nav-base-frame", "gimbal_yaw_fake",
            "--goal-frame", str(goal_frame or goal_data.get("frame", "odom")),
            "--live-report",
            "--result-file", str(result_file),
            "--live-report-file", str(report_file),
        ]
        self.score_child = subprocess.Popen(
            command, cwd=str(PROJECT_ROOT),
            stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT,
        )
        self.score_started = True
        self.score_started_at = f"{time.strftime('%Y%m%d_%H%M%S')}_{time.time_ns()}"
        self.open_score_dashboard()
        self._poll_score_report()

    def open_score_dashboard(self):
        if self.score_dashboard is None or not self.score_dashboard.window.winfo_exists():
            self.score_dashboard = ScoreDashboard(self)
        self.score_dashboard.show()

    def _poll_score_report(self):
        report_file = self.score_report_file
        if report_file is not None and report_file.exists():
            try:
                result = json.loads(report_file.read_text(encoding="utf-8"))
                self.open_score_dashboard()
                self.score_dashboard.update(result)
            except (OSError, json.JSONDecodeError, tk.TclError):
                pass
        if self.score_child is not None and self.score_child.poll() is None:
            self.root.after(500, self._poll_score_report)

    def stop_scorer(self):
        child = self.score_child
        self.score_child = None
        if child is not None and child.poll() is None:
            try:
                child.send_signal(signal.SIGINT)
                child.wait(timeout=4)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                if child.poll() is None:
                    child.terminate()
                    try:
                        child.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        child.kill()
                        child.wait()
        result_file = self.score_result_file
        report_file = self.score_report_file
        self.score_result_file = None
        self.score_report_file = None
        self.score_started = False
        result = None
        try:
            if result_file is not None and result_file.exists():
                result = json.loads(result_file.read_text(encoding="utf-8"))
            elif report_file is not None and report_file.exists():
                result = json.loads(report_file.read_text(encoding="utf-8"))
            if result is not None:
                self.open_score_dashboard()
                self.score_dashboard.update(result, finished=True)
                score = result.get("score")
                score_text = f"{score:.1f}/100" if isinstance(score, (int, float)) else "暂无分数"
                state = "成功" if result.get("success") else "失败/未完成"
                self.status.config(text=f"本次导航：{state}，综合评分 {score_text}")
                history_name = f"score_{self.score_started_at or time.strftime('%Y%m%d_%H%M%S')}_{os.getpid()}.json"
                history_file = self.score_history_dir / history_name
                history_payload = dict(result)
                history_payload.update(
                    {
                        "gui_environment": self.environment,
                        "gui_namespace": self.namespace,
                        "completed_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
                        "series": (
                            self.score_dashboard.get_series()
                            if self.score_dashboard is not None
                            else []
                        ),
                    }
                )
                try:
                    history_file.write_text(
                        json.dumps(history_payload, ensure_ascii=False, indent=2) + "\n",
                        encoding="utf-8",
                    )
                    self.status.config(text=f"本次导航：{state}，综合评分 {score_text}，已保存 {history_file.name}")
                except OSError:
                    pass
                messagebox.showinfo(
                    "导航评分",
                    f"综合评分：{score_text}\n结果：{state}\n"
                    f"平均跟踪误差：{result.get('mean_tracking_error_m')} m\n"
                    f"最小障碍距离：{result.get('min_obstacle_distance_m')} m\n"
                    f"实际路径长度：{result.get('path_length_m')} m",
                    parent=self.root,
                )
        except (OSError, json.JSONDecodeError, tk.TclError):
            self.status.config(text="导航结束，但评分结果无法读取。")
        try:
            result_file.unlink()
        except OSError:
            pass
        if report_file is not None:
            try:
                report_file.unlink()
            except OSError:
                pass

    def stop_child(self):
        child = self.child
        self.child = None
        if child is None or child.poll() is not None:
            self.stop_scorer()
            return

        try:
            child.send_signal(signal.SIGINT)
        except ProcessLookupError:
            pass
        if child.poll() is None:
            try:
                child.wait(timeout=3)
            except subprocess.TimeoutExpired:
                child.terminate()
        if child.poll() is None:
            try:
                child.wait(timeout=2)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()
        self.stop_scorer()

    def close(self):
        if self.closing:
            return
        self.closing = True
        self.stop_child()
        self.stop_scorer()
        if self.pending_file is not None and self.pending_file.exists():
            self.pending_file.unlink()
        self.pending_file = None
        self.root.destroy()

    def confirm_test_start(self):
        if not self.confirm_before_start:
            return True
        return messagebox.askyesno(
            "实车测试放行确认",
            (
                "当前为实车测试模式，不会等待裁判系统比赛开始信号。\n\n"
                "请确认机器人已架起或位于安全空旷区域、急停和遥控接管可用，"
                "并且周围无人。确认后，导航系统一旦就绪就可能开始移动。\n\n"
                "是否允许本次测试移动？"
            ),
            parent=self.root,
        )

    def saved_goal_files(self):
        return sorted(
            path
            for path in self.goal_dir.glob("*.json")
            if not path.name.startswith(".")
        )

    def saved_route_files(self):
        return sorted(
            path
            for path in self.route_dir.glob("*.json")
            if not path.name.startswith(".")
        )

    def find_saved_goal(self, name):
        requested = name.strip()
        if requested.endswith(".json"):
            requested = requested[:-5]
        for path in self.saved_goal_files():
            if path.stem == requested:
                return path
        return None

    def find_saved_route(self, name):
        requested = name.strip()
        if requested.endswith(".json"):
            requested = requested[:-5]
        for path in self.saved_route_files():
            if path.stem == requested:
                return path
        return None

    def start_saved_goal(self, path):
        if self.child is not None and self.child.poll() is None:
            messagebox.showwarning("目标节点正在运行", "当前已经有一个目标节点在运行。")
            return False
        if not self.confirm_test_start():
            self.status.config(text="已取消实车测试放行，未启动目标节点。")
            return False
        try:
            goal_data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            messagebox.showerror("目标读取失败", str(exc), parent=self.root)
            return False
        self.start_scorer(goal_data)
        self.start_core(["--use-saved-goal", "--goal-file", str(path)])
        self.status.config(text=f"已使用目标：{path.name}，等待地图和 TF 后自动导航。")
        return True

    def start_saved_goal_by_name(self, name):
        path = self.find_saved_goal(name)
        if path is None:
            available = "、".join(item.stem for item in self.saved_goal_files())
            detail = f"\n当前可用目标：{available}" if available else "\n当前没有已保存目标。"
            self.status.config(text=f"未找到已保存目标：{name}")
            messagebox.showerror(
                "找不到目标点",
                f"保存目录中不存在目标“{name}”。{detail}",
                parent=self.root,
            )
            return False
        return self.start_saved_goal(path)

    def start_saved_route(self, path):
        if self.child is not None and self.child.poll() is None:
            messagebox.showwarning("目标节点正在运行", "当前已经有一个目标节点在运行。")
            return False
        if not self.confirm_test_start():
            self.status.config(text="已取消实车测试放行，未启动路线节点。")
            return False
        try:
            route = json.loads(path.read_text(encoding="utf-8"))
            waypoints = route.get("waypoints", [])
            if waypoints:
                self.start_scorer(waypoints[-1], route.get("frame", "odom"))
        except (OSError, json.JSONDecodeError, TypeError):
            pass
        self.start_core(["--use-saved-route", "--route-file", str(path)])
        self.status.config(text=f"已使用路线：{path.name}，将按顺序逐点导航。")
        return True

    def start_saved_route_by_name(self, name):
        path = self.find_saved_route(name)
        if path is None:
            available = "、".join(item.stem for item in self.saved_route_files())
            detail = f"\n当前可用路线：{available}" if available else "\n当前没有已保存路线。"
            self.status.config(text=f"未找到已保存路线：{name}")
            messagebox.showerror(
                "找不到巡航路线",
                f"路线目录中不存在“{name}”。{detail}",
                parent=self.root,
            )
            return False
        return self.start_saved_route(path)

    def record_goal(self, record_only=False):
        if self.child is not None and self.child.poll() is None:
            messagebox.showwarning("目标节点正在运行", "当前已经有一个目标节点在运行。")
            return
        if not record_only and not self.confirm_test_start():
            self.status.config(text="已取消实车测试放行，未启动目标节点。")
            return
        pending = self.goal_dir / f".pending_{os.getpid()}.json"
        if pending.exists():
            pending.unlink()
        self.pending_file = pending
        self.pending_record_only = record_only
        self.name_prompt_open = False
        core_args = [
            "--wait-click",
            "--goal-frame",
            "odom",
            "--goal-file",
            str(pending),
        ]
        if record_only:
            core_args.append("--record-only")
        self.start_core(core_args)
        self.status.config(
            text=(
                "请在 RViz 使用 Publish Point 点击位置（可以位于灰色地图范围外），"
                "点击后会弹出命名窗口。"
            )
        )
        self.root.after(400, self.watch_pending_goal)

    def watch_pending_goal(self):
        if self.pending_file is None or self.name_prompt_open:
            return
        if self.pending_file.exists():
            if not self.pending_record_only and not self.score_started:
                try:
                    self.start_scorer(json.loads(self.pending_file.read_text(encoding="utf-8")))
                except (OSError, json.JSONDecodeError):
                    pass
            self.name_prompt_open = True
            self.root.after(50, self.name_pending_goal)
            return
        if self.child is not None and self.child.poll() is not None:
            self.status.config(text="目标节点已退出，没有检测到点击目标。")
            return
        self.root.after(400, self.watch_pending_goal)

    def name_pending_goal(self):
        default = f"goal_{time.strftime('%Y%m%d_%H%M%S')}"
        name = simpledialog.askstring(
            "保存目标点",
            "请输入目标点名称：",
            initialvalue=default,
            parent=self.root,
        )
        if not name:
            name = default
        target = self.goal_dir / f"{safe_name(name)}.json"
        if target.exists():
            overwrite = messagebox.askyesno(
                "目标已存在",
                f"{target.name} 已存在，是否覆盖？",
                parent=self.root,
            )
            if not overwrite:
                target = self.goal_dir / (
                    f"{safe_name(name)}_{time.strftime('%H%M%S')}.json"
                )
        shutil.move(str(self.pending_file), str(target))
        self.pending_file = None
        self.name_prompt_open = False
        if self.pending_record_only:
            self.status.config(text=f"已保存目标：{target.name}，未发送导航目标。")
        else:
            self.status.config(text=f"已保存目标：{target.name}。本次导航继续执行。")

    def choose_saved_goal(self):
        if self.child is not None and self.child.poll() is None:
            messagebox.showwarning("目标节点正在运行", "当前已经有一个目标节点在运行。")
            return
        files = self.saved_goal_files()
        if not files:
            messagebox.showinfo("没有保存点", "保存目录中还没有目标点，请先选择“录入新目标点”。")
            return

        dialog = tk.Toplevel(self.root)
        dialog.title("选择已保存目标")
        dialog.geometry("420x300")
        dialog.transient(self.root)
        ttk.Label(dialog, text="选择本次要导航的目标点：").pack(pady=10)
        listbox = tk.Listbox(dialog, height=10, width=48)
        listbox.pack(padx=16, fill="both", expand=True)
        for path in files:
            listbox.insert(tk.END, path.stem)
        listbox.selection_set(0)

        def start_selected():
            selection = listbox.curselection()
            if not selection:
                return
            chosen = files[selection[0]]
            dialog.destroy()
            self.start_saved_goal(chosen)

        ttk.Button(dialog, text="开始导航", command=start_selected).pack(pady=10)

    def choose_saved_route(self):
        if self.child is not None and self.child.poll() is None:
            messagebox.showwarning("目标节点正在运行", "当前已经有一个目标节点在运行。")
            return
        files = self.saved_route_files()
        if not files:
            messagebox.showinfo("没有巡航路线", "请先选择“编排顺序巡航路线”。")
            return

        dialog = tk.Toplevel(self.root)
        dialog.title("选择已保存巡航路线")
        dialog.geometry("440x320")
        dialog.transient(self.root)
        ttk.Label(dialog, text="选择要执行的顺序巡航路线：").pack(pady=10)
        listbox = tk.Listbox(dialog, height=11, width=50)
        listbox.pack(padx=16, fill="both", expand=True)
        for path in files:
            listbox.insert(tk.END, path.stem)
        listbox.selection_set(0)

        def start_selected():
            selection = listbox.curselection()
            if not selection:
                return
            chosen = files[selection[0]]
            dialog.destroy()
            self.start_saved_route(chosen)

        ttk.Button(dialog, text="开始顺序导航", command=start_selected).pack(pady=10)

    def open_route_editor(self):
        point_files = self.saved_goal_files()
        if not point_files:
            messagebox.showinfo(
                "没有单点",
                "请先录入一个或多个独立目标点，再编排巡航路线。",
                parent=self.root,
            )
            return

        dialog = tk.Toplevel(self.root)
        dialog.title("编排顺序巡航路线")
        dialog.geometry("780x500")
        dialog.transient(self.root)
        dialog.grab_set()

        content = ttk.Frame(dialog, padding=14)
        content.pack(fill="both", expand=True)
        ttk.Label(
            content,
            text=(
                "左侧为已录制单点，添加到右侧后可调整先后顺序；"
                "同一点可以重复添加。第一个点必须位于初始可导航范围内。"
            ),
            wraplength=730,
        ).grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 10))

        ttk.Label(content, text="独立导航点").grid(row=1, column=0)
        ttk.Label(content, text="路线执行顺序").grid(row=1, column=2)
        available = tk.Listbox(content, selectmode=tk.EXTENDED, width=30, height=17)
        ordered_box = tk.Listbox(content, width=38, height=17)
        available.grid(row=2, column=0, sticky="nsew", padx=(0, 10))
        ordered_box.grid(row=2, column=2, sticky="nsew", padx=(10, 0))
        for path in point_files:
            available.insert(tk.END, path.stem)

        ordered = []

        def refresh_ordered(select_index=None):
            ordered_box.delete(0, tk.END)
            for index, path in enumerate(ordered, start=1):
                ordered_box.insert(tk.END, f"{index:02d}. {path.stem}")
            if select_index is not None and ordered:
                select_index = max(0, min(select_index, len(ordered) - 1))
                ordered_box.selection_set(select_index)
                ordered_box.see(select_index)

        def add_selected(_event=None):
            selections = available.curselection()
            for index in selections:
                ordered.append(point_files[index])
            refresh_ordered(len(ordered) - 1 if ordered else None)

        def remove_selected():
            selection = ordered_box.curselection()
            if not selection:
                return
            index = selection[0]
            ordered.pop(index)
            refresh_ordered(min(index, len(ordered) - 1) if ordered else None)

        def move_selected(offset):
            selection = ordered_box.curselection()
            if not selection:
                return
            index = selection[0]
            new_index = index + offset
            if new_index < 0 or new_index >= len(ordered):
                return
            ordered[index], ordered[new_index] = ordered[new_index], ordered[index]
            refresh_ordered(new_index)

        controls = ttk.Frame(content)
        controls.grid(row=2, column=1, sticky="n", pady=20)
        ttk.Button(controls, text="添加 →", command=add_selected, width=10).pack(pady=4)
        ttk.Button(controls, text="移除", command=remove_selected, width=10).pack(pady=4)
        ttk.Button(
            controls, text="上移", command=lambda: move_selected(-1), width=10
        ).pack(pady=4)
        ttk.Button(
            controls, text="下移", command=lambda: move_selected(1), width=10
        ).pack(pady=4)
        available.bind("<Double-Button-1>", add_selected)

        bottom = ttk.Frame(content)
        bottom.grid(row=3, column=0, columnspan=3, sticky="ew", pady=(14, 0))
        ttk.Label(bottom, text="路线名称：").pack(side="left")
        route_name = tk.StringVar(value=f"route_{time.strftime('%Y%m%d_%H%M%S')}")
        ttk.Entry(bottom, textvariable=route_name, width=34).pack(
            side="left", padx=(0, 12)
        )

        def save_route():
            if not ordered:
                messagebox.showwarning("路线为空", "请至少添加一个导航点。", parent=dialog)
                return
            name = safe_name(route_name.get())
            try:
                payload = build_route_payload(name, ordered)
            except (OSError, json.JSONDecodeError, TypeError, ValueError) as exc:
                messagebox.showerror("路线生成失败", str(exc), parent=dialog)
                return

            target = self.route_dir / f"{name}.json"
            if target.exists() and not messagebox.askyesno(
                "路线已存在", f"{target.name} 已存在，是否覆盖？", parent=dialog
            ):
                return
            try:
                target.write_text(
                    json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
                    encoding="utf-8",
                )
            except OSError as exc:
                messagebox.showerror("路线保存失败", str(exc), parent=dialog)
                return
            self.status.config(
                text=(
                    f"已保存路线：routes/{target.name}，"
                    f"共 {len(payload['waypoints'])} 个点。"
                )
            )
            dialog.destroy()

        ttk.Button(bottom, text="保存路线", command=save_route).pack(side="right")
        content.columnconfigure(0, weight=1)
        content.columnconfigure(2, weight=1)
        content.rowconfigure(2, weight=1)


def main():
    parser = argparse.ArgumentParser(description="在线建图导航目标可视化管理器")
    parser.add_argument(
        "--environment", choices=tuple(ENVIRONMENT_DEFAULTS), default="simulation",
        help="运行环境；仿真默认 namespace 为 red_standard_robot1，实车默认为空",
    )
    parser.add_argument(
        "--namespace", default=None, help="覆盖环境默认的 ROS 命名空间"
    )
    parser.add_argument(
        "--use-sim-time",
        choices=("true", "false"),
        default=None,
        help="是否使用 /clock；仿真用 true，实车用 false",
    )
    parser.add_argument(
        "--start-mode",
        choices=("immediate", "referee"),
        default="immediate",
        help="目标发送门控：立即发送，或等待裁判系统 RUNNING",
    )
    parser.add_argument(
        "--game-status-topic",
        default="/referee/game_status",
        help="pb_rm_interfaces/msg/GameStatus 类型的比赛状态话题",
    )
    parser.add_argument(
        "--confirm-before-start",
        action="store_true",
        help="启动目标节点前弹出实车移动安全确认",
    )
    parser.add_argument(
        "--goal-dir",
        default=None,
        help="目标 JSON 目录；默认按 environment 选择 simulation/reality 目录",
    )
    parser.add_argument(
        "--auto-goal",
        metavar="NAME",
        help="打开窗口后自动使用指定名称的已保存目标，无需人工选择",
    )
    parser.add_argument(
        "--auto-route",
        metavar="NAME",
        help="打开窗口后自动使用指定名称的顺序巡航路线",
    )
    parser.add_argument(
        "--enable-scoring",
        action="store_true",
        help="启动时勾选导航评分；不会单独启动导航",
    )
    parser.add_argument(
        "--list-goals",
        action="store_true",
        help="输出固定目录中的已保存目标名称并退出",
    )
    parser.add_argument(
        "--list-routes",
        action="store_true",
        help="输出 routes 目录中的已保存路线名称并退出",
    )
    args = parser.parse_args()

    if args.auto_goal and args.auto_route:
        parser.error("--auto-goal 和 --auto-route 不能同时使用")

    environment_defaults = ENVIRONMENT_DEFAULTS[args.environment]
    namespace = (args.namespace if args.namespace is not None else environment_defaults["namespace"])
    use_sim_time = (
        args.use_sim_time == "true"
        if args.use_sim_time is not None
        else environment_defaults["use_sim_time"]
    )
    goal_dir = Path(args.goal_dir or environment_defaults["goal_dir"]).expanduser()
    goal_dir.mkdir(parents=True, exist_ok=True)
    if args.list_goals:
        for path in sorted(goal_dir.glob("*.json")):
            if not path.name.startswith("."):
                print(path.stem)
        return
    if args.list_routes:
        route_dir = goal_dir / "routes"
        for path in sorted(route_dir.glob("*.json")):
            if not path.name.startswith("."):
                print(path.stem)
        return

    root = tk.Tk()
    launcher = GoalLauncher(
        root,
        namespace,
        args.environment,
        goal_dir,
        use_sim_time=use_sim_time,
        start_mode=args.start_mode,
        game_status_topic=args.game_status_topic,
        confirm_before_start=args.confirm_before_start,
        enable_scoring=args.enable_scoring,
    )
    if args.auto_goal:
        root.after(150, lambda: launcher.start_saved_goal_by_name(args.auto_goal))
    elif args.auto_route:
        root.after(150, lambda: launcher.start_saved_route_by_name(args.auto_route))
    try:
        root.mainloop()
    except KeyboardInterrupt:
        launcher.close()
    finally:
        launcher.stop_child()


if __name__ == "__main__":
    main()
