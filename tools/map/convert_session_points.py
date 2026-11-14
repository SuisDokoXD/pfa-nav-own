#!/usr/bin/env python3
"""把一次建图会话里的 odom 点转换成绑定最终地图的 map 点。

背景：
    建图期 GUI 录的是 odom 系临时点（`runtime/sessions/<id>/points/*.json`）。
    建图结束、地图定稿后，需要把它们变成带 `map_id` 的 map 点，才能在先验导航里复用。
    建图期间 `/map` 每 0.2 s 刷新，那时写下的 map_id 会失效，所以转换必须**在地图定稿之后**做，
    并以**当前运行中的 map_server** 发布的 `/map` 作为权威身份来源。

用法（先按 tools/map/README.md 用 annotate_map.sh 起好地图）：
    python3 tools/map/convert_session_points.py --session-id <id> [--out DIR] [--force]
    python3 tools/map/convert_session_points.py --session-id <id> --map-odom "0.5 -0.2 1.5"

参数：
    --session-id   runtime/sessions 下的会话目录名
    --sessions-root 会话根目录，默认 <repo>/runtime/sessions
    --out         输出目录，默认 <repo>/online_slam_goals/reality（与离线 GUI 录点同级）
    --map-topic   /map 话题名，默认 /map
    --map-odom    "dx dy dyaw_deg"，建图会话的 map→odom 变换；SLAM 模式通常就是 0 0 0
    --timeout     等待 /map 的秒数，默认 10
    --force       允许覆盖输出目录中已存在的同名文件

注意：
    - 只处理 frame == "odom" 的点；已是 map 的点会跳过并提示。
    - 不做碰撞/可达性判断，只做坐标与身份转换。
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "auto_nav"))

from route_contract import map_metadata_from_occupancy  # noqa: E402


class MapProbe(Node):
    """取一次 /map 的元数据（/map 是 latched 话题，必须用 transient_local 订阅）。"""

    def __init__(self, topic: str):
        super().__init__("convert_session_points_probe")
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.metadata = None
        self.create_subscription(OccupancyGrid, topic, self._on_map, qos)

    def _on_map(self, msg: OccupancyGrid) -> None:
        try:
            self.metadata = map_metadata_from_occupancy(msg.info, msg.data)
        except (TypeError, ValueError) as exc:
            self.get_logger().error(f"无法从 /map 生成地图身份: {exc}")


def parse_map_odom(text: str):
    parts = text.replace(",", " ").split()
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("--map-odom 需要三个数：dx dy dyaw_deg")
    try:
        dx, dy, dyaw = (float(value) for value in parts)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"--map-odom 解析失败: {exc}") from exc
    return dx, dy, dyaw


def main() -> int:
    parser = argparse.ArgumentParser(description="odom 点 → 绑定最终地图的 map 点")
    parser.add_argument("--session-id", required=True, help="runtime/sessions 下的会话目录名")
    parser.add_argument("--sessions-root", default=str(REPO / "runtime" / "sessions"))
    parser.add_argument("--out", default=None,
                        help="输出目录；默认 online_slam_goals/reality（离线 GUI 直接可见）")
    parser.add_argument("--map-topic", default="/map")
    parser.add_argument("--map-odom", type=parse_map_odom, default=(0.0, 0.0, 0.0),
                        help="建图会话的 map→odom：dx dy dyaw_deg，SLAM 模式一般是 0 0 0")
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()

    points_dir = Path(args.sessions_root) / args.session_id / "points"
    if not points_dir.is_dir():
        print(f"错误：找不到会话点目录 {points_dir}", file=sys.stderr)
        return 2
    sources = sorted(path for path in points_dir.glob("*.json") if not path.name.startswith("."))
    if not sources:
        print(f"错误：{points_dir} 里没有点文件", file=sys.stderr)
        return 2

    out_dir = Path(args.out) if args.out else REPO / "online_slam_goals" / "reality"
    out_dir.mkdir(parents=True, exist_ok=True)

    # 取地图身份
    rclpy.init()
    probe = MapProbe(args.map_topic)
    deadline = probe.get_clock().now().nanoseconds / 1e9 + args.timeout
    try:
        while rclpy.ok() and probe.metadata is None:
            rclpy.spin_once(probe, timeout_sec=0.2)
            if probe.get_clock().now().nanoseconds / 1e9 > deadline:
                break
    finally:
        metadata = probe.metadata
        probe.destroy_node()
        rclpy.shutdown()

    if metadata is None:
        print(
            f"错误：{args.timeout:.0f}s 内没有从 {args.map_topic} 收到地图。\n"
            f"      请先启动最终地图（例如 tools/map/annotate_map.sh --map <最终.yaml>）再执行本脚本。",
            file=sys.stderr,
        )
        return 3

    dx, dy, dyaw_deg = args.map_odom
    cos_yaw, sin_yaw = math.cos(math.radians(dyaw_deg)), math.sin(math.radians(dyaw_deg))

    converted, skipped, existed = 0, 0, 0
    for path in sources:
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            print(f"跳过 {path.name}: 读取失败 {exc}", file=sys.stderr)
            skipped += 1
            continue
        if data.get("frame") != "odom":
            print(f"跳过 {path.name}: frame={data.get('frame')}（只转换 odom 点）", file=sys.stderr)
            skipped += 1
            continue
        try:
            x, y = float(data["x"]), float(data["y"])
        except (KeyError, TypeError, ValueError):
            print(f"跳过 {path.name}: 缺少合法的 x/y", file=sys.stderr)
            skipped += 1
            continue

        x_map = dx + cos_yaw * x - sin_yaw * y
        y_map = dy + sin_yaw * x + cos_yaw * y
        yaw_map = float(data.get("yaw", 0.0)) + math.radians(dyaw_deg)

        target = out_dir / path.name
        if target.exists() and not args.force:
            existed += 1
            continue
        payload = {
            "frame": "map",
            "x": round(x_map, 6),
            "y": round(y_map, 6),
            "yaw": round(yaw_map, 6),
            "map": metadata,
            "source": {"session_id": args.session_id, "from": str(path.name)},
        }
        target.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        converted += 1

    print(f"会话     : {args.session_id}")
    print(f"map_id   : {metadata.get('map_id')}")
    print(f"输入     : {points_dir}（{len(sources)} 个文件）")
    print(f"输出     : {out_dir}")
    print(f"结果     : 转换 {converted}，跳过 {skipped}，已存在未覆盖 {existed}")
    if existed and not args.force:
        print("提示：如需覆盖已存在文件，追加 --force")
    return 0 if converted or existed else 1


if __name__ == "__main__":
    raise SystemExit(main())
