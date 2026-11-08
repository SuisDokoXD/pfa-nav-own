#!/usr/bin/env bash
# 只加载先验地图用于离线标点：map_server + 静态 TF，配合 RViz 的 Publish Point
# 与 auto_nav/online_slam_goal_gui.py 的 map 帧录点。
# 不启动 LIO、不启动 Nav2、不需要实车。
set -Eeo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
MAP_YAML=""
USE_RVIZ=0
BASE_FRAME="base_footprint"
MAP_TOPIC="map"
GOAL_DIR=""
PIDS=()

usage() {
    cat <<'EOF'
用法：
  tools/map/annotate_map.sh --map /path/to/final_map.yaml [选项]

选项：
  --map FILE          必填，map_server 的 yaml（同目录需有对应 .pgm/.png）
  --rviz              同时打开 RViz
  --base-frame FRAME  静态 TF 的子坐标系，默认 base_footprint
  --topic NAME        map_server 发布的话题名，默认 map
  --goal-dir DIR      提示用的目标保存目录（仅打印命令，不影响本脚本）
  -h, --help          显示帮助

说明：
  - 本脚本只把地图变成 /map(OccupancyGrid) 并提供一条 map→base 的静态 TF，
    使 RViz 的 Publish Point 能给出 map 帧坐标。
  - 标点必须在“最终版地图”上做：地图一旦调平/改 origin/替换，map_id 会变，
    旧点位会失效（GUI 会拒绝加载，这是预期行为）。
EOF
}

cleanup() {
    local pid
    for pid in ${PIDS[@]+"${PIDS[@]}"}; do
        kill "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

while [[ $# -gt 0 ]]; do
    case "$1" in
        --map)        MAP_YAML="${2:-}"; shift 2 ;;
        --rviz)       USE_RVIZ=1; shift ;;
        --base-frame) BASE_FRAME="${2:-}"; shift 2 ;;
        --topic)      MAP_TOPIC="${2:-}"; shift 2 ;;
        --goal-dir)   GOAL_DIR="${2:-}"; shift 2 ;;
        -h|--help)    usage; exit 0 ;;
        *) echo "未知参数: $1" >&2; usage; exit 2 ;;
    esac
done

if [[ -z "$MAP_YAML" ]]; then
    echo "错误：必须用 --map 指定地图 yaml。" >&2
    usage
    exit 2
fi
MAP_YAML="$(readlink -f -- "$MAP_YAML" 2>/dev/null || true)"
if [[ ! -f "$MAP_YAML" ]]; then
    echo "错误：找不到地图文件 $MAP_YAML" >&2
    exit 2
fi

# 注意：ROS 的 setup.bash 不能在 set -u 下 source（存在未绑定变量）。
source /opt/ros/humble/setup.bash
if [[ -f "$ROOT/install/setup.bash" ]]; then
    source "$ROOT/install/setup.bash"
else
    echo "警告：未找到 $ROOT/install/setup.bash（工作区是否已 colcon build？）" >&2
fi

# 清掉 ros2 daemon 的陈旧节点缓存：否则 lifecycle 查询可能看到已退出的重名节点
# （现象：node list 出现重复 /map_server，随后 lifecycle get 报 Node not found）
ros2 daemon stop >/dev/null 2>&1 || true

# 防重名：daemon 已重启，这里看到的就是真实存在的节点
if ros2 node list --no-daemon 2>/dev/null | grep -qx "/map_server"; then
    echo "错误：已存在 /map_server 节点，请先结束它再运行本脚本。" >&2
    exit 4
fi

echo "[annotate] 地图: $MAP_YAML"
echo "[annotate] 话题: $MAP_TOPIC   静态 TF: map -> $BASE_FRAME"

ros2 run nav2_map_server map_server --ros-args \
    -p yaml_filename:="$MAP_YAML" \
    -p topic_name:="$MAP_TOPIC" &
PIDS+=("$!")


ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map "$BASE_FRAME" &
PIDS+=("$!")
# 按 map_server 的实际生命周期状态推进（ros2 CLI 每次约 0.5 s，用状态判断而不是死等）
activate_map_server() {
    local attempt state
    for attempt in $(seq 1 40); do
        state="$(timeout 5 ros2 lifecycle get /map_server 2>/dev/null || true)"
        case "$state" in
            *unconfigured*)
                timeout 5 ros2 lifecycle set /map_server configure >/dev/null 2>&1 || true
                ;;
            *inactive*)
                timeout 5 ros2 lifecycle set /map_server activate >/dev/null 2>&1 || true
                ;;
            *active*)
                return 0
                ;;
        esac
        sleep 0.5
    done
    return 1
}
activate_map_server || {
    echo "错误：map_server 未能进入 active 状态，请检查上面的日志。" >&2
    exit 3
}
cat <<EOF

==================== 现在可以标点了 ====================
1) RViz 的 Fixed Frame 必须设为 map（否则 Publish Point 发出的是别的帧，
   录点会被 GUI 拒绝：online_slam_goal.py:333 会校验 header.frame_id）。
2) 用 RViz 的 "Publish Point" 点选目标位置。
3) 另开终端运行录点界面（map 帧）：
     source /opt/ros/humble/setup.bash && source $ROOT/install/setup.bash
     python3 auto_nav/online_slam_goal_gui.py --record-frame map${GOAL_DIR:+ --goal-dir $GOAL_DIR}
4) 点保存后，JSON 会带 map_id；先验导航时若地图不是这张，GUI 会拒绝加载。
Ctrl-C 结束本脚本（会一并关闭 map_server 与静态 TF）。
=======================================================
EOF

if [[ "$USE_RVIZ" -eq 1 ]]; then
    rviz2 &
    PIDS+=("$!")
fi

wait
