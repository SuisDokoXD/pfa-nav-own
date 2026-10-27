#!/usr/bin/env bash
set -Eeuo pipefail

# One-key Gazebo + simulation prior-navigation helper.
# Starts a Gazebo competition world, then starts pb2025 simulation navigation
# using the installed prior map, defaulting to world:=game.

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"

GAZEBO_WORLD="${GAZEBO_WORLD:-rmuc_2027}"
NAV_WORLD="${NAV_WORLD:-}"
NAMESPACE="${NAMESPACE:-red_standard_robot1}"
SLAM_MODE="${SLAM_MODE:-False}"
GAZEBO_STARTUP_SECONDS="${GAZEBO_STARTUP_SECONDS:-8}"
ROS_LOG_DIR="${ROS_LOG_DIR:-$SCRIPT_DIR/log/ros}"

if [[ $# -gt 0 && "$1" != *":="* ]]; then
  GAZEBO_WORLD="$1"
  shift
fi

# 没显式给 NAV_WORLD 时，跟随 Gazebo 场地名（传什么就用什么地图导航）
NAV_WORLD="${NAV_WORLD:-$GAZEBO_WORLD}"

SIM_PCD_DIR="$SCRIPT_DIR/src/pb2025_sentry_nav/pb2025_nav_bringup/pcd/simulation"
SIM_MAP_DIR="$SCRIPT_DIR/src/pb2025_sentry_nav/pb2025_nav_bringup/map/simulation"
echo "[sim_prior_nav] Gazebo 场地=$GAZEBO_WORLD  导航地图=$NAV_WORLD"
echo "[sim_prior_nav] 地图 -> $SIM_MAP_DIR/$NAV_WORLD.yaml"
if [[ ! -f "$SIM_MAP_DIR/$NAV_WORLD.yaml" ]]; then
  echo "[sim_prior_nav] 警告：地图不存在，先跑 sim_slam_install_map.sh $NAV_WORLD 建图"
fi
if [[ ! -f "$SIM_PCD_DIR/$NAV_WORLD.pcd" ]]; then
  echo "[sim_prior_nav] 警告：$SIM_PCD_DIR/$NAV_WORLD.pcd 不存在"
  echo "[sim_prior_nav]       将回退到 scans.pcd；若它属于别的场地，定位会漂"
  echo "[sim_prior_nav]       先跑一次：./tools/match_start/simulation/sim_slam_install_map.sh $NAV_WORLD"
fi
if [[ "${DRY_RUN:-0}" == "1" ]]; then
  echo "[sim_prior_nav] DRY_RUN=1，只解析参数不启动"
  exit 0
fi

nav_args=("$@")

log() {
  echo "[sim_prior_nav] $*"
}

die() {
  echo "[sim_prior_nav] ERROR: $*" >&2
  exit 1
}

stop_process_group() {
  local label="$1"
  local leader_pid="$2"
  local signal attempt

  [[ -n "$leader_pid" ]] || return 0
  kill -0 -- "-$leader_pid" 2>/dev/null || return 0

  for signal in INT TERM KILL; do
    log "Stopping $label with SIG$signal..."
    kill -"$signal" -- "-$leader_pid" 2>/dev/null || true
    for attempt in {1..20}; do
      kill -0 -- "-$leader_pid" 2>/dev/null || break 2
      sleep 0.25
    done
  done

  wait "$leader_pid" 2>/dev/null || true
}

source_ros() {
  if [[ -f "$SCRIPT_DIR/install/setup.bash" ]]; then
    set +u
    # shellcheck source=/dev/null
    source "$SCRIPT_DIR/install/setup.bash"
    set -u
  elif [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/$ROS_DISTRO/setup.bash" ]]; then
    set +u
    # shellcheck source=/dev/null
    source "/opt/ros/$ROS_DISTRO/setup.bash"
    set -u
  elif [[ -f /opt/ros/humble/setup.bash ]]; then
    set +u
    # shellcheck source=/dev/null
    source /opt/ros/humble/setup.bash
    set -u
  fi

  command -v ros2 >/dev/null 2>&1 || die "ros2 not found. Build/source the workspace first."
}

cleanup() {
  trap - INT TERM EXIT
  stop_process_group "prior navigation launch" "${nav_pid:-}"
  stop_process_group "Gazebo launch" "${gazebo_pid:-}"
}

source_ros
mkdir -p "$ROS_LOG_DIR"
export ROS_LOG_DIR

gazebo_cmd=(
  ros2 launch rmu_gazebo_simulator bringup_sim.launch.py
  world:="$GAZEBO_WORLD"
)

# 优先用与地图同名的点云；没有就回退到默认 scans.pcd
PRIOR_PCD="$SIM_PCD_DIR/$NAV_WORLD.pcd"
pcd_args=()
[[ -f "$PRIOR_PCD" ]] && pcd_args+=("prior_pcd_file:=$PRIOR_PCD")

nav_cmd=(
  ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py
  world:="$NAV_WORLD"
  slam:="$SLAM_MODE"
  namespace:="$NAMESPACE"
  "${pcd_args[@]}"
  "${nav_args[@]}"
)

log "Starting Gazebo world: $GAZEBO_WORLD"
printf '[sim_prior_nav] gazebo command:'
printf ' %q' "${gazebo_cmd[@]}"
printf '\n'

trap cleanup INT TERM

setsid "${gazebo_cmd[@]}" &
gazebo_pid=$!

log "Waiting ${GAZEBO_STARTUP_SECONDS}s for Gazebo startup..."
sleep "$GAZEBO_STARTUP_SECONDS"

log "Starting simulation prior navigation with map world: $NAV_WORLD"
printf '[sim_prior_nav] nav command:'
printf ' %q' "${nav_cmd[@]}"
printf '\n'

setsid "${nav_cmd[@]}" &
nav_pid=$!

set +e
wait "$nav_pid"
nav_status=$?
set -e
cleanup
exit "$nav_status"
