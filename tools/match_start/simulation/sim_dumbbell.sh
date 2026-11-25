#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"

# 哑铃形过洞测试场地：两个 6x6 m 房间由 3 m 通道连接，
# 洞口净宽 0.533 m（单侧余量 50 mm）、净高 0.544 m（顶部余量 10 mm）。
# 默认启用 voxel_clearance 与 tunnel_align（含仿真 C 板模拟器），
# 可通过环境变量 USE_VOXEL_CLEARANCE=False / USE_TUNNEL_ALIGN=False 关闭。
export GAZEBO_WORLD="${GAZEBO_WORLD:-dumbbell}"
export NAV_WORLD="${NAV_WORLD:-dumbbell}"
export SLAM_MODE="${SLAM_MODE:-True}"
USE_VOXEL_CLEARANCE="${USE_VOXEL_CLEARANCE:-True}"
USE_TUNNEL_ALIGN="${USE_TUNNEL_ALIGN:-True}"

exec "$SCRIPT_DIR/tools/match_start/simulation/sim_prior_nav_game.sh" \
  use_voxel_clearance:="$USE_VOXEL_CLEARANCE" \
  use_tunnel_align:="$USE_TUNNEL_ALIGN" "$@"
