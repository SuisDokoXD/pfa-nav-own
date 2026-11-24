#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"

# 多角度斜坡（6° / 11° / 14° / 11°迂回 / 23°陡坡）+ 大幅度直角弯 + 180 度发卡弯迂回上坡测试场地
# 路线：P0(0m) -> 6°向东 -> P1(90°左转) -> 11°向北 -> P2(90°左转) -> 14°向西 -> P3(180°发卡弯) -> 11°向东 -> P4(90°左转) -> 23°向北 -> P5顶层(3.20m)
export GAZEBO_WORLD="${GAZEBO_WORLD:-switchback_ramps}"
export NAV_WORLD="${NAV_WORLD:-switchback_ramps}"
export SLAM_MODE="${SLAM_MODE:-True}"
USE_VOXEL_CLEARANCE="${USE_VOXEL_CLEARANCE:-True}"

exec "$SCRIPT_DIR/tools/match_start/simulation/sim_prior_nav_game.sh" \
  use_voxel_clearance:="$USE_VOXEL_CLEARANCE" "$@"
