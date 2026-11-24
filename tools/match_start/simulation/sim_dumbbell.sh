#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"

# 哑铃形过洞测试场地：两个 6x6 m 房间由 3 m 通道连接，
# 洞口净宽 0.633 m、净高 1.034 m（锁死在 dumbbell_world.sdf 里）。
# 没有现成的先验地图，默认走 SLAM 建图；建成后再用 sim_prior_nav_game.sh
# 以 NAV_WORLD=dumbbell 跑先验导航。
export GAZEBO_WORLD="${GAZEBO_WORLD:-dumbbell}"
export NAV_WORLD="${NAV_WORLD:-dumbbell}"
export SLAM_MODE="${SLAM_MODE:-True}"

exec "$SCRIPT_DIR/tools/match_start/simulation/sim_prior_nav_game.sh" "$@"
