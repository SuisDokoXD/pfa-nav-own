#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"

# 狭窄多拐角迷宫测试场地（14 个连续 90 度拐角，净宽从 1.40 m 逐步收紧至 1.00 m）
export GAZEBO_WORLD="${GAZEBO_WORLD:-narrow_maze}"
export NAV_WORLD="${NAV_WORLD:-narrow_maze}"
export SLAM_MODE="${SLAM_MODE:-True}"

exec "$SCRIPT_DIR/tools/match_start/simulation/sim_prior_nav_game.sh" "$@"
