#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"

export GAZEBO_WORLD="${GAZEBO_WORLD:-corner_corridor}"
export NAV_WORLD="${NAV_WORLD:-corner_corridor}"
export SLAM_MODE="${SLAM_MODE:-True}"

exec "$SCRIPT_DIR/tools/match_start/simulation/sim_prior_nav_game.sh" "$@"
