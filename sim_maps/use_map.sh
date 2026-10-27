#!/usr/bin/env bash
# 仿真地图切换器：列出 / 打印启动命令 / 直接执行
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
INDEX="$HERE/INDEX.yaml"

die() { echo "错误：$*" >&2; exit 1; }
[ -f "$INDEX" ] || die "找不到 $INDEX，先跑 python3 sim_maps/build_index.py"

field() {  # field <case-dir> <key>
  python3 - "$INDEX" "$1" "$2" <<'PY'
import sys
import yaml
idx, case, key = sys.argv[1], sys.argv[2], sys.argv[3]
d = yaml.safe_load(open(idx, encoding="utf-8"))
for e in d["entries"]:
    if e["name"] == case:
        v = e.get(key)
        print("" if v is None else v)
        break
PY
}

list_all() {
  python3 - "$INDEX" <<'PY'
import yaml
d = yaml.safe_load(open(__import__("sys").argv[1], encoding="utf-8"))
print(f"默认场地：{d.get('default_world')}\n")
print(f"{'场地':<18}{'分类':<10}{'地图':<6}{'分辨率':<10}{'范围':<14}{'参数'}")
print("-" * 72)
for e in d["entries"]:
    dims = e.get("dims")
    if e["has_map"] and dims:
        res = float((e.get("map_meta") or {}).get("resolution", 0) or 0)
        extent = f"{dims[0]*res:.1f}x{dims[1]*res:.1f}m"
        r = f"{res}"
    else:
        r, extent = "—", "—"
    print(f"{e['name']:<18}{e['cat']:<10}{'有' if e['has_map'] else '无':<6}{r:<10}{extent:<14}"
          f"{'专用' if e['has_params'] else '共享'}")
PY
}

usage() {
  cat <<'EOF'
用法：
  ./sim_maps/use_map.sh                列出所有仿真场地
  ./sim_maps/use_map.sh <场地名>        打印启动命令
  ./sim_maps/use_map.sh <场地名> -x     直接执行 Gazebo 那一条（前台）

例：
  ./sim_maps/use_map.sh rmuc_2026
  ./sim_maps/use_map.sh upper_segment -x
EOF
}

[ $# -eq 0 ] && { list_all; echo; usage; exit 0; }

CASE="$1"
RUN="${2:-}"
[ "$CASE" = "-h" ] || [ "$CASE" = "--help" ] && { usage; exit 0; }

KNOWN=$(field "$CASE" name)
[ -n "$KNOWN" ] || { list_all; echo; die "没有叫 '$CASE' 的场地"; }

HAS_PARAMS=$(field "$CASE" has_params)
NAVCMD="ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \\
    world:=$CASE use_rviz:=True"
if [ "$HAS_PARAMS" = "True" ]; then
  NAVCMD="$NAVCMD \\
    params_file:=\$PWD/src/pb2025_sentry_nav/pb2025_nav_bringup/config/simulation/nav2_params_$CASE.yaml"
fi

GZCMD="ros2 launch rmu_gazebo_simulator bringup_sim.launch.py world:=$CASE"

if [ "$RUN" = "-x" ]; then
  cd "$REPO" || die "进不去 $REPO"
  echo "[run] $GZCMD"
  echo "[提示] 另一个终端跑导航："
  echo "       $NAVCMD"
  echo
  exec bash -lc "source /opt/ros/humble/setup.bash && source install/setup.bash 2>/dev/null; $GZCMD"
fi

cat <<EOF
场地：$CASE
分类：$(field "$CASE" cat)
地图：$( [ "$(field "$CASE" has_map)" = "True" ] && echo "有（见 META.md）" || echo "（该场地没有地图）" )

终端 1（Gazebo）：
  cd $REPO
  source /opt/ros/humble/setup.bash && source install/setup.bash
  $GZCMD

终端 2（导航）：
  $NAVCMD
EOF
