#!/usr/bin/env python3
"""在 pfa-nav-own 顶层生成 sim_maps/ 汇总目录（原位置文件不动，只复制副本 + 元数据）。"""
import os
import shutil
import subprocess
from pathlib import Path

ROOT = Path.home() / "sight/new/pfa-nav-own"
WORLDS = ROOT / "src/rmu_gazebo_simulator/rmu_gazebo_simulator/resource/worlds"
MODELS = ROOT / "src/rmu_gazebo_simulator/rmu_gazebo_simulator/resource/models"
MAPS = ROOT / "src/pb2025_sentry_nav/pb2025_nav_bringup/map/simulation"
PARAMS = ROOT / "src/pb2025_sentry_nav/pb2025_nav_bringup/config/simulation"
GZWORLD = ROOT / "src/rmu_gazebo_simulator/rmu_gazebo_simulator/config/gz_world.yaml"
OUT = ROOT / "sim_maps"

import yaml


def pgm_size(path):
    """解析 P5 头，跳过 ROS map_saver 写的 # 注释行。"""
    with open(path, "rb") as f:
        data = f.read(512)
    toks = []
    for line in data.split(b"\n"):
        if line.startswith(b"#"):
            continue
        toks += line.split()
        if len(toks) >= 3:
            break
    if not toks or toks[0] != b"P5" or len(toks) < 3:
        return None
    return int(toks[1]), int(toks[2])


def read_yaml(path):
    try:
        return yaml.safe_load(open(path, encoding="utf-8")) or {}
    except Exception:
        return {}


def main():
    gz = read_yaml(GZWORLD)
    robots = gz.get("robots", {}) or {}

    # 只清理分类子目录和索引；不要 rmtree 整个 sim_maps（里面有 build_index.py / use_map.sh）
    OUT.mkdir(parents=True, exist_ok=True)
    for cat in ("rmuc", "rmul", "custom"):
        if (OUT / cat).exists():
            shutil.rmtree(OUT / cat)

    entries = []
    for wf in sorted(WORLDS.glob("*_world.sdf")):
        name = wf.name[: -len("_world.sdf")]
        if name.startswith("rmuc"):
            cat = "rmuc"
        elif name.startswith("rmul"):
            cat = "rmul"
        else:
            cat = "custom"

        dst = OUT / cat / name
        dst.mkdir(parents=True, exist_ok=True)
        shutil.copy2(wf, dst / "world.sdf")

        # 地图
        mp, my = MAPS / f"{name}.pgm", MAPS / f"{name}.yaml"
        has_map = mp.exists() and my.exists()
        if has_map:
            shutil.copy2(mp, dst / "map.pgm")
            shutil.copy2(my, dst / "map.yaml")
            meta = read_yaml(my)
            dims = pgm_size(mp)
        else:
            meta, dims = {}, None

        # 专用 nav2 参数（目前只有 upper_segment 有）
        pv = PARAMS / f"nav2_params_{name}.yaml"
        has_params = pv.exists()
        if has_params:
            shutil.copy2(pv, dst / "nav2_params.yaml")

        # 场地模型
        model_dir = MODELS / name
        has_model = model_dir.is_dir()

        spawn = (robots.get(name) or [{}])[0]
        entries.append(dict(
            name=name, cat=cat, dir=f"{cat}/{name}",
            world=str(wf.relative_to(ROOT)),
            map_pgm=str(mp.relative_to(ROOT)) if has_map else None,
            map_yaml=str(my.relative_to(ROOT)) if has_map else None,
            has_map=has_map, has_params=has_params, has_model=has_model,
            dims=dims, map_meta=dict(resolution=meta.get("resolution"), origin=meta.get("origin"), mode=meta.get("mode")) if has_map else None, spawn=spawn,
        ))

        # META.md
        L = [f"# {name}", ""]
        L.append(f"- 分类：{cat}")
        L.append(f"- 源 world：`{wf.relative_to(ROOT)}`")
        L.append(f"- 场地模型：{'`' + str(model_dir.relative_to(ROOT)) + '`' if has_model else '无（world 内直接建模）'}")
        if has_map:
            L.append(f"- 地图：`{mp.relative_to(ROOT)}` + `{my.relative_to(ROOT)}`")
            L.append(f"  - 尺寸：{dims[0]}×{dims[1]} px @ {meta.get('resolution')} m"
                     f" → {dims[0]*float(meta.get('resolution',0)):.2f} × {dims[1]*float(meta.get('resolution',0)):.2f} m")
            L.append(f"  - origin：{meta.get('origin')}")
            L.append(f"  - mode：{meta.get('mode')}")
        else:
            L.append("- 地图：**无**（该场地没有配 pgm/yaml）")
        L.append(f"- nav2 参数：{'`' + str(pv.relative_to(ROOT)) + '`' if has_params else '`config/simulation/nav2_params.yaml`（共享）'}"
                 + ("　← 专用文件，切这个场地要显式传参" if has_params else ""))
        if spawn:
            L.append(f"- 出生点：({spawn.get('x_pose')}, {spawn.get('y_pose')}, {spawn.get('z_pose')}) yaw={spawn.get('yaw')}")
        else:
            L.append("- 出生点：`gz_world.yaml` 里没有该场地条目")
        L += ["", "## 启动", "", "```bash", f"./sim_maps/use_map.sh {name}", "```", ""]
        (dst / "META.md").write_text("\n".join(L), encoding="utf-8")

    # INDEX.yaml
    idx = dict(
        note="仿真地图汇总索引。这里的 world/map 都是副本，权威文件在各自源路径；改完源文件请重新跑 sim_maps/build_index.py。",
        default_world=gz.get("world"),
        entries=[{k: v for k, v in e.items()} for e in entries],
    )
    (OUT / "INDEX.yaml").write_text(
        yaml.safe_dump(idx, allow_unicode=True, sort_keys=False), encoding="utf-8")

    # README.md
    R = ["# 仿真地图汇总（sim_maps）", "",
         "`pfa-nav-own` 里所有可跑的 Gazebo 仿真场地都汇总在这里，按赛制分类。",
         "**原位置的文件没有移动**，这里全部是副本，方便一眼看清有哪些图、怎么切。", "",
         f"当前默认场地：`{gz.get('world')}`", "",
         "## 一览", "",
         "| 场地 | 分类 | 地图 | 分辨率 | 地图范围 | 出生点 | 参数 |",
         "|---|---|---|---|---|---|---|"]
    for e in entries:
        if e["has_map"]:
            res = (e.get("map_meta") or {}).get("resolution")
            dims = e["dims"]
            extent = f"{dims[0]*float(res):.1f}×{dims[1]*float(res):.1f} m"
            mapcell = "有"
        else:
            res, extent, mapcell = "—", "—", "**无**"
        sp = e["spawn"]
        spawn = f"({sp['x_pose']}, {sp['y_pose']})" if sp else "—"
        param = "专用" if e["has_params"] else "共享"
        R.append(f"| `{e['name']}` | {e['cat']} | {mapcell} | {res} | {extent} | {spawn} | {param} |")

    R += ["", "## 切场地", "",
          "```bash",
          "./sim_maps/use_map.sh                    # 列出所有场地",
          "./sim_maps/use_map.sh rmuc_2026           # 打印该场地的启动命令",
          "./sim_maps/use_map.sh rmuc_2026 -x        # 直接执行（两个终端要分开放，建议先不加 -x）",
          "```", "",
          "底层其实就是两个 launch：", "",
          "```bash",
          "ros2 launch rmu_gazebo_simulator bringup_sim.launch.py world:=<名字>",
          "ros2 launch pb2025_sentry_nav rm_navigation_simulation_launch.py world:=<名字> use_rviz:=True \\",
          "    params_file:=<按 INDEX.yaml 决定的参数文件>",
          "```", "",
          "## 注意", "",
          "- `world:=<名字>` 会同时决定 world 文件、地图文件名、`gz_world.yaml` 里的出生点 —— **三者必须同名**。",
          "- 只有 `upper_segment` 有专用 nav2 参数；其余用共享 `config/simulation/nav2_params.yaml`。",
          "- `rmuc_2027` 的地图 `rmuc_2027.pgm/.yaml` 目前**没有进 git**（本地未跟踪），这里的副本可用但仓库里缺源文件。",
          ""]
    (OUT / "README.md").write_text("\n".join(R), encoding="utf-8")

    print(f"[done] {OUT}")
    for e in entries:
        print(f"  {e['cat']:7s} {e['name']:16s} map={'Y' if e['has_map'] else 'N'} "
              f"params={'Y' if e['has_params'] else 'N'} model={'Y' if e['has_model'] else 'N'}")


if __name__ == "__main__":
    main()
