#!/usr/bin/env python3
"""校验 upper_segment 的生成物：SDF 结构 + 地图元数据 + 同源一致性 + 锚点。

只读生成物（world / pgm / yaml / report.json），不 import 生成器，避免「自己验证自己」。
任何一条失败 exit 1。

用法：
    python3 tools/sim/verify_upper_segment.py \
        --world src/rmu_gazebo_simulator/rmu_gazebo_simulator/resource/worlds/upper_segment_world.sdf \
        --map src/pb2025_sentry_nav/pb2025_nav_bringup/map/simulation/upper_segment.yaml \
        --report /tmp/upper_report.json
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import sys
import xml.etree.ElementTree as ET

import numpy as np

FAILS = []
OKS = []


def check(name, ok, detail=""):
    (OKS if ok else FAILS).append(name)
    print(f"[{'OK  ' if ok else 'FAIL'}] {name} {detail}")


# --------------------------------------------------------------------------- #
def parse_pgm(path):
    with open(path, "rb") as f:
        data = f.read()
    parts = data.split(b"\n", 3)
    if parts[0].strip() != b"P5":
        raise SystemExit(f"{path}: 不是 P5 PGM")
    w, h = [int(x) for x in parts[1].split()]
    px = np.frombuffer(parts[3][: w * h], dtype=np.uint8).reshape(h, w)
    return px


def parse_yaml(path):
    meta = {}
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            k, _, v = line.partition(":")
            k = k.strip()
            v = v.strip()
            if v.startswith("[") and v.endswith("]"):
                meta[k] = [float(x) for x in v[1:-1].split(",")]
            else:
                try:
                    meta[k] = float(v)
                except ValueError:
                    meta[k] = v
    return meta


def parse_sdf(path):
    tree = ET.parse(path)
    root = tree.getroot()
    world = root.find("world")
    models = []
    for mdl in world.findall("model"):
        links = []
        for ln in mdl.findall("link"):
            pose_txt = ln.findtext("pose", "0 0 0 0 0 0").split()
            pose = [float(x) for x in pose_txt]
            size = None
            has_col = ln.find("collision") is not None
            has_vis = ln.find("visual") is not None
            col_size = vis_size = None
            if has_col:
                col_size = [float(x) for x in
                            ln.find("collision/geometry/box/size").text.split()]
            if has_vis:
                vis_size = [float(x) for x in
                            ln.find("visual/geometry/box/size").text.split()]
            links.append(dict(name=ln.get("name"), pose=pose, size=vis_size,
                              has_col=has_col, has_vis=has_vis,
                              col_size=col_size, vis_size=vis_size))
        models.append(dict(name=mdl.get("name"),
                           static=(mdl.findtext("static", "").strip() == "true"),
                           links=links))
    physics = world.find("physics")
    return models, physics


# --------------------------------------------------------------------------- #
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", required=True)
    ap.add_argument("--map", required=True)
    ap.add_argument("--report", default="")
    ap.add_argument("--self-test", action="store_true", help="负向自测：证明检查会失败")
    args = ap.parse_args()

    models, physics = parse_sdf(args.world)
    meta = parse_yaml(args.map)
    pgm_path = os.path.join(os.path.dirname(os.path.abspath(args.map)), meta["image"])
    occ = parse_pgm(pgm_path)

    res = float(meta["resolution"])
    ox, oy = meta["origin"][0], meta["origin"][1]
    h, w = occ.shape
    y_max = oy + h * res

    # ---------- A3 地图元数据 ----------
    check("A3 map size 260x91", (w, h) == (260, 91), f"got {w}x{h}")
    check("A3 resolution 0.05", abs(res - 0.05) < 1e-9, f"got {res}")
    check("A3 origin [-6.500,-0.300]", abs(ox + 6.5) < 1e-6 and abs(oy + 0.3) < 1e-6,
          f"got [{ox},{oy}]")
    check("A3 mode trinary", meta.get("mode") == "trinary", f"got {meta.get('mode')}")

    # ---------- A8 SDF 结构 ----------
    names = [m["name"] for m in models]
    check("A8 4 models", len(models) == 4, f"got {names}")
    check("A8 all static", all(m["static"] for m in models), "")
    check("A8 physics present", physics is not None, "")
    if physics is not None:
        step = float(physics.findtext("max_step_size", "0"))
        check("A8 max_step_size=0.01", abs(step - 0.01) < 1e-9, f"got {step}")

    bad_pair = []
    bad_size = []
    min_side = 1e9
    for m in models:
        if m["name"].endswith("_markings"):
            continue          # 地贴按设计只有 visual，没有 collision
        for ln in m["links"]:
            if ln["name"].startswith("ground"):
                continue
            if ln["has_col"] != ln["has_vis"]:
                bad_pair.append(ln["name"])
            if ln["has_col"] and ln["col_size"] != ln["vis_size"]:
                bad_pair.append(ln["name"])
            if ln["size"]:
                min_side = min(min_side, min(ln["size"][0], ln["size"][1]))
    check("A8 collision/visual 成对且同尺寸", not bad_pair, f"{bad_pair[:5]}")
    check("A8 无退化 link (>=0.05m)", min_side >= 0.049, f"min side {min_side:.3f}")

    wall_z = [ln["size"][2] for m in models for ln in m["links"]
              if ln["name"].startswith("wall_") and ln["size"]]
    check("A8 墙高只允许 0.60/0.45", wall_z and all(min(abs(z - 0.60), abs(z - 0.45)) < 1e-6 for z in wall_z),
          f"z set {sorted(set(round(z, 3) for z in wall_z))}")

    # ---------- A7 同源：盒子 -> 栅格 ----------
    def cell_of(x, y):
        return int(math.floor((y_max - y) / res)), int(math.floor((x - ox) / res))

    covered = np.zeros_like(occ, dtype=bool)
    box_hits = 0
    box_total = 0
    for m in models:
        for ln in m["links"]:
            if not ln["has_col"] or not ln["size"]:
                continue
            if ln["name"].startswith("ground"):
                continue
            cx, cy, cz = ln["pose"][0], ln["pose"][1], ln["pose"][2]
            sx, sy, sz = ln["size"]
            box_total += 1
            hx, hy = sx / 2.0, sy / 2.0
            r0 = int(math.floor((y_max - (cy + hy)) / res + 1e-9))
            r1 = int(math.floor((y_max - (cy - hy)) / res + 1e-9))
            c0 = int(math.floor(((cx - hx) - ox) / res + 1e-9))
            c1 = int(math.floor(((cx + hx) - ox) / res + 1e-9))
            r0, r1 = max(0, min(r0, r1)), min(h - 1, max(r0, r1))
            c0, c1 = max(0, min(c0, c1)), min(w - 1, max(c0, c1))
            covered[r0:r1 + 1, c0:c1 + 1] = True
            rr, cc = cell_of(cx, cy)
            if 0 <= rr < h and 0 <= cc < w and occ[rr, cc] == 0:
                box_hits += 1
    check("A7 每个碰撞盒中心落在占据格 >=95%",
          box_hits / max(1, box_total) >= 0.95, f"{box_hits}/{box_total}")

    # ---------- A4 占据率 ----------
    field = np.zeros_like(occ, dtype=bool)
    r_top = int(math.floor((y_max - 3.75) / res))
    r_bot = int(math.floor((y_max - 0.20) / res))
    c_l = int(math.floor((-6.0 - ox) / res))
    c_r = int(math.floor((6.0 - ox) / res))
    field[max(0, r_top):min(h, r_bot + 1), max(0, c_l):min(w, c_r + 1)] = True
    f_occ = float((occ[field] == 0).mean())
    f_free = float((occ[field] == 254).mean())
    unk = float((occ == 127).mean())
    check("A4 场地占据率 35%~60%", 0.35 <= f_occ <= 0.60, f"{100*f_occ:.1f}%")
    check("A4 场地自由率 >=40%", f_free >= 0.40, f"{100*f_free:.1f}%")
    check("A4 unknown <=30%", unk <= 0.30, f"{100*unk:.1f}%")

    # ---------- A7b 占据格都被某个盒覆盖 ----------
    occ_in_field = field & (occ == 0)
    uncovered = int((occ_in_field & ~covered).sum())
    check("A7 场地内占据格未被盒覆盖 = 0", uncovered == 0, f"{uncovered} cells")

    # ---------- A6 锚点（独立于生成器重算） ----------
    def ratio(x0, x1, y0, y1, val):
        r0 = int(math.floor((y_max - y1) / res))
        r1 = int(math.floor((y_max - y0) / res))
        c0 = int(math.floor((x0 - ox) / res))
        c1 = int(math.floor((x1 - ox) / res))
        rr0, rr1 = max(0, min(r0, r1)), min(h - 1, max(r0, r1))
        cc0, cc1 = max(0, min(c0, c1)), min(w - 1, max(c0, c1))
        sub = occ[rr0:rr1 + 1, cc0:cc1 + 1]
        return float((sub == val).mean()) if sub.size else 0.0

    anchors = [
        ("spawn free", ratio(5.45, 5.65, 1.79, 1.99, 254) >= 0.99),
        ("start zone free", ratio(5.40, 5.85, 1.60, 2.25, 254) >= 0.95),
        ("start exit free", ratio(4.90, 5.20, 1.62, 2.09, 254) >= 0.80),
        ("right wall occupied", ratio(4.90, 5.20, 2.35, 3.10, 0) >= 0.58),
        ("center obstacle occupied", ratio(-0.55, -0.37, 1.30, 2.50, 0) >= 0.50),
        ("notch wall body occupied", ratio(0.55, 0.95, 2.40, 3.50, 0) >= 0.80),
        # 语义验证：浅紫 = 墙/场外，深紫 = 赛道
        ("left apron is outside", ratio(-4.20, -3.60, 1.60, 2.40, 0) >= 0.90),
        ("track open area is free", ratio(-1.30, -0.80, 1.60, 2.20, 254) >= 0.90),
    ]
    for name, ok in anchors:
        check("A6 " + name, ok, "")

    # 围栏（障碍物 / 起伏路）：从 SDF 侧独立核对，不复用生成器的 bbox
    bars = [ln for mm in models for ln in mm["links"]
            if ln["has_col"] and ln["size"] and abs(ln["size"][2] - 0.45) < 1e-6]
    check("A6 围栏薄墙数量 = 8（2 组 x 4 面）", len(bars) == 8, f"got {len(bars)}")
    bar_ok = 0
    for b in bars:
        bx, by = b["pose"][0], b["pose"][1]
        bx = max(-6.0 + 1e-3, min(6.0 - 1e-3, bx))
        by = max(0.20 + 1e-3, min(3.75 - 1e-3, by))
        rr2, cc2 = cell_of(bx, by)
        if 0 <= rr2 < h and 0 <= cc2 < w and occ[rr2, cc2] == 0:
            bar_ok += 1
    check("A6 每根围栏中心落在占据格", bar_ok == len(bars), f"{bar_ok}/{len(bars)}")

    # ---------- A9 出生点净空 ----------
    rr, cc = cell_of(5.55, 1.89)
    if occ[rr, cc] == 254:
        dist = None
        for r in range(h):
            for c in range(w):
                if occ[r, c] == 0:
                    d = math.hypot((c - cc) * res, (r - rr) * res)
                    if dist is None or d < dist:
                        dist = d
        detail = f"{dist:.2f} m" if dist is not None else "no obstacle cell"
        check("A9 出生点最近障碍距离 >=0.30m", dist is not None and dist >= 0.30, detail)
    else:
        check("A9 出生点可通行", False, f"occ={occ[rr, cc]}")

    # ---------- report 交叉核对 ----------
    if args.report and os.path.exists(args.report):
        rep = json.load(open(args.report, encoding="utf-8"))
        r_map = rep["map"]
        check("report 与产物一致(尺寸)", r_map["w"] == w and r_map["h"] == h, "")
        check("report 与产物一致(场地占据率)",
              abs(r_map["field_occupied"] - 100 * f_occ) < 0.6,
              f"report {r_map['field_occupied']:.2f} vs {100*f_occ:.2f}")

    # ---------- 负向自测：证明这些检查真的会失败（复审 blocker 3） ----------
    if args.self_test:
        # (1) 把 origin 平移一格，A3 的判据必须为 False
        shifted = abs((ox - res) + 6.5) < 1e-6 and abs(oy + 0.3) < 1e-6
        check("selftest origin 平移一格应判负", not shifted, "")
        # (2) 去掉一个墙盒，覆盖检查应出现空洞
        cov2 = covered.copy()
        victim = None
        for mm in models:
            for ln in mm["links"]:
                if ln["name"].startswith("wall_") and ln["has_col"]:
                    victim = ln
                    break
            if victim:
                break
        if victim:
            vx, vy = victim["pose"][0], victim["pose"][1]
            vhx, vhy = victim["size"][0] / 2, victim["size"][1] / 2
            r0 = int(math.floor((y_max - (vy + vhy)) / res))
            r1 = int(math.floor((y_max - (vy - vhy)) / res))
            c0 = int(math.floor(((vx - vhx) - ox) / res))
            c1 = int(math.floor(((vx + vhx) - ox) / res))
            cov2[max(0, r0):max(0, r1) + 1, max(0, c0):max(0, c1) + 1] = False
            holes = int((field & (occ == 0) & ~cov2).sum())
            check("selftest 去掉一个墙盒应出现空洞", holes > 0, f"{holes} cells")
        else:
            check("selftest 找不到可删的墙盒", False, "")

    print()
    if FAILS:
        print(f"FAIL: {len(FAILS)} 项未通过 -> {FAILS}")
        return 1
    print(f"PASS: {len(OKS)} 项全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())