#!/usr/bin/env python3
"""按 CAD 图纸「上半段」生成 Gazebo 世界 + Nav2 栅格地图（v2 语义修正版）。

v2 相对 v1 的改动（对应 note 里的缺陷清单）：
  1. 碰撞语义不再依赖「紫色深浅」：深紫一律算地面；障碍 = 蓝线膨胀
     ∪ 夹在蓝线之间的浅紫带体 ∪ 薄洞 ∪ 台阶。
  2. 裁剪在所有形态学操作之前（v1 是先填洞后裁剪，会在裁剪线附近填死通道）。
  3. 禁止全局 binary_fill_holes；只补「宽度 ≤ 0.60 m 且面积 ≤ 1.0 m²」的薄洞。
  4. 生成 <physics> 块（0.01 / 100 Hz），墙高 0.60 m（高于雷达安装面）。
  5. 边界墙外移到场地外，避免压住红色启动区 / 绿色目标带。
  6. 栅格 cell == map resolution == 0.05，取消 0.10 m max-pooling。
  7. 像素/盒 -> 栅格统一用 floor()，消除半格偏移。
  8. 自检锚点换成与真实几何一致的一组（v1 的「顶部平台应占据」前提是错的）。

用法：
    python3 tools/sim/drawing_to_upper_segment.py \
        --drawing tools/sim/drawings/rmu_arena_2026.png \
        --out-world src/rmu_gazebo_simulator/rmu_gazebo_simulator/resource/worlds \
        --out-map  src/pb2025_sentry_nav/pb2025_nav_bringup/map/simulation \
        --preview  tools/sim/drawings/upper_segment_preview.png \
        --report   /tmp/upper_report.json

依赖：numpy, Pillow, scipy
"""
from __future__ import annotations

import argparse
import json
import math
import os

import numpy as np
from PIL import Image, ImageDraw
from scipy import ndimage

# --------------------------------------------------------------------------- #
# 参数
# --------------------------------------------------------------------------- #
DEFAULTS = dict(
    field_w=12.0,
    field_h=7.5,
    crop_y_min=0.20,
    crop_y_max=3.75,
    wall_height=0.60,          # 围墙 / 赛道边界墙（高于雷达安装面）
    boundary_thickness=0.20,
    boundary_height=1.00,
    boundary_gap=0.10,         # 边界墙贴场地外缘再外移这么多
    enclosure_thickness=0.10,  # 障碍物 / 起伏路 的围栏薄墙厚度
    enclosure_height=0.45,     # 围栏墙高：与机器人整体高度相近或略高
    map_resolution=0.05,
    map_margin=0.50,
    min_corridor=0.50,         # 最小可通行净宽：底盘 0.47 m（轮距外沿）× 0.37 m + 余量。
                               # 不用 0.6（=2×robot_radius 圆）：那是保守外接圆，
                               # 图纸自己的启动门洞只有 0.69 m，按圆要求会判成不可通行。
)

# 原图标定：紫色场地填充外接框 -> 12.0 m x 7.5 m
CAL = dict(U0=61, U1=1021, V0=73, V1=630)
CROP_V0, CROP_V1 = 73, 336          # 上半段（含）
SX = DEFAULTS["field_w"] / (CAL["U1"] - CAL["U0"])   # 0.0125 m/px
SY = DEFAULTS["field_h"] / (CAL["V1"] - CAL["V0"])   # 0.013465 m/px
PX = min(SX, SY)

# 出生点（写在 config/gz_world.yaml 的 upper_segment 段）
SPAWN = dict(x=5.55, y=1.89, z=0.10, yaw=math.pi)


def wx(u):
    return (u - CAL["U0"]) * SX - DEFAULTS["field_w"] / 2.0


def wy(v):
    return DEFAULTS["field_h"] / 2.0 - (v - CAL["V0"]) * SY


def parse_args():
    ap = argparse.ArgumentParser(description="图纸上半段 -> Gazebo world + Nav2 map (v2)")
    ap.add_argument("--drawing", required=True)
    ap.add_argument("--name", default="upper_segment")
    ap.add_argument("--out-world", default=".")
    ap.add_argument("--out-map", default=".")
    ap.add_argument("--preview", default="")
    ap.add_argument("--report", default="")
    ap.add_argument("--no-crop-wall", action="store_true",
                    help="不在 y=crop_y_min 处生成封闭墙（默认生成，否则机器人会掉出地面）")
    return ap.parse_args()


# --------------------------------------------------------------------------- #
# 颜色掩码（先裁剪，再做任何形态学）
# --------------------------------------------------------------------------- #
def build_masks(im):
    a = np.asarray(im.convert("RGB")).astype(np.int16)
    r, g, b = a[:, :, 0], a[:, :, 1], a[:, :, 2]
    m = {}
    m["blue"] = ((b > 120) & ((b - r) > 60) & ((b - g) > 60) & (r < 130) & (g < 130))
    m["red"] = (r > 180) & (g < 100) & (b < 100)
    m["green"] = (g > 150) & (r < 120) & (b < 120)
    m["cyan"] = ((r < 180) & (g > 190) & (b > 190) & ((g - r) > 60) & ((b - r) > 60))
    m["gray"] = ((np.abs(r - g) < 18) & (np.abs(g - b) < 18) & (r > 160) & (r < 225))
    # 浅紫：深紫不算，二者是不同的填充色
    m["light"] = ((np.abs(r - 168) <= 12) & (np.abs(g - 168) <= 12) & (np.abs(b - 196) <= 12))
    # 深紫：赛道可行驶地面。v2 里它永不作为障碍源，只用于「别把凹槽当薄洞填死」
    m["deep"] = ((np.abs(r - 121) <= 12) & (np.abs(g - 121) <= 12) & (np.abs(b - 171) <= 12))
    m["_rgb"] = a
    return a, m


def crop_mask(mask, v0=CROP_V0, v1=CROP_V1):
    out = np.zeros_like(mask)
    out[v0:v1 + 1, CAL["U0"]:CAL["U1"] + 1] = mask[v0:v1 + 1, CAL["U0"]:CAL["U1"] + 1]
    return out


def disk(radius):
    y, x = np.mgrid[-radius:radius + 1, -radius:radius + 1]
    return (x * x + y * y) <= radius * radius


def px_of(meters):
    return max(1, int(round(meters / PX)))


def blobs(mask, min_area):
    """返回 [(u0,u1,v0,v1,area)]，按面积降序。"""
    mm = ndimage.binary_closing(mask, structure=np.ones((5, 5)))
    mm = ndimage.binary_opening(mm, structure=np.ones((3, 3)))
    lab, n = ndimage.label(mm)
    if n == 0:
        return []
    objs = ndimage.find_objects(lab)
    sizes = ndimage.sum(mm, lab, range(1, n + 1))
    out = []
    for i in range(n):
        if sizes[i] < min_area:
            continue
        sl = objs[i]
        out.append((sl[1].start, sl[1].stop - 1, sl[0].start, sl[0].stop - 1, int(sizes[i])))
    out.sort(key=lambda t: -t[4])
    return out


def biggest_blob(mask, min_area, u_range=None, v_range=None):
    for bb in blobs(mask, min_area):
        u0, u1, v0, v1, _ = bb
        if u_range and not (u_range[0] <= u0 and u1 <= u_range[1]):
            continue
        if v_range and not (v_range[0] <= v0 and v1 <= v_range[1]):
            continue
        return bb
    return None


# --------------------------------------------------------------------------- #
# 障碍掩码
# --------------------------------------------------------------------------- #
def solid_mask(m, floor_marks=None, exclude=None):
    """障碍掩码（v4）。

    图纸着色含义（已按局部放大与用户口径确认）：
      深紫 (121,121,171) = 赛道可行驶地面
      浅紫 (168,168,196) = 墙体本体 / 场外非行驶区
      蓝线（1–2 px）     = 边界描边（不单独膨胀，否则会在赛道里切出假墙）
      灰 (192,192,192)   = 障碍物（大块平台 + 中央小障碍条）
      红                 = 启动区，也是赛道，只是出生点在那里
      青（灰台下）       = 起伏路，地形不建，用墙围起来
      青（左侧）         = 免打区，地面涂装

    exclude：围栏足迹，交回给 enclosure 自己生成薄墙，避免这里再叠一个 0.60 m 实心块。
    """
    blue = crop_mask(m["blue"])
    light = crop_mask(m["light"])
    gray = crop_mask(m["gray"])

    inside = np.zeros_like(blue)
    inside[CAL["V0"]:CROP_V1 + 1, CAL["U0"]:CAL["U1"] + 1] = True

    # 灰 = 障碍物（大块平台 + 中央小条），蓝线只作描边不膨胀。
    solid = (light | gray) & inside
    if exclude is not None:
        # 围栏足迹交给 enclosure（薄墙 + 围内 unknown），这里不重复建模
        solid &= ~crop_mask(exclude)

    # 地面涂装（红启动区 / 绿目标带 / 青免打区）强制可通行
    if floor_marks is not None:
        solid &= ~ndimage.binary_dilation(crop_mask(floor_marks), structure=disk(px_of(0.06)))

    stats = dict(
        blue_px=int(blue.sum()),
        light_px=int(light.sum()),
        gray_px=int(gray.sum()),
        solid_px=int(solid.sum()),
    )
    return solid, stats


# --------------------------------------------------------------------------- #
# 栅格
# --------------------------------------------------------------------------- #
class Map:
    def __init__(self):
        self.res = DEFAULTS["map_resolution"]
        self.mg = DEFAULTS["map_margin"]
        self.ox = -DEFAULTS["field_w"] / 2.0 - self.mg
        self.oy = DEFAULTS["crop_y_min"] - self.mg
        self.w = int(round((DEFAULTS["field_w"] + 2 * self.mg) / self.res))
        self.h = int(round((DEFAULTS["crop_y_max"] + self.mg - self.oy) / self.res))
        self.y_max = self.oy + self.h * self.res
        self.occ = np.full((self.h, self.w), 127, dtype=np.uint8)

    def col(self, x):
        return int(math.floor((x - self.ox) / self.res))

    def row(self, y):
        return int(math.floor((self.y_max - y) / self.res))

    def x_of(self, c):
        return self.ox + (c + 0.5) * self.res

    def y_of(self, r):
        return self.y_max - (r + 0.5) * self.res

    def rect_span(self, x0, x1, y0, y1):
        """闭区间外扩一格，保证边缘像素不被 floor 掉。

        与地图无交集时返回空区间（r0>=r1 或 c0>=c1），调用方切片自然为空；
        不能只做 max(0, min())，否则「完全在地图上方」的矩形会退化成 occ[0:-k]。
        """
        c0, c1 = self.col(x0), self.col(x1)
        r0, r1 = self.row(y1), self.row(y0)
        r_lo, r_hi = min(r0, r1), max(r0, r1) + 1
        c_lo, c_hi = min(c0, c1), max(c0, c1) + 1
        if r_hi <= 0 or r_lo >= self.h or c_hi <= 0 or c_lo >= self.w:
            return (0, 0, 0, 0)
        return (max(0, r_lo), min(self.h, r_hi), max(0, c_lo), min(self.w, c_hi))

    def fill_field(self):
        r0, r1, c0, c1 = self.rect_span(-DEFAULTS["field_w"] / 2, DEFAULTS["field_w"] / 2,
                                       DEFAULTS["crop_y_min"], DEFAULTS["crop_y_max"])
        self.occ[r0:r1, c0:c1] = 254

    def mark_mask(self, mask):
        vs, us = np.nonzero(mask)
        if vs.size == 0:
            return
        xs = (us - CAL["U0"]) * SX - DEFAULTS["field_w"] / 2.0
        ys = DEFAULTS["field_h"] / 2.0 - (vs - CAL["V0"]) * SY
        cc = np.floor((xs - self.ox) / self.res).astype(int)
        rr = np.floor((self.y_max - ys) / self.res).astype(int)
        ok = (cc >= 0) & (cc < self.w) & (rr >= 0) & (rr < self.h)
        self.occ[rr[ok], cc[ok]] = 0

    def ratio(self, x0, x1, y0, y1, value):
        r0, r1, c0, c1 = self.rect_span(x0, x1, y0, y1)
        sub = self.occ[r0:r1, c0:c1]
        return float((sub == value).mean()) if sub.size else 0.0


def clean_grid(grid):
    """去掉单格毛刺、补上单格缺口（2x2 开/闭）。

    用 2x2 而不是十字 3x3：0.10 m 的单线墙只有 2 格宽，3x3 开运算会把它整条吃掉。
    2x2 只删「四周凑不出一个 2x2」的孤立格，也就是图右上角那些毛刺。
    """
    k = np.ones((2, 2), dtype=bool)
    out = ndimage.binary_opening(grid, structure=k)
    out = ndimage.binary_closing(out, structure=k)
    # 矩形化：连通的墙块若填满自己外接框 >=90%，就整体替换成外接框。
    # 图纸里绝大多数墙本来就是矩形，栅格化后边缘会变成一格一格的台阶；这一步直接把台阶抹平。
    lab, n = ndimage.label(out, structure=np.ones((3, 3), dtype=bool))
    if n:
        objs = ndimage.find_objects(lab)
        fixed = out.copy()
        for i in range(n):
            sl = objs[i]
            sub = (lab[sl] == i + 1)
            area = float(sub.sum())
            bbox = float(sub.size)
            if area / max(1.0, bbox) >= 0.90:
                fixed[sl] |= True          # 整块填实 -> 边缘变成直角
        out = fixed
    return out


def merge_rects(rects):
    """把相邻且对齐的矩形合并成长条，消掉同一条墙上的拼接缝。"""
    rects = [list(r) for r in rects]
    changed = True
    while changed:
        changed = False
        out = []
        while rects:
            a = rects.pop()
            merged = False
            for i, b in enumerate(rects):
                if a[1] == b[1] and a[3] == b[3] and (a[2] + 1 == b[0] or b[2] + 1 == a[0]):
                    rects[i] = [min(a[0], b[0]), a[1], max(a[2], b[2]), a[3]]
                    merged = changed = True
                    break
                if a[0] == b[0] and a[2] == b[2] and (a[3] + 1 == b[1] or b[3] + 1 == a[1]):
                    rects[i] = [a[0], min(a[1], b[1]), a[2], max(a[3], b[3])]
                    merged = changed = True
                    break
            if not merged:
                out.append(a)
        rects = out
    return [tuple(r) for r in rects]


def greedy_rects(grid):
    h, w = grid.shape
    used = np.zeros_like(grid, dtype=bool)
    rects = []
    for y in range(h):
        row = grid[y]
        for x in range(w):
            if not row[x] or used[y, x]:
                continue
            x1 = x
            while x1 + 1 < w and row[x1 + 1] and not used[y, x1 + 1]:
                x1 += 1
            y1 = y
            while y1 + 1 < h:
                if not grid[y1 + 1, x:x1 + 1].all() or used[y1 + 1, x:x1 + 1].any():
                    break
                y1 += 1
            used[y:y1 + 1, x:x1 + 1] = True
            rects.append((x, y, x1, y1))
    return rects


# --------------------------------------------------------------------------- #
# SDF
# --------------------------------------------------------------------------- #
PHYSICS = """    <!-- Physics settings: 与其它场地一致（0.01 s / 100 Hz），便于 RTF 对比 -->
    <physics name="default_physics" type="ode">
      <max_step_size>0.01</max_step_size>
      <real_time_update_rate>100</real_time_update_rate>
      <real_time_factor>1.0</real_time_factor>
    </physics>
"""

SDF_HEAD = """<?xml version='1.0' encoding='utf-8'?>
<!-- 由 tools/sim/drawing_to_upper_segment.py 从图纸上半段生成，请勿手改 -->
<sdf version='1.7'>
  <world name='default'>
    <gui fullscreen="0">\n      <camera name="top_down">\n        <pose>0 1.98 13.5 0 1.5708 0</pose>\n        <view_controller>orbit</view_controller>\n      </camera>\n    </gui>\n""" + PHYSICS + """
    <plugin filename="libignition-gazebo-physics-system.so" name="ignition::gazebo::systems::Physics">
    </plugin>
    <plugin filename="libignition-gazebo-scene-broadcaster-system.so" name="ignition::gazebo::systems::SceneBroadcaster">
    </plugin>
    <plugin filename="libignition-gazebo-user-commands-system.so" name="ignition::gazebo::systems::UserCommands">
    </plugin>
    <plugin filename="libignition-gazebo-sensors-system.so" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="libignition-gazebo-imu-system.so" name="ignition::gazebo::systems::Imu">
    </plugin>

    <light type='directional' name='direct_light'>
      <visualize>false</visualize>
      <cast_shadows>true</cast_shadows>
      <pose>0 0 12 0 0 0</pose>
      <diffuse>1.0 1.0 1.0 1</diffuse>
      <specular>0.5 0.5 0.5 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>0 0 -1</direction>
    </light>
"""


def link_box(name, cx, cy, cz, sx, sy, sz, rgb, with_collision=True):
    col = f"{rgb[0]:.3f} {rgb[1]:.3f} {rgb[2]:.3f} 1"
    collision = ""
    if with_collision:
        collision = f"""        <collision name='{name}_collision'>
          <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
        </collision>
"""
    return f"""      <link name='{name}'>
        <pose>{cx:.4f} {cy:.4f} {cz:.4f} 0 0 0</pose>
{collision}        <visual name='{name}_visual'>
          <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
          <material>
            <ambient>{col}</ambient>
            <diffuse>{col}</diffuse>
          </material>
        </visual>
      </link>
"""


def write_sdf(path, name, ground, walls, platforms, markings, boundary):
    p = [SDF_HEAD]
    p.append(f"""    <model name='arena_ground_upper'>
      <static>true</static>
      <link name='ground_link'>
        <pose>{ground['cx']:.4f} {ground['cy']:.4f} {ground['cz']:.4f} 0 0 0</pose>
        <collision name='ground_collision'>
          <geometry><box><size>{ground['sx']:.4f} {ground['sy']:.4f} {ground['sz']:.4f}</size></box></geometry>
        </collision>
        <visual name='ground_visual'>
          <geometry><box><size>{ground['sx']:.4f} {ground['sy']:.4f} {ground['sz']:.4f}</size></box></geometry>
          <material>
            <ambient>0.45 0.45 0.48 1</ambient>
            <diffuse>0.45 0.45 0.48 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
""")
    p.append(f"    <model name='{name}_geometry'>\n      <static>true</static>\n")
    for i, w in enumerate(walls):
        p.append(link_box(f"wall_{i:03d}", w["cx"], w["cy"], w["cz"], w["sx"], w["sy"], w["sz"],
                          (0.90, 0.90, 0.92)))
    for pl in platforms:
        p.append(link_box(pl["name"], pl["cx"], pl["cy"], pl["cz"], pl["sx"], pl["sy"], pl["sz"], pl["rgb"]))
    p.append("    </model>\n")

    p.append(f"    <model name='{name}_boundary'>\n      <static>true</static>\n")
    for i, w in enumerate(boundary):
        p.append(link_box(f"boundary_{i:02d}", w["cx"], w["cy"], w["cz"], w["sx"], w["sy"], w["sz"],
                          (0.75, 0.75, 0.80)))
    p.append("    </model>\n")

    p.append(f"    <model name='{name}_markings'>\n      <static>true</static>\n")
    for mk in markings:
        p.append(link_box(mk["name"], mk["cx"], mk["cy"], 0.002, mk["sx"], mk["sy"], 0.002,
                          mk["rgb"], with_collision=False))
    p.append("    </model>\n  </world>\n</sdf>\n")
    with open(path, "w", encoding="utf-8") as f:
        f.write("".join(p))


def write_map(pgm_path, yaml_path, occ, res, ox, oy):
    h, w = occ.shape
    with open(pgm_path, "wb") as f:
        f.write(b"P5\n")
        f.write(f"{w} {h}\n".encode())
        f.write(b"255\n")
        f.write(occ.ravel().tobytes())
    with open(yaml_path, "w", encoding="utf-8") as f:
        f.write(f"image: {os.path.basename(pgm_path)}\n")
        f.write("mode: trinary\n")
        f.write(f"resolution: {res}\n")
        f.write(f"origin: [{ox:.3f}, {oy:.3f}, 0.0]\n")
        f.write("negate: 0\n")
        f.write("occupied_thresh: 0.65\n")
        f.write("free_thresh: 0.25\n")


# --------------------------------------------------------------------------- #
def main():
    args = parse_args()
    im = Image.open(args.drawing).convert("RGB")
    a, m = build_masks(im)
    print(f"[calib] sx={SX*1000:.3f} mm/px sy={SY*1000:.3f} mm/px")
    print(f"[crop]  v={CROP_V0}..{CROP_V1} -> y=[{wy(CROP_V1):.3f}, {wy(CROP_V0):.3f}] m")

    # --- 色块（全部先裁剪） ---
    gray_bb = biggest_blob(crop_mask(m["gray"]), 8000, u_range=(680, 880), v_range=(150, 270))
    red_bb = biggest_blob(crop_mask(m["red"]), 1000)
    green_bb = biggest_blob(crop_mask(m["green"]), 1000)

    # 青色有两类，必须分开处理（复审 blocker 5）：
    #   灰平台下方那块 = 一级台阶（实体 + 占据）
    #   左侧那块「免打区」= 地面涂装（可通行）
    cyan_all = crop_mask(m["cyan"])
    cyan_step = np.zeros_like(cyan_all)
    cyan_mark = np.zeros_like(cyan_all)
    for (u0, u1, v0, v1, ar) in blobs(cyan_all, 200):
        sub = cyan_all[v0:v1 + 1, u0:u1 + 1]
        if 690 <= u0 and u1 <= 890:
            cyan_step[v0:v1 + 1, u0:u1 + 1] |= sub
        else:
            cyan_mark[v0:v1 + 1, u0:u1 + 1] |= sub
    cyan_bb = biggest_blob(cyan_step, 1000)
    if cyan_mark.any():
        vs, us = np.nonzero(cyan_mark)
        print(f"[blob] cyan-mark u[{us.min()}..{us.max()}] v[{vs.min()}..{vs.max()}] "
              f"-> x[{wx(us.min()):.2f}..{wx(us.max()):.2f}] y[{wy(vs.max()):.2f}..{wy(vs.min()):.2f}]（免打区，可通行）")

    for tag, bb in (("gray", gray_bb), ("cyan-step", cyan_bb), ("red", red_bb), ("green", green_bb)):
        if bb is None:
            print(f"[blob] {tag}: NOT FOUND")
        else:
            u0, u1, v0, v1, ar = bb
            print(f"[blob] {tag:9s} u[{u0}..{u1}] v[{v0}..{v1}] area={ar} -> "
                  f"x[{wx(u0):.2f}..{wx(u1):.2f}] y[{wy(v1):.2f}..{wy(v0):.2f}]")

    # --- 障碍掩码 ---
    floor_marks = crop_mask(m["red"] | m["green"]) | cyan_mark
    excl = np.zeros_like(crop_mask(m["light"]))
    for _bb in (gray_bb, cyan_bb):
        if _bb:
            _u0, _u1, _v0, _v1, _a = _bb
            excl[_v0:_v1 + 1, _u0:_u1 + 1] = True
    solid, mstats = solid_mask(m, floor_marks, exclude=excl)
    print("[mask]", json.dumps(mstats, ensure_ascii=False))

    # --- 栅格：唯一真源 ---
    mp = Map()
    mp.fill_field()
    mp.mark_mask(solid)

    # 0.05 m 网格（不再降采样）-> 碰撞盒
    grid_raw = (mp.occ == 0)
    grid = clean_grid(grid_raw)
    print(f"[clean] 去毛刺/补缺口: {int(grid_raw.sum())} -> {int(grid.sum())} cells")
    # 地图与几何必须用同一份清洗后的栅格
    mp.occ[grid_raw] = 254
    mp.occ[grid] = 0
    rects = merge_rects(greedy_rects(grid))
    walls = []
    for (x0, y0, x1, y1) in rects:
        cx = mp.ox + (x0 + (x1 - x0 + 1) / 2.0) * mp.res
        cy = mp.y_max - (y0 + (y1 - y0 + 1) / 2.0) * mp.res
        sxw = (x1 - x0 + 1) * mp.res
        syw = (y1 - y0 + 1) * mp.res
        walls.append(dict(cx=cx, cy=cy, cz=DEFAULTS["wall_height"] / 2.0,
                          sx=sxw, sy=syw, sz=DEFAULTS["wall_height"]))
    print(f"[geom] wall boxes = {len(walls)}")

    # --- 障碍物 / 起伏路：只做「围栏薄墙」，不做台阶、也不做起伏地形 ---
    # 口径（用户确认）：
    #   灰色块 = 障碍物，沿它与赛道接触的边界做薄墙，墙高与机器人相当或略高；
    #   青色起伏路 = 地形不用建，直接用墙围起来；
    #   红色启动区 = 也是赛道，只是出生点在那里（不围、不挡）。
    platforms = []
    enclosures = []
    ET = DEFAULTS["enclosure_thickness"]
    EH = DEFAULTS["enclosure_height"]

    def add_enclosure(tag, x0, x1, y0, y1):
        enclosures.append(dict(
            tag=tag, x0=x0, x1=x1, y0=y0, y1=y1, thick=ET, height=EH,
            bars=[
                dict(cx=(x0 + x1) / 2.0, cy=y1 + ET / 2.0, sx=(x1 - x0) + 2 * ET, sy=ET),
                dict(cx=(x0 + x1) / 2.0, cy=y0 - ET / 2.0, sx=(x1 - x0) + 2 * ET, sy=ET),
                dict(cx=x0 - ET / 2.0, cy=(y0 + y1) / 2.0, sx=ET, sy=(y1 - y0)),
                dict(cx=x1 + ET / 2.0, cy=(y0 + y1) / 2.0, sx=ET, sy=(y1 - y0)),
            ]))

    if gray_bb:
        u0, u1, v0, v1, _ = gray_bb
        gx0, gx1, gy0, gy1 = wx(u0), wx(u1), wy(v1), wy(v0)
        add_enclosure("obstacle_block", gx0, gx1, gy0, gy1)
        print(f"[enclose] obstacle_block x[{gx0:.2f}..{gx1:.2f}] y[{gy0:.2f}..{gy1:.2f}] "
              f"墙厚 {ET:.2f} m 高 {EH:.2f} m")
    if cyan_bb:
        u0, u1, v0, v1, _ = cyan_bb
        cx0, cx1, cy0, cy1 = wx(u0), wx(u1), wy(v1), wy(v0)
        add_enclosure("rough_road", cx0, cx1, cy0, cy1)
        print(f"[enclose] rough_road     x[{cx0:.2f}..{cx1:.2f}] y[{cy0:.2f}..{cy1:.2f}] "
              f"（起伏地形不建，只围起来）")

    for e in enclosures:
        for b in e["bars"]:
            # 与校验器的“盒 -> 格”约定完全一致（floor + 闭区间），否则会多占一格
            r_a = int(math.floor((mp.y_max - (b["cy"] + b["sy"] / 2)) / mp.res + 1e-9))
            r_b = int(math.floor((mp.y_max - (b["cy"] - b["sy"] / 2)) / mp.res + 1e-9))
            c_a = int(math.floor(((b["cx"] - b["sx"] / 2) - mp.ox) / mp.res + 1e-9))
            c_b = int(math.floor(((b["cx"] + b["sx"] / 2) - mp.ox) / mp.res + 1e-9))
            r0, r1 = max(0, min(r_a, r_b)), min(mp.h - 1, max(r_a, r_b))
            c0, c1 = max(0, min(c_a, c_b)), min(mp.w - 1, max(c_a, c_b))
            mp.occ[r0:r1 + 1, c0:c1 + 1] = 0    # 围栏本体：占据
            walls.append(dict(cx=b["cx"], cy=b["cy"], cz=e["height"] / 2.0,
                              sx=b["sx"], sy=b["sy"], sz=e["height"]))
        # 围内不可行驶，但也不作为障碍：记为 unknown，规划器默认不进入
        r0, r1, c0, c1 = mp.rect_span(e["x0"] + mp.res, e["x1"] - mp.res, e["y0"] + mp.res, e["y1"] - mp.res)
        mp.occ[r0:r1, c0:c1] = 127
    print(f"[geom] wall boxes = {len(walls)} (含 {len(enclosures)} 组围栏)")

    # --- 边界墙（外移到场地之外，避免压住红区/绿带） ---
    FW, FH = DEFAULTS["field_w"], DEFAULTS["field_h"]
    g = DEFAULTS["boundary_gap"]
    bt = DEFAULTS["boundary_thickness"]
    bh = DEFAULTS["boundary_height"]
    y0f, y1f = DEFAULTS["crop_y_min"], DEFAULTS["crop_y_max"]
    if args.no_crop_wall:
        y0f_wall = None
    else:
        y0f_wall = y0f
    boundary = []
    boundary.append(dict(cx=0.0, cy=y1f + g + bt / 2, cz=bh / 2, sx=FW + 2 * (g + bt), sy=bt, sz=bh))
    # 侧墙长度加 2*(g+bt)，与上下横墙端部重叠 —— 否则四角各留 0.10 m 缺口（复审 blocker 4）
    side_len = (y1f - y0f) + 2 * (g + bt)
    side_cy = (y0f + y1f) / 2
    boundary.append(dict(cx=-FW / 2 - g - bt / 2, cy=side_cy, cz=bh / 2,
                         sx=bt, sy=side_len, sz=bh))
    boundary.append(dict(cx=FW / 2 + g + bt / 2, cy=side_cy, cz=bh / 2,
                         sx=bt, sy=side_len, sz=bh))
    if y0f_wall is not None:
        boundary.append(dict(cx=0.0, cy=y0f - g - bt / 2, cz=bh / 2, sx=FW + 2 * (g + bt), sy=bt, sz=bh))
    for w in boundary:
        r0, r1, c0, c1 = mp.rect_span(w["cx"] - w["sx"] / 2, w["cx"] + w["sx"] / 2,
                                      w["cy"] - w["sy"] / 2, w["cy"] + w["sy"] / 2)
        mp.occ[r0:r1, c0:c1] = 0

    # --- 地贴 ---
    markings = []
    if red_bb:
        u0, u1, v0, v1, _ = red_bb
        markings.append(dict(name="start_zone", cx=(wx(u0) + wx(u1)) / 2, cy=(wy(v1) + wy(v0)) / 2,
                             sx=wx(u1) - wx(u0), sy=wy(v0) - wy(v1), rgb=(0.85, 0.15, 0.15)))
    if green_bb:
        u0, u1, v0, v1, _ = green_bb
        markings.append(dict(name="target_zone", cx=(wx(u0) + wx(u1)) / 2, cy=(wy(v1) + wy(v0)) / 2,
                             sx=wx(u1) - wx(u0), sy=wy(v0) - wy(v1), rgb=(0.10, 0.75, 0.35)))
    if cyan_mark.any():
        vv, uu = np.nonzero(cyan_mark)
        markings.append(dict(name="no_hit_zone",
                             cx=(wx(uu.min()) + wx(uu.max())) / 2,
                             cy=(wy(vv.max()) + wy(vv.min())) / 2,
                             sx=wx(uu.max()) - wx(uu.min()),
                             sy=wy(vv.min()) - wy(vv.max()),
                             rgb=(0.10, 0.80, 0.85)))
    for mk in markings:
        print(f"[mark] {mk['name']:12s} center=({mk['cx']:.2f},{mk['cy']:.2f}) size=({mk['sx']:.2f}x{mk['sy']:.2f})")

    # --- 统计 ---
    occ_pct = 100.0 * float((mp.occ == 0).mean())
    free_pct = 100.0 * float((mp.occ == 254).mean())
    unk_pct = 100.0 * float((mp.occ == 127).mean())
    field_occ = mp.ratio(-FW / 2, FW / 2, y0f, y1f, 0)
    field_free = mp.ratio(-FW / 2, FW / 2, y0f, y1f, 254)
    print(f"[map] {mp.w}x{mp.h} @{mp.res} origin=[{mp.ox:.3f},{mp.oy:.3f}] "
          f"occupied={occ_pct:.1f}% free={free_pct:.1f}% unknown={unk_pct:.1f}%")
    print(f"[map] field: occupied={field_occ*100:.1f}% free={field_free*100:.1f}%")

    # --- 锚点自检 ---
    def free_ratio(x0, x1, y0, y1):
        return mp.ratio(x0, x1, y0, y1, 254)

    def occ_ratio(x0, x1, y0, y1):
        return mp.ratio(x0, x1, y0, y1, 0)

    checks = []

    def add(name, value, lo, hi, note=""):
        ok = (lo is None or value >= lo) and (hi is None or value <= hi)
        checks.append(dict(name=name, value=round(value, 4), lo=lo, hi=hi, ok=bool(ok), note=note))
        print(f"[check] {'OK  ' if ok else 'FAIL'} {name:34s} = {value*100:5.1f}%  "
              f"(期望 {('%.0f%%' % (lo*100)) if lo is not None else '-'}..{('%.0f%%' % (hi*100)) if hi is not None else '-'}) {note}")

    add("spawn_3x3_free", free_ratio(SPAWN["x"] - 0.10, SPAWN["x"] + 0.10,
                                     SPAWN["y"] - 0.10, SPAWN["y"] + 0.10), 0.999, None)
    add("start_zone_free", free_ratio(5.40, 5.85, 1.60, 2.25), 0.95, None)
    add("start_exit_free", free_ratio(4.90, 5.20, 1.62, 2.09), 0.80, None)
    add("right_wall_occupied", occ_ratio(4.90, 5.20, 2.35, 3.10), 0.58, None)
    add("center_small_obstacle", occ_ratio(-0.55, -0.37, 1.30, 2.50), 0.50, None)
    # 墙体带体：x=0.55..0.95 实测 light 2093 px / deep 0 px，颜色就是浅紫 = 墙。
    add("notch_wall_body", occ_ratio(0.55, 0.95, 2.40, 3.50), 0.80, None, "浅紫=墙")
    # 场外验证：左侧靶标区（蓝墙之外）必须是「不可行驶」
    add("left_apron_is_outside", occ_ratio(-4.20, -3.60, 1.60, 2.40), 0.90, None, "浅紫=场外")
    # 赛道开阔区必须是自由
    add("track_open_free", free_ratio(-1.30, -0.80, 1.60, 2.20), 0.90, None, "深紫=赛道")
    # 围栏验证（不复用「先写 occ 再查 occ」的恒真式）：查围栏本体是否占据、
    # 围内是否 unknown、以及围内不是 free。
    for e in enclosures:
        per = []
        for b in e["bars"]:
            per.append(occ_ratio(b["cx"] - b["sx"] / 2 + 0.01, b["cx"] + b["sx"] / 2 - 0.01,
                                 b["cy"] - b["sy"] / 2 + 0.01, b["cy"] + b["sy"] / 2 - 0.01))
        add(f"enclosure_{e['tag']}_ring", float(np.mean(per)), 0.90, None, "围栏本体应占据")
        inner = mp.ratio(e["x0"] + 0.05, e["x1"] - 0.05, e["y0"] + 0.05, e["y1"] - 0.05, 127)
        add(f"enclosure_{e['tag']}_inside_unknown", inner, 0.90, None, "围内应为 unknown")
        inner_free = free_ratio(e["x0"] + 0.05, e["x1"] - 0.05, e["y0"] + 0.05, e["y1"] - 0.05)
        add(f"enclosure_{e['tag']}_inside_not_free", 1.0 - inner_free, 0.90, None, "围内不应可行驶")
    if cyan_mark.any():
        vs2, us2 = np.nonzero(cyan_mark)
        add("cyan_mark_free",
            free_ratio(wx(us2.min()) + 0.03, wx(us2.max()) - 0.03,
                       wy(vs2.max()) + 0.03, wy(vs2.min()) - 0.03), 0.90, None, "免打区可行驶")

    # 连通性：出生点能到达哪些目标点（A10 口径：free 做 0.25 m 腐蚀后再看连通）
    free = (mp.occ == 254)
    free_lab, _ = ndimage.label(free)
    sc, sr = mp.col(SPAWN["x"]), mp.row(SPAWN["y"])
    spawn_comp = int(free_lab[sr, sc])
    raw_reach = int((free_lab == spawn_comp).sum()) if spawn_comp else 0
    total_free = int(free.sum())
    # 主连通域：出生点所在自由连通域必须是「最大」的那块。
    # 注意左侧靶标区是被蓝墙隔开的场外地面（图纸如此），它自成一域，不算缺陷，
    # 所以这里不比「占全部自由格的比例」，只要求出生域是最大域。
    _sizes = np.bincount(free_lab.ravel())
    _largest = int(_sizes[1:].argmax() + 1) if _sizes.size > 1 else 0
    add("spawn_component_is_largest", 1.0 if spawn_comp == _largest else 0.0, 1.0, None,
        "出生域 %d 格 / 最大域 %d 格" % (raw_reach, int(_sizes[_largest]) if _largest else 0))

    # 启动口袋 / 出口净宽：沿 y 方向量 x=5.05 处的连续自由长度
    def vertical_free_run(xc, y_lo, y_hi):
        c = mp.col(xc)
        best = 0
        cur = 0
        for r in range(mp.row(y_hi), mp.row(y_lo) + 1):
            if mp.occ[r, c] == 254:
                cur += 1
                best = max(best, cur)
            else:
                cur = 0
        return best * mp.res

    gate_w = vertical_free_run(5.05, 1.30, 2.60)
    # 出口净宽必须能过车体（min_corridor=0.50 m），不再是「仅记录」
    add("start_gate_width", gate_w / DEFAULTS["min_corridor"], 1.0, None,
        "x=5.05 净宽 %.2f m (要求 >=%.2f m)" % (gate_w, DEFAULTS["min_corridor"]))

    # 可通行性（稳健口径，替代之前的 maximin 单点法）：
    #  1) 按底盘半宽做腐蚀，看出生点能覆盖多少场内自由空间；
    #  2) 用阈值扫描求「出生点 -> 场内最开阔点」的最小净空（等价 maximin，但不依赖路径重建）。
    free_dt = ndimage.distance_transform_edt(free) * mp.res
    field_mask = np.zeros_like(free)
    fr0, fr1, fc0, fc1 = mp.rect_span(-6.0, 6.0, y0f, y1f)
    field_mask[fr0:fr1, fc0:fc1] = True
    field_free_cells = free & field_mask

    half_robot = DEFAULTS["min_corridor"] / 2.0
    er = ndimage.binary_erosion(free, structure=disk(int(round(half_robot / mp.res))))
    er_lab, er_n = ndimage.label(er)
    spawn_er = int(er_lab[mp.row(SPAWN["y"]), mp.col(SPAWN["x"])])
    reach = int((er_lab == spawn_er).sum()) if spawn_er else 0
    n_field_free = max(1, int(field_free_cells.sum()))
    add("reachable_area_ratio", reach / n_field_free, 0.40, None,
        "半宽 %.2f m 腐蚀后可达 %d 格 (场内自由 %d)" % (half_robot, reach, n_field_free))
    add("reachable_area_m2", reach * mp.res * mp.res / 8.0, 1.0, None, "可达面积 %.1f m2 (要求 >=8 m2)" % (reach * mp.res * mp.res))
    add("reachable_single_component", 1.0 if er_n == 1 else 0.0, 1.0, None,
        "腐蚀后连通域数 %d" % er_n)

    # 目标点：出生点所在自由域里离障碍最远的那格
    in_comp = (free_lab == spawn_comp) if spawn_comp else np.zeros_like(free)
    score = np.where(in_comp, free_dt, -1.0)
    tr, tc = np.unravel_index(int(np.argmax(score)), score.shape)
    target = (mp.x_of(tc), mp.y_of(tr))
    sr3, sc3 = mp.row(SPAWN["y"]), mp.col(SPAWN["x"])
    best_t = 0.0
    for i in range(1, int(round(0.50 / mp.res)) + 1):
        t = i * mp.res
        m_ok = ndimage.binary_erosion(free, structure=disk(i))
        if not (m_ok[sr3, sc3] and m_ok[tr, tc]):
            break
        lab_t, _ = ndimage.label(m_ok)
        if lab_t[sr3, sc3] == lab_t[tr, tc]:
            best_t = t
        else:
            break
    add("trav_clearance", best_t / half_robot, 1.0, None,
        "出生点 -> (%.2f,%.2f) 最小净空 %.2f m (通道 %.2f m，要求 >=%.2f m)" % (
            target[0], target[1], best_t, 2 * best_t, DEFAULTS["min_corridor"]))
    report = dict(
        version="v3",
        name=args.name,
        crop_y=[y0f, y1f],
        calib=dict(sx=SX, sy=SY, u0=CAL["U0"], u1=CAL["U1"], v0=CAL["V0"], v1=CAL["V1"]),
        map=dict(w=mp.w, h=mp.h, res=mp.res, origin=[mp.ox, mp.oy],
                 occupied=round(occ_pct, 2), free=round(free_pct, 2), unknown=round(unk_pct, 2),
                 field_occupied=round(field_occ * 100, 2), field_free=round(field_free * 100, 2)),
        counts=dict(wall_boxes=len(walls), enclosures=len(enclosures),
                    boundary=len(boundary), markings=len(markings)),
        params=dict(wall_height=DEFAULTS["wall_height"],
                    enclosure_thickness=DEFAULTS["enclosure_thickness"],
                    enclosure_height=DEFAULTS["enclosure_height"],
                    min_corridor=DEFAULTS["min_corridor"],
                    cell=mp.res, crop_wall=(not args.no_crop_wall)),
        mask=mstats,
        spawn=SPAWN,
        anchors=checks,
        anchors_ok=all(c["ok"] for c in checks),
    )

    # --- 写 world ---
    os.makedirs(args.out_world, exist_ok=True)
    world_path = os.path.join(args.out_world, f"{args.name}_world.sdf")
    ymid = (y0f + y1f) / 2
    ground = dict(cx=0.0, cy=mp.y_max - mp.h * mp.res / 2, cz=-0.005,
                  sx=mp.w * mp.res, sy=mp.h * mp.res, sz=0.01)
    write_sdf(world_path, args.name, ground, walls, platforms, markings, boundary)
    print(f"[write] {world_path}")

    # --- 写 map ---
    os.makedirs(args.out_map, exist_ok=True)
    pgm = os.path.join(args.out_map, f"{args.name}.pgm")
    yml = os.path.join(args.out_map, f"{args.name}.yaml")
    write_map(pgm, yml, mp.occ, mp.res, mp.ox, mp.oy)
    print(f"[write] {pgm}")
    print(f"[write] {yml}")

    # --- 预览 ---
    if args.preview:
        os.makedirs(os.path.dirname(args.preview) or ".", exist_ok=True)
        prev = a.astype(np.uint8).copy()
        prev[solid] = (255, 60, 60)
        img = Image.fromarray(prev)
        dr = ImageDraw.Draw(img)
        if gray_bb:
            u0, u1, v0, v1, _ = gray_bb
            dr.rectangle([u0, v0, u1, v1], outline=(255, 220, 0), width=2)
        if cyan_bb:
            u0, u1, v0, v1, _ = cyan_bb
            dr.rectangle([u0, v0, u1, v1], outline=(0, 255, 255), width=2)
        if red_bb:
            u0, u1, v0, v1, _ = red_bb
            dr.rectangle([u0, v0, u1, v1], outline=(255, 120, 120), width=2)
        dr.line([CAL["U0"], CROP_V1, CAL["U1"], CROP_V1], fill=(255, 255, 0), width=1)
        su = int((SPAWN["x"] + FW / 2) / SX + CAL["U0"])
        sv = int((FH / 2 - SPAWN["y"]) / SY + CAL["V0"])
        dr.ellipse([su - 4, sv - 4, su + 4, sv + 4], outline=(255, 255, 255), width=2)
        dr.line([su, sv, su - 30, sv], fill=(255, 255, 255), width=2)
        img.save(args.preview)
        print(f"[write] {args.preview}")

    if args.report:
        with open(args.report, "w", encoding="utf-8") as f:
            json.dump(report, f, indent=2, ensure_ascii=False)
        print(f"[write] {args.report}")

    print(json.dumps({k: report[k] for k in ("map", "counts", "anchors_ok")}, indent=2, ensure_ascii=False))
    failed = [c["name"] for c in checks if not c["ok"]]
    if failed:
        print(f"[FAIL] anchors: {failed}")
        return 1
    print("[OK] all anchors passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())