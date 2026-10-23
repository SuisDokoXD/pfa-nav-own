#!/usr/bin/env python3
"""把官方场地 CAD（STEP）转换成 Gazebo 仿真世界可用的 STL 网格。

流程：
    1. OCP(OpenCASCADE) 读取 STEP（官方图纸通常是毫米、场地坐标系不居中）
    2. 三角化成 STL（deflection 控制精度）
    3. 平移到「场地中心 = 原点、地板 = z0」，并把毫米换算成米
    4. 体素抽稀，分别产出视觉用（细）与碰撞用（粗）两个网格

依赖：pip install cadquery-ocp（提供 OCP）+ numpy

用法：
    # 先探查尺寸、地板高度与场地中心
    python3 cad_to_field_mesh.py --step field.stp --probe

    # 正式导出（自动使用探查得到的偏移）
    python3 cad_to_field_mesh.py --step field.stp --out-dir meshes --name rmuc_2027

说明：官方 CAD 里地板往往不在 z=0（RMUC2027 为 z=-1.94m），场地在 y 方向也
不居中，因此脚本用「水平面面积最大的那一层」作为地板，用包围盒中点作为场地中心。
"""
import argparse
import os
import struct
import sys
import time

import numpy as np

from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_Reader
from OCP.StlAPI import StlAPI_Writer


def read_step(path):
    t0 = time.time()
    reader = STEPControl_Reader()
    if reader.ReadFile(path) != IFSelect_RetDone:
        sys.exit('STEP 读取失败')
    reader.TransferRoots()
    shape = reader.OneShape()
    if shape.IsNull():
        sys.exit('STEP 内容为空')
    print(f'[read] {time.time() - t0:.1f}s')
    return shape


def mesh_to_stl(shape, dst, deflection):
    t0 = time.time()
    BRepMesh_IncrementalMesh(shape, deflection, False, 0.5, True)
    writer = StlAPI_Writer()
    writer.ASCIIMode = False
    if not writer.Write(shape, dst):
        sys.exit('STL 写出失败')
    print(f'[mesh] deflection={deflection}mm {time.time() - t0:.1f}s -> {dst}')


def load_stl(path):
    raw = np.fromfile(path, dtype=np.uint8)
    header = raw[:80]
    count = struct.unpack('<I', raw[80:84].tobytes())[0]
    body = raw[84:84 + count * 50].reshape(count, 50)
    tri = body[:, 12:48].copy().view('<f4').reshape(count, 3, 3).astype(np.float64)
    return header, tri


def write_stl(path, header, tri):
    count = len(tri)
    v0 = tri[:, 1] - tri[:, 0]
    v1 = tri[:, 2] - tri[:, 0]
    normals = np.cross(v0, v1)
    lengths = np.linalg.norm(normals, axis=1, keepdims=True)
    lengths[lengths == 0] = 1.0
    normals /= lengths

    out = np.zeros((count, 50), dtype=np.uint8)
    out[:, 0:12] = np.frombuffer(normals.astype('<f4').tobytes(),
                                 dtype=np.uint8).reshape(count, 12)
    out[:, 12:48] = np.frombuffer(tri.astype('<f4').tobytes(),
                                  dtype=np.uint8).reshape(count, 36)

    with open(path, 'wb') as fh:
        fh.write(header.tobytes()[:80].ljust(80, b'\0'))
        fh.write(struct.pack('<I', count))
        fh.write(out.tobytes())


def horizontal_floor(tri):
    """返回（地板高度, 水平面面积较大的前几层）——单位与输入一致。"""
    v0 = tri[:, 1] - tri[:, 0]
    v1 = tri[:, 2] - tri[:, 0]
    cross = np.cross(v0, v1)
    area = 0.5 * np.linalg.norm(cross, axis=1)
    nz = cross[:, 2] / np.maximum(2.0 * area, 1e-12)
    zt = tri[:, :, 2].mean(axis=1)

    buckets = {}
    for idx in np.nonzero(np.abs(nz) > 0.9)[0]:
        key = round(float(zt[idx]), 2)
        buckets[key] = buckets.get(key, 0.0) + float(area[idx])
    if not buckets:
        sys.exit('未找到水平面，无法自动定位地板')
    ranked = sorted(buckets.items(), key=lambda kv: -kv[1])
    return ranked[0][0], ranked[:5]


def decimate(tri, grid):
    snapped = np.round(tri / grid).astype(np.int64)
    keep = ~(
        ((snapped[:, 0] == snapped[:, 1]).all(axis=1))
        | ((snapped[:, 1] == snapped[:, 2]).all(axis=1))
        | ((snapped[:, 0] == snapped[:, 2]).all(axis=1))
    )
    return snapped[keep].astype(np.float64) * grid


def main():
    ap = argparse.ArgumentParser(description='官方场地 CAD(STEP) -> Gazebo STL 网格')
    ap.add_argument('--step', required=True, help='输入的 .stp/.step 文件')
    ap.add_argument('--out-dir', default='.', help='输出目录')
    ap.add_argument('--name', default='field', help='输出网格名前缀，如 rmuc_2027')
    ap.add_argument('--deflection', type=float, default=20.0, help='三角化精度(mm)')
    ap.add_argument('--visual-grid', type=float, default=0.012, help='视觉网格体素(m)')
    ap.add_argument('--collision-grid', type=float, default=0.04, help='碰撞网格体素(m)')
    ap.add_argument('--scale', type=float, default=0.001, help='单位换算，默认 mm->m')
    ap.add_argument('--floor-z', type=float, default=None, help='地板高度(输入单位)，默认自动探测')
    ap.add_argument('--center-x', type=float, default=None, help='场地中心 x(输入单位)')
    ap.add_argument('--center-y', type=float, default=None, help='场地中心 y(输入单位)')
    ap.add_argument('--probe', action='store_true', help='只探查尺寸/地板/中心，不导出')
    args = ap.parse_args()

    shape = read_step(args.step)
    tmp = os.path.join(args.out_dir, f'_{args.name}_raw.stl')
    os.makedirs(args.out_dir, exist_ok=True)
    mesh_to_stl(shape, tmp, args.deflection)

    header, tri_mm = load_stl(tmp)
    lo = tri_mm.reshape(-1, 3).min(axis=0)
    hi = tri_mm.reshape(-1, 3).max(axis=0)
    floor_z, ranked = horizontal_floor(tri_mm)

    print('\n=== 探查结果（输入单位）===')
    print(f'  bbox min=({lo[0]:.1f}, {lo[1]:.1f}, {lo[2]:.1f}) '
          f'max=({hi[0]:.1f}, {hi[1]:.1f}, {hi[2]:.1f})')
    print(f'  size=({hi[0] - lo[0]:.1f}, {hi[1] - lo[1]:.1f}, {hi[2] - lo[2]:.1f})')
    print(f'  自动判定地板 z={floor_z}（水平面面积前三: '
          + ', '.join(f'z={z}:{a:.0f}' for z, a in ranked[:3]) + '）')

    floor_z = floor_z if args.floor_z is None else args.floor_z
    center_x = (lo[0] + hi[0]) / 2.0 if args.center_x is None else args.center_x
    center_y = (lo[1] + hi[1]) / 2.0 if args.center_y is None else args.center_y
    if args.probe:
        os.remove(tmp)
        print(f'\n  将使用: 地板 z={floor_z}, 中心=({center_x:.1f}, {center_y:.1f})'
              f'  换算系数={args.scale}')
        return

    offset = np.array([-center_x * args.scale, -center_y * args.scale, -floor_z * args.scale])
    base = tri_mm * args.scale + offset
    print(f'\n=== 归一化 ===\n  偏移(米)={offset.round(4)}')

    for tag, grid in (('visual', args.visual_grid), ('collision', args.collision_grid)):
        dec = decimate(base, grid)
        dst = os.path.join(args.out_dir, f'{args.name}_{tag}.stl')
        write_stl(dst, header, dec)
        lo2 = dec.reshape(-1, 3).min(axis=0)
        hi2 = dec.reshape(-1, 3).max(axis=0)
        print(f'  {tag}: {len(dec):,} 面  {os.path.getsize(dst) / 1024 / 1024:.1f} MB  '
              f'x∈[{lo2[0]:.2f},{hi2[0]:.2f}] y∈[{lo2[1]:.2f},{hi2[1]:.2f}] '
              f'z∈[{lo2[2]:.2f},{hi2[2]:.2f}]')

    os.remove(tmp)
    print('完成。')


if __name__ == '__main__':
    main()
