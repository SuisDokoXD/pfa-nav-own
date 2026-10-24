#!/usr/bin/env python3
"""机器人 CAD(STEP) -> Gazebo 网格(STL)。

与 cad_to_field_mesh.py（场地）同一套路：OCP 读 STEP -> 三角化 -> 平移/缩放 -> 写 STL。
区别是机器人要按 link 分组，并把每个 link 的网格原点摆到关节坐标系上。

用法示例：
    # 舵轮模块里的单个轮子（轮子在模块装配体里只有一个）
    python3 cad_to_robot_mesh.py \
        --step 轮组总装.STEP --out wheel_left.stl --frame wheel \
        --include 包胶轮 轮毂 --report wheel.report.json

    # 只统计不导出
    python3 cad_to_robot_mesh.py --step 新舵轮.STEP --list

坐标约定：
    STEP 来自 SolidWorks，Y 轴向上；Gazebo 是 Z 轴向上。
    本脚本用 (x, y, z)_step -> (x, -z, y)_sdf 的右手映射，再做 link 内部摆正。

依赖：pip install cadquery-ocp numpy
"""
import argparse
import json
import os
import struct
import sys
import time

import numpy as np

from OCP.Bnd import Bnd_Box
from OCP.BRep import BRep_Builder
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepGProp import BRepGProp
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.GProp import GProp_GProps
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.StlAPI import StlAPI_Writer
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDF import TDF_Label, TDF_LabelSequence, TDF_AttributeIterator
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS_Compound
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ShapeTool

NAME_TYPES = ('TDataStd_Name', 'TDataStd_GenericExtString')
# 装配体里的辅助几何：地面、限定面、干涉/范围包络体，不参与网格
JUNK_KEYS = ('地面', '限定面', '干涉范围', '范围')


def label_name(lab):
    it = TDF_AttributeIterator(lab)
    while it.More():
        a = it.Value()
        try:
            if a.DynamicType().Name() in NAME_TYPES:
                s = a.Get().ToExtString()
                if s:
                    return str(s)
        except Exception:
            pass
        it.Next()
    return ''


def part_name(lab):
    inst = label_name(lab)
    try:
        ref = TDF_Label()
        if XCAFDoc_ShapeTool.GetReferredShape_s(lab, ref):
            nm = label_name(ref)
            if nm:
                return nm
    except Exception:
        pass
    return inst


def asm_target(lab):
    if XCAFDoc_ShapeTool.IsAssembly_s(lab):
        return lab
    try:
        ref = TDF_Label()
        if XCAFDoc_ShapeTool.GetReferredShape_s(lab, ref) and XCAFDoc_ShapeTool.IsAssembly_s(ref):
            return ref
    except Exception:
        pass
    return None


def bbox_mm(shape):
    b = Bnd_Box()
    BRepBndLib.Add_s(shape, b, True)
    return list(b.Get())


def vol_cm3(shape):
    p = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, p)
    return p.Mass() / 1000.0


def n_solids(shape):
    ex = TopExp_Explorer(shape, TopAbs_SOLID)
    n = 0
    while ex.More():
        n += 1
        ex.Next()
    return n


def read_step_parts(path):
    """STEP -> [{name, trail, shape(装配体坐标), bbox_mm, vol}]"""
    t0 = time.time()
    doc = TDocStd_Document(TCollection_ExtendedString('MDTV-XCAF'))
    rd = STEPCAFControl_Reader()
    rd.SetNameMode(True)
    rd.SetColorMode(False)
    rd.SetLayerMode(False)
    if rd.ReadFile(path) != IFSelect_RetDone:
        sys.exit('STEP 读取失败: ' + path)
    rd.Transfer(doc)
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    roots = TDF_LabelSequence()
    tool.GetFreeShapes(roots)

    out = []

    def walk(lab, trsf, trail):
        loc = XCAFDoc_ShapeTool.GetLocation_s(lab)
        cur = trsf.Multiplied(loc.Transformation())
        nm = part_name(lab)
        tgt = asm_target(lab)
        if tgt is not None:
            comps = TDF_LabelSequence()
            XCAFDoc_ShapeTool.GetComponents_s(tgt, comps, False)
            for i in range(1, comps.Length() + 1):
                walk(comps.Value(i), cur, trail + [nm or 'asm'])
            return
        shp = XCAFDoc_ShapeTool.GetShape_s(lab)
        if shp.IsNull():
            return
        shp = shp.Moved(TopLoc_Location(cur))
        if n_solids(shp) == 0:
            return
        out.append({'name': nm, 'trail': '/'.join(trail), 'shape': shp,
                    'bbox_mm': bbox_mm(shp), 'vol_cm3': vol_cm3(shp)})

    for i in range(1, roots.Length() + 1):
        r = roots.Value(i)
        walk(r, XCAFDoc_ShapeTool.GetLocation_s(r).Transformation(), [])
    print('[step] %s: %d 个实体零件, %.1fs' % (os.path.basename(path), len(out), time.time() - t0))
    return out


def filter_parts(parts, include, exclude, include_trail):
    keep, drop = [], []
    for p in parts:
        nm = p['name'] or ''
        if any(k in nm for k in JUNK_KEYS):
            drop.append((p, 'junk'))
            continue
        if include_trail and not any(k in p['trail'] for k in include_trail):
            drop.append((p, 'trail'))
            continue
        if include and not any(k in nm for k in include):
            drop.append((p, 'not-include'))
            continue
        if exclude and any(k in nm for k in exclude):
            drop.append((p, 'exclude'))
            continue
        keep.append(p)
    return keep, drop


def mesh_to_tri(shape, deflection, tmp):
    BRepMesh_IncrementalMesh(shape, deflection, False, 0.5, True)
    writer = StlAPI_Writer()
    writer.ASCIIMode = False
    if not writer.Write(shape, tmp):
        sys.exit('STL 写出失败')
    return load_stl(tmp)


def load_stl(path):
    raw = np.fromfile(path, dtype=np.uint8)
    count = struct.unpack('<I', raw[80:84].tobytes())[0]
    body = raw[84:84 + count * 50].reshape(count, 50)
    return body[:, 12:48].copy().view('<f4').reshape(count, 3, 3).astype(np.float64)


def write_stl(path, tri, header=b'pfa-nav robot mesh'):
    count = len(tri)
    v0 = tri[:, 1] - tri[:, 0]
    v1 = tri[:, 2] - tri[:, 0]
    n = np.cross(v0, v1)
    ln = np.linalg.norm(n, axis=1, keepdims=True)
    ln[ln == 0] = 1.0
    n /= ln
    out = np.zeros((count, 50), dtype=np.uint8)
    out[:, 0:12] = np.frombuffer(n.astype('<f4').tobytes(), dtype=np.uint8).reshape(count, 12)
    out[:, 12:48] = np.frombuffer(tri.astype('<f4').tobytes(), dtype=np.uint8).reshape(count, 36)
    with open(path, 'wb') as fh:
        fh.write(header[:80].ljust(80, b'\0'))
        fh.write(struct.pack('<I', count))
        fh.write(out.tobytes())


def to_sdf_axes(tri):
    """(x, y, z)_step(Y上) -> (x, -z, y)_sdf(Z上)，右手系保持。"""
    xy = tri[..., [0, 2, 1]].copy()
    xy[..., 1] *= -1.0
    return xy


def align_wheel(tri, ref=None):
    """把轮轴摆到 Y 轴、轮心移到原点。返回 (tri, R, width, note)。

    轮子在装配体里常带一个转向角，包围盒会把轮子撑成斜的方块，所以这里用
    面积加权的惯性主轴（PCA）求轮轴：圆柱体沿轴向的方差最小。
    ref: 用于定轴的参考面片（轮胎本体）；None 时用 tri 自己。
    """
    r = tri if ref is None else ref
    v0 = r[:, 1] - r[:, 0]
    v1 = r[:, 2] - r[:, 0]
    area = 0.5 * np.linalg.norm(np.cross(v0, v1), axis=1)
    pts = r.reshape(-1, 3)
    w = np.repeat(area / 3.0, 3)
    if w.sum() <= 0:
        sys.exit('参考面片面积为 0，无法定轴')
    c = (pts * w[:, None]).sum(axis=0) / w.sum()
    d = pts - c
    cov = (d * w[:, None]).T @ d / w.sum()
    evals, evecs = np.linalg.eigh(cov)
    axle = evecs[:, 0]
    u = evecs[:, 2]
    v = np.cross(axle, u)
    basis = np.column_stack([u, axle, v])
    if np.linalg.det(basis) < 0:
        basis[:, 2] *= -1.0
    rot = basis.T                      # 世界 -> 轮子坐标系
    tri = np.einsum('ij,tkj->tki', rot, tri - c)
    rr = np.einsum('ij,tkj->tki', rot, r - c)
    radial = rr[..., [0, 2]].reshape(-1, 2)
    radius = float(np.linalg.norm(radial, axis=1).max())
    width = float(rr[..., 1].max() - rr[..., 1].min())
    note = 'PCA axle (方差最小方向) -> Y'
    return tri, radius, width, note


def main():
    ap = argparse.ArgumentParser(description='机器人 STEP -> Gazebo link 网格')
    ap.add_argument('--step', required=True)
    ap.add_argument('--out')
    ap.add_argument('--frame', choices=['wheel', 'center', 'raw'], default='center')
    ap.add_argument('--include', nargs='*', default=[])
    ap.add_argument('--exclude', nargs='*', default=[])
    ap.add_argument('--include-trail', nargs='*', default=[])
    ap.add_argument('--anchor', nargs='*', default=[],
                    help='用这些零件定坐标（如轮胎本体），网格仍包含全部保留件')
    ap.add_argument('--deflection', type=float, default=0.3, help='三角化弦差(mm)')
    ap.add_argument('--scale', type=float, default=0.001, help='mm->m')
    ap.add_argument('--report')
    ap.add_argument('--list', action='store_true', help='只打印零件清单，不导出')
    ap.add_argument('--max-faces', type=int, default=0, help='>0 时按体素抽样压面数')
    args = ap.parse_args()

    parts = read_step_parts(args.step)
    keep, drop = filter_parts(parts, args.include, args.exclude, args.include_trail)

    if args.list:
        for p in sorted(parts, key=lambda q: -q['vol_cm3']):
            b = p['bbox_mm']
            print('%-30s size=(%7.1f,%7.1f,%7.1f) V=%8.1f trail=%s'
                  % (p['name'][:30], b[3] - b[0], b[4] - b[1], b[5] - b[2],
                     p['vol_cm3'], p['trail'][:38]))
        print('合计 %d 个' % len(parts))
        return

    if not keep:
        sys.exit('过滤后没有零件，检查 --include/--exclude')
    print('[filter] 保留 %d / 丢弃 %d' % (len(keep), len(drop)))
    for p, why in drop[:12]:
        print('   丢 %-28s (%s)' % ((p['name'] or '?')[:28], why))

    bb = BRep_Builder()
    comp = TopoDS_Compound()
    bb.MakeCompound(comp)
    for p in keep:
        bb.Add(comp, p['shape'])

    if args.out:
        os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    tmp = args.out + '.tmp.stl' if args.out else None
    tri = mesh_to_tri(comp, args.deflection, tmp)
    if tmp and os.path.exists(tmp):
        os.remove(tmp)
    print('[mesh] %.1f mm 弦差 -> %d 面' % (args.deflection, len(tri)))

    ref = None
    if args.anchor:
        anchor = [p for p in keep if any(k in (p['name'] or '') for k in args.anchor)]
        if not anchor:
            sys.exit('--anchor 没有匹配到任何零件')
        cbb = BRep_Builder()
        ctmp = TopoDS_Compound()
        cbb.MakeCompound(ctmp)
        for p in anchor:
            cbb.Add(ctmp, p['shape'])
        ref_tmp = (args.out or 'ref') + '.ref.tmp.stl'
        ref = mesh_to_tri(ctmp, args.deflection, ref_tmp)
        if os.path.exists(ref_tmp):
            os.remove(ref_tmp)
        print('[anchor] %d 个零件定坐标: %s'
              % (len(anchor), ', '.join(p['name'] for p in anchor)))

    tri = to_sdf_axes(tri)
    ref = None if ref is None else to_sdf_axes(ref)
    info = {}
    if args.frame == 'wheel':
        tri, radius, width, note = align_wheel(tri, ref)
        info = {'kind': 'wheel', 'radius_m': radius * args.scale,
                'width_m': width * args.scale, 'axis': note}
        print('[frame] 轮轴 %s, 半径 %.1f mm, 宽 %.1f mm' % (note, radius, width))
    elif args.frame == 'center':
        base = tri if ref is None else ref
        lo = base.reshape(-1, 3).min(axis=0)
        hi = base.reshape(-1, 3).max(axis=0)
        tri = tri - (lo + hi) / 2.0
        info = {'kind': 'center', 'size_mm': (hi - lo).tolist()}

    tri = tri * args.scale
    if args.max_faces > 0 and len(tri) > args.max_faces:
        grid = 0.002
        snapped = np.round(tri / grid).astype(np.int64)
        while True:
            keepmask = ~(((snapped[:, 0] == snapped[:, 1]).all(axis=1))
                         | ((snapped[:, 1] == snapped[:, 2]).all(axis=1))
                         | ((snapped[:, 0] == snapped[:, 2]).all(axis=1)))
            tri = snapped[keepmask].astype(np.float64) * grid
            snapped = np.round(tri / grid).astype(np.int64)
            if len(tri) <= args.max_faces:
                break
            grid *= 1.5
        print('[decimate] -> %d 面 (体素 %.4f m)' % (len(tri), grid))

    lo = tri.reshape(-1, 3).min(axis=0)
    hi = tri.reshape(-1, 3).max(axis=0)
    print('[out] bbox(m) x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'
          % (lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]))
    if args.out:
        write_stl(args.out, tri)
        print('[out] %s  %.2f MB  %d 面'
              % (args.out, os.path.getsize(args.out) / 1048576.0, len(tri)))
    if args.report:
        rep = {'step': args.step, 'frame': args.frame, 'n_parts': len(keep),
               'parts': [p['name'] for p in keep],
               'faces': len(tri), 'bbox_m': [lo.tolist(), hi.tolist()]}
        rep.update(info)
        with open(args.report, 'w', encoding='utf-8') as fh:
            json.dump(rep, fh, ensure_ascii=False, indent=1)
        print('[report] ' + args.report)


if __name__ == '__main__':
    main()
