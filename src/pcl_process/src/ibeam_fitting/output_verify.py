#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
工字钢检测结果可视化验证脚本
用于3D可视化验证工字钢识别结果（含平面方程、切片、工作点、OBB全要素）

使用方法:
1. 从日志文件解析: python3 output_verify.py --file log.txt
2. 从标准输入解析: python3 output_verify.py --stdin (然后粘贴日志内容，按Ctrl+D结束)
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import argparse
import re
import sys
import os


# ─────────────────────────────────────────────
# 数学工具
# ─────────────────────────────────────────────

def normalize(v):
    v = np.array(v, dtype=float)
    n = np.linalg.norm(v)
    return v / n if n > 1e-9 else v


def build_local_frame(main_axis):
    """由主轴构造局部坐标系 (u, v, w): u=主轴, v=法向, w=截面高度方向"""
    u   = normalize(main_axis)
    w_g = np.array([0.0, 0.0, 1.0])
    v   = np.cross(w_g, u)
    if np.linalg.norm(v) < 1e-6:
        v = np.array([0.0, 1.0, 0.0])
    v = normalize(v)
    w = normalize(np.cross(u, v))
    return u, v, w


def verify_work_point(surface_point_m, work_point_m, normal, d_base_m):
    """验证工作点 = 表面点 + normal * d_base，返回(期望值, 误差mm)"""
    expected = np.array(surface_point_m) + normalize(normal) * d_base_m
    err      = np.linalg.norm(expected - np.array(work_point_m)) * 1000
    return expected, err


# ─────────────────────────────────────────────
# 日志解析
# ─────────────────────────────────────────────

def _floats(s):
    return [float(x.strip()) for x in s.split(',')]


def parse_log_content(log_content):
    """
    从日志内容中解析工字钢所有信息。
    支持新格式 plane_coeffs=(a,b,c,d) 及旧格式 normal=(x,y,z)。
    """
    result = {
        # 每切片数据（基座坐标系，单位 m）
        'slice_ids':         [],
        'surface_points':    [],   # m
        'work_points':       [],   # m
        'normals':           [],   # 单位向量（取前3分量）
        'plane_coeffs':      [],   # 4分量 [a,b,c,d]，坐标单位 m
        'main_axes':         [],
        'slice_widths':      [],   # mm
        'slice_heights':     [],   # mm
        'wing_plate_widths': [],   # mm

        # 工字钢检测结果
        'beam_center_lidar': None,
        'beam_dimensions':   None,  # [L, W, H] m
        'beam_center_base':  None,  # m
        'beam_height_mm':    None,
        'working_length_mm': None,
        'wing_plate_width_mm': None,
        'plane_coeffs_lidar': None, # 4分量，雷达坐标系
        'plane_rx_lidar':    None,
    }

    # ── 新格式: plane_coeffs=(a,b,c,d)，单位 mm ─────────────────────────
    new_pat = re.compile(
        r'Published target info for slice_id (\d+):\s*'
        r'surface_point=\(([^)]+)\)\s*mm,\s*'
        r'work_point=\(([^)]+)\)\s*mm,'
        r'plane_coeffs=\(([^)]+)\),\s*'
        r'main_axis=\(([^)]+)\),'
        r'slice_width=([\d.]+)\s*mm,\s*slice_height=([\d.]+)\s*mm,\s*wing_plate_width=([\d.]+)'
    )
    for m in new_pat.finditer(log_content):
        pc = _floats(m.group(4))
        result['slice_ids'].append(int(m.group(1)))
        # surface_point 单位为 mm，转换为 m
        result['surface_points'].append([v / 1000.0 for v in _floats(m.group(2))])
        # work_point 日志标注 mm 但实际是 m（C++ 代码 pose.x = target_base/1000），保持原值
        result['work_points'].append(_floats(m.group(3)))
        result['normals'].append(pc[:3])
        result['plane_coeffs'].append(pc if len(pc) == 4 else pc + [None])
        result['main_axes'].append(_floats(m.group(5)))
        result['slice_widths'].append(float(m.group(6)))
        result['slice_heights'].append(float(m.group(7)))
        result['wing_plate_widths'].append(float(m.group(8)))

    # ── 新格式(m): plane_coeffs=(a,b,c,d)，单位 m（兼容旧日志）────────
    if not result['surface_points']:
        new_pat_m = re.compile(
            r'Published target info for slice_id (\d+):\s*'
            r'surface_point=\(([^)]+)\)\s*m,'
            r'work_point=\(([^)]+)\)\s*m,'
            r'plane_coeffs=\(([^)]+)\),\s*'
            r'main_axis=\(([^)]+)\),'
            r'slice_width=([\d.]+)\s*mm,\s*slice_height=([\d.]+)\s*mm,\s*wing_plate_width=([\d.]+)'
        )
        for m in new_pat_m.finditer(log_content):
            pc = _floats(m.group(4))
            result['slice_ids'].append(int(m.group(1)))
            result['surface_points'].append(_floats(m.group(2)))
            result['work_points'].append(_floats(m.group(3)))
            result['normals'].append(pc[:3])
            result['plane_coeffs'].append(pc if len(pc) == 4 else pc + [None])
            result['main_axes'].append(_floats(m.group(5)))
            result['slice_widths'].append(float(m.group(6)))
            result['slice_heights'].append(float(m.group(7)))
            result['wing_plate_widths'].append(float(m.group(8)))

    # ── 旧格式: normal=(x,y,z) ──────────────────────────────────────────
    if not result['surface_points']:
        old_pat = re.compile(
            r'Published target info for slice[_\s]+(\d+):\s*'
            r'surface_point=\(([^)]+)\)\s*m,'
            r'work_point=\(([^)]+)\)\s*m,'
            r'normal=\(([^)]+)\).*?'
            r'slice_width=([\d.]+)\s*mm,\s*slice_height=([\d.]+)\s*mm',
            re.DOTALL
        )
        for m in old_pat.finditer(log_content):
            n = _floats(m.group(4))
            result['slice_ids'].append(int(m.group(1)))
            result['surface_points'].append(_floats(m.group(2)))
            result['work_points'].append(_floats(m.group(3)))
            result['normals'].append(n)
            result['plane_coeffs'].append(n + [None])
            result['main_axes'].append([1, 0, 0])
            result['slice_widths'].append(float(m.group(5)))
            result['slice_heights'].append(float(m.group(6)))
            result['wing_plate_widths'].append(0.0)

    # ── 工字钢本体尺寸 ────────────────────────────────────────────────
    dm = re.search(r'Dimensions \(LxWxH\): ([\d.]+) x ([\d.]+) x ([\d.]+) m', log_content)
    if dm:
        result['beam_dimensions'] = [float(dm.group(i)) for i in (1, 2, 3)]

    # ── 雷达坐标系位置 ────────────────────────────────────────────────
    pm = re.search(r'I-beam #\d+: Position=\(([^)]+)\)', log_content)
    if pm:
        result['beam_center_lidar'] = _floats(pm.group(1))

    # ── 基座坐标系中心 ────────────────────────────────────────────────
    # 新格式: mm 单位
    bm = re.search(r'IBeam center: \(([^)]+)\) mm in base frame', log_content)
    if bm:
        result['beam_center_base'] = [v / 1000.0 for v in _floats(bm.group(1))]
    else:
        # 旧格式: m 单位
        bm = re.search(r'IBeam center: \(([^)]+)\) m in base frame', log_content)
        if bm:
            result['beam_center_base'] = _floats(bm.group(1))

    # ── updateOutputs plane_rx（雷达坐标系主轴）──────────────────────
    rxm = re.search(r'plane_rx: \(([^)]+)\)', log_content)
    if rxm:
        result['plane_rx_lidar'] = _floats(rxm.group(1))

    # ── updateOutputs plane_coeffs（雷达坐标系，4分量）───────────────
    pcm = re.search(r'plane_coeffs: \(([^)]+)\)', log_content)
    if pcm:
        result['plane_coeffs_lidar'] = _floats(pcm.group(1))

    # ── beam_height / working_length / wing_plate_width ───────────────
    for key, pat in [
        ('beam_height_mm',    r'beam_height\s*:\s*([\d.]+)\s*mm'),
        ('working_length_mm', r'beam_length_of_working\s*:\s*([\d.]+)\s*mm'),
        ('wing_plate_width_mm', r'wing_plate_width\s*:\s*([\d.]+)\s*mm'),
    ]:
        m_ = re.search(pat, log_content)
        if m_:
            result[key] = float(m_.group(1))

    # ── SprayInfo ────────────────────────────────────────────────────
    sm = re.search(r'valid_slice_count=(\d+)\s*,\s*valid_total_area=([\d.]+)\s*m\^2', log_content)
    if sm:
        result['spray_num']  = int(sm.group(1))
        result['spray_area'] = float(sm.group(2))

    return result


# ─────────────────────────────────────────────
# 数值验证报告
# ─────────────────────────────────────────────

def print_verification_report(data):
    sep = "=" * 66
    print(sep)
    print("  工字钢发布数据 — 数值验证报告")
    print(sep)

    sps  = data['surface_points']
    wps  = data['work_points']
    nrms = data['normals']
    pcs  = data['plane_coeffs']
    ids  = data['slice_ids']
    sw   = data['slice_widths']
    sh   = data['slice_heights']
    n    = len(sps)

    print(f"\n  共解析到 {n} 个切片\n")

    # 推断 d_base
    if n > 0:
        diff     = np.array(wps[0]) - np.array(sps[0])
        d_base_m = np.linalg.norm(diff)
        print(f"  [推断] d_base ≈ {d_base_m*1000:.2f} mm  (切片#1 surface→work 距离)\n")
    else:
        d_base_m = 1.1

    ok_plane = ok_work = ok_norm = 0

    for i, sid in enumerate(ids):
        sp = np.array(sps[i])
        wp = np.array(wps[i])
        n_ = normalize(nrms[i])
        pc = pcs[i]

        errs = []

        # a) 法向量归一化
        nn = np.linalg.norm(nrms[i])
        if abs(nn - 1.0) < 0.015:
            ok_norm += 1
        else:
            errs.append(f"|n|={nn:.4f}≠1")

        # b) 平面方程残差
        if pc[3] is not None:
            a, b, c, d = pc
            val = a*sp[0] + b*sp[1] + c*sp[2] + d
            if abs(val) < 0.01:
                ok_plane += 1
            else:
                errs.append(f"plane_residual={val:.4f}")

        # c) 工作点对应
        _, wp_err = verify_work_point(sp, wp, n_, d_base_m)
        if wp_err < 5.0:
            ok_work += 1
        else:
            errs.append(f"WP_err={wp_err:.2f}mm")

        d_str = f"{pc[3]:.3f}" if pc[3] is not None else "N/A"
        mark  = "✓" if not errs else "✗"
        print(f"  Slice#{sid:2d} {mark}  SP=({sp[0]:.2f},{sp[1]:.2f},{sp[2]:.2f})"
              f"  WP=({wp[0]:.2f},{wp[1]:.2f},{wp[2]:.2f})"
              f"  d={d_str}"
              + (f"  [{'; '.join(errs)}]" if errs else ""))

    print(f"\n  法向量归一化  通过 {ok_norm}/{n}")
    print(f"  平面方程验证  通过 {ok_plane}/{n}")
    print(f"  工作点对应    通过 {ok_work}/{n}")

    # 切片间距一致性
    if n >= 2:
        diffs  = [np.linalg.norm(np.array(sps[i+1]) - np.array(sps[i])) for i in range(n-1)]
        mean_d = np.mean(diffs) * 1000
        std_d  = np.std(diffs) * 1000
        exp_d  = sw[0] if sw else 0
        ok_sp  = "✓" if std_d < 2.0 else "✗"
        print(f"\n  切片间距 {ok_sp}  均值={mean_d:.2f}mm  σ={std_d:.2f}mm  期望≈{exp_d:.2f}mm")

    # 面积
    if n > 0 and sw and sh:
        calc  = n * sw[0] * sh[0] / 1e6
        larea = data.get('spray_area')
        print(f"\n  总面积  {n}×{sw[0]:.2f}×{sh[0]:.2f}/1e6 = {calc:.4f} m²"
              + (f"  (日志={larea:.4f} m²  Δ={abs(calc-larea)*1e4:.1f} cm²)" if larea else ""))

    # beam 参数摘要
    print(f"\n  beam_height         = {data.get('beam_height_mm', '?')} mm")
    print(f"  beam_length_working = {data.get('working_length_mm', '?')} mm")
    print(f"  wing_plate_width    = {data.get('wing_plate_width_mm', '?')} mm  (翼缘半宽)")
    print(sep)


# ─────────────────────────────────────────────
# 可视化
# ─────────────────────────────────────────────

class IBeamVisualizer:
    """工字钢切片全要素 3D 可视化器

    坐标约定（局部梁体坐标系，以腹板面中心为原点）：
      u : 沿梁轴方向（≈ X）
      n : 腹板法向，指向机器人（≈ -Y）；翼缘在此方向展开 ±B/2
      w : 截面高度方向（= cross(u,n)，≈ Z）

    截面示意（n-w 平面，n 向右＝朝机器人，w 向上）：

        ←──── B ────→
        _______________        ← w = +H/2  上翼缘外缘
       |   top flange  |  tf
       |_______________|       ← w = +H/2-tf
              |   | tw
              | W |            ← 腹板 (tw × hw)
              |___|
        _______________        ← w = -(H/2-tf)
       |  bottom flange|  tf
       |_______________|       ← w = -H/2  下翼缘外缘
       |               |
      -B/2            +B/2 → n

    B/2 = wing_plate_width (翼缘半宽, mm)  ← 在 n 方向延伸
    H   = beam_height      (截面总高, mm)  ← 在 w 方向
    L   = beam length      (梁全长, m)     ← 在 u 方向
    """

    def __init__(self):
        self.fig = None
        self.ax  = None

    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    # 内部基础绘图工具
    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    def _box(self, center, u, n, w, hl, hn, hh,
             facecolor='white', edgecolor='#888888', alpha=0.25, lw=0.4):
        """在 (u, n, w) 坐标系中绘制长方体。hl/hn/hh 为沿 u/n/w 的半长。"""
        c = np.array(center)
        u, n, w = np.array(u), np.array(n), np.array(w)
        # 8 个角点：(s1,s2,s3) ∈ {-1,+1}³，顺序固定以保证面索引正确
        corners = np.array([
            c + s1*hl*u + s2*hn*n + s3*hh*w
            for s1 in (-1, 1) for s2 in (-1, 1) for s3 in (-1, 1)
        ])
        # 索引：idx=(s1+1)//2*4 + (s2+1)//2*2 + (s3+1)//2
        # 0:---  1:--+  2:-+-  3:-++  4:+--  5:+-+  6:++-  7:+++
        face_idx = [
            [0, 1, 3, 2], [4, 5, 7, 6],   # ±u 面
            [0, 1, 5, 4], [2, 3, 7, 6],   # ±n 面
            [0, 2, 6, 4], [1, 3, 7, 5],   # ±w 面
        ]
        poly = Poly3DCollection(
            [[corners[i] for i in fi] for fi in face_idx],
            alpha=alpha, facecolor=facecolor, edgecolor=edgecolor, linewidth=lw
        )
        self.ax.add_collection3d(poly)

    def _h_profile_pts(self, pos, H, B_half, tf, tw, n, w):
        """
        计算工字形截面的 12 个顶点（在 n-w 平面内，闭合）。
        顶点顺序（顺时针，从左上角开始）：
          0(-B/2,+H/2) → 1(-B/2,+H/2-tf) → 2(-tw/2,+H/2-tf) →
          3(-tw/2,-(H/2-tf)) → 4(-B/2,-(H/2-tf)) → 5(-B/2,-H/2) →
          6(+B/2,-H/2) → 7(+B/2,-(H/2-tf)) → 8(+tw/2,-(H/2-tf)) →
          9(+tw/2,+H/2-tf) → 10(+B/2,+H/2-tf) → 11(+B/2,+H/2)
        """
        p = np.array(pos)
        n, w = np.array(n), np.array(w)
        verts_nw = [
            (-B_half,   H/2      ),   # 0
            (-B_half,   H/2 - tf ),   # 1
            (-tw/2,     H/2 - tf ),   # 2
            (-tw/2,    -(H/2-tf) ),   # 3
            (-B_half,  -(H/2-tf) ),   # 4
            (-B_half,  -H/2      ),   # 5
            ( B_half,  -H/2      ),   # 6
            ( B_half,  -(H/2-tf) ),   # 7
            ( tw/2,    -(H/2-tf) ),   # 8
            ( tw/2,     H/2 - tf ),   # 9
            ( B_half,   H/2 - tf ),   # 10
            ( B_half,   H/2      ),   # 11
        ]
        return [p + pn*n + pw*w for (pn, pw) in verts_nw]

    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    # 工字钢几何体绘制
    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    def _draw_ibeam_solid(self, center, L, H, B_half, u, n, w, alpha=0.20):
        """
        绘制工字钢三维实体（腹板 + 上/下翼缘）。
        比例估算：tf ≈ 5%H（翼缘厚），tw ≈ 2.8%H（腹板厚）。
        颜色：腹板=钢蓝，翼缘=金黄。
        """
        center = np.array(center)
        tf = H * 0.050   # 翼缘厚
        tw = H * 0.028   # 腹板厚
        hw = H - 2 * tf  # 腹板净高

        # 腹板（钢蓝）
        self._box(center, u, n, w, L/2, tw/2, hw/2,
                  facecolor='#3d78b5', edgecolor='#1a4a8f',
                  alpha=alpha * 1.4, lw=0.5)
        # 上翼缘（金黄）
        self._box(center + (H/2 - tf/2) * w, u, n, w, L/2, B_half, tf/2,
                  facecolor='#c89030', edgecolor='#906000',
                  alpha=alpha * 1.9, lw=0.5)
        # 下翼缘（金黄）
        self._box(center - (H/2 - tf/2) * w, u, n, w, L/2, B_half, tf/2,
                  facecolor='#c89030', edgecolor='#906000',
                  alpha=alpha * 1.9, lw=0.5)

        # 两端截面轮廓线
        for end in [center - (L/2)*u, center + (L/2)*u]:
            pts = self._h_profile_pts(end, H, B_half, tf, tw, n, w)
            pts_cl = pts + [pts[0]]
            self.ax.plot([p[0] for p in pts_cl],
                         [p[1] for p in pts_cl],
                         [p[2] for p in pts_cl],
                         color='white', linewidth=1.1, alpha=0.85)

    def _draw_slice_cross_section(self, sp, H, B_half, tf, tw, n, w,
                                  line_color='#55bbee', lw=0.9,
                                  filled=True, fill_alpha=0.08):
        """在切片位置 sp 绘制工字形截面轮廓（可选填充）。"""
        pts = self._h_profile_pts(sp, H, B_half, tf, tw, n, w)
        pts_cl = pts + [pts[0]]
        self.ax.plot([p[0] for p in pts_cl],
                     [p[1] for p in pts_cl],
                     [p[2] for p in pts_cl],
                     color=line_color, linewidth=lw, alpha=0.90)
        if filled:
            poly = Poly3DCollection([pts], alpha=fill_alpha,
                                    facecolor='deepskyblue',
                                    edgecolor=line_color, linewidth=0.5)
            self.ax.add_collection3d(poly)

    def _draw_web_face_plane(self, center, L, H, u, w, alpha=0.07):
        """绘制腹板面（机器人喷涂的目标面），半透明蓝色矩形。"""
        c = np.array(center)
        u, w = np.array(u), np.array(w)
        corners = [c - (L/2)*u - (H/2)*w,
                   c + (L/2)*u - (H/2)*w,
                   c + (L/2)*u + (H/2)*w,
                   c - (L/2)*u + (H/2)*w]
        poly = Poly3DCollection([corners], alpha=alpha,
                                facecolor='deepskyblue',
                                edgecolor='deepskyblue', linewidth=1.5)
        self.ax.add_collection3d(poly)

    def _draw_dimension_annotations(self, center, L, H, B_half, u, n, w):
        """
        在梁侧绘制三条尺寸标注线：
          H    (截面高度，沿 w 方向，在梁右端外侧)   → 黄色
          B/2  (翼缘半宽，沿 n 方向，在翼缘顶端)    → 橙色
          L    (工作区长度，沿 u 方向，在腹板前方)  → 绿色
        """
        c  = np.array(center)
        u_ = np.array(u);  n_ = np.array(n);  w_ = np.array(w)
        tf = H * 0.05

        # ── H 标注 ──────────────────────────────────────────────
        ann_u = c + (L/2 + 0.07) * u_        # 在梁右端 70mm 外侧
        pt_top = ann_u + (H/2) * w_
        pt_bot = ann_u - (H/2) * w_
        self.ax.plot(*zip(pt_top, pt_bot),
                     color='yellow', lw=1.1, linestyle='--', alpha=0.85)
        # 端部短横线
        for pt, sign in [(pt_top, -1), (pt_bot, +1)]:
            tick = pt + sign * 0.03 * w_
            self.ax.plot(*zip(pt, tick), color='yellow', lw=1.1, alpha=0.85)
        mid_H = (pt_top + pt_bot) / 2
        self.ax.text(mid_H[0] + 0.03, mid_H[1] + 0.03, mid_H[2],
                     f'H = {H*1000:.0f} mm',
                     fontsize=8, color='yellow', fontweight='bold',
                     bbox=dict(fc='#10102a', alpha=0.88, pad=2, boxstyle='round'))

        # ── B/2 标注 ─────────────────────────────────────────────
        ann_w = ann_u + (H/2 + 0.04) * w_    # 翼缘顶部再上方
        pt_n  = ann_w + B_half * n_
        pt_p  = ann_w - B_half * n_
        self.ax.plot(*zip(pt_n, pt_p),
                     color='orange', lw=1.1, linestyle='--', alpha=0.85)
        for pt, sign in [(pt_n, -1), (pt_p, +1)]:
            tick = pt + sign * 0.025 * n_
            self.ax.plot(*zip(pt, tick), color='orange', lw=1.1, alpha=0.85)
        mid_B = (pt_n + pt_p) / 2
        self.ax.text(mid_B[0], mid_B[1], mid_B[2] + 0.04,
                     f'B/2 = {B_half*1000:.0f} mm',
                     fontsize=8, color='orange', fontweight='bold',
                     bbox=dict(fc='#10102a', alpha=0.88, pad=2, boxstyle='round'))

        # ── L 标注 ───────────────────────────────────────────────
        off_n = (B_half + 0.06) * n_          # 在翼缘前方偏移
        pt_l  = c - (L/2) * u_ + off_n
        pt_r  = c + (L/2) * u_ + off_n
        self.ax.plot(*zip(pt_l, pt_r),
                     color='lime', lw=1.1, linestyle='--', alpha=0.80)
        for pt, sign in [(pt_l, +1), (pt_r, -1)]:
            tick = pt + sign * 0.03 * u_
            self.ax.plot(*zip(pt, tick), color='lime', lw=1.1, alpha=0.80)
        mid_L = (pt_l + pt_r) / 2
        self.ax.text(mid_L[0], mid_L[1], mid_L[2] + 0.05,
                     f'L_work = {L*1000:.0f} mm',
                     fontsize=8, color='lime', fontweight='bold',
                     bbox=dict(fc='#10102a', alpha=0.88, pad=2, boxstyle='round'))

    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    # 主可视化入口
    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    def visualize(self, data):
        sps  = np.array(data['surface_points'])   # (N,3) m
        wps  = np.array(data['work_points'])       # (N,3) m
        nrms = data['normals']
        axes = data['main_axes']
        ids  = data['slice_ids']
        pcs  = data['plane_coeffs']
        sw   = data['slice_widths'][0]    if data['slice_widths']    else 370.0   # mm
        sh   = data['slice_heights'][0]   if data['slice_heights']   else 360.0   # mm
        wpw  = data['wing_plate_widths'][0] if data['wing_plate_widths'] else 0.0

        # ── 局部坐标系（从第 1 个切片的平面方程提取） ─────────────
        n_hat = normalize(nrms[0])   # 腹板法向（base frame，用于数值验证、法向箭头）
        u_hat = normalize(axes[0])   # 梁轴方向

        # ── 可视化用坐标系（世界对齐，消除 base frame 倾斜带来的视觉畸变）──
        #
        # 问题根因：LIDAR 系法向 n_z=0（工字钢真正垂直），但变换到 base 系后
        # n_z=-0.351（20.6° 倾斜），这是机械臂安装坐标系与世界垂直方向的偏差。
        # 若直接用 base 系法向构建截面，工字钢会看起来倾斜 20°。
        #
        # 修复：以世界 Z(0,0,1) 做 Gram-Schmidt 正交化得到高度方向 w_vis，
        # 再推导可视化法向 n_vis，使截面始终"竖直"渲染。
        z_world  = np.array([0.0, 0.0, 1.0])
        w_vis    = normalize(z_world - u_hat * u_hat.dot(z_world))   # ≈ (0,0,1) 竖直
        n_vis    = normalize(np.cross(u_hat, w_vis))                 # ≈ (0,-1,0) 朝机器人
        # 若 n_vis 与真实法向 n_hat 同侧则保留，否则翻转
        if n_vis.dot(n_hat) < 0:
            n_vis = -n_vis

        # 注：w_hat / n_hat 保留用于数值验证（平面方程、法向箭头）
        w_hat = w_vis   # 统一命名，以下绘图都用 w_vis/n_vis

        # ── 几何参数（m） ─────────────────────────────────────────
        H      = (data.get('beam_height_mm') or sh) / 1000.0   # 截面总高
        B_half = (data.get('wing_plate_width_mm') or sw * 0.15) / 1000.0  # 翼缘半宽
        tf     = H * 0.050   # 翼缘厚（估算）
        tw     = H * 0.028   # 腹板厚（估算）

        # ── 全梁与工作区 ──────────────────────────────────────────
        if data.get('beam_dimensions'):
            L_full   = data['beam_dimensions'][0]
        else:
            L_full   = len(sps) * sw / 1000.0

        if data.get('beam_center_base'):
            beam_ctr = np.array(data['beam_center_base'])   # 全梁 OBB 中心
        else:
            beam_ctr = sps.mean(axis=0)

        work_ctr = sps.mean(axis=0)            # 工作区中心（切片序列中点，在腹板面）
        L_work   = len(sps) * sw / 1000.0      # 工作区长度

        # ── 图形初始化 ─────────────────────────────────────────────
        self.fig = plt.figure(figsize=(18, 12))
        self.ax  = self.fig.add_subplot(111, projection='3d')
        self.ax.set_facecolor('#141424')
        self.fig.patch.set_facecolor('#141424')

        # ══════════════════════════════════════════════════════════
        # 层 1：工字钢三维实体（全梁，低透明度）
        # ══════════════════════════════════════════════════════════
        self._draw_ibeam_solid(beam_ctr, L_full, H, B_half,
                               u_hat, n_vis, w_hat, alpha=0.38)

        # ══════════════════════════════════════════════════════════
        # 层 2：腹板面（工作区范围，喷涂目标平面）
        # ══════════════════════════════════════════════════════════
        self._draw_web_face_plane(work_ctr, L_work, H, u_hat, w_hat, alpha=0.10)

        # ══════════════════════════════════════════════════════════
        # 层 3：各切片的工字形截面轮廓
        # ══════════════════════════════════════════════════════════
        for sp in sps:
            self._draw_slice_cross_section(sp, H, B_half, tf, tw,
                                           n_vis, w_hat,
                                           line_color='#44bbee', lw=0.85,
                                           filled=True, fill_alpha=0.07)

        # ══════════════════════════════════════════════════════════
        # 层 4：切片序号标签（在截面顶部）
        # ══════════════════════════════════════════════════════════
        for sid, sp in zip(ids, sps):
            label_pos = np.array(sp) + (H/2 + 0.07) * w_hat
            self.ax.text(label_pos[0], label_pos[1], label_pos[2],
                         f'#{sid}', fontsize=7, color='white',
                         bbox=dict(fc='steelblue', alpha=0.75, pad=1.5,
                                   boxstyle='round'))

        # ══════════════════════════════════════════════════════════
        # 层 5：表面点（蓝·在腹板面上）
        # ══════════════════════════════════════════════════════════
        self.ax.scatter(sps[:, 0], sps[:, 1], sps[:, 2],
                        c='deepskyblue', s=28, marker='o', depthshade=True,
                        label=f'Surface Points (n={len(sps)})', zorder=8)

        # ══════════════════════════════════════════════════════════
        # 层 6：工作点（品红·TCP 目标位置）及连线
        # ══════════════════════════════════════════════════════════
        self.ax.scatter(wps[:, 0], wps[:, 1], wps[:, 2],
                        c='magenta', s=45, marker='X', depthshade=True,
                        label='Work Points (TCP)', zorder=8)
        for sp, wp in zip(sps, wps):
            self.ax.plot([sp[0], wp[0]], [sp[1], wp[1]], [sp[2], wp[2]],
                         color='cyan', linewidth=1.1, alpha=0.70)

        # ══════════════════════════════════════════════════════════
        # 层 7：法向量箭头（绿·腹板法向，指向机器人）
        # ══════════════════════════════════════════════════════════
        for i, (sp, nr) in enumerate(zip(sps, nrms)):
            self.ax.quiver(*sp, *(normalize(nr) * 0.28),
                           color='lime', linewidth=1.4, arrow_length_ratio=0.25,
                           label='Normal (→robot)' if i == 0 else '')

        # ══════════════════════════════════════════════════════════
        # 层 8：切片序列连线（梁轴方向）
        # ══════════════════════════════════════════════════════════
        if len(sps) > 1:
            self.ax.plot(sps[:, 0], sps[:, 1], sps[:, 2],
                         color='orange', linewidth=2.2, linestyle='--',
                         alpha=0.85, label='Beam Axis (working)')

        # ══════════════════════════════════════════════════════════
        # 层 9：梁中心（全梁 OBB 中心）
        # ══════════════════════════════════════════════════════════
        self.ax.scatter(*beam_ctr, c='red', s=85, marker='D',
                        zorder=9, label='Beam Center (OBB)')

        # ══════════════════════════════════════════════════════════
        # 层 10：原点
        # ══════════════════════════════════════════════════════════
        self.ax.scatter([0], [0], [0], c='yellow', s=80, marker='*',
                        zorder=10, label='Origin (Base Frame)')

        # ══════════════════════════════════════════════════════════
        # 层 11：尺寸标注（H / B/2 / L_work）
        # ══════════════════════════════════════════════════════════
        self._draw_dimension_annotations(work_ctr, L_work, H, B_half,
                                         u_hat, n_vis, w_hat)

        # ══════════════════════════════════════════════════════════
        # 注释：左下角平面方程验证
        # ══════════════════════════════════════════════════════════
        if pcs and pcs[0][3] is not None:
            a, b, c, d = pcs[0]
            sp0 = sps[0]
            res = a*sp0[0] + b*sp0[1] + c*sp0[2] + d
            ok_s = '✓' if abs(res) < 0.01 else '✗'
            txt = (f'Plane eq (slice#{ids[0]})  [Base Frame]\n'
                   f'  {a:+.3f}·x  {b:+.3f}·y  {c:+.3f}·z  {d:+.3f} = 0\n'
                   f'  n̂ = ({a:.3f}, {b:.3f}, {c:.3f})\n'
                   f'  residual @ SP₁ = {res:.5f}  {ok_s}')
            self.ax.text2D(0.01, 0.02, txt,
                           transform=self.ax.transAxes,
                           fontsize=8, color='lightgreen',
                           bbox=dict(fc='#141432', alpha=0.92, boxstyle='round'))

        # ══════════════════════════════════════════════════════════
        # 注释：右上角参数摘要
        # ══════════════════════════════════════════════════════════
        bh   = data.get('beam_height_mm',    '?')
        wl   = data.get('working_length_mm', '?')
        area = data.get('spray_area',        '?')
        nsl  = data.get('spray_num', len(sps))
        info = (
            f'─── 工字钢参数 ─────────────────\n'
            f'beam_height  H  : {bh} mm\n'
            f'flange half B/2 : {wpw:.2f} mm  (n 方向)\n'
            f'beam_L (full)   : {L_full*1000:.0f} mm\n'
            f'─── 工作区 ──────────────────────\n'
            f'slice_width     : {sw:.2f} mm\n'
            f'slice_height    : {sh:.2f} mm\n'
            f'slices          : {nsl}\n'
            f'work_L          : {L_work*1000:.0f} mm\n'
            f'total_area      : {area} m²'
        )
        self.ax.text2D(0.995, 0.995, info,
                       transform=self.ax.transAxes,
                       fontsize=7.5, color='lightyellow',
                       va='top', ha='right', family='monospace',
                       bbox=dict(fc='#141432', alpha=0.92, boxstyle='round'))

        # ── 坐标轴：以梁体为中心，不把原点算入 range ────────────────
        # range 基于全梁长度与截面高度，保证梁体充满画面
        ctr = np.array(beam_ctr)
        rng = max(L_full * 0.65,    # 至少显示全梁长度
                  H     * 3.50,    # 截面高度 × 3.5 以便看清 H 形
                  B_half* 12.0,    # 翼缘宽度留足裕量
                  ) + 0.20         # 固定边距

        self.ax.set_xlim(ctr[0]-rng, ctr[0]+rng)
        self.ax.set_ylim(ctr[1]-rng, ctr[1]+rng)
        self.ax.set_zlim(ctr[2]-rng, ctr[2]+rng)
        self.ax.set_xlabel('X (m)', fontsize=10, color='white')
        self.ax.set_ylabel('Y (m)', fontsize=10, color='white')
        self.ax.set_zlabel('Z (m)', fontsize=10, color='white')
        self.ax.tick_params(colors='white', labelsize=8)
        for pane in [self.ax.xaxis.pane, self.ax.yaxis.pane, self.ax.zaxis.pane]:
            pane.fill = False
            pane.set_edgecolor('#333355')

        self.ax.set_title('I-Beam (工字钢) Geometry & Slice Verification — Base Frame',
                          fontsize=13, fontweight='bold', color='white', pad=12)
        self.ax.legend(loc='upper left', fontsize=7.5,
                       facecolor='#141432', edgecolor='#5555aa',
                       labelcolor='white')
        self.ax.view_init(elev=28, azim=38)   # 稍俯视以看清截面
        plt.tight_layout()
        return self.fig, self.ax

    def show(self):
        plt.show()

    def save(self, filename):
        if self.fig:
            self.fig.savefig(filename, dpi=150, bbox_inches='tight',
                             facecolor=self.fig.get_facecolor())
            print(f"[INFO] Figure saved to {filename}")


# ─────────────────────────────────────────────
# 主程序
# ─────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='I-Beam Output Verification & Visualization Tool',
        formatter_class=argparse.RawDescriptionHelpFormatter)

    ig = parser.add_mutually_exclusive_group(required=True)
    ig.add_argument('--file',  '-f', type=str, help='从日志文件解析')
    ig.add_argument('--stdin', '-i', action='store_true', help='从标准输入读取日志')

    parser.add_argument('--save',  type=str, default=None, help='保存图像到文件')
    parser.add_argument('--no-3d', action='store_true',    help='仅输出验证报告，不显示3D图')

    args = parser.parse_args()

    if args.file:
        if not os.path.exists(args.file):
            print(f"[ERROR] 文件不存在: {args.file}")
            return
        with open(args.file, 'r', encoding='utf-8') as f:
            log_content = f.read()
        print(f"[INFO] 读取日志文件: {args.file}")
    else:
        print("请粘贴日志内容，然后按 Ctrl+D (Linux) 结束输入:")
        log_content = sys.stdin.read()

    data = parse_log_content(log_content)

    if not data['surface_points']:
        print("[ERROR] 未能解析到任何切片数据，请检查日志格式。")
        sys.exit(1)

    # 数值验证报告
    print_verification_report(data)

    if args.no_3d:
        return

    # 3D 可视化
    viz = IBeamVisualizer()
    viz.visualize(data)
    if args.save:
        viz.save(args.save)
    viz.show()


if __name__ == '__main__':
    main()
