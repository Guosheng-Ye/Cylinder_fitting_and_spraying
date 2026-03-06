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

    # ── 新格式: plane_coeffs=(a,b,c,d) ─────────────────────────────────
    new_pat = re.compile(
        r'Published target info for slice_id (\d+):\s*'
        r'surface_point=\(([^)]+)\)\s*m,'
        r'work_point=\(([^)]+)\)\s*m,'
        r'plane_coeffs=\(([^)]+)\),\s*'
        r'main_axis=\(([^)]+)\),'
        r'slice_width=([\d.]+)\s*mm,\s*slice_height=([\d.]+)\s*mm,\s*wing_plate_width=([\d.]+)'
    )
    for m in new_pat.finditer(log_content):
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
    """工字钢切片全要素 3D 可视化器"""

    def __init__(self):
        self.fig = None
        self.ax  = None

    def _draw_obb(self, center, dimensions, main_axis, color='salmon', alpha=0.15):
        center     = np.array(center)
        dimensions = np.array(dimensions)
        u, v, w    = build_local_frame(main_axis)
        hl, hw, hh = dimensions / 2.0

        corners = []
        for sx in [-1, 1]:
            for sy in [-1, 1]:
                for sz in [-1, 1]:
                    corners.append(center + sx*hl*u + sy*hw*v + sz*hh*w)
        corners = np.array(corners)

        face_idx = [
            [0,1,3,2], [4,5,7,6],
            [0,1,5,4], [2,3,7,6],
            [0,2,6,4], [1,3,7,5],
        ]
        polys = [[corners[i] for i in fi] for fi in face_idx]
        pc = Poly3DCollection(polys, alpha=alpha, facecolor=color,
                              edgecolor=color, linewidth=0.6)
        self.ax.add_collection3d(pc)
        self.ax.quiver(*center, *(u * hl * 1.15),
                       color='red', linewidth=2.0, arrow_length_ratio=0.12)
        self.ax.scatter(*center, c='red', s=70, marker='D', zorder=6, label='Beam Center (base)')

    def _draw_slice_planes(self, surface_points, normals, main_axes, sw_mm, sh_mm):
        for sp, n, ax in zip(surface_points, normals, main_axes):
            sp  = np.array(sp)
            u_s = normalize(ax)
            n_s = normalize(n)
            # 切片高度方向 = 主轴 × 法向（近似竖直）
            w_s = normalize(np.cross(u_s, n_s))
            hw  = (sw_mm / 2.0) / 1000.0
            hh  = (sh_mm / 2.0) / 1000.0
            corners = [sp - hw*u_s - hh*w_s, sp + hw*u_s - hh*w_s,
                       sp + hw*u_s + hh*w_s, sp - hw*u_s + hh*w_s]
            poly = Poly3DCollection([corners], alpha=0.10,
                                    facecolor='deepskyblue', edgecolor='steelblue', linewidth=0.8)
            self.ax.add_collection3d(poly)

    def visualize(self, data):
        sps  = np.array(data['surface_points'])
        wps  = np.array(data['work_points'])
        nrms = data['normals']
        axes = data['main_axes']
        ids  = data['slice_ids']
        pcs  = data['plane_coeffs']
        sw   = data['slice_widths'][0]   if data['slice_widths']   else 370.0
        sh   = data['slice_heights'][0]  if data['slice_heights']  else 360.0
        wpw  = data['wing_plate_widths'][0] if data['wing_plate_widths'] else 0.0

        self.fig = plt.figure(figsize=(16, 11))
        self.ax  = self.fig.add_subplot(111, projection='3d')
        self.ax.set_facecolor('#1c1c2e')
        self.fig.patch.set_facecolor('#1c1c2e')

        # 0. 原点
        self.ax.scatter([0], [0], [0], c='yellow', s=260, marker='*',
                        zorder=10, label='Origin (Base Frame)')

        # 1. 切片面片
        self._draw_slice_planes(sps, nrms, axes, sw, sh)

        # 2. 表面点（蓝）
        self.ax.scatter(sps[:, 0], sps[:, 1], sps[:, 2],
                        c='deepskyblue', s=65, marker='o', depthshade=True,
                        label=f'Surface Points (n={len(sps)})', zorder=8)

        # 3. 工作点（品红）
        self.ax.scatter(wps[:, 0], wps[:, 1], wps[:, 2],
                        c='magenta', s=95, marker='X', depthshade=True,
                        label='Work Points', zorder=8)

        # 4. surface → work 连线
        for sp, wp in zip(sps, wps):
            self.ax.plot([sp[0], wp[0]], [sp[1], wp[1]], [sp[2], wp[2]],
                         color='cyan', linewidth=1.2, alpha=0.75)

        # 5. 法向量箭头（绿）
        for i, (sp, n) in enumerate(zip(sps, nrms)):
            self.ax.quiver(*sp, *(normalize(n) * 0.25),
                           color='lime', linewidth=1.5, arrow_length_ratio=0.28,
                           label='Normal (→work)' if i == 0 else "")

        # 6. 主轴串连线（橙色虚线）
        if len(sps) > 1:
            self.ax.plot(sps[:, 0], sps[:, 1], sps[:, 2],
                         color='orange', linewidth=2.0, linestyle='--',
                         alpha=0.85, label='Main Axis (slice sequence)')

        # 7. OBB（若有基座下中心和主轴）
        bc = data.get('beam_center_base')
        bd = data.get('beam_dimensions')
        ax_rx = data.get('plane_rx_lidar') or (axes[0] if axes else None)
        if bc and bd and ax_rx:
            self._draw_obb(bc, bd, ax_rx)

        # 8. 切片序号标签
        for sid, sp in zip(ids, sps):
            self.ax.text(sp[0], sp[1], sp[2] + 0.06, f"#{sid}",
                         fontsize=7, color='white',
                         bbox=dict(fc='steelblue', alpha=0.65, pad=1.5, boxstyle='round'))

        # 9. 左下角：平面方程验证（以第1切片）
        if pcs and pcs[0][3] is not None:
            a, b, c, d = pcs[0]
            sp0 = sps[0]
            res = a*sp0[0] + b*sp0[1] + c*sp0[2] + d
            ok_s = "✓" if abs(res) < 0.01 else "✗"
            self.ax.text2D(0.01, 0.02,
                           f"Plane eq (slice#{ids[0]}): "
                           f"{a:.3f}x + {b:.3f}y + {c:.3f}z + {d:.3f} = 0\n"
                           f"代入 surface point: residual = {res:.5f}  {ok_s}",
                           transform=self.ax.transAxes,
                           fontsize=8, color='lightgreen',
                           bbox=dict(fc='#2a2a40', alpha=0.88, boxstyle='round'))

        # 10. 右上角：参数摘要
        bh    = data.get('beam_height_mm',    '?')
        wl    = data.get('working_length_mm', '?')
        area  = data.get('spray_area',        '?')
        nslice = data.get('spray_num', len(sps))
        info  = (
            f"beam_height:    {bh} mm\n"
            f"slice_width:    {sw:.2f} mm\n"
            f"slice_height:   {sh:.2f} mm\n"
            f"wing_plate_w:   {wpw:.2f} mm (half)\n"
            f"slices:         {nslice}\n"
            f"total_area:     {area} m²"
        )
        self.ax.text2D(0.995, 0.995, info,
                       transform=self.ax.transAxes,
                       fontsize=8, color='lightyellow', va='top', ha='right',
                       family='monospace',
                       bbox=dict(fc='#2a2a40', alpha=0.88, boxstyle='round'))

        # ── 坐标轴设置 ────────────────────────────────────────────────
        all_pts = np.vstack([sps, wps, [[0, 0, 0]]])
        mn  = all_pts.min(axis=0)
        mx  = all_pts.max(axis=0)
        ctr = (mn + mx) / 2.0
        rng = max((mx - mn).max() * 0.65, 1.0)

        self.ax.set_xlim(ctr[0]-rng, ctr[0]+rng)
        self.ax.set_ylim(ctr[1]-rng, ctr[1]+rng)
        self.ax.set_zlim(ctr[2]-rng, ctr[2]+rng)
        self.ax.set_xlabel('X (m)', fontsize=10, color='white')
        self.ax.set_ylabel('Y (m)', fontsize=10, color='white')
        self.ax.set_zlabel('Z (m)', fontsize=10, color='white')
        self.ax.tick_params(colors='white', labelsize=8)
        for pane in [self.ax.xaxis.pane, self.ax.yaxis.pane, self.ax.zaxis.pane]:
            pane.fill = False
            pane.set_edgecolor('#444466')

        self.ax.set_title('I-Beam Slice Verification  (Base Frame)',
                          fontsize=14, fontweight='bold', color='white', pad=12)
        self.ax.legend(loc='upper left', fontsize=8,
                       facecolor='#2a2a40', edgecolor='#6666aa', labelcolor='white')
        self.ax.view_init(elev=22, azim=55)
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
