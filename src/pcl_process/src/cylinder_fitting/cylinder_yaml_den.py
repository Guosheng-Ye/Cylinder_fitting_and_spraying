#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
倾斜细长圆柱离线分析工具 (v3)
==============================
功能：
  1. 加载 PCD 点云文件
  2. 移除地面和墙面平面
  3. 使用 RANSAC + 最小二乘 拟合圆柱模型（支持任意倾斜角度）
  4. 支持 --tilt-angle 参数，按目标倾斜角过滤候选聚类
  5. Matplotlib 交互式 3D 可视化拟合结果
  6. 输出推荐的 YAML 配置参数

依赖安装：
  conda activate algorithm  # 或含有 open3d 的环境
  pip install open3d numpy scipy pyyaml matplotlib

用法：
  python cylinder_yaml_den.py -f 113.pcd -v              # 指定文件 + 可视化
  python cylinder_yaml_den.py -f 113.pcd -v --tilt-angle 45  # 检测 45° 倾斜圆柱
  python cylinder_yaml_den.py -f 113.pcd                  # 只分析不可视化
  python cylinder_yaml_den.py --dir pcd/ -v               # 扫描目录 + 可视化
  python cylinder_yaml_den.py -f 113.pcd -v --raw         # 先看原始点云再分析
"""

import argparse
import glob
import os
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

try:
    import open3d as o3d
except ImportError:
    print("错误: 请安装 open3d: pip install open3d")
    sys.exit(1)

try:
    import yaml
except ImportError:
    print("警告: pyyaml 未安装，将跳过 YAML 输出。pip install pyyaml")
    yaml = None

import matplotlib
# 尝试交互式后端
try:
    matplotlib.use('TkAgg')
except Exception:
    try:
        matplotlib.use('Qt5Agg')
    except Exception:
        pass  # 使用默认后端

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


# ============================================================================
# 数据结构
# ============================================================================

@dataclass
class CylinderResult:
    """圆柱拟合结果"""
    center: np.ndarray = field(default_factory=lambda: np.zeros(3))
    axis: np.ndarray = field(default_factory=lambda: np.array([0, 0, 1.0]))
    radius: float = 0.0
    height: float = 0.0
    inlier_count: int = 0
    inlier_ratio: float = 0.0
    tilt_angle_deg: float = 0.0
    radial_rel_std: float = 0.0
    radial_range_ratio: float = 0.0
    aspect_ratio: float = 0.0
    circularity: float = 0.0
    inlier_indices: np.ndarray = field(default_factory=lambda: np.array([]))
    valid: bool = False


# ============================================================================
# 点云预处理
# ============================================================================

def load_pcd(filepath: str) -> o3d.geometry.PointCloud:
    """加载 PCD 文件"""
    print(f"[加载] {filepath}")
    pcd = o3d.io.read_point_cloud(filepath)
    print(f"  点数: {len(pcd.points)}")
    if len(pcd.points) == 0:
        raise ValueError(f"点云为空: {filepath}")
    return pcd


def preprocess_cloud(pcd: o3d.geometry.PointCloud,
                     voxel_size: float = 0.005,
                     nb_neighbors: int = 30,
                     std_ratio: float = 2.0) -> o3d.geometry.PointCloud:
    """点云预处理：降采样 + 去噪"""
    print(f"[预处理] 原始点数: {len(pcd.points)}")
    if voxel_size > 0:
        pcd = pcd.voxel_down_sample(voxel_size)
        print(f"  降采样后: {len(pcd.points)} (voxel={voxel_size}m)")
    if len(pcd.points) > nb_neighbors:
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
        print(f"  去噪后: {len(pcd.points)}")
    return pcd


def remove_planes(pcd: o3d.geometry.PointCloud,
                  max_planes: int = 5,
                  distance_threshold: float = 0.02,
                  num_iterations: int = 5000,
                  min_plane_points: int = 200,
                  min_keep_ratio: float = 0.10) -> o3d.geometry.PointCloud:
    """移除地面和墙面等大平面"""
    print(f"[平面移除] 最多移除 {max_planes} 个平面")
    remaining = pcd
    original_count = len(pcd.points)

    for i in range(max_planes):
        if len(remaining.points) < min_plane_points:
            break
        if len(remaining.points) < original_count * min_keep_ratio:
            print(f"  已达到最小保留比例 ({min_keep_ratio:.0%})，停止")
            break

        plane_model, inliers = remaining.segment_plane(
            distance_threshold=distance_threshold,
            ransac_n=3,
            num_iterations=num_iterations)

        if len(inliers) < min_plane_points:
            break

        normal = np.array(plane_model[:3])
        normal = normal / np.linalg.norm(normal)
        angle_to_z = np.degrees(np.arccos(np.abs(np.dot(normal, [0, 0, 1]))))

        if angle_to_z < 20:
            ptype = "地面/天花板"
        elif angle_to_z > 70:
            ptype = "墙面"
        else:
            ptype = f"倾斜面({angle_to_z:.1f}°)"

        print(f"  平面 {i+1}: {ptype}, Z角={angle_to_z:.1f}°, "
              f"内点={len(inliers)}, 移除")
        remaining = remaining.select_by_index(inliers, invert=True)

    print(f"  平面移除后: {len(remaining.points)} 点 "
          f"({len(remaining.points)/original_count:.1%})")
    return remaining


# ============================================================================
# 圆柱拟合算法
# ============================================================================

def ransac_cylinder_fit(points: np.ndarray,
                        normals: np.ndarray,
                        distance_threshold: float = 0.015,
                        num_iterations: int = 80000,
                        radius_min: float = 0.005,
                        radius_max: float = 1.0) -> Optional[CylinderResult]:
    """
    RANSAC 圆柱拟合 — 直接在全局点云上工作
    使用法线交叉来估计轴向，鲁棒的中位数估计半径
    """
    print(f"[RANSAC圆柱拟合] 点数={len(points)}, 迭代={num_iterations}, "
          f"R范围=[{radius_min:.3f}, {radius_max:.3f}]m")

    n_points = len(points)
    if n_points < 20:
        print("  点数不足")
        return None

    best_result = None
    best_inlier_count = 0
    rng = np.random.default_rng(42)
    t0 = time.time()

    early_stop_ratio = 0.4

    for it in range(num_iterations):
        idx = rng.choice(n_points, size=2, replace=False)
        n1, n2 = normals[idx[0]], normals[idx[1]]

        axis = np.cross(n1, n2)
        axis_norm = np.linalg.norm(axis)
        if axis_norm < 1e-6:
            continue
        axis = axis / axis_norm
        if axis[2] < 0:
            axis = -axis

        p1 = points[idx[0]]

        diff = points - p1
        proj = np.dot(diff, axis)
        perp = diff - proj[:, np.newaxis] * axis
        dists = np.linalg.norm(perp, axis=1)

        radius = np.median(dists)
        if radius < radius_min or radius > radius_max:
            continue

        residuals = np.abs(dists - radius)
        inlier_mask = residuals < distance_threshold
        inlier_count = np.sum(inlier_mask)

        if inlier_count > best_inlier_count:
            best_inlier_count = inlier_count
            result = CylinderResult()
            result.center = p1.copy()
            result.axis = axis.copy()
            result.radius = radius
            result.inlier_count = inlier_count
            result.inlier_ratio = inlier_count / n_points
            result.inlier_indices = np.where(inlier_mask)[0]
            best_result = result

            if it % 10000 == 0:
                print(f"  迭代 {it}: 内点={inlier_count} "
                      f"({inlier_count/n_points:.1%}), R={radius:.4f}m")

            if result.inlier_ratio > early_stop_ratio:
                print(f"  迭代 {it}: 提前收敛 内点率={result.inlier_ratio:.1%}")
                break

    elapsed = time.time() - t0
    print(f"  完成: 耗时 {elapsed:.1f}s, 最佳内点={best_inlier_count}")

    if best_result is not None:
        best_result = refine_cylinder(points, best_result)
    return best_result


def refine_cylinder_leastsq(points: np.ndarray,
                            initial: CylinderResult) -> Optional[CylinderResult]:
    """scipy 最小二乘精化"""
    from scipy.optimize import least_squares

    x0 = np.concatenate([initial.center, initial.axis, [initial.radius]])

    def residuals(params):
        cx, cy, cz, ax, ay, az, r = params
        center = np.array([cx, cy, cz])
        axis = np.array([ax, ay, az])
        al = np.linalg.norm(axis)
        if al < 1e-10:
            return np.ones(len(points)) * 1e6
        axis = axis / al
        diff = points - center
        proj = np.dot(diff, axis)
        perp = diff - proj[:, np.newaxis] * axis
        return np.linalg.norm(perp, axis=1) - r

    try:
        res = least_squares(residuals, x0, method='lm', max_nfev=5000)
        cx, cy, cz, ax, ay, az, r = res.x
        axis = np.array([ax, ay, az])
        axis = axis / np.linalg.norm(axis)
        if axis[2] < 0:
            axis = -axis
        cyl = CylinderResult()
        cyl.center = np.array([cx, cy, cz])
        cyl.axis = axis
        cyl.radius = abs(r)
        return cyl
    except Exception:
        return None


def refine_cylinder(points: np.ndarray, result: CylinderResult) -> CylinderResult:
    """用内点精化圆柱参数并计算质量指标"""
    if result.inlier_count < 10:
        return result

    inliers = points[result.inlier_indices]
    refined = refine_cylinder_leastsq(inliers, result)
    if refined is not None and refined.radius > 0:
        result.center = refined.center
        result.axis = refined.axis
        result.radius = refined.radius

    diff = inliers - result.center
    proj = np.dot(diff, result.axis)
    perp = diff - proj[:, np.newaxis] * result.axis
    radii = np.linalg.norm(perp, axis=1)

    result.radius = np.mean(radii)
    result.height = np.max(proj) - np.min(proj)
    result.center = result.center + result.axis * (np.min(proj) + np.max(proj)) / 2

    cos_angle = np.abs(np.dot(result.axis, [0, 0, 1]))
    result.tilt_angle_deg = np.degrees(np.arccos(np.clip(cos_angle, -1, 1)))

    mean_r = np.mean(radii)
    std_r = np.std(radii)
    result.radial_rel_std = std_r / mean_r if mean_r > 0 else 0
    result.radial_range_ratio = (np.max(radii) - np.min(radii)) / mean_r if mean_r > 0 else 0
    result.aspect_ratio = result.height / (2 * result.radius) if result.radius > 0 else 0
    result.circularity = compute_circularity(inliers, result.center, result.axis)
    result.valid = True
    return result


def compute_circularity(points: np.ndarray, center: np.ndarray,
                        axis: np.ndarray, num_sectors: int = 24) -> float:
    """圆度评分：按角度分扇区统计覆盖率"""
    diff = points - center
    proj = np.dot(diff, axis)[:, np.newaxis] * axis
    perp = diff - proj

    ref = np.array([1, 0, 0]) if abs(axis[0]) < 0.9 else np.array([0, 1, 0])
    u = np.cross(axis, ref)
    u = u / np.linalg.norm(u)
    v = np.cross(axis, u)

    angles = np.arctan2(np.dot(perp, v), np.dot(perp, u))
    edges = np.linspace(-np.pi, np.pi, num_sectors + 1)
    occupied = sum(1 for i in range(num_sectors)
                   if np.any((angles >= edges[i]) & (angles < edges[i + 1])))
    return occupied / num_sectors


# ============================================================================
# 多策略检测管线
# ============================================================================

def detect_cylinder(pcd: o3d.geometry.PointCloud,
                    voxel_size: float = 0.005,
                    visualize: bool = False,
                    show_raw: bool = False,
                    radius_min: float = 0.005,
                    radius_max: float = 1.0,
                    tilt_angle: Optional[float] = None,
                    tilt_tolerance: float = 20.0) -> Optional[CylinderResult]:
    """
    检测管线：
      当指定 tilt_angle 时: 先用倾斜角过滤聚类，再做定向 RANSAC
      否则: 全局 RANSAC → 聚类拟合 → 放宽重试
    """
    pcd_clean = preprocess_cloud(pcd, voxel_size=voxel_size)

    if show_raw:
        pts = np.asarray(pcd_clean.points)
        print(f"\n[原始点云预览] {len(pts)} 点")
        _show_raw_cloud(pts, title="原始点云（关闭窗口继续分析）")

    pcd_no_planes = remove_planes(pcd_clean)
    n_remaining = len(pcd_no_planes.points)
    if n_remaining < 50:
        print("[错误] 移除平面后点数不足")
        return None

    print("[法线估计]")
    pcd_no_planes.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamKNN(knn=30))
    pcd_no_planes.orient_normals_consistent_tangent_plane(k=15)

    points = np.asarray(pcd_no_planes.points)
    normals = np.asarray(pcd_no_planes.normals)

    result = None

    if tilt_angle is not None:
        # === 倾斜角定向检测模式 ===
        print(f"\n[定向检测] 目标倾斜角={tilt_angle:.0f}° ± {tilt_tolerance:.0f}°")
        result = _strategy_tilt_targeted(
            points, normals, radius_min, radius_max,
            tilt_angle, tilt_tolerance)

        # === 种子扩展：沿轴方向搜索全残余点，扩展检测到的局部圆柱 ===
        if result and result.valid:
            result = _expand_cylinder_along_axis(
                points, normals, result,
                surface_tol_factor=1.5,
                max_iterations=5)
    else:
        # === 策略1：全局 RANSAC ===
        print(f"\n[策略1] 全局 RANSAC 圆柱拟合 ({len(points)} 点)")
        result = ransac_cylinder_fit(
            points, normals,
            distance_threshold=0.015,
            num_iterations=80000,
            radius_min=radius_min,
            radius_max=radius_max)

        if result and result.valid and result.inlier_ratio > 0.05 and result.aspect_ratio > 2:
            print(f"  [策略1成功] R={result.radius:.4f}m, H={result.height:.3f}m, "
                  f"倾斜={result.tilt_angle_deg:.1f}°, 长宽比={result.aspect_ratio:.1f}")
        else:
            # === 策略2：聚类后拟合 ===
            print(f"\n[策略2] DBSCAN 聚类后逐聚类拟合")
            result2 = _strategy_cluster_fit(points, normals, radius_min, radius_max)
            if result2 is not None:
                result = result2

    if result is None or not result.valid:
        # === 后备策略：放宽参数 ===
        print(f"\n[后备] 放宽参数重试")
        result = ransac_cylinder_fit(
            points, normals,
            distance_threshold=0.03,
            num_iterations=100000,
            radius_min=radius_min,
            radius_max=radius_max)

    # 可视化
    if visualize:
        if result is not None and result.valid:
            _visualize_interactive(points, result,
                                   title="圆柱拟合结果（可旋转/缩放，关闭窗口继续）")
        else:
            _show_raw_cloud(points, title="去平面后的点云（拟合失败，查看数据）")

    return result


def _pca_cluster_info(pts: np.ndarray):
    """计算聚类的 PCA 信息: 主方向、倾斜角、线性度"""
    centered = pts - pts.mean(0)
    cov = np.cov(centered.T)
    evals, evecs = np.linalg.eigh(cov)
    idx = np.argsort(evals)[::-1]
    evals = evals[idx]
    evecs = evecs[:, idx]
    main_dir = evecs[:, 0]
    if main_dir[2] < 0:
        main_dir = -main_dir
    tilt = np.degrees(np.arccos(np.clip(abs(main_dir[2]), -1, 1)))
    linearity = (evals[0] - evals[1]) / evals[0] if evals[0] > 0 else 0
    proj = centered @ main_dir
    length = proj.max() - proj.min()
    perp = centered - np.outer(proj, main_dir)
    lateral = np.median(np.linalg.norm(perp, axis=1))
    return main_dir, tilt, linearity, length, lateral


def _expand_cylinder_along_axis(all_points: np.ndarray,
                                all_normals: np.ndarray,
                                seed_result: CylinderResult,
                                surface_tol_factor: float = 1.5,
                                max_iterations: int = 5) -> CylinderResult:
    """
    种子扩展法: 以检测到的局部圆柱为种子，沿轴方向在全部残余点中
    搜索圆柱面附近的点，迭代扩展并重拟合，直到收敛。

    这解决了稀疏长圆柱因 DBSCAN 断裂只检测到局部片段的问题。

    参数:
        all_points:  去平面后的全部残余点
        all_normals: 对应法线
        seed_result: 初始种子圆柱拟合结果
        surface_tol_factor: 表面容差 = R * factor
        max_iterations: 最大迭代次数
    """
    print(f"\n[种子扩展] 初始种子: R={seed_result.radius*1000:.1f}mm, "
          f"H={seed_result.height:.3f}m, {seed_result.inlier_count}点")

    center = seed_result.center.copy()
    axis = seed_result.axis.copy()
    radius = seed_result.radius

    prev_count = 0

    for it in range(max_iterations):
        # 计算全部点到当前圆柱轴的距离
        diff = all_points - center
        proj = diff @ axis
        perp = diff - proj[:, np.newaxis] * axis
        dist_to_axis = np.linalg.norm(perp, axis=1)

        # 圆柱面附近的点: |dist - R| < surface_tol
        surface_tol = radius * surface_tol_factor
        surface_mask = np.abs(dist_to_axis - radius) < surface_tol

        # 也包括轴心附近的点（可能是内部散射点）
        inner_mask = dist_to_axis < radius * 0.8
        combined_mask = surface_mask | inner_mask

        inlier_pts = all_points[combined_mask]
        inlier_normals = all_normals[combined_mask]
        n_inliers = len(inlier_pts)

        if n_inliers < 30:
            print(f"  迭代 {it}: 内点不足 ({n_inliers}), 停止扩展")
            break

        print(f"  迭代 {it}: 找到 {n_inliers} 点在圆柱面附近 "
              f"(容差=R*{surface_tol_factor:.1f}={surface_tol*1000:.0f}mm)")

        # 收敛检查: 新增点数 < 5%
        if prev_count > 0 and abs(n_inliers - prev_count) < prev_count * 0.05:
            print(f"  迭代 {it}: 已收敛 (Δ<5%)")
            break
        prev_count = n_inliers

        # 用扩展后的点重拟合圆柱
        fit = _fit_cylinder_scipy(inlier_pts, axis)
        if fit is None:
            print(f"  迭代 {it}: scipy 拟合失败, 停止")
            break

        # 更新参数
        center = fit['center']
        axis = fit['axis']
        radius = fit['radius']

        proj_new = (inlier_pts - center) @ axis
        height = proj_new.max() - proj_new.min()

        print(f"    → R={radius*1000:.1f}mm, H={height:.3f}m, "
              f"倾斜={fit['tilt']:.1f}°, RMSE={fit['rmse']*1000:.1f}mm, "
              f"内点率={fit['inlier_ratio']:.1%}")

    # 最终精确筛选内点: |dist - R| < R * 0.5 (严格容差)
    diff = all_points - center
    proj = diff @ axis
    perp = diff - proj[:, np.newaxis] * axis
    dist_to_axis = np.linalg.norm(perp, axis=1)
    strict_tol = radius * 0.5
    final_mask = np.abs(dist_to_axis - radius) < strict_tol
    final_pts = all_points[final_mask]
    final_normals = all_normals[final_mask]
    final_indices = np.where(final_mask)[0]

    # 最终拟合
    if len(final_pts) >= 30:
        fit_final = _fit_cylinder_scipy(final_pts, axis)
        if fit_final is not None:
            center = fit_final['center']
            axis = fit_final['axis']
            radius = fit_final['radius']

    # 构建最终结果
    final_diff = final_pts - center
    final_proj = final_diff @ axis
    final_perp = final_diff - final_proj[:, np.newaxis] * axis
    final_radii = np.linalg.norm(final_perp, axis=1)

    result = CylinderResult()
    result.center = center + axis * (final_proj.min() + final_proj.max()) / 2
    result.axis = axis
    result.radius = radius
    result.height = final_proj.max() - final_proj.min()
    result.inlier_count = len(final_pts)
    result.inlier_ratio = len(final_pts) / len(all_points)
    result.tilt_angle_deg = np.degrees(np.arccos(np.clip(abs(axis[2]), -1, 1)))
    result.inlier_indices = final_indices

    mean_r = np.mean(final_radii)
    std_r = np.std(final_radii)
    result.radial_rel_std = std_r / mean_r if mean_r > 0 else 0
    result.radial_range_ratio = (np.max(final_radii) - np.min(final_radii)) / mean_r if mean_r > 0 else 0
    result.aspect_ratio = result.height / (2 * radius) if radius > 0 else 0
    result.circularity = compute_circularity(final_pts, result.center, axis)
    result.valid = True

    print(f"\n  [扩展完成] R={result.radius*1000:.1f}mm, H={result.height:.3f}m, "
          f"倾斜={result.tilt_angle_deg:.1f}°, "
          f"内点={result.inlier_count} ({result.inlier_ratio:.1%})")

    return result


def _strategy_tilt_targeted(points: np.ndarray, normals: np.ndarray,
                            radius_min: float, radius_max: float,
                            target_tilt: float, tilt_tolerance: float
                            ) -> Optional[CylinderResult]:
    """
    倾斜角定向检测：
    1. 多尺度 DBSCAN (eps=0.05~0.30) 聚类
    2. PCA 预筛: 倾斜角 ∈ [target ± tolerance] & 线性度 > 0.7
    3. 对符合条件的聚类尝试合并（如果方向一致且近邻）
    4. scipy 圆柱拟合 + 评分
    """
    pcd_tmp = o3d.geometry.PointCloud()
    pcd_tmp.points = o3d.utility.Vector3dVector(points)

    tilt_min = target_tilt - tilt_tolerance
    tilt_max = target_tilt + tilt_tolerance
    print(f"  倾斜角范围: [{tilt_min:.0f}°, {tilt_max:.0f}°]")

    all_candidates = []  # (points, normals, pca_tilt, linearity, length, eps_label)
    seen_center_hashes = set()

    for eps, min_pts in [(0.05, 25), (0.08, 20), (0.12, 15),
                         (0.15, 12), (0.20, 10), (0.25, 8)]:
        labels = np.array(pcd_tmp.cluster_dbscan(eps=eps, min_points=min_pts))
        if len(labels) == 0 or labels.max() < 0:
            continue

        n_clusters = labels.max() + 1
        matched = 0

        for c in range(n_clusters):
            mask = labels == c
            cp = points[mask]
            cn = normals[mask]
            if len(cp) < 20:
                continue

            main_dir, tilt, linearity, length, lateral = _pca_cluster_info(cp)

            if not (tilt_min <= tilt <= tilt_max):
                continue
            if linearity < 0.7:
                continue

            # 去重: 如果中心位置很接近已有候选则跳过
            center = cp.mean(0)
            ch = tuple((center * 5).astype(int))  # 量化到 0.2m 网格
            if ch in seen_center_hashes:
                continue
            seen_center_hashes.add(ch)

            matched += 1
            all_candidates.append({
                'points': cp, 'normals': cn,
                'pca_dir': main_dir, 'tilt': tilt,
                'linearity': linearity, 'length': length,
                'lateral': lateral, 'center': center,
                'eps': eps, 'n_pts': len(cp)
            })
            print(f"  eps={eps:.2f} 聚类{c}: {len(cp)}点, "
                  f"倾斜={tilt:.1f}°, 线性={linearity:.2f}, "
                  f"长={length:.3f}m, 宽={lateral:.4f}m")

        if matched == 0 and eps <= 0.12:
            continue

    if not all_candidates:
        print("  [定向检测] 未找到匹配倾斜角的聚类")
        return None

    print(f"\n  共找到 {len(all_candidates)} 个候选聚类")

    # 尝试合并方向一致且空间接近的候选
    merged_groups = _try_merge_candidates(all_candidates, target_tilt)

    # 对每个候选（或合并组）做圆柱拟合并评分
    best_result = None
    best_score = -1

    for group in merged_groups:
        gpts = group['points']
        gnormals = group['normals']
        n_pts = len(gpts)

        if n_pts < 20:
            continue

        # scipy 最小化拟合（比 RANSAC 对稀疏数据更稳定）
        main_dir, tilt, linearity, length, lateral = _pca_cluster_info(gpts)
        fit = _fit_cylinder_scipy(gpts, main_dir)

        if fit is None:
            continue

        # 也做一轮 RANSAC 看能否拿到更好结果
        res_ransac = ransac_cylinder_fit(
            gpts, gnormals,
            distance_threshold=0.02,
            num_iterations=30000,
            radius_min=radius_min,
            radius_max=radius_max)

        # 选择较好的拟合
        if res_ransac and res_ransac.valid and res_ransac.inlier_ratio > 0.3:
            result = res_ransac
        else:
            # 用 scipy 结果构建 CylinderResult
            result = CylinderResult()
            result.center = fit['center']
            result.axis = fit['axis']
            result.radius = fit['radius']
            result.height = fit['height']
            result.tilt_angle_deg = fit['tilt']
            result.inlier_count = fit['n_inliers']
            result.inlier_ratio = fit['inlier_ratio']
            result.radial_rel_std = fit['rmse'] / fit['radius'] if fit['radius'] > 0 else 1.0
            result.aspect_ratio = fit['height'] / (2 * fit['radius']) if fit['radius'] > 0 else 0
            result.circularity = compute_circularity(gpts, fit['center'], fit['axis'])
            # 计算内点索引（在 gpts 中）
            diff = gpts - fit['center']
            proj = np.dot(diff, fit['axis'])
            perp = diff - proj[:, np.newaxis] * fit['axis']
            dists = np.linalg.norm(perp, axis=1)
            result.inlier_indices = np.where(np.abs(dists - fit['radius']) < 0.02)[0]
            result.inlier_count = len(result.inlier_indices)
            result.inlier_ratio = result.inlier_count / n_pts
            result.valid = True

        if not result.valid:
            continue

        # === 倾斜角感知评分 ===
        tilt_dev = abs(result.tilt_angle_deg - target_tilt)
        tilt_score = max(0, 1.0 - tilt_dev / tilt_tolerance) * 25  # 0~25
        fit_score = min(result.inlier_ratio, 0.8) / 0.8 * 20       # 0~20
        # 形状分: 长宽比 2~12 为佳，过大说明多段合并或是线结构
        ar_scaled = min(result.aspect_ratio / 5.0, 1.0)
        if result.aspect_ratio > 12:
            ar_scaled *= max(0.2, 1.0 - (result.aspect_ratio - 12) / 10)
        shape_score = ar_scaled * 12                                  # 0~12
        circ_score = result.circularity * 5                            # 0~5
        size_score = min(n_pts / 200.0, 1.0) * 5                     # 0~5
        # 线性度分: 0.85~0.97 为佳，>0.98 反而说明是线而非面
        lin_eff = linearity
        if linearity > 0.98:
            lin_eff = 0.98 - (linearity - 0.98) * 3  # 惩罚过高线性度
        lin_score = lin_eff * 13                                       # 0~13
        # 径向一致性: std/mean 越小越好（真实圆柱 < 0.3）
        rad_consist = max(0, 1.0 - result.radial_rel_std / 0.5) * 10  # 0~10
        # 高度合理性: 0.3~2.5m 为佳, 过短或过长惩罚
        h = result.height
        if 0.3 <= h <= 2.5:
            height_score = 10.0
        elif h < 0.3:
            height_score = max(0, h / 0.3) * 10
        else:
            height_score = max(0, 1.0 - (h - 2.5) / 3.0) * 10  # 5.5m 归零
        total = (tilt_score + fit_score + shape_score + circ_score +
                 size_score + lin_score + rad_consist + height_score)

        is_merged = group.get('merged', False)
        label = "合并组" if is_merged else "单聚类"
        print(f"\n  [{label}] {n_pts}点, R={result.radius*1000:.1f}mm, "
              f"H={result.height:.3f}m, 倾斜={result.tilt_angle_deg:.1f}°")
        print(f"    评分: 总={total:.1f} (倾斜={tilt_score:.1f}, 拟合={fit_score:.1f}, "
              f"形状={shape_score:.1f}, 圆度={circ_score:.1f}, "
              f"大小={size_score:.1f}, 线性={lin_score:.1f}, "
              f"径向={rad_consist:.1f}, 高度={height_score:.1f})")

        if total > best_score:
            best_score = total
            best_result = result

    if best_result:
        print(f"\n  [定向检测] 最佳: R={best_result.radius*1000:.1f}mm, "
              f"H={best_result.height:.3f}m, 倾斜={best_result.tilt_angle_deg:.1f}°, "
              f"得分={best_score:.1f}")

    return best_result


def _try_merge_candidates(candidates: list, target_tilt: float) -> list:
    """
    尝试合并同一圆柱的不同片段。
    合并条件: PCA 方向余弦 > 0.95 且 中心距离 < 2m
    """
    if len(candidates) <= 1:
        return [{'points': c['points'], 'normals': c['normals'],
                 'merged': False} for c in candidates]

    # 每个候选单独作为一组
    groups = [{'points': c['points'], 'normals': c['normals'],
               'merged': False, 'pca_dir': c['pca_dir'],
               'center': c['center']} for c in candidates]

    # 尝试两两合并
    merged_groups = []
    used = set()

    for i in range(len(candidates)):
        if i in used:
            continue
        for j in range(i + 1, len(candidates)):
            if j in used:
                continue
            ci = candidates[i]
            cj = candidates[j]

            # 只合并高线性度的候选 (排除散乱团簇)
            if ci['linearity'] < 0.90 or cj['linearity'] < 0.90:
                continue

            # 方向一致性
            cos_sim = abs(np.dot(ci['pca_dir'], cj['pca_dir']))
            # 中心距离
            dist = np.linalg.norm(ci['center'] - cj['center'])

            if cos_sim > 0.95 and dist < 2.0:
                merged_pts = np.vstack([ci['points'], cj['points']])
                merged_norms = np.vstack([ci['normals'], cj['normals']])
                # 检查合并后是否仍满足倾斜要求
                main_dir, tilt, lin, length, lat = _pca_cluster_info(merged_pts)
                if abs(tilt - target_tilt) < 25:
                    merged_groups.append({
                        'points': merged_pts, 'normals': merged_norms,
                        'merged': True, 'pca_dir': main_dir,
                        'center': merged_pts.mean(0)
                    })
                    used.add(i)
                    used.add(j)
                    print(f"  → 合并候选 {i}+{j}: {len(merged_pts)}点, "
                          f"方向cos={cos_sim:.3f}, 距离={dist:.2f}m, "
                          f"合并后倾斜={tilt:.1f}°")

    # 未被合并的单独保留
    for i in range(len(candidates)):
        if i not in used:
            merged_groups.append(groups[i])

    return merged_groups


def _fit_cylinder_scipy(pts: np.ndarray, init_axis: np.ndarray) -> Optional[dict]:
    """scipy least_squares 圆柱拟合 (球坐标参数化)"""
    from scipy.optimize import least_squares

    center = pts.mean(0)
    theta0 = np.arccos(np.clip(init_axis[2], -1, 1))
    phi0 = np.arctan2(init_axis[1], init_axis[0])
    proj = (pts - center) @ init_axis
    perp = (pts - center) - np.outer(proj, init_axis)
    r0 = np.median(np.linalg.norm(perp, axis=1))

    def residuals(params):
        cx, cy, cz, theta, phi, R = params
        ax = np.array([np.sin(theta) * np.cos(phi),
                        np.sin(theta) * np.sin(phi),
                        np.cos(theta)])
        diff = pts - np.array([cx, cy, cz])
        proj = diff @ ax
        perp = diff - np.outer(proj, ax)
        return np.linalg.norm(perp, axis=1) - R

    try:
        res = least_squares(residuals,
                            [center[0], center[1], center[2], theta0, phi0, r0],
                            method='lm', max_nfev=5000)
        cx, cy, cz, theta, phi, R = res.x
        axis = np.array([np.sin(theta) * np.cos(phi),
                          np.sin(theta) * np.sin(phi),
                          np.cos(theta)])
        if axis[2] < 0:
            axis = -axis
        center_fit = np.array([cx, cy, cz])
        resid = np.abs(res.fun)
        inlier_mask = resid < 0.02
        proj_vals = (pts - center_fit) @ axis
        return {
            'center': center_fit, 'axis': axis, 'radius': abs(R),
            'height': proj_vals.max() - proj_vals.min(),
            'tilt': np.degrees(np.arccos(np.clip(abs(axis[2]), -1, 1))),
            'rmse': np.sqrt(np.mean(res.fun ** 2)),
            'inlier_ratio': inlier_mask.sum() / len(pts),
            'n_inliers': int(inlier_mask.sum())
        }
    except Exception:
        return None


def _strategy_cluster_fit(points: np.ndarray, normals: np.ndarray,
                          radius_min: float, radius_max: float) -> Optional[CylinderResult]:
    """DBSCAN 聚类 → 逐聚类拟合 → 返回最佳"""
    pcd_tmp = o3d.geometry.PointCloud()
    pcd_tmp.points = o3d.utility.Vector3dVector(points)

    best_result = None
    best_score = 0

    for eps, min_pts in [(0.05, 30), (0.08, 20), (0.15, 15), (0.20, 10)]:
        labels = np.array(pcd_tmp.cluster_dbscan(eps=eps, min_points=min_pts))
        if len(labels) == 0 or labels.max() < 0:
            continue

        n_clusters = labels.max() + 1
        print(f"  DBSCAN eps={eps}: {n_clusters} 聚类")

        for c in range(n_clusters):
            mask = labels == c
            cp = points[mask]
            cn = normals[mask]
            if len(cp) < 30:
                continue

            centered = cp - cp.mean(0)
            evals = np.linalg.eigvalsh(np.cov(centered.T))[::-1]
            linearity = (evals[0] - evals[1]) / evals[0] if evals[0] > 0 else 0
            if linearity < 0.5:
                continue

            res = ransac_cylinder_fit(
                cp, cn,
                distance_threshold=0.01,
                num_iterations=20000,
                radius_min=radius_min,
                radius_max=radius_max)

            if res is None or not res.valid:
                continue

            score = (min(res.inlier_ratio, 0.8) * 30 +
                     min(res.aspect_ratio / 10.0, 1.0) * 30 +
                     res.circularity * 20 +
                     max(0, 1 - res.radial_rel_std / 0.3) * 20)

            print(f"    聚类{c}: 得分={score:.1f}, R={res.radius:.4f}m, "
                  f"H={res.height:.3f}m, 倾斜={res.tilt_angle_deg:.1f}°")

            if score > best_score:
                best_score = score
                best_result = res

    return best_result


# ============================================================================
# Matplotlib 交互式可视化
# ============================================================================

def _show_raw_cloud(points: np.ndarray, title: str = "点云",
                    max_plot: int = 15000):
    """显示原始点云的交互式 3D 视图"""
    if len(points) > max_plot:
        idx = np.random.choice(len(points), max_plot, replace=False)
        pts = points[idx]
    else:
        pts = points

    z_norm = (pts[:, 2] - pts[:, 2].min()) / max(pts[:, 2].max() - pts[:, 2].min(), 1e-6)

    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')
    sc = ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2],
                    c=z_norm, cmap='viridis', s=1, alpha=0.6)
    plt.colorbar(sc, ax=ax, label='高度 (归一化)', shrink=0.6)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title(title)
    _set_equal_aspect(ax, pts)
    plt.tight_layout()
    print(f"  [交互窗口] {title}")
    print(f"  提示: 拖拽旋转, 滚轮缩放, 关闭窗口继续运行")
    plt.show()


def _visualize_interactive(points: np.ndarray, result: CylinderResult,
                           title: str = "圆柱拟合结果",
                           max_plot: int = 15000):
    """交互式可视化拟合结果"""
    print(f"\n[可视化] {title}")

    if len(points) > max_plot:
        idx = np.random.choice(len(points), max_plot, replace=False)
        pts = points[idx]
    else:
        pts = points
        idx = np.arange(len(points))

    inlier_set = set(result.inlier_indices.tolist())
    is_inlier = np.array([i in inlier_set for i in idx])

    fig = plt.figure(figsize=(18, 10))

    # 3D 交互视图
    ax1 = fig.add_subplot(1, 2, 1, projection='3d')
    if np.any(~is_inlier):
        ax1.scatter(pts[~is_inlier, 0], pts[~is_inlier, 1], pts[~is_inlier, 2],
                    c='#AAAAAA', s=1, alpha=0.3, label=f'其他点 ({np.sum(~is_inlier)})')
    if np.any(is_inlier):
        ax1.scatter(pts[is_inlier, 0], pts[is_inlier, 1], pts[is_inlier, 2],
                    c='red', s=3, alpha=0.7, label=f'圆柱内点 ({result.inlier_count})')

    _draw_cylinder_3d(ax1, result)

    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title('3D 视图（拖拽旋转）')
    ax1.legend(loc='upper left', fontsize=9)
    _set_equal_aspect(ax1, pts)

    # 信息面板
    ax2 = fig.add_subplot(1, 2, 2)
    ax2.axis('off')
    info = (
        f"圆柱拟合结果\n"
        f"{'─' * 35}\n"
        f"半径:       {result.radius:.4f} m  ({result.radius*1000:.1f} mm)\n"
        f"高度:       {result.height:.3f} m  ({result.height*1000:.0f} mm)\n"
        f"倾斜角:     {result.tilt_angle_deg:.1f}°  (与竖直方向)\n"
        f"长宽比:     {result.aspect_ratio:.1f}\n"
        f"{'─' * 35}\n"
        f"中心点:     ({result.center[0]:.3f}, {result.center[1]:.3f}, {result.center[2]:.3f})\n"
        f"轴线方向:   ({result.axis[0]:.4f}, {result.axis[1]:.4f}, {result.axis[2]:.4f})\n"
        f"{'─' * 35}\n"
        f"内点数:     {result.inlier_count}\n"
        f"内点率:     {result.inlier_ratio:.1%}\n"
        f"径向偏差:   {result.radial_rel_std:.4f}\n"
        f"圆度评分:   {result.circularity:.3f}\n"
    )
    ax2.text(0.05, 0.95, info, transform=ax2.transAxes,
             fontsize=13, verticalalignment='top', fontfamily='monospace',
             bbox=dict(boxstyle='round,pad=0.5', facecolor='lightyellow', alpha=0.9))

    fig.suptitle(title, fontsize=14, fontweight='bold')
    plt.tight_layout()

    save_path = 'cylinder_fit_result.png'
    plt.savefig(save_path, dpi=120, bbox_inches='tight')
    print(f"  已保存图片: {os.path.abspath(save_path)}")
    print(f"  提示: 拖拽旋转, 滚轮缩放, 关闭窗口继续运行")
    plt.show()


def _draw_cylinder_3d(ax, result: CylinderResult):
    """在 matplotlib 3D 轴上绘制拟合圆柱"""
    half_h = result.height / 2.0
    axis = result.axis

    ext = 0.1 * result.height
    start = result.center + axis * (-half_h - ext)
    end = result.center + axis * (half_h + ext)
    ax.plot([start[0], end[0]], [start[1], end[1]], [start[2], end[2]],
            color='yellow', linewidth=3, label='拟合轴线')

    ref = np.array([1, 0, 0]) if abs(axis[0]) < 0.9 else np.array([0, 1, 0])
    u = np.cross(axis, ref)
    u = u / np.linalg.norm(u)
    v = np.cross(axis, u)
    v = v / np.linalg.norm(v)

    theta = np.linspace(0, 2 * np.pi, 50)
    for t in np.linspace(-half_h, half_h, 7):
        c = result.center + axis * t
        ring = (c[np.newaxis, :]
                + result.radius * np.outer(np.cos(theta), u)
                + result.radius * np.outer(np.sin(theta), v))
        ax.plot(ring[:, 0], ring[:, 1], ring[:, 2],
                color='lime', linewidth=1.0, alpha=0.7)

    for angle in np.linspace(0, 2 * np.pi, 12, endpoint=False):
        offset = result.radius * (np.cos(angle) * u + np.sin(angle) * v)
        p1 = result.center + axis * (-half_h) + offset
        p2 = result.center + axis * half_h + offset
        ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]],
                color='lime', linewidth=0.5, alpha=0.4)


def _set_equal_aspect(ax, pts):
    """设置 3D 轴等比例"""
    max_range = max(pts[:, 0].max() - pts[:, 0].min(),
                    pts[:, 1].max() - pts[:, 1].min(),
                    pts[:, 2].max() - pts[:, 2].min()) / 2
    mid = pts.mean(axis=0)
    ax.set_xlim(mid[0] - max_range, mid[0] + max_range)
    ax.set_ylim(mid[1] - max_range, mid[1] + max_range)
    ax.set_zlim(mid[2] - max_range, mid[2] + max_range)


# ============================================================================
# YAML 配置生成
# ============================================================================

def generate_yaml_config(result: CylinderResult,
                         output_path: Optional[str] = None) -> str:
    """根据拟合结果生成推荐的 YAML 配置"""
    tilt = result.tilt_angle_deg
    R = result.radius
    H = result.height

    eps_angle = min(tilt + 15.0, 90.0)
    min_radius = max(0.005, R * 0.5)
    max_radius = R * 2.0

    # ROS 在线检测可能只看到圆柱的一部分，min_height 不能太大
    # 使用 max(0.1, min(H*0.15, 0.5)) 确保即使只看到局部也能检测
    min_height = max(0.1, min(H * 0.15, 0.5))
    max_height = H * 1.5

    # RANSAC 距离阈值: 与半径成比例但有下限
    ransac_dist = max(0.01, min(R * 0.15, 0.03))

    yaml_text = f"""# 圆柱检测推荐参数 (自动生成)
# 基于实测: R={R:.4f}m, H={H:.3f}m, 倾斜={tilt:.1f}°
# 轴向: [{result.axis[0]:.4f}, {result.axis[1]:.4f}, {result.axis[2]:.4f}]
# 中心: [{result.center[0]:.4f}, {result.center[1]:.4f}, {result.center[2]:.4f}]

cylinder_detection:
  # 几何约束
  eps_angle_deg: {eps_angle:.1f}
  min_radius: {min_radius:.4f}
  max_radius: {max_radius:.4f}
  min_height: {min_height:.3f}
  max_height: {max_height:.3f}

  # RANSAC 参数
  normal_distance_weight: 0.1
  ransac_max_iterations: 50000
  ransac_distance_threshold: {ransac_dist:.4f}

  # 验证阈值
  max_radial_range_ratio: {max(0.5, min(result.radial_range_ratio * 1.5, 2.0)):.2f}
  min_aspect_ratio: {max(1.0, min(result.aspect_ratio * 0.3, 5.0)):.1f}
  min_circularity: {max(0.2, result.circularity * 0.5):.2f}

  # 法线估计
  normal_k_search: 30

  # 环境
  wall_distance_threshold: 0.02
  wall_max_iterations: 3000
  max_walls: 3
"""

    if output_path:
        with open(output_path, 'w') as f:
            f.write(yaml_text)
        print(f"[保存] YAML: {os.path.abspath(output_path)}")

    return yaml_text


def print_analysis_report(result: CylinderResult):
    """打印拟合报告"""
    print(f"""
{'=' * 70}
圆柱拟合分析报告
{'=' * 70}
  半径:     {result.radius:.4f} m  ({result.radius * 1000:.1f} mm)
  高度:     {result.height:.3f} m  ({result.height * 1000:.0f} mm)
  倾斜角:   {result.tilt_angle_deg:.1f}° (与竖直方向)
  长宽比:   {result.aspect_ratio:.1f} (H/2R)

  轴线中心: [{result.center[0]:.4f}, {result.center[1]:.4f}, {result.center[2]:.4f}]
  轴线方向: [{result.axis[0]:.4f}, {result.axis[1]:.4f}, {result.axis[2]:.4f}]

  内点数:   {result.inlier_count}
  内点比例: {result.inlier_ratio:.1%}
  径向偏差: {result.radial_rel_std:.4f}
  圆度:     {result.circularity:.3f}
{'=' * 70}
""")


# ============================================================================
# 主入口
# ============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="倾斜细长圆柱离线分析工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python cylinder_yaml_den.py -f 113.pcd -v                    # 指定文件 + 可视化
  python cylinder_yaml_den.py -f 113.pcd -v --tilt-angle 45    # 检测 45° 倾斜圆柱
  python cylinder_yaml_den.py -f 113.pcd -v --raw              # 先看原始点云
  python cylinder_yaml_den.py --dir pcd/ -v                    # 扫描目录
  python cylinder_yaml_den.py -f 113.pcd -o out.yaml           # 指定输出
        """)

    parser.add_argument('--file', '-f', type=str, default=None,
                        help='指定 PCD 文件路径')
    parser.add_argument('--dir', '-d', type=str, default='pcd',
                        help='PCD 文件目录 (默认: pcd/)')
    parser.add_argument('--visualize', '-v', action='store_true',
                        help='启用交互式 matplotlib 可视化')
    parser.add_argument('--raw', action='store_true',
                        help='先显示原始点云（配合 -v 使用）')
    parser.add_argument('--output', '-o', type=str,
                        default='recommended_cylinder_params.yaml',
                        help='输出 YAML 文件路径')
    parser.add_argument('--voxel-size', type=float, default=0.005,
                        help='体素降采样尺寸 (米, 默认 0.005)')
    parser.add_argument('--radius-min', type=float, default=0.005,
                        help='圆柱最小半径 (米, 默认 0.005)')
    parser.add_argument('--radius-max', type=float, default=1.0,
                        help='圆柱最大半径 (米, 默认 1.0)')
    parser.add_argument('--all', '-a', action='store_true',
                        help='分析目录中所有 PCD 文件')
    parser.add_argument('--tilt-angle', '-t', type=float, default=None,
                        help='目标倾斜角度 (与竖直方向的夹角, 度)。'
                             '例如 45 表示检测约 45° 倾斜的圆柱')
    parser.add_argument('--tilt-tolerance', type=float, default=20.0,
                        help='倾斜角容差 (度, 默认 20)')

    args = parser.parse_args()

    pcd_files = []
    if args.file:
        if os.path.exists(args.file):
            pcd_files = [args.file]
        else:
            print(f"错误: 文件不存在: {args.file}")
            sys.exit(1)
    else:
        if not os.path.isdir(args.dir):
            print(f"错误: 目录不存在: {args.dir}")
            sys.exit(1)
        pcd_files = sorted(glob.glob(os.path.join(args.dir, '*.pcd')))
        if not pcd_files:
            print(f"错误: {args.dir}/ 中未找到 .pcd 文件")
            sys.exit(1)
        print(f"找到 {len(pcd_files)} 个 PCD 文件")
        if not args.all:
            mid = len(pcd_files) // 2
            pcd_files = [pcd_files[mid]]
            print(f"默认分析中间帧: {pcd_files[0]}")

    all_results = []
    for pcd_path in pcd_files:
        print(f"\n{'#' * 70}")
        print(f"# 分析: {pcd_path}")
        print(f"{'#' * 70}")

        try:
            pcd = load_pcd(pcd_path)
            result = detect_cylinder(
                pcd,
                voxel_size=args.voxel_size,
                visualize=args.visualize,
                show_raw=args.raw and args.visualize,
                radius_min=args.radius_min,
                radius_max=args.radius_max,
                tilt_angle=args.tilt_angle,
                tilt_tolerance=args.tilt_tolerance)

            if result and result.valid:
                all_results.append((pcd_path, result))
                print_analysis_report(result)
            else:
                print(f"\n[失败] 未能检测到圆柱")
        except Exception as e:
            print(f"\n[错误] {e}")
            import traceback
            traceback.print_exc()

    if not all_results:
        print("\n[结论] 未检测到圆柱。建议:")
        print("  1. 用 -v --raw 查看点云")
        print("  2. 调整 --radius-min / --radius-max")
        print("  3. 调整 --voxel-size (试 0.002)")
        sys.exit(1)

    if len(all_results) > 1:
        all_results.sort(key=lambda x: x[1].inlier_count, reverse=True)

    best_file, best_result = all_results[0]

    print(f"\n{'=' * 70}")
    print("生成推荐 YAML 配置")
    print(f"{'=' * 70}")
    yaml_text = generate_yaml_config(best_result, args.output)
    print(yaml_text)
    print(f"\n[完成] 推荐配置已保存到: {args.output}")


if __name__ == '__main__':
    main()