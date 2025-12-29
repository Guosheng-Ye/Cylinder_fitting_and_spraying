#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
圆柱拟合日志解析工具
用于解析ROS圆柱检测日志，提取关键参数并生成分析报告
支持3D可视化：圆柱体、切片点、坐标原点、虚拟机械臂、工作球形等
"""

import re
import sys
import math
import numpy as np
from dataclasses import dataclass, field
from typing import List, Optional, Tuple
from pathlib import Path

# 可视化依赖（可选）
try:
    import matplotlib
    # 使用交互式后端
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    # 设置中文字体支持
    plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans', 'WenQuanYi Micro Hei', 'Noto Sans CJK SC', 'Arial Unicode MS']
    plt.rcParams['axes.unicode_minus'] = False
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False
    print("警告: matplotlib 未安装，可视化功能不可用。安装: pip install matplotlib")


@dataclass
class CylinderInfo:
    """圆柱体检测信息"""
    status: str = ""  # ACCEPTED / REJECTED
    reject_reason: str = ""
    inliers: int = 0
    inlier_ratio: float = 0.0
    height_m: float = 0.0
    radius_m: float = 0.0
    radial_rel_std: float = 0.0
    radial_range: float = 0.0
    aspect_ratio: float = 0.0
    circularity: float = 0.0
    bottom_center: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    geo_center: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    top_center: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    axis: Tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass
class SliceInfo:
    """切片信息"""
    slice_id: int = 0
    circle_center: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # LiDAR坐标系 (m)
    spray_center: Tuple[float, float, float] = (0.0, 0.0, 0.0)   # LiDAR坐标系 (m)
    work_point: Tuple[float, float, float] = (0.0, 0.0, 0.0)     # Base坐标系 (m)
    work_point_lidar: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # LiDAR坐标系 (m)
    # Base坐标系下的圆心和喷涂中心 (mm)
    circle_center_base_mm: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    spray_center_base_mm: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    target_id: int = 0
    layer: int = 0
    skipped: bool = False
    skip_reason: str = ""


@dataclass
class SlicingParams:
    """切片参数"""
    slice_width_mm: float = 0.0
    slice_height_mm: float = 0.0
    total_slices: int = 0
    height_layers: int = 0
    width_divisions: int = 0
    single_slice_area_m2: float = 0.0
    total_area_m2: float = 0.0
    slice_angle_deg: float = 0.0
    coverage_angle_deg: float = 0.0


@dataclass
class WorkParams:
    """工作参数"""
    spray_distance_mm: float = 0.0
    work_surface_mm: float = 0.0
    robot_arm_reach_mm: float = 0.0
    effective_reach_mm: float = 0.0
    virtual_base_to_axis_mm: float = 0.0
    work_point_radius_mm: float = 0.0
    virtual_base_radius_mm: float = 0.0


@dataclass
class OptimizationParams:
    """优化参数"""
    max_coverage_angle_deg: float = 0.0
    optimal_slice_width_deg: float = 0.0
    robot_base_positions: int = 0
    spray_passes_per_position: int = 0
    total_spray_passes: int = 0
    initial_overlap_ratio: float = 0.0
    adjusted_overlap_ratio: float = 0.0
    adjusted_w0_deg: float = 0.0
    adjusted_w0_mm: float = 0.0
    step_width_mm: float = 0.0


@dataclass
class ParseResult:
    """解析结果"""
    filename: str = ""
    cylinder: CylinderInfo = field(default_factory=CylinderInfo)
    slicing: SlicingParams = field(default_factory=SlicingParams)
    work: WorkParams = field(default_factory=WorkParams)
    optimization: OptimizationParams = field(default_factory=OptimizationParams)
    slices: List[SliceInfo] = field(default_factory=list)
    valid_slices: int = 0
    skipped_slices: int = 0
    valid_layers: int = 0
    max_working_z_mm: float = 0.0
    z_offset_m: float = 0.0
    # 额外信息用于可视化
    virtual_base_position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    cylinder_angle_deg: float = 0.0


def parse_tuple(s: str) -> Tuple[float, float, float]:
    """解析三元组字符串"""
    # 匹配 [x, y, z] 或 (x, y, z) 格式
    match = re.search(r'[\[\(]\s*([-\d.]+)[,\s]+([-\d.]+)[,\s]+([-\d.]+)\s*[\]\)]', s)
    if match:
        return (float(match.group(1)), float(match.group(2)), float(match.group(3)))
    return (0.0, 0.0, 0.0)


def parse_log_file(filepath: str) -> ParseResult:
    """解析日志文件"""
    result = ParseResult()
    result.filename = Path(filepath).name
    
    with open(filepath, 'r', encoding='utf-8') as f:
        content = f.read()
    
    lines = content.split('\n')
    
    # 查找最后一个成功的圆柱检测块
    # 首先找到所有 "Cylinder ACCEPTED" 的位置
    accepted_positions = [m.start() for m in re.finditer(r'Cylinder ACCEPTED', content)]
    
    if accepted_positions:
        # 找到最后一个 ACCEPTED 之前的验证块起始位置
        last_accepted_pos = accepted_positions[-1]
        # 往前找 "========== Cylinder Validation ==========" 
        validation_starts = [m.start() for m in re.finditer(r'=+ Cylinder Validation =+', content)]
        
        # 找到最后一个ACCEPTED对应的验证块
        relevant_start = 0
        for pos in validation_starts:
            if pos < last_accepted_pos:
                relevant_start = pos
        
        # 截取相关内容用于解析圆柱参数
        cylinder_content = content[relevant_start:last_accepted_pos + 1000]
    else:
        # 没有成功的检测，使用最后一个验证块
        cylinder_content = content
    
    # 解析圆柱验证信息
    cylinder = CylinderInfo()
    
    # 检测状态
    if 'Cylinder ACCEPTED' in content:
        cylinder.status = "ACCEPTED"
    elif 'Cylinder REJECTED' in content:
        cylinder.status = "REJECTED"
        match = re.search(r'Cylinder REJECTED:\s*(.+)', content)
        if match:
            cylinder.reject_reason = match.group(1).strip()
    
    # 从相关内容中提取参数
    # 内点数
    match = re.search(r'Inliers:\s+(\d+)', cylinder_content)
    if match:
        cylinder.inliers = int(match.group(1))
    
    # 内点比例
    match = re.search(r'Inlier ratio:\s+([\d.]+)', cylinder_content)
    if match:
        cylinder.inlier_ratio = float(match.group(1))
    
    # 高度
    match = re.search(r'Height:\s+([\d.]+)\s*m', cylinder_content)
    if match:
        cylinder.height_m = float(match.group(1))
    
    # 半径
    match = re.search(r'Radius:\s+([\d.]+)\s*m\s+\[', cylinder_content)
    if match:
        cylinder.radius_m = float(match.group(1))
    
    # 径向相对标准差
    match = re.search(r'Radial rel std:\s+([\d.]+)', cylinder_content)
    if match:
        cylinder.radial_rel_std = float(match.group(1))
    
    # 径向范围
    match = re.search(r'Radial range:\s+([\d.]+)', cylinder_content)
    if match:
        cylinder.radial_range = float(match.group(1))
    
    # 长宽比
    match = re.search(r'Aspect ratio:\s+([\d.]+)', cylinder_content)
    if match:
        cylinder.aspect_ratio = float(match.group(1))
    
    # 圆度
    match = re.search(r'Circularity:\s+([\d.]+)', cylinder_content)
    if match:
        cylinder.circularity = float(match.group(1))
    
    # 底面中心
    match = re.search(r'底面中心:\s*\[([-\d.\s]+)\]', content)
    if match:
        vals = re.findall(r'[-\d.]+', match.group(1))
        if len(vals) >= 3:
            cylinder.bottom_center = (float(vals[0]), float(vals[1]), float(vals[2]))
    
    # 几何中心
    match = re.search(r'几何中心:\s*\[([-\d.\s]+)\]', content)
    if match:
        vals = re.findall(r'[-\d.]+', match.group(1))
        if len(vals) >= 3:
            cylinder.geo_center = (float(vals[0]), float(vals[1]), float(vals[2]))
    
    # 顶面中心
    match = re.search(r'顶面中心:\s*\[([-\d.\s]+)\]', content)
    if match:
        vals = re.findall(r'[-\d.]+', match.group(1))
        if len(vals) >= 3:
            cylinder.top_center = (float(vals[0]), float(vals[1]), float(vals[2]))
    
    # 轴向
    match = re.search(r'轴向:\s*\[([-\d.\s]+)\]', content)
    if match:
        vals = re.findall(r'[-\d.]+', match.group(1))
        if len(vals) >= 3:
            cylinder.axis = (float(vals[0]), float(vals[1]), float(vals[2]))
    
    result.cylinder = cylinder
    
    # 解析切片参数
    slicing = SlicingParams()
    
    # 切片宽度
    match = re.search(r'SLICE_WIDTH_MM:\[([\d.]+)\]', content)
    if match:
        slicing.slice_width_mm = float(match.group(1))
    
    # 切片高度
    match = re.search(r'SLICE_HEIGHT_MM:\[([\d.]+)\]', content)
    if match:
        slicing.slice_height_mm = float(match.group(1))
    
    # 总切片数
    match = re.search(r'slice_count=(\d+)', content)
    if match:
        slicing.total_slices = int(match.group(1))
    
    # 高度层数和宽度份数
    match = re.search(r'height_total=(\d+),\s*width_total=(\d+)', content)
    if match:
        slicing.height_layers = int(match.group(1))
        slicing.width_divisions = int(match.group(2))
    
    # 单片面积
    match = re.search(r'single_slice_area=([\d.]+)\s*m\^2', content)
    if match:
        slicing.single_slice_area_m2 = float(match.group(1))
    
    # 总面积
    match = re.search(r'total_area=([\d.]+)\s*m\^2', content)
    if match:
        slicing.total_area_m2 = float(match.group(1))
    
    # 切片角度
    match = re.search(r'slice angle:\s*([\d.]+)\s*deg', content)
    if match:
        slicing.slice_angle_deg = float(match.group(1))
    
    # 覆盖角度
    match = re.search(r'ANGEL_RANGE_DEG:\[([\d.]+)\]', content)
    if match:
        slicing.coverage_angle_deg = float(match.group(1))
    
    result.slicing = slicing
    
    # 解析工作参数
    work = WorkParams()
    
    # 喷涂距离
    match = re.search(r'sparay_distance:([\d.]+)', content)
    if match:
        work.spray_distance_mm = float(match.group(1))
    
    # 工作表面距离
    match = re.search(r'work_surface:([\d.]+)', content)
    if match:
        work.work_surface_mm = float(match.group(1))
    
    # 机器人臂长
    match = re.search(r'robot_R_arm:([\d.]+)', content)
    if match:
        work.robot_arm_reach_mm = float(match.group(1))
    
    # 有效水平伸展
    match = re.search(r'Effective horizontal reach \(R_eff\):\s*([\d.]+)\s*mm', content)
    if match:
        work.effective_reach_mm = float(match.group(1))
    
    # 虚拟基座到圆柱轴距离
    match = re.search(r'Distance from virtual base to cylinder axis:\s*([\d.]+)\s*mm', content)
    if match:
        work.virtual_base_to_axis_mm = float(match.group(1))
    
    # 工作点半径
    match = re.search(r'Work Point Radius:\s*([\d.]+)\s*m', content)
    if match:
        work.work_point_radius_mm = float(match.group(1)) * 1000
    
    # 虚拟基座半径
    match = re.search(r'Virtual Base Radius:\s*([\d.]+)\s*m', content)
    if match:
        work.virtual_base_radius_mm = float(match.group(1)) * 1000
    
    result.work = work
    
    # 解析优化参数
    opt = OptimizationParams()
    
    # 最大覆盖角度
    match = re.search(r'Max Coverage Angle \(theta_max\)[^:]*:\s*([\d.]+)\s*deg', content)
    if match:
        opt.max_coverage_angle_deg = float(match.group(1))
    
    # 最优切片宽度
    match = re.search(r'Optimal Slice Width \(w\):\s*([\d.]+)\s*deg', content)
    if match:
        opt.optimal_slice_width_deg = float(match.group(1))
    
    # 机器人基座位置数
    match = re.search(r'Robot Base Positions:\s*(\d+)', content)
    if match:
        opt.robot_base_positions = int(match.group(1))
    
    # 每位置喷涂次数
    match = re.search(r'Spray Passes per Position \(k\):\s*(\d+)', content)
    if match:
        opt.spray_passes_per_position = int(match.group(1))
    
    # 总喷涂次数
    match = re.search(r'Total Spray Passes:\s*(\d+)', content)
    if match:
        opt.total_spray_passes = int(match.group(1))
    
    # 初始重叠率
    match = re.search(r'\[Before\] Overlap Ratio \(r\):\s*([\d.]+)', content)
    if match:
        opt.initial_overlap_ratio = float(match.group(1))
    
    # 调整后重叠率
    match = re.search(r'Adjusted Overlap Ratio:\s*([\d.]+)', content)
    if match:
        opt.adjusted_overlap_ratio = float(match.group(1))
    
    # 调整后 w0
    match = re.search(r'Adjusted w0:\s*([\d.]+)\s*deg\s*\(([\d.]+)\s*mm', content)
    if match:
        opt.adjusted_w0_deg = float(match.group(1))
        opt.adjusted_w0_mm = float(match.group(2))
    
    # 步宽
    match = re.search(r'step_width=([\d.]+)', content)
    if match:
        opt.step_width_mm = float(match.group(1))
    
    result.optimization = opt
    
    # 解析切片详情
    # 日志中有两组坐标：
    # 1. Source pos_... work_point: [x, y, z] - LiDAR坐标系 (用于可视化)
    # 2. Slice id(base mm): ... work_point: [x, y, z] - Base坐标系 (机器人实际使用)
    # 我们需要同时保存两个坐标系的数据
    
    slices = []
    valid_count = 0
    skipped_count = 0
    
    # 使用多行匹配来获取完整的切片信息
    # 每个切片有: Source行 -> Slice id行 -> (Target ID行 或 Skipping行)
    
    # 先找到所有Source行及其后续信息
    source_pattern = re.compile(
        r'Source pos_circle_center:\s*\[([-\d.,\s]+)\],\s*'
        r'pos_spray_center:\s*\[([-\d.,\s]+)\]\s*,\s*'
        r'work_point:\s*\[([-\d.,\s]+)\]'
    )
    
    # 找到所有匹配
    source_matches = list(source_pattern.finditer(content))
    
    for i, source_match in enumerate(source_matches):
        slice_info = SliceInfo()
        
        # 解析Source行 (LiDAR坐标系, mm)
        circle_coords = re.findall(r'[-\d.]+', source_match.group(1))
        spray_coords = re.findall(r'[-\d.]+', source_match.group(2))
        work_coords = re.findall(r'[-\d.]+', source_match.group(3))
        
        if len(circle_coords) >= 3:
            slice_info.circle_center = (float(circle_coords[0])/1000, 
                                        float(circle_coords[1])/1000, 
                                        float(circle_coords[2])/1000)
        if len(spray_coords) >= 3:
            slice_info.spray_center = (float(spray_coords[0])/1000, 
                                       float(spray_coords[1])/1000, 
                                       float(spray_coords[2])/1000)
        if len(work_coords) >= 3:
            # 存储LiDAR坐标系下的工作点 (m) - 这是Source行中的work_point
            slice_info.work_point_lidar = (float(work_coords[0])/1000, 
                                            float(work_coords[1])/1000, 
                                            float(work_coords[2])/1000)
        
        # 查找后续的Slice id行和状态
        start_pos = source_match.end()
        end_pos = source_matches[i+1].start() if i+1 < len(source_matches) else len(content)
        following_text = content[start_pos:end_pos]
        
        # 获取Slice ID
        slice_id_match = re.search(r'Slice id\(base mm\):\s*(\d+)', following_text)
        if slice_id_match:
            slice_info.slice_id = int(slice_id_match.group(1))
        
        # 检查是否被跳过
        if 'exceeds max working Z' in following_text or 'Skipping' in following_text:
            slice_info.skipped = True
            slice_info.skip_reason = "超过最大工作高度"
            layer_match = re.search(r'layer (\d+)\)', following_text)
            if layer_match:
                slice_info.layer = int(layer_match.group(1))
            skipped_count += 1
        else:
            # 有效切片，获取Target ID
            target_match = re.search(r'Target ID:\s*(\d+)', following_text)
            if target_match:
                slice_info.target_id = int(target_match.group(1))
            
            # 从 Cylinder ID 行中获取 Base 坐标系下的坐标
            # 格式: Cylinder ID: 1,circle:[165.491,8066.396,8185.623],=> Zspray_center:[-92.929,7807.976,8185.623],  => work_point:[-0.517,7.384,8.186], ...
            # circle 和 Zspray_center 单位是 mm，work_point 单位是 m
            cylinder_line_match = re.search(
                r'Cylinder ID:.*?circle:\[([-\d.,\s]+)\].*?Zspray_center:\[([-\d.,\s]+)\].*?work_point:\[([-\d.,\s]+)\]',
                following_text
            )
            if cylinder_line_match:
                # 解析 circle_center (Base坐标系, mm)
                circle_coords = re.findall(r'[-\d.]+', cylinder_line_match.group(1))
                if len(circle_coords) >= 3:
                    slice_info.circle_center_base_mm = (float(circle_coords[0]),
                                                        float(circle_coords[1]),
                                                        float(circle_coords[2]))
                # 解析 spray_center (Base坐标系, mm)
                spray_coords = re.findall(r'[-\d.]+', cylinder_line_match.group(2))
                if len(spray_coords) >= 3:
                    slice_info.spray_center_base_mm = (float(spray_coords[0]),
                                                       float(spray_coords[1]),
                                                       float(spray_coords[2]))
                # 解析 work_point (Base坐标系, m)
                work_base_coords = re.findall(r'[-\d.]+', cylinder_line_match.group(3))
                if len(work_base_coords) >= 3:
                    slice_info.work_point = (float(work_base_coords[0]), 
                                             float(work_base_coords[1]), 
                                             float(work_base_coords[2]))
            
            slice_info.skipped = False
            valid_count += 1
        
        slices.append(slice_info)
    
    result.slices = slices
    result.valid_slices = valid_count
    result.skipped_slices = skipped_count
    
    # 有效层数
    match = re.search(r'Valid layers count:\s*(\d+)', content)
    if match:
        result.valid_layers = int(match.group(1))
    
    # 最大工作Z
    match = re.search(r'max working Z \(([\d.]+) mm\)', content)
    if match:
        result.max_working_z_mm = float(match.group(1))
    
    # Z偏移量
    match = re.search(r'Z Offset Applied \(m\):\s*([\d.]+)', content)
    if match:
        result.z_offset_m = float(match.group(1))
    
    # 虚拟基座位置
    match = re.search(r'Virtual robot base position \(lidar frame\):\s*\[([-\d.,\s]+)\]\s*mm', content)
    if match:
        vals = re.findall(r'[-\d.]+', match.group(1))
        if len(vals) >= 3:
            result.virtual_base_position = (float(vals[0])/1000, float(vals[1])/1000, float(vals[2])/1000)
    
    # 圆柱角度
    match = re.search(r'Angle:\s*([\d.]+)\s*deg', content)
    if match:
        result.cylinder_angle_deg = float(match.group(1))
    
    return result


def format_tuple(t: Tuple[float, float, float], precision: int = 3) -> str:
    """格式化三元组"""
    return f"[{t[0]:.{precision}f}, {t[1]:.{precision}f}, {t[2]:.{precision}f}]"


def print_analysis_report(result: ParseResult):
    """打印分析报告"""
    print("=" * 70)
    print(f"📊 日志分析报告: {result.filename}")
    print("=" * 70)
    
    # 1. 圆柱体检测结果
    print("\n" + "─" * 70)
    print("1️⃣  圆柱体检测结果")
    print("─" * 70)
    cyl = result.cylinder
    status_icon = "✅" if cyl.status == "ACCEPTED" else "❌"
    print(f"  检测状态:       {status_icon} {cyl.status}")
    if cyl.reject_reason:
        print(f"  拒绝原因:       {cyl.reject_reason}")
    print(f"  内点数:         {cyl.inliers}")
    print(f"  内点比例:       {cyl.inlier_ratio:.4f}")
    print(f"  底面中心:       {format_tuple(cyl.bottom_center)} m")
    print(f"  几何中心:       {format_tuple(cyl.geo_center)} m")
    print(f"  顶面中心:       {format_tuple(cyl.top_center)} m")
    print(f"  轴向:           {format_tuple(cyl.axis, 6)}")
    print(f"  半径:           {cyl.radius_m * 1000:.3f} mm ({cyl.radius_m:.4f} m)")
    print(f"  高度:           {cyl.height_m * 1000:.2f} mm ({cyl.height_m:.2f} m)")
    print(f"  圆度:           {cyl.circularity:.4f}")
    print(f"  长宽比:         {cyl.aspect_ratio:.2f}")
    print(f"  径向相对标准差: {cyl.radial_rel_std:.4f}")
    
    # 2. 切片参数
    print("\n" + "─" * 70)
    print("2️⃣  切片参数")
    print("─" * 70)
    slc = result.slicing
    print(f"  单个切片宽度:   {slc.slice_width_mm:.2f} mm")
    print(f"  单个切片高度:   {slc.slice_height_mm:.2f} mm")
    print(f"  总切片数:       {slc.total_slices} 片")
    print(f"  高度方向层数:   {slc.height_layers} 层")
    print(f"  宽度方向份数:   {slc.width_divisions} 份")
    print(f"  单片面积:       {slc.single_slice_area_m2:.4f} m²")
    print(f"  总覆盖面积:     {slc.total_area_m2:.4f} m²")
    print(f"  切片角度:       {slc.slice_angle_deg:.2f} deg")
    print(f"  覆盖角度:       {slc.coverage_angle_deg:.2f} deg")
    
    # 3. 工作距离参数
    print("\n" + "─" * 70)
    print("3️⃣  工作距离参数")
    print("─" * 70)
    wrk = result.work
    print(f"  喷涂距离 (spray_distance):     {wrk.spray_distance_mm:.2f} mm")
    print(f"  工作表面距离 (work_surface):   {wrk.work_surface_mm:.2f} mm")
    print(f"  机器人臂长 (robot_R_arm):      {wrk.robot_arm_reach_mm:.2f} mm")
    print(f"  有效水平伸展:                  {wrk.effective_reach_mm:.2f} mm")
    print(f"  虚拟基座到圆柱轴距离:          {wrk.virtual_base_to_axis_mm:.2f} mm")
    print(f"  工作点半径:                    {wrk.work_point_radius_mm:.2f} mm")
    print(f"  虚拟基座半径:                  {wrk.virtual_base_radius_mm:.2f} mm")
    
    # 4. 喷涂优化参数
    print("\n" + "─" * 70)
    print("4️⃣  喷涂优化参数")
    print("─" * 70)
    opt = result.optimization
    print(f"  最大覆盖角度 (theta_max):      {opt.max_coverage_angle_deg:.2f} deg")
    print(f"  最优切片宽度 (w):              {opt.optimal_slice_width_deg:.2f} deg")
    print(f"  机器人基座位置数 (N):          {opt.robot_base_positions}")
    print(f"  每位置喷涂次数 (k):            {opt.spray_passes_per_position}")
    print(f"  总喷涂次数:                    {opt.total_spray_passes}")
    print(f"  初始重叠率:                    {opt.initial_overlap_ratio * 100:.2f}%")
    print(f"  调整后重叠率:                  {opt.adjusted_overlap_ratio * 100:.2f}%")
    print(f"  调整后 w0:                     {opt.adjusted_w0_deg:.2f} deg ({opt.adjusted_w0_mm:.2f} mm)")
    print(f"  步宽 (step_width):             {opt.step_width_mm:.2f} mm")
    
    # 5. 有效切片统计
    print("\n" + "─" * 70)
    print("5️⃣  有效切片与跳过切片统计")
    print("─" * 70)
    print(f"  总切片数:       {result.slicing.total_slices}")
    print(f"  有效切片数:     {result.valid_slices}")
    print(f"  跳过切片数:     {result.skipped_slices}")
    print(f"  有效层数:       {result.valid_layers}")
    print(f"  最大工作 Z:     {result.max_working_z_mm:.2f} mm")
    print(f"  Z 偏移量:       {result.z_offset_m:.3f} m")
    
    if result.slicing.total_slices > 0:
        valid_ratio = result.valid_slices / result.slicing.total_slices * 100
        print(f"  有效率:         {valid_ratio:.1f}%")
    
    # 6. 工作点列表
    print("\n" + "─" * 70)
    print("6️⃣  有效工作点坐标列表")
    print("─" * 70)
    valid_slices = [s for s in result.slices if not s.skipped]
    if valid_slices:
        print(f"  {'Target ID':<10} {'工作点坐标 (m)':<36} {'到喷涂中心距离':<16} {'到圆心距离':<16}")
        print(f"  {'-'*8:<10} {'-'*34:<36} {'-'*14:<16} {'-'*14:<16}")
        for s in valid_slices:
            # 工作点坐标 (m -> mm)
            wp_mm = (s.work_point[0] * 1000, s.work_point[1] * 1000, s.work_point[2] * 1000)
            # 计算到喷涂中心的距离 (mm)
            dist_to_spray = ((wp_mm[0] - s.spray_center_base_mm[0])**2 +
                             (wp_mm[1] - s.spray_center_base_mm[1])**2 +
                             (wp_mm[2] - s.spray_center_base_mm[2])**2) ** 0.5
            # 计算到圆心的距离 (mm)
            dist_to_circle = ((wp_mm[0] - s.circle_center_base_mm[0])**2 +
                              (wp_mm[1] - s.circle_center_base_mm[1])**2 +
                              (wp_mm[2] - s.circle_center_base_mm[2])**2) ** 0.5
            print(f"  {s.target_id:<10} [{s.work_point[0]:.3f}, {s.work_point[1]:.3f}, {s.work_point[2]:.3f}]" +
                  f"{'':4}{dist_to_spray:>10.2f} mm   {dist_to_circle:>10.2f} mm")
    else:
        print("  无有效工作点")
    
    # 7. TargetPose 与 TargetInfo 对比
    print("\n" + "─" * 120)
    print("7️⃣  TargetPose 与 TargetInfo 数据对比 (按 ID)")
    print("─" * 120)
    if valid_slices:
        print(f"  {'ID':<4} │ {'TargetPose (x,y,z) [m]':<32} │ {'TargetInfo: circle_center (mm)':<36} │ {'spray_center (mm)':<32}")
        print(f"  {'-'*3:<4} │ {'-'*30:<32} │ {'-'*34:<36} │ {'-'*30:<32}")
        for s in valid_slices:
            # TargetPose: work_point (m)
            tp_str = f"[{s.work_point[0]:>8.3f}, {s.work_point[1]:>7.3f}, {s.work_point[2]:>7.3f}]"
            # TargetInfo: circle_center (mm)
            cc_str = f"[{s.circle_center_base_mm[0]:>8.2f}, {s.circle_center_base_mm[1]:>8.2f}, {s.circle_center_base_mm[2]:>8.2f}]"
            # TargetInfo: spray_center (mm)
            sc_str = f"[{s.spray_center_base_mm[0]:>8.2f}, {s.spray_center_base_mm[1]:>8.2f}, {s.spray_center_base_mm[2]:>8.2f}]"
            print(f"  {s.target_id:<4} │ {tp_str:<32} │ {cc_str:<36} │ {sc_str:<32}")
        
        # 打印附加信息 (只打印一次，因为圆柱参数相同)
        print("\n  TargetInfo 圆柱参数 (所有切片共用):")
        print(f"    cylinder_axis: [0.000, 0.000, 1.000]")
        print(f"    Radius: {result.cylinder.radius_m * 1000:.3f} mm")
        print(f"    Angle: 90.000 deg")
        print(f"    Height: {result.cylinder.height_m * 1000:.3f} mm")
    else:
        print("  无有效数据")
    
    print("\n" + "=" * 70)


def compare_results(results: List[ParseResult]):
    """比较多个解析结果"""
    if len(results) < 2:
        print("需要至少两个结果进行比较")
        return
    
    print("\n" + "=" * 90)
    print("📊 多结果对比分析")
    print("=" * 90)
    
    # 表头
    headers = ["参数"] + [r.filename for r in results]
    col_width = max(20, max(len(h) for h in headers) + 2)
    
    def print_row(label: str, values: list):
        row = f"  {label:<30}"
        for v in values:
            row += f" {str(v):<{col_width}}"
        print(row)
    
    def print_separator():
        print("  " + "-" * (30 + col_width * len(results)))
    
    # 圆柱检测结果对比
    print("\n【圆柱体检测结果对比】")
    print_separator()
    print_row("检测状态", [r.cylinder.status for r in results])
    print_row("内点数", [r.cylinder.inliers for r in results])
    print_row("半径 (mm)", [f"{r.cylinder.radius_m * 1000:.3f}" for r in results])
    print_row("高度 (mm)", [f"{r.cylinder.height_m * 1000:.2f}" for r in results])
    print_row("圆度", [f"{r.cylinder.circularity:.4f}" for r in results])
    print_row("长宽比", [f"{r.cylinder.aspect_ratio:.2f}" for r in results])
    
    # 切片参数对比
    print("\n【切片参数对比】")
    print_separator()
    print_row("切片宽度 (mm)", [f"{r.slicing.slice_width_mm:.2f}" for r in results])
    print_row("切片高度 (mm)", [f"{r.slicing.slice_height_mm:.2f}" for r in results])
    print_row("总切片数", [r.slicing.total_slices for r in results])
    print_row("高度层数", [r.slicing.height_layers for r in results])
    print_row("宽度份数", [r.slicing.width_divisions for r in results])
    print_row("总覆盖面积 (m²)", [f"{r.slicing.total_area_m2:.4f}" for r in results])
    
    # 工作参数对比
    print("\n【工作参数对比】")
    print_separator()
    print_row("喷涂距离 (mm)", [f"{r.work.spray_distance_mm:.2f}" for r in results])
    print_row("工作表面距离 (mm)", [f"{r.work.work_surface_mm:.2f}" for r in results])
    print_row("机器人臂长 (mm)", [f"{r.work.robot_arm_reach_mm:.2f}" for r in results])
    
    # 优化参数对比
    print("\n【优化参数对比】")
    print_separator()
    print_row("最大覆盖角度 (deg)", [f"{r.optimization.max_coverage_angle_deg:.2f}" for r in results])
    print_row("最优切片宽度 (deg)", [f"{r.optimization.optimal_slice_width_deg:.2f}" for r in results])
    print_row("基座位置数 (N)", [r.optimization.robot_base_positions for r in results])
    print_row("每位置喷涂次数 (k)", [r.optimization.spray_passes_per_position for r in results])
    print_row("调整后重叠率 (%)", [f"{r.optimization.adjusted_overlap_ratio * 100:.2f}" for r in results])
    
    # 切片统计对比
    print("\n【切片统计对比】")
    print_separator()
    print_row("有效切片数", [r.valid_slices for r in results])
    print_row("跳过切片数", [r.skipped_slices for r in results])
    print_row("有效层数", [r.valid_layers for r in results])
    print_row("最大工作Z (mm)", [f"{r.max_working_z_mm:.2f}" for r in results])
    
    valid_ratios = []
    for r in results:
        if r.slicing.total_slices > 0:
            valid_ratios.append(f"{r.valid_slices / r.slicing.total_slices * 100:.1f}%")
        else:
            valid_ratios.append("N/A")
    print_row("有效率", valid_ratios)
    
    print("\n" + "=" * 90)


def visualize_3d(results: List[ParseResult], show_all: bool = True):
    """
    3D可视化圆柱体、切片点、坐标原点、虚拟机械臂等
    参考 cal.py 风格实现
    
    Args:
        results: 解析结果列表
        show_all: 是否显示所有结果（多个子图）
    """
    if not HAS_MATPLOTLIB:
        print("错误: matplotlib 未安装，无法进行可视化")
        return
    
    import matplotlib.patches as mpatches
    
    n_results = len(results)
    if n_results == 0:
        print("错误: 没有可视化的结果")
        return
    
    # 创建图形
    if n_results == 1:
        fig = plt.figure(figsize=(16, 12))
        axes = [fig.add_subplot(111, projection='3d')]
    else:
        cols = min(2, n_results)
        rows = (n_results + cols - 1) // cols
        fig = plt.figure(figsize=(16 * cols, 12 * rows))
        axes = [fig.add_subplot(rows, cols, i+1, projection='3d') for i in range(n_results)]
    
    colors = ['cyan', 'lime', 'orange', 'purple', 'pink', 'yellow']
    
    for idx, (result, ax) in enumerate(zip(results, axes)):
        color = colors[idx % len(colors)]
        _visualize_single_result_cal_style(ax, result, color, idx)
        # 添加鼠标滚轮缩放功能
        _add_zoom_support(ax, fig)
    
    plt.tight_layout()
    
    # 显示交互提示
    print("\n📌 3D可视化交互提示:")
    print("  - 鼠标左键拖动: 旋转视角")
    print("  - 鼠标滚轮: 放大/缩小")
    print("  - 鼠标右键拖动: 平移视角")
    print("  - 关闭窗口退出")
    
    plt.show()


def _add_zoom_support(ax, fig):
    """为3D图形添加鼠标滚轮缩放支持"""
    def on_scroll(event):
        if event.inaxes != ax:
            return
        # 获取当前缩放比例
        scale_factor = 1.15 if event.button == 'down' else 0.85
        
        # 获取当前坐标轴范围
        xlim = ax.get_xlim3d()
        ylim = ax.get_ylim3d()
        zlim = ax.get_zlim3d()
        
        # 计算中心点
        x_center = (xlim[0] + xlim[1]) / 2
        y_center = (ylim[0] + ylim[1]) / 2
        z_center = (zlim[0] + zlim[1]) / 2
        
        # 计算新的范围
        x_range = (xlim[1] - xlim[0]) * scale_factor / 2
        y_range = (ylim[1] - ylim[0]) * scale_factor / 2
        z_range = (zlim[1] - zlim[0]) * scale_factor / 2
        
        # 设置新的范围
        ax.set_xlim3d([x_center - x_range, x_center + x_range])
        ax.set_ylim3d([y_center - y_range, y_center + y_range])
        ax.set_zlim3d([z_center - z_range, z_center + z_range])
        
        fig.canvas.draw_idle()
    
    fig.canvas.mpl_connect('scroll_event', on_scroll)


def _visualize_single_result_cal_style(ax, result: ParseResult, cyl_color: str, idx: int):
    """
    按照 cal.py 风格可视化单个结果
    """
    import matplotlib.patches as mpatches
    
    cyl = result.cylinder
    work = result.work
    opt = result.optimization
    
    # 转换为 mm 单位（与 cal.py 保持一致）
    P_base = np.array([0.0, 0.0, 0.0])  # LiDAR原点
    P_cyl = np.array(cyl.bottom_center) * 1000  # 圆柱底面中心 (mm)
    P_cyl_geo = np.array(cyl.geo_center) * 1000  # 几何中心 (mm)
    P_cyl_top = np.array(cyl.top_center) * 1000  # 顶面中心 (mm)
    V_axis = np.array(cyl.axis)
    V_axis = V_axis / np.linalg.norm(V_axis) if np.linalg.norm(V_axis) > 0 else np.array([0, 0, 1])
    
    cylinder_radius = cyl.radius_m * 1000  # mm
    cylinder_height = cyl.height_m * 1000  # mm
    robot_max_reach = work.robot_arm_reach_mm  # mm
    R_task = cylinder_radius + work.spray_distance_mm  # 工作半径
    R_eff = work.effective_reach_mm  # 有效臂长
    
    # 虚拟基座位置
    vb = np.array(result.virtual_base_position) * 1000  # mm
    
    # ========== 1. 绘制原点 (LiDAR) ==========
    ax.scatter(*P_base, color='red', s=200, marker='o', label=f'LiDAR Origin (0,0,0)')
    
    # ========== 2. 绘制虚拟机器人基座 ==========
    if any(v != 0 for v in vb):
        ax.scatter(*vb, color='magenta', s=200, marker='^',
                   label=f'Virtual Robot Base ({vb[0]:.0f},{vb[1]:.0f},{vb[2]:.0f})')
    
    # ========== 3. 绘制圆柱中心点 ==========
    ax.scatter(*P_cyl_geo, color='blue', s=100, marker='D',
               label=f'Cylinder Center ({P_cyl_geo[0]:.0f},{P_cyl_geo[1]:.0f},{P_cyl_geo[2]:.0f})')
    
    # ========== 4. 绘制从原点到圆柱中心的连线 ==========
    dist_to_center = np.linalg.norm(P_cyl_geo - P_base)
    ax.plot([P_base[0], P_cyl_geo[0]], [P_base[1], P_cyl_geo[1]], [P_base[2], P_cyl_geo[2]],
            'b--', linewidth=2, label=f'Distance to Center={dist_to_center:.0f}mm')
    
    # ========== 5. 绘制圆柱表面最近点 ==========
    direction_to_origin = (P_base - P_cyl_geo)
    if np.linalg.norm(direction_to_origin) > 0:
        direction_to_origin = direction_to_origin / np.linalg.norm(direction_to_origin)
        surface_point = P_cyl_geo + cylinder_radius * direction_to_origin
        dist_to_surface = np.linalg.norm(surface_point - P_base)
        ax.scatter(*surface_point, color='green', s=100, marker='s',
                   label=f'Nearest Surface Point')
        ax.plot([P_base[0], surface_point[0]], [P_base[1], surface_point[1]], [P_base[2], surface_point[2]],
                'g-', linewidth=2, label=f'Surface Distance={dist_to_surface:.0f}mm')
    
    # ========== 6. 绘制圆柱体 ==========
    if cylinder_radius > 0 and cylinder_height > 0:
        _draw_cylinder_mm(ax, P_cyl, V_axis, cylinder_radius, cylinder_height, cyl_color, alpha=0.3)
    
    # ========== 7. 绘制机器人最大工作球形 (以虚拟基座为中心) ==========
    if robot_max_reach > 0 and any(v != 0 for v in vb):
        u = np.linspace(0, 2 * np.pi, 25)
        v_angle = np.linspace(0, np.pi, 15)
        X_sphere = vb[0] + robot_max_reach * np.outer(np.cos(u), np.sin(v_angle))
        Y_sphere = vb[1] + robot_max_reach * np.outer(np.sin(u), np.sin(v_angle))
        Z_sphere = vb[2] + robot_max_reach * np.outer(np.ones(np.size(u)), np.cos(v_angle))
        ax.plot_wireframe(X_sphere, Y_sphere, Z_sphere, color='red', alpha=0.1, linewidth=0.5)
    
    # ========== 8. 绘制有效臂长圆 (在工作高度平面) ==========
    if R_eff > 0 and any(v != 0 for v in vb):
        theta_circle = np.linspace(0, 2 * np.pi, 100)
        # 工作高度范围
        work_z_min = 100  # mm
        work_z_max = 700  # mm
        X_eff = vb[0] + R_eff * np.cos(theta_circle)
        Y_eff = vb[1] + R_eff * np.sin(theta_circle)
        Z_eff = np.full_like(theta_circle, vb[2] + work_z_max)
        ax.plot(X_eff, Y_eff, Z_eff, 'r-', linewidth=2,
                label=f'Effective Reach R_eff={R_eff:.0f}mm')
    
    # ========== 9. 绘制 R_task 圆 (圆柱半径+喷涂距离) ==========
    if R_task > 0:
        theta_circle = np.linspace(0, 2 * np.pi, 100)
        # 建立圆柱局部坐标系
        z_axis = V_axis
        if abs(z_axis[0]) < 0.9:
            temp = np.array([1, 0, 0])
        else:
            temp = np.array([0, 1, 0])
        x_axis = np.cross(z_axis, temp)
        x_axis = x_axis / np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)
        
        X_task = P_cyl_geo[0] + R_task * (np.cos(theta_circle) * x_axis[0] + np.sin(theta_circle) * y_axis[0])
        Y_task = P_cyl_geo[1] + R_task * (np.cos(theta_circle) * x_axis[1] + np.sin(theta_circle) * y_axis[1])
        Z_task = np.full_like(theta_circle, P_cyl_geo[2])
        ax.plot(X_task, Y_task, Z_task, 'orange', linewidth=2, linestyle='--',
                label=f'R_task={R_task:.0f}mm (R+spray_dist)')
    
    # ========== 10. 绘制垂直距离 D ==========
    if any(v != 0 for v in vb):
        # 找到从虚拟基座到圆柱轴的最近点
        t = np.dot(vb - P_cyl_geo, V_axis)
        closest_on_axis = P_cyl_geo + t * V_axis
        D = np.linalg.norm(vb - closest_on_axis)
        ax.plot([vb[0], closest_on_axis[0]], [vb[1], closest_on_axis[1]], [vb[2], closest_on_axis[2]],
                'purple', linewidth=2, linestyle=':', label=f'Perp Distance D={D:.0f}mm')
        ax.scatter(*closest_on_axis, color='purple', s=80, marker='x')
    
    # ========== 11. 绘制覆盖区域 (红色填充) ==========
    if opt.max_coverage_angle_deg > 0 and any(v != 0 for v in vb):
        alpha_rad = np.radians(opt.max_coverage_angle_deg / 2)
        
        # 找到从轴到虚拟基座的方向作为参考方向
        t = np.dot(vb - P_cyl_geo, V_axis)
        closest_on_axis = P_cyl_geo + t * V_axis
        to_base = vb - closest_on_axis
        to_base_horizontal = to_base - np.dot(to_base, V_axis) * V_axis
        if np.linalg.norm(to_base_horizontal) > 0:
            to_base_horizontal = to_base_horizontal / np.linalg.norm(to_base_horizontal)
        else:
            to_base_horizontal = x_axis if 'x_axis' in dir() else np.array([1, 0, 0])
        
        # 绘制覆盖区域表面
        theta_coverage = np.linspace(-alpha_rad, alpha_rad, 25)
        z_coverage = np.linspace(0, min(cylinder_height, 1500), 15)  # 限制可视化高度
        Theta_cov, Z_cov = np.meshgrid(theta_coverage, z_coverage)
        
        X_cov = np.zeros_like(Theta_cov)
        Y_cov = np.zeros_like(Theta_cov)
        Z_cov_plot = np.zeros_like(Theta_cov)
        
        for i in range(Theta_cov.shape[0]):
            for j in range(Theta_cov.shape[1]):
                ang = Theta_cov[i, j]
                z_val = Z_cov[i, j]
                cos_a, sin_a = np.cos(ang), np.sin(ang)
                rot_vec = cos_a * to_base_horizontal + sin_a * np.cross(V_axis, to_base_horizontal)
                point = P_cyl + cylinder_radius * rot_vec + z_val * V_axis
                X_cov[i, j] = point[0]
                Y_cov[i, j] = point[1]
                Z_cov_plot[i, j] = point[2]
        
        ax.plot_surface(X_cov, Y_cov, Z_cov_plot, alpha=0.5, color='red')
        
        # 绘制覆盖边界弧 (绿色)
        for z_offset in [0, cylinder_height/3, cylinder_height*2/3]:
            arc_x, arc_y, arc_z = [], [], []
            for ang in theta_coverage:
                cos_a, sin_a = np.cos(ang), np.sin(ang)
                rot_vec = cos_a * to_base_horizontal + sin_a * np.cross(V_axis, to_base_horizontal)
                point = P_cyl + cylinder_radius * rot_vec + z_offset * V_axis
                arc_x.append(point[0])
                arc_y.append(point[1])
                arc_z.append(point[2])
            ax.plot(arc_x, arc_y, arc_z, 'lime', linewidth=3, alpha=0.8)
    
    # ========== 12. 绘制有效喷涂点和工作点 ==========
    # spray_center: LiDAR 坐标系 (m) - 喷涂目标点(在圆柱表面)
    # work_point_lidar: LiDAR 坐标系 (m) - 机械臂工作点(在圆柱和原点之间)
    valid_slices = [s for s in result.slices if not s.skipped]
    if valid_slices:
        # 绘制喷涂点 (LiDAR坐标系, 黄色星形) - 在圆柱表面
        spray_points = np.array([s.spray_center for s in valid_slices]) * 1000  # 转为mm
        ax.scatter(spray_points[:, 0], spray_points[:, 1], spray_points[:, 2],
                   c='yellow', s=100, marker='*', edgecolors='black', linewidths=0.5,
                   label=f'Spray Points ({len(valid_slices)})')
        
        # 绘制工作点 (LiDAR坐标系, 青色圆点) - 在圆柱和原点之间
        work_points = np.array([s.work_point_lidar for s in valid_slices]) * 1000  # 转为mm
        ax.scatter(work_points[:, 0], work_points[:, 1], work_points[:, 2],
                   c='cyan', s=80, marker='o', edgecolors='black', linewidths=0.5,
                   label=f'Work Points ({len(valid_slices)})')
        
        # 添加序号标签 (在喷涂点上方)
        for i, s in enumerate(valid_slices):
            sp = np.array(s.spray_center) * 1000
            ax.text(sp[0], sp[1], sp[2] + 100, f'{s.target_id}', fontsize=7, ha='center', color='orange')
            # 在工作点上方也标注ID
            wp = np.array(s.work_point_lidar) * 1000
            ax.text(wp[0], wp[1], wp[2] + 100, f'{s.target_id}', fontsize=7, ha='center', color='cyan')

    # ========== 13. 绘制圆柱轴线 ==========
    ax.plot([P_cyl[0], P_cyl_top[0]], [P_cyl[1], P_cyl_top[1]], [P_cyl[2], P_cyl_top[2]],
            'm-', linewidth=3, label=f'Cylinder Axis (H={cylinder_height:.0f}mm)')
    
    # ========== 设置图形属性 ==========
    ax.set_xlabel('X (mm)', fontsize=12)
    ax.set_ylabel('Y (mm)', fontsize=12)
    ax.set_zlabel('Z (mm)', fontsize=12)
    
    title = f'Robot Working Range & Cylinder Visualization: {result.filename}\n'
    title += f'Cylinder: R={cylinder_radius:.0f}mm, H={cylinder_height:.0f}mm | '
    title += f'Coverage: {opt.max_coverage_angle_deg:.1f} deg | '
    title += f'Valid Slices: {result.valid_slices}/{result.slicing.total_slices}'
    ax.set_title(title, fontsize=12)
    
    # 设置坐标轴范围
    max_range = max(robot_max_reach if robot_max_reach > 0 else 2000,
                    abs(P_cyl_geo[0]), abs(P_cyl_geo[1]), abs(P_cyl_geo[2])) * 1.2
    ax.set_xlim([-max_range, max_range])
    ax.set_ylim([-max_range, max_range])
    ax.set_zlim([-max_range/4, max_range * 1.5])
    
    # 添加图例补丁
    cylinder_patch = mpatches.Patch(color=cyl_color, alpha=0.3, label='Cylinder Surface')
    sphere_patch = mpatches.Patch(color='red', alpha=0.1, label=f'Robot Max Range R={robot_max_reach:.0f}mm')
    arc_patch = mpatches.Patch(color='lime', alpha=0.8, label='Coverage Boundary')
    coverage_patch = mpatches.Patch(color='red', alpha=0.5, label=f'Coverage Area ({opt.max_coverage_angle_deg:.1f} deg)')
    
    handles, labels = ax.get_legend_handles_labels()
    handles.extend([cylinder_patch, sphere_patch, arc_patch, coverage_patch])
    
    ax.legend(handles=handles, loc='upper left', fontsize=8)
    
    # 设置默认视角：斜上方俯视，更清楚看到工作点对称分布
    ax.view_init(elev=45, azim=-60)


def _draw_cylinder_mm(ax, P_cyl: np.ndarray, V_axis: np.ndarray, radius: float, 
                      height: float, color: str, alpha: float = 0.3):
    """绘制圆柱体 (mm单位)"""
    # 建立局部坐标系
    z_axis = V_axis / np.linalg.norm(V_axis)
    if abs(z_axis[0]) < 0.9:
        temp = np.array([1, 0, 0])
    else:
        temp = np.array([0, 1, 0])
    x_axis = np.cross(z_axis, temp)
    x_axis = x_axis / np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    
    # 圆柱表面
    theta = np.linspace(0, 2 * np.pi, 50)
    z_cyl = np.linspace(0, height, 30)
    Theta, Z_cyl = np.meshgrid(theta, z_cyl)
    
    X_cyl = P_cyl[0] + radius * (np.cos(Theta) * x_axis[0] + np.sin(Theta) * y_axis[0]) + Z_cyl * z_axis[0]
    Y_cyl = P_cyl[1] + radius * (np.cos(Theta) * x_axis[1] + np.sin(Theta) * y_axis[1]) + Z_cyl * z_axis[1]
    Z_cyl_plot = P_cyl[2] + radius * (np.cos(Theta) * x_axis[2] + np.sin(Theta) * y_axis[2]) + Z_cyl * z_axis[2]
    
    ax.plot_surface(X_cyl, Y_cyl, Z_cyl_plot, alpha=alpha, color=color, edgecolor='none')
    
    # 绘制顶部和底部圆边
    circle_theta = np.linspace(0, 2 * np.pi, 100)
    
    # 底部圆
    x_bottom = P_cyl[0] + radius * (np.cos(circle_theta) * x_axis[0] + np.sin(circle_theta) * y_axis[0])
    y_bottom = P_cyl[1] + radius * (np.cos(circle_theta) * x_axis[1] + np.sin(circle_theta) * y_axis[1])
    z_bottom = P_cyl[2] + radius * (np.cos(circle_theta) * x_axis[2] + np.sin(circle_theta) * y_axis[2])
    ax.plot(x_bottom, y_bottom, z_bottom, color=color, linewidth=2)
    
    # 顶部圆
    P_top = P_cyl + height * z_axis
    x_top = P_top[0] + radius * (np.cos(circle_theta) * x_axis[0] + np.sin(circle_theta) * y_axis[0])
    y_top = P_top[1] + radius * (np.cos(circle_theta) * x_axis[1] + np.sin(circle_theta) * y_axis[1])
    z_top = P_top[2] + radius * (np.cos(circle_theta) * x_axis[2] + np.sin(circle_theta) * y_axis[2])
    ax.plot(x_top, y_top, z_top, color=color, linewidth=2)


def visualize_comparison(results: List[ParseResult]):
    """在同一个图中可视化多个结果的对比 (mm单位，cal.py风格)"""
    if not HAS_MATPLOTLIB:
        print("错误: matplotlib 未安装，无法进行可视化")
        return
    
    import matplotlib.patches as mpatches
    
    if len(results) < 1:
        print("错误: 没有可视化的结果")
        return
    
    fig = plt.figure(figsize=(16, 12))
    ax = fig.add_subplot(111, projection='3d')
    
    colors = ['cyan', 'lime', 'orange', 'purple', 'pink', 'yellow']
    
    # 绘制坐标原点 (LiDAR)
    ax.scatter([0], [0], [0], c='red', s=200, marker='o', label='LiDAR Origin (0,0,0)')
    
    max_range = 2000
    
    for idx, result in enumerate(results):
        color = colors[idx % len(colors)]
        cyl = result.cylinder
        work = result.work
        
        # 转换为mm单位
        P_cyl = np.array(cyl.bottom_center) * 1000
        P_cyl_geo = np.array(cyl.geo_center) * 1000
        V_axis = np.array(cyl.axis)
        V_axis = V_axis / np.linalg.norm(V_axis) if np.linalg.norm(V_axis) > 0 else np.array([0, 0, 1])
        cylinder_radius = cyl.radius_m * 1000
        cylinder_height = cyl.height_m * 1000
        
        # 绘制圆柱体
        if cylinder_radius > 0 and cylinder_height > 0:
            _draw_cylinder_mm(ax, P_cyl, V_axis, cylinder_radius, cylinder_height, color, alpha=0.2)
        
        # 绘制圆柱中心
        ax.scatter(*P_cyl_geo, color=color, s=100, marker='D',
                   label=f'{result.filename} Center')
        
        # 绘制喷涂点和工作点
        valid_slices = [s for s in result.slices if not s.skipped]
        if valid_slices:
            # 喷涂点 (LiDAR坐标系, 星形)
            spray_points = np.array([s.spray_center for s in valid_slices]) * 1000  # mm
            ax.scatter(spray_points[:, 0], spray_points[:, 1], spray_points[:, 2], 
                       c=color, s=80, marker='*', edgecolors='black', linewidths=0.5,
                       label=f'{result.filename} Spray Points ({len(valid_slices)})')
            # 工作点 (LiDAR坐标系, 圆点)
            work_points = np.array([s.work_point_lidar for s in valid_slices]) * 1000  # mm
            ax.scatter(work_points[:, 0], work_points[:, 1], work_points[:, 2], 
                       c=color, s=60, marker='o', edgecolors='white', linewidths=0.5,
                       alpha=0.7)
        
        # 绘制虚拟基座
        vb = np.array(result.virtual_base_position) * 1000  # mm
        if any(v != 0 for v in vb):
            ax.scatter(*vb, c=color, s=200, marker='^',
                       edgecolors='black', linewidths=1)
        
        # 更新范围
        max_range = max(max_range, abs(P_cyl_geo[0]), abs(P_cyl_geo[1]), abs(P_cyl_geo[2]),
                        work.robot_arm_reach_mm if work.robot_arm_reach_mm > 0 else 0)
    
    ax.set_xlabel('X (mm)', fontsize=12)
    ax.set_ylabel('Y (mm)', fontsize=12)
    ax.set_zlabel('Z (mm)', fontsize=12)
    ax.set_title('Multi-Result Comparison Visualization (mm)', fontsize=14)
    
    # 设置坐标范围
    max_range *= 1.2
    ax.set_xlim([-max_range, max_range])
    ax.set_ylim([-max_range, max_range])
    ax.set_zlim([-max_range/4, max_range * 1.5])
    
    ax.legend(loc='upper left', fontsize=9)
    
    # 设置默认视角：斜上方俯视，更清楚看到工作点对称分布
    ax.view_init(elev=45, azim=-60)
    
    # 添加鼠标滚轮缩放支持
    _add_zoom_support(ax, fig)
    
    plt.tight_layout()
    
    # 显示交互提示
    print("\n📌 3D可视化交互提示:")
    print("  - 鼠标左键拖动: 旋转视角")
    print("  - 鼠标滚轮: 放大/缩小")
    print("  - 鼠标右键拖动: 平移视角")
    print("  - 关闭窗口退出")
    
    plt.show()


def main():
    """主函数"""
    if len(sys.argv) < 2:
        print("用法: python statics.py <日志文件1> [日志文件2] ... [选项]")
        print("示例: python statics.py res1 res2")
        print("      python statics.py 12-22/res1 12-22/res2")
        print("      python statics.py 12-22/res1 --viz        # 单独可视化")
        print("      python statics.py 12-22/res1 12-22/res2 --viz  # 多个单独可视化")
        print("      python statics.py 12-22/res1 12-22/res2 --compare  # 对比可视化")
        print("\n选项:")
        print("  --viz      对每个结果单独进行3D可视化")
        print("  --compare  将多个结果在同一图中进行对比可视化")
        sys.exit(1)
    
    # 解析命令行参数
    viz_mode = None
    filepaths = []
    
    for arg in sys.argv[1:]:
        if arg == '--viz':
            viz_mode = 'single'
        elif arg == '--compare':
            viz_mode = 'compare'
        else:
            filepaths.append(arg)
    
    results = []
    for filepath in filepaths:
        try:
            result = parse_log_file(filepath)
            results.append(result)
            print_analysis_report(result)
        except FileNotFoundError:
            print(f"错误: 文件不存在 - {filepath}")
        except Exception as e:
            print(f"错误: 解析文件 {filepath} 失败 - {e}")
            import traceback
            traceback.print_exc()
    
    # 如果有多个结果，进行对比
    if len(results) >= 2:
        compare_results(results)
    
    # 可视化
    if viz_mode == 'single' and results:
        visualize_3d(results)
    elif viz_mode == 'compare' and results:
        visualize_comparison(results)


if __name__ == "__main__":
    main()
