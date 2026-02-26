#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
工字钢检测结果可视化验证脚本
用于3D可视化验证工字钢识别结果

使用方法:
1. 从日志文件解析: python3 output_verify.py --file log.txt
2. 从标准输入解析: python3 output_verify.py (然后粘贴日志内容，按Ctrl+D结束)
3. 手动指定参数: python3 output_verify.py --center="-1.96,4.59,1.87" --dimensions="3.92x0.17x0.43" --axis="0.999,-0.022,-0.028"
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import argparse
import re
import sys
import os


class IBeamVisualizer:
    """工字钢切片点可视化器"""
    
    def __init__(self):
        self.fig = None
        self.ax = None
    
    def draw_obb(self, center, dimensions, main_axis):
        """绘制工字钢OBB包围盒"""
        center = np.array(center)
        dimensions = np.array(dimensions)
        main_axis = np.array(main_axis)
        if np.linalg.norm(main_axis) > 1e-6:
            main_axis = main_axis / np.linalg.norm(main_axis)
        
        # 构建局部坐标系 (u, v, w)
        u = main_axis
        w_global = np.array([0, 0, 1])
        v = np.cross(w_global, u)
        if np.linalg.norm(v) < 1e-6:
            v = np.array([0, 1, 0])
        v = v / np.linalg.norm(v)
        w = np.cross(u, v)
        w = w / np.linalg.norm(w)
        
        # 计算8个顶点
        hl, hw, hh = dimensions / 2.0
        corners = []
        for dx in [-1, 1]:
            for dy in [-1, 1]:
                for dz in [-1, 1]:
                    pt = center + dx*hl*u + dy*hw*v + dz*hh*w
                    corners.append(pt)
        corners = np.array(corners)
        
        # 绘制12条边
        edges = [
            (0, 1), (2, 3), (4, 5), (6, 7), # Z方向
            (0, 2), (1, 3), (4, 6), (5, 7), # Y方向
            (0, 4), (1, 5), (2, 6), (3, 7)  # X方向
        ]
        for start, end in edges:
            self.ax.plot3D(*zip(corners[start], corners[end]), color='red', linestyle='--', linewidth=1, alpha=0.5)

    def visualize(self, surface_points, work_points, normals,
                  beam_center=None, beam_dimensions=None, beam_axis=None,
                  show_origin=True, title="I-Beam Slice Points Visualization"):
        """
        可视化工字钢的表面点、工作点及其关系
        
        Args:
            surface_points: 表面点列表 [[x,y,z], ...]
            work_points: 工作点列表 [[x,y,z], ...]
            normals: 法向量列表 [[x,y,z], ...]
            beam_center: 工字钢中心 [x,y,z]
            beam_dimensions: 工字钢尺寸 [l,w,h]
            beam_axis: 工字钢主轴 [x,y,z]
            show_origin: 是否显示原点 (机械臂基坐标系)
            title: 标题
        """
        self.fig = plt.figure(figsize=(14, 10))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        surface_points = np.array(surface_points)
        work_points = np.array(work_points)
        normals = np.array(normals)
        
        # 1. 绘制原点 (机械臂基坐标系)
        if show_origin:
            self.ax.scatter([0], [0], [0], c='yellow', s=200, marker='o', label='Origin (Base Frame)')
            
        # 2. 绘制表面点
        self.ax.scatter(surface_points[:, 0], surface_points[:, 1], surface_points[:, 2],
                       c='blue', s=50, marker='o', depthshade=True, label='Surface Points')
                       
        # 3. 绘制工作点
        self.ax.scatter(work_points[:, 0], work_points[:, 1], work_points[:, 2],
                       c='magenta', s=80, marker='X', depthshade=True, label='Work Points')
                       
        # 4. 绘制连接线和法向量
        for i in range(len(surface_points)):
            sp = surface_points[i]
            wp = work_points[i]
            n = normals[i]
            
            # 从表面点到工作点的连线
            self.ax.plot([sp[0], wp[0]], [sp[1], wp[1]], [sp[2], wp[2]], 
                        'cyan', linewidth=1.5, linestyle='-')
                        
            # 从表面点出发的法向量
            arrow_length = 0.5 # 0.5m 长的法向量箭头
            self.ax.quiver(sp[0], sp[1], sp[2],
                          n[0] * arrow_length, 
                          n[1] * arrow_length, 
                          n[2] * arrow_length,
                          color='lime', arrow_length_ratio=0.3, linewidth=2,
                          label='Normal' if i == 0 else "") # 只为第一个添加图例

        # 5. 绘制工字钢包围盒
        if beam_center is not None and beam_dimensions is not None and beam_axis is not None:
            self.draw_obb(beam_center, beam_dimensions, beam_axis)
            self.ax.scatter(*beam_center, c='red', marker='s', s=50, label='Beam Center')

        # 6. 设置坐标轴范围和标签
        all_points = np.vstack([surface_points, work_points])
        min_coords = all_points.min(axis=0)
        max_coords = all_points.max(axis=0)
        center = (min_coords + max_coords) / 2
        max_range = (max_coords - min_coords).max() * 0.7 + 0.5 # 保证有足够空间
        
        self.ax.set_xlabel('X (m)', fontsize=12)
        self.ax.set_ylabel('Y (m)', fontsize=12)
        self.ax.set_zlabel('Z (m)', fontsize=12)
        
        self.ax.set_xlim([center[0] - max_range, center[0] + max_range])
        self.ax.set_ylim([center[1] - max_range, center[1] + max_range])
        self.ax.set_zlim([center[2] - max_range, center[2] + max_range])
        
        # 7. 添加标题和图例
        self.ax.set_title(title, fontsize=14, fontweight='bold')
        self.ax.legend(loc='upper right')
        
        # 设置视角
        self.ax.view_init(elev=25, azim=60)
        
        plt.tight_layout()
        return self.fig, self.ax
    
    def show(self):
        """显示图形"""
        plt.show()
    
    def save(self, filename):
        """保存图形"""
        if self.fig:
            self.fig.savefig(filename, dpi=150, bbox_inches='tight')
            print(f"Figure saved to {filename}")


def parse_log_content(log_content):
    """
    从日志内容中解析工字钢切片点信息
    
    支持的日志格式:
    - Published target info for slice 0: surface_point=(-3.23, 7.92, 4.33) m,work_point=(-3.25, 6.82, 4.33) m,normal=(-0.02, -1.00, 0.00)
    
    Returns:
        dict: {'surface_points': [...], 'work_points': [...], 'normals': [...]}
    """
    result = {
        'surface_points': [],
        'work_points': [],
        'normals': [],
        'beam_center': None,
        'beam_dimensions': None,
        'beam_axis': None
    }
    
    # 解析所有切片数据 (在 Base frame)
    target_info_matches = re.findall(r'Published target info for slice \d+:\s*surface_point=\(([^)]+)\)\s*m,work_point=\(([^)]+)\)\s*m,normal=\(([^)]+)\)', log_content)
    
    for sp_str, wp_str, n_str in target_info_matches:
        try:
            sp = [float(x.strip()) for x in sp_str.split(',')]
            wp = [float(x.strip()) for x in wp_str.split(',')]
            n = [float(x.strip()) for x in n_str.split(',')]
            if len(sp) == 3 and len(wp) == 3 and len(n) == 3:
                result['surface_points'].append(sp)
                result['work_points'].append(wp)
                result['normals'].append(n)
        except ValueError:
            continue
            
    # 解析工字钢本体信息
    # Dimensions (LxWxH): 3.92 x 0.17 x 0.42 m
    dim_match = re.search(r'Dimensions \(LxWxH\): ([0-9.]+) x ([0-9.]+) x ([0-9.]+) m', log_content)
    if dim_match:
        result['beam_dimensions'] = [float(dim_match.group(1)), float(dim_match.group(2)), float(dim_match.group(3))]

    # 优先解析 Base Frame 下的中心点 (IBeam center: (-1.41, 7.88, 4.26) m in base frame)
    pos_match = re.search(r'IBeam center: \(([^)]+)\) m in base frame', log_content)
    
    if not pos_match:
        # Position=(-2.00, 4.59, 1.86) 或 surface_point: (-2.00, 4.59, 1.86) m
        pos_match = re.search(r'Position=\(([^)]+)\)', log_content)
    if not pos_match:
        pos_match = re.search(r'surface_point: \(([^)]+)\) m', log_content)
    if pos_match:
        result['beam_center'] = [float(x.strip()) for x in pos_match.group(1).split(',')]

    # plane_rx: (0.999, -0.019, -0.029)
    axis_match = re.search(r'plane_rx: \(([^)]+)\)', log_content)
    if axis_match:
        result['beam_axis'] = [float(x.strip()) for x in axis_match.group(1).split(',')]
    
    return result


def read_log_file(filepath):
    """读取日志文件内容"""
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"Log file not found: {filepath}")
    with open(filepath, 'r', encoding='utf-8') as f:
        return f.read()


def read_stdin():
    """从标准输入读取内容"""
    print("请粘贴日志内容，然后按 Ctrl+D (Linux/Mac) 或 Ctrl+Z (Windows) 结束输入:")
    return sys.stdin.read()


def main():
    parser = argparse.ArgumentParser(
        description='I-Beam Slice Points Visualization Tool',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用示例:
  1. 从日志文件解析:
     python3 output_verify.py --file log.txt
     
  2. 从标准输入解析 (粘贴日志后按Ctrl+D):
     python3 output_verify.py --stdin
        """)
    
    # 输入方式
    input_group = parser.add_mutually_exclusive_group(required=True)
    input_group.add_argument('--file', '-f', type=str, default=None,
                            help='从日志文件解析参数')
    input_group.add_argument('--stdin', '-i', action='store_true',
                            help='从标准输入读取日志内容')
    
    # 显示选项
    parser.add_argument('--no-origin', action='store_true',
                        help='Do not show origin point (Base Frame)')
    parser.add_argument('--save', type=str, default=None,
                        help='Save figure to file')
    parser.add_argument('--title', type=str, default='I-Beam Slice Points Visualization',
                        help='Figure title')
    
    args = parser.parse_args()
    
    parsed_info = {}
    
    if args.file:
        try:
            log_content = read_log_file(args.file)
            parsed_info = parse_log_content(log_content)
            print(f"[INFO] 从文件 '{args.file}' 解析日志")
        except Exception as e:
            print(f"[ERROR] 读取文件失败: {e}")
            return
    elif args.stdin:
        try:
            log_content = read_stdin()
            parsed_info = parse_log_content(log_content)
            print("[INFO] 从标准输入解析日志")
        except Exception as e:
            print(f"[ERROR] 解析输入失败: {e}")
            return
    
    # 获取解析数据
    surface_points = parsed_info.get('surface_points')
    work_points = parsed_info.get('work_points')
    normals = parsed_info.get('normals')
    beam_center = parsed_info.get('beam_center')
    beam_dimensions = parsed_info.get('beam_dimensions')
    beam_axis = parsed_info.get('beam_axis')
    
    # 检查必要数据
    if not surface_points or not work_points or not normals:
        print("[ERROR] 日志文件不包含必需的切片数据 (surface_point, work_point, normal)。")
        return

    # 打印信息
    print("=" * 60)
    print("I-Beam Slice Visualization Parameters (Base Frame):")
    print(f"  找到 {len(surface_points)} 个表面点")
    print(f"  找到 {len(work_points)} 个工作点")
    print(f"  找到 {len(normals)} 个法向量")
    if beam_center is not None:
        print(f"  工字钢中心: {beam_center}")
        print(f"  工字钢尺寸: {beam_dimensions}")
    print("=" * 60)
    
    # 创建可视化
    visualizer = IBeamVisualizer()
    visualizer.visualize(
        surface_points=surface_points,
        work_points=work_points,
        normals=normals,
        beam_center=beam_center,
        beam_dimensions=beam_dimensions,
        beam_axis=beam_axis,
        show_origin=not args.no_origin,
        title=args.title
    )
    
    # 保存或显示
    if args.save:
        visualizer.save(args.save)
    
    visualizer.show()


if __name__ == '__main__':
    main()
