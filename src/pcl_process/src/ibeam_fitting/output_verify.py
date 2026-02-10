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
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import argparse
import re
import sys
import os


class IBeamVisualizer:
    """工字钢可视化器"""
    
    def __init__(self):
        self.fig = None
        self.ax = None
        
    def create_obb_vertices(self, center, dimensions, main_axis):
        """
        创建OBB边界框的8个顶点
        
        Args:
            center: 中心点 [x, y, z]
            dimensions: 尺寸 [length, width, height]
            main_axis: 主轴方向 [x, y, z]
        
        Returns:
            8x3 的顶点数组
        """
        center = np.array(center)
        dimensions = np.array(dimensions)
        main_axis = np.array(main_axis)
        
        # 归一化主轴
        u = main_axis / np.linalg.norm(main_axis)
        
        # 构建局部坐标系
        up = np.array([0, 0, 1])  # 垂直向上
        right = np.cross(up, u)
        if np.linalg.norm(right) < 1e-6:
            right = np.array([1, 0, 0])
        right = right / np.linalg.norm(right)
        new_up = np.cross(u, right)
        new_up = new_up / np.linalg.norm(new_up)
        
        # 半尺寸
        half_l = dimensions[0] / 2.0  # 长度方向
        half_w = dimensions[1] / 2.0  # 宽度方向
        half_h = dimensions[2] / 2.0  # 高度方向
        
        # 8个顶点
        vertices = np.array([
            center - half_l * u - half_w * right - half_h * new_up,
            center + half_l * u - half_w * right - half_h * new_up,
            center + half_l * u + half_w * right - half_h * new_up,
            center - half_l * u + half_w * right - half_h * new_up,
            center - half_l * u - half_w * right + half_h * new_up,
            center + half_l * u - half_w * right + half_h * new_up,
            center + half_l * u + half_w * right + half_h * new_up,
            center - half_l * u + half_w * right + half_h * new_up,
        ])
        
        return vertices
    
    def create_ibeam_section(self, center, dimensions, main_axis, num_sections=20):
        """
        创建工字钢截面形状的3D点
        
        Args:
            center: 中心点
            dimensions: 尺寸 [length, width, height]
            main_axis: 主轴方向
            num_sections: 沿长度方向的截面数
        
        Returns:
            点云数组
        """
        center = np.array(center)
        dimensions = np.array(dimensions)
        main_axis = np.array(main_axis)
        
        # 归一化主轴
        u = main_axis / np.linalg.norm(main_axis)
        
        # 构建局部坐标系
        up = np.array([0, 0, 1])
        right = np.cross(up, u)
        if np.linalg.norm(right) < 1e-6:
            right = np.array([1, 0, 0])
        right = right / np.linalg.norm(right)
        new_up = np.cross(u, right)
        new_up = new_up / np.linalg.norm(new_up)
        
        length = dimensions[0]
        width = dimensions[1]
        height = dimensions[2]
        
        # 工字钢截面参数（相对尺寸）
        flange_thickness = height * 0.15  # 翼缘厚度
        web_thickness = width * 0.3       # 腹板厚度
        
        points = []
        
        # 沿长度方向生成截面
        for i in range(num_sections + 1):
            t = -length / 2 + i * length / num_sections
            base = center + t * u
            
            # 上翼缘
            for w in np.linspace(-width/2, width/2, 10):
                for h in np.linspace(height/2 - flange_thickness, height/2, 3):
                    p = base + w * right + h * new_up
                    points.append(p)
            
            # 下翼缘
            for w in np.linspace(-width/2, width/2, 10):
                for h in np.linspace(-height/2, -height/2 + flange_thickness, 3):
                    p = base + w * right + h * new_up
                    points.append(p)
            
            # 腹板
            for w in np.linspace(-web_thickness/2, web_thickness/2, 3):
                for h in np.linspace(-height/2 + flange_thickness, height/2 - flange_thickness, 10):
                    p = base + w * right + h * new_up
                    points.append(p)
        
        return np.array(points)
    
    def visualize(self, center, dimensions, main_axis, 
                  show_obb=True, show_section=True, show_origin=True,
                  title="I-Beam Visualization"):
        """
        可视化工字钢
        
        Args:
            center: 中心点 [x, y, z]
            dimensions: 尺寸 [length, width, height]
            main_axis: 主轴方向 [x, y, z]
            show_obb: 是否显示OBB边界框
            show_section: 是否显示工字钢截面形状
            show_origin: 是否显示原点（雷达位置）
            title: 标题
        """
        self.fig = plt.figure(figsize=(14, 10))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        center = np.array(center)
        dimensions = np.array(dimensions)
        main_axis = np.array(main_axis)
        
        # 1. 绘制原点（雷达位置）
        if show_origin:
            self.ax.scatter([0], [0], [0], c='yellow', s=200, marker='o', label='Origin (LiDAR)')
            # 从原点到工字钢中心的连线
            self.ax.plot([0, center[0]], [0, center[1]], [0, center[2]], 
                        'purple', linewidth=2, linestyle='--', label='Distance to origin')
        
        # 2. 绘制工字钢中心点
        self.ax.scatter([center[0]], [center[1]], [center[2]], 
                       c='red', s=100, marker='*', label='I-Beam Center')
        
        # 3. 绘制主轴方向箭头
        arrow_length = dimensions[0] * 0.3
        end_point = center + main_axis / np.linalg.norm(main_axis) * arrow_length
        self.ax.quiver(center[0], center[1], center[2],
                      end_point[0] - center[0], 
                      end_point[1] - center[1], 
                      end_point[2] - center[2],
                      color='green', arrow_length_ratio=0.2, linewidth=3,
                      label='Main Axis')
        
        # 4. 绘制OBB边界框
        if show_obb:
            vertices = self.create_obb_vertices(center, dimensions, main_axis)
            
            # 定义6个面
            faces = [
                [vertices[0], vertices[1], vertices[2], vertices[3]],  # 底面
                [vertices[4], vertices[5], vertices[6], vertices[7]],  # 顶面
                [vertices[0], vertices[1], vertices[5], vertices[4]],  # 前面
                [vertices[2], vertices[3], vertices[7], vertices[6]],  # 后面
                [vertices[0], vertices[3], vertices[7], vertices[4]],  # 左面
                [vertices[1], vertices[2], vertices[6], vertices[5]],  # 右面
            ]
            
            # 绘制半透明面
            collection = Poly3DCollection(faces, alpha=0.15, facecolor='cyan', edgecolor='red', linewidth=2)
            self.ax.add_collection3d(collection)
            
            # 绘制边框线（更明显）
            edges = [
                (0, 1), (1, 2), (2, 3), (3, 0),  # 底面
                (4, 5), (5, 6), (6, 7), (7, 4),  # 顶面
                (0, 4), (1, 5), (2, 6), (3, 7),  # 侧边
            ]
            for e in edges:
                self.ax.plot3D([vertices[e[0]][0], vertices[e[1]][0]],
                              [vertices[e[0]][1], vertices[e[1]][1]],
                              [vertices[e[0]][2], vertices[e[1]][2]],
                              'r-', linewidth=2)
        
        # 5. 绘制工字钢截面形状点云
        if show_section:
            section_points = self.create_ibeam_section(center, dimensions, main_axis)
            self.ax.scatter(section_points[:, 0], section_points[:, 1], section_points[:, 2],
                           c='blue', s=1, alpha=0.5, label='I-Beam Section')
        
        # 6. 设置坐标轴
        self.ax.set_xlabel('X (m)', fontsize=12)
        self.ax.set_ylabel('Y (m)', fontsize=12)
        self.ax.set_zlabel('Z (m)', fontsize=12)
        
        # 设置等比例显示
        max_range = max(dimensions) * 1.5
        self.ax.set_xlim([center[0] - max_range, center[0] + max_range])
        self.ax.set_ylim([center[1] - max_range, center[1] + max_range])
        self.ax.set_zlim([center[2] - max_range, center[2] + max_range])
        
        # 7. 添加信息文本
        info_text = (
            f"Center: ({center[0]:.2f}, {center[1]:.2f}, {center[2]:.2f}) m\n"
            f"Dimensions (L×W×H): {dimensions[0]:.2f} × {dimensions[1]:.2f} × {dimensions[2]:.2f} m\n"
            f"Main Axis: ({main_axis[0]:.3f}, {main_axis[1]:.3f}, {main_axis[2]:.3f})\n"
            f"Distance from origin: {np.linalg.norm(center):.2f} m"
        )
        self.ax.text2D(0.02, 0.98, info_text, transform=self.ax.transAxes, 
                      fontsize=10, verticalalignment='top',
                      bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        self.ax.set_title(title, fontsize=14, fontweight='bold')
        self.ax.legend(loc='upper right')
        
        # 设置视角
        self.ax.view_init(elev=20, azim=-60)
        
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


def parse_dimensions(dim_str):
    """解析尺寸字符串 '3.92x0.17x0.43' 或 '3.92*0.17*0.43'"""
    dim_str = dim_str.lower().replace('m', '').strip()
    # 支持 x 和 * 作为分隔符
    if 'x' in dim_str:
        parts = dim_str.split('x')
    elif '*' in dim_str:
        parts = dim_str.split('*')
    else:
        raise ValueError(f"Invalid dimensions format: {dim_str}")
    if len(parts) != 3:
        raise ValueError(f"Invalid dimensions format: {dim_str}")
    return [float(p) for p in parts]


def parse_vector(vec_str):
    """解析向量字符串 '(-1.96, 4.59, 1.87)' 或 '0.999,-0.022,-0.028'"""
    vec_str = vec_str.strip('()[] ')
    parts = vec_str.split(',')
    if len(parts) != 3:
        raise ValueError(f"Invalid vector format: {vec_str}")
    return [float(p) for p in parts]


def parse_log_content(log_content):
    """
    从日志内容中解析工字钢检测结果
    
    支持的日志格式:
    - surface_point: (-1.96, 4.59, 1.87) m
    - Center=(-1.96, 4.59, 1.87)m
    - Dimensions=3.92x0.17x0.43m
    - plane_rx: (0.999, -0.022, -0.028)
    
    Returns:
        dict: {'center': [...], 'dimensions': [...], 'main_axis': [...], ...}
    """
    result = {
        'center': None,
        'dimensions': None,
        'main_axis': None,
        'beam_height': None,
        'beam_length_of_working': None,
        'wing_plate_width': None,
        'linearity': None,
        'aspect_ratio': None,
        'distance': None,
    }
    
    # 解析 surface_point 或 Center
    # 格式: surface_point: (-1.96, 4.59, 1.87) m
    # 或: Center=(-1.96, 4.59, 1.87)m
    surface_match = re.search(r'surface_point:\s*\(([^)]+)\)', log_content)
    center_match = re.search(r'Center=\(([^)]+)\)', log_content)
    if surface_match:
        result['center'] = [float(x.strip()) for x in surface_match.group(1).split(',')]
    elif center_match:
        result['center'] = [float(x.strip()) for x in center_match.group(1).split(',')]
    
    # 解析 Dimensions
    # 格式: Dimensions=3.92x0.17x0.43m (Source W=1.05m)
    # 或: Dimensions (LxWxH): 3.92 x 0.17 x 0.43 m
    dim_match = re.search(r'Dimensions[=:\s]+([0-9.]+)\s*[x*]\s*([0-9.]+)\s*[x*]\s*([0-9.]+)\s*m', log_content)
    if dim_match:
        result['dimensions'] = [float(dim_match.group(1)), 
                                float(dim_match.group(2)), 
                                float(dim_match.group(3))]
    
    # 解析 plane_rx (主轴方向)
    # 格式: plane_rx: (0.999, -0.022, -0.028)
    axis_match = re.search(r'plane_rx:\s*\(([^)]+)\)', log_content)
    if axis_match:
        result['main_axis'] = [float(x.strip()) for x in axis_match.group(1).split(',')]
    
    # 解析其他参数
    # beam_height
    height_match = re.search(r'beam_height\s*:\s*([0-9.]+)\s*mm', log_content)
    if height_match:
        result['beam_height'] = float(height_match.group(1))
    
    # beam_length_of_working
    working_match = re.search(r'beam_length_of_working\s*:\s*([0-9.]+)\s*mm', log_content)
    if working_match:
        result['beam_length_of_working'] = float(working_match.group(1))
    
    # wing_plate_width
    wing_match = re.search(r'wing_plate_width:\s*([0-9.]+)\s*mm', log_content)
    if wing_match:
        result['wing_plate_width'] = float(wing_match.group(1))
    
    # Linearity
    lin_match = re.search(r'Linearity[=:\s]+([0-9.]+)', log_content)
    if lin_match:
        result['linearity'] = float(lin_match.group(1))
    
    # AspectRatio
    ar_match = re.search(r'AspectRatio[=:\s]+([0-9.]+)', log_content)
    if ar_match:
        result['aspect_ratio'] = float(ar_match.group(1))
    
    # Distance
    dist_match = re.search(r'Distance[=:\s]+([0-9.]+)\s*m', log_content)
    if dist_match:
        result['distance'] = float(dist_match.group(1))
    
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
        description='I-Beam Detection Visualization Tool',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用示例:
  1. 从日志文件解析:
     python3 output_verify.py --file log.txt
     
  2. 从标准输入解析 (粘贴日志后按Ctrl+D):
     python3 output_verify.py --stdin
     
  3. 手动指定参数:
     python3 output_verify.py --center="-1.96,4.59,1.87" --dimensions="3.92x0.17x0.43" --axis="0.999,-0.022,-0.028"
        """)
    
    # 输入方式
    input_group = parser.add_mutually_exclusive_group()
    input_group.add_argument('--file', '-f', type=str, default=None,
                            help='从日志文件解析参数')
    input_group.add_argument('--stdin', '-i', action='store_true',
                            help='从标准输入读取日志内容')
    
    # 手动参数
    parser.add_argument('--center', type=str, default=None,
                        help='I-beam center position (x,y,z) in meters')
    parser.add_argument('--dimensions', type=str, default=None,
                        help='I-beam dimensions (LxWxH) in meters')
    parser.add_argument('--axis', type=str, default=None,
                        help='Main axis direction (x,y,z)')
    
    # 显示选项
    parser.add_argument('--no-obb', action='store_true',
                        help='Do not show OBB bounding box')
    parser.add_argument('--no-section', action='store_true',
                        help='Do not show I-beam section shape')
    parser.add_argument('--no-origin', action='store_true',
                        help='Do not show origin point')
    parser.add_argument('--save', type=str, default=None,
                        help='Save figure to file')
    parser.add_argument('--title', type=str, default='I-Beam Detection Verification',
                        help='Figure title')
    
    args = parser.parse_args()
    
    center = None
    dimensions = None
    main_axis = None
    parsed_info = {}
    
    # 优先从文件或标准输入解析
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
    
    # 从解析结果获取参数，命令行参数优先级更高
    if parsed_info.get('center'):
        center = parsed_info['center']
    if parsed_info.get('dimensions'):
        dimensions = parsed_info['dimensions']
    if parsed_info.get('main_axis'):
        main_axis = parsed_info['main_axis']
    
    # 手动指定的参数会覆盖解析结果
    try:
        if args.center:
            center = parse_vector(args.center)
        if args.dimensions:
            dimensions = parse_dimensions(args.dimensions)
        if args.axis:
            main_axis = parse_vector(args.axis)
    except ValueError as e:
        print(f"[ERROR] 解析参数失败: {e}")
        return
    
    # 检查必要参数
    if center is None:
        print("[ERROR] 缺少必要参数: center (中心点坐标)")
        print("请使用 --file 指定日志文件，或使用 --center 手动指定")
        return
    if dimensions is None:
        print("[ERROR] 缺少必要参数: dimensions (尺寸)")
        print("请使用 --file 指定日志文件，或使用 --dimensions 手动指定")
        return
    if main_axis is None:
        # 如果没有主轴信息，使用默认值
        main_axis = [1.0, 0.0, 0.0]
        print("[WARN] 未找到主轴信息，使用默认值 [1, 0, 0]")
    
    # 打印解析结果
    print("=" * 60)
    print("I-Beam Visualization Parameters:")
    print(f"  Center: ({center[0]:.2f}, {center[1]:.2f}, {center[2]:.2f}) m")
    print(f"  Dimensions (L×W×H): {dimensions[0]:.2f} × {dimensions[1]:.2f} × {dimensions[2]:.2f} m")
    print(f"  Main Axis: ({main_axis[0]:.3f}, {main_axis[1]:.3f}, {main_axis[2]:.3f})")
    print(f"  Distance from origin: {np.linalg.norm(center):.2f} m")
    
    # 打印额外解析信息
    if parsed_info:
        print("-" * 60)
        print("Additional Parsed Info:")
        if parsed_info.get('beam_height'):
            print(f"  beam_height: {parsed_info['beam_height']:.2f} mm")
        if parsed_info.get('beam_length_of_working'):
            print(f"  beam_length_of_working: {parsed_info['beam_length_of_working']:.2f} mm")
        if parsed_info.get('wing_plate_width'):
            print(f"  wing_plate_width: {parsed_info['wing_plate_width']:.2f} mm")
        if parsed_info.get('linearity'):
            print(f"  Linearity: {parsed_info['linearity']:.2f}")
        if parsed_info.get('aspect_ratio'):
            print(f"  Aspect Ratio: {parsed_info['aspect_ratio']:.2f}")
    print("=" * 60)
    
    # 创建可视化
    visualizer = IBeamVisualizer()
    visualizer.visualize(
        center=center,
        dimensions=dimensions,
        main_axis=main_axis,
        show_obb=not args.no_obb,
        show_section=not args.no_section,
        show_origin=not args.no_origin,
        title=args.title
    )
    
    # 保存或显示
    if args.save:
        visualizer.save(args.save)
    
    visualizer.show()


if __name__ == '__main__':
    main()
