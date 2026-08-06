#!/usr/bin/env python3
"""从 ROS1 bag 文件提取 PointCloud2 消息为 PCD 文件"""

import struct
import sys
from pathlib import Path

import numpy as np

from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore


def pointcloud2_to_xyz(msg, typestore):
    """将 PointCloud2 消息解析为 numpy Nx3 xyz 数组"""
    # 解析 fields
    fields = {}
    for f in msg.fields:
        fields[f.name] = (f.offset, f.datatype)

    point_step = msg.point_step
    data = bytes(msg.data)
    n_points = msg.width * msg.height

    # datatype: 7 = FLOAT32
    xyz = np.zeros((n_points, 3), dtype=np.float32)
    for i in range(n_points):
        base = i * point_step
        x = struct.unpack_from('<f', data, base + fields['x'][0])[0]
        y = struct.unpack_from('<f', data, base + fields['y'][0])[0]
        z = struct.unpack_from('<f', data, base + fields['z'][0])[0]
        xyz[i] = [x, y, z]

    # 过滤 NaN 和 inf
    valid = np.isfinite(xyz).all(axis=1)
    return xyz[valid]


def save_pcd(filepath: Path, points: np.ndarray):
    """保存为 ASCII PCD 文件"""
    n = len(points)
    header = f"""# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH {n}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {n}
DATA ascii
"""
    with open(filepath, 'w') as f:
        f.write(header)
        for p in points:
            f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")


def main():
    bag_path = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(
        '/home/oneko/projects/ros_airy_work/ros_bag/2026-03-10-14-25-44.bag')

    # 输出目录
    out_dir = bag_path.parent / bag_path.stem
    out_dir.mkdir(exist_ok=True)

    # 提取间隔：每隔 N 帧提取一帧，避免过多文件
    step = int(sys.argv[2]) if len(sys.argv) > 2 else 10
    max_frames = int(sys.argv[3]) if len(sys.argv) > 3 else 20

    typestore = get_typestore(Stores.ROS1_NOETIC)

    with Reader(bag_path) as reader:
        count = 0
        saved = 0
        for conn, timestamp, rawdata in reader.messages():
            if conn.topic != '/front_arm_lidar':
                continue
            count += 1
            if count % step != 0:
                continue

            msg = typestore.deserialize_ros1(rawdata, conn.msgtype)
            xyz = pointcloud2_to_xyz(msg, typestore)

            out_file = out_dir / f"frame_{count:04d}.pcd"
            save_pcd(out_file, xyz)
            saved += 1
            print(f"[{saved}/{max_frames}] Saved {out_file.name}: {len(xyz)} points")

            if saved >= max_frames:
                break

    print(f"\n提取完成: {saved} 帧 PCD 保存到 {out_dir}")
    print(f"用法: python cylinder_yaml_den.py -f {out_dir}/frame_0010.pcd -v --tilt-angle 45")


if __name__ == '__main__':
    main()
