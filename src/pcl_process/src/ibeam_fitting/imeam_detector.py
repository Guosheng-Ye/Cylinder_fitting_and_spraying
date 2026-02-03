import open3d as o3d
import numpy as np
import os
import struct
import sys
from datetime import datetime

# 尝试设置为 X11 后端(解决 Wayland 兼容性问题)
os.environ['XDG_SESSION_TYPE'] = 'x11'

# 固定随机种子,确保结果可重复
# np.random.seed(42)
# o3d.utility.random.seed(42)


def detect_ibeams_final(pcd_path, max_distance=5.0, min_distance=1.0, min_height=1.0):
    """
    检测点云中的工字钢结构(改进版)
    
    核心策略:
    1. 先移除背景墙面(使用 RANSAC 检测大平面)
    2. 只关注 min_distance < 距离 < max_distance 且悬空(Z > min_height)的物体
    3. 使用 PCA 分析形状特征,检测线状结构(高线性度)
    4. 工字钢特征:细长结构,主轴水平,高线性度(> 0.8)
    
    参考特征(基于已验证的工字钢):
    - 长度 > 1m,截面尺寸约 0.4 x 0.25 m
    - 线性度 > 0.9,长宽比 > 3
    - 主轴水平(Z分量 < 0.1)e
    
    Args:
        pcd_path: PCD 文件路径
        max_distance: 最大检测距离(米),过滤远处背景墙
        min_distance: 最小检测距离(米),过滤原点附近物体
        min_height: 最小高度(米),过滤地面物体
    """
    # 1. 加载点云
    print(f"正在分析点云: {pcd_path}")
    pcd = o3d.io.read_point_cloud(pcd_path)
    if not pcd.has_points():
        print("错误:无法读取点云文件。")
        return

    # 诊断:打印原始点云范围
    points_raw = np.asarray(pcd.points)
    print(f"\n=== 原始点云诊断 ===")
    print(f"总点数: {len(points_raw)}")
    print(f"X 范围: [{points_raw[:, 0].min():.2f}, {points_raw[:, 0].max():.2f}] m")
    print(f"Y 范围: [{points_raw[:, 1].min():.2f}, {points_raw[:, 1].max():.2f}] m")
    print(f"Z 范围: [{points_raw[:, 2].min():.2f}, {points_raw[:, 2].max():.2f}] m")

    # 2. 预处理:降采样与去噪
    pcd = pcd.voxel_down_sample(voxel_size=0.02)
    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.5)
    print(f"预处理后点数: {len(pcd.points)}")

    # 3. 移除背景墙面(RANSAC 检测大型垂直平面)
    print("\n=== 移除背景墙面 ===")
    pcd_filtered = remove_wall_planes(pcd, distance_threshold=0.08, min_plane_points=500)
    print(f"移除墙面后点数: {len(pcd_filtered.points)}")
    
    # 4. 距离和高度过滤:只保留指定范围内的悬空物体
    points = np.asarray(pcd_filtered.points)
    distances = np.linalg.norm(points[:, :2], axis=1)  # XY平面距离
    
    # 过滤条件:min_distance < 距离 < max_distance 且高度 > min_height
    spatial_mask = (distances > min_distance) & (distances < max_distance) & (points[:, 2] > min_height)
    indices = np.where(spatial_mask)[0]
    
    if len(indices) < 50:
        print(f"警告:符合条件的点云过少 ({len(indices)} 点),尝试放宽条件...")
        # 放宽条件
        spatial_mask = (distances > min_distance * 0.5) & (distances < max_distance * 1.5) & (points[:, 2] > min_height * 0.5)
        indices = np.where(spatial_mask)[0]
        if len(indices) < 50:
            print("警告:仍无足够点云,无法继续检测。")
            visualize_with_matplotlib(pcd, [pcd])
            return
    
    pcd_roi = pcd_filtered.select_by_index(indices)
    roi_points = np.asarray(pcd_roi.points)
    print(f"ROI区域 ({min_distance}m<距离<{max_distance}m, 高度>{min_height}m) 点数: {len(roi_points)}")

    # 5. 分层聚类策略:按高度分层,在每层内单独聚类
    print("\n=== 分层聚类分割 ===")
    
    # 定义高度层
    height_layers = [
        (1.5, 2.5, "低层"),   # 工字钢主要区域
        (2.5, 4.0, "中层"),
        (4.0, 7.0, "高层"),
    ]
    
    all_labels = np.full(len(roi_points), -1, dtype=int)
    current_label = 0
    
    for z_min, z_max, layer_name in height_layers:
        layer_mask = (roi_points[:, 2] >= z_min) & (roi_points[:, 2] < z_max)
        layer_indices = np.where(layer_mask)[0]
        
        if len(layer_indices) < 10:
            continue
        
        layer_pcd = pcd_roi.select_by_index(layer_indices)
        
        # 在该层内进行聚类（eps=0.18平衡过分割和欠分割）
        layer_labels = np.array(layer_pcd.cluster_dbscan(eps=0.18, min_points=10, print_progress=False))
        
        n_clusters = layer_labels.max() + 1
        if n_clusters > 0:
            print(f"  {layer_name} (Z: {z_min}-{z_max}m): {len(layer_indices)} 点, {n_clusters} 个聚类")
            
            # 将层内标签映射到全局标签
            for local_label in range(n_clusters):
                local_mask = layer_labels == local_label
                global_indices = layer_indices[local_mask]
                all_labels[global_indices] = current_label
                current_label += 1
    
    labels = all_labels
    max_label = labels.max()
    print(f"分层聚类共检测到 {max_label + 1} 个聚类")
    
    if max_label < 0:
        print("警告:未检测到任何聚类")
        visualize_with_matplotlib(pcd, [pcd])
        return

    # 5.5 多轮合并策略：先严格合并，再尝试扩展
    # 第一轮：严格合并（高置信度）
    labels = merge_ibeam_clusters(roi_points, labels, 
                                   max_gap=0.5,           # 较小间隔
                                   max_height_diff=0.10,  # 严格高度一致
                                   min_axis_similarity=0.97)  # 严格主轴对齐
    
    # 第二轮：中等条件合并（扩展短片段）
    labels = merge_ibeam_clusters(roi_points, labels, 
                                   max_gap=1.0,           # 允许更大间隔
                                   max_height_diff=0.15,  # 稍微放宽高度差
                                   min_axis_similarity=0.95)  # 稍微放宽主轴对齐
    
    max_label = labels.max()
    print(f"合并后剩余 {max_label + 1} 个聚类")

    # 6. 对每个聚类进行 PCA 形状分析
    geometries = [pcd]  # 背景显示原始点云
    found_ibeams = []

    print("\n=== 聚类形状分析 ===")
    for cluster_id in range(max_label + 1):
        cluster_mask = labels == cluster_id
        cluster_points = roi_points[cluster_mask]
        
        if len(cluster_points) < 30:
            continue
        
        # PCA 分析
        center = np.mean(cluster_points, axis=0)
        centered = cluster_points - center
        cov = np.cov(centered.T)
        eigenvalues, eigenvectors = np.linalg.eigh(cov)
        
        # 按特征值降序排列
        sorted_idx = np.argsort(eigenvalues)[::-1]
        eigenvalues = eigenvalues[sorted_idx]
        eigenvectors = eigenvectors[:, sorted_idx]
        
        # 主轴方向
        main_axis = eigenvectors[:, 0]
        
        # 计算沿主轴方向的尺寸
        proj = np.dot(centered, eigenvectors)
        dims = proj.max(axis=0) - proj.min(axis=0)
        length, width, thickness = dims[0], dims[1], dims[2]
        
        # 计算形状特征
        linearity = (eigenvalues[0] - eigenvalues[1]) / (eigenvalues[0] + 1e-6)
        planarity = (eigenvalues[1] - eigenvalues[2]) / (eigenvalues[0] + 1e-6)
        
        dist = np.sqrt(center[0]**2 + center[1]**2)
        aspect_ratio = length / max(width, 0.01)
        
        # 主轴水平性:Z分量越小越水平
        main_axis_z = abs(main_axis[2])
        is_horizontal = main_axis_z < 0.3  # 与水平面夹角 < 17°
        
        print(f"\n聚类 #{cluster_id}: {len(cluster_points)} 点")
        print(f"  中心: ({center[0]:.2f}, {center[1]:.2f}, {center[2]:.2f}), 距离={dist:.2f}m")
        print(f"  尺寸: {length:.2f} x {width:.2f} x {thickness:.2f} m")
        print(f"  主轴: ({main_axis[0]:.2f}, {main_axis[1]:.2f}, {main_axis[2]:.2f})")
        print(f"  形状: 线性度={linearity:.2f}, 长宽比={aspect_ratio:.1f}, 主轴水平性={1-main_axis_z:.2f}")
        
        # ===== 工字钢判断条件(基于已验证特征优化)=====
        # 参考标准:线性度 0.96,长宽比 4.69,长度 1.88m,主轴Z分量 0.024
        
        # 条件1:高线性度(核心特征)
        is_highly_linear = linearity > 0.8
        
        # 条件2:足够长度（放宽初筛阈值，依靠精细化检测过滤）
        is_long_enough = length > 1.5
        
        # 条件3:细长结构（初筛阈值较宽松，依靠精细化过滤）
        is_elongated = aspect_ratio > 2.5
        
        # 条件4:主轴水平(核心特征 - Z分量小)
        is_axis_horizontal = main_axis_z < 0.2
        
        # 条件5:截面尺寸合理(放宽范围,工字钢截面可能包含翼缘和腹板)
        cross_section_ok = 0.1 < width < 1.2 and 0.03 < thickness < 0.8
        
        # 综合判断:满足核心条件(线性度 + 长度 + 主轴水平)
        is_ibeam = is_highly_linear and is_long_enough and is_elongated and is_axis_horizontal and cross_section_ok
        
        # 创建 OBB 用于可视化
        cluster_indices = np.where(cluster_mask)[0]
        cluster_pcd = pcd_roi.select_by_index(cluster_indices)
        obb = cluster_pcd.get_oriented_bounding_box()
        
        if is_ibeam:
            # 对候选工字钢进行精细化处理
            print(f"  -> ✓ 候选工字钢,进行精细化识别...")
            refined_result = refine_ibeam_detection(cluster_points, main_axis, center)
            
            if refined_result is not None:
                refined_points, refined_center, refined_dims = refined_result
                refined_length, refined_width, refined_height = refined_dims
                
                # 创建精细化后的点云和 OBB
                refined_pcd = o3d.geometry.PointCloud()
                refined_pcd.points = o3d.utility.Vector3dVector(refined_points)
                refined_obb = refined_pcd.get_oriented_bounding_box()
                refined_obb.color = np.array([1.0, 0.0, 0.0])  # 红色
                geometries.append(refined_obb)
                
                # 添加原始 OBB 为半透明黄色(参考)
                obb.color = np.array([1.0, 1.0, 0.0])  # 黄色(原始区域)
                geometries.append(obb)
                
                found_ibeams.append({
                    'id': len(found_ibeams) + 1,
                    'center': refined_center,
                    'dimensions': (refined_length, refined_width, refined_height),
                    'main_axis': main_axis,
                    'linearity': linearity,
                    'aspect_ratio': refined_length / max(refined_width, 0.01),
                    'points': len(refined_points),
                    'original_points': len(cluster_points),
                    'cluster_points': cluster_points  # 保存原始聚类点用于后续扩展
                })
                print(f"       精细化完成: {len(cluster_points)} -> {len(refined_points)} 点")
                print(f"       精细尺寸: {refined_length:.2f} x {refined_width:.2f} x {refined_height:.2f} m")
            else:
                # 精细化失败,使用原始结果
                obb.color = np.array([1.0, 0.5, 0.0])  # 橙色(未精细化)
                geometries.append(obb)
                found_ibeams.append({
                    'id': len(found_ibeams) + 1,
                    'center': center,
                    'dimensions': (length, width, thickness),
                    'main_axis': main_axis,
                    'linearity': linearity,
                    'aspect_ratio': aspect_ratio,
                    'points': len(cluster_points),
                    'cluster_points': cluster_points  # 保存原始聚类点用于后续扩展
                })
                print(f"       精细化失败,使用原始结果")
        else:
            obb.color = np.array([0.3, 0.3, 1.0])  # 蓝色:未识别
            geometries.append(obb)
            reasons = []
            if not is_highly_linear:
                reasons.append(f"线性度不足({linearity:.2f}<0.8)")
            if not is_long_enough:
                reasons.append(f"长度不足({length:.2f}m<0.8m)")
            if not is_elongated:
                reasons.append(f"长宽比不足({aspect_ratio:.1f}<2.5)")
            if not is_axis_horizontal:
                reasons.append(f"主轴不水平(Z={main_axis_z:.2f}>0.2)")
            if not cross_section_ok:
                reasons.append(f"截面异常({width:.2f}x{thickness:.2f})")
            print(f"  -> ✗ 未识别: {', '.join(reasons)}")

    # 7. 输出结果
    if found_ibeams:
        print(f"\n{'='*50}")
        print(f"成功识别到 {len(found_ibeams)} 处工字钢:")
        for beam in found_ibeams:
            c = beam['center']
            d = beam['dimensions']
            orig_pts = beam.get('original_points', beam['points'])
            print(f"  工字钢 #{beam['id']}: 位置=({c[0]:.2f}, {c[1]:.2f}, {c[2]:.2f})")
            print(f"       尺寸: {d[0]:.2f} x {d[1]:.2f} x {d[2]:.2f} m")
            print(f"       点数: {beam['points']} (原始: {orig_pts})")
        
        # 7.5 后处理：对于长度不足的工字钢，尝试在ROI中扩展
        found_ibeams = extend_short_ibeams(found_ibeams, roi_points, min_target_length=3.5)
        
    else:
        print(f"\n{'='*50}")
        print("未识别到工字钢目标,显示所有聚类边界框供参考。")
    
    # 使用 matplotlib 进行可视化
    visualize_with_matplotlib(pcd, geometries, found_ibeams)
    
    return found_ibeams


def extend_short_ibeams(found_ibeams, roi_points, min_target_length=3.5):
    """
    扩展长度不足的工字钢检测结果
    
    通过在ROI点云中搜索沿主轴方向的相邻点来扩展短工字钢
    
    Args:
        found_ibeams: 已检测到的工字钢列表
        roi_points: ROI区域的所有点
        min_target_length: 目标最小长度
        
    Returns:
        扩展后的工字钢列表
    """
    if not found_ibeams:
        return found_ibeams
    
    extended_ibeams = []
    
    for beam in found_ibeams:
        length = beam['dimensions'][0]
        
        # 如果长度已经足够，不需要扩展
        if length >= min_target_length:
            extended_ibeams.append(beam)
            continue
        
        print(f"\n  尝试扩展工字钢 #{beam['id']} (当前长度={length:.2f}m)")
        
        # 获取工字钢的主轴和中心
        main_axis = beam['main_axis']
        center = beam['center']
        
        # 确保主轴方向一致
        if main_axis[0] < 0:
            main_axis = -main_axis
        
        # 构建局部坐标系
        u = main_axis / np.linalg.norm(main_axis)
        w = np.array([0, 0, 1])  # 垂直向上
        v = np.cross(w, u)
        v = v / np.linalg.norm(v)
        
        # 将ROI点转换到局部坐标系
        points_centered = roi_points - center
        local_u = np.dot(points_centered, u)  # 沿主轴
        local_v = np.dot(points_centered, v)  # 沿翼缘宽度
        local_w = np.dot(points_centered, w)  # 沿高度
        
        # 定义搜索范围：在工字钢截面附近的点
        # 使用当前检测到的宽度和高度来限制搜索
        width = beam['dimensions'][1]
        height = beam['dimensions'][2]
        
        v_tolerance = max(0.25, width * 0.8)
        w_tolerance = max(0.30, height * 0.8)
        
        # 搜索沿主轴方向延伸的点
        cross_section_mask = (np.abs(local_v) < v_tolerance) & (np.abs(local_w) < w_tolerance)
        
        # 在主轴方向上搜索更远的点
        search_length = min_target_length * 0.6  # 搜索半径
        axis_mask = (local_u > -search_length) & (local_u < search_length)
        
        extended_mask = cross_section_mask & axis_mask
        extended_points = roi_points[extended_mask]
        
        if len(extended_points) < 50:
            print(f"    扩展失败：找到的点太少 ({len(extended_points)})")
            extended_ibeams.append(beam)
            continue
        
        # 重新计算尺寸
        extended_centered = extended_points - center
        ext_u = np.dot(extended_centered, u)
        ext_v = np.dot(extended_centered, v)
        ext_w = np.dot(extended_centered, w)
        
        new_length = ext_u.max() - ext_u.min()
        new_width = ext_v.max() - ext_v.min()
        new_height = ext_w.max() - ext_w.min()
        new_center = extended_points.mean(axis=0)
        
        # 检查扩展后的尺寸是否合理
        if new_length < length * 1.1:  # 几乎没有扩展
            print(f"    扩展效果不明显：{length:.2f}m -> {new_length:.2f}m")
            extended_ibeams.append(beam)
            continue
        
        if new_width > 0.6 or new_height > 0.8:  # 截面变得太大
            print(f"    扩展后截面异常：{new_width:.2f} x {new_height:.2f}m")
            extended_ibeams.append(beam)
            continue
        
        print(f"    扩展成功：{length:.2f}m -> {new_length:.2f}m")
        print(f"    新尺寸：{new_length:.2f} x {new_width:.2f} x {new_height:.2f}m")
        
        # 更新工字钢信息
        extended_beam = beam.copy()
        extended_beam['center'] = new_center
        extended_beam['dimensions'] = (new_length, new_width, new_height)
        extended_beam['points'] = len(extended_points)
        extended_beam['extended'] = True
        extended_ibeams.append(extended_beam)
    
    return extended_ibeams


def merge_ibeam_clusters(points, labels, max_gap=0.5, max_height_diff=0.2, min_axis_similarity=0.9):
    """
    合并可能属于同一工字钢的聚类
    
    判断条件:
    1. 主轴方向相似(点积接近 1)
    2. 高度(Z坐标)相近
    3. 在主轴方向上有一定间隔但不太远
    
    Args:
        points: 点云坐标数组
        labels: 聚类标签数组
        max_gap: 沿主轴方向的最大间隔距离
        max_height_diff: 最大高度差
        min_axis_similarity: 最小主轴相似度(点积)
    
    Returns:
        合并后的标签数组
    """
    max_label = labels.max()
    if max_label < 1:
        return labels  # 只有一个或没有聚类,无需合并
    
    # 计算每个聚类的属性
    cluster_info = []
    for cluster_id in range(max_label + 1):
        cluster_mask = labels == cluster_id
        cluster_points = points[cluster_mask]
        
        if len(cluster_points) < 10:
            cluster_info.append(None)
            continue
        
        center = np.mean(cluster_points, axis=0)
        
        # PCA 计算主轴
        centered = cluster_points - center
        cov = np.cov(centered.T)
        eigenvalues, eigenvectors = np.linalg.eigh(cov)
        sorted_idx = np.argsort(eigenvalues)[::-1]
        main_axis = eigenvectors[:, sorted_idx[0]]
        
        # 确保主轴方向一致(指向 X 负方向)
        if main_axis[0] > 0:
            main_axis = -main_axis
        
        # 计算沿主轴的范围
        proj = np.dot(centered, main_axis)
        axis_min = proj.min() + np.dot(center, main_axis)
        axis_max = proj.max() + np.dot(center, main_axis)
        
        cluster_info.append({
            'center': center,
            'main_axis': main_axis,
            'axis_min': axis_min,
            'axis_max': axis_max,
            'height': center[2]
        })
    
    # 查找需要合并的聚类对
    merge_pairs = []
    for i in range(max_label + 1):
        if cluster_info[i] is None:
            continue
        for j in range(i + 1, max_label + 1):
            if cluster_info[j] is None:
                continue
            
            info_i = cluster_info[i]
            info_j = cluster_info[j]
            
            # 条件1:主轴方向相似
            axis_similarity = abs(np.dot(info_i['main_axis'], info_j['main_axis']))
            if axis_similarity < min_axis_similarity:
                continue
            
            # 条件2:高度相近
            height_diff = abs(info_i['height'] - info_j['height'])
            if height_diff > max_height_diff:
                continue
            
            # 条件3:在主轴方向上相邻(有间隔但不重叠太多)
            # 计算间隔:一个聚类的 max 与另一个的 min 之间的距离
            gap1 = info_j['axis_min'] - info_i['axis_max']  # i 在 j 左边
            gap2 = info_i['axis_min'] - info_j['axis_max']  # j 在 i 左边
            gap = max(gap1, gap2)  # 取正的间隔值
            
            # 条件4:两个聚类中心的垂直距离（Y方向）应该相近
            # 这确保它们在同一条线上，而不是平行的不同工字钢
            center_i = info_i['center']
            center_j = info_j['center']
            lateral_dist = np.sqrt((center_i[1] - center_j[1])**2)  # Y方向距离
            
            # 垂直距离应该很小（同一工字钢）
            if lateral_dist > 0.3:  # 如果Y方向距离超过0.3m，可能是不同的工字钢
                continue
            
            if gap < max_gap and gap > -0.3:  # 减少允许的重叠量
                print(f"  -> 合并聚类 #{i} 和 #{j}: 主轴相似度={axis_similarity:.2f}, 高度差={height_diff:.2f}m, 间隔={gap:.2f}m, Y距离={lateral_dist:.2f}m")
                merge_pairs.append((i, j))
    
    # 执行合并(使用并查集思想)
    if not merge_pairs:
        return labels
    
    # 创建映射
    label_map = {i: i for i in range(max_label + 1)}
    
    def find_root(x):
        while label_map[x] != x:
            x = label_map[x]
        return x
    
    for i, j in merge_pairs:
        root_i = find_root(i)
        root_j = find_root(j)
        if root_i != root_j:
            label_map[root_j] = root_i
    
    # 应用映射
    new_labels = labels.copy()
    for old_label in range(max_label + 1):
        new_label = find_root(old_label)
        new_labels[labels == old_label] = new_label
    
    # 重新编号(使标签连续)
    unique_labels = np.unique(new_labels[new_labels >= 0])
    label_remap = {old: new for new, old in enumerate(unique_labels)}
    for old, new in label_remap.items():
        new_labels[new_labels == old] = new
    
    return new_labels


def remove_wall_planes(pcd, distance_threshold=0.08, min_plane_points=500, max_iterations=3):
    """
    使用 RANSAC 迭代移除大型垂直平面(墙面)
    
    Args:
        pcd: 输入点云
        distance_threshold: RANSAC 平面拟合距离阈值
        min_plane_points: 最小平面点数阈值
        max_iterations: 最大迭代次数
    
    Returns:
        移除墙面后的点云
    """
    pcd_remaining = pcd
    removed_total = 0
    
    for i in range(max_iterations):
        if len(pcd_remaining.points) < min_plane_points:
            break
        
        # RANSAC 平面分割
        plane_model, inliers = pcd_remaining.segment_plane(
            distance_threshold=distance_threshold,
            ransac_n=3,
            num_iterations=1000
        )
        
        if len(inliers) < min_plane_points:
            break
        
        a, b, c, d = plane_model
        normal = np.array([a, b, c])
        
        # 检查是否为垂直平面(法向量接近水平,即 Z 分量小)
        angle_to_vertical = np.degrees(np.arccos(np.abs(normal[2])))
        is_vertical_plane = angle_to_vertical > 60  # 与垂直方向夹角 > 60°,即接近水平法向量
        
        # 检查平面的空间范围(墙面通常跨度大)
        inlier_points = np.asarray(pcd_remaining.select_by_index(inliers).points)
        z_span = inlier_points[:, 2].max() - inlier_points[:, 2].min()
        xy_dist = np.sqrt(inlier_points[:, 0]**2 + inlier_points[:, 1]**2)
        
        # 墙面特征:垂直平面 + Z方向跨度大 + 距离较远
        is_wall = is_vertical_plane and z_span > 1.0 and xy_dist.mean() > 4.0
        
        if is_wall:
            print(f"  移除墙面 {i+1}: {len(inliers)} 点, Z跨度={z_span:.1f}m, 平均距离={xy_dist.mean():.1f}m")
            pcd_remaining = pcd_remaining.select_by_index(inliers, invert=True)
            removed_total += len(inliers)
        else:
            # 不是墙面,停止移除
            break
    
    print(f"  共移除 {removed_total} 个墙面点")
    return pcd_remaining


def refine_ibeam_detection(cluster_points, main_axis, center):
    """
    对候选工字钢区域进行精细化识别,提取真正的工字钢点
    
    基于工字钢截面特征(I形截面):
    - 沿主轴方向切片分析截面
    - 保留符合工字钢截面特征的点
    - 排除截面过大或形态异常的区域
    
    工字钢截面特征:
    - 宽度(Y方向):约 0.1-0.3m
    - 高度(Z方向):约 0.15-0.5m
    - 形状:上下翼缘 + 中间腹板
    
    Args:
        cluster_points: 候选区域的所有点 (N, 3)
        main_axis: 工字钢主轴方向(单位向量)
        center: 工字钢中心点
    
    Returns:
        refined_points: 精细化后的点 (M, 3)
        refined_center: 精细化后的中心
        refined_dims: 精细化后的尺寸 (length, width, height)
        如果精细化失败返回 None
    """
    if len(cluster_points) < 50:
        return None
    
    # 确保主轴方向一致(指向 X 正方向)
    if main_axis[0] < 0:
        main_axis = -main_axis
    
    # 构建局部坐标系
    # u: 主轴方向(沿工字钢长度)
    # v: 水平垂直于主轴(沿翼缘宽度)
    # w: 垂直方向(沿工字钢高度)
    u = main_axis / np.linalg.norm(main_axis)
    w = np.array([0, 0, 1])  # 垂直向上
    v = np.cross(w, u)
    v = v / np.linalg.norm(v)
    
    # 将点转换到局部坐标系(以 center 为原点)
    points_centered = cluster_points - center
    local_u = np.dot(points_centered, u)  # 沿主轴
    local_v = np.dot(points_centered, v)  # 沿翼缘宽度
    local_w = np.dot(points_centered, w)  # 沿高度
    
    # 分析参数
    slice_width = 0.2  # 切片宽度(沿主轴)
    u_min, u_max = local_u.min(), local_u.max()
    n_slices = max(5, int((u_max - u_min) / slice_width))
    
    # 工字钢截面尺寸限制(基于分析结果)
    max_v_span = 0.30   # 最大翼缘宽度(收紧)
    # 工字钢截面尺寸限制(放宽范围,避免过度过滤)
    max_v_span = 0.50   # 最大翼缘宽度（放宽）
    max_w_span = 0.60   # 最大截面高度（放宽）
    min_v_span = 0.03   # 最小翼缘宽度（放宽）
    min_w_span = 0.05   # 最小截面高度（放宽）
    
    # 存储每个切片的有效点索引
    valid_point_mask = np.zeros(len(cluster_points), dtype=bool)
    valid_slices = 0
    
    # 收集每个切片的 v 和 w 范围,用于统计典型截面
    slice_v_centers = []
    slice_w_centers = []
    slice_v_spans = []
    slice_w_spans = []
    
    for i in range(n_slices):
        u_start = u_min + i * (u_max - u_min) / n_slices
        u_end = u_start + (u_max - u_min) / n_slices
        
        # 当前切片的点
        slice_mask = (local_u >= u_start) & (local_u < u_end)
        slice_indices = np.where(slice_mask)[0]
        
        if len(slice_indices) < 3:  # 降低最小点数要求
            continue
        
        v_slice = local_v[slice_indices]
        w_slice = local_w[slice_indices]
        
        v_span = v_slice.max() - v_slice.min()
        w_span = w_slice.max() - w_slice.min()
        v_center = (v_slice.max() + v_slice.min()) / 2
        w_center = (w_slice.max() + w_slice.min()) / 2
        
        # 检查截面是否在合理范围内
        if min_v_span < v_span < max_v_span and min_w_span < w_span < max_w_span:
            valid_point_mask[slice_indices] = True
            valid_slices += 1
            slice_v_centers.append(v_center)
            slice_w_centers.append(w_center)
            slice_v_spans.append(v_span)
            slice_w_spans.append(w_span)
    
    # 如果有效切片数太少，尝试使用更宽松的标准
    if valid_slices < 3:
        # 尝试仅基于位置一致性，不做严格的截面尺寸过滤
        valid_point_mask = np.ones(len(cluster_points), dtype=bool)
        valid_slices = n_slices
        slice_v_centers = [local_v.mean()]
        slice_w_centers = [local_w.mean()]
        print(f"       精细化:使用宽松模式")
    
    # 计算典型截面中心位置(用于进一步过滤离群点)
    median_v = np.median(slice_v_centers)
    median_w = np.median(slice_w_centers)
    
    # 计算典型截面尺寸，用于动态调整容差
    if slice_v_spans:
        typical_v_span = np.median(slice_v_spans)
        typical_w_span = np.median(slice_w_spans)
    else:
        typical_v_span = 0.3
        typical_w_span = 0.4
    
    # 第二遍过滤:移除偏离典型截面中心太远的点
    # 使用动态容差，基于典型截面尺寸
    v_tolerance = max(0.20, typical_v_span * 0.7)  # 至少0.20m或截面宽度的70%
    w_tolerance = max(0.25, typical_w_span * 0.7)  # 至少0.25m或截面高度的70%
    
    distance_v = np.abs(local_v - median_v)
    distance_w = np.abs(local_w - median_w)
    
    refined_mask = valid_point_mask & (distance_v < v_tolerance) & (distance_w < w_tolerance)
    
    refined_points = cluster_points[refined_mask]
    
    if len(refined_points) < 30:
        print(f"       精细化警告:精细化后点数 {len(refined_points)} < 30")
        return None
    
    # 计算精细化后的尺寸
    local_u_refined = local_u[refined_mask]
    local_v_refined = local_v[refined_mask]
    local_w_refined = local_w[refined_mask]
    
    refined_length = local_u_refined.max() - local_u_refined.min()
    refined_width = local_v_refined.max() - local_v_refined.min()
    refined_height = local_w_refined.max() - local_w_refined.min()
    
    # 计算精细化后的中心
    refined_center = refined_points.mean(axis=0)
    
    return refined_points, refined_center, (refined_length, refined_width, refined_height)


def visualize_with_matplotlib(pcd, geometries, found_ibeams=None, use_plotly=False):
    """使用 Open3D 或 plotly 可视化点云和 OBB 边界框(支持鼠标交互)
    
    Args:
        use_plotly: 强制使用 plotly(浏览器)可视化
    """
    
    if use_plotly:
        visualize_with_plotly(pcd, geometries, found_ibeams)
        return
    
    # 准备可视化几何体列表
    vis_geometries = []
    
    # 添加点云(灰色)
    pcd_vis = o3d.geometry.PointCloud(pcd)
    pcd_vis.paint_uniform_color([0.5, 0.5, 0.5])  # 灰色
    vis_geometries.append(pcd_vis)
    
    # 添加坐标系(帮助理解方向)
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])
    vis_geometries.append(coord_frame)
    
    # 添加边界框
    for geom in geometries:
        if isinstance(geom, o3d.geometry.OrientedBoundingBox):
            vis_geometries.append(geom)
    
    # 打印交互说明
    print("\n=== 3D 可视化交互说明 ===")
    print("  鼠标左键拖动: 旋转视角")
    print("  鼠标滚轮: 缩放")
    print("  鼠标右键拖动: 平移")
    print("  Shift + 左键: 平移")
    print("  Ctrl + 左键: 滚动")
    print("  R: 重置视角")
    print("  Q 或 ESC: 关闭窗口")
    print("  +/-: 调整点大小")
    print("  红色边界框: 识别为工字钢")
    print("  蓝色边界框: 未识别的聚类")
    print("=" * 30)
    
    # 尝试使用 Open3D 可视化
    try:
        # 创建可视化器
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name="I-Beam Detection Result", width=1280, height=720)
        
        for geom in vis_geometries:
            vis.add_geometry(geom)
        
        # 设置渲染选项
        opt = vis.get_render_option()
        opt.background_color = np.array([0.1, 0.1, 0.15])  # 深色背景
        opt.point_size = 2.0
        
        # 设置初始视角
        ctr = vis.get_view_control()
        ctr.set_zoom(0.5)
        
        vis.run()
        vis.destroy_window()
        
    except Exception as e:
        print(f"\nOpen3D 可视化失败: {e}")
        print("正在使用 plotly 进行交互式可视化...")
        visualize_with_plotly(pcd, geometries, found_ibeams)


def visualize_with_plotly(pcd, geometries, found_ibeams=None):
    """使用 plotly 进行交互式 3D 可视化(浏览器中打开)"""
    try:
        import plotly.graph_objects as go
    except ImportError:
        print("请安装 plotly: pip install plotly")
        return
    
    fig = go.Figure()
    
    # 添加点云
    points = np.asarray(pcd.points)
    step = max(1, len(points) // 10000)  # 限制点数提升性能
    fig.add_trace(go.Scatter3d(
        x=points[::step, 0],
        y=points[::step, 1],
        z=points[::step, 2],
        mode='markers',
        marker=dict(size=1, color='gray', opacity=0.5),
        name='Point Cloud'
    ))
    
    # 添加边界框
    for i, geom in enumerate(geometries):
        if isinstance(geom, o3d.geometry.OrientedBoundingBox):
            corners = np.asarray(geom.get_box_points())
            color = geom.color
            
            # 判断颜色
            if color[0] > 0.5:  # 红色 - 工字钢
                line_color = 'red'
                name = f'I-Beam {i}'
            else:  # 蓝色 - 未识别
                line_color = 'blue'
                name = f'Cluster {i}'
            
            # 绘制边界框的 12 条边
            edges = [
                [0, 1], [1, 2], [2, 3], [3, 0],  # 底面
                [4, 5], [5, 6], [6, 7], [7, 4],  # 顶面
                [0, 4], [1, 5], [2, 6], [3, 7]   # 侧边
            ]
            
            for edge in edges:
                fig.add_trace(go.Scatter3d(
                    x=[corners[edge[0], 0], corners[edge[1], 0]],
                    y=[corners[edge[0], 1], corners[edge[1], 1]],
                    z=[corners[edge[0], 2], corners[edge[1], 2]],
                    mode='lines',
                    line=dict(color=line_color, width=4),
                    showlegend=False
                ))
    
    # 设置布局
    fig.update_layout(
        title='I-Beam Detection Result (交互式3D视图)',
        scene=dict(
            xaxis_title='X (m)',
            yaxis_title='Y (m)',
            zaxis_title='Z (m)',
            aspectmode='data',
            camera=dict(
                up=dict(x=0, y=0, z=1),
                center=dict(x=0, y=0, z=0),
                eye=dict(x=1.5, y=1.5, z=1.0)
            )
        ),
        width=1200,
        height=800,
        margin=dict(l=0, r=0, t=40, b=0)
    )
    
    # 保存为 HTML 并打开
    html_path = "/tmp/ibeam_detection_result.html"
    fig.write_html(html_path)
    print(f"\n可视化结果已保存到: {html_path}")
    
    # 尝试在浏览器中打开
    import webbrowser
    webbrowser.open(f"file://{html_path}")


# ============================================================================
# ROS Bag 持续检测验证功能
# ============================================================================

def pointcloud2_to_open3d(msg_data):
    """
    将 ROS PointCloud2 消息转换为 Open3D 点云
    
    Args:
        msg_data: 反序列化后的 PointCloud2 消息数据
    
    Returns:
        o3d.geometry.PointCloud 对象
    """
    height = msg_data.height
    width = msg_data.width
    point_step = msg_data.point_step
    data = bytes(msg_data.data)
    
    # 查找 x, y, z 字段的偏移量
    field_offsets = {}
    for field in msg_data.fields:
        field_offsets[field.name] = (field.offset, field.datatype)
    
    # 数据类型映射 (ROS PointField 类型)
    dtype_map = {
        1: ('b', 1), 2: ('B', 1), 3: ('h', 2), 4: ('H', 2),
        5: ('i', 4), 6: ('I', 4), 7: ('f', 4), 8: ('d', 8),
    }
    
    points = []
    n_points = height * width
    
    for i in range(n_points):
        offset = i * point_step
        try:
            x_off, x_type = field_offsets.get('x', (0, 7))
            y_off, y_type = field_offsets.get('y', (4, 7))
            z_off, z_type = field_offsets.get('z', (8, 7))
            
            fmt_x, size_x = dtype_map[x_type]
            fmt_y, size_y = dtype_map[y_type]
            fmt_z, size_z = dtype_map[z_type]
            
            x = struct.unpack(fmt_x, data[offset + x_off:offset + x_off + size_x])[0]
            y = struct.unpack(fmt_y, data[offset + y_off:offset + y_off + size_y])[0]
            z = struct.unpack(fmt_z, data[offset + z_off:offset + z_off + size_z])[0]
            
            if np.isfinite(x) and np.isfinite(y) and np.isfinite(z):
                points.append([x, y, z])
        except Exception:
            continue
    
    pcd = o3d.geometry.PointCloud()
    if points:
        pcd.points = o3d.utility.Vector3dVector(np.array(points))
    return pcd


def detect_ibeam_from_pointcloud(pcd, max_distance=6.0, min_distance=2.0, min_height=1.5):
    """
    从 Open3D 点云中检测工字钢(用于 rosbag 验证)
    
    Args:
        pcd: Open3D 点云对象
        max_distance: 最大检测距离
        min_distance: 最小检测距离
        min_height: 最小高度
    
    Returns:
        检测结果字典列表
    """
    if not pcd.has_points() or len(pcd.points) < 100:
        return []
    
    # 预处理
    pcd = pcd.voxel_down_sample(voxel_size=0.02)
    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.5)
    
    if len(pcd.points) < 50:
        return []
    
    # 移除墙面
    pcd_filtered = remove_wall_planes(pcd, distance_threshold=0.08, min_plane_points=500)
    
    if len(pcd_filtered.points) < 50:
        return []
    
    # 距离和高度过滤
    points = np.asarray(pcd_filtered.points)
    distances = np.linalg.norm(points[:, :2], axis=1)
    spatial_mask = (distances > min_distance) & (distances < max_distance) & (points[:, 2] > min_height)
    indices = np.where(spatial_mask)[0]
    
    if len(indices) < 50:
        return []
    
    pcd_roi = pcd_filtered.select_by_index(indices)
    roi_points = np.asarray(pcd_roi.points)
    
    # 分层聚类
    height_layers = [(1.5, 2.5), (2.5, 4.0), (4.0, 7.0)]
    all_labels = np.full(len(roi_points), -1, dtype=int)
    current_label = 0
    
    for z_min, z_max in height_layers:
        layer_mask = (roi_points[:, 2] >= z_min) & (roi_points[:, 2] < z_max)
        layer_indices = np.where(layer_mask)[0]
        
        if len(layer_indices) < 10:
            continue
        
        layer_pcd = pcd_roi.select_by_index(layer_indices)
        layer_labels = np.array(layer_pcd.cluster_dbscan(eps=0.15, min_points=10, print_progress=False))
        
        n_clusters = layer_labels.max() + 1
        if n_clusters > 0:
            for local_label in range(n_clusters):
                local_mask = layer_labels == local_label
                global_indices = layer_indices[local_mask]
                all_labels[global_indices] = current_label
                current_label += 1
    
    if current_label == 0:
        return []
    
    # 合并聚类
    all_labels = merge_ibeam_clusters(roi_points, all_labels, max_gap=1.5, min_axis_similarity=0.85)
    
    # 分析每个聚类
    unique_labels = [l for l in np.unique(all_labels) if l >= 0]
    found_ibeams = []
    
    for label in unique_labels:
        cluster_mask = all_labels == label
        cluster_points = roi_points[cluster_mask]
        
        if len(cluster_points) < 30:
            continue
        
        # PCA 分析
        center = cluster_points.mean(axis=0)
        centered = cluster_points - center
        cov = np.cov(centered.T)
        eigenvalues, eigenvectors = np.linalg.eigh(cov)
        
        idx = np.argsort(eigenvalues)[::-1]
        eigenvalues = eigenvalues[idx]
        eigenvectors = eigenvectors[:, idx]
        
        linearity = (eigenvalues[0] - eigenvalues[1]) / (eigenvalues[0] + 1e-6)
        std_devs = np.sqrt(eigenvalues)
        length, width, thickness = std_devs[0] * 4, std_devs[1] * 4, std_devs[2] * 4
        
        main_axis = eigenvectors[:, 0]
        main_axis_z = abs(main_axis[2])
        aspect_ratio = length / max(width, 0.01)
        
        is_ibeam = (linearity > 0.8 and length > 0.8 and aspect_ratio > 2.5 and 
                    main_axis_z < 0.2 and width < 1.5 and thickness < 1.0)
        
        if is_ibeam:
            refined_result = refine_ibeam_detection(cluster_points, main_axis, center)
            if refined_result is not None:
                refined_points, refined_center, refined_dims = refined_result
                found_ibeams.append({
                    'center': refined_center, 'dimensions': refined_dims,
                    'main_axis': main_axis, 'linearity': linearity,
                    'points': len(refined_points), 'original_points': len(cluster_points)
                })
            else:
                found_ibeams.append({
                    'center': center, 'dimensions': (length, width, thickness),
                    'main_axis': main_axis, 'linearity': linearity,
                    'points': len(cluster_points), 'original_points': len(cluster_points)
                })
    
    return found_ibeams


def validate_with_rosbag(bag_path, topic='/front_arm_lidar', skip_frames=10, max_frames=None, 
                         visualize=False, export_csv=False, save_results=True, res_folder=None):
    """
    使用 ROS bag 文件持续验证工字钢检测算法的稳健性
    
    Args:
        bag_path: rosbag 文件路径
        topic: 点云话题名称
        skip_frames: 跳帧间隔(默认每10帧处理一次)
        max_frames: 最大处理帧数(None表示处理所有帧)
        visualize: 是否启用实时3D可视化
        export_csv: 是否导出结果到 CSV 文件
        save_results: 是否保存每帧的点云和可视化结果
        res_folder: 结果保存文件夹(默认为脚本目录下的res文件夹)
    
    使用示例:
        validate_with_rosbag('/path/to/bag', skip_frames=10, max_frames=50, save_results=True)
    """
    try:
        from rosbags.rosbag1 import Reader
        from rosbags.serde import deserialize_cdr, ros1_to_cdr
    except ImportError:
        print("错误: 请安装 rosbags 库: pip install rosbags")
        return
    
    if not os.path.exists(bag_path):
        print(f"错误: 文件不存在 - {bag_path}")
        return
    
    # 创建结果保存文件夹
    session_folder = None
    pcd_folder = None
    img_folder = None
    if save_results:
        if res_folder is None:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            res_folder = os.path.join(script_dir, "res")
        
        # 创建带时间戳的子文件夹
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        session_folder = os.path.join(res_folder, f"validation_{timestamp_str}")
        pcd_folder = os.path.join(session_folder, "pcd")
        img_folder = os.path.join(session_folder, "images")
        
        try:
            os.makedirs(pcd_folder, exist_ok=True)
            os.makedirs(img_folder, exist_ok=True)
            print(f"结果将保存到: {session_folder}")
        except PermissionError:
            # 如果没有权限，使用用户主目录下的备选位置
            fallback_folder = os.path.expanduser("~/ibeam_validation_results")
            session_folder = os.path.join(fallback_folder, f"validation_{timestamp_str}")
            pcd_folder = os.path.join(session_folder, "pcd")
            img_folder = os.path.join(session_folder, "images")
            os.makedirs(pcd_folder, exist_ok=True)
            os.makedirs(img_folder, exist_ok=True)
            print(f"警告: 无法写入 {res_folder}，使用备选目录")
            print(f"结果将保存到: {session_folder}")
    
    print(f"\n{'='*60}")
    print(f"ROS Bag 工字钢检测验证")
    print(f"{'='*60}")
    print(f"Bag 文件: {bag_path}")
    print(f"话题: {topic}")
    print(f"跳帧: 每 {skip_frames} 帧处理一次")
    if max_frames:
        print(f"最大帧数: {max_frames}")
    if save_results:
        print(f"保存结果: {session_folder}")
    print(f"{'='*60}\n")
    
    results = []
    frame_idx = 0
    processed_count = 0
    
    # 可视化窗口
    vis = None
    if visualize:
        vis = o3d.visualization.Visualizer()
        vis.create_window("I-Beam Detection", width=1280, height=720)
    
    with Reader(bag_path) as reader:
        connections = [c for c in reader.connections if c.topic == topic]
        
        if not connections:
            print(f"错误: 找不到话题 {topic}")
            print(f"可用话题: {[c.topic for c in reader.connections]}")
            return
        
        conn = connections[0]
        print(f"消息类型: {conn.msgtype}")
        
        for connection, timestamp, rawdata in reader.messages(connections=[conn]):
            frame_idx += 1
            
            if frame_idx % skip_frames != 0:
                continue
            
            if max_frames and processed_count >= max_frames:
                break
            
            processed_count += 1
            
            try:
                msg = deserialize_cdr(ros1_to_cdr(rawdata, conn.msgtype), conn.msgtype)
            except Exception as e:
                print(f"帧 {frame_idx}: 反序列化失败 - {e}")
                continue
            
            pcd = pointcloud2_to_open3d(msg)
            n_points = len(pcd.points)
            
            try:
                ibeams = detect_ibeam_from_pointcloud(pcd)
            except Exception as e:
                print(f"帧 {frame_idx}: 检测失败 - {e}")
                ibeams = []
            
            time_sec = timestamp / 1e9
            results.append({
                'frame': frame_idx, 'timestamp': time_sec,
                'n_points': n_points, 'n_ibeams': len(ibeams), 'ibeams': ibeams
            })
            
            if ibeams:
                ibeam_info = ", ".join([f"({ib['center'][0]:.1f}, {ib['center'][1]:.1f}, {ib['center'][2]:.1f})" for ib in ibeams])
                print(f"帧 {frame_idx:4d} | 点数: {n_points:5d} | 工字钢: {len(ibeams)} 个 | 位置: {ibeam_info}")
            else:
                print(f"帧 {frame_idx:4d} | 点数: {n_points:5d} | 工字钢: 0 个")
            
            if visualize and vis:
                vis.clear_geometries()
                pcd.paint_uniform_color([0.5, 0.5, 0.5])
                vis.add_geometry(pcd)
                for ibeam in ibeams:
                    c, d = ibeam['center'], ibeam['dimensions']
                    bbox = o3d.geometry.AxisAlignedBoundingBox(
                        min_bound=c - np.array([d[0]/2, d[1]/2, d[2]/2]),
                        max_bound=c + np.array([d[0]/2, d[1]/2, d[2]/2])
                    )
                    bbox.color = np.array([1.0, 0.0, 0.0])
                    vis.add_geometry(bbox)
                coord = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)
                vis.add_geometry(coord)
                if not vis.poll_events():
                    break
                vis.update_renderer()
                
                # 保存可视化截图
                if save_results:
                    img_path = os.path.join(img_folder, f"frame_{frame_idx:06d}.png")
                    vis.capture_screen_image(img_path, do_render=True)
            
            # 保存点云和检测结果（无论是否可视化）
            if save_results:
                # 保存原始点云
                pcd_path = os.path.join(pcd_folder, f"frame_{frame_idx:06d}.pcd")
                o3d.io.write_point_cloud(pcd_path, pcd)
                
                # 保存该帧的检测结果到 JSON
                import json
                result_data = {
                    'frame': frame_idx,
                    'timestamp': time_sec,
                    'n_points': n_points,
                    'n_ibeams': len(ibeams),
                    'ibeams': []
                }
                for ib in ibeams:
                    result_data['ibeams'].append({
                        'center': ib['center'].tolist() if hasattr(ib['center'], 'tolist') else list(ib['center']),
                        'dimensions': list(ib['dimensions']),
                        'linearity': float(ib['linearity']),
                        'points': int(ib['points']),
                        'original_points': int(ib.get('original_points', ib['points']))
                    })
                
                json_path = os.path.join(pcd_folder, f"frame_{frame_idx:06d}.json")
                with open(json_path, 'w') as f:
                    json.dump(result_data, f, indent=2)
    
    if vis:
        vis.destroy_window()
    
    # 打印统计
    if results:
        print(f"\n{'='*60}")
        print("检测统计")
        print(f"{'='*60}")
        
        n_frames = len(results)
        frames_with_ibeam = sum(1 for r in results if r['n_ibeams'] > 0)
        total_ibeams = sum(r['n_ibeams'] for r in results)
        
        print(f"总帧数: {n_frames}")
        print(f"检测到工字钢的帧数: {frames_with_ibeam} ({100*frames_with_ibeam/n_frames:.1f}%)")
        print(f"平均每帧检测数: {total_ibeams/n_frames:.2f}")
        
        positions = None
        if frames_with_ibeam > 0:
            positions_list = []
            for r in results:
                for ib in r['ibeams']:
                    positions_list.append(ib['center'])
            
            if positions_list:
                positions = np.array(positions_list)
                print(f"\n工字钢位置统计:")
                print(f"  X: [{positions[:,0].min():.2f}, {positions[:,0].max():.2f}] m, "
                      f"均值={positions[:,0].mean():.2f}, 标准差={positions[:,0].std():.2f}")
                print(f"  Y: [{positions[:,1].min():.2f}, {positions[:,1].max():.2f}] m, "
                      f"均值={positions[:,1].mean():.2f}, 标准差={positions[:,1].std():.2f}")
                print(f"  Z: [{positions[:,2].min():.2f}, {positions[:,2].max():.2f}] m, "
                      f"均值={positions[:,2].mean():.2f}, 标准差={positions[:,2].std():.2f}")
        
        print(f"{'='*60}")
        
        # 导出 CSV
        if export_csv or save_results:
            import csv
            
            # 确定输出路径
            if save_results and session_folder:
                output_path = os.path.join(session_folder, "detection_results.csv")
            else:
                timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
                output_path = f"ibeam_detection_results_{timestamp_str}.csv"
            
            with open(output_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['frame', 'timestamp', 'n_points', 'n_ibeams',
                                 'ibeam_x', 'ibeam_y', 'ibeam_z', 'length', 'width', 'height', 'linearity', 'points'])
                for r in results:
                    if r['ibeams']:
                        for ib in r['ibeams']:
                            writer.writerow([
                                r['frame'], f"{r['timestamp']:.3f}", r['n_points'], r['n_ibeams'],
                                f"{ib['center'][0]:.3f}", f"{ib['center'][1]:.3f}", f"{ib['center'][2]:.3f}",
                                f"{ib['dimensions'][0]:.3f}", f"{ib['dimensions'][1]:.3f}", f"{ib['dimensions'][2]:.3f}",
                                f"{ib['linearity']:.3f}", ib['points']
                            ])
                    else:
                        writer.writerow([r['frame'], f"{r['timestamp']:.3f}", r['n_points'], 0, '', '', '', '', '', '', '', ''])
            print(f"\n结果已导出到: {output_path}")
            
            # 如果启用save_results，还保存一个汇总信息
            if save_results:
                summary_path = os.path.join(session_folder, "summary.txt")
                with open(summary_path, 'w') as f:
                    f.write(f"验证时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                    f.write(f"Bag文件: {bag_path}\n")
                    f.write(f"跳帧: {skip_frames}\n")
                    f.write(f"总帧数: {n_frames}\n")
                    f.write(f"检测到工字钢的帧数: {frames_with_ibeam} ({100*frames_with_ibeam/n_frames:.1f}%)\n")
                    f.write(f"平均每帧检测数: {total_ibeams/n_frames:.2f}\n")
                    if frames_with_ibeam > 0 and positions is not None and len(positions) > 0:
                        f.write(f"\n工字钢位置统计:\n")
                        f.write(f"  X: [{positions[:,0].min():.2f}, {positions[:,0].max():.2f}] m\n")
                        f.write(f"  Y: [{positions[:,1].min():.2f}, {positions[:,1].max():.2f}] m\n")
                        f.write(f"  Z: [{positions[:,2].min():.2f}, {positions[:,2].max():.2f}] m\n")
                print(f"汇总已保存到: {summary_path}")
    
    return results


if __name__ == "__main__":
    # 默认:单帧检测
    file_path = "/home/oneko/projects/ros_airy_work/ros_bag/113pcd/1768291167.129343748.pcd"
    detect_ibeams_final(file_path, max_distance=6.0, min_distance=2.0, min_height=1.5)
    
    # ===== ROS Bag 持续验证 =====
    # 如需验证算法稳健性,取消下面注释并运行:
    #
    # bag_path = "/home/oneko/projects/ros_airy_work/ros_bag/2026-01-13-15-59-20.bag"
    # res_folder = "/home/oneko/projects/ros_airy_work/src/pcl_process/src/ibeam_fitting/res"
    # validate_with_rosbag(bag_path, skip_frames=5, max_frames=500, visualize=True, save_results=True, res_folder=res_folder)
    #
    # 参数说明:
    #   skip_frames: 每隔多少帧处理一次(减少处理时间)
    #   max_frames: 最多处理多少帧(None 表示全部)
    #   visualize: 是否实时3D可视化
    #   export_csv: 是否导出检测结果到 CSV
    
    # 使用说明
    # 1. 单帧检测(直接运行脚本)
    # python3 imeam_detector.py

    # # 2. ROS Bag 持续验证(在脚本末尾取消注释或导入调用)
    # from imeam_detector import validate_with_rosbag

    # bag_path = "/home/oneko/projects/ros_airy_work/ros_bag/2026-01-13-15-59-20.bag"

    # # 快速验证:每20帧处理1次,最多50帧
    # validate_with_rosbag(bag_path, skip_frames=20, max_frames=50)

    # # 完整验证:处理所有帧
    # validate_with_rosbag(bag_path, skip_frames=1)

    # # 带可视化
    # validate_with_rosbag(bag_path, skip_frames=10, visualize=True)

    # # 导出CSV结果
    # validate_with_rosbag(bag_path, skip_frames=10, export_csv=True)