# 12. 圆柱识别与规划

## 12.1 处理流程

圆柱入口为 `CylinderFitter::run_with_rslidar()`：

1. 检查点云、规划输入和可视化对象；
2. 对输入点云进行墙面/水平大平面迭代移除；
3. KNN 法线估计；
4. PCL `SACSegmentationFromNormals` 圆柱 RANSAC；
5. 在模型参数限制内提取圆柱内点；
6. 计算拟合圆柱的中心、轴、半径、高度和底部中心；
7. 校验内点数、内点比例、半径、径向标准差、径向极差、长宽比、圆度和轴向；
8. 在雷达系模型上进行规划；ROS 发布包装层把点坐标转换到基座 O，圆柱轴向量则依赖当前 `R_OB=I` 的安装假设直接复用；
9. 手动或自动计算可覆盖角、切片角和搭接率；
10. 对固定 180° 半圆弧按高度与弧长重分片；
11. 按配置从上到下或从下到上，以 S 形生成切片和工作点；
12. 发布参数、切片、轨迹列表和面积；首次成功后停止处理。

## 12.2 墙面移除

每轮使用平面 RANSAC：

- 法向量接近水平、即平面近似竖直时视作墙面；
- 法向量接近 Z 时也可识别水平大平面；
- 平面点数小于阈值时停止；
- 删除后剩余点低于 `wall_min_keep_ratio` 时拒绝该次删除；
- 最多处理 `max_walls` 个大平面。

## 12.3 圆柱验证

圆度把截面分为 24 个 15° 扇区，综合非空覆盖率、连续空洞和各扇区半径一致性评分。RANSAC 成功不等于最终接受，所有几何门限仍需通过。

`preset_A` 可把接近竖直的拟合轴校正为理想 `(0,0,1)`；`preset_D` 保留拟合轴。

## 12.4 自动可达角计算

自动模式使用：

```text
R_task = cylinder_radius + surface_offset
Delta_Z_max = 工作高度上下边界到机械臂基座 Z=0 的最大绝对距离
R_eff = sqrt(R_arm² - Delta_Z_max²)
D = 基座到圆柱轴线的最短距离
cos(alpha) = (R_task² + D² - R_eff²) / (2*R_task*D)
max_coverage_angle = 2*alpha
```

随后根据喷幅、初始搭接率、半径和表面偏移优化切片角与切片数。`execute_slice_planning()` 在不可达、分母为零或高度超臂展时会返回失败状态和非正切片角，但调用者不会因此拒绝整次任务：`run_with_rslidar()` 会进入 `AUTO MODE FALLBACK`，以 `k=1` 和输入喷幅/搭接率调用手动切片计算，然后仍按 180° 生成目标。因而当前可达性计算只是自动优化条件，不是 fail-closed 安全门。当前自动规划还把虚拟坐标中的工作高度中心固定为 `z_center_mm=400`，源码已标注 TODO；它不是从实际模型中心动态推导。

## 12.5 手动模式

手动模式把输入 `k` 和 `lap` 作为初值：

```text
w0 = step_width * (1 - lap)
slice_width = w0 * k
```

随后调整 `k` 和搭接率，使半圆弧能够被整数片均分，并保证有效喷幅/切片宽度合理。回传的 `k`、`lap` 和 `w0` 可能与输入不同；其中 `w0` 被转换并保存为 degree。

## 12.6 切片与工作点

- 实际规划弧总角固定为 180°；
- 高度片数：`round(model_height / requested_height)`；
- 弧向片数：`round(radius * π / requested_width)`；
- 实际片高和角宽重新均分，因此可能与输入值略有差异；
- 每一高度层的弧向顺序交替，构成 S 型；
- `slice_start_from_top=true` 时从最高层开始，否则从最低层开始；
- 表面切片中心沿法向外移 `distance`，得到机械臂工作点。

圆柱 `SprayInfo.area` 使用 `slice_width_mm × requested_height_mm / 1e6 × 有效发布切片数`，单位为 m²；它不一定等于按重新均分后的精确几何面积。

## 12.7 默认规划输入

圆柱 `main()` 在构造 ROS 包装后设置：

| 参数 | 默认值 |
|---|---:|
| `distance` | 750 mm |
| `step_width` | 300 mm |
| `step_height` | 100 mm |
| `height` | 600 mm |
| `lap` | 0.5 |
| `arm_reach` | 1500 mm |
| `mode` | 2（自动） |
| `k` | 0 |

## 12.8 圆柱 YAML 参数

启动时顶层参数等同 `preset_A`；切换 `/work_object` 为 A/D 时重新加载对应组。

| 参数 | A | D | 单位/作用 |
|---|---:|---:|---|
| `virtual_base_to_surface` | 1100 | 1100 | mm，虚拟基座到表面 |
| `min_inliers` | 300 | 30 | 最小 RANSAC 内点 |
| `max_inliers` | 600 | 1000 | 最大内点 |
| `min_inlier_ratio` | 0.02 | 0.005 | 内点比例下限 |
| `max_inlier_ratio` | 0.5 | 0.6 | 内点比例上限 |
| `min_height_m` | 1.0 | 0.08 | m，最小模型高度 |
| `min_radius_m` | 0.30 | 0.06 | m |
| `max_radius_m` | 0.40 | 0.25 | m |
| `min_radial_rel_std` | 0.0 | 0.0 | 径向相对标准差下限 |
| `max_radial_rel_std` | 0.15 | 0.30 | 径向相对标准差上限 |
| `ransac_axis_x` | 0 | 0.707 | 参考轴 X |
| `ransac_axis_y` | 0 | 0 | 参考轴 Y |
| `ransac_axis_z` | 1 | 0.707 | 参考轴 Z |
| `eps_angle_deg` | 15 | 30 | RANSAC 轴向容差，degree |
| `normal_distance_weight` | 0.05 | 0.05 | 法线距离权重 |
| `ransac_max_iterations` | 20000 | 150000 | 最大迭代数 |
| `ransac_distance_threshold` | 0.015 | 0.018 | m |
| `max_radial_range_ratio` | 0.3 | 1.8 | 径向极差/均值上限 |
| `min_aspect_ratio` | 1.5 | 0.5 | 高度/直径下限 |
| `min_circularity` | 0.15 | 0.2 | 圆度下限 |
| `normal_k_search` | 30 | 20 | 法线 KNN |
| `wall_angle_threshold` | 20 | 20 | degree |
| `wall_distance_threshold` | 0.02 | 0.02 | m |
| `wall_max_iterations` | 5000 | 5000 | 平面 RANSAC 迭代 |
| `max_walls` | 3 | 3 | 最大平面数 |
| `wall_min_plane_points` | 100 | 100 | 最小平面点数 |
| `wall_min_keep_ratio` | 0.1 | 0.1 | 删除后最小保留比例 |
| `use_ideal_axis` | true | false | 是否校正为竖直轴 |
| `axis_angle_threshold_deg` | 15 | 50 | 轴校正/验证阈值 |
| `min_center_xy_dist_m` | 默认 0 | 2.0 | D 模式圆心 XY 最小距离 |
| `slice_start_from_top` | true | true | true 从上到下，false 从下到上 |

`min_center_xy_dist_m` 和 `slice_start_from_top` 只在 YAML/预设路径中存在，不在 `CylinderDetection.cfg` 的 rqt 面板中。

`preset_D` 有三个值超出 cfg 范围：`min_height_m=0.08`（下限 0.1）、`ransac_max_iterations=150000`（上限 100000）、`max_radial_range_ratio=1.8`（上限 1.0）。切换预设时 `updateConfig()` 不做 `__clamp__()`，原始值会直接进算法；通过 rqt 提交的请求会先钳位。实际生效值以 `rosparam get` 或运行日志为准。

---

[← 返回 README](../README.md)
