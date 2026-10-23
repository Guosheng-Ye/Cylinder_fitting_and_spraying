# 13. 工字钢识别与规划

## 13.1 处理流程

`IBeamFitter::detect()` 的主要阶段：

1. ROI：按雷达水平距离和 Z 高度裁剪；
2. 体素降采样和统计离群点滤波；
3. 迭代 RANSAC 去墙；
4. 按低、中、高高度层分别欧式聚类（硬编码层界约为 `0–2.5 m`、`2.5–4.0 m`、`4.0–7.0 m`，再受 ROI 的 `min_height/max_height` 限制）；
5. 计算各聚类 PCA 主轴、包围尺寸和密度；
6. 严格条件合并相邻共线聚类；
7. 宽松条件再次合并可能断开的梁段；
8. 用线性度、长度、长宽比、水平性、宽度和厚度筛选候选；
9. 构造局部 `u/v/w` 坐标，检查 I/H 型截面；
10. 根据端部分块的跨度、中心偏移、密度、长宽比和截面分数修剪异常端；
11. 沿主轴搜索扩展稀疏的真实梁端，限制超出原聚类端点距离；
12. 计算腹板高度、翼缘宽度、法向和可工作长度；
13. 按长度均分切片，生成表面中心和机械臂工作点；
14. 当前只发布第一个检测结果。

## 13.2 局部坐标

- `u`：工字钢主轴/长度方向，规范为 X 分量非负；
- `w=(0,0,1)`：竖直、截面高度方向；
- `v=w×u`：水平翼缘方向；
- 正面法向会根据雷达/工字钢相对位置选择符号。

## 13.3 自动长度模式

自动模式先把机械臂最大臂展按 0.75 折减：

```text
R_eff = 0.75 * arm_reach
D_target = d_base - d_work
h_top = beam_height/2 - base_z_offset
h_bottom = -beam_height/2 - base_z_offset
h_max = max(abs(h_top), abs(h_bottom))
working_length = 2 * sqrt(R_eff² - D_target² - h_max²)
```

根号内小于零、或水平距离超过有效半径时规划失败。

## 13.4 手动长度模式

手动模式把 `length` 解释为“允许的最大单片长度”：

```text
n = ceil(识别总长 / 用户 length)
adjusted_slice_length = 识别总长 / n
```

返回值是调整后的单片长度，虽然函数注释和部分输出字段把它称为“总工作长度”。后续 `sliceIBeam()` 又把该值作为切片宽度使用。集成方应按实际行为理解 `beam_length_of_working`。

## 13.5 切片与工作点

- 当前调用传入一个 0 参数，因此 `sliceIBeam()` 使用整段截面高度和 `beam_length_of_working` 作为目标切片尺寸；
- 长度片数使用 `ceil(total_length / target_slice_length)`，然后严格均分；
- 切片沿 `u` 从最小投影到最大投影编号；
- 表面点位于选定正面；
- 机械臂工作点沿 `slice.normal` 指向机器人的方向正向偏移 `d_base`，并叠加 `work_z_offset`；
- `IBeamProcessParam.distance`、`step_height`、`step_length`、`lap`、`mode_theta`、`theta` 目前主要是透传，不驱动切片或工作点几何。

## 13.6 默认规划输入

工字钢 `main()` 不像圆柱节点那样额外调用 `setOption()`；启动时使用 `IBeamInputs` 的值初始化成员，再由 YAML 和后续 bridge 消息覆盖：

| 参数 | 启动初值 |
|---|---:|
| `distance_tcp2surface` | 1100 mm |
| `step_height` / `step_length` / `lap` | 0（值初始化） |
| `arm_reach` | 1500 mm |
| `mode_length` | 2（自动） |
| `length` | -1 mm |
| `mode_theta` | 2（自动/透传） |
| `theta` | -1 |
| `d_base` | 头文件 1100 mm，标准 launch 的 YAML 覆盖为 800 mm |
| `d_work` | 650 mm |
| `base_z_offset` | 460 mm |
| `work_z_offset` | -300 mm |

## 13.7 面积语义

工字钢 `SprayInfo.area` 计算：

```text
切片数 × 切片长度_mm × 腹板高度_mm / 1e6
```

结果单位是 m²。它是正面矩形投影面积，不包含两侧翼缘的全部展开表面积。

## 13.8 启动参数

| 组 | 参数 | YAML 值 | 作用 |
|---|---|---:|---|
| ROI | `min_distance` / `max_distance` | 2.0 / 6.0 m | 雷达水平距离范围 |
| ROI | `min_height` / `max_height` | 1.5 / 3.5 m | Z 范围 |
| 预处理 | `voxel_size` | 0.02 m | 体素边长 |
| 预处理 | `sor_neighbors` | 20 | 统计滤波邻居数 |
| 预处理 | `sor_std_ratio` | 1.5 | 标准差倍率 |
| 去墙 | `wall_distance_threshold` | 0.08 m | 平面距离阈值 |
| 去墙 | `min_wall_points` | 500 | 最小墙面点数 |
| 去墙 | `max_wall_iterations` | 3 | 最大墙面轮数 |
| 聚类 | `cluster_tolerance` | 0.18 m | 欧式聚类容差 |
| 聚类 | `min_cluster_size` | 10 | 最小聚类点数 |
| 严格合并 | `merge_max_gap_strict` | 0.15 m | 最大间隙 |
| 严格合并 | `merge_max_height_diff_strict` | 0.10 m | 最大高度差 |
| 严格合并 | `merge_min_axis_similarity_strict` | 0.98 | 最小轴相似度 |
| 宽松合并 | `merge_max_gap_loose` | 0.50 m | 最大间隙 |
| 宽松合并 | `merge_max_height_diff_loose` | 0.15 m | 最大高度差 |
| 宽松合并 | `merge_min_axis_similarity_loose` | 0.96 | 最小轴相似度 |
| 候选判断 | `min_linearity` | 0.9 | PCA 线性度下限 |
| 候选判断 | `min_length` | 1.5 m | 最小长度 |
| 候选判断 | `min_aspect_ratio` | 2.5 | 最小长宽比 |
| 候选判断 | `max_axis_z` | 0.1 | 主轴 Z 分量上限 |
| 候选判断 | `min_width` / `max_width` | 0.1 / 1.2 m | 宽度范围 |
| 候选判断 | `min_thickness` / `max_thickness` | 0.03 / 0.8 m | 厚度范围 |
| 精修 | `refine_max_v_span` | 0.5 m | 翼缘方向最大跨度 |
| 精修 | `refine_min_v_span` | 0.1 m | 翼缘方向最小跨度 |
| 精修 | `refine_max_w_span` | 0.6 m | 截面高度最大跨度 |
| 精修 | `refine_min_w_span` | 0.05 m | 截面高度最小跨度 |
| 端部修剪 | `trim_outlier_ratio` | 1.8 | 尺寸异常倍率 |
| 端部修剪 | `trim_v_shift_threshold` | 0.08 m | v 中心偏移阈值 |
| 端部修剪 | `trim_w_shift_threshold` | 0.12 m | w 中心偏移阈值 |
| 端部修剪 | `trim_aspect_diff_ratio` | 0.5 | 长宽比偏差阈值 |
| 端部修剪 | `trim_density_ratio_high` | 2.5 | 密度上限倍率 |
| 端部修剪 | `trim_density_ratio_low` | 0.15 | 密度下限倍率 |
| 端部修剪 | `trim_score_ratio` | 0.15 | 截面分数衰减阈值 |
| 扩展 | `extend_min_target_length` | 3.5 m | 触发扩展的目标长度 |
| 扩展 | `extend_max_overshoot` | 1.0 m | 每侧最大超出距离 |
| 机械臂 | `d_base` | 800 mm | 基座到腹板表面的水平安装距离 |
| 机械臂 | `d_work` | 650 mm | 喷枪物理长度 + 喷涂挂距，仅用于可达性 |
| 机械臂 | `base_z_offset` | 460 mm | 基座中心与腹板中线高度差 |
| 机械臂 | `work_z_offset` | -300 mm | 最终工作点 Z 偏移 |

`d_base`、`d_work`、`base_z_offset`、`work_z_offset` 不在 `IBeamDetection.cfg` 中，只能通过参数服务器/YAML改变。

---

[← 返回 README](../README.md)
