# IBeam Fitting Algorithm Issues & Suggestions

本文档记录了 `ibeam_fitting_node.cpp` 中存在的潜在问题、Bug 以及改进建议。

## 1. 严重编译错误：变量重定义
**位置**: `sliceIBeam` 函数 (约 1225-1235 行)

**问题描述**:
代码中存在变量 `w` 和 `v` 的重复定义。这看起来是代码修改或合并时的残留错误，会导致编译失败。

```cpp
// 当前代码片段
Eigen::Vector3d u = main_axis;
Eigen::Vector3d w(0, 0, 1); // <--- 定义 1
Eigen::Vector3d v = w.cross(u); // <--- 定义 1
Eigen::Vector3d w_global(0, 0, 1); 
Eigen::Vector3d v = w_global.cross(u); // <--- 重定义错误！
// ...
Eigen::Vector3d w = u.cross(v).normalized(); // <--- 重定义错误！
```

**建议更改**:
删除旧的定义，保留修正后的逻辑，确保构建正交坐标系。
```cpp
Eigen::Vector3d u = main_axis;
Eigen::Vector3d w_global(0, 0, 1); 
Eigen::Vector3d v = w_global.cross(u);
if (v.norm() < 1e-6) {
    v = Eigen::Vector3d(0, 1, 0); // 处理垂直情况
}
v.normalize();
// 构建局部正交坐标系，w 垂直于 u 和 v
Eigen::Vector3d w = u.cross(v).normalized();
```

## 2. 墙面移除策略过于激进 (Wall Removal Risk)
**位置**: `removeWallPlanes` 函数

**问题描述**:
判定条件 `bool is_wall = is_vertical_plane && z_span > 1.0 && mean_xy_dist > 4.0;` 存在风险。
该逻辑假设“墙面”一定在 4.0 米以外。如果大型工字钢（高度>1m）距离雷达超过 4.0 米，其腹板会被误判为墙面并被直接移除，导致检测完全失败。

**建议更改**:
1. **移除硬编码距离阈值**：或者将其显著增大（例如 > 10m）。
2. **增加特征判断**：墙面通常在水平方向（XY平面）延伸很广，而工字钢腹板宽度有限（通常 < 1.0m）。可以增加对平面水平跨度的检查。

## 3. 硬编码的分层聚类高度 (Hardcoded Layering)
**位置**: `IBeamFitter` 构造函数

**问题描述**:
`height_layers_` 初始化使用了硬编码的绝对 Z 值 (`1.5~2.5`, `2.5~4.0` 等)。
1. **漏检风险**：不在这些高度范围内的工字钢将被忽略。
2. **切断风险**：跨越层边界的工字钢会被切成两段，依赖后续合并逻辑，增加了不稳定性。

**建议更改**:
将分层参数提取到 ROS 参数服务器中，或者改用不依赖绝对高度的整体聚类方法（如欧式聚类 + 区域生长）。

## 4. 切片中心位于内部而非表面 (Slice Center Depth)
**位置**: `sliceIBeam` 函数

**问题描述**:
切片中心 `slice_center_m` 是基于工字钢的几何中心（Centroid）计算的。对于工字钢，质心位于腹板内部（厚度中心）。如果下游路径规划直接将此点作为目标点，实际 TCP 到表面的距离会比预期小半个腹板厚度。

**建议更改**:
明确 `slice_center` 的定义。如果是为了喷涂腹板表面，应沿法向量将中心点投影到腹板表面。
```cpp
// 假设 normal 指向机器人一侧
Eigen::Vector3d surface_point = slice_center_m + normal * (web_thickness / 2.0);
```

## 5. 双翼缘检测对遮挡敏感 (Occlusion Sensitivity)
**位置**: `hasDoubleFlangeStructure` 函数

**问题描述**:
算法依赖直方图出现两个峰值 (`peaks.size() == 2`)。在实际扫描中，由于视角原因，远端的翼缘经常被腹板遮挡，导致点云缺失，直方图可能只有一个峰值。

**建议更改**:
放宽条件，允许单峰（单侧翼缘可见），前提是线性度和截面长宽比特征非常显著。

## 6. 连续性检测阈值固定
**位置**: `checkPointCloudContinuity` 函数
**问题描述**: `max_absolute_gap` 固定为 0.15m。对于远距离或稀疏点云，扫描线之间的间隙可能超过此值，导致完整的工字钢被错误切断。
**建议更改**: 根据点云的平均密度或距离动态调整 `max_absolute_gap`。