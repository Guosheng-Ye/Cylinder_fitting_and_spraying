# 切片算法已知问题记录

---

## ISSUE-001：细圆柱 N=2 换位冗余问题

**记录日期**：2026-03-06
**相关日志**：`cal_logs/3-5/a1.txt`（粗圆柱）、`cal_logs/3-5/b1.txt`（细圆柱）
**状态**：待修复

---

### 问题描述

不同半径的圆柱（粗竖直 R=366mm / 细斜 R=89mm），同一高度层均输出 **2 个机械臂工作点位**（N=2）。

细圆柱的单次最大覆盖角 theta_max = **163.9°**，距目标覆盖角 180° 仅差 **16.1°**，但算法仍强制安排第二次换位，造成不必要的机器人重定位。

---

### 完整计算过程

#### 公共参数

```
R_arm    = 1500 mm
H        = 800 mm（delta_Z_max，工作高度差）
D_offset = 1100 mm（喷涂距离）
d_spray  = 300 mm（喷枪幅宽）

R_eff = sqrt(R_arm² - H²)
      = sqrt(1500² - 800²)
      = 1268.86 mm      ← 两种圆柱完全相同
```

#### Step 1：R_task

```
R_task = R_cyl + D_offset

粗圆柱(a1): R_task = 365.96 + 1100 = 1465.96 mm
细圆柱(b1): R_task =  88.52 + 1100 = 1188.52 mm
```

#### Step 2：D（虚拟基座到圆柱轴线水平距离）

```
粗圆柱(a1): D = 983.49 mm（竖直圆柱，D ≈ R_cyl + 600 + 几何偏差）
细圆柱(b1): D = 640.92 mm（倾斜 43°，轴线投影后 D 略小于 R_cyl+600=688.52mm）
```

#### Step 3：余弦定理求最大覆盖半角 α

三角形：`[虚拟基座] — [圆柱轴线投影] — [工作点]`，对应边：D、R_task、R_eff

```
cos α = (D² + R_task² - R_eff²) / (2 · D · R_task)

粗圆柱(a1):
  cos α = (983.49² + 1465.96² - 1268.86²) / (2 × 983.49 × 1465.96)
        = (967,253 + 2,148,039 - 1,609,984) / 2,883,666
        = 1,505,308 / 2,883,666 = 0.5224
  α = 58.51°  →  theta_max = 2α = 117.02°

细圆柱(b1):
  cos α = (640.92² + 1188.52² - 1268.86²) / (2 × 640.92 × 1188.52)
        = (410,779 + 1,412,420 - 1,609,984) / 1,524,427
        = 213,215 / 1,524,427 = 0.1399
  α = 81.95°  →  theta_max = 2α = 163.90°
```

#### Step 4：计算机器人位置数 N（问题根源）

```
N = ceil(180° / theta_max)

粗圆柱(a1): N = ceil(180 / 117.02) = ceil(1.538) = 2   ← 合理
细圆柱(b1): N = ceil(180 / 163.90) = ceil(1.098) = 2   ← 冗余换位
```

#### Step 5：N 均分后的每位置覆盖角和 k

```
coverage_per_position = 180° / N = 90°

粗圆柱(a1): w0_angle=23.48°, k = round(90/23.48) = 4 道/位置，总 8 道
细圆柱(b1): w0_angle=97.08°, k = round(90/97.08) = 1 道/位置，总 2 道
```

---

### 根本原因

算法使用**均分策略**：固定将 180° 等分给 N 个位置，不考虑单次实际覆盖余量。

对细圆柱（b1）：
```
位置1 实际能力: 163.90°，但只分配了 90°（浪费 73.9° 能力）
位置2 实际能力: 163.90°，但只需补充 16.1°
→ 为补 16.1° 触发一次完整换位，不合理
```

单次覆盖 ≥ 180° 的几何条件：
```
要使 theta_max ≥ 180°（即 cos α ≤ 0），需满足：
  D² + R_task² ≤ R_eff²
  即 sqrt(D² + R_task²) ≤ R_eff = 1268.86 mm

细圆柱(b1): sqrt(640.92² + 1188.52²) = 1373.2 mm > 1268.86 mm  ✗
→ 当前参数下，单次完整覆盖几何上不可能
```

即便如此，theta_max=163.9° 已高度接近 180°，强制均分为 2×90° 是次优策略。

---

### 建议修改方向

在 `cylinder_fitting_ros.cpp` 的切片优化算法中，增加**近似单次覆盖**判断逻辑：

```
当前逻辑:
  N = ceil(180° / theta_max)

建议逻辑:
  gap = 180° - theta_max
  if gap <= NEAR_FULL_COVERAGE_THRESHOLD_DEG:
      N = 1（直接截断到 180°，允许轻微超量覆盖）
  else:
      N = ceil(180° / theta_max)
```

- `NEAR_FULL_COVERAGE_THRESHOLD_DEG` 建议初始值：**20°**（即 theta_max ≥ 160° 时 N=1）
- 此时细圆柱从 2 次换位减少为 1 次，k=1，单次喷涂覆盖 180°（略超实际能力 163.9°，需下游截断处理）

---

### 影响范围

- 文件：`src/pcl_process/src/cylinder_fitting/cylinder_fitting_ros.cpp`
- 函数：切片优化算法（`calculate_max_slice_angle` 之后的 Step1）
- 关联参数：无新增参数（阈值可后续通过 YAML 配置化）

---

## ISSUE-002：圆柱检测单次触发后重复发布结果

**文件**：`src/pcl_process/src/cylinder_fitting/cylinder_fitting_ros.cpp`
**状态**：已修复
**发现日期**：2026-03-10
**修复日期**：2026-03-10

---

### 现象

主控发送 `achieve_work_pose = true` 触发一次检测，收到结果后立即发送 `false`，
但程序在停止前又额外进行了若干次检测并重复发布结果。

### 根本原因：ros::spin() 单线程回调阻塞

`ros::spin()` 是单线程模型，所有回调（`cloudCallback`、`mainCtlCallback` 等）共用同一线程，**一次只能执行一个回调**。

`fitter_->run_with_rslidar()` 内含 RANSAC，耗时约 200~500ms。
在此期间，主控回复的 `false` 消息虽然已到达网络层，但因线程被 `cloudCallback` 占用而进入 ROS 消息队列等待。
下一帧点云也同样进入队列。`cloudCallback` 返回后，ROS 按队列顺序调度：
- 若下一帧点云先于 `false` 被调度 → 再次检测、再次发布
- `false` 回调姗姗来迟才真正停止

时序如下：
```
主控              ROS消息队列              单线程执行
 │─true────────>   │                       │
 │                 │─mainCtlCallback(true)─>│ processing_=true
 │                 frame1入队              │
 │                 │─cloudCallback(f1)────>│ RANSAC(200-500ms 阻塞)
 │                 frame2入队              │ ← false消息此时入队
 │ <─结果#1 pub────────────────────────────│ 检测完，发布
 │─false────────>  │                       │
 │                 │─cloudCallback(f2)────>│ processing_仍true! 再次检测
 │                 │─mainCtlCallback(false)>│ 此时才 processing_=false
 │                 │─cloudCallback(f3)────>│ 终于停止
```

### 修复方案

在 `cloudCallback` 检测成功后、发布结果前立即将 `processing_` 置为 false，
实现"单次触发"语义，不依赖外部 false 信号的到达时机。

**修改位置**：`cylinder_fitting_ros.cpp` 的 `cloudCallback()`，
在 `fitter_->run_with_rslidar()` 成功分支开头添加：

```cpp
// 立即停止，防止 ros::spin() 单线程延迟导致队列中的下一帧触发重复检测
processing_.store(false, std::memory_order_release);
```

### 注意事项
- 停止标志应在发布**之前**设置，确保下一帧点云到来时已经读到 false
- 此方案实现"每次 true 命令只执行一次有效检测"的单次触发语义
- 若未来需要连续检测模式，可通过参数区分两种模式

---

## ISSUE-003：preset_D 斜圆柱检测失败（验证阈值不适配）

**文件**：`src/pcl_process/config/cylinder_detection_params.yaml` → `preset_D`
**状态**：待修复
**发现日期**：2026-03-10
**日志**：`src/pcl_process/src/cal_logs/3-10/D.txt`
**rosbag**：`ros_bag/2026-03-10-14-25-44.bag`

---

### 现象

切换到工作对象 D 后，持续输出 `No valid cylinder model found`，始终无法识别斜圆柱并发布结果。

---

### 日志关键信息

```
# RANSAC 参数（来自 preset_D）
RANSAC input: ~2900 pts, axis=[0.707,0.000,0.707], dist_thresh=0.0250, eps_angle=20.0, R=[0.040-0.180]

# 大多数帧：RANSAC直接找不到内点
No cylinder found in filtered cloud (RANSAC returned 0 inliers).

# 少数帧（2帧）：RANSAC找到了内点，但验证被拒绝
RANSAC raw result: 153 inliers, R=0.0827m, axis=[0.623,0.227,0.749], tilt=41.5deg
  Height:       0.09 m [0.50-???]  → FAIL
  Aspect ratio: 0.55  (H/D) [>3.0] → FAIL
Cylinder REJECTED: Height too small

RANSAC raw result: 154 inliers, R=0.0826m, axis=[-0.746,-0.310,-0.590], tilt=53.9deg
  Height:       0.13 m [0.50-???]  → FAIL
  Aspect ratio: 0.78  (H/D) [>3.0] → FAIL
Cylinder REJECTED: Height too small
```

---

### 根本原因分析

#### 原因1（主要）：`min_height_m=0.50` 过大，与斜圆柱实际可见段不符

RANSAC 实际已找到正确圆柱（R≈83mm，轴向 41.5°，均在合理范围），
但内点沿轴向的跨度仅 0.09~0.13m，被 `min_height_m=0.50` 拒绝。

斜圆柱（约43°倾斜）在 LiDAR 视角下截面面积小、可见段短，
RANSAC 找到的内点集往往只覆盖圆柱的局部一小段，沿轴向高度天然偏小。

#### 原因2（主要）：`min_aspect_ratio=3.0` 过严

```
aspect_ratio = H / (2*R) = 0.09 / (2*0.083) = 0.54   →  远低于阈值 3.0
```

对细圆柱（R=89mm）短段识别而言，此阈值等同于要求 H≥0.498m，
与原因1 形成双重拦截，导致所有有效候选均被拒绝。

#### 原因3（次要）：`ransac_distance_threshold=0.025m` 相对圆柱半径偏大

```
dist_thresh / R = 0.025 / 0.083 ≈ 30%
```

阈值占半径的30%，偏大。易将非圆柱点纳入考虑，降低 RANSAC
采样命中有效内点集的概率，导致大多数帧直接返回 0 内点。

#### 原因4（次要）：`eps_angle_deg=20°` 可能不够宽松

大多数帧 RANSAC 返回 0 内点，可能轴向偏差在部分帧超过 20°。
实际已检出帧的轴向为 [0.623, 0.227, 0.749]，Y 分量=0.227，
偏离参考轴 [0.707, 0.0, 0.707] 约 13°，尚在范围内，
但建议适当放宽以提高命中率。

---

### 修改建议（仅修改 `preset_D` 部分，不影响 preset_A）

| 参数 | 当前值 | 建议值 | 原因 |
|------|--------|--------|------|
| `min_height_m` | 0.50 | 0.08 | 实测高度约 0.09m，降低阈值使有效候选通过验证 |
| `min_aspect_ratio` | 3.0 | 0.5 | 细圆柱短段长宽比天然小，放宽限制 |
| `ransac_distance_threshold` | 0.025 | 0.015 | 减小阈值降低误匹配，提高 RANSAC 命中概率 |
| `eps_angle_deg` | 20.0 | 30.0 | 增大轴向容差，提高部分帧的内点命中率 |

**配置文件修改位置**：`src/pcl_process/config/cylinder_detection_params.yaml` → `preset_D` 节

---

### 注意事项

- `min_height_m` 降低后，需确保验证中的 `min_aspect_ratio` 同步降低，否则仍会被拦截
- `ransac_distance_threshold` 减小后若内点数不足（<30），需同步确认 `min_inliers` 合理
- 建议用 rosbag `2026-03-10-14-25-44.bag` 回放验证修改效果

---

### 2026-03-10 实机 rosbag 离线分析更新

**rosbag**：`ros_bag/2026-03-10-14-25-44.bag`
**分析脚本**：`extract_bag_to_pcd.py` + 离线 RANSAC

#### 实测参数（4帧一致性验证）

| 帧 | R (mm) | tilt (°) | 轴向 | 中心 (m) |
|----|--------|----------|------|----------|
| 1 | 121.3 | 43.0 | [0.682, 0.031, 0.731] | [-2.629, 5.002, 3.741] |
| 2 | 121.0 | 43.2 | [0.684, 0.028, 0.729] | [-2.636, 5.004, 3.748] |
| 3 | 121.5 | 43.1 | [0.683, 0.029, 0.730] | [-2.634, 5.003, 3.746] |
| 4 | 120.8 | 43.1 | [0.683, 0.028, 0.730] | [-2.632, 5.002, 3.744] |
| **均值** | **121.2** | **43.1** | **[0.683, 0.029, 0.730]** | **[-2.633, 5.003, 3.745]** |

与之前离线分析基准（R≈89mm）差异较大，以实机 bag 数据为准。

#### 在线检测失败根因分析

1. **近处假目标干扰**：RANSAC 总是收敛到雷达正下方的假结构（XY距离≈0.1m），真实目标距雷达 XY≈5.65m
2. **真实目标点数过少**：去除平面后残余点约 2900 点，其中斜圆柱仅约 100~150 点，占比 ~5%
3. **原 preset_D 半径范围偏小**：原 `[0.04, 0.18]` 对应 R≈89mm 基准，实测 R≈121mm 超出范围

#### 已完成修复

**1. 增加 `min_center_xy_dist_m` 参数过滤近处假目标**

- **位置**：hpp（结构体字段）、cpp（验证逻辑）、ros.hpp（缓存变量）、ros.cpp（loadParams/switchPreset/applyFittingParams）、yaml（preset_D: 2.0）
- **效果**：过滤 XY<2m 的假目标，真实斜圆柱 XY≈5.65m 不受影响

**2. 更新 preset_D 参数（基于实测 R≈121mm）**

| 参数 | 旧值 | 新值 | 原因 |
|------|------|------|------|
| `min_radius_m` | 0.04 | 0.06 | R×0.5≈61mm |
| `max_radius_m` | 0.18 | 0.25 | R×2≈242mm |
| `ransac_distance_threshold` | 0.015 | 0.018 | 基于新 R 的推荐值 |
| `min_center_xy_dist_m` | (新增) | 2.0 | 过滤近处假目标 |

#### 待验证

- 用 rosbag `2026-03-10-14-25-44.bag` 在线回放，确认 RANSAC 能成功检测到 XY≈5.65m 的真实目标
- 若真实目标点数仍不足（<30 内点），可能需要进一步调整 `ransac_max_iterations` 或前端点云滤波参数

---

## ISSUE-004：工字钢扩展后端部稀疏切片未被裁剪

**文件**：`src/pcl_process/src/ibeam_fitting/ibeam_fitting_node.cpp`
**状态**：已修复
**发现日期**：2026-03-10
**修复日期**：2026-03-10

---

### 现象

工字钢切片扩展后，最右端 Slice #8 内无工字钢点云（仅 3~6 个散点/噪声），但仍被保留。
最终检测长度 3.58m，右端约 0.4m 是无效区域。

---

### 根本原因

`checkSliceAbnormal` 的异常判定只检测两种**密度过高**的情况：
1. **圆柱特征**：`density_abnormal (cnt > typ_density*2.0) && distribution_abnormal (mid_v_frac > 0.35)`
2. **尺寸+密度异常**：`size_abnormal && density_abnormal`

两种判定都要求 `cnt > typ_density * 2.0`（密度过高），**完全没有检测密度过低**的情况。

右端扩展切片实际情况：
- 209 点 / 23 切片 ≈ 9 点/slice 典型密度
- 右端扩展区域仅 3~6 个散点（噪声/圆柱边缘）
- `cnt=4, typ_density=9 → 4 > 9*2.0 = false → density_abnormal=false → 不裁剪`

已有的 `cnt < 3 → return true` 防护层无法覆盖 cnt=3~6 的稀疏切片。

---

### 修复方案

在 `checkSliceAbnormal`（L2126-2131）增加 `sparse_abnormal` 稀疏判定：

```cpp
bool cylinder_like = density_abnormal && distribution_abnormal;
bool sparse_abnormal = typ_density > 0 && cnt < typ_density * 0.4;
bool abnormal = cylinder_like || (size_abnormal && density_abnormal) || sparse_abnormal;
```

阈值 `cnt < typ_density * 0.4`（低于典型密度 40%）：
- 典型 9 点/slice → 阈值 3.6 → cnt≤3 被裁剪
- 保守阈值，避免误裁正常的稍稀疏切片
- 配合已有的 `cnt < 3` 防护，形成两层端部裁剪机制

同时更新日志输出，区分"密度过高"和"密度过低"标识。

---

### 影响范围

- 仅影响扩展后端部切片的扫描判定，中间区域不参与端部扫描
- 正常端部切片（密度接近典型值）不受影响

---

## ISSUE-005：工字钢左端 1-2 切片漏检（ROI 距离截断）

**文件**：`src/pcl_process/src/ibeam_fitting/ibeam_fitting_node.cpp`
**状态**：已修复
**发现日期**：2026-03-10
**修复日期**：2026-03-10

---

### 现象

检测出的工字钢（2.77×0.21×0.40m，7 切片），左侧仍有 1-2 片可见的工字钢点云未被包含在检测结果中。

---

### 关键日志数据

```
聚类 #0: 212 点
  中心: (-1.89794, 4.91839, 1.80947), 距离=5.27188m
  尺寸: 2.81792 x 0.445741 x 0.29423 m

精细化完成: 212 -> 58 点
精细尺寸: 0.469058 x 0.174307 x 0.388048 m

扩展成功：0.469058m -> 2.76785m（右端裁剪后）
  Slice #0 center=(-3.127, 4.926, 1.816)  ← 最左端，XY距离≈5.94m
```

---

### 根本原因：ROI `max_distance=6.0m` 截断了工字钢左端

ROI 过滤使用 XY 距离（`filterByDistanceAndHeight`，L397-399）：
```cpp
double dist = std::sqrt(pt.x * pt.x + pt.y * pt.y);
if (dist > min_distance && dist < max_distance && pt.z > min_height)
```

工字钢位于 y≈4.92m，检测左边缘处（x≈-3.33）的 XY 距离：
- Slice #0 左边缘 (-3.33, 4.92): `√(3.33²+4.92²) = 5.94m` ← 距 6.0m 仅剩 0.06m 余量

再向左 1-2 个切片（x≈-3.7~-4.1）的距离：
- (-3.73, 4.92): `√(3.73²+4.92²) = 6.17m` ← **超出 6.0m，被 ROI 过滤**
- (-4.13, 4.92): `√(4.13²+4.92²) = 6.42m` ← **超出 6.0m，被 ROI 过滤**

---

### 影响链路

```
左端工字钢点 (XY距离 6.0~6.5m)
    ↓ ROI过滤 (max_distance=6.0)
  被丢弃，不进入 roi_cloud_
    ↓
  不参与聚类 → 原始聚类缺少左端
    ↓
  扩展搜索 roi_cloud → 左端找不到点
    ↓
  检测结果缺少左端 1-2 切片
```

---

### 修复方案：扩展搜索使用 `filtered_cloud_`（post-wall-removal cloud）

在 `extendShortIBeams`（L1996）中，将搜索池从 `roi_cloud` 改为 `filtered_cloud_`（墙面移除后、ROI 过滤前的点云），跳过 `max_distance` 限制，但保留 `min_distance` 和 `min_height` 约束。

```cpp
// 修改前：
for (const auto &pt : roi_cloud->points) { ... }

// 修改后：
const auto &search_cloud = filtered_cloud_ ? filtered_cloud_ : roi_cloud;
for (const auto &pt : search_cloud->points) {
    if (pt.z < detection_params_.min_height) continue;
    double xy_dist = std::sqrt(pt.x * pt.x + pt.y * pt.y);
    if (xy_dist < detection_params_.min_distance) continue;
    // ... 原有 u/v/w 几何约束不变
}
```

**优势**：
- 不改变全局 ROI（聚类/精细化阶段保持 6.0m，噪声控制不变）
- 仅扩展阶段解除距离限制，精准定位
- 扩展已有 v/w 容差 + u 范围约束，不会引入远处噪声
- `filtered_cloud_`（1412 点）比 `roi_cloud`（586 点）多 826 点，但经几何约束后只有工字钢远端点能通过

### 效果预估

- 新增左端 ROI 外的工字钢点（距离 6.0~6.5m）
- 检测长度：2.77m → ~3.2m（+1-2 个切片）
- 右端不受影响（右端点距 LiDAR 更近，始终在 ROI 内）

---

### 影响范围

- 仅影响 `extendShortIBeams` 的搜索池，聚类和精细化阶段不受影响
- `filtered_cloud_` 已有墙面移除，扩展的几何约束进一步限制搜索范围

---

## ISSUE-006：工字钢检测两端侵入非工字钢物体（精细化失败 + 扩展过度）

**文件**：`src/pcl_process/src/ibeam_fitting/ibeam_fitting_node.cpp`、`ibeam_fitting_node.hpp`
**状态**：已修复
**发现日期**：2026-03-11
**修复日期**：2026-03-11

---

### 现象

检测结果 **4.59×0.41×0.38m（12 切片）** 远大于真实工字钢（约 2.8m），两端侵入了非工字钢物体：
- **右侧 #8~#11**：侵入了右侧的竖直圆柱/管道结构（白色点云弧形）
- **左侧 #0~#1**：侵入了左端超出工字钢的区域

---

### 两份日志对比

| | log.txt（较好结果） | logimg.txt（问题结果） |
|--|--|--|
| 聚类 | 212点, 2.82m | 212点, 2.83m |
| 精细化 | **成功** → 58点, 0.47m | **失败** → 间隙1.42m, 回退原始 |
| 扩展后 | 0.47m → 3.58m (9片) | 2.83m → **4.59m** (12片) |
| 问题 | 右端略有溢出 | **两端严重侵入** |

---

### 根因分析

#### 根因1（核心）：精细化失败后 Fallback 使用完整原始聚类

精细化检测到 **1.42m 间隙**（说明聚类包含两段不连续的结构），分割后右段 63 点评分仅 2/6，验证失败。

但 Fallback 条件过于宽松（`width < 0.50 && wh_ratio ∈ [0.5, 3.0]`），原始聚类 W=0.44m, W/H=1.53 直接通过，**包含间隙两侧不相关结构的 212 点原始聚类被当作完整工字钢**。

#### 根因2：扩展容差来自未精细化的原始聚类

精细化失败后，`beam.dimensions` = 原始聚类 OBB 尺寸（2.83 × 0.44 × 0.29），扩展容差：
- `v_tolerance = max(0.15, 0.44 * 0.6) = 0.264m`（过宽）

精细化成功时（log.txt）容差仅 0.15m。**0.264m vs 0.15m 的差异直接导致圆柱点被纳入。**

#### 根因3：扩展搜索无长度上限

原始聚类已 2.83m，扩展后到 4.59m（增加 62%），无上限约束。

#### 根因4：`checkSliceAbnormal` 端部裁剪门槛过高

- 密度检测 `cnt > typ_density * 2.0`：圆柱端部 1.2~1.8 倍密度不触发
- 分布检测要求 `cnt≥8`：端部切片点少于 8 个时跳过
- 密度与分布必须同时满足才判定圆柱特征：任一不触发就放行

---

### 修复方案（5 处修改）

#### 修改1：精细化失败时连续性分割（L187-216）— 最关键

精细化失败的 Fallback 不再直接使用完整原始聚类，而是对原始聚类做连续性分割，只保留最大连续段。

新增逻辑 `extractLargestContinuousSegment`（内联实现）：
1. 将点投影到 main_axis 上，按 u 坐标排序
2. 检测相邻点间距 > 0.20m 的断裂处
3. 返回点数最多的连续段
4. 基于连续段重新计算中心和 OBB 尺寸

**效果**：包含 1.42m 间隙的聚类被分割为两段，仅保留较大段（工字钢本体），排除间隙后的不相关结构。

#### 修改2：扩展 v/w 容差增加上限约束（L1964-1968）

```cpp
// 修改前:
double v_tolerance = std::max(0.15, width * 0.6);
double w_tolerance = std::max(0.20, height * 0.6);

// 修改后:
double v_tolerance = std::min(std::max(width * 0.6, 0.15), 0.20);   // 上限 0.20m
double w_tolerance = std::min(std::max(height * 0.6, 0.20), 0.30);  // 上限 0.30m
```

工字钢翼缘宽度通常 ≤0.30m，v_tolerance 上限 0.20m 足以覆盖半翼缘宽度，同时排除更远处的圆柱点。

#### 修改3：增加扩展长度上限（~L2033）

```cpp
double orig_cluster_length = beam.original_dimensions[0];
double max_allowed = std::max(orig_cluster_length * 1.5,
                              detection_params_.extend_min_target_length);
```

超出上限时从两端等比裁剪。基于**原始聚类长度**（非精细化后长度）计算上限，确保精细化成功时（0.47m → 3.58m）不被误限。

#### 修改4：增强 `checkSliceAbnormal` 端部裁剪（L2094-2153）

三处增强：
- **密度异常阈值**：`2.0` → `1.5`（降低门槛，更容易检测圆柱端部）
- **分布检测最低点数**：`8` → `5`（允许少量点的切片参与分布检测）
- **新增强分布异常独立判定**：`mid_v_frac > 0.5` 时直接触发裁剪（不依赖密度异常）

```cpp
bool strong_distribution = mid_v_frac > 0.5;
bool abnormal = cylinder_like || (size_abnormal && density_abnormal)
                || sparse_abnormal || strong_distribution;
```

#### 修改5：`IBeamResult` 新增 `original_dimensions` 字段

**文件**：`ibeam_fitting_node.hpp`

```cpp
Eigen::Vector3d original_dimensions;  // 原始聚类 OBB 尺寸（扩展长度限制用）
```

在精细化成功和失败两处结果赋值时均记录 `result.original_dimensions = dimensions`。
精细化成功时 `dimensions` = 原始聚类 OBB（不是精细化后的），确保扩展长度上限基于正确基准。

---

### 影响范围

- **精细化成功场景**（log.txt）：连续性分割不触发（精细化分支不走 fallback），容差上限可能稍收紧 v_tolerance（0.15→0.15 不变或微调），扩展长度上限 = max(2.82*1.5, 3.5) = 4.23m（当前 3.58m 不受限），端部裁剪更敏感可能多裁 0~1 片
- **精细化失败场景**（logimg.txt）：连续性分割将 212 点缩减到工字钢本体段，容差收紧到 0.20m，扩展上限 ~4.2m，端部裁剪增强。预期检测长度从 4.59m 缩短到 ~2.8m

### 验证方法

1. `catkin_make --pkg pcl_process` 编译通过
2. 回放 rosbag 观察：
   - 精细化失败场景：检测长度应从 4.59m 缩短到 ~2.8m
   - 精细化成功场景：检测长度应保持 ~2.8-3.2m，不被新限制影响
   - 右端不应侵入圆柱结构
   - 左端正常工字钢点不被误裁

---

## ISSUE-007：雷达高度变化导致工字钢长度检测不足

**文件**：`src/pcl_process/src/ibeam_fitting/ibeam_fitting_node.cpp` — `refineIBeamDetection()` 函数
**状态**：参数调优中
**发现日期**：2026-03-10
**日志**：`src/pcl_process/src/ibeam_fitting/log.txt`

---

### 现象

同一根工字钢（截面 W=0.17m, H=0.35m 完全一致），仅因雷达安装高度不同，检测结果差异显著：

| 对比项 | Bag 1 (3-10, 高位雷达) | Bag 2 (1-13, 低位雷达) |
|--------|----------------------|----------------------|
| 检测长度 | **2.19m** | **2.93m** |
| 点数 | 134 (原211) | 226 (原362) |
| 切片数 | 6 | 8 |
| 喷涂面积 | 0.76 m² | 1.04 m² |

长度差 0.74m（25%），切片少 2 个，面积少 27%。

### 根本原因：精细化阶段多轮端部裁剪过于激进

高位雷达 → 俯视角增大 → 端部扫描线更少、点云更稀疏 → 被精细化阶段误判为异常并裁掉。

处理链路中存在 **3 次串联裁剪**，每轮都会缩短长度：

#### 第 1 轮：截面尺寸精化过滤（~L1240-1323）
- 按 0.15m 步长切片，检查 v_span/w_span 是否在 `[refine_min_v_span, refine_max_v_span]` 范围内
- **问题**：端部稀疏时 v_span 天然偏小，`refine_min_v_span=0.30` 过大，端部整片被丢弃

#### 第 2 轮：端部尺寸异常检测（~L1362-1521）— 最严重
- 以中间 50% 切片的 diameter/v_span/w_span 中位数为"典型值"
- 端部任一指标超过 `典型值 × trim_outlier_ratio(1.3)` 即裁掉
- **问题**：1.3 倍阈值过严，高位雷达端部截面形状偏离合理范围内即被误裁

#### 第 3 轮：端部形状一致性检测（~L1523-1720）— 二次裁剪
- H 型分数 < `中间段 × trim_score_ratio(0.3)` → 裁
- v/w 中心偏移 > 5cm/8cm → 裁
- 点密度比 < 0.25 → 裁
- **问题**：LiDAR 单视角下端部 H 型特征天然退化，多个条件叠加导致再次误裁

#### 扩展阶段未能补救
- Bag1 检出 2.19m < `extend_min_target_length=3.5m`，触发扩展
- 但扩展后仍执行端部裁剪（同样阈值），扩展出的点再次被裁掉

### 已完成：参数调优（第一轮）

修改文件：`config/ibeam_detection_params.yaml`

| 参数 | 旧值 | 新值 | 原因 |
|------|------|------|------|
| `refine_min_v_span` | 0.30 | 0.10 | 端部稀疏 v_span 天然偏小，过大丢弃端部整片 |
| `trim_outlier_ratio` | 1.3 | 1.8 | 端部截面与中间段差异合理范围更大 |
| `trim_v_shift_threshold` | 0.05 | 0.08 | LiDAR 噪声可达 5cm，需放宽 |
| `trim_w_shift_threshold` | 0.08 | 0.12 | 同理放宽 w 方向容差 |
| `trim_density_ratio_low` | 0.25 | 0.15 | 端部密度天然低于中间段 |
| `trim_score_ratio` | 0.3 | 0.15 | H 型分数在端部可见性差时天然低 |

### 待验证

- 用两个 rosbag 分别回放，对比调参前后的检测长度
- 确认 Bag1（高位雷达）长度是否接近 Bag2（低位雷达）的 2.93m
- 观察是否引入误检（非工字钢结构被保留）

### 2026-03-10 修复：左侧漏检 + 右侧入侵圆柱

**日志**：`ibeam_fitting/logimg.txt` + `logimg.png`

**现象**：
- 检测结果 3.59×0.40×0.39m，9 切片
- 左侧漏检：#0 切片区域无红色检测点，工字钢实际延伸更远
- 右侧入侵：#7、#8 切片侵入右侧圆柱/弧形点云

**根因分析**：

A. **右侧入侵圆柱**（主要）：`extendShortIBeams()` 扩展仅检查 v/w 截面容差，
   圆柱截面恰好与工字钢兼容。端部裁剪仅检查 v_span/w_span > 典型值×1.3，
   但圆柱 v_span/w_span 与工字钢相近。**缺少点密度检查**。

B. **左侧漏检**：精细化 `v_tolerance = typical_v_span * 0.4 + 0.025` 过紧，
   对于 typical_v_span=0.145m，v_tol 仅 0.083m，丢弃 59% 的点，
   制造 1.52m 伪间隙，左端 7 点被丢弃。回退后扩展主要向右延伸。

**修复内容**：

1. `extendShortIBeams()` 端部裁剪增加逐切片点密度检查：
   - 中间段收集 `mid_counts`，计算典型密度 `typ_density`（中位数）
   - 左端/右端判定增加 `density_abnormal = cnt > typ_density * 3.0`
   - 圆柱密度远超工字钢 3 倍，被识别为异常裁掉

2. `refineIBeamDetection()` 精细化 v_tolerance 放宽：
   - `typical_v_span * 0.4 + 0.025` → `typical_v_span * 0.6 + 0.04`
   - v_tol: 0.083m → 0.127m (+53%)，保留更多端部点，消除伪间隙

### 潜在代码层面优化（待后续确认）

若参数调优效果不足，可考虑以下代码修改：

1. **端部裁剪逻辑改为渐进式容差**：越靠近端部容差越大，而非统一阈值
2. **扩展后跳过端部裁剪**：扩展阶段搜到的点已经过截面验证，不应再被裁剪
3. **基于距离的自适应阈值**：根据工字钢到雷达的距离动态调整精细化参数

---

## 代码审查建议（ibeam_fitting_node.cpp）

以下为代码审查中发现的潜在问题和改进建议。

### CR-001：变量重定义（编译错误）
**位置**: `sliceIBeam` 函数 (约 1225-1235 行)

代码中存在变量 `w` 和 `v` 的重复定义，是代码修改或合并时的残留错误，会导致编译失败。

**建议**：删除旧定义，保留修正后的逻辑：
```cpp
Eigen::Vector3d u = main_axis;
Eigen::Vector3d w_global(0, 0, 1);
Eigen::Vector3d v = w_global.cross(u);
if (v.norm() < 1e-6) {
    v = Eigen::Vector3d(0, 1, 0);
}
v.normalize();
Eigen::Vector3d w = u.cross(v).normalized();
```

### CR-002：墙面移除策略过于激进
**位置**: `removeWallPlanes` 函数

判定条件 `bool is_wall = is_vertical_plane && z_span > 1.0 && mean_xy_dist > 4.0;` 存在风险。如果大型工字钢（高度>1m）距离雷达超过 4.0m，其腹板会被误判为墙面。

**建议**：移除硬编码距离阈值或显著增大（>10m），增加对平面水平跨度的检查。

### CR-003：硬编码的分层聚类高度
**位置**: `IBeamFitter` 构造函数

`height_layers_` 使用硬编码绝对 Z 值（`1.5~2.5`, `2.5~4.0` 等），存在漏检和切断风险。

**建议**：提取到 ROS 参数服务器，或改用不依赖绝对高度的整体聚类方法。

### CR-004：切片中心位于内部而非表面
**位置**: `sliceIBeam` 函数

切片中心基于几何中心（Centroid）计算，质心位于腹板内部（厚度中心），TCP 到表面距离会比预期小半个腹板厚度。

**建议**：沿法向量将中心点投影到腹板表面：
```cpp
Eigen::Vector3d surface_point = slice_center_m + normal * (web_thickness / 2.0);
```

### CR-005：双翼缘检测对遮挡敏感
**位置**: `hasDoubleFlangeStructure` 函数

算法依赖直方图出现两个峰值（`peaks.size() == 2`），远端翼缘经常被腹板遮挡导致只有一个峰值。

**建议**：放宽条件，允许单峰（单侧翼缘可见），前提是线性度和截面长宽比特征显著。

### CR-006：连续性检测阈值固定
**位置**: `checkPointCloudContinuity` 函数

`max_absolute_gap` 固定为 0.15m，远距离或稀疏点云间隙可能超过此值，导致完整工字钢被错误切断。

**建议**：根据点云平均密度或距离动态调整 `max_absolute_gap`。
