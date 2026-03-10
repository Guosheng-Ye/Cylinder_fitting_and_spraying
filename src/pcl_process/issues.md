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
