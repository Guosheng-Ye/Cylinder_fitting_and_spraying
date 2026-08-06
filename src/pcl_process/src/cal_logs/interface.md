# 圆柱拟合与切片算法接口文档

> 最后更新: 2025年12月23日

---

## 一、ROS 消息接口参数

### 1.1 ProcessParam 消息（输入/输出）

| 字段名 | 类型 | 单位 | 说明 |
|--------|------|------|------|
| `distance` | float64 | mm | 喷涂工作点与圆柱表面的距离（喷涂距离） |
| `step_width` | float64 | mm | 喷幅宽度 d（喷枪单次喷涂宽度） |
| `step_height` | float64 | mm | 步进高度（预留，当前未使用） |
| `height` | int16 | mm | 切片高度（工作区总高度） |
| `lap` | float64 | - | 搭接比例 r（0~1之间），如0.5表示50%重叠 |
| `arm_reach` | int16 | mm | 机械臂最大臂展 R_arm |
| `mode` | int16 | - | 工作模式：1=手动，2=自动 |
| `k` | int16 | - | 每个切片位置的喷涂次数（手动模式输入，自动模式输出） |
| `w0` | float64 | deg | 调整后的有效单次喷幅（角度，输出参数） |

### 1.2 TargetPose 消息（输出）

| 字段名 | 类型 | 单位 | 说明 |
|--------|------|------|------|
| `id` | int32 | - | 目标点序号（重新编号后的有效切片ID） |
| `x` | float64 | m | 工作点 X 坐标（Base坐标系） |
| `y` | float64 | m | 工作点 Y 坐标（Base坐标系） |
| `z` | float64 | m | 工作点 Z 坐标（Base坐标系，已抬高） |

### 1.3 TargetInfo 消息（输出）

| 字段名 | 类型 | 单位 | 说明 |
|--------|------|------|------|
| `id` | int32 | - | 目标点序号 |
| `circle_center[3]` | float64 | mm | 圆柱轴线上的切片中心（Base坐标系） |
| `spray_center[3]` | float64 | mm | 圆柱表面切片中心（Base坐标系） |
| `cylinder_axis_direction[3]` | float64 | - | 圆柱轴线方向（单位向量） |
| `radius` | float64 | mm | 圆柱半径 |
| `angle` | float64 | deg | 该切片的角度跨度 |
| `height` | float64 | mm | 切片高度 |

### 1.4 SprayInfo 消息（输出）

| 字段名 | 类型 | 单位 | 说明 |
|--------|------|------|------|
| `num` | int32 | - | 总切片数量 |
| `area` | float64 | m² | 总喷涂面积 |

---

## 二、坐标系说明

### 2.1 坐标系定义

| 坐标系 | 说明 | 原点位置 |
|--------|------|----------|
| **LiDAR坐标系** | 雷达原始数据坐标系 | 雷达安装位置 |
| **Base坐标系** | 机器人基座坐标系 | 机器人基座中心 |
| **虚拟基座坐标系** | 用于最大覆盖角计算的虚拟坐标系 | 虚拟机械臂原点 |

### 2.2 坐标转换

```
LiDAR坐标系 → Base坐标系
- 通过 tf_lidar2_base_->to_base_from_lidar() 实现
- Z轴统一抬高 MIN_WORK_HEIGHT_MM = 2000mm
```

### 2.3 日志中的坐标输出

```
Source pos_... : LiDAR坐标系 (mm) - 用于可视化和调试
Slice id(base mm): ... : Base坐标系 (mm) - 机器人实际使用的坐标
```

---

## 三、整体算法流程

```
┌─────────────────────────────────────────────────────────────────┐
│                      点云输入 (RSLidar)                          │
└───────────────────────────┬─────────────────────────────────────┘
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│              法线估计 → 墙面去除 → 圆柱拟合                        │
│                    fitCylinderPCL()                              │
└───────────────────────────┬─────────────────────────────────────┘
                            ▼
            ┌───────────────┴───────────────┐
            ▼                               ▼
    ┌───────────────┐               ┌───────────────┐
    │   自动模式    │               │   手动模式    │
    │  (mode = 2)   │               │  (mode = 1)   │
    └───────┬───────┘               └───────┬───────┘
            ▼                               ▼
┌───────────────────────┐       ┌───────────────────────┐
│ 计算虚拟机械臂原点      │       │ 使用给定的 k 和 r     │
│ calculateVirtualBaseP_cyl()│   │                       │
└───────────┬───────────┘       └───────────┬───────────┘
            ▼                               ▼
┌───────────────────────┐       ┌───────────────────────┐
│ 计算最大覆盖角          │       │ 计算切片宽度整除180°   │
│ calculate_max_slice_angle()│  │ calculate_manual_     │
└───────────┬───────────┘       │ slice_params()        │
            ▼                   └───────────┬───────────┘
┌───────────────────────┐                   │
│ 优化搭接比例           │                   │
│ calculate_optimal_    │                   │
│ slice_angle()         │                   │
└───────────┬───────────┘                   │
            │                               │
            └───────────────┬───────────────┘
                            ▼
            ┌───────────────────────────────┐
            │       执行切片计算             │
            │       sliceCylinder()         │
            └───────────────┬───────────────┘
                            ▼
            ┌───────────────────────────────┐
            │    计算工作点并发布消息         │
            │   (LiDAR → Base 坐标转换)      │
            └───────────────────────────────┘
```

---

## 四、虚拟机械臂原点方案

### 4.1 为什么需要虚拟原点？

由于圆柱拟合结果是在**雷达坐标系**下，而最大覆盖角计算需要基于**机械臂基座坐标系**。为简化坐标转换，采用"虚拟原点"方案：

- 在雷达坐标系下，假设机械臂基座位于距离圆柱轴线 `R_cyl + virtual_base_to_surface` 的位置
- 虚拟原点位于 XY 平面上（Z=0），沿着圆柱中心指向雷达原点的方向

### 4.2 虚拟原点计算代码

```cpp
Eigen::Vector3d CylinderFitter::calculateVirtualBaseP_cyl(
    const Eigen::Vector3f &geo_center_m,      // 圆柱几何中心 (m)
    double cylinder_radius_mm,                 // 圆柱半径 (mm)
    double work_distance_mm,                   // 工作距离 (mm)
    double z_center_mm)                        // 工作高度中心 (mm)
{
    // 圆柱几何中心转为 mm
    Eigen::Vector3d geo_center_mm = geo_center_m.cast<double>() * 1000.0;
    
    // 虚拟原点到轴线的距离 = R_cyl + virtual_base_to_surface
    double virtual_base_dist = cylinder_radius_mm + fitting_params_.virtual_base_to_surface_;
    
    // 计算虚拟原点：从圆柱中心向雷达方向延伸
    Eigen::Vector3d geo_center_xy(geo_center_mm.x(), geo_center_mm.y(), 0.0);
    Eigen::Vector3d dir_xy = -geo_center_xy.normalized();  // 指向雷达原点方向
    
    Eigen::Vector3d virtual_base_pos = geo_center_mm + dir_xy * virtual_base_dist;
    virtual_base_pos.z() = 0.0;  // 虚拟原点在 z=0 平面
    
    // P_cyl = 圆柱中心相对于虚拟原点的位置
    Eigen::Vector3d P_cyl = geo_center_mm - virtual_base_pos;
    P_cyl.z() = z_center_mm;  // 使用指定的工作高度中心
    
    return P_cyl;
}
```

### 4.3 几何关系图

```
                  雷达原点 ●
                          │
                          │ (圆柱到雷达的距离)
                          │
        圆柱表面          │          虚拟机械臂原点
           │              │              │
           ●──── R_cyl ───●──── D_surf ──●
         表面           圆柱轴线         (虚拟基座)
           
        ← 圆柱中心到表面 →← 虚拟基座距离 →
        
其中: D_surf = virtual_base_to_surface (默认 1100mm)
```

---

## 五、最大覆盖角计算 (calculate_max_slice_angle)

### 5.1 输入参数

| 参数 | 说明 | 典型值 |
|------|------|--------|
| `robot_max_reach` | 机械臂最大臂长 R_arm | 1500 mm |
| `cylinder_center` | P_cyl (相对虚拟原点) | [x, y, z_center] mm |
| `cylinder_axis` | 圆柱轴线方向 V_axis | [0, 0, ±1] |
| `cylinder_radius` | 圆柱半径 r_cyl | 379 mm |
| `offset_dist` | 喷涂距离 D_offset (spray_distance) | 750 mm |
| `regional_height` | 工作高度范围 H | 600 mm |

### 5.2 计算步骤

#### Step 1: 确定最远工作高度 ΔZ_max

```cpp
// 工作区域上下限
double Z_max_limit = Z_cyl_center + H / 2.0;
double Z_min_limit = Z_cyl_center - H / 2.0;

// 最远垂直距离 (从虚拟基座z=0到工作区域边界)
double Delta_Z_max = max(|Z_max_limit|, |Z_min_limit|);
```

#### Step 2: 计算机械臂基座到圆柱轴线的垂直距离 D

```cpp
// 点到直线的距离公式
// P_base = (0, 0, 0) 是虚拟基座位置
// w = P_base - P_cyl
std::vector<double> w = {-P_cyl[0], -P_cyl[1], -P_cyl[2]};

// D = |w × V_axis| / |V_axis|
double D = norm(cross_product(w, V_axis));
```

#### Step 3: 计算有效水平臂展 R_eff

```cpp
// 机械臂在高度 Delta_Z_max 处的有效水平伸展
// 根据球面工作空间的几何约束
double R_eff = sqrt(R_arm² - Delta_Z_max²);
```

**原理图：**
```
                 │
                 │ R_arm (球半径)
                ╱│
              ╱  │
            ╱    │ Delta_Z_max
          ╱      │
    ────●────────┴───────
      基座      R_eff
         └─────────┘
         有效水平臂展
```

#### Step 4: 余弦定理计算覆盖角

```cpp
// 工作半径 = 圆柱半径 + 喷涂距离
double R_task = r_cyl + D_offset;

// 在三角形 (虚拟基座, 圆柱轴线, 工作点) 中应用余弦定理
// R_eff² = R_task² + D² - 2·R_task·D·cos(α)
double cos_alpha = (R_task² + D² - R_eff²) / (2 × R_task × D);

double alpha = acos(cos_alpha);
double total_angle = 2 × alpha;  // 总覆盖角度（左右对称）
```

**几何示意图：**
```
              工作点A (R_task处)
               /\
              /  \
         R_eff/    \R_task
            /      \
           / α      \
          ●─────────●  圆柱轴线
        虚拟基座  D
               \
                \R_task
                 \
                  \/
              工作点B (R_task处)
              
覆盖角 θ_max = 2α
```

### 5.3 输出

- **max_coverage_angle**: 机械臂在单个虚拟位置能覆盖的最大角度（度）
- **status**: 计算状态信息

---

## 六、自动模式切片优化 (calculate_optimal_slice_angle)

### 6.1 优化目标

- **目标覆盖角度**: 180°（圆柱的一半）
- **约束条件**: 
  - 搭接比例只能**增大**，不能减小（确保喷涂质量）
  - 切片次数必须为整数
  - 每次切片宽度必须一致

### 6.2 输入参数

| 参数 | 说明 | 示例值 |
|------|------|--------|
| `max_coverage_angle` | 单次最大覆盖角 θ_max | 117.8° |
| `spray_width` | 喷幅 d (step_width) | 300 mm |
| `initial_overlap_ratio` | 初始搭接比例 r | 0.5 (50%) |
| `cylinder_radius` | 圆柱半径 R_cyl | 379 mm |
| `offset_dist` | 喷涂距离 | 750 mm |

### 6.3 计算步骤

#### Step 1: 计算机械臂基座需要移动的次数 N

```cpp
int N_positions = ceil(180° / max_coverage_angle);
// 例：ceil(180 / 117.8) = 2 次
```

#### Step 2: 计算每次基座位置需覆盖的角度

```cpp
double coverage_per_position = 180° / N_positions;
// 例：180 / 2 = 90°
```

#### Step 3: 计算初始有效喷幅 w0

```cpp
// 有效喷幅 (线性距离) = 喷幅 × (1 - 搭接比例)
double w0_init_linear = spray_width × (1 - initial_overlap_ratio);
// 例：300 × (1 - 0.5) = 150 mm

// 转换为角度 (基于圆柱表面)
double w0_init_deg = (w0_init_linear / R_cyl) × (180/π);
// 例：(150 / 379) × 57.3 ≈ 22.67°
```

**注意**: 角度计算基于圆柱表面半径 R_cyl，而非工作点半径 R_task。

#### Step 4: 计算每次基座位置的喷涂次数 k

```cpp
int k_spray_passes = round(coverage_per_position / w0_init_deg);
// 例：round(90 / 22.67) = round(3.97) = 4 次
```

#### Step 5: 调整 w0 实现精确覆盖

```cpp
// 使 k × w0_adjusted = coverage_per_position（精确覆盖）
double w0_adjusted_deg = coverage_per_position / k_spray_passes;
// 例：90 / 4 = 22.5°

// 转换为线性距离
double w0_adjusted_linear = w0_adjusted_deg × (π/180) × R_cyl;
// 例：22.5 × 0.01745 × 379 ≈ 148.87 mm
```

#### Step 6: 反推调整后的搭接比例

```cpp
// w0 = d × (1 - r)  →  r = 1 - w0/d
double r_adjusted = 1 - (w0_adjusted_linear / spray_width);
// 例：1 - (148.87 / 300) = 0.5038 (50.38%)
```

### 6.4 约束检查

```cpp
// 约束1：搭接比例只能增大
if (r_adjusted < initial_overlap_ratio) {
    // 增加喷涂次数 k 直到满足约束
    while (r_adjusted < initial_overlap_ratio) {
        k_spray_passes++;
        w0_adjusted_deg = coverage_per_position / k_spray_passes;
        r_adjusted = 1 - (w0_adjusted_deg × R_cyl × π/180 / spray_width);
    }
}

// 约束2：搭接比例不能 >= 1.0
// 约束3：w0 必须为正
```

### 6.5 输出参数

| 输出 | 说明 | 示例值 |
|------|------|--------|
| `optimal_slice_angle` | 每次基座位置的覆盖角度 | 90° |
| `final_overlap_ratio` | 调整后的搭接比例 | 0.5038 |
| `k_spray_passes` | 每位置喷涂次数 | 4 |
| `w0_adjusted_deg` | 调整后有效喷幅 (角度) | 22.5° |
| `w0_adjusted_mm` | 调整后有效喷幅 (线性) | 148.87 mm |
| `n_positions` | 基座位置数 | 2 |

---

## 七、手动模式切片计算 (calculate_manual_slice_params)

### 7.1 输入参数

| 参数 | 说明 |
|------|------|
| `spray_width` | 喷幅宽度 d (mm) |
| `initial_k` | 初始喷涂次数 k |
| `initial_r` | 初始搭接比例 r |
| `cylinder_radius_mm` | 圆柱半径 (mm) |

### 7.2 计算步骤

#### Step 1: 计算初始切片宽度

```cpp
// 初始有效喷幅
double w0_init_mm = spray_width × (1 - initial_r);

// 初始切片宽度 = w0 × k
double slice_width_init_mm = w0_init_mm × initial_k;

// 转换为角度
double slice_angle_init_deg = (slice_width_init_mm / R_cyl) × (180/π);
```

#### Step 2: 计算切片数量使之整除 180°

```cpp
int N_slices = round(180° / slice_angle_init_deg);
if (N_slices < 1) N_slices = 1;
```

#### Step 3: 调整切片角度精确覆盖 180°

```cpp
double slice_angle_adjusted_deg = 180° / N_slices;
double slice_width_adjusted_mm = slice_angle_adjusted_deg × (π/180) × R_cyl;
```

#### Step 4: 反推调整后的 w0 和 r

```cpp
// 保持 k 不变
double w0_adjusted_mm = slice_width_adjusted_mm / k;
double r_adjusted = 1 - (w0_adjusted_mm / spray_width);
```

#### Step 5: 约束检查

```cpp
// 如果 r < 原始值，增加 k
if (r_adjusted < initial_r) {
    while (r_adjusted < initial_r && k < 100) {
        k++;
        w0_adjusted_mm = slice_width_adjusted_mm / k;
        r_adjusted = 1 - (w0_adjusted_mm / spray_width);
    }
}
```

### 7.3 输出参数

| 输出 | 说明 |
|------|------|
| `final_k` | 调整后的喷涂次数 |
| `final_r` | 调整后的搭接比例 |
| `final_w0_mm` | 调整后的有效单次喷幅 (mm) |
| `final_slice_width_mm` | 最终切片宽度 (mm) |
| 返回值 `N_slices` | 切片数量 |

---

## 八、切片执行 (sliceCylinder)

### 8.1 切片网格计算

```cpp
void sliceCylinder(double angle_range_deg,    // 覆盖角度 (180°)
                   double slice_height_mm,     // 切片高度
                   double slice_width_mm)      // 切片宽度 (mm)
{
    // 转换为弧度
    double angle_range = angle_range_deg × (π/180);
    
    // 高度方向切片数
    int num_height_slices = round(cylinder_height_mm / slice_height_mm);
    
    // 宽度方向切片数
    int num_width_slices = round(cylinder_circumference × angle_range / (2π) / slice_width_mm);
    
    // 双重循环生成切片
    for (int i = 0; i < num_height_slices; i++) {
        for (int j = 0; j < num_width_slices; j++) {
            // 计算切片中心、喷涂中心、工作点
            ...
        }
    }
}
```

### 8.2 切片数据结构 (SliceData)

```cpp
struct SliceData {
    int id;                          // 切片全局ID
    int layer_index;                 // 高度层索引
    int angular_index;               // 角度索引
    Eigen::Vector3f center_mm;       // 切片表面中心 (圆柱表面)
    Eigen::Vector3f circle_center_mm; // 圆柱轴线上该高度层中心
    Eigen::Vector3f work_mm;         // 工作点 (切片中心向外延伸 spray_distance)
    double angle_deg;                // 该切片的角度跨度
};
```

### 8.3 工作点计算

```cpp
// 计算从圆柱轴线中心到表面点的方向向量
Eigen::Vector3f v = center_mm - circle_center_mm;
Eigen::Vector3f dir = v.normalized();

// 工作点 = 表面中心 + 方向 × 喷涂距离
work_mm = center_mm + dir × spray_distance;
```

**示意图：**
```
    圆柱轴线
       │
       ● circle_center (轴线上)
       │
       │ ← R_cyl →
       ●──────────● center (表面) ──────● work_point
                     ← spray_distance →
```

---

## 九、完整计算示例

### 输入条件

```
机械臂臂长: 1500 mm
虚拟基座到表面距离: 1100 mm
圆柱半径: 379 mm
圆柱高度: 5690 mm
喷涂距离: 750 mm
喷幅: 300 mm
初始搭接比例: 50%
工作高度中心: 400 mm
工作高度范围: 600 mm
工作模式: 自动 (mode=2)
```

### 计算过程

```
1. 虚拟原点计算
   - virtual_base_dist = 379 + 1100 = 1479 mm
   - virtual_base_pos ≈ [-900.54, 3292.33, 0.00] mm

2. 最大覆盖角计算
   - Z_center = 400 mm
   - Z_max_limit = 400 + 600/2 = 700 mm
   - Z_min_limit = 400 - 600/2 = 100 mm
   - Delta_Z_max = max(|700|, |100|) = 700 mm
   - R_eff = √(1500² - 700²) = 1326.65 mm
   - D = 1491.69 mm (虚拟基座到轴线距离)
   - R_task = 379 + 750 = 1129 mm
   - cos_alpha = (1129² + 1491.69² - 1326.65²) / (2×1129×1491.69) = 0.5165
   - alpha = 58.90°
   - max_coverage_angle = 2 × 58.90 = 117.80°

3. 切片优化
   - N_positions = ceil(180 / 117.80) = 2
   - coverage_per_position = 180 / 2 = 90°
   - w0_init = 300 × 0.5 = 150 mm → 22.67°
   - k = round(90 / 22.67) = 4
   - w0_adjusted = 90 / 4 = 22.5° → 148.87 mm
   - r_adjusted = 1 - 148.87/300 = 50.38%

4. 切片执行
   - slice_width = 379 × (90 × π/180) = 595.47 mm
   - num_height_slices = round(5690 / 600) = 9
   - num_width_slices = 2
   - total_slices = 9 × 2 = 18

5. 最终结果
   - 机械臂基座位置数: 2
   - 每次覆盖角度: 90°
   - 每次喷涂次数: 4
   - 调整后搭接比例: 50.38%
   - 有效切片数: 12 (6层跳过，超出最大工作高度)
```

---

## 十、日志输出格式说明

### 10.1 圆柱拟合结果

```
========== Cylinder Validation ==========
Inliers:          511 [300-600] ok
Inlier ratio:     0.1892 [0.0200-0.5000] ok
Height:           5.69 m [1.00-∞] ok
Radius:           0.379 m [0.300-0.400] ok
Radial rel std:   0.0147 [0.0000-0.1500] ok
=========================================

圆柱检测成功:
  底面中心: [-1.300, 4.617, 0.066]
  几何中心: [-1.291, 4.719, 2.909]
  顶面中心: [-1.282, 4.821, 5.752]
  轴向:     [0.003, 0.036, 0.999]
  半径:     0.379 m
  高度:     5.69 m
```

### 10.2 切片参数日志

```
========== AUTO MODE OUTPUT PARAMETERS ==========
[callback] update r_initial r: 0.5038 (50.38%) <- origin: 0.5000 (50.00%)
[callback] update w0: 22.5000 deg (148.87 mm) <- origin: 150.00 mm
[callback] update k in once move: 4
[callback] update N move nums: 2
[callback] update slice angle: 90.0000 deg
[callback] update total coverage angle: 180.0000 deg (= 90.0000 * 2)
=================================================
```

### 10.3 切片坐标输出

```
Source pos_circle_center: [-1288.72, 4741.58, 3540.36], 
       pos_spray_center: [-1020.640,4473.691,3549.056],
       work_point: [-490.251, 3943.699,3566.268]
       
Slice id(base mm): 7,  
       circle_center:[-698.72, 8031.58, 7940.36], 
       spray_center:[-430.640, 7763.691, 7949.056], 
       work_point:[99.749, 7233.699, 7966.268]
```

### 10.4 最终输出参数

```
Final output parameters:
 slice_distance=750.00 mm, height=600.00 mm, step_width=300.00 mm, 
 step_height=100.00 mm, r_initial=0.5038, robot_R_arm=1500.00 mm, 
 work_mode=2, k=4, w0=22.5000 deg (148.87 mm)
```

---

## 十一、可视化元素说明

### 11.1 PCL Viewer 可视化

| 元素 | 颜色 | 说明 |
|------|------|------|
| 圆柱内点 | 🔵 蓝色 | 拟合圆柱的内点点云 |
| 圆柱轴线 | 🔴 红色 | 从底面中心到顶面中心 |
| 切片表面中心 | 🔵 蓝色小球 (10mm) | 圆柱表面的切片中心点 |
| 工作点 | 🟠 橙色大球 (25mm) | 机械臂目标工作位置 |
| 工作点连线 | 🔷 青色线 | 连接表面中心到工作点 |
| 虚拟基座 | 🔴 红色大球 | 虚拟机械臂原点位置 |
| 工作空间球 | 🔴 淡红色线框 | 机械臂最大臂展球体 |
| 有效臂展圆 | 🟡 黄色 | 工作高度处的有效水平臂展 |
| 覆盖区域 | 🟢 绿色表面 | 圆柱表面被覆盖的区域 |
| 覆盖边界 | 🟢 亮绿色 | 覆盖角度的边界弧线 |

### 11.2 statics.py 可视化

| 元素 | 颜色/样式 | 说明 |
|------|----------|------|
| LiDAR原点 | 🔴 红点 | 坐标系原点 |
| 虚拟基座 | 🔵 蓝色菱形 | 虚拟机械臂位置 |
| 圆柱中心 | 💜 紫色菱形 | 圆柱几何中心 |
| 圆柱表面 | 🔷 青色透明 | 圆柱外表面 |
| 工作点 | ⭐ 黄色星号 | 有效切片的工作点 |
| 机器人臂展 | 🟠 橙色虚线圆 | 最大臂展范围 |
| 覆盖区域 | 🔴 红色表面 | 实际覆盖的圆柱表面区域 |
| 覆盖边界 | 🟢 绿色弧线 | 覆盖角度边界 |

---

## 十二、关键参数关系图

```
┌─────────────────────────────────────────────────────────────┐
│                      输入参数关系                            │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  spray_width (d)     喷幅宽度                                │
│       │                                                     │
│       ├──× (1-r) ──→  w0_init  初始有效喷幅                  │
│       │                  │                                  │
│       │                  ├──× k ──→ slice_width 切片宽度     │
│       │                  │              │                   │
│       │                  │              └──→ N = 180°/angle │
│       │                  │                   切片数量        │
│       │                  │                                  │
│  overlap_ratio (r) ─────┘                                   │
│                                                             │
│  cylinder_radius (R_cyl)                                    │
│       │                                                     │
│       ├──+ spray_distance ──→ R_task 工作半径               │
│       │                           │                         │
│       │                           └──→ max_coverage_angle   │
│       │                                最大覆盖角            │
│       │                                                     │
│  virtual_base_to_surface ──→ 虚拟基座距离                    │
│                                                             │
│  robot_R_arm (R_arm) ──→ 机械臂球形工作空间                  │
│       │                                                     │
│       └──→ R_eff = √(R_arm² - ΔZ²) 有效水平臂展             │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 十三、回退策略

### 13.1 自动模式回退

当自动模式优化失败时（`optimal_slice_angle <= 0`），系统会：

1. 使用初始 `k=1` 调用手动模式计算函数
2. 重新计算切片参数使之整除 180°
3. 输出回退日志 `========== AUTO MODE FALLBACK ==========`

### 13.2 约束违反处理

| 约束 | 处理方式 |
|------|----------|
| `r_adjusted < r_initial` | 增加 k 直到满足约束 |
| `r_adjusted >= 1.0` | 使用初始值回退 |
| `w0 <= 0` | 使用初始值回退 |
| `Delta_Z_max >= R_arm` | 返回错误，无法触及 |

---

## 十四、常见问题排查

### Q1: 工作点位置不对称？

工作点分布取决于覆盖角度和切片数量。检查：
- 覆盖角度是否为 180°
- 切片数量 `num_width_slices` 是否正确
- 日志中工作点 X 坐标是否相对圆柱中心对称

### Q2: 切片被跳过？

切片跳过条件：`spray_center_z + MIN_WORK_HEIGHT_MM > working_max_z_base_mm`

检查：
- `MIN_WORK_HEIGHT_MM` 抬高值（默认 2000mm）
- 圆柱高度和工作高度范围设置

### Q3: 搭接比例变化？

搭接比例只会**增大**不会减小，这是为了确保喷涂覆盖质量。
- 增大幅度通常很小（<1%）
- 如果增大幅度较大，检查初始 k 值是否合理

---

## 十五、默认参数配置

### 15.1 CylinderFittingROS 默认参数

```cpp
// cylinder_fitting_ros.cpp main() 中的默认设置
node.setOption(
    750,    // distance: 喷涂距离 (mm)
    300,    // step_width: 喷幅宽度 (mm)
    100,    // step_height: 步进高度 (mm)
    600,    // height: 切片高度 (mm)
    0.5,    // lap: 搭接比例 (50%)
    1500,   // arm_reach: 机械臂臂展 (mm)
    2,      // mode: 工作模式 (2=自动)
    0       // k: 喷涂次数 (自动模式下计算)
);
```

### 15.2 CylinderFitter 拟合参数

```cpp
CylinderFitter::CylinderFitterParams {
    .min_inliers              = 300,     // 最小内点数
    .max_inliers              = 600,     // 最大内点数
    .min_inlier_ratio         = 0.02,    // 最小内点比例
    .max_inlier_ratio         = 0.50,    // 最大内点比例
    .min_height_m             = 1.00,    // 最小高度 (m)
    .min_radius_m             = 0.30,    // 最小半径 (m)
    .max_radius_m             = 0.40,    // 最大半径 (m)
    .min_radial_rel_std       = 0.00,    // 最小径向相对标准差
    .max_radial_rel_std       = 0.15,    // 最大径向相对标准差
    .virtual_base_to_surface_ = 1100.0,  // 虚拟基座到表面距离 (mm)
};
```

### 15.3 切片可视化选项

```cpp
SliceOptions {
    .surface_       = false,  // 弧形面网格
    .border_        = false,  // 切片边框线
    .height_rings_  = false,  // 高度层环线
    .print_centers_ = true,   // 切片中心点
};
```

---

## 十六、ROS 话题列表

| 话题名 | 消息类型 | 方向 | 说明 |
|--------|----------|------|------|
| `/filtered_cloud` | sensor_msgs/PointCloud2 | 输入 | 过滤后的点云数据 |
| `/process_param` | pcl_process/ProcessParam | 输入/输出 | 工艺参数 |
| `/main_control` | std_msgs/Bool | 输入 | 主控制信号（启动/停止） |
| `/target_pose_list` | pcl_process/TargetPoseList | 输出 | 工作点列表 |
| `/spray_target_info` | pcl_process/TargetInfo | 输出 | 喷涂目标详细信息 |
| `/spray_info` | pcl_process/SprayInfo | 输出 | 喷涂面积统计 |
