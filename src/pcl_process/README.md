# PCL Process & Cylinder Fitting 模块说明

模块主要负责处理激光雷达点云数据，实现圆柱体目标的自动识别、拟合以及喷涂作业的路径规划。

## 核心节点：cylinder_fitting_ros


### 1. 系统工作流程概览

整体处理流程分为四个主要阶段：**数据采集与预处理**、**几何模型拟合**、**表面切片与离散化**、**路径生成与坐标变换**。

#### 阶段一：数据输入与控制
- **输入数据**：接收来自 RSLidar 的原始点云数据 (`sensor_msgs::PointCloud2`)。
- **控制指令**：通过 `main_ctl` 话题接收bool指令，控制算法启停，确保在需要时进行计算拟合操作。
- **参数动态配置**：支持实时调整切片参数（切片高度、宽度、工作距离），以适应不同的作业工艺要求。

#### 阶段二：点云处理与圆柱拟合 (Core Algorithm)
在 `CylinderFitter` 类中实现，核心目标是从含噪的原始点云中精确提取圆柱几何参数。

1.  **感兴趣区域提取 (ROI Filtering)**：
    - **角度扇区过滤**：针对雷达侧向安装场景（+Y 为前向），将点云坐标转换到极坐标系 $(\rho, \theta)$。仅保留 $\theta \in [-30^\circ, +30^\circ]$ 且 $y > 0$ 的前向扇区数据，幅减少无效计算。
    - **距离过滤**：剔除距离雷达过近（盲区）或过远（背景）的离群点。

2.  **法线估计 (Normal Estimation)**：
    - **原理**：利用主成分分析 (PCA) 方法估计局部表面法线。
    - **过程**：对每个点 $P_i$，在 KD-Tree 中搜索其 $k$ 个最近邻点，计算协方差矩阵 $C$。
    - **特征值分解**：$C \cdot v_j = \lambda_j \cdot v_j$，最小特征值 $\lambda_0$ 对应的特征向量即为该点的法向量 $n_i$。
    - **作用**：法线信息是区分圆柱表面与平面（如墙壁、地面）的关键特征，提高后续 RANSAC 拟合的准确性。

3.  **RANSAC 模型拟合 (Model Segmentation)**：
    - **算法**：带有法线约束的随机采样一致性 (RANSAC) 算法 (`SACSegmentationFromNormals`)。
    - **模型定义**：圆柱模型由轴线上一点 $P_{axis}$、轴线方向向量 $\vec{v}_{axis}$ 和半径 $r$ 定义。
    - **迭代过程**：
        1. 随机选取两个点及其法线。
        2. 计算潜在的圆柱模型参数。
        3. 统计所有点中符合该模型（距离表面小于阈值 `distance_threshold`）的内点数量。
        4. 重复上述步骤，选取内点数量最多且法线偏差最小的模型作为最优解。
    - **参数约束**：设置半径范围 (`min_radius`, `max_radius`) 和轴线倾角阈值，防止拟合到错误的几何体。

4.  **后处理与优化**：
    - **统计滤波**：对拟合出的内点云进行统计离群点移除 (SOR)，进一步去除附着在表面的噪点。
    - **参数提取**：输出圆柱轴线方程、底面/顶面中心坐标及平均半径。

#### 阶段三：表面切片与离散化 (Surface Slicing)
为了将连续的几何表面转化为机械臂可执行的离散路径点，需进行精细的网格化切片处理。

1.  **切片参数定义**：
    - **输入**：拟合好的圆柱模型、切片高度 $H_{slice}$ (mm)、切片宽度 $W_{slice}$ (mm)。
    - **计算网格维度**：
        - 轴向层数 $N_{layers} = \lceil H_{cylinder} / H_{slice} \rceil$
        - 径向份数 $N_{sectors} = \lceil (\text{ArcLength}) / W_{slice} \rceil$

2.  **网格生成算法**：
    - **轴向遍历**：沿圆柱轴线 $\vec{v}_{axis}$，从底面到底面+高度，按步长 $H_{slice}$ 生成一系列高度层平面。
    - **径向展开**：在每个高度层平面上，以轴线为中心，按角度步长 $\Delta \theta$ 旋转生成扇区。
    - **中心点计算**：
        - **轴线中心 (Point C)**：$C_i = P_{bottom} + i \cdot H_{slice} \cdot \vec{v}_{axis}$
        - **表面中心 (Point A)**：在当前高度层，取扇区角度中值 $\theta_j$，计算圆柱表面上的点：
          $A_{ij} = C_i + r \cdot (\cos\theta_j \cdot \vec{x}_{ref} + \sin\theta_j \cdot \vec{y}_{ref})$
          其中 $\vec{x}_{ref}, \vec{y}_{ref}$ 为垂直于轴线的参考坐标基。

3.  **切片属性计算**：
    - **法向量**：对于每个切片，计算从轴线指向表面中心的单位向量 $\vec{n}_{slice} = (A - C) / ||A - C||$，代表该作业面的理想喷涂方向。
    - **有效性检查**：检查切片区域内是否包含足够的原始点云内点。若某切片区域内点过少（如圆柱表面的孔洞或遮挡区域），则标记为无效切片，避免机械臂对空作业。
    - **数据结构化**：将每个有效切片封装为 `SliceData` 对象，包含 ID、层级索引、几何中心坐标及法线方向，存入列表供下游节点使用。

#### 阶段四：路径生成与坐标变换 (Trajectory Generation)
在 `CylinderFittingROS` 节点中实现，该阶段负责将抽象的几何切片转化为机械臂控制器可直接执行的运动指令，涉及多坐标系变换与安全约束。

1.  **多坐标系变换 (TF Transformation)**：
    - **背景**：点云数据基于雷达坐标系 (`rslidar`)，而机械臂控制基于基座坐标系 (`base_link`)。由于雷达侧向安装（+Y 前向），两者存在旋转与平移偏差。
    - **变换矩阵**：通过 ROS TF 树获取变换矩阵 $T_{lidar}^{base}$。
    - **点变换**：对所有关键点（表面中心 $A_{lidar}$、轴线中心 $C_{lidar}$）执行刚体变换：
      $P_{base} = T_{lidar}^{base} \cdot P_{lidar}$
      确保所有后续计算均在统一的机器人基座坐标系下进行。

2.  **作业目标点计算 (Work Point Calculation)**：
    - **定义**：作业点 $B$ 是喷枪末端执行器的目标位置，需位于圆柱表面法线方向上，且保持特定的工艺距离。
    - **计算公式**：
      $B = A_{base} + \vec{n}_{slice} \times D_{offset}$
      其中：
      - $A_{base}$：基座坐标系下的切片表面中心。
      - $\vec{n}_{slice} = (A_{base} - C_{base}) / ||A_{base} - C_{base}||$：切片法向量（指向外侧）。
      - $D_{offset}$：设定的喷涂作业距离（如 1150mm）。
    - **退化处理**：若 $A$ 与 $C$ 重合（数学奇点），则沿预设安全方向（如全局 +X）进行偏移，防止计算错误。

3.  **高度约束与安全校验 (Height Constraints)**：
    - **Z轴统一抬升**：考虑到机械臂基座安装高度与作业面的相对关系，对所有目标点 $Z$ 坐标施加统一偏移量 `MIN_WORK_HEIGHT_MM`：
      $Z_{target} = Z_{computed} + \Delta Z_{lift}$
    - **越界检测**：计算抬升后的目标高度 $Z_{target}$，并与圆柱体的物理最大高度 $Z_{max}$ 进行比对。
      - 若 $Z_{target} > Z_{max}$，则判定该切片超出作业范围，自动剔除，防止机械臂碰撞或奇异。

4.  **路径规划与发布 (Path Planning & Publishing)**：
    - **路径排序**：为了优化机械臂运动效率，减少空行程，对生成的切片目标点进行排序。
      - **策略**：采用从高到低（Top-down），生成连续的作业路径。
    - **消息封装**：
      - **`TargetPoseList`**：包含一系列有序的 `TargetPose` (ID, x, y, z)，供运动控制节点使用。
      - **`TargetInfo`**：包含调试信息（圆柱参数、法线、半径、喷涂角度），用于 Rviz 可视化与系统监控。
    - **实时发布**：通过 ROS Topic (`/target_pose_list`, `/spray_target_pose`) 以 10Hz 或事件触发方式发布路径信息。

### 关键技术指标
- **拟合算法**：RANSAC (Random Sample Consensus) with Normal constraints.
- **坐标一致性**：支持多坐标系自动转换 (Lidar -> Base)。
- **鲁棒性**：包含异常点剔除、高度越界检测及空切片处理机制。

# PCL Process - 圆柱拟合与喷涂路径规划模块

> 最后更新: 2025年12月23日

本模块负责处理激光雷达点云数据，实现圆柱体目标的自动识别、拟合以及喷涂作业的路径规划。

## 📋 目录

- [功能概述](#功能概述)
- [系统架构](#系统架构)
- [快速开始](#快速开始)
- [ROS 接口](#ros-接口)
- [工作模式](#工作模式)
- [核心算法](#核心算法)
- [可视化工具](#可视化工具)
- [参数配置](#参数配置)
- [常见问题](#常见问题)

---

## 功能概述

### 核心功能

1. **圆柱体检测与拟合** - 从点云中自动识别并拟合圆柱体几何参数
2. **表面切片计算** - 将圆柱表面离散化为网格切片
3. **工作点生成** - 计算机械臂的喷涂作业目标位置
4. **路径规划** - 生成优化的作业路径序列

### 支持的作业模式

| 模式 | 说明 |
|------|------|
| **自动模式** (mode=2) | 根据机械臂臂展自动计算最优切片参数 |
| **手动模式** (mode=1) | 使用用户指定的 k 和 r 参数进行切片 |

---

## 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│                   cylinder_fitting_ros                       │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐     │
│  │  点云输入   │───▶│  圆柱拟合   │───▶│  切片计算   │     │
│  └─────────────┘    └─────────────┘    └─────────────┘     │
│         │                                      │            │
│         ▼                                      ▼            │
│  ┌─────────────┐                      ┌─────────────┐      │
│  │  参数配置   │                      │  坐标转换   │      │
│  └─────────────┘                      └─────────────┘      │
│                                               │             │
│                                               ▼             │
│                                      ┌─────────────┐       │
│                                      │  路径发布   │       │
│                                      └─────────────┘       │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 快速开始

### 编译

```bash
cd ~/projects/ros_airy_work
catkin_make
source devel/setup.bash
```

### 运行

```bash
# 启动圆柱拟合节点
rosrun pcl_process cylinder_fitting_ros

# 或使用 launch 文件
roslaunch pcl_process cylinder_fitting.launch
```

### 触发拟合

```bash
# 发送启动命令
rostopic pub /main_control std_msgs/Bool "data: true"

# 停止拟合
rostopic pub /main_control std_msgs/Bool "data: false"
```

---

## ROS 接口

### 订阅话题

| 话题名 | 消息类型 | 说明 |
|--------|----------|------|
| `/filtered_cloud` | sensor_msgs/PointCloud2 | 过滤后的点云数据 |
| `/process_param` | pcl_process/ProcessParam | 工艺参数配置 |
| `/main_control` | std_msgs/Bool | 启动/停止控制 |

### 发布话题

| 话题名 | 消息类型 | 说明 |
|--------|----------|------|
| `/target_pose_list` | pcl_process/TargetPoseList | 工作点列表 (m) |
| `/spray_target_info` | pcl_process/TargetInfo | 喷涂目标详细信息 (mm) |
| `/spray_info` | pcl_process/SprayInfo | 喷涂面积统计 |
| `/process_param` | pcl_process/ProcessParam | 输出调整后的工艺参数 |

### ProcessParam 消息字段

| 字段 | 类型 | 单位 | 说明 |
|------|------|------|------|
| `distance` | float64 | mm | 喷涂距离 (工作点到表面距离) |
| `step_width` | float64 | mm | 喷幅宽度 |
| `step_height` | float64 | mm | 步进高度 (预留) |
| `height` | int16 | mm | 切片高度 |
| `lap` | float64 | - | 搭接比例 (0~1) |
| `arm_reach` | int16 | mm | 机械臂臂展 |
| `mode` | int16 | - | 1=手动, 2=自动 |
| `k` | int16 | - | 每位置喷涂次数 |
| `w0` | float64 | deg | 有效单次喷幅角度 |

> 📖 **详细接口文档**: [src/cal_logs/interface.md](src/cal_logs/interface.md)

---

## 工作模式

### 自动模式 (mode=2)

自动模式下，系统会根据机械臂臂展自动计算最优的切片参数：

1. **计算虚拟基座位置** - 在雷达坐标系下模拟机械臂位置
2. **计算最大覆盖角** - 基于臂展和几何约束计算单次可覆盖角度
3. **优化搭接比例** - 调整参数使切片精确覆盖 180°

**输出参数：**
- `N` - 基座位置数
- `k` - 每位置喷涂次数
- `r` - 调整后的搭接比例
- `w0` - 有效单次喷幅

### 手动模式 (mode=1)

手动模式下，使用用户指定的 `k` 和 `r` 参数：

1. 计算初始切片宽度 `slice_width = step_width × (1-r) × k`
2. 调整参数使切片数量整除 180°
3. 可能微调 `r` 值以满足约束

---

## 核心算法

### 1. 圆柱拟合

使用 RANSAC 算法从点云中拟合圆柱模型：

- **法线估计** - PCA 方法估计局部表面法线
- **墙面去除** - 自动检测并移除平面障碍物
- **模型验证** - 检查内点数、半径、高度、圆度等指标

**验证指标：**

| 指标 | 范围 | 说明 |
|------|------|------|
| 内点数 | 300-600 | 拟合点数量 |
| 内点比例 | 2%-50% | 相对于总点数 |
| 高度 | ≥1.0m | 圆柱最小高度 |
| 半径 | 0.30-0.40m | 目标圆柱半径范围 |
| 径向标准差 | ≤15% | 表面平整度 |

### 2. 切片计算

将圆柱表面离散化为网格切片：

```
高度方向: num_height_slices = round(cylinder_height / slice_height)
宽度方向: num_width_slices = round(180° / slice_angle)
总切片数: total = num_height_slices × num_width_slices
```

### 3. 工作点计算

```cpp
// 表面中心 A 到工作点 B
direction = (A - C).normalized();  // C 是轴线中心
work_point = A + direction × spray_distance;
```

### 4. 坐标转换

```
LiDAR坐标系 → Base坐标系
- TF 变换: tf_lidar2_base_->to_base_from_lidar()
- Z轴抬高: +2000mm (MIN_WORK_HEIGHT_MM)
```

---

## 可视化工具

### PCL Viewer 实时可视化

运行节点时自动显示 PCL 可视化窗口：

| 元素 | 颜色 | 说明 |
|------|------|------|
| 圆柱内点 | 🔵 蓝色 | 拟合的点云 |
| 切片中心 | 🔵 蓝色球 | 表面中心点 |
| 工作点 | 🟠 橙色球 | 机械臂目标位置 |
| 工作连线 | 🔷 青色 | 表面到工作点 |
| 覆盖区域 | 🟢 绿色 | 可覆盖范围 |
| 虚拟基座 | 🔴 红色球 | 虚拟机械臂原点 |

### 日志分析工具

使用 `statics.py` 分析日志文件：

```bash
cd src/pcl_process/src/cal_logs

# 分析单个日志
python3 statics.py 12-22/res1

# 可视化结果
python3 statics.py 12-22/res1 --viz

# 对比多个日志
python3 statics.py 12-22/res1 12-22/res2 --compare
```

---

## 参数配置

### 默认参数

```cpp
// 工艺参数
distance   = 750mm   // 喷涂距离
step_width = 300mm   // 喷幅宽度
height     = 600mm   // 切片高度
lap        = 0.5     // 搭接比例 50%
arm_reach  = 1500mm  // 机械臂臂展
mode       = 2       // 自动模式

// 拟合参数
min_radius = 0.30m   // 最小半径
max_radius = 0.40m   // 最大半径
min_height = 1.00m   // 最小高度
virtual_base_to_surface = 1100mm  // 虚拟基座距离
```

### 运行时修改参数

```bash
# 发布新的工艺参数
rostopic pub /process_param pcl_process/ProcessParam \
  "distance: 800
   step_width: 300
   step_height: 100
   height: 600
   lap: 0.5
   arm_reach: 1500
   mode: 2
   k: 0
   w0: 0"
```

---

## 常见问题

### Q1: 圆柱拟合失败？

检查：
- 点云数据是否正常 (`rostopic echo /filtered_cloud`)
- 目标圆柱是否在视野范围内
- 半径是否在 0.30-0.40m 范围内

### Q2: 切片被跳过？

原因：切片高度超出最大工作高度

检查：
- `MIN_WORK_HEIGHT_MM` 设置（默认 2000mm）
- 圆柱实际高度

### Q3: 工作点位置不对？

检查：
- 坐标系转换是否正确
- `spray_distance` 参数设置
- 日志中 Source/Slice 坐标对比

### Q4: 搭接比例变化了？

这是正常的！系统会微调搭接比例使切片精确覆盖 180°。
- 只会增大，不会减小
- 增大幅度通常 <1%

---

## 文件结构

```
pcl_process/
├── CMakeLists.txt
├── package.xml
├── README.md                          # 本文件
├── launch/
│   └── cylinder_fitting.launch
├── msg/
│   ├── ProcessParam.msg
│   ├── TargetPose.msg
│   ├── TargetPoseList.msg
│   ├── TargetInfo.msg
│   └── SprayInfo.msg
└── src/
    ├── cylinder_fitting/
    │   ├── cylinder_fitting_node.cpp  # 核心算法
    │   ├── cylinder_fitting_node.hpp
    │   ├── cylinder_fitting_ros.cpp   # ROS 封装
    │   ├── cylinder_fitting_ros.hpp
    │   └── cal.py                     # 计算验证脚本
    └── cal_logs/
        ├── interface.md               # 详细接口文档
        └── statics.py                 # 日志分析工具
```

---

## 相关文档

- [接口详细文档](src/cal_logs/interface.md) - 完整的算法和接口说明
- [日志分析工具](src/cal_logs/statics.py) - 日志解析和可视化

---

## 更新日志

### 2025-12-23
- 新增手动/自动工作模式切换
- 添加工作点可视化连线
- 优化搭接比例计算算法
- 新增日志分析工具 statics.py
- 完善接口文档
