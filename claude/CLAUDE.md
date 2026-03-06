# CLAUDE.md

本文件为 Claude Code (claude.ai/code) 提供该仓库的开发指引。

## 项目概述

这是一个基于 LiDAR 的机器人喷涂路径规划 **ROS1 (Catkin) 工作空间**。系统处理 RSLidar 点云，检测几何目标（圆柱或工字钢），在目标表面计算切片网格，并向机械臂喷涂器发布目标位姿。

## 编译与运行

```bash
# 编译
cd ~/projects/ros_airy_work
catkin_make
source devel/setup.bash

# 启动所有节点（推荐）
roslaunch pcl_process pcl_proces.launch

# 仅启动圆柱拟合流水线
roslaunch pcl_process cylinder_fitting.launch

# 仅启动工字钢拟合流水线
roslaunch pcl_process ibeam_fitting.launch

# 手动启动各节点
rosrun pcl_process rslidar_process_node_ros    # 雷达过滤节点
rosrun pcl_process cylinder_fitting_node_ros   # 圆柱检测节点
rosrun pcl_process ibeam_fitting_node_ros      # 工字钢检测节点
```

**选择作业对象（影响参数预设）：**
```bash
# A = 粗竖直圆柱 (preset_A)
# D = 细长倾斜圆柱 (preset_D)
# C = 工字钢
rostopic pub /work_object std_msgs/String "A"
```

**通过话题触发检测：**
```bash
rostopic pub /achieve_work_pose std_msgs/Bool "data: true"
```

**运行时发布工艺参数：**
```bash
rostopic pub /process_param_bridge pcl_process/ProcessParam \
  "distance: 750  step_width: 300  step_height: 100  height: 600  lap: 0.5  arm_reach: 1500  mode: 2  k: 0  w0: 0"
```

**无需重新编译，动态调整检测参数：**
```bash
# 编辑 YAML 后重新启动，或在运行时使用 rqt_reconfigure
rqt_reconfigure
```

**离线 PCD 分析：**
```bash
cd src/pcl_process/src/cylinder_fitting
python3 cylinder_yaml_den.py <file.pcd> [--tilt-angle 45]  # 生成 recommended_cylinder_params.yaml

cd src/pcl_process/src/cal_logs
python3 statics.py 12-22/res1 [--viz] [--compare 12-22/res2]
```

## 包结构

```
src/
├── main/               # 入口包（仅 package.xml，当前无活动节点）
├── pcl_process/        # 主点云处理包
│   ├── msg/            # 自定义 ROS 消息
│   ├── config/         # YAML 参数文件（修改后无需重新编译）
│   │   ├── cylinder_detection_params.yaml   # 含 preset_A（粗竖直）和 preset_D（细斜）
│   │   └── ibeam_detection_params.yaml
│   ├── cfg/            # dynamic_reconfigure 配置文件
│   │   ├── CylinderDetection.cfg
│   │   └── IBeamDetection.cfg
│   ├── launch/
│   │   ├── pcl_proces.launch        # 启动所有节点（雷达 + 圆柱 + 工字钢）
│   │   ├── cylinder_fitting.launch
│   │   ├── ibeam_fitting.launch
│   │   └── livox_accumulator.launch
│   └── src/
│       ├── rslidar_process/    # RSLidarFilterNode：将原始点云过滤为两个话题
│       ├── cylinder_fitting/   # CylinderFitter + CylinderFittingROS 节点
│       ├── ibeam_fitting/      # IBeamFitter + IBeamFittingROS 节点
│       ├── tf/                 # FlexedArmTF：硬编码的 LiDAR→Base 变换
│       ├── pcd_viewer/         # 离线 PCD 查看工具
│       ├── tests/              # 测试工具
│       └── cal_logs/           # 日志分析工具（statics.py）及接口文档
└── robot_bt_manager/   # 行为管理 + 目标位姿桥接包
    ├── msg/            # 自定义 ROS 消息（镜像 pcl_process 消息类型）
    │   ├── ProcessParam.msg / IBeamProcessParam.msg
    │   ├── TargetInfo.msg / IBeamTargetInfo.msg
    │   ├── TargetPose.msg / TargetPoseList.msg
    └── srv/
```

## 系统架构

### 数据流

```
RSLidar → /rslidar_points
            ↓ RSLidarFilterNode (rslidar_process)
            ├─→ /rslidar_filtered_cylinder  (±40° 扇区)
            └─→ /rslidar_filtered_ibeam     (±50° 扇区)
                        ↓
        ┌───────────────┴────────────────┐
        ↓                                ↓
CylinderFittingROS               IBeamFittingROS
        ↓                                ↓
/target_pose_list            /target_pose_list
/spray_target_info           /ibeam_target_info
/spray_info                  /spray_info
/process_param               /ibeam_process_param
        ↓
target_pose_bridge (robot_bt_manager)
        ↓
/pending_target_pose_with_platform_angle
        ↓
bt_nodes (robot_bt_manager)
        ↓
/target_pose_with_platform_angle → 曲臂执行
```

### 坐标系

所有坐标系由 `FlexedArmTF`（`src/pcl_process/src/tf/lidar2flexed_arm.hpp`）管理：
- **O** – 机器人基座系（X 前、Y 左、Z 上），所有输出目标位姿均在此坐标系下。
- **B** – LiDAR 系（Y 前、X 右、Z 上），原始点云输入。
- **C** – 小臂系（与 O 对齐，相对 O 有偏移），用于部分工作点计算。
- 变换为**硬编码**（非 ROS TF 树）：LiDAR 相对臂末端 A 的偏移 = (820, 0, -160) mm；A 在基座系中 = (590, 2470, 2560) mm，偏航 +90°。
- 内部计算全部使用 **mm**；`/target_pose_list` 输出单位为**米**。
- 转换到基座系时，Z 轴抬高 `MIN_WORK_HEIGHT_MM = 2000 mm`。

### 圆柱检测流水线（`cylinder_fitting/`）

1. **ROI 过滤** – 对输入点云做角度/距离直通滤波
2. **墙面去除**（`removeWalls()`）– 迭代 RANSAC 平面去除（最多 `max_walls` 个平面）
3. **法线估计** – 墙面去除后进行 KNN-PCA 法线计算
4. **RANSAC 圆柱拟合**（`SACSegmentationFromNormals`）– 受 `ransac_axis_{x,y,z}` 和 `eps_angle_deg` 约束
5. **验证** – 检查内点数量、内点比、高度、半径范围、径向标准差
6. **切片** – `sliceCylinder()` 生成 `SliceData` 对象（表面中心、轴心、工作点）
7. **模式分发**：
   - **mode=2（自动）**：计算虚拟基座位置，用余弦定理求最大覆盖角，优化重叠率
   - **mode=1（手动）**：使用用户提供的 `k` 和 `lap` 计算切片宽度
8. **发布** – 在 `/target_pose_list` 上按从上到下顺序发布目标位姿

**预设切换**：`/work_object` 变化时调用 `switchPreset(object)`，从参数服务器读取 `preset_A/` 或 `preset_D/` 子命名空间，更新所有成员，刷新 dynamic_reconfigure 面板，最后调用 `applyFittingParams()`。

### 工字钢检测流水线（`ibeam_fitting/`）

使用 DBSCAN 聚类 + PCA 形状提取。输出平面方程、轴向量、翼板宽度和表面中心点。已知问题见 `src/pcl_process/src/ibeam_fitting/ibeam_fitting_node_issue.md`。

### robot_bt_manager 包

管理完整任务执行生命周期：
- **bt_nodes**：行为管理节点。按目标位姿 ID 顺序串行驱动曲臂运动，每个点到达后触发小臂动作。
- **target_pose_bridge**：将 `/target_pose_list`（批量）转换为 `/pending_target_pose_with_platform_angle`（单点，供 bt_nodes 消费）。

完整接口文档见 `src/robot_bt_manager/README.md`。

### 动态参数重配置

两个检测节点均支持 `dynamic_reconfigure`，配置文件位于：
- `cfg/CylinderDetection.cfg` → `pcl_process/CylinderDetectionConfig`
- `cfg/IBeamDetection.cfg` → `pcl_process/IBeamDetectionConfig`

无需重启节点，可通过 `rqt_reconfigure` 在运行时修改参数。

## 关键话题

| 话题 | 类型 | 方向 | 说明 |
|-------|------|-----------|-------------|
| `/rslidar_points` | sensor_msgs/PointCloud2 | 输入 | 原始雷达点云 |
| `/rslidar_filtered_cylinder` | sensor_msgs/PointCloud2 | 内部 | ±40° 过滤点云 |
| `/rslidar_filtered_ibeam` | sensor_msgs/PointCloud2 | 内部 | ±50° 过滤点云 |
| `/work_object` | std_msgs/String | 输入 | 目标类型：`A`=粗圆柱、`D`=细斜圆柱、`C`=工字钢 |
| `/achieve_work_pose` | std_msgs/Bool | 输入/输出 | 启停检测；bt_nodes 的 latched 输出 |
| `/achieve_goal` | std_msgs/Bool | 输入 | 导航到位触发（false→true 上升沿启动任务） |
| `/process_param_bridge` | pcl_process/ProcessParam | 输入 | 圆柱喷涂工艺参数 |
| `/ibeam_process_param_bridge` | pcl_process/IBeamProcessParam | 输入 | 工字钢工艺参数 |
| `/target_pose_list` | pcl_process/TargetPoseList | 输出 | 基座系下的工作点（单位：米） |
| `/spray_target_info` | pcl_process/TargetInfo | 输出 | 详细切片信息（单位：mm） |
| `/spray_info` | pcl_process/SprayInfo | 输出 | 切片总数与总面积 |
| `/ibeam_target_info` | pcl_process/IBeamTargetInfo | 输出 | 工字钢检测结果 |
| `/pending_target_pose_with_platform_angle` | crankshaft_control/... | 内部 | bridge→bt_nodes 目标队列 |
| `/target_pose_with_platform_angle` | crankshaft_control/... | 输出 | bt_nodes→曲臂执行指令 |
| `/crankshaft_status` | std_msgs/String | 输入 | `TARGET_REACHED` 推进任务状态 |
| `/main_control_command` | std_msgs/String | 输出 | 向曲臂发送 `START_MISSION` / `ABORT_MISSION` |
| `/work_done` | std_msgs/Bool | 输出 | 任务完成标志（latched） |
| `/small_arm_task_control` | std_msgs/String | 输入 | `START` / `PAUSE` / `CANCEL_ALL` |

## 自定义消息（`msg/`）

### pcl_process
- **ProcessParam** – 喷涂工艺配置（distance、step_width、height、lap、arm_reach、mode、k、w0）
- **TargetPose** – 单个工作点（id、x、y、z，单位：米，基座系）
- **TargetPoseList** – TargetPose 列表
- **TargetInfo** – 完整切片几何信息，用于调试（circle_center、spray_center、axis、radius、angle、height，均为 mm）
- **SprayInfo** – 汇总统计（切片数、总面积 m²）
- **IBeamProcessParam / IBeamTargetInfo** – 工字钢对应消息

### robot_bt_manager
镜像 pcl_process 消息类型，供 bt_nodes 使用。包含 `ProcessParam`、`IBeamProcessParam`、`TargetInfo`、`IBeamTargetInfo`、`TargetPose`、`TargetPoseList`。

## 圆柱检测参数

`config/cylinder_detection_params.yaml` 使用两个命名预设：

| 参数 | preset_A（粗竖直） | preset_D（细斜 45°） | 说明 |
|-----------|--------------------------|---------------------------|-------------|
| `ransac_axis_{x,y,z}` | 0,0,1 | ~0.707,0,0.707 | RANSAC 参考轴方向 |
| `eps_angle_deg` | 15° | 20° | 最大轴偏差角 |
| `min/max_radius_m` | 0.30–0.40 | 0.04–0.18 | 目标半径范围 |
| `use_ideal_axis` | true | false | 切片时是否强制使用竖直轴 |
| `normal_distance_weight` | 0.05 | 0.05 | RANSAC 法线权重 |

发布 `/work_object` 为 `A` 或 `D` 时自动切换预设。YAML 顶层平铺参数在节点启动时通过 `loadParams()` 用于初始化。

## 注意事项

- `src/main` 包当前无编译节点（仅含 package.xml 和 CMakeLists.txt）。
- `FlexedArmTF` 偏移量为**硬编码硬件常量**——物理臂结构改变时需同步更新。
- `MIN_WORK_HEIGHT_MM = 2000mm` Z 轴抬升量硬编码在 `cylinder_fitting_ros.cpp` 中，不在 YAML 里。
- 日志文件位于 `src/pcl_process/src/ibeam_fitting/log.txt`（已加入 .gitignore）。
- `bt_nodes` 订阅 `/process_param` 和 `/ibeam_process_param`；gRPC 默认发布 `/process_param_bridge` 和 `/ibeam_process_param_bridge`。若无桥接节点，需在 launch 中添加 remap。
- 作业对象代码：**A** = 粗竖直圆柱，**D** = 细斜圆柱（原为 B），**C** = 工字钢。
