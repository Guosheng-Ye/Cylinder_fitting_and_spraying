# Cylinder / I-Beam 点云识别与喷涂规划工作空间

## 目录

- [1. 项目概述](#1-项目概述)
- [2. 系统架构与数据流](#2-系统架构与数据流)
- [3. 仓库结构](#3-仓库结构)
- [4. 环境与依赖](#4-环境与依赖)
- [5. 编译](#5-编译)
- [6. 快速运行](#6-快速运行)
- [7. 启动文件、可执行程序与脚本](#7-启动文件可执行程序与脚本)
- [8. ROS 话题与控制状态机](#8-ros-话题与控制状态机)
- [9. 自定义消息](#9-自定义消息)
- [10. 坐标系与单位](#10-坐标系与单位)
- [11. 雷达点云预处理](#11-雷达点云预处理)
- [12. 圆柱识别与规划](#12-圆柱识别与规划)
- [13. 工字钢识别与规划](#13-工字钢识别与规划)
- [14. 动态参数调整](#14-动态参数调整)
- [15. Livox 滑动累积节点](#15-livox-滑动累积节点)
- [16. 离线分析工具与样例资产](#16-离线分析工具与样例资产)
- [17. 测试与验证](#17-测试与验证)
- [18. 故障排查](#18-故障排查)


---

## 1. 项目概述

这是一个 **ROS 1 Catkin 工作空间**，核心包为 `pcl_process`。系统面向喷涂机器人，通过前臂雷达点云完成：

1. 将原始雷达点云按前向扇区分成圆柱和工字钢两路；
2. 对圆柱执行墙面去除、法线估计、RANSAC 拟合、几何验证和弧面切片；
3. 对工字钢执行 ROI、降采样、墙面去除、分层聚类、候选合并、截面判定、端部修剪和轴向扩展；
4. 根据机械臂臂展、喷涂距离和工作模式计算可工作范围；
5. 发布每个切片、机械臂工作点列表、喷涂区域数量和名义面积；
6. 通过 PCL Visualizer 实时显示检测、拟合和切片结果。

### 1.1 工件编码

| `/work_object` 值 | 工件 | 参数组 | 对应节点 |
|---|---|---|---|
| `A` | 粗、近似竖直圆柱 | `preset_A` | `cylinder_fitting_node_ros` |
| `D` | 细、倾斜圆柱 | `preset_D` | `cylinder_fitting_node_ros` |
| `C` | 工字钢 | `ibeam_detection_params.yaml` | `ibeam_fitting_node_ros` |
| 其他值 | 非本节点工件 | 不处理 | 两个拟合节点都会停用 |

### 1.2 当前仓库规模

审阅前基线由 Git 跟踪 63 个文件（不含本次新增的 `READMD.md`），主要包括：

- 9 个 C++ 源文件、6 个 C++ 头文件；
- 7 个 Python 分析脚本；
- 7 个 ROS 自定义消息；
- 4 个 launch 文件、2 个动态重配置定义、2 个 YAML 参数文件；
- 3 张验证图片和多份历史日志；
- 5 个启动脚本与 1 个 systemd 服务样例。

### 1.3 包状态

| 包 | 版本 | 当前作用 |
|---|---:|---|
| `pcl_process` | `0.0.1` | 完整功能包：点云过滤、拟合、规划、消息和工具 |
| `main` | `0.0.1` | 占位包；CMake 中的示例可执行目标已注释，包内也没有对应源码 |

`pcl_process` 声明 MIT；`main` 声明 BSD。两个 `package.xml` 的维护者姓名和邮箱仍是占位值，发布前应修改。

---

## 2. 系统架构与数据流

```mermaid
flowchart LR
    L[前臂雷达<br/>/front_arm_lidar] --> F[rslidar_process_node_ros]
    F -->|+Y 前向 ±40°| PC[/rslidar_filtered_cylinder]
    F -->|+Y 前向 ±50°| PI[/rslidar_filtered_ibeam]

    WO[/work_object] --> CY[cylinder_fitting_node_ros]
    GO[/achieve_work_pose] --> CY
    PP[/process_param_bridge] --> CY
    PC --> CY

    WO --> IB[ibeam_fitting_node_ros]
    GO --> IB
    IP[/ibeam_process_param_bridge] --> IB
    PI --> IB

    CY --> TI[/target_info]
    CY --> TPL[/target_pose_list]
    CY --> SI[/spray_info]
    CY --> PO[/process_param]

    IB --> ITI[/ibeam_target_info]
    IB --> TPL
    IB --> SI
    IB --> IPO[/ibeam_process_param]
```

### 2.1 关键行为差异

- **圆柱节点是成功后单次停止**：触发后若拟合失败，会继续尝试后续点云；首次成功发布后把内部 `processing_` 清零。
- **工字钢节点是持续处理**：只要工件为 `C` 且 `/achieve_work_pose=true`，每帧点云都可能重新识别并发布。
- 两个节点共用 `/target_pose_list` 和 `/spray_info`，同时运行时存在多个发布者，订阅方必须结合当前工件或发布者来源消歧。
- 核心订阅和发布话题使用以 `/` 开头的绝对名称，命名空间和普通相对 remap 不会改变它们。

---

## 3. 仓库结构

```text
.
├── .vscode/                         # VS Code C++/CMake 配置，绑定另一个 ros_airy_work checkout
├── claude/CLAUDE.md                 # 历史开发说明，部分包和接口已经过时
├── src/
│   ├── CMakeLists.txt               # 指向 ROS Noetic catkin toplevel.cmake 的绝对符号链接
│   ├── main/                        # 占位 Catkin 包，没有实际构建目标
│   └── pcl_process/
│       ├── CMakeLists.txt           # 消息、动态参数、库、节点及安装规则
│       ├── package.xml
│       ├── cfg/
│       │   ├── CylinderDetection.cfg
│       │   └── IBeamDetection.cfg
│       ├── config/
│       │   ├── cylinder_detection_params.yaml
│       │   └── ibeam_detection_params.yaml
│       ├── launch/
│       │   ├── pcl_proces.launch    # 过滤 + 圆柱 + 工字钢；文件名确实少一个 s
│       │   ├── cylinder_fitting.launch
│       │   ├── ibeam_fitting.launch
│       │   └── livox_accumulator.launch
│       ├── msg/                     # 7 个自定义消息
│       ├── issues.md                # 已知问题、历史修复和待办
│       └── src/
│           ├── main.cpp             # `pcl_process` 占位可执行程序，只 ros::spin()
│           ├── rslidar_process/
│           │   ├── rslidar_process.cpp/.hpp
│           │   └── livox_process.cpp
│           ├── cylinder_fitting/
│           │   ├── cylinder_fitting_node.cpp/.hpp
│           │   ├── cylinder_fitting_ros.cpp/.hpp
│           │   ├── cylinder_yaml_den.py
│           │   ├── extract_bag_to_pcd.py
│           │   └── cal.py
│           ├── ibeam_fitting/
│           │   ├── ibeam_fitting_node.cpp/.hpp
│           │   ├── ibeam_fitting_ros.cpp/.hpp
│           │   ├── imeam_detector.py
│           │   ├── output_verify.py
│           │   ├── length.py
│           │   └── log.txt
│           ├── tf/lidar2flexed_arm.hpp
│           ├── pcd_viewer/pcd_viewer.cpp
│           ├── tests/achieve_work_pose_pub.cpp
│           └── cal_logs/            # 历史日志、统计脚本和验证图片
├── cylinder.sh                      # 本地编译并启动综合流水线；shebang 有误
├── ibeam.sh                         # 本地编译并启动工字钢流水线；shebang 有误
├── pcl_process.sh                   # 启动占位程序；shebang 有误
├── show.sh                          # 启动硬编码 PCD Viewer；shebang 有误
├── ros_cylinder.sh                  # 绑定 ros_airy_work checkout 的自启动脚本
├── ros_cylinder.service             # 与该固定 checkout 绑定的 systemd 样例
└── daily_todo.md                    # 历史开发记录
```

### 3.1 生成目录

`.gitignore` 排除了 Catkin 的 `build/`、`devel/`、`log/`，以及 `ros_bag/`、若干 PCD/结果文件和特定版本的 `.pyc`。它**没有**统一排除 `install/` 或所有 `__pycache__/`，所以新 Python 版本生成的缓存仍可能出现在 `git status` 中；现场数据是否被忽略应以 `git check-ignore -v <path>` 实测。

---

## 4. 环境与依赖

### 4.1 预期平台

仓库中的路径、Catkin 顶层链接、启动脚本和 `rosbags` 类型库都指向：

- Ubuntu 20.04 或兼容系统；
- ROS 1 Noetic；
- Catkin；
- Python 3；
- 带图形显示的运行环境（PCL/VTK Visualizer）。

> PCL 拟合类在构造时会创建可视化窗口和独立 viewer 线程。因此即使只需要 ROS 输出，当前实现仍依赖可用的 `DISPLAY`/X11；纯 headless 服务需先改造可视化开关。

### 4.2 ROS/C++ 依赖

`pcl_process/package.xml` 和 CMake 实际使用：

- `roscpp`
- `sensor_msgs`
- `std_msgs`
- `geometry_msgs`
- `pcl_ros`
- `pcl_conversions`
- `message_generation` / `message_runtime`
- `dynamic_reconfigure`
- `roslib`
- PCL
- Eigen3
- Boost（代码包含线程/绑定相关头文件，主要由系统或 PCL/ROS 间接提供）

典型安装方式：

```bash
sudo apt update
sudo apt install -y \
  ros-noetic-roscpp \
  ros-noetic-sensor-msgs \
  ros-noetic-std-msgs \
  ros-noetic-geometry-msgs \
  ros-noetic-pcl-ros \
  ros-noetic-pcl-conversions \
  ros-noetic-dynamic-reconfigure \
  ros-noetic-roslib \
  ros-noetic-message-generation \
  libpcl-dev \
  libeigen3-dev
```

也可以让 rosdep 安装核心包依赖：

```bash
cd /home/oneko/projects/cylinder
source /opt/ros/noetic/setup.bash
rosdep update
rosdep install --from-paths src/pcl_process --ignore-src -r -y
```

`main/package.xml` 仍声明外部 `rslidar_sdk`，但其 CMake 已注释该依赖且没有运行目标。对整个 `src` 执行 rosdep 时，若本机没有相应 rosdep key/包，可能在这个占位包处失败；核心构建只需处理 `src/pcl_process`。

### 4.3 C++ 标准兼容性

- `pcl_process/CMakeLists.txt` 指定 `-std=c++14`；
- `main/CMakeLists.txt` 指定 `-std=c++11`；
- 当前 C++ 源码使用了结构化绑定和指定初始化器等较新语法。

本机 GCC 在非严格 `-std=c++14` 下会把这些语法作为扩展接受；加 `-pedantic-errors` 后，工字钢 `IBeamInputs{ .field = ... }` 指定初始化器会直接失败。结构化绑定至少要求 C++17，而标准指定初始化器要求 C++20。建议要么统一切换到 C++20 并完成目标机回归，要么把指定初始化器改成普通字段赋值后至少使用 C++17，不能只把标准改为 C++17 就认为已完全修复。

### 4.4 Python 离线工具依赖

仓库没有 `requirements.txt` 或锁文件。不同脚本按需使用：

| 依赖 | 使用位置 |
|---|---|
| `numpy` | 所有数值分析脚本 |
| `matplotlib` | `cal.py`、`statics.py`、`output_verify.py`、`cylinder_yaml_den.py` 可视化 |
| `open3d` | `cylinder_yaml_den.py`、`imeam_detector.py` |
| `scipy` | 圆柱拟合/优化离线工具 |
| `PyYAML` | 参数建议输出和本文静态 YAML 检查 |
| `rosbags` | ROS1 bag 离线读取与 PointCloud2 解析 |
| `plotly` | `imeam_detector.py` 可选 Web 可视化 |

例如：

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -U pip
python -m pip install numpy matplotlib scipy open3d pyyaml rosbags plotly
```

离线工具环境与 ROS 系统 Python 可以分开；不要在不了解 ROS Python 依赖的情况下覆盖系统包。

---

## 5. 编译

### 5.1 检查 Catkin 顶层链接

仓库跟踪的 `src/CMakeLists.txt` 是绝对符号链接：

```text
/opt/ros/noetic/share/catkin/cmake/toplevel.cmake
```

在安装了 Noetic 的目标机上该链接应有效：

```bash
cd /home/oneko/projects/cylinder
source /opt/ros/noetic/setup.bash
readlink src/CMakeLists.txt
 test -e src/CMakeLists.txt && echo OK || echo BROKEN
```

若 ROS 已安装但链接失效，可重新初始化 Catkin 顶层文件：

```bash
cd /home/oneko/projects/cylinder
rm -f src/CMakeLists.txt                 # 这里只删除失效符号链接
catkin_init_workspace src
```

### 5.2 编译核心包

```bash
cd /home/oneko/projects/cylinder
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release --pkg pcl_process
source devel/setup.bash
```

完整工作空间也可执行：

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

### 5.3 构建产物

| 类型 | 名称 | 作用 |
|---|---|---|
| 库 | `cylinder_fitting_node` | 圆柱算法核心 |
| 库 | `ibeam_fitting_node` | 工字钢算法核心 |
| 库 | `rslidar_process` | 雷达扇区过滤实现 |
| 节点 | `rslidar_process_node_ros` | 原始点云分流 |
| 节点 | `cylinder_fitting_node_ros` | 圆柱 ROS 包装与规划发布 |
| 节点 | `ibeam_fitting_node_ros` | 工字钢 ROS 包装与规划发布 |
| 节点 | `livox_accumulator_node` | Livox 滑动窗口累积和过滤 |
| 节点 | `test_achieve_work_pose_pub` | 每 5 秒切换一次触发布尔值 |
| 节点 | `pcd_viewer_node` | 查看一个硬编码绝对路径的 PCD |
| 节点 | `pcl_process` | 占位主程序，仅初始化 ROS 并 `spin` |

安装规则没有包含 `pcd_viewer_node`；它只在开发空间中可用。

当前 `catkin_package()` 的库导出也不完整：它导出 `cylinder_fitting_node`、`rslidar_process` 和不存在的 `tests`，却没有导出实际存在的 `ibeam_fitting_node`；安装规则又没有安装三个算法库或头文件。工作空间内节点构建不一定受影响，但其他 Catkin 包不能把当前 `pcl_process` 当作可靠的已安装开发库使用。

### 5.4 当前机器上的验证边界

审阅时当前机器没有 `roscore`、`roslaunch`、`rosversion` 和 `catkin_make`，同时 Catkin 顶层绝对链接失效。因此本文完成了源码、配置和静态格式验证，但没有伪造 ROS 编译或运行结果；真实集成验证必须在 Noetic/PCL 环境执行。

---

## 6. 快速运行

### 6.1 启动完整流水线

```bash
cd /home/oneko/projects/cylinder
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch pcl_process pcl_proces.launch
```

注意文件名是仓库中的 **`pcl_proces.launch`**，不是 `pcl_process.launch`。

该 launch 启动：

1. `rslidar_process_node_ros`；
2. `cylinder_fitting_node_ros` 并载入圆柱 YAML；
3. `ibeam_fitting_node_ros` 并载入工字钢 YAML。

雷达驱动必须另行发布 `/front_arm_lidar`。

### 6.2 查看点云链路

```bash
rostopic hz /front_arm_lidar
rostopic hz /rslidar_filtered_cylinder
rostopic hz /rslidar_filtered_ibeam
rostopic info /target_pose_list
```

### 6.3 圆柱 A：默认自动规划示例

```bash
# 1. 选择工件，加载 preset_A
rostopic pub -1 /work_object std_msgs/String "data: 'A'"

# 2. 下发规划参数
rostopic pub -1 /process_param_bridge pcl_process/ProcessParam \
"{distance: 750.0, step_width: 300.0, step_height: 100.0, height: 600.0, lap: 0.5, arm_reach: 1500.0, mode: 2, k: 0, w0: 0.0}"

# 3. 到位后触发
rostopic pub -1 /achieve_work_pose std_msgs/Bool "data: true"
```

成功后圆柱节点自动停止。再次识别必须形成一次布尔边沿：

```bash
rostopic pub -1 /achieve_work_pose std_msgs/Bool "data: false"
rostopic pub -1 /achieve_work_pose std_msgs/Bool "data: true"
```

### 6.4 倾斜细圆柱 D

```bash
rostopic pub -1 /work_object std_msgs/String "data: 'D'"
rostopic pub -1 /process_param_bridge pcl_process/ProcessParam \
"{distance: 750.0, step_width: 180.0, step_height: 100.0, height: 600.0, lap: 0.5, arm_reach: 1500.0, mode: 2, k: 0, w0: 0.0}"
rostopic pub -1 /achieve_work_pose std_msgs/Bool "data: true"
```

`preset_D` 在 `issues.md` 中仍有检测稳定性待办，现场使用前必须结合实际 bag 验证。

### 6.5 工字钢 C

```bash
rostopic pub -1 /work_object std_msgs/String "data: 'C'"

rostopic pub -1 /ibeam_process_param_bridge pcl_process/IBeamProcessParam \
"{distance: 1100.0, step_height: 0.0, step_length: 0.0, lap: 0.0, arm_reach: 1500.0, mode_length: 2, length: 0.0, mode_theta: 2, theta: 0.0}"

rostopic pub -1 /achieve_work_pose std_msgs/Bool "data: true"
```

工字钢节点会持续处理。停止时发布：

```bash
rostopic pub -1 /achieve_work_pose std_msgs/Bool "data: false"
```

### 6.6 观察结果

```bash
rostopic echo /process_param
rostopic echo /ibeam_process_param
rostopic echo /target_info
rostopic echo /ibeam_target_info
rostopic echo /target_pose_list
rostopic echo /spray_info
```

`/target_info` 和 `/ibeam_target_info` 会按切片逐条发送；`/target_pose_list` 一次发送完整工作点数组。

---

## 7. 启动文件、可执行程序与脚本

### 7.1 Launch 文件

| Launch | 启动内容 | 输入点云 | 备注 |
|---|---|---|---|
| `pcl_proces.launch` | 过滤 + 圆柱 + 工字钢 | `/front_arm_lidar` | 推荐综合入口 |
| `cylinder_fitting.launch` | 过滤 + 圆柱 | `/front_arm_lidar` | 载入圆柱参数 |
| `ibeam_fitting.launch` | 过滤 + 工字钢 | `/front_arm_lidar` | 载入工字钢参数 |
| `livox_accumulator.launch` | Livox 累积器 | `/livox/lidar` | 输出 `/livox/accumulated`，未自动接入拟合节点 |

工字钢 launch 设置了私有参数 `input_topic=/rslidar_filtered_ibeam`，但当前 `IBeamFittingROS` 仍硬编码订阅绝对话题 `/rslidar_filtered_ibeam`，因此这个参数只是文档性配置。`cylinder_fitting.launch` 和 `ibeam_fitting.launch` 都会启动同名 `rslidar_process_node`，不要同时启动这两个独立 launch；需要两条流水线时使用综合 `pcl_proces.launch`。

### 7.2 根目录脚本现状

| 文件 | 原意 | 当前问题 |
|---|---|---|
| `cylinder.sh` | 编译后启动综合流水线 | 第一行是 `#/bin/bash`，缺少 `!` |
| `ibeam.sh` | 编译后启动工字钢流水线 | 同上；未显式 source `/opt/ros/noetic/setup.bash` |
| `pcl_process.sh` | 编译并运行 `pcl_process` | 同上；运行的是无功能占位程序 |
| `show.sh` | 启动 PCD Viewer | 同上；Viewer 内部 PCD 路径硬编码且通常不存在 |
| `ros_cylinder.sh` | 开机拉起 roscore 和拟合 launch | 固定绑定 `/home/oneko/projects/ros_airy_work`；引用不存在的 `pcl_process.launch` |
| `ros_cylinder.service` | systemd 用户图形服务 | 工作目录/ExecStart 固定为 `ros_airy_work` checkout；`ExecStop` 使用宽泛的 `killall -9` |

审阅时 `/home/oneko/projects/ros_airy_work` 实际存在，仍位于算法源码基线提交 `798df9a`；其 `devel/lib/pcl_process/` 中也存在 7 个历史构建产物。因此这里的问题是**固定 checkout 耦合和错误 launch 拼写**，不能简单理解为目录必然不存在。当前系统 ROS 环境仍不完整：仅 source 该 checkout 的 `devel/setup.bash` 不会恢复缺失的 `roslaunch`/`catkin_make`。

推荐在修复前使用本文件中的显式 `source` + `roslaunch` 命令，不要直接部署现有 systemd 服务。

---

## 8. ROS 话题与控制状态机

### 8.1 话题总表

| 节点 | 方向 | 话题 | 类型 | 队列 | Latched | 说明 |
|---|---|---|---|---:|---|---|
| 雷达过滤 | 订阅 | `/front_arm_lidar` | `sensor_msgs/PointCloud2` | 10 | 否 | 原始 `PointXYZI` 点云 |
| 雷达过滤 | 发布 | `/rslidar_filtered_cylinder` | `sensor_msgs/PointCloud2` | 10 | 否 | +Y 前向 ±40° |
| 雷达过滤 | 发布 | `/rslidar_filtered_ibeam` | `sensor_msgs/PointCloud2` | 10 | 否 | +Y 前向 ±50° |
| 圆柱 | 订阅 | `/rslidar_filtered_cylinder` | `sensor_msgs/PointCloud2` | 10 | — | 转为 `PointXYZ` 后拟合 |
| 工字钢 | 订阅 | `/rslidar_filtered_ibeam` | `sensor_msgs/PointCloud2` | 1 | — | 转为 `PointXYZ` 后识别 |
| 两拟合节点 | 订阅 | `/work_object` | `std_msgs/String` | 10 | — | `A`/`D`/`C` 工件选择 |
| 两拟合节点 | 订阅 | `/achieve_work_pose` | `std_msgs/Bool` | 10 | — | 到位/处理门控 |
| 圆柱 | 订阅 | `/process_param_bridge` | `pcl_process/ProcessParam` | 10 | — | 圆柱规划输入 |
| 工字钢 | 订阅 | `/ibeam_process_param_bridge` | `pcl_process/IBeamProcessParam` | 10 | — | 工字钢规划输入 |
| 圆柱 | 发布 | `/process_param` | `pcl_process/ProcessParam` | 1 | 是 | 实际/回传规划参数 |
| 工字钢 | 发布 | `/ibeam_process_param` | `pcl_process/IBeamProcessParam` | 10 | 是 | 回传规划参数 |
| 圆柱 | 发布 | `/target_info` | `pcl_process/TargetInfo` | 1 | 是 | 每个圆柱弧面切片 |
| 工字钢 | 发布 | `/ibeam_target_info` | `pcl_process/IBeamTargetInfo` | 10 | 是 | 每个工字钢切片 |
| 圆柱/工字钢 | 发布 | `/target_pose_list` | `pcl_process/TargetPoseList` | 1 / 10 | 是 | 完整机械臂工作点列表 |
| 两拟合节点 | 发布 | `/spray_info` | `pcl_process/SprayInfo` | 1 | 是 | 切片数与名义面积 |
| Livox | 订阅 | `/livox/lidar` | `sensor_msgs/PointCloud2` | 100 | 否 | 默认，可通过私有参数改名 |
| Livox | 发布 | `/livox/accumulated` | `sensor_msgs/PointCloud2` | 10 | 否 | 默认，可通过私有参数改名 |

Latched 的逐切片话题只会为新订阅者保留**最后一条切片**，不是整组切片；该行为与发布队列是 1 还是 10 无关。完整轨迹应读取 `/target_pose_list`。

### 8.2 圆柱状态机

初始工件为字符串 `"0"`，处理状态为 false。

```text
/work_object=A 或 D
        ↓ 加载对应 YAML preset
/achieve_work_pose=true
        ↓ processing=true
每帧尝试拟合
  ├─ 失败：保持 processing=true，等待下一帧
  └─ 成功：发布全部结果，processing=false
```

重复发布相同的 `/achieve_work_pose=true` 会被去重；重新触发需要先发 false。
若主控状态仍保存为 true，切换到另一个工件再切回 A/D 也会重新计算 `should_process` 并启动，但现场协议更推荐显式 false→true 边沿。

### 8.3 工字钢状态机

初始工件为 `"O"`。只有 `work_object == "C"` 时 true 才能启动。

```text
/work_object=C + /achieve_work_pose=true
        ↓
每帧执行识别并发布，直到：
  /achieve_work_pose=false 或 /work_object 不再是 C
```

当前实现只用 `results_[0]` 生成规划输出，即使检测器返回多个工字钢候选，也不会为后续候选生成工作点。
如果 `/achieve_work_pose` 已经是 true，之后才把工件切到 C，同样会立即开始处理，无需再次发送 true。

### 8.4 参数消息去重注意事项

- 圆柱参数变更判断包含 `distance`、`step_width`、`height`、`lap`、`arm_reach`、`mode`、`k`，但**不包含仅变化的 `step_height`**；只改它会被忽略。输入 `w0` 不参与变更判断，而且没有传给 `setOption()`，因此当前被完全忽略。
- 工字钢参数变更判断包含大多数字段，但**不包含仅变化的 `theta`**；只改 `theta` 可能被忽略。
- 圆柱 `w0` 的消息注释写“mm”，实际自动规划回传值是优化后的切片角度（degree）。

---

## 9. 自定义消息

### 9.1 `ProcessParam.msg`

| 字段 | 类型 | 当前实现语义/单位 |
|---|---|---|
| `distance` | `float64` | 喷涂轨迹相对圆柱表面的偏移，mm |
| `step_width` | `float64` | 单次喷幅弧长，mm |
| `step_height` | `float64` | 单次高度，mm；当前实现只保存/回传，`sliceCylinder()` 实际使用 `height` 作为目标片高 |
| `height` | `float64` | 规划工作高度，mm；当前也作为 `sliceCylinder()` 的目标片高 |
| `lap` | `float64` | 搭接比例，理论范围 0～1 |
| `arm_reach` | `float64` | 机械臂臂展，mm |
| `mode` | `int16` | 1 手动，2 自动 |
| `k` | `int16` | 手动模式的喷涂次数/有效喷幅倍率初值；输出端回传调整后的每位置喷涂次数，不是切片总数 |
| `w0` | `float64` | 输入值当前不生效；输出端复用该字段回传有效单次喷幅对应角度，degree |

### 9.2 `IBeamProcessParam.msg`

| 字段 | 类型 | 当前实现语义/单位 |
|---|---|---|
| `distance` | `float64` | TCP 到表面的期望距离，mm；当前只透传，不参与工作点计算 |
| `step_height` | `float64` | 透传 |
| `step_length` | `float64` | 透传 |
| `lap` | `float64` | 透传 |
| `arm_reach` | `float64` | 自动长度模式使用，mm |
| `mode_length` | `int16` | 1 手动，2 自动 |
| `length` | `float64` | 手动模式最大切片长度，mm |
| `mode_theta` | `int16` | 透传 |
| `theta` | `float64` | 透传 |

### 9.3 `TargetInfo.msg`

每个圆柱切片一条：

- `circle_center`（`float64[3]`）：该高度层的圆轴中心，基座 O 坐标系、mm；
- `spray_center`（`float64[3]`）：弧面切片中心，基座 O 坐标系、mm；
- `cylinder_axis_direction`（`float64[3]`）：`axisUsed()` 的单位轴向量；发布端未显式做方向变换，但当前硬编码 `R_OB=I`，所以数值也适用于基座 O；若未来改变雷达安装旋转，必须同步补上向量旋转；
- `radius`：圆柱半径，mm；
- `angle`：该切片角宽，degree；
- `height`：直接发布 `fitting_params.height_`，即规划输入/配置中的目标片高，mm；不是按模型高度重新均分后的实际 `height_mm / num_height_slices`；
- `id`：有效层过滤后重新生成的 1-based S 形顺序编号。

### 9.4 `IBeamTargetInfo.msg`

- `surface_point`（`float64[3]`）：切片表面中心，**基座坐标系、mm**；
- `plane_coeffs`（`float64[4]`）：`[a,b,c,d]`；法向量无量纲，`d` 按米计算；
- `plane_rx`（`float64[3]`）：工字钢长度方向单位向量；
- `height`：切片高度，mm；
- `length`：切片长度，mm；
- `wing_plate_width`：代码发布的是**完整翼缘估计宽度的一半**，mm；
- `id`：1-based 切片编号。

使用平面方程时必须先把点坐标换成米：

```text
a*x_m + b*y_m + c*z_m + d = 0
```

不要直接把 `surface_point` 的毫米值代入该方程。另需注意：发布端用 `(target_base - center_base).normalized()` 生成平面法向，而 `target_base` 已包含 `work_z_offset`（默认 -300 mm）；因此当前 `plane_coeffs` 的法向可能被 Z 偏移倾斜，并不严格等于检测得到的工字钢表面法向 `slice.normal`。

### 9.5 工作点与喷涂汇总

`TargetPose.msg`：

- `x`、`y`、`z`：机械臂工作点，**基座 O 坐标系、m**；两个 ROS 包装节点都会把内部毫米值除以 1000 后写入；
- `id`：`uint32`，与对应已发布切片的 1-based ID 相同。

`TargetPoseList.msg`：

- `targets`（`TargetPose[]`）：完整有序工作点数组。

该 `.msg` 文件的注释提到通过 `/achieve_goal false→true` 清空下游行为树缓存，但当前仓库没有订阅或发布 `/achieve_goal`；本仓库两个拟合节点的实际门控话题是 `/achieve_work_pose`。因此 `/achieve_goal` 只能视为仓库外下游约定，不能拿它触发本项目重新拟合。

`SprayInfo.msg`：

- `num`：`int16` 切片数量；
- `area`：名义喷涂面积，**m²**；两个发布端都把内部毫米面积除以 `1e6` 后写入。

---

## 10. 坐标系与单位

### 10.1 点云算法内部

- ROS/PCL 输入点云：米；
- 圆柱和工字钢几何检测、PCA、RANSAC：米；
- 算法内部切片/工作点缓存：毫米；
- `TargetInfo` 与 `IBeamTargetInfo` 的点和尺寸字段：毫米；
- `TargetPoseList.targets[].x/y/z`：米；
- 角度输出：degree；内部三角函数：radian。

### 10.2 雷达方向约定

`rslidar_process` 以：

- `+Y` 为前方；
- `X` 为横向；
- 过滤角为 `atan2(x, y)`；
- `y <= 0` 的后方点全部删除。

### 10.3 固定雷达到机械臂变换

`src/pcl_process/src/tf/lidar2flexed_arm.hpp` 使用硬编码安装尺寸：

| 向量 | 值（mm） | 含义 |
|---|---:|---|
| `t_OA` | `(590, 2470, 2560)` | 全局原点到中间安装点 A |
| `t_AB` | `(820, 0, -160)` | A 到雷达 B |
| `t_AC` | `(630, 0, 160)` | A 到机械臂基座 C |
| `R_OA` | Z 轴旋转 +90° | A 相对全局旋转 |
| `R_AB` | Z 轴旋转 -90° | 雷达相对 A 旋转 |

计算结果：

```text
t_OB = (590, 3290, 2400) mm
t_OC = (590, 3100, 2720) mm
R_OB = I
雷达点到基座 C：p_C = p_B + (0, 190, -320) mm
```

两个拟合 ROS 包装当前都调用 `to_base_from_lidar()` 发布点位，没有调用 `to_robot_from_lidar()`；因此输出点位属于**基座 O**，不是小机械臂 C。当前实现令雷达 B 与基座 O 坐标轴对齐（`R_OB=I`），所以 `to_base_from_lidar()` 实际只加平移；工字钢仍通过“两点之差”显式生成基座系 `plane_rx`，圆柱则直接复制 `axisUsed()`，依赖这个恒等旋转假设。当前代码没有订阅 `/tf`、使用 `tf2` 或提供运行时标定参数。换传感器、移动安装位置或改变轴向约定时，必须修改并重新编译该头文件及复核方向向量发布。

7 个自定义消息都没有 `std_msgs/Header`，所以消息本身不携带 `frame_id` 或采样时间；“基座 O”只是当前代码约定，消费者无法从消息自证坐标系，也不能直接按时间戳与传感器数据同步。

---

## 11. 雷达点云预处理

`RSLidarFilterNode` 订阅 `PointXYZI` 点云，遍历所有点；它会跳过任一坐标为 NaN 的点，但没有用 `std::isfinite()` 显式过滤正负无穷值。随后生成两份输出：

```cpp
angle = atan2(point.x, point.y)
point.y > 0
```

| 输出 | 实际半角 | 总扇区 | 用途 |
|---|---:|---:|---|
| `/rslidar_filtered_cylinder` | 40° | 80° | 圆柱候选区域 |
| `/rslidar_filtered_ibeam` | 50° | 100° | 工字钢候选区域 |

过滤节点不执行距离、高度、体素或异常点过滤；这些由下游拟合器处理。输出保留输入 header 和点强度字段。watchdog 每 1 秒检查一次，超过 2 秒未收到数据时告警，恢复时记录恢复日志。

源码中的个别注释和 launch 说明仍写 ±20°/±45°，以头文件常量 `40.0`/`50.0` 和实际判断为准。

---

## 12. 圆柱识别与规划

### 12.1 处理流程

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
10. 对固定 **180° 半圆弧**按高度与弧长重分片；
11. 按配置从上到下或从下到上，以 S 形生成切片和工作点；
12. 发布参数、切片、轨迹列表和面积；首次成功后停止处理。

### 12.2 墙面移除

每轮使用平面 RANSAC：

- 法向量接近水平、即平面近似竖直时视作墙面；
- 法向量接近 Z 时也可识别水平大平面；
- 平面点数小于阈值时停止；
- 删除后剩余点低于 `wall_min_keep_ratio` 时拒绝该次删除；
- 最多处理 `max_walls` 个大平面。

### 12.3 圆柱验证

圆度把截面分为 24 个 15° 扇区，综合非空覆盖率、连续空洞和各扇区半径一致性评分。RANSAC 成功不等于最终接受，所有几何门限仍需通过。

`preset_A` 可把接近竖直的拟合轴校正为理想 `(0,0,1)`；`preset_D` 保留拟合轴。

### 12.4 自动可达角计算

自动模式使用：

```text
R_task = cylinder_radius + surface_offset
Delta_Z_max = 工作高度上下边界到机械臂基座 Z=0 的最大绝对距离
R_eff = sqrt(R_arm² - Delta_Z_max²)
D = 基座到圆柱轴线的最短距离
cos(alpha) = (R_task² + D² - R_eff²) / (2*R_task*D)
max_coverage_angle = 2*alpha
```

随后根据喷幅、初始搭接率、半径和表面偏移优化切片角与切片数。`execute_slice_planning()` 在不可达、分母为零或高度超臂展时会返回失败状态和非正切片角，**但调用者并不会因此拒绝整次任务**：`run_with_rslidar()` 会进入 `AUTO MODE FALLBACK`，以 `k=1` 和输入喷幅/搭接率调用手动切片计算，然后仍按 180° 生成目标。因而当前可达性计算只是自动优化条件，不是 fail-closed 安全门。当前自动规划还把虚拟坐标中的工作高度中心固定为 `z_center_mm=400`，源码已标注 TODO；它不是从实际模型中心动态推导。

### 12.5 手动模式

手动模式把输入 `k` 和 `lap` 作为初值：

```text
w0 = step_width * (1 - lap)
slice_width = w0 * k
```

随后调整 `k` 和搭接率，使半圆弧能够被整数片均分，并保证有效喷幅/切片宽度合理。回传的 `k`、`lap` 和 `w0` 可能与输入不同；其中 `w0` 被转换并保存为 degree。

### 12.6 切片与工作点

- 实际规划弧总角固定为 180°；
- 高度片数：`round(model_height / requested_height)`；
- 弧向片数：`round(radius * π / requested_width)`；
- 实际片高和角宽重新均分，因此可能与输入值略有差异；
- 每一高度层的弧向顺序交替，构成 S 型；
- `slice_start_from_top=true` 时从最高层开始，否则从最低层开始；
- 表面切片中心沿法向外移 `distance`，得到机械臂工作点。

圆柱 `SprayInfo.area` 使用 `slice_width_mm × requested_height_mm / 1e6 × 有效发布切片数`，单位为 m²；它不一定等于按重新均分后的精确几何面积。

### 12.7 默认规划输入

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

### 12.8 圆柱 YAML 参数

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

`preset_D` 的 `min_height_m=0.08`、`ransac_max_iterations=150000`、`max_radial_range_ratio=1.8` 超出动态重配置文件分别定义的最小/最大范围 `0.1`、`100000`、`1.0`。当前 `switchPreset()` 调用 `Server::updateConfig()` 后又直接 `applyFittingParams()`；ROS Noetic 的 `updateConfig()` 本身不执行 `__clamp__()`，所以切换预设时这些原始值仍会进入算法。通过 rqt/`set_parameters` 提交的配置请求则会先执行 `__clamp__()`，因此一次人工调整可能把它们压回 cfg 范围并改变检测行为。必须通过运行日志或 `rosparam get` 核对当前有效值。

---

## 13. 工字钢识别与规划

### 13.1 处理流程

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

### 13.2 局部坐标

- `u`：工字钢主轴/长度方向，规范为 X 分量非负；
- `w=(0,0,1)`：竖直、截面高度方向；
- `v=w×u`：水平翼缘方向；
- 正面法向会根据雷达/工字钢相对位置选择符号。

### 13.3 自动长度模式

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

### 13.4 手动长度模式

手动模式把 `length` 解释为“允许的最大单片长度”：

```text
n = ceil(识别总长 / 用户 length)
adjusted_slice_length = 识别总长 / n
```

返回值是调整后的**单片长度**，虽然函数注释和部分输出字段把它称为“总工作长度”。后续 `sliceIBeam()` 又把该值作为切片宽度使用。集成方应按实际行为理解 `beam_length_of_working`。

### 13.5 切片与工作点

- 当前调用传入一个 0 参数，因此 `sliceIBeam()` 使用整段截面高度和 `beam_length_of_working` 作为目标切片尺寸；
- 长度片数使用 `ceil(total_length / target_slice_length)`，然后严格均分；
- 切片沿 `u` 从最小投影到最大投影编号；
- 表面点位于选定正面；
- 机械臂工作点沿 `slice.normal` 指向机器人的方向正向偏移 `d_base`，并叠加 `work_z_offset`；
- `IBeamProcessParam.distance`、`step_height`、`step_length`、`lap`、`mode_theta`、`theta` 目前主要是透传，不驱动切片或工作点几何。

### 13.6 默认规划输入

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

### 13.7 面积语义

工字钢 `SprayInfo.area` 计算：

```text
切片数 × 切片长度_mm × 腹板高度_mm / 1e6
```

结果单位是 m²。它是正面矩形投影面积，不包含两侧翼缘的全部展开表面积。

### 13.8 启动参数

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

## 14. 动态参数调整

两个拟合节点都创建 `dynamic_reconfigure::Server`。

```bash
source /opt/ros/noetic/setup.bash
source /home/oneko/projects/cylinder/devel/setup.bash
rosrun rqt_reconfigure rqt_reconfigure
```

可在界面选择：

- `/cylinder_fitting_node`
- `/ibeam_fitting_node`

### 14.1 圆柱动态参数组

- 模型门限；
- RANSAC 参考轴和轴向容差；
- RANSAC 迭代/距离参数；
- 径向、长宽比、圆度验证；
- 法线 KNN；
- 墙面移除；
- 理想轴修正；
- 虚拟基座距离。

不包含 `slice_start_from_top` 和 `min_center_xy_dist_m`。

### 14.2 工字钢动态参数组

- ROI；
- 体素与 SOR；
- 去墙；
- 聚类；
- 严格/宽松合并；
- 候选几何；
- 截面精修；
- 端部修剪；
- 轴向扩展。

机械臂安装参数不在动态面板中。

### 14.3 参数生效规则

- launch 先把 YAML 加载到节点私有命名空间；
- dynamic_reconfigure 回调更新算法参数；
- 圆柱切换 A/D 时会重新读取对应预设并同步动态配置；
- 调参后应同时记录 `rosparam dump`、工件编码和 bag，避免只保存 rqt 截图而无法复现。

---

## 15. Livox 滑动累积节点

`livox_accumulator_node` 与 RSLidar 分流节点独立，默认参数：

| 私有参数 | 默认/launch 值 | 说明 |
|---|---:|---|
| `input_topic` | `/livox/lidar` | 输入 |
| `output_topic` | `/livox/accumulated` | 输出 |
| `accumulate_frames` | 10 | 保留最近 N 帧的滑动窗口 |
| `publish_rate` | 10 Hz | 定时合并发布 |
| `enable_angle_filter` | true | 启用前向扇区 |
| `angle_filter_deg` | 45° | 半角 |
| `forward_axis` | `x` | Livox 默认 +X 前向；也支持 `y` |
| `enable_voxel_filter` | true | 发布前降采样 |
| `voxel_size` | 0.05 m | 目标体素大小 |

节点每次定时发布时重新合并当前 deque 中最近 N 帧，不是累计满 N 帧后清空。若点云跨度过大，会增大体素尺寸，避免单轴网格数超过约 1000。输出沿用最新输入的 header/frame_id，但把 `stamp` 改为发布时间。超过 2 秒无输入时 watchdog 告警。

该输出没有被现有圆柱/工字钢 launch 自动接入；若要替换输入，需要修改绝对订阅名或增加正确的 ROS remap/参数化支持。

---

## 16. 离线分析工具与样例资产

### 16.1 工具总表

| 文件 | 用途 | 主要依赖 | 重要限制 |
|---|---|---|---|
| `cylinder_yaml_den.py` | 从 PCD 离线检测圆柱，输出参数建议 YAML/报告 | Open3D、NumPy、SciPy、Matplotlib、PyYAML | 默认扫描当前目录下的 `pcd/`；建议显式传入文件 |
| `extract_bag_to_pcd.py` | 从 ROS1 bag 提取 PointCloud2 为 ASCII PCD | NumPy、rosbags | 假设 `x/y/z` 为 little-endian float32；只保存 XYZ |
| `cal.py` | 圆柱与机械臂覆盖角 3D 几何可视化 | NumPy、Matplotlib | 参数和样例数据写在脚本中 |
| `statics.py` | 解析圆柱日志、汇总输出并画 3D 覆盖图 | NumPy、Matplotlib | 日志格式与 C++ ROS_INFO 文本耦合 |
| `imeam_detector.py` | 工字钢 Python 原型：PCD/bag、聚类、截面和 3D 可视化 | Open3D、NumPy、rosbags、Plotly 可选 | 文件名保留了 `imeam` 拼写；默认绝对路径过时 |
| `output_verify.py` | 解析工字钢/切片日志，检查切片、平面和工作点 | NumPy、Matplotlib | 必须包含逐切片 `Published target info for slice_id ...` 日志；不是自动单元测试 |
| `length.py` | 单一几何公式演算样例 | Python 标准库 | 参数硬编码；`d_base=1100 mm` 与标准 launch YAML 的 800 mm 不同 |

审阅时实际运行 `length.py` 得到 `1566.78 mm`；若只把同一公式的 `d_base` 改为运行配置中的 800 mm，结果为 `1781.80 mm`。因此该脚本只能作为公式样例，不能当作当前 launch 参数的预期输出。

### 16.2 ROS bag 转 PCD

```bash
source .venv/bin/activate
python src/pcl_process/src/cylinder_fitting/extract_bag_to_pcd.py \
  /absolute/path/input.bag \
  10 \
  20
```

第二、三个位置参数分别是抽帧间隔和最大保存帧数。脚本固定读取 `/front_arm_lidar`，并自动创建与 bag 同目录、同名（无扩展名）的输出目录。它生成 ASCII PCD、过滤 NaN/Inf，固定按小端 `<f` 读取 `x/y/z`，不保留 intensity、ring、timestamp 等字段。

### 16.3 圆柱离线检测

```bash
source .venv/bin/activate
python src/pcl_process/src/cylinder_fitting/cylinder_yaml_den.py \
  --file /absolute/path/cloud.pcd \
  --output /absolute/path/recommended_cylinder_params.yaml \
  --tilt-angle 0
```

`--visualize/-v` 才会开启交互式图形；默认无图形。还支持目录扫描、批量分析、体素尺寸、半径范围、目标倾角和倾角容差；不直接读取 bag，bag 需要先用上一节脚本转成 PCD。以 `--help` 为当前版本准。

### 16.4 日志统计

`statics.py` 支持：

```bash
python src/pcl_process/src/cal_logs/statics.py \
  src/pcl_process/src/cal_logs/log1.txt \
  src/pcl_process/src/cal_logs/log2.txt
python src/pcl_process/src/cal_logs/statics.py \
  src/pcl_process/src/cal_logs/log1.txt --viz
python src/pcl_process/src/cal_logs/statics.py \
  src/pcl_process/src/cal_logs/log1.txt \
  src/pcl_process/src/cal_logs/log2.txt --compare
```

`output_verify.py` 的准确调用形式为：

```bash
python src/pcl_process/src/ibeam_fitting/output_verify.py \
  --file /path/to/full_ibeam_run.log \
  --no-3d

# 或从标准输入解析，并把 3D 图保存为文件
some_log_command | python src/pcl_process/src/ibeam_fitting/output_verify.py \
  --stdin --save /tmp/ibeam_verify.png
```

仓库自带的 `src/pcl_process/src/ibeam_fitting/log.txt` 只有本体尺寸、工作长度等汇总行，没有逐切片 `Published target info ...` 行；当前解析器对它只能提取部分汇总字段，随后会因没有 `surface_points` 而退出，不能直接作为上面命令的完整验证输入。

### 16.5 样例数据

- `cal_logs/log1.txt`、`log2.txt`：两组圆柱几何与工作点日志；
- `cal_logs/12-22/res1`、`res2`：更完整的圆柱输出样例；
- `cal_logs/12-22/cap_1.png`：PCL 圆柱拟合窗口与 Matplotlib 机械臂覆盖范围的并排截图；
- `cal_logs/test_ims/virtual_dis_800.png`：虚拟距离 800 mm 的覆盖角示意，图中约 133.36°；
- `cal_logs/test_ims/virtual_dis_1100.png`：虚拟距离 1100 mm 的覆盖角示意，图中约 112.45°；
- `ibeam_fitting/log.txt`：工字钢本体尺寸与工作长度汇总样例，不含逐切片目标日志。

历史文档还引用未跟踪的 `ros_bag/`、`cal_logs/3-5/`、`cal_logs/3-10/` 等现场数据；这些目录受 `.gitignore` 或未包含在当前仓库中。

---

## 17. 测试与验证

### 17.1 静态检查

在没有 ROS 环境时仍可执行：

```bash
cd /home/oneko/projects/cylinder

# Python 与 dynamic_reconfigure CFG 语法（不生成 __pycache__）
python3 - <<'PY'
from pathlib import Path
paths = list(Path('src/pcl_process/src').rglob('*.py'))
paths += list(Path('src/pcl_process/cfg').glob('*.cfg'))
for p in paths:
    compile(p.read_text(encoding='utf-8-sig'), str(p), 'exec')
    print('OK', p)
PY

# launch/package XML
python3 - <<'PY'
from pathlib import Path
import xml.etree.ElementTree as ET
for p in list(Path('src').rglob('package.xml')) + list(Path('src').rglob('*.launch')):
    ET.parse(p)
    print('OK', p)
PY

# YAML
python3 - <<'PY'
from pathlib import Path
import yaml
for p in Path('src/pcl_process/config').glob('*.yaml'):
    yaml.safe_load(p.read_text())
    print('OK', p)
PY

# Shell 语法（不会发现错误 shebang、固定 checkout 或错误 launch 名）
bash -n ./*.sh

# systemd 单元语法（不会验证 ExecStart 的业务行为）
systemd-analyze verify ./ros_cylinder.service
```

审阅时以上 Python/CFG、XML、YAML、Shell 检查均通过，`systemd-analyze verify` 也返回 0；这只证明静态格式可接受，不代表当前服务中的固定路径、`killall -9` 或缺失 launch 名称可安全运行。

### 17.2 ROS 构建验证

```bash
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release --pkg pcl_process
source devel/setup.bash
rospack find pcl_process
rosmsg show pcl_process/TargetInfo
rosmsg show pcl_process/IBeamTargetInfo
```

### 17.3 运行时冒烟检查

```bash
roslaunch pcl_process pcl_proces.launch

# 另一个终端
rostopic list | sort
rosnode list
rostopic hz /front_arm_lidar
rostopic hz /rslidar_filtered_cylinder
rostopic hz /rslidar_filtered_ibeam
```

### 17.4 触发测试节点

```bash
rosrun pcl_process test_achieve_work_pose_pub
```

该节点使用 `ros::Rate(0.2)`，即每 5 秒切换一次 true/false；源码注释中的“5 Hz”不准确。测试前仍需先发布正确的 `/work_object`。

### 17.5 当前测试缺口

- 没有 gtest/rostest 自动测试；
- `src/tests` 只有一个触发发布器；
- 历史日志和图片没有接入 CI；
- 没有参数 schema、单位或消息语义回归测试；
- 没有 headless 模式测试；
- 没有验证两个拟合节点同时发布共享话题时的消费者行为。

---

## 18. 故障排查

### 18.1 `catkin_make: command not found`

```bash
source /opt/ros/noetic/setup.bash
command -v catkin_make
```

若仍找不到，安装 ROS Noetic/Catkin，而不是在普通 Python 环境中模拟 ROS 命令。

### 18.2 `src/CMakeLists.txt` 是 broken symlink

确认 `/opt/ros/noetic/share/catkin/cmake/toplevel.cmake` 是否存在；安装 ROS 后按 [5.1](#51-检查-catkin-顶层链接) 重新初始化。

### 18.3 没有过滤点云

依次检查：

```bash
rostopic hz /front_arm_lidar
rostopic echo -n 1 /front_arm_lidar/header
rosnode info /rslidar_process_node
rostopic hz /rslidar_filtered_cylinder
```

过滤器只保留 `+Y` 前向点。如果传感器实际是 `+X` 前向，输出可能接近空，需要修改坐标约定。

### 18.4 有点云但不识别

- 确认先设置工件，再发布到位 true；
- 查看对应过滤话题是否覆盖完整目标；
- 查看 ROI 距离/高度是否截断目标；
- 记录算法的拒绝原因日志；
- A/D 是否使用了正确 preset；
- D 模式确认预设切换后的实际值；若操作过 rqt，再检查值是否被 cfg 范围钳位；
- 工字钢检查墙面是否误删、分层是否造成断裂、严格/宽松合并是否通过。

### 18.5 圆柱成功一次后不再处理

这是当前单次成功门控设计。发布 false，再发布 true。

### 18.6 工字钢持续重复发布

这是当前持续模式设计。发布 `/achieve_work_pose=false` 或切换 `/work_object`。

### 18.7 新订阅者只看到最后一个切片

逐切片话题是 latched；圆柱发布队列为 1，工字钢为 10，但 latch 对每个发布者都只缓存最后一条消息。读取完整 `/target_pose_list`，或修改协议为包含数组/批次 ID 的消息。

### 18.8 PCL/VTK 窗口启动失败

检查：

```bash
printf '%s\n' "$DISPLAY"
xdpyinfo >/dev/null
```

systemd/SSH/headless 环境需要正确 XAUTHORITY，或者从代码中增加 `enable_visualization` 并允许不构造 viewer。

### 18.9 Python 脚本导入失败

审阅环境实际缺少 `matplotlib`、`open3d` 和 `rosbags`，对应脚本会在 `--help` 前即导入失败。激活正确虚拟环境并按 [4.4](#44-python-离线工具依赖) 安装，不要把导入失败误认为算法失败。

### 18.10 `/target_pose_list` 来源不明确

```bash
rostopic info /target_pose_list
```

综合 launch 下会同时显示圆柱和工字钢发布者。消费者必须用 `/work_object`、节点连接信息或将话题拆分为不同名称。

---

## 附录 A：常用命令速查

```bash
# 编译
cd /home/oneko/projects/cylinder
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release --pkg pcl_process
source devel/setup.bash

# 启动综合流水线
roslaunch pcl_process pcl_proces.launch

# 动态调参
rosrun rqt_reconfigure rqt_reconfigure

# 查看连接
rosnode list
rostopic list
rostopic info /target_pose_list

# 停止处理
rostopic pub -1 /achieve_work_pose std_msgs/Bool "data: false"

# 保存参数
rosparam dump /tmp/cylinder_params.yaml
```

