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

这是一个 ROS 1 Catkin 工作空间，核心包为 `pcl_process`。系统面向喷涂机器人，通过前臂雷达点云完成：

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

### 1.2 包状态

| 包 | 版本 | 当前作用 |
|---|---:|---|
| `pcl_process` | `0.0.1` | 完整功能包：点云过滤、拟合、规划、消息和工具 |
| `main` | `0.0.1` | 占位包；CMake 中的示例可执行目标已注释，包内也没有对应源码 |

`pcl_process` 声明 MIT；`main` 声明 BSD。

---

## 2. 系统架构与数据流

```mermaid
flowchart LR
    L["前臂雷达<br/>/front_arm_lidar"] --> F["rslidar_process_node_ros"]
    F -->|"+Y 前向 ±40°"| PC["/rslidar_filtered_cylinder"]
    F -->|"+Y 前向 ±50°"| PI["/rslidar_filtered_ibeam"]

    WO["/work_object"] --> CY["cylinder_fitting_node_ros"]
    GO["/achieve_work_pose"] --> CY
    PP["/process_param_bridge"] --> CY
    PC --> CY

    WO --> IB["ibeam_fitting_node_ros"]
    GO --> IB
    IP["/ibeam_process_param_bridge"] --> IB
    PI --> IB

    CY --> TI["/target_info"]
    CY --> TPL["/target_pose_list"]
    CY --> SI["/spray_info"]
    CY --> PO["/process_param"]

    IB --> ITI["/ibeam_target_info"]
    IB --> TPL
    IB --> SI
    IB --> IPO["/ibeam_process_param"]
```

### 2.1 关键行为差异

- 圆柱节点是成功后单次停止：触发后若拟合失败，会继续尝试后续点云；首次成功发布后把内部 `processing_` 清零。
- 工字钢节点是持续处理：只要工件为 `C` 且 `/achieve_work_pose=true`，每帧点云都可能重新识别并发布。
- 两个节点共用 `/target_pose_list` 和 `/spray_info`，同时运行时存在多个发布者，订阅方必须结合当前工件或发布者来源消歧。
- 核心订阅和发布话题使用以 `/` 开头的绝对名称，命名空间和普通相对 remap 不会改变它们。

---

## 3. 仓库结构

```text
.
├── .vscode/                         # VS Code C++/CMake 配置，绑定另一个 ros_airy_work checkout
├── claude/CLAUDE.md                 # 历史开发说明，部分包和接口已经过时
├── docs/
│   ├── lidar-preprocessing.md       # 雷达分流细节
│   ├── cylinder-fitting.md          # 圆柱识别与规划
│   └── ibeam-fitting.md             # 工字钢识别与规划
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

`.gitignore` 排除了 `build/`、`devel/`、`log/`、`ros_bag/`，没有统一排除 `install/` 和所有 `__pycache__/`。某个路径是否被忽略，用 `git check-ignore -v <path>` 实测。

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

`pcl_process/package.xml` 和 CMake 用到 `roscpp`、`sensor_msgs`、`std_msgs`、`geometry_msgs`、`pcl_ros`、`pcl_conversions`、`message_generation`/`message_runtime`、`dynamic_reconfigure`、`roslib`，以及 PCL、Eigen3、Boost。

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

`pcl_process` 指定 `-std=c++14`，`main` 指定 `-std=c++11`，但源码里用了结构化绑定和指定初始化器。GCC 在非严格 C++14 下把它们当扩展接受，加 `-pedantic-errors` 后 `IBeamInputs{ .field = ... }` 会直接编译失败。要彻底解决：统一切到 C++20，或把指定初始化器改成普通字段赋值后至少用 C++17。

### 4.4 Python 离线工具依赖

仓库没有 `requirements.txt` 或锁文件。不同脚本按需使用：

| 依赖 | 使用位置 |
|---|---|
| `numpy` | 所有数值分析脚本 |
| `matplotlib` | `cal.py`、`statics.py`、`output_verify.py`、`cylinder_yaml_den.py` 可视化 |
| `open3d` | `cylinder_yaml_den.py`、`imeam_detector.py` |
| `scipy` | 圆柱拟合/优化离线工具 |
| `PyYAML` | 参数建议输出和静态 YAML 检查 |
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

去掉 `--pkg pcl_process` 可编译整个工作空间。

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

`pcd_viewer_node` 没有安装规则，只在开发空间可用。`catkin_package()` 导出了不存在的 `tests`、漏了 `ibeam_fitting_node`，三个算法库和头文件也没装，所以外部包不能把 `pcl_process` 当作已安装的库使用。

---

## 6. 快速运行

### 6.1 启动完整流水线

```bash
cd /home/oneko/projects/cylinder
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch pcl_process pcl_proces.launch
```

注意文件名是仓库中的 `pcl_proces.launch`，不是 `pcl_process.launch`。

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

`ros_airy_work` 是另一份 checkout，这些脚本固定依赖它，加上 launch 名拼写错误，所以不能直接部署。修复前用上面的 `source` + `roslaunch` 命令。

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

Latched 的逐切片话题只会为新订阅者保留最后一条切片，不是整组切片；该行为与发布队列是 1 还是 10 无关。完整轨迹应读取 `/target_pose_list`。

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

- 圆柱参数变更判断包含 `distance`、`step_width`、`height`、`lap`、`arm_reach`、`mode`、`k`，但不包含仅变化的 `step_height`；只改它会被忽略。输入 `w0` 不参与变更判断，而且没有传给 `setOption()`，因此当前被完全忽略。
- 工字钢参数变更判断包含大多数字段，但不包含仅变化的 `theta`；只改 `theta` 可能被忽略。
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

- `surface_point`（`float64[3]`）：切片表面中心，基座坐标系、mm；
- `plane_coeffs`（`float64[4]`）：`[a,b,c,d]`；法向量无量纲，`d` 按米计算；
- `plane_rx`（`float64[3]`）：工字钢长度方向单位向量；
- `height`：切片高度，mm；
- `length`：切片长度，mm；
- `wing_plate_width`：代码发布的是完整翼缘估计宽度的一半，mm；
- `id`：1-based 切片编号。

使用平面方程时必须先把点坐标换成米：

```text
a*x_m + b*y_m + c*z_m + d = 0
```

不要直接把 `surface_point` 的毫米值代入该方程。另需注意：发布端用 `(target_base - center_base).normalized()` 生成平面法向，而 `target_base` 已包含 `work_z_offset`（默认 -300 mm）；因此当前 `plane_coeffs` 的法向可能被 Z 偏移倾斜，并不严格等于检测得到的工字钢表面法向 `slice.normal`。

### 9.5 工作点与喷涂汇总

`TargetPose.msg`：

- `x`、`y`、`z`：机械臂工作点，基座 O 坐标系、m；两个 ROS 包装节点都会把内部毫米值除以 1000 后写入；
- `id`：`uint32`，与对应已发布切片的 1-based ID 相同。

`TargetPoseList.msg`：

- `targets`（`TargetPose[]`）：完整有序工作点数组。

`.msg` 注释里提到 `/achieve_goal` 的 false→true 边沿，但仓库里没有节点订阅或发布它。两个拟合节点的门控话题是 `/achieve_work_pose`。

`SprayInfo.msg`：

- `num`：`int16` 切片数量；
- `area`：名义喷涂面积，m²；两个发布端都把内部毫米面积除以 `1e6` 后写入。

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

两个拟合 ROS 包装当前都调用 `to_base_from_lidar()` 发布点位，没有调用 `to_robot_from_lidar()`；因此输出点位属于基座 O，不是小机械臂 C。当前实现令雷达 B 与基座 O 坐标轴对齐（`R_OB=I`），所以 `to_base_from_lidar()` 实际只加平移；工字钢仍通过“两点之差”显式生成基座系 `plane_rx`，圆柱则直接复制 `axisUsed()`，依赖这个恒等旋转假设。当前代码没有订阅 `/tf`、使用 `tf2` 或提供运行时标定参数。换传感器、移动安装位置或改变轴向约定时，必须修改并重新编译该头文件及复核方向向量发布。

7 个自定义消息都没有 `std_msgs/Header`，所以消息本身不携带 `frame_id` 或采样时间；“基座 O”只是当前代码约定，消费者无法从消息自证坐标系，也不能直接按时间戳与传感器数据同步。

---

## 11. 雷达点云预处理

`RSLidarFilterNode` 订阅 `/front_arm_lidar`，按 `atan2(x, y)` 把 +Y 前向 ±40° 的点发到 `/rslidar_filtered_cylinder`，±50° 发到 `/rslidar_filtered_ibeam`。不做距离、高度、体素或异常点过滤，这些交给下游拟合器。watchdog 每 1 秒检查一次，超过 2 秒没有数据就告警。

实现细节和角度常量见 [docs/lidar-preprocessing.md](docs/lidar-preprocessing.md)。

---

## 12. 圆柱识别与规划

墙面移除 → 法线估计 → RANSAC 拟合 → 几何验证 → 按高度和弧长切分固定 180° 半圆弧。首次成功后停止，等待下一次 false→true 边沿。`preset_A` 面向竖直圆柱，`preset_D` 面向细长倾斜圆柱。

可达角计算、切片规则、默认输入和 YAML 参数见 [docs/cylinder-fitting.md](docs/cylinder-fitting.md)。

---

## 13. 工字钢识别与规划

ROI 裁剪 → 体素降采样 → 统计滤波 → 迭代去墙 → 分层欧式聚类 → 严格/宽松合并 → 截面判定 → 端部修剪 → 轴向扩展。工件为 `C` 且 `/achieve_work_pose=true` 时每帧都重新识别。

局部坐标、长度模式、切片、面积语义和启动参数见 [docs/ibeam-fitting.md](docs/ibeam-fitting.md)。

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

`length.py` 里写死 `d_base=1100 mm`，launch 实际用 800 mm，输出对不上现场参数，只能当公式样例。

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
- `cal_logs/12-22/`：更完整的圆柱输出样例和拟合截图；
- `cal_logs/test_ims/virtual_dis_800.png`、`virtual_dis_1100.png`：虚拟距离 800/1100 mm 的覆盖角示意（约 133.36°/112.45°）；
- `ibeam_fitting/log.txt`：工字钢尺寸与工作长度汇总，不含逐切片目标日志。

历史文档还引用未跟踪的 `ros_bag/`、`cal_logs/3-5/`、`cal_logs/3-10/` 等现场数据；这些目录受 `.gitignore` 或未包含在当前仓库中。

---

## 17. 测试与验证

### 17.1 静态检查

没有 ROS 环境时只能验证语法：

```bash
cd /home/oneko/projects/cylinder

# Python 与 dynamic_reconfigure CFG
python3 -c "import pathlib; [compile(p.read_text(encoding='utf-8-sig'), str(p), 'exec') for p in list(pathlib.Path('src/pcl_process').rglob('*.py')) + list(pathlib.Path('src/pcl_process/cfg').glob('*.cfg'))]"

# launch/package XML 与 YAML
python3 -c "import xml.etree.ElementTree as ET, pathlib; [ET.parse(p) for p in list(pathlib.Path('src').rglob('*.xml')) + list(pathlib.Path('src').rglob('*.launch'))]"
python3 -c "import yaml, pathlib; [yaml.safe_load(p.read_text()) for p in pathlib.Path('src/pcl_process/config').glob('*.yaml')]"

# Shell 与 systemd 单元
bash -n ./*.sh
systemd-analyze verify ./ros_cylinder.service
```

这些只验证语法，不验证路径、`killall -9` 或 launch 名。

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

### 17.5 测试缺口

没有 gtest/rostest，没有参数、单位或消息语义的回归测试，也没有覆盖 headless 运行和两个节点同时发布共享话题的情况。`src/tests` 只有一个触发发布器，历史日志和图片没有接入 CI。

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

缺依赖时脚本会在 `--help` 之前就导入失败。先激活 [4.4](#44-python-离线工具依赖) 的虚拟环境，别把导入错误当成算法错误。

### 18.10 `/target_pose_list` 来源不明确

```bash
rostopic info /target_pose_list
```

综合 launch 下会同时显示圆柱和工字钢发布者。消费者必须用 `/work_object`、节点连接信息或将话题拆分为不同名称。
