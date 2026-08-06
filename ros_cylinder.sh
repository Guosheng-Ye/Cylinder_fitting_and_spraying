#!/bin/bash

# 日志文件
LOG_FILE="/home/oneko/projects/ros_airy_work/ros_startup.log"
echo "$(date): Starting ROS nodes..." >> $LOG_FILE

# 等待系统和网络完全启动
sleep 15

# 加载 ROS 环境
source /opt/ros/noetic/setup.bash
source /home/oneko/projects/ros_airy_work/devel/setup.bash

# 设置 ROS 环境变量
# pexort ROS_MASTER_URI=http://localhost:11311
# export ROS_HOSTNAME=localhost

# 记录环境变量（调试用）
echo "$(date): LD_LIBRARY_PATH=$LD_LIBRARY_PATH" >> $LOG_FILE
echo "$(date): ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH" >> $LOG_FILE

# 启动 roscore（后台运行）
roscore &
ROSCORE_PID=$!
echo "$(date): roscore started with PID $ROSCORE_PID" >> $LOG_FILE

# 等待 roscore 完全启动
sleep 5

# 检查 roscore 是否正常运行
if ! rostopic list > /dev/null 2>&1; then
    echo "$(date): ERROR - roscore failed to start" >> $LOG_FILE
    exit 1
fi

echo "$(date): roscore is running, starting cylinder_fitting node..." >> $LOG_FILE

# 启动圆柱拟合节点
roslaunch /home/oneko/projects/ros_airy_work/src/pcl_process/launch/pcl_process.launch 2>&1 | tee -a $LOG_FILE

# 脚本结束时清理
cleanup() {
    echo "$(date): Shutting down ROS nodes..." >> $LOG_FILE
    kill $ROSCORE_PID 2>/dev/null
    killall -9 roscore rosmaster rosout 2>/dev/null
}
trap cleanup EXIT