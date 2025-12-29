#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

using CloudI        = pcl::PointCloud<pcl::PointXYZI>;
using CloudIPtr     = CloudI::Ptr;
using Cloud         = pcl::PointCloud<pcl::PointXYZ>;
using CloudPtr      = Cloud::Ptr;
using CloudCallback = std::function<void(const CloudIPtr &)>;

/// @brief ralidar96 雷达数据订阅/rslidar_points与过滤,
///        并发布/rslidar_filtered
class RSLidarFilterNode {
public:
    RSLidarFilterNode(ros::NodeHandle &nh);

private:
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
    void watchdogCallback(const ros::TimerEvent &event);

private:
    ros::Subscriber sub_;             // rslidar订阅器
    ros::Publisher  pub_;             // rslidar过滤发布器
    ros::Timer      watchdog_timer_;  // 数据超时检测定时器
    ros::Time       last_msg_time_;   // 最后一次收到消息的时间
    bool            data_active_ = false;     // 数据是否活跃
    bool            last_has_output_ = false; // 上次是否有输出
};
