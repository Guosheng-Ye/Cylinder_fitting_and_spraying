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
    ros::Subscriber sub_;                    // rslidar订阅器
    ros::Publisher  pub_cylinder_;           // 圆柱拟合过滤发布器 (±20度)
    ros::Publisher  pub_ibeam_;              // 工字钢拟合过滤发布器 (±45度)
    ros::Timer      watchdog_timer_;         // 数据超时检测定时器
    ros::Time       last_msg_time_;          // 最后一次收到消息的时间
    bool            data_active_ = false;    // 数据是否活跃
    bool            last_has_output_cylinder_ = false; // 圆柱过滤上次是否有输出
    bool            last_has_output_ibeam_ = false;    // 工字钢过滤上次是否有输出
    
    // 过滤角度参数
    static constexpr double CYLINDER_ANGLE_DEG = 20.0;  // 圆柱拟合：左右各20度
    static constexpr double IBEAM_ANGLE_DEG = 50.0;     // 工字钢拟合：左右各50度
};
