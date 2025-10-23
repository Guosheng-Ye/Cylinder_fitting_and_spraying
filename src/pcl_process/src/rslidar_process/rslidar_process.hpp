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

private:
    ros::Subscriber sub_; // rslidar订阅器
    ros::Publisher  pub_; // rslidar过滤发布器
};
