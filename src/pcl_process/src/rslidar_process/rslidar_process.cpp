#include "rslidar_process.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

RSLidarFilterNode::RSLidarFilterNode(ros::NodeHandle &nh) {
    sub_ = nh.subscribe("/rslidar_points", 10, &RSLidarFilterNode::cloudCallback, this);
    pub_ = nh.advertise<sensor_msgs::PointCloud2>("/rslidar_filtered", 10);
}

void RSLidarFilterNode::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud);

    if (cloud->empty()) return;

    // ============== 角度过滤 ==============
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);
    const double                         angle_limit = 30.0 * M_PI / 180.0;

    for (const auto &p : cloud->points) {
        if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z)) continue;
        if (p.x <= 0) continue;
        double angle = std::atan2(p.y, p.x);
        if (std::abs(angle) <= angle_limit) filtered->push_back(p);
    }

    std::cout << "[Angle Filter] Input: " << cloud->size() << " -> Output: " << filtered->size() << " points\n";

    // ============== 发布过滤结果 ==============
    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*filtered, output_msg);
    output_msg.header = msg->header;
    pub_.publish(output_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "rslidar_filter_node");
    ros::NodeHandle nh;

    ROS_INFO("PCL pipeline initialized: Filter + Fitting running in one process.");
    RSLidarFilterNode node(nh);

    ros::spin();
    return 0;
}