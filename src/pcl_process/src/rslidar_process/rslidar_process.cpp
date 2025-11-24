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

    // ============== 角度过滤（侧向安装：+Y 为前向，+X 为右侧） ==============
    // 目标：保留相对于 +Y 前向左右各20度（总 40 度扇区）内且位于传感器前方 (y>0) 的点。
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);
    const double                         angle_limit = 20.0 * M_PI / 180.0; // 弧度

    for (const auto &p : cloud->points) {
        if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z)) continue; // 跳过 NaN
        if (p.y <= 0) continue;                                              // 位于后方或原点附近，不保留
        // 计算点相对于前向(+Y)的偏转角：前向向量为 (0,1)，使用 atan2(横向, 前向)
        double angle = std::atan2(p.x, p.y); // p.x 为横向（右正），p.y 为前向
        if (std::abs(angle) <= angle_limit) {
            filtered->push_back(p);
        }

        // if (p.x <= 0) continue; // 位于后方或原点附近，不保留
        // // 计算点相对于前向(+X)的偏转角：前向向量为 (1,0)，使用 atan2(横向, 前向)
        // double angle = std::atan2(p.y, p.x); // p.x 为横向（右正），p.y 为前向
        // if (std::abs(angle) <= angle_limit) {
        //     filtered->push_back(p);
        // }
    }

    ROS_INFO_STREAM("[Angle Filter] Input: " << cloud->size() << " -> Output: " << filtered->size()
                                             << " points (forward " << (angle_limit * 180.0 / M_PI) << " around +Y)");

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