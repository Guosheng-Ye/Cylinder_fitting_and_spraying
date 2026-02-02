#include "ibeam_fitting_ros.hpp"
#include <pcl_conversions/pcl_conversions.h>

IBeamFittingROS::IBeamFittingROS(ros::NodeHandle& nh) : nh_(nh) {
    fitter_ = std::make_shared<IBeamFitter>();

    // 订阅过滤后的雷达数据
    sub_ = nh_.subscribe("/rslidar_filtered", 1, &IBeamFittingROS::cloudCallback, this);
    
    // 发布可视化 Marker
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("ibeam_marker", 1);
    
    ROS_INFO("IBeamFittingROS initialized.");
}

IBeamFittingROS::~IBeamFittingROS() {}

void IBeamFittingROS::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    fitter_->setInputCloud(cloud);
    
    if (fitter_->compute()) {
        std::vector<double> pose = fitter_->getPose();
        publishMarker(pose);
        ROS_INFO("IBeam fitted successfully.");
    }
}

void IBeamFittingROS::publishMarker(const std::vector<double>& pose) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "rslidar"; // 根据实际 frame 修改
    marker.header.stamp = ros::Time::now();
    marker.ns = "ibeam";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    
    // 简单设置位置
    marker.pose.position.x = pose[0];
    marker.pose.position.y = pose[1];
    marker.pose.position.z = pose[2];
    marker.pose.orientation.w = 1.0;
    
    marker.scale.x = 0.5; // 示例尺寸
    marker.scale.y = 0.5;
    marker.scale.z = 1.0;
    
    marker.color.a = 1.0; 
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    
    marker_pub_.publish(marker);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ibeam_fitting_node");
    ros::NodeHandle nh("~");
    IBeamFittingROS node(nh);
    ros::spin();
    return 0;
}
