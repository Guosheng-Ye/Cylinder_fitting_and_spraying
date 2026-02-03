#include "ibeam_fitting_ros.hpp"
#include <pcl_conversions/pcl_conversions.h>

IBeamFittingROS::IBeamFittingROS(ros::NodeHandle& nh) : nh_(nh) {
    
    fitter_ = std::make_shared<IBeamFitter>();
    
    // 加载参数
    loadParams();

    // 订阅过滤后的雷达数据
    std::string input_topic;
    nh_.param<std::string>("input_topic", input_topic, "/rslidar_filtered");
    sub_ = nh_.subscribe(input_topic, 1, &IBeamFittingROS::cloudCallback, this);
    
    ROS_INFO("IBeamFittingROS initialized.");
    ROS_INFO("  Input topic: %s", input_topic.c_str());
    ROS_INFO("  Visualization: PCL Viewer (no ROS topic publishing)");
}

IBeamFittingROS::~IBeamFittingROS() {}

void IBeamFittingROS::loadParams() {
    // 加载检测参数
    IBeamDetectionParams params;
    
    // 空间过滤参数
    nh_.param<double>("max_distance", params.max_distance, 6.0);
    nh_.param<double>("min_distance", params.min_distance, 2.0);
    nh_.param<double>("min_height", params.min_height, 1.5);
    
    // 预处理参数
    nh_.param<double>("voxel_size", params.voxel_size, 0.02);
    nh_.param<int>("sor_neighbors", params.sor_neighbors, 20);
    nh_.param<double>("sor_std_ratio", params.sor_std_ratio, 1.5);
    
    // 墙面移除参数
    nh_.param<double>("wall_distance_threshold", params.wall_distance_threshold, 0.08);
    nh_.param<int>("min_wall_points", params.min_wall_points, 500);
    
    // 聚类参数
    nh_.param<double>("cluster_tolerance", params.cluster_tolerance, 0.18);
    nh_.param<int>("min_cluster_size", params.min_cluster_size, 10);
    
    // 工字钢判断阈值
    nh_.param<double>("min_linearity", params.min_linearity, 0.8);
    nh_.param<double>("min_length", params.min_length, 1.5);
    nh_.param<double>("min_aspect_ratio", params.min_aspect_ratio, 2.5);
    nh_.param<double>("max_axis_z", params.max_axis_z, 0.2);
    
    fitter_->setParams(params);
    
    ROS_INFO("Detection params loaded:");
    ROS_INFO("  Distance range: [%.1f, %.1f] m", params.min_distance, params.max_distance);
    ROS_INFO("  Min height: %.1f m", params.min_height);
    ROS_INFO("  Min linearity: %.2f", params.min_linearity);
    ROS_INFO("  Min length: %.1f m", params.min_length);
}

void IBeamFittingROS::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    
    if (cloud->empty()) {
        ROS_WARN_THROTTLE(5.0, "Received empty point cloud");
        return;
    }

    fitter_->setInputCloud(cloud);
    
    if (fitter_->detect()) {
        std::vector<IBeamResult> results = fitter_->getResults();
        ROS_INFO("Detected %zu I-beams", results.size());
        
        // 使用PCL Viewer可视化结果
        fitter_->visualizeResults();
    } else {
        ROS_WARN_THROTTLE(5.0, "No I-beams detected in current frame");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ibeam_fitting_node");
    ros::NodeHandle nh("~");
    IBeamFittingROS node(nh);
    ros::spin();
    return 0;
}
