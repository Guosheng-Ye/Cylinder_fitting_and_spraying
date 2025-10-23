

#include "cylinder_fitting_ros.hpp"

void CylinderFittingROS::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    if (cloud->empty()) return;

    ROS_INFO("Received filtered cloud with %zu points", cloud->size());

    try {
        fitter_->run_with_rslidar(cloud);
    } catch (const std::exception &e) {
        ROS_ERROR("Cylinder fitting failed: %s", e.what());
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "cylinder_fitting_ros");
    ros::NodeHandle nh;

    ROS_INFO("CylinderFittingROS node started, waiting for filtered point clouds...");
    CylinderFittingROS node(nh);

    ros::spin();
    return 0;
}