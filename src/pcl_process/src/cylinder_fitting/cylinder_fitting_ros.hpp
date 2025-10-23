#include "cylinder_fitting_node.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class CylinderFittingROS {
public:
    CylinderFittingROS(ros::NodeHandle &nh)
        : fitter_(new CylinderFitter("")) {
        sub_ = nh.subscribe("/rslidar_filtered", 1, &CylinderFittingROS::cloudCallback, this);
        ROS_INFO("CylinderFittingROS: Waiting for filtered point clouds...");
    }

private:
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);

private:
    ros::Subscriber                 sub_;
    std::shared_ptr<CylinderFitter> fitter_;
};