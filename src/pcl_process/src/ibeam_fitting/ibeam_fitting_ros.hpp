#ifndef IBEAM_FITTING_ROS_HPP
#define IBEAM_FITTING_ROS_HPP

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <pcl_ros/point_cloud.h>
#include "ibeam_fitting_node.hpp"

class IBeamFittingROS {
public:
    IBeamFittingROS(ros::NodeHandle& nh);
    ~IBeamFittingROS();

private:
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void publishMarker(const std::vector<double>& pose);

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher marker_pub_;
    
    std::shared_ptr<IBeamFitter> fitter_;
};

#endif // IBEAM_FITTING_ROS_HPP
