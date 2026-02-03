#ifndef IBEAM_FITTING_ROS_HPP
#define IBEAM_FITTING_ROS_HPP

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include "ibeam_fitting_node.hpp"

class IBeamFittingROS {
public:
    IBeamFittingROS(ros::NodeHandle& nh);
    ~IBeamFittingROS();

private:
    // 回调函数
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
    
    // 从ROS参数加载检测参数
    void loadParams();

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    
    std::shared_ptr<IBeamFitter> fitter_;
};

#endif // IBEAM_FITTING_ROS_HPP
