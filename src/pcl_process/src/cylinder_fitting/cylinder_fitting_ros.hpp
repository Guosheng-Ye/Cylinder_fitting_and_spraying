#include "cylinder_fitting_node.hpp"
#include <geometry_msgs/Point.h>
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_process/TargetPoseList.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>

#include "lidar2flexed_arm.hpp"
#include <pcl_process/TargetInfo.h>
#include <pcl_process/TargetPoseList.h>
class CylinderFittingROS {
public:
    CylinderFittingROS(ros::NodeHandle &nh)
        : fitter_(new CylinderFitter(""))
        , tf_lidar2_base_(new FlexedArmTF()) {
        // 主控命令订阅
        main_ctl_sub_ = nh.subscribe("/achieve_work_pose", 1, &CylinderFittingROS::mainCtlCallback, this);
        ROS_INFO("CylinderFittingROS: Waiting for main control messages...");

        // 雷达点云数据订阅
        lidar_data_sub_ = nh.subscribe("/rslidar_filtered", 1, &CylinderFittingROS::cloudCallback, this);
        ROS_INFO("CylinderFittingROS: Waiting for filtered point clouds...");

        // 曲臂切片目标位姿发布
        target_pose_pub_ = nh.advertise<pcl_process::TargetPoseList>("/target_pose_list", 1, true);

        // 喷涂作业目标位姿发布
        spray_target_pose_pub_ = nh.advertise<pcl_process::TargetInfo>("/target_info", 1, true);
    }

public:
    void setOption();

private:
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
    void mainCtlCallback(const std_msgs::Bool &msg);
    void publishTargetPose();

private:
    ros::Subscriber                 lidar_data_sub_; // 雷达数据订阅
    ros::Subscriber                 main_ctl_sub_;   //主控命令订阅
    std::shared_ptr<CylinderFitter> fitter_;         // 圆柱拟合
    std::atomic<bool>               processing_{false};

    ros::Publisher target_pose_pub_;       //曲臂切片目标位姿发布
    ros::Publisher spray_target_pose_pub_; //喷涂作业目标位姿发布

    std::shared_ptr<FlexedArmTF> tf_lidar2_base_; // 雷达到车辆基座坐标变换
};