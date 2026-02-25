#ifndef IBEAM_FITTING_ROS_HPP
#define IBEAM_FITTING_ROS_HPP

#include "ibeam_fitting_node.hpp"
#include "lidar2flexed_arm.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include <pcl_process/IBeamProcessParam.h>
#include <pcl_process/IBeamTargetInfo.h>
#include <pcl_process/TargetInfo.h>
#include <pcl_process/TargetPoseList.h>

class IBeamFittingROS {
public:
    IBeamFittingROS(ros::NodeHandle &nh)
        : nh_(nh)
        , fitter_(new IBeamFitter())
        , tf_lidar2_base_(new FlexedArmTF()) {

        ROS_INFO("IBeamFittingROS initialized.");

        // 加载参数
        loadParams();
        ROS_INFO("IBeamFittingROS parameters loaded.");

        // 订阅过滤后的雷达数据 /rslidar_filtered_ibeam
        lidar_data_sub_ = nh_.subscribe("/rslidar_filtered_ibeam", 1, &IBeamFittingROS::cloudCallback, this);
        ROS_INFO("IBeamFittingROS: Subscribed to /rslidar_filtered_ibeam");

        // 订阅主控命令 /achieve_ibeam_work
        main_ctl_sub_ = nh_.subscribe("/achieve_ibeam_work", 10, &IBeamFittingROS::mainCtlCallback, this);
        ROS_INFO("IBeamFittingROS: Subscribed to /achieve_ibeam_work");

        // 订阅工艺参数 /ibeam_process_param_bridge
        process_param_sub_ =
            nh_.subscribe("/ibeam_process_param_bridge", 10, &IBeamFittingROS::ibeamProcessParamCallback, this);
        ROS_INFO("IBeamFittingROS: Subscribed to /ibeam_process_param_bridge");

        // 发布工艺参数 /ibeam_process_param
        process_param_pub_ = nh_.advertise<pcl_process::IBeamProcessParam>("/ibeam_process_param", 10, true);
        ROS_INFO("IBeamFittingROS: Ready to publish process parameters...");

        // 发布目标位姿列表 /target_pose_list
        target_pose_pub_ = nh_.advertise<pcl_process::TargetPoseList>("/target_pose_list", 10, true);
        ROS_INFO("IBeamFittingROS: Ready to publish target pose list...");

        // 发布喷涂作业目标位姿 /spray_target_pose
        spray_target_pose_pub_ = nh_.advertise<pcl_process::IBeamTargetInfo>("/spray_target_pose", 10, true);
        ROS_INFO("IBeamFittingROS: Ready to publish spray target pose...");
    }

public:
    void setOption(const float distance, const float step_height, const float step_length, const float lap,
                   const float arm_reach, const int mode_length, const float length, const int mode_theta,
                   const float theta);

private:
    // 主控命令回调函数
    void mainCtlCallback(const std_msgs::Bool::ConstPtr &msg);
    // 回调函数
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
    void ibeamProcessParamCallback(const pcl_process::IBeamProcessParam::ConstPtr &msg);

    // 从ROS参数加载检测参数
    void loadParams();

    // 生成基座作业点
    void generateTargetPoses(const IBeamOutputs& outputs);

    ros::NodeHandle   nh_;
    ros::Subscriber   lidar_data_sub_;    // 订阅过滤后的雷达数据
    ros::Subscriber   main_ctl_sub_;      // 订阅主控命令
    ros::Subscriber   process_param_sub_; // 订阅工艺参数
    std::atomic<bool> processing_{false};

    ros::Publisher process_param_pub_;     // 发布工艺参数
    ros::Publisher target_pose_pub_;       // 发布目标位姿列表
    ros::Publisher spray_target_pose_pub_; // 发布喷涂作业目标位姿

    std::shared_ptr<IBeamFitter> fitter_;
    std::shared_ptr<FlexedArmTF> tf_lidar2_base_; // 雷达到车辆基座坐标变换

    // 记录历史值（用于检测参数变化）
    double last_distance_    = -1.0;
    double last_step_height_ = -1.0;
    double last_step_length_ = -1.0;
    double last_lap_         = -1.0;
    double last_arm_reach_   = -1.0;
    int    last_mode_length_ = -1;
    double last_length_      = -1.0;
    int    last_mode_theta_  = -1;
    double last_theta_       = -1.0;
};

#endif // IBEAM_FITTING_ROS_HPP
