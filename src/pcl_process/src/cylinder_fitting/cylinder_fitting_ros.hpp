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
#include <std_msgs/Float64.h>

#include "lidar2flexed_arm.hpp"
#include <pcl_process/ProcessParam.h>
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

        // 工艺参数订阅 process_param_bridge
        process_param_sub_ = nh.subscribe("/process_param_bridge", 1, &CylinderFittingROS::processParamsCallback, this);
        ROS_INFO("CylinderFittingROS: Waiting for process parameters...");

        // 工艺参数发布 process_param
        process_param_pub_ = nh.advertise<pcl_process::ProcessParam>("/process_param", 1, true);
        ROS_INFO("CylinderFittingROS: Ready to publish process parameters...");

        // 曲臂切片目标位姿发布
        target_pose_pub_ = nh.advertise<pcl_process::TargetPoseList>("/target_pose_list", 1, true);

        // 喷涂作业目标位姿发布
        spray_target_pose_pub_ = nh.advertise<pcl_process::TargetInfo>("/target_info", 1, true);

        //
    }

public:
    void setOption(const float distance, const float step_width, const float step_height, const float height,
                   const float lap, const int arm_reach, const int mode, const int k);

private:
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
    void mainCtlCallback(const std_msgs::Bool &msg);
    void processParamsCallback(const pcl_process::ProcessParam &msg);

private:
    ros::Subscriber                 lidar_data_sub_;    // 雷达数据订阅
    ros::Subscriber                 main_ctl_sub_;      //主控命令订阅
    ros::Subscriber                 process_param_sub_; //工艺参数订阅
    ros::Publisher                  process_param_pub_; //工艺参数发布
    std::shared_ptr<CylinderFitter> fitter_;            // 圆柱拟合
    std::atomic<bool>               processing_{false};

    ros::Publisher target_pose_pub_;       //曲臂切片目标位姿发布
    ros::Publisher spray_target_pose_pub_; //喷涂作业目标位姿发布

    std::shared_ptr<FlexedArmTF> tf_lidar2_base_; // 雷达到车辆基座坐标变换

    // 记录历史值
    double last_distance_   = -1.0;
    double last_step_width_ = -1.0;
    double last_height_     = -1.0;
    double last_lap_        = -1.0;
    int    last_arm_reach_  = -1;
    int    last_mode_       = -1;
    int    last_k_          = -1;
    int    last_w0_         = -1;
};