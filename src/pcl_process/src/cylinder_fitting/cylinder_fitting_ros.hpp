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
#include <pcl_process/SprayInfo.h>
#include <pcl_process/TargetInfo.h>
#include <pcl_process/TargetPoseList.h>

#include <dynamic_reconfigure/server.h>
#include <pcl_process/CylinderDetectionConfig.h>
class CylinderFittingROS {
public:
    CylinderFittingROS(ros::NodeHandle &nh)
        : fitter_(new CylinderFitter(""))
        , tf_lidar2_base_(new FlexedArmTF())
        , nh_(nh) {

        // 从YAML配置文件/参数服务器加载所有检测参数
        loadParams();
        ROS_INFO("CylinderFittingROS: detection parameters loaded.");

        // 主控命令订阅 (增大队列到10)
        main_ctl_sub_ = nh.subscribe("/achieve_work_pose", 10, &CylinderFittingROS::mainCtlCallback, this);
        ROS_INFO("CylinderFittingROS: Subscribed to /achieve_work_pose");

        // 雷达点云数据订阅 source:front_arm_lidar
        lidar_data_sub_ = nh.subscribe("/rslidar_filtered_cylinder", 10, &CylinderFittingROS::cloudCallback, this);
        ROS_INFO("CylinderFittingROS: Subscribed to /rslidar_filtered_cylinder");

        // 工艺参数订阅 process_param_bridge
        process_param_sub_ = nh.subscribe("/process_param_bridge", 10, &CylinderFittingROS::processParamsCallback, this);
        ROS_INFO("CylinderFittingROS: Subscribed to /process_param_bridge");

        // 工艺参数发布 process_param
        process_param_pub_ = nh.advertise<pcl_process::ProcessParam>("/process_param", 1, true);
        ROS_INFO("CylinderFittingROS: Ready to publish process parameters...");

        // 曲臂切片目标位姿发布
        target_pose_pub_ = nh.advertise<pcl_process::TargetPoseList>("/target_pose_list", 1, true);
        ROS_INFO("CylinderFittingROS: Ready to publish target pose list...");

        // 喷涂作业目标位姿发布
        spray_target_pose_pub_ = nh.advertise<pcl_process::TargetInfo>("/target_info", 1, true);
        ROS_INFO("CylinderFittingROS: Ready to publish spray target info...");

        // 喷涂作业区域面积发布
        spray_area_pub_ = nh.advertise<pcl_process::SprayInfo>("/spray_info", 1, true);
        ROS_INFO("CylinderFittingROS: Ready to publish spray area info...");

        // 初始化 dynamic_reconfigure 服务器
        dyn_reconf_server_ = std::make_shared<dynamic_reconfigure::Server<pcl_process::CylinderDetectionConfig>>(nh_);

        // 将 YAML 加载的参数回写给 dynamic_reconfigure，防止初始回调用 .cfg 默认值覆盖
        pcl_process::CylinderDetectionConfig init_config;
        init_config.min_inliers              = loaded_min_inliers_;
        init_config.max_inliers              = loaded_max_inliers_;
        init_config.min_inlier_ratio         = loaded_min_inlier_ratio_;
        init_config.max_inlier_ratio         = loaded_max_inlier_ratio_;
        init_config.min_height_m             = loaded_min_height_m_;
        init_config.min_radius_m             = loaded_min_radius_m_;
        init_config.max_radius_m             = loaded_max_radius_m_;
        init_config.min_radial_rel_std       = loaded_min_radial_rel_std_;
        init_config.max_radial_rel_std       = loaded_max_radial_rel_std_;
        init_config.eps_angle_deg            = loaded_eps_angle_deg_;
        init_config.use_ideal_axis           = loaded_use_ideal_axis_;
        init_config.axis_angle_threshold_deg = loaded_axis_angle_threshold_deg_;
        init_config.virtual_base_to_surface  = virtual_base_to_surface_;

        // RANSAC 拟合参数
        init_config.normal_distance_weight    = loaded_normal_distance_weight_;
        init_config.ransac_max_iterations     = loaded_ransac_max_iterations_;
        init_config.ransac_distance_threshold = loaded_ransac_distance_threshold_;

        // 圆柱验证阈值
        init_config.max_radial_range_ratio = loaded_max_radial_range_ratio_;
        init_config.min_aspect_ratio       = loaded_min_aspect_ratio_;
        init_config.min_circularity        = loaded_min_circularity_;

        // 法线估计参数
        init_config.normal_k_search = loaded_normal_k_search_;

        // 墙面移除参数
        init_config.wall_angle_threshold    = loaded_wall_angle_threshold_;
        init_config.wall_distance_threshold = loaded_wall_distance_threshold_;
        init_config.wall_max_iterations     = loaded_wall_max_iterations_;
        init_config.max_walls               = loaded_max_walls_;
        init_config.wall_min_plane_points   = loaded_wall_min_plane_points_;
        init_config.wall_min_keep_ratio     = loaded_wall_min_keep_ratio_;

        dyn_reconf_server_->updateConfig(init_config);

        dyn_reconf_server_->setCallback(
            boost::bind(&CylinderFittingROS::dynamicReconfigureCallback, this, _1, _2));
        ROS_INFO("CylinderFittingROS: dynamic_reconfigure server initialized (synced with YAML).");
    }

public:
    void setOption(const float distance, const float step_width, const float step_height, const float height,
                   const float lap, const double arm_reach, const int mode, const int k);

private:
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
    void mainCtlCallback(const std_msgs::Bool::ConstPtr &msg);
    void processParamsCallback(const pcl_process::ProcessParam::ConstPtr &msg);

    // 从ROS参数加载检测参数
    void loadParams();
    // dynamic_reconfigure 回调
    void dynamicReconfigureCallback(pcl_process::CylinderDetectionConfig &config, uint32_t level);
    // 将缓存的模型参数 + 工艺参数合并后设置到 fitter
    void applyFittingParams();

private:
    ros::NodeHandle                 nh_;                // 节点句柄
    ros::Subscriber                 lidar_data_sub_;    // 雷达数据订阅
    ros::Subscriber                 main_ctl_sub_;      // 主控命令订阅
    ros::Subscriber                 process_param_sub_; // 工艺参数订阅
    ros::Publisher                  process_param_pub_; // 工艺参数发布
    std::shared_ptr<CylinderFitter> fitter_;            // 圆柱拟合
    std::atomic<bool>               processing_{false};

    ros::Publisher target_pose_pub_;       //曲臂切片目标位姿发布
    ros::Publisher spray_target_pose_pub_; //喷涂作业目标位姿发布
    ros::Publisher spray_area_pub_;        //喷涂作业区域面积发布

    std::shared_ptr<FlexedArmTF> tf_lidar2_base_; // 雷达到车辆基座坐标变换

    // dynamic_reconfigure 服务器
    std::shared_ptr<dynamic_reconfigure::Server<pcl_process::CylinderDetectionConfig>> dyn_reconf_server_;

    // 记录历史值（工艺参数，来自processParamsCallback）
    double last_distance_    = -1.0;
    double last_step_width_  = -1.0;
    double last_step_height_ = -1.0;
    double last_height_      = -1.0;
    double last_lap_         = -1.0;
    double last_arm_reach_   = -1;
    int    last_mode_        = -1;
    int    last_k_           = -1;
    int    last_w0_          = -1;

    // 缓存的模型参数（来自 YAML / dynamic_reconfigure）
    int    loaded_min_inliers_{300};
    int    loaded_max_inliers_{600};
    double loaded_min_inlier_ratio_{0.02};
    double loaded_max_inlier_ratio_{0.50};
    double loaded_min_height_m_{1.00};
    double loaded_min_radius_m_{0.30};
    double loaded_max_radius_m_{0.40};
    double loaded_min_radial_rel_std_{0.00};
    double loaded_max_radial_rel_std_{0.15};
    bool   loaded_use_ideal_axis_{true};
    double loaded_axis_angle_threshold_deg_{15.0};
    double loaded_eps_angle_deg_{10.0};

    // RANSAC 拟合参数
    double loaded_normal_distance_weight_{0.1};
    int    loaded_ransac_max_iterations_{10000};
    double loaded_ransac_distance_threshold_{0.015};

    // 圆柱验证阈值
    double loaded_max_radial_range_ratio_{0.30};
    double loaded_min_aspect_ratio_{1.5};
    double loaded_min_circularity_{0.15};

    // 法线估计参数
    int    loaded_normal_k_search_{50};

    // 墙面移除参数
    double loaded_wall_angle_threshold_{15.0};
    double loaded_wall_distance_threshold_{0.02};
    int    loaded_wall_max_iterations_{5000};
    int    loaded_max_walls_{2};
    int    loaded_wall_min_plane_points_{100};
    double loaded_wall_min_keep_ratio_{0.15};

    // 从 YAML / dynamic_reconfigure 读取的参数
    double virtual_base_to_surface_{600.0}; // 虚拟原点到圆柱表面的距离 (mm)
};