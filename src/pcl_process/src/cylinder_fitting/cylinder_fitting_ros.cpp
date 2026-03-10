

#include "cylinder_fitting_ros.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <atomic>
#include <set>

/// @brief 从ROS参数服务器加载检测参数（YAML配置文件 + launch文件设置）
void CylinderFittingROS::loadParams() {
    // 圆柱模型拟合参数
    int min_inliers, max_inliers;
    nh_.param<int>("min_inliers", min_inliers, 300);
    nh_.param<int>("max_inliers", max_inliers, 600);

    double min_inlier_ratio, max_inlier_ratio;
    nh_.param<double>("min_inlier_ratio", min_inlier_ratio, 0.02);
    nh_.param<double>("max_inlier_ratio", max_inlier_ratio, 0.50);

    double min_height_m, min_radius_m, max_radius_m;
    nh_.param<double>("min_height_m", min_height_m, 1.00);
    nh_.param<double>("min_radius_m", min_radius_m, 0.30);
    nh_.param<double>("max_radius_m", max_radius_m, 0.40);

    double min_radial_rel_std, max_radial_rel_std;
    nh_.param<double>("min_radial_rel_std", min_radial_rel_std, 0.00);
    nh_.param<double>("max_radial_rel_std", max_radial_rel_std, 0.15);

    // 轴向校正参数
    bool   use_ideal_axis;
    double axis_angle_threshold_deg;
    nh_.param<bool>("use_ideal_axis", use_ideal_axis, true);
    nh_.param<double>("axis_angle_threshold_deg", axis_angle_threshold_deg, 15.0);

    // RANSAC 轴向约束参数
    double eps_angle_deg;
    nh_.param<double>("eps_angle_deg", eps_angle_deg, 10.0);

    // RANSAC 参考轴方向 (竖直立柱默认 0,0,1; 倾斜圆柱设为实际轴向)
    double ransac_axis_x, ransac_axis_y, ransac_axis_z;
    nh_.param<double>("ransac_axis_x", ransac_axis_x, 0.0);
    nh_.param<double>("ransac_axis_y", ransac_axis_y, 0.0);
    nh_.param<double>("ransac_axis_z", ransac_axis_z, 1.0);

    // RANSAC 拟合参数
    double normal_distance_weight;
    int    ransac_max_iterations;
    double ransac_distance_threshold;
    nh_.param<double>("normal_distance_weight", normal_distance_weight, 0.1);
    nh_.param<int>("ransac_max_iterations", ransac_max_iterations, 10000);
    nh_.param<double>("ransac_distance_threshold", ransac_distance_threshold, 0.015);

    // 圆柱验证阈值
    double max_radial_range_ratio, min_aspect_ratio, min_circularity;
    nh_.param<double>("max_radial_range_ratio", max_radial_range_ratio, 0.30);
    nh_.param<double>("min_aspect_ratio", min_aspect_ratio, 1.5);
    nh_.param<double>("min_circularity", min_circularity, 0.15);
    double min_center_xy_dist_m;
    nh_.param<double>("min_center_xy_dist_m", min_center_xy_dist_m, 0.0);

    // 法线估计参数
    int normal_k_search;
    nh_.param<int>("normal_k_search", normal_k_search, 50);

    // 墙面移除参数
    double wall_angle_threshold, wall_distance_threshold, wall_min_keep_ratio;
    int    wall_max_iterations, max_walls, wall_min_plane_points;
    nh_.param<double>("wall_angle_threshold", wall_angle_threshold, 15.0);
    nh_.param<double>("wall_distance_threshold", wall_distance_threshold, 0.02);
    nh_.param<int>("wall_max_iterations", wall_max_iterations, 5000);
    nh_.param<int>("max_walls", max_walls, 2);
    nh_.param<int>("wall_min_plane_points", wall_min_plane_points, 100);
    nh_.param<double>("wall_min_keep_ratio", wall_min_keep_ratio, 0.15);

    // 虚拟原点参数
    nh_.param<double>("virtual_base_to_surface", virtual_base_to_surface_, 600.0);

    // 保存模型参数到成员变量（供 setOption 使用）
    loaded_min_inliers_              = min_inliers;
    loaded_max_inliers_              = max_inliers;
    loaded_min_inlier_ratio_         = min_inlier_ratio;
    loaded_max_inlier_ratio_         = max_inlier_ratio;
    loaded_min_height_m_             = min_height_m;
    loaded_min_radius_m_             = min_radius_m;
    loaded_max_radius_m_             = max_radius_m;
    loaded_min_radial_rel_std_       = min_radial_rel_std;
    loaded_max_radial_rel_std_       = max_radial_rel_std;
    loaded_use_ideal_axis_           = use_ideal_axis;
    loaded_axis_angle_threshold_deg_ = axis_angle_threshold_deg;
    loaded_eps_angle_deg_            = eps_angle_deg;
    loaded_ransac_axis_x_            = ransac_axis_x;
    loaded_ransac_axis_y_            = ransac_axis_y;
    loaded_ransac_axis_z_            = ransac_axis_z;

    // RANSAC 拟合参数
    loaded_normal_distance_weight_    = normal_distance_weight;
    loaded_ransac_max_iterations_     = ransac_max_iterations;
    loaded_ransac_distance_threshold_ = ransac_distance_threshold;

    // 圆柱验证阈值
    loaded_max_radial_range_ratio_ = max_radial_range_ratio;
    loaded_min_aspect_ratio_       = min_aspect_ratio;
    loaded_min_circularity_        = min_circularity;
    loaded_min_center_xy_dist_m_   = min_center_xy_dist_m;

    // 法线估计参数
    loaded_normal_k_search_ = normal_k_search;

    // 墙面移除参数
    loaded_wall_angle_threshold_    = wall_angle_threshold;
    loaded_wall_distance_threshold_ = wall_distance_threshold;
    loaded_wall_max_iterations_     = wall_max_iterations;
    loaded_max_walls_               = max_walls;
    loaded_wall_min_plane_points_   = wall_min_plane_points;
    loaded_wall_min_keep_ratio_     = wall_min_keep_ratio;

    ROS_INFO("Cylinder detection params loaded:");
    ROS_INFO("  [Model]  Inliers: [%d, %d], Ratio: [%.3f, %.3f]", min_inliers, max_inliers, min_inlier_ratio,
             max_inlier_ratio);
    ROS_INFO("  [Model]  Height: %.2f m, Radius: [%.2f, %.2f] m", min_height_m, min_radius_m, max_radius_m);
    ROS_INFO("  [Model]  Radial std: [%.3f, %.3f]", min_radial_rel_std, max_radial_rel_std);
    ROS_INFO(
        "  [RANSAC] Eps angle: %.1f deg, Axis: [%.3f,%.3f,%.3f], NormalWeight: %.3f, MaxIter: %d, DistThresh: %.4f",
        eps_angle_deg, ransac_axis_x, ransac_axis_y, ransac_axis_z, normal_distance_weight, ransac_max_iterations,
        ransac_distance_threshold);
    ROS_INFO("  [Valid]  RadialRange: %.2f, AspectRatio: %.1f, Circularity: %.2f", max_radial_range_ratio,
             min_aspect_ratio, min_circularity);
    ROS_INFO("  [Normal] K-search: %d", normal_k_search);
    ROS_INFO("  [Wall]   Angle: %.1f deg, Dist: %.3f, MaxIter: %d, MaxWalls: %d, MinPts: %d, KeepRatio: %.2f",
             wall_angle_threshold, wall_distance_threshold, wall_max_iterations, max_walls, wall_min_plane_points,
             wall_min_keep_ratio);
    ROS_INFO("  [Axis]   Use ideal: %s, Threshold: %.1f deg", use_ideal_axis ? "true" : "false",
             axis_angle_threshold_deg);
    ROS_INFO("  [VBase]  Virtual base to surface: %.1f mm", virtual_base_to_surface_);
}

/// @brief 根据工作对象名称切换参数预设，并同步更新 dynamic_reconfigure
void CylinderFittingROS::switchPreset(const std::string &object) {
    const std::string ns = "preset_" + object + "/"; // e.g. "preset_A/"

    nh_.param<int>(ns + "min_inliers", loaded_min_inliers_, 300);
    nh_.param<int>(ns + "max_inliers", loaded_max_inliers_, 600);
    nh_.param<double>(ns + "min_inlier_ratio", loaded_min_inlier_ratio_, 0.02);
    nh_.param<double>(ns + "max_inlier_ratio", loaded_max_inlier_ratio_, 0.50);
    nh_.param<double>(ns + "min_height_m", loaded_min_height_m_, 1.00);
    nh_.param<double>(ns + "min_radius_m", loaded_min_radius_m_, 0.30);
    nh_.param<double>(ns + "max_radius_m", loaded_max_radius_m_, 0.40);
    nh_.param<double>(ns + "min_radial_rel_std", loaded_min_radial_rel_std_, 0.00);
    nh_.param<double>(ns + "max_radial_rel_std", loaded_max_radial_rel_std_, 0.15);
    nh_.param<double>(ns + "ransac_axis_x", loaded_ransac_axis_x_, 0.0);
    nh_.param<double>(ns + "ransac_axis_y", loaded_ransac_axis_y_, 0.0);
    nh_.param<double>(ns + "ransac_axis_z", loaded_ransac_axis_z_, 1.0);
    nh_.param<double>(ns + "eps_angle_deg", loaded_eps_angle_deg_, 15.0);
    nh_.param<double>(ns + "normal_distance_weight", loaded_normal_distance_weight_, 0.05);
    nh_.param<int>(ns + "ransac_max_iterations", loaded_ransac_max_iterations_, 20000);
    nh_.param<double>(ns + "ransac_distance_threshold", loaded_ransac_distance_threshold_, 0.015);
    nh_.param<double>(ns + "max_radial_range_ratio", loaded_max_radial_range_ratio_, 0.30);
    nh_.param<double>(ns + "min_aspect_ratio", loaded_min_aspect_ratio_, 1.5);
    nh_.param<double>(ns + "min_circularity", loaded_min_circularity_, 0.15);
    nh_.param<double>(ns + "min_center_xy_dist_m", loaded_min_center_xy_dist_m_, 0.0);
    nh_.param<int>(ns + "normal_k_search", loaded_normal_k_search_, 30);
    nh_.param<double>(ns + "wall_angle_threshold", loaded_wall_angle_threshold_, 20.0);
    nh_.param<double>(ns + "wall_distance_threshold", loaded_wall_distance_threshold_, 0.02);
    nh_.param<int>(ns + "wall_max_iterations", loaded_wall_max_iterations_, 5000);
    nh_.param<int>(ns + "max_walls", loaded_max_walls_, 3);
    nh_.param<int>(ns + "wall_min_plane_points", loaded_wall_min_plane_points_, 100);
    nh_.param<double>(ns + "wall_min_keep_ratio", loaded_wall_min_keep_ratio_, 0.10);
    nh_.param<bool>(ns + "use_ideal_axis", loaded_use_ideal_axis_, true);
    nh_.param<double>(ns + "axis_angle_threshold_deg", loaded_axis_angle_threshold_deg_, 15.0);
    nh_.param<double>(ns + "virtual_base_to_surface", virtual_base_to_surface_, 600.0);

    ROS_WARN("[switchPreset] Loaded preset_%s: radius=[%.2f,%.2f]m, eps=%.1fdeg, axis=[%.3f,%.3f,%.3f], iter=%d",
             object.c_str(), loaded_min_radius_m_, loaded_max_radius_m_, loaded_eps_angle_deg_, loaded_ransac_axis_x_,
             loaded_ransac_axis_y_, loaded_ransac_axis_z_, loaded_ransac_max_iterations_);

    // 同步更新 dynamic_reconfigure 面板
    pcl_process::CylinderDetectionConfig cfg;
    cfg.min_inliers               = loaded_min_inliers_;
    cfg.max_inliers               = loaded_max_inliers_;
    cfg.min_inlier_ratio          = loaded_min_inlier_ratio_;
    cfg.max_inlier_ratio          = loaded_max_inlier_ratio_;
    cfg.min_height_m              = loaded_min_height_m_;
    cfg.min_radius_m              = loaded_min_radius_m_;
    cfg.max_radius_m              = loaded_max_radius_m_;
    cfg.min_radial_rel_std        = loaded_min_radial_rel_std_;
    cfg.max_radial_rel_std        = loaded_max_radial_rel_std_;
    cfg.ransac_axis_x             = loaded_ransac_axis_x_;
    cfg.ransac_axis_y             = loaded_ransac_axis_y_;
    cfg.ransac_axis_z             = loaded_ransac_axis_z_;
    cfg.eps_angle_deg             = loaded_eps_angle_deg_;
    cfg.normal_distance_weight    = loaded_normal_distance_weight_;
    cfg.ransac_max_iterations     = loaded_ransac_max_iterations_;
    cfg.ransac_distance_threshold = loaded_ransac_distance_threshold_;
    cfg.max_radial_range_ratio    = loaded_max_radial_range_ratio_;
    cfg.min_aspect_ratio          = loaded_min_aspect_ratio_;
    cfg.min_circularity           = loaded_min_circularity_;
    cfg.normal_k_search           = loaded_normal_k_search_;
    cfg.wall_angle_threshold      = loaded_wall_angle_threshold_;
    cfg.wall_distance_threshold   = loaded_wall_distance_threshold_;
    cfg.wall_max_iterations       = loaded_wall_max_iterations_;
    cfg.max_walls                 = loaded_max_walls_;
    cfg.wall_min_plane_points     = loaded_wall_min_plane_points_;
    cfg.wall_min_keep_ratio       = loaded_wall_min_keep_ratio_;
    cfg.use_ideal_axis            = loaded_use_ideal_axis_;
    cfg.axis_angle_threshold_deg  = loaded_axis_angle_threshold_deg_;
    cfg.virtual_base_to_surface   = virtual_base_to_surface_;
    dyn_reconf_server_->updateConfig(cfg);

    // 立即应用到 fitter
    applyFittingParams();
}

/// @brief dynamic_reconfigure 回调 - 运行时动态更新检测参数
void CylinderFittingROS::dynamicReconfigureCallback(pcl_process::CylinderDetectionConfig &config, uint32_t level) {
    ROS_INFO("=== dynamic_reconfigure: Updating cylinder detection parameters (level=%u) ===", level);

    // 更新缓存的模型参数
    loaded_min_inliers_              = config.min_inliers;
    loaded_max_inliers_              = config.max_inliers;
    loaded_min_inlier_ratio_         = config.min_inlier_ratio;
    loaded_max_inlier_ratio_         = config.max_inlier_ratio;
    loaded_min_height_m_             = config.min_height_m;
    loaded_min_radius_m_             = config.min_radius_m;
    loaded_max_radius_m_             = config.max_radius_m;
    loaded_min_radial_rel_std_       = config.min_radial_rel_std;
    loaded_max_radial_rel_std_       = config.max_radial_rel_std;
    loaded_use_ideal_axis_           = config.use_ideal_axis;
    loaded_axis_angle_threshold_deg_ = config.axis_angle_threshold_deg;
    loaded_eps_angle_deg_            = config.eps_angle_deg;
    loaded_ransac_axis_x_            = config.ransac_axis_x;
    loaded_ransac_axis_y_            = config.ransac_axis_y;
    loaded_ransac_axis_z_            = config.ransac_axis_z;
    virtual_base_to_surface_         = config.virtual_base_to_surface;

    // RANSAC 拟合参数
    loaded_normal_distance_weight_    = config.normal_distance_weight;
    loaded_ransac_max_iterations_     = config.ransac_max_iterations;
    loaded_ransac_distance_threshold_ = config.ransac_distance_threshold;

    // 圆柱验证阈值
    loaded_max_radial_range_ratio_ = config.max_radial_range_ratio;
    loaded_min_aspect_ratio_       = config.min_aspect_ratio;
    loaded_min_circularity_        = config.min_circularity;

    // 法线估计参数
    loaded_normal_k_search_ = config.normal_k_search;

    // 墙面移除参数
    loaded_wall_angle_threshold_    = config.wall_angle_threshold;
    loaded_wall_distance_threshold_ = config.wall_distance_threshold;
    loaded_wall_max_iterations_     = config.wall_max_iterations;
    loaded_max_walls_               = config.max_walls;
    loaded_wall_min_plane_points_   = config.wall_min_plane_points;
    loaded_wall_min_keep_ratio_     = config.wall_min_keep_ratio;

    // 用当前工艺参数 + 更新后的模型参数重新设置 fitter
    applyFittingParams();

    ROS_INFO("Cylinder detection parameters updated via dynamic_reconfigure.");
}

/// @brief 将缓存的模型参数 + 工艺参数合并后设置到 fitter
void CylinderFittingROS::applyFittingParams() {
    CylinderFitter::CylinderFitterParams params;

    // 模型参数（来自 YAML / dynamic_reconfigure）
    params.min_inliers        = loaded_min_inliers_;
    params.max_inliers        = loaded_max_inliers_;
    params.min_inlier_ratio   = loaded_min_inlier_ratio_;
    params.max_inlier_ratio   = loaded_max_inlier_ratio_;
    params.min_height_m       = loaded_min_height_m_;
    params.min_radius_m       = loaded_min_radius_m_;
    params.max_radius_m       = loaded_max_radius_m_;
    params.min_radial_rel_std = loaded_min_radial_rel_std_;
    params.max_radial_rel_std = loaded_max_radial_rel_std_;

    // 轴向校正参数（来自 YAML / dynamic_reconfigure）
    params.eps_angle_deg_            = loaded_eps_angle_deg_;
    params.ransac_axis_x_            = loaded_ransac_axis_x_;
    params.ransac_axis_y_            = loaded_ransac_axis_y_;
    params.ransac_axis_z_            = loaded_ransac_axis_z_;
    params.use_ideal_axis_           = loaded_use_ideal_axis_;
    params.axis_angle_threshold_deg_ = loaded_axis_angle_threshold_deg_;

    // RANSAC 拟合参数
    params.normal_distance_weight_    = loaded_normal_distance_weight_;
    params.ransac_max_iterations_     = loaded_ransac_max_iterations_;
    params.ransac_distance_threshold_ = loaded_ransac_distance_threshold_;

    // 圆柱验证阈值
    params.max_radial_range_ratio_ = loaded_max_radial_range_ratio_;
    params.min_aspect_ratio_       = loaded_min_aspect_ratio_;
    params.min_circularity_        = loaded_min_circularity_;
    params.min_center_xy_dist_m    = loaded_min_center_xy_dist_m_;

    // 法线估计参数
    params.normal_k_search_ = loaded_normal_k_search_;

    // 墙面移除参数
    params.wall_angle_threshold_    = loaded_wall_angle_threshold_;
    params.wall_distance_threshold_ = loaded_wall_distance_threshold_;
    params.wall_max_iterations_     = loaded_wall_max_iterations_;
    params.max_walls_               = loaded_max_walls_;
    params.wall_min_plane_points_   = loaded_wall_min_plane_points_;
    params.wall_min_keep_ratio_     = loaded_wall_min_keep_ratio_;

    // 虚拟原点参数
    params.virtual_base_to_surface_ = virtual_base_to_surface_;

    // 工艺参数（来自 processParamsCallback / setOption）
    params.slice_distance_ = last_distance_ > 0 ? last_distance_ : 750.0;
    params.height_         = last_height_ > 0 ? last_height_ : 600.0;
    params.step_width_     = last_step_width_ > 0 ? last_step_width_ : 300.0;
    params.step_height_    = last_step_height_ > 0 ? last_step_height_ : 200.0;
    params.r_initial_      = last_lap_ > 0 ? last_lap_ : 0.5;
    params.robot_R_arm_    = last_arm_reach_ > 0 ? last_arm_reach_ : 1500.0;
    params.work_mode_      = last_mode_ > 0 ? last_mode_ : 2;
    params.k_              = last_k_ >= 0 ? last_k_ : 0;

    this->fitter_->setFittingParams(params);
}

/// @brief 点云回调
/// @param msg
void CylinderFittingROS::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    try {
        static std::atomic<bool> logged_disabled{false};
        bool                     processing = processing_.load(std::memory_order_acquire);

        if (!processing) {
            if (!logged_disabled.load(std::memory_order_relaxed)) {
                ROS_INFO("Cylinder fitting is currently disabled by main control command.");
                logged_disabled.store(true, std::memory_order_relaxed);
            }
            return;
        } else {
            if (logged_disabled.load(std::memory_order_relaxed)) {
                logged_disabled.store(false, std::memory_order_relaxed);
            }
        }

        ROS_ERROR("Cylinder fitting is currently enabled by main control command.");

        if (fitter_->run_with_rslidar(cloud) != -1) {
            // 立即停止，防止 ros::spin() 单线程延迟导致队列中的下一帧触发重复检测
            processing_.store(false, std::memory_order_release);
            ROS_WARN("Cylinder detection done (one-shot), processing stopped.");
            ROS_ERROR("Publishing target pose list...");
            std::vector<CylinderFitter::SliceData> slices = fitter_->getSliceResults();
            pcl_process::TargetPoseList            tpl_msg;
            pcl_process::TargetInfo                ti_msg;

            // Z轴工作高度约束
            const double MIN_WORK_HEIGHT_MM = 0; // 2000.0; // Z 轴统一抬高的高度 (mm)
            // const double SPRAY_OFFSET_MM    = 1150.0; // 喷涂工作点偏移距离 (mm)

            // 转换圆柱底部和顶部中心 (m -> mm) 到 Base 坐标系
            Eigen::Vector3d pos_bottom_center(fitter_->bottomCenter().x() * 1000.0,
                                              fitter_->bottomCenter().y() * 1000.0,
                                              fitter_->bottomCenter().z() * 1000.0);
            Eigen::Vector3d bottom_center_base_mm = tf_lidar2_base_->to_base_from_lidar(pos_bottom_center);

            Eigen::Vector3d pos_top_center(fitter_->topCenter().x() * 1000.0, fitter_->topCenter().y() * 1000.0,
                                           fitter_->topCenter().z() * 1000.0);
            Eigen::Vector3d top_center_base_mm = tf_lidar2_base_->to_base_from_lidar(pos_top_center);

            // 计算原始圆柱在 Base 坐标系上的 Z 轴高度范围
            double cylinder_max_z_base_mm = std::max(bottom_center_base_mm.z(), top_center_base_mm.z());

            double working_max_z_base_mm = cylinder_max_z_base_mm;

            ROS_INFO("Cylinder Original Max Z in Base (mm): %.2f", cylinder_max_z_base_mm);
            ROS_INFO("Working Max Z (Original + Offset) in Base (mm): %.2f", working_max_z_base_mm);
            ROS_INFO("Z Offset Applied (m): %.3f", MIN_WORK_HEIGHT_MM / 1000.0);

            // end of 高度限制

            tpl_msg.targets.reserve(slices.size());

            // ========== 预处理：确定哪些高度层是有效的 ==========
            // 同一层只要有一个切片在 Z 范围内，整层都保留
            std::set<int> valid_layers;
            for (const auto &slice : slices) {
                // 坐标转换：Lidar (mm) -> Base (mm)
                Eigen::Vector3d pos_spray_center(slice.center_mm.x(), slice.center_mm.y(), slice.center_mm.z());
                Eigen::Vector3d A_base_mm               = tf_lidar2_base_->to_base_from_lidar(pos_spray_center);
                double          target_z_base_mm_raised = A_base_mm.z() + MIN_WORK_HEIGHT_MM;

                // 如果该切片在 Z 范围内，标记该层为有效
                if (target_z_base_mm_raised <= working_max_z_base_mm) {
                    valid_layers.insert(slice.layer_index);
                }
            }
            ROS_INFO("Valid layers count: %zu (layers with at least one valid slice)", valid_layers.size());

            int published_id_counter = 0; //有效切片id号

            for (const auto &slice : slices) { //切片中心
                // 坐标转换：Lidar (mm) -> Base (mm)
                Eigen::Vector3d pos_spray_center(slice.center_mm.x(), slice.center_mm.y(), slice.center_mm.z());
                Eigen::Vector3d pos_circle_center(slice.circle_center_mm.x(), slice.circle_center_mm.y(),
                                                  slice.circle_center_mm.z());

                // 表面点 A (Base mm)
                Eigen::Vector3d A_base_mm = tf_lidar2_base_->to_base_from_lidar(pos_spray_center);
                // 圆柱中心 C (Base mm)
                Eigen::Vector3d C_base_mm = tf_lidar2_base_->to_base_from_lidar(pos_circle_center);

                // 喷涂工作点B
                Eigen::Vector3d pos_nearpoint_pre_center(slice.work_mm.x(), slice.work_mm.y(), slice.work_mm.z());
                Eigen::Vector3d work_point =
                    tf_lidar2_base_->to_base_from_lidar(pos_nearpoint_pre_center); // 喷涂工作点

                ROS_ERROR("Source pos_circle_center: [%.2f, %.2f, %.2f], pos_spray_center: [%.3f,%.3f,%.3f] "
                          ",work_point: [%.3f, %.3f,%.3f]",
                          slice.circle_center_mm.x(), slice.circle_center_mm.y(), slice.circle_center_mm.z(),
                          slice.center_mm.x(), slice.center_mm.y(), slice.center_mm.z(), slice.work_mm.x(),
                          slice.work_mm.y(), slice.work_mm.z());

                // 统一 Z 轴抬高
                double target_z_base_mm_raised = A_base_mm.z() + MIN_WORK_HEIGHT_MM;
                double circle_z_base_mm_raised = C_base_mm.z() + MIN_WORK_HEIGHT_MM;
                double work_z_base_mm_raised   = work_point.z() + MIN_WORK_HEIGHT_MM;
                ROS_ERROR("Slice id(base mm): %d,  circle_center:[%.2f, %.2f, %.2f], spray_center:[%.3f, %.3f, %.3f], "
                          "work_point:[%.3f, %.3f, %.3f]",
                          slice.id + 1, C_base_mm.x(), C_base_mm.y(), circle_z_base_mm_raised, A_base_mm.x(),
                          A_base_mm.y(), target_z_base_mm_raised, work_point.x(), work_point.y(),
                          work_z_base_mm_raised);

                // 检查该层是否在有效层集合中（同一层只要有一个切片有效，整层都保留）
                if (valid_layers.find(slice.layer_index) == valid_layers.end()) {
                    ROS_WARN("Slice ID %d (layer %d) Z (%.2f mm) - entire layer exceeds max working Z (%.2f mm). "
                             "Skipping.\n",
                             slice.id + 1, slice.layer_index, target_z_base_mm_raised, working_max_z_base_mm);
                    continue;
                }

                // 对未丢弃的切片进行重新编号
                published_id_counter++;
                int new_sequential_id = published_id_counter;

                // 显式构造 A_final_mm 和 C_final_mm，确保 Z 坐标是抬高后的值**
                // 使用 (X_base, Y_base, Z_raised) 构造最终点
                Eigen::Vector3d A_final_mm(A_base_mm.x(), A_base_mm.y(), target_z_base_mm_raised);
                Eigen::Vector3d C_final_mm(C_base_mm.x(), C_base_mm.y(), circle_z_base_mm_raised);

                // 填充 TargetPose (给曲臂的消息，单位：m) - 使用 work_point
                pcl_process::TargetPose pose;
                pose.id = new_sequential_id;

                pose.x = work_point.x() / 1000.0; // mm -> m
                pose.y = work_point.y() / 1000.0;
                pose.z = work_z_base_mm_raised / 1000.0;
                // ROS_INFO(" Target ID: %d, Work Position in Base (m): [%.3f, %.3f, %.3f]", pose.id, pose.x, pose.y,
                //  pose.z);

                // 填充 TargetInfo (给喷涂目标信息的消息，单位：mm)
                ti_msg.id = new_sequential_id;

                // C圆柱中心 (使用抬高后的 Z)
                ti_msg.circle_center[0] = C_final_mm.x();
                ti_msg.circle_center[1] = C_final_mm.y();
                ti_msg.circle_center[2] = C_final_mm.z();

                // A喷涂中心 (使用抬高后的 Z)
                ti_msg.spray_center[0] = A_final_mm.x();
                ti_msg.spray_center[1] = A_final_mm.y();
                ti_msg.spray_center[2] = A_final_mm.z();

                // 圆柱轴向 (使用实际切片时的轴向，可能是理想轴向)
                Eigen::Vector3f axis_used         = fitter_->axisUsed();
                ti_msg.cylinder_axis_direction[0] = axis_used.x();
                ti_msg.cylinder_axis_direction[1] = axis_used.y();
                ti_msg.cylinder_axis_direction[2] = axis_used.z();
                // 半径
                ti_msg.radius = fitter_->radius() * 1000.0f; // m -> mm
                // 其他参数
                ti_msg.angle  = std::round((slice.angle_deg) * 1000.0) / 1000.0;
                ti_msg.height = fitter_->fittingParams().height_;

                ROS_WARN("Cylinder ID: %d, Work_point:[%.3f,%.3f,%.3f], Circle:[%.3f,%.3f,%.3f], => "
                         "Spray_center:[%.3f,%.3f,%.3f], "
                         "Cylinder_axis:[%.3f,%.3f,%.3f], Radius: %.3f mm, Angle: %.3f deg, Height: %.3f mm\n",
                         ti_msg.id, pose.x, pose.y, pose.z, // <-- work_point (m)
                         ti_msg.circle_center[0], ti_msg.circle_center[1], ti_msg.circle_center[2],
                         ti_msg.spray_center[0], ti_msg.spray_center[1], ti_msg.spray_center[2],
                         ti_msg.cylinder_axis_direction[0], ti_msg.cylinder_axis_direction[1],
                         ti_msg.cylinder_axis_direction[2], ti_msg.radius, ti_msg.angle, ti_msg.height);

                // 添加到消息列表
                tpl_msg.targets.push_back(pose);

                // 发布喷涂目标信息
                spray_target_pose_pub_.publish(ti_msg);
            }
            // 发布曲臂workPoint消息src/pcl_process/src/ibeam_fitting/log.txt
            target_pose_pub_.publish(tpl_msg);

            ROS_ERROR("ONCE cylinder fitting completed.");

            // 发布工艺参数消息
            const auto               &out = fitter_->getOutputParams();
            pcl_process::ProcessParam process_param_msg;
            process_param_msg.distance    = out.slice_distance_;
            process_param_msg.step_width  = out.step_width_;
            process_param_msg.step_height = out.step_height_;
            process_param_msg.height      = out.height_;
            process_param_msg.lap         = out.r_initial_;
            process_param_msg.arm_reach   = out.robot_R_arm_;
            process_param_msg.mode        = out.work_mode_;
            process_param_msg.k           = out.k_;
            process_param_msg.w0          = out.w0_; // 单位: deg
            process_param_pub_.publish(process_param_msg);
            ROS_ERROR(
                "Published ProcessParam: distance=%.2f, step_width=%.2f, step_height=%.2f, height=%.2f, lap=%.2f, "
                "arm_reach=%.2f, mode=%d, k=%d, w0=%.2f",
                process_param_msg.distance, process_param_msg.step_width, process_param_msg.step_height,
                process_param_msg.height, process_param_msg.lap, static_cast<double>(process_param_msg.arm_reach),
                static_cast<int>(process_param_msg.mode), static_cast<int>(process_param_msg.k), process_param_msg.w0);

            // 发布面积参数
            pcl_process::SprayInfo spray_info_msg;
            spray_info_msg.num  = published_id_counter; // 有效切片数
            spray_info_msg.area = out.single_slice_area_m2_ * published_id_counter;
            spray_area_pub_.publish(spray_info_msg);
            ROS_INFO("Published SprayInfo: valid_slice_count=%d (total=%d), valid_total_area=%.4f m^2",
                     spray_info_msg.num, out.slice_count_, spray_info_msg.area);

        } else {
            ROS_WARN("No valid cylinder model found; skipping target pose publishing.");
        }
    } catch (const std::exception &e) {
        ROS_ERROR("Cylinder fitting failed: %s", e.what());
    }
}

/// @brief 处理参数回调函数
/// @param msg
// float64 distance       # 轨迹点与圆弧表面的距离
// float64 step_width     # 工作区域单次喷幅(mm)
// float64 step_height    # 工作区域单次高度(mm)
// int16   height		  # 工作区总高度，与单次高度成整数倍关系
// float64 lap            # 两次喷幅搭接比例（0~1之间）
// int16   arm_reach      # 机械臂臂展(mm)
// int16   mode           # 1手动 2自动
// int16   width          # 手动模式下直接透传即可；自动模式需要动态计算后传值
void CylinderFittingROS::processParamsCallback(const pcl_process::ProcessParam::ConstPtr &msg) {
    // ROS_WARN("=== processParamsCallback TRIGGERED ===");
    // ROS_INFO("Received ProcessParam: distance=%.2f, step_width=%.2f, height=%.2f, lap=%.2f, arm_reach=%d, mode=%d,
    // k=%d, w0=%d",
    //          msg.distance, msg.step_width, msg.height, msg.lap, msg.arm_reach, msg.mode, msg.k, msg.w0);

    double distance    = msg->distance;
    double step_width  = msg->step_width;
    double step_height = msg->step_height;
    double height      = msg->height;
    double lap         = msg->lap;

    double arm_reach = msg->arm_reach;
    int    mode      = msg->mode;
    int    k         = msg->k;
    int    w0        = msg->w0;

    auto nanSafeNeq = [](double a, double b) { return std::isnan(a) ? !std::isnan(b) : (a != b); };
    bool changed    = (distance != last_distance_) || (step_width != last_step_width_) ||
                   nanSafeNeq(height, last_height_) || (lap != last_lap_) || (arm_reach != last_arm_reach_) ||
                   (mode != last_mode_) || (k != last_k_);
    if (changed) {
        ROS_WARN("Cylinder Process parameters changed: distance=%.2f (was %.2f), step_width=%.2f (was %.2f), step_height=%.2f "
                 "(was %.2f),height=%.2f (was %.2f),lap=%.2f (was %.2f), arm_reach=%.2f (was %.2f), mode=%d (was %d), "
                 "k=%d (was %d)",
                 distance, last_distance_, step_width, last_step_width_, step_height, last_step_height_, height,
                 last_height_, lap, last_lap_, arm_reach, last_arm_reach_, mode, last_mode_, k, last_k_);

        last_distance_    = distance;
        last_step_width_  = step_width;
        last_step_height_ = step_height;
        last_height_      = height;
        last_lap_         = lap;
        last_arm_reach_   = arm_reach;
        last_mode_        = mode;
        last_k_           = k;
        last_w0_          = w0;

        setOption(distance, step_width, step_height, height, lap, arm_reach, mode, k);
    } else {
        ROS_DEBUG("Process parameters unchanged, skipping setOption call.");
    }
}

void CylinderFittingROS::workingObjectCallback(const std_msgs::String::ConstPtr &msg) {
    std::string new_object = msg->data;
    if (new_object != current_work_object_) {
        ROS_WARN("Received new working object: %s (previous: %s)", new_object.c_str(), current_work_object_.c_str());
        current_work_object_ = new_object;

        // A / D 切换时自动加载对应参数预设
        if (new_object == "A" || new_object == "D") {
            ROS_WARN("=== Switching to preset_%s ===", new_object.c_str());
            switchPreset(new_object);
        }

        // 工作对象切换时，根据 achieve_work_pose_ 联动更新 processing_ 状态
        bool cmd_active     = achieve_work_pose_.load(std::memory_order_acquire);
        bool should_process = cmd_active && (new_object == "A" || new_object == "D");
        bool was_processing = processing_.load(std::memory_order_acquire);
        if (should_process && !was_processing) {
            ROS_WARN("=== Work object switched to %s while active: START cylinder fitting ===", new_object.c_str());
            processing_.store(true, std::memory_order_release);
        } else if (!should_process && was_processing) {
            ROS_WARN("=== Work object switched away from A/D while active: STOP cylinder fitting ===");
            processing_.store(false, std::memory_order_release);
        }
    } else {
        ROS_DEBUG("Received working object: %s (unchanged)", new_object.c_str());
    }
}

void CylinderFittingROS::mainCtlCallback(const std_msgs::Bool::ConstPtr &msg) {
    bool prev_cmd = achieve_work_pose_.load(std::memory_order_acquire);
    bool next_cmd = msg->data;

    // 命令未变化时不打印日志，避免刷屏
    if (prev_cmd == next_cmd) {
        return;
    }
    achieve_work_pose_.store(next_cmd, std::memory_order_release);

    bool should_process = next_cmd && (getCurrentWorkObject() == "A" || getCurrentWorkObject() == "D");
    bool was_processing = processing_.load(std::memory_order_acquire);
    if (should_process && !was_processing) {
        ROS_WARN("=== Main control command: START cylinder fitting ===");
        processing_.store(true, std::memory_order_release);
    } else if (!should_process && was_processing) {
        ROS_WARN("=== Main control command: STOP cylinder fitting ===");
        processing_.store(false, std::memory_order_release);
    }
}

/// @brief
/// @param distance    # 轨迹点与圆弧表面的距离
/// @param step_width  # 工作区域单次喷幅(mm)
/// @param step_height # 工作区域单次高度(mm)
/// @param height # 工作区域单次高度(mm)
/// @param lap         # 两次喷幅搭接比例（0~1之间）
/// @param arm_reach   # 机械臂臂展(mm)
/// @param mode        # 1手动 2自动
/// @param k # 手动模式下直接透传即可；自动模式需要动态计算后传值
void CylinderFittingROS::setOption(const float distance, const float step_width, const float step_height,
                                   const float height, const float lap, const double arm_reach, const int mode,
                                   const int k) {

    this->fitter_->sliceOptions().surface_       = false; // 弧形面网格
    this->fitter_->sliceOptions().border_        = false; // 切片边框线
    this->fitter_->sliceOptions().height_rings_  = false; // 高度层环线
    this->fitter_->sliceOptions().print_centers_ = true;  // 切片中心点

    // 更新工艺参数缓存
    last_distance_    = distance;
    last_step_width_  = step_width;
    last_step_height_ = step_height;
    last_height_      = height;
    last_lap_         = lap;
    last_arm_reach_   = arm_reach;
    last_mode_        = mode;
    last_k_           = k;

    // 使用 YAML/dynamic_reconfigure 加载的模型参数 + 工艺参数
    applyFittingParams();

    ROS_WARN("Cylinder fitting parameters have been set (virtual_base_to_surface=%.2f mm).", virtual_base_to_surface_);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "cylinder_fitting_ros");
    ros::NodeHandle nh("~"); // 使用私有命名空间，匹配 launch 文件中 <rosparam> 加载的参数路径

    ROS_INFO("CylinderFittingROS node started, waiting for filtered point clouds and main control commands...");
    CylinderFittingROS node(nh);
    node.setOption(750, 300, 100, 600, 0.5, 1500, 2, 0);

    ros::spin();
    return 0;
}