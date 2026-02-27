#include "ibeam_fitting_ros.hpp"

void IBeamFittingROS::loadParams() {
    // 加载检测参数（从 ROS 参数服务器，含 YAML 配置文件和 launch 文件设置）
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
    nh_.param<int>("max_wall_iterations", params.max_wall_iterations, 3);

    // 聚类参数
    nh_.param<double>("cluster_tolerance", params.cluster_tolerance, 0.18);
    nh_.param<int>("min_cluster_size", params.min_cluster_size, 10);

    // 聚类合并参数(第一轮 - 严格)
    nh_.param<double>("merge_max_gap_strict", params.merge_max_gap_strict, 0.15);
    nh_.param<double>("merge_max_height_diff_strict", params.merge_max_height_diff_strict, 0.10);
    nh_.param<double>("merge_min_axis_similarity_strict", params.merge_min_axis_similarity_strict, 0.98);

    // 聚类合并参数(第二轮 - 宽松)
    nh_.param<double>("merge_max_gap_loose", params.merge_max_gap_loose, 0.5);
    nh_.param<double>("merge_max_height_diff_loose", params.merge_max_height_diff_loose, 0.15);
    nh_.param<double>("merge_min_axis_similarity_loose", params.merge_min_axis_similarity_loose, 0.96);

    // 工字钢判断阈值
    nh_.param<double>("min_linearity", params.min_linearity, 0.9);
    nh_.param<double>("min_length", params.min_length, 1.5);
    nh_.param<double>("min_aspect_ratio", params.min_aspect_ratio, 2.5);
    nh_.param<double>("max_axis_z", params.max_axis_z, 0.1);
    nh_.param<double>("min_width", params.min_width, 0.1);
    nh_.param<double>("max_width", params.max_width, 1.2);
    nh_.param<double>("min_thickness", params.min_thickness, 0.03);
    nh_.param<double>("max_thickness", params.max_thickness, 0.8);

    // 精细化参数
    nh_.param<double>("refine_max_v_span", params.refine_max_v_span, 0.50);
    nh_.param<double>("refine_max_w_span", params.refine_max_w_span, 0.60);
    nh_.param<double>("refine_min_v_span", params.refine_min_v_span, 0.30);
    nh_.param<double>("refine_min_w_span", params.refine_min_w_span, 0.05);

    // 端部裁剪参数
    nh_.param<double>("trim_outlier_ratio", params.trim_outlier_ratio, 1.3);
    nh_.param<double>("trim_v_shift_threshold", params.trim_v_shift_threshold, 0.05);
    nh_.param<double>("trim_w_shift_threshold", params.trim_w_shift_threshold, 0.08);
    nh_.param<double>("trim_aspect_diff_ratio", params.trim_aspect_diff_ratio, 0.5);
    nh_.param<double>("trim_density_ratio_high", params.trim_density_ratio_high, 2.5);
    nh_.param<double>("trim_density_ratio_low", params.trim_density_ratio_low, 0.25);
    nh_.param<double>("trim_score_ratio", params.trim_score_ratio, 0.3);

    // 扩展参数
    nh_.param<double>("extend_min_target_length", params.extend_min_target_length, 3.5);

    fitter_->setParams(params);

    ROS_INFO("Detection params loaded (all %d parameters):", 37);
    ROS_INFO("  [Spatial]  Distance: [%.1f, %.1f] m, Min height: %.1f m", params.min_distance, params.max_distance, params.min_height);
    ROS_INFO("  [Preprocess]  Voxel: %.3f, SOR neighbors: %d, SOR std: %.2f", params.voxel_size, params.sor_neighbors, params.sor_std_ratio);
    ROS_INFO("  [Wall]  Threshold: %.3f, Min points: %d, Max iters: %d", params.wall_distance_threshold, params.min_wall_points, params.max_wall_iterations);
    ROS_INFO("  [Cluster]  Tolerance: %.3f, Min size: %d", params.cluster_tolerance, params.min_cluster_size);
    ROS_INFO("  [IBeam]  Linearity: %.2f, Length: %.1f, Aspect: %.1f, AxisZ: %.2f", params.min_linearity, params.min_length, params.min_aspect_ratio, params.max_axis_z);
    ROS_INFO("  [IBeam]  Width: [%.2f, %.2f], Thickness: [%.3f, %.2f]", params.min_width, params.max_width, params.min_thickness, params.max_thickness);
    ROS_INFO("  [Refine]  V-span: [%.2f, %.2f], W-span: [%.2f, %.2f]", params.refine_min_v_span, params.refine_max_v_span, params.refine_min_w_span, params.refine_max_w_span);
    ROS_INFO("  [Trim]  Outlier: %.2f, V-shift: %.3f, W-shift: %.3f, Aspect: %.2f, Density: [%.2f, %.2f], Score: %.2f",
             params.trim_outlier_ratio, params.trim_v_shift_threshold, params.trim_w_shift_threshold,
             params.trim_aspect_diff_ratio, params.trim_density_ratio_low, params.trim_density_ratio_high,
             params.trim_score_ratio);
    ROS_INFO("  [Extend]  Min target length: %.1f m", params.extend_min_target_length);
}

/// @brief dynamic_reconfigure 回调 - 运行时动态更新检测参数
void IBeamFittingROS::dynamicReconfigureCallback(pcl_process::IBeamDetectionConfig &config, uint32_t level) {
    ROS_INFO("=== dynamic_reconfigure: Updating detection parameters (level=%u) ===", level);

    IBeamDetectionParams params;

    // 空间过滤参数
    params.max_distance = config.max_distance;
    params.min_distance = config.min_distance;
    params.min_height   = config.min_height;

    // 预处理参数
    params.voxel_size    = config.voxel_size;
    params.sor_neighbors = config.sor_neighbors;
    params.sor_std_ratio = config.sor_std_ratio;

    // 墙面移除参数
    params.wall_distance_threshold = config.wall_distance_threshold;
    params.min_wall_points         = config.min_wall_points;
    params.max_wall_iterations     = config.max_wall_iterations;

    // 聚类参数
    params.cluster_tolerance = config.cluster_tolerance;
    params.min_cluster_size  = config.min_cluster_size;

    // 聚类合并参数(第一轮 - 严格)
    params.merge_max_gap_strict             = config.merge_max_gap_strict;
    params.merge_max_height_diff_strict     = config.merge_max_height_diff_strict;
    params.merge_min_axis_similarity_strict = config.merge_min_axis_similarity_strict;

    // 聚类合并参数(第二轮 - 宽松)
    params.merge_max_gap_loose             = config.merge_max_gap_loose;
    params.merge_max_height_diff_loose     = config.merge_max_height_diff_loose;
    params.merge_min_axis_similarity_loose = config.merge_min_axis_similarity_loose;

    // 工字钢判断阈值
    params.min_linearity    = config.min_linearity;
    params.min_length       = config.min_length;
    params.min_aspect_ratio = config.min_aspect_ratio;
    params.max_axis_z       = config.max_axis_z;
    params.min_width        = config.min_width;
    params.max_width        = config.max_width;
    params.min_thickness    = config.min_thickness;
    params.max_thickness    = config.max_thickness;

    // 精细化参数
    params.refine_max_v_span = config.refine_max_v_span;
    params.refine_max_w_span = config.refine_max_w_span;
    params.refine_min_v_span = config.refine_min_v_span;
    params.refine_min_w_span = config.refine_min_w_span;

    // 端部裁剪参数
    params.trim_outlier_ratio      = config.trim_outlier_ratio;
    params.trim_v_shift_threshold  = config.trim_v_shift_threshold;
    params.trim_w_shift_threshold  = config.trim_w_shift_threshold;
    params.trim_aspect_diff_ratio  = config.trim_aspect_diff_ratio;
    params.trim_density_ratio_high = config.trim_density_ratio_high;
    params.trim_density_ratio_low  = config.trim_density_ratio_low;
    params.trim_score_ratio        = config.trim_score_ratio;

    // 扩展参数
    params.extend_min_target_length = config.extend_min_target_length;

    fitter_->setParams(params);
    ROS_INFO("Detection parameters updated via dynamic_reconfigure.");
}

/// @brief 工艺参数设置函数
/// @param distance      轨迹点与表面的距离 (mm)
/// @param step_height   工作区域单次喷幅 (mm)（透传）
/// @param step_length   工作区域单次长度 (mm)（透传）
/// @param lap           两次喷幅搭接比例（透传）（0~1）
/// @param arm_reach     机械臂臂展 (mm)
/// @param mode_length   1手动2自动
/// @param length        工作区域长度 (mm)
/// @param mode_theta    1手动2自动（透传）
/// @param theta         手动模式下喷涂翼板时的角度（透传）
void IBeamFittingROS::setOption(const float distance, const float step_height, const float step_length, const float lap,
                                const float arm_reach, const int mode_length, const float length, const int mode_theta,
                                const float theta) {

    this->fitter_->setIBeamParams(IBeamInputs{
        .distance_tcp2surface = distance,    // 轨迹点与表面的距离
        .step_height          = step_height, // 工作区域单次喷幅（mm) 透传
        .step_length          = step_length, // 工作区域单次长度（mm) 透传
        .lap                  = lap,         // 两次喷幅搭接比例      透传
        .arm_reach            = arm_reach,   // arm_reach 机械臂最大工作半径（mm）
        .mode_length          = mode_length, // mode_length 工作区域长度模式（1-手动，2-自动
        .length               = length,      // length 工作区域长度（mm，mode_length=1时有效
        .mode_theta           = mode_theta,  // mode_theta 喷涂角度模式（1-手动，2-自动）
        .theta                = theta,       // theta 手动模式下喷涂翼板时的角度（透传）
    });
    ROS_WARN(
        "IBeam fitting parameters have been set distance=%.2f mm, step_height=%.2f mm, step_length=%.2f mm, lap=%.2f, "
        "arm_reach=%.2f mm, "
        "mode_length=%d, length=%.2f mm, mode_theta=%d, theta=%.2f degrees",
        distance, step_height, step_length, lap, arm_reach, mode_length, length, mode_theta, theta);
}

/// @brief 主控命令订阅回调
/// @param msg
void IBeamFittingROS::mainCtlCallback(const std_msgs::Bool::ConstPtr &msg) {
    bool prev = processing_.load(std::memory_order_acquire);
    bool next = msg->data;
    if (prev == next) {
        return;
    }
    if (next) {
        ROS_WARN("=== Main control command: START I-beam fitting ===");
        processing_.store(true, std::memory_order_release);
    } else {
        ROS_WARN("=== Main control command: STOP I-beam fitting ===");
        processing_.store(false, std::memory_order_release);
    }
}

/// @brief 工艺参数订阅回调
/// @param msg IBeamProcessParam消息
// float64     distance     # 轨迹点与表面的距离
// float64     step_height  # 工作区域单次喷幅（透传）
// float64     step_length  # 工作区域单次长度（透传）
// float64     lap          # 两次喷幅搭接比例（透传）
// int16       arm_reach    # 机械臂臂展
// int16       mode_length  # 1手动2自动
// float64     length       # 工作区域长度
// int16       mode_theta    # 1手动2自动（透传）
// float64     theta        # 手动模式下喷涂翼板时的角度（透传）
void IBeamFittingROS::ibeamProcessParamCallback(const pcl_process::IBeamProcessParam::ConstPtr &msg) {

    double distance    = msg->distance;
    double step_height = msg->step_height;
    double step_length = msg->step_length;
    double lap         = msg->lap;
    double arm_reach   = msg->arm_reach;
    int    mode_length = msg->mode_length;
    double length      = msg->length;
    int    mode_theta  = msg->mode_theta;
    double theta       = msg->theta;

    bool changed = (distance != last_distance_) || (step_height != last_step_height_) ||
                   (step_length != last_step_length_) || (lap != last_lap_) || (arm_reach != last_arm_reach_) ||
                   (mode_length != last_mode_length_) || (length != last_length_) || (mode_theta != last_mode_theta_);
    if (changed) {
        ROS_WARN("Process parameters changed: distance=%.2f (was %.2f), step_height=%.2f (was %.2f), step_length=%.2f "
                 "(was %.2f), lap=%.2f (was %.2f), arm_reach=%.2f (was %.2f), mode_length=%d (was %d), length=%.2f "
                 "(was %.2f), mode_theta=%d (was %d), theta=%.2f (was %.2f)",
                 distance, last_distance_, step_height, last_step_height_, step_length, last_step_length_, lap,
                 last_lap_, arm_reach, last_arm_reach_, mode_length, last_mode_length_, length, last_length_,
                 mode_theta, last_mode_theta_, theta, last_theta_);

        last_distance_    = distance;
        last_step_height_ = step_height;
        last_step_length_ = step_length;
        last_lap_         = lap;
        last_arm_reach_   = arm_reach;
        last_mode_length_ = mode_length;
        last_length_      = length;
        last_mode_theta_  = mode_theta;
        last_theta_       = theta;

        setOption(distance, step_height, step_length, lap, arm_reach, mode_length, length, mode_theta, theta);
    } else {
        ROS_DEBUG("Process parameters unchanged, skipping setOption call.");
    }
}

void IBeamFittingROS::generateTargetPoses(const IBeamOutputs &outputs) {
    std::vector<IBeamSliceData> slices = fitter_->getSliceResults();
    if (slices.empty()) {
        return;
    }

    pcl_process::TargetPoseList tpl_msg;
    const IBeamInputs          &inputs = fitter_->getIBeamParams();
    double                      d_base = inputs.d_base; // 底座中心到腹板表面的水平距离 (mm)

    // 获取工字钢的几何中心点（切片中心）和法向量（指向机器人方向）
    Eigen::Vector3d ibeam_center = outputs.surface_point;
    ibeam_center[0] *= 1000.0; // 转换为mm
    ibeam_center[1] *= 1000.0;
    ibeam_center[2] *= 1000.0;
    Eigen::Vector3d ibeam_base = tf_lidar2_base_->to_base_from_lidar(ibeam_center);
    ROS_INFO("[IBeamFittingROS] IBeam center: (%.2f, %.2f, %.2f) m in base frame", ibeam_base.x() / 1000.0,
             ibeam_base.y() / 1000.0, ibeam_base.z() / 1000.0);

    ROS_INFO("[IBeamFittingROS] Generating target poses for %zu slices (d_base=%.2f mm)", slices.size(), d_base);

    for (const auto &slice : slices) {
        // 1. 获取切片中心（视为表面点） - 雷达坐标系
        Eigen::Vector3d center_lidar = slice.center_mm;

        // 2. 计算基座作业点（目标点） - 雷达坐标系
        // 目标点 = 表面点 + 法向量 * d_base
        // slice.normal 指向机器人（即远离工字钢表面的方向），符合 d_base 的延伸方向
        // Eigen::Vector3d target_lidar = center_lidar + slice.normal.normalized() * d_base;
        Eigen::Vector3d target_lidar = slice.work_point_mm;

        // 3. 坐标转换：雷达坐标系 -> 基座坐标系
        Eigen::Vector3d target_base = tf_lidar2_base_->to_base_from_lidar(target_lidar);
        Eigen::Vector3d center_base = tf_lidar2_base_->to_base_from_lidar(center_lidar);

        // 4. 填充 TargetPose (单位: m)
        pcl_process::TargetPose pose;
        pose.id = slice.id;
        pose.x  = target_base.x() / 1000.0;
        pose.y  = target_base.y() / 1000.0;
        pose.z  = target_base.z() / 1000.0;

        tpl_msg.targets.push_back(pose);

        // 5. 发布单切片信息 (IBeamTargetInfo)
        pcl_process::IBeamTargetInfo ti_msg;
        // ti_msg.id = slice.id; // IBeamTargetInfo has no id field
        // 填充表面点 (基座坐标系)
        ti_msg.surface_point = {center_base.x(), center_base.y(), center_base.z()};
        // 填充法向量 (基座坐标系)
        Eigen::Vector3d normal_base = (target_base - center_base).normalized();
        ti_msg.plane_coeffs         = {normal_base.x(), normal_base.y(), normal_base.z()};

        // 填充主轴方向 (基座坐标系)
        // 向量变换: V_base = R * V_lidar = to_base(P+V) - to_base(P)
        Eigen::Vector3d axis_lidar            = slice.main_axis;
        Eigen::Vector3d center_plus_axis_base = tf_lidar2_base_->to_base_from_lidar(center_lidar + axis_lidar);
        Eigen::Vector3d axis_base             = (center_plus_axis_base - center_base).normalized();
        ti_msg.plane_rx                       = {axis_base.x(), axis_base.y(), axis_base.z()};

        // 填充切片尺寸 (mm)
        ti_msg.length           = slice.slice_width_mm;
        ti_msg.height           = slice.slice_height_mm;
        ti_msg.wing_plate_width = outputs.wing_plate_width;

        // 发布喷涂目标点信息 /spray_target_pose
        spray_target_pose_pub_.publish(ti_msg);
        ROS_WARN("[IBeamFittingROS] Published target info for slice %d: surface_point=(%.2f, %.2f, %.2f) "
                 "m,work_point=(%.2f, %.2f, %.2f) m,"
                 "normal=(%.2f, %.2f, %.2f), "
                 "main_axis=(%.2f, %.2f, %.2f), slice_width=%.2f mm, slice_height=%.2f mm",
                 slice.id, center_base.x() / 1000.0, center_base.y() / 1000.0, center_base.z() / 1000.0, pose.x, pose.y,
                 pose.z, normal_base.x(), normal_base.y(), normal_base.z(), axis_base.x(), axis_base.y(), axis_base.z(),
                 slice.slice_width_mm, slice.slice_height_mm);
    }

    // 发布 /target_pose_list
    target_pose_pub_.publish(tpl_msg);
    ROS_WARN("[IBeamFittingROS] Published TargetPoseList with %zu targets ", tpl_msg.targets.size());

    // 发布工艺参数 /ibeam_process_param
    pcl_process::IBeamProcessParam param_msg;
    param_msg.distance    = inputs.distance_tcp2surface;
    param_msg.step_height = inputs.step_height;
    param_msg.step_length = inputs.step_length;
    param_msg.lap         = inputs.lap;
    param_msg.arm_reach   = inputs.arm_reach;
    param_msg.mode_length = inputs.mode_length;
    param_msg.length      = outputs.beam_length_of_working; // 使用计算得到的工作长度
    param_msg.mode_theta  = inputs.mode_theta;
    param_msg.theta       = inputs.theta;
    process_param_pub_.publish(param_msg);
    ROS_WARN("IBeamFittingROS: Published process parameters distance=%.2f mm, step_height=%.2f mm, step_length=%.2f "
             "mm, lap=%.2f, arm_reach=%.2f mm, "
             "mode_length=%d, length=%.2f mm, mode_theta=%d, theta=%.2f degrees",
             param_msg.distance, inputs.step_height, inputs.step_length, inputs.lap, inputs.arm_reach,
             inputs.mode_length, outputs.beam_length_of_working, inputs.mode_theta, inputs.theta);
}

void IBeamFittingROS::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    try {
        bool processing = processing_.load(std::memory_order_acquire);
        if (!processing) {
            ROS_DEBUG_THROTTLE(5.0, "I-beam fitting is currently paused. Ignoring incoming point cloud.");
            return;
        }
        if (cloud->empty()) {
            ROS_WARN_THROTTLE(5.0, "Received empty point cloud");
            return;
        }

        fitter_->setInputCloud(cloud);

        if (fitter_->detect()) {
            std::vector<IBeamResult> results = fitter_->getResults();
            ROS_WARN("[IBeamFittingROS::cloudCallback] Detected %zu I-beams", results.size());

            // 使用已设置的工艺参数
            const IBeamInputs &inputs = fitter_->ibeam_inputs_;

            IBeamOutputs outputs;
            if (!results.empty()) {
                fitter_->updateOutputs(inputs, results[0], outputs);
            }
            // 作业切片 - 使用设置的切片参数 //宽使用手动设置宽，高使用工字钢高度
            fitter_->sliceIBeam(results[0], outputs, 0, outputs.beam_length_of_working);

            // 生成并发布基座作业点
            generateTargetPoses(outputs);

            // 可视化
            fitter_->visualizeResults();
        } else {
            ROS_WARN_THROTTLE(5.0, "No I-beams detected in current frame");
        }
        ROS_WARN("[IBeamFittingROS::cloudCallback] Processing completed");
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ibeam_fitting_node");
    ros::NodeHandle nh("~");
    IBeamFittingROS node(nh);
    ros::spin();
    return 0;
}
