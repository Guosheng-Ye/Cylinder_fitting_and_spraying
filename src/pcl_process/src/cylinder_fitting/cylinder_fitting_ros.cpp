

#include "cylinder_fitting_ros.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <atomic>
#include <set>

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
            ROS_ERROR("Publishing target pose list...");
            std::vector<CylinderFitter::SliceData> slices = fitter_->getSliceResults();
            pcl_process::TargetPoseList            tpl_msg;
            pcl_process::TargetInfo                ti_msg;

            // Z轴工作高度约束
            const double MIN_WORK_HEIGHT_MM = 2000.0; // Z 轴统一抬高的高度 (mm)
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
                ROS_INFO(" Target ID: %d, Work Position in Base (m): [%.3f, %.3f, %.3f]", pose.id, pose.x, pose.y,
                         pose.z);

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

                // 圆柱轴向
                Eigen::Vector3f axis              = fitter_->axis().normalized();
                ti_msg.cylinder_axis_direction[0] = axis.x();
                ti_msg.cylinder_axis_direction[1] = axis.y();
                ti_msg.cylinder_axis_direction[2] = axis.z();
                // 半径
                ti_msg.radius = fitter_->radius() * 1000.0f; // m -> mm
                // 其他参数
                ti_msg.angle  = std::round((slice.angle_deg) * 1000.0) / 1000.0;
                ti_msg.height = fitter_->fittingParams().height_;

                ROS_WARN("Cylinder ID: %d,circle:[%.3f,%.3f,%.3f],=> Zspray_center:[%.3f,%.3f,%.3f], "
                         " => work_point:[%.3f,%.3f,%.3f], "
                         "cylinder_axis:[%.3f,%.3f,%.3f], Radius: %.3f mm, Angle: %.3f deg, Height: %.3f mm\n",
                         ti_msg.id, ti_msg.circle_center[0], ti_msg.circle_center[1], ti_msg.circle_center[2],
                         ti_msg.spray_center[0], ti_msg.spray_center[1], ti_msg.spray_center[2], pose.x, pose.y,
                         pose.z, // <-- work_point (m)
                         ti_msg.cylinder_axis_direction[0], ti_msg.cylinder_axis_direction[1],
                         ti_msg.cylinder_axis_direction[2], ti_msg.radius, ti_msg.angle, ti_msg.height);

                // 添加到消息列表
                tpl_msg.targets.push_back(pose);

                // 发布喷涂目标信息
                spray_target_pose_pub_.publish(ti_msg);
            }
            // 发布曲臂workPoint消息
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
                "arm_reach=%d, mode=%d, k=%d, w0=%.2f",
                process_param_msg.distance, process_param_msg.step_width, process_param_msg.step_height,
                process_param_msg.height, process_param_msg.lap, process_param_msg.arm_reach, process_param_msg.mode,
                process_param_msg.k, process_param_msg.w0);

            // 发布面积参数
            pcl_process::SprayInfo spray_info_msg;
            spray_info_msg.num  = out.slice_count_;
            spray_info_msg.area = out.total_slice_area_m2_;
            spray_area_pub_.publish(spray_info_msg);
            ROS_INFO("Published SprayInfo: slice_count=%d, total_area=%.4f m^2", spray_info_msg.num,
                     spray_info_msg.area);

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
void CylinderFittingROS::processParamsCallback(const pcl_process::ProcessParam &msg) {
    // ROS_WARN("=== processParamsCallback TRIGGERED ===");
    // ROS_INFO("Received ProcessParam: distance=%.2f, step_width=%.2f, height=%.2f, lap=%.2f, arm_reach=%d, mode=%d,
    // k=%d, w0=%d",
    //          msg.distance, msg.step_width, msg.height, msg.lap, msg.arm_reach, msg.mode, msg.k, msg.w0);

    double distance    = msg.distance;
    double step_width  = msg.step_width;
    double step_height = msg.step_height;
    double height      = msg.height;
    double lap         = msg.lap;

    int arm_reach = msg.arm_reach;
    int mode      = msg.mode;
    int k         = msg.k;
    int w0        = msg.w0;

    bool changed = (distance != last_distance_) || (step_width != last_step_width_) || (height != last_height_) ||
                   (lap != last_lap_) || (arm_reach != last_arm_reach_) || (mode != last_mode_) || (k != last_k_);
    if (changed) {
        ROS_WARN("Process parameters changed: distance=%.2f (was %.2f), step_width=%.2f (was %.2f), step_height=%.2f "
                 "(was %.2f),lap=%.2f (was %.2f), arm_reach=%d (was %d), mode=%d (was %d), k=%d (was %d)",
                 distance, last_distance_, step_width, last_step_width_, height, last_height_, lap, last_lap_,
                 arm_reach, last_arm_reach_, mode, last_mode_, k, last_k_);

        last_distance_   = distance;
        last_step_width_ = step_width;
        last_height_     = height;
        last_lap_        = lap;
        last_arm_reach_  = arm_reach;
        last_mode_       = mode;
        last_k_          = k;
        last_w0_         = w0;

        setOption(distance, step_width, step_height, height, lap, arm_reach, mode, k);
    } else {
        ROS_DEBUG("Process parameters unchanged, skipping setOption call.");
    }
}

void CylinderFittingROS::mainCtlCallback(const std_msgs::Bool &msg) {
    bool prev = processing_.load(std::memory_order_acquire);
    bool next = msg.data;
    if (prev == next) {
        return;
    }
    if (next) {
        ROS_INFO("Main control command received: Start cylinder fitting.");
        processing_.store(true, std::memory_order_release);
    } else {
        ROS_INFO("Main control command received: Stop cylinder fitting.");
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
                                   const float height, const float lap, const int arm_reach, const int mode,
                                   const int k) {

    this->fitter_->sliceOptions().surface_       = false; // 弧形面网格
    this->fitter_->sliceOptions().border_        = false; // 切片边框线
    this->fitter_->sliceOptions().height_rings_  = false; // 高度层环线
    this->fitter_->sliceOptions().print_centers_ = true;  // 切片中心点

    this->fitter_->setFittingParams(CylinderFitter::CylinderFitterParams{
        .min_inliers              = 300,                        // 最小内点数
        .max_inliers              = 600,                        // 最大内点数
        .min_inlier_ratio         = 0.02,                       // 最小内点比例
        .max_inlier_ratio         = 0.50,                       // 最大内点比例
        .min_height_m             = 1.00,                       // 最小高度（米）
        .min_radius_m             = 0.30,                       // 最小半径（米）
        .max_radius_m             = 0.40,                       // 最大半径(米)
        .min_radial_rel_std       = 0.00,                       // 最小径向相对标准差
        .max_radial_rel_std       = 0.15,                       // 最大径向相对标准差
        .slice_distance_          = distance,                   // 喷头工作间距（毫米）
        .height_                  = height,                     // 工作区域高度(mm)
        .step_width_              = step_width,                 // 工作区域单次喷幅(mm)
        .step_height_             = step_height,                // 工作区域单次高度(mm)
        .r_initial_               = lap,                        // 初始搭接比例0.5
        .robot_R_arm_             = arm_reach,                  // 机械臂最大臂长1500mm
        .virtual_base_to_surface_ = virtual_base_to_surface_,   // 虚拟原点到圆柱表面的距离(mm)
        .work_mode_               = mode,                       // 工作模式 1 手动 2 自动
        .k_                       = k,                          // 宽度参数
    });
    ROS_WARN("Cylinder fitting parameters have been set (virtual_base_to_surface=%.2f mm).", virtual_base_to_surface_);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "cylinder_fitting_ros");
    ros::NodeHandle nh;

    ROS_INFO("CylinderFittingROS node started, waiting for filtered point clouds and main control commands...");
    CylinderFittingROS node(nh);
    node.setOption(750, 300, 100, 600, 0.5, 1500, 2, 0);

    ros::spin();
    return 0;
}