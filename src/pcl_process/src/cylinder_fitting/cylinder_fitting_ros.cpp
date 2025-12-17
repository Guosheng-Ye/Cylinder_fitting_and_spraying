

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
                Eigen::Vector3d A_base_mm = tf_lidar2_base_->to_base_from_lidar(pos_spray_center);
                double target_z_base_mm_raised = A_base_mm.z() + MIN_WORK_HEIGHT_MM;

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
                    ROS_WARN("Slice ID %d (layer %d) Z (%.2f mm) - entire layer exceeds max working Z (%.2f mm). Skipping.\n",
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
                ti_msg.height = fitter_->fittingParams().slice_height_;

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
        } else {
            ROS_WARN("No valid cylinder model found; skipping target pose publishing.");
        }
    } catch (const std::exception &e) {
        ROS_ERROR("Cylinder fitting failed: %s", e.what());
    }
}

/// @brief 处理参数回调函数
/// @param msg
void CylinderFittingROS::processParamsCallback(const pcl_process::ProcessParam &msg) {
    double distance = msg.distance;
    double width    = msg.width;
    double height   = msg.height;

    bool changed = (distance != last_distance_) || (width != last_width_) || (height != last_height_);
    if (changed) {
        ROS_WARN("Process parameters changed: distance=%.2f (was %.2f), width=%.2f (was %.2f), height=%.2f (was %.2f)",
                 distance, last_distance_, width, last_width_, height, last_height_);

        last_distance_ = distance;
        last_width_    = width;
        last_height_   = height;
        setOption(distance, width, height);
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

void CylinderFittingROS::setOption(const float distance, const float width, const float height) {

    this->fitter_->sliceOptions().surface_       = false; // 弧形面网格
    this->fitter_->sliceOptions().border_        = false; // 切片边框线
    this->fitter_->sliceOptions().height_rings_  = false; // 高度层环线
    this->fitter_->sliceOptions().print_centers_ = true;  // 切片中心点

    this->fitter_->setFittingParams(CylinderFitter::CylinderFitterParams{
        .min_inliers        = 300,      // 最小内点数
        .max_inliers        = 600,      // 最大内点数
        .min_inlier_ratio   = 0.02,     // 最小内点比例
        .max_inlier_ratio   = 0.50,     // 最大内点比例
        .min_height_m       = 1.00,     // 最小高度（米）
        .min_radius_m       = 0.30,     // 最小半径（米）
        .max_radius_m       = 0.40,     // 最大半径(米)
        .min_radial_rel_std = 0.00,     // 最小径向相对标准差
        .max_radial_rel_std = 0.15,     // 最大径向相对标准差
        .slice_height_      = height,   // 切片高度（毫米）
        .slice_width_       = width,    // 切片宽度（毫米）
        .slice_distance_    = distance, // 喷头工作间距（毫米）
        .r_initial_         = 0.50,     // 初始搭接比例0.5
        .robot_R_arm_       = 1500.0,   // 机械臂最大臂长1500mm
        .work_width_        = width,    // 喷幅（毫米）
    });
    ROS_WARN("Cylinder fitting parameters have been set.");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "cylinder_fitting_ros");
    ros::NodeHandle nh;

    ROS_INFO("CylinderFittingROS node started, waiting for filtered point clouds and main control commands...");
    CylinderFittingROS node(nh);
    node.setOption(1150, 300, 600);

    ros::spin();
    return 0;
}