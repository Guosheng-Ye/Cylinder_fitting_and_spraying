

#include "cylinder_fitting_ros.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <atomic>
/*
void CylinderFittingROS::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    // static ros::Time last_log_time = ros::Time(0);
    // ros::Time        now           = ros::Time::now();
    // if ((now - last_log_time).toSec() > 10.0) {
    //     ROS_INFO("Received filtered cloud with %zu points", cloud->size());
    //     last_log_time = now;
    // }

    try {
        static std::atomic<bool> logged_disabled{false};
        bool                     processing = processing_.load(std::memory_order_acquire);

        if (!processing) {
            if (!logged_disabled.load(std::memory_order_relaxed)) {
                ROS_INFO("Cylinder fitting is currently disabled by main control command.");
                logged_disabled.store(true, std::memory_order_relaxed);
            }
            return; // 直接返回，避免重复进入后续逻辑
        } else {
            // 状态恢复为开启时，重置标志
            if (logged_disabled.load(std::memory_order_relaxed)) {
                logged_disabled.store(false, std::memory_order_relaxed);
            }
        }

        ROS_ERROR("Cylinder fitting is currently enabled by main control command.");

        /// acheive_work_pose :true 开启检测，false：不检测
        // std_msgs::Bool data;

        ROS_ERROR("Cylinder fitting is currently enabled by main control command.");
        if (fitter_->run_with_rslidar(cloud) != -1) {
            ROS_ERROR("Publishing target pose list...");
            std::vector<CylinderFitter::SliceData> slices = fitter_->getSliceResults();
            pcl_process::TargetPoseList            tpl_msg;
            pcl_process::TargetInfo                ti_msg;

            //------- 高度限制------
            const double MIN_WORK_HEIGHT_MM = 2000.0; //  工作区域的高度约束 (单位: mm)
            const double SPRAY_OFFSET_MM    = 1150.0; // 喷涂工作点偏移距离
            // 获取圆柱的实际高度（米）并转换为毫米
            const double CYLINDER_HEIGHT_MM = fitter_->height() * 1000.0;

            Eigen::Vector3d bottom_center_base_mm(0, 0, 0);
            Eigen::Vector3d top_center_base_mm(0, 0, 0);
            // 转换圆柱底部中心 (m -> mm) 到 Base 坐标系
            Eigen::Vector3d pos_bottom_center(fitter_->bottomCenter().x() * 1000.0,
                                              fitter_->bottomCenter().y() * 1000.0,
                                              fitter_->bottomCenter().z() * 1000.0);
            bottom_center_base_mm = tf_lidar2_base_->to_base_from_lidar(pos_bottom_center);

            // 预估圆柱顶部中心 (m -> mm) 到 Base 坐标系
            Eigen::Vector3d pos_top_center(fitter_->topCenter().x() * 1000.0, fitter_->topCenter().y() * 1000.0,
                                           fitter_->topCenter().z() * 1000.0);
            top_center_base_mm = tf_lidar2_base_->to_base_from_lidar(pos_top_center);

            ROS_INFO("Cylinder Bottom Center in Base (mm): [%.2f, %.2f, %.2f]", bottom_center_base_mm.x(),
                     bottom_center_base_mm.y(), bottom_center_base_mm.z());
            ROS_INFO("Cylinder Top Center in Base (mm): [%.2f, %.2f, %.2f]", top_center_base_mm.x(),
                     top_center_base_mm.y(), top_center_base_mm.z());

            // 原始圆柱在 Base 坐标系上的 Z 轴高度范围（假设圆柱轴向与 Base Z 轴近似平行）
            double cylinder_min_z_base_mm = std::min(bottom_center_base_mm.z(), top_center_base_mm.z());
            double cylinder_max_z_base_mm = std::max(bottom_center_base_mm.z(), top_center_base_mm.z());

            // 抬高后的圆柱在 Base 坐标系上的 Z 轴高度范围
            double working_min_z_base_mm = cylinder_min_z_base_mm + MIN_WORK_HEIGHT_MM;
            // 抬高后的最大 Z 轴（保持原始高度）
            double working_max_z_base_mm = cylinder_max_z_base_mm;

            // end of 高度限制-------

            tpl_msg.targets.reserve(slices.size());
            for (const auto &slice : slices) { //切片中心
                // 转换 喷涂中心 (mm) to base (mm)
                Eigen::Vector3d pos_spray_center(slice.center_mm.x(), slice.center_mm.y(), slice.center_mm.z());
                Eigen::Vector3d pos_spray_center2base = tf_lidar2_base_->to_base_from_lidar(pos_spray_center);

                // 转换 圆柱轴线中心 (mm) to base (mm)
                Eigen::Vector3d pos_circle_center(slice.circle_center_mm.x(), slice.circle_center_mm.y(),
                                                  slice.circle_center_mm.z());
                Eigen::Vector3d pos_circle_center2base = tf_lidar2_base_->to_base_from_lidar(pos_circle_center);

                // Z 轴约束 ---
                // 将切片中心的 Z 坐标加上抬高的 MIN_WORK_HEIGHT_MM
                double target_z_base_mm = pos_spray_center2base.z() + MIN_WORK_HEIGHT_MM;
                double circle_z_base_mm = pos_circle_center2base.z() + MIN_WORK_HEIGHT_MM;

                // 检查抬高后的 Z 坐标是否超出圆柱的原始顶部 Z 坐标
                // 超过顶部部分切片无效。
                if (target_z_base_mm > working_max_z_base_mm) {
                    ROS_WARN("Slice ID %d Z (%.2f mm) exceeds max working Z (%.2f mm) after (%.1f m) offset. Skipping.",
                             slice.id + 1, target_z_base_mm, working_max_z_base_mm, MIN_WORK_HEIGHT_MM / 1000.0);
                    continue; // 跳过此切片
                }
                {
                    // 计算喷涂工作点 nearPoint（mm）
                    // A = 表面点（pos_spray_center2base），
                    // C = 圆柱中心（pos_circle_center2base）
                    // B_nearPoint 在 A 与 C 连线延长线上，远离圆柱中心方向，距离 A 为 offset（通常 1150 mm）

                    Eigen::Vector3d A    = pos_spray_center2base;  // 表面点
                    Eigen::Vector3d C    = pos_circle_center2base; // 圆柱中心
                    Eigen::Vector3d v    = A - C;                  // 指向外侧
                    double          norm = v.norm();
                    Eigen::Vector3d B_nearPoint; // 喷涂工作点
                    if (norm < 1e-6) {
                        ROS_WARN("Slice ID %d: surface point coincides with circle center, fallback offset applied.",
                                 slice.id + 1);
                        B_nearPoint = A + Eigen::Vector3d(SPRAY_OFFSET_MM, 0.0, 0.0); // 简单退化处理
                    } else {
                        Eigen::Vector3d dir = v / norm; // 从圆心指向表面（外侧方向）
                        B_nearPoint         = A + dir * SPRAY_OFFSET_MM;
                    }
                    ROS_WARN(" Target ID: %d, B_nearPoint Position in Base (m): [%.3f, %.3f, %.3f] (B from A offset "
                             "%.0f mm)",
                             pose.id, B_nearPoint.x(), B_nearPoint.y(), B_nearPoint.z(), SPRAY_OFFSET_MM);
                }

                pcl_process::TargetPose pose;
                pose.id   = slice.id + 1;
                ti_msg.id = slice.id + 1;

                pose.x = pos_spray_center2base.x() / 1000.0; // mm -> m
                pose.y = pos_spray_center2base.y() / 1000.0;
                pose.z = target_z_base_mm / 1000.0; // 使用约束后的 Z (m)

                ROS_WARN(" Target ID: %d, Spray Position in Base (m): [%.2f, %.2f, %.2f]", pose.id, pose.x, pose.y,
                         pose.z);

                // 圆柱中心
                ti_msg.circle_center[0] = pos_circle_center2base.x();
                ti_msg.circle_center[1] = pos_circle_center2base.y();
                ti_msg.circle_center[2] = circle_z_base_mm; // 使用约束后的 Z (mm)

                // 复用pose的喷涂中心变量
                ti_msg.spray_center[0] = pose.x * 1000.0; // m -> mm
                ti_msg.spray_center[1] = pose.y * 1000.0;
                ti_msg.spray_center[2] = pose.z * 1000.0;

                // 圆柱轴向
                Eigen::Vector3f axis              = fitter_->axis().normalized();
                ti_msg.cylinder_axis_direction[0] = axis.x();
                ti_msg.cylinder_axis_direction[1] = axis.y();
                ti_msg.cylinder_axis_direction[2] = axis.z();
                // 半径
                ti_msg.radius = fitter_->radius();
                // 计算喷涂中心点与圆柱轴线的夹角
                Eigen::Vector3f spray_dir = (slice.center_mm - slice.circle_center_mm).normalized();
                double          angle_rad = std::acos(axis.dot(spray_dir));
                ti_msg.angle              = slice.angle_deg;
                ti_msg.height             = fitter_->fittingParams().slice_height_;

                // 直线上接近点

                ROS_WARN("Cylinder ID: %d,circle:[%.2f,%.2f,%.2f],spray_center:[%.2f,%.2f,%.2f], "
                         "cylinder_axis:[%.2f,%.2f,%.2f], Radius: %.2f mm, Angle: %.2f deg, Height: %.2f mm",
                         ti_msg.id, ti_msg.circle_center[0], ti_msg.circle_center[1], ti_msg.circle_center[2],
                         ti_msg.spray_center[0], ti_msg.spray_center[1], ti_msg.spray_center[2],
                         ti_msg.cylinder_axis_direction[0], ti_msg.cylinder_axis_direction[1],
                         ti_msg.cylinder_axis_direction[2], ti_msg.radius, ti_msg.angle, ti_msg.height);
                // 添加到消息列表
                tpl_msg.targets.push_back(pose);

                // 发布 喷涂目标信息
                spray_target_pose_pub_.publish(ti_msg);
            }
            // 发布曲臂消息
            target_pose_pub_.publish(tpl_msg);
            // for (const auto &slice : slices) { //切片中心
            //     pcl_process::TargetPose pose;
            //     pose.id   = slice.id + 1;
            //     ti_msg.id = slice.id + 1;

            //     // 转换 喷涂中心 to base
            //     Eigen::Vector3d pos_spray_center(slice.center_mm.x(), slice.center_mm.y(), slice.center_mm.z());
            //     Eigen::Vector3d pos_spray_center2base = tf_lidar2_base_->to_base_from_lidar(pos_spray_center);
            //     pose.x                                = pos_spray_center2base.x() / 1000.0; // mm -> m
            //     pose.y                                = pos_spray_center2base.y() / 1000.0;
            //     pose.z                                = pos_spray_center2base.z() / 1000.0;

            //     ROS_WARN(" Target ID: %d, Spray Position in Base (mm): [%.2f, %.2f, %.2f]", pose.id, pose.x, pose.y,
            //              pose.z);

            //     // 转换 圆柱中心 to base
            //     Eigen::Vector3d pos_circle_center(slice.circle_center_mm.x(), slice.circle_center_mm.y(),
            //                                       slice.circle_center_mm.z());
            //     Eigen::Vector3d pos_circle_center2base = tf_lidar2_base_->to_base_from_lidar(pos_circle_center);
            //     // 圆柱中心
            //     ti_msg.circle_center[0] = pos_circle_center2base.x();
            //     ti_msg.circle_center[1] = pos_circle_center2base.y();
            //     ti_msg.circle_center[2] = pos_circle_center2base.z();
            //     ti_msg.circle_center[2] += 1700.0; // 圆柱中心提升1700mm

            //     // 复用pose的喷涂中心变量
            //     ti_msg.spray_center[0] = pose.x * 1000.0; // m -> mm
            //     ti_msg.spray_center[1] = pose.y * 1000.0;
            //     ti_msg.spray_center[2] = pose.z * 1000.0;
            //     // 圆柱轴向
            //     Eigen::Vector3f axis              = fitter_->axis().normalized();
            //     ti_msg.cylinder_axis_direction[0] = axis.x();
            //     ti_msg.cylinder_axis_direction[1] = axis.y();
            //     ti_msg.cylinder_axis_direction[2] = axis.z();
            //     // 半径
            //     ti_msg.radius = fitter_->radius();
            //     // 计算喷涂中心点与圆柱轴线的夹角
            //     Eigen::Vector3f spray_dir = (slice.center_mm - slice.circle_center_mm).normalized();
            //     double          angle_rad = std::acos(axis.dot(spray_dir));
            //     ti_msg.angle              = slice.angle_deg;
            //     ti_msg.height             = fitter_->fittingParams().slice_height_;

            //     ROS_WARN("Cylinder ID: %d,circle:[%.2f,%.2f,%.2f],spray_center:[%.2f,%.2f,%.2f], "
            //              "cylinder_axis:[%.2f,%.2f,%.2f], Radius: %.2f mm, Angle: %.2f deg, Height: %.2f mm",
            //              ti_msg.id, ti_msg.circle_center[0], ti_msg.circle_center[1], ti_msg.circle_center[2],
            //              ti_msg.spray_center[0], ti_msg.spray_center[1], ti_msg.spray_center[2],
            //              ti_msg.cylinder_axis_direction[0], ti_msg.cylinder_axis_direction[1],
            //              ti_msg.cylinder_axis_direction[2], ti_msg.radius, ti_msg.angle, ti_msg.height);
            //     // 添加到消息列表
            //     tpl_msg.targets.push_back(pose);

            //     // 发布 喷涂目标信息
            //     spray_target_pose_pub_.publish(ti_msg);
            // }
            // 发布曲臂消息
            // target_pose_pub_.publish(tpl_msg);

            ROS_ERROR("ONCE cylinder fitting completed.");
        } else {
            ROS_WARN("No valid cylinder model found; skipping target pose publishing.");
        }

    } catch (const std::exception &e) {
        ROS_ERROR("Cylinder fitting failed: %s", e.what());
    }
}
*/
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

            // --- 统一 Z 轴工作高度约束定义 ---
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
            // double cylinder_min_z_base_mm = std::min(bottom_center_base_mm.z(), top_center_base_mm.z()); //
            // 移除不需要的min
            double cylinder_max_z_base_mm = std::max(bottom_center_base_mm.z(), top_center_base_mm.z());

            double working_max_z_base_mm = cylinder_max_z_base_mm;

            ROS_INFO("Cylinder Original Max Z in Base (mm): %.2f", cylinder_max_z_base_mm);
            ROS_INFO("Working Max Z (Original + Offset) in Base (mm): %.2f", working_max_z_base_mm);
            ROS_INFO("Z Offset Applied (m): %.3f", MIN_WORK_HEIGHT_MM / 1000.0);

            // end of 高度限制-------

            tpl_msg.targets.reserve(slices.size());
            for (const auto &slice : slices) { //切片中心
                // 1. 坐标转换：Lidar (mm) -> Base (mm)
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

                ROS_ERROR("source pos_circle_center: [%.2f, %.2f, %.2f], pos_spray_center: [%.3f,%.3f,%.3f] "
                          ",work_point: [%.3f, %.3f,%.3f]",
                          slice.circle_center_mm.x(), slice.circle_center_mm.y(), slice.circle_center_mm.z(),
                          slice.center_mm.x(), slice.center_mm.y(), slice.center_mm.z(), slice.work_mm.x(),
                          slice.work_mm.y(), slice.work_mm.z());

                // 统一 Z 轴抬高
                double target_z_base_mm_raised = A_base_mm.z() + MIN_WORK_HEIGHT_MM;
                double circle_z_base_mm_raised = C_base_mm.z() + MIN_WORK_HEIGHT_MM;
                double work_z_base_mm_raised   = work_point.z() + MIN_WORK_HEIGHT_MM;

                // 3. Z 轴上限约束检查
                if (target_z_base_mm_raised > working_max_z_base_mm) {
                    ROS_WARN("Slice ID %d Z (%.2f mm) exceeds max working Z (%.2f mm) after offset. Skipping.",
                             slice.id + 1, target_z_base_mm_raised, working_max_z_base_mm);
                    continue;
                }

                // // 显式构造 A_final_mm 和 C_final_mm，确保 Z 坐标是抬高后的值**
                // // 使用 (X_base, Y_base, Z_raised) 构造最终点
                Eigen::Vector3d A_final_mm(A_base_mm.x(), A_base_mm.y(), target_z_base_mm_raised);
                Eigen::Vector3d C_final_mm(C_base_mm.x(), C_base_mm.y(), circle_z_base_mm_raised);

                // // 4. 计算喷涂工作点 B_nearPoint (Base mm)
                // Eigen::Vector3d v    = A_final_mm - C_final_mm; // 指向外侧
                // double          norm = v.norm();
                // Eigen::Vector3d B_nearPoint; // 喷涂工作点

                // if (norm < 1e-6) {
                //     ROS_WARN("Slice ID %d: surface point coincides with circle center, fallback offset applied.",
                //              slice.id + 1);
                //     B_nearPoint = A_final_mm + Eigen::Vector3d(SPRAY_OFFSET_MM, 0.0, 0.0); // 简单退化处理
                // } else {
                //     Eigen::Vector3d dir = v / norm; // 从圆心指向表面（外侧方向）的单位向量
                //     B_nearPoint         = A_final_mm + dir * SPRAY_OFFSET_MM;
                // }

                // 调试输出 B_nearPoint
                // ROS_WARN(
                //     " Target ID: %d, B_nearPoint Position in Base (m): [%.3f, %.3f, %.3f], (B from A offset %.0f
                //     mm)", slice.id + 1, work_point.x() / 1000.0, work_point.y() / 1000.0, work_z_base_mm_raised /
                //     1000.0, SPRAY_OFFSET_MM);

                // 5. 填充 TargetPose (给曲臂的消息，单位：m) - 使用 work_point
                pcl_process::TargetPose pose;
                pose.id = slice.id + 1;

                pose.x = work_point.x() / 1000.0; // mm -> m
                pose.y = work_point.y() / 1000.0;
                pose.z = work_z_base_mm_raised / 1000.0;
                ROS_INFO(" Target ID: %d, Work Position in Base (m): [%.3f, %.3f, %.3f]", pose.id, pose.x, pose.y,
                         pose.z);

                // 填充 TargetInfo (给喷涂目标信息的消息，单位：mm)
                ti_msg.id = slice.id + 1;

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
                ti_msg.angle  = slice.angle_deg;
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

                // 发布 喷涂目标信息
                spray_target_pose_pub_.publish(ti_msg);
            }
            // 发布曲臂消息
            target_pose_pub_.publish(tpl_msg);

            ROS_ERROR("ONCE cylinder fitting completed.");
        } else {
            ROS_WARN("No valid cylinder model found; skipping target pose publishing.");
        }
    } catch (const std::exception &e) {
        ROS_ERROR("Cylinder fitting failed: %s", e.what());
    }
}

void CylinderFittingROS::processParamsCallback(const pcl_process::ProcessParam &msg) {
    double distance = msg.distance;
    double width    = msg.width;
    double height   = msg.height;

    // 是否有任何参数发生变化
    bool changed = (distance != last_distance_) || (width != last_width_) || (height != last_height_);
    if (changed) {
        ROS_WARN("Process parameters changed: distance=%.2f (was %.2f), width=%.2f (was %.2f), height=%.2f (was %.2f)",
                 distance, last_distance_, width, last_width_, height, last_height_);

        // 更新历史值
        last_distance_ = distance;
        last_width_    = width;
        last_height_   = height;

        // 调用 setOption 更新配置
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
        .min_inliers        = 300,     // 最小内点数
        .max_inliers        = 600,     // 最大内点数
        .min_inlier_ratio   = 0.02,    // 最小内点比例
        .max_inlier_ratio   = 0.50,    // 最大内点比例
        .min_height_m       = 1.00,    // 最小高度（米）
        .min_radius_m       = 0.30,    // 最小半径（米）
        .max_radius_m       = 0.40,    // 最大半径(米)
        .min_radial_rel_std = 0.00,    // 最小径向相对标准差
        .max_radial_rel_std = 0.15,    // 最大径向相对标准差
        .slice_height_      = height,  // 切片高度（毫米）
        .slice_weight_      = width,   // 切片宽度（毫米）
        .slice_distance_    = distance // 切片间距（毫米）
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