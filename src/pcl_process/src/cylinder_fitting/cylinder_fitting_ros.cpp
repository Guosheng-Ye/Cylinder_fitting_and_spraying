

#include "cylinder_fitting_ros.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <atomic>

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

            tpl_msg.targets.reserve(slices.size());
            for (const auto &slice : slices) { //切片中心
                pcl_process::TargetPose pose;
                pose.id   = slice.id + 1;
                ti_msg.id = slice.id + 1;

                // 转换 喷涂中心 to base
                Eigen::Vector3d pos_spray_center(slice.center_mm.x(), slice.center_mm.y(), slice.center_mm.z());
                Eigen::Vector3d pos_spray_center2base = tf_lidar2_base_->to_base_from_lidar(pos_spray_center);
                pose.x                                = pos_spray_center2base.x() / 1000.0; // mm -> m
                pose.y                                = pos_spray_center2base.y() / 1000.0;
                pose.z                                = pos_spray_center2base.z() / 1000.0;
                pose.z += 0.2; // 喷涂高度提升 200mm
                ROS_WARN(" Target ID: %d, Spray Position in Base (mm): [%.2f, %.2f, %.2f]", pose.id, pose.x, pose.y,
                         pose.z);

                // 转换 圆柱中心 to base
                Eigen::Vector3d pos_circle_center(slice.circle_center_mm.x(), slice.circle_center_mm.y(),
                                                  slice.circle_center_mm.z());
                Eigen::Vector3d pos_circle_center2base = tf_lidar2_base_->to_base_from_lidar(pos_circle_center);
                // 圆柱中心
                ti_msg.circle_center[0] = pos_circle_center2base.x();
                ti_msg.circle_center[1] = pos_circle_center2base.y();
                ti_msg.circle_center[2] = pos_circle_center2base.z();
                ti_msg.circle_center[2] += 200.0; // 圆柱中心提升200mm

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

            // for (const auto &target : tpl_msg.targets) {
            //     Eigen::Vector3d pos_spray_center(target.x, target.y, target.z);
            //     ROS_ERROR(" Target ID: %d, Position (mm): [%.2f, %.2f, %.2f]", target.id, target.x, target.y,
            //     target.z);

            //     Eigen::Vector3d pos_circle_center(fitter_->slicesData()[target.id - 1].circle_center_mm.x(),
            //                                       fitter_->slicesData()[target.id - 1].circle_center_mm.y(),
            //                                       fitter_->slicesData()[target.id - 1].circle_center_mm.z());
            //     // cal1: 转换雷达坐标到车辆基坐标系
            //     Eigen::Vector3d pos_spray_center2base = tf_lidar2_base_->to_base_from_lidar(pos_spray_center);
            //     ROS_ERROR(" Transformed pos_spray_center in Base (mm): [%.2f, %.2f, %.2f]",
            //     pos_spray_center2base.x(),
            //               pos_spray_center2base.y(), pos_spray_center2base.z());
            //     Eigen::Vector3d pos_circle_center2base = tf_lidar2_base_->to_base_from_lidar(pos_circle_center);
            //     ROS_ERROR(" Transformed pos_circle_center in Base (mm): [%.2f, %.2f, %.2f]",
            //     pos_circle_center2base.x(),
            //               pos_circle_center2base.y(), pos_circle_center2base.z());

            //     // // cal2: 转换雷达坐标到机械臂坐标系
            //     // Eigen::Vector3d pos_robot = tf_lidar2_base_->to_robot_from_lidar(pos_mm);
            //     // ROS_ERROR(" Transformed Position in Robot (mm): [%.2f, %.2f, %.2f]", pos_robot.x(), pos_robot.y(),
            //     //           pos_robot.z());
            // }
            ROS_ERROR("ONCE cylinder fitting completed.");
        } else {
            ROS_WARN("No valid cylinder model found; skipping target pose publishing.");
        }

    } catch (const std::exception &e) {
        ROS_ERROR("Cylinder fitting failed: %s", e.what());
    }
}

void CylinderFittingROS::mainCtlCallback(const std_msgs::Bool &msg) {
    bool prev = processing_.load(std::memory_order_acquire);
    bool next = msg.data;
    if (prev == next) {
        // 状态未变化，不重复打印
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

void CylinderFittingROS::setOption() {

    this->fitter_->sliceOptions().surface_       = false; // 弧形面网格
    this->fitter_->sliceOptions().border_        = false; // 切片边框线
    this->fitter_->sliceOptions().height_rings_  = false; // 高度层环线
    this->fitter_->sliceOptions().print_centers_ = true;  // 切片中心点

    this->fitter_->setFittingParams(CylinderFitter::CylinderFitterParams{
        .min_inliers        = 300,    // 最小内点数
        .max_inliers        = 600,    // 最大内点数
        .min_inlier_ratio   = 0.02,   // 最小内点比例
        .max_inlier_ratio   = 0.50,   // 最大内点比例
        .min_height_m       = 1.00,   // 最小高度（米）
        .min_radius_m       = 0.30,   // 最小半径（米）
        .max_radius_m       = 0.40,   // 最大半径(米)
        .min_radial_rel_std = 0.00,   // 最小径向相对标准差
        .max_radial_rel_std = 0.15,   // 最大径向相对标准差
        .slice_height_      = 600.00, // 切片高度（毫米）
        .slice_weight_      = 600.00, // 切片宽度（毫米）
    });
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "cylinder_fitting_ros");
    ros::NodeHandle nh;

    ROS_INFO("CylinderFittingROS node started, waiting for filtered point clouds and main control commands...");
    CylinderFittingROS node(nh);
    node.setOption();

    ros::spin();
    return 0;
}