

#include "cylinder_fitting_ros.hpp"

void CylinderFittingROS::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    static ros::Time last_log_time = ros::Time(0);
    ros::Time        now           = ros::Time::now();
    if ((now - last_log_time).toSec() > 10.0) {
        ROS_INFO("Received filtered cloud with %zu points", cloud->size());
        last_log_time = now;
    }

    try {
        /// archive_work_pose :true 开启检测，false：不检测
        // std_msgs::Bool data;
        if (processing_.load(std::memory_order_acquire)) {
            ROS_ERROR("Cylinder fitting is currently enabled by main control command.");
            if (fitter_->run_with_rslidar(cloud) != -1) {

                ROS_ERROR("Publishing target pose list...");
                std::vector<CylinderFitter::SliceData> slices = fitter_->getSliceResults(); //获取切片结果
                pcl_process::TargetPoseList            tpl_msg;                             //构造消息
                tpl_msg.targets.reserve(slices.size());
                for (const auto &slice : slices) {
                    pcl_process::TargetPose pose;
                    pose.id = slice.id + 1;
                    pose.x  = slice.center_mm.x();
                    pose.y  = slice.center_mm.y();
                    pose.z  = slice.center_mm.z();
                    tpl_msg.targets.push_back(pose);
                }
                target_pose_pub_.publish(tpl_msg);
                for (const auto &target : tpl_msg.targets) {
                    ROS_ERROR(" Target ID: %d, Position (mm): [%.2f, %.2f, %.2f]", target.id, target.x, target.y,
                              target.z);
                }
                ROS_ERROR("ONCE cylinder fitting completed.");
                // TODO 单次检测后续处理
                // target_pose_list
            } else {
                ROS_WARN("No valid cylinder model found; skipping target pose publishing.");
            }
        } else {
            ROS_INFO("Cylinder fitting is currently disabled by main control command.");
        }

    } catch (const std::exception &e) {
        ROS_ERROR("Cylinder fitting failed: %s", e.what());
    }
}

void CylinderFittingROS::mainCtlCallback(const std_msgs::Bool &msg) {
    if (msg.data) {
        ROS_INFO("Main control command received: Start cylinder fitting.");
        processing_.store(true, std::memory_order_release);
    } else {
        ROS_INFO("Main control command received: Stop cylinder fitting.");
        processing_.store(false, std::memory_order_release);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "cylinder_fitting_ros");
    ros::NodeHandle nh;

    ROS_INFO("CylinderFittingROS node started, waiting for filtered point clouds and main control commands...");
    CylinderFittingROS node(nh);

    ros::spin();
    return 0;
}