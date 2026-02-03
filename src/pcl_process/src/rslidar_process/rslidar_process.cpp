#include "rslidar_process.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

RSLidarFilterNode::RSLidarFilterNode(ros::NodeHandle &nh) {
    // sub_ = nh.subscribe("/rslidar_points", 10, &RSLidarFilterNode::cloudCallback, this);
    sub_ = nh.subscribe("/front_arm_lidar", 10, &RSLidarFilterNode::cloudCallback, this);

    // 圆柱拟合：±20度过滤
    pub_cylinder_ = nh.advertise<sensor_msgs::PointCloud2>("/rslidar_filtered_cylinder", 10);
    // 工字钢拟合：±45度过滤
    pub_ibeam_ = nh.advertise<sensor_msgs::PointCloud2>("/rslidar_filtered_ibeam", 10);

    // 每1秒检查一次是否有数据输入
    watchdog_timer_ = nh.createTimer(ros::Duration(1.0), &RSLidarFilterNode::watchdogCallback, this);
    last_msg_time_  = ros::Time::now();

    ROS_INFO("RSLidar Filter initialized:");
    ROS_INFO("  Input topic: /front_arm_lidar");
    ROS_INFO("  Cylinder output: /rslidar_filtered (+/-%.0f deg)", CYLINDER_ANGLE_DEG);
    ROS_INFO("  IBeam output: /rslidar_filtered_ibeam (+/-%.0f deg)", IBEAM_ANGLE_DEG);
}

void RSLidarFilterNode::watchdogCallback(const ros::TimerEvent &event) {
    double elapsed = (ros::Time::now() - last_msg_time_).toSec();
    // 超过2秒没有数据输入，认为数据断开
    if (elapsed > 2.0 && data_active_) {
        ROS_WARN_STREAM("[Angle Filter] No point cloud input for " << elapsed << " seconds");
        data_active_              = false;
        last_has_output_cylinder_ = false; // 重置输出状态，恢复时重新打印
        last_has_output_ibeam_    = false;
    }
}

void RSLidarFilterNode::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    // 更新最后消息时间
    last_msg_time_ = ros::Time::now();
    if (!data_active_) {
        ROS_INFO_STREAM("[Angle Filter] Point cloud input resumed");
        data_active_ = true;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud);

    if (cloud->empty()) {
        if (last_has_output_cylinder_ || last_has_output_ibeam_) {
            ROS_WARN_STREAM("[Angle Filter] No input points received");
            last_has_output_cylinder_ = false;
            last_has_output_ibeam_    = false;
        }
        return;
    }

    // ============== 角度过滤（侧向安装：+Y 为前向，+X 为右侧） ==============
    // 圆柱拟合：保留相对于 +Y 前向左右各20度（总 40 度扇区）
    // 工字钢拟合：保留相对于 +Y 前向左右各45度（总 90 度扇区）

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cylinder(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_ibeam(new pcl::PointCloud<pcl::PointXYZI>);

    const double angle_limit_cylinder = CYLINDER_ANGLE_DEG * M_PI / 180.0;
    const double angle_limit_ibeam    = IBEAM_ANGLE_DEG * M_PI / 180.0;

    for (const auto &p : cloud->points) {
        if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z)) continue; // 跳过 NaN
        if (p.y <= 0) continue;                                              // 位于后方或原点附近，不保留

        // 计算点相对于前向(+Y)的偏转角：前向向量为 (0,1)，使用 atan2(横向, 前向)
        double angle     = std::atan2(p.x, p.y); // p.x 为横向（右正），p.y 为前向
        double abs_angle = std::abs(angle);

        // 圆柱拟合过滤 (±20度)
        if (abs_angle <= angle_limit_cylinder) {
            filtered_cylinder->push_back(p);
        }

        // 工字钢拟合过滤 (±45度)
        if (abs_angle <= angle_limit_ibeam) {
            filtered_ibeam->push_back(p);
        }
    }

    // 仅在有点/无点状态切换时打印 - 圆柱过滤
    bool has_output_cylinder = !filtered_cylinder->empty();
    if (has_output_cylinder != last_has_output_cylinder_) {
        if (has_output_cylinder) {
            ROS_INFO_STREAM("[Cylinder Filter] Input: " << cloud->size() << " -> Output: " << filtered_cylinder->size()
                                                        << " points (+/-" << CYLINDER_ANGLE_DEG << " deg)");
        } else {
            ROS_WARN_STREAM("[Cylinder Filter] No points in filtered output");
        }
        last_has_output_cylinder_ = has_output_cylinder;
    }

    // 仅在有点/无点状态切换时打印 - 工字钢过滤
    bool has_output_ibeam = !filtered_ibeam->empty();
    if (has_output_ibeam != last_has_output_ibeam_) {
        if (has_output_ibeam) {
            ROS_INFO_STREAM("[IBeam Filter] Input: " << cloud->size() << " -> Output: " << filtered_ibeam->size()
                                                     << " points (+/-" << IBEAM_ANGLE_DEG << " deg)");
        } else {
            ROS_WARN_STREAM("[IBeam Filter] No points in filtered output");
        }
        last_has_output_ibeam_ = has_output_ibeam;
    }

    // ============== 发布过滤结果 ==============
    // 发布圆柱拟合用点云
    sensor_msgs::PointCloud2 output_cylinder;
    pcl::toROSMsg(*filtered_cylinder, output_cylinder);
    output_cylinder.header = msg->header;
    pub_cylinder_.publish(output_cylinder);

    // 发布工字钢拟合用点云
    sensor_msgs::PointCloud2 output_ibeam;
    pcl::toROSMsg(*filtered_ibeam, output_ibeam);
    output_ibeam.header = msg->header;
    pub_ibeam_.publish(output_ibeam);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "rslidar_filter_node");
    ros::NodeHandle nh;

    ROS_INFO("PCL pipeline initialized: Filter + Fitting running in one process.");
    RSLidarFilterNode node(nh);

    ros::spin();
    return 0;
}