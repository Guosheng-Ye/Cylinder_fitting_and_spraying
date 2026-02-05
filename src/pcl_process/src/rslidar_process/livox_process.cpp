/**
 * @file livox_process.cpp
 * @brief Livox 激光雷达点云累积处理节点
 * 
 * 订阅 /livox/lidar 话题，累积多帧点云数据后发布
 * 用于解决 Livox 非重复扫描模式下单帧点云稀疏的问题
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <mutex>
#include <deque>

class LivoxAccumulatorNode {
public:
    LivoxAccumulatorNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
        // 从参数服务器获取配置
        pnh.param<int>("accumulate_frames", accumulate_frames_, 10);
        pnh.param<double>("voxel_size", voxel_size_, 0.05);  // 体素滤波器分辨率 (米)，默认5cm
        pnh.param<bool>("enable_voxel_filter", enable_voxel_filter_, true);
        pnh.param<std::string>("input_topic", input_topic_, "/livox/lidar");
        pnh.param<std::string>("output_topic", output_topic_, "/livox/accumulated");
        pnh.param<double>("publish_rate", publish_rate_, 10.0);  // 发布频率 Hz
        
        // 角度过滤参数 (与 rslidar_process 保持一致)
        pnh.param<bool>("enable_angle_filter", enable_angle_filter_, true);
        pnh.param<double>("angle_filter_deg", angle_filter_deg_, 45.0);  // 前向±45度
        pnh.param<std::string>("forward_axis", forward_axis_, "x");  // Livox 默认 X 为前向

        // 订阅与发布
        sub_ = nh.subscribe(input_topic_, 100, &LivoxAccumulatorNode::cloudCallback, this);
        pub_ = nh.advertise<sensor_msgs::PointCloud2>(output_topic_, 10);

        // 定时发布累积点云
        publish_timer_ = nh.createTimer(ros::Duration(1.0 / publish_rate_), 
                                         &LivoxAccumulatorNode::publishCallback, this);

        // Watchdog 定时器检测数据断开
        watchdog_timer_ = nh.createTimer(ros::Duration(1.0), 
                                          &LivoxAccumulatorNode::watchdogCallback, this);
        last_msg_time_ = ros::Time::now();

        ROS_INFO("========================================");
        ROS_INFO("Livox Accumulator Node Initialized:");
        ROS_INFO("  Input topic: %s", input_topic_.c_str());
        ROS_INFO("  Output topic: %s", output_topic_.c_str());
        ROS_INFO("  Accumulate frames: %d", accumulate_frames_);
        ROS_INFO("  Voxel filter: %s (size: %.3f m)", 
                 enable_voxel_filter_ ? "enabled" : "disabled", voxel_size_);
        ROS_INFO("  Publish rate: %.1f Hz", publish_rate_);
        ROS_INFO("  Angle filter: %s (+/-%.0f deg, forward: %s)",
                 enable_angle_filter_ ? "enabled" : "disabled", 
                 angle_filter_deg_, forward_axis_.c_str());
        ROS_INFO("========================================");
    }

private:
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        // 更新最后消息时间
        last_msg_time_ = ros::Time::now();
        if (!data_active_) {
            ROS_INFO("[Livox Accumulator] Point cloud input resumed");
            data_active_ = true;
        }

        // 转换为 PCL 点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty()) {
            return;
        }

        // 保存帧头信息
        latest_header_ = msg->header;

        // 添加到缓冲队列
        cloud_buffer_.push_back(cloud);
        total_points_ += cloud->size();

        // 保持队列大小
        while (cloud_buffer_.size() > static_cast<size_t>(accumulate_frames_)) {
            total_points_ -= cloud_buffer_.front()->size();
            cloud_buffer_.pop_front();
        }

        frame_count_++;
    }

    void publishCallback(const ros::TimerEvent& event) {
        std::lock_guard<std::mutex> lock(mutex_);

        if (cloud_buffer_.empty()) {
            return;
        }

        // 合并所有缓冲的点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr accumulated(new pcl::PointCloud<pcl::PointXYZI>);
        for (const auto& cloud : cloud_buffer_) {
            *accumulated += *cloud;
        }

        if (accumulated->empty()) {
            return;
        }

        size_t before_filter = accumulated->size();

        // 角度过滤 (类似 rslidar_process 的前向扇形过滤)
        if (enable_angle_filter_) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr angle_filtered(new pcl::PointCloud<pcl::PointXYZI>);
            const double angle_limit = angle_filter_deg_ * M_PI / 180.0;
            
            for (const auto& p : accumulated->points) {
                if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z)) continue;
                
                double forward, lateral;
                // 根据配置的前向轴选择坐标
                if (forward_axis_ == "x" || forward_axis_ == "X") {
                    forward = p.x;  // Livox: X 为前向
                    lateral = p.y;  // Y 为侧向
                } else {
                    forward = p.y;  // RSLidar: Y 为前向
                    lateral = p.x;  // X 为侧向
                }
                
                if (forward <= 0) continue;  // 后方点云不保留
                
                double angle = std::atan2(lateral, forward);
                if (std::abs(angle) <= angle_limit) {
                    angle_filtered->push_back(p);
                }
            }
            
            ROS_DEBUG_THROTTLE(5.0, "[Livox] Angle filter: %zu -> %zu points",
                              accumulated->size(), angle_filtered->size());
            accumulated = angle_filtered;
            
            if (accumulated->empty()) {
                ROS_WARN_THROTTLE(5.0, "[Livox Accumulator] No points after angle filtering");
                return;
            }
        }

        // 可选的体素滤波降采样
        if (enable_voxel_filter_ && voxel_size_ > 0) {
            // 计算点云范围，检查体素大小是否合适
            pcl::PointXYZI min_pt, max_pt;
            pcl::getMinMax3D(*accumulated, min_pt, max_pt);
            
            double range_x = max_pt.x - min_pt.x;
            double range_y = max_pt.y - min_pt.y;
            double range_z = max_pt.z - min_pt.z;
            double max_range = std::max({range_x, range_y, range_z});
            
            // 动态调整体素大小，确保每个维度的格子数不超过 1000
            double safe_voxel_size = voxel_size_;
            double min_safe_size = max_range / 1000.0;
            if (safe_voxel_size < min_safe_size) {
                safe_voxel_size = min_safe_size;
                ROS_WARN_THROTTLE(5.0, "[Livox Accumulator] Voxel size adjusted from %.3f to %.3f to prevent overflow",
                                  voxel_size_, safe_voxel_size);
            }
            
            pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
            voxel_filter.setInputCloud(accumulated);
            voxel_filter.setLeafSize(safe_voxel_size, safe_voxel_size, safe_voxel_size);
            voxel_filter.filter(*filtered);
            accumulated = filtered;
        }

        // 发布累积点云
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*accumulated, output);
        output.header = latest_header_;
        output.header.stamp = ros::Time::now();  // 使用当前时间
        pub_.publish(output);

        // 状态打印 (每50帧打印一次)
        if (frame_count_ % 50 == 0) {
            ROS_INFO("[Livox Accumulator] Frames: %zu, Before filter: %zu, After: %zu points",
                     cloud_buffer_.size(), before_filter, accumulated->size());
        }
    }

    void watchdogCallback(const ros::TimerEvent& event) {
        double elapsed = (ros::Time::now() - last_msg_time_).toSec();
        if (elapsed > 2.0 && data_active_) {
            ROS_WARN("[Livox Accumulator] No point cloud input for %.1f seconds", elapsed);
            data_active_ = false;
        }
    }

private:
    ros::Subscriber sub_;
    ros::Publisher pub_;
    ros::Timer publish_timer_;
    ros::Timer watchdog_timer_;

    std::mutex mutex_;
    std::deque<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_buffer_;
    std_msgs::Header latest_header_;

    // 参数
    int accumulate_frames_;
    double voxel_size_;
    bool enable_voxel_filter_;
    std::string input_topic_;
    std::string output_topic_;
    double publish_rate_;
    
    // 角度过滤参数
    bool enable_angle_filter_;
    double angle_filter_deg_;
    std::string forward_axis_;

    // 状态
    ros::Time last_msg_time_;
    bool data_active_ = false;
    size_t frame_count_ = 0;
    size_t total_points_ = 0;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "livox_accumulator_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ROS_INFO("Starting Livox Accumulator Node...");
    LivoxAccumulatorNode node(nh, pnh);

    ros::spin();
    return 0;
}
