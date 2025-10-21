#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <mutex>
#include <pcl/visualization/point_cloud_color_handlers.h>

// 使用强度点云类型
pcl::visualization::PCLVisualizer::Ptr viewer;
pcl::PointCloud<pcl::PointXYZI>::Ptr global_cloud(new pcl::PointCloud<pcl::PointXYZI>);
std::mutex cloud_mutex;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    std::lock_guard<std::mutex> lock(cloud_mutex);
    pcl::fromROSMsg(*cloud_msg, *global_cloud);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_test_node");
    ros::NodeHandle nh;

    viewer.reset(new pcl::visualization::PCLVisualizer("PCL ROS Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    ros::Subscriber sub = nh.subscribe("/rslidar_points", 1, cloudCallback);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    bool cloud_added = false;
    while (!viewer->wasStopped())
    {
        {
            std::lock_guard<std::mutex> lock(cloud_mutex);
            if (global_cloud->points.size() > 0) {
                pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_handler(global_cloud, "intensity");
                if (!cloud_added) {
                    viewer->addPointCloud<pcl::PointXYZI>(global_cloud, intensity_handler, "cloud");
                    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
                    cloud_added = true;
                } else {
                    viewer->updatePointCloud<pcl::PointXYZI>(global_cloud, intensity_handler, "cloud");
                }
            }
        }
        viewer->spinOnce(10);
        ros::Duration(0.01).sleep();
    }

    spinner.stop();
    return 0;
}