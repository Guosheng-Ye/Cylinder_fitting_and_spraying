#include "cylinder_fitting_node.hpp"
// #include "cylinder_fitting_ros.hpp"
#include "rslidar_process.hpp"
#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "pcl_process_main");
    ros::NodeHandle nh;

    // RSLidarFilterNode  rslidar_node(nh);
    // CylinderFitter fitter("/home/oneko/projects/lidar/ros_airy_work/src/pcd_datas/4.pcd");

    // // 可视化输出参数
    // CylinderFitter::SliceVizOptions opt;
    // opt.border_        = true;
    // opt.height_rings_  = false;
    // opt.surface_       = false;
    // opt.print_centers_ = true;

    // //圆柱拟合参数，切片参数
    // CylinderFitter::CylinderFitterParams params;
    // params.slice_height_ = 500.0;
    // params.slice_weight_ = 500.0;
    // fitter.setFittingParams(params);
    // fitter.setSliceVizOptions(opt);
    // fitter.run_with_pcd();

    ROS_INFO("PCL pipeline initialized: Filter + Fitting running in one process.");

    ros::spin(); // 同时处理两个订阅者的回调
    return 0;
}
