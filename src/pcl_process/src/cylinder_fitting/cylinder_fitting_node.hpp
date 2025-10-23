#ifndef CYLINDER_FITTING_NODE_HPP
#define CYLINDER_FITTING_NODE_HPP

#include <Eigen/Dense>
#include <chrono>
#include <iostream>
#include <limits>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <string>
#include <thread>

class CylinderFitter {
public:
    CylinderFitter(const std::string &pcd_path);
    ~CylinderFitter();

public:
    int run_with_pcd();
    int run_with_rslidar(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input);

private:
    /// @brief 法线修复
    /// @param cloud 点云
    /// @param normals 法线
    void estimateNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals);

    /// @brief 圆柱拟合
    /// @param cloud 点云
    /// @param normals 法线
    /// @param axis 法线方向
    /// @param center 圆柱中心
    /// @param radius 半径
    /// @param cylinder_points 圆柱内点
    void fitCylinderPCL(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals,
                        Eigen::Vector3f &axis, Eigen::Vector3f &center, double &radius,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder_points);

    /// @brief 可视化
    /// @param cloud 点云
    /// @param cylinder_points 圆柱内点
    /// @param center 中心
    /// @param axis 轴向
    /// @param radius 半径
    void visualizeResults(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder_points, const Eigen::Vector3f &center,
                          const Eigen::Vector3f &axis, double radius);

    /// @brief
    /// @param center 圆柱中线
    /// @param axis 圆柱轴向
    /// @param radius 圆柱半径
    /// @param cylinder_points
    void printResults(const Eigen::Vector3f &center, const Eigen::Vector3f &axis, double radius,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder_points);

    /// @brief 
    void viewerThread();

private:
    std::string                            pcd_path_;
    pcl::visualization::PCLVisualizer::Ptr viewer_;                                      //可视化器
    pcl::PointCloud<pcl::PointXYZ>::Ptr    cloud_{new pcl::PointCloud<pcl::PointXYZ>()}; // 点云数据

    // 非阻塞绘制
    std::thread       viewer_thread_;
    std::atomic<bool> running_{true};
    std::mutex        viewer_mutex_;
};

#endif