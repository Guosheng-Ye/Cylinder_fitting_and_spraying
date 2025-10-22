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
    int run();

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
    /// @param cloud
    /// @param cylinder_points
    /// @param center
    /// @param axis
    /// @param radius
    void visualizeResults(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder_points, const Eigen::Vector3f &center,
                          const Eigen::Vector3f &axis, double radius);

private:
    std::string                            pcd_path_;
    pcl::visualization::PCLVisualizer::Ptr viewer_;                                      //可视化器
    pcl::PointCloud<pcl::PointXYZ>::Ptr    cloud_{new pcl::PointCloud<pcl::PointXYZ>()}; // 点云数据
};

#endif