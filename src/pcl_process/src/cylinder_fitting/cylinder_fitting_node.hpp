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
#include <ros/ros.h>
#include <string>
#include <thread>
#include <vector>

class CylinderFitter {
public:
    CylinderFitter(const std::string &pcd_path);
    ~CylinderFitter();

public:
    int run_with_pcd();
    int run_with_rslidar(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input);

    // 获取拟合结果
    bool                                hasModel() const { return cylinder_model_.valid; }
    Eigen::Vector3f                     axis() const { return cylinder_model_.axis; }
    Eigen::Vector3f                     bottomCenter() const { return cylinder_model_.bottom_center; }
    Eigen::Vector3f                     topCenter() const { return cylinder_model_.top_center; }
    Eigen::Vector3f                     geoCenter() const { return cylinder_model_.geo_center; }
    double                              radius() const { return cylinder_model_.radius; }
    double                              height() const { return cylinder_model_.height; }
    pcl::PointCloud<pcl::PointXYZ>::Ptr inlierCloud() const { return cylinder_model_.inliers; }

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
    void fitCylinderPCL(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals);

    /// @brief 对圆柱表面切片
    /// @param center 拟合圆柱中心
    /// @param axis 圆柱轴线方向向量
    /// @param radius 半径
    /// @param height 高度
    /// @param angle_range_deg 角度范围（度）
    /// @param slice_height 切片高度
    /// @param slice_width 切片宽度
    void sliceCylinder(double angle_range_deg, double slice_height_mm, double slice_width_mm);

    /// @brief 可视化
    /// @param cloud 点云
    /// @param cylinder_points 圆柱内点
    /// @param center 中心
    /// @param axis 轴向
    /// @param radius 半径
    void visualizeResults();

    void printResults();
    void viewerThread();

private:
    std::string                            pcd_path_;                                    // PCD文件路径
    pcl::visualization::PCLVisualizer::Ptr viewer_;                                      //可视化器
    pcl::PointCloud<pcl::PointXYZ>::Ptr    cloud_{new pcl::PointCloud<pcl::PointXYZ>()}; // 点云数据

    // 非阻塞绘制
    std::thread                  viewer_thread_;             // 视图线程
    std::atomic<bool>            running_{true};             // 运行标志
    std::mutex                   viewer_mutex_;              // 保护viewer线程
    std::vector<std::string>     slice_shape_ids_;           // 切片形状ID
    std::vector<Eigen::Vector3f> slice_centers_mm_;          // 存储切片中心
    std::vector<std::string>     slice_text_ids_;            // 文本ID
    std::vector<std::string>     slice_line_ids_;            // 连线ID
    bool                         show_slice_surface_{false}; // 是否显示弧形面网格
    bool                         show_slice_border_{false};  // 是否显示边框线
    bool                         print_slice_centers_{true}; // 是否打印切片中心点

    struct CylinderModel {
        bool                                valid{false};                                  // 圆柱是否有效
        Eigen::Vector3f                     axis{0, 0, 1};                                 // 轴向
        Eigen::Vector3f                     bottom_center{0, 0, 0};                        // 底面中心
        Eigen::Vector3f                     top_center{0, 0, 0};                           // 顶面中心
        Eigen::Vector3f                     geo_center{0, 0, 0};                           // 几何中心
        double                              radius{0.0};                                   // 半径
        double                              height{0.0};                                   // 高度
        pcl::PointCloud<pcl::PointXYZ>::Ptr inliers{new pcl::PointCloud<pcl::PointXYZ>()}; // 内点
    } cylinder_model_;
};

#endif