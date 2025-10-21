#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
// #include <pcl/segmentation/sac_segmentation_from_normals.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <Eigen/Dense>
#include <chrono>
#include <iostream>
#include <limits>
#include <string>
#include <thread>

class CylinderFitter {
public:
    CylinderFitter(const std::string &pcd_path)
        : pcd_path_(pcd_path)
        , viewer_(new pcl::visualization::PCLVisualizer("Cylinder Fitting Viewer")) {
        viewer_->setBackgroundColor(0, 0, 0);
        viewer_->addCoordinateSystem(1.0);
        viewer_->initCameraParameters();
    }

    int run() {
        // Step 1: 读取 PCD
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path_, *cloud_) == -1) {
            PCL_ERROR("Couldn't read file %s\n", pcd_path_.c_str());
            return -1;
        }
        std::cout << "Loaded " << cloud_->size() << " points from " << pcd_path_ << std::endl;

        // ============================= 角度过滤 =============================
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_angle(new pcl::PointCloud<pcl::PointXYZ>);
        const double                        angle_limit = 30.0 * M_PI / 180.0;

        for (const auto &p : cloud_->points) {
            if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z)) continue;
            if (p.x <= 0) continue;
            double angle = std::atan2(p.y, p.x);
            if (std::abs(angle) <= angle_limit) {
                cloud_filtered_angle->push_back(p);
            }
        }
        std::cout << "After angle filtering: " << cloud_filtered_angle->size() << " points remain.\n";
        cloud_ = cloud_filtered_angle;

        // ============================= 移除地面和平面 =============================
        //removePlanes(cloud_, 0.02, 3, 10.0 * M_PI / 180.0); // 最大移除5个平面，角度阈值10°

        if (cloud_->empty()) {
            std::cerr << "[WARN] No points left after plane removal.\n";
            return -1;
        }

        // ============================= 法线估计 =============================
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        estimateNormals(cloud_, normals);

        // ============================= 圆柱拟合 =============================
        Eigen::Vector3f                     axis, center;
        double                              radius = 0.0;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        fitCylinderPCL(cloud_, normals, axis, center, radius, cylinder_cloud);

        // ============================= 可视化 =============================
        visualizeResults(cloud_, cylinder_cloud, center, axis, radius);

        return 0;
    }

private:
    std::string                            pcd_path_;
    pcl::visualization::PCLVisualizer::Ptr viewer_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr    cloud_{new pcl::PointCloud<pcl::PointXYZ>()};

    // ============================= 移除水平地面和平面 =============================
    // void removePlanes(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double dist_thresh, int max_planes,
    //                   double horizontal_angle_thresh) // 单位弧度
    // {
    //     int planes_removed = 0;

    //     while (planes_removed < max_planes) {
    //         pcl::SACSegmentation<pcl::PointXYZ> seg;
    //         pcl::PointIndices::Ptr              inliers(new pcl::PointIndices);
    //         pcl::ModelCoefficients::Ptr         coefficients(new pcl::ModelCoefficients);
    //         pcl::ExtractIndices<pcl::PointXYZ>  extract;

    //         seg.setOptimizeCoefficients(true);
    //         seg.setModelType(pcl::SACMODEL_PLANE);
    //         seg.setMethodType(pcl::SAC_RANSAC);
    //         seg.setDistanceThreshold(dist_thresh);
    //         seg.setInputCloud(cloud);
    //         seg.segment(*inliers, *coefficients);

    //         if (inliers->indices.empty()) break;

    //         // 判断平面是否水平（地面）
    //         Eigen::Vector3f normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    //         normal.normalize();
    //         double angle = std::acos(normal.dot(Eigen::Vector3f::UnitZ()));

    //         // 如果水平或接近平面，移除
    //         if (angle < horizontal_angle_thresh || angle > M_PI - horizontal_angle_thresh) {
    //             extract.setInputCloud(cloud);
    //             extract.setIndices(inliers);
    //             extract.setNegative(true);
    //             extract.filter(*cloud);
    //             std::cout << "[INFO] Ground plane removed. Remaining points: " << cloud->size() << std::endl;
    //             ++planes_removed;
    //         } else {
    //             // 非水平平面也移除（墙面等）
    //             extract.setInputCloud(cloud);
    //             extract.setIndices(inliers);
    //             extract.setNegative(true);
    //             extract.filter(*cloud);
    //             std::cout << "[INFO] Other plane removed. Remaining points: " << cloud->size() << std::endl;
    //             ++planes_removed;
    //         }
    //     }
    // }

    // ============================= 法线估计 =============================
    void estimateNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals) {
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr           tree(new pcl::search::KdTree<pcl::PointXYZ>());
        ne.setInputCloud(cloud);
        ne.setSearchMethod(tree);
        ne.setKSearch(200); // 搜索点数
        ne.compute(*normals);
    }

    // ============================= 圆柱拟合 =============================
    void fitCylinderPCL(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals,
                        Eigen::Vector3f &axis, Eigen::Vector3f &center, double &radius,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder_points) {
        pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_CYLINDER);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setNormalDistanceWeight(0.1);
        seg.setMaxIterations(20000);
        seg.setDistanceThreshold(0.02);
        seg.setRadiusLimits(0.01, 0.50); // 放宽范围
        seg.setInputCloud(cloud);
        seg.setInputNormals(normals);

        // 强制轴线接近垂直
        seg.setAxis(Eigen::Vector3f(0, 0, 1));
        seg.setEpsAngle(10.0f * M_PI / 180.0f); // 最大允许偏差10°

        pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr      inliers_cylinder(new pcl::PointIndices);
        seg.segment(*inliers_cylinder, *coefficients_cylinder);

        if (inliers_cylinder->indices.empty()) {
            std::cerr << "[WARN] No cylinder found.\n";
            return;
        }

        // 提取内点
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inliers_cylinder);
        extract.setNegative(false);
        extract.filter(*cylinder_points);

        // ================== 半径优化 ==================
        // 将内点投影到 XY 平面（垂直截面），计算最大外接圆半径
        Eigen::Vector3f fitted_axis(coefficients_cylinder->values[3], coefficients_cylinder->values[4],
                                    coefficients_cylinder->values[5]);
        fitted_axis.normalize();
        Eigen::Vector3f proj_plane_normal = fitted_axis; // 投影平面法向量
        Eigen::Vector3f proj_center(coefficients_cylinder->values[0], coefficients_cylinder->values[1],
                                    coefficients_cylinder->values[2]);

        double max_r = 0;
        for (const auto &p : cylinder_points->points) {
            Eigen::Vector3f pt(p.x, p.y, p.z);
            // 投影到截面（垂直轴向）
            Eigen::Vector3f v      = pt - proj_center;
            Eigen::Vector3f v_proj = v - (v.dot(fitted_axis)) * fitted_axis;
            double          r      = v_proj.norm();
            if (r > max_r) max_r = r;
        }
        radius = max_r; // 更新半径

        center = proj_center;
        axis   = fitted_axis;

        std::cout << "Cylinder detected (optimized radius):\n"
                  << "  Center: [" << center.transpose() << "]\n"
                  << "  Axis:   [" << axis.transpose() << "]\n"
                  << "  Radius: " << radius << "\n"
                  << "  Inliers: " << inliers_cylinder->indices.size() << " points\n";
    }

    // ============================= 可视化 =============================
    void visualizeResults(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder_points, const Eigen::Vector3f &center,
                          const Eigen::Vector3f &axis, double radius) {
        viewer_->removeAllPointClouds();
        viewer_->removeAllShapes();

        // 白色原始点云
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> white(cloud, 255, 255, 255);
        viewer_->addPointCloud<pcl::PointXYZ>(cloud, white, "cloud");

        // 绿色内点
        if (cylinder_points && !cylinder_points->empty()) {
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(cylinder_points, 0, 255, 0);
            viewer_->addPointCloud<pcl::PointXYZ>(cylinder_points, green, "cylinder_points");
            viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3,
                                                      "cylinder_points");
        }

        // 自动计算圆柱高度范围（内点沿轴方向投影）
        double          min_proj  = std::numeric_limits<double>::max();
        double          max_proj  = std::numeric_limits<double>::lowest();
        Eigen::Vector3f axis_norm = axis.normalized();

        for (const auto &p : cylinder_points->points) {
            Eigen::Vector3f pt(p.x, p.y, p.z);
            double          proj = axis_norm.dot(pt - center);
            min_proj             = std::min(min_proj, proj);
            max_proj             = std::max(max_proj, proj);
        }

        // 构建红色圆柱表面点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr red_cylinder(new pcl::PointCloud<pcl::PointXYZ>);
        int                                 n_circ   = 36; // 每层点数
        int                                 n_height = 50; // 高度层数

        Eigen::Vector3f x = axis_norm.unitOrthogonal();
        Eigen::Vector3f y = axis_norm.cross(x);

        for (int i = 0; i <= n_height; ++i) {
            double          h            = min_proj + (max_proj - min_proj) * i / n_height;
            Eigen::Vector3f layer_center = center + h * axis_norm;
            for (int j = 0; j < n_circ; ++j) {
                double          theta = 2.0 * M_PI * j / n_circ;
                Eigen::Vector3f pt    = layer_center + radius * (std::cos(theta) * x + std::sin(theta) * y);
                red_cylinder->push_back(pcl::PointXYZ(pt.x(), pt.y(), pt.z()));
            }
        }

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(red_cylinder, 255, 0, 0);
        viewer_->addPointCloud<pcl::PointXYZ>(red_cylinder, red, "cylinder_surface");
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cylinder_surface");

        // 圆柱轴线
        Eigen::Vector3f start = center + min_proj * axis_norm;
        Eigen::Vector3f end   = center + max_proj * axis_norm;
        viewer_->addArrow(pcl::PointXYZ(end.x(), end.y(), end.z()), pcl::PointXYZ(start.x(), start.y(), start.z()), 1.0,
                          0.0, 0.0, false, "cylinder_axis");

        while (!viewer_->wasStopped()) {
            viewer_->spinOnce(10);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
};

int main(int argc, char **argv) {
    std::string    path = "/home/oneko/projects/lidar/ros_airy_work/src/pcd_datas/3.pcd";
    CylinderFitter app(path);
    return app.run();
}
