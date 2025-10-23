#include "cylinder_fitting_node.hpp"

CylinderFitter::CylinderFitter(const std::string &pcd_path)
    : pcd_path_(pcd_path)
    , viewer_(new pcl::visualization::PCLVisualizer("Cylinder Fitting Viewer")) {
    viewer_->setBackgroundColor(0, 0, 0);
    viewer_->addCoordinateSystem(1.0);
    viewer_->initCameraParameters();

    //
    viewer_thread_ = std::thread(&CylinderFitter::viewerThread, this);
}
CylinderFitter::~CylinderFitter() {
    running_ = false;
    if (viewer_thread_.joinable()) viewer_thread_.join();
}
/// @brief
/// @return
int CylinderFitter::run_with_pcd() {
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
    // removePlanes(cloud_, 0.02, 3, 10.0 * M_PI / 180.0); // 最大移除5个平面，角度阈值10°
    // if (cloud_->empty()) {
    //     std::cerr << "[WARN] No points left after plane removal.\n";
    //     return -1;
    // }

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    // 法线估计
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    estimateNormals(cloud_, normals);

    // 圆柱拟合
    Eigen::Vector3f                     axis, center;
    double                              radius = 0.0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    fitCylinderPCL(cloud_, normals, axis, center, radius, cylinder_cloud);

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_span  = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "Cylinder fitting time: " << time_span.count() << " seconds.\n";

    // 可视化
    visualizeResults(cloud_, cylinder_cloud, center, axis, radius);

    /// 打印数值结果
    printResults(center, axis, radius, cylinder_cloud);

    // 计算圆柱高度并调用切片 =============================
    if (cylinder_cloud && !cylinder_cloud->empty()) {
        Eigen::Vector3f axis_norm = axis.normalized();
        double          min_proj  = std::numeric_limits<double>::max();
        double          max_proj  = std::numeric_limits<double>::lowest();

        for (const auto &p : cylinder_cloud->points) {
            Eigen::Vector3f pt(p.x, p.y, p.z);
            double          proj = axis_norm.dot(pt - center);
            min_proj             = std::min(min_proj, proj);
            max_proj             = std::max(max_proj, proj);
        }

        double          height_m  = (max_proj - min_proj); // m
        double          radius_mm = radius * 1000.0;
        double          height_mm = height_m * 1000.0;
        Eigen::Vector3f center_mm = center * 1000.0;

        std::cout << "\n调用切片函数..." << std::endl;
        sliceCylinder(center_mm, axis, radius_mm, height_mm,
                      180.0, // 角度范围：180°
                      500.0, // 每层高度 500 mm
                      300.0, // 每个切片宽度 300 mm
                      cylinder_cloud);
    }

    return 0;
}

/// @brief
/// @param cloud_input
/// @return
int CylinderFitter::run_with_rslidar(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input) {
    if (!cloud_input || cloud_input->empty()) {
        std::cerr << "[ERROR] Empty point cloud input.\n";
        return -1;
    }

    cloud_ = cloud_input;

    // 法线估计
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    estimateNormals(cloud_, normals);

    // 圆柱拟合
    Eigen::Vector3f                     axis, center;
    double                              radius = 0.0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    fitCylinderPCL(cloud_, normals, axis, center, radius, cylinder_cloud);

    // 可视化
    visualizeResults(cloud_, cylinder_cloud, center, axis, radius);

    // 打印结果
    printResults(center, axis, radius, cylinder_cloud);

    return 0;
}

/// @brief 法线估计
/// @param cloud 点云
/// @param normals 法线
void CylinderFitter::estimateNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                     pcl::PointCloud<pcl::Normal>::Ptr   normals) {
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr           tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    ne.setKSearch(200); // 搜索点数
    ne.compute(*normals);
}

/// @brief 圆柱拟合
/// @param cloud 点云
/// @param normals 法线
/// @param axis 法线方向
/// @param center 圆柱中心
/// @param radius 半径
/// @param cylinder_points 圆柱内点
void CylinderFitter::fitCylinderPCL(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                    pcl::PointCloud<pcl::Normal>::Ptr normals, Eigen::Vector3f &axis,
                                    Eigen::Vector3f &center, double &radius,
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

    center     = proj_center;
    center.z() = 0.0; // 强制中心在 z=0 平面
    axis       = fitted_axis;

    std::cout << "圆柱检测（优化半径）:\n"
              << "  中心: [" << center.transpose() << "]\n"
              << "  轴:   [" << axis.transpose() << "]\n"
              << "  半径: " << radius << "\n"
              << "  内点数: " << inliers_cylinder->indices.size() << " points\n";
}

/// @brief 对圆柱表面切片计算
/// @param center 圆柱中心
/// @param axis 圆柱轴向
/// @param radius 圆柱半径
/// @param height 圆柱总高度
/// @param angle_range_deg 角度范围（°）
/// @param slice_height 切片高
/// @param slice_width 切片宽

void CylinderFitter::sliceCylinder(const Eigen::Vector3f &center_mm, const Eigen::Vector3f &axis, double radius_mm,
                                   double height_mm, double angle_range_deg, double slice_height_mm,
                                   double slice_width_mm, pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder_points) {
    Eigen::Vector3f z = axis.normalized();
    Eigen::Vector3f world_x(-1.0f, 0.0f, 0.0f);

    // --- 底部中心投影到XY平面 ---
    Eigen::Vector3f bottom_center_mm = center_mm;
    bottom_center_mm.z()             = 0.0;

    // 构造局部坐标系 (x, y, z)
    Eigen::Vector3f x;
    if (std::abs(z.dot(world_x)) > 0.99f) {
        x = (Eigen::Vector3f(0.0f, 0.0f, 1.0f).cross(z)).normalized();
    } else {
        x = (world_x - z * (z.dot(world_x))).normalized();
    }
    Eigen::Vector3f y = z.cross(x).normalized();

    double angle_range = angle_range_deg * M_PI / 180.0;
    double min_theta   = -angle_range / 2.0;

    int num_height_slices = std::max(1, static_cast<int>(std::round(height_mm / slice_height_mm)));
    int num_width_slices  = std::max(1, static_cast<int>(std::round((radius_mm * angle_range) / slice_width_mm)));

    viewer_->removeAllShapes();

    for (int i = 0; i < num_height_slices; ++i) {
        double h_mm = (i + 0.5) * slice_height_mm;

        for (int j = 0; j < num_width_slices; ++j) {
            double theta = min_theta + (j + 0.5) * (angle_range / num_width_slices);

            Eigen::Vector3f slice_center_mm =
                bottom_center_mm - h_mm * z + radius_mm * (std::cos(theta) * x + std::sin(theta) * y);

            std::string box_id = "slice_" + std::to_string(i) + "_" + std::to_string(j);
            viewer_->addSphere(
                pcl::PointXYZ(slice_center_mm.x() / 1000.0, slice_center_mm.y() / 1000.0, slice_center_mm.z() / 1000.0),
                5.0 / 1000.0, 1.0, 0.5, 0.0, box_id);
        }
    }
}

/// @brief
/// @param cloud
/// @param cylinder_points
/// @param center
/// @param axis
/// @param radius
void CylinderFitter::visualizeResults(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                      pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder_points,
                                      const Eigen::Vector3f &center, const Eigen::Vector3f &axis, double radius) {

    std::lock_guard<std::mutex> lock(viewer_mutex_);

    viewer_->removeAllPointClouds();
    viewer_->removeAllShapes();
    // 白色原始点云
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> white(cloud, 255, 255, 255);
    viewer_->addPointCloud<pcl::PointXYZ>(cloud, white, "cloud");

    // 绿色内点
    if (cylinder_points && !cylinder_points->empty()) {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(cylinder_points, 0, 255, 0);
        viewer_->addPointCloud<pcl::PointXYZ>(cylinder_points, green, "cylinder_points");
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cylinder_points");
    }

    // 计算圆柱高度范围（内点沿轴方向投影）
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

    // ---------- 源点到圆柱中点连线 ----------
    Eigen::Vector3f source_point(0, 0, 0); // 假设原点为传感器位置
    Eigen::Vector3f cylinder_mid = center + 0.5 * (min_proj + max_proj) * axis_norm;
    viewer_->addLine(pcl::PointXYZ(source_point.x(), source_point.y(), source_point.z()),
                     pcl::PointXYZ(cylinder_mid.x(), cylinder_mid.y(), cylinder_mid.z()), 0.0, 0.0, 1.0,
                     "source_to_cylinder_mid"); // 蓝色

    // ---------- 计算距离 ----------
    double dist_3d = (cylinder_mid - source_point).norm() * 1000.0;           // m -> mm
    double dist_xy = (cylinder_mid - source_point).head<2>().norm() * 1000.0; // m -> mm
    std::cout << "[Distance] 3D: " << dist_3d << " mm, Horizontal XY: " << dist_xy << " mm\n";

    // while (!viewer_->wasStopped()) {
    //     viewer_->spinOnce(10);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // }
}
/// @brief
/// @param center
/// @param axis
/// @param radius
/// @param cylinder_points
void CylinderFitter::printResults(const Eigen::Vector3f &center, const Eigen::Vector3f &axis, double radius,
                                  pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder_points) {
    if (!cylinder_points || cylinder_points->empty()) {
        std::cerr << "[WARN] No cylinder points for result computation.\n";
        return;
    }

    Eigen::Vector3f axis_norm = axis.normalized();

    // 高度（单位 mm）
    double min_proj = std::numeric_limits<double>::max();
    double max_proj = std::numeric_limits<double>::lowest();
    for (const auto &p : cylinder_points->points) {
        Eigen::Vector3f pt(p.x, p.y, p.z);
        double          proj = axis_norm.dot(pt - center);
        min_proj             = std::min(min_proj, proj);
        max_proj             = std::max(max_proj, proj);
    }
    double height = (max_proj - min_proj) * 1000.0; // m -> mm

    // 圆弧角度
    Eigen::Vector3f x         = axis_norm.unitOrthogonal();
    Eigen::Vector3f y         = axis_norm.cross(x);
    double          min_theta = std::numeric_limits<double>::max();
    double          max_theta = std::numeric_limits<double>::lowest();
    for (const auto &p : cylinder_points->points) {
        Eigen::Vector3f v      = Eigen::Vector3f(p.x, p.y, p.z) - center;
        Eigen::Vector3f v_proj = v - v.dot(axis_norm) * axis_norm;
        double          theta  = std::atan2(v_proj.dot(y), v_proj.dot(x));
        min_theta              = std::min(min_theta, theta);
        max_theta              = std::max(max_theta, theta);
    }
    double angle     = std::abs(max_theta - min_theta);
    double angle_deg = angle * 180.0 / M_PI;

    // 喷涂区域中心（单位 mm）
    double          theta_mid = 0.5 * (min_theta + max_theta);
    Eigen::Vector3f spray_center =
        center + radius * (std::cos(theta_mid) * x + std::sin(theta_mid) * y) + axis_norm * 0.5 * (min_proj + max_proj);

    // 转换为 mm
    Eigen::Vector3f center_mm       = center * 1000.0;
    Eigen::Vector3f spray_center_mm = spray_center * 1000.0;
    // Eigen::Vector3f axis_dir_mm     = axis_norm * 1000.0; // 如果需要用长度表示

    double radius_mm = radius * 1000.0;

    // 输出结果
    std::cout << "\n=========== 圆柱拟合结果 ===========" << std::endl;
    std::cout << "圆柱中心: [" << center_mm.x() << ", " << center_mm.y() << ", " << center_mm.z() << "]" << std::endl;
    std::cout << "喷涂区域中心:  [" << spray_center_mm.x() << ", " << spray_center_mm.y() << ", " << spray_center_mm.z()
              << "]" << std::endl;
    std::cout << "圆柱轴方向: [" << axis_norm.x() << ", " << axis_norm.y() << ", " << axis_norm.z() << "]" << std::endl;
    std::cout << "半径: " << radius_mm << " mm" << std::endl;
    std::cout << "角度: " << angle_deg << " deg" << std::endl;
    std::cout << "高度: " << height << " mm" << std::endl;
    std::cout << "======================================================================" << std::endl;
}
void CylinderFitter::viewerThread() {
    while (running_) {
        {
            std::lock_guard<std::mutex> lock(viewer_mutex_);
            viewer_->spinOnce(30);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}
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

// int main(int argc, char **argv) {
//     std::string    path = "/home/oneko/projects/lidar/ros_airy_work/src/pcd_datas/2.pcd";
//     CylinderFitter app(path);
//     return app.run();
// }
