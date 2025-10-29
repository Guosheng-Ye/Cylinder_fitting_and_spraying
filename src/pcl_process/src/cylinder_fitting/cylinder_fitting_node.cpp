#include "cylinder_fitting_node.hpp"

CylinderFitter::CylinderFitter(const std::string &pcd_path)
    : pcd_path_(pcd_path)
    , viewer_(new pcl::visualization::PCLVisualizer("Cylinder Fitting Viewer")) {
    viewer_->setBackgroundColor(0, 0, 0);
    viewer_->addCoordinateSystem(1.0);
    viewer_->initCameraParameters();

    viewer_thread_ = std::thread(&CylinderFitter::viewerThread, this);
}
CylinderFitter::~CylinderFitter() {
    running_ = false;
    if (viewer_thread_.joinable()) viewer_thread_.join();
}
/// @brief PCD 数据文件进行圆柱拟合
/// @return
int CylinderFitter::run_with_pcd() {
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path_, *cloud_) == -1) {
        PCL_ERROR("Couldn't read file %s\n", pcd_path_.c_str());
        return -1;
    }
    ROS_INFO("Loaded %zu points from %s", cloud_->size(), pcd_path_.c_str());

    // 角度过滤
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
    ROS_INFO("After angle filtering: %zu points remain.", cloud_filtered_angle->size());
    cloud_ = cloud_filtered_angle;

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    // 法线估计
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    estimateNormals(cloud_, normals);

    // 圆柱拟合
    fitCylinderPCL(cloud_, normals);

    if (!hasModel()) {
        ROS_WARN("No valid cylinder model found; skipping slicing.");
        return -1;
    }

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_span  = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    ROS_INFO("Cylinder fitting time: %f seconds.", time_span.count());

    // 可视化
    visualizeResults();

    // 计算圆柱高度并调用切片
    if (hasModel()) {
        std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
        sliceCylinder(180.0, 500.0, 300.0);
        std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();
        std::chrono::duration<double> slice_time = std::chrono::duration_cast<std::chrono::duration<double>>(t4 - t3);
        ROS_INFO("Slice time: %f seconds.", slice_time.count());
    }

    /// 打印数值结果
    printResults();
    return 0;
}

/// @brief RSLidar 点云进行圆柱拟合
/// @param cloud_input
/// @return
int CylinderFitter::run_with_rslidar(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input) {
    if (!cloud_input || cloud_input->empty()) {
        ROS_ERROR("Empty point cloud input.");
        return -1;
    }

    cloud_ = cloud_input;

    // 法线估计
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    estimateNormals(cloud_, normals);

    // 圆柱拟合
    fitCylinderPCL(cloud_, normals);

    if (!hasModel()) {
        ROS_WARN("No valid cylinder model found; skipping slicing.");
        return -1;
    }
    // 可视化
    visualizeResults();

    // 计算圆柱高度并调用切片
    if (hasModel()) {
        std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
        sliceCylinder(180.0, 500.0, 300.0);
        std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();
        std::chrono::duration<double> slice_time = std::chrono::duration_cast<std::chrono::duration<double>>(t4 - t3);
        ROS_WARN("Slice time: %f seconds.", slice_time.count());
        // 打印结果
        printResults();
        return 1;
    } else {
        return -1;
    }
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

/// @brief 标准圆柱拟合
/// @param cloud 点云
/// @param normals 法线
void CylinderFitter::fitCylinderPCL(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                    pcl::PointCloud<pcl::Normal>::Ptr   normals) {

    // init
    cylinder_model_.valid = false;
    cylinder_model_.inliers->clear();
    const auto &fp = fitting_params_;

    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(0.1);
    seg.setMaxIterations(20000);
    seg.setDistanceThreshold(0.02);
    seg.setRadiusLimits(fp.min_radius_m, fp.max_radius_m); // 半径范围
    seg.setInputCloud(cloud);
    seg.setInputNormals(normals);

    // 强制轴线接近垂直
    seg.setAxis(Eigen::Vector3f(0, 0, 1));
    seg.setEpsAngle(10.0f * M_PI / 180.0f); // 最大允许偏差10°

    pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr      inliers_cylinder(new pcl::PointIndices);
    seg.segment(*inliers_cylinder, *coefficients_cylinder);

    if (inliers_cylinder->indices.empty()) {
        ROS_WARN("No cylinder found.");
        cylinder_model_.valid = false;
        return;
    }

    // 数量与比例
    size_t inlier_count = inliers_cylinder->indices.size();
    double inlier_ratio = (double)inlier_count / (double)cloud->size();

    // 提取内点
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers_cylinder);
    extract.setNegative(false);
    extract.filter(*cylinder_model_.inliers);

    // 半径优化
    // 将内点投影到 XY 平面（垂直截面），计算最大外接圆半径
    Eigen::Vector3f fitted_axis(coefficients_cylinder->values[3], coefficients_cylinder->values[4],
                                coefficients_cylinder->values[5]);
    fitted_axis.normalize();
    Eigen::Vector3f proj_plane_normal = fitted_axis; // 投影平面法向量
    Eigen::Vector3f proj_center(coefficients_cylinder->values[0], coefficients_cylinder->values[1],
                                coefficients_cylinder->values[2]);

    double              max_r = 0.0;
    double              sum_r = 0.0;
    std::vector<double> radii;
    radii.reserve(cylinder_model_.inliers->size());

    for (const auto &p : cylinder_model_.inliers->points) {
        Eigen::Vector3f pt(p.x, p.y, p.z);
        // 投影到截面（垂直轴向）
        Eigen::Vector3f v      = pt - proj_center;
        Eigen::Vector3f v_proj = v - (v.dot(fitted_axis)) * fitted_axis;
        double          r      = v_proj.norm();
        radii.push_back(r);
        sum_r += r;
        if (r > max_r) max_r = r;
    }

    double mean_r = sum_r / (double)radii.size();
    double var_r  = 0.0;
    for (const auto &r : radii) {
        var_r += (r - mean_r) * (r - mean_r);
    }
    var_r /= (double)radii.size();
    double std_r          = std::sqrt(var_r);
    double radial_rel_std = std_r / mean_r;

    // 计算高度范围
    Eigen::Vector3f axis_norm = fitted_axis.normalized();
    double          min_proj  = std::numeric_limits<double>::max();
    double          max_proj  = std::numeric_limits<double>::lowest();
    for (const auto &p : cylinder_model_.inliers->points) {
        Eigen::Vector3f pt(p.x, p.y, p.z);
        double          proj = (pt - proj_center).dot(axis_norm);
        min_proj             = std::min(min_proj, proj);
        max_proj             = std::max(max_proj, proj);
    }
    double cylinder_height = max_proj - min_proj;

    // 模型数据判断
    ROS_INFO("Inlier count: %zu[%d-%d], ratio: %.4f[%.4f,%.4f], height: %.2f m[%.2f,~], radius: %.3f m[%.3f,%.3f], "
             "radial_rel_std: %.4f[%.4f,%.4f]",
             inlier_count, fp.min_inliers, fp.max_inliers, inlier_ratio, fp.min_inlier_ratio, fp.max_inlier_ratio,
             cylinder_height, fp.min_height_m, mean_r, fp.min_radius_m, fp.max_radius_m, radial_rel_std,
             fp.min_radial_rel_std, fp.max_radial_rel_std);

    if (inlier_count < (size_t)fp.min_inliers || inlier_count > (size_t)fp.max_inliers ||
        inlier_ratio < fp.min_inlier_ratio || inlier_ratio > fp.max_inlier_ratio || cylinder_height < fp.min_height_m ||
        mean_r < fp.min_radius_m || mean_r > fp.max_radius_m || radial_rel_std < fp.min_radial_rel_std ||
        radial_rel_std > fp.max_radial_rel_std) {
        ROS_ERROR("Fitted cylinder model did not meet quality criteria.");
        cylinder_model_.inliers->clear();
        return;
    }
    cylinder_model_.valid         = true;
    cylinder_model_.radius        = mean_r;
    cylinder_model_.bottom_center = proj_center + min_proj * axis_norm;
    cylinder_model_.top_center    = proj_center + max_proj * axis_norm;
    cylinder_model_.geo_center    = 0.5f * (cylinder_model_.bottom_center + cylinder_model_.top_center);
    cylinder_model_.axis          = axis_norm;       // 保存归一化轴向
    cylinder_model_.height        = cylinder_height; // 米

    // 强制轴向朝向 +Z
    if (cylinder_model_.axis.z() < 0.0f) {
        cylinder_model_.axis = -cylinder_model_.axis;
        std::swap(cylinder_model_.bottom_center, cylinder_model_.top_center);
        cylinder_model_.geo_center = 0.5f * (cylinder_model_.bottom_center + cylinder_model_.top_center);
    }
    cylinder_model_.valid = true;

    std::cout << "圆柱检测（优化半径）:\n"
              << "  底面中心: [" << cylinder_model_.bottom_center.transpose() << "]\n"
              << "  几何中心: [" << cylinder_model_.geo_center.transpose() << "]\n"
              << "  顶面中心: [" << cylinder_model_.top_center.transpose() << "]\n"
              << "  轴:   [" << cylinder_model_.axis.transpose() << "]\n"
              << "  半径: " << cylinder_model_.radius << "\n"
              << "  高度: " << cylinder_model_.height << "\n"
              << "  内点数: " << inliers_cylinder->indices.size() << " points\n";
}

/// @brief 对标准圆柱表面切片计算
/// @param angle_range_deg 角度范围（度）
/// @param slice_height_mm 切片高度（毫米）
/// @param slice_width_mm 切片宽度（毫米）
void CylinderFitter::sliceCylinder(double angle_range_deg, double slice_height_mm, double slice_width_mm) {
    if (!hasModel()) return;

    const auto &opt = slice_opts_;

    Eigen::Vector3f bottom_center_mm = cylinder_model_.bottom_center * 1000.0f;
    Eigen::Vector3f axis             = cylinder_model_.axis;
    Eigen::Vector3f z                = axis;
    double          radius_mm        = cylinder_model_.radius * 1000.0;
    double          height_mm        = cylinder_model_.height * 1000.0;

    Eigen::Vector3f ref(-1.0f, 0.0f, 0.0f);
    if (std::abs(ref.dot(z)) > 0.95f) ref = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
    Eigen::Vector3f x = (ref - z * ref.dot(z)).normalized();
    Eigen::Vector3f y = z.cross(x).normalized();

    double angle_range = angle_range_deg * M_PI / 180.0;
    double min_theta   = -angle_range / 2.0;

    int num_height_slices = std::max(1, (int)std::round(height_mm / slice_height_mm));
    int num_width_slices  = std::max(1, (int)std::round((radius_mm * angle_range) / slice_width_mm));

    {
        std::lock_guard<std::mutex> lock(viewer_mutex_);
        for (auto &id : slice_shape_ids_)
            viewer_->removeShape(id);
        for (auto &id : slice_text_ids_)
            viewer_->removeShape(id);
        for (auto &id : slice_line_ids_)
            viewer_->removeShape(id);
        slice_shape_ids_.clear();
        slice_text_ids_.clear();
        slice_line_ids_.clear();
        slice_centers_mm_.clear();
        slices_data_.clear();
    }

    int slice_count = 0;
    for (int i = 0; i < num_height_slices; ++i) {
        double h_mm = (i + 0.5) * (height_mm / num_height_slices);

        std::vector<Eigen::Vector3f> layer_centers_mm; // 收集该高度层的切片中心

        for (int j = 0; j < num_width_slices; ++j) {
            double theta = min_theta + (j + 0.5) * (angle_range / num_width_slices);

            Eigen::Vector3f axial_pos_mm    = bottom_center_mm + h_mm * z;
            Eigen::Vector3f ring_offset_mm  = radius_mm * (std::cos(theta) * x + std::sin(theta) * y);
            Eigen::Vector3f slice_center_mm = axial_pos_mm + ring_offset_mm;

            // 计算弧片范围
            double dtheta = angle_range / num_width_slices;
            double dh     = height_mm / num_height_slices;
            double theta0 = theta - 0.5 * dtheta;
            double theta1 = theta + 0.5 * dtheta;
            double h0     = h_mm - 0.5 * dh;
            double h1     = h_mm + 0.5 * dh;

            // 采样分辨率
            int theta_samples  = 10;
            int height_samples = 5;
            if (theta_samples < 2) theta_samples = 2;
            if (height_samples < 2) height_samples = 2;

            pcl::PointCloud<pcl::PointXYZ> patch_cloud;
            std::vector<pcl::Vertices>     polygons;

            if (opt.surface_) { // true 生成弧形面网格
                patch_cloud.reserve(theta_samples * height_samples);
                for (int hi = 0; hi < height_samples; ++hi) {
                    double          hh_mm = h0 + (h1 - h0) * (double)hi / (height_samples - 1);
                    Eigen::Vector3f base  = bottom_center_mm + hh_mm * z;
                    for (int ti = 0; ti < theta_samples; ++ti) {
                        double          th    = theta0 + (theta1 - theta0) * (double)ti / (theta_samples - 1);
                        Eigen::Vector3f pt_mm = base + radius_mm * (std::cos(th) * x + std::sin(th) * y);
                        patch_cloud.emplace_back(pt_mm.x() / 1000.0f, pt_mm.y() / 1000.0f, pt_mm.z() / 1000.0f);
                    }
                }
                for (int hi = 0; hi < height_samples - 1; ++hi) {
                    for (int ti = 0; ti < theta_samples - 1; ++ti) {
                        int           idx0 = hi * theta_samples + ti;
                        int           idx1 = idx0 + 1;
                        int           idx2 = (hi + 1) * theta_samples + ti;
                        int           idx3 = idx2 + 1;
                        pcl::Vertices v1;
                        v1.vertices = {(uint32_t)idx0, (uint32_t)idx1, (uint32_t)idx2};
                        pcl::Vertices v2;
                        v2.vertices = {(uint32_t)idx1, (uint32_t)idx3, (uint32_t)idx2};
                        polygons.push_back(v1);
                        polygons.push_back(v2);
                    }
                }
            }

            std::string sphere_id = "slice_center_" + std::to_string(i) + "_" + std::to_string(j);
            std::string text_id   = "slice_txt_" + std::to_string(i) + "_" + std::to_string(j);
            std::string patch_id  = "slice_patch_" + std::to_string(i) + "_" + std::to_string(j);

            // 统一加锁添加
            {
                std::lock_guard<std::mutex> lock(viewer_mutex_);

                // 球
                viewer_->addSphere(pcl::PointXYZ(slice_center_mm.x() / 1000.0f, slice_center_mm.y() / 1000.0f,
                                                 slice_center_mm.z() / 1000.0f),
                                   5.0 / 1000.0, 1.0, 0.5, 0.0, sphere_id);
                slice_shape_ids_.push_back(sphere_id);

                // 文本
                // std::ostringstream oss;
                // oss << i << "," << j;
                // viewer_->addText3D(oss.str(),
                //                    pcl::PointXYZ(slice_center_mm.x() / 1000.0f, slice_center_mm.y() / 1000.0f,
                //                                  slice_center_mm.z() / 1000.0f),
                //                    0.05, 1.0, 1.0, 1.0, text_id);
                // slice_text_ids_.push_back(text_id);

                // 可选弧形面
                if (opt.surface_ && !patch_cloud.empty() && !polygons.empty()) {
                    pcl::PolygonMesh mesh;
                    pcl::toPCLPointCloud2(patch_cloud, mesh.cloud);
                    mesh.polygons = polygons;
                    if (viewer_->addPolygonMesh(mesh, patch_id)) {
                        double r_color = (double)i / num_height_slices;
                        double g_color = (double)j / num_width_slices;
                        double b_color = 0.5 + 0.3 * g_color;
                        viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r_color, g_color,
                                                             b_color, patch_id);
                        viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.6, patch_id);
                        slice_shape_ids_.push_back(patch_id);
                    }
                }

                // 边框线：采样用于弧线 & 竖线
                if (opt.border_) {
                    auto addLineSeg = [&](const Eigen::Vector3f &p0_mm, const Eigen::Vector3f &p1_mm, double r,
                                          double g, double b, const std::string &id) {
                        viewer_->addLine(pcl::PointXYZ(p0_mm.x() / 1000.0f, p0_mm.y() / 1000.0f, p0_mm.z() / 1000.0f),
                                         pcl::PointXYZ(p1_mm.x() / 1000.0f, p1_mm.y() / 1000.0f, p1_mm.z() / 1000.0f),
                                         r, g, b, id);
                        slice_line_ids_.push_back(id);
                    };

                    int arc_samples = std::max(8, theta_samples); // 底/顶弧细分
                    // 底弧
                    for (int k = 0; k < arc_samples - 1; ++k) {
                        double          t0 = theta0 + (theta1 - theta0) * (double)k / (arc_samples - 1);
                        double          t1 = theta0 + (theta1 - theta0) * (double)(k + 1) / (arc_samples - 1);
                        Eigen::Vector3f p0 =
                            bottom_center_mm + h0 * z + radius_mm * (std::cos(t0) * x + std::sin(t0) * y);
                        Eigen::Vector3f p1 =
                            bottom_center_mm + h0 * z + radius_mm * (std::cos(t1) * x + std::sin(t1) * y);
                        addLineSeg(p0, p1, 1.0, 1.0, 0.0,
                                   "slice_line_bottom_" + std::to_string(i) + "_" + std::to_string(j) + "_" +
                                       std::to_string(k));
                    }
                    // 顶弧
                    for (int k = 0; k < arc_samples - 1; ++k) {
                        double          t0 = theta0 + (theta1 - theta0) * (double)k / (arc_samples - 1);
                        double          t1 = theta0 + (theta1 - theta0) * (double)(k + 1) / (arc_samples - 1);
                        Eigen::Vector3f p0 =
                            bottom_center_mm + h1 * z + radius_mm * (std::cos(t0) * x + std::sin(t0) * y);
                        Eigen::Vector3f p1 =
                            bottom_center_mm + h1 * z + radius_mm * (std::cos(t1) * x + std::sin(t1) * y);
                        addLineSeg(p0, p1, 1.0, 1.0, 0.0,
                                   "slice_line_top_" + std::to_string(i) + "_" + std::to_string(j) + "_" +
                                       std::to_string(k));
                    }
                    // 竖直边(左 / 右)
                    for (int k = 0; k < height_samples - 1; ++k) {
                        double          thL = theta0;
                        double          thR = theta1;
                        double          hh0 = h0 + (h1 - h0) * (double)k / (height_samples - 1);
                        double          hh1 = h0 + (h1 - h0) * (double)(k + 1) / (height_samples - 1);
                        Eigen::Vector3f vl0 =
                            bottom_center_mm + hh0 * z + radius_mm * (std::cos(thL) * x + std::sin(thL) * y);
                        Eigen::Vector3f vl1 =
                            bottom_center_mm + hh1 * z + radius_mm * (std::cos(thL) * x + std::sin(thL) * y);
                        Eigen::Vector3f vr0 =
                            bottom_center_mm + hh0 * z + radius_mm * (std::cos(thR) * x + std::sin(thR) * y);
                        Eigen::Vector3f vr1 =
                            bottom_center_mm + hh1 * z + radius_mm * (std::cos(thR) * x + std::sin(thR) * y);
                        addLineSeg(vl0, vl1, 0.0, 0.5, 1.0,
                                   "slice_line_left_" + std::to_string(i) + "_" + std::to_string(j) + "_" +
                                       std::to_string(k));
                        addLineSeg(vr0, vr1, 0.0, 0.5, 1.0,
                                   "slice_line_right_" + std::to_string(i) + "_" + std::to_string(j) + "_" +
                                       std::to_string(k));
                    }
                }
            }

            layer_centers_mm.push_back(slice_center_mm);
            slice_centers_mm_.push_back(slice_center_mm);

            //构建 SliceData
            SliceData sd;
            sd.layer_index   = i;
            sd.angular_index = j;
            // sd.id               = std::to_string(i) + "_" + std::to_string(j);
            sd.id               = slice_count++;
            sd.center_mm        = slice_center_mm; //切片表面中心
            sd.circle_center_mm = axial_pos_mm;    //轴线上该高度层中心
            slices_data_.push_back(std::move(sd));
        }

        // 绘制该高度层的环线
        if (opt.height_rings_ && layer_centers_mm.size() > 1) {
            std::lock_guard<std::mutex> lock(viewer_mutex_);
            int                         ring_samples = 96;
            for (int k = 0; k < ring_samples; ++k) {
                double          th0 = 2.0 * M_PI * k / ring_samples;
                double          th1 = 2.0 * M_PI * (k + 1) / ring_samples;
                Eigen::Vector3f p0_mm =
                    bottom_center_mm + h_mm * z + radius_mm * (std::cos(th0) * x + std::sin(th0) * y);
                Eigen::Vector3f p1_mm =
                    bottom_center_mm + h_mm * z + radius_mm * (std::cos(th1) * x + std::sin(th1) * y);
                std::string ring_id = "height_full_ring_" + std::to_string(i) + "_" + std::to_string(k);
                viewer_->addLine(pcl::PointXYZ(p0_mm.x() / 1000.0f, p0_mm.y() / 1000.0f, p0_mm.z() / 1000.0f),
                                 pcl::PointXYZ(p1_mm.x() / 1000.0f, p1_mm.y() / 1000.0f, p1_mm.z() / 1000.0f), 1.0, 0.2,
                                 0.8, ring_id); // 洋红色
                slice_line_ids_.push_back(ring_id);
            }
        }
    }

    ROS_WARN("[Slice] Generate Slice:Height: (%f,%f) mm, height_total=%d, width_total=%d, surface=%s", slice_height_mm,
             slice_width_mm, num_height_slices, num_width_slices, (opt.surface_ ? "ON" : "OFF"));

    // 打印所有切片中心
    // if (opt.print_centers_) {
    //     ROS_INFO("[Slice Centers] Total=%zu", slice_centers_mm_.size());
    //     for (size_t k = 0; k < slice_centers_mm_.size(); ++k) {
    //         int         i = static_cast<int>(k / num_width_slices);
    //         int         j = static_cast<int>(k % num_width_slices);
    //         const auto &c = slice_centers_mm_[k];
    //         ROS_INFO("  Slice (%d,%d): [%.2f mm, %.2f mm, %.2f mm]", i, j, c.x(), c.y(), c.z());
    //     }
    // }
}

/// @brief 可视化结果
void CylinderFitter::visualizeResults() {
    if (!hasModel()) return;
    auto   cylinder_points = cylinder_model_.inliers;
    auto   center          = cylinder_model_.bottom_center;
    auto   axis            = cylinder_model_.axis;
    double radius          = cylinder_model_.radius;

    std::lock_guard<std::mutex> lock(viewer_mutex_);

    viewer_->removeAllPointClouds();
    viewer_->removeAllShapes();
    // 白色原始点云
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> white(cloud_, 255, 255, 255);
    viewer_->addPointCloud<pcl::PointXYZ>(cloud_, white, "cloud");

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
    int                                 n_circ   = 20; // 每层点数
    int                                 n_height = 25; // 高度层数

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

    // 底面圆环与顶面圆环
    pcl::PointCloud<pcl::PointXYZ>::Ptr bottom_ring(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr top_ring(new pcl::PointCloud<pcl::PointXYZ>);
    int                                 ring_pts      = 72;
    Eigen::Vector3f                     bottom_center = cylinder_model_.bottom_center;
    Eigen::Vector3f                     top_center    = cylinder_model_.top_center;
    for (int i = 0; i < ring_pts; ++i) {
        double          theta = 2.0 * M_PI * i / ring_pts;
        Eigen::Vector3f p_b   = bottom_center + radius * (std::cos(theta) * x + std::sin(theta) * y);
        Eigen::Vector3f p_t   = top_center + radius * (std::cos(theta) * x + std::sin(theta) * y);
        bottom_ring->push_back(pcl::PointXYZ(p_b.x(), p_b.y(), p_b.z()));
        top_ring->push_back(pcl::PointXYZ(p_t.x(), p_t.y(), p_t.z()));
    }
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> bottom_color(bottom_ring, 255, 255, 0); // 黄
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> top_color(top_ring, 0, 255, 255);       // 青
    viewer_->addPointCloud<pcl::PointXYZ>(bottom_ring, bottom_color, "bottom_ring");
    viewer_->addPointCloud<pcl::PointXYZ>(top_ring, top_color, "top_ring");
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "bottom_ring");
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "top_ring");

    // 调试：底面与顶面中心球体
    viewer_->addSphere(pcl::PointXYZ(cylinder_model_.bottom_center.x(), cylinder_model_.bottom_center.y(),
                                     cylinder_model_.bottom_center.z()),
                       0.03, 1.0, 1.0, 0.0, "bottom_center_sphere");
    viewer_->addSphere(
        pcl::PointXYZ(cylinder_model_.top_center.x(), cylinder_model_.top_center.y(), cylinder_model_.top_center.z()),
        0.03, 0.0, 1.0, 1.0, "top_center_sphere");

    // 圆柱轴线
    Eigen::Vector3f start = center + min_proj * axis_norm;
    Eigen::Vector3f end   = center + max_proj * axis_norm;
    viewer_->addArrow(pcl::PointXYZ(end.x(), end.y(), end.z()), pcl::PointXYZ(start.x(), start.y(), start.z()), 1.0,
                      0.0, 0.0, false, "cylinder_axis");

    // 源点到圆柱中点连线
    Eigen::Vector3f source_point(0, 0, 0); // 假设原点为传感器位置
    viewer_->addLine(
        pcl::PointXYZ(source_point.x(), source_point.y(), source_point.z()),
        pcl::PointXYZ(cylinder_model_.geo_center.x(), cylinder_model_.geo_center.y(), cylinder_model_.geo_center.z()),
        1.0, 0.0, 1.0, "source_to_cylinder_mid"); // 紫色

    // 源点到圆柱底面中心连线
    viewer_->addLine(pcl::PointXYZ(source_point.x(), source_point.y(), source_point.z()),
                     pcl::PointXYZ(cylinder_model_.bottom_center.x(), cylinder_model_.bottom_center.y(),
                                   cylinder_model_.bottom_center.z()),
                     1.0, 1.0, 0.0, "source_to_cylinder_bottom"); // 黄色

    // 源点到圆柱顶面中心连线
    viewer_->addLine(
        pcl::PointXYZ(source_point.x(), source_point.y(), source_point.z()),
        pcl::PointXYZ(cylinder_model_.top_center.x(), cylinder_model_.top_center.y(), cylinder_model_.top_center.z()),
        0.0, 1.0, 1.0, "source_to_cylinder_top"); // 青色

    // 计算距离
    double dist_3d = (cylinder_model_.geo_center - source_point).norm() * 1000.0; // 米 -> mm
    double dist_xy = (Eigen::Vector3f(cylinder_model_.geo_center.x(), cylinder_model_.geo_center.y(), 0.0f) -
                      Eigen::Vector3f(source_point.x(), source_point.y(), 0.0f))
                         .norm() *
                     1000.0; // 米 -> mm
    ROS_INFO("[Distance] Geo center: %f mm, Bottom center: %f mm", dist_3d, dist_xy);
}

/// @brief 打印结果
void CylinderFitter::printResults() {
    if (!hasModel()) {
        ROS_ERROR(" No cylinder model available.");
        return;
    }
    auto   cylinder_points = cylinder_model_.inliers;
    auto   center          = cylinder_model_.bottom_center;
    auto   axis            = cylinder_model_.axis;
    double radius          = cylinder_model_.radius;
    if (!cylinder_points || cylinder_points->empty()) {
        ROS_ERROR(" No cylinder points for result computation.");
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

    // 输出结果
    ROS_INFO("Cylinder");
    ROS_INFO("Bottom center: [%.2f, %.2f, %.2f]", cylinder_model_.bottom_center.x() * 1000.0,
             cylinder_model_.bottom_center.y() * 1000.0, cylinder_model_.bottom_center.z() * 1000.0);
    ROS_INFO("Axis: [%.2f, %.2f, %.2f]", axis_norm.x(), axis_norm.y(), axis_norm.z());
    ROS_INFO("Radius: %.2f mm", cylinder_model_.radius * 1000.0);
    ROS_INFO("Angle: %.2f deg", cylinder_model_.valid ? angle_deg : 0.0);
    ROS_INFO("Height: %.2f mm", cylinder_model_.height * 1000.0);

    // const auto &all = slices_data_;
    // for (const auto &s : all) {
    //     std::cout << "Slice " << s.id << " center=[" << s.center_mm.x() << ", " << s.center_mm.y() << ", "
    //               << s.center_mm.z() << "] axis_center=[" << s.circle_center_mm.x() << ", " <<
    //               s.circle_center_mm.y()
    //               << ", " << s.circle_center_mm.z() << "] axis: [" << cylinder_model_.axis.transpose()
    //               << "], radius: " << cylinder_model_.radius * 1000.0 << " mm\n";
    // }

    // ROS_WARN(" Cylinder fitting completed.");
}

/// @brief 视图线程
void CylinderFitter::viewerThread() {
    while (running_) {
        {
            std::lock_guard<std::mutex> lock(viewer_mutex_);
            viewer_->spinOnce(30);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

// 移除水平地面和平面
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
