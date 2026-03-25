#include "cylinder_fitting_node.hpp"
#include <ctime>
#include <pcl/io/pcd_io.h>
#include <ros/package.h>
#include <sys/stat.h>

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
    const auto &fp = fitting_params_;
    ROS_WARN("Fitting params: min_inliers=%d, max_inliers=%d, min_inlier_ratio=%.4f, max_inlier_ratio=%.4f, "
             "min_height_m=%.2f, min_radius_m=%.2f, max_radius_m=%.2f, min_radial_rel_std=%.4f, max_radial_rel_std=%.4f"
             "sparay_distance:%.2f, work_surface:%.2f,height=%.2f, step_weight=%.2f, r_initial:%.2f, robot_R_arm:%.2f, "
             "work_mode:%d, k:%d",
             fp.min_inliers, fp.max_inliers, fp.min_inlier_ratio, fp.max_inlier_ratio, fp.min_height_m, fp.min_radius_m,
             fp.max_radius_m, fp.min_radial_rel_std, fp.max_radial_rel_std, fp.slice_distance_,
             fp.virtual_base_to_surface_, fp.height_, fp.step_width_, fp.r_initial_, fp.robot_R_arm_, fp.work_mode_,
             fp.k_);

    fitCylinderPCL(cloud_, normals);

    if (!hasModel()) {
        ROS_WARN("No valid cylinder model found; skipping slicing.");
        return -1;
    }
    // 可视化
    visualizeResults();

    // 计算圆柱高度并调用切片
    if (hasModel()) {
        ROS_ERROR("[MAX SLICE ANGLE] Cal max slice angle...");
        // 自动作业，动态计算optimized_slice_width_mm
        if (fp.work_mode_ == 2) {
            ROS_INFO("Automatic mode: calculating optimal slice width...");
            PlanningInputs inputs;
            // 获取圆柱参数（雷达坐标系mm）
            inputs.r_cyl  = cylinder_model_.radius * 1000.0;
            inputs.V_axis = {cylinder_model_.axis.x(), cylinder_model_.axis.y(), cylinder_model_.axis.z() * (-1.0)};

            // 使用虚拟机械臂原点方案计算 P_cyl
            // z_center_mm: 工作高度中心,固定值400mm作为工作高度中心
            double          z_center_mm = 400.0; // TODO: 可以根据实际工作高度范围计算
            Eigen::Vector3d P_cyl_eigen =
                calculateVirtualBaseP_cyl(cylinder_model_.geo_center, inputs.r_cyl, fp.slice_distance_, z_center_mm);
            inputs.P_cyl = {P_cyl_eigen.x(), P_cyl_eigen.y(), z_center_mm}; //  P_cyl_eigen.z()};

            // 计算虚拟原点位置（用于可视化）
            Eigen::Vector3d geo_center_mm = cylinder_model_.geo_center.cast<double>() * 1000.0;
            double virtual_base_dist = inputs.r_cyl + fp.virtual_base_to_surface_; // 虚拟基座到轴线的距离
            Eigen::Vector3d geo_center_xy(geo_center_mm.x(), geo_center_mm.y(), 0.0);
            Eigen::Vector3d virtual_base_pos_mm;
            if (geo_center_xy.norm() > 1e-6) {
                Eigen::Vector3d dir_xy  = -geo_center_xy.normalized();
                virtual_base_pos_mm     = geo_center_mm + dir_xy * virtual_base_dist;
                virtual_base_pos_mm.z() = 0.0;
            } else {
                virtual_base_pos_mm = Eigen::Vector3d(virtual_base_dist, 0.0, 0.0);
            }

            ROS_ERROR("Fitted cylinder - P_cyl (relative to virtual base): [%.2f, %.2f, %.2f] mm, "
                      "axis direction: [%.4f, %.4f, %.4f], radius: %.2f mm",
                      inputs.P_cyl[0], inputs.P_cyl[1], inputs.P_cyl[2], inputs.V_axis[0], inputs.V_axis[1],
                      inputs.V_axis[2], inputs.r_cyl);

            // 机械臂工作区域(入参)
            inputs.robot_R_arm = fp.robot_R_arm_;    // 机械臂最大臂长 R_arm
            inputs.D_offset    = fp.slice_distance_; // 喷涂距离 D

            // 工作高度范围 H_region（800）
            inputs.H_region = fp.height_; // fp.slice_height_; // 总工作高度 H

            // 喷涂工艺参数
            inputs.d_spray   = fp.step_width_; // 喷幅 300
            inputs.r_initial = fp.r_initial_;  // 初始搭接比例 50%
            inputs.h_slice   = fp.height_;     // fp.slice_height_; // 切片高度

            // 计算输出
            PlanningOutputs outputs = execute_slice_planning(inputs);
            ROS_ERROR("PlanningOutputs Calculated max coverage angle: %.2f deg, optimal slice angle: %.2f deg, final "
                      "overlap ratio: %.2f, "
                      "status: %s",
                      outputs.max_coverage_angle, outputs.optimal_slice_angle, outputs.final_overlap_ratio,
                      outputs.status_message.c_str());

            // 更新cylinder_fitter_outputs
            // 不变量
            cylinder_fitter_outputs.slice_distance_ = fp.slice_distance_;
            cylinder_fitter_outputs.height_         = fp.height_;
            cylinder_fitter_outputs.step_width_     = fp.step_width_;
            cylinder_fitter_outputs.step_height_    = fp.step_height_;
            cylinder_fitter_outputs.robot_R_arm_    = fp.robot_R_arm_;
            cylinder_fitter_outputs.work_mode_      = fp.work_mode_;
            // 变化量（从优化算法直接获取）
            // 回传  ：1. k 每个位置的喷涂次数
            //        2. w0 调整后的有效单次喷幅 (mm)
            //        3. r 更新后的搭接比例
            cylinder_fitter_outputs.r_initial_ = outputs.final_overlap_ratio;
            cylinder_fitter_outputs.w0_        = outputs.w0_adjusted_deg;
            cylinder_fitter_outputs.k_         = outputs.k_spray_passes;

            // ========== 自动模式回传参数日志打印 ==========
            ROS_WARN("========== AUTO MODE OUTPUT PARAMETERS ==========");
            ROS_WARN("[callback] update r_initial r: %.4f (%.2f%%) <- origin: %.4f (%.2f%%)",
                     outputs.final_overlap_ratio, outputs.final_overlap_ratio * 100.0, fp.r_initial_,
                     fp.r_initial_ * 100.0);
            ROS_WARN("[callback] update w0: %.4f deg (%.2f mm) <- origin: %.2f mm", outputs.w0_adjusted_deg,
                     outputs.w0_adjusted_mm, fp.step_width_ * (1.0 - fp.r_initial_));
            ROS_WARN("[callback] update k in once move: %d", outputs.k_spray_passes);
            ROS_WARN("[callback] update N move nums: %d", outputs.n_positions);
            ROS_WARN("[callback] update slice angle: %.4f deg", outputs.optimal_slice_angle);
            ROS_WARN("[callback] update total coverage angle: %.4f deg (= %.4f * %d)",
                     outputs.optimal_slice_angle * outputs.n_positions, outputs.optimal_slice_angle,
                     outputs.n_positions);
            ROS_WARN("=================================================");

            // ====== 可视化覆盖范围 ======
            visualizeCoverageRange(virtual_base_pos_mm,
                                   inputs.robot_R_arm,         // 机械臂最大臂长
                                   outputs.max_coverage_angle, // 最大覆盖角度
                                   z_center_mm,                // 工作高度中心
                                   inputs.H_region);           // 工作高度范围

            // 将优化后的切片角度转换为切片宽度（毫米）
            // slice_width_mm = radius_mm * angle_rad = radius_mm * (angle_deg * π/180)
            double radius_mm                = cylinder_model_.radius * 1000.0;
            double optimized_slice_width_mm = radius_mm * (outputs.optimal_slice_angle * M_PI / 180.0);

            // ====== 回退策略：优化失败时使用固定切片宽度计算 ======
            if (outputs.optimal_slice_angle <= 0.0 || optimized_slice_width_mm <= 0.0) {
                ROS_WARN(
                    "========== AUTO MODE FALLBACK: Optimization failed, using fixed slice calculation ==========");
                ROS_WARN("[fallback] Optimization failed reason: %s", outputs.status_message.c_str());

                // 使用初始 k=1 调用手动模式计算函数
                int    fallback_k           = 1; // 自动模式回退时初始k设为1
                int    final_k              = 0;
                double final_r              = 0.0;
                double final_w0_mm          = 0.0;
                double final_slice_width_mm = 0.0;

                int N_slices = calculate_manual_slice_params(fp.step_width_, fallback_k, fp.r_initial_, radius_mm,
                                                             final_k, final_r, final_w0_mm, final_slice_width_mm);

                // 计算角度用于日志
                double final_w0_deg          = (final_w0_mm / radius_mm) * (180.0 / M_PI);
                double final_slice_angle_deg = (final_slice_width_mm / radius_mm) * (180.0 / M_PI);

                // 更新 outputs 以反映回退计算结果（用于后续日志输出）
                outputs.optimal_slice_angle = final_slice_angle_deg;
                outputs.final_overlap_ratio = final_r;
                outputs.k_spray_passes      = final_k;
                outputs.w0_adjusted_mm      = final_w0_mm;
                outputs.w0_adjusted_deg     = final_w0_deg;
                outputs.n_positions         = N_slices;

                // 更新 cylinder_fitter_outputs 以反映回退计算结果
                optimized_slice_width_mm           = final_slice_width_mm;
                cylinder_fitter_outputs.r_initial_ = final_r;
                cylinder_fitter_outputs.w0_        = final_w0_deg; // 存储为 deg
                cylinder_fitter_outputs.k_         = final_k;

                ROS_WARN("========== AUTO MODE FALLBACK OUTPUT PARAMETERS ==========");
                ROS_WARN("[fallback] update r: %.4f (%.2f%%) <- origin: %.4f (%.2f%%)", final_r, final_r * 100.0,
                         fp.r_initial_, fp.r_initial_ * 100.0);
                ROS_WARN("[fallback] update w0: %.4f deg (%.2f mm) <- origin: %.2f mm", final_w0_deg, final_w0_mm,
                         fp.step_width_ * (1.0 - fp.r_initial_));
                ROS_WARN("[fallback] update k: %d <- initial k: %d", final_k, fallback_k);
                ROS_WARN("[fallback] final slice width: %.4f deg (%.2f mm)", final_slice_angle_deg,
                         final_slice_width_mm);
                ROS_WARN("[fallback] slice count N: %d (covering 180.00 deg)", N_slices);
                ROS_WARN("===========================================================");
            }

            ROS_ERROR("Converting slice angle to width: %.4f deg -> %.2f mm (radius: %.2f mm)",
                      outputs.optimal_slice_angle, optimized_slice_width_mm, radius_mm);

            // 使用优化后的切片宽度进行切片
            sliceCylinder(180.0, fp.height_, optimized_slice_width_mm);

        } else if (fp.work_mode_ == 1) {
            // 手动作业，给定 k 和 r
            // 切片宽：喷幅 * (1-r) * k => 需要180度整除 => 可能更新 r 和 k
            ROS_INFO("Manual mode: calculating slice width with given k=%d, r=%.4f", fp.k_, fp.r_initial_);

            // 圆柱半径 (mm)
            double radius_mm = cylinder_model_.radius * 1000.0;

            // 调用手动模式优化函数
            int    final_k              = 0;
            double final_r              = 0.0;
            double final_w0_mm          = 0.0;
            double final_slice_width_mm = 0.0;

            int N_slices = calculate_manual_slice_params(fp.step_width_, fp.k_, fp.r_initial_, radius_mm, final_k,
                                                         final_r, final_w0_mm, final_slice_width_mm);

            // 计算角度用于日志
            double final_w0_deg          = (final_w0_mm / radius_mm) * (180.0 / M_PI);
            double final_slice_angle_deg = (final_slice_width_mm / radius_mm) * (180.0 / M_PI);
            double w0_init_mm            = fp.step_width_ * (1.0 - fp.r_initial_);

            // ========== 更新 cylinder_fitter_outputs ==========
            // 不变量
            cylinder_fitter_outputs.slice_distance_ = fp.slice_distance_;
            cylinder_fitter_outputs.height_         = fp.height_;
            cylinder_fitter_outputs.step_width_     = fp.step_width_;
            cylinder_fitter_outputs.step_height_    = fp.step_height_;
            cylinder_fitter_outputs.robot_R_arm_    = fp.robot_R_arm_;
            cylinder_fitter_outputs.work_mode_      = fp.work_mode_;
            // 变化量（w0_ 存储为 deg）
            cylinder_fitter_outputs.r_initial_ = final_r;
            cylinder_fitter_outputs.w0_        = final_w0_deg; // 存储为 deg
            cylinder_fitter_outputs.k_         = final_k;

            // ========== 手动模式回传参数日志打印 ==========
            ROS_WARN("========== MANUAL MODE OUTPUT PARAMETERS ==========");
            ROS_WARN("[callback] update r: %.4f (%.2f%%) <- origin: %.4f (%.2f%%)", final_r, final_r * 100.0,
                     fp.r_initial_, fp.r_initial_ * 100.0);
            ROS_WARN("[callback] update effective single spray width w0: %.4f deg (%.2f mm) <- origin: %.2f mm",
                     final_w0_deg, final_w0_mm, w0_init_mm);
            ROS_WARN("[callback] update spray move times k: %d <- origin: %d", final_k, fp.k_);
            ROS_WARN("[callback] final slice width: %.4f deg (%.2f mm) = w0 * k = %.4f * %d", final_slice_angle_deg,
                     final_slice_width_mm, final_w0_deg, final_k);
            ROS_WARN("[callback] slice count N: %d (covering 180.00 deg)", N_slices);
            ROS_WARN("[callback] N * slice_angle = %d * %.4f = %.4f deg (target: 180.00 deg)", N_slices,
                     final_slice_angle_deg, N_slices * final_slice_angle_deg);
            ROS_WARN("===================================================");

            // 使用调整后的切片宽度进行切片
            sliceCylinder(180.0, fp.height_, final_slice_width_mm);
        }

        // w0_ 存储的是 deg，计算对应的 mm 值
        double w0_mm = cylinder_fitter_outputs.w0_ * (M_PI / 180.0) * (cylinder_model_.radius * 1000.0);
        ROS_WARN("Final output parameters:\n slice_distance=%.2f mm, height=%.2f mm, step_width=%.2f mm, "
                 "step_height=%.2f mm, r_initial=%.4f, "
                 "robot_R_arm=%.2f mm, work_mode=%d, k=%d, w0=%.4f deg (%.2f mm)\n",
                 cylinder_fitter_outputs.slice_distance_, cylinder_fitter_outputs.height_,
                 cylinder_fitter_outputs.step_width_, cylinder_fitter_outputs.step_height_,
                 cylinder_fitter_outputs.r_initial_, cylinder_fitter_outputs.robot_R_arm_,
                 cylinder_fitter_outputs.work_mode_, cylinder_fitter_outputs.k_, cylinder_fitter_outputs.w0_, w0_mm);
        // 打印结果
        printResults();
        // 保存可视化数据（包括切片信息）
        // saveVisualizationData();
        return 1;
    } else {
        return -1;
    }
}

/// @brief 计算虚拟机械臂原点方案下的 P_cyl
/// @param geo_center_m 圆柱几何中心（雷达坐标系，单位：m）
/// @param cylinder_radius_mm 圆柱半径（mm）
/// @param work_distance_mm 工作距离（mm）
/// @param z_center_mm 输出的P_cyl的z坐标（工作高度中心，mm）
/// @return P_cyl_eigen 圆柱轴线上一点相对于虚拟原点的位置（mm）
Eigen::Vector3d CylinderFitter::calculateVirtualBaseP_cyl(const Eigen::Vector3f &geo_center_m,
                                                          double cylinder_radius_mm, double work_distance_mm,
                                                          double z_center_mm) {
    // ============ 虚拟机械臂原点方案 ============
    // 1. 在雷达坐标系下计算虚拟机械臂原点位置
    // 2. 虚拟原点到圆柱轴线的距离 = R_cyl + D_work（工作距离）
    // 3. 以虚拟原点为基准计算 P_cyl，从而得到最大覆盖角
    // 4. 切片基于圆柱表面，与虚拟原点位置无关
    // 5. 实际工作点从切片中心向外延伸相同的工作距离

    // 圆柱几何中心（雷达坐标系，单位：m -> mm）
    Eigen::Vector3d geo_center_mm = geo_center_m.cast<double>() * 1000.0;

    // 虚拟基座到轴线的距离 = R_cyl + virtual_base_to_surface_（物理基座安装距离）
    // 注意：参数 work_distance_mm（= slice_distance_，喷涂距离）在此函数中不使用。
    // 虚拟基座位置由 virtual_base_to_surface_ 决定，喷涂距离用于 work_mm 计算（sliceCylinder）。
    double virtual_base_dist = cylinder_radius_mm + fitting_params_.virtual_base_to_surface_;

    // 计算虚拟机械臂原点：在XY平面上，从圆柱中心向外延伸
    // 选择几何中心投影到XY平面的反方向（朝向激光雷达的方向）
    Eigen::Vector3d geo_center_xy(geo_center_mm.x(), geo_center_mm.y(), 0.0);
    double          xy_dist = geo_center_xy.norm();

    Eigen::Vector3d virtual_base_pos;
    if (xy_dist > 1e-6) {
        // 虚拟原点在圆柱中心的XY投影方向上，距离轴线 virtual_base_dist
        Eigen::Vector3d dir_xy = -geo_center_xy.normalized(); // 指向原点方向
        virtual_base_pos       = geo_center_mm + dir_xy * virtual_base_dist;
        virtual_base_pos.z()   = 0.0; // 虚拟原点在z=0平面上
    } else {
        // 圆柱中心在原点附近，选择X轴正方向
        virtual_base_pos = Eigen::Vector3d(virtual_base_dist, 0.0, 0.0);
    }

    ROS_INFO("Virtual robot base position (lidar frame): [%.2f, %.2f, %.2f] mm", virtual_base_pos.x(),
             virtual_base_pos.y(), virtual_base_pos.z());
    ROS_INFO("Distance from virtual base to cylinder axis: %.2f mm (R=%.2f + surface_dist=%.2f)", virtual_base_dist,
             cylinder_radius_mm, fitting_params_.virtual_base_to_surface_);

    // 计算圆柱轴线上一点相对于虚拟原点的位置
    // P_cyl = geo_center - virtual_base_pos （以虚拟原点为原点）
    Eigen::Vector3d P_cyl_eigen = geo_center_mm - virtual_base_pos;

    // 使用指定的z坐标（工作高度中心）
    P_cyl_eigen.z() = z_center_mm;

    return P_cyl_eigen;
}

/// @return
std::vector<double> CylinderFitter::cylinder_center_convert2robot_base(const std::vector<double> target_pose,
                                                                       const std::vector<double> spray_center,
                                                                       const std::vector<double> circle_center) {

    // 任意角度变换：假设 theta 为小臂相对于曲臂坐标系的旋转角度
    // theta = 90度 + arctan(spray_center_y / spray_center_x)
    // 曲臂指向喷涂中心方向，且小臂相对于曲臂有 90 度的固定旋转
    double yaw   = std::atan2(spray_center[1], spray_center[0]);
    double theta = M_PI / 2.0 + yaw;
    double cos_t = std::cos(theta);
    double sin_t = std::sin(theta);

    // 计算全局坐标系下的差值向量 (Target -> Spray Center)
    double dx_s = spray_center[0] - target_pose[0] * 1000;
    // 列补偿：若该目标属于“大 x 列”，根据桥接节点对 x 额外 -0.3m 的调整，在此再手动减小 300mm
    if (target_pose[0] > 0.8) {
        dx_s -= 500.0; // 额外减小 500mm
        ROS_INFO_STREAM_THROTTLE(1.0, "[robot_bt_manager] dx_s column compensation applied (x>1.0m): -300mm");
    }
    double dy_s = spray_center[1] - target_pose[1] * 1000 - 700;
    // 旋转矩阵逆变换 (R^T) 转换到小臂局部坐标系
    double sx = dx_s * cos_t + dy_s * sin_t;
    double sy = -dx_s * sin_t + dy_s * cos_t;
    double sz = spray_center[2] - target_pose[2] * 1000;
    // 同理计算 circle_center
    double dx_c = circle_center[0] - target_pose[0] * 1000;
    if (target_pose[0] > 0.8) {
        dx_c -= 500.0; // 额外减小 500mm
        ROS_INFO_STREAM_THROTTLE(1.0, "[robot_bt_manager] dx_s column compensation applied (x>1.0m): -300mm");
    }
    double dy_c = circle_center[1] - target_pose[1] * 1000 - 700;

    double cx = dx_c * cos_t + dy_c * sin_t;
    double cy = -dx_c * sin_t + dy_c * cos_t;
    double cz = circle_center[2] - target_pose[2] * 1000;
    std::cout << " Converted spray center to robot base: [" << sx << "," << sy << "," << sz << "] mm" << std::endl;
    std::cout << " Converted circle center to robot base: [" << cx << "," << cy << "," << cz << "] mm" << std::endl;
    return {cx, cy, cz};
}

/// @brief 法线估计
/// @param cloud 点云
/// @param normals 法线
void CylinderFitter::estimateNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                     pcl::PointCloud<pcl::Normal>::Ptr   normals) {
    const auto                                       &fp = fitting_params_;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr           tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    ne.setKSearch(fp.normal_k_search_); // 法线估计KNN搜索邻居数
    ne.compute(*normals);
}

/// @brief 改进的平面去除函数（支持墙面 + 地面/天花板）
/// 同时移除竖直墙面和水平地面/天花板，确保稀疏圆柱不被大平面淹没
void CylinderFitter::removeWalls(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, double angle_threshold,
                                 double distance_threshold, int max_iterations) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    *temp_cloud = *cloud;

    int       walls_removed  = 0;
    int       floors_removed = 0;
    int       planes_removed = 0;
    const int max_walls      = fitting_params_.max_walls_;          // 最多检测墙面数
    const int max_planes     = max_walls + 2;                       // 墙面 + 地面 + 天花板
    const double min_keep_ratio = fitting_params_.wall_min_keep_ratio_; // 使用参数而非硬编码
    int       skipped_planes = 0;
    const int max_skip       = 2;  // 最多跳过2个非墙非地面的平面后停止

    while (planes_removed < max_planes && temp_cloud->size() > cloud->size() * min_keep_ratio) {
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr              inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr         coefficients(new pcl::ModelCoefficients);

        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(max_iterations);
        seg.setDistanceThreshold(distance_threshold);
        seg.setInputCloud(temp_cloud);

        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty() || (int)inliers->indices.size() < fitting_params_.wall_min_plane_points_) {
            break; // 没有更多显著平面
        }

        // 计算平面与Z轴的夹角
        Eigen::Vector3f plane_normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
        plane_normal.normalize();

        Eigen::Vector3f z_axis(0.0, 0.0, 1.0);
        double          angle = std::acos(std::abs(plane_normal.dot(z_axis))) * 180.0 / M_PI;

        // 判断平面类型
        bool is_wall  = std::abs(angle - 90.0) < angle_threshold;  // 竖直墙面: 法线与Z轴接近垂直
        bool is_floor = angle < angle_threshold;                    // 水平地面/天花板: 法线与Z轴接近平行

        if (is_wall || is_floor) {
            // 去除平面点
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(temp_cloud);
            extract.setIndices(inliers);
            extract.setNegative(true);

            pcl::PointCloud<pcl::PointXYZ>::Ptr temp_filtered(new pcl::PointCloud<pcl::PointXYZ>);
            extract.filter(*temp_filtered);

            const char *plane_type = is_wall ? "wall" : "floor/ceiling";
            ROS_INFO("Removed %s %d with %zu points (angle: %.2f°)", plane_type,
                     (is_wall ? walls_removed : floors_removed) + 1, inliers->indices.size(), angle);

            temp_cloud = temp_filtered;
            planes_removed++;
            skipped_planes = 0; // 修复 ISSUE-009：成功移除后重置，确保"连续跳过"语义正确
            if (is_wall) walls_removed++;
            else floors_removed++;
        } else {
            // 非墙面也非地面的倾斜平面：从搜索池中移除以继续寻找后面的墙面/地面
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(temp_cloud);
            extract.setIndices(inliers);
            extract.setNegative(true);

            pcl::PointCloud<pcl::PointXYZ>::Ptr temp_remaining(new pcl::PointCloud<pcl::PointXYZ>);
            extract.filter(*temp_remaining);

            ROS_INFO("Skipped oblique plane with %zu points (angle: %.2f°), continuing search...",
                     inliers->indices.size(), angle);

            temp_cloud = temp_remaining;
            // 修复 ISSUE-009：倾斜平面不计入 planes_removed（不占用墙面移除配额）
            skipped_planes++;
            if (skipped_planes >= max_skip) {
                break; // 连续跳过太多非目标平面，停止
            }
        }
    }

    // 安全检查：保证至少保留 min_keep_ratio 的点
    if (temp_cloud->size() < cloud->size() * min_keep_ratio) {
        ROS_WARN("Too many points removed (%.1f%%), keeping original cloud.",
                 100.0 * (1.0 - (double)temp_cloud->size() / cloud->size()));
        *cloud_filtered = *cloud;
    } else {
        *cloud_filtered = *temp_cloud;
        ROS_INFO("Plane removal complete: %d walls + %d floors removed, %zu -> %zu points (kept %.1f%%)",
                 walls_removed, floors_removed,
                 cloud->size(), cloud_filtered->size(), 100.0 * cloud_filtered->size() / cloud->size());
    }
}

/// @brief 提取对应的法向量
bool CylinderFitter::extractCorrespondingNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_original,
                                                 pcl::PointCloud<pcl::Normal>::Ptr   normals_original,
                                                 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered,
                                                 pcl::PointCloud<pcl::Normal>::Ptr   normals_filtered) {

    if (cloud_original->size() != normals_original->size()) {
        ROS_ERROR("Cloud and normals size mismatch: %zu vs %zu!", cloud_original->size(), normals_original->size());
        return false;
    }

    // 构建KD树用于快速查找
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud_original);

    normals_filtered->points.clear();
    normals_filtered->points.reserve(cloud_filtered->size());

    int matched = 0;
    for (const auto &pt : cloud_filtered->points) {
        std::vector<int>   indices(1);
        std::vector<float> distances(1);

        if (kdtree.nearestKSearch(pt, 1, indices, distances) > 0) {
            if (distances[0] < 1e-6) { // 确保是同一个点（距离几乎为0）
                normals_filtered->points.push_back(normals_original->points[indices[0]]);
                matched++;
            } else {
                // 如果找不到精确匹配，使用最近邻的法向量
                normals_filtered->points.push_back(normals_original->points[indices[0]]);
            }
        }
    }

    normals_filtered->width    = normals_filtered->points.size();
    normals_filtered->height   = 1;
    normals_filtered->is_dense = true;

    ROS_INFO("Extracted %d/%zu corresponding normals", matched, cloud_filtered->size());
    return normals_filtered->size() == cloud_filtered->size();
}

/// @brief 计算圆度（点在圆周上的分布均匀性）
double CylinderFitter::calculateCircularity(pcl::PointCloud<pcl::PointXYZ>::Ptr inliers, const Eigen::Vector3f &center,
                                            const Eigen::Vector3f &axis, double radius) {

    if (inliers->size() < 10) return 0.0;

    // 将点投影到垂直于轴的平面，计算角度分布
    const int        num_sectors = 24; // 15度一个扇区（从36改为24，降低对完整性要求）
    std::vector<int> sector_counts(num_sectors, 0);

    Eigen::Vector3f axis_norm = axis.normalized();

    // 构建局部坐标系
    Eigen::Vector3f ref_x = (std::abs(axis_norm.z()) < 0.9) ? Eigen::Vector3f(0, 0, 1).cross(axis_norm).normalized()
                                                            : Eigen::Vector3f(1, 0, 0).cross(axis_norm).normalized();
    Eigen::Vector3f ref_y = axis_norm.cross(ref_x).normalized();

    for (const auto &p : inliers->points) {
        Eigen::Vector3f pt(p.x, p.y, p.z);
        Eigen::Vector3f v = pt - center;

        // 投影到垂直于轴的平面
        Eigen::Vector3f v_proj = v - (v.dot(axis_norm)) * axis_norm;

        if (v_proj.norm() < 1e-6) continue;

        double x_comp = v_proj.dot(ref_x);
        double y_comp = v_proj.dot(ref_y);
        double angle  = std::atan2(y_comp, x_comp);
        if (angle < 0) angle += 2.0 * M_PI;

        int sector = static_cast<int>(angle / (2.0 * M_PI) * num_sectors);
        sector     = std::min(sector, num_sectors - 1);
        sector_counts[sector]++;
    }

    // 计算非空扇区的比例
    int non_empty_sectors         = 0;
    int max_consecutive_empty     = 0;
    int current_consecutive_empty = 0;

    for (int i = 0; i < num_sectors * 2; ++i) { // 循环两次检测连续空扇区
        int idx = i % num_sectors;
        if (sector_counts[idx] > 0) {
            non_empty_sectors         = (i < num_sectors) ? non_empty_sectors + 1 : non_empty_sectors;
            max_consecutive_empty     = std::max(max_consecutive_empty, current_consecutive_empty);
            current_consecutive_empty = 0;
        } else {
            current_consecutive_empty++;
        }
    }

    double coverage_ratio = (double)non_empty_sectors / (double)num_sectors;

    // **改进1：如果覆盖率达到50%以上，就认为是有效圆柱**
    // 这对于被遮挡的圆柱更友好
    if (coverage_ratio < 0.5) {
        return coverage_ratio; // 覆盖率太低，直接返回
    }

    // **改进2：检查是否存在大的连续空洞（说明可能是墙角）**
    // 如果最大连续空洞超过180度（一半），可能不是圆柱
    double max_gap_ratio = (double)max_consecutive_empty / (double)num_sectors;
    if (max_gap_ratio > 0.6) {
        return coverage_ratio * 0.6; // 惩罚有大空洞的情况
    }

    // **改进3：计算有点扇区的径向一致性**
    // 真圆柱的点到轴心距离应该接近一致
    std::vector<double> radii_by_sector;
    for (int i = 0; i < num_sectors; ++i) {
        if (sector_counts[i] > 0) {
            // 计算该扇区点的平均半径
            double sector_sum_r       = 0.0;
            int    sector_point_count = 0;

            for (const auto &p : inliers->points) {
                Eigen::Vector3f pt(p.x, p.y, p.z);
                Eigen::Vector3f v      = pt - center;
                Eigen::Vector3f v_proj = v - (v.dot(axis_norm)) * axis_norm;

                if (v_proj.norm() < 1e-6) continue;

                double x_comp = v_proj.dot(ref_x);
                double y_comp = v_proj.dot(ref_y);
                double angle  = std::atan2(y_comp, x_comp);
                if (angle < 0) angle += 2.0 * M_PI;

                int sector = static_cast<int>(angle / (2.0 * M_PI) * num_sectors);
                sector     = std::min(sector, num_sectors - 1);

                if (sector == i) {
                    sector_sum_r += v_proj.norm();
                    sector_point_count++;
                }
            }

            if (sector_point_count > 0) {
                radii_by_sector.push_back(sector_sum_r / sector_point_count);
            }
        }
    }

    // 计算各扇区平均半径的变异系数
    if (radii_by_sector.size() > 2) {
        double mean_r = 0.0;
        for (double r : radii_by_sector)
            mean_r += r;
        mean_r /= radii_by_sector.size();

        double var_r = 0.0;
        for (double r : radii_by_sector) {
            var_r += (r - mean_r) * (r - mean_r);
        }
        var_r /= radii_by_sector.size();
        double std_r = std::sqrt(var_r);
        double cv    = std_r / mean_r;

        // 如果各扇区半径变化太大（>20%），说明不是真圆柱
        if (cv > 0.2) {
            return coverage_ratio * (1.0 - cv); // 根据变异系数降低评分
        }
    }

    // **最终圆度评分：覆盖率加权**
    // 覆盖率50%给0.5分，100%给1.0分
    double final_circularity = 0.5 + coverage_ratio * 0.5;

    return std::min(1.0, final_circularity);
}

/// @brief 标准圆柱拟合
/// @param cloud 点云
/// @param normals 法线（旧接口保留，实际在平面去除后重新估计）
void CylinderFitter::fitCylinderPCL(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                    pcl::PointCloud<pcl::Normal>::Ptr   normals) {

    // 输入验证
    if (cloud->empty()) {
        ROS_ERROR("Input cloud is empty!");
        cylinder_model_.valid = false;
        return;
    }

    // 初始化
    cylinder_model_.valid = false;
    cylinder_model_.inliers->clear();
    const auto &fp = fitting_params_;

    // 去除墙面 + 地面点
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    removeWalls(cloud, cloud_filtered, fp.wall_angle_threshold_, fp.wall_distance_threshold_, fp.wall_max_iterations_);

    // 预过滤：移除 XY 距离小于阈值的点（消除雷达正下方的假目标诱饵）
    if (fp.min_center_xy_dist_m > 0.0) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xy_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        double min_xy_sq = fp.min_center_xy_dist_m * fp.min_center_xy_dist_m;
        for (const auto &pt : cloud_filtered->points) {
            double xy_sq = pt.x * pt.x + pt.y * pt.y;
            if (xy_sq >= min_xy_sq) {
                cloud_xy_filtered->points.push_back(pt);
            }
        }
        cloud_xy_filtered->width = cloud_xy_filtered->points.size();
        cloud_xy_filtered->height = 1;
        cloud_xy_filtered->is_dense = cloud_filtered->is_dense;
        size_t removed = cloud_filtered->size() - cloud_xy_filtered->size();
        ROS_INFO("XY distance pre-filter: removed %zu points with XY < %.2f m, %zu points remaining",
                 removed, fp.min_center_xy_dist_m, cloud_xy_filtered->size());
        cloud_filtered = cloud_xy_filtered;
    }

    if (cloud_filtered->empty()) {
        ROS_WARN("No valid points left after plane removal.");
        cylinder_model_.valid = false;
        return;
    }

    // **关键改进：在去除平面后重新估计法线**
    // 原来使用全量点云的法线，稀疏圆柱点的K邻居大多是墙/地面点，导致法线被污染
    // 去除平面后重新估计，圆柱点的法线更准确，RANSAC法线权重才能有效发挥作用
    pcl::PointCloud<pcl::Normal>::Ptr normals_filtered(new pcl::PointCloud<pcl::Normal>);
    estimateNormals(cloud_filtered, normals_filtered);

    if (cloud_filtered->size() != normals_filtered->size()) {
        ROS_ERROR("Cloud and normals size mismatch after re-estimation!");
        cylinder_model_.valid = false;
        return;
    }

    // 配置圆柱拟合分割器
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(fp.normal_distance_weight_);
    seg.setMaxIterations(fp.ransac_max_iterations_);
    seg.setDistanceThreshold(fp.ransac_distance_threshold_);
    seg.setRadiusLimits(fp.min_radius_m, fp.max_radius_m);

    seg.setInputCloud(cloud_filtered);
    seg.setInputNormals(normals_filtered);

    // 轴向约束：参考轴从参数配置，通过 eps_angle_deg_ 控制允许的偏离角度
    // 竖直立柱: axis=(0,0,1), eps=10~15°
    // 倾斜圆柱: axis=实测方向(0.707,0,0.707等), eps=15~25°
    Eigen::Vector3f ref_axis(fp.ransac_axis_x_, fp.ransac_axis_y_, fp.ransac_axis_z_);
    if (ref_axis.norm() < 1e-6) {
        ref_axis = Eigen::Vector3f(0, 0, 1); // 安全回退
    }
    ref_axis.normalize();
    seg.setAxis(ref_axis);
    seg.setEpsAngle(static_cast<float>(fp.eps_angle_deg_) * M_PI / 180.0f);

    ROS_INFO("RANSAC input: %zu pts, axis=[%.3f,%.3f,%.3f], normal_w=%.3f, dist_thresh=%.4f, eps_angle=%.1f deg, R=[%.3f-%.3f], max_iter=%d",
             cloud_filtered->size(), ref_axis.x(), ref_axis.y(), ref_axis.z(),
             fp.normal_distance_weight_, fp.ransac_distance_threshold_,
             fp.eps_angle_deg_, fp.min_radius_m, fp.max_radius_m, fp.ransac_max_iterations_);

    pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr      inliers_cylinder(new pcl::PointIndices);
    seg.segment(*inliers_cylinder, *coefficients_cylinder);

    if (inliers_cylinder->indices.empty()) {
        ROS_WARN("No cylinder found in filtered cloud (RANSAC returned 0 inliers).");
        cylinder_model_.valid = false;
        return;
    }

    // 诊断: 输出RANSAC原始结果
    if (coefficients_cylinder->values.size() >= 7) {
        Eigen::Vector3f raw_axis(coefficients_cylinder->values[3], coefficients_cylinder->values[4],
                                 coefficients_cylinder->values[5]);
        raw_axis.normalize();
        double tilt_from_z = std::acos(std::abs(raw_axis.z())) * 180.0 / M_PI;
        ROS_INFO("RANSAC raw result: %zu inliers, R=%.4f m, axis=[%.3f,%.3f,%.3f], tilt_from_vertical=%.1f deg",
                 inliers_cylinder->indices.size(), coefficients_cylinder->values[6],
                 raw_axis.x(), raw_axis.y(), raw_axis.z(), tilt_from_z);
    }

    // 计算内点统计（基于过滤后的点云）
    size_t inlier_count = inliers_cylinder->indices.size();
    double inlier_ratio = (double)inlier_count / (double)cloud_filtered->size();

    if (inlier_count < (size_t)fp.min_inliers) {
        ROS_WARN("Inlier count too low (%zu < %d), likely no cylinder present.", inlier_count, fp.min_inliers);
        cylinder_model_.valid = false;
        return;
    }

    // 提取内点
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers_cylinder);
    extract.setNegative(false);
    extract.filter(*cylinder_model_.inliers);

    // 圆柱几何参数计算
    Eigen::Vector3f fitted_axis(coefficients_cylinder->values[3], coefficients_cylinder->values[4],
                                coefficients_cylinder->values[5]);
    fitted_axis.normalize();

    Eigen::Vector3f proj_center(coefficients_cylinder->values[0], coefficients_cylinder->values[1],
                                coefficients_cylinder->values[2]);

    // 半径统计
    double              max_r = 0.0;
    double              min_r = std::numeric_limits<double>::max();
    double              sum_r = 0.0;
    std::vector<double> radii;
    radii.reserve(cylinder_model_.inliers->size());

    for (const auto &p : cylinder_model_.inliers->points) {
        Eigen::Vector3f pt(p.x, p.y, p.z);
        Eigen::Vector3f v      = pt - proj_center;
        Eigen::Vector3f v_proj = v - (v.dot(fitted_axis)) * fitted_axis;
        double          r      = v_proj.norm();

        radii.push_back(r);
        sum_r += r;
        max_r = std::max(max_r, r);
        min_r = std::min(min_r, r);
    }

    double mean_r = sum_r / (double)radii.size();

    // 计算标准差
    double var_r = 0.0;
    for (const auto &r : radii) {
        var_r += (r - mean_r) * (r - mean_r);
    }
    var_r /= (double)radii.size();
    double std_r          = std::sqrt(var_r);
    double radial_rel_std = std_r / mean_r;

    // 径向极差
    double radial_range       = max_r - min_r;
    double radial_range_ratio = radial_range / mean_r;

    // 高度计算
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

    // 长宽比
    double aspect_ratio = cylinder_height / (2.0 * mean_r);

    // 圆度检查
    double circularity = calculateCircularity(cylinder_model_.inliers, proj_center, fitted_axis, mean_r);

    ROS_INFO("========== Cylinder Validation ==========");
    ROS_INFO("Inliers:          %zu [%d-%d] %s", inlier_count, fp.min_inliers, fp.max_inliers,
             (inlier_count >= (size_t)fp.min_inliers && inlier_count <= (size_t)fp.max_inliers) ? "ok" : "false");
    ROS_INFO("Inlier ratio:     %.4f [%.4f-%.4f] %s", inlier_ratio, fp.min_inlier_ratio, fp.max_inlier_ratio,
             (inlier_ratio >= fp.min_inlier_ratio && inlier_ratio <= fp.max_inlier_ratio) ? "ok" : "false");
    ROS_INFO("Height:           %.2f m [%.2f-∞] %s", cylinder_height, fp.min_height_m,
             (cylinder_height >= fp.min_height_m) ? "ok" : "false");
    ROS_INFO("Radius:           %.3f m [%.3f-%.3f] %s", mean_r, fp.min_radius_m, fp.max_radius_m,
             (mean_r >= fp.min_radius_m && mean_r <= fp.max_radius_m) ? "ok" : "false");
    ROS_INFO("Radial rel std:   %.4f [%.4f-%.4f] %s", radial_rel_std, fp.min_radial_rel_std, fp.max_radial_rel_std,
             (radial_rel_std >= fp.min_radial_rel_std && radial_rel_std <= fp.max_radial_rel_std) ? "ok" : "false");
    ROS_INFO("Radial range:     %.4f (%.1f%% of radius) [<%.0f%%] %s", radial_range, radial_range_ratio * 100.0,
             fp.max_radial_range_ratio_ * 100.0, (radial_range_ratio <= fp.max_radial_range_ratio_) ? "ok" : "false");
    ROS_INFO("Aspect ratio:     %.2f (H/D) [>%.1f] %s", aspect_ratio, fp.min_aspect_ratio_,
             (aspect_ratio >= fp.min_aspect_ratio_) ? "ok" : "false");
    ROS_INFO("Circularity:      %.4f [>%.2f] %s", circularity, fp.min_circularity_,
             (circularity >= fp.min_circularity_) ? "ok" : "false");
    double center_xy_dist = Eigen::Vector2f(proj_center.x(), proj_center.y()).norm();
    ROS_INFO("Center XY dist:   %.3f m [>%.2f] %s", center_xy_dist, fp.min_center_xy_dist_m,
             (fp.min_center_xy_dist_m <= 0.0 || center_xy_dist >= fp.min_center_xy_dist_m) ? "ok" : "false");
    ROS_INFO("=========================================");

    bool        valid = true;
    std::string reject_reason;

    if (inlier_count < (size_t)fp.min_inliers || inlier_count > (size_t)fp.max_inliers) {
        valid         = false;
        reject_reason = "Inlier count out of range";
    } else if (inlier_ratio < fp.min_inlier_ratio || inlier_ratio > fp.max_inlier_ratio) {
        valid         = false;
        reject_reason = "Inlier ratio out of range";
    } else if (cylinder_height < fp.min_height_m) {
        valid         = false;
        reject_reason = "Height too small";
    } else if (mean_r < fp.min_radius_m || mean_r > fp.max_radius_m) {
        valid         = false;
        reject_reason = "Radius out of range";
    } else if (radial_rel_std < fp.min_radial_rel_std || radial_rel_std > fp.max_radial_rel_std) {
        valid         = false;
        reject_reason = "Radial relative std out of range";
    } else if (radial_range_ratio > fp.max_radial_range_ratio_) {
        valid         = false;
        reject_reason = "Radial range too large (not circular)";
    } else if (aspect_ratio < fp.min_aspect_ratio_) {
        valid         = false;
        reject_reason = "Aspect ratio too small (too flat)";
    } else if (circularity < fp.min_circularity_) {
        valid         = false;
        reject_reason = "Circularity too low (not round enough)";
    } else if (fp.min_center_xy_dist_m > 0.0 && center_xy_dist < fp.min_center_xy_dist_m) {
        valid         = false;
        reject_reason = "Cylinder center too close to LiDAR in XY plane";
    }

    if (!valid) {
        ROS_ERROR("Cylinder REJECTED: %s", reject_reason.c_str());
        cylinder_model_.valid = false;
        cylinder_model_.inliers->clear();
        return;
    }

    // 构建圆柱模型
    cylinder_model_.valid         = true;
    cylinder_model_.radius        = mean_r;
    cylinder_model_.bottom_center = proj_center + min_proj * axis_norm;
    cylinder_model_.top_center    = proj_center + max_proj * axis_norm;
    cylinder_model_.geo_center    = 0.5f * (cylinder_model_.bottom_center + cylinder_model_.top_center);
    cylinder_model_.axis          = axis_norm;
    cylinder_model_.height        = cylinder_height;

    // 强制轴向朝向 +Z
    if (cylinder_model_.axis.z() < 0.0f) {
        cylinder_model_.axis = -cylinder_model_.axis;
        std::swap(cylinder_model_.bottom_center, cylinder_model_.top_center);
        cylinder_model_.geo_center = 0.5f * (cylinder_model_.bottom_center + cylinder_model_.top_center);
    }

    // 成功输出
    ROS_INFO("========== Cylinder ACCEPTED ==========");
    std::cout << "圆柱检测成功:\n"
              << "  底面中心: [" << cylinder_model_.bottom_center.transpose() << "]\n"
              << "  几何中心: [" << cylinder_model_.geo_center.transpose() << "]\n"
              << "  顶面中心: [" << cylinder_model_.top_center.transpose() << "]\n"
              << "  轴向:     [" << cylinder_model_.axis.transpose() << "]\n"
              << "  半径:     " << cylinder_model_.radius << " m\n"
              << "  高度:     " << cylinder_model_.height << " m\n"
              << "  内点数:   " << inlier_count << "\n"
              << "  圆度:     " << circularity << "\n"
              << "  长宽比:   " << aspect_ratio << "\n";
    ROS_INFO("=======================================");
}

/// @brief 几何约束计算 (移植自 cal.py)
/// @param robot_max_reach 机械臂最大臂展（球半径）如1500mm
/// @param cylinder_center 圆柱轴线上一点[x,y,z](机械臂坐标系)
/// @param cylinder_axis  圆柱轴线防线[i,j,k]
/// @param cylinder_radius  圆柱半径
/// @param offset_dist 喷涂距离
/// @param regional_height 总工作高度
/// @return
std::pair<double, std::string> CylinderFitter::calculate_max_slice_angle(double                     robot_max_reach,
                                                                         const std::vector<double> &cylinder_center,
                                                                         const std::vector<double> &cylinder_axis,
                                                                         double cylinder_radius, double offset_dist,
                                                                         double regional_height) {

    // 调试输出输入参数
    ROS_INFO("========== calculate_max_slice_angle inputs ==========");
    ROS_INFO("robot_max_reach (R_arm): %.2f mm", robot_max_reach);
    ROS_INFO("cylinder_center (P_cyl): [%.2f, %.2f, %.2f] mm", cylinder_center[0], cylinder_center[1],
             cylinder_center[2]);
    ROS_INFO("cylinder_axis (V_axis): [%.4f, %.4f, %.4f]", cylinder_axis[0], cylinder_axis[1], cylinder_axis[2]);
    ROS_INFO("cylinder_radius (r_cyl): %.2f mm", cylinder_radius);
    ROS_INFO("offset_dist (D_offset): %.2f mm", offset_dist);
    ROS_INFO("regional_height (H): %.2f mm", regional_height);

    // 1. 确定最远工作高度 Delta_Z_max
    double Z_cyl_center = cylinder_center[2];
    double H            = regional_height;
    double Z_max_limit  = Z_cyl_center + H / 2.0;
    double Z_min_limit  = Z_cyl_center - H / 2.0;
    double Delta_Z_max  = std::max(std::abs(Z_max_limit), std::abs(Z_min_limit));

    ROS_INFO("Work range Z: [%.2f, %.2f] mm", Z_min_limit, Z_max_limit);
    ROS_INFO("Delta_Z_max (max vertical distance from base): %.2f mm", Delta_Z_max);

    // 2. 计算垂直距离 D
    std::vector<double> P_base      = {0.0, 0.0, 0.0};
    std::vector<double> P_cyl       = cylinder_center;
    std::vector<double> V_axis_unit = cylinder_axis;
    double              axis_norm   = norm(V_axis_unit);
    if (axis_norm == 0.0) {
        return {0.0, "Error: Cylinder axis vector has zero length"};
    }
    // 归一化 V_axis
    V_axis_unit[0] /= axis_norm;
    V_axis_unit[1] /= axis_norm;
    V_axis_unit[2] /= axis_norm;

    std::vector<double> w = {P_base[0] - P_cyl[0], P_base[1] - P_cyl[1], P_base[2] - P_cyl[2]};
    double              D = norm(cross_product(w, V_axis_unit));

    ROS_INFO("Distance from base to cylinder axis (D): %.2f mm", D);

    // 3. 计算有效臂长 R_eff
    double R_task = cylinder_radius + offset_dist;

    ROS_INFO("R_task (r_cyl + D_offset): %.2f mm", R_task);

    if (Delta_Z_max >= robot_max_reach) {
        ROS_ERROR("Delta_Z_max (%.2f) >= robot_max_reach (%.2f), work height exceeds arm range!", Delta_Z_max,
                  robot_max_reach);
        return {0.0, "Error: Work height exceeds robot max reach"};
    }

    double R_eff_sq = robot_max_reach * robot_max_reach - Delta_Z_max * Delta_Z_max;
    if (R_eff_sq < 0.0) {
        ROS_ERROR("R_eff_sq is negative: %.2f", R_eff_sq);
        return {0.0, "Error: Effective horizontal reach squared is negative"};
    }
    double R_eff = std::sqrt(R_eff_sq);

    ROS_INFO("Effective horizontal reach (R_eff): %.2f mm", R_eff);

    // 4. 余弦定理计算角度
    if (R_eff < std::abs(D - R_task)) {
        ROS_ERROR("R_eff (%.2f) < |D - R_task| (%.2f), cannot reach surface!", R_eff, std::abs(D - R_task));
        return {0.0, "Error: Cannot reach the cylinder surface"};
    }

    double denominator = 2.0 * R_task * D;
    if (denominator == 0.0) {
        return {0.0, "Error: Denominator is zero, cannot calculate angle"};
    }

    double cos_alpha = (R_task * R_task + D * D - R_eff * R_eff) / denominator;
    cos_alpha        = std::max(-1.0, std::min(1.0, cos_alpha));

    double alpha_rad       = std::acos(cos_alpha);
    double total_angle_deg = 2.0 * (alpha_rad * 180.0 / M_PI);

    ROS_INFO("cos_alpha: %.4f, alpha: %.2f deg, total_angle: %.2f deg", cos_alpha, alpha_rad * 180.0 / M_PI,
             total_angle_deg);
    ROS_INFO("========== calculate_max_slice_angle DONE ==========");

    return {total_angle_deg, "Success"};
}

/// @brief 喷涂角度优化 (自动调节搭接比例)
/// @param max_coverage_angle 最大覆盖角度
/// @param spray_width 喷涂宽度
/// @param initial_overlap_ratio 初始搭接比例
/// @param cylinder_radius 圆柱半径
/// @param offset_dist 喷头工作偏移距离
/// @param final_overlap_ratio 最终搭接比例 (输出)
/// @return
double CylinderFitter::calculate_optimal_slice_angle(double max_coverage_angle, double spray_width,
                                                     double initial_overlap_ratio, double cylinder_radius,
                                                     double offset_dist, double &final_overlap_ratio) {
    // ==================== 参数配置 ====================
    const double TARGET_COVERAGE_ANGLE = 180.0; // 目标覆盖角度：圆柱的一半

    if (max_coverage_angle <= 0.0) {
        final_overlap_ratio = initial_overlap_ratio;
        return 0.0;
    }

    // 1. 计算工作半径 R_task (mm)
    double R_task = cylinder_radius + offset_dist;
    // 1. 使用圆柱表面半径 R_cyl (mm) 来计算角度
    // 注意：角度是相对于圆柱表面的，不是相对于工作点的
    double R_cyl = cylinder_radius;

    // 2. 初始有效喷幅 w0_init (线性距离 mm)
    double w0_init_linear = spray_width * (1.0 - initial_overlap_ratio);

    // 3. 将初始有效喷幅转换为圆柱表面角度 (度)
    // 弧长 = 半径 × 角度(弧度)  =>  角度(度) = 弧长 / 半径 × (180/π)
    double w0_init_deg = (w0_init_linear / R_cyl) * (180.0 / M_PI);

    if (w0_init_deg <= 0.0) {
        ROS_ERROR("Initial w0 must be positive (w0_init_deg = %.4f)", w0_init_deg);
        final_overlap_ratio = initial_overlap_ratio;
        return 0.0;
    }

    ROS_INFO("============ Slice Optimization Algorithm ============");
    ROS_WARN("----------- Before Optimization -----------");
    ROS_INFO("[Before] Target Coverage Angle: %.2f deg (half cylinder)", TARGET_COVERAGE_ANGLE);
    ROS_INFO("[Before] Max Coverage Angle per Position: %.4f deg", max_coverage_angle);
    ROS_INFO("[Before] Spray Width (d): %.2f mm", spray_width);
    ROS_INFO("[Before] Overlap Ratio (r): %.4f (%.1f%%)", initial_overlap_ratio, initial_overlap_ratio * 100.0);
    ROS_INFO("[Before] w0 = d * (1 - r) = %.2f mm (%.4f deg on cylinder surface)", w0_init_linear, w0_init_deg);
    ROS_INFO("[Before] Cylinder Radius (R_cyl): %.2f mm", R_cyl);

    // ==================== 新优化逻辑 ====================

    // 步骤1: 计算需要多少次机械臂基座移动
    // N = ceil(目标覆盖角度 / 单次最大覆盖角度)
    int N_positions = static_cast<int>(std::ceil(TARGET_COVERAGE_ANGLE / max_coverage_angle));
    if (N_positions < 1) N_positions = 1;

    ROS_INFO("[Step1] Number of robot base positions: N = ceil(%.2f / %.4f) = %d", TARGET_COVERAGE_ANGLE,
             max_coverage_angle, N_positions);

    // 步骤2: 计算每次基座位置需要覆盖的角度
    // 每次覆盖 = 目标覆盖角度 / N
    double coverage_per_position = TARGET_COVERAGE_ANGLE / static_cast<double>(N_positions);

    ROS_INFO("[Step2] Coverage per position: %.4f deg (= %.2f / %d)", coverage_per_position, TARGET_COVERAGE_ANGLE,
             N_positions);

    // 检查：每次覆盖角度必须 <= 机械臂单次最大覆盖角度
    if (coverage_per_position > max_coverage_angle + 1e-6) {
        ROS_ERROR("Coverage per position (%.4f deg) exceeds max coverage angle (%.4f deg)!", coverage_per_position,
                  max_coverage_angle);
        final_overlap_ratio = initial_overlap_ratio;
        return max_coverage_angle;
    }

    // 步骤3: 在每次覆盖范围内，计算需要多少个 w0
    // k = round(coverage_per_position / w0_init_deg)
    int k_spray_passes = static_cast<int>(std::round(coverage_per_position / w0_init_deg));
    if (k_spray_passes < 1) k_spray_passes = 1;

    ROS_INFO("[Step3] Number of spray passes per position: k = round(%.4f / %.4f) = %d", coverage_per_position,
             w0_init_deg, k_spray_passes);

    // 步骤4: 调整 w0 使得 k × w0_adjusted = coverage_per_position（精确覆盖）
    double w0_adjusted_deg = coverage_per_position / static_cast<double>(k_spray_passes);
    // 将角度转换回圆柱表面的线性距离
    double w0_adjusted_linear = w0_adjusted_deg * (M_PI / 180.0) * R_cyl;

    ROS_INFO("[Step4] Adjusted w0: %.4f deg (%.2f mm on cylinder surface)", w0_adjusted_deg, w0_adjusted_linear);

    // 步骤5: 反推新的搭接比例
    // w0 = d × (1 - r) => r = 1 - w0 / d
    double r_adjusted = 1.0 - (w0_adjusted_linear / spray_width);

    ROS_INFO("[Step5] Adjusted overlap ratio: r = 1 - %.2f / %.2f = %.4f (%.1f%%)", w0_adjusted_linear, spray_width,
             r_adjusted, r_adjusted * 100.0);

    // ==================== 约束检查 ====================

    bool        need_fallback = false;
    std::string fallback_reason;

    // 约束1：搭接比例必须 >= 初始值（只能增大，不能减小）
    if (r_adjusted < initial_overlap_ratio - 1e-6) {
        ROS_WARN("Constraint violation: r_adjusted (%.4f) < initial_overlap_ratio (%.4f)", r_adjusted,
                 initial_overlap_ratio);
        ROS_WARN("This means we need fewer spray passes than expected. Increasing k...");

        // 增加 k 直到满足约束
        while (r_adjusted < initial_overlap_ratio - 1e-6 && k_spray_passes < 100) {
            k_spray_passes++;
            w0_adjusted_deg    = coverage_per_position / static_cast<double>(k_spray_passes);
            w0_adjusted_linear = w0_adjusted_deg * (M_PI / 180.0) * R_cyl;
            r_adjusted         = 1.0 - (w0_adjusted_linear / spray_width);
        }
        ROS_INFO("After adjustment: k = %d, r = %.4f", k_spray_passes, r_adjusted);
    }

    // 约束2：搭接比例不能 >= 1.0（否则 w0 为零或负）
    if (r_adjusted >= 1.0 - 1e-6) {
        need_fallback   = true;
        fallback_reason = "Overlap ratio >= 1.0 (invalid)";
    }

    // 约束3：w0 必须为正
    if (w0_adjusted_linear <= 0) {
        need_fallback   = true;
        fallback_reason = "w0_adjusted <= 0 (invalid)";
    }

    // ==================== 回退处理 ====================
    if (need_fallback) {
        ROS_WARN("Fallback triggered: %s", fallback_reason.c_str());
        ROS_WARN("Using initial overlap ratio and simple calculation.");

        k_spray_passes     = static_cast<int>(std::ceil(coverage_per_position / w0_init_deg));
        w0_adjusted_deg    = w0_init_deg;
        w0_adjusted_linear = w0_init_linear;
        r_adjusted         = initial_overlap_ratio;
    }

    // ==================== 输出结果 ====================
    final_overlap_ratio = r_adjusted;
    // 通过引用参数返回 k 和 w0（将在调用处处理）
    // 这里先存储到静态变量供后续使用
    last_k_spray_passes_  = k_spray_passes;
    last_w0_adjusted_mm_  = w0_adjusted_linear;
    last_w0_adjusted_deg_ = w0_adjusted_deg;
    last_n_positions_     = N_positions;

    double total_coverage     = coverage_per_position * N_positions;
    int    total_spray_passes = k_spray_passes * N_positions;

    ROS_WARN("----------- After Optimization -----------");
    ROS_INFO("[Result] Robot Base Positions: %d", N_positions);
    ROS_INFO("[Result] Coverage per Position: %.4f deg", coverage_per_position);
    ROS_INFO("[Result] Spray Passes per Position (k): %d", k_spray_passes);
    ROS_INFO("[Result] Total Spray Passes: %d", total_spray_passes);
    ROS_INFO("[Result] Adjusted w0: %.4f deg (%.2f mm)", w0_adjusted_deg, w0_adjusted_linear);
    ROS_INFO("[Result] Adjusted Overlap Ratio: %.4f (%.1f%%)", r_adjusted, r_adjusted * 100.0);
    ROS_INFO("[Result] Total Coverage: %.4f deg (target: %.2f deg)", total_coverage, TARGET_COVERAGE_ANGLE);

    // 显示优化效果
    double overlap_change = (r_adjusted - initial_overlap_ratio) * 100.0;
    if (std::abs(overlap_change) > 0.01) {
        if (overlap_change > 0) {
            ROS_WARN("Overlap ratio INCREASED by %.2f%% (from %.1f%% to %.1f%%) for exact coverage", overlap_change,
                     initial_overlap_ratio * 100.0, r_adjusted * 100.0);
        } else {
            ROS_INFO("Overlap ratio decreased by %.2f%% (from %.1f%% to %.1f%%)", -overlap_change,
                     initial_overlap_ratio * 100.0, r_adjusted * 100.0);
        }
    } else {
        ROS_INFO("Overlap ratio unchanged (%.1f%%)", r_adjusted * 100.0);
    }

    // 显示精度验证
    double actual_coverage_per_pos = k_spray_passes * w0_adjusted_deg;
    double coverage_error          = actual_coverage_per_pos - coverage_per_position;
    ROS_INFO("[Verify] k * w0 = %d * %.4f = %.4f deg (error: %.6f deg)", k_spray_passes, w0_adjusted_deg,
             actual_coverage_per_pos, coverage_error);

    ROS_INFO("=============================================");

    // 返回每次基座位置对应的覆盖角度
    return coverage_per_position;
}

/// @brief 手动模式切片优化 (给定k和r，计算整除180度的切片宽度)
/// 计算初始有效喷幅和切片宽度
/// 将切片宽度转换为角度
/// 计算整除180度所需的切片数量
/// 调整切片角度以精确覆盖180度
/// 反推调整后的 w0 和 r
/// 约束检查（r 不能小于初始值，也不能 >= 1.0）
/// 返回切片数量 N
/// @param spray_width 喷幅宽度 (mm)
/// @param initial_k 初始喷涂移动次数
/// @param initial_r 初始搭接比例
/// @param cylinder_radius_mm 圆柱半径 (mm)
/// @param final_k 输出：调整后的喷涂移动次数
/// @param final_r 输出：调整后的搭接比例
/// @param final_w0_mm 输出：调整后的有效单次喷幅 (mm)
/// @param final_slice_width_mm 输出：最终切片宽度 (mm)
/// @return 切片数量 N
int CylinderFitter::calculate_manual_slice_params(double spray_width, int initial_k, double initial_r,
                                                  double cylinder_radius_mm, int &final_k, double &final_r,
                                                  double &final_w0_mm, double &final_slice_width_mm) {
    const double TARGET_COVERAGE_ANGLE = 180.0; // 目标覆盖角度：圆柱的一半

    // 1. 计算初始有效喷幅 w0_init (mm)
    double w0_init_mm = spray_width * (1.0 - initial_r);

    // 2. 计算初始切片宽度 (mm) = w0 * k
    double slice_width_init_mm = w0_init_mm * initial_k;

    // 3. 将切片宽度转换为角度 (deg)
    // 弧长 = 半径 × 角度(弧度) => 角度(度) = 弧长 / 半径 × (180/π)
    double slice_angle_init_deg = (slice_width_init_mm / cylinder_radius_mm) * (180.0 / M_PI);

    ROS_WARN("----------- Manual Mode Before Optimization -----------");
    ROS_INFO("[Before] Spray Width (d): %.2f mm", spray_width);
    ROS_INFO("[Before] Overlap Ratio (r): %.4f (%.1f%%)", initial_r, initial_r * 100.0);
    ROS_INFO("[Before] Spray Passes per Position (k): %d", initial_k);
    ROS_INFO("[Before] w0 = d * (1 - r) = %.2f mm", w0_init_mm);
    ROS_INFO("[Before] Slice Width = w0 * k = %.2f mm (%.4f deg)", slice_width_init_mm, slice_angle_init_deg);
    ROS_INFO("[Before] Cylinder Radius: %.2f mm", cylinder_radius_mm);

    // 4. 计算需要多少个切片来覆盖180度
    // N = round(180 / slice_angle_init_deg)
    int N_slices = static_cast<int>(std::round(TARGET_COVERAGE_ANGLE / slice_angle_init_deg));
    if (N_slices < 1) N_slices = 1;

    ROS_INFO("[Step1] Number of slices: N = round(%.2f / %.4f) = %d", TARGET_COVERAGE_ANGLE, slice_angle_init_deg,
             N_slices);

    // 5. 调整切片角度使得 N × slice_angle = 180（精确覆盖）
    double slice_angle_adjusted_deg = TARGET_COVERAGE_ANGLE / static_cast<double>(N_slices);

    // 6. 将调整后的角度转换回切片宽度 (mm)
    double slice_width_adjusted_mm = slice_angle_adjusted_deg * (M_PI / 180.0) * cylinder_radius_mm;

    ROS_INFO("[Step2] Adjusted slice angle: %.4f deg (%.2f mm)", slice_angle_adjusted_deg, slice_width_adjusted_mm);

    // 7. 反推调整后的 w0 和 r
    // slice_width = w0 * k => w0 = slice_width / k
    int    k_adjusted     = initial_k; // 保持 k 不变
    double w0_adjusted_mm = slice_width_adjusted_mm / static_cast<double>(k_adjusted);

    // w0 = d * (1 - r) => r = 1 - w0 / d
    double r_adjusted = 1.0 - (w0_adjusted_mm / spray_width);

    ROS_INFO("[Step3] w0_adjusted = slice_width / k = %.2f / %d = %.2f mm", slice_width_adjusted_mm, k_adjusted,
             w0_adjusted_mm);
    ROS_INFO("[Step4] r_adjusted = 1 - w0 / d = 1 - %.2f / %.2f = %.4f (%.1f%%)", w0_adjusted_mm, spray_width,
             r_adjusted, r_adjusted * 100.0);

    // 8. 约束检查：如果 r < 原始值，需要增加 k
    if (r_adjusted < initial_r - 1e-6) {
        ROS_WARN("Constraint: r_adjusted (%.4f) < r_initial (%.4f), need to increase k", r_adjusted, initial_r);

        // 增加 k 直到 r >= r_initial
        while (r_adjusted < initial_r - 1e-6 && k_adjusted < 100) {
            k_adjusted++;
            w0_adjusted_mm = slice_width_adjusted_mm / static_cast<double>(k_adjusted);
            r_adjusted     = 1.0 - (w0_adjusted_mm / spray_width);
        }
        ROS_INFO("After adjustment: k = %d, w0 = %.2f mm, r = %.4f", k_adjusted, w0_adjusted_mm, r_adjusted);
    }

    // 9. 约束检查：r 不能 >= 1.0
    if (r_adjusted >= 1.0 - 1e-6) {
        ROS_WARN("Fallback: r_adjusted >= 1.0, using initial values");
        k_adjusted              = initial_k;
        w0_adjusted_mm          = w0_init_mm;
        r_adjusted              = initial_r;
        slice_width_adjusted_mm = slice_width_init_mm;
    }

    // 设置输出参数
    final_k              = k_adjusted;
    final_r              = r_adjusted;
    final_w0_mm          = w0_adjusted_mm;
    final_slice_width_mm = slice_width_adjusted_mm;

    ROS_WARN("----------- Manual Mode After Optimization -----------");
    ROS_INFO("[Result] Adjusted k: %d", final_k);
    ROS_INFO("[Result] Adjusted r: %.4f (%.1f%%)", final_r, final_r * 100.0);
    ROS_INFO("[Result] Adjusted w0: %.2f mm", final_w0_mm);
    ROS_INFO("[Result] Final Slice Width: %.2f mm", final_slice_width_mm);
    ROS_INFO("[Result] Number of Slices: %d", N_slices);
    ROS_INFO("=====================================================");

    return N_slices;
}

// 核心执行函数 (将您的测试代码封装)

PlanningOutputs CylinderFitter::execute_slice_planning(const PlanningInputs &inputs) {
    PlanningOutputs outputs;

    // 初始化输出变量
    outputs.max_coverage_angle  = 0.0;
    outputs.optimal_slice_angle = 0.0;
    outputs.final_overlap_ratio = inputs.r_initial;
    outputs.k_spray_passes      = 0;
    outputs.w0_adjusted_mm      = 0.0;
    outputs.w0_adjusted_deg     = 0.0;
    outputs.n_positions         = 0;
    outputs.status_message      = "";

    try {
        // 1. 调用几何约束计算
        auto result = calculate_max_slice_angle(inputs.robot_R_arm, inputs.P_cyl, inputs.V_axis, inputs.r_cyl,
                                                inputs.D_offset, inputs.H_region);
        outputs.max_coverage_angle = result.first;
        outputs.status_message     = result.second;

        if (outputs.max_coverage_angle <= 0.0) {
            // 几何计算失败或无法触及
            ROS_ERROR("Geometric calculation failed or cannot reach the cylinder surface.");
            outputs.optimal_slice_angle = 0.0;
            outputs.final_overlap_ratio = inputs.r_initial;
            return outputs;
        }
        ROS_INFO("Max Coverage Angle (theta_max) calculated: %.2f deg before optimization", outputs.max_coverage_angle);

        // 2. 调用喷涂角度优化
        outputs.optimal_slice_angle =
            calculate_optimal_slice_angle(outputs.max_coverage_angle, inputs.d_spray, inputs.r_initial, inputs.r_cyl,
                                          inputs.D_offset, outputs.final_overlap_ratio);

        // 从成员变量获取优化结果
        outputs.k_spray_passes  = last_k_spray_passes_;
        outputs.w0_adjusted_mm  = last_w0_adjusted_mm_;
        outputs.w0_adjusted_deg = last_w0_adjusted_deg_;
        outputs.n_positions     = last_n_positions_;

        outputs.status_message += "; Optimization done";

        // 打印结果（可选，用于调试）
        ROS_INFO("--- Slice Planning Result ---");
        ROS_INFO("Status: %s", outputs.status_message.c_str());
        ROS_INFO("Max Coverage Angle (theta_max): %.2f deg", outputs.max_coverage_angle);
        ROS_INFO("Optimal Slice Width (w): %.2f deg", outputs.optimal_slice_angle);
        ROS_INFO("Final Overlap Ratio (r): %.4f (%.1f%%)", outputs.final_overlap_ratio,
                 outputs.final_overlap_ratio * 100.0);

        if (outputs.final_overlap_ratio > inputs.r_initial) {
            ROS_WARN("Overlap ratio increased from %.1f%% to %.1f%% to achieve integer multiple coverage",
                     inputs.r_initial * 100.0, outputs.final_overlap_ratio * 100.0);
        }

    } catch (const std::exception &e) {
        outputs.max_coverage_angle  = 0.0;
        outputs.optimal_slice_angle = 0.0;
        outputs.final_overlap_ratio = inputs.r_initial;
        outputs.status_message      = "exe error: " + std::string(e.what());
    }

    return outputs;
}

/// @brief 对标准圆柱表面切片计算
/// @param angle_range_deg 角度范围（度）
/// @param slice_height_mm 切片高度（毫米）
/// @param slice_width_mm 切片宽度（毫米）
void CylinderFitter::sliceCylinder(double angle_range_deg, double slice_height_mm, double slice_width_mm) {
    slices_data_.clear();

    if (!hasModel()) return;
    ROS_WARN("Starting cylinder slicing======ANGEL_RANGE_DEG:[%.3F], SLICE_HEIGHT_MM:[%.3F], SLICE_WIDTH_MM:[%.3F], "
             "WORK_DIS:[%.3F]",
             angle_range_deg, slice_height_mm, slice_width_mm, fitting_params_.slice_distance_);
    const auto &opt = slice_opts_;

    Eigen::Vector3f bottom_center_mm = cylinder_model_.bottom_center * 1000.0f;

    // ========== 轴向选择：使用理想轴向或拟合轴向 ==========
    Eigen::Vector3f axis_fitted = cylinder_model_.axis;
    Eigen::Vector3f axis;
    Eigen::Vector3f ideal_axis(0.0f, 0.0f, 1.0f);

    // 计算拟合轴向与理想竖直轴向的夹角
    double cos_angle      = std::abs(axis_fitted.normalized().dot(ideal_axis));
    double angle_diff_deg = std::acos(std::min(1.0, (double)cos_angle)) * 180.0 / M_PI;

    if (fitting_params_.use_ideal_axis_ && angle_diff_deg < fitting_params_.axis_angle_threshold_deg_) {
        // 拟合轴向接近竖直，使用理想轴向
        axis = ideal_axis;
        ROS_INFO("[Slice] Axis correction ENABLED: Using IDEAL vertical axis (0, 0, 1)");
        ROS_INFO("[Slice] Fitted axis: [%.6f, %.6f, %.6f]", axis_fitted.x(), axis_fitted.y(), axis_fitted.z());
        ROS_INFO("[Slice] Deviation from vertical: %.4f deg (threshold: %.2f deg)", angle_diff_deg,
                 fitting_params_.axis_angle_threshold_deg_);
    } else {
        // 使用原始拟合轴向
        axis = axis_fitted;
        if (fitting_params_.use_ideal_axis_) {
            ROS_WARN("[Slice] Axis correction DISABLED: Deviation %.4f deg exceeds threshold %.2f deg", angle_diff_deg,
                     fitting_params_.axis_angle_threshold_deg_);
        } else {
            ROS_INFO("[Slice] Using FITTED axis [%.6f, %.6f, %.6f] (ideal axis disabled)", axis.x(), axis.y(),
                     axis.z());
        }
    }
    // ==========================================================

    Eigen::Vector3f z         = axis;
    double          radius_mm = cylinder_model_.radius * 1000.0;
    double          height_mm = cylinder_model_.height * 1000.0;

    // 工作点参考方向：X轴负方向（-1，0，0）
    //         Y轴负方向 (0，-1，0）
    Eigen::Vector3f ref(0.0f, -1.0f, 0.0f);
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
    // id 从低到高自增，j 采用 S 型扫描顺序
    // for (int i = 0; i < num_height_slices; ++i) {
    // id 从高到低自增，j 采用 S 型扫描顺序
    for (int i = num_height_slices - 1; i >= 0; --i) {
        double h_mm = (i + 0.5) * (height_mm / num_height_slices);

        std::vector<Eigen::Vector3f> layer_centers_mm; // 收集该高度层的切片中心

        // S 型顺序：偶数层J正向，奇数层J反向
        for (int j_it = 0; j_it < num_width_slices; ++j_it) {

            int j = (i % 2 == 0) ? j_it : (num_width_slices - 1 - j_it);

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

                // 球（切片中心点）
                viewer_->addSphere(pcl::PointXYZ(slice_center_mm.x() / 1000.0f, slice_center_mm.y() / 1000.0f,
                                                 slice_center_mm.z() / 1000.0f),
                                   10.0 / 1000.0, 0.0, 0.0, 1.0, sphere_id);
                slice_shape_ids_.push_back(sphere_id);

                // 文本
                // std::ostringstream oss;
                // oss << slice_center_mm.x();
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
            // ROS_INFO("[Slice] Slice ID %d: Center at [%.3f, %.3f, %.3f] mm", sd.id, sd.center_mm.x(),
            // sd.center_mm.y(),
            //          sd.center_mm.z());

            // 计算工作点 sd.work_mm
            // 计算从圆心 C 指向表面点 A 的向量 v = A - C
            Eigen::Vector3f v    = sd.center_mm - sd.circle_center_mm;
            double          norm = v.norm();
            if (norm < 1e-6) {
                // 退化情况：如果圆心和表面点重合，通常不会发生，但作为安全措施
                // 默认沿 X 轴负方向偏移
                ROS_WARN("[Slice] Slice ID %d: Center coincides with circle center, fallback offset applied.", sd.id);
                sd.work_mm = sd.center_mm + Eigen::Vector3f(fitting_params_.slice_distance_, 0.0f, 0.0f);
            } else {
                // 2. 计算单位方向向量 dir
                Eigen::Vector3f dir = v / norm;
                // 3. 计算工作点 B = A + dir * offset
                // 使用 slice_distance_（喷枪到表面距离），而非 virtual_base_to_surface_（物理基座到表面距离）
                // 喷枪(TCP)应位于圆柱表面外侧 slice_distance_ 处
                sd.work_mm = sd.center_mm + dir * fitting_params_.slice_distance_;
            }

            { // 可视化工作点
                std::lock_guard<std::mutex> lock(viewer_mutex_);
                // 橙色工作点球
                viewer_->addSphere(
                    pcl::PointXYZ(sd.work_mm.x() / 1000.0f, sd.work_mm.y() / 1000.0f, sd.work_mm.z() / 1000.0f),
                    25.0 / 1000.0, 1.0, 0.5, 0.0, "slice_workpoint_" + std::to_string(sd.id));
                slice_shape_ids_.push_back("slice_workpoint_" + std::to_string(sd.id));

                // 从切片表面中心到工作点的连线（青色），用于验证工作点对应关系
                // std::string link_id = "slice_link_" + std::to_string(sd.id);
                // viewer_->addLine(
                //     pcl::PointXYZ(sd.center_mm.x() / 1000.0f, sd.center_mm.y() / 1000.0f, sd.center_mm.z() /
                //     1000.0f), pcl::PointXYZ(sd.work_mm.x() / 1000.0f, sd.work_mm.y() / 1000.0f, sd.work_mm.z() /
                //     1000.0f), 0.0, 1.0, 1.0, link_id);  // 青色连线
                // viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2.0, link_id);
                // slice_shape_ids_.push_back(link_id);
            }

            // 该切片的角度跨度 (总角度 / 宽度切片数)
            double raw_angle_deg = (angle_range / num_width_slices) * 180.0 / M_PI;
            sd.angle_deg         = std::round(raw_angle_deg * 1000.0) / 1000.0;
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

    // 绘制工作点轨迹圆（绿色，在底部绘制一个圆）
    {
        std::lock_guard<std::mutex> lock(viewer_mutex_);
        int                         ring_samples = 96;
        // 使用 slice_distance_ 与 sd.work_mm 计算一致（TCP = 圆柱表面 + 喷涂距离）
        double work_radius_mm = radius_mm + fitting_params_.slice_distance_;
        double h_mm           = height_mm / 2.0; // 在中间高度绘制
        for (int k = 0; k < ring_samples; ++k) {
            double          th0 = 2.0 * M_PI * k / ring_samples;
            double          th1 = 2.0 * M_PI * (k + 1) / ring_samples;
            Eigen::Vector3f p0_mm =
                bottom_center_mm + h_mm * z + work_radius_mm * (std::cos(th0) * x + std::sin(th0) * y);
            Eigen::Vector3f p1_mm =
                bottom_center_mm + h_mm * z + work_radius_mm * (std::cos(th1) * x + std::sin(th1) * y);
            std::string ring_id = "work_point_ring_" + std::to_string(k);
            viewer_->addLine(pcl::PointXYZ(p0_mm.x() / 1000.0f, p0_mm.y() / 1000.0f, p0_mm.z() / 1000.0f),
                             pcl::PointXYZ(p1_mm.x() / 1000.0f, p1_mm.y() / 1000.0f, p1_mm.z() / 1000.0f), 0.0, 1.0,
                             0.0, ring_id); // 绿色 (工作点圆)
            slice_line_ids_.push_back(ring_id);
        }
    }

    ROS_WARN("[Slice] Generate Slice:Height: (%f,%f) mm, height_total=%d, width_total=%d, surface=%s", slice_height_mm,
             slice_width_mm, num_height_slices, num_width_slices, (opt.surface_ ? "ON" : "OFF"));

    // ========== 计算并存储切片统计信息 ==========
    // 使用传入的实际切片宽度和切片高度计算
    // slice_width_mm: 传入的切片宽度（已经是优化后的实际切片弧长）
    // slice_height_mm: 传入的工作区域单次喷幅高度

    // 单个切片面积 (m^2) = 切片宽 * 切片高
    double single_slice_area_m2 = slice_width_mm * slice_height_mm / 1e6;
    // 切片总数
    int total_slice_count = num_height_slices * num_width_slices;
    // 总面积 (m^2)
    double total_slice_area_m2                    = single_slice_area_m2 * total_slice_count;
    cylinder_fitter_outputs.slice_count_          = total_slice_count;
    cylinder_fitter_outputs.single_slice_area_m2_ = single_slice_area_m2;
    cylinder_fitter_outputs.total_slice_area_m2_  = total_slice_area_m2;
    ROS_INFO("[Slice] Statistics: slice_count=%d, single_slice_area=%.2f m^2 (%.4f mm^2) [width=%.4f mm, height=%.2f "
             "mm], total_area=%.4f m^2 (%.4f mm^2)",
             total_slice_count, single_slice_area_m2, single_slice_area_m2 * 1e6, slice_width_mm, slice_height_mm,
             total_slice_area_m2, total_slice_area_m2 * 1e6);

    /*
    if (opt.print_centers_) {
        //  打印所有切片中心
        ROS_INFO("[Slice Centers] Total=%zu", slice_centers_mm_.size());
        for (size_t k = 0; k < slice_centers_mm_.size(); ++k) {
            int         i = static_cast<int>(k / num_width_slices);
            int         j = static_cast<int>(k % num_width_slices);
            const auto &c = slice_centers_mm_[k];
            ROS_INFO("  Slice (%d,%d): [%.2f mm, %.2f mm, %.2f mm]", i, j, c.x(), c.y(), c.z());
        }
    }
    */
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

    // 可视化切片中心与工作点（包含连接线
    // int work_sphere_count = 0;
    // for (const auto &slice : slices_data_) {
    //     // 1. 工作点（work_mm）：蓝色球体
    //     std::string work_sphere_id = "slice_work_" + std::to_string(work_sphere_count);
    //     viewer_->addSphere(
    //         pcl::PointXYZ(slice.work_mm.x() / 1000.0f, slice.work_mm.y() / 1000.0f, slice.work_mm.z() / 1000.0f),
    //         0.010, 0.0, 0.0, 1.0, work_sphere_id); // 蓝色 (Blue: 0, 0, 1)

    //     // 连接线：从表面中心（center_mm）到工作点（work_mm）
    //     // 白色虚线，表示喷枪的轴线方向。
    //     std::string connect_line_id = "slice_connect_" + std::to_string(work_sphere_count);
    //     viewer_->addLine(
    //         pcl::PointXYZ(slice.center_mm.x() / 1000.0f, slice.center_mm.y() / 1000.0f, slice.center_mm.z() /
    //         1000.0f), pcl::PointXYZ(slice.work_mm.x() / 1000.0f, slice.work_mm.y() / 1000.0f, slice.work_mm.z() /
    //         1000.0f), 1.0, 1.0, 1.0, connect_line_id); // 白色 (White: 1, 1, 1)

    //     work_sphere_count++;
    // }

    // 源点到圆柱中点连线
    Eigen::Vector3f source_point(0, 0, 0);
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

    // 源点 Y 轴无限延长线
    // Y 轴正方向
    Eigen::Vector3f y_pos_extend = source_point + Eigen::Vector3f(0.0f, 100.0f, 0.0f); // 100m 沿 +Y
    viewer_->addLine(pcl::PointXYZ(source_point.x(), source_point.y(), source_point.z()),
                     pcl::PointXYZ(y_pos_extend.x(), y_pos_extend.y(), y_pos_extend.z()), 0.0, 1.0, 0.0,
                     "source_y_positive"); // 绿色（+Y 方向）

    // 计算距离
    double dist_3d = (cylinder_model_.geo_center - source_point).norm() * 1000.0; // 米 -> mm
    double dist_xy = (Eigen::Vector3f(cylinder_model_.geo_center.x(), cylinder_model_.geo_center.y(), 0.0f) -
                      Eigen::Vector3f(source_point.x(), source_point.y(), 0.0f))
                         .norm() *
                     1000.0; // 米 -> mm
    ROS_INFO("[Distance] Geo center: %f mm, Bottom center: %f mm", dist_3d, dist_xy);
}

/// @brief 可视化虚拟机械臂原点、圆柱体、球形工作空间和覆盖角度范围
/// @param virtual_base_pos_mm
/// @param robot_max_reach_mm
/// @param coverage_angle_deg
/// @param work_height_center_mm
/// @param work_height_range_mm
// 元素	            颜色	说明
// 虚拟机械臂原点	    🔴  红色大球	显示虚拟机械臂基座位置
// 虚拟原点到圆柱连线	🟣  紫色	    显示从虚拟原点到圆柱几何中心的连线
// 球形工作空间	       🔴   淡红色线框	机械臂最大臂长形成的球体（多圆环表示）
// 有效臂展圆   	   🟡   黄色	    在工作高度上下限处的有效水平臂展范围
// 覆盖区域	           🟢   绿色点云	圆柱表面上被覆盖的扇形区域
// 覆盖边界线	       🟢   亮绿色	    覆盖角度的左右边界线
// 工作高度范围	       🟣   品红色	    工作高度的上下限
// 臂展射线	           🔵   青色	    从虚拟原点到覆盖边界工作点的射线
void CylinderFitter::visualizeCoverageRange(const Eigen::Vector3d &virtual_base_pos_mm, double robot_max_reach_mm,
                                            double coverage_angle_deg, double work_height_center_mm,
                                            double work_height_range_mm) {
    if (!hasModel()) return;

    std::lock_guard<std::mutex> lock(viewer_mutex_);

    // 单位转换：mm -> m（PCL可视化使用米）
    double          scale                = 0.001;
    Eigen::Vector3f virtual_base_m       = (virtual_base_pos_mm * scale).cast<float>();
    double          robot_reach_m        = robot_max_reach_mm * scale;
    double          work_height_center_m = work_height_center_mm * scale;
    double          work_height_range_m  = work_height_range_mm * scale;

    // 圆柱参数
    Eigen::Vector3f geo_center_m = cylinder_model_.geo_center;
    Eigen::Vector3f axis_unit    = cylinder_model_.axis.normalized();
    double          radius_m     = cylinder_model_.radius;

    // 1. 虚拟机械臂原点（大红球）
    viewer_->addSphere(pcl::PointXYZ(virtual_base_m.x(), virtual_base_m.y(), virtual_base_m.z()), 0.05, 1.0, 0.0, 0.0,
                       "virtual_robot_base");

    // 添加文字标注
    viewer_->addText3D("Virtual Base", pcl::PointXYZ(virtual_base_m.x(), virtual_base_m.y(), virtual_base_m.z() + 0.1),
                       0.05, 1.0, 0.0, 0.0, "virtual_base_label");

    // 2. 虚拟原点到圆柱几何中心的连线（紫色）
    viewer_->addLine(pcl::PointXYZ(virtual_base_m.x(), virtual_base_m.y(), virtual_base_m.z()),
                     pcl::PointXYZ(geo_center_m.x(), geo_center_m.y(), geo_center_m.z()), 1.0, 0.0, 1.0,
                     "virtual_base_to_cylinder");

    // 3.  球形工作空间（半透明红色线框）
    // 用多个圆环表示球体
    int n_rings           = 12;
    int n_points_per_ring = 36;
    for (int i = 0; i <= n_rings; ++i) {
        double phi         = M_PI * i / n_rings; // 从0到π
        double ring_radius = robot_reach_m * std::sin(phi);
        double ring_z      = robot_reach_m * std::cos(phi);

        pcl::PointCloud<pcl::PointXYZ>::Ptr ring(new pcl::PointCloud<pcl::PointXYZ>);
        for (int j = 0; j <= n_points_per_ring; ++j) {
            double theta = 2.0 * M_PI * j / n_points_per_ring;
            float  x     = virtual_base_m.x() + ring_radius * std::cos(theta);
            float  y     = virtual_base_m.y() + ring_radius * std::sin(theta);
            float  z     = virtual_base_m.z() + ring_z;
            ring->push_back(pcl::PointXYZ(x, y, z));
        }

        std::string                                                     ring_id = "sphere_ring_h_" + std::to_string(i);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ring_color(ring, 255, 100, 100);
        viewer_->addPointCloud<pcl::PointXYZ>(ring, ring_color, ring_id);
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, ring_id);
    }

    // 垂直方向的圆环
    for (int i = 0; i < 6; ++i) {
        double theta_base = M_PI * i / 6;

        pcl::PointCloud<pcl::PointXYZ>::Ptr v_ring(new pcl::PointCloud<pcl::PointXYZ>);
        for (int j = 0; j <= n_points_per_ring; ++j) {
            double phi = 2.0 * M_PI * j / n_points_per_ring;
            float  x   = virtual_base_m.x() + robot_reach_m * std::sin(phi) * std::cos(theta_base);
            float  y   = virtual_base_m.y() + robot_reach_m * std::sin(phi) * std::sin(theta_base);
            float  z   = virtual_base_m.z() + robot_reach_m * std::cos(phi);
            v_ring->push_back(pcl::PointXYZ(x, y, z));
        }

        std::string                                                     ring_id = "sphere_ring_v_" + std::to_string(i);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ring_color(v_ring, 255, 100, 100);
        viewer_->addPointCloud<pcl::PointXYZ>(v_ring, ring_color, ring_id);
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, ring_id);
    }

    // 4.  有效水平臂展圆（在工作高度处）
    double z_work_max  = work_height_center_m + work_height_range_m / 2.0;
    double z_work_min  = work_height_center_m - work_height_range_m / 2.0;
    double delta_z_max = std::max(std::abs(z_work_max - virtual_base_m.z()), std::abs(z_work_min - virtual_base_m.z()));

    double r_eff_sq = robot_reach_m * robot_reach_m - delta_z_max * delta_z_max;
    double r_eff_m  = (r_eff_sq > 0) ? std::sqrt(r_eff_sq) : 0.0;

    // 在最远工作高度处画有效臂展圆（黄色）
    pcl::PointCloud<pcl::PointXYZ>::Ptr eff_ring_top(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr eff_ring_bottom(new pcl::PointCloud<pcl::PointXYZ>);
    for (int j = 0; j <= 72; ++j) {
        double theta = 2.0 * M_PI * j / 72;
        float  x     = virtual_base_m.x() + r_eff_m * std::cos(theta);
        float  y     = virtual_base_m.y() + r_eff_m * std::sin(theta);
        eff_ring_top->push_back(pcl::PointXYZ(x, y, z_work_max));
        eff_ring_bottom->push_back(pcl::PointXYZ(x, y, z_work_min));
    }

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> eff_color_top(eff_ring_top, 255, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> eff_color_bottom(eff_ring_bottom, 255, 200, 0);
    viewer_->addPointCloud<pcl::PointXYZ>(eff_ring_top, eff_color_top, "eff_reach_ring_top");
    viewer_->addPointCloud<pcl::PointXYZ>(eff_ring_bottom, eff_color_bottom, "eff_reach_ring_bottom");
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "eff_reach_ring_top");
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3,
                                              "eff_reach_ring_bottom");

    // 5.  覆盖角度范围（绿色扇形区域）
    // 计算从虚拟原点到圆柱中心的方向（作为覆盖角度的中心方向）
    Eigen::Vector3f to_cylinder = geo_center_m - virtual_base_m;
    to_cylinder.z()             = 0; // 投影到XY平面
    to_cylinder.normalize();

    double half_angle_rad = coverage_angle_deg * M_PI / 360.0;                  // 半角
    double r_task_m       = radius_m + fitting_params_.slice_distance_ * scale; // 工作半径

    // 在圆柱表面绘制覆盖区域（绿色）
    pcl::PointCloud<pcl::PointXYZ>::Ptr coverage_surface(new pcl::PointCloud<pcl::PointXYZ>);

    // 构建圆柱局部坐标系
    Eigen::Vector3f cyl_x = axis_unit.unitOrthogonal();
    Eigen::Vector3f cyl_y = axis_unit.cross(cyl_x);

    // 计算覆盖起始角度
    // 从圆柱中心到虚拟原点的方向
    Eigen::Vector3f from_cyl_to_base = virtual_base_m - geo_center_m;
    from_cyl_to_base = from_cyl_to_base - from_cyl_to_base.dot(axis_unit) * axis_unit; // 投影到垂直于轴线的平面
    from_cyl_to_base.normalize();

    double base_angle = std::atan2(from_cyl_to_base.dot(cyl_y), from_cyl_to_base.dot(cyl_x));

    // 绘制覆盖扇形
    int n_angle_steps  = 30;
    int n_height_steps = 15;
    for (int h = 0; h <= n_height_steps; ++h) {
        double z_local = z_work_min + (z_work_max - z_work_min) * h / n_height_steps;
        // 找到该高度对应的轴线上的点
        double          t          = (z_local - geo_center_m.z()) / axis_unit.z();
        Eigen::Vector3f axis_point = geo_center_m + t * axis_unit;

        for (int a = 0; a <= n_angle_steps; ++a) {
            double angle_offset = -half_angle_rad + 2.0 * half_angle_rad * a / n_angle_steps;
            double theta        = base_angle + angle_offset;

            Eigen::Vector3f surface_point = axis_point + radius_m * (std::cos(theta) * cyl_x + std::sin(theta) * cyl_y);
            coverage_surface->push_back(pcl::PointXYZ(surface_point.x(), surface_point.y(), surface_point.z()));
        }
    }

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> coverage_color(coverage_surface, 0, 255, 0);
    viewer_->addPointCloud<pcl::PointXYZ>(coverage_surface, coverage_color, "coverage_surface");
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "coverage_surface");

    // 6.  覆盖边界线（亮绿色）
    // 左边界
    pcl::PointCloud<pcl::PointXYZ>::Ptr left_boundary(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr right_boundary(new pcl::PointCloud<pcl::PointXYZ>);

    for (int h = 0; h <= n_height_steps * 2; ++h) {
        double          z_local    = z_work_min + (z_work_max - z_work_min) * h / (n_height_steps * 2);
        double          t          = (z_local - geo_center_m.z()) / axis_unit.z();
        Eigen::Vector3f axis_point = geo_center_m + t * axis_unit;

        double theta_left  = base_angle - half_angle_rad;
        double theta_right = base_angle + half_angle_rad;

        Eigen::Vector3f left_pt = axis_point + radius_m * (std::cos(theta_left) * cyl_x + std::sin(theta_left) * cyl_y);
        Eigen::Vector3f right_pt =
            axis_point + radius_m * (std::cos(theta_right) * cyl_x + std::sin(theta_right) * cyl_y);

        left_boundary->push_back(pcl::PointXYZ(left_pt.x(), left_pt.y(), left_pt.z()));
        right_boundary->push_back(pcl::PointXYZ(right_pt.x(), right_pt.y(), right_pt.z()));
    }

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> boundary_color(left_boundary, 100, 255, 100);
    viewer_->addPointCloud<pcl::PointXYZ>(left_boundary, boundary_color, "left_boundary");
    viewer_->addPointCloud<pcl::PointXYZ>(right_boundary, boundary_color, "right_boundary");
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "left_boundary");
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "right_boundary");

    // 7.  工作高度范围线（品红色）
    viewer_->addLine(pcl::PointXYZ(geo_center_m.x(), geo_center_m.y(), z_work_min),
                     pcl::PointXYZ(geo_center_m.x(), geo_center_m.y(), z_work_max), 1.0, 0.0, 1.0, "work_height_range");

    // 8.  从虚拟原点到覆盖边界的射线（显示臂展约束）
    // 计算覆盖边界在工作半径上的点
    double          z_mid          = work_height_center_m;
    double          t_mid          = (z_mid - geo_center_m.z()) / axis_unit.z();
    Eigen::Vector3f axis_point_mid = geo_center_m + t_mid * axis_unit;

    double theta_left  = base_angle - half_angle_rad;
    double theta_right = base_angle + half_angle_rad;

    Eigen::Vector3f left_work_pt =
        axis_point_mid + r_task_m * (std::cos(theta_left) * cyl_x + std::sin(theta_left) * cyl_y);
    Eigen::Vector3f right_work_pt =
        axis_point_mid + r_task_m * (std::cos(theta_right) * cyl_x + std::sin(theta_right) * cyl_y);

    viewer_->addLine(pcl::PointXYZ(virtual_base_m.x(), virtual_base_m.y(), virtual_base_m.z()),
                     pcl::PointXYZ(left_work_pt.x(), left_work_pt.y(), left_work_pt.z()), 0.0, 1.0, 1.0,
                     "reach_ray_left");
    viewer_->addLine(pcl::PointXYZ(virtual_base_m.x(), virtual_base_m.y(), virtual_base_m.z()),
                     pcl::PointXYZ(right_work_pt.x(), right_work_pt.y(), right_work_pt.z()), 0.0, 1.0, 1.0,
                     "reach_ray_right");

    // 9.  虚拟基座轨迹圆（紫色，以圆柱轴线在Z=0的投影为圆心）
    // 虚拟基座在Z=0平面，所以紫色圆也应该在Z=0平面
    // 圆心是圆柱几何中心在XY平面的投影
    double          virtual_base_radius_m = radius_m + fitting_params_.virtual_base_to_surface_ * scale;
    Eigen::Vector3f cyl_center_xy(geo_center_m.x(), geo_center_m.y(), 0.0f); // 圆柱中心的XY投影，Z=0
    pcl::PointCloud<pcl::PointXYZ>::Ptr virtual_base_ring(new pcl::PointCloud<pcl::PointXYZ>);
    for (int j = 0; j <= 72; ++j) {
        double theta = 2.0 * M_PI * j / 72;
        float  x     = cyl_center_xy.x() + virtual_base_radius_m * std::cos(theta);
        float  y     = cyl_center_xy.y() + virtual_base_radius_m * std::sin(theta);
        float  z     = 0.0f; // Z=0 平面
        virtual_base_ring->push_back(pcl::PointXYZ(x, y, z));
    }
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> vb_ring_color(virtual_base_ring, 200, 0, 255);
    viewer_->addPointCloud<pcl::PointXYZ>(virtual_base_ring, vb_ring_color, "virtual_base_trajectory_ring");
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3,
                                              "virtual_base_trajectory_ring");

    // 10.  工作点轨迹圆（绿色，以圆柱轴线在Z=0的投影为圆心）
    // 工作点半径 = 圆柱半径 + slice_distance_（与 sd.work_mm 计算一致，TCP 在喷涂距离处）
    double work_point_radius_m = radius_m + fitting_params_.slice_distance_ * scale;
    pcl::PointCloud<pcl::PointXYZ>::Ptr work_point_ring(new pcl::PointCloud<pcl::PointXYZ>);
    for (int j = 0; j <= 72; ++j) {
        double theta = 2.0 * M_PI * j / 72;
        float  x     = cyl_center_xy.x() + work_point_radius_m * std::cos(theta);
        float  y     = cyl_center_xy.y() + work_point_radius_m * std::sin(theta);
        float  z     = 0.0f; // Z=0 平面
        work_point_ring->push_back(pcl::PointXYZ(x, y, z));
    }
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> wp_ring_color(work_point_ring, 0, 255, 0);
    viewer_->addPointCloud<pcl::PointXYZ>(work_point_ring, wp_ring_color, "work_point_trajectory_ring");
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3,
                                              "work_point_trajectory_ring");

    // 输出调试信息
    ROS_INFO("======== Coverage Visualization ========");
    ROS_INFO("Virtual Base (m): [%.3f, %.3f, %.3f]", virtual_base_m.x(), virtual_base_m.y(), virtual_base_m.z());
    ROS_INFO("Robot Max Reach: %.2f m", robot_reach_m);
    ROS_INFO("Effective Reach at Z=%.2f m: %.3f m", delta_z_max + virtual_base_m.z(), r_eff_m);
    ROS_INFO("Coverage Angle: %.2f deg (half: %.2f deg)", coverage_angle_deg, coverage_angle_deg / 2.0);
    ROS_INFO("Work Height Range: [%.2f, %.2f] m", z_work_min, z_work_max);
    ROS_INFO("Virtual Base Radius: %.3f m (R=%.3f + %.3f)", virtual_base_radius_m, radius_m,
             fitting_params_.virtual_base_to_surface_ * scale);
    ROS_INFO("Work Point Radius: %.3f m (R=%.3f + %.3f)", work_point_radius_m, radius_m,
             fitting_params_.slice_distance_ * scale);
    ROS_INFO("=========================================");
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

    // 计算实际使用的轴向（与 sliceCylinder 逻辑一致）
    Eigen::Vector3f ideal_axis(0.0f, 0.0f, 1.0f);
    Eigen::Vector3f axis_used;
    double          cos_angle      = std::abs(axis_norm.dot(ideal_axis));
    double          angle_diff_deg = std::acos(std::min(1.0, (double)cos_angle)) * 180.0 / M_PI;

    if (fitting_params_.use_ideal_axis_ && angle_diff_deg < fitting_params_.axis_angle_threshold_deg_) {
        axis_used = ideal_axis;
    } else {
        axis_used = axis_norm;
    }

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
    ROS_INFO("Axis (fitted):  [%.4f, %.4f, %.4f]", axis_norm.x(), axis_norm.y(), axis_norm.z());
    if (fitting_params_.use_ideal_axis_ && angle_diff_deg < fitting_params_.axis_angle_threshold_deg_) {
        ROS_INFO("Axis (used):    [%.4f, %.4f, %.4f] (ideal, deviation %.2f deg)", axis_used.x(), axis_used.y(),
                 axis_used.z(), angle_diff_deg);
    } else {
        ROS_INFO("Axis (used):    [%.4f, %.4f, %.4f] (fitted)", axis_used.x(), axis_used.y(), axis_used.z());
    }
    ROS_INFO("Radius: %.2f mm", cylinder_model_.radius * 1000.0);
    ROS_INFO("Angle: %.2f deg", cylinder_model_.valid ? angle_deg : 0.0);
    ROS_INFO("Height: %.2f mm", cylinder_model_.height * 1000.0);

    // // ========== 距离计算（与其他函数使用一致的变量名和计算方式）==========
    // // 圆柱半径 (mm) - 统一使用 cylinder_radius_mm
    // double cylinder_radius_mm = cylinder_model_.radius * 1000.0;

    // // 圆柱几何中心 (mm) - 统一使用 geo_center_mm
    // Eigen::Vector3d geo_center_mm = cylinder_model_.geo_center.cast<double>() * 1000.0;

    // // 虚拟基座到轴线的距离 = R_cyl + virtual_base_to_surface_ (与 calculateVirtualBaseP_cyl 一致)
    // double virtual_base_dist = cylinder_radius_mm + fitting_params_.virtual_base_to_surface_;

    // // 计算虚拟机械臂原点位置 (与 calculateVirtualBaseP_cyl 和 run_with_rslidar 一致)
    // Eigen::Vector3d geo_center_xy(geo_center_mm.x(), geo_center_mm.y(), 0.0);
    // Eigen::Vector3d virtual_base_pos;
    // if (geo_center_xy.norm() > 1e-6) {
    //     Eigen::Vector3d dir_xy = -geo_center_xy.normalized(); // 指向原点方向
    //     virtual_base_pos       = geo_center_mm + dir_xy * virtual_base_dist;
    //     virtual_base_pos.z()   = 0.0; // 虚拟原点在z=0平面上
    // } else {
    //     virtual_base_pos = Eigen::Vector3d(virtual_base_dist, 0.0, 0.0);
    // }

    // // 1. 从原点到圆柱底面中心的距离 (mm)
    // double dist_origin_to_bottom_center = cylinder_model_.bottom_center.norm() * 1000.0;

    // // 2. 从原点到圆柱底面表面的距离 (mm) - 底面中心距离减去半径
    // double dist_origin_to_bottom_surface = dist_origin_to_bottom_center - cylinder_radius_mm;

    // // 3. 从原点到虚拟机械臂中心的距离 (mm)
    // double dist_origin_to_virtual_arm = virtual_base_pos.norm();

    // // 4. 从圆柱中心到表面的距离 (mm) - 即半径
    // double dist_center_to_surface = cylinder_radius_mm;

    // // 5. 从圆柱中心到虚拟机械臂中心的距离 (mm)
    // // 即 virtual_base_dist (因为虚拟原点在圆柱中心XY方向延伸 virtual_base_dist 的位置)
    // double dist_center_to_virtual_arm = virtual_base_dist;

    // ROS_INFO("Distances from origin: BottomCenter=%.2f mm, BottomSurface=%.2f mm, VirtualArmCenter=%.2f mm",
    //          dist_origin_to_bottom_center, dist_origin_to_bottom_surface, dist_origin_to_virtual_arm);
    // ROS_INFO("Distances from cylinder center: ToSurface=%.2f mm (radius), ToVirtualArmCenter=%.2f mm (R+%.2f)",
    //          dist_center_to_surface, dist_center_to_virtual_arm, fitting_params_.virtual_base_to_surface_);

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

/// @brief 保存所有可视化数据（原始点云+拟合几何+切片）带颜色信息
void CylinderFitter::saveVisualizationData() {
    if (!hasModel()) {
        ROS_WARN("No cylinder model available for saving.");
        return;
    }

    try {
        std::string package_path = ros::package::getPath("pcl_process");
        std::string res_dir      = package_path + "/res";

        struct stat st = {0};
        if (stat(res_dir.c_str(), &st) == -1) {
            mkdir(res_dir.c_str(), 0777);
        }

        std::time_t now      = std::time(nullptr);
        std::tm    *local_tm = std::localtime(&now);
        char        time_str[64];
        std::strftime(time_str, sizeof(time_str), "%Y-%m-%d-%H-%M-%S", local_tm);

        // 创建带颜色的点云
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        // 创建颜色记录文件
        std::string   color_log_file = res_dir + "/visualization_colors_" + std::string(time_str) + ".txt";
        std::ofstream color_log(color_log_file, std::ios::app);
        color_log << "====== Visualization Color Information ======\n";
        color_log << "Timestamp: " << time_str << "\n\n";

        auto addPointsWithColor = [&](const std::string &name, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                      uint8_t r, uint8_t g, uint8_t b) {
            if (!cloud || cloud->empty()) return;
            color_log << name << " (RGB: " << (int)r << ", " << (int)g << ", " << (int)b << ") - " << cloud->size()
                      << " points\n";
            for (const auto &pt : cloud->points) {
                pcl::PointXYZRGB rgb_pt;
                rgb_pt.x = pt.x;
                rgb_pt.y = pt.y;
                rgb_pt.z = pt.z;
                rgb_pt.r = r;
                rgb_pt.g = g;
                rgb_pt.b = b;
                colored_cloud->push_back(rgb_pt);
            }
        };

        auto addPoint = [&](const std::string &name, const Eigen::Vector3f &pt, uint8_t r, uint8_t g, uint8_t b) {
            color_log << name << " (RGB: " << (int)r << ", " << (int)g << ", " << (int)b << ")\n";
            pcl::PointXYZRGB rgb_pt;
            rgb_pt.x = pt.x();
            rgb_pt.y = pt.y();
            rgb_pt.z = pt.z();
            rgb_pt.r = r;
            rgb_pt.g = g;
            rgb_pt.b = b;
            colored_cloud->push_back(rgb_pt);
        };

        auto addLineSegments = [&](const std::string &name, const Eigen::Vector3f &p0, const Eigen::Vector3f &p1,
                                   uint8_t r, uint8_t g, uint8_t b, int samples = 100) {
            color_log << name << " (RGB: " << (int)r << ", " << (int)g << ", " << (int)b << ") - " << samples
                      << " segments\n";
            for (int i = 0; i <= samples; ++i) {
                double           t  = (double)i / samples;
                Eigen::Vector3f  pt = p0 + t * (p1 - p0);
                pcl::PointXYZRGB rgb_pt;
                rgb_pt.x = pt.x();
                rgb_pt.y = pt.y();
                rgb_pt.z = pt.z();
                rgb_pt.r = r;
                rgb_pt.g = g;
                rgb_pt.b = b;
                colored_cloud->push_back(rgb_pt);
            }
        };

        color_log << "\n--- Visualization Elements ---\n";

        // 1. 白色原始点云 (255, 255, 255)
        addPointsWithColor("1. Original Point Cloud (White)", cloud_, 255, 255, 255);

        // 2. 红色圆柱表面点云 (255, 0, 0)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder_surface(new pcl::PointCloud<pcl::PointXYZ>);
            Eigen::Vector3f                     axis_norm = cylinder_model_.axis.normalized();
            Eigen::Vector3f                     x         = axis_norm.unitOrthogonal();
            Eigen::Vector3f                     y         = axis_norm.cross(x);
            double                              radius    = cylinder_model_.radius;
            Eigen::Vector3f                     center    = cylinder_model_.bottom_center;
            double                              min_proj  = (cylinder_model_.bottom_center - center).dot(axis_norm);
            double                              max_proj  = (cylinder_model_.top_center - center).dot(axis_norm);

            int n_circ = 20;
            int n_h    = 25;
            for (int i = 0; i <= n_h; ++i) {
                double          h     = min_proj + (max_proj - min_proj) * i / n_h;
                Eigen::Vector3f layer = center + h * axis_norm;
                for (int j = 0; j < n_circ; ++j) {
                    double          theta = 2.0 * M_PI * j / n_circ;
                    Eigen::Vector3f pt    = layer + radius * (std::cos(theta) * x + std::sin(theta) * y);
                    cylinder_surface->push_back(pcl::PointXYZ(pt.x(), pt.y(), pt.z()));
                }
            }
            addPointsWithColor("2. Cylinder Surface (Red)", cylinder_surface, 255, 0, 0);
        }

        // 3. 黄色底面圆环 (255, 255, 0)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr bottom_ring(new pcl::PointCloud<pcl::PointXYZ>);
            Eigen::Vector3f                     axis_norm = cylinder_model_.axis.normalized();
            Eigen::Vector3f                     x         = axis_norm.unitOrthogonal();
            Eigen::Vector3f                     y         = axis_norm.cross(x);
            double                              radius    = cylinder_model_.radius;
            Eigen::Vector3f                     center    = cylinder_model_.bottom_center;
            int                                 ring_pts  = 72;
            for (int i = 0; i < ring_pts; ++i) {
                double          theta = 2.0 * M_PI * i / ring_pts;
                Eigen::Vector3f pt    = center + radius * (std::cos(theta) * x + std::sin(theta) * y);
                bottom_ring->push_back(pcl::PointXYZ(pt.x(), pt.y(), pt.z()));
            }
            addPointsWithColor("3. Bottom Ring (Yellow)", bottom_ring, 255, 255, 0);
        }

        // 4. 青色顶面圆环 (0, 255, 255)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr top_ring(new pcl::PointCloud<pcl::PointXYZ>);
            Eigen::Vector3f                     axis_norm = cylinder_model_.axis.normalized();
            Eigen::Vector3f                     x         = axis_norm.unitOrthogonal();
            Eigen::Vector3f                     y         = axis_norm.cross(x);
            double                              radius    = cylinder_model_.radius;
            Eigen::Vector3f                     center    = cylinder_model_.top_center;
            int                                 ring_pts  = 72;
            for (int i = 0; i < ring_pts; ++i) {
                double          theta = 2.0 * M_PI * i / ring_pts;
                Eigen::Vector3f pt    = center + radius * (std::cos(theta) * x + std::sin(theta) * y);
                top_ring->push_back(pcl::PointXYZ(pt.x(), pt.y(), pt.z()));
            }
            addPointsWithColor("4. Top Ring (Cyan)", top_ring, 0, 255, 255);
        }

        // 5. 底面中心球体 (255, 255, 0) 黄色
        color_log << "\n--- Sphere Centers ---\n";
        addPoint("5. Bottom Center Sphere (Yellow)", cylinder_model_.bottom_center, 255, 255, 0);

        // 6. 顶面中心球体 (0, 255, 255) 青色
        addPoint("6. Top Center Sphere (Cyan)", cylinder_model_.top_center, 0, 255, 255);

        // 7. 圆柱轴线 (255, 0, 0) 红色
        color_log << "\n--- Cylinder Axis ---\n";
        Eigen::Vector3f axis_norm = cylinder_model_.axis.normalized();
        Eigen::Vector3f center    = cylinder_model_.bottom_center;
        double          min_proj  = (cylinder_model_.bottom_center - center).dot(axis_norm);
        double          max_proj  = (cylinder_model_.top_center - center).dot(axis_norm);
        Eigen::Vector3f start     = center + min_proj * axis_norm;
        Eigen::Vector3f end       = center + max_proj * axis_norm;
        addLineSegments("7. Cylinder Axis (Red)", start, end, 255, 0, 0, 100);

        // 8. 紫色源点到圆柱中点连线 (255, 0, 255)
        color_log << "\n--- Reference Lines ---\n";
        Eigen::Vector3f source_point(0, 0, 0);
        addLineSegments("8. Source to Cylinder Center (Magenta)", source_point, cylinder_model_.geo_center, 255, 0, 255,
                        100);

        // 9. 黄色源点到圆柱底面中心连线 (255, 255, 0)
        addLineSegments("9. Source to Bottom Center (Yellow)", source_point, cylinder_model_.bottom_center, 255, 255, 0,
                        100);

        // 10. 青色源点到圆柱顶面中心连线 (0, 255, 255)
        addLineSegments("10. Source to Top Center (Cyan)", source_point, cylinder_model_.top_center, 0, 255, 255, 100);

        // 11. 绿色Y轴正方向 (0, 255, 0)
        color_log << "\n--- Y Axis Reference ---\n";
        Eigen::Vector3f y_pos_extend = source_point + Eigen::Vector3f(0.0f, 100.0f, 0.0f);
        addLineSegments("11. Source Y+ Axis (Green)", source_point, y_pos_extend, 0, 255, 0, 200);

        // 12. 切片中心 (绿色) (0, 255, 0) - 加大显示
        color_log << "\n--- Slice Centers ---\n";
        color_log << "12. Slice Centers (Green) - " << slices_data_.size() << " slices\n";
        for (const auto &slice : slices_data_) {
            // 为了让点显示得更大，在中心点周围添加多个相同颜色的点（形成一个小球）
            Eigen::Vector3f center_pos(slice.center_mm.x() / 1000.0f, slice.center_mm.y() / 1000.0f,
                                       slice.center_mm.z() / 1000.0f);
            const float     sphere_radius  = 0.015f; // 15mm 半径
            int             sphere_samples = 20;     // 采样点数

            for (int s = 0; s < sphere_samples; ++s) {
                float theta = 2.0f * M_PI * s / sphere_samples;
                for (int t = 0; t < 3; ++t) {
                    float phi = M_PI * t / 3.0f;
                    float x   = sphere_radius * std::sin(phi) * std::cos(theta);
                    float y   = sphere_radius * std::sin(phi) * std::sin(theta);
                    float z   = sphere_radius * std::cos(phi);

                    pcl::PointXYZRGB rgb_pt;
                    rgb_pt.x = center_pos.x() + x;
                    rgb_pt.y = center_pos.y() + y;
                    rgb_pt.z = center_pos.z() + z;
                    rgb_pt.r = 0;
                    rgb_pt.g = 255;
                    rgb_pt.b = 0;
                    colored_cloud->push_back(rgb_pt);
                }
            }
        }

        // 13. 工作点 (蓝色) (0, 0, 255) - 加大显示
        color_log << "\n--- Slice Work Points ---\n";
        color_log << "13. Slice Work Points (Blue) - " << slices_data_.size() << " work points\n";
        for (const auto &slice : slices_data_) {
            // 为了让点显示得更大，在工作点周围添加多个相同颜色的点（形成一个小球）
            Eigen::Vector3f work_pos(slice.work_mm.x() / 1000.0f, slice.work_mm.y() / 1000.0f,
                                     slice.work_mm.z() / 1000.0f);
            const float     sphere_radius  = 0.015f; // 15mm 半径
            int             sphere_samples = 20;     // 采样点数

            for (int s = 0; s < sphere_samples; ++s) {
                float theta = 2.0f * M_PI * s / sphere_samples;
                for (int t = 0; t < 3; ++t) {
                    float phi = M_PI * t / 3.0f;
                    float x   = sphere_radius * std::sin(phi) * std::cos(theta);
                    float y   = sphere_radius * std::sin(phi) * std::sin(theta);
                    float z   = sphere_radius * std::cos(phi);

                    pcl::PointXYZRGB rgb_pt;
                    rgb_pt.x = work_pos.x() + x;
                    rgb_pt.y = work_pos.y() + y;
                    rgb_pt.z = work_pos.z() + z;
                    rgb_pt.r = 0;
                    rgb_pt.g = 0;
                    rgb_pt.b = 255;
                    colored_cloud->push_back(rgb_pt);
                }
            }
        }

        // 14. 轴线中心 (白色) (255, 255, 255)
        color_log << "\n--- Slice Axis Centers ---\n";
        color_log << "14. Slice Axis Centers (White) - " << slices_data_.size() << " axis centers\n";
        for (const auto &slice : slices_data_) {
            pcl::PointXYZRGB rgb_pt;
            rgb_pt.x = slice.circle_center_mm.x() / 1000.0f;
            rgb_pt.y = slice.circle_center_mm.y() / 1000.0f;
            rgb_pt.z = slice.circle_center_mm.z() / 1000.0f;
            rgb_pt.r = 255;
            rgb_pt.g = 255;
            rgb_pt.b = 255;
            colored_cloud->push_back(rgb_pt);
        }

        // 设置点云宽度和高度
        colored_cloud->width    = colored_cloud->size();
        colored_cloud->height   = 1;
        colored_cloud->is_dense = true;

        // 保存带颜色的点云
        std::string colored_pcd = res_dir + "/cylinder_visualization_colored_" + std::string(time_str) + ".pcd";
        if (pcl::io::savePCDFileBinary(colored_pcd, *colored_cloud) == 0) {
            ROS_INFO("✓ Saved colored visualization data to %s (%zu points)", colored_pcd.c_str(),
                     colored_cloud->size());
        } else {
            ROS_ERROR("✗ Failed to save colored visualization data");
        }

        // 保存颜色日志
        color_log << "\n====== Summary ======\n";
        color_log << "Total Points Saved: " << colored_cloud->size() << "\n";
        color_log << "Original Cloud Points: " << cloud_->size() << "\n";
        color_log << "Cylinder Surface Points: " << (20 * 25) << "\n";
        color_log << "Bottom Ring Points: " << 72 << "\n";
        color_log << "Top Ring Points: " << 72 << "\n";
        color_log << "Reference Lines Points: " << (101 + 101 + 101 + 101 + 201) << "\n";
        color_log << "Slices Count: " << slices_data_.size() << "\n";
        color_log << "Slice Centers (Green): " << (slices_data_.size() * 60) << "\n";
        color_log << "Slice Work Points (Blue): " << (slices_data_.size() * 60) << "\n";
        color_log << "Slice Axis Centers: " << slices_data_.size() << "\n";
        color_log.close();

        ROS_INFO("✓ Saved visualization color information to %s", color_log_file.c_str());

    } catch (const std::exception &e) {
        ROS_ERROR("Error saving visualization data: %s", e.what());
    }
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

// 辅助向量操作实现
double CylinderFitter::norm(const std::vector<double> &v) const {
    if (v.size() != 3) throw std::invalid_argument("Vector must be 3-dimensional for norm calculation.");
    double sum_of_squares = 0.0;
    for (double val : v) {
        sum_of_squares += val * val;
    }
    return std::sqrt(sum_of_squares);
}

std::vector<double> CylinderFitter::cross_product(const std::vector<double> &a, const std::vector<double> &b) const {
    if (a.size() != 3 || b.size() != 3) throw std::invalid_argument("Vectors must be 3-dimensional for cross product.");
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}