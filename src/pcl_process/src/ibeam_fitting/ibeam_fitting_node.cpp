#include "ibeam_fitting_node.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <set>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

IBeamFitter::IBeamFitter()
    : cloud_(new pcl::PointCloud<pcl::PointXYZ>)
    , filtered_cloud_(new pcl::PointCloud<pcl::PointXYZ>)
    , roi_cloud_(new pcl::PointCloud<pcl::PointXYZ>)
    , viewer_(new pcl::visualization::PCLVisualizer("IBeam Fitting Viewer")) {

    // 初始化高度层
    height_layers_ = {{1.5, 2.5, "低层"}, {2.5, 4.0, "中层"}, {4.0, 7.0, "高层"}};

    // 初始化可视化器
    viewer_->setBackgroundColor(0.1, 0.1, 0.15);
    viewer_->addCoordinateSystem(1.0);
    viewer_->initCameraParameters();

    // 启动可视化线程
    viewer_thread_ = std::thread(&IBeamFitter::viewerThread, this);
}

IBeamFitter::~IBeamFitter() {
    running_ = false;
    if (viewer_thread_.joinable()) {
        viewer_thread_.join();
    }
}

void IBeamFitter::setInputCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) { cloud_ = cloud; }

void IBeamFitter::setParams(const IBeamDetectionParams &params) { detection_params_ = params; }

IBeamDetectionParams IBeamFitter::getDetectionParams() const { return detection_params_; }

std::vector<IBeamResult> IBeamFitter::getResults() const { return results_; }

pcl::PointCloud<pcl::PointXYZ>::Ptr IBeamFitter::getFilteredCloud() const { return filtered_cloud_; }

pcl::PointCloud<pcl::PointXYZ>::Ptr IBeamFitter::getROICloud() const { return roi_cloud_; }

/// @brief 主检测
/// @return
bool IBeamFitter::detect() {
    results_.clear();

    if (cloud_->empty()) {
        std::cerr << "[IBeamFitter] 错误: 输入点云为空!" << std::endl;
        return false;
    }

    std::cout << "\n=== 工字钢检测开始 ===" << std::endl;
    std::cout << "原始点云点数: " << cloud_->size() << std::endl;

    // 1. 预处理：降采样与去噪
    filtered_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>(*cloud_));
    voxelDownsample(filtered_cloud_);
    statisticalOutlierRemoval(filtered_cloud_);
    std::cout << "预处理后点数: " << filtered_cloud_->size() << std::endl;

    // 2. 移除背景墙面
    std::cout << "\n=== 移除背景墙面 ===" << std::endl;
    filtered_cloud_ = removeWallPlanes(filtered_cloud_);
    std::cout << "移除墙面后点数: " << filtered_cloud_->size() << std::endl;

    // 3. 距离和高度过滤
    roi_cloud_ = filterByDistanceAndHeight(filtered_cloud_);
    std::cout << "ROI区域点数: " << roi_cloud_->size() << std::endl;

    if (roi_cloud_->size() < 50) {
        std::cerr << "[IBeamFitter] 警告: ROI区域点数不足" << std::endl;
        return false;
    }

    // 4. 分层聚类
    std::cout << "\n=== 分层聚类分割 ===" << std::endl;
    std::vector<int> labels = layeredClustering(roi_cloud_);

    int max_label = *std::max_element(labels.begin(), labels.end());
    std::cout << "分层聚类共检测到 " << (max_label + 1) << " 个聚类" << std::endl;

    if (max_label < 0) {
        std::cerr << "[IBeamFitter] 警告: 未检测到任何聚类" << std::endl;
        return false;
    }

    // 5. 多轮合并策略
    // 第一轮：严格合并
    labels = mergeIBeamClusters(roi_cloud_, labels, detection_params_.merge_max_gap_strict,
                                detection_params_.merge_max_height_diff_strict,
                                detection_params_.merge_min_axis_similarity_strict);

    // 第二轮：中等条件合并
    labels = mergeIBeamClusters(roi_cloud_, labels, detection_params_.merge_max_gap_loose,
                                detection_params_.merge_max_height_diff_loose,
                                detection_params_.merge_min_axis_similarity_loose);

    max_label = *std::max_element(labels.begin(), labels.end());
    std::cout << "合并后剩余 " << (max_label + 1) << " 个聚类" << std::endl;

    // 6. 对每个聚类进行PCA形状分析
    std::cout << "\n=== 聚类形状分析 ===" << std::endl;

    for (int cluster_id = 0; cluster_id <= max_label; ++cluster_id) {
        // 提取当前聚类的点
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (size_t i = 0; i < labels.size(); ++i) {
            if (labels[i] == cluster_id) {
                cluster_cloud->points.push_back(roi_cloud_->points[i]);
            }
        }

        if (cluster_cloud->size() < 30) {
            continue;
        }

        // PCA分析
        Eigen::Vector3d center, dimensions, main_axis;
        Eigen::Matrix3d eigenvectors;
        double          linearity, planarity;

        if (!analyzeClusterShape(cluster_cloud, center, dimensions, main_axis, eigenvectors, linearity, planarity)) {
            continue;
        }

        double length       = dimensions[0];
        double width        = dimensions[1];
        double thickness    = dimensions[2];
        double aspect_ratio = length / std::max(width, 0.01);
        double main_axis_z  = std::abs(main_axis[2]);
        double dist         = std::sqrt(center[0] * center[0] + center[1] * center[1]);

        std::cout << "\n聚类 #" << cluster_id << ": " << cluster_cloud->size() << " 点" << std::endl;
        std::cout << "  中心: (" << center[0] << ", " << center[1] << ", " << center[2] << "), 距离=" << dist << "m"
                  << std::endl;
        std::cout << "  尺寸: " << length << " x " << width << " x " << thickness << " m" << std::endl;
        std::cout << "  主轴: (" << main_axis[0] << ", " << main_axis[1] << ", " << main_axis[2] << ")" << std::endl;
        std::cout << "  形状: 线性度=" << linearity << ", 长宽比=" << aspect_ratio
                  << ", 主轴水平性=" << (1 - main_axis_z) << std::endl;

        // 判断是否为工字钢
        if (isIBeam(linearity, length, width, thickness, aspect_ratio, main_axis_z)) {
            std::cout << "  -> ✓ 候选工字钢, 进行精细化识别..." << std::endl;

            // 精细化检测
            pcl::PointCloud<pcl::PointXYZ>::Ptr refined_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            Eigen::Vector3d                     refined_center, refined_dims;

            if (refineIBeamDetection(cluster_cloud, main_axis, center, refined_cloud, refined_center, refined_dims)) {
                std::cout << "       精细化完成: " << cluster_cloud->size() << " -> " << refined_cloud->size() << " 点"
                          << std::endl;
                std::cout << "       精细尺寸: " << refined_dims[0] << " x " << refined_dims[1] << " x "
                          << refined_dims[2] << " m" << std::endl;

                // 创建检测结果
                IBeamResult result;
                result.id                   = results_.size() + 1;
                result.center               = refined_center;
                result.dimensions           = refined_dims;
                result.main_axis            = main_axis;
                result.linearity            = linearity;
                result.aspect_ratio         = refined_dims[0] / std::max(refined_dims[1], 0.01);
                result.point_count          = refined_cloud->size();
                result.original_point_count = cluster_cloud->size();
                result.points               = refined_cloud;

                // 计算姿态四元数
                Eigen::Vector3d z_axis(0, 0, 1);
                Eigen::Vector3d rotation_axis = z_axis.cross(main_axis);
                if (rotation_axis.norm() > 1e-6) {
                    rotation_axis.normalize();
                    double angle       = std::acos(z_axis.dot(main_axis));
                    result.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(angle, rotation_axis));
                } else {
                    result.orientation = Eigen::Quaterniond::Identity();
                }

                results_.push_back(result);
            } else {
                // 精细化验证失败（不通过连续性/双翼缘/截面检测）
                // 如果宽度异常大（可能包含了支架/连接件），不盲目使用原始结果
                // 但如果宽度在合理范围内，则保留（可能只是点云稀疏导致验证未通过）
                double wh_ratio = width / std::max(thickness, 0.01);

                if (width < 0.50 && wh_ratio >= 0.5 && wh_ratio <= 3.0) {
                    // 原始截面尺寸合理，直接使用
                    std::cout << "       精细化失败, 但原始截面尺寸合理(W=" << width
                              << "m, W/H=" << wh_ratio << "), 使用原始结果" << std::endl;

                    IBeamResult result;
                    result.id                   = results_.size() + 1;
                    result.center               = center;
                    result.dimensions           = dimensions;
                    result.main_axis            = main_axis;
                    result.linearity            = linearity;
                    result.aspect_ratio         = aspect_ratio;
                    result.point_count          = cluster_cloud->size();
                    result.original_point_count = cluster_cloud->size();
                    result.points               = cluster_cloud;
                    result.orientation          = Eigen::Quaterniond::Identity();

                    results_.push_back(result);
                } else {
                    // 宽度异常大说明混入了非工字钢结构，拒绝该聚类
                    std::cout << "       精细化失败, 原始截面宽度异常(W=" << width
                              << "m, H=" << thickness << "m, W/H=" << wh_ratio
                              << "), 拒绝该聚类(可能包含支架/连接件)" << std::endl;
                }
            }
        } else {
            std::vector<std::string> reasons;
            if (linearity < detection_params_.min_linearity) {
                reasons.push_back("线性度不足(" + std::to_string(linearity) + "<" +
                                  std::to_string(detection_params_.min_linearity) + ")");
            }
            if (length < detection_params_.min_length) {
                reasons.push_back("长度不足(" + std::to_string(length) + "m<" +
                                  std::to_string(detection_params_.min_length) + "m)");
            }
            if (aspect_ratio < detection_params_.min_aspect_ratio) {
                reasons.push_back("长宽比不足(" + std::to_string(aspect_ratio) + "<" +
                                  std::to_string(detection_params_.min_aspect_ratio) + ")");
            }
            if (main_axis_z > detection_params_.max_axis_z) {
                reasons.push_back("主轴不水平(Z=" + std::to_string(main_axis_z) + ">" +
                                  std::to_string(detection_params_.max_axis_z) + ")");
            }

            std::cout << "  -> ✗ 未识别: ";
            for (size_t i = 0; i < reasons.size(); ++i) {
                std::cout << reasons[i];
                if (i < reasons.size() - 1) std::cout << ", ";
            }
            std::cout << std::endl;
        }
    }

    // 7. 扩展短工字钢
    if (!results_.empty()) {
        extendShortIBeams(results_, roi_cloud_);
    }

    // 8. 输出结果
    std::cout << "\n" << std::string(50, '=') << std::endl;
    ROS_WARN("[IBeamFitter::detect] IBeam Input Parameters: \ndistance_tcp2surface=%.2f mm,\nstep_height=%.2f mm,\n"
             "step_length=%.2f mm,\nlap=%.2f,\n"
             "arm_reach=%.2f mm, \nmode_length=%d, \nlength=%.2f mm, \nmode_theta=%d, \ntheta=%.2f degrees",
             ibeam_inputs_.distance_tcp2surface, ibeam_inputs_.step_height, ibeam_inputs_.step_length,
             ibeam_inputs_.lap, ibeam_inputs_.arm_reach, ibeam_inputs_.mode_length, ibeam_inputs_.length,
             ibeam_inputs_.mode_theta, ibeam_inputs_.theta);

    if (!results_.empty()) {
        ROS_ERROR("[IBeamFitter::detect] IBeamFitter detected %zu I-beams:", results_.size());
        for (const auto &beam : results_) {
            // 计算修正后的翼缘宽度
            double flange_w        = calculateFlangeWidth(beam);
            double corrected_width = (flange_w > 0) ? flange_w : beam.dimensions[2] * 0.4;
            if (beam.dimensions[1] < beam.dimensions[2] * 1.2 && beam.dimensions[1] > beam.dimensions[2] * 0.1) {
                corrected_width = beam.dimensions[1];
            }

            ROS_ERROR("[IBeamFitter::detect] I-beam #%d: Position=(%.2f, %.2f, %.2f)", beam.id, beam.center[0],
                      beam.center[1], beam.center[2]);
            ROS_ERROR("[IBeamFitter::detect]   Dimensions (LxWxH): %.2f x %.2f x %.2f m", beam.dimensions[0],
                      corrected_width, beam.dimensions[2]);
            ROS_ERROR("[IBeamFitter::detect]   Linearity: %.3f, Aspect Ratio: %.2f", beam.linearity, beam.aspect_ratio);
            ROS_ERROR("[IBeamFitter::detect]   Point Count: %zu (Original: %zu)", beam.point_count,
                      beam.original_point_count);
        }
    } else {
        ROS_ERROR("[IBeamFitter::detect] FAILED: No I-beams detected");
    }
    ROS_ERROR("%s", std::string(50, '=').c_str());

    return !results_.empty();
}

/// @brief 预处理
/// @param cloud
void IBeamFitter::voxelDownsample(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(detection_params_.voxel_size, detection_params_.voxel_size, detection_params_.voxel_size);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    vg.filter(*filtered);
    cloud = filtered;
}

/// @brief 统计离群点移除
/// @param cloud
void IBeamFitter::statisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(detection_params_.sor_neighbors);
    sor.setStddevMulThresh(detection_params_.sor_std_ratio);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    sor.filter(*filtered);
    cloud = filtered;
}

/// @brief 墙面移除
/// @param cloud
/// @return
pcl::PointCloud<pcl::PointXYZ>::Ptr IBeamFitter::removeWallPlanes(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr remaining(new pcl::PointCloud<pcl::PointXYZ>(*cloud));
    int                                 removed_total = 0;

    for (int i = 0; i < detection_params_.max_wall_iterations; ++i) {
        if (remaining->size() < static_cast<size_t>(detection_params_.min_wall_points)) {
            break;
        }

        // RANSAC平面分割
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::ModelCoefficients::Ptr         coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr              inliers(new pcl::PointIndices);

        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(detection_params_.wall_distance_threshold);
        seg.setMaxIterations(1000);
        seg.setInputCloud(remaining);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() < static_cast<size_t>(detection_params_.min_wall_points)) {
            break;
        }

        // 获取平面法向量
        Eigen::Vector3d normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);

        // 检查是否为垂直平面(法向量接近水平)
        double angle_to_vertical = std::acos(std::abs(normal[2])) * 180.0 / M_PI;
        bool   is_vertical_plane = angle_to_vertical > 60.0;

        // 检查平面的空间范围
        double z_min       = std::numeric_limits<double>::max();
        double z_max       = std::numeric_limits<double>::lowest();
        double xy_dist_sum = 0.0;

        for (int idx : inliers->indices) {
            const auto &pt = remaining->points[idx];
            z_min          = std::min(z_min, static_cast<double>(pt.z));
            z_max          = std::max(z_max, static_cast<double>(pt.z));
            xy_dist_sum += std::sqrt(pt.x * pt.x + pt.y * pt.y);
        }

        double z_span       = z_max - z_min;
        double mean_xy_dist = xy_dist_sum / inliers->indices.size();

        // 墙面特征判断
        bool is_wall = is_vertical_plane && z_span > 1.0 && mean_xy_dist > 4.0;

        if (is_wall) {
            std::cout << "  移除墙面 " << (i + 1) << ": " << inliers->indices.size() << " 点, Z跨度=" << z_span
                      << "m, 平均距离=" << mean_xy_dist << "m" << std::endl;

            // 移除墙面点
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(remaining);
            extract.setIndices(inliers);
            extract.setNegative(true);

            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
            extract.filter(*filtered);
            remaining = filtered;
            removed_total += inliers->indices.size();
        } else {
            break;
        }
    }

    std::cout << "  共移除 " << removed_total << " 个墙面点" << std::endl;
    return remaining;
}

/// @brief 空间过滤，距离和高度过滤
/// @param cloud
/// @return
pcl::PointCloud<pcl::PointXYZ>::Ptr
    IBeamFitter::filterByDistanceAndHeight(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);

    for (const auto &pt : cloud->points) {
        double dist = std::sqrt(pt.x * pt.x + pt.y * pt.y);
        if (dist > detection_params_.min_distance && dist < detection_params_.max_distance &&
            pt.z > detection_params_.min_height) {
            filtered->points.push_back(pt);
        }
    }

    filtered->width    = filtered->points.size();
    filtered->height   = 1;
    filtered->is_dense = true;

    return filtered;
}

/// @brief 分层聚类
/// @param cloud
/// @return
std::vector<int> IBeamFitter::layeredClustering(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {

    std::vector<int> all_labels(cloud->size(), -1);
    int              current_label = 0;

    for (const auto &layer : height_layers_) {
        // 提取当前层的点索引
        std::vector<int> layer_indices;
        for (size_t i = 0; i < cloud->size(); ++i) {
            double z = cloud->points[i].z;
            if (z >= layer.z_min && z < layer.z_max) {
                layer_indices.push_back(i);
            }
        }

        if (layer_indices.size() < 10) {
            continue;
        }

        // 创建层点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr layer_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (int idx : layer_indices) {
            layer_cloud->points.push_back(cloud->points[idx]);
        }
        layer_cloud->width  = layer_cloud->points.size();
        layer_cloud->height = 1;

        // DBSCAN聚类
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(layer_cloud);

        std::vector<pcl::PointIndices>                 cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(detection_params_.cluster_tolerance);
        ec.setMinClusterSize(detection_params_.min_cluster_size);
        ec.setMaxClusterSize(layer_cloud->size());
        ec.setSearchMethod(tree);
        ec.setInputCloud(layer_cloud);
        ec.extract(cluster_indices);

        int n_clusters = cluster_indices.size();
        if (n_clusters > 0) {
            std::cout << "  " << layer.name << " (Z: " << layer.z_min << "-" << layer.z_max
                      << "m): " << layer_indices.size() << " 点, " << n_clusters << " 个聚类" << std::endl;

            // 将层内标签映射到全局标签
            for (size_t c = 0; c < cluster_indices.size(); ++c) {
                for (int local_idx : cluster_indices[c].indices) {
                    int global_idx         = layer_indices[local_idx];
                    all_labels[global_idx] = current_label;
                }
                current_label++;
            }
        }
    }

    return all_labels;
}

/// @brief 聚类合并函数，对应 Python 的 merge_ibeam_clusters
/// @param cloud 点云
/// @param labels 标签
/// @param max_gap 最大间隙
/// @param max_height_diff 最大高度差
/// @param min_axis_similarity 最小主轴相似度
/// @return 新的标签列表
std::vector<int> IBeamFitter::mergeIBeamClusters(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                                 const std::vector<int> &labels, double max_gap, double max_height_diff,
                                                 double min_axis_similarity) {

    int max_label = *std::max_element(labels.begin(), labels.end());
    if (max_label < 1) {
        return labels;
    }

    // 计算每个聚类的属性
    std::vector<ClusterInfo> cluster_info(max_label + 1);
    for (int cluster_id = 0; cluster_id <= max_label; ++cluster_id) {
        cluster_info[cluster_id] = computeClusterInfo(cloud, labels, cluster_id);
    }

    // 查找需要合并的聚类对
    std::vector<std::pair<int, int>> merge_pairs;

    for (int i = 0; i <= max_label; ++i) {
        if (!cluster_info[i].valid) continue;

        for (int j = i + 1; j <= max_label; ++j) {
            if (!cluster_info[j].valid) continue;

            const auto &info_i = cluster_info[i];
            const auto &info_j = cluster_info[j];

            // 条件1: 主轴方向相似
            double axis_similarity = std::abs(info_i.main_axis.dot(info_j.main_axis));
            if (axis_similarity < min_axis_similarity) continue;

            // 条件2: 高度相近
            double height_diff = std::abs(info_i.height - info_j.height);
            if (height_diff > max_height_diff) continue;

            // 条件3: 在主轴方向上相邻
            double gap1 = info_j.axis_min - info_i.axis_max;
            double gap2 = info_i.axis_min - info_j.axis_max;
            double gap  = std::max(gap1, gap2);

            // 条件4: Y方向距离应该相近
            double lateral_dist = std::abs(info_i.center[1] - info_j.center[1]);
            if (lateral_dist > 0.3) continue;

            // 条件5: 截面尺寸兼容性 - 防止将支架/连接件与工字钢错误合并
            // 计算两个聚类垂直于主轴的截面跨度，差异过大则不合并
            {
                // 使用平均主轴方向
                Eigen::Vector3d avg_axis = (info_i.main_axis + info_j.main_axis).normalized();
                Eigen::Vector3d w_up(0, 0, 1);
                Eigen::Vector3d v_perp = w_up.cross(avg_axis);
                if (v_perp.norm() > 0.1) v_perp.normalize();
                else v_perp = Eigen::Vector3d(0, 1, 0);

                // 计算聚类 i 的截面跨度
                double i_v_min = 1e9, i_v_max = -1e9, i_w_min = 1e9, i_w_max = -1e9;
                double j_v_min = 1e9, j_v_max = -1e9, j_w_min = 1e9, j_w_max = -1e9;

                for (size_t idx = 0; idx < labels.size(); ++idx) {
                    if (labels[idx] == i) {
                        Eigen::Vector3d p(cloud->points[idx].x - info_i.center[0],
                                          cloud->points[idx].y - info_i.center[1],
                                          cloud->points[idx].z - info_i.center[2]);
                        double pv = p.dot(v_perp), pw = p.dot(w_up);
                        i_v_min = std::min(i_v_min, pv); i_v_max = std::max(i_v_max, pv);
                        i_w_min = std::min(i_w_min, pw); i_w_max = std::max(i_w_max, pw);
                    } else if (labels[idx] == j) {
                        Eigen::Vector3d p(cloud->points[idx].x - info_j.center[0],
                                          cloud->points[idx].y - info_j.center[1],
                                          cloud->points[idx].z - info_j.center[2]);
                        double pv = p.dot(v_perp), pw = p.dot(w_up);
                        j_v_min = std::min(j_v_min, pv); j_v_max = std::max(j_v_max, pv);
                        j_w_min = std::min(j_w_min, pw); j_w_max = std::max(j_w_max, pw);
                    }
                }

                double i_v_span = i_v_max - i_v_min;
                double j_v_span = j_v_max - j_v_min;
                double i_w_span = i_w_max - i_w_min;
                double j_w_span = j_w_max - j_w_min;

                // 截面跨度比如果超过 2.0 倍，拒绝合并
                double v_ratio = std::max(i_v_span, j_v_span) / std::max(std::min(i_v_span, j_v_span), 0.01);
                double w_ratio = std::max(i_w_span, j_w_span) / std::max(std::min(i_w_span, j_w_span), 0.01);

                if (v_ratio > 2.0 || w_ratio > 2.0) {
                    std::cout << "  -> 拒绝合并 #" << i << " 和 #" << j
                              << ": 截面尺寸不兼容 (v比=" << v_ratio << ", w比=" << w_ratio << ")" << std::endl;
                    continue;
                }
            }

            if (gap < max_gap && gap > -0.3) {
                std::cout << "  -> 合并聚类 #" << i << " 和 #" << j << ": 主轴相似度=" << axis_similarity
                          << ", 高度差=" << height_diff << "m, 间隔=" << gap << "m, Y距离=" << lateral_dist << "m"
                          << std::endl;
                merge_pairs.push_back({i, j});
            }
        }
    }

    if (merge_pairs.empty()) {
        return labels;
    }

    // 使用并查集执行合并
    std::map<int, int> label_map;
    for (int i = 0; i <= max_label; ++i) {
        label_map[i] = i;
    }

    for (const auto &pair : merge_pairs) {
        int root_i = findRoot(label_map, pair.first);
        int root_j = findRoot(label_map, pair.second);
        if (root_i != root_j) {
            label_map[root_j] = root_i;
        }
    }

    // 应用映射
    std::vector<int> new_labels = labels;
    for (size_t i = 0; i < labels.size(); ++i) {
        if (labels[i] >= 0) {
            new_labels[i] = findRoot(label_map, labels[i]);
        }
    }

    // 重新编号
    return relabelClusters(new_labels);
}

/// @brief 计算聚类信息的辅助函数
/// @param cloud 点云
/// @param labels 标签
/// @param cluster_id 聚类ID
/// @return 聚类信息
ClusterInfo IBeamFitter::computeClusterInfo(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                            const std::vector<int> &labels, int cluster_id) {

    ClusterInfo info;
    info.valid = false;

    // 提取聚类点
    std::vector<Eigen::Vector3d> points;
    for (size_t i = 0; i < labels.size(); ++i) {
        if (labels[i] == cluster_id) {
            const auto &pt = cloud->points[i];
            points.push_back(Eigen::Vector3d(pt.x, pt.y, pt.z));
        }
    }

    if (points.size() < 10) {
        return info;
    }

    // 计算中心
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    for (const auto &pt : points) {
        center += pt;
    }
    center /= points.size();
    info.center      = center;
    info.height      = center[2];
    info.point_count = points.size();

    // PCA计算主轴
    Eigen::MatrixXd centered(points.size(), 3);
    for (size_t i = 0; i < points.size(); ++i) {
        centered.row(i) = (points[i] - center).transpose();
    }

    Eigen::Matrix3d                                cov = centered.transpose() * centered / (points.size() - 1);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
    Eigen::Vector3d                                eigenvalues  = solver.eigenvalues();
    Eigen::Matrix3d                                eigenvectors = solver.eigenvectors();

    // 按特征值降序排列
    std::vector<std::pair<double, int>> eig_pairs;
    for (int i = 0; i < 3; ++i) {
        eig_pairs.push_back({eigenvalues[i], i});
    }
    std::sort(eig_pairs.begin(), eig_pairs.end(), std::greater<std::pair<double, int>>());

    info.main_axis = eigenvectors.col(eig_pairs[0].second);

    // 确保主轴方向一致
    if (info.main_axis[0] > 0) {
        info.main_axis = -info.main_axis;
    }

    // 计算沿主轴的范围
    double axis_proj_min = std::numeric_limits<double>::max();
    double axis_proj_max = std::numeric_limits<double>::lowest();

    for (const auto &pt : points) {
        double proj   = (pt - center).dot(info.main_axis) + center.dot(info.main_axis);
        axis_proj_min = std::min(axis_proj_min, proj);
        axis_proj_max = std::max(axis_proj_max, proj);
    }

    info.axis_min = axis_proj_min;
    info.axis_max = axis_proj_max;
    info.valid    = true;

    return info;
}

/// @brief PCA形状分析函数,对应 Python 的 analyze_cluster_shape
/// @param cluster_cloud 聚类点云
/// @param center 中心
/// @param dimensions 尺寸
/// @param main_axis 主轴
/// @param eigenvectors 特征向量
/// @param linearity 线性度
/// @param planarity 平面度
/// @return 是否成功
bool IBeamFitter::analyzeClusterShape(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cluster_cloud, Eigen::Vector3d &center,
                                      Eigen::Vector3d &dimensions, Eigen::Vector3d &main_axis,
                                      Eigen::Matrix3d &eigenvectors, double &linearity, double &planarity) {

    if (cluster_cloud->size() < 10) {
        return false;
    }

    // 计算中心
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cluster_cloud, centroid);
    center = Eigen::Vector3d(centroid[0], centroid[1], centroid[2]);

    // 计算协方差矩阵
    Eigen::MatrixXd points(cluster_cloud->size(), 3);
    for (size_t i = 0; i < cluster_cloud->size(); ++i) {
        points(i, 0) = cluster_cloud->points[i].x - center[0];
        points(i, 1) = cluster_cloud->points[i].y - center[1];
        points(i, 2) = cluster_cloud->points[i].z - center[2];
    }

    Eigen::Matrix3d cov = points.transpose() * points / (cluster_cloud->size() - 1);

    // 特征值分解
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
    Eigen::Vector3d                                eigenvalues = solver.eigenvalues();
    eigenvectors                                               = solver.eigenvectors();

    // 按特征值降序排列
    std::vector<std::pair<double, int>> eig_pairs;
    for (int i = 0; i < 3; ++i) {
        eig_pairs.push_back({eigenvalues[i], i});
    }
    std::sort(eig_pairs.begin(), eig_pairs.end(), std::greater<std::pair<double, int>>());

    Eigen::Vector3d sorted_eigenvalues;
    Eigen::Matrix3d sorted_eigenvectors;
    for (int i = 0; i < 3; ++i) {
        sorted_eigenvalues[i]      = eig_pairs[i].first;
        sorted_eigenvectors.col(i) = eigenvectors.col(eig_pairs[i].second);
    }

    eigenvectors = sorted_eigenvectors;
    main_axis    = eigenvectors.col(0);

    // 计算沿主轴方向的尺寸
    Eigen::MatrixXd proj = points * eigenvectors;
    dimensions[0]        = proj.col(0).maxCoeff() - proj.col(0).minCoeff(); // length
    dimensions[1]        = proj.col(1).maxCoeff() - proj.col(1).minCoeff(); // width
    dimensions[2]        = proj.col(2).maxCoeff() - proj.col(2).minCoeff(); // thickness

    // 计算形状特征
    linearity = (sorted_eigenvalues[0] - sorted_eigenvalues[1]) / (sorted_eigenvalues[0] + 1e-6);
    planarity = (sorted_eigenvalues[1] - sorted_eigenvalues[2]) / (sorted_eigenvalues[0] + 1e-6);

    return true;
}

/// @brief 工字钢判断逻辑,对应 Python 的 is_ibeam
/// @param linearity 线性度
/// @param length 长度
/// @param width 宽度
/// @param thickness 厚度
/// @param aspect_ratio 长宽比
/// @param main_axis_z 主轴Z方向分量
/// @return
bool IBeamFitter::isIBeam(double linearity, double length, double width, double thickness, double aspect_ratio,
                          double main_axis_z) {

    // ============ 负向过滤（快速排除） ============
    // 1. 长度过长，可能是多个结构被错误合并
    if (length > 5.0) {
        std::cout << "       [负向过滤] 长度过长: " << length << "m > 5.0m" << std::endl;
        return false;
    }

    // 2. 厚度异常大，可能包含端部圆柱/法兰结构
    if (thickness > 0.30 && thickness > width * 0.5) {
        std::cout << "       [负向过滤] 厚度异常: " << thickness << "m > 0.30m 且 > width*0.5" << std::endl;
        return false;
    }

    // 3. 宽高比异常，不符合工字钢截面特征
    double width_height_ratio = width / std::max(thickness, 0.01);
    if (width_height_ratio < 0.5 || width_height_ratio > 3.0) {
        std::cout << "       [负向过滤] 宽高比异常: " << width_height_ratio << " (应在0.5~3.0之间)" << std::endl;
        return false;
    }

    // ============ 正向判断 ============
    bool is_highly_linear   = linearity > detection_params_.min_linearity;
    bool is_long_enough     = length > detection_params_.min_length;
    bool is_elongated       = aspect_ratio > detection_params_.min_aspect_ratio;
    bool is_axis_horizontal = main_axis_z < detection_params_.max_axis_z;
    bool cross_section_ok   = width > detection_params_.min_width && width < detection_params_.max_width &&
                            thickness > detection_params_.min_thickness && thickness < detection_params_.max_thickness;

    return is_highly_linear && is_long_enough && is_elongated && is_axis_horizontal && cross_section_ok;
}

// ============================================================================
// 点云沿主轴的连续性检测 - 检查是否有明显间隙（使用绝对阈值）
// ============================================================================
static bool checkPointCloudContinuity(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const Eigen::Vector3f &axis,
                                      const Eigen::Vector3f &center,
                                      double max_absolute_gap  = 0.15, // 最大允许绝对间隙（米）
                                      double min_density_ratio = 0.15) // 切片最小点密度比
{
    if (!cloud || cloud->size() < 50) return false;

    Eigen::Vector3f u = axis.normalized();

    // 投影到主轴并收集位置
    std::vector<float> projections;
    projections.reserve(cloud->size());

    for (const auto &p : cloud->points) {
        Eigen::Vector3f pt(p.x, p.y, p.z);
        float           proj = (pt - center).dot(u);
        projections.push_back(proj);
    }

    // 排序后直接检测相邻点间隙
    std::sort(projections.begin(), projections.end());

    float u_min        = projections.front();
    float u_max        = projections.back();
    float total_length = u_max - u_min;

    if (total_length < 0.5f) return true; // 太短，跳过检测

    // 方法1：检测排序后相邻点的最大间隙
    float max_point_gap = 0;
    for (size_t i = 1; i < projections.size(); ++i) {
        float gap     = projections[i] - projections[i - 1];
        max_point_gap = std::max(max_point_gap, gap);
    }

    std::cout << "       连续性检测: 最大点间隙=" << max_point_gap << "m" << std::endl;

    // 如果相邻点间隙超过阈值，说明有断裂
    if (max_point_gap > max_absolute_gap) {
        std::cout << "       连续性检测: ✗ 发现明显间隙 (" << max_point_gap << "m > " << max_absolute_gap << "m)"
                  << std::endl;
        return false;
    }

    // 方法2：切片统计（作为辅助验证）
    const int        num_slices = 30;
    std::vector<int> slice_counts(num_slices, 0);
    float            slice_len = total_length / num_slices;

    for (float proj : projections) {
        int idx = std::min(num_slices - 1, int((proj - u_min) / slice_len));
        slice_counts[idx]++;
    }

    // 统计空切片数量
    int empty_slices          = 0;
    int consecutive_empty     = 0;
    int max_consecutive_empty = 0;

    for (int i = 0; i < num_slices; ++i) {
        if (slice_counts[i] == 0) {
            empty_slices++;
            consecutive_empty++;
            max_consecutive_empty = std::max(max_consecutive_empty, consecutive_empty);
        } else {
            consecutive_empty = 0;
        }
    }

    // 如果有连续2个以上空切片（约0.2m间隙），认为不连续
    float consecutive_gap = max_consecutive_empty * slice_len;
    if (consecutive_gap > max_absolute_gap) {
        std::cout << "       连续性检测: ✗ 存在连续空切片 (gap=" << consecutive_gap << "m)" << std::endl;
        return false;
    }

    std::cout << "       连续性检测: ✓ 点云连续 (max_gap=" << max_point_gap << "m)" << std::endl;
    return true;
}

// ============================================================================
// 增强版双翼缘结构检测 - 真正验证工字钢的"工"字形截面
// ============================================================================
static bool hasDoubleFlangeStructure(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const Eigen::Vector3f &axis,
                                     int histogram_bins = 60, float peak_ratio_thresh = 0.30f,
                                     float valley_ratio_thresh = 0.45f) {
    if (!cloud || cloud->size() < 50) return false;

    // Step 1：构建局部坐标系
    Eigen::Vector3f u = axis.normalized();
    Eigen::Vector3f w_axis(0, 0, 1); // 垂直向上
    Eigen::Vector3f v_axis = w_axis.cross(u).normalized();
    if (v_axis.norm() < 0.1f) {
        // u 接近垂直，使用备选
        Eigen::Vector3f tmp(1, 0, 0);
        v_axis = tmp.cross(u).normalized();
    }
    w_axis = u.cross(v_axis).normalized();

    // Step 2：计算中心
    Eigen::Vector3f center(0, 0, 0);
    for (const auto &p : cloud->points)
        center += Eigen::Vector3f(p.x, p.y, p.z);
    center /= cloud->size();

    // Step 3：沿主轴分成多个切片，对每个切片分析截面形状
    std::vector<float> all_u, all_v, all_w;
    all_u.reserve(cloud->size());
    all_v.reserve(cloud->size());
    all_w.reserve(cloud->size());

    float u_min = 1e9, u_max = -1e9;

    for (const auto &p : cloud->points) {
        Eigen::Vector3f pt(p.x, p.y, p.z);
        Eigen::Vector3f rel = pt - center;
        float           pu  = rel.dot(u);
        float           pv  = rel.dot(v_axis);
        float           pw  = rel.dot(w_axis);

        all_u.push_back(pu);
        all_v.push_back(pv);
        all_w.push_back(pw);

        u_min = std::min(u_min, pu);
        u_max = std::max(u_max, pu);
    }

    float total_length = u_max - u_min;
    if (total_length < 0.5f) {
        std::cout << "       双翼缘检测: 长度过短 (" << total_length << "m)" << std::endl;
        return false;
    }

    // Step 4：取中间60%区域的多个切片分析
    const int num_slices = 5;
    float     slice_len  = total_length * 0.6f / num_slices;
    float     start_u    = u_min + total_length * 0.2f;

    int valid_ibeam_slices = 0;
    int total_analyzed     = 0;

    for (int s = 0; s < num_slices; ++s) {
        float slice_start = start_u + s * slice_len;
        float slice_end   = slice_start + slice_len;

        // 收集该切片的点
        std::vector<float> slice_v, slice_w;
        for (size_t i = 0; i < all_u.size(); ++i) {
            if (all_u[i] >= slice_start && all_u[i] < slice_end) {
                slice_v.push_back(all_v[i]);
                slice_w.push_back(all_w[i]);
            }
        }

        if (slice_v.size() < 15) continue;
        total_analyzed++;

        // 分析该切片的v方向分布（检测双翼缘）
        float v_min  = *std::min_element(slice_v.begin(), slice_v.end());
        float v_max  = *std::max_element(slice_v.begin(), slice_v.end());
        float v_span = v_max - v_min;

        if (v_span < 0.05f) continue;

        // 构建v方向直方图
        std::vector<int> hist(histogram_bins, 0);
        float            step = v_span / histogram_bins;

        for (float v : slice_v) {
            int idx = std::min(histogram_bins - 1, int((v - v_min) / step));
            hist[idx]++;
        }

        // 平滑
        std::vector<float> smooth(histogram_bins, 0);
        for (int i = 1; i < histogram_bins - 1; i++)
            smooth[i] = (hist[i - 1] + hist[i] + hist[i + 1]) / 3.0f;

        // 寻找峰值
        int              max_val     = *std::max_element(hist.begin(), hist.end());
        float            peak_thresh = max_val * peak_ratio_thresh;
        std::vector<int> peaks;

        for (int i = 2; i < histogram_bins - 2; i++) {
            if (smooth[i] > peak_thresh && smooth[i] > smooth[i - 1] && smooth[i] > smooth[i - 2] &&
                smooth[i] > smooth[i + 1] && smooth[i] > smooth[i + 2]) {
                // 避免相邻峰重复计数
                if (peaks.empty() || i - peaks.back() > 3) {
                    peaks.push_back(i);
                }
            }
        }

        // 工字钢特征：恰好2个峰，且中间有明显低谷
        if (peaks.size() == 2) {
            int left_peak  = peaks[0];
            int right_peak = peaks[1];

            // 检查双峰间距（翼缘宽度应该合理）
            float peak_dist = (right_peak - left_peak) * step;
            if (peak_dist < 0.03f || peak_dist > 0.5f) continue;

            // 检查低谷深度
            int valley = std::numeric_limits<int>::max();
            for (int i = left_peak; i <= right_peak; i++)
                valley = std::min(valley, hist[i]);

            float valley_ratio = (float)valley / max_val;

            // 工字钢：两峰间低谷明显（腹板区域点少）
            if (valley_ratio < valley_ratio_thresh) {
                valid_ibeam_slices++;
            }
        }
    }

    // 判断：多数切片应符合工字钢特征
    float ibeam_ratio = total_analyzed > 0 ? (float)valid_ibeam_slices / total_analyzed : 0;

    std::cout << "       双翼缘检测: " << valid_ibeam_slices << "/" << total_analyzed
              << " 切片符合 (比例=" << ibeam_ratio << ")" << std::endl;

    if (ibeam_ratio < 0.5f) {
        std::cout << "       双翼缘检测: ✗ 不符合工字钢双翼缘特征" << std::endl;
        return false;
    }

    std::cout << "       双翼缘检测: ✓ 符合工字钢双翼缘结构" << std::endl;
    return true;
}

// ============================================================================
// 截面形状验证 - 检查是否为"工"字形而非圆弧
// ============================================================================
static bool verifyIBeamCrossSection(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const Eigen::Vector3f &axis) {
    if (!cloud || cloud->size() < 50) return false;

    // 构建局部坐标系
    Eigen::Vector3f u = axis.normalized();
    Eigen::Vector3f w_axis(0, 0, 1);
    Eigen::Vector3f v_axis = w_axis.cross(u).normalized();
    if (v_axis.norm() < 0.1f) {
        Eigen::Vector3f tmp(1, 0, 0);
        v_axis = tmp.cross(u).normalized();
    }
    w_axis = u.cross(v_axis).normalized();

    // 计算中心
    Eigen::Vector3f center(0, 0, 0);
    for (const auto &p : cloud->points)
        center += Eigen::Vector3f(p.x, p.y, p.z);
    center /= cloud->size();

    // 收集v-w平面投影
    std::vector<float> all_v, all_w;
    for (const auto &p : cloud->points) {
        Eigen::Vector3f pt(p.x, p.y, p.z);
        Eigen::Vector3f rel = pt - center;
        all_v.push_back(rel.dot(v_axis));
        all_w.push_back(rel.dot(w_axis));
    }

    float v_min = *std::min_element(all_v.begin(), all_v.end());
    float v_max = *std::max_element(all_v.begin(), all_v.end());
    float w_min = *std::min_element(all_w.begin(), all_w.end());
    float w_max = *std::max_element(all_w.begin(), all_w.end());

    float v_span = v_max - v_min;
    float w_span = w_max - w_min;

    if (v_span < 0.05f || w_span < 0.05f) return false;

    // 创建2D网格统计点分布
    const int                     grid_size = 10;
    std::vector<std::vector<int>> grid(grid_size, std::vector<int>(grid_size, 0));

    float v_step = v_span / grid_size;
    float w_step = w_span / grid_size;

    for (size_t i = 0; i < all_v.size(); ++i) {
        int vi = std::min(grid_size - 1, int((all_v[i] - v_min) / v_step));
        int wi = std::min(grid_size - 1, int((all_w[i] - w_min) / w_step));
        grid[vi][wi]++;
    }

    // 工字钢特征检测：
    // 1. 左右两侧（翼缘位置）应该有点
    // 2. 中间区域上下应该有点（腹板）
    // 3. 中间区域左右应该相对空

    // 统计各区域点数
    int left_flange = 0, right_flange = 0; // 左右翼缘
    int center_region = 0;                 // 中心区域

    for (int w = 0; w < grid_size; ++w) {
        // 左侧 (v: 0-2)
        for (int v = 0; v < 2; ++v)
            left_flange += grid[v][w];
        // 右侧 (v: 8-9)
        for (int v = grid_size - 2; v < grid_size; ++v)
            right_flange += grid[v][w];
    }

    // 中心水平带 (v: 3-6, w: 4-5)
    for (int v = 3; v <= 6; ++v) {
        for (int w = 4; w <= 5; ++w) {
            center_region += grid[v][w];
        }
    }

    int   total_points = cloud->size();
    float left_ratio   = (float)left_flange / total_points;
    float right_ratio  = (float)right_flange / total_points;
    float center_ratio = (float)center_region / total_points;

    // 工字钢：两侧翼缘都应该有显著点数，中心水平带点数相对少
    bool has_left_flange  = left_ratio > 0.1f;
    bool has_right_flange = right_ratio > 0.1f;
    bool center_sparse    = center_ratio < 0.3f;

    std::cout << "       截面验证: 左翼=" << left_ratio << ", 右翼=" << right_ratio << ", 中心=" << center_ratio
              << std::endl;

    if (has_left_flange && has_right_flange && center_sparse) {
        std::cout << "       截面验证: ✓ 符合工字钢截面" << std::endl;
        return true;
    }

    // 圆柱特征：点分布较均匀，或集中在弧形区域
    std::cout << "       截面验证: ✗ 不符合工字钢截面（可能是圆柱）" << std::endl;
    return false;
}

// ============================================================================
// 单切片工字钢截面评分 - 检查某一段切片是否具有工字钢的双翼缘分布特征
// 返回: 0.0~1.0 的置信度分数, 越高越像 H 型截面
// ============================================================================
static float computeSliceIBeamScore(const std::vector<float> &slice_v,
                                     const std::vector<float> &slice_w,
                                     int histogram_bins = 25)
{
    if (slice_v.size() < 8) return 0.0f;

    float v_min = *std::min_element(slice_v.begin(), slice_v.end());
    float v_max = *std::max_element(slice_v.begin(), slice_v.end());
    float v_span = v_max - v_min;
    if (v_span < 0.03f) return 0.0f;

    // 构建 v 方向直方图
    std::vector<int> hist(histogram_bins, 0);
    float step = v_span / histogram_bins;
    for (float v : slice_v) {
        int idx = std::min(histogram_bins - 1, int((v - v_min) / step));
        hist[idx]++;
    }

    // 平滑
    std::vector<float> smooth(histogram_bins, 0);
    for (int i = 1; i < histogram_bins - 1; i++)
        smooth[i] = (hist[i - 1] + hist[i] + hist[i + 1]) / 3.0f;
    smooth[0] = (hist[0] + hist[1]) / 2.0f;
    smooth[histogram_bins - 1] = (hist[histogram_bins - 2] + hist[histogram_bins - 1]) / 2.0f;

    float max_val = *std::max_element(smooth.begin(), smooth.end());
    if (max_val < 1.0f) return 0.0f;

    float peak_thresh = max_val * 0.25f;
    std::vector<int> peaks;
    for (int i = 1; i < histogram_bins - 1; i++) {
        if (smooth[i] > peak_thresh && smooth[i] >= smooth[i - 1] && smooth[i] >= smooth[i + 1]) {
            if (peaks.empty() || i - peaks.back() > 2)
                peaks.push_back(i);
        }
    }

    if (peaks.size() != 2) return 0.0f;

    float peak_dist = (peaks[1] - peaks[0]) * step;
    if (peak_dist < 0.02f || peak_dist > 0.5f) return 0.0f;

    // 检查谷底深度
    float valley_min = std::numeric_limits<float>::max();
    for (int i = peaks[0]; i <= peaks[1]; i++)
        valley_min = std::min(valley_min, smooth[i]);

    float valley_ratio = valley_min / max_val;
    // valley_ratio 越低说明越像 H 型截面; 0.0 表示完美 H 型, 1.0 表示无谷
    float score = std::max(0.0f, 1.0f - valley_ratio * 2.0f);
    return score;
}

/// @brief 精细化检测,Python 的 refine_ibeam_detection
/// @param cluster_cloud 待精细化的聚类点云
/// @param main_axis 主轴方向
/// @param center 初始中心
/// @param refined_cloud 精细化后的点云
/// @param refined_center 精细化后的中心
/// @param refined_dims 精细化后的尺寸
/// @return 是否成功
bool IBeamFitter::refineIBeamDetection(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cluster_cloud,
                                       const Eigen::Vector3d &main_axis, const Eigen::Vector3d &center,
                                       pcl::PointCloud<pcl::PointXYZ>::Ptr &refined_cloud,
                                       Eigen::Vector3d &refined_center, Eigen::Vector3d &refined_dims) {

    if (cluster_cloud->size() < 50) {
        return false;
    }

    // 确保主轴方向一致
    Eigen::Vector3d u = main_axis;
    if (u[0] < 0) {
        u = -u;
    }
    u.normalize();

    // 构建局部坐标系
    Eigen::Vector3d w(0, 0, 1); // 垂直向上
    Eigen::Vector3d v = w.cross(u);
    v.normalize();

    // 转换点到局部坐标系
    std::vector<double> local_u, local_v, local_w;
    for (const auto &pt : cluster_cloud->points) {
        Eigen::Vector3d p(pt.x - center[0], pt.y - center[1], pt.z - center[2]);
        local_u.push_back(p.dot(u));
        local_v.push_back(p.dot(v));
        local_w.push_back(p.dot(w));
    }

    // 分析参数
    double slice_width = 0.2;
    double u_min       = *std::min_element(local_u.begin(), local_u.end());
    double u_max       = *std::max_element(local_u.begin(), local_u.end());
    int    n_slices    = std::max(5, static_cast<int>((u_max - u_min) / slice_width));

    // 收集有效切片信息
    std::vector<bool>   valid_mask(cluster_cloud->size(), false);
    std::vector<double> slice_v_centers, slice_w_centers;
    std::vector<double> slice_v_spans, slice_w_spans;
    int                 valid_slices = 0;

    for (int i = 0; i < n_slices; ++i) {
        double u_start = u_min + i * (u_max - u_min) / n_slices;
        double u_end   = u_start + (u_max - u_min) / n_slices;

        std::vector<int> slice_indices;
        for (size_t j = 0; j < local_u.size(); ++j) {
            if (local_u[j] >= u_start && local_u[j] < u_end) {
                slice_indices.push_back(j);
            }
        }

        if (slice_indices.size() < 3) continue;

        double v_min_slice = std::numeric_limits<double>::max();
        double v_max_slice = std::numeric_limits<double>::lowest();
        double w_min_slice = std::numeric_limits<double>::max();
        double w_max_slice = std::numeric_limits<double>::lowest();

        for (int idx : slice_indices) {
            v_min_slice = std::min(v_min_slice, local_v[idx]);
            v_max_slice = std::max(v_max_slice, local_v[idx]);
            w_min_slice = std::min(w_min_slice, local_w[idx]);
            w_max_slice = std::max(w_max_slice, local_w[idx]);
        }

        double v_span = v_max_slice - v_min_slice;
        double w_span = w_max_slice - w_min_slice;

        if (v_span > detection_params_.refine_min_v_span && v_span < detection_params_.refine_max_v_span &&
            w_span > detection_params_.refine_min_w_span && w_span < detection_params_.refine_max_w_span) {

            for (int idx : slice_indices) {
                valid_mask[idx] = true;
            }
            valid_slices++;
            slice_v_centers.push_back((v_max_slice + v_min_slice) / 2);
            slice_w_centers.push_back((w_max_slice + w_min_slice) / 2);
            slice_v_spans.push_back(v_span);
            slice_w_spans.push_back(w_span);
        }
    }

    // 如果有效切片太少，使用宽松模式
    if (valid_slices < 3) {
        std::fill(valid_mask.begin(), valid_mask.end(), true);
        double v_sum = 0, w_sum = 0;
        for (size_t i = 0; i < local_v.size(); ++i) {
            v_sum += local_v[i];
            w_sum += local_w[i];
        }
        slice_v_centers.push_back(v_sum / local_v.size());
        slice_w_centers.push_back(w_sum / local_w.size());
        std::cout << "       精细化: 使用宽松模式" << std::endl;
    }

    // 计算典型截面中心
    double median_v = slice_v_centers[slice_v_centers.size() / 2];
    double median_w = slice_w_centers[slice_w_centers.size() / 2];

    double typical_v_span = slice_v_spans.empty() ? 0.3 : slice_v_spans[slice_v_spans.size() / 2];
    double typical_w_span = slice_w_spans.empty() ? 0.4 : slice_w_spans[slice_w_spans.size() / 2];

    // 收紧容差：更严格地过滤端部大截面结构
    double v_tolerance = typical_v_span * 0.4 + 0.025;
    double w_tolerance = typical_w_span * 0.45 + 0.03;

    std::cout << "       精细化容差: v_tol=" << v_tolerance << ", w_tol=" << w_tolerance << std::endl;

    // 提取精细化后的点
    refined_cloud->clear();
    std::vector<double> refined_local_u, refined_local_v, refined_local_w;

    for (size_t i = 0; i < cluster_cloud->size(); ++i) {
        if (valid_mask[i] && std::abs(local_v[i] - median_v) < v_tolerance &&
            std::abs(local_w[i] - median_w) < w_tolerance) {
            refined_cloud->points.push_back(cluster_cloud->points[i]);
            refined_local_u.push_back(local_u[i]);
            refined_local_v.push_back(local_v[i]);
            refined_local_w.push_back(local_w[i]);
        }
    }

    if (refined_cloud->size() < 40) {
        std::cout << "       精细化失败: 点数不足 (" << refined_cloud->size() << " < 40)" << std::endl;
        return false;
    }

    // 统计滤波：移除离群点（端部残余）
    size_t                                        before_sor = refined_cloud->size();
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(refined_cloud);
    sor.setMeanK(30);
    sor.setStddevMulThresh(1.0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr sor_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    sor.filter(*sor_filtered);

    std::cout << "       精细化统计滤波: " << before_sor << " -> " << sor_filtered->size() << " 点" << std::endl;

    if (sor_filtered->size() < 40) {
        std::cout << "       精细化失败: 统计滤波后点数不足 (" << sor_filtered->size() << " < 40)" << std::endl;
        return false;
    }

    refined_cloud = sor_filtered;

    // 重新计算局部坐标（滤波后点集可能变化）
    refined_local_u.clear();
    refined_local_v.clear();
    refined_local_w.clear();
    for (const auto &pt : refined_cloud->points) {
        Eigen::Vector3d p(pt.x - center[0], pt.y - center[1], pt.z - center[2]);
        refined_local_u.push_back(p.dot(u));
        refined_local_v.push_back(p.dot(v));
        refined_local_w.push_back(p.dot(w));
    }

    refined_cloud->width  = refined_cloud->points.size();
    refined_cloud->height = 1;

    // ============================================================================
    // 端部切片检测：去除端部连接结构（圆柱/法兰/支撑帽）
    // 合并两种检测方式：1. 截面尺寸突变  2. 切片直径突变
    // ============================================================================
    {
        double min_u        = *std::min_element(refined_local_u.begin(), refined_local_u.end());
        double max_u        = *std::max_element(refined_local_u.begin(), refined_local_u.end());
        double rough_length = max_u - min_u;

        const double slice_step       = 0.12; // 切片步长
        int          num_slices       = std::max(5, static_cast<int>(rough_length / slice_step));
        double       actual_slice_len = rough_length / num_slices;

        // 收集每个切片的信息
        struct SliceInfo {
            double u_start, u_end;
            double v_span, w_span;
            double diameter; // 切片包围盒对角线
            int    point_count;
        };
        std::vector<SliceInfo> slices(num_slices);

        for (int i = 0; i < num_slices; ++i) {
            slices[i].u_start     = min_u + i * actual_slice_len;
            slices[i].u_end       = slices[i].u_start + actual_slice_len;
            slices[i].point_count = 0;

            double v_min = std::numeric_limits<double>::max();
            double v_max = std::numeric_limits<double>::lowest();
            double w_min = std::numeric_limits<double>::max();
            double w_max = std::numeric_limits<double>::lowest();

            pcl::PointCloud<pcl::PointXYZ>::Ptr slice_cloud(new pcl::PointCloud<pcl::PointXYZ>);

            for (size_t j = 0; j < refined_local_u.size(); ++j) {
                if (refined_local_u[j] >= slices[i].u_start && refined_local_u[j] < slices[i].u_end) {
                    slices[i].point_count++;
                    v_min = std::min(v_min, refined_local_v[j]);
                    v_max = std::max(v_max, refined_local_v[j]);
                    w_min = std::min(w_min, refined_local_w[j]);
                    w_max = std::max(w_max, refined_local_w[j]);
                    slice_cloud->push_back(refined_cloud->points[j]);
                }
            }

            if (slices[i].point_count > 0) {
                slices[i].v_span = v_max - v_min;
                slices[i].w_span = w_max - w_min;
                // 计算切片直径（包围盒对角线）
                if (slice_cloud->size() >= 3) {
                    pcl::PointXYZ pt_min, pt_max;
                    pcl::getMinMax3D<pcl::PointXYZ>(*slice_cloud, pt_min, pt_max);
                    slices[i].diameter = std::sqrt(std::pow(pt_max.x - pt_min.x, 2) + std::pow(pt_max.y - pt_min.y, 2) +
                                                   std::pow(pt_max.z - pt_min.z, 2));
                } else {
                    slices[i].diameter =
                        std::sqrt(slices[i].v_span * slices[i].v_span + slices[i].w_span * slices[i].w_span);
                }
            } else {
                slices[i].v_span   = 0;
                slices[i].w_span   = 0;
                slices[i].diameter = 0;
            }
        }

        // 计算中间段（排除两端各25%）的典型值
        int                 start_idx = num_slices / 4;
        int                 end_idx   = num_slices - num_slices / 4;
        std::vector<double> mid_diams, mid_v_spans, mid_w_spans;

        for (int i = start_idx; i < end_idx; ++i) {
            if (slices[i].point_count >= 3) {
                mid_diams.push_back(slices[i].diameter);
                mid_v_spans.push_back(slices[i].v_span);
                mid_w_spans.push_back(slices[i].w_span);
            }
        }

        if (mid_diams.size() >= 2) {
            std::sort(mid_diams.begin(), mid_diams.end());
            std::sort(mid_v_spans.begin(), mid_v_spans.end());
            std::sort(mid_w_spans.begin(), mid_w_spans.end());

            double typical_diam = mid_diams[mid_diams.size() / 2];
            double typical_v    = mid_v_spans[mid_v_spans.size() / 2];
            double typical_w    = mid_w_spans[mid_w_spans.size() / 2];

            // 异常阈值：比典型值大 outlier_ratio 以上（过滤端部圆柱/法兰）
            double diam_threshold = typical_diam * detection_params_.trim_outlier_ratio;
            double v_threshold    = typical_v * detection_params_.trim_outlier_ratio;
            double w_threshold    = typical_w * detection_params_.trim_outlier_ratio;

            std::cout << "       端部检测: 典型截面 diam=" << typical_diam << ", v=" << typical_v << ", w=" << typical_w
                      << std::endl;

            // 从左端向右扫描
            int left_trim = 0;
            for (int i = 0; i < num_slices / 3; ++i) {
                if (slices[i].point_count < 3) continue;
                bool is_abnormal = (slices[i].diameter > diam_threshold) || (slices[i].v_span > v_threshold) ||
                                   (slices[i].w_span > w_threshold);
                if (is_abnormal) {
                    left_trim = i + 1;
                    std::cout << "       端部检测: 左端切片#" << i << " 异常 (diam=" << slices[i].diameter
                              << ", v=" << slices[i].v_span << ", w=" << slices[i].w_span << ")" << std::endl;
                } else {
                    break;
                }
            }

            // 从右端向左扫描
            int right_trim = num_slices;
            for (int i = num_slices - 1; i >= num_slices * 2 / 3; --i) {
                if (slices[i].point_count < 3) continue;
                bool is_abnormal = (slices[i].diameter > diam_threshold) || (slices[i].v_span > v_threshold) ||
                                   (slices[i].w_span > w_threshold);
                if (is_abnormal) {
                    right_trim = i;
                    std::cout << "       端部检测: 右端切片#" << i << " 异常 (diam=" << slices[i].diameter
                              << ", v=" << slices[i].v_span << ", w=" << slices[i].w_span << ")" << std::endl;
                } else {
                    break;
                }
            }

            // 执行裁剪
            if (left_trim > 0 || right_trim < num_slices) {
                double new_u_min = (left_trim < num_slices) ? slices[left_trim].u_start : min_u;
                double new_u_max = (right_trim > 0) ? slices[right_trim - 1].u_end : max_u;

                pcl::PointCloud<pcl::PointXYZ>::Ptr trimmed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                std::vector<double>                 trimmed_u, trimmed_v, trimmed_w;

                for (size_t i = 0; i < refined_cloud->size(); ++i) {
                    if (refined_local_u[i] >= new_u_min && refined_local_u[i] <= new_u_max) {
                        trimmed_cloud->points.push_back(refined_cloud->points[i]);
                        trimmed_u.push_back(refined_local_u[i]);
                        trimmed_v.push_back(refined_local_v[i]);
                        trimmed_w.push_back(refined_local_w[i]);
                    }
                }

                std::cout << "       端部检测: 裁剪 " << refined_cloud->size() << " -> " << trimmed_cloud->size()
                          << " 点" << std::endl;

                if (trimmed_cloud->size() >= 40) {
                    refined_cloud         = trimmed_cloud;
                    refined_cloud->width  = refined_cloud->size();
                    refined_cloud->height = 1;
                    refined_local_u       = trimmed_u;
                    refined_local_v       = trimmed_v;
                    refined_local_w       = trimmed_w;
                } else {
                    std::cout << "       端部检测: 裁剪后点数不足，保持原样" << std::endl;
                }
            } else {
                std::cout << "       端部检测: 未发现异常端部" << std::endl;
            }
        }
    }

    // ============================================================================
    // 端部截面形状一致性检测：去除不具有工字钢双翼缘截面特征的端部区域
    // 目的：过滤被错误包含的支架、连接件、噪声等非工字钢结构
    // ============================================================================
    {
        double min_u_shape = *std::min_element(refined_local_u.begin(), refined_local_u.end());
        double max_u_shape = *std::max_element(refined_local_u.begin(), refined_local_u.end());
        double length_shape = max_u_shape - min_u_shape;

        if (length_shape > 0.5 && refined_cloud->size() >= 50) {
            const int num_shape_slices = std::max(8, static_cast<int>(length_shape / 0.10));
            double shape_slice_len = length_shape / num_shape_slices;

            struct ShapeSliceInfo {
                double ibeam_score;      // H型截面置信度
                double v_center;         // v方向中心
                double w_center;         // w方向中心
                double v_span;
                double w_span;
                double aspect_ratio;     // v_span / w_span
                int    point_count;
            };

            std::vector<ShapeSliceInfo> shape_slices(num_shape_slices);

            for (int s = 0; s < num_shape_slices; ++s) {
                double s_start = min_u_shape + s * shape_slice_len;
                double s_end = s_start + shape_slice_len;

                std::vector<float> sv, sw;
                for (size_t j = 0; j < refined_local_u.size(); ++j) {
                    if (refined_local_u[j] >= s_start && refined_local_u[j] < s_end) {
                        sv.push_back(static_cast<float>(refined_local_v[j]));
                        sw.push_back(static_cast<float>(refined_local_w[j]));
                    }
                }

                shape_slices[s].point_count = static_cast<int>(sv.size());
                if (sv.size() >= 5) {
                    float sv_min = *std::min_element(sv.begin(), sv.end());
                    float sv_max = *std::max_element(sv.begin(), sv.end());
                    float sw_min = *std::min_element(sw.begin(), sw.end());
                    float sw_max = *std::max_element(sw.begin(), sw.end());
                    shape_slices[s].v_span = sv_max - sv_min;
                    shape_slices[s].w_span = sw_max - sw_min;
                    shape_slices[s].v_center = (sv_max + sv_min) / 2.0;
                    shape_slices[s].w_center = (sw_max + sw_min) / 2.0;
                    shape_slices[s].aspect_ratio = shape_slices[s].v_span / std::max(shape_slices[s].w_span, 0.01);
                    shape_slices[s].ibeam_score = computeSliceIBeamScore(sv, sw);
                } else {
                    shape_slices[s] = {0, 0, 0, 0, 0, 0, shape_slices[s].point_count};
                }
            }

            // 计算中间段（排除两端各25%）作为参考基准
            int ref_start = num_shape_slices / 4;
            int ref_end = num_shape_slices - num_shape_slices / 4;

            std::vector<double> ref_scores, ref_v_centers, ref_w_centers, ref_aspects, ref_densities;
            for (int s = ref_start; s < ref_end; ++s) {
                if (shape_slices[s].point_count >= 5) {
                    ref_scores.push_back(shape_slices[s].ibeam_score);
                    ref_v_centers.push_back(shape_slices[s].v_center);
                    ref_w_centers.push_back(shape_slices[s].w_center);
                    ref_aspects.push_back(shape_slices[s].aspect_ratio);
                    ref_densities.push_back(static_cast<double>(shape_slices[s].point_count));
                }
            }

            if (ref_scores.size() >= 2) {
                std::sort(ref_scores.begin(), ref_scores.end());
                std::sort(ref_v_centers.begin(), ref_v_centers.end());
                std::sort(ref_w_centers.begin(), ref_w_centers.end());
                std::sort(ref_aspects.begin(), ref_aspects.end());
                std::sort(ref_densities.begin(), ref_densities.end());

                double ref_score_med = ref_scores[ref_scores.size() / 2];
                double ref_v_med = ref_v_centers[ref_v_centers.size() / 2];
                double ref_w_med = ref_w_centers[ref_w_centers.size() / 2];
                double ref_aspect_med = ref_aspects[ref_aspects.size() / 2];
                double ref_density_med = ref_densities[ref_densities.size() / 2];

                std::cout << "       截面形状一致性: 参考H型分数=" << ref_score_med
                          << ", v_center=" << ref_v_med
                          << ", w_center=" << ref_w_med
                          << ", 长宽比=" << ref_aspect_med << std::endl;

                // 端部异常判定函数
                auto isSliceAbnormal = [&](int s) -> std::pair<bool, std::string> {
                    if (shape_slices[s].point_count < 3) return {false, ""};
                    bool bad = false;
                    std::string reason;

                    // 条件1: H型截面分数远低于中间段参考值
                    // 条件1: H型截面分数远低于中间段参考值
                    if (ref_score_med > 0.2 && shape_slices[s].ibeam_score < ref_score_med * detection_params_.trim_score_ratio) {
                        bad = true;
                        reason += "H分数低(" + std::to_string(shape_slices[s].ibeam_score).substr(0,4) + ")";
                    }

                    // 条件2: v方向中心偏移过大（侧向偏移，可能是支架/连接件）
                    double v_shift = std::abs(shape_slices[s].v_center - ref_v_med);
                    if (v_shift > detection_params_.trim_v_shift_threshold) {
                        bad = true;
                        reason += " v偏移=" + std::to_string(v_shift).substr(0,5);
                    }

                    // 条件3: w方向中心偏移过大（高度偏移）
                    double w_shift = std::abs(shape_slices[s].w_center - ref_w_med);
                    if (w_shift > detection_params_.trim_w_shift_threshold) {
                        bad = true;
                        reason += " w偏移=" + std::to_string(w_shift).substr(0,5);
                    }

                    // 条件4: 截面长宽比与参考段差异超过阈值
                    if (ref_aspect_med > 0.1) {
                        double aspect_diff = std::abs(shape_slices[s].aspect_ratio - ref_aspect_med) / ref_aspect_med;
                        if (aspect_diff > detection_params_.trim_aspect_diff_ratio) {
                            bad = true;
                            reason += " 长宽比偏差=" + std::to_string(aspect_diff).substr(0,4);
                        }
                    }

                    // 条件5: 点密度异常（过高=密集连接件, 过低=稀疏噪声）
                    if (ref_density_med > 5) {
                        double density_ratio = shape_slices[s].point_count / ref_density_med;
                        if (density_ratio > detection_params_.trim_density_ratio_high || density_ratio < detection_params_.trim_density_ratio_low) {
                            bad = true;
                            reason += " 密度比=" + std::to_string(density_ratio).substr(0,4);
                        }
                    }

                    return {bad, reason};
                };

                // 从左端向右扫描
                int left_shape_trim = 0;
                for (int s = 0; s < num_shape_slices / 3; ++s) {
                    if (shape_slices[s].point_count < 3) continue;
                    auto [is_bad, reason] = isSliceAbnormal(s);
                    if (is_bad) {
                        left_shape_trim = s + 1;
                        std::cout << "       截面一致性: 左端切片#" << s << " 异常: " << reason << std::endl;
                    } else {
                        break;
                    }
                }

                // 从右端向左扫描
                int right_shape_trim = num_shape_slices;
                for (int s = num_shape_slices - 1; s >= num_shape_slices * 2 / 3; --s) {
                    if (shape_slices[s].point_count < 3) continue;
                    auto [is_bad, reason] = isSliceAbnormal(s);
                    if (is_bad) {
                        right_shape_trim = s;
                        std::cout << "       截面一致性: 右端切片#" << s << " 异常: " << reason << std::endl;
                    } else {
                        break;
                    }
                }

                // 执行裁剪
                if (left_shape_trim > 0 || right_shape_trim < num_shape_slices) {
                    double new_u_min_s = min_u_shape + left_shape_trim * shape_slice_len;
                    double new_u_max_s = min_u_shape + right_shape_trim * shape_slice_len;

                    pcl::PointCloud<pcl::PointXYZ>::Ptr shape_trimmed(new pcl::PointCloud<pcl::PointXYZ>);
                    std::vector<double> st_u, st_v, st_w;

                    for (size_t i = 0; i < refined_cloud->size(); ++i) {
                        if (refined_local_u[i] >= new_u_min_s && refined_local_u[i] <= new_u_max_s) {
                            shape_trimmed->points.push_back(refined_cloud->points[i]);
                            st_u.push_back(refined_local_u[i]);
                            st_v.push_back(refined_local_v[i]);
                            st_w.push_back(refined_local_w[i]);
                        }
                    }

                    std::cout << "       截面一致性裁剪: " << refined_cloud->size() << " -> " << shape_trimmed->size()
                              << " 点 (左端去除" << left_shape_trim << "片, 右端去除"
                              << (num_shape_slices - right_shape_trim) << "片)" << std::endl;

                    if (shape_trimmed->size() >= 40) {
                        refined_cloud = shape_trimmed;
                        refined_cloud->width = refined_cloud->size();
                        refined_cloud->height = 1;
                        refined_local_u = st_u;
                        refined_local_v = st_v;
                        refined_local_w = st_w;
                    } else {
                        std::cout << "       截面一致性裁剪: 裁剪后点数不足，保持原样" << std::endl;
                    }
                } else {
                    std::cout << "       截面一致性: 两端截面形状一致，无需裁剪" << std::endl;
                }
            }
        }
    }

    if (refined_cloud->size() < 30) return false;

    // 计算精细化后的尺寸
    refined_dims[0] = *std::max_element(refined_local_u.begin(), refined_local_u.end()) -
                      *std::min_element(refined_local_u.begin(), refined_local_u.end());
    refined_dims[1] = *std::max_element(refined_local_v.begin(), refined_local_v.end()) -
                      *std::min_element(refined_local_v.begin(), refined_local_v.end());
    refined_dims[2] = *std::max_element(refined_local_w.begin(), refined_local_w.end()) -
                      *std::min_element(refined_local_w.begin(), refined_local_w.end());

    // 计算精细化后的中心
    refined_center = Eigen::Vector3d::Zero();
    for (const auto &pt : refined_cloud->points) {
        refined_center += Eigen::Vector3d(pt.x, pt.y, pt.z);
    }
    refined_center /= refined_cloud->size();

    // ============================================================================
    // 综合验证：连续性 + 双翼缘 + 截面形状
    // ============================================================================
    Eigen::Vector3f axis_f   = u.cast<float>();
    Eigen::Vector3f center_f = refined_center.cast<float>();

    // 1. 点云连续性检测（检查是否有间隙），如果有间隙则分割
    if (!checkPointCloudContinuity(refined_cloud, axis_f, center_f)) {
        std::cout << "       发现间隙，尝试分割取最大连续段..." << std::endl;

        // 沿主轴投影并排序，找到最大间隙位置
        std::vector<std::pair<float, size_t>> projections; // (投影值, 原索引)
        for (size_t i = 0; i < refined_cloud->size(); ++i) {
            Eigen::Vector3f pt(refined_cloud->points[i].x, refined_cloud->points[i].y, refined_cloud->points[i].z);
            float           proj = (pt - center_f).dot(axis_f);
            projections.push_back({proj, i});
        }
        std::sort(projections.begin(), projections.end());

        // 找最大间隙位置
        float  max_gap = 0;
        size_t gap_pos = 0;
        for (size_t i = 1; i < projections.size(); ++i) {
            float gap = projections[i].first - projections[i - 1].first;
            if (gap > max_gap) {
                max_gap = gap;
                gap_pos = i;
            }
        }

        if (max_gap > 0.2f && gap_pos > 0) {
            // 分割：尝试两段，选择更符合工字钢特征的那段
            size_t left_count  = gap_pos;
            size_t right_count = projections.size() - gap_pos;

            // 构建左段和右段
            pcl::PointCloud<pcl::PointXYZ>::Ptr left_seg(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr right_seg(new pcl::PointCloud<pcl::PointXYZ>);
            std::vector<double> left_u, left_v, left_w, right_u, right_v, right_w;

            for (size_t i = 0; i < gap_pos; ++i) {
                size_t idx = projections[i].second;
                left_seg->push_back(refined_cloud->points[idx]);
                left_u.push_back(refined_local_u[idx]);
                left_v.push_back(refined_local_v[idx]);
                left_w.push_back(refined_local_w[idx]);
            }
            for (size_t i = gap_pos; i < projections.size(); ++i) {
                size_t idx = projections[i].second;
                right_seg->push_back(refined_cloud->points[idx]);
                right_u.push_back(refined_local_u[idx]);
                right_v.push_back(refined_local_v[idx]);
                right_w.push_back(refined_local_w[idx]);
            }

            std::cout << "       分割: 左段 " << left_seg->size() << " 点, 右段 " << right_seg->size()
                      << " 点 (间隙=" << max_gap << "m)" << std::endl;

            // 对两段分别评估工字钢特征（连续性+结构+尺寸）
            auto evaluateSegment = [&](const pcl::PointCloud<pcl::PointXYZ>::Ptr &seg,
                                       const std::vector<double> &su,
                                       const std::vector<double> &sv,
                                       const std::vector<double> &sw,
                                       const std::string &name) -> int {
                if (seg->size() < 30) return 0;
                int score = 0;
                // 连续性
                Eigen::Vector3f seg_center(0, 0, 0);
                for (const auto &p : seg->points)
                    seg_center += Eigen::Vector3f(p.x, p.y, p.z);
                seg_center /= seg->size();
                if (checkPointCloudContinuity(seg, axis_f, seg_center)) score += 1;
                // 双翼缘（单视点可能无法看到双翼缘）
                if (hasDoubleFlangeStructure(seg, axis_f)) score += 2;
                // 截面形状
                if (verifyIBeamCrossSection(seg, axis_f)) score += 2;
                // 尺寸合理性评分（用于单视点无法进行结构验证时辅助判断）
                if (!su.empty() && !sv.empty() && !sw.empty()) {
                    double s_len = *std::max_element(su.begin(), su.end()) - *std::min_element(su.begin(), su.end());
                    double s_vsp = *std::max_element(sv.begin(), sv.end()) - *std::min_element(sv.begin(), sv.end());
                    double s_wsp = *std::max_element(sw.begin(), sw.end()) - *std::min_element(sw.begin(), sw.end());
                    // 工字钢截面典型范围: 翼缘方向0.03-0.40m, 高度方向0.08-0.55m
                    bool dim_ok = s_len > 0.5 && s_vsp >= 0.03 && s_vsp <= 0.40
                                  && s_wsp >= 0.05 && s_wsp <= 0.55;
                    if (dim_ok) score += 1;
                    std::cout << "       分割评估 " << name << ": 工字钢评分=" << score
                              << "/6 (L=" << s_len << ", V=" << s_vsp << ", W=" << s_wsp << ")" << std::endl;
                } else {
                    std::cout << "       分割评估 " << name << ": 工字钢评分=" << score << "/6" << std::endl;
                }
                return score;
            };

            int left_score  = evaluateSegment(left_seg, left_u, left_v, left_w, "左段");
            int right_score = evaluateSegment(right_seg, right_u, right_v, right_w, "右段");

            pcl::PointCloud<pcl::PointXYZ>::Ptr segment(new pcl::PointCloud<pcl::PointXYZ>);
            std::vector<double> seg_u, seg_v, seg_w;

            // 选择评分更高的段；若评分相同则取较大段
            bool pick_left = false;
            if (left_score > right_score && left_count >= 30) {
                pick_left = true;
            } else if (right_score > left_score && right_count >= 30) {
                pick_left = false;
            } else if (left_count >= right_count && left_count >= 30) {
                pick_left = true;
            } else if (right_count >= 30) {
                pick_left = false;
            } else {
                std::cout << "       精细化失败: 分割后段落太小" << std::endl;
                return false;
            }

            if (pick_left) {
                segment = left_seg; seg_u = left_u; seg_v = left_v; seg_w = left_w;
                std::cout << "       分割: 选择左段 " << segment->size() << " 点 (评分" << left_score << ")" << std::endl;
            } else {
                segment = right_seg; seg_u = right_u; seg_v = right_v; seg_w = right_w;
                std::cout << "       分割: 选择右段 " << segment->size() << " 点 (评分" << right_score << ")" << std::endl;
            }

            // 更新数据
            refined_cloud         = segment;
            refined_cloud->width  = refined_cloud->size();
            refined_cloud->height = 1;
            refined_local_u       = seg_u;
            refined_local_v       = seg_v;
            refined_local_w       = seg_w;

            // 重新计算尺寸和中心
            refined_dims[0] = *std::max_element(refined_local_u.begin(), refined_local_u.end()) -
                              *std::min_element(refined_local_u.begin(), refined_local_u.end());
            refined_dims[1] = *std::max_element(refined_local_v.begin(), refined_local_v.end()) -
                              *std::min_element(refined_local_v.begin(), refined_local_v.end());
            refined_dims[2] = *std::max_element(refined_local_w.begin(), refined_local_w.end()) -
                              *std::min_element(refined_local_w.begin(), refined_local_w.end());

            refined_center = Eigen::Vector3d::Zero();
            for (const auto &pt : refined_cloud->points) {
                refined_center += Eigen::Vector3d(pt.x, pt.y, pt.z);
            }
            refined_center /= refined_cloud->size();
            center_f = refined_center.cast<float>();

            // 重新检测连续性
            if (!checkPointCloudContinuity(refined_cloud, axis_f, center_f)) {
                std::cout << "       精细化失败: 分割后仍不连续" << std::endl;
                return false;
            }
        } else {
            std::cout << "       精细化失败: 点云不连续，无法有效分割" << std::endl;
            return false;
        }
    }

    // 2-3. 结构验证（双翼缘 + 截面形状）
    // 注意：单视点激光雷达只能扫描到工字钢面向传感器的一侧，
    // 因此双翼缘和对称截面检测可能对稀疏单视点数据不可靠。
    // 策略：结构检测作为优先验证，失败时用尺寸验证兜底。
    bool flange_ok = hasDoubleFlangeStructure(refined_cloud, axis_f);
    bool section_ok = verifyIBeamCrossSection(refined_cloud, axis_f);

    if (flange_ok && section_ok) {
        std::cout << "       结构验证: ✓ 双翼缘和截面验证均通过" << std::endl;
    } else if (flange_ok || section_ok) {
        std::cout << "       结构验证: ✓ 部分通过 (双翼缘=" << (flange_ok ? "是" : "否")
                  << ", 截面=" << (section_ok ? "是" : "否") << ")" << std::endl;
    } else {
        // 双翼缘和截面验证都失败 — 用尺寸验证兜底
        double final_length = refined_dims[0];
        double final_width  = refined_dims[1];
        double final_height = refined_dims[2];
        double final_aspect = final_length / std::max(final_width, 0.01);

        // 尺寸验证：精细化后的尺寸应符合工字钢的截面特征
        bool dim_ok = final_length >= 0.8
                      && final_width  >= 0.03 && final_width  <= 0.45
                      && final_height >= 0.05 && final_height <= 0.60
                      && final_aspect >= 2.0;

        if (dim_ok) {
            std::cout << "       结构验证: 双翼缘/截面未通过(可能因单视点扫描), 尺寸验证通过"
                      << " (L=" << final_length << ", V=" << final_width
                      << ", W=" << final_height << ", AR=" << final_aspect << ")" << std::endl;
        } else {
            std::cout << "       精细化失败: 结构验证和尺寸验证均未通过"
                      << " (L=" << final_length << ", V=" << final_width
                      << ", W=" << final_height << ", AR=" << final_aspect << ")" << std::endl;
            return false;
        }
    }

    return true;
}

/// @brief 扩展短工字钢 - 对应 Python 的 extend_short_ibeams
/// @param ibeams
/// @param roi_cloud
void IBeamFitter::extendShortIBeams(std::vector<IBeamResult>                  &ibeams,
                                    const pcl::PointCloud<pcl::PointXYZ>::Ptr &roi_cloud) {

    for (auto &beam : ibeams) {
        double length = beam.dimensions[0];

        if (length >= detection_params_.extend_min_target_length) {
            continue;
        }

        std::cout << "\n  尝试扩展工字钢 #" << beam.id << " (当前长度=" << length << "m)" << std::endl;

        Eigen::Vector3d main_axis = beam.main_axis;
        if (main_axis[0] < 0) {
            main_axis = -main_axis;
        }

        Eigen::Vector3d u = main_axis.normalized();
        Eigen::Vector3d w(0, 0, 1);
        Eigen::Vector3d v = w.cross(u).normalized();

        double width  = beam.dimensions[1];
        double height = beam.dimensions[2];
        // 收紧容差：更严格地过滤异形截面（圆柱等）
        double v_tolerance   = std::max(0.15, width * 0.6);
        double w_tolerance   = std::max(0.20, height * 0.6);
        double search_length = detection_params_.extend_min_target_length * 0.6;

        pcl::PointCloud<pcl::PointXYZ>::Ptr extended_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        std::vector<double>                 ext_u, ext_v, ext_w;

        for (const auto &pt : roi_cloud->points) {
            Eigen::Vector3d p(pt.x - beam.center[0], pt.y - beam.center[1], pt.z - beam.center[2]);
            double          local_u = p.dot(u);
            double          local_v = p.dot(v);
            double          local_w = p.dot(w);

            if (std::abs(local_v) < v_tolerance && std::abs(local_w) < w_tolerance && local_u > -search_length &&
                local_u < search_length) {
                extended_cloud->points.push_back(pt);
                ext_u.push_back(local_u);
                ext_v.push_back(local_v);
                ext_w.push_back(local_w);
            }
        }

        if (extended_cloud->size() < 50) {
            std::cout << "    扩展失败：找到的点太少 (" << extended_cloud->size() << ")" << std::endl;
            continue;
        }

        double new_length =
            *std::max_element(ext_u.begin(), ext_u.end()) - *std::min_element(ext_u.begin(), ext_u.end());
        double new_width =
            *std::max_element(ext_v.begin(), ext_v.end()) - *std::min_element(ext_v.begin(), ext_v.end());
        double new_height =
            *std::max_element(ext_w.begin(), ext_w.end()) - *std::min_element(ext_w.begin(), ext_w.end());

        if (new_length < length * 1.1) {
            std::cout << "    扩展效果不明显：" << length << "m -> " << new_length << "m" << std::endl;
            continue;
        }

        if (new_width > 0.5 || new_height > 0.7) {
            std::cout << "    扩展后截面异常：" << new_width << " x " << new_height << "m" << std::endl;
            continue;
        }

        // 扩展后进行端部切片过滤：去除可能包含的圆柱/法兰
        {
            double    ext_u_min        = *std::min_element(ext_u.begin(), ext_u.end());
            double    ext_u_max        = *std::max_element(ext_u.begin(), ext_u.end());
            double    ext_length       = ext_u_max - ext_u_min;
            const int num_check_slices = std::max(5, static_cast<int>(ext_length / 0.15));
            double    slice_len        = ext_length / num_check_slices;

            // 收集中间段的典型截面尺寸
            std::vector<double> mid_v_spans, mid_w_spans;
            int                 start_slice = num_check_slices / 4;
            int                 end_slice   = num_check_slices - num_check_slices / 4;

            for (int s = start_slice; s < end_slice; ++s) {
                double s_start = ext_u_min + s * slice_len;
                double s_end   = s_start + slice_len;
                double v_min_s = 1e9, v_max_s = -1e9, w_min_s = 1e9, w_max_s = -1e9;
                int    cnt = 0;
                for (size_t k = 0; k < ext_u.size(); ++k) {
                    if (ext_u[k] >= s_start && ext_u[k] < s_end) {
                        v_min_s = std::min(v_min_s, ext_v[k]);
                        v_max_s = std::max(v_max_s, ext_v[k]);
                        w_min_s = std::min(w_min_s, ext_w[k]);
                        w_max_s = std::max(w_max_s, ext_w[k]);
                        cnt++;
                    }
                }
                if (cnt >= 3) {
                    mid_v_spans.push_back(v_max_s - v_min_s);
                    mid_w_spans.push_back(w_max_s - w_min_s);
                }
            }

            if (mid_v_spans.size() >= 2) {
                std::sort(mid_v_spans.begin(), mid_v_spans.end());
                std::sort(mid_w_spans.begin(), mid_w_spans.end());
                double typ_v    = mid_v_spans[mid_v_spans.size() / 2];
                double typ_w    = mid_w_spans[mid_w_spans.size() / 2];
                double v_thresh = typ_v * 1.3;
                double w_thresh = typ_w * 1.3;

                // 从两端裁剪
                double trim_u_min = ext_u_min, trim_u_max = ext_u_max;

                // 左端扫描
                for (int s = 0; s < num_check_slices / 3; ++s) {
                    double s_start = ext_u_min + s * slice_len;
                    double s_end   = s_start + slice_len;
                    double v_min_s = 1e9, v_max_s = -1e9, w_min_s = 1e9, w_max_s = -1e9;
                    int    cnt = 0;
                    for (size_t k = 0; k < ext_u.size(); ++k) {
                        if (ext_u[k] >= s_start && ext_u[k] < s_end) {
                            v_min_s = std::min(v_min_s, ext_v[k]);
                            v_max_s = std::max(v_max_s, ext_v[k]);
                            w_min_s = std::min(w_min_s, ext_w[k]);
                            w_max_s = std::max(w_max_s, ext_w[k]);
                            cnt++;
                        }
                    }
                    if (cnt >= 3 && ((v_max_s - v_min_s) > v_thresh || (w_max_s - w_min_s) > w_thresh)) {
                        trim_u_min = s_end;
                        std::cout << "    扩展端部裁剪: 左端切片#" << s << " 异常" << std::endl;
                    } else {
                        break;
                    }
                }

                // 右端扫描
                for (int s = num_check_slices - 1; s >= num_check_slices * 2 / 3; --s) {
                    double s_start = ext_u_min + s * slice_len;
                    double s_end   = s_start + slice_len;
                    double v_min_s = 1e9, v_max_s = -1e9, w_min_s = 1e9, w_max_s = -1e9;
                    int    cnt = 0;
                    for (size_t k = 0; k < ext_u.size(); ++k) {
                        if (ext_u[k] >= s_start && ext_u[k] < s_end) {
                            v_min_s = std::min(v_min_s, ext_v[k]);
                            v_max_s = std::max(v_max_s, ext_v[k]);
                            w_min_s = std::min(w_min_s, ext_w[k]);
                            w_max_s = std::max(w_max_s, ext_w[k]);
                            cnt++;
                        }
                    }
                    if (cnt >= 3 && ((v_max_s - v_min_s) > v_thresh || (w_max_s - w_min_s) > w_thresh)) {
                        trim_u_max = s_start;
                        std::cout << "    扩展端部裁剪: 右端切片#" << s << " 异常" << std::endl;
                    } else {
                        break;
                    }
                }

                // 应用裁剪
                if (trim_u_min > ext_u_min || trim_u_max < ext_u_max) {
                    pcl::PointCloud<pcl::PointXYZ>::Ptr trimmed(new pcl::PointCloud<pcl::PointXYZ>);
                    std::vector<double>                 trim_ext_u, trim_ext_v, trim_ext_w;
                    for (size_t k = 0; k < extended_cloud->size(); ++k) {
                        if (ext_u[k] >= trim_u_min && ext_u[k] <= trim_u_max) {
                            trimmed->points.push_back(extended_cloud->points[k]);
                            trim_ext_u.push_back(ext_u[k]);
                            trim_ext_v.push_back(ext_v[k]);
                            trim_ext_w.push_back(ext_w[k]);
                        }
                    }
                    if (trimmed->size() >= 40) {
                        extended_cloud = trimmed;
                        ext_u          = trim_ext_u;
                        ext_v          = trim_ext_v;
                        ext_w          = trim_ext_w;
                        new_length     = *std::max_element(ext_u.begin(), ext_u.end()) -
                                     *std::min_element(ext_u.begin(), ext_u.end());
                        new_width = *std::max_element(ext_v.begin(), ext_v.end()) -
                                    *std::min_element(ext_v.begin(), ext_v.end());
                        new_height = *std::max_element(ext_w.begin(), ext_w.end()) -
                                     *std::min_element(ext_w.begin(), ext_w.end());
                        std::cout << "    扩展端部裁剪后: " << new_length << " x " << new_width << " x " << new_height
                                  << "m" << std::endl;
                    }
                }
            }
        }

        std::cout << "    扩展成功：" << length << "m -> " << new_length << "m" << std::endl;
        std::cout << "    新尺寸：" << new_length << " x " << new_width << " x " << new_height << "m" << std::endl;

        // 更新工字钢信息
        Eigen::Vector3d new_center = Eigen::Vector3d::Zero();
        for (const auto &pt : extended_cloud->points) {
            new_center += Eigen::Vector3d(pt.x, pt.y, pt.z);
        }
        new_center /= extended_cloud->size();

        beam.center      = new_center;
        beam.dimensions  = Eigen::Vector3d(new_length, new_width, new_height);
        beam.point_count = extended_cloud->size();
        beam.points      = extended_cloud;
    }
}

/// @brief 距离计算
/// @param point
/// @return std::sqrt(P1,P2)
double IBeamFitter::computeXYDistance(const pcl::PointXYZ &point) {
    return std::sqrt(point.x * point.x + point.y * point.y);
}

int IBeamFitter::findRoot(std::map<int, int> &parent, int x) {
    while (parent[x] != x) {
        parent[x] = parent[parent[x]]; // 路径压缩
        x         = parent[x];
    }
    return x;
}

std::vector<int> IBeamFitter::relabelClusters(const std::vector<int> &labels) {
    std::set<int> unique_labels;
    for (int label : labels) {
        if (label >= 0) {
            unique_labels.insert(label);
        }
    }

    std::map<int, int> label_map;
    int                new_label = 0;
    for (int old_label : unique_labels) {
        label_map[old_label] = new_label++;
    }

    std::vector<int> new_labels = labels;
    for (size_t i = 0; i < labels.size(); ++i) {
        if (labels[i] >= 0) {
            new_labels[i] = label_map[labels[i]];
        }
    }

    return new_labels;
}

std::vector<double> IBeamFitter::getPose() {
    if (results_.empty()) {
        return {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    }

    const auto &result = results_[0];
    auto        euler  = result.orientation.toRotationMatrix().eulerAngles(0, 1, 2);

    return {result.center[0], result.center[1], result.center[2], euler[0], euler[1], euler[2]};
}

IBeamParams IBeamFitter::getParams() {
    if (!results_.empty()) {
        const auto &result = results_[0];
        params_.length     = result.dimensions[0];
        params_.width      = result.dimensions[1];
        params_.height     = result.dimensions[2];
    }
    return params_;
}

/// @brief 可视化线程
void IBeamFitter::viewerThread() {
    while (running_) {
        {
            std::lock_guard<std::mutex> lock(viewer_mutex_);
            viewer_->spinOnce(30);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

/// @brief 计算机械臂在工字钢上的可喷涂长度
/// @param R_max 机械臂最大半径 (mm)
/// @param D_base 底座中心到腹板表面的水平距离 (mm)
/// @param D_work 喷枪物理长度 + 喷涂挂距 (mm)
/// @param height 工字钢腹板高度 (mm)，即 beam_height
/// @param base_z_offset 机械臂底座中心与腹板中心高度线的高度差 (mm)
/// @param error_msg 错误信息(如有)
/// @return 可喷涂的总长度 (mm)，如果计算失败返回 0
double IBeamFitter::calculateBeamSprayingLimit(double R_max, double D_base, double D_work, double height,
                                               double base_z_offset, std::string &error_msg) {
    error_msg.clear();
    // 参数检查
    if (this->ibeam_inputs_.mode_length == 2) { //自动工作区域长度模式
        ROS_INFO("[IBeamFitter::calculateBeamSprayingLimit] Mode: AUTO");
        // 1. 姿态折减后的有效半径
        double R_eff = R_max * 0.75;

        // 2. 机械臂法兰中心距离底座中心的水平深度
        double D_target = D_base - D_work;

        // 3. 计算垂直方向的最极端偏距 (考虑高度差)
        double h_top      = (height / 2.0) - base_z_offset;
        double h_bottom   = -(height / 2.0) - base_z_offset;
        double h_max_dist = std::max(std::abs(h_top), std::abs(h_bottom));

        // 4. 几何可行性检查
        if (std::abs(D_target) > R_eff) {
            error_msg = "错误：安装距离 D_base (" + std::to_string(D_base) + ") 减去工作间距后超出机械臂半径";
            return 0.0;
        }

        double square_val = R_eff * R_eff - D_target * D_target - h_max_dist * h_max_dist;

        if (square_val < 0) {
            error_msg = "错误：在当前高度偏差或安装距离下，机械臂无法够到腹板边缘角点";
            return 0.0;
        }

        // 5. 计算结果
        double l_half       = std::sqrt(square_val);
        double total_length = l_half * 2.0;
        // ROS_INFO("[IBeamFitter::calculateBeamSprayingLimit]\nR_max=%.2fmm, \nD_base=%.2fmm, \nD_work=%.2fmm, "
        //          "\nheight=%.2fmm, \nbase_z_offset=%.2fmm, \nR_eff=%.2fmm, \nD_target=%.2fmm, "
        //          "\nh_top=%.2fmm, \nh_bottom=%.2fmm, \nh_max_dist=%.2fmm, \nsquare_val=%.2f, \nl_half=%.2fmm, "
        //          "\ntotal_length=%.2fmm",
        //          R_max, D_base, D_work, height, base_z_offset, R_eff, D_target, h_top, h_bottom, h_max_dist,
        //          square_val, l_half, total_length);
        ROS_INFO("[IBeamFitter::calculateBeamSprayingLimit], TOTAL_LENGTH = %.2fmm", total_length);
        return total_length;
    } else { //手动工作区域长度模式
        ROS_INFO("[IBeamFitter::calculateBeamSprayingLimit] Mode: MANUAL");

        double user_slice_length = ibeam_inputs_.length; // 用户指定的切片长度 (mm)

        if (user_slice_length <= 0) {
            error_msg = "错误：手动模式下 length 参数无效 (length=" + std::to_string(user_slice_length) + ")";
            return 0.0;
        }

        if (results_.empty()) {
            error_msg = "错误：手动模式下无工字钢检测结果，无法计算总长度";
            return 0.0;
        }

        // 获取识别到的工字钢物理总长度 (m → mm)
        double beam_total_length = results_[0].dimensions[0] * 1000.0;

        if (beam_total_length <= 0) {
            error_msg = "错误：工字钢检测长度无效 (beam_total=" + std::to_string(beam_total_length) + ")";
            return 0.0;
        }

        // 向上取整计算切片数，保证 adjusted_slice_length <= user_slice_length（只缩小不增大）
        int n_slices = static_cast<int>(std::ceil(beam_total_length / user_slice_length));
        if (n_slices < 1) n_slices = 1;

        // 均匀切分后的实际切片长度（<= user_slice_length）
        double adjusted_slice_length = beam_total_length / static_cast<double>(n_slices);

        ROS_INFO("[IBeamFitter::calculateBeamSprayingLimit] Manual length mode "
                 "beam_total=%.2f mm, user_slice_length=%.2f mm, n_slices=%d, adjusted_slice_length=%.2f mm",
                 beam_total_length, user_slice_length, n_slices, adjusted_slice_length);

        // 返回总工作长度（= n_slices * adjusted_slice_length = beam_total_length）
        return adjusted_slice_length;
    }
}

/// @brief 通过分析截面形状计算翼缘宽度
/// @param beam_result 工字钢检测结果
/// @return 翼缘宽度 (m)，如果计算失败返回 -1
double IBeamFitter::calculateFlangeWidth(const IBeamResult &beam_result) {
    if (!beam_result.points || beam_result.points->empty()) {
        return -1.0;
    }

    // 获取主轴方向
    Eigen::Vector3d u = beam_result.main_axis.normalized();
    if (u[0] < 0) u = -u;

    // 构建局部坐标系
    Eigen::Vector3d w(0, 0, 1);                  // 垂直向上 (截面高度方向)
    Eigen::Vector3d v = w.cross(u).normalized(); // 水平方向 (翼缘方向)

    // 计算点在 v 方向（翼缘方向）的投影
    std::vector<double> v_projections;
    for (const auto &pt : beam_result.points->points) {
        Eigen::Vector3d p(pt.x - beam_result.center[0], pt.y - beam_result.center[1], pt.z - beam_result.center[2]);
        v_projections.push_back(p.dot(v));
    }

    if (v_projections.empty()) {
        return -1.0;
    }

    // 使用直方图分析找到翼缘的实际宽度
    // 工字钢截面呈 H 形，大部分点集中在翼缘的两侧
    double v_min   = *std::min_element(v_projections.begin(), v_projections.end());
    double v_max   = *std::max_element(v_projections.begin(), v_projections.end());
    double v_range = v_max - v_min;

    // 将 v 方向分成多个区间，统计点分布
    const int        num_bins = 50;
    std::vector<int> histogram(num_bins, 0);
    double           bin_width = v_range / num_bins;

    for (double v_proj : v_projections) {
        int bin = std::min(static_cast<int>((v_proj - v_min) / bin_width), num_bins - 1);
        histogram[bin]++;
    }

    // 找到密度最高的区域（对应翼缘位置）
    // 对于H型截面，应该有两个峰值（上下翼缘）
    int max_count = *std::max_element(histogram.begin(), histogram.end());
    int threshold = max_count / 4; // 阈值：最大值的 1/4

    // 找到有效区间（点密度超过阈值的区间）
    int first_valid = -1, last_valid = -1;
    for (int i = 0; i < num_bins; ++i) {
        if (histogram[i] > threshold) {
            if (first_valid < 0) first_valid = i;
            last_valid = i;
        }
    }

    if (first_valid < 0 || last_valid < 0 || last_valid <= first_valid) {
        return -1.0;
    }

    // 翼缘宽度 = 有效区间的范围
    double flange_width = (last_valid - first_valid + 1) * bin_width;

    // 合理性检查：翼缘宽度应该在截面高度的 0.2-0.8 倍之间
    double section_height = beam_result.dimensions[2];
    if (flange_width < section_height * 0.15 || flange_width > section_height * 1.5) {
        // 使用截面高度的 0.4 倍作为默认值
        return section_height * 0.4;
    }

    return flange_width;
}

/// @brief 根据工字钢识别结果和输入参数更新输出结果
/// @param inputs 输入参数
/// @param beam_result 工字钢检测结果
/// @param outputs 输出结果
/// @return 是否成功更新
bool IBeamFitter::updateOutputs(const IBeamInputs &inputs, const IBeamResult &beam_result, IBeamOutputs &outputs) {
    // 1. 直接从输入参数复制
    outputs.distance_tcp2surface = inputs.distance_tcp2surface;

    // 2. 从检测结果中获取工字钢尺寸
    // beam_result.dimensions: (length, v_span, w_span) 单位是米
    // v_span: 水平面内垂直于主轴的跨度（可能包含端部结构，值偏大）
    // w_span: Z方向（垂直方向）的跨度，即截面高度
    double v_span = beam_result.dimensions[1] * 1000.0; // mm 水平跨度（可能偏大）
    double w_span = beam_result.dimensions[2] * 1000.0; // mm 截面高度

    // 工字钢截面高度：使用 Z 方向（w_span），因为工字钢通常水平放置
    outputs.beam_height = w_span;

    // 翼缘宽度计算：优先使用截面形状分析
    double flange_width = calculateFlangeWidth(beam_result);
    if (flange_width > 0) {
        // 截面分析成功
        outputs.wing_plate_width = (flange_width * 1000.0) / 2.0; // 翼缘宽度的一半 (mm)
        ROS_INFO("[IBeamFitter::updateOutputs] Section analysis: Flange Width %.2fmm (Half : %.2fmm)",
                 flange_width * 1000.0, outputs.wing_plate_width);
    } else {
        // 截面分析失败，使用备选方案
        double estimated_flange_width;
        if (v_span < w_span * 1.2 && v_span > 0.1 * w_span) {
            // v_span 在合理范围内，使用它作为翼缘宽度
            estimated_flange_width = v_span;
        } else {
            // v_span 不合理，使用截面高度的 0.4 倍作为估算
            estimated_flange_width = w_span * 0.4;
            ROS_WARN("[IBeamFitter::updateOutputs] v_span(%.2fmm) Error, using estimated flange width %.2fmm", v_span,
                     estimated_flange_width);
        }
        outputs.wing_plate_width = estimated_flange_width / 2.0; // 翼缘宽度的一半 (mm)
    }

    // 3. 计算工作区域长度
    std::string error_msg;
    double      working_length =
        calculateBeamSprayingLimit(inputs.arm_reach,    // R_max: 机械臂最大工作半径
                                   inputs.d_base,       // D_base: 底座中心到腹板表面的水平距离
                                   inputs.d_work,       // D_work: 喷枪物理长度 + 喷涂挂距
                                   outputs.beam_height, // height: 工字钢腹板高度(使用识别出的 beam_height)
                                   inputs.base_z_offset, // base_z_offset: 机械臂底座中心与腹板中心高度线的高度差
                                   error_msg);
    // 更新工字钢工作区域长度
    //inputs.length = working_length; // 将计算得到的工作长度更新回输入参数，供后续使用
    outputs.beam_length_of_working = working_length;
    ROS_INFO("[IBeamFitter::updateOutputs] Calculated working length: %.2fmm (Error: %s)", working_length,
             error_msg.empty() ? "None" : error_msg.c_str());

    // 4. 设置工作区域中心点坐标(机械臂坐标系下)
    outputs.surface_point = beam_result.center;

    // 5. 计算平面方程系数
    // 假设腹板平面的法向量垂直于主轴方向，朝向机器人
    Eigen::Vector3d main_axis = beam_result.main_axis;

    // 构建一个垂直于主轴的平面
    // 平面法向量：从工字钢中心指向原点的方向(在XY平面上的投影)
    Eigen::Vector3d to_origin = -beam_result.center;
    to_origin[2]              = 0; // 投影到XY平面
    double norm               = to_origin.norm();
    if (norm > 1e-6) {
        to_origin.normalize();
    } else {
        // 如果中心点在原点附近，使用默认方向
        to_origin = Eigen::Vector3d(1.0, 0.0, 0.0);
    }

    // 确保法向量垂直于主轴
    Eigen::Vector3d plane_normal = to_origin - to_origin.dot(main_axis) * main_axis;
    norm                         = plane_normal.norm();
    if (norm > 1e-6) {
        plane_normal.normalize();
    } else {
        // 退化情况：使用另一种方式计算
        plane_normal = main_axis.cross(Eigen::Vector3d(0, 0, 1));
        norm         = plane_normal.norm();
        if (norm > 1e-6) {
            plane_normal.normalize();
        } else {
            plane_normal = Eigen::Vector3d(1.0, 0.0, 0.0);
        }
    }

    // 平面方程: ax + by + cz + d = 0
    // d = -(a*x0 + b*y0 + c*z0)
    double d             = -plane_normal.dot(beam_result.center);
    outputs.plane_coeffs = Eigen::Vector4d(plane_normal[0], plane_normal[1], plane_normal[2], d);

    // 6. 设置区域长度方向的单位向量(主轴方向)
    outputs.plane_rx = main_axis.normalized();

    ROS_WARN("[IBeamFitter::updateOutputs] IBeamOutputs (Detect of Beam: L*W*H: %.2f*%.2f*%.2f m):",
             beam_result.dimensions[0], beam_result.dimensions[1], beam_result.dimensions[2]);
    ROS_WARN("  - beam_height : %.2f mm", outputs.beam_height);
    ROS_WARN("  - beam_length_of_working : %.2f mm", outputs.beam_length_of_working);
    ROS_WARN("  - wing_plate_width: %.2f mm", outputs.wing_plate_width);
    ROS_WARN("  - surface_point: (%.2f, %.2f, %.2f) m", outputs.surface_point[0], outputs.surface_point[1],
             outputs.surface_point[2]);
    ROS_WARN("  - plane_coeffs: (%.3f, %.3f, %.3f, %.3f)", outputs.plane_coeffs[0], outputs.plane_coeffs[1],
             outputs.plane_coeffs[2], outputs.plane_coeffs[3]);
    ROS_WARN("  - plane_rx: (%.3f, %.3f, %.3f)", outputs.plane_rx[0], outputs.plane_rx[1], outputs.plane_rx[2]);

    return true;
}

/// @brief 将工字钢工作区域切片为多块区域，便于机械臂工作
/// @param beam_result 工字钢检测结果
/// @param outputs 工字钢输出参数（包含beam_height和beam_length_of_working）
/// @param slice_height_mm 切片高度 (mm)，非0时生效
/// @param slice_width_mm 切片宽度 (mm)，非0时生效
/// @note 两参数都非0时按传入值切片；都为0时使用beam_height作为切片高、beam_length_of_working作为切片宽
void IBeamFitter::sliceIBeam(const IBeamResult &beam_result, const IBeamOutputs &outputs, double slice_height_mm,
                             double slice_width_mm) {
    slices_data_.clear();

    // 获取工字钢参数
    double beam_height_mm       = outputs.beam_height;                // 工字钢截面高度 (mm)
    double beam_total_length_mm = beam_result.dimensions[0] * 1000.0; // 工字钢实际物理总长度 (mm)

    // 参数校验
    if (beam_height_mm <= 0 || beam_total_length_mm <= 0) {
        ROS_WARN("[IBeamFitter::sliceIBeam] Invalid beam dimensions: height=%.2f mm, total_length=%.2f mm",
                 beam_height_mm, beam_total_length_mm);
        return;
    }

    // 确定切片尺寸
    if (slice_height_mm != 0 && slice_width_mm != 0) {
        // 两参数都非0：按传入值切片
        ROS_INFO("[IBeamFitter::sliceIBeam] Using user-specified slice size: %.2f x %.2f mm", slice_height_mm,
                 slice_width_mm);
    } else {
        // 都为0（或任一为0）：使用默认值
        slice_height_mm = beam_height_mm;                 // 工字钢截面高度作为切片高
        slice_width_mm  = outputs.beam_length_of_working; // 计算的工作长度作为切片宽
        ROS_INFO("[IBeamFitter::sliceIBeam] Using default slice size: height=beam_height(%.2f mm), "
                 "width=beam_length_of_working(%.2f mm)",
                 slice_height_mm, slice_width_mm);
    }

    if (slice_height_mm <= 0 || slice_width_mm <= 0) {
        ROS_WARN("[IBeamFitter::sliceIBeam] Invalid slice dimensions: slice_height=%.2f mm, slice_width=%.2f mm",
                 slice_height_mm, slice_width_mm);
        return;
    }

    ROS_INFO("[IBeamFitter::sliceIBeam] Starting I-beam slicing:");
    ROS_INFO("  - Beam height: %.2f mm, Total length: %.2f mm", beam_height_mm, beam_total_length_mm);
    ROS_INFO("  - Slice height: %.2f mm, Slice width: %.2f mm", slice_height_mm, slice_width_mm);

    // 获取工字钢几何信息
    Eigen::Vector3d center    = beam_result.center;                 // 工字钢中心 (m)
    Eigen::Vector3d main_axis = beam_result.main_axis.normalized(); // 主轴方向

    // 确保主轴方向一致性（X分量为正）
    if (main_axis[0] < 0) {
        main_axis = -main_axis;
    }

    // 构建局部坐标系
    // u: 主轴方向（沿工字钢长度方向）
    // w: 垂直向上（Z方向，沿截面高度方向）
    // v: 水平方向（垂直于主轴和Z轴，指向机器人方向）
    Eigen::Vector3d u = main_axis;
    Eigen::Vector3d w(0, 0, 1); // 垂直向上
    Eigen::Vector3d v = w.cross(u);
    if (v.norm() < 1e-6) {
        v = Eigen::Vector3d(0, 1, 0); // 退化情况
    }
    v.normalize();

    // 计算法向量（工字钢腹板平面的几何法向量）
    // v 是垂直于主轴 u 和竖直轴 w 的向量，即腹板法向量
    Eigen::Vector3d normal = v;

    // 确保法向量指向原点/机器人一侧
    // 检查 normal 与 (-center) 的点积，如果小于0说明背离原点，需要翻转
    Eigen::Vector3d to_origin = -center;
    to_origin[2]              = 0; // 仅在水平面上判断方向
    if (normal.dot(to_origin) < 0) {
        normal = -normal;
    }

    // 计算切片数量
    int num_width_slices  = std::max(1, static_cast<int>(std::ceil(beam_total_length_mm / slice_width_mm)));
    int num_height_slices = std::max(1, static_cast<int>(std::ceil(beam_height_mm / slice_height_mm)));

    // 实际每个切片的尺寸（均匀分布）
    double actual_slice_width_mm  = beam_total_length_mm / num_width_slices;
    double actual_slice_height_mm = beam_height_mm / num_height_slices;

    ROS_INFO("  - Number of slices: %d (width) x %d (height) = %d total", num_width_slices, num_height_slices,
             num_width_slices * num_height_slices);
    ROS_INFO("  - Actual slice size: %.2f x %.2f mm", actual_slice_width_mm, actual_slice_height_mm);

    // 计算工作区域的起始位置（沿主轴方向从中心偏移）
    double half_total_length_m = (beam_total_length_mm / 2.0) / 1000.0; // 转换为米
    double half_beam_height_m  = (beam_height_mm / 2.0) / 1000.0;       // 转换为米

    int slice_id = 0;

    // 采用S型扫描顺序：从高到低（便于喷涂时重力影响），奇偶行切换方向
    for (int h_idx = num_height_slices - 1; h_idx >= 0; --h_idx) {
        // 高度方向的偏移（相对于中心）
        double h_offset_m = -half_beam_height_m + (h_idx + 0.5) * (actual_slice_height_mm / 1000.0);

        for (int w_iter = 0; w_iter < num_width_slices; ++w_iter) {
            // S型顺序：偶数行正向，奇数行反向
            int w_idx = ((num_height_slices - 1 - h_idx) % 2 == 0) ? w_iter : (num_width_slices - 1 - w_iter);

            // 沿主轴方向的偏移（相对于中心）
            double u_offset_m = -half_total_length_m + (w_idx + 0.5) * (actual_slice_width_mm / 1000.0);

            // 计算切片中心坐标 (m)
            Eigen::Vector3d slice_center_m = center + u_offset_m * u + h_offset_m * w;

            // 计算作业点坐标 (m)
            // work_point = surface_point + normal * d_base
            double          d_base_m     = ibeam_inputs_.d_base / 1000.0;
            Eigen::Vector3d work_point_m = slice_center_m + normal * d_base_m;

            // 填充切片数据
            IBeamSliceData slice;
            slice.id              = slice_id++;
            slice.layer_index     = w_idx;                         // 沿主轴方向的索引
            slice.row_index       = num_height_slices - 1 - h_idx; // 从上到下的行索引
            slice.center_m        = slice_center_m;
            slice.center_mm       = slice_center_m * 1000.0; // 转换为mm
            slice.work_point_m    = work_point_m;
            slice.work_point_mm   = work_point_m * 1000.0;
            slice.slice_height_mm = actual_slice_height_mm;
            slice.slice_width_mm  = actual_slice_width_mm;
            slice.normal          = normal;
            slice.main_axis       = main_axis;

            slices_data_.push_back(slice);
        }
    }

    ROS_INFO("[IBeamFitter::sliceIBeam] Generated %zu slices", slices_data_.size());

    // 打印切片信息
    for (const auto &slice : slices_data_) {
        ROS_INFO("  Slice #%d: center=(%.3f, %.3f, %.3f)m, size=%.1fx%.1fmm, layer=%d, row=%d", slice.id,
                 slice.center_m[0], slice.center_m[1], slice.center_m[2], slice.slice_width_mm, slice.slice_height_mm,
                 slice.layer_index, slice.row_index);
    }
}

/// @brief 灰色：原始点云
// 白色：ROI区域点云
// 彩色点云：检测到的工字钢（不同工字钢用不同颜色）
// 中心球体：工字钢中心位置
// 箭头：工字钢主轴方向
// OBB边界框：12条边组成的包围盒
// 紫色连线：从原点到工字钢中心
// 3D文本标签：显示ID、尺寸、线性度
void IBeamFitter::visualizeResults() {
    if (results_.empty() && (!cloud_ || cloud_->empty())) {
        return;
    }

    std::lock_guard<std::mutex> lock(viewer_mutex_);

    // 清除之前的可视化内容
    viewer_->removeAllPointClouds();
    viewer_->removeAllShapes();

    // 1. 添加原始点云(灰色)
    if (cloud_ && !cloud_->empty()) {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> gray(cloud_, 128, 128, 128);
        viewer_->addPointCloud<pcl::PointXYZ>(cloud_, gray, "original_cloud");
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "original_cloud");
    }

    // 2. 添加ROI区域点云(白色)
    if (roi_cloud_ && !roi_cloud_->empty()) {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> white(roi_cloud_, 255, 255, 255);
        viewer_->addPointCloud<pcl::PointXYZ>(roi_cloud_, white, "roi_cloud");
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "roi_cloud");
    }

    // 3. 添加坐标系原点球体(表示雷达位置)
    viewer_->addSphere(pcl::PointXYZ(0, 0, 0), 0.05, 1.0, 1.0, 0.0, "origin_sphere");

    // 工字钢颜色表
    std::vector<std::array<double, 3>> colors = {
        {1.0, 0.0, 0.0}, // 红
        {0.0, 1.0, 0.0}, // 绿
        {0.0, 0.0, 1.0}, // 蓝
        {1.0, 1.0, 0.0}, // 黄
        {1.0, 0.0, 1.0}, // 品红
        {0.0, 1.0, 1.0}  // 青
    };

    // 4. 为每个检测到的工字钢添加可视化
    for (size_t i = 0; i < results_.size(); ++i) {
        const auto &beam      = results_[i];
        auto        color     = colors[i % colors.size()];
        std::string id_prefix = "ibeam_" + std::to_string(i);

        // 4.1 添加工字钢点云(彩色)
        if (beam.points && !beam.points->empty()) {
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> beam_color(
                beam.points, static_cast<int>(color[0] * 255), static_cast<int>(color[1] * 255),
                static_cast<int>(color[2] * 255));
            viewer_->addPointCloud<pcl::PointXYZ>(beam.points, beam_color, id_prefix + "_points");
            viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3,
                                                      id_prefix + "_points");
        }

        // 4.2 添加工字钢中心球体
        pcl::PointXYZ center_pt(beam.center[0], beam.center[1], beam.center[2]);
        viewer_->addSphere(center_pt, 0.05, color[0], color[1], color[2], id_prefix + "_center");

        // 4.3 添加主轴箭头(显示工字钢方向)
        double          arrow_length = beam.dimensions[0] * 0.5; // 箭头长度为工字钢长度的一半
        Eigen::Vector3d arrow_end    = beam.center + beam.main_axis.normalized() * arrow_length;
        pcl::PointXYZ   start_pt(beam.center[0], beam.center[1], beam.center[2]);
        pcl::PointXYZ   end_pt(arrow_end[0], arrow_end[1], arrow_end[2]);
        viewer_->addArrow(end_pt, start_pt, color[0], color[1], color[2], false, id_prefix + "_axis");

        // 4.4 添加OBB边界框(使用线段绘制)
        // 计算OBB的8个顶点
        Eigen::Vector3d main_axis = beam.main_axis.normalized();
        Eigen::Vector3d up(0, 0, 1);
        Eigen::Vector3d right = up.cross(main_axis);
        if (right.norm() < 1e-6) {
            right = Eigen::Vector3d(1, 0, 0);
        }
        right.normalize();
        Eigen::Vector3d new_up = main_axis.cross(right).normalized();

        double half_l = beam.dimensions[0] / 2.0;

        // 使用修正后的翼缘宽度（通过截面分析）
        double flange_width = calculateFlangeWidth(beam);
        double actual_width = (flange_width > 0) ? flange_width : beam.dimensions[2] * 0.4;
        // 如果原始宽度在合理范围内，则使用原始值
        if (beam.dimensions[1] < beam.dimensions[2] * 1.2 && beam.dimensions[1] > beam.dimensions[2] * 0.1) {
            actual_width = beam.dimensions[1];
        }
        double half_w = actual_width / 2.0;
        double half_h = beam.dimensions[2] / 2.0;

        // 8个顶点
        std::vector<Eigen::Vector3d> corners(8);
        corners[0] = beam.center - half_l * main_axis - half_w * right - half_h * new_up;
        corners[1] = beam.center + half_l * main_axis - half_w * right - half_h * new_up;
        corners[2] = beam.center + half_l * main_axis + half_w * right - half_h * new_up;
        corners[3] = beam.center - half_l * main_axis + half_w * right - half_h * new_up;
        corners[4] = beam.center - half_l * main_axis - half_w * right + half_h * new_up;
        corners[5] = beam.center + half_l * main_axis - half_w * right + half_h * new_up;
        corners[6] = beam.center + half_l * main_axis + half_w * right + half_h * new_up;
        corners[7] = beam.center - half_l * main_axis + half_w * right + half_h * new_up;

        // 12条边
        std::vector<std::pair<int, int>> edges = {
            {0, 1}, {1, 2}, {2, 3}, {3, 0}, // 底面
            {4, 5}, {5, 6}, {6, 7}, {7, 4}, // 顶面
            {0, 4}, {1, 5}, {2, 6}, {3, 7}  // 侧边
        };

        for (size_t e = 0; e < edges.size(); ++e) {
            int           idx1 = edges[e].first;
            int           idx2 = edges[e].second;
            pcl::PointXYZ p1(corners[idx1][0], corners[idx1][1], corners[idx1][2]);
            pcl::PointXYZ p2(corners[idx2][0], corners[idx2][1], corners[idx2][2]);
            std::string   line_id = id_prefix + "_edge_" + std::to_string(e);
            viewer_->addLine(p1, p2, color[0], color[1], color[2], line_id);
        }

        // 4.5 添加从原点到工字钢中心的连线(紫色)
        viewer_->addLine(pcl::PointXYZ(0, 0, 0), center_pt, 0.8, 0.0, 0.8, id_prefix + "_to_origin");

        // 4.6 添加文本标签
        pcl::PointXYZ text_pos(beam.center[0], beam.center[1], beam.center[2] + half_h + 0.2);
        char          label[128];
        snprintf(label, sizeof(label), "IBeam #%d\n%.2fx%.2fx%.2fm\nL=%.2f", beam.id, beam.dimensions[0], actual_width,
                 beam.dimensions[2], beam.linearity);
        viewer_->addText3D(label, text_pos, 0.08, 1.0, 1.0, 1.0, id_prefix + "_label");

        //可视化切片框（绿色边框 + 蓝色中心点）
        // 切片坐标系: main_axis(u)方向为slice_width, Z(w)方向为slice_height
        for (const auto &slice : slices_data_) {
            // 通过main_axis匹配切片与当前工字钢
            double axis_sim = std::abs(slice.main_axis.normalized().dot(beam.main_axis.normalized()));
            if (axis_sim < 0.95) continue; // 不属于此工字钢

            Eigen::Vector3d sc      = slice.center_m;
            double          half_sw = slice.slice_width_mm / 2000.0;  // 宽度半值(m)，沿main_axis方向
            double          half_sh = slice.slice_height_mm / 2000.0; // 高度半值(m)，沿Z方向

            Eigen::Vector3d u_dir = slice.main_axis.normalized(); // 沿工字钢长度方向
            Eigen::Vector3d w_dir(0, 0, 1);                       // 垂直向上

            // 切片矩形的4个顶点
            Eigen::Vector3d c0 = sc - half_sw * u_dir - half_sh * w_dir;
            Eigen::Vector3d c1 = sc + half_sw * u_dir - half_sh * w_dir;
            Eigen::Vector3d c2 = sc + half_sw * u_dir + half_sh * w_dir;
            Eigen::Vector3d c3 = sc - half_sw * u_dir + half_sh * w_dir;

            std::string sid = id_prefix + "_slice_" + std::to_string(slice.id);

            // 绿色边框 (4条边)
            pcl::PointXYZ p0(c0[0], c0[1], c0[2]), p1(c1[0], c1[1], c1[2]);
            pcl::PointXYZ p2(c2[0], c2[1], c2[2]), p3(c3[0], c3[1], c3[2]);
            viewer_->addLine(p0, p1, 0.0, 1.0, 0.0, sid + "_e0");
            viewer_->addLine(p1, p2, 0.0, 1.0, 0.0, sid + "_e1");
            viewer_->addLine(p2, p3, 0.0, 1.0, 0.0, sid + "_e2");
            viewer_->addLine(p3, p0, 0.0, 1.0, 0.0, sid + "_e3");

            // 蓝色中心点
            pcl::PointXYZ center_p(sc[0], sc[1], sc[2]);
            viewer_->addSphere(center_p, 0.02, 0.0, 0.0, 1.0, sid + "_center");

            // 工作点 (品红色)
            pcl::PointXYZ work_p(slice.work_point_m[0], slice.work_point_m[1], slice.work_point_m[2]);
            viewer_->addSphere(work_p, 0.02, 1.0, 0.0, 1.0, sid + "_work");

            // 连接线 (青色)
            // viewer_->addLine(center_p, work_p, 0.0, 1.0, 1.0, sid + "_conn");

            // 切片ID文本标签（小号白色）
            pcl::PointXYZ label_pos(sc[0], sc[1], sc[2] + half_sh + 0.03);
            char          slice_label[32];
            snprintf(slice_label, sizeof(slice_label), "#%d", slice.id);
            viewer_->addText3D(slice_label, label_pos, 0.04, 1.0, 1.0, 1.0, sid + "_label");
        }
    }

    // 5. 打印检测结果到终端
    for (const auto &beam : results_) {
        double dist = std::sqrt(beam.center[0] * beam.center[0] + beam.center[1] * beam.center[1] +
                                beam.center[2] * beam.center[2]);

        // 计算修正后的翼缘宽度
        double flange_w        = calculateFlangeWidth(beam);
        double corrected_width = (flange_w > 0) ? flange_w : beam.dimensions[2] * 0.4;
        if (beam.dimensions[1] < beam.dimensions[2] * 1.2 && beam.dimensions[1] > beam.dimensions[2] * 0.1) {
            corrected_width = beam.dimensions[1];
        }

        ROS_INFO("[IBeamFitter::visualizeResults]\nIBeam #%d: \nCenter=(%.2f, %.2f, %.2f)m, \nDistance=%.2fm, \n"
                 "Dimensions=%.2fx%.2fx%.2fm (Source W=%.2fm), \n"
                 "Linearity=%.2f, \nAspectRatio=%.2f, \nPoints=%d (original: %d)",
                 beam.id, beam.center[0], beam.center[1], beam.center[2], dist, beam.dimensions[0], corrected_width,
                 beam.dimensions[2], beam.dimensions[1], beam.linearity, beam.aspect_ratio, beam.point_count,
                 beam.original_point_count);
    }
}