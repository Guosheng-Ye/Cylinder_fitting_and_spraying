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
                std::cout << "       精细化失败, 使用原始结果" << std::endl;

                // 使用原始结果
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
    if (!results_.empty()) {
        std::cout << "成功识别到 " << results_.size() << " 处工字钢:" << std::endl;
        for (const auto &beam : results_) {
            std::cout << "  工字钢 #" << beam.id << ": 位置=(" << beam.center[0] << ", " << beam.center[1] << ", "
                      << beam.center[2] << ")" << std::endl;
            std::cout << "       尺寸: " << beam.dimensions[0] << " x " << beam.dimensions[1] << " x "
                      << beam.dimensions[2] << " m" << std::endl;
            std::cout << "       点数: " << beam.point_count << " (原始: " << beam.original_point_count << ")"
                      << std::endl;
        }
    } else {
        std::cout << "未识别到工字钢目标" << std::endl;
    }
    std::cout << std::string(50, '=') << std::endl;

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
    // ============================================================================
    {
        // 重新沿主轴分割切片
        double refined_u_min    = *std::min_element(refined_local_u.begin(), refined_local_u.end());
        double refined_u_max    = *std::max_element(refined_local_u.begin(), refined_local_u.end());
        double slice_len        = 0.15; // 每个切片长度 15cm
        int    num_slices       = std::max(5, static_cast<int>((refined_u_max - refined_u_min) / slice_len));
        double actual_slice_len = (refined_u_max - refined_u_min) / num_slices;

        // 收集每个切片的截面信息
        struct SliceInfo {
            double           u_start, u_end;
            double           v_span, w_span;
            double           v_center, w_center;
            int              point_count;
            std::vector<int> indices;
        };
        std::vector<SliceInfo> slices(num_slices);

        for (int i = 0; i < num_slices; ++i) {
            slices[i].u_start     = refined_u_min + i * actual_slice_len;
            slices[i].u_end       = slices[i].u_start + actual_slice_len;
            slices[i].point_count = 0;

            double v_min = std::numeric_limits<double>::max();
            double v_max = std::numeric_limits<double>::lowest();
            double w_min = std::numeric_limits<double>::max();
            double w_max = std::numeric_limits<double>::lowest();

            for (size_t j = 0; j < refined_local_u.size(); ++j) {
                if (refined_local_u[j] >= slices[i].u_start && refined_local_u[j] < slices[i].u_end) {
                    slices[i].indices.push_back(j);
                    slices[i].point_count++;
                    v_min = std::min(v_min, refined_local_v[j]);
                    v_max = std::max(v_max, refined_local_v[j]);
                    w_min = std::min(w_min, refined_local_w[j]);
                    w_max = std::max(w_max, refined_local_w[j]);
                }
            }

            if (slices[i].point_count > 0) {
                slices[i].v_span   = v_max - v_min;
                slices[i].w_span   = w_max - w_min;
                slices[i].v_center = (v_max + v_min) / 2.0;
                slices[i].w_center = (w_max + w_min) / 2.0;
            } else {
                slices[i].v_span = 0;
                slices[i].w_span = 0;
            }
        }

        // 计算中间切片的典型截面特征（排除两端各20%）
        int                 start_idx = num_slices / 5;
        int                 end_idx   = num_slices - num_slices / 5;
        std::vector<double> mid_v_spans, mid_w_spans;
        for (int i = start_idx; i < end_idx; ++i) {
            if (slices[i].point_count >= 3) {
                mid_v_spans.push_back(slices[i].v_span);
                mid_w_spans.push_back(slices[i].w_span);
            }
        }

        if (mid_v_spans.size() >= 2 && mid_w_spans.size() >= 2) {
            std::sort(mid_v_spans.begin(), mid_v_spans.end());
            std::sort(mid_w_spans.begin(), mid_w_spans.end());
            double typical_v = mid_v_spans[mid_v_spans.size() / 2];
            double typical_w = mid_w_spans[mid_w_spans.size() / 2];

            // 端部异常阈值：截面比典型截面大50%以上认为是端部结构
            double v_threshold = typical_v * 1.5;
            double w_threshold = typical_w * 1.5;

            std::cout << "       端部检测: 典型截面 v=" << typical_v << ", w=" << typical_w << std::endl;

            // 从左端向右扫描，标记异常切片
            int left_trim = 0;
            for (int i = 0; i < num_slices / 3; ++i) {
                if (slices[i].point_count < 3) continue;
                if (slices[i].v_span > v_threshold || slices[i].w_span > w_threshold) {
                    left_trim = i + 1;
                    std::cout << "       端部检测: 左端切片 #" << i << " 异常 (v=" << slices[i].v_span
                              << ", w=" << slices[i].w_span << ")" << std::endl;
                } else {
                    break; // 遇到正常切片，停止
                }
            }

            // 从右端向左扫描，标记异常切片
            int right_trim = num_slices;
            for (int i = num_slices - 1; i >= num_slices * 2 / 3; --i) {
                if (slices[i].point_count < 3) continue;
                if (slices[i].v_span > v_threshold || slices[i].w_span > w_threshold) {
                    right_trim = i;
                    std::cout << "       端部检测: 右端切片 #" << i << " 异常 (v=" << slices[i].v_span
                              << ", w=" << slices[i].w_span << ")" << std::endl;
                } else {
                    break; // 遇到正常切片，停止
                }
            }

            // 如果需要裁剪端部
            if (left_trim > 0 || right_trim < num_slices) {
                double new_u_min = slices[left_trim].u_start;
                double new_u_max = slices[right_trim - 1].u_end;

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
            }
        }
    }

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

        double width         = beam.dimensions[1];
        double height        = beam.dimensions[2];
        double v_tolerance   = std::max(0.25, width * 0.8);
        double w_tolerance   = std::max(0.30, height * 0.8);
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

        if (new_width > 0.6 || new_height > 0.8) {
            std::cout << "    扩展后截面异常：" << new_width << " x " << new_height << "m" << std::endl;
            continue;
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
        double half_w = beam.dimensions[1] / 2.0;
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
        snprintf(label, sizeof(label), "IBeam #%d\n%.2fx%.2fx%.2fm\nL=%.2f", beam.id, beam.dimensions[0],
                 beam.dimensions[1], beam.dimensions[2], beam.linearity);
        viewer_->addText3D(label, text_pos, 0.08, 1.0, 1.0, 1.0, id_prefix + "_label");
    }

    // 5. 打印检测结果到终端
    std::cout << "\n========== IBeam Detection Results ==========" << std::endl;
    std::cout << "Detected " << results_.size() << " I-beam(s)" << std::endl;
    for (const auto &beam : results_) {
        double dist = std::sqrt(beam.center[0] * beam.center[0] + beam.center[1] * beam.center[1] +
                                beam.center[2] * beam.center[2]);
        std::cout << "  IBeam #" << beam.id << ":" << std::endl;
        std::cout << "    Center: (" << beam.center[0] << ", " << beam.center[1] << ", " << beam.center[2] << ") m"
                  << std::endl;
        std::cout << "    Distance from origin: " << dist << " m" << std::endl;
        std::cout << "    Dimensions (LxWxH): " << beam.dimensions[0] << " x " << beam.dimensions[1] << " x "
                  << beam.dimensions[2] << " m" << std::endl;
        std::cout << "    Main axis: (" << beam.main_axis[0] << ", " << beam.main_axis[1] << ", " << beam.main_axis[2]
                  << ")" << std::endl;
        std::cout << "    Linearity: " << beam.linearity << std::endl;
        std::cout << "    Aspect ratio: " << beam.aspect_ratio << std::endl;
        std::cout << "    Point count: " << beam.point_count << " (original: " << beam.original_point_count << ")"
                  << std::endl;
    }
    std::cout << "==============================================" << std::endl;
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
