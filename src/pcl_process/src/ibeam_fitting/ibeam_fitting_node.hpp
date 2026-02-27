#ifndef IBEAM_FITTING_NODE_HPP
#define IBEAM_FITTING_NODE_HPP

#include <Eigen/Dense>
#include <atomic>
#include <map>
#include <mutex>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <ros/ros.h>
#include <string>
#include <thread>
#include <vector>

struct IBeamInputs {
    // 工艺参数
    double distance_tcp2surface = 1100; // 轨迹点与表面的距离
    double step_height;                 // 工作区域单次喷幅（mm) 透传
    double step_length;                 // 工作区域单次长度（mm) 透传
    double lap;                         // 两次喷幅搭接比例      透传
    double arm_reach   = 1500;          // arm_reach 机械臂最大工作半径（mm）
    int    mode_length = 2;             // mode_length 工作区域长度模式（1-手动，2-自动）
    double length      = -1;            // length 工作区域长度（mm，mode_length=1时有效）
    int    mode_theta  = 2;             // mode_theta 喷涂角度模式（1-手动，2-自动）
    double theta       = -1;            // theta 手动模式下喷涂翼板时的角度（透传）

    // 长度计算所需参数
    double d_base        = 1100; // 底座中心到腹板表面的水平距离 (mm)1100
    double d_work        = 650;  // 喷枪物理长度 + 喷涂挂距 (mm)650
    double base_z_offset = 460;  // 机械臂底座中心与腹板中心高度线的高度差 (mm)460
};

// 作业对象参数结构体
struct IBeamOutputs {
    Eigen::Vector3d surface_point;          // surface_point 工作区域中心点坐标(机械臂坐标系下）
    Eigen::Vector4d plane_coeffs;           // plane_coeffs 平面方程系数
    Eigen::Vector3d plane_rx;               // plane_rx 区域长度方向的向量单位
    double          beam_height;            // height 工作区域的总高度（mm）
    double          beam_length_of_working; // length 工作区域的总长度（mm）
    double          wing_plate_width;       // wing_plate_width 翼板宽度的一半（mm）
    double          distance_tcp2surface;   // distance 轨迹点与表面的距离，直接传T30数值
};

// 工字钢切片数据结构体
struct IBeamSliceData {
    int             id;             // 切片id
    int             layer_index{0}; // 高度层索引 (沿主轴方向)
    int             row_index{0};   // 行索引 (高度方向，对应beam_height)
    Eigen::Vector3d center_mm;      // 切片中心坐标 (mm)
    Eigen::Vector3d center_m;       // 切片中心坐标 (m)
    Eigen::Vector3d work_point_mm; // 作业点坐标 (mm)，在切片中心基础上沿法向偏移distance_tcp2surface
    Eigen::Vector3d work_point_m;  // 作业点坐标 (m)，在切片中心基础上沿法向偏移distance_tcp2surface
    double          slice_height_mm; // 切片高度 (mm)，来源于 beam_height
    double          slice_width_mm;  // 切片宽度 (mm)，来源于 beam_length_of_working
    Eigen::Vector3d normal;          // 切片法向量（指向机器人方向）
    Eigen::Vector3d main_axis;       // 工字钢主轴方向
};

// 定义工字钢参数结构体
struct IBeamParams {
    double height;           // 高度(Z方向)
    double width;            // 宽度(翼缘方向)
    double length;           // 长度(主轴方向)
    double web_thickness;    // 腹板厚度
    double flange_thickness; // 翼缘厚度
};

// 工字钢检测结果结构体
struct IBeamResult {
    int                                 id;                   // 检测ID
    Eigen::Vector3d                     center;               // 中心位置
    Eigen::Vector3d                     dimensions;           // 尺寸 (length, width, height)
    Eigen::Vector3d                     main_axis;            // 主轴方向
    Eigen::Quaterniond                  orientation;          // 姿态四元数
    double                              linearity;            // 线性度
    double                              aspect_ratio;         // 长宽比
    int                                 point_count;          // 点数
    int                                 original_point_count; // 原始聚类点数
    pcl::PointCloud<pcl::PointXYZ>::Ptr points;               // 工字钢点云
};

// 聚类信息结构体(用于合并判断)
struct ClusterInfo {
    Eigen::Vector3d center;
    Eigen::Vector3d main_axis;
    double          axis_min;
    double          axis_max;
    double          height;
    int             point_count;
    bool            valid;
};

// 检测参数结构体
struct IBeamDetectionParams {
    // 空间过滤参数
    double max_distance = 6.0; // 最大检测距离(米)
    double min_distance = 2.0; // 最小检测距离(米)
    double min_height   = 1.5; // 最小高度(米)

    // 预处理参数
    double voxel_size    = 0.02; // 体素降采样大小
    int    sor_neighbors = 20;   // 统计滤波邻居数
    double sor_std_ratio = 1.5;  // 统计滤波标准差比

    // 墙面移除参数
    double wall_distance_threshold = 0.08; // RANSAC距离阈值
    int    min_wall_points         = 500;  // 最小墙面点数
    int    max_wall_iterations     = 3;    // 最大墙面移除迭代次数

    // 聚类参数
    double cluster_tolerance = 0.18; // DBSCAN聚类容差
    int    min_cluster_size  = 10;   // 最小聚类点数

    // 聚类合并参数(第一轮 - 严格)
    double merge_max_gap_strict             = 0.15; // 降低：最大间隙0.15m
    double merge_max_height_diff_strict     = 0.10;
    double merge_min_axis_similarity_strict = 0.98;

    // 聚类合并参数(第二轮 - 宽松)
    double merge_max_gap_loose             = 0.5; // 降低：最大间隙0.5m（原1.0m太大）
    double merge_max_height_diff_loose     = 0.15;
    double merge_min_axis_similarity_loose = 0.96;

    // 工字钢判断阈值
    double min_linearity    = 0.9;  // 最小线性度
    double min_length       = 1.5;  // 最小长度(米)
    double min_aspect_ratio = 2.5;  // 最小长宽比
    double max_axis_z       = 0.1;  // 主轴Z分量最大值(水平性)
    double min_width        = 0.1;  // 最小宽度
    double max_width        = 1.2;  // 最大宽度
    double min_thickness    = 0.03; // 最小厚度
    double max_thickness    = 0.8;  // 最大厚度

    // 精细化参数
    double refine_max_v_span = 0.50; // 最大翼缘宽度
    double refine_max_w_span = 0.60; // 最大截面高度
    double refine_min_v_span = 0.30; // 最小翼缘宽度
    double refine_min_w_span = 0.05; // 最小截面高度

    // 扩展参数
    double extend_min_target_length = 3.5; // 扩展目标最小长度
};

// 高度层定义
struct HeightLayer {
    double      z_min;
    double      z_max;
    std::string name;
};

class IBeamFitter {
public:
    IBeamFitter();
    ~IBeamFitter();

    // 设置输入点云
    void setInputCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

    // 设置检测参数
    void setParams(const IBeamDetectionParams &params);

    // 设置/获取工艺参数
    void               setIBeamParams(const IBeamInputs &inputs) { ibeam_inputs_ = inputs; }
    const IBeamInputs &getIBeamParams() const { return ibeam_inputs_; }

    // 获取当前参数
    IBeamDetectionParams getDetectionParams() const;

    // 执行检测(主入口函数 - 对应 detect_ibeams_final)
    bool detect();

    // 获取检测结果
    std::vector<IBeamResult> getResults() const;

    // 获取第一个工字钢的位姿(兼容旧接口)
    std::vector<double> getPose();

    // 获取工字钢参数(兼容旧接口)
    IBeamParams getParams();

    // 获取处理后的点云(用于可视化)
    pcl::PointCloud<pcl::PointXYZ>::Ptr getFilteredCloud() const;
    pcl::PointCloud<pcl::PointXYZ>::Ptr getROICloud() const;

    // ========== 可视化函数 ==========
    // 可视化检测结果(使用PCL Visualizer)
    void visualizeResults();

    // ========== 工作区域计算函数 ==========
    /**
     * @brief 计算机械臂在工字钢上的可喷涂长度
     * @param R_max 机械臂最大半径 (mm)
     * @param D_base 底座中心到腹板表面的水平距离 (mm)
     * @param D_work 喷枪物理长度 + 喷涂挂距 (mm)
     * @param height 工字钢腹板高度 (mm)，即 beam_height
     * @param base_z_offset 机械臂底座中心与腹板中心高度线的高度差 (mm)
     * @param[out] error_msg 错误信息(如有)
     * @return 可喷涂的总长度 (mm)，如果计算失败返回 0
     */
    double calculateBeamSprayingLimit(double R_max, double D_base, double D_work, double height, double base_z_offset,
                                      std::string &error_msg);

    /**
     * @brief 根据工字钢识别结果和输入参数更新输出结果
     * @param inputs 输入参数
     * @param beam_result 工字钢检测结果
     * @param[out] outputs 输出结果
     * @return 是否成功更新
     */
    bool updateOutputs(const IBeamInputs &inputs, const IBeamResult &beam_result, IBeamOutputs &outputs);

    /**
     * @brief 通过分析截面形状计算翼缘宽度
     * @param beam_result 工字钢检测结果
     * @return 翼缘宽度 (m)，如果计算失败返回 -1
     */
    double calculateFlangeWidth(const IBeamResult &beam_result);

    // ========== 切片函数 ==========
    /**
     * @brief 将工字钢工作区域切片为多块区域，便于机械臂工作
     * @param beam_result 工字钢检测结果
     * @param outputs 工字钢输出参数（包含beam_height和beam_length_of_working）
     * @param slice_height_mm 切片高度 (mm)
     * @param slice_width_mm 切片宽度 (mm)
     * @note 两参数都非0时按传入值切片；都为0时使用beam_height作为切片高、beam_length_of_working作为切片宽
     */
    void sliceIBeam(const IBeamResult &beam_result, const IBeamOutputs &outputs, double slice_height_mm = 0,
                    double slice_width_mm = 0);

    /**
     * @brief 获取切片结果
     * @return 切片数据列表
     */
    std::vector<IBeamSliceData> getSliceResults() const { return slices_data_; }

    /**
     * @brief 清空切片数据
     */
    void clearSlices() { slices_data_.clear(); }

    // ========== 兼容旧接口 ==========
    bool compute() { return detect(); }

private:
    // ========== 预处理函数 ==========
    // 体素降采样
    void voxelDownsample(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

    // 统计滤波去噪
    void statisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

    // ========== 墙面移除 ==========
    // 使用RANSAC迭代移除大型垂直平面(墙面)
    pcl::PointCloud<pcl::PointXYZ>::Ptr removeWallPlanes(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

    // ========== 空间过滤 ==========
    // 距离和高度过滤
    pcl::PointCloud<pcl::PointXYZ>::Ptr filterByDistanceAndHeight(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

    // ========== 分层聚类 ==========
    // 按高度分层聚类
    std::vector<int> layeredClustering(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

    // 在单层内进行DBSCAN聚类
    std::vector<int> clusterLayer(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                  const std::vector<int>                    &layer_indices);

    // ========== 聚类合并 ==========
    // 合并可能属于同一工字钢的聚类
    std::vector<int> mergeIBeamClusters(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                        const std::vector<int> &labels, double max_gap, double max_height_diff,
                                        double min_axis_similarity);

    // 计算聚类信息
    ClusterInfo computeClusterInfo(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const std::vector<int> &labels,
                                   int cluster_id);

    // ========== PCA形状分析 ==========
    // 对聚类进行PCA分析
    bool analyzeClusterShape(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cluster_cloud, Eigen::Vector3d &center,
                             Eigen::Vector3d &dimensions, Eigen::Vector3d &main_axis, Eigen::Matrix3d &eigenvectors,
                             double &linearity, double &planarity);

    // ========== 工字钢判断 ==========
    // 判断聚类是否为工字钢
    bool isIBeam(double linearity, double length, double width, double thickness, double aspect_ratio,
                 double main_axis_z);

    // ========== 精细化检测 ==========
    // 对候选工字钢区域进行精细化识别
    bool refineIBeamDetection(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cluster_cloud,
                              const Eigen::Vector3d &main_axis, const Eigen::Vector3d &center,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr &refined_cloud, Eigen::Vector3d &refined_center,
                              Eigen::Vector3d &refined_dims);

    // ========== 扩展检测 ==========
    // 扩展长度不足的工字钢
    void extendShortIBeams(std::vector<IBeamResult> &ibeams, const pcl::PointCloud<pcl::PointXYZ>::Ptr &roi_cloud);

    // ========== 工具函数 ==========
    // 计算点到XY平面原点的距离
    double computeXYDistance(const pcl::PointXYZ &point);

    // 并查集查找根节点
    int findRoot(std::map<int, int> &parent, int x);

    // 重新编号标签(使标签连续)
    std::vector<int> relabelClusters(const std::vector<int> &labels);

    // ========== 可视化线程 ==========
    void viewerThread();

public:
    IBeamInputs ibeam_inputs_{}; // 工艺参数输入

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;          // 输入点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_; // 预处理后点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr roi_cloud_;      // ROI区域点云

    IBeamDetectionParams     detection_params_; // 检测参数
    IBeamParams              params_;           // 工字钢参数(兼容旧接口)
    std::vector<IBeamResult> results_;          // 检测结果

    // 切片数据
    std::vector<IBeamSliceData> slices_data_; // 所有切片数据

    // 高度层定义
    std::vector<HeightLayer> height_layers_;

    // PCL可视化器
    pcl::visualization::PCLVisualizer::Ptr viewer_;
    std::thread                            viewer_thread_;
    std::mutex                             viewer_mutex_;
    std::atomic<bool>                      running_{true};
};

#endif // IBEAM_FITTING_NODE_HPP
