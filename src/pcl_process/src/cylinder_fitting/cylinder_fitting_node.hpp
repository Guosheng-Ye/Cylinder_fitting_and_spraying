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
#include <random>
#include <ros/ros.h>
#include <string>
#include <thread>
#include <utility>
#include <vector>

// 定义M_PI常量
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 定义输入参数结构体
struct PlanningInputs {
    // 圆柱参数 (假设已拟合得到)
    std::vector<double> P_cyl;  // 圆柱轴线上一点 [x, y, z]
    std::vector<double> V_axis; // 圆柱轴线方向 [i, j, k]
    double              r_cyl;  // 圆柱半径

    // 机器人和工作区域参数
    double robot_R_arm; // 机械臂最大臂长 R_arm
    double D_offset;    // 喷涂距离 D_offset
    double H_region;    // 总工作高度 H

    // 喷涂工艺参数
    double d_spray;   // 喷幅 d
    double r_initial; // 初始搭接比例 r
    double h_slice;   // 切片高度 h (未在角度计算中使用)
};

// 定义输出结果结构体
struct PlanningOutputs {
    double      max_coverage_angle;  // 机械臂最大覆盖角度 (theta_max)
    double      optimal_slice_angle; // 最佳切片角度宽度 w -> 90 deg
    double      final_overlap_ratio; // 调整后的搭接比例 r_final
    int         k_spray_passes;      // 每个位置的喷涂次数 k
    double      w0_adjusted_mm;      // 调整后的有效单次喷幅 (mm)
    double      w0_adjusted_deg;     // 调整后的有效单次喷幅 (deg)
    int         n_positions;         // 机械臂基座移动次数 N
    std::string status_message;      // 状态信息
};

class CylinderFitter {
public:
    CylinderFitter(const std::string &pcd_path);
    ~CylinderFitter();

    /// @brief 输出参数结构体（用于回传给上位机）
    struct CylinderFitterParamsOutpus {
        // 切片参数
        double slice_distance_{750.00}; // 喷头距离圆柱表面偏移 (毫米)
        double height_{600.00};         // 工作区域单次喷幅高(mm)
        double step_width_{300.00};     // 工作区域单次喷幅宽(mm)
        double step_height_{200.00};    // 工作区域单次喷幅高(mm)

        // 工艺参数
        double r_initial_{0.50};     // 初始搭接比例0.5
        double robot_R_arm_{1500.0}; // 机械臂最大臂长1500mm
        int    work_mode_{2};        // 工作模式 1 手动,2 自动 动态计算回传
        int    k_{0};                // 手动模式下的倍数k
        double w0_{0.0};             // 有效喷幅宽度 (deg)

        // 切片统计信息
        int    slice_count_{0};            // 切片总数
        double single_slice_area_m2_{0.0}; // 单个切片面积 (m^2)
        double total_slice_area_m2_{0.0};  // 切片总面积 (m^2) = 单个切片面积 * 切片数
    };

public:
    /// @brief 圆柱模型结构体
    struct CylinderFitterParams {
        // 圆柱模型参数
        int    min_inliers{300};         // 最小内点数
        int    max_inliers{600};         // 最大内点数
        double min_inlier_ratio{0.02};   // 最小内点比例
        double max_inlier_ratio{0.50};   // 最大内点比例
        double min_height_m{1.00};       // 最小高度（米）
        double min_radius_m{0.30};       // 最小半径（米）
        double max_radius_m{0.40};       // 最大半径(米)
        double min_radial_rel_std{0.00}; // 最小径向相对标准差
        double max_radial_rel_std{0.15}; // 最大径向相对标准差

        // 切片参数
        double slice_distance_{750.00}; // 喷头距离圆柱表面偏移 (毫米)
        double height_{600.00};         // 工作区域总喷幅高(mm)
        double step_width_{300.00};     // 工作区域单次喷幅宽(mm)
        double step_height_{200.00};    // 工作区域单次喷幅高(mm)

        // 工艺参数
        double r_initial_{0.50};               // 初始搭接比例0.5
        double robot_R_arm_{1500.0};           // 机械臂最大臂长1500mm
        double virtual_base_to_surface_{600.0}; // 虚拟原点到圆柱表面的距离(mm)
        int    work_mode_{2};                  // 工作模式 1 手动,2 自动 动态计算回传
        int    k_{0};                          // 手动模式下的倍数k
        float  w0_{0};                         // 有效喷副宽度

        // RANSAC 轴向约束参数
        // eps_angle_deg_ 控制 RANSAC 拟合时允许圆柱轴向偏离参考轴的最大角度
        // 竖直立柱检测：设为 10~15；倾斜圆柱检测：设为 45~90
        double eps_angle_deg_{10.0};            // RANSAC轴向容差(度)，偏离参考轴超过此值的圆柱将被拒绝

        // 轴向校正参数（切片阶段使用）
        // 当 use_ideal_axis_ = true 且拟合轴向与竖直方向偏差 < 阈值时：使用理想轴向 (0, 0, 1)
        // 当偏差超过阈值时：保留拟合轴向并输出警告
        // 当 use_ideal_axis_ = false 时：始终使用拟合轴向
        bool use_ideal_axis_{true};             // 是否使用理想竖直轴向 (0, 0, 1)
        double axis_angle_threshold_deg_{15.0}; // 拟合轴向与竖直方向夹角阈值 (度)，小于此值时使用理想轴向
    };

    void                        setFittingParams(const CylinderFitterParams &p) { fitting_params_ = p; }
    const CylinderFitterParams &fittingParams() const { return fitting_params_; }

    // 获取输出参数结构体（用于发布）
    const CylinderFitterParamsOutpus &getOutputParams() const { return cylinder_fitter_outputs; }

    struct SliceData {
        int             id;               // 切片id
        int             layer_index{0};   // 高度层索引 i
        int             angular_index{0}; // 角度层索引 j
        Eigen::Vector3f center_mm;        // 切片中心坐标 (mm)
        Eigen::Vector3f circle_center_mm; // 该高度平面圆心(轴线上点) (mm)
        Eigen::Vector3f work_mm;          // 切片工作点 (mm)
        double          angle_deg{0.0};   // 切片对应扇形角度（度）
    };

    /// @brief 可视化绘制控制项
    struct SliceVizOptions {
        bool surface_{false};      // 弧形面网格
        bool border_{false};       // 切片边框线
        bool height_rings_{false}; // 高度层环线
        bool print_centers_{true}; // 切片中心点
    };

    void                          setSliceVizOptions(const SliceVizOptions &opt) { slice_opts_ = opt; }
    SliceVizOptions              &sliceOptions() { return slice_opts_; }
    const SliceVizOptions        &sliceOptions() const { return slice_opts_; }
    const std::vector<SliceData> &slicesData() const { return slices_data_; } // 获取切片数据

public:
    // int run_with_pcd();
    int run_with_rslidar(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input);

    /// @brief 获取切片结果
    /// @return std::vector<Eigen::Vector3f> slice_centers_mm_;
    std::vector<SliceData> getSliceResults() const { return slices_data_; };

    // 获取拟合结果
    bool                                hasModel() const { return cylinder_model_.valid; }
    Eigen::Vector3f                     axis() const { return cylinder_model_.axis; }  // 原始拟合轴向
    /// @brief 获取实际使用的轴向（考虑理想轴向校正）
    Eigen::Vector3f                     axisUsed() const {
        Eigen::Vector3f axis_fitted = cylinder_model_.axis.normalized();
        Eigen::Vector3f ideal_axis(0.0f, 0.0f, 1.0f);
        double cos_angle = std::abs(axis_fitted.dot(ideal_axis));
        double angle_diff_deg = std::acos(std::min(1.0, (double)cos_angle)) * 180.0 / M_PI;
        if (fitting_params_.use_ideal_axis_ && angle_diff_deg < fitting_params_.axis_angle_threshold_deg_) {
            return ideal_axis;
        }
        return axis_fitted;
    }
    Eigen::Vector3f                     bottomCenter() const { return cylinder_model_.bottom_center; }
    Eigen::Vector3f                     topCenter() const { return cylinder_model_.top_center; }
    Eigen::Vector3f                     geoCenter() const { return cylinder_model_.geo_center; }
    double                              radius() const { return cylinder_model_.radius; }
    double                              height() const { return cylinder_model_.height; }
    pcl::PointCloud<pcl::PointXYZ>::Ptr inlierCloud() const { return cylinder_model_.inliers; }

    /// @brief 核心执行函数：整合几何计算和喷涂优化。
    /// @param inputs 包含所有输入参数的结构体。
    /// @return PlanningOutputs 包含所有计算结果和状态信息的结构体。
    PlanningOutputs execute_slice_planning(const PlanningInputs &inputs);

private:
    /// @brief
    /// @param target_post 机械臂基座
    /// @param spray_center 喷涂作业点
    /// @param circle_center 圆柱中心坐标
    /// @return
    std::vector<double> cylinder_center_convert2robot_base(const std::vector<double> target_pose,
                                                           const std::vector<double> spray_center,
                                                           const std::vector<double> circle_center);

    /// @brief 计算虚拟机械臂原点方案下的 P_cyl
    /// @param geo_center_m 圆柱几何中心（雷达坐标系，单位：m）
    /// @param cylinder_radius_mm 圆柱半径（mm）
    /// @param work_distance_mm 工作距离（mm）
    /// @param z_center_mm 输出的P_cyl的z坐标（工作高度中心，mm）
    /// @return P_cyl_eigen 圆柱轴线上一点相对于虚拟原点的位置（mm）
    Eigen::Vector3d calculateVirtualBaseP_cyl(const Eigen::Vector3f &geo_center_m, double cylinder_radius_mm,
                                              double work_distance_mm, double z_center_mm);

    /// @brief 辅助向量操作函数
    double              norm(const std::vector<double> &v) const;
    std::vector<double> cross_product(const std::vector<double> &a, const std::vector<double> &b) const;

    /// @brief 几何约束计算 (移植自 cal.py
    /// @param robot_max_reach
    /// @param cylinder_center
    /// @param cylinder_axis
    /// @param cylinder_radius
    /// @param offset_dist
    /// @param regional_height
    /// @return
    std::pair<double, std::string> calculate_max_slice_angle(double                     robot_max_reach,
                                                             const std::vector<double> &cylinder_center,
                                                             const std::vector<double> &cylinder_axis,
                                                             double cylinder_radius, double offset_dist,
                                                             double regional_height);

    /// @brief 喷涂角度优化 (自动调节搭接比例)
    /// @param max_coverage_angle
    /// @param spray_width
    /// @param initial_overlap_ratio
    /// @param cylinder_radius
    /// @param offset_dist
    /// @param final_overlap_ratio
    /// @return
    double calculate_optimal_slice_angle(double max_coverage_angle, double spray_width, double initial_overlap_ratio,
                                         double cylinder_radius, double offset_dist, double &final_overlap_ratio);

    /// @brief 手动模式切片优化 (给定k和r，计算整除180度的切片宽度)
    /// @param spray_width 喷幅宽度 (mm)
    /// @param initial_k 初始喷涂移动次数
    /// @param initial_r 初始搭接比例
    /// @param cylinder_radius_mm 圆柱半径 (mm)
    /// @param final_k 输出：调整后的喷涂移动次数
    /// @param final_r 输出：调整后的搭接比例
    /// @param final_w0_mm 输出：调整后的有效单次喷幅 (mm)
    /// @param final_slice_width_mm 输出：最终切片宽度 (mm)
    /// @return 切片数量 N
    int calculate_manual_slice_params(double spray_width, int initial_k, double initial_r, double cylinder_radius_mm,
                                      int &final_k, double &final_r, double &final_w0_mm, double &final_slice_width_mm);

    /// @brief 法线修复
    /// @param cloud 点云
    /// @param normals 法线
    void estimateNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals);

    /// @brief 去除墙面点
    /// @param cloud 点云
    /// @param cloud_filtered 输出点云
    /// @param angle_threshold 角度阈值（度）
    void removeWalls(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered,
                     double angle_threshold, double distance_threshold, int max_iterations);
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

    bool extractCorrespondingNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_original,
                                     pcl::PointCloud<pcl::Normal>::Ptr   normals_original,
                                     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered,
                                     pcl::PointCloud<pcl::Normal>::Ptr   normals_filtered);

    double calculateCircularity(pcl::PointCloud<pcl::PointXYZ>::Ptr inliers, const Eigen::Vector3f &center,
                                const Eigen::Vector3f &axis, double radius);

    /// @brief 可视化
    /// @param cloud 点云
    /// @param cylinder_points 圆柱内点
    /// @param center 中心
    /// @param axis 轴向
    /// @param radius 半径
    void visualizeResults();

    /// @brief 可视化虚拟机械臂原点、圆柱体、球形工作空间和覆盖角度范围
    /// @param virtual_base_pos_mm 虚拟机械臂原点位置（雷达坐标系，mm）
    /// @param robot_max_reach_mm 机械臂最大臂长（mm）
    /// @param coverage_angle_deg 覆盖角度（度）
    /// @param work_height_center_mm 工作高度中心（mm）
    /// @param work_height_range_mm 工作高度范围（mm）
    void visualizeCoverageRange(const Eigen::Vector3d &virtual_base_pos_mm, double robot_max_reach_mm,
                                double coverage_angle_deg, double work_height_center_mm, double work_height_range_mm);

    void printResults();
    void saveVisualizationData();
    void viewerThread();

    CylinderFitterParams fitting_params_{};

    // 优化算法的临时结果存储（用于在 calculate_optimal_slice_angle 和 execute_slice_planning 之间传递）
    int    last_k_spray_passes_{0};    // 上次计算的每位置喷涂次数
    double last_w0_adjusted_mm_{0.0};  // 上次计算的有效喷幅 (mm)
    double last_w0_adjusted_deg_{0.0}; // 上次计算的有效喷幅 (deg)
    int    last_n_positions_{0};       // 上次计算的基座移动次数

private:
    std::string                            pcd_path_;                                    // PCD文件路径
    pcl::visualization::PCLVisualizer::Ptr viewer_;                                      // 可视化器
    pcl::PointCloud<pcl::PointXYZ>::Ptr    cloud_{new pcl::PointCloud<pcl::PointXYZ>()}; // 点云数据

    // 非阻塞绘制
    std::thread                  viewer_thread_;    // 视图线程
    std::atomic<bool>            running_{true};    // 运行标志
    std::mutex                   viewer_mutex_;     // 保护viewer线程
    std::vector<std::string>     slice_shape_ids_;  // 切片形状ID
    std::vector<Eigen::Vector3f> slice_centers_mm_; // 存储切片中心
    std::vector<std::string>     slice_text_ids_;   // 文本ID
    std::vector<std::string>     slice_line_ids_;   // 连线ID
    std::vector<SliceData>       slices_data_;      // 所有切片数据

    SliceVizOptions slice_opts_; // 集中管理切片显示/打印参数
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

    CylinderFitterParamsOutpus cylinder_fitter_outputs;
};

#endif