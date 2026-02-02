#ifndef IBEAM_FITTING_NODE_HPP
#define IBEAM_FITTING_NODE_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <vector>

// 定义工字钢参数结构体
struct IBeamParams {
    double height;      // 高度
    double width;       // 宽度
    double web_thickness;   // 腹板厚度
    double flange_thickness;// 翼缘厚度
};

class IBeamFitter {
public:
    IBeamFitter();
    ~IBeamFitter();

    // 设置输入点云
    void setInputCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    // 执行拟合
    bool compute();

    // 获取拟合结果（例如中心位置和姿态）
    // 这里简单定义，具体根据需求调整
    std::vector<double> getPose();

    // 获取工字钢参数
    IBeamParams getParams();

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    IBeamParams params_;
};

#endif // IBEAM_FITTING_NODE_HPP
