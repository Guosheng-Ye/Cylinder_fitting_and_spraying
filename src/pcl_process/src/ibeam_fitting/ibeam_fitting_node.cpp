#include "ibeam_fitting_node.hpp"
#include <iostream>

IBeamFitter::IBeamFitter() : cloud_(new pcl::PointCloud<pcl::PointXYZ>) {
}

IBeamFitter::~IBeamFitter() {
}

void IBeamFitter::setInputCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    cloud_ = cloud;
}

bool IBeamFitter::compute() {
    if (cloud_->empty()) {
        std::cerr << "Input cloud is empty!" << std::endl;
        return false;
    }
    
    // TODO: 实现工字钢的点云分割与拟合算法
    // 1. 预处理（滤波、降采样）
    // 2. 特征提取（例如寻找平面）
    // 3. 模型匹配
    
    std::cout << "IBeam fitting computation..." << std::endl;
    return true;
}

std::vector<double> IBeamFitter::getPose() {
    // 返回示例姿态 [x, y, z, roll, pitch, yaw]
    return {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
}

IBeamParams IBeamFitter::getParams() {
    return params_;
}
