#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <string>
#include <thread>

class PCDViewer {
public:
    PCDViewer() {
        pcd_path_ = "/home/oneko/projects/lidar/ros_airy_work/src/pcd_datas/2.pcd"; // 修改路径
        viewer_.reset(new pcl::visualization::PCLVisualizer("PCD Viewer"));
        viewer_->setBackgroundColor(0, 0, 0);
        viewer_->addCoordinateSystem(1.0);
        viewer_->initCameraParameters();
    }

    int run() {
        // 尝试按 PointXYZI 加载
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_path_, *cloud_i_) == 0) {
            std::cout << "Loaded as PointXYZI, points=" << cloud_i_->size() << std::endl;
            filterByAngleI(30.0); // 保留 ±30° 数据
            colorByDistanceFromCloudI();
            show();
            return 0;
        }

        // 回退到 PointXYZ
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path_, *cloud_) == 0) {
            std::cout << "Loaded as PointXYZ, points=" << cloud_->size() << std::endl;
            filterByAngle(30.0); // 保留 ±30° 数据
            colorByDistanceFromCloud();
            show();
            return 0;
        }

        std::cerr << "Failed to load PCD: " << pcd_path_ << std::endl;
        return 1;
    }

    // 新增：直接显示外部传入的 PointXYZI 点云
    void showCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud) {
        if (!input_cloud || input_cloud->empty()) return;
        cloud_i_ = input_cloud;
        colorByDistanceFromCloudI();
        if (!viewer_->updatePointCloud<pcl::PointXYZRGB>(colored_, "cloud")) {
            viewer_->addPointCloud<pcl::PointXYZRGB>(colored_, "cloud");
            viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
        }
        viewer_->spinOnce(10);
    }

private:
    std::string                            pcd_path_;
    pcl::visualization::PCLVisualizer::Ptr viewer_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr   cloud_i_{new pcl::PointCloud<pcl::PointXYZI>};
    pcl::PointCloud<pcl::PointXYZ>::Ptr    cloud_{new pcl::PointCloud<pcl::PointXYZ>};
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_{new pcl::PointCloud<pcl::PointXYZRGB>};

    // === 新增: 角度过滤函数 ===
    void filterByAngle(double degree_limit) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
        double                              rad_limit = degree_limit * M_PI / 180.0;
        for (const auto &p : cloud_->points) {
            double angle = std::atan2(p.y, p.x);
            if (std::abs(angle) <= rad_limit) filtered->push_back(p);
        }
        std::cout << "Filtered to " << filtered->size() << " points within ±" << degree_limit << "° around +X axis."
                  << std::endl;
        cloud_.swap(filtered);
    }

    void filterByAngleI(double degree_limit) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);
        double                               rad_limit = degree_limit * M_PI / 180.0;
        for (const auto &p : cloud_i_->points) {
            double angle = std::atan2(p.y, p.x);
            if (std::abs(angle) <= rad_limit) filtered->push_back(p);
        }
        std::cout << "Filtered to " << filtered->size() << " points within ±" << degree_limit << "° around +X axis."
                  << std::endl;
        cloud_i_.swap(filtered);
    }

    void colorByDistanceFromCloudI() {
        colored_->clear();
        colored_->reserve(cloud_i_->size());
        float min_d = std::numeric_limits<float>::max();
        float max_d = 0.0f;
        for (const auto &pt : cloud_i_->points) {
            float d = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
            if (d < min_d) min_d = d;
            if (d > max_d) max_d = d;
        }
        if (min_d >= max_d) max_d = min_d + 1e-3f;

        for (const auto &pt : cloud_i_->points) {
            pcl::PointXYZRGB p;
            p.x     = pt.x;
            p.y     = pt.y;
            p.z     = pt.z;
            float d = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
            float t = (d - min_d) / (max_d - min_d);
            setColor(p, t);
            colored_->push_back(p);
        }
    }

    void colorByDistanceFromCloud() {
        colored_->clear();
        colored_->reserve(cloud_->size());
        float min_d = std::numeric_limits<float>::max();
        float max_d = 0.0f;
        for (const auto &pt : cloud_->points) {
            float d = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
            if (d < min_d) min_d = d;
            if (d > max_d) max_d = d;
        }
        if (min_d >= max_d) max_d = min_d + 1e-3f;

        for (const auto &pt : cloud_->points) {
            pcl::PointXYZRGB p;
            p.x     = pt.x;
            p.y     = pt.y;
            p.z     = pt.z;
            float d = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
            float t = (d - min_d) / (max_d - min_d);
            setColor(p, t);
            colored_->push_back(p);
        }
    }

    void setColor(pcl::PointXYZRGB &p, float t) {
        uint8_t  r   = (uint8_t)(std::min(1.0f, std::max(0.0f, 2.0f * t - 0.5f)) * 255);
        uint8_t  g   = (uint8_t)(std::min(1.0f, std::max(0.0f, 1.0f - std::abs(2.0f * t - 1.0f))) * 255);
        uint8_t  b   = (uint8_t)(std::min(1.0f, std::max(0.0f, 1.0f - 2.0f * t + 0.5f)) * 255);
        uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
        p.rgb        = *reinterpret_cast<float *>(&rgb);
    }

    void show() {
        if (!viewer_->updatePointCloud<pcl::PointXYZRGB>(colored_, "cloud")) {
            viewer_->addPointCloud<pcl::PointXYZRGB>(colored_, "cloud");
            viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
        }
        while (!viewer_->wasStopped()) {
            viewer_->spinOnce(10);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
};

int main(int argc, char **argv) {
    PCDViewer v;
    return v.run();
}
