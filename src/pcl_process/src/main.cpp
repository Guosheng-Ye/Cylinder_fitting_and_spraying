#include "cylinder_fitting/cylinder_fitting_node.hpp"
#include <iostream>


int main(int argc, char **argv) {
    std::string pcd_path = "/home/oneko/projects/lidar/ros_airy_work/src/pcd_datas/2.pcd";
    if (argc > 1) pcd_path = argv[1];

    std::cout << "Testing cylinder fitting on: " << pcd_path << std::endl;

    CylinderFitter app(pcd_path);
    return app.run();
}
