cmake_minimum_required(VERSION 3.22)
project(pcl_process)

## -----------------------------------------------------------------------------
## 编译选项
## -----------------------------------------------------------------------------
add_compile_options(-std=c++14)

## -----------------------------------------------------------------------------
## 查找依赖包
## -----------------------------------------------------------------------------
find_package(catkin REQUIRED COMPONENTS
    roscpp
    pcl_ros
    pcl_conversions
    sensor_msgs
    std_msgs
)

find_package(PCL REQUIRED)

## -----------------------------------------------------------------------------
## catkin 包配置
## -----------------------------------------------------------------------------
catkin_package(
    CATKIN_DEPENDS
    roscpp
    pcl_ros
    pcl_conversions
    sensor_msgs
    std_msgs
    DEPENDS
    PCL
    INCLUDE_DIRS
    src/cylinder_fitting
    src/pcd_viewer
    LIBRARIES
    cylinder_fitting
    pcd_viewer
)

## -----------------------------------------------------------------------------
## 头文件路径
## -----------------------------------------------------------------------------
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/src/cylinder_fitting
    ${CMAKE_CURRENT_SOURCE_DIR}/src/pcd_viewer
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## -----------------------------------------------------------------------------
## 模块库：cylinder_fitting
## -----------------------------------------------------------------------------
add_library(cylinder_fitting
    src/cylinder_fitting/cylinder_fitting_node.cpp
)

target_link_libraries(cylinder_fitting
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

## -----------------------------------------------------------------------------
## 主程序节点
## -----------------------------------------------------------------------------
add_executable(${PROJECT_NAME}
    src/main.cpp
)

target_link_libraries(${PROJECT_NAME}
    cylinder_fitting
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

## -----------------------------------------------------------------------------
## PCD 查看节点
## -----------------------------------------------------------------------------
add_executable(pcd_viewer_node
    src/pcd_viewer/pcd_test.cpp
)
target_link_libraries(pcd_viewer_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

## -----------------------------------------------------------------------------
## 安装
## -----------------------------------------------------------------------------
install(TARGETS
    ${PROJECT_NAME}
    cylinder_fitting
    pcd_viewer_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    OPTIONAL
)
