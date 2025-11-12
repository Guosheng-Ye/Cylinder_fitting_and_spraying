cmake_minimum_required(VERSION 3.22)
project(pcl_process)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    pcl_ros
    pcl_conversions
    sensor_msgs
    std_msgs
    message_generation
    geometry_msgs
)
find_package(PCL REQUIRED)

## -----------------------------------------------------------------------------
## Catkin 包配置（导出头文件与库）
## -----------------------------------------------------------------------------
## -----------------------------------------------------------------------------
## 消息文件设置
## -----------------------------------------------------------------------------
add_message_files(
    FILES
    TargetPose.msg
    TargetInfo.msg
    TargetPoseList.msg
    # CylinderSlice.msg
    # CylinderSliceArray.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS
    roscpp
    pcl_ros
    pcl_conversions
    sensor_msgs
    std_msgs
    message_runtime
    geometry_msgs
    DEPENDS
    PCL
    INCLUDE_DIRS
    src/cylinder_fitting
    src/rslidar_process
    src/pcd_viewer
    src/tests
    src/tf
    LIBRARIES
    cylinder_fitting_node
    rslidar_process
    tests
)


include_directories(
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/src/cylinder_fitting
    ${CMAKE_CURRENT_SOURCE_DIR}/src/rslidar_process
    ${CMAKE_CURRENT_SOURCE_DIR}/src/pcd_viewer
    ${CMAKE_CURRENT_SOURCE_DIR}/src/tf
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


## 圆柱拟合
add_library(cylinder_fitting_node
    src/cylinder_fitting/cylinder_fitting_node.cpp
)
target_link_libraries(cylinder_fitting_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)


## 雷达数据过滤
add_library(rslidar_process
    src/rslidar_process/rslidar_process.cpp
)
target_include_directories(rslidar_process PRIVATE src/rslidar_process)
target_link_libraries(rslidar_process
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)


# main executable
add_executable(${PROJECT_NAME}
    src/main.cpp
)
target_link_libraries(${PROJECT_NAME}
    cylinder_fitting_node
    rslidar_process
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

## 雷达数据过滤节点
add_executable(rslidar_process_node
    src/rslidar_process/rslidar_process.cpp
)
target_link_libraries(rslidar_process_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

## -----------------------------------------------------------------------------
## 圆柱拟合 ROS 节点（
## -----------------------------------------------------------------------------
add_executable(cylinder_fitting_node_ros
    src/cylinder_fitting/cylinder_fitting_ros.cpp
)
target_link_libraries(cylinder_fitting_node_ros
    cylinder_fitting_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

##
## Tests 测试节点
##
add_executable(test_achieve_work_pose_pub
    src/tests/achieve_work_pose_pub.cpp
)
target_link_libraries(
    test_achieve_work_pose_pub
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)


## -----------------------------------------------------------------------------
## PCD 查看节点
## -----------------------------------------------------------------------------
# add_executable(pcd_viewer_node
#     src/pcd_viewer/pcd_viewer.cpp
# )
# target_link_libraries(pcd_viewer_node
#     ${catkin_LIBRARIES}
#     ${PCL_LIBRARIES}
# )


install(TARGETS
    ${PROJECT_NAME}
    rslidar_process_node
    cylinder_fitting_node_ros
    test_achieve_work_pose_pub
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    OPTIONAL
)
