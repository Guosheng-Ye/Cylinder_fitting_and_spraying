cmake_minimum_required(VERSION 3.22)
project(pcl_test)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    pcl_conversions
    pcl_ros
)
find_package(PCL REQUIRED)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcl_test main.cpp)
target_link_libraries(pcl_test ${PCL_LIBRARIES})

add_executable(pcl_test_node pcl_test_node.cpp)
target_link_libraries(pcl_test_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

add_executable(cylinder_fitting_node cylinder_fitting_node.cpp)
target_link_libraries(cylinder_fitting_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

# Standalone PCD viewer (non-ROS, depends only on PCL and C++17 filesystem)
add_executable(pcd_test_viewer pcd_test.cpp)
target_link_libraries(pcd_test_viewer ${PCL_LIBRARIES})


set_target_properties(pcd_test_viewer PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)
install(TARGETS
    pcl_test_node
    cylinder_fitting_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
