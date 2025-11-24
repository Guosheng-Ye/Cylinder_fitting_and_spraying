cmake_minimum_required(VERSION 3.0.2)
project(main)

## Compile options
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	pcl_ros
	pcl_conversions
	sensor_msgs
	std_msgs
	rslidar_sdk
)

find_package(PCL REQUIRED)

catkin_package(
	CATKIN_DEPENDS roscpp pcl_ros pcl_conversions sensor_msgs std_msgs rslidar_sdk
)

include_directories(
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	include
	${CMAKE_CURRENT_LIST_DIR}/src/
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# add_executable(rslidar_sdk_test_node main.cpp)
# target_link_libraries(rslidar_sdk_test_node
# 	${catkin_LIBRARIES}
# 	${PCL_LIBRARIES}
# )
# add_dependencies(rslidar_sdk_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
