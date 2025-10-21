cmake_minimum_required(VERSION 3.5)
project(rslidar_sdk_test)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rslidar_sdk
)

catkin_package()

include_directories(
	${catkin_INCLUDE_DIRS}
)

add_executable(rslidar_sdk_test_node main.cpp)
target_link_libraries(rslidar_sdk_test_node
	${catkin_LIBRARIES}
)
