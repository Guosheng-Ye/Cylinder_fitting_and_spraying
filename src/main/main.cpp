#include <ros/ros.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "rslidar_sdk_test_node");
	ros::NodeHandle nh;

	ROS_INFO("rslidar_sdk_test_node started.");

	// 可在此处添加 rslidar_sdk 节点相关测试代码
	ros::spin();
	return 0;
}
