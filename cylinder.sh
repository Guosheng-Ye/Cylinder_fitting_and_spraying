#/bin/bash
catkin_make
source devel/setup.bash
#rosrun pcl_process cylinder_fitting_node_ros
roslaunch pcl_process pcl_proces.launch