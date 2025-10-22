#/bin/bash
catkin_make
source devel/setup.bash
rosrun pcl_process pcl_process
