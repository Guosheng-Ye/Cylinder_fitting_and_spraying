#/bin/bash
catkin_make
source devel/setup.bash
rosrun pcl_test cylinder_fitting_node 
