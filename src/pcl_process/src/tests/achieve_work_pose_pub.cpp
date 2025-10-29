#include "ros/ros.h"
#include "std_msgs/Bool.h"

class AchieveWorkPosePub {
public:
    AchieveWorkPosePub(ros::NodeHandle &nh) { pub_ = nh.advertise<std_msgs::Bool>("/archieve_work_pose", 1); }

    void publish(bool value) {
        std_msgs::Bool msg;
        msg.data = value;
        pub_.publish(msg);
    }

private:
    ros::Publisher pub_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "achieve_work_pose_pub");
    ros::NodeHandle nh;

    AchieveWorkPosePub achieve_pub(nh);
    ros::Rate          loop_rate(0.2); // 5 Hz

    bool toggle = false;

    while (ros::ok()) {
        toggle = !toggle;
        achieve_pub.publish(toggle);
        ROS_INFO("Published /archieve_work_pose: %s", toggle ? "true" : "false");
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}