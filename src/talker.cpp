#include "ros/ros.h"
#include "std_msgs/String.h"

// Simple node to test whether the project is building properly and connecting to ROS

int main(int argc, char **argv) {
    ros::init(argc, argv, "talker");
    ros::NodeHandle nh;
    ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);

    ros::Rate loop_rate(10);

    while(ros::ok()) {
        std_msgs::String msg;
        msg.data = "hello";

        chatter_pub.publish(msg);

        ros::spinOnce();
    }

    return 0;
}