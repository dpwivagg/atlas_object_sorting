#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <std_msgs/String.h>
#include <tough_footstep/robot_walker.h>
#include "geometry_msgs/Pose2D.h"

class RobotWalkToGoal {

public:
  RobotWalkToGoal(ros::NodeHandle nh):n_(nh) {
    sub_ = n_.subscribe("atlas_object_sorting_perception", 1000, &RobotWalkToGoal::callback, this);
  }

  void callback(const geometry_msgs::Pose2D& goal) {
    RobotWalker walk(n_, 0.4, 0.4, 0);
    std::string msg = "Walking to x: " + std::to_string(goal.x) + ", y: " + std::to_string(goal.y) +
                      ", theta: " + std::to_string(goal.theta);
    ROS_INFO("%s", msg.c_str());
    walk.walkToGoal(goal);
  }

private:
  ros::NodeHandle n_;
  ros::Subscriber sub_;

};

int main(int argc, char **argv) {
  ros::init(argc, argv, "atlas_object_sorting_navigation"); // Initialize ROS node
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(2);
  spinner.start();

  RobotWalkToGoal atlas(nh);

  spinner.stop();

  return 0;
}
