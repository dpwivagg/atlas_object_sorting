#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <std_msgs/String.h>
#include <touch_footstep/robot_walker.h>
#include "geometry_msgs/Pose2D.h"


void atlas_walk_to_goal(const geometry_msgs::Pose2D& goal) {
  RobotWalker walk(nh, 1.0, 1.0, 0);
  std::string msg = "Walking to x: " + std::to_string(goal.x) + ", y: " + std::to_string(goal.y) +
                      ", theta: " + std::to_string(goal.theta);
  ROS_INFO("%s", msg.c_str());
  walk.walkToGoal(goal);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "atlas_object_sorting_navigation"); // Initialize ROS node
  ros::NodeHandle nh; // Create node handle so that all libraries are in the same node

  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::Subscriber nav_goal_sub = nh.subscribe("atlas_object_sorting_perception", 1000, atlas_walk_to_goal);

  spinner.stop();

  return 0;
}
