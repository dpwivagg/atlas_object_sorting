#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <std_msgs/String.h>
#include <touch_footstep/robot_walker.h>


void atlas_walk_to_goal(const std::string& image_name, bool status) {
  ROS_INFO("%s status %s", image_name.c_str(), status ? "true" : "false");
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "atlas_object_sorting_navigation"); // Initialize ROS node
  ros::NodeHandle nh; // Create node handle so that all libraries are in the same node

  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::Subscriber nav_goal_sub = nh.subscribe("atlas_object_sorting_perception", 1000, atlas_walk_to_goal);

 // Subscribe to topic that publishes geometry_msgs::Pose2D goal
 // WalkToGoal(goal)

  spinner.stop();

  return 0;
}
