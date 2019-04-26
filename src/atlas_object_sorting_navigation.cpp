#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <std_msgs/String.h>
#include <tough_footstep/robot_walker.h>
#include "geometry_msgs/Pose2D.h"

/*
This node takes in information from the percception node to navigate to the desired goal.
A map generator, a point cloud of the walkway, and a marker array for footstep planning is required.
*/

class RobotWalkToGoal {

public:
  RobotWalkToGoal(ros::NodeHandle nh):n_(nh) {
    // Subscribe to perception node
    sub_ = n_.subscribe("atlas_object_sorting_perception", 1000, &RobotWalkToGoal::callback, this);
  }

  void callback(const geometry_msgs::Pose2D& goal) {
    // Create RobotWalker object
    RobotWalker walk(n_, 0.4, 0.4, 0);
    // Output message
    std::string msg = "Walking to x: " + std::to_string(goal.x) + ", y: " + std::to_string(goal.y) +
                      ", theta: " + std::to_string(goal.theta);
    ROS_INFO("%s", msg.c_str());
    // Call walkToGoal function from robot_walker.cpp to walk to given goal
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

  // Create RobotWalkToGoal object to execute walkToGoal given this node handle
  RobotWalkToGoal atlas(nh);

  If (!atlas(nh)) {
    ROS_INFO("Could not walk to goal.")
  }

  spinner.stop();

  return 0;
}
