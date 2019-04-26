#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <std_msgs/String.h>
#include <tough_perception_common/MultisenseImageInterface.h>
#include <tough_perception_common/MultisensePointCloud.h>
#include <opencv2/core.hpp>
#include <opencv/highgui.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseStamped.h"

typedef pcl::PointXYZRGB PointT;

int main(int argc, char **argv) {
  ros::init(argc, argv, "atlas_object_sorting_perception"); // Initialize ROS node named test_multisense_image
  ros::NodeHandle nh; // Create node handle so that all libraries are in the same node

  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::Publisher pcPub = nh.advertise<sensor_msgs::PointCloud2>("stereoPointCloud", 1000);
  ros::Publisher debug_publisher = nh.advertise<pcl::PCLPointCloud2>("/debug/RGBD", 1);
  ros::Publisher pose_pub = nh.advertise<geometry_msgs::Pose2D>("walkingGoal", 1);
  ros::Publisher pose_pub_rviz = nh.advertise<geometry_msgs::PoseStamped>("walkingGoalRviz", 1);

  tough_perception::MultisenseImageInterfacePtr imageHandler;
  imageHandler = tough_perception::MultisenseImageInterface::getMultisenseImageInterface(nh);

  tough_perception::MultisenseCameraModel cam_model;
  imageHandler->getCameraInfo(cam_model);
  
  cv::Mat color;
  cv::Mat_<float> disp;

  bool new_color = false;
  bool new_disp = false;

  tough_perception::StereoPointCloudColor::Ptr organized_cloud(new tough_perception::StereoPointCloudColor);

  if (imageHandler->getImage(color))
  {
    new_color = true;
  }
  ROS_INFO("image status %s",new_color ? "true" : "false");
  if (imageHandler->getDisparity(disp, true))
  {
    new_disp = true;
  }
  ROS_INFO("disparity status %s",new_disp ? "true" : "false");
  
  if (new_disp && new_color)
  {
    tough_perception::generateOrganizedRGBDCloud(disp, color, cam_model.Q, organized_cloud);
    ROS_INFO_STREAM("Organized cloud size: " << organized_cloud->size());
    
    const std::string leftOptFrame = "left_camera_optical_frame";
    new_disp = new_color = false;

    ROS_INFO("Computing color-based centroids");

    pcl::ConstCloudIterator<PointT> pointIter(*organized_cloud);
    pcl::CentroidPoint<PointT> blueCentroid, redCentroid, greenCentroid;
    PointT point;
    for(int i = pointIter.size(); pointIter.getCurrentIndex() < i; pointIter++) {
      // Points not within the specified ranges in x, y, and z are considered to be noise and we ignore them
      if(!(pointIter->x < 0.2 && pointIter->y > 0 && pointIter->z > 0)) continue;

      uint8_t r = pointIter->r;
      uint8_t g = pointIter->g;
      uint8_t b = pointIter->b;

      // For the non-noise points we add them to a centroid of the dominant color in that point
      // This excludes all grey points, which are irrelevant
      if(b > r && b > g) {
        blueCentroid.add(*pointIter);
      } else if(r > b && r > g) {
        redCentroid.add(*pointIter);
      } else if(g > b && g > r) {
        greenCentroid.add(*pointIter);
      }
    }

    blueCentroid.get(point);

    geometry_msgs::Pose2D goal;
    goal.x = point.x;
    goal.y = point.y;
    goal.theta = 0;
    pose_pub.publish(goal);    

    ROS_INFO_STREAM("Centroid of points: " << point);

    spinner.stop();
    return 0;
}
