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

void show_image(cv::Mat& image, std::string name)
{
  cv::namedWindow(name, cv::WINDOW_AUTOSIZE);
  cv::imshow(name, image);
  ROS_INFO("Press any key to continue");
  cv::waitKey(0);
  cv::destroyWindow(name);
  ros::Duration(0.5).sleep();  // wait some time for the window to destroy cleanly.
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "atlas_object_sorting_perception"); // Initialize ROS node named test_multisense_image
  ros::NodeHandle nh("~"); // Create node handle so that all libraries are in the same node

  std::string desiredColor;
  nh.getParam("color", desiredColor);

  if(desiredColor != "green" && desiredColor != "red" && desiredColor != "blue") {
    ROS_ERROR_STREAM("Invalid color input: " << desiredColor);
    return 1;
  }

  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::Publisher pcPub = nh.advertise<sensor_msgs::PointCloud2>("stereoPointCloud", 1000);
  ros::Publisher debug_publisher = nh.advertise<pcl::PCLPointCloud2>("/debug/RGBDTest", 1);
  ros::Publisher pose_pub = nh.advertise<geometry_msgs::Pose2D>("/colorCentroid", 1);

  tough_perception::MultisenseImageInterfacePtr imageHandler;
  imageHandler = tough_perception::MultisenseImageInterface::getMultisenseImageInterface(nh);
  imageHandler->setSpindleSpeed(2.0);

  ROS_INFO("Multisense image handler initialized");

  tough_perception::MultisenseCameraModel cam_model;
  imageHandler->getCameraInfo(cam_model);

  ROS_INFO("Camera model parameters obtained");

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


    pcl::PCLPointCloud2 output;
    pcl::toPCLPointCloud2(*organized_cloud, output);
    sensor_msgs::PointCloud2 outputRos;
    pcl::toROSMsg(*organized_cloud, outputRos);
    outputRos.header.frame_id = leftOptFrame;
    outputRos.header.stamp = ros::Time::now();
    debug_publisher.publish(outputRos);
    ROS_INFO("Cloud published to /debug/RGBDTest");

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

    if(desiredColor == "blue") blueCentroid.get(point);
    else if(desiredColor == "red") redCentroid.get(point);
    else if(desiredColor == "green") greenCentroid.get(point);

    geometry_msgs::Pose2D goal;
    goal.x = point.x;
    goal.y = point.y;
    goal.theta = 0;
    pose_pub.publish(goal);    

    ROS_INFO_STREAM("Centroid of points: " << point);
  }
  spinner.stop();
  return 0;
}
