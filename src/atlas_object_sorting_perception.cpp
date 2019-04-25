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

void showImage(cv::Mat &image,std::string name) {
  cv::namedWindow(name,cv::WINDOW_AUTOSIZE);
  cv::imshow(name,image);
  ROS_INFO("Press ESC or q to continue");
  while(cv::waitKey(1)!=27 && cv::waitKey(1)!='q');
  ROS_INFO("closing window");
  cv::destroyWindow(name); // close that window
}

void printStatus(const std::string& image_name, bool status) {
  ROS_INFO("%s status %s", image_name.c_str(), status ? "true" : "false");
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "atlas_object_sorting_perception"); // Initialize ROS node named test_multisense_image
  ros::NodeHandle nh; // Create node handle so that all libraries are in the same node

  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::Publisher pcPub = nh.advertise<sensor_msgs::PointCloud2>("stereoPointCloud", 1000);
  ros::Publisher debug_publisher = nh.advertise<pcl::PCLPointCloud2>("/debug/RGBDTest", 1);

  bool status, pcStatus;

  ROS_INFO("Hello, world!");
  tough_perception::MultisenseImageInterfacePtr imageHandler;
  imageHandler = tough_perception::MultisenseImageInterface::getMultisenseImageInterface(nh);

  ROS_INFO("Interface started");

  tough_perception::MultisenseCameraModel cam_model;
  imageHandler->getCameraInfo(cam_model);

  cv::Mat image, blueMask, redMask, greenMask;
  cv::Mat color;
  cv::Mat_<float> disp;

  cv::Mat depthImage;

  bool new_color = false;
  bool new_disp = false;

  ROS_INFO("Grabbing organized point cloud");

  tough_perception::StereoPointCloudColor::Ptr organized_cloud(new tough_perception::StereoPointCloudColor);

  ROS_INFO("Point cloud recieved");
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
    std::cout << disp.cols << " x " << disp.rows << std::endl;
    tough_perception::generateOrganizedRGBDCloud(disp, color, cam_model.Q, organized_cloud);
    std::cout << "Post point cloud organizing" << std::endl;
    ROS_INFO_STREAM("Organized cloud size: " << organized_cloud->size());
    
    pcl::PCLPointCloud2 output;
    pcl::toPCLPointCloud2(*organized_cloud, output);
    sensor_msgs::PointCloud2 outputRos;
    pcl::toROSMsg(*organized_cloud, outputRos);
    const std::string leftOptFrame = "left_camera_optical_frame";
    outputRos.header.frame_id = leftOptFrame;
    outputRos.header.stamp = ros::Time::now();
    debug_publisher.publish(outputRos);
    ROS_INFO("Cloud published to /debug/RGBDTest");
    new_disp = new_color = false;

    if(!organized_cloud->empty()) {
      // Generate and fill cv Mats for processing
      cv::Mat image(organized_cloud->height, organized_cloud->width, CV_8UC4);

      #pragma omp parallel for
      for(int y = 0; y < image.rows; y++) {
        for(int x = 0; x < image.cols; x++) {
          pcl::PointXYZRGB point = organized_cloud->at(x, y);
          image.at<cv::Vec4b>(y, x)[0] = point.b;
          image.at<cv::Vec4b>(y, x)[1] = point.g;
          image.at<cv::Vec4b>(y, x)[2] = point.r;
          image.at<cv::Vec4b>(y, x)[3] = point.a;
        }
      }
    }

    showImage(image,"BGR Image");

    // Convert the images to binary color threshold images using
    // BGR values indicating a range of blues, greens, and reds that we want to detect

    cv::inRange(image, cv::Scalar(0, 0, 200), cv::Scalar(50, 50, 255), redMask);
    showImage(redMask, "Red-filtered Image");

    cv::inRange(image, cv::Scalar(200, 0, 0), cv::Scalar(255, 50, 50), blueMask);
    showImage(blueMask, "Blue-filtered Image");

    cv::inRange(image, cv::Scalar(0, 200, 0), cv::Scalar(50, 255, 50), greenMask);
    showImage(greenMask, "Green-filtered Image");

    // Compute centroids of each color object -- this will not work if multiple objects of the same color are visible
    // In that case we can find contours of each object and compute the centroids respectively
    // Use https://www.learnopencv.com/find-center-of-blob-centroid-using-opencv-cpp-python/

    cv::Moments blueMoments = moments(blueMask, true);

    cv::Point p(blueMoments.m10/blueMoments.m00, blueMoments.m01/blueMoments.m00);

    // Plot a red circle on the blue centroid for visualization purposes
    circle(image, p, 1, CV_RGB(255,0,0), 3);
    showImage(image, "Blue Centroid");

  }

  // const geometry_msgs::Pose2D goal

  spinner.stop();

// *************************************************************
// Image color filtering code below
// *************************************************************
/*
  status = imageHandler->getImage(image);

  // Get a point cloud from the stereo vision system
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  sensor_msgs::PointCloud2 rosCloud;
  // tough_perception::StereoPointCloud cloud;
  pcStatus = pcHandler.giveStereoCloud(cloud);

  ROS_INFO("point cloud status %s",pcStatus ? "true" : "false");
  if(pcStatus) {
    pcl::toROSMsg(*cloud, rosCloud);
    pcPub.publish(rosCloud);
  }
  
  ROS_INFO_STREAM("[Height]" << imageHandler.giveHeight() << " [width]" <<imageHandler.giveWidth());
  ROS_INFO("image status %s",status ? "true" : "false");

  if(1 == 0) {
    showImage(image,"BGR Image");

    // Convert the images to binary color threshold images using
    // BGR values indicating a range of blues, greens, and reds that we want to detect

    cv::inRange(image, cv::Scalar(0, 0, 200), cv::Scalar(50, 50, 255), redMask);
    showImage(redMask, "Red-filtered Image");

    cv::inRange(image, cv::Scalar(200, 0, 0), cv::Scalar(255, 50, 50), blueMask);
    showImage(blueMask, "Blue-filtered Image");

    cv::inRange(image, cv::Scalar(0, 200, 0), cv::Scalar(50, 255, 50), greenMask);
    showImage(greenMask, "Green-filtered Image");

    // Compute centroids of each color object -- this will not work if multiple objects of the same color are visible
    // In that case we can find contours of each object and compute the centroids respectively
    // Use https://www.learnopencv.com/find-center-of-blob-centroid-using-opencv-cpp-python/

    cv::Moments blueMoments = moments(blueMask, true);

    cv::Point p(blueMoments.m10/blueMoments.m00, blueMoments.m01/blueMoments.m00);

    // Plot a red circle on the blue centroid for visualization purposes
    circle(image, p, 1, CV_RGB(255,0,0), 3);
    showImage(image, "Blue Centroid");

    // TODO: Use LIDAR or stereo to detect depth for determining 3D location for walking target
  }
*/
  return 0;
}
