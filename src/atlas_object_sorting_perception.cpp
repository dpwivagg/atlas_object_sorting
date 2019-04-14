#include <stdlib.h>
#include <stdio.h>
#include <std_msgs/String.h>
#include <tough_perception_common/MultisenseImage.h>
#include <tough_perception_common/MultisensePointCloud.h>
#include <opencv2/core.hpp>
#include <opencv/highgui.h>

void showImage(cv::Mat &image,std::string name)
{
    cv::namedWindow(name,cv::WINDOW_AUTOSIZE);
    cv::imshow(name,image);
    ROS_INFO("Press ESC or q to continue");
    while(cv::waitKey(1)!=27 && cv::waitKey(1)!='q');
    ROS_INFO("closing window");
    cv::destroyWindow(name); // close that window
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_multisense_image"); // Initialize ROS node named test_multisense_image
  ros::NodeHandle nh; // Create node handle so that all libraries are in the same node
  bool status, pcStatus;

  tough_perception::MultisenseImage imageHandler(nh);
  tough_perception::MultisensePointCloud pcHandler(nh, "/head", "/left_camera_optical_frame");

  cv::Mat image, blueMask, redMask, greenMask;
  status = imageHandler.giveImage(image);

  // Get a point cloud from the stereo vision system
  // tough_perception::StereoPointCloud cloud;
  // pcStatus = pcHandler.giveStereoCloud(cloud);
    
  ROS_INFO_STREAM("[Height]" << imageHandler.giveHeight() << " [width]" <<imageHandler.giveWidth());
  ROS_INFO("image status %s",status ? "true" : "false");

  if(status) {
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

  return 0;
}
