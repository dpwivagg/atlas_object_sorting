#include <stdlib.h>
#include <stdio.h>
#include <std_msgs/String.h>
#include <tough_perception_common/MultisenseImage.h>
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
  bool status;

  tough_perception::MultisenseImage imageHandler(nh);

  cv::Mat image, blueMask, redMask, greenMask, 
          blueGreyscale, redGreyscale, greenGreyscale,
          blueBinary, redBinary, greenBinary;
  status = imageHandler.giveImage(image);
    
  ROS_INFO_STREAM("[Height]" << imageHandler.giveHeight() << " [width]" <<imageHandler.giveWidth());
  ROS_INFO("image status %s",status ? "true" : "false");

  if(status) {
    showImage(image,"RGB Image");

    // RGB values indicating a range of blues that we want to detect
    cv::inRange(image, cv::Scalar(0, 0, 200), cv::Scalar(20, 20, 255), blueMask);
    showImage(blueMask, "Blue-filtered Image");

    // RGB values indicating a range of reds that we want to detect
    cv::inRange(image, cv::Scalar(200, 0, 0), cv::Scalar(225, 20, 20), redMask);
    showImage(redMask, "Red-filtered Image");

    // RGB values indicating a range of greens that we want to detect
    cv::inRange(image, cv::Scalar(0, 200, 0), cv::Scalar(20, 225, 20), greenMask);
    showImage(greenMask, "Green-filtered Image");

    // Convert the images to grayscale and binarize

    cv::cvtColor(blueMask, blueGreyscale, CV_RGB2GRAY);
    cv::cvtColor(redMask, redGreyscale, CV_RGB2GRAY);
    cv::cvtColor(greenMask, greenGreyscale, CV_RGB2GRAY);

    blueBinary = blueGreyscale > 128;
    redBinary = redGreyscale > 128;
    greenBinary = greenGreyscale > 128;

    // Compute centroids of each color object -- this will not work if multiple objects of the same color are visible
    // In that case we can find contours of each object and compute the centroids respectively
    // Use https://www.learnopencv.com/find-center-of-blob-centroid-using-opencv-cpp-python/

    cv::Moments blueMoments = moments(blueBinary, true);
    cv::Point p(blueMoments.m10/blueMoments.m00, blueMoments.m01/blueMoments.m00);

    // Plot a red circle on the blue centroid for visualization purposes
    circle(blueGreyscale, p, 1, CV_RGB(255,0,0), 3);
    showImage(blueGreyscale, "Green-filtered Image");

    // TODO: Use LIDAR or stereo to detect depth for determining 3D location for walking target
  }

  return 0;
}
