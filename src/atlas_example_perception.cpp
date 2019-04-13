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

  cv::Mat image, blueMask, redMask, greenMask;
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

    // cv::GaussianBlur( gray, blur, cv::Size(3, 3), 0, 0, cv::BORDER_DEFAULT ); // Blur
    // showImage( blur, "Blurred Image");

    // cv::Laplacian( blur, edges, CV_16S, 3, 1, 0, cv::BORDER_DEFAULT ); // Compute Laplacian
    // cv::convertScaleAbs( edges, abs_edges );
    // showImage( abs_edges, "Edges");
  }

  return 0;
}
