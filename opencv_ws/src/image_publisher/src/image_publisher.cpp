#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include <opencv2/opencv.hpp> 
#include <iostream>
#include "opencv2/imgproc/imgproc.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("image_publisher", options);
  image_transport::ImageTransport it(node);
  image_transport::Publisher pub = it.advertise("camera/image_raw", 1);

  //cv::Mat image = cv::imread(argv[1], cv::IMREAD_COLOR);

  cv::VideoCapture cap("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)1920, height=(int)1080,format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert !  appsink");

  if ( !cap.isOpened() )  // if not success, exit program
    {
         std::cout << "Cannot open the web cam" << std::endl;
         return -1;
    }

  std_msgs::msg::Header hdr;
  cv::Mat image;
  sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(hdr, "bgr8", image).toImageMsg();

    //std::cout << "Cannot open the web cam" << std::endl;
    cap.read(image);
    //std::cout << "hey" << std::endl;
    msg = cv_bridge::CvImage(hdr, "bgr8", image).toImageMsg();
    pub.publish(msg);

}