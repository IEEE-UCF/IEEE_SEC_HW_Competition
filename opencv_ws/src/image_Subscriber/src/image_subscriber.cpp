#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/highgui.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace cv;
using namespace std;

void findCubes(cv::Mat img);

void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  try {
    cv_bridge::CvImageConstPtr bridgeImg = cv_bridge::toCvShare(msg, "bgr8");
    cv::Mat img = bridgeImg->image;
    if(!(img.empty())){
      //auto logger = rclcpp::get_logger("image_subscriber");
      //RCLCPP_ERROR(logger, "Could not convert from '%s' to 'bgr8'.",img.);
      cv::imwrite("Test.bmp", img);
      findCubes(img);
      //cv::imshow("view", img);
    }
    cv::waitKey(10);
  } catch (const cv_bridge::Exception & e) {
    auto logger = rclcpp::get_logger("image_subscriber");
    RCLCPP_ERROR(logger, "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  } catch (cv::Exception & e){
    // cv_bridge::CvImageConstPtr bridgeImg = cv_bridge::toCvShare(msg, "bgr8");
    // cv::Mat img = bridgeImg->image;
    // cv::Size size = img.size();
    // auto logger = rclcpp::get_logger("image_subscriber");
    // RCLCPP_ERROR(logger, "Width: '%d', Height: '%d'", size.width, size.height);
    auto logger = rclcpp::get_logger("image_subscriber");
    RCLCPP_ERROR(logger, "'%s'", msg->encoding.c_str());
  }
}
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("image_listener", options);
  // TransportHints does not actually declare the parameter
  node->declare_parameter<std::string>("image_transport", "raw");
  //cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(node);
  image_transport::TransportHints hints(node.get());
  image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback, &hints);
  rclcpp::spin(node);
  cv::destroyWindow("view");

  return 0;
}

 void findCubes( cv::Mat imgOriginal )
 {

  //namedWindow("Control", WINDOW_AUTOSIZE); //create a window called "Control"

 int iLowH = 170;
 int iHighH = 179;

 int iLowS = 150; 
 int iHighS = 255;

 int iLowV = 60;
 int iHighV = 255;

 int pixelArea = 1000;

 //Create trackbars in "Control" window
//  createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
//  createTrackbar("HighH", "Control", &iHighH, 179);

//  createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
//  createTrackbar("HighS", "Control", &iHighS, 255);

//  createTrackbar("LowV", "Control", &iLowV, 255);//Value (0 - 255)
//  createTrackbar("HighV", "Control", &iHighV, 255);

//  createTrackbar("Minimum Area", "Control", &pixelArea, 15000);

 //int iLastX = -1; 
 //int iLastY = -1;

 //Capture a temporary image from the camera
  cv::Mat imgTmp;
 //cap.read(imgTmp); 
  imgTmp = imgOriginal.clone();
 
 //Create a black image with the size as the camera output
  cv::Mat imgLines = Mat::zeros( imgTmp.size(), CV_8UC3 );;

  cv::Mat imgHSV;

  cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
 
  cv::Mat imgThresholded;

  inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
      
  //morphological opening (removes small objects from the foreground)
  erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
  dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

  //morphological closing (removes small holes from the foreground)
  dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
  erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );



  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours( imgThresholded, 
                    contours, 
                    hierarchy, 
                    RETR_TREE, 
                    CHAIN_APPROX_SIMPLE, 
                    cv::Point(0, 0) );

  std::vector<std::vector<cv::Point> > contours_poly( contours.size() );
  std::vector<cv::Rect> boundRect( contours.size() );
  Mat tmp = cv::Mat::zeros( imgThresholded.size(), CV_8UC3 );
   for( unsigned long int i = 0; i < contours.size(); i++ ){ 
          approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true );
          boundRect[i] = cv::boundingRect( cv::Mat(contours_poly[i]) );
          if(boundRect[i].area() > pixelArea){
            rectangle( tmp, boundRect[i].tl(), boundRect[i].br(), cv::Scalar(0, 255, 0), 2, 8, 0 );
            //line(tmp, boundRect[i].tl(), boundRect[i].br(), Scalar(0,0,255), 5); 
            line(tmp, (boundRect[i].tl() + boundRect[i].br())/2, (boundRect[i].tl() + boundRect[i].br())/2, Scalar(0,255,0), 5); 
          }
   }           

   cv::imwrite("Rectangle Image.bmp", tmp); 
   cv::imwrite("Thresholded Image.bmp", imgThresholded); //show the thresholded image

   imgOriginal = imgOriginal + imgLines;
   cv::imwrite("Original.bmp", imgOriginal); //show the original image

   return;
}