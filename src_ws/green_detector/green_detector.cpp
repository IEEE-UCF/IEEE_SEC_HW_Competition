#include <opencv2/opencv.hpp> 
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <fstream>

using namespace cv;
using namespace std;

 int main( int argc, char** argv )
 {
    VideoCapture cap("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)1920, height=(int)1080,format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert !  appsink");
    //VideoCapture cap(-1); //capture the video from webcam

    if ( !cap.isOpened() )  // if not success, exit program
    {
         std::cout << "Cannot open the web cam" << std::endl;
         return -1;
    }

//     namedWindow("Control", WINDOW_AUTOSIZE); //create a window called "Control"

 int iLowH = 80;
 int iHighH = 90;

 int iLowS = 50; 
 int iHighS = 250;

 int iLowV = 86;
 int iHighV = 255;

 int pixelArea = 500;

//  createTrackbar("Minimum Area", "Control", &pixelArea, 15000);


 //Create trackbars in "Control" window
//  createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
//  createTrackbar("HighH", "Control", &iHighH, 179);

//  createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
//  createTrackbar("HighS", "Control", &iHighS, 255);

//  createTrackbar("LowV", "Control", &iLowV, 255);//Value (0 - 255)
//  createTrackbar("HighV", "Control", &iHighV, 255);

 int iLastX = -1; 
 int iLastY = -1;

 //Capture a temporary image from the camera
 Mat imgTmp;
 cap.read(imgTmp); 

 //Create a black image with the size as the camera output
 Mat imgLines = Mat::zeros( imgTmp.size(), CV_8UC3 );;
 
 ofstream myfile("greenlightstart.txt");
 myfile << "";
 myfile.close();

    while (true)
    {
        Mat imgOriginal;

        bool bSuccess = cap.read(imgOriginal); // read a new frame from video



         if (!bSuccess) //if not success, break loop
        {
             cout << "Cannot read a frame from video stream" << endl;
             break;
        }

  Mat imgHSV;

  cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
 
  Mat imgThresholded;

  inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
      
  //morphological opening (removes small objects from the foreground)
  erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
  dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

  //morphological closing (removes small holes from the foreground)
  dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
  erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

  /*
  //Calculate the moments of the thresholded image
  Moments oMoments = moments(imgThresholded);

  double dM01 = oMoments.m01;
  double dM10 = oMoments.m10;
  double dArea = oMoments.m00;

  // if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
  if (dArea > 10000)
  {
   //calculate the position of the ball
   int posX = dM10 / dArea;
   int posY = dM01 / dArea;        
        
   if (iLastX >= 0 && iLastY >= 0 && posX >= 0 && posY >= 0)
   {
    //Draw a red line from the previous point to the current point
    line(imgLines, Point(posX, posY), Point(iLastX, iLastY), Scalar(0,0,255), 2);
   }

   iLastX = posX;
   iLastY = posY;
  }
  */

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
for( int i = 0; i < contours.size(); i++ ){ 
          cv::approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true );
          boundRect[i] = cv::boundingRect( cv::Mat(contours_poly[i]) );
          if(boundRect[i].area() > pixelArea){
          //   rectangle( imgOriginal, boundRect[i].tl(), boundRect[i].br(), cv::Scalar(0, 255, 0), 2, 8, 0 );
            //line(tmp, boundRect[i].tl(), boundRect[i].br(), Scalar(0,0,255), 5); 
          //   line(imgOriginal, (boundRect[i].tl() + boundRect[i].br())/2, (boundRect[i].tl() + boundRect[i].br())/2, Scalar(0,255,0), 5); 
            ofstream myfile("greenlightstart.txt");
            myfile << "start";
            myfile.close();
          }
  }
 
  //imshow("Rectangle Image", tmp); 
  //imshow("Thresholded Image", imgThresholded); //show the thresholded image

  //imgOriginal = imgOriginal + imgLines;
//   imshow("Original", imgOriginal); //show the original image

//         if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
//        {
//             cout << "esc key is pressed by user" << endl;
//             break; 
//        }
    }

   return 0;
}