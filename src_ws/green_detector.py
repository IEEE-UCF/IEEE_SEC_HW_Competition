import cv2
import numpy as np  
import os
import subprocess

class GreenDetector():

  def __init__(self):

    f = open("greenlightstart.txt", "w")
    f.write("")

    global cap
    cap = cv2.VideoCapture("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)1920, height=(int)1080,format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert !  appsink")
    if not cap.isOpened():
      self.get_logger().info("Error opening video stream")
      return
    else: 
      self.get_logger().info("Video stream opened")

    global cv_image
    cv_image = cap.read()

    cv_image     = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)  

    lower_green = np.array([40,50,50]) 
    upper_green = np.array([70,255,255]) 

    working_image = cv2.inRange(cv_image, lower_green, upper_green)

    # Dilate and Erode
    working_image = cv2.dilate(working_image, None, iterations=2)
    working_image = cv2.erode(working_image, None, iterations=2)

    contours, hierarchy = cv2.findContours(working_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
      if(cv2.contourArea(contour)>100):
        f.write("start")

    return

  def publishImg(self):
    self.get_logger().info("Sending start message")
    
    cv_image = cap.read()
    if cv_image is not None:
        # send the image 
      return  