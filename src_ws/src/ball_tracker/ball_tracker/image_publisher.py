import rclpy
from rclpy.node import Node
from sensor_msgs.msg        import Image
from geometry_msgs.msg      import Point, Twist
from cv_bridge              import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
import rclpy
import time
import subprocess

class ImagePublisher(object):

  def __init__(self):

    self.node_rate = 1

    self.image_pub = rospy.Publisher("camera/image_raw", Image)
    self.bridge = CvBridge()

    global cap
    cap = cv2.VideoCapture("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)1920, height=(int)1080,format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, format=(string)BGRx ! videoconvert !  appsink")
    if not cap.isOpened():
      rospy.logerr("Error opening video stream")
      return

    global cv_image
    cv_image = cap.read()

    try:
      self.image_message = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
    except CvBridgeError as e:
      print(e)

    time.sleep(5)
    self.image_pub.publish(self.image_message)

  def publishImg(self):
    rospy.loginfo('publishing image!')
    
    cv_image = cap.read()
    if cv_image is not None:
        self.image_message = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        self.image_pub.publish(self.image_message)

  def run(self):
    loop = rospy.Rate(self.node_rate)
    while not rospy.is_shutdown():
      self.publishImg()
      loop.sleep()

def main(args=None):
  rclpy.init(args=args)
  image_publisher = ImagePublisher()
  rclpy.spin(image_publisher)
  image_publisher.destroy_node()
  rclpy.shutdown()