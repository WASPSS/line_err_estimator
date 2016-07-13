#!/usr/bin/env python
from __future__ import print_function
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time
from geometry_msgs.msg import Twist

class image_converter:

  def __init__(self):
    self.err_pub = rospy.Publisher("/control/line_err",Twist,queue_size=1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/ardrone/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape

    # Convert BGR to HSV
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)



    # define range of blue color in HSV
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])

    lower_red2 = np.array([160, 100, 100])
    upper_red2 =np.array([179, 255, 255])

    # Threshold the HSV image to get only blue colors
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.add(mask1,mask2)

    # Canny edge and Hough transform
    mask = cv2.Canny(mask, 10, 100)
    lines = cv2.HoughLines(mask, 1,np.pi/180,100)
    if lines is not None:
        lines = np.mean(lines, axis=0)
        theta = lines[0][1]
        rho = 720/2-lines[0][0]
        if theta > np.pi/2:
            theta = -(np.pi-theta)
        err_twist = Twist()
        err_twist.angular.z = theta
        err_twist.linear.y = rho
        self.err_pub.publish(err_twist)

    #cv2.imshow("Image window", cv_image)
    #cv2.waitKey(3)



def main(args):
  ic = image_converter()
  rospy.init_node('line_err_estimator', anonymous=True)

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
