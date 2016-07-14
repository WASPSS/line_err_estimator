#!/usr/bin/env python
from __future__ import print_function
from __future__ import division
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
      # Convert BGR to HSV

    (rows,cols,channels) = cv_image.shape
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
    lines = cv2.HoughLines(mask, 5,np.pi/90,200)

    if lines is not None:
        for rho,theta in lines[0]:
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho
            x1 = int(x0 + 1000*(-b))
            y1 = int(y0 + 1000*(a))
            x2 = int(x0 - 1000*(-b))
            y2 = int(y0 - 1000*(a))
            if (x1-x2)!=0:
                k = (y2-y1)/(x2-x1)
                m = y1-k*x1
            else:
                k = float("inf")
                m = float("inf")
        y_c = 180
        y_0 = 0
        if k!=float("inf") and k!=0:
            x_c = (y_c-m)/k
            x_0 = (y_0-m)/k
        else:
            x_c = x1
            x_0 = x1

        cv2.line(cv_image,(int(x_c),int(y_c)),(int(x_0),int(y_0)),(0,0,255),2)

        if k!=0:
            theta = 1/k
        else:
            theta = 0
        rho = 640/2-x_c
        if theta > np.pi/2:
            theta = -(np.pi-theta)
        err_twist = Twist()
        err_twist.angular.z = theta
        err_twist.linear.y = rho
        self.err_pub.publish(err_twist)




    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)



def main(args):
  rospy.init_node('line_err_estimator', anonymous=True)
  ic = image_converter()


  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
