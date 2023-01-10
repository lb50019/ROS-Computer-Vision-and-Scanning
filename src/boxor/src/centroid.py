#!/usr/bin/env python
from __future__ import print_function

import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("centroid_topic",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("robot1/image_raw",Image,self.callback)
    self.blob_pub  = rospy.Publisher("/blob/point_blob",Point,queue_size=1)
    self.blob_point = Point()
  
  def get_blob_relative_position(self, image, xc, yc):
    rows = float(image.shape[0])
    cols = float(image.shape[1])
    # print(rows, cols)
    center_x    = 0.5*cols
    center_y    = 0.5*rows
    # print(center_x)
    x = (xc - center_x)/(center_x)
    y = (yc - center_y)/(center_y)
    return(x,y)  

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)
    
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    lower_lim = np.array([107, 170, 60])
    upper_lim = np.array([144, 255, 153])
    
    mask = cv2.inRange(hsv, lower_lim, upper_lim)
    
    im2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    cv2.drawContours(mask, contours, -1, (0,255,0), 3)
    
    for c in contours:
    	# calculate moments for each contour
    	M = cv2.moments(c)
 
    	# calculate x,y coordinate of center
    	cX = int(M["m10"] / M["m00"])
    	cY = int(M["m01"] / M["m00"])
    	mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
    	cv2.circle(mask, (cX, cY), 5, (0, 255, 0), -1)
    	x, y = self.get_blob_relative_position(mask, cX, cY)
    	self.blob_point.x = x
    	self.blob_point.y = y
    	self.blob_pub.publish(self.blob_point)

    cv2.imshow("centroid", mask)
    cv2.waitKey(3) 

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
