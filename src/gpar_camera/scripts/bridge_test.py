#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('gpar_camera')
import sys
import rospy
import cv2
import time
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
#    self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)
    cv2.namedWindow('mouseRGB')
    cv2.setMouseCallback('mouseRGB',self.mouseRGB)


  def mouseRGB(event,flags,x,y,param,z):
          rospy.loginfo('x %f , y %f',x,y)

         



  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)

    cv2.imshow('mouseRGB', cv_image)
    cv2.waitKey(3)

    try:
      image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
      image_msg.header.frame_id = 'map'
      # self.image_pub.publish(image_msg)
    except CvBridgeError as e:
      print(e)

def main(args):
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

