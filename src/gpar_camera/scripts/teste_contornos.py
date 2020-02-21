#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('gpar_camera')
import sys
import rospy
import cv2
import numpy as np
#from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Vector3


def nothing(x):
    pass



class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image,queue_size=10)
    self.vec_pub = rospy.Publisher('vetor_drone', Vector3, queue_size=10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
      
          # Redimensiona imagem
    scale_percent = 100 # percent of original size
    width = int(cv_image.shape[1] * scale_percent / 100)
    height = int(cv_image.shape[0] * scale_percent / 100)
    dim = (width, height)
    img = cv2.resize(cv_image, dim)
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lower_blue = np.array([5, 100, 100])
    upper_blue = np.array([20, 255, 255])

    mask = cv2.inRange(img_hsv, lower_blue, upper_blue)

    # Filtragem de imagem - Reduz ruidos    
    nf = 2
    kernel_er = np.ones((nf, nf), np.uint8)
    erosion = cv2.erode(mask, kernel_er, iterations=3)
    cv2.imshow('erosion', erosion)
    # cv2.waitKey(0)

    # Adiciona filtro, borrando imagem
   # gaussied = cv2.GaussianBlur(erosion, (5, 5), 0)
   # cv2.imshow('gaussian', gaussied)


    # Filtragem de imagem - Preenche vazios
    nf = 5
    kernel_dil = np.ones((nf, nf), np.uint8)
    dilation = cv2.dilate(erosion, kernel_dil, iterations=1)
    cv2.imshow('dilation', dilation)
     
    (_,contours,_) = cv2.findContours(dilation, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        area = cv2.contourArea(contour)
        if(area > 7000):
	    x,y,w,h = cv2.boundingRect(contour)

	    frame_rect = cv2.rectangle(img, (x,y),(x+w,y+h),(0,0,255),10)
            pos = [x,y, w, h]
	    x_ret = x+w/2
	    y_ret = y+h/2
            print([ x_ret, y_ret]) # centro do retangulo
	    v_x = x_ret - width/2
	    v_y = y_ret - height/2
	    vec3 = Vector3()
            vec3.x = v_x
            vec3.y = v_y		
            self.vec_pub.publish(vec3)
            print('vetor = ' ,[v_x,-v_y])
	    pt1 = (width/2, height/2)
   	    pt2 = (x_ret, y_ret)
	    cv2.arrowedLine(img, pt1, pt2, (255,0,0), 2)
            #cv2.imshow("tracking", frame_rect)
            
    cv2.imshow("img", img)
    cv2.imshow("Image window", mask)
    cv2.waitKey(3)


    try:
      image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
      image_msg.header.frame_id = 'map'
      self.image_pub.publish(image_msg)
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

