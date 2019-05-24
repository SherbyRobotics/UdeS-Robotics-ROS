#!/usr/bin/env python

import math
import sys
import time
import cv2
import numpy as np
import rospy


from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from collections import deque

class LaneDetector:

  ##########################################################

  def __init__(self):

    self.firstRun = True
    self.t_0 = rospy.get_rostime()
    self.image_pub = rospy.Publisher("ycam_msg", Float32, queue_size=1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("cam_img_raw",Image,self.callback)

    self.init = 0
    self.detec = 0
    self.state_y_last = 0

  ##########################################################
  
  def callback(self,data):
 
    width = 0.4
    image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    green = self.select_green(image)
    smooth_gray  = self.apply_smoothing(green)
    edges        = self.detect_edges(smooth_gray)
    regions      = self.select_region(edges)
    regions_ori  = self.select_region(image)
    lines, img_line       = self.hough_lines(regions)

    if lines is None:
      state_y = self.state_y_last
      msg = Float32()
      msg.data = state_y
      self.image_pub.publish(msg)   
      return None

    botx,boty,upx,upy = self.average_dot(lines)
 
    if botx is not None and upx is not None:
      state_y      = 320-upx
      final_img    = self.draw_lines(image, botx, boty, upx, upy, state_y)
      
      msg = Float32()
      msg.data = state_y
      self.image_pub.publish(msg)
      self.state_y_last = state_y
      if self.init == 0:
        rospy.loginfo("Lane detected: Publishing data...")
        self.init = 1

      #cv2.imshow("GRAY", smooth_gray)
      #cv2.imshow("Canny", edges)
      #cv2.imshow("ROI", regions)
      #cv2.imshow("ROI ori", regions_ori)
      cv2.imshow("Lines", img_line)
      cv2.imshow("Final", final_img)
      cv2.waitKey(3)      
      #cv2.imshow("Filtered",white_yellow)

  ##########################################################

  def select_green(self, image):
    
    converted = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)
    # green color mask
    lower = np.uint8([  50, 150, 150])
    upper = np.uint8([150, 255, 255])
    green_mask = cv2.inRange(converted, lower, upper)

    return cv2.bitwise_and(image, image, mask = green_mask)


  ##########################################################

  def apply_smoothing(self, image, kernel_size=15):

    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    """
    kernel_size must be postivie and odd
    """
    return cv2.GaussianBlur(image, (kernel_size, kernel_size), 0)

  ##########################################################
  
  def detect_edges(self, image, low_threshold=50, high_threshold=150):
    return cv2.Canny(image, low_threshold, high_threshold)

 ##########################################################

  def filter_region(self, image, vertices):
    """
    Create the mask using the vertices and apply it to the input image
    """
    mask = np.zeros_like(image)
    if len(mask.shape)==2:
        cv2.fillPoly(mask, vertices, 255)
    else:
        cv2.fillPoly(mask, vertices, (255,)*mask.shape[2]) # in case, the input image has a channel dimension        
    return cv2.bitwise_and(image, mask)

  ##########################################################

  def select_region(self, image):
    """
    It keeps the region surrounded by the `vertices` (i.e. polygon).  Other area is set to 0 (black).
    """
    # first, define the polygon by vertices
    rows, cols = image.shape[:2]
    bottom_left  = [cols*0.00, rows*0.9]
    top_left     = [cols*0.2, rows*0.4]
    bottom_right = [cols*1.00, rows*0.9]
    top_right    = [cols*0.8, rows*0.4] 
    # the vertices are an array of polygons (i.e array of arrays) and the data type must be integer
    vertices = np.array([[bottom_left, top_left, top_right, bottom_right]], dtype=np.int32)
    return self.filter_region(image, vertices)

  ##########################################################
  
  def hough_lines(self, image):
    """
    `image` should be the output of a Canny transform.
    
    Returns hough lines (not the image with lines)
    """
    img_line = np.copy(image)
    clr_img_line = cv2.cvtColor(img_line, cv2.COLOR_GRAY2BGR)
    lines = cv2.HoughLinesP(image, rho=1, theta=np.pi/180, threshold=20, minLineLength=20, maxLineGap=300)

   # Time
    t_now = rospy.get_rostime()
    t_duration = t_now - self.t_0
    t_secs = int(t_duration.secs)
    t_lostlane = t_now

   # Gives info about wether or not lane have been detected
    if lines is None and self.firstRun is True:
        rospy.loginfo("Starting vision algorithm...")
       

   # Print the lines detected by the Hough transform on the original image
    if lines is not None:
        for i in range(0, len(lines)):
            l = lines[i][0]
            cv2.line(clr_img_line, (l[0], l[1]), (l[2], l[3]), (0,0,255), 2, cv2.LINE_AA)

   # Turn the "firstRun" parameter to false on the first run
    self.firstRun = False
            
    return lines, clr_img_line

 
  ##########################################################

  def average_dot(self, lines):

    bottomdotx = [] 
    bottomdoty = []
    upperdotx  = [] 
    upperdoty  = [] 

    if lines is None:
      return None
    
    for line in lines:
        for x1, y1, x2, y2 in line:
            bottomdotx.append(x1)
	    bottomdoty.append(y1)
	    upperdotx.append(x2)
            upperdoty.append(y2)
    
    # Mean the bottom and upper dots    
    botx  = np.sum(bottomdotx)/len(bottomdotx)  if len(bottomdotx) >0 else None
    boty  = np.sum(bottomdoty)/len(bottomdoty)  if len(bottomdoty) >0 else None
    upx   = np.sum(upperdotx)/len(upperdotx)  if len(upperdotx) >0 else None
    upy   = np.sum(upperdoty)/len(upperdoty)  if len(upperdoty) >0 else None
    return botx, boty, upx, upy

  ##########################################################

  def draw_lines(self, image, botx, boty, upx, upy, state_y, color=[0, 0, 255], thickness=20):
    line_image = np.zeros_like(image)
    cv2.line(line_image, (botx,boty), (upx,upy),  color, thickness)
    font = cv2.FONT_HERSHEY_SIMPLEX
    bottomLeftCornerOfText = (180,400)
    fontScale              = 1
    fontColor              = (0,0,255)
    lineType               = 2
    cv2.putText(line_image,"State y = %.5f" %state_y, bottomLeftCornerOfText,font,fontScale,fontColor,lineType)        
    final_img = cv2.addWeighted(image, 1.0, line_image, 1.0, 0.0)

    return final_img

  ##########################################################

if __name__ == '__main__':

  rospy.init_node('LINE_DETECTION', anonymous=False)
  node = LaneDetector()
  rospy.spin()
