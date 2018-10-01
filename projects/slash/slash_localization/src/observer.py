#!/usr/bin/env python

import math
import time
from slash_vision.msg import CamState
from razor_imu_9dof.msg import ImuStates
from slash_localization.msg import StatesHat
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt


import numpy as np
import rospy
import sys


class Observer:
  ##########################################################

  def __init__(self):

    self.image_sub = rospy.Subscriber("ycam_msg",CamState,self.callback1)
    self.input_sub = rospy.Subscriber("cmd_vel",Twist,self.callback3)
    self.imu_sub = rospy.Subscriber("imu_msg",ImuStates,self.callback2)
  
    self.states_pub = rospy.Publisher("x_hat", StatesHat, queue_size=1)

    self.tlast = rospy.get_rostime()
    self.tgain = -10
    self.dgain = 15*3.1416/180
    self.x = np.array([[0],[0],[0]])
    self.torque = 0
    self.delta = 0
    self.observer = 1

  ##########################################################

  
  def callback1(self,cam):
      self.cam_y     = cam.state_y

      
  ##########################################################

  
  def callback3(self, cmd):
      torque_cmd = cmd.linear.x
      self.torque     = self.tgain*torque_cmd # N/m
      delta_cmd  = cmd.angular.z
      self.delta      = self.dgain*delta_cmd  # degree


  ##########################################################

  
  def callback2(self,imu):
      self.imu_accx     = imu.state_accx
      self.imu_accy     = imu.state_accy
      self.imu_theta = imu.state_theta
      self.t = rospy.get_rostime()

      if self.cam_y == 1000:
        self.obsB(self.imu_accx,self.imu_accy,self.imu_theta,self.torque,self.delta, self.t)       
      else:
        self.obsA(self.cam_y,self.imu_accx,self.imu_accy,self.imu_theta,self.torque,self.delta, self.t)        

  #######################################################
  #--------------Observer A, with vision----------------#
  #######################################################

  def obsA(self, cam_y, imu_accx, imu_accy, imu_theta, torque, delta, t):

     if self.observer == 1:
        rospy.loginfo('Lane detected: Data based on observer A with vision')
        self.observer = 2

     a = 0.3
     b = 0.8
     c1 = 1
     c2 = 1
     v = 1
     dt = (t-self.tlast).to_sec()
 
  #---------------------State-Space---------------------#

     A = np.array([[-c2,0.0,0.0],[0.0,0.0,v],[0.0,0.0,0.0]])
     B = np.array([[c1,0.0],[0.0,0.0],[0.0,v/b]])
     C = np.array([[0.0,1.0,0.0],[-c2,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,1.0]])
     D = np.array([[0.0,0.0],[c1,0.0],[0.0,-v**2],[0.0,0.0]])
     L = np.array([[0, 0.4, 0, 0],[1,0,0.3,0],[0,0,0,0]])

  #-----------------------Inputs------------------------#

     u = np.array([[torque],[delta]])

  #-----------------------Sensors-----------------------#

     y = np.array([[cam_y],[imu_accx],[imu_accy],[imu_theta]])

  #----------------------Observer-----------------------# 
     
     x_hat = np.dot(B,u)*dt + np.dot(A,self.x)*dt + self.x + dt*(np.dot(L,y) - np.dot(L,np.dot(D,u)) - np.dot(L,np.dot(C,self.x)))
     self.x = x_hat 
     self.tlast = t

  #----------------------Publisher----------------------#

     msg = StatesHat()
     msg.header.stamp = rospy.Time.now()
     msg.state_Vx = x_hat[0]
     msg.state_y= x_hat[1]
     msg.state_theta = x_hat[2]
     self.states_pub.publish(msg)


  #######################################################
  #------------Observer B, without vision---------------#
  #######################################################

  def obsB(self, imu_accx, imu_accy, imu_theta, torque, delta, t):

     if self.observer == 2:
        rospy.loginfo('No lane detected: Data based on observer B without vision')
        self.observer = 1
       

     a = 0.3
     b = 0.8
     c1 = 1
     c2 = 1
     v = 1
     dt = (t-self.tlast).to_sec()
 
  #---------------------State-Space---------------------#

     A = np.array([[-c2,0.0,0.0],[0.0,0.0,v],[0.0,0.0,0.0]])
     B = np.array([[c1,0.0],[0.0,0.0],[0.0,v/b]])
     C = np.array([[-c2,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,1.0]])
     D = np.array([[c1,0.0],[0.0,-v**2],[0.0,0.0]])
     L = np.array([[0,0,0],[0,0,0],[0,0,0]])

  #----------------------Inputs-------------------------#

     u = np.array([[torque],[delta]])

  #----------------------Sensors------------------------#

     y = np.array([[imu_accx],[imu_accy],[imu_theta]])

  #----------------------Observer-----------------------#   
     
     x_hat = np.dot(B,u)*dt + np.dot(A,self.x)*dt + self.x + dt*(np.dot(L,y) - np.dot(L,np.dot(D,u)) - np.dot(L,np.dot(C,self.x)))
     self.x = x_hat 
     self.tlast = t

  #----------------------Publisher----------------------#

     msg = StatesHat()
     msg.header.stamp = rospy.Time.now()
     msg.state_Vx = x_hat[0]
     msg.state_y= x_hat[1]
     msg.state_theta = x_hat[2]
     self.states_pub.publish(msg)

  ##########################################################   

if __name__ == '__main__': 
    rospy.init_node('OBSERVER',anonymous=False)
    node = Observer()
    rospy.spin()
