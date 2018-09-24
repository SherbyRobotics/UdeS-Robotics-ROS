#!/usr/bin/env python

import math
import time
from slash_vision.msg import CamState
from razor_imu_9dof.msg import ImuStates
from slash_localization.msg import StatesHat
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt


import numpy as np
import scipy.linalg
import rospy
import sys


class Controller:

  ##########################################################
  #---------------Controller initialization----------------#
  ##########################################################

  def __init__(self):

    self.image_sub = rospy.Subscriber("ycam_msg",CamState,self.callback1)
    self.imu_sub = rospy.Subscriber("imu_msg",ImuStates,self.callback2)
  
    self.input_pub = rospy.Publisher("cmd_vel",Twist, queue_size=1)
    self.estates_pub = rospy.Publisher("x_hat", StatesHat, queue_size=1)

    self.tlast = rospy.get_rostime()
    self.x = np.array([[0],[0],[0]])
    self.observer = 1 #Guess that a lane will be detected on the first loop, only has an impact on the LOGINFO

  #----------------Input initialization--------------------#

    self.tgain = -10
    self.dgain = 15*3.1416/180
    self.torque = 0
    self.delta = 0

  #-----------------States initialization------------------#

    self.i_Vx     = 0
    self.i_y      = 0
    self.i_theta  = 0
    self.i_states = np.array([[self.i_Vx],[self.i_y],[self.i_theta]])

  #------------------Targeted states ----------------------# ***LETS TRY IT WITH CONSTANT VALUES FOR NOW***

    self.t_Vx     = 1
    self.t_y      = 0
    self.t_theta  = 0
    self.t_states = np.array([[self.t_Vx],[self.t_y],[self.t_theta]])


  ##########################################################
  #----------------------Cam reader------------------------#
  ##########################################################
  
  def callback1(self,cam):
      self.cam_y     = cam.state_y


  ##########################################################
  #----------------------IMU reader------------------------#
  ##########################################################
  
  def callback2(self,imu):
      self.imu_accx     = imu.state_accx
      self.imu_accy     = imu.state_accy
      self.imu_theta = imu.state_theta
      self.t = rospy.get_rostime()

  #-------------------Calls the observer-------------------#

      if self.cam_y == 1000: #Random value given to cam_y if no lane is detected
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

  #---------------------Params-setup--------------------# 

     a = 0.3
     b = 0.8
     c1 = 1
     c2 = 1
     v = 1
     dt = (t-self.tlast).to_sec()
 
  #---------------------State-Space---------------------#

     A  = np.array([[-c2,0,0],[0,0,v],[0,0,0]])
     B  = np.array([[c1,0,0],[0,a*v/b,0],[0,v/b,0]])
     C  = np.array([[0,1,0],[-c2,0,0],[0,0,0],[0,0,1]])
     D  = np.array([[0,0,0],[c1,0,0],[0,(v**2)/b,a*v/b],[0,0,0]])
     Q  = np.array([[100,0,0],[0,10,0],[0,0,10]])
     R  = np.array([[10,0,0],[0,1000,0],[0,0,1000]])
     Vd = 0.00001*np.array([[1,0,0],[0,1,0],[0,0,1]])      # Matrix of disturbance covariances on the plant for each state [Vx,y,theta]
     Vn = np.array([[0.001,0,0,0],[0,0.00001,0,0],[0,0,0.000001,0],[0,0,0,0.000001]]) # Matrix of sensor noise covariances
     
  #--------------------Kalman-Filters-------------------#
  
     [obsK, obsX, obsEigs] = self.lqr((np.transpose(A)),(np.transpose(C)),Vd,Vn)
     obsK = np.transpose(obsK)
     [ctrK, ctrX, ctrEigs] = self.lqr(A,B,Q,R)

  #-----------------------Inputs------------------------#

     u = np.array([[torque],[delta],[0]])

  #-----------------------Sensors-----------------------#

     y = np.array([[cam_y],[imu_accx],[imu_accy],[imu_theta]])

  #----------------------Observer-----------------------# 
     
     x_hat = np.dot(B,u)*dt + np.dot(A,self.x)*dt + self.x + dt*(np.dot(obsK,y) - np.dot(obsK,np.dot(D,u)) - np.dot(obsK,np.dot(C,self.x)))
     self.x = x_hat 
     self.tlast = t

  #---------------States error estimator----------------#
    
     states_error = x_hat-self.t_states

  #---------States error to inputs conversion-----------#

     torque = float(np.dot(-ctrK,states_error)[0])
     delta  = float(np.dot(-ctrK,states_error)[1])

  #----------------------Publisher----------------------#

     estates = StatesHat()
     estates.header.stamp = rospy.Time.now()
     estates.state_Vx = x_hat[0]
     estates.state_y= x_hat[1]
     estates.state_theta = x_hat[2]
     self.estates_pub.publish(estates)

     inputs = Twist() # Might need to create a custom msg to have a header
     #inputs.header.stamp = rospy.Time.now()
     inputs.linear.x = torque/self.tgain
     inputs.angular.z = delta/self.dgain
     self.input_pub.publish(inputs)


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
  #--------------Linear-quatratic-regulator----------------#
  ##########################################################  
 
  def lqr(self,A,B,Q,R):
    
    X = np.matrix(scipy.linalg.solve_continuous_are(A, B, Q, R))
    K = np.matrix(scipy.linalg.inv(R)*(B.T*X)) 
    eigVals, eigVecs = scipy.linalg.eig(A-B*K)
 
    return K, X, eigVals

  ##########################################################

if __name__ == '__main__': 
    rospy.init_node('CONTROLLER',anonymous=False)
    node = Controller()
    rospy.spin()
