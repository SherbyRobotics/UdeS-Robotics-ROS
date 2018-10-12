#!/usr/bin/env python

from slash_vision.msg import CamState
from razor_imu_9dof.msg import ImuStates
from slash_localization.msg import StatesHat
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from xlwt import Workbook

import rospy
import time

class Converter:
  ##########################################################

  def __init__(self):

    self.createWorkbook()
    self.createWbHeader()
    self.t_init = rospy.get_rostime()

    self.image_sub = rospy.Subscriber("ycam_msg",CamState,self.cam_receiver)
    self.imu_sub = rospy.Subscriber("imu_msg",ImuStates,self.imu_receiver)
    self.input_sub = rospy.Subscriber("cmd_vel",Twist,self.input_receiver)
    #self.feedback_sub = rospy.Subscriber("arduino_debug_feedback",String,self.feedback_receiver)
    self.estates_sub = rospy.Subscriber("x_hat", StatesHat, self.estates_receiver)

    self.line      = 0
    self.cam_y     = 0
    self.imu_accx  = 0
    self.imu_accy  = 0
    self.imu_theta = 0
    self.torque    = 0
    self.delta     = 0
    self.e_Vx      = 0
    self.e_y       = 0
    self.e_theta   = 0
     
  ##########################################################
  
  def cam_receiver(self,cam):
      self.t_cam = rospy.get_rostime() 
      self.cam_y = cam.state_y
     
  ##########################################################

  def imu_receiver(self,imu):
      self.t_imu     = rospy.get_rostime() 
      self.imu_accx  = imu.state_accx
      self.imu_accy  = imu.state_accy
      self.imu_theta = imu.state_theta
      self.dataLogging() #Log infos at the rate of the fastest sensor. Move that line if needed.   

  ##########################################################
  
  def input_receiver(self, cmd):
      self.t_cmd  = rospy.get_rostime() 
      self.torque = cmd.linear.x
      self.delta  = cmd.angular.z

  ##########################################################
  
#  def feedback_receiver(self, fb):
#      self.t_fb     = rospy.get_rostime()       
#      self.feedback = fb

  ##########################################################
  
  def estates_receiver(self, estates):

      self.t_estates = rospy.get_rostime()       
      self.e_Vx      = estates.state_Vx
      self.e_y       = estates.state_y
      self.e_theta   = estates.state_theta     


  ##########################################################

  def createWorkbook(self):

    self.xlsxName = rospy.get_param("~xlsxName","Data")
    self.wb = Workbook()

  ##########################################################

  def createWbHeader(self):

    self.sheet1 = self.wb.add_sheet('Sheet1')
    self.sheet1.write(0,0,'Time (secs)')
    self.sheet1.write(0,1,'Time (nsecs)')
    self.sheet1.write(0,2,'Data camera (y)')
    self.sheet1.write(0,3,'Data imu (acc_x)')
    self.sheet1.write(0,4,'Data imu (acc_y)')
    self.sheet1.write(0,5,'Data imu (theta)')
    self.sheet1.write(0,6,'Data cmd (torque)')
    self.sheet1.write(0,7,'Data cmd (delta)')
    self.sheet1.write(0,8,'Data feedback')
    self.sheet1.write(0,9,'Estimated states (Vx)')
    self.sheet1.write(0,10,'Estimated states (y)')
    self.sheet1.write(0,11,'Estimated states (theta)')
    for i in range (0,3):
      self.sheet1.col(i).width = 256*14
    for i in range (2,9):
      self.sheet1.col(i).width = 256*17
    for i in range (9,12):
      self.sheet1.col(i).width = 256*20

  ##########################################################
  
  def dataLogging(self):

      self.t_now = rospy.get_rostime()        
      self.line += 1

      self.sheet1.write(self.line,0,self.t_now.secs)
      self.sheet1.write(self.line,1,self.t_now.nsecs)
      self.sheet1.write(self.line,2,self.cam_y)
      self.sheet1.write(self.line,3,self.imu_accx)
      self.sheet1.write(self.line,4,self.imu_accy)
      self.sheet1.write(self.line,5,self.imu_theta)
      self.sheet1.write(self.line,6,self.torque)
      self.sheet1.write(self.line,7,self.delta)
      #sheet1.write(self.line,8,self.feedback)
      self.sheet1.write(self.line,9,self.e_Vx)
      self.sheet1.write(self.line,10,self.e_y)
      self.sheet1.write(self.line,11,self.e_theta)
      
      self.wb.save('/home/nvidia/catkin_ws/src/UdeS-Robotics-ROS/projects/slash/slash_datalogging/xlsxfiles/'+str(self.xlsxName)+'.xlsx')
      


  ##########################################################   

if __name__ == '__main__': 
     rospy.init_node('DATA LOGGER',anonymous=False)
     node = Converter()
     rospy.spin()


































