#!/usr/bin/env python


from geometry_msgs.msg import Twist
from xlwt import Workbook

import rospy
import time

class Converter:
  ##########################################################

  def __init__(self):

    self.createWorkbook()
    self.createWbHeaderEncd()
    self.t_init = rospy.get_time()
    self.line      = 0
    self.posA      = 0
    self.posB      = 0
    self.velA      = 0
    self.velB      = 0
    self.accA      = 0
    self.accB      = 0
    self.cmdA      = 0
    self.mod_x     = 0
    self.mod_y     = 0
    self.mod_theta = 0
    self.mod_vel   = 0

    self.encd_sub  = rospy.Subscriber("vel_info",Twist, self.encd_receiver, queue_size=1)
    self.pos_sub   = rospy.Subscriber("pos_info",Twist, self.pos_receiver, queue_size=1)
    self.ardu_sub  = rospy.Subscriber("arduino_debug_feedback",Twist, self.ard_receiver, queue_size=1)
    self.model_sub = rospy.Subscriber("model_vel",Twist, self.mod_receiver, queue_size=1)


  ##########################################################

  def ard_receiver(self,data):

      self.cmdA = data.linear.x

  ##########################################################

  def pos_receiver(self,data):

      self.posA = data.linear.x
  ##########################################################

  def mod_receiver(self,data):

      self.mod_x = data.linear.x
      self.mod_y = data.linear.y
      self.mod_theta = data.linear.z
      self.mod_vel   = data.angular.x

  ##########################################################

  def encd_receiver(self,data):

      self.velA = data.linear.x


      self.dataLogging()
 
  ##########################################################

  def createWorkbook(self):

    self.xlsxName = rospy.get_param("~xlsxName","Data")
    self.wb = Workbook()

  ##########################################################

  def createWbHeaderEncd(self):

    self.sheet1 = self.wb.add_sheet('Sheet1')
    self.sheet1.write(0,0,'Real Time (secs)')
    self.sheet1.write(0,1,'Velocity A (rad/s)')
    self.sheet1.write(0,2,'Model x')
    self.sheet1.write(0,3,'Model y')
    self.sheet1.write(0,4,'Model theta')
    self.sheet1.write(0,5,'Model velocity')
    self.sheet1.write(0,6,'Cmd')

    for i in range (0,12):
      self.sheet1.col(i).width = 256*20


  ##########################################################
  
  def dataLogging(self):

      self.t_now = rospy.get_time() 
      self.t_s   = self.t_now - self.t_init       
      self.line += 1

      self.sheet1.write(self.line,0,self.t_s)
      self.sheet1.write(self.line,1,self.velA)
      self.sheet1.write(self.line,2,self.mod_x)
      self.sheet1.write(self.line,3,self.mod_y)
      self.sheet1.write(self.line,4,self.mod_theta)
      self.sheet1.write(self.line,5,self.mod_vel)
      self.sheet1.write(self.line,6,self.cmdA)



      
      self.wb.save('/home/ubuntu/catkin_ws/src/UdeS-Robotics-ROS/projects/slash/slash_datalogging/xlsxfiles/'+str(self.xlsxName)+'.xlsx')
      


  ##########################################################   

if __name__ == '__main__': 
     rospy.init_node('DATA LOGGER',anonymous=False)
     node = Converter()
     rospy.spin()


































