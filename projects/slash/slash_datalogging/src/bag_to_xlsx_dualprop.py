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
    self.t_init = rospy.get_rostime()
    self.line      = 0
    self.posA      = 0
    self.posB      = 0
    self.velA      = 0
    self.velB      = 0
    self.accA      = 0
    self.accB      = 0
    self.encd_sub = rospy.Subscriber("encd_info",Twist, self.encd_receiver, queue_size=1)
    self.encd_sub = rospy.Subscriber("cmd_prop",Twist, self.cmd_receiver, queue_size=1)

  ##########################################################

  def cmd_receiver(self,data):
      self.t_cmd = rospy.get_rostime()
      self.cmdA = data.linear.x
      self.cmdB = data.linear.y
      self.CC   = data.linear.z
      self.tarA = data.angular.x
      self.tarB = data.angular.y
      self.serv = data.angular.z 

  ##########################################################

  def encd_receiver(self,data):
      self.t_encd = rospy.get_rostime()
      self.posA = data.linear.x
      self.velA = data.linear.y
      self.accA = data.linear.z
      self.posB = data.angular.x
      self.velB = data.angular.y
      self.accB = data.angular.z

      self.dataLogging()
 
  ##########################################################

  def createWorkbook(self):

    self.xlsxName = rospy.get_param("~xlsxName","Data")
    self.wb = Workbook()

  ##########################################################

  def createWbHeaderEncd(self):

    self.sheet1 = self.wb.add_sheet('Sheet1')
    self.sheet1.write(0,0,'Time (secs)')
    self.sheet1.write(0,1,'Time (nsecs)')
    self.sheet1.write(0,2,'Position A (rad)')
    self.sheet1.write(0,3,'Velocity A (rad/s)')
    self.sheet1.write(0,4,'Acceleration A (rad/s^2')
    self.sheet1.write(0,5,'Position B (rad)')
    self.sheet1.write(0,6,'Velocity B (rad/s)')
    self.sheet1.write(0,7,'Acceleration B (rad/s^2')
    self.sheet1.write(0,8,'Cmd A')
    self.sheet1.write(0,9,'Tar A')
    self.sheet1.write(0,10,'Cmd B')
    self.sheet1.write(0,11,'Tar B')

    for i in range (0,12):
      self.sheet1.col(i).width = 256*20


  ##########################################################
  
  def dataLogging(self):

      self.t_now = rospy.get_rostime()        
      self.line += 1

      self.sheet1.write(self.line,0,self.t_now.secs)
      self.sheet1.write(self.line,1,self.t_now.nsecs)
      self.sheet1.write(self.line,2,self.posA)
      self.sheet1.write(self.line,3,self.velA)
      self.sheet1.write(self.line,4,self.accA)
      self.sheet1.write(self.line,5,self.posB)
      self.sheet1.write(self.line,6,self.velB)
      self.sheet1.write(self.line,7,self.accB)
      self.sheet1.write(self.line,8,self.cmdA)
      self.sheet1.write(self.line,9,self.tarA)
      self.sheet1.write(self.line,10,self.cmdB)
      self.sheet1.write(self.line,11,self.tarB)


      
      self.wb.save('/home/nvidia/catkin_ws/src/UdeS-Robotics-ROS/projects/slash/slash_datalogging/xlsxfiles/'+str(self.xlsxName)+'.xlsx')
      


  ##########################################################   

if __name__ == '__main__': 
     rospy.init_node('DATA LOGGER',anonymous=False)
     node = Converter()
     rospy.spin()


































