#!/usr/bin/env python

import math
import time
from geometry_msgs.msg import Twist
import rospy


class MotorModelInputs:

  ##########################################################
  #--------------Voltage Inputs Motor model----------------#
  ##########################################################

  def __init__(self):
  
    self.input_pub = rospy.Publisher("motor_volt_in",Twist, queue_size=1)
    self.voltIn = Twist()
    self.sleep_time = 0.02
    self.t_init = rospy.get_rostime()
    self.t_max = 10      #seconds
    self.pwm_min = 40
    self.pwm_max = 100
    self.pwm_incr = (self.pwm_max-self.pwm_min)*(self.sleep_time)/self.t_max
    self.incr_nbr = int((self.pwm_max-self.pwm_min)/self.pwm_incr)
    self.voltIn.linear.x = self.pwm_min
    self.createInputs()


  def createInputs(self):
  
     for i in range(0,self.incr_nbr):
       
       t_duration = rospy.get_rostime() - self.t_init
       t_nsecs = int(t_duration.nsecs)
       self.voltIn.linear.x += self.pwm_incr
       self.voltIn.angular.x = t_nsecs
       self.input_pub.publish(self.voltIn)
       print(self.voltIn.linear.x)
       time.sleep(self.sleep_time)

  ##########################################################

if __name__ == '__main__': 
    rospy.init_node('VOLTAGE_INPUTS',anonymous=False)
    node = MotorModelInputs()
    rospy.spin()
