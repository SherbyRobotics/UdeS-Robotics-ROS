#!/usr/bin/env python
import rospy
import numpy as np
import time
from geometry_msgs.msg import Twist


#########################################
class planif(object):

    #######################################
    #-----------Initialization------------#
    ####################################### 
    
    def __init__(self):
        
        self.verbose = False

        # Init subscribers
        self.sub_cmd     = rospy.Subscriber("cmd_vel", Twist, self.cmdRead, queue_size=1)                          

        # Init publisher    
        self.pub_cmd     = rospy.Publisher("planif", Twist , queue_size=1)
  
        # Init time
        self.t_init = rospy.get_time()   #Set first time used for velocity calculation to 0

    ##########################################################################################
    #                                                                                        #
    #            *****COMMANDS FROM TELEOPERATION FOR CTRLCHOICE 0 AND 1*****                # 
    #                                                                                        #
    ##########################################################################################

    #######################################
    #---Reloads params every iteration----#
    #######################################

    def getparam(self):
        
        # Init CtrlChoice
        self.CtrlChoice =  rospy.get_param("~CtrlChoice",  0)

        # Init teleop related params
        self.cmd2volts = rospy.get_param("~batteryV", 8.4) #cmd_vel range from -1 to 1 and the battery pack is a 8.4V
        self.cmd2vel   = rospy.get_param("~cmd2vel", 15)   #Determined by the propulsion model ***Needs to be tuned for each vehicle***
        self.maxStAng  = rospy.get_param("~strAngle", 30)  #Supposing +/- 30 degrees max for the steering angle
        self.cmd2rad   = self.maxStAng*2*3.1416/360     

    #######################################
    #------------Read commands------------#
    #######################################

    def cmdRead(self,cmd):

      #Get params function
      self.getparam()
   
      #Read commands
      cmd_Motors = cmd.linear.x
      cmd_Servo  = cmd.angular.z

      #Call the right function depending on the CtrlChoice
      if (self.CtrlChoice == 0):                #Teleop openloop in Volts
        msg_M = self.CC0(cmd_Motors)
      elif (self.CtrlChoice == 1):              #Teleop openloop in m/s estimated
        msg_M = self.CC1(cmd_Motors)

      #Convert cmd_S to servo angle in rad
      msg_S = cmd_Servo*self.cmd2rad

      #Call the msg publisher function
      self.msgPub(msg_M,msg_M,0,0,msg_S) 

    #######################################
    #------Teleop openloop in Volts-------#
    #######################################

    def CC0(self, cmd_M):

      #Convert cmd_vel to motor voltage
      voltM = cmd_M*self.cmd2volts

      return voltM

    #######################################
    #--------Teleop openloop in m/s-------#
    #######################################

    def CC1(self, cmd_M):

      #Convert cmd_vel to motor voltage
      velM = cmd_M*self.cmd2vel

      return velM 

    ##########################################################################################
    #                                                                                        #
    #                  *****CMD MESSAGE PUBLISHER TO -----> ARDUINO*****                     # 
    #                                                                                        #
    ##########################################################################################

    def msgPub(self,cmd_MA,cmd_MB,targA,targB,cmd_S):
 
      #Init encd_info msg
      cmd_prop = Twist()
      
      #Msg
      cmd_prop.linear.x  = cmd_MA             #Command sent to motor A
      cmd_prop.linear.y  = cmd_MB             #Command sent to motor B
      cmd_prop.linear.z  = self.CtrlChoice    #Control choice
      cmd_prop.angular.x = targA              #Value targeted for the control choice of motor A
      cmd_prop.angular.y = targB              #Value targeted for the control choice of motor B
      cmd_prop.angular.z = cmd_S              #Command sent to the steering servo

      # Publish cmd msg
      self.pub_cmd.publish(cmd_prop)

#########################################

if __name__ == '__main__':
    
    rospy.init_node('PLANIFICATION',anonymous=False)
    node = planif()
    rospy.spin()
