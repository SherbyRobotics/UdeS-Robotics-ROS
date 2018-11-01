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
        self.pub_planif  = rospy.Publisher("planif", Twist , queue_size=1)
  
        # Init time
        self.t_init = rospy.get_time()   #Set first time used for velocity calculation to 0

    ##########################################################################################
    #                                                                                        #
    #                     *****PLANIFICATION BASED ON TELEOPERATION*****                     # 
    #                                                                                        #
    ##########################################################################################

    #######################################
    #---Reloads params every iteration----#
    #######################################

    def getparam(self):

        # Init teleop related params
        self.cmd2volts = rospy.get_param("~batteryV", 8.4) #cmd_vel range from -1 to 1 and the battery pack is a 8.4V
        self.cmd2vel   = rospy.get_param("~cmd2vel", 15)   #Determined by the propulsion model ***Needs to be tuned for each vehicle***
        self.maxStAng  = rospy.get_param("~strAngle", 30)  #Supposing +/- 30 degrees max for the steering angle
        self.cmd2rad   = self.maxStAng*2*3.1416/360 

        # Init targeted values for closedloop control    
        self.t_MA_A = rospy.get_param("~t_MA_A", 20) #Targeted current (A) for MotorA
        self.t_MA_m = rospy.get_param("~t_MA_m", 2)  #Targeted position (m) for MotorA
        self.t_MA_v = rospy.get_param("~t_MA_v", 1)  #Targeted velocity (m/s) for MotorA
        self.t_MA_w = rospy.get_param("~t_MA_w", 1)  #Targeted angular velocity (rad/s) for MotorA
        self.t_MA_r = rospy.get_param("~t_MA_r", 1)  #Targeted angular position (rad) for MotorA
        self.t_MA_T = rospy.get_param("~t_MA_r", 15) #Targeted Torque (Nm) for MotorA
 

    #######################################
    #------------Read commands------------#
    #######################################

    def cmdRead(self,cmd):
   
      #Read commands
      self.cmd_MotorA = cmd.linear.x
      self.cmd_ServoA = cmd.angular.y

      #Get params function
      self.getparam()

      #Chose CtrlChoice
      self.CtrlChoice = cmd.linear.z

      #Call the right function depending on the CtrlChoice

    #######################################
    #-------CC0 - OpenLoop in Volts-------#
    #######################################
  
      if (self.CtrlChoice == 0):                #Left button to choose CC0
        msg_MA = self.CC0(self.cmd_MotorA)      #Right joystick controls MotorA
        t_MA   = 0                              #No targeted values since we are in an openloop

        #Convert cmd_S to servo angle in rad
        msg_S = -self.cmd_ServoA*self.cmd2rad  #Means both joysticks for servo command

    #######################################
    #--------CC1 - OpenLoop in m/s--------#
    #######################################

      elif (self.CtrlChoice == 1):              #Right button to choose CC1
        msg_MA = self.CC1(self.cmd_MotorA)      #Right joystick controls MotorA
        t_MA   = 0                              #No targeted values since we are in an openloop

        #Convert cmd_S to servo angle in rad
        msg_S = -self.cmd_ServoA*self.cmd2rad  #Means both joysticks for servo command

    #######################################
    #--------CC2 - Closedloop in A--------#
    #######################################

      elif (self.CtrlChoice == 2):
        msg_MA = 0                   #The commands will be determined by the closedloop control in the propulsion algorithm
        t_MA   = self.t_MA_A         #The targeted values are set either by the config file or by the closedloop with the observer                             
        msg_S  = 0                   #The servo values are set either by the config file or by the closedloop with the observer

    #######################################
    #--------CC3 - Closedloop in m--------#
    #######################################

      elif (self.CtrlChoice == 3):
        msg_MA = 0                   #The commands will be determined by the closedloop control in the propulsion algorithm
        t_MA   = self.t_MA_m         #The targeted values are set either by the config file or by the closedloop with the observer                             
        msg_S  = 0                   #The servo values are set either by the config file or by the closedloop with the observer

    #######################################
    #------CC4 - Closedloop in m/s--------#
    #######################################

      elif (self.CtrlChoice == 4):
        msg_MA = 0                   #The commands will be determined by the closedloop control in the propulsion algorithm
        t_MA   = self.t_MA_v         #The targeted values are set either by the config file or by the closedloop with the observer                               
        msg_S  = 0                   #The servo values are set either by the config file or by the closedloop with the observer

    #######################################
    #------CC5 - Closedloop in rad/s------#
    #######################################

      elif (self.CtrlChoice == 5):
        msg_MA = 0                   #The commands will be determined by the closedloop control in the propulsion algorithm
        t_MA   = self.t_MA_w         #The targeted values are set either by the config file or by the closedloop with the observer                             
        msg_S  = 0                   #The servo values are set either by the config file or by the closedloop with the observer

    #######################################
    #-------CC6 - Closedloop in rad-------#
    #######################################

      elif (self.CtrlChoice == 6):
        msg_MA = 0                   #The commands will be determined by the closedloop control in the propulsion algorithm
        t_MA   = self.t_MA_r         #The targeted values are set either by the config file or by the closedloop with the observer                             
        msg_S  = 0                   #The servo values are set either by the config file or by the closedloop with the observer

    #######################################
    #-----CC7 - Closedloop in Torque------#
    #######################################

      elif (self.CtrlChoice == 7):
        msg_MA = 0                   #The commands will be determined by the closedloop control in the propulsion algorithm
        t_MA   = self.t_MA_T         #The targeted values are set either by the config file or by the closedloop with the observer                             
        msg_S  = 0                   #The servo values are set either by the config file or by the closedloop with the observer


      #Call the msg publisher function
      self.msgPub(msg_MA,t_MA,msg_S) 

    #######################################
    #------Teleop openloop in Volts-------#
    #######################################

    def CC0(self, cmd_M):

      #Convert cmd_vel to motor voltage
      voltM = cmd_M*self.cmd2volts

      return voltM

    #######################################
    #--------Teleop openloop in Nm--------#
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

    def msgPub(self,cmd_MA,targA,cmd_S):
 
      #Init encd_info msg
      cmd_prop = Twist()
      
      #Msg
      cmd_prop.linear.x  = cmd_MA             #Command sent to motor A
      cmd_prop.linear.z  = self.CtrlChoice    #Control choice
      cmd_prop.angular.x = targA              #Value targeted for the control choice of motor A
      cmd_prop.angular.z = cmd_S              #Command sent to the steering servo

      # Publish cmd msg
      self.pub_planif.publish(cmd_prop)

#########################################

if __name__ == '__main__':
    
    rospy.init_node('PLANIFICATION',anonymous=False)
    node = planif()
    rospy.spin()
