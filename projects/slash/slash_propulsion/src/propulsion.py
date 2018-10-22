#!/usr/bin/env python
import rospy
import numpy as np
import time
from geometry_msgs.msg import Twist


#########################################
class propulsion(object):

    #######################################
    #-----------Initialization------------#
    ####################################### 
    
    def __init__(self):
        
        self.verbose = False

        # Init subscribers
        self.sub_cmd     = rospy.Subscriber("cmd_vel", Twist, self.cmdRead, queue_size=1)                          
        self.sub_planif  = rospy.Subscriber("planif", Twist, self.planRead, queue_size=1)
        self.sub_encd    = rospy.Subscriber("encoders_counts", Twist , self.encdRead, queue_size=1)
        self.sub_pwm     = rospy.Subscriber("arduino_debug_feedback", Twist, self.pwmRead, queue_size=1)

        # Init publisher    
        self.pub_cmd     = rospy.Publisher("cmd_prop", Twist , queue_size=1)
        self.pub_encd    = rospy.Publisher("encd_info", Twist , queue_size=1)
  
        # Init time
        self.tp_last = rospy.get_time()   #Set first time used for velocity calculation to 0
        self.tv_last = rospy.get_time()   #Set first time used for acceleration calculation to 0

        # Init encoders related params
        self.encdA_last = 0                  
        self.encdB_last = 0
        self.posA_last = 0
        self.posB_last = 0
        self.velA_last = 0
        self.velB_last = 0
        self.accA_last = 0
        self.accB_last = 0

        # Init gains for encoders counts conversion
        self.res_encd = 2048                #Set encoders resolution (pulses per rounds)
        self.round2rad = 2*3.1416           #Set the gain for rounds to rad conversion
        self.nsecs2secs = 1000000000        #Set the gain for nanoseconds to seconds conv.

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

        # Init closedloop related params
        self.wheelrad = rospy.get_param("~wheelrad", 0.05) #Wheel radius in meters
        self.rads2ms  = self.wheelrad
        self.gearR    = rospy.get_param("~gearR", 10) #Gear ratio
        self.KpCA     = rospy.get_param("~KpCA", 5)   #Proportional gain for motor A current control  ****Needs to be tuned and/or add Ki Kd****
        self.KpCB     = rospy.get_param("~KpCB", 5)   #Proportional gain for motor B current control  ****Needs to be tuned and/or add Ki Kd****
        self.KpPA     = rospy.get_param("~KpPA", 5)   #Proportional gain for motor B current control  ****Needs to be tuned and/or add Ki Kd****
        self.KpPB     = rospy.get_param("~KpPB", 5)   #Proportional gain for motor B current control  ****Needs to be tuned and/or add Ki Kd****
        self.KpVA     = rospy.get_param("~KpVA", 5)   #Proportional gain for motor A velocity control  ****Needs to be tuned and/or add Ki Kd****
        self.KpVB     = rospy.get_param("~KpVB", 5)   #Proportional gain for motor B velocity control  ****Needs to be tuned and/or add Ki Kd****

    #######################################
    #------------Read commands------------#
    #######################################

    def cmdRead(self,cmd):

      #Get params function
      self.getparam()
   
      #Read commands
      cmd_Motors = cmd.linear.x
      cmd_Servo  = cmd.angular.z

      #Read the CtrlChoice determined either by the operator or the planif algorithm
      self.CtrlChoice =  rospy.get_param("~CtrlChoice",  0)

      #Call the right function depending on the CtrlChoice
      if (self.CtrlChoice == 0):                #Teleop openloop in Volts
        msg_M, msg_S = self.CC0(cmd_Motors, cmd_Servo)
      elif (self.CtrlChoice == 1):              #Teleop openloop in m/s estimated
        msg_M, msg_S = self.CC1(cmd_Motors, cmd_Servo)

      #Call the msg publisher function
      self.msgPub(msg_M,msg_M,0,0,msg_S) 

    #######################################
    #------Teleop openloop in Volts-------#
    #######################################

    def CC0(self, cmd_M, cmd_S):

      #Convert cmd_vel to motor voltage
      voltM = cmd_M*self.cmd2volts

      #Convert cmd_S to servo angle in rad
      radS  = cmd_S*self.cmd2rad

      return voltM, radS

    #######################################
    #--------Teleop openloop in m/s-------#
    #######################################

    def CC1(self, cmd_M, cmd_S):

      #Convert cmd_vel to motor voltage
      velM = cmd_M*self.cmd2vel

      #Convert cmd_S to servo angle in rad
      radS  = cmd_S*self.cmd2rad

      return velM, radS  

    ##########################################################################################
    #                                                                                        #
    #            *****COMMANDS FROM PLANIF ALGO FOR CTRLCHOICE 2, 3, 4 & 5*****              # 
    #                                                                                        #
    ##########################################################################################

    #######################################
    #------------Read commands------------#
    #######################################  

    def planRead(self,cmd):
   
      #Read commands
      cmd_MotorA = cmd.linear.x
      cmd_MotorB = cmd.linear.y
      cmd_Servo  = cmd.angular.z

      #Read the CtrlChoice determined either by the operator or the planif algorithm
      self.CtrlChoice =  rospy.get_param("~CtrlChoice",  0)

      #Call the right function depending on the CtrlChoice
      if (self.CtrlChoice == 2):                #Planif closedloop in A
        msg_MA,msg_MB,msg_S = self.CC2(cmd_MotorA, cmd_MotorB, cmd_Servo)
      elif (self.CtrlChoice == 3):              #Planif closedloop in m
        msg_MA,msg_MB,msg_S = self.CC3(cmd_MotorA, cmd_MotorB, cmd_Servo)
      elif (self.CtrlChoice == 4):              #Planif closedloop in m/s
        msg_MA,msg_MB,msg_S = self.CC4(cmd_MotorA, cmd_MotorB, cmd_Servo)

      #Call the msg publisher function
      self.msgPub(msg_MA,msg_MB,cmd_MotorA,cmd_MotorB,msg_S)

    #######################################
    #-------Planif closedloop in A--------#
    #######################################

    def CC2(self, cmd_MA, cmd_MB, cmd_S):

      #Set the targeted current from the planif algo
      amp_tA = cmd_MA
      amp_tB = cmd_MB

      #Closedloop       *****We need a current sensor*****
      #ampMA = (ampM_tA - m_ampA)*self.KpCA                ***Where Kp is the proportional gain and m_amp is the measured current***Will probably come from the Arduino
      #ampMB = (ampM_tB - m_ampB)*self.KpCB                ***Where Kp is the proportional gain and m_amp is the measured current***

      #Convert cmd_S to servo angle in rad
      radS  = cmd_S*self.cmd2rad

      return ampM_tA, ampM_tB, radS        #****Change to ampMA and ampMB once we have the need hardware****

    #######################################
    #-------Planif closedloop in m--------#              ***Might be useful to add a speed limitor here***
    #######################################

    def CC3(self, cmd_MA, cmd_MB, cmd_S):

      #Set the targeted position (m) from the planif algo
      pos_tA = cmd_MA
      pos_tB = cmd_MB

      #Calculate the longitudinal velocity according to measured angular velocity
      m_posA = self.posA*self.wheelrad*self.gearR
      m_posB = self.posB*self.wheelrad*self.gearR

      #Closedloop   
      posMA = (pos_tA - m_posA)*self.KpPA      
      posMB = (pos_tB - m_posB)*self.KpPB              

      #Convert cmd_S to servo angle in rad
      radS  = cmd_S*self.cmd2rad

      return posMA, posMB, radS  

    #######################################
    #------Planif closedloop in m/s-------#
    #######################################

    def CC4(self, cmd_MA, cmd_MB, cmd_S):

      #Set the targeted velocity (m/s) from the planif algo
      vel_tA = cmd_MA
      vel_tB = cmd_MB

      #Calculate the longitudinal velocity according to measured angular velocity
      m_velA = self.velA*self.rads2ms*self.gearR
      m_velB = self.velB*self.rads2ms*self.gearR

      #Closedloop   
      velMA = (vel_tA - m_velA)*self.KpVA      
      velMB = (vel_tB - m_velB)*self.KpVB              

      #Convert cmd_S to servo angle in rad
      radS  = cmd_S*self.cmd2rad

      return velMA, velMB, radS  

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
         

    ##########################################################################################
    #                                                                                        #
    #                       *****ENCODERS FEEDBACK FROM ARDUINO*****                         # 
    #                                                                                        #
    ##########################################################################################
        
    #######################################
    #-----------Position (rad)------------#
    ####################################### 

    def pos(self, encdA, encdB):

        # Counts to postion in rad calculation
        posA = encdA/self.res_encd*self.round2rad
        posB = encdB/self.res_encd*self.round2rad
        
        return posA, posB

    #######################################
    #---------Velocity (rad/s)------------#
    #######################################  

    def vel(self, posA, posB, t_now):
        
        # Time variation calculation
        dT = t_now - self.tp_last   

        # Position variation
        dposA = posA - self.posA_last
        dposB = posB - self.posB_last
        
        # Set the "now" params to "last" param
        self.tp_last = t_now
        self.posA_last = posA
        self.posB_last = posB

        if dT == 0:                         #dT might be equal to 0 while initializing
          return 0, 0
        else:       
          # Calculate velocity
          velA = dposA/dT*self.nsecs2secs   #dT is in nsecs so we get rad/s
          velB = dposB/dT*self.nsecs2secs

          return velA, velB
        
    #######################################
    #-------Acceleration (rad/s^2)--------#
    ####################################### 

    def acc(self, velA, velB, t_now):
        
        # Time variation calculation
        dT = t_now - self.tv_last

        # Position variation
        dvelA = velA - self.velA_last
        dvelB = velB - self.velB_last
        
        # Set the "now" params to "last" param
        self.tv_last = t_now
        self.velA_last = velA
        self.velB_last = velB

        if dT == 0:                         #dT might be equal to 0 while initializing
          return 0,0
        else:
          # Calculate acceleration
          accA = dvelA/dT*self.nsecs2secs   #dT is in nsecs so we get rad/s^2
          accB = dvelB/dT*self.nsecs2secs
        
          return accA, accB

    #######################################
    #----------Read Encoders A&B----------#
    ####################################### 

    def encdRead( self, encd):
        
        # init encd_info msg
        encd_info = Twist()
        
        # Read both encoders and note time
        encdA = encd.linear.x
        encdB = encd.linear.y
        t_now = rospy.get_time()

        # Call position (rad), velocity (rad/s) and acceleration (rad/s^2) calculator functions
        self.posA, self.posB = self.pos(encdA, encdB)
        self.velA, self.velB = self.vel(self.posA, self.posB, t_now)
        self.accA, self.accB = self.acc(self.velA, self.velB, t_now)

        # Msg
        encd_info.linear.x = self.posA
        encd_info.linear.y = self.velA
        encd_info.linear.z = self.accA
        encd_info.angular.x = self.posB
        encd_info.angular.y = self.velB
        encd_info.angular.z = self.accB
                   
        
        # Publish cmd msg
        self.pub_encd.publish( encd_info )

#########################################

if __name__ == '__main__':
    
    rospy.init_node('PROPULSION',anonymous=False)
    node = propulsion()
    rospy.spin()
