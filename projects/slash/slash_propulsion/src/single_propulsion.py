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

        # Init  CC
        self.CtrlChoice = 0

        # Init subscribers  
        
        self.sub_cmd     = rospy.Subscriber("cmd_vel", Twist, self.planRead, queue_size=1)
        self.sub_encd    = rospy.Subscriber("encoders_counts", Twist , self.encdRead, queue_size=1)

        # Init publisher    
        self.pub_cmd    = rospy.Publisher("cmd_prop", Twist , queue_size=1)
        self.pub_pos    = rospy.Publisher("pos_info", Twist , queue_size=1)
        self.pub_vel    = rospy.Publisher("vel_info", Twist , queue_size=1)
        self.pub_acc    = rospy.Publisher("acc_info", Twist , queue_size=1)

        # Init pub_cmd params
        self.cmd_MotorA = 0
        self.tar_MotorA = 0
        self.cmd_Servo  = 0   
  
        # Init time
        self.tp_last = rospy.get_time()   #Set first time used for velocity calculation to 0
        self.tv_last = rospy.get_time()   #Set first time used for acceleration calculation to 0

        # Init closedloop control related params
        self.f1,self.f2 = 0,0

        # Init encoders related params
        # Raw data
        self.encdA_last = 0                 
        self.posA_last  = 0
        self.velA_last  = 0
        self.accA_last  = 0
        #Filtered data
        self.velA_Flast = 0
        self.accA_Flast = 0

        # Init gains for encoders counts conversion
        self.res_encd = 512*4               #Set encoders resolution (pulses per rounds)
        self.round2rad = 2*3.1416           #Set the gain for rounds to rad conversion
        self.gearR    = rospy.get_param("~gearR", 15.3) #Gear ratio

    ##########################################################################################
    #                                                                                        #
    #             *****READ COMMANDS AND LOAD PARAMS FOR ALL CTRLCHOICES*****                # 
    #                                                                                        #
    ##########################################################################################

    #######################################
    #---Reloads params every iteration----#
    #######################################

    def getparam(self):   

        # Init closedloop related params
        self.wheelrad = rospy.get_param("~wheelrad", 0.0538) #Wheel radius in meters
        self.rads2ms  = self.wheelrad
        self.gearR    = rospy.get_param("~gearR", 15.3) #Gear ratio

        # Gains for CC1 (Closed loop in rad/s)
        self.KpCC1    = rospy.get_param("~KpCC1", 0.15) #Proportional gain for angular velocity control  
        self.KiCC1    = rospy.get_param("~KiCC1", 0.00) #Integral gain for angular velocity control

        # Gains for CC2 (Closed loop in rad)
        self.KpCC2    = rospy.get_param("~KpCC2", 0.25) #Proportional gain for angular position control  
        self.KiCC2    = rospy.get_param("~KiCC2", 0.25) #Integral gain for angular position control 
 


    #######################################
    #------------Read commands------------#
    #######################################

    def planRead(self,cmd):

      #Get params function
      self.getparam()

      #Chose CtrlChoice
      self.CtrlChoice = cmd.linear.z
   
      #Read commands
      self.tar_MotorA = cmd.linear.x   #Porpulsion command
      self.cmd_Servo  = cmd.angular.z  #Command for servo control


    ##########################################################################################
    #                                                                                        #
    #                *****COMMANDS FROM PLANIF ALGO FOR OPENLOOPS CC0*****                   # 
    #                                                                                        #
    ##########################################################################################

    def writeCmd(self):

      #For openloop controls, simply pass the info from planif to prop
      if (self.CtrlChoice == 0):  #CC0 is openloop in Volts              
        cmd_MA     = self.tar_MotorA        
        self.f1,self.f2 = 0,0     #Init all started closedloop          


    ##########################################################################################
    #                                                                                        #
    #                     *****COMMANDS FOR CLOSED LOOP CONTROL*****                         # 
    #                                                                                        #
    ##########################################################################################

    #######################################
    #------------Read commands------------#
    #######################################  

      #Call the right function depending on the CtrlChoice
      elif (self.CtrlChoice == 1):              #Closedloop in rad/s
        cmd_MA = self.CC1(self.tar_MotorA)
      elif (self.CtrlChoice == 2):              #Closedloop in rad
        cmd_MA = self.CC2(self.tar_MotorA)

      msg_S = self.cmd_Servo
      
      #Call the msg publisher function
      self.msgPub(cmd_MA,msg_S)

    #######################################
    #-----   Closedloop in rad/s    ------#
    #######################################

    def CC1(self, tar_MA):

      # Init
      if (self.f1 == 0):
        self.tlast_A_CC1 = rospy.get_time()
        self.Ilast_A_CC1,self.elast_A_CC1 = 0,0
        self.f1 = 1 
        self.f2 = 0   

      # ClosedLoop      
      cmd_MA,e_A_CC1,t_A_CC1,I_A_CC1  = self.PI(tar_MA,self.velA,self.KpCC1,self.KiCC1,self.elast_A_CC1, self.tlast_A_CC1,self.Ilast_A_CC1) 

      if(cmd_MA<0 and tar_MA>0):
        cmd_MA = 0
      if(cmd_MA>0 and tar_MA<0):
        cmd_MA = 0

      # Re-init
      self.Ilast_A_CC1 = I_A_CC1
      self.tlast_A_CC1 = t_A_CC1
      self.elast_A_CC1 = e_A_CC1          

      return cmd_MA

    #######################################
    #------   Closedloop in rad    -------#    ***Might be useful to add a speed limitor here***           
    #######################################

    def CC2(self, tar_MA):

      # Init
      if (self.f2 == 0):
        self.tlast_A_CC2 = rospy.get_time()
        self.Ilast_A_CC2,self.elast_A_CC2 = 0,0
        self.f2 = 1 
        self.f1 = 0  

      # ClosedLoop      
      cmd_MA,e_A_CC2,t_A_CC2,I_A_CC2  = self.PI(tar_MA,self.posA,self.KpCC2,self.KiCC2,self.elast_A_CC2, self.tlast_A_CC2,self.Ilast_A_CC2) #Change self.velB  for self.velA eventually

      # Re-init
      self.Ilast_A_CC2 = I_A_CC2
      self.tlast_A_CC2 = t_A_CC2
      self.elast_A_CC2 = e_A_CC2         

      return cmd_MA

    #######################################
    #------------PI Controller------------#             
    #######################################

    def PI(self, dsrd, meas, Kp, Ki, e_last, t_last, I_last):

      #Closedloop error: Error = (desired - measured)
      e = dsrd-meas     

      #Proportional command
      P     = e*Kp
     
      #Time
      t_now = rospy.get_time()
     
      #Integral command
      I_now = (((e_last+e)*(t_now-t_last))/2)*Ki    #Find area under curves for each dT
      I_tot = I_now+I_last                          #Sum all areas

      #Sum of both P and I for PI controller
      cmd = P+I_tot 

      return cmd,e,t_now,I_tot
  
    ##########################################################################################
    #                                                                                        #
    #                  *****CMD MESSAGE PUBLISHER TO -----> ARDUINO*****                     # 
    #                                                                                        #
    ##########################################################################################

    def msgPub(self,cmd_MA,cmd_S):
 
      #Init encd_info msg
      cmd_prop = Twist()
      
      #Msg
      cmd_prop.linear.x  = cmd_MA             #Command sent to motor A
      cmd_prop.linear.z  = self.CtrlChoice    #Control choice

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

    def pos(self, encdA):

        # Counts to postion in rad calculation
        posA = encdA/(self.res_encd*self.gearR)*self.round2rad
        
        return posA

    #######################################
    #---------Velocity (rad/s)------------#
    #######################################  

    def vel(self, posA, t_now):
        
        # Time variation calculation
        dT = t_now - self.tp_last  

        # Position variation
        dposA = posA - self.posA_last

        if dT == 0:                         #dT might be equal to 0 while initializing
          return 0, 0
        else:       
          # Calculate velocity
          velA = dposA/dT

          #Low-pass filter
          a = dT/(10*dT)
          velA_F = (1-a)*self.velA_Flast+a*velA

        # Set the "now" params to "last" param
        self.tp_last = t_now
        self.posA_last = posA
        self.velA_Flast = velA_F

        return velA_F
        
    #######################################
    #-------Acceleration (rad/s^2)--------#
    ####################################### 

    def acc(self, velA, t_now):
        
        # Time variation calculation
        dT = t_now - self.tv_last

        # Position variation
        dvelA = velA - self.velA_last
        
        # Set the "now" params to "last" param
        self.tv_last = t_now
        self.velA_last = velA

        if dT == 0:                         #dT might be equal to 0 while initializing
          return 0,0
        else:
          # Calculate acceleration
          accA = dvelA/dT
          #Low-pass filter
          z = dT/(40*dT)
          accA_F = (1-z)*self.accA_Flast+z*accA

        # Set the "now" params to "last" param
        self.tv_last = t_now
        self.velA_last = velA
        self.accA_Flast = accA_F
        
        return accA_F

    #######################################
    #----------Read Encoders A&B----------#
    ####################################### 

    def encdRead( self, encd):
        
        # init encd_info msg
        pos_info = Twist()
        vel_info = Twist()
        acc_info = Twist()
        
        # Read both encoders and note time
        encdA = encd.linear.x
        t_now = rospy.get_time()

        # Call position (rad), velocity (rad/s) and acceleration (rad/s^2) calculator functions
        self.posA = self.pos(encdA)
        self.velA = self.vel(self.posA, t_now)
        self.accA = self.acc(self.velA, t_now)

        # Once the velocity and the acceleration is up to date, compute the estimated Torque and friction coefficient
        #self.torque_calc()

        self.writeCmd()

        # Msg
        pos_info.linear.x = self.posA
        vel_info.linear.x = self.velA
        acc_info.linear.x = self.accA
        
        # Publish cmd msg
        self.pub_pos.publish( pos_info )
        self.pub_vel.publish( vel_info )
        self.pub_acc.publish( acc_info )

#########################################

if __name__ == '__main__':
    
    rospy.init_node('PROPULSION',anonymous=False)
    node = propulsion()
    rospy.spin()
