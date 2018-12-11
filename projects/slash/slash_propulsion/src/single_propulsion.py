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
        
        self.sub_planif  = rospy.Subscriber("planif", Twist, self.planRead, queue_size=1)
        self.sub_encd    = rospy.Subscriber("encoders_counts", Twist , self.encdRead, queue_size=1)
        #self.sub_pwm     = rospy.Subscriber("arduino_debug_feedback", Twist, self.pwmRead, queue_size=1)

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
        self.f2,self.f3,self.f4,self.f5,self.f6,self.f7 = 0,0,0,0,0,0

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

        # Gains for CC2 (Closed loop in A)
        self.KpCC2    = rospy.get_param("~KpCC2", 0.25) #Proportional gain for current control  
        self.KiCC2    = rospy.get_param("~KiCC2", 0.25) #Integral gain for current control 

        # Gains for CC3 (Closed loop in m)
        self.KpCC3    = rospy.get_param("~KpCC3", 0.25) #Proportional gain for position control  
        self.KiCC3    = rospy.get_param("~KiCC3", 0.25) #Integral gain for position control 

        # Gains for CC4 (Closed loop in m/s)
        self.KpCC4    = rospy.get_param("~KpCC4", 0.25) #Proportional gain for velocity control  
        self.KiCC4    = rospy.get_param("~KiCC4", 0.25) #Integral gain for velocity control 

        # Gains for CC5 (Closed loop in rad/s)
        self.KpCC5    = rospy.get_param("~KpCC5", 0.15) #Proportional gain for angular velocity control  
        self.KiCC5    = rospy.get_param("~KiCC5", 0.00) #Integral gain for angular velocity control

        # Gains for CC6 (Closed loop in rad)
        self.KpCC6    = rospy.get_param("~KpCC6", 0.25) #Proportional gain for angular position control  
        self.KiCC6    = rospy.get_param("~KiCC6", 0.25) #Integral gain for angular position control 

        # Gains for CC7 (Closed loop in Nm)
        self.KpCC7    = rospy.get_param("~KpCC7", 0.25) #Proportional gain for Torque control  
        self.KiCC7    = rospy.get_param("~KiCC7", 0.25) #Integral gain for Torque control 
 


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
      self.cmd_Servo  = cmd.angular.y  #Command for servo control


    ##########################################################################################
    #                                                                                        #
    #            *****COMMANDS FROM PLANIF ALGO FOR OPENLOOPS CC0 & CC1*****                 # 
    #                                                                                        #
    ##########################################################################################

    def writeCmd(self):

      #For openloop controls, simply pass the info from planif to prop
      if (self.CtrlChoice == 0 or self.CtrlChoice == 1):  #CC0 is openloop in Volts and CC1 is openloop Torque (Nm)               
        cmd_MA     = self.tar_MotorA        
        self.f2,self.f3,self.f4,self.f5,self.f6,self.f7 = 0,0,0,0,0,0 #Init all started closedloop          


    ##########################################################################################
    #                                                                                        #
    #            *****COMMANDS FROM PLANIF ALGO FOR CTRLCHOICE 2, 3, 4 & 5*****              # 
    #                                                                                        #
    ##########################################################################################

    #######################################
    #------------Read commands------------#
    #######################################  

      #Call the right function depending on the CtrlChoice
      elif (self.CtrlChoice == 2):              #Planif closedloop in A
        cmd_MA = self.CC2(self.tar_MotorA)
      elif (self.CtrlChoice == 3):              #Planif closedloop in m
        cmd_MA = self.CC3(self.tar_MotorA)
      elif (self.CtrlChoice == 4):              #Planif closedloop in m/s
        cmd_MA = self.CC4(self.tar_MotorA)
      elif (self.CtrlChoice == 5):              #Planif closedloop in rad/s
        cmd_MA = self.CC5(self.tar_MotorA)
      elif (self.CtrlChoice == 6):              #Planif closedloop in rad
        cmd_MA = self.CC6(self.tar_MotorA)
      elif (self.CtrlChoice == 7):              #Planif closedloop in Torque
        cmd_MA = self.CC7(self.tar_MotorA)

      msg_S = self.cmd_Servo
      
      #Call the msg publisher function
      self.msgPub(cmd_MA,msg_S)

    #######################################
    #-------Planif closedloop in A--------#
    #######################################

    def CC2(self, tar_MA):

      #Closedloop       *****We need a current sensor*****
      #cmd_MA = (t_MA_A - m_ampA)*self.KpCA                ***Where Kp is the proportional gain and m_amp is the measured current***Will probably come from the Arduino

      return 0      #****Change to cmd_MA and cmd_MB once we have the need hardware****

    #######################################
    #-------Planif closedloop in m--------#              ***Might be useful to add a speed limitor here***
    #######################################

    def CC3(self, tar_MA):

      # Init
      if (self.f3 == 0):
        self.tlast_A_CC3 = rospy.get_time()
        self.Ilast_A_CC3,self.elast_A_CC3 = 0,0
        self.f3 = 1 
        self.f2,self.f4,self.f5,self.f6,self.f7 = 0,0,0,0,0 

      # Calculate the longitudinal velocity according to measured angular velocity
      posA = self.posA*self.wheelrad*self.gearR 

      # ClosedLoop      
      cmd_MA,e_A_CC3,t_A_CC3,I_A_CC3  = self.PI(tar_MA,posA,self.KpCC3,self.KiCC3,self.elast_A_CC3, self.tlast_A_CC3,self.Ilast_A_CC3)

      # Re-init
      self.Ilast_A_CC3 = I_A_CC3
      self.tlast_A_CC3 = t_A_CC3
      self.elast_A_CC3 = e_A_CC3         

      return cmd_MA  

    #######################################
    #------Planif closedloop in m/s-------#
    #######################################

    def CC4(self, tar_MA):
 
      # Init
      if (self.f4 == 0):
        self.tlast_A_CC4 = rospy.get_time()
        self.Ilast_A_CC4,self.elast_A_CC4 = 0,0
        self.f4 = 1 
        self.f2,self.f3,self.f5,self.f6,self.f7 = 0,0,0,0,0 
  
      # Calculate the longitudinal velocity according to measured angular velocity
      velA = self.velA*self.rads2ms*self.gearR

      # ClosedLoop      
      cmd_MA,e_A_CC4,t_A_CC4,I_A_CC4  = self.PI(tar_MA,velA,self.KpCC4,self.KiCC4,self.elast_A_CC4, self.tlast_A_CC4,self.Ilast_A_CC4) #Change self.velB  for self.velA eventually

      # Re-init
      self.Ilast_A_CC4 = I_A_CC4
      self.tlast_A_CC4 = t_A_CC4
      self.elast_A_CC4 = e_A_CC4         

      return cmd_MA

    #######################################
    #-----Planif closedloop in rad/s------#
    #######################################

    def CC5(self, tar_MA):

      # Init
      if (self.f5 == 0):
        self.tlast_A_CC5 = rospy.get_time()
        self.Ilast_A_CC5,self.elast_A_CC5 = 0,0
        self.f5 = 1 
        self.f2,self.f3,self.f4,self.f6,self.f7 = 0,0,0,0,0   

      # ClosedLoop      
      cmd_MA,e_A_CC5,t_A_CC5,I_A_CC5  = self.PI(tar_MA,self.velA,self.KpCC5,self.KiCC5,self.elast_A_CC5, self.tlast_A_CC5,self.Ilast_A_CC5) 

      if(cmd_MA<0 and tar_MA>0):
        cmd_MA = 0
      if(cmd_MA>0 and tar_MA<0):
        cmd_MA = 0

      # Re-init
      self.Ilast_A_CC5 = I_A_CC5
      self.tlast_A_CC5 = t_A_CC5
      self.elast_A_CC5 = e_A_CC5          

      return cmd_MA

    #######################################
    #------Planif closedloop in rad-------#              ***Might be useful to add a speed limitor here***
    #######################################

    def CC6(self, tar_MA):

      # Init
      if (self.f6 == 0):
        self.tlast_A_CC6 = rospy.get_time()
        self.Ilast_A_CC6,self.elast_A_CC6 = 0,0
        self.f6 = 1 
        self.f2,self.f3,self.f4,self.f5,self.f7 = 0,0,0,0,0   

      # ClosedLoop      
      cmd_MA,e_A_CC6,t_A_CC6,I_A_CC6  = self.PI(tar_MA,self.posA,self.KpCC6,self.KiCC6,self.elast_A_CC6, self.tlast_A_CC6,self.Ilast_A_CC6) #Change self.velB  for self.velA eventually

      # Re-init
      self.Ilast_A_CC6 = I_A_CC6
      self.tlast_A_CC6 = t_A_CC6
      self.elast_A_CC6 = e_A_CC6         

      return cmd_MA

    #######################################
    #------Planif closedloop in Nm--------#             
    #######################################

    def CC7(self, tar_MA):

      # Init
      if (self.f7 == 0):
        self.tlast_A_CC7 = rospy.get_time()
        self.Ilast_A_CC7,self.elast_A_CC7 = 0,0
        self.f7 = 1 
        self.f2,self.f3,self.f4,self.f5,self.f6 = 0,0,0,0,0   

      # Torque covnersion
      torqueA = self.velA*15     #A modifier lorsque le modele de propulsion sera complet

      # ClosedLoop      
      cmd_MA,e_A_CC7,t_A_CC7,I_A_CC7  = self.PI(tar_MA,torqueA,self.KpCC7,self.KiCC7,self.elast_A_CC7, self.tlast_A_CC7,self.Ilast_A_CC7) #Change self.velB  for self.velA eventually

      # Re-init
      self.Ilast_A_CC7 = I_A_CC7
      self.tlast_A_CC7 = t_A_CC7
      self.elast_A_CC7 = e_A_CC7        

      return cmd_MA

    #######################################
    #------------PI Controller------------#              ***Might be useful to add a speed limitor here***
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
    #                             *****TORQUE ESTIMATION*****                                # 
    #                                                                                        #
    ##########################################################################################

    #def torque_calc(self):   #Might be interesting to compute torque using a current sensor and then use a Kalman Filter to have a better Torque estimate

      # Estimate Torque at each wheel based on current sensors and equation: Torque = i*Kt*gearRatio
      #self.TA_c = self.currentA*self.Kt*self.gearR
      #self.TB_c = self.currentB*self.Kt*self.gearR
      
      # Estimate Torque at each wheel based on equation: Torque = ((Vpwm-Vfcem)/R)*Kt*gearRatio
      #self.TA_model = (self.pwmA*self.Hbridge-self.velA*self.Kfcem)/self.R*self.Kt*self.gearR
      #self.TB_model = (self.pwmB*self.Hbridge-self.velB*self.Kfcem)/self.R*self.Kt*self.gearR
      
      # Not using a kalman filter right now, just a mean
      #self.TA_K = (self.TA_c+self.TA_model)/2
      #self.TB_K = (self.TB_c+self.TB_model)/2

      #If alpha is equal to 0, T-Fr = I*alpha ---> T = Fr ---> T = N*u
      #if (self.accA<0.1 and self.accA>-0.1):                          
        #self.uA = self.TA_K/(self.m*self.g*self.b/(2*(self.a+self.b)))   #Based on car DCL on flat surface and torque obtained from Kalman filter

      #if (self.accB<0.1 and self.accB>-0.1): 
        #self.uB = self.TB_K/(self.m*self.g*self.b/(2*(self.a+self.b)))
           
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
