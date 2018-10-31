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
        self.sub_teleop  = rospy.Subscriber("cmd_vel", Twist, self.planRead, queue_size=1)                      
        self.sub_planif  = rospy.Subscriber("planif", Twist, self.planRead, queue_size=1)
        self.sub_encd    = rospy.Subscriber("encoders_counts", Twist , self.encdRead, queue_size=1)
        #self.sub_pwm     = rospy.Subscriber("arduino_debug_feedback", Twist, self.pwmRead, queue_size=1)

        # Init publisher    
        self.pub_cmd     = rospy.Publisher("cmd_prop", Twist , queue_size=1)
        self.pub_encd    = rospy.Publisher("encd_info", Twist , queue_size=1)
  
        # Init time
        self.tp_last = rospy.get_time()   #Set first time used for velocity calculation to 0
        self.tv_last = rospy.get_time()   #Set first time used for acceleration calculation to 0
        self.tCC5_last = rospy.get_time()

        # Init encoders related params
        self.encdA_last = 0                  
        self.encdB_last = 0
        self.posA_last  = 0
        self.posB_last  = 0
        self.velA_last  = 0
        self.velB_last  = 0
        self.velA_Flast = 0
        self.velB_Flast = 0
        self.accA_last  = 0
        self.accB_last  = 0
        self.accA_Flast = 0
        self.accB_Flast = 0

        # Init gains for encoders counts conversion
        self.res_encd = 2048                #Set encoders resolution (pulses per rounds)
        self.round2rad = 2*3.1416           #Set the gain for rounds to rad conversion
        self.nsecs2secs = 1000000000        #Set the gain for nanoseconds to seconds conv.

    ##########################################################################################
    #                                                                                        #
    #             *****READ COMMANDS AND LOAD PARAMS FOR ALL CTRLCHOICES*****                # 
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

        # Init closedloop related params
        self.wheelrad = rospy.get_param("~wheelrad", 0.05) #Wheel radius in meters
        self.rads2ms  = self.wheelrad
        self.gearR    = rospy.get_param("~gearR", 10) #Gear ratio
        self.KpCA     = rospy.get_param("~KpCA", 5)   #Proportional gain for motor A current control  ****Needs to be tuned and/or add Ki Kd****
        self.KpCB     = rospy.get_param("~KpCB", 5)   #Proportional gain for motor B current control  ****Needs to be tuned and/or add Ki Kd****
        self.KpPA     = rospy.get_param("~KpPA", 0.3)   #Proportional gain for motor B current control  ****Needs to be tuned and/or add Ki Kd****
        self.KpPB     = rospy.get_param("~KpPB", 5)   #Proportional gain for motor B current control  ****Needs to be tuned and/or add Ki Kd****
        self.KpVA     = rospy.get_param("~KpVA", 0.25)   #Proportional gain for motor A velocity control  ****Needs to be tuned and/or add Ki Kd****
        self.KiVA     = rospy.get_param("~KpVA", 0.25)   #Proportional gain for motor A velocity control  ****Needs to be tuned and/or add Ki Kd****
        self.KpVB     = rospy.get_param("~KpVB", 0.5)   #Proportional gain for motor B velocity control  ****Needs to be tuned and/or add Ki Kd****

        # Init closed loop CC5
        self.tCC5_last   = rospy.get_time()
        self.I_last      = 0
        self.errorA_last = self.cmd_MotorA

    #######################################
    #------------Read commands------------#
    #######################################

    def planRead(self,cmd):
   
      #Read commands
      self.cmd_MotorA = cmd.linear.x
      self.cmd_MotorB = cmd.linear.y
      self.cmd_Servo  = cmd.angular.z

      #Get params function
      self.getparam()

      #Chose CtrlChoice
      self.CtrlChoice = cmd.linear.z

    ##########################################################################################
    #                                                                                        #
    #            *****COMMANDS FROM PLANIF ALGO FOR CTRLCHOICE 2, 3, 4 & 5*****              # 
    #                                                                                        #
    ##########################################################################################

    def writeCmd(self):

      #For openloop controls, simply pass the info from planif to prop
      if (self.CtrlChoice == 0 or self.CtrlChoice == 1 or self.CtrlChoice == 7):                
        msg_MA     = self.cmd_MotorA     
        msg_MB     = self.cmd_MotorB
        cmd_MotorA = 0          #No targeted value, only write the teleop info
        cmd_MotorB = 0         
        msg_S      = self.cmd_Servo

    ##########################################################################################
    #                                                                                        #
    #            *****COMMANDS FROM PLANIF ALGO FOR CTRLCHOICE 2, 3, 4 & 5*****              # 
    #                                                                                        #
    ##########################################################################################

    #######################################
    #------------Read commands------------#
    #######################################  

      #Call the right function depending on the CtrlChoice
      elif (self.CtrlChoice == 2):                #Planif closedloop in A
        msg_MA,msg_MB,msg_S = self.CC2(self.cmd_MotorA, self.cmd_MotorB, self.cmd_servo)
      elif (self.CtrlChoice == 3):              #Planif closedloop in m
        msg_MA,msg_MB,msg_S = self.CC3(self.cmd_MotorA, self.cmd_MotorB, self.cmd_Servo)
      elif (self.CtrlChoice == 4):              #Planif closedloop in m/s
        msg_MA,msg_MB,msg_S = self.CC4(self.cmd_MotorA, self.cmd_MotorB, self.cmd_Servo)
      elif (self.CtrlChoice == 5):              #Planif closedloop in rad/s
        msg_MA,msg_MB,msg_S = self.CC5(self.cmd_MotorA, self.cmd_MotorB, self.cmd_Servo)
      elif (self.CtrlChoice == 6):              #Planif closedloop in rad
        msg_MA,msg_MB,msg_S = self.CC6(self.cmd_MotorA, self.cmd_MotorB, self.cmd_Servo)
      
      #Call the msg publisher function
      self.msgPub(msg_MA,msg_MB,self.cmd_MotorA,self.cmd_MotorB,msg_S)

    #######################################
    #-------Planif closedloop in A--------#
    #######################################

    def CC2(self, cmd_MA, cmd_MB, cmd_Servo):

      #Set the targeted current from the planif algo
      amp_tA = cmd_MA
      amp_tB = cmd_MB

      #Closedloop       *****We need a current sensor*****
      #ampMA = (ampM_tA - m_ampA)*self.KpCA                ***Where Kp is the proportional gain and m_amp is the measured current***Will probably come from the Arduino
      #ampMB = (ampM_tB - m_ampB)*self.KpCB                ***Where Kp is the proportional gain and m_amp is the measured current***

      #Convert cmd_S to servo angle in rad
      radS = cmd_Servo*self.cmd2rad

      return ampM_tA, ampM_tB, radS       #****Change to ampMA and ampMB once we have the need hardware****

    #######################################
    #-------Planif closedloop in m--------#              ***Might be useful to add a speed limitor here***
    #######################################

    def CC3(self, cmd_MA, cmd_MB, cmd_Servo):

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
      radS = cmd_Servo*self.cmd2rad            

      return posMA, posMB, radS  

    #######################################
    #------Planif closedloop in m/s-------#
    #######################################

    def CC4(self, cmd_MA, cmd_MB, cmd_Servo):

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
      radS = cmd_Servo*self.cmd2rad         

      return velMA, velMB, radS 

    #######################################
    #-----Planif closedloop in rad/s------#
    #######################################

    def CC5(self, cmd_MA, cmd_MB, cmd_Servo):

      # Time
      t_now = rospy.get_time()

      #Set the targeted velocity (rad/s) from the planif algo
      rad_tA = cmd_MA
      rad_tB = cmd_MB

      #Closedloop error
      if ((rad_tA-self.velB)>0):   
        errorA = rad_tA - self.velB     
      else:
        errorA = 0

      #Proportional command
      P     = errorA*self.KpVA 
     

      I_now = ((self.errorA_last*self.KiVA+errorA*self.KiVA)*(t_now-self.tCC5_last))/2
      I_tot = I_now+self.I_last
      velMA = P+I_tot
   
      velMB = (rad_tB - self.velB)*self.KpVB 

      #Convert cmd_S to servo angle in rad
      radS = cmd_Servo*self.cmd2rad

      # Re-init
      self.I_last      = I_tot
      self.tCC5_last   = t_now
      self.errorA_last = errorA         

      return velMA, velMB, radS 

    #######################################
    #------Planif closedloop in rad-------#              ***Might be useful to add a speed limitor here***
    #######################################

    def CC6(self, cmd_MA, cmd_MB, cmd_Servo):

      #Set the targeted position (rad) from the planif algo
      pos_tA = cmd_MA
      pos_tB = cmd_MB

      #Closedloop   
      posMA = (pos_tA - self.posB)*self.KpPA      
      posMB = (pos_tB - self.posB)*self.KpPB  

      #Convert cmd_S to servo angle in rad
      radS = cmd_Servo*self.cmd2rad            

      return posMA, posMB, radS  

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

        if dT == 0:                         #dT might be equal to 0 while initializing
          return 0, 0
        else:       
          # Calculate velocity
          velA = dposA/dT
          velB = dposB/dT

          #Low-pass filter
          a = dT/(40*dT)
          velA_F = (1-a)*self.velA_Flast+a*velA
          velB_F = (1-a)*self.velB_Flast+a*velB

        # Set the "now" params to "last" param
        self.tp_last = t_now
        self.posA_last = posA
        self.posB_last = posB
        self.velA_Flast = velA_F
        self.velB_Flast = velB_F

        return velA_F, velB_F
        
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
          accA = dvelA/dT
          accB = dvelB/dT
          #Low-pass filter
          z = dT/(40*dT)
          accA_F = (1-z)*self.accA_Flast+z*accA
          accB_F = (1-z)*self.accB_Flast+z*accB

        # Set the "now" params to "last" param
        self.tv_last = t_now
        self.velA_last = velA
        self.velB_last = velB
        self.accA_Flast = accA_F
        self.accB_Flast = accB_F
        
        return accA_F, accB_F

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

        # Once the velocity and the acceleration is up to date, compute the estimated Torque and friction coefficient
        #self.torque_calc()

        self.writeCmd()

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
