#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


#########################################
class teleop(object):
    """
    teleoperation
    """
    def __init__(self):
        
        self.verbose = False
        
        self.sub_joy      = rospy.Subscriber("joy", Joy , self.joy_callback , queue_size=1)
        self.pub_cmd      = rospy.Publisher("cmd_vel", Twist , queue_size=1  ) 

        self.max_vel  = 40   #Max velocity set at 40 rad/s (2.16 m/s)(7.78 km/h)
        self.max_volt = 6    #Max voltage is set at 6 volts   
        self.maxStAng  = 40  #Supposing +/- 40 degrees max for the steering angle
        self.cmd2rad   = self.maxStAng*2*3.1416/360     
        
    #######################################
    #---Converts joysticks info to msg----#
    #######################################

    def pubCmd(self,cmd_MA,cmd_ser):
        
        self.cmd_msg.linear.x = cmd_MA  
        self.cmd_msg.linear.y = cmd_MA 
        
        self.cmd_msg.angular.z = -cmd_ser*self.cmd2rad

    ####################################### 

    def pubCmdTank(self,cmd_MA,cmd_MB,cmd_ser):
        
        self.cmd_msg.linear.x = cmd_MA  
        self.cmd_msg.linear.y = cmd_MB 
        
        self.cmd_msg.angular.z = -cmd_ser*self.cmd2rad

    ####################################### 
        
    def joy_callback( self, joy_msg ):
        """ 
	Note: (REFER TO THE READ ME FILE ~/UdeS-Robotics-ROS/projects/slash/slash_teleop/READ_ME)

	*********************      All control choices except from Tank Drive      ***************************
	-Right joystick controls throttle either in openloop Voltage or closedloop velocity
	-Left joystick controls steering angle

	*********************                Tank Drive control                    ***************************
	-Right joystick controls the throttle of the right motor in openloop Voltage or closedloop velocity.
	-Left joystick controlsthe throttle of the left motor in openloop Voltage or closedloop velocity.
	-The mean of both joystick x-axis values controls the steering angle
	"""
        self.cmd_msg = Twist()
    #######################################
    #------------Read joysticks-----------#
    #######################################
    
        cmd_MA = joy_msg.axes[3]    #Motor A     (Right joystick) 
        cmd_MB = joy_msg.axes[1]    #Motor B     (Left joystick)
        ser_DA = joy_msg.axes[2]    #Direction A (Right joystick)
        ser_DB = joy_msg.axes[0]    #Direction B (Left joystick)

    #######################################
    #------CC0 - Openloop in Volts--------#
    #######################################

        #If left button is active 
        if (joy_msg.buttons[4] == 1):
            
            enable = True
            self.cmd_msg.linear.z = 0   #CtrlChoice
            volt = cmd_MA*self.max_volt
            self.pubCmd(volt,ser_DB)

    #######################################
    #----CC1 - Closedloop in rad/s--------#
    #######################################
 
        #If right button is active       
        elif (joy_msg.buttons[5] == 1):    
            
            enable = True
            self.cmd_msg.linear.z = 1   #CtrlChoice
            vel = cmd_MA*self.max_vel          #Max velocity is set to 40 rad/s
            self.pubCmd(vel,ser_DB)

    #######################################
    #--CC1 - Closedloop set at 20 rad/s---#
    #######################################
          
        #If button A is active 
        elif(joy_msg.buttons[1] == 1):   
            
            enable = True
            self.cmd_msg.linear.z = 1   #CtrlChoice
            self.pubCmd(20,ser_DB)

    #######################################
    #--CC1 - Closedloop set at 25 rad/s---#
    #######################################

        #If button B is active
        elif(joy_msg.buttons[2] == 1):   
            
            enable = True
            self.cmd_msg.linear.z = 1   #CtrlChoice
            self.pubCmd(25,ser_DB)

    #######################################
    #--CC1 - Closedloop set at 30 rad/s---#
    #######################################

        #If button Y is active
        elif(joy_msg.buttons[3] == 1):   
            
            enable = True
            self.cmd_msg.linear.z = 1    #CtrlChoice
            self.pubCmd(30,ser_DB)

    #######################################
    #--CC1 - Closedloop set at 35 rad/s---#
    #######################################

        #If button X is active
        elif(joy_msg.buttons[0] == 1):   
            
            enable = True
            self.cmd_msg.linear.z = 1    #CtrlChoice
            self.pubCmd(35,ser_DB)

    #######################################
    #--CC2 - Closedloop distance 100rad---#
    #######################################

        #If bottom arrow is active
        elif(joy_msg.axes[5] == 1):   
            
            enable = True
            self.cmd_msg.linear.z = 2    #CtrlChoice
            self.pubCmd(100,ser_DB)

    #######################################
    #---CC3  - Tank Drive for dual prop---#
    #######################################

        #If left trigger is active 
        if (joy_msg.buttons[6] == 1):
            
            enable = True
            self.cmd_msg.linear.z = 3   #CtrlChoice
            cmd_ser = (ser_DA+ser_DB)/2
            self.pubCmdTank(cmd_MA,cmd_MB,cmd_ser)



    #######################################
    #---Reinit when buttons are inactive--#
    #######################################

        elif((joy_msg.buttons[4] == 0) and (joy_msg.buttons[5] == 0) and (joy_msg.buttons[1] == 0) and (joy_msg.buttons[2] == 0) and (joy_msg.buttons[3] == 0) and (joy_msg.buttons[0] == 0) and (joy_msg.buttons[6] == 0)):
            
            enable = False
            
            self.cmd_msg.linear.x = 0 
            self.cmd_msg.linear.y = 0
            self.cmd_msg.linear.z = 0            
            
            self.cmd_msg.angular.y = 0
            self.cmd_msg.angular.z = 0 

    #######################################
    #------------Publish msg--------------#
    #######################################                   
        
        # Publish cmd msg
        self.pub_cmd.publish( self.cmd_msg )
            

#########################################
if __name__ == '__main__':
    
    rospy.init_node('teleop',anonymous=False)
    node = teleop()
    rospy.spin()
