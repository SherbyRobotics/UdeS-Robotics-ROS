#!/usr/bin/env python
import rospy
import numpy as np
from time import sleep
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
        
    #######################################
    #---Converts joysticks info to msg----#
    #######################################

    def readJoy(self,cmd_MA,cmd_ser):

        self.cmd_msg.linear.x = cmd_MA  
        self.cmd_msg.linear.y = cmd_MA 
        
        self.cmd_msg.angular.y = 0                 
        self.cmd_msg.angular.z = cmd_ser

    #######################################
    #---Converts joysticks info to msg----#
    #######################################

    def readJoyTank(self,cmd_MA,cmd_MB,cmd_SA, cmd_SB):

        self.cmd_msg.linear.x = cmd_MA  
        self.cmd_msg.linear.y = cmd_MB 

        self.cmd_msg.angular.y = cmd_SA            
        self.cmd_msg.angular.z = cmd_SB

    ####################################### 
        
    def joy_callback( self, joy_msg ):
        """ """
        
    #######################################
    #------------Read joysticks-----------#
    #######################################
        self.cmd_msg = Twist()
     
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
            self.readJoy(cmd_MA,ser_DB)

    #######################################
    #--------CC1 - Openloop in m/s--------#
    #######################################
 
        #If right button is active       
        elif (joy_msg.buttons[5] == 1):    
            
            enable = True
            self.cmd_msg.linear.z = 1   #CtrlChoice
            self.readJoy(cmd_MA,ser_DB)

    #######################################
    #--------CC2 - Closedloop in A--------#
    #######################################
          
        #If button A is active 
        elif(joy_msg.buttons[1] == 1):   
            
            self.cmd_msg.linear.x = 0

            self.cmd_msg.linear.y = 0
            self.cmd_msg.linear.z = 0            
            
            #cmd_msg.angular.x = 0
            self.cmd_msg.angular.y = 0
            self.cmd_msg.angular.z = 1 

    #######################################
    #--------CC3 - Closedloop in m--------#
    #######################################

        #If button B is active
        elif(joy_msg.buttons[2] == 1):   
            
            self.cmd_msg.linear.x = 0.38

            self.cmd_msg.linear.y = 0
            self.cmd_msg.linear.z = 0            
            
            #cmd_msg.angular.x = 0
            self.cmd_msg.angular.y = 0
            self.cmd_msg.angular.z = ser_DB 

    #######################################
    #------CC4 - Closedloop in m/s--------#
    #######################################

        #If button Y is active
        elif(joy_msg.buttons[3] == 1):   
            
            self.cmd_msg.linear.x = 0.27

            self.cmd_msg.linear.y = 0
            self.cmd_msg.linear.z = 0            
            
            #cmd_msg.angular.x = 0
            self.cmd_msg.angular.y = 0
            self.cmd_msg.angular.z = ser_DB 

    #######################################
    #------CC5 - Closedloop in rad/s------#
    #######################################

        #If button Y is active
        elif(joy_msg.buttons[0] == 1):   
            
            enable = True
            self.cmd_msg.linear.z = 5    #CtrlChoice
            self.readJoy(cmd_MA,ser_DB)

    #######################################
    #-------CC6 - Closedloop in rad-------#
    #######################################

        #If bottom arrow is active
        elif(joy_msg.axes[5] < 0):   
            
            enable = True
            self.cmd_msg.linear.z = 6    #CtrlChoice
            self.readJoy(cmd_MA,ser_DB)

    #######################################
    #-----CC7 - Closedloop in Torque------#
    #######################################

        #If right arrow is active
        elif(joy_msg.axes[4] < 0):   
            
            enable = True
            self.cmd_msg.linear.z = 7    #CtrlChoice
            self.readJoy(cmd_MA,ser_DB)

    #######################################
    #-----Tank Drive for dual prop--------#
    #######################################

        #If left button is active 
        if (joy_msg.buttons[6] == 1):
            
            enable = True
            self.cmd_msg.linear.z = 8   #CtrlChoice
            self.readJoyTank(cmd_MA,cmd_MB,ser_DA,ser_DB)

    #######################################
    #---Reinit when buttons are inactive--#
    #######################################

        elif((joy_msg.buttons[4] == 0) and (joy_msg.buttons[5] == 0) and (joy_msg.buttons[1] == 0) and (joy_msg.buttons[2] == 0) and (joy_msg.buttons[3] == 0) and (joy_msg.buttons[0] == 0) and (joy_msg.buttons[6] == 0)):
            
            enable = False
            
            self.cmd_msg.linear.x = 0

            self.cmd_msg.linear.y = 0
            self.cmd_msg.linear.z = 0            
            
            #cmd_msg.angular.x = 0
            self.cmd_msg.angular.y = 0
            self.cmd_msg.angular.z = rospy.get_param('~cmd_volts', 0)

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
