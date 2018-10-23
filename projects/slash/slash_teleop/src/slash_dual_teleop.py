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
        
        
        self.sub_joy      = rospy.Subscriber("joy", Joy , self.joy_callback , queue_size=1       )
        
        self.pub_cmd      = rospy.Publisher("cmd_vel", Twist , queue_size=1  )
        
        
    #######################################   
    def joy_callback( self, joy_msg ):
        """ """
        
        # init cmd msg
        cmd_msg = Twist()
        
        #If left trigger is active --> CC0 openloop in Volts
        if ((joy_msg.buttons[4] == 1) and (joy_msg.buttons[5] == 0)):
            
            enable = True


            # For logitech joystick
            #mapping button --> cmd
            cmd_msg.linear.x = joy_msg.axes[3]*8.4  #Motor A
            cmd_msg.linear.y = joy_msg.axes[1]*8.4  #Motor B
            cmd_msg.linear.z = 0                    #CtrlChoice
            
            #cmd_msg.angular.x = 0
            #cmd_msg.angular.y = 0
            cmd_msg.angular.z = (joy_msg.axes[2]+joy_msg.axes[0])/2*-1*30*2*3.1416/360  #Servo cmd (mean of both joysticks)
 
        #If left trigger is active --> CC1 openloop in Torque       
        elif ((joy_msg.buttons[5] == 1) and (joy_msg.buttons[4] == 0)):    
            
            enable = True

            # For logitech joystick
            #mapping button --> cmd
            cmd_msg.linear.x = joy_msg.axes[3]*15  #Motor A
            cmd_msg.linear.y = joy_msg.axes[1]*15  #Motor B
            cmd_msg.linear.z = 1                   #CtrlChoice
            
            #cmd_msg.angular.x = 0
            #cmd_msg.angular.y = 0
            cmd_msg.angular.z = (joy_msg.axes[2]+joy_msg.axes[0])/2*-1*30*2*3.1416/360  #Servo cmd (mean of both joysticks)

        elif(joy_msg.buttons[4] == joy_msg.buttons[5]):
            
            enable = False
            
            cmd_msg.linear.x = 0.0 
            cmd_msg.linear.y = 0
            cmd_msg.linear.z = 0            
            
            #cmd_msg.angular.x = 0
            #cmd_msg.angular.y = 0
            cmd_msg.angular.z = 0.0 
                   
        
        # Publish cmd msg
        self.pub_cmd.publish( cmd_msg )
            


#########################################
if __name__ == '__main__':
    
    rospy.init_node('teleop',anonymous=False)
    node = teleop()
    rospy.spin()
