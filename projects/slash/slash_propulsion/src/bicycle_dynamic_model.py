#!/usr/bin/env python
import rospy
import numpy as np
import time
from geometry_msgs.msg import Twist

#########################################
class propulsion_model(object):

    #######################################
    #-----------Initialization------------#
    ####################################### 
    
	def __init__(self):

			# Model param
			self.t_last = 0
			self.omega_last = 0
			self.R   = 0.63
			self.k   = 0.002791
			self.r   = 0.0538
			self.gR  = 15.3
			self.J   = 0.020881
			self.V   = 0
			self.b   = 0
			self.L   = 0.33
			self.cmd = 0
			self.ser = 0 
			self.t_init = rospy.get_time()
			self.x_last     = 0
			self.y_last     = 0
			self.theta_last = 0
        
			# Init subscribers  
			self.sub_ard    = rospy.Subscriber("arduino_debug_feedback", Twist, self.cmdread, queue_size=1)
			self.sub_vel    = rospy.Subscriber("vel_info", Twist , self.callback, queue_size=1)
			self.sub_cmd    = rospy.Subscriber("cmd_prop", Twist , self.cmdProp, queue_size=1)

        	# Init publisher    
			self.pub_model  = rospy.Publisher("model_vel", Twist , queue_size=1)
		

	#######################################
	def cmdProp(self,data):

			self.ser = data.angular.z

	#######################################
	def cmdread(self,data):

			self.cmd = data.linear.x

			if (self.cmd >= 23 and self.cmd <=53):
				self.V  = 0.00036993*self.cmd**3-0.068285*self.cmd**2+4.3291*self.cmd-96.469
				self.b  = 0.00009239*self.V+0.0023315
			elif (self.cmd >= 54 and self.cmd <=69):
				self.V  = 0.00036993*self.cmd**3-0.068285*self.cmd**2+4.3291*self.cmd-96.469
				self.b  = 0.0013411*self.V**3+0.011976*self.V**2+0.036401*self.V+0.040378
			elif (self.cmd >= 70 and self.cmd<=110):
				self.V  = 0
				self.b  = 0.01036
			elif (self.cmd >= 111 and self.cmd <= 126):
				self.V  = 0.000028424*self.cmd**3-0.012945569*self.cmd**2+2.025529854*self.cmd-102.8750848
				self.b  = -0.0013411*self.V**3+0.011976*self.V**2-0.036401*self.V+0.040378
			elif (self.cmd >= 127 and self.cmd <= 158):
				self.V  = 0.000028424*self.cmd**3-0.012945569*self.cmd**2+2.025529854*self.cmd-102.8750848
				self.b  = -0.00009239*self.V+0.0023315


	#######################################
	def callback(self,data):
			
		#Propulsion Model omega(Vin)
			t = rospy.get_time()-self.t_init
			omega = float((self.V*(t-self.t_last)-((self.R*self.b+self.k**2*self.gR**2)/(self.k*self.gR))*self.omega_last*(t-self.t_last))/((self.R*self.J)/(self.k*self.gR)) + self.omega_last)


		#State estimation
			x      = omega*self.r*np.cos(self.theta_last)*(t-self.t_last)+self.x_last
			y      = omega*self.r*np.sin(self.theta_last)*(t-self.t_last)+self.y_last
			theta  = (omega*self.r*np.tan(self.ser))/self.L*(t-self.t_last)+self.theta_last

		#Re-initialization
			self.t_last     = t
			self.omega_last = omega
			self.x_last     = x
			self.y_last     = y
			self.theta_last = theta

      		#Init model msg
			msg = Twist()
      
      		#Msg
			msg.linear.x   = x  	 #Model States
			msg.linear.y   = y
			msg.linear.z   = theta
			msg.angular.x  = omega #Model velocity

      		# Publish cmd msg
			self.pub_model.publish(msg)

#########################################

if __name__ == '__main__':
    
	 rospy.init_node('MODEL',anonymous=False)
	 node = propulsion_model()
	 rospy.spin()

