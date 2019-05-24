#!/usr/bin/env python

from std_msgs.msg import Float32
import rospy

class PurePursuitLine:

  ##########################################################
  #---------------Controller initialization----------------#
  ##########################################################

	def __init__(self):

		self.image_sub = rospy.Subscriber("ycam_msg",Float32,self.controller)
		self.steering_pub = rospy.Publisher("steer_ang",Float32, queue_size=1)

		self.y_raw_last = 0
		self.y_last     = 0
		self.t_last     = 0


############################################################

	def controller(self,msg_cam):

		y     = msg_cam.data
		delta = 0.001*y
		print(delta)
		msg = Float32()
		msg.data = delta
		self.steering_pub.publish(msg)


#########################################

if __name__ == '__main__':
    
    rospy.init_node('CONTROLLER',anonymous=False)
    node = PurePursuitLine()
    rospy.spin()
		




