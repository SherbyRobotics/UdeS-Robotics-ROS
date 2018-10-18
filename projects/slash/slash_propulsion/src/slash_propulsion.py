#!/usr/bin/env python
import rospy
import numpy as np
import time
from geometry_msgs.msg import Twist


#########################################
class propulsion(object):
    """
    teleoperation
    """
    def __init__(self):
        
        self.verbose = False
        self.sub_encd      = rospy.Subscriber("encoders_counts", Twist , self.encdRead, queue_size=1)    
        self.pub_encd      = rospy.Publisher("encd_info", Twist , queue_size=1)
        
        # Init params
        self.tp_last = rospy.get_rostime()
        self.tv_last = rospy.get_rostime()
        self.encdA_last = 0
        self.encdB_last = 0
        self.posA_last = 0
        self.posB_last = 0
        self.velA_last = 0
        self.velB_last = 0
        self.accA_last = 0
        self.accB_last = 0
        self.res_encd = 2048
        self.round2rad = 2*3.1416
        
    ####################################### 

    def pos(self, encdA, encdB):

        # Count variation
        posA = encdA/self.res_encd*self.round2rad
        posB = encdB/self.res_encd*self.round2rad
        
        return posA, posB

    ####################################### 

    def vel(self, posA, posB, t_now):
        
        # Time variation calculation
        dTd = t_now - self.tp_last
        dT = int(dTd.nsecs)
        

        # Position variation
        dposA = posA - self.posA_last
        dposB = posB - self.posB_last
        
        # Set the "now" params to "last" param
        self.tp_last = t_now
        self.posA_last = posA
        self.posB_last = posB

        if dT == 0:

          return 0, 0

        else: 
          
          # Calculate velocity
          velA = dposA/dT*1000000000 # dT is in nsecs so we get rad/s
          velB = dposB/dT*1000000000

          return velA, velB
        

    ####################################### 

    def acc(self, velA, velB, t_now):
        
        # Time variation calculation
        dTd = t_now - self.tv_last
        dT = int(dTd.nsecs)

        # Position variation
        dvelA = velA - self.velA_last
        dvelB = velB - self.velB_last
        
        # Set the "now" params to "last" param
        self.tv_last = t_now
        self.velA_last = velA
        self.velB_last = velB

        if dT == 0:
          return 0,0
        else:
          # Calculate velocity
          accA = dvelA/dT*1000000000
          accB = dvelB/dT*1000000000
        
          return accA, accB

    #######################################
  
    def encdRead( self, encd):
        
        # init encd_info msg
        encd_info = Twist()
        
        # Read both encoders and note time
        encdA = encd.linear.x
        encdB = encd.linear.y
        t_now = rospy.get_rostime()

        # Call position, velocity and acceleration calculator function
        posA, posB = self.pos(encdA, encdB)
        velA, velB = self.vel(posA, posB, t_now)
        accA, accB = self.acc(velA, velB, t_now)

        # Msg
        encd_info.linear.x = posA
        encd_info.linear.y = velA
        encd_info.linear.z = accA
        encd_info.angular.x = posB
        encd_info.angular.y = velB
        encd_info.angular.z = accB
                   
        
        # Publish cmd msg
        self.pub_encd.publish( encd_info )
            


#########################################
if __name__ == '__main__':
    
    rospy.init_node('PROPULSION',anonymous=False)
    node = propulsion()
    rospy.spin()
