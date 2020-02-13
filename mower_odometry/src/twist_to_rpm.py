#!/usr/bin/env python
"""
   twist_to_motors - converts a twist message to motor commands.  Needed for navigation stack
   
   
    Copyright (C) 2012 Jon Stephan. 
     
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""

import rospy
import roslib
from std_msgs.msg import Int8
from geometry_msgs.msg import Twist 

#############################################################
#############################################################
class TwistToMotors():
#############################################################
#############################################################

    #############################################################
    def __init__(self):
    #############################################################
        rospy.init_node("twist_to_motors")
    
        self.w = 0.36
        self.r = 0.125
    
        self.pub_lmotor = rospy.Publisher('L_RPM', Int8, queue_size=10)
        self.pub_rmotor = rospy.Publisher('R_RPM', Int8, queue_size=10)
        rospy.Subscriber('twist', Twist, self.twistCallback)
    
        self.dx = 0
        self.dr = 0
        self.dy = 0
        self.rate = 10
        self.left = 0
        self.right = 0
        self.lrpm = 0
        self.rrpm = 0

    #############################################################
    def spin(self):
    #############################################################
    
        r = rospy.Rate(self.rate)
        
        ###### main loop  ######
        while not rospy.is_shutdown():
            self.spinOnce()
            r.sleep()

            
                
    #############################################################
    def spinOnce(self):
    #############################################################
    
        # dx = (l + r) / 2
        # dr = (r - l) / w
            
        # self.right = (2.0 * self.dx + self.dr * self.w) / (2 * self.r) 
        # self.left = (2.0 * self.dx - self.dr * self.w) / (2 * self.r)
        self.right = 1.0 * self.dx + self.dr * self.w / 2 
        self.left = 1.0 * self.dx - self.dr * self.w / 2
        # rospy.loginfo("publishing: (%d, %d)", left, right) 

        # self.lrpm = self.left /  0.10472
        # self.rrpm = self.right / 0.10472
        self.rrpm = self.right / (self.r * 0.10472) 
        self.lrpm = self.left / (self.r * 0.10472) 
        
        self.pub_lmotor.publish(self.lrpm)
        self.pub_rmotor.publish(self.rrpm)
            

    #############################################################
    def twistCallback(self,msg):
    #############################################################
        # rospy.loginfo("-D- twistCallback: %s" % str(msg))
        self.dx = msg.linear.x
        self.dr = msg.angular.z
        self.dy = msg.linear.y
    
#############################################################
#############################################################
if __name__ == '__main__':
    """ main """
    try:
        twistToMotors = TwistToMotors()
        twistToMotors.spin()
    except rospy.ROSInterruptException:
        pass
