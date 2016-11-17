#!/usr/bin/env python

"""
The effector. It just listen to new distance variables and change the parameter server

"""

import roslib
import rospy
from std_msgs.msg import Float64


###################################################
#Class defining the version manager
class Effector:
#
	def __init__(self):
		rospy.Subscriber('hri_distance/new_distance', Float64, self.changeDistance
    		rospy.spin()

	def changeDistance(self,msg):
		rospy.set_param('hri_distance/hri_distance', msg.data)

###################################################
#Main execution of the version manager
if __name__=="__main__":
	rospy.init_node('effector')
	StopExperiment()

