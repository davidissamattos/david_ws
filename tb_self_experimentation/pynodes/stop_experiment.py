#!/usr/bin/env python

"""


"""

import roslib
import rospy
import math

import random
from tb_self_experimentation.srv import *


###################################################
#Class defining the version manager
class StopExperiment:
#

	def __init__(self):

    		rospy.spin()



###################################################
#Main execution of the version manager
if __name__=="__main__":
	rospy.init_node('stop_experiment')
	StopExperiment()

