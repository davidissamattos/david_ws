#!/usr/bin/env python

"""

"""

import roslib
import rospy
import math

import random
from std_msgs.msg import String
from tb_self_experimentation.srv import *
import constants


###################################################
#Class defining the version manager
class ExperimentCoordinator:

	def __init__(self):
		self.rate = rospy.Rate(1)			
		self.mode = constants.crossover_mode
		self.number_experiments = 0

		#Getting some parameters for the experiment coordinator
		if rospy.has_param('hri_distance/number_experiments_to_learn'):		
			self.number_experiments_to_learn = rospy.get_param('hri_distance/number_experiments_to_learn')		
		else:
			self.number_experiments_to_learn = 15
		
		#Wait for version manager services to come up
		rospy.wait_for_service('hri_distance/generate_random_version')
		rospy.wait_for_service('hri_distance/safe_version')
		rospy.wait_for_service('hri_distance/learn_version')
		rospy.wait_for_service('hri_distance/static_version')

		#Setting the subscriber that will define the experiment mode		
		rospy.Subscriber('hri_distance/experiment_mode', String, self.updateMode)
		#Dont let the node die
    		rospy.spin()

	def updateMode(self, msg):
		self.mode = msg.data
		if self.mode == constants.crossover_mode:
			self.CrossoverExperimentMode()
		elif self.mode == constants.ab_mode:
			self.ABExperimentMode()
		elif self.mode == constants.static_mode:
			self.StaticMode()
		elif self.mode == constants.safe_mode:
			self.SafeMode()

	def CrossoverExperimentMode(self):
		if self.number_experiments < self.number_experiments_to_learn:
			#call generate random version
		else:
			#call learned version

	def ABExperimentMode(self):
		#set a defined value

	def StaticMode(self):
		#set a learned value or from the user 

	def SafeMode(self):
		#set the safe mode value

###################################################
#Main execution of the version manager
if __name__=="__main__":
	rospy.init_node('experiment_coordinator')
	ExperimentCoordinator()

