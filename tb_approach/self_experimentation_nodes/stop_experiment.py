#!/usr/bin/env python

"""
Stop experiment module

"""

import roslib
import rospy
import math
import constants
import random
from tb_approach.srv import *
from tb_approach.msg import *
from std_msgs.msg import String


###################################################
#Class defining the version manager
class StopExperiment:
#

	def __init__(self):
		self.UpdateParameters()
		self.pub = rospy.Publisher('hri_distance/experiment_mode', String, queue_size=1)
		rospy.Subscriber('hri_distance/feedback/stats', data_stats,self.StatsCallback)
    		rospy.spin()



	def StatsCallback(self,msg):
		self.number_experiments = msg.numberExperiments
		print "number of experiments: " + str(self.number_experiments)
		print "number of experiments to learn: " + str(self.number_experiments_to_learn)
		self.positiveFeedbackPercentage = msg.positiveFeedbackPercentage
		print "% of positive: " + str(self.positiveFeedbackPercentage)
		#updating parameter
		self.UpdateParameters()
		
		#Experiment policy
		#Crossover experiment		
		if (self.number_experiments < self.number_experiments_to_learn):
			print "Crossover mode -> full random"
			self.pub.publish(constants.crossover_mode)
		if (self.number_experiments > self.number_experiments_to_learn) and ((self.positiveFeedbackPercentage < self.positive_threshold) or (self.positiveFeedbackPercentage > self.negative_threshold)):
			print "Crossover mode -> learn at each iteration"
			self.pub.publish(constants.crossover_mode)
		#Static mode		
		if (self.number_experiments > self.number_experiments_to_learn) and self.positiveFeedbackPercentage > self.positive_threshold:
			print "Static mode -> with learned version"
			self.pub.publish(constants.static_mode)
		#Safe mode		
		if (self.number_experiments > self.number_experiments_to_learn) and self.positiveFeedbackPercentage < self.negative_threshold:
			print "Safe mode"
			self.pub.publish(constants.safe_mode)
		#AB experiment
		#Still to implement





	def UpdateParameters(self):
		if rospy.has_param('hri_distance/number_experiments_to_learn'):		
			self.number_experiments_to_learn = rospy.get_param('hri_distance/number_experiments_to_learn')
		else:
			self.number_experiments_to_learn = 15
		if rospy.has_param('hri_distance/positive_threshold'):		
			self.positive_threshold = rospy.get_param('hri_distance/positive_threshold')
		else:
			self.positive_threshold = 0.8
		if rospy.has_param('hri_distance/negative_threshold'):		
			self.negative_threshold = rospy.get_param('hri_distance/negative_threshold')
		else:
			self.negative_threshold = 0.3

###################################################
#Main execution of the version manager
if __name__=="__main__":
	rospy.init_node('stop_experiment')
	StopExperiment()

