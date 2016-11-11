#!/usr/bin/env python

"""
1- This module receives input from the monitor
2- Compute the value function
3- Save the input data and the value function in a log file


"""

import roslib
import rospy
import math
import sys
import csv
from std_msgs.msg import Float64
from tb_self_experimentation.srv import *

##################################################
#Class
class MetricAnalysis:

	def __init__(self):
		#Listing class variables
		#self.voice_feedback
		#self.step_feedback
		#self.value
		#self.hri_distance
		#self.csvfile
		#self.csvdata
		#self.logfile

		#Initializing and getting parameters
		if rospy.has_param('hri_logfile'):		
			rospy.get_param('hri_logfile', self.logfile)
		else:
			self.logfile = 'hri.csv'
		
		#Defining the services
		#This service will be responsible for when we get data from the feedback.py we can calculate the value, save the data and learn
		self.service = rospy.Service('hri_distance/analyze_feedback', analyze_feedback, self.analyzeFeedback)


		#Dont let the node die
    		rospy.spin()
	
	def analyzeFeedback(self, req):
		self.voice_feedback = req.voice_feedback
		self.step_feedback = req.step_feedback
		self.hri_distance = req.hri_distance
		self.value = self.calculateValue(self.voice_feedback, self.step_feedback)
		self.appendData_CSVFile()
		return analyze_feedbackResponse(self.value)


	def calculateValue(self, voice_feedback, step_feedback):
		value = 0.8*(voice_feedback) + 0.2*(step_feedback)
		return value		
	
	def appendData_CSVFile(self):
		csvfile = open(self.logfile, 'a+')
		writer = csv.writer(csvfile)
		writer.writerows([[self.hri_distance,self.value]])#We can write more than one row
		csvfile.close()		
		
	def readData_CSVFile(self):
		csvfile = open(self.logfile)
		reader = list(csv.reader(csvfile))
    		for column in reader:
        		print(column[1])
		csvfile.close()	

###################################################
#Main execution of the version manager
if __name__=="__main__":
	rospy.init_node('metric_analysis')
	MetricAnalysis()

