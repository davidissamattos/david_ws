#!/usr/bin/env python

"""

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
			self.logfile = rospy.get_param('hri_logfile')
		else:
			self.logfile = 'hri.csv'
		
		#Defining the services
		#This service will be responsible for when we get data from the feedback.py we can calculate the value, save the data and learn
		self.analyze_service = rospy.Service('hri_distance/analyze_feedback', analyze_feedback, self.analyzeFeedbackService)
		self.learn_service = rospy.Service('hri_distance/learn', learn_service, self.learnService)
		#Dont let the node die
    		rospy.spin()

################################################### Services callbacks	
	def analyzeFeedbackService(self, req):
		self.voice_feedback = req.voice_feedback
		self.step_feedback = req.step_feedback
		self.hri_distance = req.hri_distance
		self.value = self.calculateValue(self.voice_feedback, self.step_feedback)
		print self.value
		#changing the face of the robot		
		self.changeFace()
		#saving the data
		self.appendData_CSVFile()
		return analyze_feedbackResponse(self.value)

	def learnService(self, req):
		response = learn_serviceResponse()
		#response.max_distance =
		#response.min_distance =
		return response



################################################### Auxiliar functions
	#Possible values for voice_feedback and step_feedback = (-1,0,1)
	#1 -> positive feedback
	#0 -> no feedback -> we consider a good feedback
	#-1 -> we consider a negative feedback
	# table =  0.8voice + 0.2step
	#Consider the following table of responses
	#Step\value| -1   | 0        |1
	#-1        |-1    | -0.2     |0.6
	#0         |-0.8  | 0(->0.5) |0.8
	#1         |-0.6  | 0.2      |1
	#If we get a negative value we output -1 else we output 1
	def calculateValue(self, voice_feedback, step_feedback):
		value_interm = 0.8*(voice_feedback) + 0.2*(step_feedback)
		if value_interm == 0.0:
			value_iterm = 0.5
		if value_interm > 0:
			value = 1
		else:
			value = -1
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


## Change the face of the robot using the robot_feedback service
##not a important function but it is good to give feedback
	def changeFace(self):
		try:
			change_face = rospy.ServiceProxy('hri_distance/robot/face_feedback', robot_feedback_service)
			if self.value == 1.0:
				response = change_face("happy")
			if self.value == 0.0:
				response = change_face("flat")
			if self.value == -1.0:
				response = change_face("sad")
			print response
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

###################################################
#Main execution of the version manager
if __name__=="__main__":
	rospy.init_node('metric_analysis')
	MetricAnalysis()

