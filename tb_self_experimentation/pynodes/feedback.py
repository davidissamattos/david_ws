#!/usr/bin/env python

"""
This feedback.py node is responsible for receiving the robot data that we want to self-experiment, process it and bring to the metric analysis node.

"""

import roslib
import rospy
import math

from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from geometry.msg import Pose2D

##################################### Class definition

############################################################################################
class Feedback:
	def __init__(self):
		#Class variables
		self.tb_position = Pose2D()
		self.apriltag_position = Pose2D()
		self.voice_output_msg = Float64()
		self.step_output_msg = Float64()
		
		#Waiting for services
		rospy.wait_for_service('hri_distance/analyze_feedback')

		#Defining the publishers - Just for debug or latter
		self.pub_voice = rospy.Publisher('hri_distance/feedback/voice', Float64)
		self.pub_step = rospy.Publisher('hri_distance/feedback/step', Float64)

		#Subscribing to the data processed by the turtlebot
		rospy.Subscriber('recognizer/output', String, self.speechCallback) #Speech recognizer
		rospy.Subscriber('apriltag/global_position', Pose2D, self.apriltagPositionCallback) #Apriltag global position from apriltag_tf_listener
		rospy.Subscriber('tb/global_position', Pose2D, self.tbPositionCallback) #Tb global position from tb_tf_listener
		rospy.Subscriber('hri_distance/robot/conclude_approach', Bool, self.concludeApproachcallback)

		#Dont let the node die
    		rospy.spin()


	def calculateDistance(self, pointA, pointB):
		distance = math.sqrt(math.pow(pointA.x-pointB.x,2)+math.pow(pointA.y-pointB.y,2))		
		return distance
	
	#This  function process if the person has taken a step back or not in respect to the turtle bot
	# If it takes a step back or forward them it is a negative feedback. Value 0
	# If it stays still them it is a positive feedback. Value 1
	def stepFeedback(self):
		distance = self.calculateDistance(self.tb_position,self.apriltag_position)
			self.hri_distance = rospy.get_param('hri_distance')
			if distance > self.hri_distance + 0.4:
				self.step_output_msg = 0
			elif distance < self.hri_distance - 0.4:
				self.step_output_msg = 0
			else:
				self.step_output_msg = 1
			


######################### Callbacks
	#save tb last position	
	def tbPositionCallback(self,msg):
		self.tb_position = msg
	
	#save apriltag last position
	def apriltagPositionCallback(self,msg):
		self.apriltag_position = msg

	#This callback function process the recognized voice strings into numbers
	#Value 0 means negative feedback
	#Value 1 means positive feedback
	def speechCallback(self, msg):
		if msg.data.find("good") > -1:
			self.voice_output_msg.data = 1   
		if msg.data.find("close") > -1:
			self.voice_output_msg.data = 0
		if msg.data.find("far") > -1:
			self.voice_output_msg.data = 0           
	
	
	# If we are done with the approaching we can send this data to the analyze_feedback service to process it
	#We are also publishing the feedback data into some topics so we can debug
	def concludeApproachCallback(self, msg):
		if msg.data == True: 
			self.stepFeedback()
			try:
				analyze_feedback_service = rospy.ServiceProxy('hri_distance/analyze_feedback', analyze_feedback)
				response = analyze_feedback_service(self.voice_output_msg , self.step_output_msg , self.hri_distance)
				print "Value %f" %response.value
			except rospy.ServiceException, e:
				print "Service call failed: %s"%e
			#reseting the value of the voice output in case we dont give feedback next time			
			self.voice_output_msg.data = 1	
			self.pub_step.publish(self.step_output_msg)
			self.pub_voice.publish(self.voice_output_msg)
					

################################# Main		
if __name__=="__main__":
    rospy.init_node('feedback')
    Feedback()

