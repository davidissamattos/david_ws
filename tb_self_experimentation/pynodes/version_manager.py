#!/usr/bin/env python

"""
Version manager. This node is responsible for generating new distances to the approach_and_go.cpp module. This node also include the effector node, as this is just changing the parameter in the ROS parameter server.

As this node just need to run once when we conclude approaching and we are ready to approach again with a new distance this node basically defines ROS services. Different versions generator can be used by creating different services.

Right now we only implemented a simple random version generator for the HRI.

This can be complicated as we want.
Defining a new srv message and a new service (with the feedback for example) we can use other search algorithms to optimize the distance

"""

import roslib
import rospy
import math

import random
from tb_self_experimentation.srv import *


###################################################
#Class defining the version manager
class Manager:
#

	def __init__(self):
		#Defining the services
		
		#Uniform Random distance generator
		# * we use the service version_manager defined in the srv folder
		# * any new generator can define its on service		 
		self.service = rospy.Service('hri_distance/generate_random_version', random_version, self.GenerateRandomDistance)
    	
		#Dont let the node die
    		rospy.spin()

	def GenerateRandomDistance(self, req):
		min_distance = req.min_distance
		max_distance = req.max_distance
		new_distance = 	random.uniform(min_distance,max_distance)
		#Setting the parameter in the parameter server
		#This is actually the effector
		rospy.set_param('hri_distance', new_distance)
		return random_versionResponse(new_distance)

###################################################
#Main execution of the version manager
if __name__=="__main__":
	rospy.init_node('version_manager')
	Manager()

