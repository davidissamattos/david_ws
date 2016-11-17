#!/usr/bin/env python

"""
Version manager. This node is responsible for generating new distances to the approach_and_go.cpp module. 

As this node just need to run once when we conclude approaching and we are ready to approach again with a new distance this node basically defines ROS services. Different versions generator can be used by creating different services.


"""

import roslib
import rospy
import math
from std_msgs.msg import Float64
import random
from tb_self_experimentation.srv import *


###################################################
#Class defining the version manager
class Manager:
#
	
	def __init__(self):
		self.pub = rospy.Publisher('hri_distance/new_distance', Float64, queue_size=1)
		#Defining the services
		#Uniform Random distance generator
		# * we use the service version_manager defined in the srv folder
		# * any new generator can define its on service		 
		self.serviceRandom = rospy.Service('hri_distance/generate_random_version', set_distance, self.GenerateRandomDistance)
		self.serviceSafe = rospy.Service('hri_distance/safe_version', set_distance, self.GenerateSafeDistance)
		self.serviceLearn = rospy.Service('hri_distance/learn_version', set_distance, self.GenerateLearnedVersion)
		self.serviceStatic = rospy.Service('hri_distance/static_version', set_distance, self.GenerateStaticVersion)
    	
		#Dont let the node die
    		rospy.spin()


	def GenerateStaticVersion(self, req):
		#Get defined static value
		if rospy.has_param('hri_distance/static_min_distance'):		
			new_distance = rospy.get_param('hri_distance/static_min_distance')		
		else:
			new_distance = 2
		self.pub.Publish(new_distance)
		return set_distanceResponse(new_distance)





	def GenerateLearnedVersion(self, req):
		#learn()
		
		if rospy.has_param('hri_distance/learned_min_distance'):		
			min_distance = rospy.get_param('hri_distance/learned_min_distance')		
		else:
			min_distance = 0.5
		if rospy.has_param('hri_distance/learned_max_distance'):		
			max_distance = rospy.get_param('hri_distance/learned_max_distance')		
		else:
			max_distance = 5
		new_distance = 	random.uniform(min_distance,max_distance)
		self.pub.Publish(new_distance)
		return set_distanceResponse(new_distance)




	#safe distance comes from parameters
	def GenerateSafeDistance(self, req):
		if rospy.has_param('hri_distance/safe_distance'):		
			new_distance = rospy.get_param('hri_distance/safe_distance')		
		else:
			new_distance = 2.0
		self.pub.Publish(new_distance)
		return set_distanceResponse(new_distance)



	#We generate a random distance version from limit values
	def GenerateRandomDistance(self, req):
		if rospy.has_param('hri_distance/initial_min_distance'):		
			min_distance = rospy.get_param('hri_distance/initial_min_distance')		
		else:
			min_distance = 0.5
		if rospy.has_param('hri_distance/initial_max_distance'):		
			max_distance = rospy.get_param('hri_distance/initial_max_distance')		
		else:
			max_distance = 5
		new_distance = 	random.uniform(min_distance,max_distance)
		self.pub.Publish(new_distance)
		return set_distanceResponse(new_distance)

###################################################
#Main execution of the version manager
if __name__=="__main__":
	rospy.init_node('version_manager')
	Manager()

