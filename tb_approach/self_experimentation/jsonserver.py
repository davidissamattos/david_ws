#!/usr/bin/env python

"""
Simple server to send JSON messages
"""

import time
import roslib
import rospy
from std_msgs.msg import String
# Importing only what we'll use
from tempfile import TemporaryFile

import web

urls = (
    '/', 'jsonserver'
)


class rosData:
	def __init__(self):
		rospy.init_node('jsonserver')	
		#initalizing ros stuff
		print "Subscribing"
		self.test = "init"
		rospy.Subscriber('test', String, self.testCallback)
		self.tempFile = TemporaryFile()


	def testCallback(self,msg):
		print "Saving callbacks"
		self.tempFile.seek(0)
		self.tempFile.write(msg.data)

class jsonserver:
	def __init__(self):
		print "starting jsonserver class"

	def GET(self):
		return test
		#return "Hello"

if __name__ == '__main__':
	rosData()
	app = web.application(urls, globals())	
	app.run()
