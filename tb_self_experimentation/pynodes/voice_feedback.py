#!/usr/bin/env python

"""
simple voice command feedback for the HRI

"""

import roslib
import rospy
import math

from std_msgs.msg import String
from std_msgs.msg import Float64

class voice_feedback:

    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        self.output_msg = Float64()

        # publish to cmd_vel, subscribe to speech output
        self.pub = rospy.Publisher('hri_distance/voice_feedback', Float64)
        rospy.Subscriber('recognizer/output', String, self.speechCallback)

        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            self.pub.publish(self.output_msg)
            rate.sleep()
        
    def speechCallback(self, msg):
        rospy.loginfo(msg.data)

        if msg.data.find("good") > -1:
		self.output_msg = 0
            
        if msg.data.find("close") > -1:
		self.output_msg = -1

        if msg.data.find("far") > -1:
		self.output_msg = 1           
        
        self.pub_.publish(self.msg)

if __name__=="__main__":
    rospy.init_node('voice_feedback')
    try:
        voice_feedback()
    except:
        pass

