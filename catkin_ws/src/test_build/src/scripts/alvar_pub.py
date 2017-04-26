#!/usr/bin/env python

# define imports
import rospy
import roslaunch
import sys
import time
import os
from std_msgs.msg import String
from ar_track_alvar_msgs.msg import AlvarMarkers

# class to read joystick messages and launch node
class alvar_detect(object):
    # define self routine
    def __init__(self):
   	# define subscriber
    	self.alvar_sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.alvar_callback)
    	#define Publisher for publishing to /action_input topic
    	self.alvar_pub = rospy.Publisher("action_input", String, queue_size=1)
        self.alvar_num_pub = rospy.Publisher("alvar_num", String, queue_size=1)
   	rospy.loginfo('started the publishing routine to action_input')
    	rate = rospy.Rate(5)
    	rospy.loginfo('started alvar routine...')
    	# configure node roslaunch api
    	package    = 'test_build'
    	######Change Executeable here #######
    	executable = 'prowling.py'
    	#####################################
    	node = roslaunch.core.Node(package, executable)
        self.one = False
        self.two = False
        self.three = False

        while not rospy.is_shutdown():
            if self.one:
                self.alvar_pub.publish('s')
                self.alvar_num_pub.publish('1')
                self.one = False
                self.alvar_sub.unregister()
                rospy.sleep(0.02)
                self.alvar_sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.alvar_callback)

            elif self.two:
                self.alvar_pub.publish('s')
                self.alvar_num_pub.publish('2')
                self.two = False
                self.alvar_sub.unregister()
                rospy.sleep(0.02)
                self.alvar_sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.alvar_callback)

            elif self.three:
                self.alvar_pub.publish('s')
                self.alvar_num_pub.publish('3')
                self.three = False
                self.alvar_sub.unregister()
                rospy.sleep(0.02)
                self.alvar_sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.alvar_callback)

            else:
                rospy.loginfo('No alvar detected')

            rate.sleep()

    # alvar callback routine
    def alvar_callback(self, data):
        try:
            if data.markers[0].id == 1:
                rospy.loginfo('alvar ID is one')
                self.one = True
                self.two = False
                self.three = False
        except:
            pass

        try:
            if data.markers[0].id == 2:
                rospy.loginfo('alvar ID is two')
                self.one = False
                self.two = True
                self.three = False
        except:
            pass

        try:
            if data.markers[0].id == 3:
                rospy.loginfo('alvar ID is three')
                self.one = False
                self.two = False
                self.three = True
        except:
            pass

        self.one = False
        self.two = False
        self.three = False

# standard boilerplate
if __name__ == "__main__":
    try:
        rospy.init_node("alvar_pub", anonymous=False)
        #read in joystick input
        run = alvar_detect()
    except rospy.ROSInterruptException:
        rospy.loginfo("Alvar node terminated.")
