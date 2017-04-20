#!/usr/bin/env python
import rospy
import math
import time
import actionlib
import random
import tf
import os
import numpy
import roslaunch
from tf import TransformListener
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from actionlib_msgs.msg import *
from geometry_msgs.msg import *

import cv2
import cv_bridge
import argparse
from sensor_msgs.msg import Image

class random_move(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])
        self.angular_min = 0
        self.angular_max = 0
        self.linear_min  = 0
        self.linear_max  = 0
        self.start_time  =  0
        self.listener = tf.TransformListener()
        self.saved_coord = []
        self.runcount = 0
        self.count = 0
        self.countLimit = random.randrange(25,75)
        self.publish_msg = Twist()
        self.waypoints = []
        self.timeout = None
        self.turnCoef = [(x ** 2 - 8100) / 10000000.0 for x in range(-90, 0)] + [(-x ** 2 + 8100) / 10000000.0 for x in range(0, 91)]
        self.speedCoef = [(-x ** 2 + 8100) / 10000000.0 for x in range(-90,91)]
        self.last_speed = 0
        self.last_turn = 0
        self.keyTime = ""
        self.keyMsg = ""

    def execute(self):
        rospy.Subscriber("/front/scan", LaserScan, self._latestScan)
        rospy.Subscriber("/lab_two_key", String, self.key_callback)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.viz_pub = rospy.Publisher("patrolling/viz_waypoints_array",
                                       MarkerArray, queue_size=10)

        rospy.loginfo("Ready to get out there and avoid some walls!")
        rospy.logdebug(self.turnCoef)
        rospy.logdebug(self.speedCoef)

        if timeout:
            self.timeout = time.time() + timeout
        rospy.spin()

class get_waypoint(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['got_waypoint'])

    def execute(self, userdata):
        rospy.loginfo('getting waypoint')
        return 'got_waypoint'

class check_waypoint(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['waypoint_present','add_waypoint'])

    def execute(self, userdata):
        rospy.loginfo('checking waypoint')
        # Subscribe to waypoint message and get the waypoint
        for i in len(waypoints):
            if got_waypoint in waypoints:
                return 'waypoint_present'
            else:
                return 'add_waypoint'

def click_picture(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=[['num_eggs'])
        self.bridge = cv_bridge.CvBridge()
    def image_callback(self, msg):
        rospy.loginfo('counting_eggs')
        
        # Count eggs


    def execute(seld, userdata):
        self.image_sb = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        return 'num_eggs'
