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
import smach
import smach_ros
from tf import TransformListener
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose, PoseStamped, PoseWithCovarianceStamped
from move_base_msgs.msg	import MoveBaseGoal, MoveBaseAction
from actionlib_msgs.msg import *

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


#Navigate Waypoints State
class prowl(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['prowl_pass', 'prowl_fail'])
        self.waypoints = []
        self.waypoint_index = None

        rospy.init_node('waypoint_nav')
        waypoint_arr = rospy.get_param('/patrolling/waypoints')
        rospy.loginfo('Printing arr:: %s', waypoint_arr)
        rospy.loginfo('Length of array:: %2f',len(waypoint_arr)-1 )
        for wp in waypoint_arr:
            temp = MoveBaseGoal()

            rospy.loginfo('wp[0]::%2f', wp[0])
            rospy.loginfo('wp[1]::%2f', wp[1])
            rospy.loginfo('wp[2]::%2f', wp[2])
            rospy.loginfo('wp[3]::%2f', wp[3])

            temp.target_pose.header.frame_id = 'map'
            temp.target_pose.pose.position.x = wp[0]
            temp.target_pose.pose.position.y = wp[1]
            temp.target_pose.pose.orientation.z = wp[2]
            temp.target_pose.pose.orientation.w = wp[3]

            self.waypoints.append(temp)


        rospy.loginfo("Waypoint Nav ready.")

    def _nearest_waypoint(self, pose):
        if not self.waypoint_index:
            shortest_len = float('inf')
            shortest_len_index = 0
            x = pose.pose.pose.position.x
            y = pose.pose.pose.position.y

            for i, wp in enumerate(self.waypoints):
                test_len = math.hypot(wp.target_pose.pose.position.x - x,
                                      wp.target_pose.pose.position.y - y)
                if test_len < shortest_len:
                    shortest_len = test_len
                    shortest_len_index = i

            self.waypoint_index = shortest_len_index

    def _update_waypoints(self, data):
        latest = MoveBaseGoal(target_pose = data)

        if rospy.get_param('/waypoints_nav/patrolling/update_patrol'):
            self.waypoints.insert(self.waypoint_index, latest)

            if rospy.get_param('/waypoints_nav/patrolling/save_latest'):
                ros.set_param_raw('/waypoints_nav/patrolling/waypoints',
                                  self.waypoints)

    def _publish_markers(self):
        markers = []
        for i, wp in enumerate(self.waypoints):
            temp = Marker()

            temp.header = wp.target_pose.header
            temp.id = i
            temp.ns = "patrolling"
            temp.action = Marker.ADD

            temp.type = Marker.ARROW
            temp.pose = wp.target_pose.pose
            temp.scale.x = 0.6
            temp.scale.y = 0.3
            temp.scale.z = 0.3
            temp.color.a = 1
            temp.color.r = 0
            temp.color.g = 0.5
            temp.color.b = 0.5
            markers.append(temp)
        self.viz_pub.publish(markers=markers)

        rospy.loginfo("Markers Published")

    def execute(self, userdata):
        self.mvbs = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        self.goal_sub = rospy.Subscriber("move_base_simple/goal", PoseStamped,
                                         self._update_waypoints) #This callback should connect to camera and take a picture. Switch states here.

        self.amcl_sub = rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped,
                                         self._nearest_waypoint)

        self.viz_pub = rospy.Publisher("patrolling/viz_waypoints_array",
                                       MarkerArray, queue_size=10)
        self._publish_markers()

        self.mvbs.wait_for_server()
        forward = True

        rospy.wait_for_message("amcl_pose", PoseWithCovarianceStamped)

        rospy.loginfo("Nearest waypoint is #{}".format(self.waypoint_index))

        rospy.loginfo(self.waypoint_index)

        rospy.loginfo(self.waypoints[self.waypoint_index])

        self._publish_markers()

        self.mvbs.send_goal(self.waypoints[self.waypoint_index])
        self.mvbs.wait_for_result()
        rospy.loginfo("Nav goal met, setting another one...")

        # if self.waypoint_index >= len(self.waypoints) - 1:
        #     forward = False
        # elif self.waypoint_index <= 0:
        #     forward = True
        #
        # if forward:
        #     self.waypoint_index = self.waypoint_index + 1
        # else:
        #     self.waypoint_index = self.waypoint_index - 1
        return 'prowl_pass'

# Error
class handle_error_state(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['error_handling_done'])

    def execute(self, userdata):
        while not(rospy.is_shutdown()):
            pass


def main():
    rospy.init_node("egg_hunter")

    sm = smach.StateMachine(outcomes=['waypoint_nav_sm_init'])

    # Open the container
    with sm:
        # Adding states to the container
        smach.StateMachine.add('prowl', prowl(),
            transitions={'prowl_pass':'click_picture',
                         'prowl_fail':'handle_error_state'})

        smach.StateMachine.add('handle_error_state', handle_error_state(),
            transitions={'error_handling_done':'handle_error_state'})

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()
