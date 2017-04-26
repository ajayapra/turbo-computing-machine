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

#Navigate Waypoints State
class prowl(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['prowl_pass'])
        self.waypoints = []
        self.waypoint_index = None

        rospy.init_node('waypoint_nav')
        waypoint_arr = rospy.get_param('/navigation/waypoints')
        rospy.loginfo('Printing arr:: %s', waypoint_arr)
        rospy.loginfo('Length of array:: %2f',len(waypoint_arr)-1 )
        for wp in waypoint_arr:
            temp = MoveBaseGoal()

            rospy.loginfo('wp[0]::%2f', wp[0]) #starting point of the course - navigate to it and detect alvar teg to set 
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

    #Not needed, pop from the array each time
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
        return 'click_picture'

class click_picture(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=[['num_eggs'])
        global radius
    	global x
    	global y
        self.bridge = cv_bridge.CvBridge()
        self.image_sb = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)

    def image_callback(self, msg):
        rospy.loginfo('counting_eggs')
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.image_sb.unregister()
        # Count eggs


    def execute(seld, userdata):
        self.image_sb = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)

        # cvtColor applies an adaptive threshold to an array.
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # Green threshold
        green_lower_range = np.array([60, 100, 100], dtype=np.uint8)
        green_upper_range = np.array([100, 255, 255], dtype=np.uint8)

        # Orange threshold
        orange_lower_range = np.array([7, 100, 100], dtype=np.uint8)
        orange_upper_range = np.array([29, 255, 255], dtype=np.uint8)

        # Blue threshold
        blue_lower_range = np.array([95, 100, 100], dtype=np.uint8)
        blue_upper_range = np.array([112, 255, 255], dtype=np.uint8)

        # Yellow threshold
        yellow_lower_range = np.array([24, 100, 100], dtype=np.uint8)
        yellow_upper_range = np.array([28, 255, 255], dtype=np.uint8)

        # Pink threshold
        pink_lower_range = np.array([165, 100, 100], dtype=np.uint8)
        pink_upper_range = np.array([200, 255, 255], dtype=np.uint8)

        # Violet threshold
        violet_lower_range = np.array([128, 100, 100], dtype=np.uint8)
        violet_upper_range = np.array([164, 255, 255], dtype=np.uint8)

        # A series of dilations and erosions to remove any small blobs left
        # in the mask.
        green_mask = cv2.inRange(hsv, green_lower_range, green_upper_range)
        orange_mask = cv2.inRange(hsv, orange_lower_range, orange_upper_range)
        blue_mask = cv2.inRange(hsv, blue_lower_range, blue_upper_range)
        yellow_mask = cv2.inRange(hsv, yellow_lower_range, yellow_upper_range)
        pink_mask = cv2.inRange(hsv, pink_lower_range, pink_upper_range)
        violet_mask = cv2.inRange(hsv, violet_lower_range, violet_upper_range)

        # Count number of violet eggs
        mask = violet_mask
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=6)
        #masked = cv2.bitwise_and(img, img, mask=mask)
        violet_count = 0
        try:
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
        	cv2.CHAIN_APPROX_SIMPLE)[-2]
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            if radius > 5 and radius < 80:
                violet_count = violet_count + 1
        except:
            pass
        #print radius
        print('violet: ' + str(violet_count))

        # Count number of green eggs
        mask = green_mask
        mask = cv2.erode(mask, None, iterations=2)
        green_mask = cv2.dilate(mask, None, iterations=6)
        #masked = cv2.bitwise_and(img, img, mask=mask)
        green_count = 0
        try:
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
        	cv2.CHAIN_APPROX_SIMPLE)[-2]
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            if radius > 5 and radius < 80:
                green_count = green_count + 1
        except:
            pass
        #print radius
        print('green: ' + str(green_count))

        # Count number of pink eggs
        mask = pink_mask
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=6)
        #masked = cv2.bitwise_and(img, img, mask=mask)
        pink_count = 0
        try:
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
        	cv2.CHAIN_APPROX_SIMPLE)[-2]
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            if radius > 5 and radius < 100:
                pink_count = pink_count + 1
        except:
            pass
        #print radius
        print('pink: ' + str(pink_count))

        # Count number of blue eggs
        mask = blue_mask
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=6)
        #masked = cv2.bitwise_and(img, img, mask=mask)
        blue_count = 0
        try:
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
        	cv2.CHAIN_APPROX_SIMPLE)[-2]
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            if radius > 5 and radius < 80:
                blue_count = blue_count + 1
        except:
            pass
        #print radius
        print('blue: ' + str(blue_count))

        # Count number of orange eggs
        mask = orange_mask
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=6)
        #masked = cv2.bitwise_and(img, img, mask=mask)
        orange_count = 0
        try:
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
        	cv2.CHAIN_APPROX_SIMPLE)[-2]
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            if radius > 5 and radius < 100:
                orange_count = orange_count + 1
        except:
            pass
        print('orange: ' + str(orange_count))

        # Count number of yellow eggs
        mask = yellow_mask
        mask = cv2.erode(mask, None, iterations=2)
        yellow_mask = cv2.dilate(mask, None, iterations=6)
        #masked = cv2.bitwise_and(img, img, mask=mask)
        yellow_count = 0
        try:
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
        	cv2.CHAIN_APPROX_SIMPLE)[-2]
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            if radius > 5 and radius < 80:
                yellow_count = yellow_count + 1
        except:
            pass
        #print radius
        print('yellow: ' + str(yellow_count))

        return 'num_eggs'

class terminate(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['terminate_success'])

    def execute(self, userdata):
	global waypoints
        #rospy.loginfo('In terminate smach state')
        #package ='map_server'
        #executable ='map_saver'
        #node = roslaunch.core.Node(package, executable, args="-f "+str(os.path.dirname(os.path.realpath(__file__)))+"/maps/map")
        #launch = roslaunch.scriptapi.ROSLaunch()
        #launch.start()
        #process = launch.launch(node)
        #while process.is_alive():
        #   pass
        #rospy.loginfo('In Terminate Success')
        #waypoints = rospy.get_param('/navigation/waypoints')
        #rospy.loginfo('Waypoints: %s', waypoints)
        #rospy.loginfo('Dumping waypoints')
        #os.system("rosparam dump "+str(os.path.dirname(os.path.realpath(__file__)))+"/waypoints/waypoints.yaml /navigation/waypoints")
        return 'terminate_success'


def main():
    rospy.init_node("egg_hunting")

    sm = smach.StateMachine(outcomes=['waypoint_nav_sm_init'])

    # Open the container
    with sm:
        # Adding states to the container
        smach.StateMachine.add('prowl', prowl(),
            transitions={'prowl_pass':'click_picture'})

        smach.StateMachine.add('click_picture', click_picture(),
transitions={'click_picture_success':'prowl'})
    
        smach.StateMachine.add('terminate', terminate(),
transitions={'terminate_success':'waypoint_nav_sm_init'})

        # smach.StateMachine.add('handle_error_state', handle_error_state(),
        #     transitions={'error_handling_done':'handle_error_state'})

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()
