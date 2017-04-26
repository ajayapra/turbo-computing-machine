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

target = ""
waypoints = []
waypoint_bunny_index = None
alvar_num = 0


class detect_alvar(smach.State):
    def __init__(self):
        global alvar_num
        smach.State.__init__(self, outcomes=['alvar_detected'])

    def alvar_callback(self, data):
        try:
            if data.markers[0].id!=0:
                alvar_num = data.markers[0].id
                rospy.loginfo('alvar ID is :: %s', alvar_num)
        except:
            pass


    def execute(self, userdata):
        while not rospy.is_shutdown():
            self.alvar_sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.alvar_callback)
            if alvar_num!=0:
                return 'alvar_detected'
            self.alvar_sub.unregister()
            rospy.sleep(0.02)
        rate.sleep()


#Navigate Waypoints State
class prowl(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['prowl_pass'])
        global waypoints
        global waypoint_bunny_index

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
            rospy.loginfo('wp[4]::%2f', wp[4])

            temp.target_pose.header.frame_id = 'map'
            temp.target_pose.pose.position.x = wp[0]
            temp.target_pose.pose.position.y = wp[1]
            temp.target_pose.pose.orientation.z = wp[2]
            temp.target_pose.pose.orientation.w = wp[3]

            waypoints.append(temp)
            waypoint_bunny_index.append(wp[4])

        self.mvbs = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.mvbs.send_goal(waypoints[0])

        # self.goal_sub = rospy.Subscriber("move_base_simple/goal", PoseStamped,
        #                                   self._update_waypoints) #This callback should connect to camera and take a picture. Switch states here.

        # for wp in len(waypoint_bunny_index):
        #     if waypoint_bunny_index[wp] == 0:
        #
        #         self.mvbs.send_goal( waypoints[waypoint_bunny_index[wp])
        #         rospy.loginfo("Waypoint to starting position::%s, waypoint_bunny_index[wp]")
        #         self.wait_for_result()



    #Not needed, pop from the array each time
    # def _nearest_waypoint(self, pose):
    #     if not self.waypoint_index:
    #         shortest_len = float('inf')
    #         shortest_len_index = 0
    #         x = pose.pose.pose.position.x
    #         y = pose.pose.pose.position.y
    #
    #         for i, wp in enumerate(self.waypoints):
    #             test_len = math.hypot(wp.target_pose.pose.position.x - x,
    #                                   wp.target_pose.pose.position.y - y)
    #             if test_len < shortest_len:
    #                 shortest_len = test_len
    #                 shortest_len_index = i
    #
    #         self.waypoint_index = shortest_len_index

    # def _update_waypoints(self, data):
    #     latest = MoveBaseGoal(target_pose = data)
    #
    #     if rospy.get_param('/waypoints_nav/patrolling/update_patrol'):
    #         self.waypoints.insert(self.waypoint_index, latest)
    #
    #         if rospy.get_param('/waypoints_nav/patrolling/save_latest'):
    #             ros.set_param_raw('/waypoints_nav/patrolling/waypoints',
    #                               self.waypoints)

    def _publish_markers(self):
        global waypoints
        markers = []
        for i, wp in enumerate(waypoints):
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
        global alvar_num
        global waypoints
        self.mvbs = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        # self.goal_sub = rospy.Subscriber("move_base_simple/goal", PoseStamped,
        #                                  self._update_waypoints) #This callback should connect to camera and take a picture. Switch states here.
        #
        # self.amcl_sub = rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped,
        #                                  self._nearest_waypoint)

        for bi in range(0, len(waypoint_bunny_index)-1):
            if waypoint_bunny_index[bi]==alvar_num:
                self.target_goal = waypoints[bi]

        self.viz_pub = rospy.Publisher("patrolling/viz_waypoints_array",
                                       MarkerArray, queue_size=10)
        self._publish_markers()

        self.mvbs.wait_for_server()
        forward = True

        rospy.wait_for_message("amcl_pose", PoseWithCovarianceStamped)

        # rospy.loginfo("Nearest waypoint is #{}".format(self.waypoint_index))
        #
        # rospy.loginfo(self.waypoint_index)
        #
        # rospy.loginfo(self.waypoints[self.waypoint_index])

        self._publish_markers()

        # self.mvbs.send_goal(self.waypoints[self.waypoint_index])

        self.mvbs.send_goal(self.target_goal)

        self.mvbs.wait_for_result()

        rospy.loginfo("Nav goal set...")
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
        smach.State.__init__(self,outcomes=['counting_eggs_success'])
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
    def toAng(self, rad):
        ang = rad * 180 / 3.14
        return ang
    def getMin(self, start, end, data):
        angSum = float(0.0)
        index = start + 1
        minScan = data.ranges[start]
        while index < end :
            if data.ranges[index] < minScan:
                prev = data.ranges[index]
            index = index + 1
        return minScan
    def _latestScan(self, data):
            #LaserScan Stuff
        zeroAng    = int((((abs(data.angle_min) + abs(data.angle_max)) / data.angle_increment) / 2) - 1)
        leftAng    = zeroAng + int(self.side_ang / self.toAng(data.angle_increment))
        rightAng   = zeroAng - int(self.side_ang / self.toAng(data.angle_increment))
        sideOffset = int(self.side_delta / self.toAng(data.angle_increment))
        zeroOffset = int(self.front_delta / self.toAng(data.angle_increment))
        # Compute averages for left, right, and front laser scan spans
        self.leftAve  = self.getMin(leftAng, leftAng + sideOffset, data)
        self.rightAve = self.getMin(rightAng - sideOffset, rightAng, data)
        self.frontAve = self.getMin(zeroAng - zeroOffset, zeroAng + zeroOffset, data)
        rospy.loginfo('\t%3.4f  -  %3.4f  -  %3.4f', self.leftAve, self.frontAve, self.rightAve)
        # All Clear, randomly drive forward with varying turn
        if (self.frontAve > 0.8) and (self.leftAve > 0.8) and (self.rightAve > 0.8):
            self.move_forward = True
        else:
            self.move_forward = False

    def execute(seld, userdata):
        #laser averaging
        self.leftAve  = 0
        self.rightAve = 0
        self.frontAve = 0
        # Constants for laser averaging
        self.front_delta = 15
        self.side_ang    = 30
        self.side_delta  = 20
        self.move_forward = False
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self._latestScan)
        self.image_sb = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=3)
        move_rate = rospy.Rate(100)
        while self.move_forward:
            linear_msg  = Vector3(x=float(0.2), y=float(0.0), z=float(0.0))
            angular_msg = Vector3(x=float(0.0), y=float(0.0), z=float(0.0))
            self.publish_msg = Twist(linear=linear_msg, angular=angular_msg)
            move_rate.sleep()

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

        return 'counting_eggs_success'


def main():
    rospy.init_node("egg_hunting")

    sm = smach.StateMachine(outcomes=['waypoint_nav_sm_init'])

    # Open the container
    with sm:
        # Adding states to the container

        smach.StateMachine.add('detect_alvar', detect_alvar(),
            transitions={'alvar_detected':'prowl'})

        smach.StateMachine.add('prowl', prowl(),
            transitions={'prowl_pass':'click_picture'})

        smach.StateMachine.add('terminate', terminate(),
transitions={'terminate_success':'waypoint_nav_sm_init'})

        smach.StateMachine.add('click_picture', click_picture(),
transitions={'counting_eggs_success':'waypoint_nav_sm_init'})
        # smach.StateMachine.add('handle_error_state', handle_error_state(),
        #     transitions={'error_handling_done':'handle_error_state'})

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()
