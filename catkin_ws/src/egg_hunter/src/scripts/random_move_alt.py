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
from move_base_msgs.msg	import MoveBaseGoal, MoveBaseAction
from visualization_msgs.msg import MarkerArray, Marker

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from actionlib_msgs.msg import *
from geometry_msgs.msg import *


class RandomMove(object):
    def __init__(self, timeout=None):
        rospy.init_node("RandomMove")
        self.angular_min = 0
        self.angular_max = 0
        self.linear_min  = 0
        self.linear_max  = 0
        self.start_time  =  0
	self.halt = 0
	self.haltcount = 1
        self.listener = tf.TransformListener()
        self.saved_coord = []
        self.runcount = 0
        self.count = 0
        self.countLimit = random.randrange(25,75)
        self.publish_msg = Twist()
	self.waypoints = []
        self.rate = 50
        #laser averaging
        self.leftAve  = 0
        self.rightAve = 0
        self.frontAve = 0
        # Constants for laser averaging
        self.front_delta = 15
        self.side_ang    = 30
        self.side_delta  = 15
        self.side_thresh = 1.35
        self.scale = 1
        self.keyMsg = ""
        self.timeout = None
        self.ref_rate =50
	self.state_transition_flag = 0
        #Define timeout
        if self.timeout:
            self.timeout = time.time() + timeout
        #Publications and subscriptions
        rospy.Subscriber("/front/scan", LaserScan, self._latestScan)
        rospy.Subscriber("/action_input", String, self.key_callback)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
	self.viz_pub = rospy.Publisher("patrolling/viz_waypoints_array",
                                       MarkerArray, queue_size=10)
        rospy.loginfo("Gonna Navigate!")
        self.rate = rospy.Rate(self.ref_rate)
        while not(rospy.is_shutdown()):
            self._key_actions()
            self._move_bot()
	    if (self.state_transition_flag == 1):
		rospy.loginfo('Transitioning state')
		rospy.signal_shutdown("Execution timer expired")
	    else:
		pass
            self.rate.sleep()

        #rospy.spin()

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
            temp.scale.x = 0.75
            temp.scale.y = 0.3
            temp.scale.z = 0.3
            temp.color.a = 1
            temp.color.r = 0
            temp.color.g = 0.5
            temp.color.b = 0.5
            markers.append(temp)
        self.viz_pub.publish(markers=markers)


    def isTimedout(self):
        return self.timeout <= time.time()


    def key_callback(self, data):
        self.keyMsg = data.data
        print ("in key callback")
        rospy.loginfo("I heard key %s", data.data)

    def _latestScan(self, data):
        def toAng(rad):
            ang = rad * 180 / 3.14
            return ang
        # Averaged Sum of scan points function
        def getSum(start, end, data):
            angSum = float(0.0)
            index = start
            while index < end :
                if data.ranges[index] < 15:
                    angSum = angSum + data.ranges[index]

                index = index + 1

            angSum = float(angSum) / float(end-start)

            return angSum
        # Averaged Sum of scan points function
        def getMin(start, end, data):
            angSum = float(0.0)
            index = start + 1
            minScan = data.ranges[start]
            while index < end :
                if data.ranges[index] < minScan:
                    prev = data.ranges[index]

                index = index + 1
            return minScan
        #LaserScan Stuff
        zeroAng    = int((((abs(data.angle_min) + abs(data.angle_max)) / data.angle_increment) / 2) - 1)
        leftAng    = zeroAng + int(self.side_ang / toAng(data.angle_increment))
        rightAng   = zeroAng - int(self.side_ang / toAng(data.angle_increment))
        sideOffset = int(self.side_delta / toAng(data.angle_increment))
        zeroOffset = int(self.front_delta / toAng(data.angle_increment))
        # Compute averages for left, right, and front laser scan spans
        self.leftAve  = getMin(leftAng, leftAng + sideOffset, data)
        self.rightAve = getMin(rightAng - sideOffset, rightAng, data)
        self.frontAve = getMin(zeroAng - zeroOffset, zeroAng + zeroOffset, data)


    def _key_actions(self):
        if (self.timeout and self.timeout <= time.time()) or self.keyMsg == 't':
            #Save maps and waypoints, and terminate
            waypoints = rospy.get_param('waypoints')
            rospy.loginfo('Waypoints: %s', waypoints)
            rospy.loginfo('Dumping waypoints')
            #os.system("rosparam dump ~/turbo-computing-machine/catkin_ws/src/egg_hunter/src/waypoints/waypoints.yaml /navigation/waypoints")
            #os.system("rosparam dump ~/EE5900_04/turbo-computing-machine/catkin_ws/src/egg_hunter/src/waypoints/waypoints.yaml /navigation/waypoints")
            os.system("rosparam dump "+str(os.path.dirname(os.path.realpath(__file__)))+"/waypoints/waypoints.yaml /navigation/waypoints")
	    os.system("rosrun map_server map_saver -f "+str(os.path.dirname(os.path.realpath(__file__)))+"/maps/SavedMap")
            rospy.loginfo('Path: %s',)
            rospy.loginfo('Waypoints dumped. Map Saved')
            #package ='map_server'
            #executable ='map_saver'
            #node = roslaunch.core.Node(package, executable, args="-f "+str(os.path.dirname(os.path.realpath(__file__)))+"/maps/map")
            #launch = roslaunch.scriptapi.ROSLaunch()
            #launch.start()
            #process = launch.launch(node)
            #while process.is_alive():
            #    pass
            self.state_transition_flag = 1
        if (self.keyMsg == 's'):
            pBase = PoseStamped()
            pMap = PoseStamped()
            pBase.header.frame_id = "/base_link";
            pBase.pose.position.x = 0.0;
            pBase.pose.position.y = 0.0;
            pBase.pose.position.z = 0.0;
            pBase.header.stamp = self.listener.getLatestCommonTime("/base_link", "/map")
            pMap = self.listener.transformPose("/map", pBase)
	    temp = MoveBaseGoal()
            temp.target_pose.header.frame_id = 'map'
            temp.target_pose.pose.position.x = pMap.pose.position.x
            temp.target_pose.pose.position.y = pMap.pose.position.y
            temp.target_pose.pose.position.z = 0
            temp.target_pose.pose.orientation.x = 0
            temp.target_pose.pose.orientation.y = 0
            temp.target_pose.pose.orientation.z = pMap.pose.orientation.z
            temp.target_pose.pose.orientation.w = pMap.pose.orientation.w
            self.waypoints.append(temp)
            #Alt implementation
            #rospy.loginfo(self.waypoints)
            #rospy.loginfo("Position %s",pMap.pose)
            # if self.listener.frameExists("/base_link") and self.listener.frameExists("/map"):
            #      t = self.listener.getLatestCommonTime("/base_link", "/map")
            #      position, quaternion = self.listener.lookupTransform("/base_link", "/map", t)
            #      print position, quaternion
            #      rospy.loginfo('Pose X Position:: %s',position.pose)
            coord = [pMap.pose.position.x,pMap.pose.position.y, pMap.pose.orientation.z, pMap.pose.orientation.w, self.count]
            self.saved_coord.append(coord)
            #rospy.loginfo('After append Waypoints: %s', self.saved_coord)
            self.count = self.count + 1
            #rospy.loginfo("Array Length %d", len(self.saved_coord))
            rospy.set_param('waypoints', numpy.array(self.saved_coord).tolist())
            self.keyMsg = ""
	if (self.keyMsg == 'h'):
		self.haltcount = self.haltcount + 1
		self.halt = (-1)**(self.haltcount)+self.halt
		self.keyMsg = ""
	rospy.loginfo("Halt Flag: %s", self.halt)


    def _move_bot(self):
        if self.frontAve < 1 :
            self.angular_min = 0.25 * self.scale
            self.angular_max = 0.5  * self.scale
            self.linear_min  = -0.05 * self.scale
            self.linear_max  = 0 * self.scale

        # All Clear, randomly drive forward with varying turn
        elif (self.frontAve > 2) and (self.leftAve > self.side_thresh) and (self.rightAve > self.side_thresh) :
            self.angular_min = -1.25 * self.scale
            self.angular_max = 1.25 * self.scale
            self.linear_min  = 0.50 * self.scale
            self.linear_max  = 1.0 * self.scale

        # Close to a wall on one side, turn to side with most time
        else :
            if self.leftAve > self.rightAve :
                self.angular_min = 0.75 * self.scale
                self.angular_max = 1.0 * self.scale
                self.linear_min  = 0.25 * self.scale
                self.linear_max  = 0.75 * self.scale
            else :
                self.angular_min = -1.0 * self.scale
                self.angular_max = -0.75 * self.scale
                self.linear_min  = 0.25 * self.scale
                self.linear_max  = 0.75 * self.scale
                    # generate random movement mapping at random interval
        if self.runcount < self.countLimit :
                self.runcount = self.runcount + 1
                self.pub.publish(self.publish_msg)
        else :
                self.runcount = 0
                self.countLimit = random.randrange(5,25)
                randLin = random.uniform(self.linear_min,self.linear_max)
                randAng = random.uniform(self.angular_min,self.angular_max)
                # push Twist msgs
                linear_msg  = Vector3(x=randLin, y=float(0.0), z=float(0.0))
                angular_msg = Vector3(x=float(0.0), y=float(0.0), z=randAng)
		if (self.halt == 1):
		    self.publish_msg = Twist()
		else:
            	    self.publish_msg = Twist(linear=linear_msg, angular=angular_msg)
	        self._publish_markers()
        rospy.loginfo('Published Twist')

if __name__ == "__main__":
    try:
        run = RandomMove() # seconds
    except rospy.ROSInterruptException:
        pass
