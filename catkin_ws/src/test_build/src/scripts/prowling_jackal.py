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
from geometry_msgs.msg import Twist, Pose, PoseStamped, PoseWithCovarianceStamped, Vector3
from move_base_msgs.msg	import MoveBaseGoal, MoveBaseAction
from actionlib_msgs.msg import *
#from ar_track_alvar_msgs.msg import AlvarMarkers

import cv2
import cv_bridge
import argparse
from sensor_msgs.msg import Image

waypoints = []
temp_waypoint = []
saved_coord = []
alvar_num = 0
keyMsg = ""
viz_pub = rospy.Publisher("patrolling/viz_waypoints_array",
                               MarkerArray, queue_size=10)

def publish_markers():
    markers = []
    global waypoints
    global viz_pub
    for i, wp in enumerate(waypoints):
        temp = Marker()
        rospy.loginfo('Visualizing Waypoints')
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
        viz_pub.publish(markers=markers)

class mapping(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['bunny_found', 'terminate'])
        #smach.State.__init__(self, outcomes=['bunny_found'])
        rospy.loginfo('In mapping smach state')
        self.angular_min = 0
        self.angular_max = 0
        self.linear_min  = 0
        self.linear_max  = 0
        self.start_time  =  0
        self.halt = 0
        self.haltcount = 1
        #self.saved_coord = []
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
        #self.side_delta  = 15 
        #self.side_thresh = 1.35
        self.scale = 0.65
        #self.keyMsg = ""
        self.timeout = None
        self.ref_rate =100
        self.state_transition_flag = 0
        #Define timeout
        if self.timeout:
            self.timeout = time.time() + timeout
        #Publications and subscriptions
        ###
        self.linear_acc  =  0.01
        self.angular_acc =  0.005
        self.count_min = 25
        self.count_max = 40
        self.escape_command = 0
        self.danger_flag = 0
        self.side_delta  = 20
        self.side_thresh = 0.875
        self.randLin = float(0.0)
        self.randAng = float(0.0)
        self.linSet = float(0.0)
        self.angSet = float(0.0)
        ###
    
        rospy.Subscriber("/scan", LaserScan, self._latestScan)
        rospy.Subscriber("/action_input", String, self.key_callback)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.loginfo("Gonna Navigate!")

    def key_callback(self, data):
        global keyMsg
        keyMsg = data.data
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
        rospy.loginfo('\t%3.4f  -  %3.4f  -  %3.4f', self.leftAve, self.frontAve, self.rightAve)

    def _move_bot(self):
        if self.frontAve < 1 :
            #self.angular_min = 0.25 * self.scale
            self.angular_min = 0.3 * self.scale
            self.angular_max = 0.5  * self.scale
            self.linear_min  = -0.05 * self.scale
            self.linear_max  = 0 * self.scale
            ##
            self.linear_acc  =  1.0 * self.scale
            self.angular_acc =  1.0 * self.scale
            self.escape_command = 1
            ##

        # All Clear, randomly drive forward with varying turn
        elif (self.frontAve > 1.75) and (self.leftAve > self.side_thresh) and (self.rightAve > self.side_thresh) :
            #self.angular_min = -1.25 * self.scale
            #self.angular_max = 1.25 * self.scale
            self.angular_min = -0.625 * self.scale
            self.angular_max = 0.625 * self.scale
        
            #self.linear_min  = 0.50 * self.scale
            #self.linear_max  = 1.0 * self.scale
            
            self.linear_min  = 0.75 * self.scale
            self.linear_max  = 1.0 * self.scale
            
            self.linear_acc  =  0.0005 * self.scale
            self.angular_acc =  0.0001 * self.scale
    
            self.danger_flag = 0

        # Close to a wall on one side, turn to side with most time
        else :
            self.linear_acc  =  0.0005 * self.scale
            self.angular_acc =  0.0001 * self.scale
            self.escape_command = 1

            if self.leftAve > self.rightAve :
                self.angular_min = 0.75 * self.scale
                self.angular_max = 1.0 * self.scale
                self.linear_min  = 0.25 * self.scale
                #self.linear_max  = 0.75 * self.scale
                self.linear_max  = 0.50 * self.scale
            else :
                self.angular_min = -1.0 * self.scale
                self.angular_max = -0.75 * self.scale
                self.linear_min  = 0.25 * self.scale
                #self.linear_max  = 0.75 * self.scale
                self.linear_max  = 0.50 * self.scale
                    # generate random movement mapping at random interval
        #if self.runcount < self.countLimit :
        #        self.runcount = self.runcount + 1
        #        self.pub.publish(self.publish_msg)
        if self.runcount < self.countLimit:
            if (self.escape_command == 1) and (self.danger_flag == 0):
                self.danger_flag = 1
                self.escape_command = 0
                self.runcount = self.countLimit
            else :
                self.runcount = self.runcount + 1
                
        else :
                self.runcount = 0
                #self.countLimit = random.randrange(5,25)
                self.countLimit = random.randrange(self.count_min, self.count_max)
                self.randLin = random.uniform(self.linear_min,self.linear_max)
                self.randAng = random.uniform(self.angular_min,self.angular_max)
                # push Twist msgs
                #linear_msg  = Vector3(x=randLin, y=float(0.0), z=float(0.0))
                #angular_msg = Vector3(x=float(0.0), y=float(0.0), z=randAng)

        rospy.loginfo('randLin: %2f, randAng: %2f',self.randLin, self.randAng )

        if (self.randLin > self.linSet):
            if (self.randLin > (self.linSet + self.linear_acc)):
                rospy.loginfo('1st if max acc exceeded')
                self.linSet = self.linSet + self.linear_acc
            else :
                self.linSet = self.randLin

        else :
            if (self.randLin < (self.linSet - self.linear_acc)):
                self.linSet = self.linSet - self.linear_acc
            else :
                self.linSet = self.randLin

        if (self.randAng > self.angSet):
            if (self.randAng > (self.angSet + self.angular_acc)):
                self.angSet = self.angSet + self.angular_acc
            else :
                self.angSet = self.randAng
        else :
            if (self.randAng < (self.angSet - self.angular_acc)):
                self.angSet = self.angSet - self.angular_acc
            else :
                self.angSet = self.randAng
  
        rospy.loginfo('LinSet:: %2f, AngSet:: %2f', self.linSet, self.angSet )     

        linear_msg  = Vector3(x=self.linSet, y=float(0.0), z=float(0.0))
        angular_msg = Vector3(x=float(0.0), y=float(0.0), z=self.angSet)
	if (self.halt == 1):
	    self.publish_msg = Twist()
	else :
            self.publish_msg = Twist(linear=linear_msg, angular=angular_msg)
        #self.publish_msg = Twist(linear=linear_msg, angular=angular_msg) 
        self.pub.publish(self.publish_msg)
	publish_markers()
        rospy.loginfo('Published Twist')

    def execute(self, userdata):
        global keyMsg
        rospy.loginfo('In execute')
        self.rate = rospy.Rate(self.ref_rate)
        while not(keyMsg == 's' or keyMsg == 't'):
            if ( keyMsg == 'h'):
                self.haltcount = self.haltcount + 1
                self.halt = (-1)**(self.haltcount)+self.halt
                keyMsg = ""
            self._move_bot()
            self.rate.sleep()
        if ( keyMsg == 's'):
            rospy.loginfo('keyMsg == s')
            return 'bunny_found'
        else:
            rospy.loginfo('keyMsg == t')
            return 'terminate'

class get_waypoint(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['got_waypoint'])
        rospy.loginfo('In get_waypoint smach state')
        rospy.loginfo('getting waypoint')
        self.pBase = PoseStamped()
        self.pMap = PoseStamped()
        self.listener = tf.TransformListener()

    #def marker_callback(self, data):
    #    global alvar_num
    #    alvar_num = data.markers[0].id

    def execute(self, userdata):
        global temp_waypoint
        #define alvar subscribe and callback
        #rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.marker_callback)
        self.pBase.header.frame_id = "/base_link";
        self.pBase.pose.position.x = 0.0;
        self.pBase.pose.position.y = 0.0;
        self.pBase.pose.position.z = 0.0;
        self.pBase.header.stamp = self.listener.getLatestCommonTime("/base_link", "/map")
        self.pMap = self.listener.transformPose("/map", self.pBase)
        temp_waypoint = [self.pMap.pose.position.x,self.pMap.pose.position.y, self.pMap.pose.orientation.z, self.pMap.pose.orientation.w, alvar_num]
        return 'got_waypoint'

class check_waypoint(smach.State):
    def __init__(self):
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        smach.State.__init__(self,outcomes=['savepoint_success'])
        rospy.loginfo('In check_waypoint smach state')
        global temp_waypoint
        global alvar_num
        global waypoints
        #comment on implementing alvar tags - Publisher subscriber
        #for recieving alvar tag num


    def execute(self, userdata):
        rospy.loginfo('checking waypoint')
        alvar_tag_found_flag=0
        global temp_waypoint
        global alvar_num
        global keyMsg
        global saved_coord
        #Comment for actual alvar tags
        alvar_num = alvar_num + 1
        #Check saved_coord:alavr num with temp_waypoint: alvar num
        #push only if not present   rospy.loginfo('In check_waypoint smach state')
        # temp_waypoint = [self.pMap.pose.position.x,self.pMap.pose.position.y, self.pMap.pose.orientation.z, self.pMap.pose.orientation.w, alvar_num]
        for wp in saved_coord:
             if wp[4]==alvar_num:
                 alvar_tag_found_flag = 1
        if alvar_tag_found_flag != 1:
            saved_coord.append(temp_waypoint)
            alvar_tag_found_flag = 0
            temp = MoveBaseGoal()
            temp.target_pose.header.frame_id = 'map'
            temp.target_pose.pose.position.x = temp_waypoint[0]
            temp.target_pose.pose.position.y = temp_waypoint[1]
            temp.target_pose.pose.position.z = 0
            temp.target_pose.pose.orientation.x = 0
            temp.target_pose.pose.orientation.y = 0
            temp.target_pose.pose.orientation.z = temp_waypoint[2]
            temp.target_pose.pose.orientation.w = temp_waypoint[3]
            waypoints.append(temp)
        #Turn robot 180 for a finite time
	timeNow = time.time()
        while (time.time() - timeNow < 2):
	    linear_msg  = Vector3(x=float(-0.05), y=float(0.0), z=float(0.0))
            angular_msg = Vector3(x=float(0.0), y=float(0.0), z=float(0.0))
            self.pub.publish(Twist(linear=linear_msg, angular=angular_msg))	
        timeNow = time.time()
        while (time.time() - timeNow < 15):
            linear_msg  = Vector3(x=float(0.0), y=float(0.0), z=float(0.0))
            angular_msg = Vector3(x=float(0.0), y=float(0.0), z=float(0.3))
            self.pub.publish(Twist(linear=linear_msg, angular=angular_msg))
        #
        rospy.set_param('waypoints', numpy.array(saved_coord).tolist())
        keyMsg = ""
        return 'savepoint_success'
#
class terminate(smach.State):
    def __init__(self):
        global waypoints
        smach.State.__init__(self,outcomes=['terminate_success'])
        rospy.loginfo('In terminate smach state')
        package ='map_server'
        executable ='map_saver'
        node = roslaunch.core.Node(package, executable, args="-f "+str(os.path.dirname(os.path.realpath(__file__)))+"/maps/map")
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()
        process = launch.launch(node)
        while process.is_alive():
           pass

    def execute(self, userdata):
        rospy.loginfo('In Terminate Success')
        waypoints = rospy.get_param('waypoints')
        rospy.loginfo('Waypoints: %s', waypoints)
        rospy.loginfo('Dumping waypoints')
        os.system("rosparam dump "+str(os.path.dirname(os.path.realpath(__file__)))+"/waypoints/waypoints.yaml /waypoints")
        return 'terminate_success'

def main():
    rospy.init_node("egg_hunter")

    sm = smach.StateMachine(outcomes=['mapping_sm_init'])

    # Open the container
    with sm:
        # Adding states to the container
        smach.StateMachine.add('mapping', mapping(),
            transitions={'bunny_found':'get_waypoint',
                         'terminate':'terminate'})

        smach.StateMachine.add('get_waypoint', get_waypoint(),
            transitions={'got_waypoint':'check_waypoint'})

        smach.StateMachine.add('check_waypoint', check_waypoint(),
            transitions={'savepoint_success':'mapping'})

        smach.StateMachine.add('terminate', terminate(),
            transitions={'terminate_success':'mapping_sm_init'})


        # smach.StateMachine.add('handle_error_state', handle_error_state(),
        #     transitions={'error_handling_done':'handle_error_state'})

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()
