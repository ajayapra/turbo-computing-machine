#!/usr/bin/env python
import rospy
import math
import time
import actionlib
import random
import tf
import os
import numpy
from tf import TransformListener

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
        self.listener = tf.TransformListener()
        self.saved_coord = []
        self.runcount = 0
        self.count = 0
        self.countLimit = random.randrange(25,75)
        self.publish_msg = Twist()

        self.turnCoef = [(x ** 2 - 8100) / 10000000.0 for x in range(-90, 0)] + [(-x ** 2 + 8100) / 10000000.0 for x in range(0, 91)]
        self.speedCoef = [(-x ** 2 + 8100) / 10000000.0 for x in range(-90,91)]

        self.last_speed = 0
        self.last_turn = 0

        rospy.Subscriber("/front/scan", LaserScan, self._latestScan)
        rospy.Subscriber("/lab_two_key", String, self.key_callback)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        rospy.loginfo("Ready to get out there and avoid some walls!")
        rospy.logdebug(self.turnCoef)
        rospy.logdebug(self.speedCoef)

        self.timeout = None
        if timeout:
            self.timeout = time.time() + timeout

        self.keyTime = ""
        self.keyMsg = ""

        rospy.spin()

    def isTimedout(self):
        return self.timeout <= time.time()

    # def getKey(self):
    #     tty.setraw(sys.stdin.fileno())
    #     select.select([sys.stdin], [], [], 0)
    #     key = sys.stdin.read(1)
    #     termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
    #     return key

    def key_callback(self, data):
        self.keyTime = time.time()
        self.keyMsg = data.data
        print ("in key callback")
        rospy.loginfo("I heard key %s", data.data)

    def _latestScan(self, data):
        if (self.timeout and self.timeout <= time.time()) or self.keyMsg == 't':
            waypoints = rospy.get_param('waypoints')

            rospy.loginfo('Waypoints: %s', waypoints)
            rospy.loginfo('Dumping waypoints')
            os.system("rosparam dump ~/turbo-computing-machine/catkin_ws/src/egg_hunter/src/waypoints/waypoints.yaml /navigation/waypoints")

            os.system("rosparam dump ~/EE5900_04/turbo-computing-machine/catkin_ws/src/egg_hunter/src/waypoints/waypoints.yaml /navigation/waypoints")
            rospy.loginfo('Waypoints dumped')

            rospy.signal_shutdown("Execution timer expired")
        if (self.keyMsg == 's'):

            pBase = PoseStamped()
            pMap = PoseStamped()
            pBase.header.frame_id = "/base_link";
            pBase.pose.position.x = 0.0;
            pBase.pose.position.y = 0.0;
            pBase.pose.position.z = 0.0;
            pBase.header.stamp = self.listener.getLatestCommonTime("/base_link", "/map")
            pMap = self.listener.transformPose("/map", pBase)
            [new_x, new_y, new_z, new_w] = tf.transformations.quaternion_from_euler(0, 0, 0)
            pBase.pose.orientation.x = new_x
            pBase.pose.orientation.y = new_y
            pBase.pose.orientation.z = new_z
            pBase.pose.orientation.w = new_w

            rospy.loginfo("Position %s",pMap.pose)
            # if self.listener.frameExists("/base_link") and self.listener.frameExists("/map"):
            #      t = self.listener.getLatestCommonTime("/base_link", "/map")
            #      position, quaternion = self.listener.lookupTransform("/base_link", "/map", t)
            #      print position, quaternion
            #      rospy.loginfo('Pose X Position:: %s',position.pose)
            coord = [pMap.pose.position.x,pMap.pose.position.y,pMap.pose.orientation.w, self.count]

            self.saved_coord.append(coord)

            rospy.loginfo('After append Waypoints: %s', self.saved_coord)

            self.count = self.count + 1

            rospy.loginfo("Array Length %d", len(self.saved_coord))

            rospy.set_param('waypoints', numpy.array(self.saved_coord).tolist())

            self.keyMsg = ""

        scale       =  1
        self.angular_min = -1
        self.linear_min  = -1
        self.angular_max =  1
        self.linear_max  =  1
        self.start_time  =  0

        # Constants for laser averaging
        front_delta = 15
        side_ang    = 30
        side_delta  = 15
        side_thresh = 1.35
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
        zeroAng    = int((((abs(data.angle_min) + abs(data.angle_max)) / data.angle_increment) / 2) - 1)
        leftAng    = zeroAng + int(side_ang / toAng(data.angle_increment))
        rightAng   = zeroAng - int(side_ang / toAng(data.angle_increment))
        sideOffset = int(side_delta / toAng(data.angle_increment))
        zeroOffset = int(front_delta / toAng(data.angle_increment))

        # Compute averages for left, right, and front laser scan spans
        leftAve  = getMin(leftAng, leftAng + sideOffset, data)
        rightAve = getMin(rightAng - sideOffset, rightAng, data)
        frontAve = getMin(zeroAng - zeroOffset, zeroAng + zeroOffset, data)

        # Output for monitoring
        #rospy.loginfo('\t%3.4f  -  %3.4f  -  %3.4f', leftAve, frontAve, rightAve)

        # Set the threshold levels for randomization

        # Too close in front, turn left and slowly back up
        if frontAve < 1 :
            self.angular_min = 0.25 * scale
            self.angular_max = 0.5  * scale
            self.linear_min  = -0.05 * scale
            self.linear_max  = 0 * scale

        # All Clear, randomly drive forward with varying turn
        elif (frontAve > 2) and (leftAve > side_thresh) and (rightAve > side_thresh) :
            self.angular_min = -1.25 * scale
            self.angular_max = 1.25 * scale
            self.linear_min  = 0.50 * scale
            self.linear_max  = 1.0 * scale

        # Close to a wall on one side, turn to side with most time
        else :
            if leftAve > rightAve :
                self.angular_min = 0.75 * scale
                self.angular_max = 1.0 * scale
                self.linear_min  = 0.25 * scale
                self.linear_max  = 0.75 * scale
            else :
                self.angular_min = -1.0 * scale
                self.angular_max = -0.75 * scale
                self.linear_min  = 0.25 * scale
                self.linear_max  = 0.75 * scale
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
                self.publish_msg = Twist(linear=linear_msg, angular=angular_msg)
        rospy.loginfo('Published Twist')

if __name__ == "__main__":
    try:
        run = RandomMove() # seconds
    except rospy.ROSInterruptException:
        pass
