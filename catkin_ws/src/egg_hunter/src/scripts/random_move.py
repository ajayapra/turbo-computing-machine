#!/usr/bin/env python
import rospy
import math
import time
import actionlib
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

        self.listener = tf.TransformListener()
        self.saved_coord = []
        self.count = 0

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
ma
    def _latestScan(self, data):
        if (self.timeout and self.timeout <= time.time()) or self.keyMsg == 't':
            waypoints = rospy.get_param('waypoints')

            rospy.loginfo('Waypoints: %s', waypoints)
            rospy.loginfo('Dumping waypoints')
<<<<<<< HEAD
            os.system("rosparam dump ~/turbo-computing-machine/catkin_ws/src/egg_hunter/src/waypoints/waypoints.yaml /navigation/waypoints")h 
=======

            os.system("rosparam dump ~/EE5900_04/turbo-computing-machine/catkin_ws/src/egg_hunter/src/waypoints/waypoints.yaml /navigation/waypoints")

>>>>>>> 41b606b113f1e9afb0d1088bc276bc7fb2cc8588
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

        turnVal = 0
        speedVal = 0
        right_zone = data.ranges[0:65]
        front_zone = data.ranges[65:115]
        left_zone = data.ranges[115:180]

        front_zone_avg = sum(front_zone) / len(front_zone)
        right_zone_avg = sum(right_zone) / len(right_zone)
        left_zone_avg = sum(left_zone) / len(left_zone)

        # If average is really REALLY close, might want to back up instead
        #if front_zone_avg < 1.5 or min(front_zone) < 0.8:
        if front_zone_avg < 4.5 or min(front_zone) < 1.8:
            speedVal = 0.0
            if self.last_turn > 0:
                rospy.loginfo("Backing up to the left...")
                turnVal = 1#0.5
            else:
                rospy.loginfo("Backing up to the right...")
                turnVal = -1#-0.3

        else:
            rospy.loginfo("Normal hallway")
            for p in range(0, 181):
                # Inf range return means its over 10m from the LIDAR
                if math.isinf(data.ranges[p]) or math.isnan(data.ranges[p]):
                    speedVal = speedVal + (self.speedCoef[p] * 10)
                    # Don't account long ranges into turn calcs
                else:
                    speedVal = speedVal + (self.speedCoef[p] * data.ranges[p])

                    # Turn away from walls
                    turnVal = turnVal + (self.turnCoef[p] * data.ranges[p])

            #speedVal = min(speedVal * 1.2, 0.4) # sets max speed
            speedVal = min(speedVal * 1.2, 1) # sets max speed
            turnVal = turnVal * 1.4

            if front_zone_avg < 3.0:
                turnVal = turnVal * 2.0
                speedVal = speedVal * 1.1

        cmd = Twist()
        cmd.linear.x = speedVal
        cmd.angular.z = turnVal

        self.last_speed = speedVal
        self.last_turn = turnVal

        rospy.loginfo(cmd)

        self.pub.publish(cmd)

# standard ros boilerplate
if __name__ == "__main__":
    try:
        run = RandomMove() # seconds
    except rospy.ROSInterruptException:
        pass
