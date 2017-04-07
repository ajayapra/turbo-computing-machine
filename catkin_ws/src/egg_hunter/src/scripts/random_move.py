#!/usr/bin/env python
import rospy
import math
import time

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from actionlib_msgs.msg import *
from geometry_msgs.msg import Point
from move_base_msgs.msg	import MoveBaseAction, MoveBaseGoal

class RandomMove(object):
    def __init__(self, timeout=None):
        rospy.init_node("RandomMove")

        self.turnCoef = [(x ** 2 - 8100) / 10000000.0 for x in range(-90, 0)] + [(-x ** 2 + 8100) / 10000000.0 for x in range(0, 91)]
        self.speedCoef = [(-x ** 2 + 8100) / 10000000.0 for x in range(-90,91)]

        self.last_speed = 0
        self.last_turn = 0

        rospy.Subscriber("/front/scan", LaserScan, self._latestScan)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        rospy.loginfo("Ready to get out there and avoid some walls!")
        rospy.logdebug(self.turnCoef)
        rospy.logdebug(self.speedCoef)

        self.timeout = None
        if timeout:
            self.timeout = time.time() + timeout

        rospy.spin()

    def isTimedout(self):
        return self.timeout <= time.time()

    def _latestScan(self, data):
        if self.timeout and self.timeout <= time.time():
            rospy.signal_shutdown("Execution timer expired")

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
