#!/usr/bin/env python
import rospy
import random
import sys
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time
import math
from datetime import datetime

angle_min= -90 * (math.pi/180)
angle_max= 90 * (math.pi/180)
angle_increment = 0.25 * (math.pi/180)

def scan_callback(msg):
    global g_range_ahead
    global g_range_left
    global g_range_right
    global g_range_ld
    global g_range_rd

    g_range_ahead = msg.ranges[360]
    g_range_left = msg.ranges[719]
    g_range_right = msg.ranges[1]
    g_range_ld = msg.ranges[540]
    g_range_rd = msg.ranges[180]

def getTime(data):
    global last_time
    last_time = rospy.get_time()

def rotate():
    current_angle = 0
    relative_angle = random.uniform(8,14)
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0.5
    t0 = time.time()
    while(current_angle < relative_angle):
        pub.publish(twist)
        t1 = time.time()
        current_angle = 0.5*(t1-t0)
    twist.angular.z = 0
    pub.publish(twist)

if __name__ == "__main__":
    last_time=0.0
    g_range_ahead = 1.0
    g_range_left = 1.0
    g_range_right = 1.0
    g_range_ld = 1.0
    g_range_rd = 1.0
    try:
    	rospy.init_node("random_map")
    	rospy.Subscriber("/front/scan", LaserScan, scan_callback)
    	rate = rospy.Rate(50)
	pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
	twist = Twist()
	end_time = time.time() + 600
	i = list(range(1,9))
	i = [x*75 for x in i]
	i = [x+time.time() for x in i]
	#rotate()
	j = 0
	while time.time() < end_time:
	    if time.time() > i[j] and time.time() < (i[j] + 20):
		#rotate()
		j = j + 1
	    #print j
    	    if g_range_ahead < 1.0 or g_range_rd < 0.5:
		twist = Twist()
    	        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    	        twist.angular.z = random.uniform(0,2)
		pub.publish(twist)
	    elif g_range_ahead < 1.0 or g_range_ld < 0.5:
                twist = Twist()
                twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
                twist.angular.z = random.uniform(-2,0)
                pub.publish(twist)
       	    elif g_range_ahead >= 1.0:
	        twist = Twist()
	        twist.linear.x = 1.0
                twist.angular.z = 0 
	        pub.publish(twist)
    	    rate.sleep()
    except rospy.ROSInterruptException:
	pass
