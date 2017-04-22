#!/usr/bin/env python

# define imports
import rospy
import roslaunch
import sys
import time
import os

from   sensor_msgs.msg import Joy

# class to read joystick messages and launch node
class joy_control(object):

    # define self routine
    def __init__(self):

        # define subscriber
        rospy.Subscriber("/bluetooth_teleop/joy", Joy, self.joy_callback)

        #define Publisher for publishing to /action_input topic
        self.key_pub = rospy.Publisher("action_input", String, queue_size=1)
        rospy.loginfo('started the publishing routine to action_input')

        rate = rospy.Rate(5)

        rospy.loginfo('started joystick routine..')

        # define and init variables
        self.start = False
        self.stop  = False

        # configure node roslaunch api
        package    = 'egg_hunter'

        ######Change Executeable here #######
        executable = 'random_move.py'
        #####################################

        node = roslaunch.core.Node(package, executable)

        while not rospy.is_shutdown():
            # if start flag set: launch main launch-file
            if self.start:
                launch = roslaunch.scriptapi.ROSLaunch()
                launch.start()
                process = launch.launch(node)

            # if stop flag set: shutdown main launch-file
            if self.stop:
                process.stop()

            # reset trigger
            self.start = False
            self.stop  = False
            rate.sleep()


    # joystick callback routine
    def joy_callback(self, data):

        # define joystick buttons
        x, circ, sq, tri, L1, R1, share, options, p4, L3, R3, DL, DR, DU, DD = data.buttons
        llr, lud, L2, rlr, rud, R2 = data.axes

        # Start object tracking
        if (x == 1) and (self.start == False):
            rospy.loginfo("Starting the mapping routine...")
            # set the start flag
            self.start = True

        # Stop tracking
        if (tri == 1):
            rospy.loginfo("Terminating the mapping routine...")
            # set stop flag
            #self.stop  = True
            #publish terminating node message
            key_pub.publish("t")

        #Halt the Jackal
        if (sq == 1):
            rospy.loginfo("Start/Stop the Jackal...")
            # set stop flag
            key_pub.publish("h")

        #Save the waypoint
        if (circ == 1):
            rospy.loginfo("Save Waypoint...")
            # set stop flag
            key_pub.publish("s")


# standard boilerplate
if __name__ == "__main__":
    try:
        rospy.init_node("joy_start", anonymous=False)
        #read in joystick input
        run = joy_control()
    except rospy.ROSInterruptException:
        rospy.loginfo("joy_start node terminated.")
