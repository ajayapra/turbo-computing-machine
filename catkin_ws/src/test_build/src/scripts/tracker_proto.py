#!/usr/bin/env python

# define imports
import rospy
import cv2
import cv_bridge
import argparse
import numpy as np

from   sensor_msgs.msg import Image

# Publisher
#def talker():
    # Node declaration. Publish x, y, and radius to topic: custom_chatter,
    # message class: position, and queue_size: 1.
#    pub = rospy.Publisher('custom_chatter', position, queue_size=1)

    # Assigning message class, position to a variable.
#    msg = position()
    # x-position is published to position.
#    msg.x = int(x)
    # y-position is published to position.
#    msg.y = int(y)
    # radius of object is published to position.
    # The jackal stops if the radius is too small or too large(for safety).
#    if (int(radius) > 15) and (int(radius) < 180) and (int(x) < 610) and (int(x) > 30) and (int(y) < 450) and (int(y) > 30) :
#        msg.radius = int(radius)
#    else:
#        msg.radius = 60
    # Tells rospy the name of the node.
#    rospy.loginfo(msg)
    # Publishes x, y, and radius values to the topic.
#    pub.publish(msg)

# Initial declarations.
radius = 0
x = 0
y = 0

# Object tracker class.
class Tracker:
    # Function to subscribe to the camera.
    def __init__(self):
	# cvBridge is a ROS library that provides an interface between ROS
        # and OpenCV.
        self.bridge = cv_bridge.CvBridge()
        # Uncomment the following two lines to launch windows.
        cv2.namedWindow("window1", 1)
        cv2.namedWindow("window2", 1)

        # rospy subscribes to the image_raw topic to receive images.
        self.image_sb = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
    # Function to process the received images.
    def image_callback(self, msg):
	global radius
	global x
	global y
        # imgmsg_to_cv2 converts an image message pointer to an OpenCV
        # message.
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
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

	# # Green threshold
	# green_lower_range = np.array([67, 100, 100], dtype=np.uint8)
	# green_upper_range = np.array([90, 255, 255], dtype=np.uint8)
    #
	# # Orange threshold
	# orange_lower_range = np.array([0, 100, 100], dtype=np.uint8)
	# orange_upper_range = np.array([16, 255, 255], dtype=np.uint8)
    #
	# # Blue threshold
	# blue_lower_range = np.array([91, 100, 100], dtype=np.uint8)
	# blue_upper_range = np.array([111, 255, 255], dtype=np.uint8)
    #
	# # Yellow threshold
	# yellow_lower_range = np.array([27, 100, 100], dtype=np.uint8)
	# yellow_upper_range = np.array([30, 255, 255], dtype=np.uint8)
    #
	# # Pink threshold
	# pink_lower_range = np.array([165, 100, 100], dtype=np.uint8)
	# pink_upper_range = np.array([169, 255, 255], dtype=np.uint8)
    #
	# # Violet threshold
	# violet_lower_range = np.array([118, 100, 100], dtype=np.uint8)
	# violet_upper_range = np.array([137, 255, 255], dtype=np.uint8)
    #
	# # A series of dilations and erosions to remove any small blobs left
	# # in the mask.
	# green_mask = cv2.inRange(hsv, green_lower_range, green_upper_range)
	# orange_mask = cv2.inRange(hsv, orange_lower_range, orange_upper_range)
	# blue_mask = cv2.inRange(hsv, blue_lower_range, blue_upper_range)
	# yellow_mask = cv2.inRange(hsv, yellow_lower_range, yellow_upper_range)
	# pink_mask = cv2.inRange(hsv, pink_lower_range, pink_upper_range)
	# violet_mask = cv2.inRange(hsv, violet_lower_range, violet_upper_range)
    #
	# # Count number of violet eggs
	# mask = violet_mask
	# mask = cv2.erode(mask, None, iterations=3)
	# mask = cv2.dilate(mask, None, iterations=6)
	# #masked = cv2.bitwise_and(img, img, mask=mask)
	# cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
	#     cv2.CHAIN_APPROX_SIMPLE)[-2]
	# try:
	#     c = max(cnts, key=cv2.contourArea)
	# except:
	#     c = 0
	# ((x, y), radius) = cv2.minEnclosingCircle(c)
	# count = 0
	# if radius > 30 and radius < 80:
	#     count = count + 1
	# print('violet: ' + str(count))
    #
	# # Count number of green eggs
	# mask = green_mask
	# mask = cv2.erode(mask, None, iterations=3)
	# mask = cv2.dilate(mask, None, iterations=6)
	# #masked = cv2.bitwise_and(img, img, mask=mask)
	# cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
	#     cv2.CHAIN_APPROX_SIMPLE)[-2]
    #     try:
    #         c = max(cnts, key=cv2.contourArea)
    #     except:
    #         c = 0
    #
	# ((x, y), radius) = cv2.minEnclosingCircle(c)
	# count = 0
	# if radius > 30 and radius < 80:
	#     count = count + 1
	# print('green: ' + str(count))
    #
	# # Count number of pink eggs
	# mask = pink_mask
	# mask = cv2.erode(mask, None, iterations=3)
	# mask = cv2.dilate(mask, None, iterations=6)
	# #masked = cv2.bitwise_and(img, img, mask=mask)
	# cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
	#     cv2.CHAIN_APPROX_SIMPLE)[-2]
    #     try:
    #         c = max(cnts, key=cv2.contourArea)
    #     except:
    #         c = 0
    #
	# ((x, y), radius) = cv2.minEnclosingCircle(c)
	# count = 0
	# if radius > 30 and radius < 80:
	#     count = count + 1
	# print('pink: ' + str(count))
    #
	# # Count number of blue eggs
	# mask = blue_mask
	# mask = cv2.erode(mask, None, iterations=3)
	# mask = cv2.dilate(mask, None, iterations=6)
	# #masked = cv2.bitwise_and(img, img, mask=mask)
	# cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
	#     cv2.CHAIN_APPROX_SIMPLE)[-2]
    #     try:
    #         c = max(cnts, key=cv2.contourArea)
    #     except:
    #         c = 0
    #
	# ((x, y), radius) = cv2.minEnclosingCircle(c)
	# count = 0
	# if radius > 30 and radius < 80:
	#     count = count + 1
	# print('blue: ' + str(count))
    #
	# # Count number of orange eggs
	# mask = orange_mask
	# mask = cv2.erode(mask, None, iterations=3)
	# mask = cv2.dilate(mask, None, iterations=6)
	# #masked = cv2.bitwise_and(img, img, mask=mask)
	# cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
	#     cv2.CHAIN_APPROX_SIMPLE)[-2]
    #     try:
    #         c = max(cnts, key=cv2.contourArea)
    #     except:
    #         c = 0
    #
	# ((x, y), radius) = cv2.minEnclosingCircle(c)
	# count = 0
	# if radius > 30 and radius < 100:
	#     count = count + 1
	# print('orange: ' + str(count))
	#
	# # Count number of orange eggs
	# mask = yellow_mask
	# mask = cv2.erode(mask, None, iterations=3)
	# mask = cv2.dilate(mask, None, iterations=6)
	# #masked = cv2.bitwise_and(img, img, mask=mask)
	# cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
	#     cv2.CHAIN_APPROX_SIMPLE)[-2]
    #     try:
    #         c = max(cnts, key=cv2.contourArea)
    #     except:
    #         c = 0
    #
	# ((x, y), radius) = cv2.minEnclosingCircle(c)
	# count = 0
	# if radius > 30 and radius < 80:
	#     count = count + 1
	# print('yellow: ' + str(count))
	#
    #     # Calling the publisher.
	# #talker()
	# # Show the frame to the screen and increment the frame counter.
	# # Uncomment the following line to view the frame.
	# cv2.imshow("window1", mask)
	# cv2.imshow("window2", img)


# Initialize node.
rospy.init_node('Track_Marker')
Track_Marker = Tracker()
rospy.spin()
