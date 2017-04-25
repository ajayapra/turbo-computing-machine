# program to test an image with easter eggs
import cv2
import numpy as np

#img = cv2.imread('LAVAR.jpeg',1)
#img = cv2.imread('testone.jpg',1)
img = cv2.imread('final_test1.png',1)
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# Uncomment the following two lines to launch windows. 
cv2.namedWindow("window1", cv2.WINDOW_NORMAL) 
cv2.resizeWindow("window1", 640,480)
cv2.namedWindow("window2", cv2.WINDOW_NORMAL)
cv2.resizeWindow("window2", 640,480)

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

# Show the frame to the screen and increment the frame counter.
# Uncomment the following line to view the frame.
cv2.imshow("window1", img)
cv2.imshow("window2", blue_mask)
 
while(1):
  k = cv2.waitKey(0)
  if(k == 27):
    break
 
cv2.destroyAllWindows()
