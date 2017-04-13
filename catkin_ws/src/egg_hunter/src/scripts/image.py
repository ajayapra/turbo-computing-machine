# program to test an image with easter eggs
import cv2
import numpy as np

img = cv2.imread('LAVAR.jpeg',1)
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# Uncomment the following two lines to launch windows. 
#cv2.namedWindow("window1", cv2.WINDOW_NORMAL) 
#cv2.resizeWindow("window1", 640,480)
#cv2.namedWindow("window2", cv2.WINDOW_NORMAL)
#cv2.resizeWindow("window2", 640,480)

# Green threshold
green_lower_range = np.array([67, 100, 100], dtype=np.uint8)
green_upper_range = np.array([90, 255, 255], dtype=np.uint8)

# Orange threshold
orange_lower_range = np.array([0, 100, 100], dtype=np.uint8)
orange_upper_range = np.array([16, 255, 255], dtype=np.uint8)

# Blue threshold
blue_lower_range = np.array([91, 100, 100], dtype=np.uint8)
blue_upper_range = np.array([111, 255, 255], dtype=np.uint8)

# Yellow threshold
yellow_lower_range = np.array([27, 100, 100], dtype=np.uint8)
yellow_upper_range = np.array([30, 255, 255], dtype=np.uint8)

# Pink threshold
pink_lower_range = np.array([165, 100, 100], dtype=np.uint8)
pink_upper_range = np.array([169, 255, 255], dtype=np.uint8)

# Violet threshold
violet_lower_range = np.array([118, 100, 100], dtype=np.uint8)
violet_upper_range = np.array([137, 255, 255], dtype=np.uint8)

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
mask = cv2.erode(mask, None, iterations=3)
mask = cv2.dilate(mask, None, iterations=6)
#masked = cv2.bitwise_and(img, img, mask=mask)
cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
    cv2.CHAIN_APPROX_SIMPLE)[-2]
c = max(cnts, key=cv2.contourArea)
((x, y), radius) = cv2.minEnclosingCircle(c)
count = 0
if radius > 30 and radius < 80:
    count = count + 1
print('violet: ' + str(count))

# Count number of green eggs
mask = green_mask
mask = cv2.erode(mask, None, iterations=3)
mask = cv2.dilate(mask, None, iterations=6)
#masked = cv2.bitwise_and(img, img, mask=mask)
cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
    cv2.CHAIN_APPROX_SIMPLE)[-2]
c = max(cnts, key=cv2.contourArea)
((x, y), radius) = cv2.minEnclosingCircle(c)
count = 0
if radius > 30 and radius < 80:
    count = count + 1
print('green: ' + str(count))

# Count number of pink eggs
mask = pink_mask
mask = cv2.erode(mask, None, iterations=3)
mask = cv2.dilate(mask, None, iterations=6)
#masked = cv2.bitwise_and(img, img, mask=mask)
cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
    cv2.CHAIN_APPROX_SIMPLE)[-2]
c = max(cnts, key=cv2.contourArea)
((x, y), radius) = cv2.minEnclosingCircle(c)
count = 0
if radius > 30 and radius < 80:
    count = count + 1
print('pink: ' + str(count))

# Count number of blue eggs
mask = blue_mask
mask = cv2.erode(mask, None, iterations=3)
mask = cv2.dilate(mask, None, iterations=6)
#masked = cv2.bitwise_and(img, img, mask=mask)
cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
    cv2.CHAIN_APPROX_SIMPLE)[-2]
c = max(cnts, key=cv2.contourArea)
((x, y), radius) = cv2.minEnclosingCircle(c)
count = 0
if radius > 30 and radius < 80:
    count = count + 1
print('blue: ' + str(count))

# Count number of orange eggs
mask = orange_mask
mask = cv2.erode(mask, None, iterations=3)
mask = cv2.dilate(mask, None, iterations=6)
#masked = cv2.bitwise_and(img, img, mask=mask)
cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
    cv2.CHAIN_APPROX_SIMPLE)[-2]
c = max(cnts, key=cv2.contourArea)
((x, y), radius) = cv2.minEnclosingCircle(c)
count = 0
if radius > 30 and radius < 100:
    count = count + 1
print('orange: ' + str(count))

# Count number of orange eggs
mask = yellow_mask
mask = cv2.erode(mask, None, iterations=3)
mask = cv2.dilate(mask, None, iterations=6)
#masked = cv2.bitwise_and(img, img, mask=mask)
cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
    cv2.CHAIN_APPROX_SIMPLE)[-2]
c = max(cnts, key=cv2.contourArea)
((x, y), radius) = cv2.minEnclosingCircle(c)
count = 0
if radius > 30 and radius < 80:
    count = count + 1
print('yellow: ' + str(count))

# Show the frame to the screen and increment the frame counter.
# Uncomment the following line to view the frame.
#cv2.imshow("window1", mask)
#cv2.imshow("window2", img)
 
while(1):
  k = cv2.waitKey(0)
  if(k == 27):
    break
 
cv2.destroyAllWindows()
