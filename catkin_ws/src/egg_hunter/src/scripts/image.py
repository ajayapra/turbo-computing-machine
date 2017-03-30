# program to test an image with easter eggs
import cv2
import numpy as np

img = cv2.imread('testone.jpg',1)
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# Uncomment the following two lines to launch windows. 
cv2.namedWindow("window1", cv2.WINDOW_NORMAL) 
cv2.resizeWindow("window1", 640,480)
cv2.namedWindow("window2", cv2.WINDOW_NORMAL)
cv2.resizeWindow("window2", 640,480)

lower_range = np.array([66, 100, 100], dtype=np.uint8)
upper_range = np.array([86, 255, 255], dtype=np.uint8)

mask = cv2.inRange(hsv, lower_range, upper_range)
 
cv2.imshow("window1",mask)
cv2.imshow("window2", img)
 
while(1):
  k = cv2.waitKey(0)
  if(k == 27):
    break
 
cv2.destroyAllWindows()
