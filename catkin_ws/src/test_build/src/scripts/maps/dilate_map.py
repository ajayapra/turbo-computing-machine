import cv2
import numpy as np
cv2.namedWindow("window1", cv2.WINDOW_NORMAL) 
cv2.resizeWindow("window1", 640,480)
img = cv2.imread('map.pgm',0)
img = 255-img
kernel = np.ones((5,5),np.uint8)
dilated_map = cv2.dilate(img,kernel,iterations = 3)
dilated_map = 255-dilated_map
cv2.imwrite('dilated_map.pgm',dilated_map)
cv2.imshow("window1", dilated_map)
cv2.waitKey(0)
