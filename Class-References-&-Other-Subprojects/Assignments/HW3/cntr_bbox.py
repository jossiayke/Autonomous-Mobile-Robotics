import cv2
import os
import imutils
import numpy as np
import matplotlib.pyplot as plot

# Read original RGB Image from library
tLight = cv2.imread("trafficLight_exp.png")

# Read hsv masked image from library
tL_hsv = cv2.imread("trfcLght_hsv.png")
gray = cv2.cvtColor(tL_hsv,cv2.COLOR_BGR2GRAY)

# Apply contour function to find edges
img, contours, hierarchy = cv2.findContours(gray,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

# Draw contours over an image, if available
if contours is not None:
    # Take the first contour
    cnt = contours[1]
    # compute the moments of contours
    momnt = cv2.moments(cnt)
    print(momnt)
    # min Enclosing circle
    (x,y),radius = cv2.minEnclosingCircle(cnt)
    # Save circle radius and center as int
    center = int(x),int(y)
    radius = int(radius)
    # Draw circle ontop of original image
    cv2.circle(tLight, center, radius, (0,255,255),2)
    cv2.circle(tLight, center,0,(0,0,255),5)
    cv2.imshow("Green Circle Edge", tLight)
    cv2.imwrite("grnCntCrcleEdge.png",tLight)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print("Contours not detected.")
    #cv2.drawContours(tLight, [cnt],0,(0,255,255),3)
