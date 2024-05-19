import cv2
import os
import imutils
import numpy as np
import matplotlib.pyplot as plot

# Read an image from library
tLight = cv2.imread("trafficLight_exp.png")
# Display Image read
cv2.imshow("Original Traffic Light Image", tLight)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Save the shape of the image array
(height, width, c)  = tLight.shape
# Convert image from BGR to HSV space
tLightHSV = cv2.cvtColor(tLight, cv2.COLOR_BGR2HSV)

# Display HSV converted image
cv2.imshow("HSV converted Traffic Light Image", tLightHSV)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Create a blank canvas with image dimensions
# then Set canvas color to black 0-255
canvasBGR = np.zeros((height, width, c), dtype="uint8")
canvasHSV = np.zeros((height, width, c), dtype="uint8")

# Define uppper lower bounds for Green pixel
# BGR Bounds
bgr = [40, 158, 16]
thrsh = 62

minBGR = np.array([bgr[0]-thrsh,bgr[1]-thrsh,bgr[2]-thrsh])
maxBGR = np.array([bgr[0]+thrsh,bgr[1]+thrsh,bgr[2]+thrsh])

# minBGR = np.array([20,80, 20])
# maxBGR = np.array([80, 225, 70])

# Need cv2.inRange() or custom function
# BGR mask
maskBGR = cv2.inRange(tLight, minBGR, maxBGR)

# Display BGR masking
cv2.imshow("BGR Masking Traffic Light Image", maskBGR)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Create array of masked pixels to identify desired pixels
imskBGR = maskBGR > 0
# Then draw white on canvas when pixels correspond
# to green
canvasBGR[imskBGR] = tLight[imskBGR]

# Display results
cv2.imshow("BGR-masked on Canvas", canvasBGR)
cv2.waitKey(0)
cv2.destroyAllWindows()

# HSV bounds
minHSV = np.array([50,100, 100])
maxHSV = np.array([70, 255, 255])

# Create a function that will search through every
# pixel and mask
# Need cv2.inRange() or custom function
# HSV mask
maskHSV = cv2.inRange(tLightHSV, minHSV, maxHSV)

# Display BGR masking
cv2.imshow("HSV Masking Traffic Light Image", maskHSV)
cv2.imwrite("trfcLght_hsv.png",maskHSV)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Create array of masked pixels to identify desired pixels
imskHSV = maskHSV > 0
# Then draw white on canvas when pixels correspond
# to green
canvasHSV[imskHSV] = tLight[imskHSV]
# Display green on canvas via HSV
cv2.imshow("HSV-masked drawn on Canvas", canvasHSV)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Apply bitwise end to create mask over original image
finBGR = cv2.bitwise_and(tLight, tLight, mask = maskBGR)
finHSV = cv2.bitwise_and(tLight, tLight, mask = maskHSV)

# Display original RGB, HSV and masked images side by side
# Use Numpy's hstack
# allTogether = np.hstack((maskBGR,maskHSV, finHSV))
allTogether = np.hstack((tLight,finBGR, tLightHSV,finHSV))
cv2.imshow("1-rgb, 2-rgb masked, 3-hsv, 4-hsv masked", allTogether)

# Saving image in local
cv2.imwrite("bgr_hsv_mask.png", allTogether)

cv2.waitKey(0)
cv2.destroyAllWindows()
    
# At the end, show RGB, HSV, and then masked images
# via numpy hstack 