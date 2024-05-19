import tkinter
import cv2
import os
import imutils
import numpy as np
import matplotlib.pyplot as plot
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
from datetime import datetime, timedelta

def img_show(name, img):
	cv2.imshow(name,img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

def mask_color(image, imageHSV):
	# HSV bounds
	minHSV = np.array([50,100, 100])
	maxHSV = np.array([70, 255, 255])
	# mask HSV
	maskHSV = cv2.inRange(imageHSV, minHSV, maxHSV)

	return maskHSV

def main():

	# Purpose: Continuosly locate and identify green light from video feed
	# Pull in code from Assignment 2 and adjust
	# initialize the Raspberry Pi camera
	camera = PiCamera()
	camera.resolution = (640, 480)
	camera.framerate = 25
	rawCapture = PiRGBArray(camera, size=(640,480))

	# Pull in code from steps 1 and 2
	# allow the camera to warmup
	time.sleep(0.1)

	start = time.time()

	# create object to read camera
	video = cv2.VideoCapture(0)

	if (video.isOpened() == False):
		print("Error reading video")

	# size = (f_wid, f_ht)

	# define the codec and create VideoWriter object
	fourcc = cv2.VideoWriter_fourcc(*'XVID')
	out = cv2.VideoWriter('videonameNew.avi', fourcc, 3, (640, 480))

	frm_cnt = 0
	duration = 0

	# Open .txt file to save data
	f = open('hw3data_4.txt','a')

	# Inirtialize circle variables
	center = 0 
	radius = 0.0


	x = 0.0
	y = 0.0

	# keep looping
	for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=False):
		# Record iteration start time
		startR = datetime.now() #.microsecond / 1000000
		# grab the current frame
		image = frame.array

		# Convert image from BGR to HSV space
		imageHSV = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)

		#img_show("Hsv img", imageHSV)
		# mask the green light from HSV and convert to grayscale
		mask = mask_color(image, imageHSV)

		## Draw contours over an image, if available
		#img, msg  = detect_green(image, mask)

		# Apply contour function to find edges
		img, contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		# Contour detection
		if (contours is not None) and (len(contours) >= 1) :

			# Take the first contour
			cnt = contours[0]

			# compute the moments of contours
			momnt = cv2.moments(cnt)
			print(momnt)

			# min Enclosing circle
			(x,y),radius = cv2.minEnclosingCircle(cnt)
			# Save circle radius and center as int
			center = int(x),int(y)
			radius = int(radius)

			# Draw circle ontop of original image
			cv2.circle(image, center, radius, (0,255,255),2)
			cv2.circle(image, center,0,(0,0,255),5)

		else:
			print("Countour not found")

		# Draw circle ontop of original image
		#cv2.circle(image, center, radius, (0,255,255),2)
		#cv2.circle(image, center,0,(0,0,255),5)

		# show the frame to our screen
		cv2.imshow("Frame", image)
		key = cv2.waitKey(1) & 0xFF

		# write frame into file
		out.write(image)

		frm_cnt += 1

		# clear the stream in preparation for the next frame
		rawCapture.truncate(0)
		duration = time.time() - start

		#first = float(startR.second) + float(startR.microsecond / 1000000)
		stopR = datetime.now() #
		#finish = float(stopR.second) + float(stopR.microsecond / 1000000)
		now = stopR - startR #round(finish - first,3) #stopR - startR
		#outstring = str(now) + '\n'
		outstring = str(now.total_seconds()) + '\n'
		f.write(outstring)
		print(now)

		# press the 'q' key to stop the video stream
		if (key == ord("q") or (duration >= 40)) and (frm_cnt > 110):
			break

	# Release video capture and video object
	video.release()
	out.release()

	# Close all windows
	cv2.destroyAllWindows()

if __name__ == "__main__":
	main()
