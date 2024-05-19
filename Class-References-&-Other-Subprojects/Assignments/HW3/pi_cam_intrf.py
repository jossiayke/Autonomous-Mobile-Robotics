# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
# import os
# import numpy as np

# initialize the Raspberry Pi camera
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 25
rawCapture = PiRGBArray(camera, size=(640,480))

# allow the camera to warmup
time.sleep(0.1)

start = time.time()
# f_wid = int(video.get(3))
# f_ht = int(video.get(4))

# create object to read camera
video = cv2.VideoCapture(0)

if (video.isOpened() == False):
    print("Error reading video")
# size = (f_wid, f_ht)

# define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')
# fourcc = cv2.VideoWriter_fourcc(*'MJPG')
out = cv2.VideoWriter('videoname2.avi', fourcc, 3, (640, 480))

frm_cnt = 0
duration = 0

# keep looping
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=False):
    
	# grab the current frame
	image = frame.array
# 	ret, frame1 = video.read()
	
	# show the frame to our screen
	cv2.imshow("Frame", image)
	key = cv2.waitKey(1) & 0xFF
	
	# write frame into file
	out.write(image)
# 	out.write(frame1)
	frm_cnt +=1
	
	# clear the stream in preparation for the next frame
	rawCapture.truncate(0)
	duration = time.time() - start
	
	# press the 'q' key to stop the video stream
	if key == ord("q") or (duration >= 40):
		break

# actualFps = np.ceil(frm_cnt/duration)
# 
# os.system('ffmpeg -y -i {} -c copy -f h264 tmp.h264'.format('videoname2.avi'))
# os.system('ffmpeg -y -r {} -i tmp.h264 -c copy {}'.format(actualFps,'videoname2.avi'))

# Release video capture and video object
video.release()
out.release()

# Close all windows
cv2.destroyAllWindows()
