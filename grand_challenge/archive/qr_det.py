import math
import cv2
import os
import imutils
import numpy as np
import matplotlib.pyplot as plot
import picamera
from picamera.array import PiRGBArray
from picamera import PiCamera

import time

import cntr_bbox

# Global variables
# 
# initialize the Raspberry Pi camera
camera = PiCamera()
camera.resolution = (640, 480)
camera.vflip = True
camera.framerate = 25
rawCapture = PiRGBArray(camera, size=(640,480))
time.sleep(2)

# create object to read camera
video = cv2.VideoCapture(0)

if (video.isOpened() == False):
    print("Error reading video")
    
## Create this video writer from the main function and pass the video write object
# define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('videonameNew.avi', fourcc, 3, (640, 480))

stream = io.BytesIO()

def wait2start(out):
    
    # Define detector
    detector = cv2.QRCodeDetector()
    
    # Check if program start initiated
    start = False
    
    cnt = 0
    
    print("Waiting for start cue . . . ")
    
    while True:
        try:
        
            #for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=False):
            for frame in camera.capture_continuous(stream, format="jpeg"):

                # grab the current frame
#                 img = frame.array
                stream.seek(0)
                img = frame.read()

                img = cv2.flip(img,1)
                
                data, bbox, _ = detector.detectAndDecode(img)

                if(bbox is not None):
                    for i in range(len(bbox)):
                        cv2.line(img, tuple(bbox[i][0]),tuple(bbox[(i+1)%len(bbox)][0]), color=(0,0,255),thickness=4)
                        #            cv2.putText(img, data, (int(bbox[0][0][0]),int(bbox[0][0][1]-10),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),2)

                if data:
                    print("Data: ", data)
                        
                # Show resutls to the screen
                cv2.imshow("QR Code detector",img)
                key = cv2.waitKey(1) & 0xFF
                #     cv2.imshow("QR Code detector", img)
            
                # write frame into file
                out.write(img)

                # clear the stream in preparation for the next frame
                stream.seek(0)
                stream.truncate()
#                 rawCapture.truncate(0)
                
                # Break out of loop by pressing the q key
                if(key == ord("q")):
                    print("Program Terminated")
                    video.release()
                    cv2.destroyAllWindows()
                    break

                if data == "ENPM701":
                    print("Cue Received \n")
                    print('@' * 45)
                    print("Starting Grand Challenge! \n")
                    print('#' * 45)
                    start = True
                    video.release()
                    cv2.destroyAllWindows()
                    break
            break
            
        except Exception as error:
            print(f"Camera Read: {error}")
            video.release()
            cv2.destroyAllWindows()
            
        except picamera.exc.PiCameraMMALError:
            print("Camera frame read error")
            video.release()
            cv2.destroyAllWindows()
            continue
    
    return start

wait2start(out)