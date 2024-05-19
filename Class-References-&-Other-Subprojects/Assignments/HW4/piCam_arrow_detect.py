import cv2
import os
import imutils
import numpy as np
import matplotlib.pyplot as plot
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
from datetime import datetime

def img_mask_blur(img):
    
    # Save the shape of the image array
    (height, width, c)  = img.shape
    
    # Convert image from BGR to HSV space
    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # HSV bounds
    minHSV = np.array([49, 103, 61]) 
    maxHSV = np.array([93, 255, 248]) 

    # Create a function that will search through every
    # pixel and mask
    
    # Need cv2.inRange() or custom function for HSV masking
    maskHSV = cv2.inRange(imgHSV, minHSV, maxHSV)
    # Apply Gaussian blur
    blurred = cv2.GaussianBlur(maskHSV,(11,11), 0)
    
    return blurred

def corner_detect(img,orig_img):

    # Detect corners from image
    corners = cv2.goodFeaturesToTrack(img,5,0.01,10)
    
    # Check if there are corners
    if corners is None:
        return orig_img, []
    else:
        corners = np.int0(corners)

    # Create a list to store the x,y location of points
    pts_loc = []
    
    # identify location of corners in image
    for i in corners:
        # Extract x,y coordinate of points
        x,y = i.ravel()
        # Draw circle of corners on image
        cv2.circle(orig_img,(x,y),3,(255,0,0),-1)
        # Store image coordinate of corners in a list
        pts_loc.append([x,y])

    # Create a column vector from pts list
    pts_loc = np.array(pts_loc)
    
    return orig_img, pts_loc

def def_det(img,pt_list):   
    
    # Extract x,y points from pt_list
    x = pt_list[:,0]
    y = pt_list[:,1]
    
    # Determine the min and max width & height values
    # of the points, as if to drow rectangle around arrow
    x_min = x.min()
    y_min = y.min()
    
    x_max = x.max()
    y_max = y.max()
    
    # Store height of bounding box
    vert_dst = y_max - y_min
    
    # Store width of bounding box
    horz_dst = x_max - x_min
    
    # Compute and store half dimensions of
    # box, will come later when determining
    # arrow direction
    y_half = vert_dst/2 + y_min
    x_half = horz_dst/2 + x_min

    # Find the percentage difference between vertical
    # and horizonal max separtaion distances of points
    ver_pnt = 100 * (vert_dst/(vert_dst+horz_dst))
    hor_pnt = 100 * (horz_dst/(vert_dst+horz_dst))
    
    # Initializing the direction of the arrow
    direction = ""
    
    # Initialize color to display direction on
    # image frame
    color = (0,0,0)
    
    # Initialize counter to track the number of
    # points below the half vertical distance.
    count = 0
    
    # When the arrow vertical (up/down) or (left/right)
    if (vert_dst > horz_dst) and (ver_pnt > 60):
        
        # Since the head has 3 corners identified,
        # if the arrow is pointing up (noting origin of image
        # is at the left top corner), there will be three
        # points below the middle of the arrow vertically
        
        # loop through all vertical points of corner ordered
        # points
        for i in y:
            if (i < y_half):
                count+=1
        # Determine direction
        if count > 2:
            direction = "Up"
            # default color
        else:
            direction = "Down"
            color = (0,0,255)
            
    elif (horz_dst > vert_dst) and (hor_pnt > 60):
        # Similar to above if logic

        # Since the head has 3 corners identified,
        # if the arrow is pointing left (noting origin of image
        # is at the left top corner), there will be three
        # points to the left fo the middle of the arrow horizontally

        # loop through all vertical points of corner ordered
        # points
        for i in x:
            if (i < x_half):
                count+=1
        # Determine direction
        if count > 2:
            direction = "Left"
            color = (255,0,0)
        else:
            direction = "Right"
            color = (0,255,0)
            
    else:
        direction = "---"
        color = (10,10,10)
        
    return direction, color

def img_dir_txt(img,direction,color):
    
    # Place direction text on image
    (cx,cy) = (img.shape[1]/4.5, img.shape[0]/8)
    # Drawing white background
    cv2.rectangle(img,(0,0),(int(cx),int(cy)),(255,255,255),-1)
    # Placing text over white background
    font = cv2.FONT_HERSHEY_COMPLEX_SMALL
    # Print direction of arrow on image
    cv2.putText(img,direction,(0,int(cy/2)),font,2,color,2)
    
    return img

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

    # Initialize time count for iteration
    start = time.time()

    # create object to read camera
    video = cv2.VideoCapture(0)

    if (video.isOpened() == False):
        print("Error reading video")

    # define the codec and create VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter('videonameNew.avi', fourcc, 3, (640, 480))

    # Initialize performance collection variables 
    # (total number of frames, duration of each iteration )
    frm_cnt = 0
    duration = 0

    # Open .txt file to save data
    f = open('hw4data.txt','a')

    # keep looping
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=False):
        # Record iteration start time
        startR = datetime.now() #.microsecond / 1000000
        # grab the current frame
        image = frame.array
        # Flig image vertically and horizontally
        # to accomodate for pi camera mount
        image = cv2.flip(image, -1)
        # Apply HSV masking and Gaussian Blur to image read
        img_blurred = img_mask_blur(image)
        # Detect corners from image
        image, pts_loc = corner_detect(img_blurred,image)
        # Go to the next iteration if on arrow detected
        if len(pts_loc) == 0:
            # show the frame to our screen
            cv2.imshow("No Arrow Detection", image)
        else:
            # Identify direction of arrow
            direction, color = def_det(image,pts_loc)
            # Image of direction detected
            image = img_dir_txt(image,direction,color)
            # show the frame to our screen
            cv2.imshow("Direction Arrow Detection", image)
        key = cv2.waitKey(1) & 0xFF
        # write frame into file
        out.write(image)

        frm_cnt += 1

        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)
        duration = time.time() - start
       
        # End time count for iteration
        stopR = datetime.now()
         
        now = stopR - startR 
        
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